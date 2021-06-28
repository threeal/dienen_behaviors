// Copyright (c) 2021 Alfi Maulana
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <argparse/argparse.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tosshin/tosshin.hpp>

#include <memory>
#include <string>

namespace tsn = tosshin;

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  auto program = argparse::ArgumentParser("odometry_synchronizer", "0.2.0");

  program.add_argument("camera-info-topic")
  .help("camera info topic name");

  program.add_argument("--odometry-target-topic")
  .default_value("/odom")
  .help("camera info topic name");

  std::string camera_info_topic;
  std::string odometry_target_topic;

  // Try to parse arguments
  try {
    program.parse_args(argc, argv);

    camera_info_topic = program.get<std::string>("camera-info-topic");
    odometry_target_topic = program.get<std::string>("--odometry-target-topic");
  } catch (const std::exception & e) {
    std::cout << e.what() << std::endl;
    std::cout << program;
    return 1;
  }

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("odometry_synchronizer");

  auto odometry_publisher = node->create_publisher<tsn::msg::Odometry>(odometry_target_topic, 10);
  auto tf_publisher = node->create_publisher<tsn::msg::TFMessage>("/tf", 10);

  tsn::msg::Odometry last_odometry;
  auto odometry_subscription = node->create_subscription<tsn::msg::Odometry>(
    "/odom", 10,
    [&](const tsn::msg::Odometry::SharedPtr msg) {
      last_odometry = *msg;
    });

  auto camera_info_subscription = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic, 10,
    [&](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      auto odometry = last_odometry;
      odometry.header.stamp = msg->header.stamp;

      // publish synchronized odometry
      odometry_publisher->publish(odometry);

      // publish synchronized tf
      tsn::msg::TFMessage tf;
      tf.transforms.push_back(tsn::make_transform_stamped(odometry));
      tf_publisher->publish(tf);
    });

  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "Synchronizing odometry with " << camera_info_subscription->get_topic_name() <<
      " into " << odometry_publisher->get_topic_name());

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

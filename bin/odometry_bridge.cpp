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

#include <nav_msgs/msg/odometry.hpp>
#include <tosshin_cpp/tosshin_cpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("odometry_bridge");

  auto odometry_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "Odometry publisher initialized on `" << odometry_publisher->get_topic_name() << "`!");

  auto odometry_consumer = std::make_shared<tosshin_cpp::OdometryConsumer>(node);
  odometry_consumer->set_on_change_odometry(
    [&](const tosshin_cpp::Odometry & odometry) {
      nav_msgs::msg::Odometry msg;

      msg.pose.pose.position.x = odometry.position.x;
      msg.pose.pose.position.y = odometry.position.y;

      // Convert from equler angles to quaternion
      {
        double cy = cos(odometry.orientation.yaw * 0.5);
        double sy = sin(odometry.orientation.yaw * 0.5);
        double cp = cos(0.0);
        double sp = sin(0.0);
        double cr = cos(0.0);
        double sr = sin(0.0);

        msg.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
        msg.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
        msg.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
        msg.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
      }

      odometry_publisher->publish(msg);
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <keisan/keisan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tosshin_cpp/tosshin_cpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("slam_bridge");

  auto odometry_consumer = std::make_shared<tosshin_cpp::OdometryConsumer>(node);

  auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  auto tf_publisher = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

  auto camera_info_subscription = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/rgb/camera_info", 10,
    [&](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      auto odometry = odometry_consumer->get_odometry();

      geometry_msgs::msg::TransformStamped odom_tf;

      odom_tf.header.frame_id = "odom";
      odom_tf.header.stamp = msg->header.stamp;

      odom_tf.child_frame_id = "base_footprint";

      odom_tf.transform.translation.x = odometry.position.x;
      odom_tf.transform.translation.y = odometry.position.y;

      // Convert orientation from equler angles to quaternion
      {
        double cy = keisan::cos(keisan::make_degree(odometry.orientation.yaw) * 0.5);
        double sy = keisan::sin(keisan::make_degree(odometry.orientation.yaw) * 0.5);
        double cp = cos(0.0);
        double sp = sin(0.0);
        double cr = cos(0.0);
        double sr = sin(0.0);

        odom_tf.transform.rotation.x = sr * cp * cy - cr * sp * sy;
        odom_tf.transform.rotation.y = cr * sp * cy + sr * cp * sy;
        odom_tf.transform.rotation.z = cr * cp * sy - sr * sp * cy;
        odom_tf.transform.rotation.w = cr * cp * cy + sr * sp * sy;
      }

      // Publish odom message
      {
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header = odom_tf.header;
        odom_msg.child_frame_id = odom_tf.child_frame_id;
        odom_msg.pose.pose.orientation = odom_tf.transform.rotation;

        odom_msg.pose.pose.position.x = odom_tf.transform.translation.x;
        odom_msg.pose.pose.position.y = odom_tf.transform.translation.y;
        odom_msg.pose.pose.position.z = odom_tf.transform.translation.z;

        odom_publisher->publish(odom_msg);
      }

      // Publish TF message
      {
        tf2_msgs::msg::TFMessage tf_msg;

        // Push odom transform
        tf_msg.transforms.push_back(odom_tf);

        // Push camera transform
        {
          geometry_msgs::msg::TransformStamped camera_tf;
          camera_tf.header = odom_tf.header;
          camera_tf.header.frame_id = odom_tf.child_frame_id;
          camera_tf.child_frame_id = msg->header.frame_id;

          tf_msg.transforms.push_back(camera_tf);
        }

        tf_publisher->publish(tf_msg);
      }
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

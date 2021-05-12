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
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tosshin_cpp/tosshin_cpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("odometry_bridge");

  auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  auto tf_publisher = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

  auto start_time = node->get_clock()->now();
  auto odometry_consumer = std::make_shared<tosshin_cpp::OdometryConsumer>(node);
  odometry_consumer->set_on_change_odometry(
    [&](const tosshin_cpp::Odometry & odometry) {
      geometry_msgs::msg::TransformStamped transform_stamped;

      auto elapsed_time = node->get_clock()->now() - start_time;

      transform_stamped.header.frame_id = "odom";
      transform_stamped.header.stamp.sec = elapsed_time.seconds();
      transform_stamped.header.stamp.nanosec = elapsed_time.nanoseconds();

      transform_stamped.child_frame_id = "base_footprint";

      transform_stamped.transform.translation.x = odometry.position.x;
      transform_stamped.transform.translation.y = odometry.position.y;

      // Convert orientation from equler angles to quaternion
      {
        double cy = cos(odometry.orientation.yaw * 0.5);
        double sy = sin(odometry.orientation.yaw * 0.5);
        double cp = cos(0.0);
        double sp = sin(0.0);
        double cr = cos(0.0);
        double sr = sin(0.0);

        transform_stamped.transform.rotation.x = sr * cp * cy - cr * sp * sy;
        transform_stamped.transform.rotation.y = cr * sp * cy + sr * cp * sy;
        transform_stamped.transform.rotation.z = cr * cp * sy - sr * sp * cy;
        transform_stamped.transform.rotation.w = cr * cp * cy + sr * sp * sy;
      }

      // Publish odom message
      {
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header = transform_stamped.header;
        odom_msg.child_frame_id = transform_stamped.child_frame_id;
        odom_msg.pose.pose.orientation = transform_stamped.transform.rotation;

        odom_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
        odom_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
        odom_msg.pose.pose.position.z = transform_stamped.transform.translation.z;

        odom_publisher->publish(odom_msg);
      }

      // Publish TF message
      {
        tf2_msgs::msg::TFMessage tf_msg;
        tf_msg.transforms.push_back(transform_stamped);

        tf_publisher->publish(tf_msg);
      }
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

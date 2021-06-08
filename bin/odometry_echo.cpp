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
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

#include <memory>
#include <string>

using nav_msgs::msg::Odometry;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("odometry_echo");

  auto odometry_subscription = node->create_subscription<Odometry>(
    "/odom", 10,
    [node](const Odometry::SharedPtr msg) {
      auto position = msg->pose.pose.position;
      auto linear = msg->twist.twist.linear;
      auto angular = msg->twist.twist.angular;

      double yaw, pitch, roll;
      tf2::getEulerYPR(msg->pose.pose.orientation, yaw, pitch, roll);

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "\nposition\t: " << position.x << ", " << position.y << ", " << position.z <<
          "\norientation\t: " <<
          tf2Degrees(yaw) << ", " << tf2Degrees(pitch) << ", " << tf2Degrees(roll) <<
          "\nlinear vel\t: " << linear.x << ", " << linear.y << ", " << linear.z <<
          "\nangular vel\t: " << angular.x << ", " << angular.y << ", " << angular.z);
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

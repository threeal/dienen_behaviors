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
#include <geometry_msgs/msg/twist.hpp>
#include <keisan/keisan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <list>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

int main(int argc, char ** argv)
{
  auto program = argparse::ArgumentParser("patrol_position", "0.2.0");

  program.add_argument("-r", "--repeat")
  .help("repeat patrol process")
  .default_value(false)
  .implicit_value(true);

  program.add_argument("--holonomic-mode")
  .help("move robot in holonomic mode (strafing)")
  .default_value(false)
  .implicit_value(true);

  program.add_argument("-l", "--linear-speed")
  .help("set maximum linear speed in meter per second")
  .default_value(1.0)
  .action([](const std::string & value) {return std::stod(value);});

  program.add_argument("-a", "--angular-speed")
  .help("set maximum angular speed in radian per second")
  .default_value(1.0)
  .action([](const std::string & value) {return std::stod(value);});

  program.add_argument("points")
  .help("list of target points in `x1 y1 x2 y2 ...` format")
  .remaining();

  bool repeat;
  bool holonomic_mode;

  double linear_speed;
  double angular_speed;

  std::list<keisan::Point2> target_points;

  try {
    program.parse_args(argc, argv);

    repeat = program.get<bool>("--repeat");
    holonomic_mode = program.get<bool>("--holonomic-mode");

    linear_speed = program.get<double>("--linear-speed");
    angular_speed = program.get<double>("--angular-speed");

    // Parse target points
    auto points = program.get<std::vector<std::string>>("points");
    if (points.size() < 2) {
      throw std::runtime_error("points must contain atleast one x and one y");
    } else if (points.size() % 2 != 0) {
      throw std::runtime_error("missing y for the last target point");
    } else {
      for (size_t i = 1; i < points.size(); i += 2) {
        target_points.push_back({stod(points[i - 1]), stod(points[i])});
      }
    }
  } catch (const std::exception & e) {
    std::cout << e.what() << std::endl;
    std::cout << program;
    return 1;
  }

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("patrol_position");

  keisan::Point2 current_position;
  keisan::Angle current_orientation;
  auto odometry_subscription = node->create_subscription<Odometry>(
    "/odom", 10,
    [&](const Odometry::SharedPtr msg) {
      current_position = keisan::Point2(msg->pose.pose.position.x, msg->pose.pose.position.y);

      double yaw, pitch, roll;
      tf2::getEulerYPR(msg->pose.pose.orientation, yaw, pitch, roll);
      current_orientation = keisan::make_radian(yaw);
    });

  auto twist_publisher = node->create_publisher<Twist>("/cmd_vel", 10);

  auto target_point = target_points.begin();
  auto update_timer = node->create_wall_timer(
    10ms, [&]() {
      Twist twist;

      // Shift target point if near
      if (current_position.distance_to(*target_point) <= 0.3) {
        ++target_point;
        if (target_point == target_points.end()) {
          if (repeat) {
            target_point = target_points.begin();
          } else {
            RCLCPP_INFO(node->get_logger(), "Finished!");

            // Set movement into stop
            twist_publisher->publish(Twist());

            rclcpp::shutdown();
          }
        }
      }

      if (holonomic_mode) {
        auto delta_position = *target_point - current_position;
        if (delta_position.magnitude() > 1.0) {
          delta_position = delta_position.normalize();
        }

        twist.linear.x = delta_position.x * linear_speed;
        twist.linear.y = delta_position.y * linear_speed;
      } else {
        // Calculate a new target angular movement
        {
          auto direction = current_position.direction_to(*target_point);
          auto delta = current_orientation.difference_to(direction);

          twist.angular.z = keisan::clamp(delta.radian() * 3, -angular_speed, angular_speed);
        }

        // Calculate a new target linear movement
        double forward = keisan::map(
          std::abs(twist.angular.z), 0.0, angular_speed / 3, linear_speed, 0.0);
        twist.linear.x = std::max(forward, 0.0);
      }

      twist_publisher->publish(twist);
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

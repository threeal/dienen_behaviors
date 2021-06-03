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
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

using namespace std::chrono_literals;

using geometry_msgs::msg::Twist;

int main(int argc, char ** argv)
{
  auto program = argparse::ArgumentParser("move_for", "0.2.0");

  program.add_argument("duration")
  .help("Duration until finished in seconds")
  .action([](const std::string & value) {return std::stod(value);});

  program.add_argument("-f", "--forward")
  .help("Set forward maneuver in meter per minute")
  .action([](const std::string & value) {return std::stod(value);})
  .default_value(0.0);

  program.add_argument("-l", "--left")
  .help("Set left maneuver in meter per minute")
  .action([](const std::string & value) {return std::stod(value);})
  .default_value(0.0);

  program.add_argument("-y", "--yaw")
  .help("Set yaw maneuver in radian per minute")
  .action([](const std::string & value) {return std::stod(value);})
  .default_value(0.0);

  // Try to parse arguments
  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error & err) {
    std::cout << err.what() << std::endl;
    std::cout << program;
    return 1;
  }

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("move_for");

  auto twist_publisher = node->create_publisher<Twist>("/cmd_vel", 10);

  auto start_time = node->now();

  rclcpp::TimerBase::SharedPtr update_timer;

  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "\n" <<
      "Move for " << program.get<double>("duration") << " seconds with maneuver:\n" <<
      "- forward : " << program.get<double>("--forward") << " m/min\n" <<
      "- left\t  : " << program.get<double>("--left") << " m/min\n" <<
      "- yaw\t  : " << program.get<double>("--yaw") << " rad/min");

  // Update process
  update_timer = node->create_wall_timer(
    10ms, [&]() {
      auto duration = node->now() - start_time;

      if (duration.seconds() < program.get<double>("duration")) {
        Twist twist;

        twist.linear.x = program.get<double>("--forward");
        twist.linear.y = program.get<double>("--left");
        twist.angular.z = program.get<double>("--yaw");

        twist_publisher->publish(twist);
      } else {
        RCLCPP_INFO(node->get_logger(), "Finished!");

        // Set maneuver into stop
        twist_publisher->publish(Twist());

        rclcpp::shutdown();
      }
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

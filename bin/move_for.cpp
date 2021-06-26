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
#include <rosgraph_msgs/msg/clock.hpp>
#include <tosshin/tosshin.hpp>

#include <memory>
#include <string>
#include <vector>

namespace tsn = tosshin;

using namespace std::chrono_literals;

using rosgraph_msgs::msg::Clock;

int main(int argc, char ** argv)
{
  auto program = argparse::ArgumentParser("move_for", "0.2.0");

  program.add_argument("-d", "--duration")
  .help("duration until finished in seconds, move forever on negative duration")
  .default_value(-1.0)
  .action([](const std::string & value) {return std::stod(value);});

  program.add_argument("-l", "--linear")
  .help("set linear speed (x, y, z) in meter per second")
  .nargs(3)
  .default_value(std::vector<double>{0.0, 0.0, 0.0})
  .action([](const std::string & value) {return std::stod(value);});

  program.add_argument("-a", "--angular")
  .help("set angular speed (x, y, z) in radian per second")
  .nargs(3)
  .default_value(std::vector<double>{0.0, 0.0, 0.0})
  .action([](const std::string & value) {return std::stod(value);});

  program.add_argument("--use-sim-time")
  .help("use simulation time to measures duration")
  .default_value(false)
  .implicit_value(true);

  double target_duration;
  bool move_forever;

  bool use_sim_time;

  std::vector<double> linear;
  std::vector<double> angular;

  // Try to parse arguments
  try {
    program.parse_args(argc, argv);

    target_duration = program.get<double>("--duration");
    move_forever = (target_duration < 0.0);

    use_sim_time = program.get<bool>("--use-sim-time");

    linear = program.get<std::vector<double>>("--linear");
    angular = program.get<std::vector<double>>("--angular");
  } catch (const std::exception & e) {
    std::cout << e.what() << std::endl;
    std::cout << program;
    return 1;
  }

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("move_for");

  auto twist_publisher = node->create_publisher<tsn::msg::Twist>("/cmd_vel", 10);

  // Print arguments information
  {
    if (move_forever) {
      RCLCPP_INFO(node->get_logger(), "Move forever");
    } else {
      RCLCPP_INFO_STREAM(node->get_logger(), "Move for " << target_duration << " seconds");
    }

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "linear speed\t: " << linear[0] << " " << linear[1] << " " << linear[2] << " m/s");

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "angular speed\t: " << angular[0] << " " << angular[1] << " " << angular[2] << " rad/s");
  }

  auto update_process = [&](rclcpp::Duration duration) {
      if (move_forever || duration.seconds() < target_duration) {
        tsn::msg::Twist twist;

        twist.linear.x = linear[0];
        twist.linear.y = linear[1];
        twist.linear.z = linear[2];

        twist.angular.x = angular[0];
        twist.angular.y = angular[1];
        twist.angular.z = angular[2];

        twist_publisher->publish(twist);
      } else {
        RCLCPP_INFO(node->get_logger(), "Finished!");

        // Set movement into stop
        twist_publisher->publish(tsn::msg::Twist());

        rclcpp::shutdown();
      }
    };

  // Update process
  if (use_sim_time) {
    std::optional<rclcpp::Time> start_time;
    auto clock_subscription = node->create_subscription<Clock>(
      "/clock", 10,
      [&](const Clock::SharedPtr msg) {
        auto now = rclcpp::Time(msg->clock.sec, msg->clock.nanosec);

        if (!start_time) {
          start_time = std::make_optional(now);
        }

        update_process(now - start_time.value());
      });

    rclcpp::spin(node);
  } else {
    auto start_time = node->now();
    auto update_timer = node->create_wall_timer(
      10ms, [&]() {
        update_process(node->now() - start_time);
      });

    rclcpp::spin(node);
  }

  rclcpp::shutdown();

  return 0;
}

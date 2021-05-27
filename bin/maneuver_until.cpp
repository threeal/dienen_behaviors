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

#include <rclcpp/rclcpp.hpp>
#include <tosshin_cpp/tosshin_cpp.hpp>

#include <memory>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("maneuver_until");

  auto maneuver_consumer = std::make_shared<tosshin_cpp::ManeuverConsumer>(node);

  auto start_time = node->now();

  rclcpp::TimerBase::SharedPtr update_timer;

  update_timer = node->create_wall_timer(
    10ms, [&]() {
      auto duration = node->now() - start_time;
      if (duration.seconds() >= 10.0) {
        RCLCPP_INFO(node->get_logger(), "Finished!");

        // Stop update timer
        update_timer->cancel();

        // Create a timeout handler
        auto stop_timeout_timer = node->create_wall_timer(
          3s, [ = ]() {
            RCLCPP_ERROR(node->get_logger(), "Failed to stop the maneuver!");
            rclcpp::shutdown();
          });

        // Request to stop the maneuver
        RCLCPP_INFO(node->get_logger(), "Requesting to stop the maneuver...");
        maneuver_consumer->configure_maneuver(
          tosshin_cpp::Maneuver(),
          [ = ](const tosshin_cpp::Maneuver & /*maneuver*/) {
            RCLCPP_INFO(node->get_logger(), "Maneuver stopped!");

            stop_timeout_timer->cancel();
            rclcpp::shutdown();
          });
      } else {
        auto maneuver = maneuver_consumer->get_maneuver();

        maneuver.forward = 20.0;
        maneuver.left = 0.0;
        maneuver.yaw = 0.0;

        maneuver_consumer->set_maneuver(maneuver);
      }
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

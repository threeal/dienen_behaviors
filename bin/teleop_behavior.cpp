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

#include <dienen_behaviors/dienen_behaviors.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

using namespace std::chrono_literals;

std::shared_ptr<dienen_behaviors::TeleopBehavior> teleop_behavior;

int main(int argc, char ** argv)
{
  if (argc < 1) {
    std::cout << "Usage: ros2 run dienen_behaviors teleop_behavior" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("teleop_behavior");

  teleop_behavior = std::make_shared<dienen_behaviors::TeleopBehavior>(node);

  // Stop maneuver on keyboard interrupt
  signal(
    SIGINT, [](int /*code*/) {
      // Create a timeout handler
      auto timer = teleop_behavior->get_node()->create_wall_timer(
        3s, []() {
          RCLCPP_ERROR(teleop_behavior->get_node()->get_logger(), "Failed to stop the maneuver!");
          rclcpp::shutdown();
        });

      RCLCPP_INFO(teleop_behavior->get_node()->get_logger(), "Requesting to stop the maneuver...");
      teleop_behavior->configure_maneuver(
        tosshin_cpp::Maneuver(), [timer](const tosshin_cpp::Maneuver & /*maneuver*/) {
          RCLCPP_INFO(teleop_behavior->get_node()->get_logger(), "Maneuver stopped!");

          timer->cancel();
          rclcpp::shutdown();
        });
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

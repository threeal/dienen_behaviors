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

#ifndef DIENEN_BEHAVIORS__NAVIGATION_BEHAVIOR_HPP_
#define DIENEN_BEHAVIORS__NAVIGATION_BEHAVIOR_HPP_

#include <keisan/keisan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tosshin_interfaces/tosshin_interfaces.hpp>

#include <string>

namespace dienen_behaviors
{

using ManeuverMsg = tosshin_interfaces::msg::Maneuver;
using OrientationMsg = tosshin_interfaces::msg::Orientation;
using PositionMsg = tosshin_interfaces::msg::Position;
using ConfigureManeuverSrv = tosshin_interfaces::srv::ConfigureManeuver;

class NavigationBehavior
{
public:
  NavigationBehavior(std::string node_name, std::string navigation_node_name);

  virtual void on_update();

  void stop();

  rclcpp::Node::SharedPtr get_node();

protected:
  rclcpp::Node::SharedPtr node;

  rclcpp::Subscription<PositionMsg>::SharedPtr position_subscription;
  rclcpp::Subscription<OrientationMsg>::SharedPtr orientation_subscription;

  rclcpp::Publisher<ManeuverMsg>::SharedPtr maneuver_input_publisher;
  rclcpp::Client<ConfigureManeuverSrv>::SharedPtr configure_maneuver_client;

  rclcpp::TimerBase::SharedPtr update_timer;

  keisan::Point2 current_position;
  double current_yaw_orientation;

  double target_forward_maneuver;
  double target_left_maneuver;
  double target_yaw_maneuver;
};

}  // namespace dienen_behaviors

#endif  // DIENEN_BEHAVIORS__NAVIGATION_BEHAVIOR_HPP_

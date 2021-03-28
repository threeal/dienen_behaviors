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

#ifndef DIENEN_BEHAVIORS__PATROL_BEHAVIOR_HPP_
#define DIENEN_BEHAVIORS__PATROL_BEHAVIOR_HPP_

#include <dienen_behaviors/point.hpp>
#include <tosshin_interfaces/msg/maneuver.hpp>
#include <tosshin_interfaces/msg/orientation.hpp>
#include <tosshin_interfaces/msg/position.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace dienen_behaviors
{

using Maneuver = tosshin_interfaces::msg::Maneuver;
using Orientation = tosshin_interfaces::msg::Orientation;
using Position = tosshin_interfaces::msg::Position;

class PatrolBehavior : public rclcpp::Node
{
public:
  PatrolBehavior(std::string node_name, std::string navigation_node_name);

  void add_point(const Point & point);
  void add_point(const double & x, const double & y);

private:
  rclcpp::Subscription<Position>::SharedPtr position_subscription;
  rclcpp::Subscription<Orientation>::SharedPtr orientation_subscription;

  rclcpp::Publisher<Maneuver>::SharedPtr maneuver_input_publisher;

  rclcpp::TimerBase::SharedPtr update_timer;

  std::vector<Point> points;

  unsigned int current_point_index;

  Point current_position;
  double current_yaw_orientation;
};

}  // namespace dienen_behaviors

#endif  // DIENEN_BEHAVIORS__PATROL_BEHAVIOR_HPP_

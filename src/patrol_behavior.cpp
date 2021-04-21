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

#include <dienen_behaviors/patrol_behavior.hpp>

#include <memory>
#include <string>

namespace dienen_behaviors
{

PatrolBehavior::PatrolBehavior(std::string node_name, std::string navigation_node_name)
: NavigationBehavior(node_name, navigation_node_name),
  current_point_index(0)
{
}

void PatrolBehavior::on_update()
{
  // Reset the target maneuver
  target_forward_maneuver = 0.0;
  target_left_maneuver = 0.0;
  target_yaw_maneuver = 0.0;

  if (points.size() < 1) {
    RCLCPP_WARN_ONCE(node->get_logger(), "Once, no points provided!");
    return;
  }

  if (current_point_index >= points.size()) {
    RCLCPP_WARN(node->get_logger(), "Current point index overflowed!");
    current_point_index = current_point_index % points.size();
    return;
  }

  auto & target_point = points[current_point_index];

  // Shift target point if near
  auto distance = keisan::Point2::distance_between(current_position, target_point);
  if (distance <= 0.1) {
    current_point_index = (current_point_index + 1) % points.size();
    target_point = points[current_point_index];
  }

  // Calculate a new target yaw maneuver
  {
    double target_direction = atan2(
      target_point.y - current_position.y,
      target_point.x - current_position.x);

    double yaw = keisan::delta_deg(
      current_yaw_orientation, keisan::rad_to_deg(target_direction));

    if (yaw > 40.0) {
      yaw = 40.0;
    }

    if (yaw < -40.0) {
      yaw = -40.0;
    }

    target_yaw_maneuver = yaw;
  }

  // Calculate a new target forward maneuver
  {
    double forward = 60.0 - abs(target_yaw_maneuver * 3.0);

    if (forward < 0.0) {
      forward = 0.0;
    }

    target_forward_maneuver = forward;
  }
}

void PatrolBehavior::add_point(const keisan::Point2 & point)
{
  points.push_back(point);
}

void PatrolBehavior::add_point(const double & x, const double & y)
{
  add_point(keisan::Point2(x, y));
}

}  // namespace dienen_behaviors

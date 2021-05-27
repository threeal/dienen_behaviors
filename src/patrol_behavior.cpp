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

#include <algorithm>

namespace dienen_behaviors
{

using namespace std::chrono_literals;

PatrolBehavior::PatrolBehavior(rclcpp::Node::SharedPtr node)
: tosshin_cpp::NavigationConsumer(node),
  point_index(0),
  repeat(false),
  finished(false)
{
  // Initialize the update timer
  update_timer = get_node()->create_wall_timer(
    10ms, [this]() {
      on_update();
    });
}

void PatrolBehavior::on_update()
{
  tosshin_cpp::Maneuver target_maneuver;

  // Reset the target maneuver
  target_maneuver.forward = 0.0;
  target_maneuver.left = 0.0;
  target_maneuver.yaw = 0.0;

  // Stop if finished
  if (finished) {
    return set_maneuver(target_maneuver);
  }

  if (points.size() < 1) {
    RCLCPP_WARN_ONCE(get_node()->get_logger(), "Once, no point provided!");
    return set_maneuver(target_maneuver);
  }

  if (point_index >= points.size()) {
    RCLCPP_WARN(get_node()->get_logger(), "Point index overflowed!");
    point_index = point_index % points.size();
    return set_maneuver(target_maneuver);
  }

  auto odometry = get_odometry();

  auto current_point = keisan::Point2(odometry.position.x, odometry.position.y);
  auto target_point = points[point_index];

  // Shift target point if near
  auto distance = keisan::Point2::distance_between(current_point, target_point);
  if (distance <= 0.3) {
    if (!repeat && point_index + 1 >= points.size()) {
      // All target points is reached
      point_index = 0;
      finished = true;

      RCLCPP_INFO(get_node()->get_logger(), "Finished!");
    } else {
      point_index = (point_index + 1) % points.size();
    }

    target_point = points[point_index];
  }

  // Calculate a new target yaw maneuver
  {
    double direction = (target_point - current_point).direction();
    double yaw = keisan::delta_deg(odometry.orientation.yaw, keisan::rad_to_deg(direction));

    target_maneuver.yaw = keisan::clamp_number(yaw, -100.0, 100.0);
  }

  // Calculate a new target forward maneuver
  double forward = keisan::map_number(std::abs(target_maneuver.yaw), 0.0, 60.0, 60.0, 0.0);
  target_maneuver.forward = std::max(forward, 0.0);

  set_maneuver(target_maneuver);
}

void PatrolBehavior::add_point(const keisan::Point2 & point)
{
  points.push_back(point);
}

void PatrolBehavior::add_point(const double & x, const double & y)
{
  add_point(keisan::Point2(x, y));
}

void PatrolBehavior::enable_repeat(const bool & enabled)
{
  repeat = enabled;

  // finished state is invalid when repeat is enabled
  if (repeat) {
    finished = false;
  }

  RCLCPP_INFO_STREAM(
    get_node()->get_logger(),
    "Repeat is " << (repeat ? "enabled" : "disabled") << "!");
}

}  // namespace dienen_behaviors

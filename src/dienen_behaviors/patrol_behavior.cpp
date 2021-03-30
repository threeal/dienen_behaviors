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

using namespace std::chrono_literals;

const double PI = atan(1) * 4;

PatrolBehavior::PatrolBehavior(
  std::string node_name = "patrol_behavior",
  std::string navigation_node_name = "navigation")
: current_point_index(0),
  current_position(),
  current_yaw_orientation(0.0)
{
  // Initialize the node
  {
    node = std::make_shared<rclcpp::Node>(node_name);

    // Initialize the position subscription
    {
      position_subscription = node->create_subscription<Position>(
        navigation_node_name + "/position", 10,
        [this](const Position::SharedPtr position) {
          current_position.x = position->x;
          current_position.y = position->y;
        }
      );

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Position subscription initialized on " <<
          position_subscription->get_topic_name() << "!"
      );
    }

    // Initialize the orientation subscription
    {
      orientation_subscription = node->create_subscription<Orientation>(
        navigation_node_name + "/orientation", 10,
        [this](const Orientation::SharedPtr orientation) {
          current_yaw_orientation = orientation->yaw;
        }
      );

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Orientation subscription initialized on " <<
          orientation_subscription->get_topic_name() << "!"
      );
    }

    // Initialize the maneuver input publisher
    {
      maneuver_input_publisher = node->create_publisher<Maneuver>(
        navigation_node_name + "/maneuver_input", 10
      );

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Maneuver input publisher initialized on " <<
          maneuver_input_publisher->get_topic_name() << "!"
      );
    }

    // Initialize the configure maneuver client
    {
      configure_maneuver_client = node->create_client<ConfigureManeuver>(
        navigation_node_name + "/configure_maneuver"
      );

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Configure maneuver client initialized on " <<
          configure_maneuver_client->get_service_name() << "!"
      );
    }

    // Initialize the update timer
    {
      update_timer = node->create_wall_timer(
        30ms, [this]() {
          Maneuver maneuver;

          maneuver.forward.push_back(0);
          maneuver.yaw.push_back(0.0);

          if (points.size() < 1) {
            RCLCPP_WARN_ONCE(node->get_logger(), "Once, no points provided!");

            return maneuver_input_publisher->publish(maneuver);
          }

          if (current_point_index >= points.size()) {
            RCLCPP_WARN(node->get_logger(), "Current point index overflowed!");
            current_point_index = current_point_index % points.size();

            return maneuver_input_publisher->publish(maneuver);
          }

          auto & target_point = points[current_point_index];

          // Shift target point if near
          if (Point::distance(current_position, target_point) <= 0.1) {
            current_point_index = (current_point_index + 1) % points.size();

            return maneuver_input_publisher->publish(maneuver);
          }

          // Calculate and publish an input maneuver
          {
            double target_direction = Point::direction(current_position, target_point);

            double a = target_direction - current_yaw_orientation;
            double b = a + (a < 0.0 ? 360.0 : -360.0);

            double yaw = abs(a) < abs(b) ? a : b;
            double forward = 20.0 - abs(yaw);

            if (forward < 0.0) {
              forward = 0.0;
            }

            maneuver.forward[0] = forward;
            maneuver.yaw[0] = yaw;

            maneuver_input_publisher->publish(maneuver);
          }
        }
      );
    }
  }
}

void PatrolBehavior::add_point(const Point & point)
{
  points.push_back(point);
}

void PatrolBehavior::add_point(const double & x, const double & y)
{
  add_point(Point(x, y));
}

void PatrolBehavior::stop()
{
  // Stop the navigation's maneuver
  {
    auto request = std::make_shared<ConfigureManeuver::Request>();

    request->maneuver.forward.push_back(0.0);
    request->maneuver.left.push_back(0.0);
    request->maneuver.yaw.push_back(0.0);

    auto result_future = configure_maneuver_client->async_send_request(request);
    auto spin_result = rclcpp::spin_until_future_complete(node, result_future);

    if (spin_result != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        node->get_logger(),
        "Failed to call service on " <<
          configure_maneuver_client->get_service_name() << "!"
      );
    }
  }
}

}  // namespace dienen_behaviors

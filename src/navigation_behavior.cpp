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

#include <dienen_behaviors/navigation_behavior.hpp>

#include <memory>
#include <string>

namespace dienen_behaviors
{

using namespace std::chrono_literals;

NavigationBehavior::NavigationBehavior(std::string node_name, std::string navigation_node_name)
: current_position(),
  current_yaw_orientation(0.0),
  target_forward_maneuver(0.0),
  target_left_maneuver(0.0),
  target_yaw_maneuver(0.0)
{
  // Initialize the node
  {
    node = std::make_shared<rclcpp::Node>(node_name);

    // Initialize the position subscription
    {
      position_subscription = node->create_subscription<PositionMsg>(
        navigation_node_name + "/position", 10,
        [this](const PositionMsg::SharedPtr msg) {
          current_position.x = msg->x;
          current_position.y = msg->y;
        });

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Position subscription initialized on " <<
          position_subscription->get_topic_name() << "!");
    }

    // Initialize the orientation subscription
    {
      orientation_subscription = node->create_subscription<OrientationMsg>(
        navigation_node_name + "/orientation", 10,
        [this](const OrientationMsg::SharedPtr msg) {
          current_yaw_orientation = msg->yaw;
        });

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Orientation subscription initialized on " <<
          orientation_subscription->get_topic_name() << "!");
    }

    // Initialize the maneuver input publisher
    {
      maneuver_input_publisher = node->create_publisher<ManeuverMsg>(
        navigation_node_name + "/maneuver_input", 10);

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Maneuver input publisher initialized on " <<
          maneuver_input_publisher->get_topic_name() << "!");
    }

    // Initialize the configure maneuver client
    {
      configure_maneuver_client = node->create_client<ConfigureManeuverSrv>(
        navigation_node_name + "/configure_maneuver");

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Configure maneuver client initialized on " <<
          configure_maneuver_client->get_service_name() << "!");
    }

    // Initialize the update timer
    {
      update_timer = node->create_wall_timer(
        10ms, [this]() {
          // Process update
          on_update();

          // Publish Maneuver input
          {
            ManeuverMsg msg;

            msg.forward.push_back(target_forward_maneuver);
            msg.left.push_back(target_left_maneuver);
            msg.yaw.push_back(target_yaw_maneuver);

            maneuver_input_publisher->publish(msg);
          }
        }
      );
    }
  }
}

void NavigationBehavior::on_update()
{
}

void NavigationBehavior::stop()
{
  // Stop the navigation's maneuver
  {
    auto request = std::make_shared<ConfigureManeuverSrv::Request>();

    request->maneuver.forward.push_back(0.0);
    request->maneuver.left.push_back(0.0);
    request->maneuver.yaw.push_back(0.0);

    auto result_future = configure_maneuver_client->async_send_request(request);
    auto spin_result = rclcpp::spin_until_future_complete(node, result_future);

    if (spin_result != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        node->get_logger(),
        "Failed to call service on " <<
          configure_maneuver_client->get_service_name() << "!");
    }
  }
}

rclcpp::Node::SharedPtr NavigationBehavior::get_node()
{
  return node;
}

}  // namespace dienen_behaviors

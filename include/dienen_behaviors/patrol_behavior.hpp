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

#include <keisan/keisan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tosshin_interfaces/tosshin_interfaces.hpp>

#include <string>
#include <vector>

#include "./navigation_behavior.hpp"

namespace dienen_behaviors
{

class PatrolBehavior : public NavigationBehavior
{
public:
  PatrolBehavior(std::string node_name, std::string navigation_node_name);

  void on_update() override;

  void add_point(const keisan::Point2 & point);
  void add_point(const double & x, const double & y);

private:
  rclcpp::TimerBase::SharedPtr update_timer;

  std::vector<keisan::Point2> points;

  size_t current_point_index;
};

}  // namespace dienen_behaviors

#endif  // DIENEN_BEHAVIORS__PATROL_BEHAVIOR_HPP_

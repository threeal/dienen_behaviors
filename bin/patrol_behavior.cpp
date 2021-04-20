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

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::cout << "Usage: ros2 run dienen_behaviors patrol_behavior " <<
      "<navigation_node_name> <point1_x> <point1_y> [ <point2_x> <point2_y> ... ]" << std::endl;
    return 1;
  }

  std::string navigation_node_name = argv[1];

  rclcpp::init(argc, argv);

  auto patrol_behavior = std::make_shared<dienen_behaviors::PatrolBehavior>(
    "patrol_behavior", navigation_node_name
  );

  for (int i = 3; i < argc; i += 2) {
    patrol_behavior->add_point(atof(argv[i - 1]), atof(argv[i]));
  }

  rclcpp::spin(patrol_behavior->node);

  rclcpp::init(argc, argv);

  patrol_behavior->stop();

  rclcpp::shutdown();

  return 0;
}

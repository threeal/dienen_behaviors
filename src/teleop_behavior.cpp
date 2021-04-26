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

#include <dienen_behaviors/teleop_behavior.hpp>

#include <ctype.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define STDIN 0

namespace dienen_behaviors
{

using namespace std::chrono_literals;

TeleopBehavior::TeleopBehavior(rclcpp::Node::SharedPtr node)
: tosshin_cpp::NavigationConsumer(node)
{
  // Initialize the update timer
  update_timer = get_node()->create_wall_timer(
    10ms, [this]() {
      on_update();
    });

  // Configure terminal
  {
    tcgetattr(STDIN, &original_term);

    nonblock_term = original_term;
    nonblock_term.c_lflag &= ~ICANON;
    nonblock_term.c_lflag &= ~ECHO;
    nonblock_term.c_lflag &= ~ISIG;
    nonblock_term.c_cc[VMIN] = 0;
    nonblock_term.c_cc[VTIME] = 0;

    tcsetattr(STDIN, TCSANOW, &nonblock_term);
  }
}

TeleopBehavior::~TeleopBehavior()
{
  tcsetattr(STDIN, TCSANOW, &original_term);
}

void TeleopBehavior::on_update()
{
  // Handle keyboard input
  {
    auto target_maneuver = get_maneuver();

    char input;

    // Modify maneuver according to the keyboard input
    while (read(STDIN, &input, 1) > 0) {
      switch (toupper(input)) {
        case 'W':
          if (target_maneuver.forward < 0.0) {
            target_maneuver.forward = 0.0;
            target_maneuver.left = 0.0;
            target_maneuver.yaw = 0.0;
            break;
          }

          target_maneuver.forward += 10.0;
          break;

        case 'S':
          if (target_maneuver.forward > 0.0) {
            target_maneuver.forward = 0.0;
            target_maneuver.left = 0.0;
            target_maneuver.yaw = 0.0;
            break;
          }

          target_maneuver.forward -= 10.0;
          break;

        case 'A':
          if (target_maneuver.left < 0.0) {
            target_maneuver.left = 0.0;
            break;
          }

          target_maneuver.left += 10.0;
          break;

        case 'D':
          if (target_maneuver.left > 0.0) {
            target_maneuver.left = 0.0;
            break;
          }

          target_maneuver.left -= 10.0;
          break;

        case 'Q':
          if (target_maneuver.yaw < 0.0) {
            target_maneuver.yaw = 0.0;
            break;
          }

          target_maneuver.yaw += 10.0;
          break;

        case 'E':
          if (target_maneuver.yaw > 0.0) {
            target_maneuver.yaw = 0.0;
            break;
          }

          target_maneuver.yaw -= 10.0;
          break;

        default:  // Escape
          // Temporarely reset the terminal configuration
          tcsetattr(STDIN, TCSANOW, &original_term);
          usleep(500 * 1000);
          tcsetattr(STDIN, TCSANOW, &nonblock_term);
      }

      set_maneuver(target_maneuver);
    }
  }

  // Output current information
  {
    // Clear screen
    std::cout << "\033[2J\033[2H" << std::endl;

    auto odometry = get_odometry();

    std::cout << "Position: " << odometry.position.x << " " << odometry.position.y << std::endl;
    std::cout << "Orientation: " << odometry.orientation.yaw << std::endl;

    std::cout << std::endl;

    auto maneuver = get_maneuver();

    std::cout << "Forward: " << maneuver.forward << std::endl;
    std::cout << "Left: " << maneuver.left << std::endl;
    std::cout << "Yaw: " << maneuver.yaw << std::endl;

    std::cout << std::endl;

    std::cout << "W/S = forward/backward" << std::endl;
    std::cout << "A/D = left/right" << std::endl;
    std::cout << "Q/E = rotate" << std::endl;
  }
}

}  // namespace dienen_behaviors

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

#include <string>
#include <iostream>

#define STDIN 0

namespace dienen_behaviors
{

TeleopBehavior::TeleopBehavior(std::string node_name, std::string navigation_node_name)
: NavigationBehavior(node_name, navigation_node_name)
{
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
    char input;

    // Modify maneuver according to the keyboard input
    while (read(STDIN, &input, 1) > 0) {
      switch (toupper(input)) {
        case 'W':
          if (target_forward_maneuver < 0.0) {
            target_forward_maneuver = 0.0;
            break;
          }

          target_forward_maneuver += 10.0;
          break;

        case 'S':
          if (target_forward_maneuver > 0.0) {
            target_forward_maneuver = 0.0;
            break;
          }

          target_forward_maneuver -= 10.0;
          break;

        case 'A':
          if (target_left_maneuver < 0.0) {
            target_left_maneuver = 0.0;
            break;
          }

          target_left_maneuver += 10.0;
          break;

        case 'D':
          if (target_left_maneuver > 0.0) {
            target_left_maneuver = 0.0;
            break;
          }

          target_left_maneuver -= 10.0;
          break;

        case 'Q':
          if (target_yaw_maneuver < 0.0) {
            target_yaw_maneuver = 0.0;
            break;
          }

          target_yaw_maneuver += 10.0;
          break;

        case 'E':
          if (target_yaw_maneuver > 0.0) {
            target_yaw_maneuver = 0.0;
            break;
          }

          target_yaw_maneuver -= 10.0;
          break;

        case 'R':
          target_forward_maneuver = 0.0;
          target_left_maneuver = 0.0;
          target_yaw_maneuver = 0.0;
          break;

        default:  // Escape
          // Temporarely reset the terminal configuration
          tcsetattr(STDIN, TCSANOW, &original_term);
          usleep(500 * 1000);
          tcsetattr(STDIN, TCSANOW, &nonblock_term);
      }
    }
  }

  // Output current information
  {
    // Clear screen
    std::cout << "\033[2J\033[2H" << std::endl;

    std::cout << "Position: " << current_position.x << " " << current_position.y << std::endl;
    std::cout << "Orientation: " << current_yaw_orientation << std::endl;

    std::cout << std::endl;

    std::cout << "Forward: " << target_forward_maneuver << std::endl;
    std::cout << "Left: " << target_left_maneuver << std::endl;
    std::cout << "Yaw: " << target_yaw_maneuver << std::endl;

    std::cout << std::endl;

    std::cout << "W/S = forward/backward" << std::endl;
    std::cout << "A/D = left/right" << std::endl;
    std::cout << "Q/E = rotate" << std::endl;
    std::cout << "R = reset" << std::endl;
  }
}

}  // namespace dienen_behaviors

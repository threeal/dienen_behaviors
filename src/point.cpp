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

#include <dienen_behaviors/point.hpp>

#include <cmath>

namespace dienen_behaviors
{

const double PI = atan(1) * 4;

Point::Point(const double & x, const double & y)
: x(x), y(y)
{
}

Point::Point(const Point & other)
: Point(other.x, other.y)
{
}

Point::Point()
: Point(0, 0)
{
}

double Point::distance(const Point & a, const Point & b)
{
  return sqrt(pow(b.x - a.x, 2.0) + pow(b.y - a.y, 2.0));
}

double Point::direction(const Point & a, const Point & b)
{
  double angle = atan2(b.y - a.y, b.x - a.x);

  while (angle < PI) {
    angle += 2 * PI;
  }

  while (angle > PI) {
    angle -= 2 * PI;
  }

  return angle * 180.0 / PI;
}

Point & Point::operator=(const Point & other)
{
  this->x = other.x;
  this->y = other.y;

  return *this;
}

Point Point::operator-(const Point & other)
{
  return Point(x - other.x, y - other.y);
}

}  // namespace dienen_behaviors

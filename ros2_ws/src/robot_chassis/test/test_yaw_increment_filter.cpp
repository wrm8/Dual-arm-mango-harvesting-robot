#include "robot_chassis/yaw_increment_filter.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition, const char *message)
{
  if (!condition) {throw std::runtime_error(message);}
}

bool close(double a, double b)
{
  return std::abs(a - b) < 1e-10;
}

int main()
{
  using robot_chassis::YawIncrementFilter;
  YawIncrementFilter filter;
  require(close(filter.update(0.3).yaw, 0.3), "initial heading must be preserved");
  // Thousands of rejected increments must not eventually leak through as drift.
  for (int i = 1; i <= 2000; ++i)
  {
    const auto sample = filter.update(0.3 + i * 0.0004);
    require(sample.suppressed && close(sample.delta, 0.0) && close(sample.yaw, 0.3),
      "small positive increments accumulated");
  }
  auto sample = filter.update(1.09);
  require(close(sample.delta, -0.01) && close(sample.yaw, 0.29),
    "accepted reverse motion must use the previous raw sample");
  sample = filter.update(1.0896);
  require(sample.suppressed && close(sample.yaw, 0.29), "negative drift must also be rejected");
  sample = filter.update(1.11);
  require(close(sample.delta, 0.0204) && close(sample.yaw, 0.3104),
    "real positive motion must survive after discarded drift");

  YawIncrementFilter boundary;
  boundary.update(0.0);
  require(!boundary.update(0.001).suppressed, "the threshold itself must be accepted");

  YawIncrementFilter wrapped;
  constexpr double pi = 3.14159265358979323846;
  wrapped.update(pi - 0.0002);
  sample = wrapped.update(-pi + 0.0002);
  require(sample.suppressed, "small wraparound must not look like a full turn");
  sample = wrapped.update(-pi + 0.0102);
  require(close(sample.delta, 0.01), "real turn across pi must remain continuous");

  YawIncrementFilter disabled;
  disabled.set_threshold(0.0);
  disabled.update(0.2);
  sample = disabled.update(0.2001);
  require(!sample.suppressed && close(sample.yaw, 0.2001), "zero threshold must disable filtering");
  for (const double invalid : {-0.001, 4.0, std::numeric_limits<double>::quiet_NaN()})
  {
    bool rejected = false;
    try {disabled.set_threshold(invalid);} catch (const std::invalid_argument &) {rejected = true;}
    require(rejected, "invalid threshold accepted");
  }
  std::cout << "yaw increment filtering: drift, real turns, wraparound, boundary and disable passed\n";
}

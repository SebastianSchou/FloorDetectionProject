#include <math.h>
#include "traversable_area_detection/helper_function.hpp"

float msUntilNow(
  const std::chrono::steady_clock::time_point& start)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - start).count() / 1000.0;
}

double square(const double v)
{
  return v * v;
}

double roundToNearestValue(const double v, const double value)
{
  return std::floor(v * (1.0 / value)) * value;
}

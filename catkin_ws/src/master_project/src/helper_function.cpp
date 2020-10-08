#include "master_project/helper_function.hpp"

float msUntilNow(
  const std::chrono::steady_clock::time_point& start)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - start).count() / 1000.0;
}

#include <math.h>
#include "master_project/helper_function.hpp"

float msUntilNow(
  const std::chrono::steady_clock::time_point& start)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - start).count() / 1000.0;
}

cv::Mat normalizeVector(const cv::Mat& v)
{
  if (std::min(v.rows, v.cols) != 1) {
    return v;
  }
  double  temp = 0.0;
  int     length = std::max(v.rows, v.cols);
  cv::Mat norm = v;
  for (int i = 0; i < length; i++) {
    temp += v.at<double>(i) * v.at<double>(i);
  }
  temp = std::sqrt(temp);
  return norm.mul(1 / temp);
}

double square(const double v)
{
  return (v * v);
}

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
  return v * v;
}

bool isMatEqual(const cv::Mat m1, const cv::Mat m2)
{
  if ((m1.rows != m2.rows) || (m1.cols != m2.cols)) {
    return false;
  }
  for (int r = 0; r < m1.rows; r++) {
    for (int c = 0; c < m1.cols; c++) {
      if (m1.at<double>(r, c) != m2.at<double>(r, c)) {
        return false;
      }
    }
  }
  return true;
}

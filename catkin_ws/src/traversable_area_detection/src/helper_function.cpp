#include <math.h>
#include "traversable_area_detection/helper_function.hpp"

float msUntilNow(
  const std::chrono::steady_clock::time_point& start)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - start).count() / 1000.0;
}

cv::Mat normalizeVector(const cv::Mat& v)
{
  double vectorLength = squareNorm(v);

  if (vectorLength == 0.0) {
    return v;
  }
  return v.mul(1 / vectorLength);
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

double squareNorm(const cv::Mat& v)
{
  if (std::min(v.rows, v.cols) != 1) {
    return 0.0;
  }
  double  vectorLength = 0.0;
  int     vectorSize = std::max(v.rows, v.cols);
  cv::Mat norm = v;
  for (int i = 0; i < vectorSize; i++) {
    vectorLength += square(v.at<double>(i));
  }
  vectorLength = std::sqrt(vectorLength);
  return vectorLength;
}

double roundToNearestValue(const double v, const double value)
{
  return std::floor(v * (1.0 / value)) * value;
}

Incrementer::Incrementer(const float min, const float max, const float step)
{
  min_ = min;
  max_ = max;
  step_ = step;
  value_ = (max - min) / 2;
  increase_ = 1;
}

float Incrementer::increment()
{
  if (value_ > max_) {
    increase_ = -1;
  } else if (value_ < min_) {
    increase_ = 1;
  }
  value_ += step_ * increase_;
  return value_;
}

float Incrementer::getValue()
{
  return value_;
}

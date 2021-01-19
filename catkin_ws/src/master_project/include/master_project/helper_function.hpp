#ifndef HELPER_FUNCTION_HPP
#define HELPER_FUNCTION_HPP

#include <chrono>
#include <opencv4/opencv2/opencv.hpp>

const double PI2 = 2.0 * CV_PI;
static const double SQRTOFCUBIC2PI = 2.0 * sqrt(2.0) * pow(CV_PI, 1.5);
const double NONZERO = 0.00001;

float   msUntilNow(const std::chrono::steady_clock::time_point& start);
cv::Mat normalizeVector(const cv::Mat& v);
double  square(const double v);
bool    isMatEqual(const cv::Mat m1, const cv::Mat m2);
double  squareNorm(const cv::Mat& v);
double  roundToNearestValue(const double v, const double value);

class Incrementer {
public:

  Incrementer(const float min, const float max, const float step);
  float increment();
  float getValue();

private:

  float min_, max_, step_, value_;
  int increase_;
};

#endif // HELPER_FUNCTION_HPP

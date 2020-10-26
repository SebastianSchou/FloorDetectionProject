#ifndef HELPER_FUNCTION_HPP
#define HELPER_FUNCTION_HPP

#include <chrono>
#include <opencv4/opencv2/opencv.hpp>

const double PI = acos(-1);
const double PI2 = 2.0 * PI;
static const double root22pi32 = 2.0 * sqrt(2.0) * pow(PI, 1.5);

float   msUntilNow(const std::chrono::steady_clock::time_point& start);
cv::Mat normalizeVector(const cv::Mat& v);
double  square(const double v);

#endif // HELPER_FUNCTION_HPP

#ifndef SUMMED_AREA_TABLE_HPP
#define SUMMED_AREA_TABLE_HPP

#include <opencv4/opencv2/opencv.hpp>

class Sat {
public:

  Sat()
  {
  }

  Sat(const int height, const int width)
  {
    this->sat = cv::Mat::zeros(cv::Size(width, height), CV_64F);
  }

  cv::Mat sat;

  void set(const int row, const int col, const double value)
  {
    this->sat.at<double>(row, col) = value;
  }

  double get(const int row, const int col)
  {
    if ((row < 0) || (row >= this->sat.rows) || (col < 0) ||
        (col >= this->sat.cols)) {
      return 0.0;
    }
    return this->sat.at<double>(row, col);
  }

  void setSumValue(const int row, const int col)
  {
    this->set(row, col,
              this->get(row, col) +
              this->get(row - 1, col) +
              this->get(row, col - 1) -
              this->get(row - 1, col - 1));
  }

  double getArea(const cv::Point& from, const cv::Point& to)
  {
    if ((to.x < from.x) || (to.y < from.y) ||
        (from.x < 0) || (from.y < 0) ||
        (to.x >= this->sat.cols) || (to.y >= this->sat.rows)) {
      return 0;
    }
    return this->get(to.y, to.x) -
           this->get(to.y, from.x - 1) -
           this->get(from.y - 1, to.x) +
           this->get(from.y - 1, from.x - 1);
  }

  cv::Mat getMat()
  {
    return this->sat;
  }
};

class SummedAreaTable {
public:

  SummedAreaTable()
  {
  }

  SummedAreaTable(const int height, const int width)
  {
    this->satX = Sat(height, width);
    this->satY = Sat(height, width);
    this->satZ = Sat(height, width);
    this->satXX = Sat(height, width);
    this->satXY = Sat(height, width);
    this->satXZ = Sat(height, width);
    this->satYY = Sat(height, width);
    this->satYZ = Sat(height, width);
    this->satZZ = Sat(height, width);
    this->satSamples = Sat(height, width);
    this->satGradient = Sat(height, width);
  }

  Sat satX;
  Sat satY;
  Sat satZ;
  Sat satXX;
  Sat satXY;
  Sat satXZ;
  Sat satYY;
  Sat satYZ;
  Sat satZZ;
  Sat satSamples;
  Sat satGradient;
};

#endif // SUMMED_AREA_TABLE_HPP

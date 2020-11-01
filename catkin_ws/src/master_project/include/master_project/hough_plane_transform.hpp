#ifndef HOUGH_PLANE_TRANSFORM_HPP
#define HOUGH_PLANE_TRANSFORM_HPP

#include "master_project/camera_data.hpp"
#include "master_project/quadtree.hpp"
#include "master_project/accumulator.hpp"
#include "master_project/voting.hpp"
#include "master_project/peak_detection.hpp"

class HoughPlaneTransform {
public:

  HoughPlaneTransform(CameraData& cameraData)
  {
    auto start = std::chrono::steady_clock::now();

    root.initializeRoot(cameraData);
    root.divideIntoQuadrants();
    timeQuadtree = msUntilNow(start);
    float rhoDelta = 0.04; // [m]
    accumulator = new Accumulator(root.maxPlaneDistance, rhoDelta, 30);
    voting(root, *accumulator, usedBins, usedKernels);
    timeVoting = msUntilNow(start) - timeQuadtree;
    peakDetection(planes, *accumulator, usedKernels, usedBins);
    timePeak = msUntilNow(start) - timeQuadtree - timeVoting;
  }

  ~HoughPlaneTransform()
  {
  }

  void assignColorToPlanes()
  {
    std::sort(planes.begin(), planes.end());
    for (unsigned int i = 0; i < planes.size(); i++) {
      int colorValue = (int)(255 / (int)(i / 6 + 1));
      cv::Mat color(cv::Size(3, 1), CV_8U, cv::Scalar(0));
      switch (i % 6) {
        case 0:
          color.at<uchar>(0) = colorValue;
          break;
        case 1:
          color.at<uchar>(1) = colorValue;
          break;
        case 2:
          color.at<uchar>(2) = colorValue;
          break;
        case 3:
          color.at<uchar>(1) = colorValue;
          color.at<uchar>(2) = colorValue;
          break;
        case 4:
          color.at<uchar>(0) = colorValue;
          color.at<uchar>(2) = colorValue;
          break;
        case 5:
          color.at<uchar>(0) = colorValue;
          color.at<uchar>(1) = colorValue;
          color.at<uchar>(2) = colorValue;
          break;
      }
      planes[i].color = color;
      for (Quadtree *node : planes[i].nodes) {
        node->color = color;
      }
    }
  }

  void printTimePartition()
  {
    ROS_INFO("Quadtree: %.3f ms. Voting: %.3f ms. Peak detection: %.3f ms",
             timeQuadtree, timeVoting, timePeak);
  }

  Quadtree root;
  Accumulator *accumulator;
  std::vector<Bin> usedBins;
  std::vector<Kernel> usedKernels;
  std::vector<Plane> planes;
  double timeQuadtree, timeVoting, timePeak;
};

#endif // HOUGH_PLANE_TRANSFORM_HPP

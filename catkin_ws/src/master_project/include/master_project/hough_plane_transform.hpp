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

    decimationFactor = cameraData.filterVariables.decimationScaleFactor;

    root.initializeRoot(cameraData);
    root.divideIntoQuadrants();
    timeQuadtree = msUntilNow(start);
    float rhoDelta = 0.04; // [m]
    accumulator = new Accumulator(root.maxPlaneDistance, rhoDelta, 30);
    voting(root, *accumulator, usedBins, usedKernels);
    timeVoting = msUntilNow(start) - timeQuadtree;
    peakDetection(planes, *accumulator, usedKernels, usedBins);
    std::sort(planes.begin(), planes.end());
    timePeak = msUntilNow(start) - timeQuadtree - timeVoting;
  }

  ~HoughPlaneTransform()
  {
  }

  void assignColorToPlanes()
  {
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

  void printPlaneInformation()
  {
    printf("=============PLANE INFORMATION=============\n");
    for (const Plane& plane : planes) {
      printf("Samples: %d, nodes: %ld, normal: [%.3f, %.3f, %.3f], "
             "mean: [%.3f, %.3f, %.3f]\n",
             plane.samples, plane.nodes.size(),
             plane.normal.at<double>(0), plane.normal.at<double>(1),
             plane.normal.at<double>(2), plane.mean.at<double>(0),
             plane.mean.at<double>(1), plane.mean.at<double>(2));
      for (const Quadtree *node : plane.nodes) {
        printf("  Node at (%d, %d) to (%d, %d) with %d samples:\n",
               node->minBounds.x * decimationFactor,
               node->minBounds.y * decimationFactor,
               node->maxBounds.x * decimationFactor,
               node->maxBounds.y * decimationFactor,
               node->samples);
        printf("    Normal: [%.3f, %.3f, %.3f], mean: [%.3f, %.3f, %.3f],"
               " node distance: %.3f\n",
               node->normal.at<double>(0),
               node->normal.at<double>(1),
               node->normal.at<double>(2),
               node->mean.at<double>(0),
               node->mean.at<double>(1),
               node->mean.at<double>(2),
               node->mean.dot(node->normal));
      }
    }
  }

  Quadtree root;
  Accumulator *accumulator;
  std::vector<Bin> usedBins;
  std::vector<Kernel> usedKernels;
  std::vector<Plane> planes;
  double timeQuadtree, timeVoting, timePeak;
  int decimationFactor;
};

#endif // HOUGH_PLANE_TRANSFORM_HPP

#ifndef HOUGH_PLANE_TRANSFORM_HPP
#define HOUGH_PLANE_TRANSFORM_HPP

#include "traversable_area_detection/camera_data.hpp"
#include "traversable_area_detection/quadtree.hpp"
#include "traversable_area_detection/accumulator.hpp"
#include "traversable_area_detection/voting.hpp"
#include "traversable_area_detection/peak_detection.hpp"
#include "traversable_area_detection/plane_analysis.hpp"

class HoughPlaneTransform {
public:

  HoughPlaneTransform(CameraData& cameraData,
                      const bool  printTime = false)
  {
    auto start = std::chrono::steady_clock::now();

    root.initializeRoot(cameraData);
    float timeInitializeQuadtree = msUntilNow(start);
    root.divideIntoQuadrants();
    float timeQuadtree = msUntilNow(start) - timeInitializeQuadtree;
    accumulator = new Accumulator(root.maxPlaneDistance, root.maxPhiAngle,
                                  MAX_DISTANCE_DIFFERENCE,
                                  MAX_ANGLE_DIFFERENCE);
    voting(root, *accumulator, usedBins, usedKernels);
    float timeVoting = msUntilNow(start) - timeQuadtree -
                       timeInitializeQuadtree;
    peakDetection(planes, *accumulator, usedBins, usedKernels);
    std::sort(planes.begin(), planes.end());
    float timePeak = msUntilNow(start) - timeQuadtree - timeVoting -
                     timeInitializeQuadtree;
    if (printTime) {
      ROS_INFO("Initialize quadtree: %.3f ms. Quadtree: %.3f ms. "
               "Voting: %.3f ms. Peak detection: %.3f ms",
               timeInitializeQuadtree, timeQuadtree, timeVoting, timePeak);
    }
  }

  ~HoughPlaneTransform()
  {
    delete accumulator;
    for (Plane& plane : planes) {
      plane.image2dPoints.release();
      plane.topView.release();
    }
  }

  Quadtree root;
  Accumulator *accumulator;
  std::vector<Bin> usedBins;
  std::vector<Kernel> usedKernels;
  std::vector<Plane> planes;
};

#endif // HOUGH_PLANE_TRANSFORM_HPP

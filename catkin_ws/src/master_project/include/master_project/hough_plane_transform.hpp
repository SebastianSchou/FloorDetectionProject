#ifndef HOUGH_PLANE_TRANSFORM_HPP
#define HOUGH_PLANE_TRANSFORM_HPP

#include "master_project/camera_data.hpp"
#include "master_project/quadtree.hpp"
#include "master_project/accumulator.hpp"
#include "master_project/voting.hpp"
#include "master_project/peak_detection.hpp"
#include "master_project/plane_analysis.hpp"

class HoughPlaneTransform {
public:
  HoughPlaneTransform(CameraData& cameraData,
                      const bool printTime = false)
  {
    auto start = std::chrono::steady_clock::now();

    root.initializeRoot(cameraData);
    float timeInitializeQuadtree = msUntilNow(start);
    root.divideIntoQuadrants();
    float timeQuadtree = msUntilNow(start) - timeInitializeQuadtree;
    float rhoDelta = 0.08;             // [m]
    float phiDelta = 4.0 * CV_PI / 180.0; // [radians]
    accumulator = new Accumulator(root.maxPlaneDistance, root.maxPhiAngle,
                                  rhoDelta, phiDelta);
    voting(root, *accumulator, usedBins, usedKernels);
    float timeVoting = msUntilNow(start) - timeQuadtree -
                       timeInitializeQuadtree;
    peakDetection(planes, *accumulator, usedBins, usedKernels);
    PlaneAnalysis::removeSmallPlanes(planes);
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
  }

  Quadtree root;
  Accumulator *accumulator;
  std::vector<Bin> usedBins;
  std::vector<Kernel> usedKernels;
  std::vector<Plane> planes;
};

#endif // HOUGH_PLANE_TRANSFORM_HPP

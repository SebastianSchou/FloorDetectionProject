#ifndef HOUGH_PLANE_TRANSFORM_HPP
#define HOUGH_PLANE_TRANSFORM_HPP

#include "master_project/camera_data.hpp"
#include "master_project/quadtree.hpp"
#include "master_project/accumulator.hpp"
#include "master_project/voting.hpp"
#include "master_project/peak_detection.hpp"

class HoughPlaneTransform {
public:

  HoughPlaneTransform(CameraData& cameraData);
  ~HoughPlaneTransform();
  void assignColorToPlanes();
  void assignColorToPlane(Plane &plane, int r, int g, int b);
  void printTimePartition();
  void printPlanesInformation();
  void printPlaneInformation(const Plane &plane);

  Quadtree root;
  Accumulator *accumulator;
  std::vector<Bin> usedBins;
  std::vector<Kernel> usedKernels;
  std::vector<Plane> planes;
  double timeQuadtree, timeVoting, timePeak;
  int decimationFactor;
};

#endif // HOUGH_PLANE_TRANSFORM_HPP

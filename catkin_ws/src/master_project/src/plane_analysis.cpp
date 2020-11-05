#include "master_project/plane_analysis.hpp"

Plane PlaneAnalysis::getGroundPlane(std::vector<Plane> &planes)
{
  int maxInclineDegrees = 15;
  float maxInclineRadians = (float)maxInclineDegrees * PI / 180.0;
  Plane floor; floor.rho = 0.0;
  for (Plane &plane : planes) {
    // Check if incline of the plane is (mostly) horizontal, that the
    // distance to the plane is furthest below the camera, that the 
    // y-direction of the normal is positive (exclude ceilings), and
    // that a minimum number of samples are in the plane
    if (std::abs(std::abs(plane.phi) - PI / 2.0) < maxInclineRadians &&
        std::abs(std::abs(plane.theta) - PI / 2.0) < maxInclineRadians &&
        floor.rho < plane.rho && plane.samples > 500 &&
        plane.normal.at<double>(1) > 0) {
      floor = plane;
    }
  }
  return floor;
}
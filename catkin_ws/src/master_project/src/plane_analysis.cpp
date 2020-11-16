#include "master_project/plane_analysis.hpp"

Plane PlaneAnalysis::getGroundPlane(std::vector<Plane>& planes)
{
  int   maxInclineDegrees = 15;
  float maxInclineRadians = (float)maxInclineDegrees * PI / 180.0;
  Plane floor; floor.rho = 0.0;

  for (Plane& plane : planes) {
    // Check if incline of the plane is (mostly) horizontal, that the
    // distance to the plane is furthest below the camera, that the
    // y-direction of the normal is positive (exclude ceilings), and
    // that a minimum number of samples are in the plane
    if ((std::abs(std::abs(plane.phi) - PI / 2.0) < maxInclineRadians) &&
        (std::abs(std::abs(plane.theta) - PI / 2.0) < maxInclineRadians) &&
        (floor.rho < plane.rho) && (plane.samples > 500) &&
        (plane.normal.at<double>(1) > 0)) {
      floor = plane;
    }
  }
  return floor;
}

bool PlaneAnalysis::hasSimilarAngle(const Plane& plane1, const Plane& plane2)
{
  return std::abs(plane1.phiAbs - plane2.phiAbs) < MAX_ANGLE_DIFFERENCE &&
         std::abs(plane1.thetaAbs - plane2.thetaAbs) < MAX_ANGLE_DIFFERENCE;
}

double PlaneAnalysis::getAngleDifference(const Plane& plane1,
                                         const Plane& plane2)
{
  return std::max(std::abs(plane1.phi - plane2.phi),
                  std::abs(plane1.theta - plane2.theta));
}

bool PlaneAnalysis::hasSimilarDistance(const Plane& plane1, const Plane& plane2)
{
  return std::abs(plane1.rho - plane2.rho) < MAX_DISTANCE_DIFFERENCE;
}

double PlaneAnalysis::getDistanceDifference(const Plane& plane1,
                                            const Plane& plane2)
{
  return std::abs(plane1.rho - plane2.rho);
}

bool PlaneAnalysis::isSimilar(const Plane& plane1, const Plane& plane2)
{
  return hasSimilarDistance(plane1, plane2) && hasSimilarAngle(plane1, plane2);
}

void PlaneAnalysis::transferNodes(Plane& plane1, Plane& plane2)
{
  for (size_t i = 0; i < plane2.nodes.size(); i++) {
    Quadtree *node = plane2.nodes[i];
    if (find(plane1.nodes.begin(), plane1.nodes.end(),
             node) != plane1.nodes.end()) {
      if (plane2.nodes.size() <= 1) {
        return;
      }
      plane2.nodes.erase(plane2.nodes.begin() + i);
      continue;
    } else {
      plane1.rootRepresentativeness += node->rootRepresentativeness;
      plane1.mean += node->mean;
      plane1.samples += node->samples;
    }
  }
  std::move(plane2.nodes.begin(), plane2.nodes.end(),
            std::inserter(plane1.nodes, plane1.nodes.end()));
}

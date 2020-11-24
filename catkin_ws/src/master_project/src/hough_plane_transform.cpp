#include "master_project/hough_plane_transform.hpp"
#include "master_project/plane_analysis.hpp"

HoughPlaneTransform::HoughPlaneTransform(CameraData& cameraData)
{
  auto start = std::chrono::steady_clock::now();

  decimationFactor = cameraData.filterVariables.decimationScaleFactor;

  root.initializeRoot(cameraData);
  root.divideIntoQuadrants();
  timeQuadtree = msUntilNow(start);
  float rhoDelta = 0.08;             // [m]
  float phiDelta = 4.0 * PI / 180.0; // [radians]
  accumulator = new Accumulator(root.maxPlaneDistance, root.maxPhiAngle,
                                rhoDelta, phiDelta);
  voting(root, *accumulator, usedBins, usedKernels);
  timeVoting = msUntilNow(start) - timeQuadtree;
  peakDetection(planes, *accumulator, usedKernels, usedBins);
  PlaneAnalysis::removeSmallPlanes(planes);
  std::sort(planes.begin(), planes.end());
  timePeak = msUntilNow(start) - timeQuadtree - timeVoting;
}

HoughPlaneTransform::~HoughPlaneTransform()
{
}

void HoughPlaneTransform::printTimePartition()
{
  ROS_INFO("Quadtree: %.3f ms. Voting: %.3f ms. Peak detection: %.3f ms",
           timeQuadtree, timeVoting, timePeak);
}

void HoughPlaneTransform::printPlanesInformation()
{
  printf("=============PLANE INFORMATION=============\n");
  for (const Plane& plane : planes) {
    printPlaneInformation(plane);
  }
}

void HoughPlaneTransform::printPlaneInformation(const Plane& plane)
{
  printf("Plane %d:\nSamples: %d, nodes: %ld, normal: [%.3f, %.3f, %.3f], "
         "position: [%.3f, %.3f, %.3f]\n"
         "Distance to plane: %.3f, phi: %.3f, theta: %.3f\n",
         plane.id, plane.samples, plane.nodes.size(),
         plane.normal.at<double>(0), plane.normal.at<double>(1),
         plane.normal.at<double>(2), plane.position.at<double>(0),
         plane.position.at<double>(1), plane.position.at<double>(2),
         plane.rho, plane.phi, plane.theta);
  for (const Quadtree *node : plane.nodes) {
    printf("  Node %d at (%d, %d) to (%d, %d) with %d samples:\n",
           node->id, node->minBounds.x * decimationFactor,
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

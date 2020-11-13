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
  std::sort(planes.begin(), planes.end());
  timePeak = msUntilNow(start) - timeQuadtree - timeVoting;
}

HoughPlaneTransform::~HoughPlaneTransform()
{
}

void HoughPlaneTransform::assignColorToPlane(Plane& plane, int r, int g, int b)
{
  cv::Mat color(cv::Size(3, 1), CV_8U, cv::Scalar(0));

  // Saturate the color values between 0 and 255
  r = std::max(0, std::min(255, r));
  g = std::max(0, std::min(255, g));
  b = std::max(0, std::min(255, b));

  // Assign to each node in plane
  color.at<uchar>(0) = r;
  color.at<uchar>(1) = g;
  color.at<uchar>(2) = b;
  plane.color = color;
  for (Quadtree *node : plane.nodes) {
    node->color = color;
  }
}

void HoughPlaneTransform::assignColorToPlanes()
{
  for (unsigned int i = 0; i < planes.size(); i++) {
    int colorValue = (int)(255 / (int)(i / 6 + 1));
    cv::Mat color(cv::Size(1, 3), CV_8U, cv::Scalar::all(0));
    if ((planes[i].color.at<uchar>(0) != 0) ||
        (planes[i].color.at<uchar>(1) != 0) ||
        (planes[i].color.at<uchar>(2) != 0)) {
      continue;
    }
    switch (i % 6) {
      case 0:
        assignColorToPlane(planes[i], colorValue, 0, 0);
        break;
      case 1:
        assignColorToPlane(planes[i], 0, colorValue, 0);
        break;
      case 2:
        assignColorToPlane(planes[i], 0, 0, colorValue);
        break;
      case 3:
        assignColorToPlane(planes[i], 0, colorValue, colorValue);
        break;
      case 4:
        assignColorToPlane(planes[i], colorValue, 0, colorValue);
        break;
      case 5:
        assignColorToPlane(planes[i], colorValue, colorValue, colorValue);
        break;
    }
  }
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

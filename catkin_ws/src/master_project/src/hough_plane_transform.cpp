#include "master_project/hough_plane_transform.hpp"

HoughPlaneTransform::HoughPlaneTransform(CameraData& cameraData)
{
  auto start = std::chrono::steady_clock::now();

  decimationFactor = cameraData.filterVariables.decimationScaleFactor;

  root.initializeRoot(cameraData);
  root.divideIntoQuadrants();
  timeQuadtree = msUntilNow(start);
  float rhoDelta = 0.08; // [m]
  accumulator = new Accumulator(root.maxPlaneDistance, rhoDelta, 30);
  voting(root, *accumulator, usedBins, usedKernels);
  timeVoting = msUntilNow(start) - timeQuadtree;
  peakDetection(planes, *accumulator, usedKernels, usedBins);
  std::sort(planes.begin(), planes.end());
  timePeak = msUntilNow(start) - timeQuadtree - timeVoting;
}

HoughPlaneTransform::~HoughPlaneTransform()
{
}

void HoughPlaneTransform::assignColorToPlanes()
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

void HoughPlaneTransform::printPlaneInformation(const Plane &plane)
{
  printf("Samples: %d, nodes: %ld, normal: [%.3f, %.3f, %.3f], "
         "position: [%.3f, %.3f, %.3f]\n"
         "Distance to plane: %.3f\n",
         plane.samples, plane.nodes.size(),
         plane.normal.at<double>(0), plane.normal.at<double>(1),
         plane.normal.at<double>(2), plane.position.at<double>(0),
         plane.position.at<double>(1), plane.position.at<double>(2),
         plane.rho);
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

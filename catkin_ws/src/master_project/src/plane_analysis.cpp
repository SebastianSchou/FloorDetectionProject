#include "master_project/plane_analysis.hpp"

#define MAX_POINT_PLANE_DISTANCE 0.1   // [m]
#define ACCEPTABLE_BEST_POINT_FIT 0.01 // [m]
#define MAX_POINT_PLANE_NORMAL_DIFF 0.3
#define ACCEPTABLE_BEST_NORMAL_DIFF 0.1
#define MAX_PLANE_NORMAL_DIFF 0.15

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

bool PlaneAnalysis::hasSimilarNormal(const Plane& plane1, const Plane& plane2)
{
  return squareNorm(plane1.normal - plane2.normal) < MAX_PLANE_NORMAL_DIFF;
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
      plane1.mean += node->mean / plane1.nodes.size();
      plane1.samples += node->samples;
    }
  }
  std::move(plane2.nodes.begin(), plane2.nodes.end(),
            std::inserter(plane1.nodes, plane1.nodes.end()));
}

void PlaneAnalysis::calculateNewNormal(Plane& plane)
{
  cv::Mat normal(cv::Size(1, 3), CV_64F, cv::Scalar(0));
  for (const Quadtree *node : plane.nodes) {
    normal += node->normal;
  }
  normal /= plane.nodes.size();
  plane.normal = normal;
  plane.rho = plane.position.dot(plane.normal);
  plane.phi = std::acos(plane.normal.at<double>(2));
  plane.theta = std::atan2(plane.normal.at<double>(1),
                           plane.normal.at<double>(0));
}

float PlaneAnalysis::leastSquareError(const Plane& plane, const Quadtree& node)
{
  return square(plane.rho - node.rho) + square(plane.phi - node.phi) +
         square(std::abs(plane.theta) - std::abs(node.theta));
}

void PlaneAnalysis::mergeSimilarPlanes(std::vector<Plane>& planes)
{
  for (size_t i = 0; i < planes.size() - 1; i++) {
    for (size_t j = i + 1; j < planes.size(); j++) {
      if (PlaneAnalysis::hasSimilarNormal(planes[i], planes[j]) &&
          PlaneAnalysis::hasSimilarDistance(planes[i], planes[j])) {
        PlaneAnalysis::transferNodes(planes[i], planes[j]);
        planes.erase(planes.begin() + j);
        j--;
        break;
      }
      for (size_t k = 0; k < planes[j].nodes.size(); k++) {
        Quadtree *node = planes[j].nodes[k];
        if (PlaneAnalysis::leastSquareError(planes[i], *node) <
            PlaneAnalysis::leastSquareError(planes[j], *node)) {
          planes[i].rootRepresentativeness += node->rootRepresentativeness;
          planes[j].rootRepresentativeness -= node->rootRepresentativeness;
          planes[i].samples += node->samples;
          planes[j].samples -= node->samples;
          planes[i].nodes.push_back(node);
          planes[j].nodes.erase(planes[j].nodes.begin() + k);
          k--;
          break;
        }
      }
    }
  }
}

// void PlaneAnalysis::swapSimilarNodes(std::vector<Plane>& planes) {

cv::Mat PlaneAnalysis::computePlanePoints(std::vector<Plane>& planes,
                                          const CameraData  & cameraData)
{
  // Mutex is needed, as OpenCV's forEach function runs in parallel threads,
  // and if they try to change the same value at the same time, the script
  // crashes
  std::mutex mutex;

  // Collect all points which are not part of a plane, while still within the
  // distance limit
  cv::Mat nonPlanePoints(cv::Size(cameraData.width, cameraData.height),
                         CV_8U, cv::Scalar::all(0));

  // Initialize planes
  for (Plane& plane : planes) {
    plane.image2dPoints = cv::Mat(cv::Size(cameraData.width, cameraData.height),
                                  CV_8U, cv::Scalar::all(0));
    plane.image3dPoints = cv::Mat(cv::Size(cameraData.width, cameraData.height),
                                  CV_64FC3, cv::Scalar::all(0));
    PlaneAnalysis::calculateNewNormal(plane);
  }

  // Merge similar planes
  PlaneAnalysis::mergeSimilarPlanes(planes);

  // Set image areas
  for (Plane& plane : planes) {
    for (const Quadtree *node : plane.nodes) {
      plane.setImageArea(node->minBounds, node->maxBounds);
    }
  }

  // For each point in the depth data, calculate distance to point
  cameraData.data3d.forEach<cv::Vec3d>(
    [&](cv::Vec3d& p, const int *position) {
    const int r = position[0], c = position[1];
    cv::Vec3d point = p, pointS, pointE, pointN, pointW;

    if ((r % POINT_DELTA != 0) || (c % POINT_DELTA != 0) ||
        (r - POINT_DELTA < 0) || (c - POINT_DELTA < 0) ||
        (r + POINT_DELTA - 1 >= cameraData.height) ||
        (c + POINT_DELTA - 1 >= cameraData.width)) {
      return;
    }

    // Only look at pixels within the distance limit
    if ((point[2] > MIN_DISTANCE) && (point[2] < MAX_DISTANCE)) {
      // Find plane normals for nearby pixels
      pointS = cameraData.data3d.at<cv::Vec3d>(r + POINT_DELTA - 1, c);
      pointE = cameraData.data3d.at<cv::Vec3d>(r, c + POINT_DELTA - 1);
      pointN = cameraData.data3d.at<cv::Vec3d>(r - POINT_DELTA + 1, c);
      pointW = cameraData.data3d.at<cv::Vec3d>(r, c - POINT_DELTA + 1);
      cv::Mat vecS = cv::Mat(pointS - point), vecE = cv::Mat(pointE - point);
      cv::Mat vecN = cv::Mat(pointN - point), vecW = cv::Mat(pointW - point);
      cv::Mat normalSE = normalizeVector(vecS.cross(vecE));
      cv::Mat normalNW = normalizeVector(vecN.cross(vecW));

      // Loop over planes and find the plane which fits best
      Plane *bestFit;
      double minDistance = MAX_POINT_PLANE_DISTANCE,
      normalDiff = MAX_POINT_PLANE_NORMAL_DIFF;
      bool skip = false, isObject = true;
      for (Plane& plane : planes) {
        // If the point already exists in a plane, skip this point.
        // Despite having to loop over all nodes, this is generally
        // faster.
        for (Quadtree *node : plane.nodes) {
          if (((node->minBounds.x <= c) && (node->minBounds.y <= r) &&
               (node->maxBounds.x > c) && (node->maxBounds.y > r))) {
            skip = true;
            break;
          }
        }
        if (skip) {
          break;
        }

        // Calculate distance from plane to point. If this is below max
        // distance and the previous best plane fit, change the point to
        // the current plane
        double dist = std::abs((plane.position - cv::Mat(point)).dot(
                                 plane.normal));

        if (dist < MAX_POINT_PLANE_DISTANCE) {
          isObject = false;

          // Calculate the difference between the plane normal and the two
          // found normals. If the largest difference is too large, ignore
          // this point
          if (point.dot(normalSE) < 0) {
            normalSE *= -1;
          }
          if (point.dot(normalNW) < 0) {
            normalNW *= -1;
          }
          cv::Mat diffNormalSE = cv::abs(plane.normal - normalSE);
          cv::Mat diffNormalNW = cv::abs(plane.normal - normalNW);
          double diff = std::max(squareNorm(diffNormalSE),
                                 squareNorm(diffNormalNW));
          if (diff < normalDiff) {
            minDistance = dist;
            normalDiff = diff;
            bestFit = &plane;
          }

          // If the distance is below the acceptable value, skip the rest
          // of the planes
          if ((dist < ACCEPTABLE_BEST_POINT_FIT) &&
              (diff < ACCEPTABLE_BEST_NORMAL_DIFF)) {
            break;
          }
        }
      }

      // Insert point in plane
      cv::Vec2i point2d(r, c);
      if (!skip && (minDistance < MAX_POINT_PLANE_DISTANCE)) {
        bestFit->setImagePoint(point2d);
        bestFit->insert3dPoint(point, mutex);
        bestFit->insert2dPoint(point2d, mutex);
      } else if (!skip && isObject) {
        // Insert non-plane point
        cv::Point p1(point2d[1] - std::floor(POINT_DELTA / 2),
                     point2d[0] - std::floor(POINT_DELTA / 2)),
        p2(point2d[1] + std::ceil(POINT_DELTA / 2),
           point2d[0] + std::ceil(POINT_DELTA / 2));
        cv::rectangle(nonPlanePoints, p1, p2, cv::Scalar::all(255), cv::FILLED);
      }
    }
  }
    );

  // Remove small holes in the plane points using structured elements and a
  // open / close combo
  cv::Mat stucturingElement = cv::getStructuringElement(cv::MORPH_RECT,
                                                        cv::Size(3, 3));
  for (Plane& plane : planes) {
    cv::morphologyEx(plane.image2dPoints, plane.image2dPoints,
                     cv::MORPH_OPEN, stucturingElement);
    cv::morphologyEx(plane.image2dPoints, plane.image2dPoints,
                     cv::MORPH_CLOSE, stucturingElement);
    nonPlanePoints -= plane.image2dPoints;
  }
  return nonPlanePoints;
}

void PlaneAnalysis::removeSmallPlanes(std::vector<Plane>& planes)
{
  for (size_t i = 0; i < planes.size(); i++) {
    if (planes[i].samples < MIN_PLANE_SAMPLE_SIZE) {
      planes.erase(planes.begin() + i);
      i--;
    }
  }
}

void PlaneAnalysis::printPlanesInformation(const std::vector<Plane>& planes)
{
  printf("=============PLANE INFORMATION=============\n");
  for (const Plane& plane : planes) {
    printPlaneInformation(plane);
  }
}

void PlaneAnalysis::printPlaneInformation(const Plane& plane)
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
           node->id, node->minBounds.x * IMAGE_SCALE,
           node->minBounds.y * IMAGE_SCALE,
           node->maxBounds.x * IMAGE_SCALE,
           node->maxBounds.y * IMAGE_SCALE,
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
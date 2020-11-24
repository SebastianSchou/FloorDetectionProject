#include "master_project/plane_analysis.hpp"

#define MAX_POINT_PLANE_DISTANCE 0.1   // [m]
#define ACCEPTABLE_BEST_POINT_FIT 0.01 // [m]
#define MAX_POINT_PLANE_NORMAL_DIFF 0.3
#define ACCEPTABLE_BEST_NORMAL_DIFF 0.1

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
      plane1.mean += node->mean / plane1.nodes.size();
      plane1.samples += node->samples;
    }
  }
  std::move(plane2.nodes.begin(), plane2.nodes.end(),
            std::inserter(plane1.nodes, plane1.nodes.end()));
}

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

  // Set points for all nodes
  for (Plane& plane : planes) {
    cv::Mat normal(cv::Size(1, 3), CV_64F, cv::Scalar(0));
    plane.image2dPoints = cv::Mat(cv::Size(cameraData.width, cameraData.height),
                                  CV_8U, cv::Scalar::all(0));
    plane.image3dPoints = cv::Mat(cv::Size(cameraData.width, cameraData.height),
                                  CV_64FC3, cv::Scalar::all(0));
    for (const Quadtree *node : plane.nodes) {
      plane.setImageArea(node->minBounds, node->maxBounds);
      normal += node->normal;
    }
    normal /= plane.nodes.size();
    plane.normal = normal;
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
      bool skip = false;
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
      } else if (!skip) {
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

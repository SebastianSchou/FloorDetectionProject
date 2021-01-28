#include "traversable_area_detection/plane_analysis.hpp"
#include <traversable_area_detection/Plane.h>
#include <traversable_area_detection/Node.h>
#include <traversable_area_detection/Area.h>
#include <traversable_area_detection/HeightArea.h>
#include <traversable_area_detection/Point.h>

#define ANGLE_TO_NORMAL(X) std::sqrt(2 - 2 * cos(X * CV_PI / 180.0))
#define MAX_POINT_PLANE_DISTANCE 0.1   // [m]
#define ACCEPTABLE_BEST_POINT_FIT 0.01 // [m]
#define MAX_POINT_PLANE_NORMAL_DIFF ANGLE_TO_NORMAL(10.0)
#define ACCEPTABLE_BEST_NORMAL_DIFF ANGLE_TO_NORMAL(5.0)
#define MAX_PLANE_NORMAL_DIFF ANGLE_TO_NORMAL(4.0)
#define MAX_INCLINE_DEGREES 8.0                                 // [degrees]
#define MAX_INCLINE_RADIANS MAX_INCLINE_DEGREES * CV_PI / 180.0 // [radians]
#define MIN_FLOOR_DISTANCE 0.5                                  // [m]
#define MAX_FLOOR_DISTANCE 1.0                                  // [m]
#define MAX_NORMAL_Y_VALUE 0.1
#define MAX_REPLACE_WALL_DISTANCE 0.5                           // [m]
#define MIN_ROBOT_HEIGHT 0.5                                    // [m]
#define MAX_ROBOT_HEIGHT 1.5                                    // [m]
#define OBJECT_MAX_HEIGHT_ABOVE_FLOOR MAX_ROBOT_HEIGHT          // [m]
#define TYPICAL_FLOOR_HEIGHT -0.8                               // [m]
#define MIN_PLANE_AREA 0.5                                      // [m^2]
#define MAX_PLANE_AREA_FILL 1.0                                 // [m^2]
#define MAX_OBJECT_DISTANCE_DIFFERENCE 0.2                      // [m]
#define MIN_CONTOUR_SIZE 3                                      // [points]
#define MAX_DISTANCE_TO_NODE 5.0                                // [m]

bool PlaneAnalysis::isGround(const Plane& currentFloor, const Plane& plane,
                             const float cameraHeight)
{
  // Checks if the plane is more likely to be the ground than the current
  // floor by doing the following checks:
  // - Distance to the plane is either closer to camera height if it is given,
  //   else is within height limit and further away than current floor
  // - Incline of the plane is (mostly) horizontal
  // - Y-direction of the normal is positive (exclude ceilings)
  if (cameraHeight == 0) {
    return (plane.rho > MIN_FLOOR_DISTANCE) &&
           (plane.rho < MAX_FLOOR_DISTANCE) &&
           (currentFloor.rho < plane.rho) &&
           (std::abs(std::abs(plane.phi) - CV_PI / 2.0) <
            MAX_INCLINE_RADIANS) &&
           (std::abs(std::abs(plane.theta) - CV_PI / 2.0) <
            MAX_INCLINE_RADIANS) &&
           (plane.normal[Y] > 0);
  } else {
    return (std::abs(cameraHeight - plane.rho) <
            std::abs(cameraHeight - currentFloor.rho)) &&
           (std::abs(std::abs(plane.phi) - CV_PI / 2.0) <
            MAX_INCLINE_RADIANS) &&
           (std::abs(std::abs(plane.theta) - CV_PI / 2.0) <
            MAX_INCLINE_RADIANS) &&
           (plane.normal[Y] > 0);
  }
}

bool PlaneAnalysis::isWall(const Plane& plane)
{
  // Checks if the plane is like a wall by doing the following checks:
  // - Has a small y-value in the normal
  return std::abs(plane.normal[Y]) < MAX_NORMAL_Y_VALUE;
}

bool PlaneAnalysis::isBetterWall(const Plane& currentWall, const Plane& plane)
{
  return PlaneAnalysis::hasSimilarNormal(currentWall, plane) &&
         currentWall.rho < plane.rho &&
         plane.rho - currentWall.rho <= MAX_REPLACE_WALL_DISTANCE;
}

bool PlaneAnalysis::isCeiling(const Plane& currentCeil, const Plane& plane)
{
  // Checks if the plane is more likely to be the ceiling than the current
  // ceiling by doing the following checks:
  // - Distance to the plane is further away than current ceiling
  // - Incline of the plane is (mostly) horizontal
  // - Y-direction of the normal is negative
  return (currentCeil.rho < plane.rho) &&
         (std::abs(std::abs(plane.phi) - CV_PI / 2.0) < MAX_INCLINE_RADIANS) &&
         (std::abs(std::abs(plane.theta) - CV_PI / 2.0) <
          MAX_INCLINE_RADIANS) &&
         (plane.normal[Y] < 0);
}

void PlaneAnalysis::assignPlaneType(std::vector<Plane>& planes,
                                    const float         cameraHeight)
{
  std::vector<Plane *> walls;
  Plane *floor = new Plane();
  Plane *ceiling = new Plane();
  floor->rho = 0.0;

  // Assign floor, walls and ceiling
  for (Plane& plane : planes) {
    if (PlaneAnalysis::isGround(*floor, plane, cameraHeight)) {
      floor->type = PLANE_TYPE_OTHER;
      floor = &plane;
      floor->type = PLANE_TYPE_FLOOR;
    } else if (PlaneAnalysis::isWall(plane)) {
      bool isExistingWall = false;
      for (size_t i = 0; i < walls.size(); i++) {
        if (isBetterWall(*walls[i], plane)) {
          walls[i]->type = PLANE_TYPE_OTHER;
          plane.type = PLANE_TYPE_WALL;
          walls.erase(walls.begin() + i);
          walls.push_back(&plane);
          isExistingWall = true;
          break;
        }
      }
      if (!isExistingWall) {
        plane.type = PLANE_TYPE_WALL;
        walls.push_back(&plane);
      }
    } else if (PlaneAnalysis::isCeiling(*ceiling, plane)) {
      ceiling->type = PLANE_TYPE_OTHER;
      ceiling = &plane;
      ceiling->type = PLANE_TYPE_CEILING;
    }
  }
}

bool PlaneAnalysis::hasSimilarNormal(const Plane& plane1, const Plane& plane2)
{
  return cv::norm(plane1.normal - plane2.normal,
                  cv::NORM_L2) < MAX_PLANE_NORMAL_DIFF;
}

bool PlaneAnalysis::hasSimilarNormal(const Plane& plane, const Quadtree& node)
{
  return cv::norm(plane.normal - node.normal,
                  cv::NORM_L2) < MAX_PLANE_NORMAL_DIFF;
}

double PlaneAnalysis::getAngleDifference(const Plane& plane1,
                                         const Plane& plane2)
{
  return std::max(std::abs(plane1.phi - plane2.phi),
                  std::abs(plane1.theta - plane2.theta));
}

bool PlaneAnalysis::hasSimilarDistance(const Plane& plane1, const Plane& plane2)
{
  return std::abs(plane1.rho - plane2.rho) < 2 * MAX_DISTANCE_DIFFERENCE;
}

bool PlaneAnalysis::hasSimilarDistance(const Plane& plane, const Quadtree& node)
{
  return std::abs(plane.rho - node.rho) < 2 * MAX_DISTANCE_DIFFERENCE;
}

double PlaneAnalysis::getDistanceDifference(const Plane& plane1,
                                            const Plane& plane2)
{
  return std::abs(plane1.rho - plane2.rho);
}

void PlaneAnalysis::transferNodes(Plane& plane1, Plane& plane2)
{
  // transfers nodes from one plane to another
  for (size_t i = 0; i < plane2.nodes.size(); i++) {
    Quadtree *node = plane2.nodes[i];
    plane1.rootRepresentativeness += node->rootRepresentativeness;
    plane1.samples += node->samples;
  }
  std::move(plane2.nodes.begin(), plane2.nodes.end(),
            std::inserter(plane1.nodes, plane1.nodes.end()));
}

void PlaneAnalysis::calculateNormalAndStandardDeviation(
  Plane& plane, std::vector<Quadtree*>& unfitNodes)
{
  // Calculates new normal for plane defined by the nodes it contains.
  // Also calculates standard deviation for theta, phi, and rho, as well
  // as removing nodes which does not fit properly in the plane
  cv::Vec3d normal(0.0, 0.0, 0.0);
  double    rho = 0.0;
  for (const Quadtree *node : plane.nodes) {
    normal += node->normal;
    rho += node->rho;
  }

  // Calculate normal, rho, position, phi and theta
  plane.normal = cv::normalize(normal / (float)plane.nodes.size());
  plane.rho = rho / (float)plane.nodes.size();
  plane.position = plane.normal * plane.rho;
  plane.phi = std::acos(plane.normal[Z]);
  plane.theta = std::atan2(plane.normal[Y], plane.normal[X]);

  // Calculate standard deviations and remove unfit nodes
  double thetaStd = 0.0, phiStd = 0.0, rhoStd = 0.0;
  for (size_t i = 0; i < plane.nodes.size(); i++) {
    Quadtree *node = plane.nodes[i];
    bool hasLowLsqe = PlaneAnalysis::leastSquareError(plane, *node) < 1.5;
    float phiDiff = std::abs(node->phi - plane.phi);
    float thetaDiff = PlaneAnalysis::getThetaDifference(plane, *node);
    if (!hasLowLsqe ||
        (phiDiff > 2 * MAX_ANGLE_DIFFERENCE &&
         thetaDiff > 2 * MAX_ANGLE_DIFFERENCE)) {
      unfitNodes.push_back(node);
      plane.nodes.erase(plane.nodes.begin() + i);
      i--;
      plane.samples -= node->samples;
      plane.rootRepresentativeness -= node->rootRepresentativeness;
      continue;
    }
    thetaStd += square(thetaDiff);
    phiStd += square(phiDiff);
    rhoStd += square(node->rho - plane.rho);
  }
  plane.thetaStd = std::sqrt(thetaStd / plane.nodes.size());
  plane.phiStd = std::sqrt(phiStd / plane.nodes.size());
  plane.rhoStd = std::sqrt(rhoStd / plane.nodes.size());

  // Remove nodes which does not fit within two standard deviations
  for (size_t i = 0; i < plane.nodes.size(); i++) {
    Quadtree *node = plane.nodes[i];
    if (!PlaneAnalysis::isWithinTwoStandardDeviations(plane, *node)) {
      unfitNodes.push_back(node);
      plane.nodes.erase(plane.nodes.begin() + i);
      i--;
      plane.samples -= node->samples;
      plane.rootRepresentativeness -= node->rootRepresentativeness;
      continue;
    }
  }
}

float PlaneAnalysis::leastSquareError(const Plane& plane, const Quadtree& node)
{
  return square(plane.normal[X] - node.normal[X]) +
         square(plane.normal[Y] - node.normal[Y]) +
         square(plane.normal[Z] - node.normal[Z]) +
         square(10.0 * (plane.rho - node.rho));
}

bool PlaneAnalysis::isWithinTwoStandardDeviations(const Plane& plane1,
                                                  const Plane& plane2)
{
  return (std::abs(plane1.rho - plane2.rho) < 2 * plane1.rhoStd &&
          std::abs(plane1.phi - plane2.phi) < 2 * plane1.phiStd &&
          PlaneAnalysis::getThetaDifference(plane1, plane2) < 2 * plane1.thetaStd);
}

bool PlaneAnalysis::isWithinTwoStandardDeviations(const Plane& plane, const Quadtree& node)
{
  return (std::abs(plane.rho - node.rho) < 2 * plane.rhoStd &&
          std::abs(plane.phi - node.phi) < 2 * plane.phiStd &&
          PlaneAnalysis::getThetaDifference(plane, node) < 2 * plane.thetaStd);
}

void PlaneAnalysis::mergeSimilarPlanes(std::vector<Plane>& planes, std::vector<Quadtree*>& unfitNodes)
{
  for (size_t i = 0; i < planes.size() - 1; i++) {
    for (size_t j = i + 1; j < planes.size(); j++) {
      // If the planes parameters are similar, merge the planes and calculate
      // a new normal
      if (PlaneAnalysis::isWithinTwoStandardDeviations(planes[i], planes[j]) ||
          PlaneAnalysis::isWithinTwoStandardDeviations(planes[j], planes[i]) ||
          (PlaneAnalysis::hasSimilarNormal(planes[i], planes[j]) &&
           PlaneAnalysis::hasSimilarDistance(planes[i], planes[j]))) {
        PlaneAnalysis::transferNodes(planes[i], planes[j]);
        planes.erase(planes.begin() + j);
        j--;
        PlaneAnalysis::calculateNormalAndStandardDeviation(planes[i], unfitNodes);
        break;
      }
    }
  }
}

double PlaneAnalysis::getDistanceToNode(const cv::Mat& m, const Quadtree& node,
                                        const int r, const int c,
                                        const cv::Vec3d p)
{
  // Finds the shortest distance from the node to border of a node
  cv::Vec3d nodeP;
  if (r <= node.minBounds.y) {
    if (c <= node.minBounds.x)
      nodeP = m.at<cv::Vec3d>(node.minBounds.y, node.minBounds.x);
    else if (c > node.maxBounds.x)
      nodeP = m.at<cv::Vec3d>(node.minBounds.y, node.maxBounds.x);
    else
      nodeP = m.at<cv::Vec3d>(node.minBounds.y, c);
  } else if (r > node.maxBounds.y) {
    if (c <= node.minBounds.x)
      nodeP = m.at<cv::Vec3d>(node.maxBounds.y, node.minBounds.x);
    else if (c > node.maxBounds.x)
      nodeP = m.at<cv::Vec3d>(node.maxBounds.y, node.maxBounds.x);
    else
      nodeP = m.at<cv::Vec3d>(node.maxBounds.y, c);
  } else {
    if (c <= node.minBounds.x)
      nodeP = m.at<cv::Vec3d>(r, node.minBounds.x);
    else if (c > node.maxBounds.x)
      nodeP = m.at<cv::Vec3d>(r, node.maxBounds.x);
    else // inside node, should not happen
      return 0.0;
  }

  return cv::norm(p - nodeP, cv::NORM_L2);
}

float PlaneAnalysis::getThetaDifference(const Plane& plane1, const Plane& plane2)
{
  // Theta goes from 0 to 2 PI, with 0 and 2 PI being the same. Therefore, if
  // the difference is too large, it is subtracted from 2 PI
  float thetaDiff = std::abs(plane1.theta - plane2.theta);
  thetaDiff = thetaDiff > CV_PI ? 2 * CV_PI - thetaDiff : thetaDiff;
  return thetaDiff;
}

float PlaneAnalysis::getThetaDifference(const Plane& plane, const Quadtree& node)
{
  float thetaDiff = std::abs(plane.theta - node.theta);
  thetaDiff = thetaDiff > CV_PI ? 2 * CV_PI - thetaDiff : thetaDiff;
  return thetaDiff;
}

void PlaneAnalysis::matchUnfitNodes(std::vector<Plane>& planes,
                                    std::vector<Quadtree*>& unfitNodes)
{
  // For each unfit node, compare it to each plane, and the plane with the
  // smallest least square error gets the node. Other requirements to get the
  // node is:
  //  - Is non-zero (disqualifies planes which most likely are tailored to the
  //    node)
  //  - Is within two standard deviations or has similar normal and distance
  //  - If lsqe is small enough, append it plane even if above requirement is
  //    not met
  for (Quadtree* node : unfitNodes) {
    Plane* bestFit;
    float bestValue = 1.0;
    for (Plane& plane : planes) {
      float lsqe = PlaneAnalysis::leastSquareError(plane, *node);
      if  (lsqe > 0.0001 && lsqe < bestValue &&
           (lsqe < 0.05 ||
            (PlaneAnalysis::isWithinTwoStandardDeviations(plane, *node) ||
             (PlaneAnalysis::hasSimilarNormal(plane, *node) &&
              PlaneAnalysis::hasSimilarDistance(plane, *node))))) {
        bestFit = &plane;
        bestValue = lsqe;
      }
    }
    if (bestValue < 1.0) {
      bestFit->nodes.push_back(node);
      bestFit->samples += node->samples;
      bestFit->rootRepresentativeness += node->rootRepresentativeness;
    }
  }
}

Plane PlaneAnalysis::computePlanePoints(std::vector<Plane>& planes,
                                        const CameraData  & cameraData,
                                        const double        cameraHeight)
{
  // Mutex is needed, as OpenCV's forEach function runs in parallel threads,
  // and if they try to change the same value at the same time, the script
  // crashes
  std::mutex mutex, mutexObjects;

  // Collect all points which are not part of a plane, while still within the
  // distance limit
  Plane nonPlanePoints;
  nonPlanePoints.image2dPoints =
    cv::Mat(cv::Size(cameraData.width, cameraData.height), CV_8U,
            cv::Scalar::all(0));

  // Initialize planes
  std::vector<Quadtree*> unfitNodes;
  for (Plane& plane : planes) {
    plane.image2dPoints = cv::Mat(cv::Size(cameraData.width, cameraData.height),
                                  CV_8U, cv::Scalar::all(0));
    PlaneAnalysis::calculateNormalAndStandardDeviation(plane, unfitNodes);
  }

  // Merge similar planes
  PlaneAnalysis::mergeSimilarPlanes(planes, unfitNodes);

  // Match unfit nodes
  PlaneAnalysis::matchUnfitNodes(planes, unfitNodes);

  // Remove planes which has too few samples
  PlaneAnalysis::removeSmallPlanes(planes);

  // Assign type to planes
  PlaneAnalysis::assignPlaneType(planes, cameraHeight);
  Plane *floor = new Plane();
  bool   hasFloor = false;

  cv::Mat restrictedArea(nonPlanePoints.topView.size(), CV_8U, cv::Scalar::all(0));

  // Set image areas and find floor height
  double maxObjectHeight = OBJECT_MAX_HEIGHT_ABOVE_FLOOR;
  double minObjectHeight = TYPICAL_FLOOR_HEIGHT;
  for (Plane& plane : planes) {
    for (const Quadtree *node : plane.nodes) {
      plane.setImageArea(node->minBounds, node->maxBounds);
    }
    if (plane.type == PLANE_TYPE_FLOOR) {
      floor = &plane;
      hasFloor = true;
      maxObjectHeight = -plane.position[Y] +
                        OBJECT_MAX_HEIGHT_ABOVE_FLOOR;
      minObjectHeight = -plane.position[Y];
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
    if ((point[Z] > MIN_DISTANCE) && (point[Z] < MAX_DISTANCE)) {
      // Return if this point is inside a node
      for (Plane& plane : planes) {
        if (plane.image2dPoints.at<uchar>(r, c) > 0) {
          return;
        }
      }

      // Find plane normals for nearby pixels
      pointS = cameraData.data3d.at<cv::Vec3d>(r + POINT_DELTA - 1, c);
      pointE = cameraData.data3d.at<cv::Vec3d>(r, c + POINT_DELTA - 1);
      pointN = cameraData.data3d.at<cv::Vec3d>(r - POINT_DELTA + 1, c);
      pointW = cameraData.data3d.at<cv::Vec3d>(r, c - POINT_DELTA + 1);
      cv::Vec3d vecS = pointS - point, vecE = pointE - point;
      cv::Vec3d vecN = pointN - point, vecW = pointW - point;
      cv::Vec3d normalSE = cv::normalize(vecS.cross(vecE));
      cv::Vec3d normalNW = cv::normalize(vecN.cross(vecW));

      // Loop over planes and find the plane which fits best
      Plane *bestFit;
      double minDistance = MAX_POINT_PLANE_DISTANCE,
      normalDiff = MAX_POINT_PLANE_NORMAL_DIFF;
      bool isObject = true;
      for (Plane& plane : planes) {
        // Calculate distance from plane to point. If this is below max
        // distance and the previous best plane fit, change the point to
        // the current plane
        double dist = std::abs((plane.position - point).dot(
                                 plane.normal));

        if (dist < MAX_POINT_PLANE_DISTANCE) {
          isObject = false;

          // Get the distance to the closest node
          double distanceToNode = MAX_DISTANCE_TO_NODE;
          for (Quadtree *node : plane.nodes) {
            if (distanceToNode >= MAX_DISTANCE_TO_NODE) {
              distanceToNode =
                std::min(distanceToNode,
                         PlaneAnalysis::getDistanceToNode(cameraData.data3d,
                                                          *node, r, c, point));
            }
          }

          // If distance to node is too big, skip this plane
          if (distanceToNode >= MAX_DISTANCE_TO_NODE) {
            continue;
          }

          // Add a scale. The further away a point is, the more precise does
          // the points normal vector has to be
          double scale = 1.0 - 0.5 * distanceToNode / MAX_DISTANCE_TO_NODE;

          // Calculate the difference between the plane normal and the two
          // found normals. If the largest difference is too large, ignore
          // this point
          if (point.dot(normalSE) < 0) {
            normalSE *= -1;
          }
          if (point.dot(normalNW) < 0) {
            normalNW *= -1;
          }
          cv::Vec3d diffNormalSE = plane.normal - normalSE;
          cv::Vec3d diffNormalNW = plane.normal - normalNW;
          double diff = std::max(cv::norm(diffNormalSE, cv::NORM_L2),
                                 cv::norm(diffNormalNW, cv::NORM_L2));
          if (diff < normalDiff * scale) {
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
      if ((minDistance < MAX_POINT_PLANE_DISTANCE)) {
        bestFit->setImagePoint(point2d);
        bestFit->insert3dPoint(point, mutex);
        bestFit->insert2dPoint(point2d, mutex);
      } else if (isObject) {
        // Check if point is relevant
        if ((maxObjectHeight > -point[Y]) &&
            !PlaneAnalysis::isFaultyObject(cameraData.data3d, point[Z], r, c)) {
          // Insert non-plane point
          nonPlanePoints.image2dPoints.at<uchar>(r, c) = 255;
          nonPlanePoints.insert3dPoint(point, mutexObjects);
          PlaneAnalysis::convert2dTo3d(point, nonPlanePoints);
          const cv::Vec2i coord = PlaneAnalysis::getTopViewCoordinates(point);
          bool isWithinRestrictedArea =
            restrictedArea.at<uchar>(coord[0], coord[1]) > 0;
          int margin = 2;
          cv::Point p1(coord[0] - margin, coord[1] - margin);
          cv::Point p2(coord[0] + margin, coord[1] - margin);
          cv::Point p3(coord[0] + margin, coord[1] + margin);
          cv::Point p4(coord[0] - margin, coord[1] + margin);
          std::vector<cv::Point> area {p1, p2, p3, p4 };
          if (!isWithinRestrictedArea &&
              -point[Y] < minObjectHeight + MIN_ROBOT_HEIGHT + HEIGHT_DELTA) {
            cv::rectangle(restrictedArea, p1, p3, cv::Scalar(255), cv::FILLED);
          } else if (!isWithinRestrictedArea) {
            nonPlanePoints.insertHeightLimitedArea(-point[Y] - minObjectHeight,
                                                    area, mutexObjects);
          }
        }
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
    nonPlanePoints.image2dPoints -= plane.image2dPoints;
  }

  // Draw planes in x-z coordinate
  cameraData.data3d.forEach<cv::Vec3d>(
    [&](cv::Vec3d& p, const int *position) {
    const int r = position[0], c = position[1];
    if ((p[Z] <= MIN_DISTANCE) || (p[Z] >= MAX_DISTANCE)) {
      return;
    }
    for (Plane& plane : planes) {
      if (plane.type == PLANE_TYPE_CEILING) {
        continue;
      }
      if (plane.image2dPoints.at<uchar>(r, c) > 0) {
        PlaneAnalysis::convert2dTo3d(p, plane);
      }
    }
  }
    );

  // As 3d points can be sporadic, use a 'close' morphology with
  // a large structuring element to connect points
  stucturingElement = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(11, 11));
  for (Plane& plane : planes) {
    if (plane.type == PLANE_TYPE_CEILING) {
      continue;
    }
    if (plane.type != PLANE_TYPE_FLOOR) {
      floor->topView -= plane.topView;
    }
    cv::morphologyEx(plane.topView, plane.topView,
                     cv::MORPH_CLOSE, stucturingElement);
  }

  // Remove spots with objects close to floor level
  floor->topView -= restrictedArea;

  return nonPlanePoints;
}

bool PlaneAnalysis::isFaultyObject(const cv::Mat& m, const double dist,
                                   const int r, const int c)
{
  // Check if it is an error by looking at the distance values around the
  // point. If more than a quarter are much larger or outside distance limit,
  // ignore the point
  int errorPoints = 0;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if ((i == 1) && (j == 1)) {
        continue;
      }
      cv::Vec3d pointNew = m.at<cv::Vec3d>(r + i, c + j);
      if ((std::abs(pointNew[Z] - dist) > MAX_OBJECT_DISTANCE_DIFFERENCE) ||
          (pointNew[Z] < MIN_DISTANCE) || (pointNew[Z] > MAX_DISTANCE)) {
        errorPoints++;
        if (errorPoints >= 3) {
          return true;
        }
      }
    }
  }
  return false;
}

cv::Vec2i PlaneAnalysis::getTopViewCoordinates(const cv::Vec3d& p)
{
  int col = std::round(p[0] / TOP_VIEW_DELTA) +
            (MAX_DISTANCE / 2.0) / TOP_VIEW_DELTA;
  int row = TOP_VIEW_HEIGHT - std::round(p[Z] / TOP_VIEW_DELTA);

  return cv::Vec2i(col, row);
}

cv::Vec2d PlaneAnalysis::convertTopViewToMeters(const cv::Point& p)
{
  double x = (p.x - MAX_DISTANCE / TOP_VIEW_DELTA / 2.0) * TOP_VIEW_DELTA;
  double y = (TOP_VIEW_HEIGHT - p.y) * TOP_VIEW_DELTA;
  return cv::Vec2d(x, y);
}

void PlaneAnalysis::convert2dTo3d(const cv::Vec3d& p, Plane& plane)
{
  const cv::Vec2i coord = PlaneAnalysis::getTopViewCoordinates(p);
  cv::Point point(coord[0], coord[1]);
  cv::rectangle(plane.topView, point, point, cv::Scalar(255), cv::FILLED);
}

void PlaneAnalysis::cleanUpHeightLimitedAreas(Plane& nonPlanePoints,
                                              Plane& floor)
{
  // Combines all contour for each height and removes contour from greater
  // heights which exists in lower heights
  for (auto const& x : nonPlanePoints.heightLimitedAreas) {
    // Create image and draw contours
    cv::Mat heightIm(nonPlanePoints.topView.size(), CV_8U, cv::Scalar::all(0));
    cv::drawContours(heightIm, x.second, -1, cv::Scalar(255), cv::FILLED);

    // Removes elements which isn't above the floor
    heightIm &= floor.topView;

    // Continue to next height if there are no non zero elements left in the
    // image for this height
    if (cv::countNonZero(heightIm) == 0) {
      continue;
    }

    // Remove all elements which appear in previous heights
    for (auto const& y : floor.heightLimitedAreas) {
      cv::drawContours(heightIm, y.second, -1, cv::Scalar(0), cv::FILLED);
    }

    // Continue to next height if there are no non zero elements left in the
    // image for this height
    if (cv::countNonZero(heightIm) == 0) {
      continue;
    }

    // Find the contours and set them as the floors height limited areas for
    // this height
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(heightIm, contours, cv::RETR_TREE,
                     cv::CHAIN_APPROX_TC89_KCOS);
    floor.heightLimitedAreas[x.first] = contours;
  }
}

void PlaneAnalysis::computePlaneContour(std::vector<Plane>& planes,
                                        Plane             & nonPlanePoints)
{
  // Compute the contour of the floor planes to detect which areas are,
  // traversable
  for (Plane& plane : planes) {
    if (plane.type == PLANE_TYPE_CEILING) {
      continue;
    }

    // Find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(plane.topView, contours, cv::RETR_TREE,
                     cv::CHAIN_APPROX_TC89_KCOS);

    // Assign contours
    for (size_t i = 0; i < contours.size(); i++) {
      const std::vector<cv::Point> contour = contours[i];
      if (contour.size() < MIN_CONTOUR_SIZE) {
        contours.erase(contours.begin() + i);
        i--;
        continue;
      }

      // Get area
      float area = cv::contourArea(contour) * square(TOP_VIEW_DELTA);

      // Check if the contour is hollow. This is done by check the mean value
      // of pixel area within the contour. If close to zero, it is hollow
      cv::Mat contourImage(plane.topView.size(), CV_8U, cv::Scalar::all(0));
      cv::drawContours(contourImage, contours, i, 255, cv::FILLED);
      if (plane.type != PLANE_TYPE_FLOOR &&
          plane.type != PLANE_TYPE_OTHER) {
        // Simplify area
        std::vector<cv::Point> approx;
        double epsilon = 0.005 * cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approx, epsilon, true);
        plane.area += area;
        plane.restrictedAreas.push_back(approx);
      }
      bool isHollow = cv::mean(plane.topView, contourImage)[0] < 125.0;

      // Get center
      auto m = cv::moments(contour);
      int  cx = int(m.m10 / m.m00);
      int  cy = int(m.m01 / m.m00);
      if ((cx < 0) || (cx >= plane.topView.cols) ||
          (cy < 0) || (cy >= plane.topView.rows)) {
        contours.erase(contours.begin() + i);
        i--;
        continue;
      }

      // Check if an object is within area
      bool isObjectRestrictedArea = false;
      for (const cv::Vec3d& p : nonPlanePoints.points3d) {
        cv::Point2f areaCenter(PlaneAnalysis::getTopViewCoordinates(p));
        isObjectRestrictedArea =
          cv::pointPolygonTest(contour, areaCenter, false) >= 0;
        if (isObjectRestrictedArea) {
          break;
        }
      }

      // Fill counter if hollow and below max area size. If not hollow
      // and below min area size, discard it. Else, add as traversable
      // or restricted area
      if (isHollow && (area < MAX_PLANE_AREA_FILL) && !isObjectRestrictedArea) {
        cv::floodFill(plane.topView, cv::Point(cx, cy), cv::Scalar(255));
        contours.erase(contours.begin() + i);
        i--;
      } else if ((area < MIN_PLANE_AREA) && !isObjectRestrictedArea) {
        contours.erase(contours.begin() + i);
        i--;
      } else {
        // Simplify area
        std::vector<cv::Point> approx;
        double epsilon = 0.005 * cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approx, epsilon, true);
        if (isHollow) {
          plane.area -= area;
          plane.restrictedAreas.push_back(approx);
        } else {
          plane.area += area;
          plane.traversableAreas.push_back(approx);
        }
      }
    }
  }

  // Add height limited restrictions to the floor plane
  for (Plane& plane : planes) {
    if (plane.type == PLANE_TYPE_FLOOR) {
      PlaneAnalysis::cleanUpHeightLimitedAreas(nonPlanePoints, plane);
      break;
    }
  }
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
  printf("Plane %d, type '%s':\nSamples: %d, nodes: %ld, "
         "normal: [%.3f, %.3f, %.3f], position: [%.3f, %.3f, %.3f]\n"
         "Distance to plane: %.3f, phi: %.3f, theta: %.3f\n",
         plane.id, PLANE_TYPE_STR[plane.type].c_str(),
         plane.samples, plane.nodes.size(),
         plane.normal[X], plane.normal[Y],
         plane.normal[Z], plane.position[X],
         plane.position[Y], plane.position[Z],
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
           node->normal[X], node->normal[Y], node->normal[Z],
           node->mean[X], node->mean[Y], node->mean[Z],
           node->mean.dot(node->normal));
  }
}

void PlaneAnalysis::insertPlanePublisherInformation(
  traversable_area_detection::HoughPlaneTransform& msg, const std::vector<Plane>& planes,
  const bool includeNodeInformation)
{
  for (const Plane& plane : planes) {
    traversable_area_detection::Plane planeMsg;
    planeMsg.id = plane.id;
    planeMsg.type = PLANE_TYPE_STR[plane.type];
    planeMsg.no_of_nodes = plane.nodes.size();
    if (includeNodeInformation) {
      for (Quadtree* node : plane.nodes) {
        traversable_area_detection::Node nodeMsg;
        nodeMsg.id = node->id;
        nodeMsg.samples = node->samples;
        nodeMsg.normal.x = node->normal[0];
        nodeMsg.normal.y = node->normal[1];
        nodeMsg.normal.z = node->normal[2];
        nodeMsg.position.x = node->mean[0];
        nodeMsg.position.y = node->mean[1];
        nodeMsg.position.z = node->mean[2];
        nodeMsg.phi = node->phi;
        nodeMsg.theta = node->theta;
        nodeMsg.rho = node->rho;
        planeMsg.nodes.push_back(nodeMsg);
      }
    }
    planeMsg.samples = plane.samples;
    planeMsg.normal.x = plane.normal[0];
    planeMsg.normal.y = plane.normal[1];
    planeMsg.normal.z = plane.normal[2];
    planeMsg.position.x = plane.position[0];
    planeMsg.position.y = plane.position[1];
    planeMsg.position.z = plane.position[2];
    planeMsg.phi = plane.phi;
    planeMsg.theta = plane.theta;
    planeMsg.distance = plane.rho;
    planeMsg.area = plane.area;
    for (const std::vector<cv::Point>& area : plane.traversableAreas) {
      traversable_area_detection::Area areaMsg;
      for (const cv::Point& point : area) {
        cv::Vec2d coord = PlaneAnalysis::convertTopViewToMeters(point);
        traversable_area_detection::Point pointMsg;
        pointMsg.x = coord[X];
        pointMsg.y = coord[Y];
        areaMsg.area.push_back(pointMsg);
      }
      planeMsg.traversable_areas.push_back(areaMsg);
    }
    for (const std::vector<cv::Point>& area : plane.restrictedAreas) {
      traversable_area_detection::Area areaMsg;
      for (const cv::Point& point : area) {
        cv::Vec2d coord = PlaneAnalysis::convertTopViewToMeters(point);
        traversable_area_detection::Point pointMsg;
        pointMsg.x = coord[X];
        pointMsg.y = coord[Y];
        areaMsg.area.push_back(pointMsg);
      }
      planeMsg.traversable_areas.push_back(areaMsg);
    }
    for (const auto& heightArea : plane.heightLimitedAreas) {
      traversable_area_detection::HeightArea heightAreaMsg;
      heightAreaMsg.height = heightArea.first;
      for (const std::vector<cv::Point>& area: heightArea.second) {
        traversable_area_detection::Area areaMsg;
        for (const cv::Point& point : area) {
          cv::Vec2d coord = PlaneAnalysis::convertTopViewToMeters(point);
          traversable_area_detection::Point pointMsg;
          pointMsg.x = coord[X];
          pointMsg.y = coord[Y];
          areaMsg.area.push_back(pointMsg);
        }
        heightAreaMsg.area.push_back(areaMsg);
      }
      planeMsg.height_limited_areas.push_back(heightAreaMsg);
    }
    msg.planes.push_back(planeMsg);
  }
}

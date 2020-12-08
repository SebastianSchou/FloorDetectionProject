#include "master_project/plane_analysis.hpp"
#include <master_project/Plane.h>
#include <master_project/Area.h>
#include <master_project/HeightArea.h>
#include <master_project/Point.h>

#define MAX_POINT_PLANE_DISTANCE 0.1   // [m]
#define ACCEPTABLE_BEST_POINT_FIT 0.01 // [m]
#define MAX_POINT_PLANE_NORMAL_DIFF 0.3
#define ACCEPTABLE_BEST_NORMAL_DIFF 0.1
#define MAX_PLANE_NORMAL_DIFF 0.15
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
#define X 0
#define Y 1
#define Z 2
#define MIN_CONTOUR_SIZE 3       // [points]
#define MAX_DISTANCE_TO_NODE 5.0 // [m]

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
      plane1.mean += node->mean.mul(1 / plane1.nodes.size());
      plane1.samples += node->samples;
    }
  }
  std::move(plane2.nodes.begin(), plane2.nodes.end(),
            std::inserter(plane1.nodes, plane1.nodes.end()));
}

void PlaneAnalysis::calculateNewNormal(Plane& plane)
{
  cv::Vec3d normal(0.0, 0.0, 0.0);
  double    rho = 0.0;
  for (const Quadtree *node : plane.nodes) {
    normal += node->normal;
    rho += node->rho;
  }
  plane.normal = cv::normalize(normal / (float)plane.nodes.size());
  plane.rho = rho / (float)plane.nodes.size();
  plane.position = plane.normal * plane.rho;
  plane.phi = std::acos(plane.normal[Z]);
  plane.theta = std::atan2(plane.normal[Y], plane.normal[X]);
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

double PlaneAnalysis::getDistanceToNode(const cv::Mat& m, const Quadtree& node,
                                        const int r, const int c,
                                        const cv::Vec3d p)
{
  cv::Vec3d nodeP;
  if (r <= node.minBounds.y)
    if (c <= node.minBounds.x)
      nodeP = m.at<cv::Vec3d>(node.minBounds.y, node.minBounds.x);
    else if (c > node.maxBounds.x)
      nodeP = m.at<cv::Vec3d>(node.minBounds.y, node.maxBounds.x);
    else
      nodeP = m.at<cv::Vec3d>(node.minBounds.y, c);
  else if (r > node.maxBounds.y)
    if (c <= node.minBounds.x)
      nodeP = m.at<cv::Vec3d>(node.maxBounds.y, node.minBounds.x);
    else if (c > node.maxBounds.x)
      nodeP = m.at<cv::Vec3d>(node.maxBounds.y, node.maxBounds.x);
    else
      nodeP = m.at<cv::Vec3d>(node.maxBounds.y, c);
  else
  if (c <= node.minBounds.x)
    nodeP = m.at<cv::Vec3d>(r, node.minBounds.x);
  else if (c > node.maxBounds.x)
    nodeP = m.at<cv::Vec3d>(r, node.maxBounds.x);
  else // inside node, should not happen
    return 0.0;

  return cv::norm(p - nodeP, cv::NORM_L2);
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
  for (Plane& plane : planes) {
    plane.image2dPoints = cv::Mat(cv::Size(cameraData.width, cameraData.height),
                                  CV_8U, cv::Scalar::all(0));
    PlaneAnalysis::calculateNewNormal(plane);
  }

  // Merge similar planes
  PlaneAnalysis::mergeSimilarPlanes(planes);

  // Assign type to planes
  PlaneAnalysis::assignPlaneType(planes, cameraHeight);
  Plane *floor = new Plane();
  bool   hasFloor = false;

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

          // If the point already exists in a plane, skip this point.
          // Despite having to loop over all nodes, this is generally
          // faster than performing the rest of the calculations. Get
          // the distance to the closest node simulationously.
          double distanceToNode = MAX_DISTANCE_TO_NODE;
          for (Quadtree *node : plane.nodes) {
            if (((node->minBounds.x <= c) && (node->minBounds.y <= r) &&
                 (node->maxBounds.x > c) && (node->maxBounds.y > r))) {
              return;
            }
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
          double scale = 1.0 - distanceToNode / MAX_DISTANCE_TO_NODE;

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
          PlaneAnalysis::convert2dTo3d(point, nonPlanePoints, false);
          const cv::Vec2i coord = PlaneAnalysis::getTopViewCoordinates(point);
          int margin = 4;
          cv::Point p1(coord[0] - margin, coord[1] - margin);
          cv::Point p2(coord[0] + margin, coord[1] - margin);
          cv::Point p3(coord[0] + margin, coord[1] + margin);
          cv::Point p4(coord[0] - margin, coord[1] + margin);
          std::vector<cv::Point> area {p1, p2, p3, p4 };
          if (-point[Y] < minObjectHeight + MIN_ROBOT_HEIGHT + HEIGHT_DELTA) {
            nonPlanePoints.insertRestrictedArea(area, mutexObjects);
          } else {
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
        PlaneAnalysis::convert2dTo3d(p, plane, true);
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
  cv::Mat objectAreas(nonPlanePoints.topView.size(), CV_8U, cv::Scalar::all(0));
  if (nonPlanePoints.restrictedAreas.size() > 0) {
    for (size_t i = 0; i < nonPlanePoints.restrictedAreas.size(); i++) {
      cv::drawContours(objectAreas, nonPlanePoints.restrictedAreas,
                       i, 255, cv::FILLED);
    }
  }
  floor->topView -= objectAreas;

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

void PlaneAnalysis::convert2dTo3d(const cv::Vec3d& p, Plane& plane,
                                  const bool addMargin)
{
  const cv::Vec2i coord = PlaneAnalysis::getTopViewCoordinates(p);
  int margin = (addMargin) ? 2 : 0;

  cv::Point p1(coord[0] - margin, coord[1] - margin);
  cv::Point p2(coord[0] + margin, coord[1] + margin);
  cv::rectangle(plane.topView, p1, p2, cv::Scalar(255), cv::FILLED);
}

bool PlaneAnalysis::isObjectOnFloor(const Plane& floor, const cv::Point& object)
{
  for (const auto& area : floor.traversableAreas) {
    if (cv::pointPolygonTest(area, object, false) < 0) {
      return true;
    }
  }
  return false;
}

void PlaneAnalysis::cleanUpHeightLimitedAreas(Plane& nonPlanePoints,
                                              Plane& floor)
{
  // Add timer which returns if this function is taking too long time
  auto start = std::chrono::steady_clock::now();
  int  timeout = 100; // [ms]

  // Use a pointer to the height limited areas for better readability
  std::map<double, std::vector<std::vector<cv::Point> > > *areas =
    &nonPlanePoints.heightLimitedAreas;

  // Loop over all possible object heights
  double minHeight = areas->begin()->first;
  double maxHeight = OBJECT_MAX_HEIGHT_ABOVE_FLOOR;
  for (double hL = minHeight; hL < maxHeight; hL += HEIGHT_DELTA) {
    if (msUntilNow(start) > timeout) {
      ROS_ERROR("Function 'cleanUpHeightLimitedAreas' reached timeout %d ms. "
                "Height limited areas are ignored.", timeout);
      return;
    }

    // Due to step values for double, round to nearest HEIGHT_DELTA value
    // to be able to access map
    hL = roundToNearestValue(hL, HEIGHT_DELTA);

    // Ignore non-existant map values and remove keys which has no values
    if (areas->find(hL) == areas->end()) {
      continue;
    } else if (areas->at(hL).size() == 0) {
      areas->erase(hL);
      continue;
    }

    // Initialize image where correct areas will be drawn, such that they can
    // be merged in the end
    cv::Mat heightImage(nonPlanePoints.topView.size(), CV_8U,
                        cv::Scalar::all(0));

    // Loop over areas for this height (hL)
    for (size_t i = 0; i < areas->at(hL).size(); i++) {
      // Get area center. If area center is not above floor, remove it.
      cv::Point center(
        (areas->at(hL)[i][1].x + areas->at(hL)[i][0].x) / 2.0,
        (areas->at(hL)[i][2].y + areas->at(hL)[i][1].y) / 2.0);
      if (PlaneAnalysis::isObjectOnFloor(floor, center)) {
        areas->at(hL).erase(areas->at(hL).begin() + i);
        i--;
        if (areas->at(hL).size() == 0) {
          areas->erase(hL);
          break;
        }
        continue;
      }

      // Draw area in image
      cv::drawContours(heightImage, areas->at(hL), i, 255, cv::FILLED);

      // Loop over all higher objects (hH)
      for (double hH = hL + HEIGHT_DELTA; hH < maxHeight; hH += HEIGHT_DELTA) {
        // Ignore non-existant map values and remove keys which has no values
        hH = roundToNearestValue(hH, HEIGHT_DELTA);
        if (areas->find(hH) == areas->end()) {
          continue;
        } else if (areas->at(hH).size() == 0) {
          areas->erase(hH);
          continue;
        }

        // Loop over areas for this height (hH)
        for (size_t j = 0; j < areas->at(hH).size(); j++) {
          if (msUntilNow(start) > timeout) {
            ROS_ERROR("Function 'cleanUpHeightLimitedAreas' reached timeout %d ms. "
                      "Height limited areas are ignored.",
                      timeout);
            return;
          }

          // Get area center. If not above floor level, or if area of lower
          // height (hL)'s center is within the area, delete the area
          cv::Point centerH(
            (areas->at(hH)[j][1].x + areas->at(hH)[j][0].x) / 2.0,
            (areas->at(hH)[j][2].y + areas->at(hH)[j][1].y) / 2.0);
          if (((center.x >= areas->at(hH)[j][0].x) &&
               (center.x <= areas->at(hH)[j][1].x) &&
               (center.y >= areas->at(hH)[j][1].y) &&
               (center.y <= areas->at(hH)[j][2].y)) ||
              PlaneAnalysis::isObjectOnFloor(floor, centerH)) {
            areas->at(hH).erase(areas->at(hH).begin() + j);
            j--;
            if (areas->at(hH).size() == 0) {
              areas->erase(hH);
              break;
            }
            continue;
          }
        }
      }

      // Find the areas for the now merged areas in this height (hL)
      std::vector<std::vector<cv::Point> > heightContours;
      cv::findContours(heightImage, heightContours, cv::RETR_TREE,
                       cv::CHAIN_APPROX_TC89_KCOS);
      areas->at(hL) = heightContours;
    }
  }

  // Set height limited areas for the floor
  floor.heightLimitedAreas = *areas;
}

void PlaneAnalysis::computePlaneContour(std::vector<Plane>& planes,
                                        Plane             & nonPlanePoints)
{
  for (Plane& plane : planes) {
    if ((plane.type != PLANE_TYPE_FLOOR) &&
        (plane.type != PLANE_TYPE_OTHER)) {
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

      // Fill counter if hollow and below max area size. If not hollow
      // and below min area size, discard it. Else, add as traversable
      // or restricted area
      bool isObjectRestrictedArea = false;
      for (const cv::Vec3d& p : nonPlanePoints.points3d) {
        cv::Point2f areaCenter(PlaneAnalysis::getTopViewCoordinates(p));
        isObjectRestrictedArea =
          cv::pointPolygonTest(contour, areaCenter, false) > 0;
        break;
      }

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
  master_project::HoughPlaneTransform& msg, const std::vector<Plane>& planes)
{
  for (const Plane& plane : planes) {
    master_project::Plane planeMsg;
    planeMsg.id = plane.id;
    planeMsg.type = PLANE_TYPE_STR[plane.type];
    planeMsg.no_of_nodes = plane.nodes.size();
    planeMsg.samples = plane.samples;
    planeMsg.normal.x = plane.normal[0];
    planeMsg.normal.y = plane.normal[1];
    planeMsg.normal.z = plane.normal[2];
    planeMsg.position.x = plane.position[0];
    planeMsg.position.y = plane.position[1];
    planeMsg.position.z = plane.position[2];
    planeMsg.distance = plane.rho;
    planeMsg.area = plane.area;
    for (const std::vector<cv::Point>& area : plane.traversableAreas) {
      master_project::Area areaMsg;
      for (const cv::Point& point : area) {
        master_project::Point pointMsg;
        pointMsg.x = point.x;
        pointMsg.y = point.y;
        areaMsg.area.push_back(pointMsg);
      }
      planeMsg.traversable_areas.push_back(areaMsg);
    }
    for (const auto& heightArea : plane.heightLimitedAreas) {
      master_project::HeightArea heightAreaMsg;
      heightAreaMsg.height = heightArea.first;
      for (const std::vector<cv::Point>& area: heightArea.second) {
        master_project::Area areaMsg;
        for (const cv::Point& point : area) {
          master_project::Point pointMsg;
          pointMsg.x = point.x;
          pointMsg.y = point.y;
          areaMsg.area.push_back(pointMsg);
        }
        heightAreaMsg.area.push_back(areaMsg);
      }
      planeMsg.height_limited_areas.push_back(heightAreaMsg);
    }
    msg.planes.push_back(planeMsg);
  }
}

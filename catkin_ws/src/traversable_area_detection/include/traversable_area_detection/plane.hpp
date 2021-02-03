#ifndef PLANE_HPP
#define PLANE_HPP

#include <traversable_area_detection/quadtree.hpp>
#include <traversable_area_detection/helper_function.hpp>
#include <mutex>

#define MAX_ANGLE_DIFFERENCE 4.0 * CV_PI / 180.0 // [radians]
#define MAX_DISTANCE_DIFFERENCE 0.08             // [m]
#define MIN_INDEPENDENT_NODE_SIZE 800 / square(IMAGE_SCALE)
#define POINT_DELTA 2
#define MIN_PLANE_SAMPLE_SIZE 8000 / square(IMAGE_SCALE)
#define MIN_NODE_NEIGHBOR_SAMPLE_SUM 1800 / square(IMAGE_SCALE)
#define TOP_VIEW_DELTA 0.03                                          // [m]
#define SIDE_VIEW_DELTA 0.01                                         // [m]
#define TOP_VIEW_HEIGHT std::ceil(MAX_DISTANCE / TOP_VIEW_DELTA)     // [pixels]
#define TOP_VIEW_WIDTH std::ceil(MAX_DISTANCE / TOP_VIEW_DELTA)      // [pixels]
#define SIDE_VIEW_WIDTH std::ceil(MAX_DISTANCE / SIDE_VIEW_DELTA)    // [pixels]
#define SIDE_VIEW_HEIGHT 1.0                                         // [m]
#define SIDE_VIEW_SIZE std::ceil(SIDE_VIEW_HEIGHT / SIDE_VIEW_DELTA) // [pixels]
#define HEIGHT_DELTA 0.1                                             // [m]

enum PlaneType {
  PLANE_TYPE_OTHER,
  PLANE_TYPE_FLOOR,
  PLANE_TYPE_WALL,
  PLANE_TYPE_CEILING,
  PLANE_TYPE_RAMP,
  PLANE_TYPE_STAIR
};

const std::string PLANE_TYPE_STR[] =
{ "Other", "Floor", "Wall", "Ceiling", "Ramp", "Stair" };

class Plane {
public:

  Plane(void)
  {
    rootRepresentativeness = 0.0;
    samples = 0;
    type = PLANE_TYPE_OTHER;
    topView = cv::Mat(cv::Size(TOP_VIEW_WIDTH, TOP_VIEW_HEIGHT), CV_8U,
                      cv::Scalar(0));
    area = 0.0;
  }

  ~Plane(void)
  {
  }

  bool computePlaneParameters(std::vector<Plane>& planes)
  {
    // Get normal vector and position
    normal[0] = std::sin(phi) * std::cos(theta);
    normal[1] = std::sin(phi) * std::sin(theta);
    normal[2] = std::cos(phi);
    position = normal * rho;

    // Get nodes representing the plane
    for (size_t i = 0; i < nodes.size(); i++) {
      Quadtree *node = nodes[i];
      if (planes.size() > 0) {
        for (Plane& plane : planes) {
          std::vector<Quadtree *>::iterator j =
            find(plane.nodes.begin(), plane.nodes.end(), node);
          if (j != plane.nodes.end()) {
            if (plane.leastSquareError(*node) < leastSquareError(*node)) {
              nodes.erase(nodes.begin() + i);
              i--;
              if (nodes.size() == 0) {
                return false;
              }
              goto skipToNextNode;
            } else {
              plane.rootRepresentativeness -= node->rootRepresentativeness;
              plane.samples -= node->samples;
              plane.nodes.erase(j);
            }
          }
        }
      }
      rootRepresentativeness += node->rootRepresentativeness;
      samples += node->samples;
      skipToNextNode: {}
    }
    if (samples > MIN_PLANE_SAMPLE_SIZE) {
      return true;
    }
    return false;
  }

  bool hasNeighbor(Quadtree *node, int& sampleSum)
  {
    sampleSum += node->samples;
    node->visited = true;
    if (sampleSum > MIN_NODE_NEIGHBOR_SAMPLE_SUM) {
      node->visited = false;
      return true;
    }
    int x1 = node->minBounds.x, x2 = node->maxBounds.x;
    int y1 = node->minBounds.y, y2 = node->maxBounds.y;
    std::vector<Quadtree*> visitedNodes;

    for (Quadtree *neighbor : nodes) {
      if (neighbor->visited) {
        continue;
      }
      int xn1 = neighbor->minBounds.x, xn2 = neighbor->maxBounds.x;
      int yn1 = neighbor->minBounds.y, yn2 = neighbor->maxBounds.y;
      if (((x1 == xn1) || (x2 == xn1) || (x1 == xn2) || (x2 == xn2)) &&
          ((y1 == yn1) || (y2 == yn1) || (y1 == yn2) || (y2 == yn2))) {
        if (hasNeighbor(neighbor, sampleSum)) {
          for (Quadtree* visitedNode : visitedNodes) {
            visitedNode->visited = false;
          }
          node->visited = false;
          return true;
        } else {
          visitedNodes.push_back(neighbor);
        }
      }
    }
    for (Quadtree* visitedNode : visitedNodes) {
      visitedNode->visited = false;
    }
    return false;
  }

  const double getAngleToNormal(const cv::Vec3d vector) const
  {
    return std::acos(normal.dot(vector));
  }

  const bool hasSimilarNormal(const Plane& plane) const
  {
    return getAngleToNormal(plane.normal) < 2 * MAX_ANGLE_DIFFERENCE;
  }

  const bool hasSimilarNormal(const Quadtree& node) const
  {
    return getAngleToNormal(node.normal) < 2 * MAX_ANGLE_DIFFERENCE;
  }

  const bool hasSimilarDistance(const Plane& plane) const
  {
    return std::abs(rho - plane.rho) < 2 * MAX_DISTANCE_DIFFERENCE;
  }

  const bool hasSimilarDistance(const Quadtree& node) const
  {
    return std::abs(rho - node.rho) < 2 * MAX_DISTANCE_DIFFERENCE;
  }

  void transferNodes(Plane& plane)
  {
    // transfers nodes from one plane to another
    for (size_t i = 0; i < plane.nodes.size(); i++) {
      Quadtree *node = plane.nodes[i];
      rootRepresentativeness += node->rootRepresentativeness;
      samples += node->samples;
    }
    std::move(plane.nodes.begin(), plane.nodes.end(),
              std::inserter(nodes, nodes.end()));
  }

  const float leastSquareError(const Quadtree& node) const
  {
    return square(normal[X] - node.normal[X]) +
           square(normal[Y] - node.normal[Y]) +
           square(normal[Z] - node.normal[Z]) +
           square(10.0 * (rho - node.rho));
  }

  const bool isWithinTwoStandardDeviations(const Plane& plane) const
  {
    return std::abs(rho - plane.rho) < 2 * rhoStd &&
           std::abs(phi - plane.phi) < 2 * phiStd &&
           getThetaDifference(plane) < 2 * thetaStd;
  }

  const bool isWithinTwoStandardDeviations(const Quadtree& node) const
  {
    return std::abs(rho - node.rho) < 2 * rhoStd &&
           std::abs(phi - node.phi) < 2 * phiStd &&
           getThetaDifference(node) < 2 * thetaStd;
  }

  const float getThetaDifference(const Plane& plane) const
  {
    // Theta goes from 0 to 2 PI, with 0 and 2 PI being the same. Therefore, if
    // the difference is too large, it is subtracted from 2 PI
    float thetaDiff = std::abs(theta - plane.theta);

    thetaDiff = thetaDiff > CV_PI ? 2 * CV_PI - thetaDiff : thetaDiff;
    return thetaDiff;
  }

  const float getThetaDifference(const Quadtree& node) const
  {
    float thetaDiff = std::abs(theta - node.theta);

    thetaDiff = thetaDiff > CV_PI ? 2 * CV_PI - thetaDiff : thetaDiff;
    return thetaDiff;
  }

  inline bool operator<(const Plane& p) const
  {
    return rootRepresentativeness > p.rootRepresentativeness;
  }

  void insert3dPoint(const cv::Vec3d& point, std::mutex& mutex)
  {
    std::lock_guard<std::mutex> lock(mutex);
    points3d.push_back(point);
  }

  void insert2dPoint(const cv::Vec2i& point, std::mutex& mutex)
  {
    std::lock_guard<std::mutex> lock(mutex);
    points2d.push_back(point);
  }

  void insertRestrictedArea(const std::vector<cv::Point>& area,
                            std::mutex                  & mutex)
  {
    std::lock_guard<std::mutex> lock(mutex);
    restrictedAreas.push_back(area);
  }

  void insertHeightLimitedArea(const double                  height,
                               const std::vector<cv::Point>& area,
                               std::mutex                  & mutex)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const double h = roundToNearestValue(height, HEIGHT_DELTA);
    heightLimitedAreas[h].push_back(area);
  }

  void setImagePoint(const cv::Vec2i& point)
  {
    cv::Point p1(point[1] - std::floor(POINT_DELTA / 2),
                 point[0] - std::floor(POINT_DELTA / 2)),
    p2(point[1] + std::ceil(POINT_DELTA / 2),
       point[0] + std::ceil(POINT_DELTA / 2));
    cv::rectangle(image2dPoints, p1, p2, cv::Scalar::all(255), cv::FILLED);
  }

  double theta, phi, rho, votes, rootRepresentativeness, thetaIndex, area,
         thetaStd, phiStd, rhoStd;
  int phiIndex, rhoIndex, samples, id, type;
  cv::Mat image2dPoints, topView;
  cv::Vec3d position, normal;
  cv::Scalar color;
  std::vector<std::vector<cv::Point> > traversableAreas, restrictedAreas;
  std::map<double, std::vector<std::vector<cv::Point> > > heightLimitedAreas;
  std::vector<Quadtree *> nodes;
  std::vector<cv::Vec3d> points3d;
  std::vector<cv::Vec2i> points2d;
};

#endif // PLANE_HPP

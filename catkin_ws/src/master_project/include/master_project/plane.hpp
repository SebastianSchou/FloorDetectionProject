#ifndef PLANE_HPP
#define PLANE_HPP

#include <master_project/quadtree.hpp>
#include <master_project/helper_function.hpp>
#include <mutex>

#define MAX_ANGLE_DIFFERENCE 4.0 * CV_PI / 180.0 // [radians]
#define MAX_DISTANCE_DIFFERENCE 0.08             // [m]
#define MIN_INDEPENDENT_NODE_SIZE 50
#define POINT_DELTA 2
#define MIN_PLANE_SAMPLE_SIZE 500
#define MIN_NODE_NEIGHBOR_SAMPLE_SUM 100
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

  void computePlaneParameters(std::vector<Plane>& planes)
  {
    cv::Vec3d newNormal(0.0, 0.0, 0.0);
    double    newRho = 0.0;
    // Get nodes representing the plane
    for (size_t i = 0; i < nodes.size(); i++) {
      Quadtree *node = nodes[i];
      if (node->samples < MIN_INDEPENDENT_NODE_SIZE) {
        if (!hasNeighbor(node)) {
          if (nodes.size() <= 1) {
            return;
          }
          nodes.erase(nodes.begin() + i);
          i--;
          continue;
        }
      }
      bool skip = false;
      if (planes.size() > 0) {
        for (Plane& plane : planes) {
          std::vector<Quadtree *>::iterator j =
            find(plane.nodes.begin(), plane.nodes.end(), node);
          if (j != plane.nodes.end()) {
            if (leastSquareError(plane, *node) <
                leastSquareError(*this, *node)) {
              skip = true;
              if (nodes.size() <= 1) {
                return;
              }
              nodes.erase(nodes.begin() + i);
              i--;
              break;
            } else {
              plane.rootRepresentativeness -= node->rootRepresentativeness;
              plane.samples -= node->samples;
              plane.nodes.erase(j);
            }
          }
        }
      }
      if (skip) {
        continue;
      }
      newNormal += node->normal;
      newRho += node->rho;
      rootRepresentativeness += node->rootRepresentativeness;
      samples += node->samples;
    }
    if (samples > 0) {
      normal = cv::normalize(newNormal / (float)nodes.size());
      rho = newRho / (float)nodes.size();
      position = normal * rho;
      phi = std::acos(normal[2]);
      theta = std::atan2(normal[1], normal[0]);
    }
  }

  float leastSquareError(const Plane& plane, const Quadtree& node)
  {
    return square(plane.normal[0] - node.normal[0]) +
           square(plane.normal[1] - node.normal[1]) +
           square(plane.normal[2] - node.normal[2]) +
           square(10 * (plane.rho - node.rho));
  }

  bool hasNeighbor(Quadtree *loneNode)
  {
    int x1 = loneNode->minBounds.x, x2 = loneNode->maxBounds.x;
    int y1 = loneNode->minBounds.y, y2 = loneNode->maxBounds.y;

    std::vector<Quadtree *> nextNodes; nextNodes.push_back(loneNode);
    int samples = loneNode->samples;
    for (const Quadtree *nextNode : nextNodes) {
      for (Quadtree *node : nodes) {
        int xn1 = node->minBounds.x, xn2 = node->maxBounds.x;
        int yn1 = node->minBounds.y, yn2 = node->maxBounds.y;
        if ((x1 == xn1) && (y1 == yn1)) {
          continue;
        }
        if (((x1 == xn1) || (x2 == xn1) || (x1 == xn2) || (x2 == xn2)) &&
            ((y1 == yn1) || (y2 == yn1) || (y1 == yn2) || (y2 == yn2))) {
          samples += nextNode->samples;
          if (samples > MIN_NODE_NEIGHBOR_SAMPLE_SUM) {
            return true;
          }
          nextNodes.push_back(node);
        }
      }
    }
    return false;
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

  void insertHeightLimitedArea(const double     height,
                               const cv::Point& point,
                               std::mutex     & mutex)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const double h = roundToNearestValue(height, HEIGHT_DELTA);
    heightLimitedAreas[h].push_back(point);
  }

  void setImagePoint(const cv::Vec2i& point)
  {
    cv::Point p1(point[1] - std::floor(POINT_DELTA / 2),
                 point[0] - std::floor(POINT_DELTA / 2)),
    p2(point[1] + std::ceil(POINT_DELTA / 2),
       point[0] + std::ceil(POINT_DELTA / 2));
    cv::rectangle(image2dPoints, p1, p2, cv::Scalar::all(255), cv::FILLED);
  }

  void setImageArea(const cv::Point& p1, const cv::Point& p2)
  {
    cv::rectangle(image2dPoints, p1, p2, cv::Scalar::all(255), cv::FILLED);
  }

  double theta, phi, rho, votes, rootRepresentativeness, thetaIndex, area,
         thetaStd, phiStd, rhoStd;
  int phiIndex, rhoIndex, samples, id, type;
  cv::Mat image2dPoints, topView;
  cv::Vec3d position, normal;
  cv::Scalar color;
  std::vector<std::vector<cv::Point> > traversableAreas, restrictedAreas;
  std::map<double, std::vector<cv::Point> > heightLimitedAreas;
  std::vector<Quadtree *> nodes;
  std::vector<cv::Vec3d> points3d;
  std::vector<cv::Vec2i> points2d;
};

#endif // PLANE_HPP

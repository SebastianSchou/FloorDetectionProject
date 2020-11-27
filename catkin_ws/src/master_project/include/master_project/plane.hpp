#ifndef PLANE_HPP
#define PLANE_HPP

#include <master_project/quadtree.hpp>
#include <master_project/helper_function.hpp>
#include <mutex>

#define MIN_INDEPENDENT_NODE_SIZE 50
#define POINT_DELTA 2
#define MIN_PLANE_SAMPLE_SIZE 500
#define MIN_NODE_NEIGHBOR_SAMPLE_SUM 100

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
    samples = 0.0;
    isShowing = true;
    rotate = 0.0;
    normal = cv::Mat::zeros(cv::Size(1, 3), CV_64F);
    mean = cv::Mat::zeros(cv::Size(1, 3), CV_64F);
    color = cv::Mat::zeros(cv::Size(1, 3), CV_8U);
    samples = 0;
    type = PLANE_TYPE_OTHER;
  }

  ~Plane(void)
  {
  }

  void computePlaneParameters(std::vector<Plane>& planes)
  {
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
              plane.mean -= node->mean / plane.nodes.size();
              plane.samples -= node->samples;
              plane.nodes.erase(j);
            }
          }
        }
      }
      if (skip) {
        continue;
      }
      rootRepresentativeness += node->rootRepresentativeness;
      mean += node->mean;
      samples += node->samples;
    }
    mean /= (double)nodes.size();
  }

  float leastSquareError(const Plane& plane, const Quadtree& node)
  {
    return square(plane.rho - node.rho) + square(plane.phi - node.phi) +
           square(std::abs(plane.theta) - std::abs(node.theta));
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

  cv::Mat getClosestPointToOrigin()
  {
    if ((theta >= 0.5 * PI) || (theta < -0.5 * PI)) {
      return normalizeVector(normal) * -rho;
    }
    return normalizeVector(normal) * rho;
  }

  double getDistanceToPlane(cv::Mat& point)
  {
    return std::abs((point - position).dot(normalizeVector(normal)));
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

  double theta, phi, rho, votes, rootRepresentativeness, rotate, thetaIndex,
         thetaAbs, phiAbs;
  int phiIndex, rhoIndex, samples, id, type;
  bool isShowing;
  cv::Mat position, mean, normal, color, image2dPoints, image3dPoints;
  std::vector<std::vector<double> > validCoordinates;
  std::vector<std::vector<int> > validPixels;
  std::vector<Quadtree *> nodes;
  std::vector<cv::Vec3d> points3d;
  std::vector<cv::Vec2i> points2d;
};

#endif // PLANE_HPP

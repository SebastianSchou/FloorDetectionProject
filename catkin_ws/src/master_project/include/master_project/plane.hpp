#ifndef PLANE_HPP
#define PLANE_HPP

#include <master_project/quadtree.hpp>
#include <master_project/helper_function.hpp>
#include <algorithm>

#define MIN_INDEPENDENT_NODE_SIZE 50

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
  }

  ~Plane(void)
  {
  }

  void computePlaneParameters(std::vector<Plane>& planes)
  {
    // Get cross product of normal vector???)
    double v[3] = { 0.0, 0.0, 1.0 };

    cv::Mat cu(cv::Size(1, 3), CV_64F, v);
    if (isMatEqual(cu, normal)) {
      v[0] = 1.0; v[2] = 0.0;
      cu = cv::Mat(cv::Size(1, 3), CV_64F, v);
    }
    cross = normalizeVector(cu.mul(normal));
    cross2 = normal.mul(cross);

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
              plane.mean -= node->mean;
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
          if (samples > 200) {
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

  double theta, phi, rho, votes, rootRepresentativeness, rotate, thetaIndex,
         thetaAbs, phiAbs;
  int phiIndex, rhoIndex, samples, id;
  bool isShowing;
  cv::Mat cross, cross2, position, mean, normal, color;
  std::vector<std::vector<double> > validCoordinates;
  std::vector<std::vector<int> > validPixels;
  std::vector<Quadtree *> nodes;
};

#endif // PLANE_HPP

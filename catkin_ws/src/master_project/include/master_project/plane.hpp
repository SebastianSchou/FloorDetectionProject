#ifndef PLANE_HPP
#define PLANE_HPP

#include <master_project/quadtree.hpp>
#include <master_project/helper_function.hpp>

class Plane {
public:

  Plane(void)
  {
    rootRepresentativeness = 0.0;
    samples = 0.0;
    isShowing = true;
    rotate = 0.0;
    normal = cv::Mat::zeros(3, 1, CV_64F);
    mean = cv::Mat::zeros(3, 1, CV_64F);
    samples = 0;
  }

  ~Plane(void)
  {
  }

  void computePlaneParameters()
  {
    // Get normal vector and plane position
    normal.at<double>(0) = std::sin(phi) * std::cos(theta);
    normal.at<double>(1) = std::sin(phi) * std::sin(theta);
    normal.at<double>(2) = std::cos(phi);
    position = normal * rho;

    // Get cross product of normal vector???)
    double  v[3] = { 0.0, 0.0, 1.0 };
    cv::Mat cu(3, 1, CV_64F, v);
    if (isMatEqual(cu, normal)) {
      v[0] = 1.0; v[2] = 0.0;
      cu = cv::Mat(3, 1, CV_64F, v);
    }
    cross = normalizeVector(cu.mul(normal));
    cross2 = normal.mul(cross);

    // Get nodes representing the plane
    for (Quadtree *node : nodes) {
      rootRepresentativeness += node->rootRepresentativeness;
      mean += node->mean;
      samples += node->samples;
    }
    mean /= (double)nodes.size();
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

  double theta, phi, rho, votes, rootRepresentativeness, rotate, thetaIndex;
  int phiIndex, rhoIndex, samples;
  bool isShowing;
  cv::Mat cross, cross2, position, mean, normal, color;
  std::vector<std::vector<double> > validCoordinates;
  std::vector<std::vector<int> > validPixels;
  std::set<Quadtree *> nodes;
};

#endif // PLANE_HPP

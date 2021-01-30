#ifndef QUADTREE_HPP
#define QUADTREE_HPP

#include "traversable_area_detection/summed_area_table.hpp"
#include "traversable_area_detection/camera_data.hpp"

#define MAX_DISTANCE 10.0  // [m]
#define MIN_DISTANCE 0.195 // [m]
#define X 0
#define Y 1
#define Z 2
#define MIN_EIGENVALUE_INDEX 2

class Quadtree {
public:

  Quadtree();
  ~Quadtree();
  void  divideIntoQuadrants();
  void  computeSampleDensity();
  void  PCA();
  void  computeMean();
  void  computeCovariance();
  void         initializeRoot(CameraData& cameraData);
  void         setMaxPlaneThickness(const double maxPlaneThickness);
  const double getMaxPlaneThickness() const;
  void         setMinSamplesInNode(const int minSamplesInNode);
  const int    getMinSamplesInNode() const;

  cv::Matx33d covariance;
  cv::Vec3d mean, normal;
  cv::Scalar color;
  cv::Point minBounds, maxBounds;
  Quadtree *root, *children;
  int samples, level, id, idNo;
  double areaThickness, minVariance, maxPlaneDistance, rootRepresentativeness,
         sampleDensity, maxPhiAngle, rho, phi, theta;
  SummedAreaTable sat;
  std::vector<Quadtree *> planes, nonPlanes;
  bool isPlane;

private:

  double maxPlaneThickness_;
  int minSamplesInNode_;
};

#endif // QUADTREE_HPP

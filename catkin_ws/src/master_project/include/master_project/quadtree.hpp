#ifndef QUADTREE_HPP
#define QUADTREE_HPP

#include "master_project/summed_area_table.hpp"
#include "master_project/camera_data.hpp"

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
  float getGradient(const cv::Mat& m,
                    const double   v,
                    const int      r,
                    const int      c);
  float        getSobelGradientX(const cv::Mat& depth, const int r,
                                 const int c);
  float        getSobelGradientY(const cv::Mat& depth, const int r,
                                 const int c);
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

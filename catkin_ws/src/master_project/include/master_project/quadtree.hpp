#ifndef QUADTREE_HPP
#define QUADTREE_HPP

#include "master_project/summed_area_table.hpp"
#include "master_project/camera_data.hpp"

class Quadtree {
public:

  Quadtree();
  ~Quadtree();
  void                divideIntoQuadrants();
  void                PCA();
  std::vector<double> computeMean();
  cv::Mat             computeCovariance();
  void                initializeRoot(const CameraData& cameraData);


  std::vector<double> mean;
  cv::Mat covariance, normal;
  cv::Point minBounds, maxBounds;
  Quadtree *root, *children;
  int samples, level;
  double areaThickness, minVariance, maxDistance;
  SummedAreaTable sat;
  std::vector<Quadtree *> planes, nonPlanes;
  std::vector<std::vector<double> > validCoordinates;
  std::vector<std::vector<int> > validPixels;
  bool isPlane;
};

#endif // QUADTREE_HPP

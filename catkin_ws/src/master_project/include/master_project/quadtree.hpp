#ifndef QUADTREE_HPP
#define QUADTREE_HPP

#include "master_project/summed_area_table.hpp"
#include "master_project/camera_data.hpp"

class Quadtree {
public:

  Quadtree();
  ~Quadtree();
  void         divideIntoQuadrants();
  void         PCA();
  void         computeMean();
  void         computeCovariance();
  void         initializeRoot(const CameraData& cameraData);
  void         setMaxPlaneThickness(const double maxPlaneThickness);
  const double getMaxPlaneThickness() const;
  void         setMinSamplesInNode(const int minSamplesInNode);
  const int    getMinSamplesInNode() const;

  cv::Mat covariance, normal, mean, color;
  cv::Point minBounds, maxBounds;
  Quadtree *root, *children;
  int samples, level;
  double areaThickness, minVariance, maxPlaneDistance, rootRepresentativeness;
  SummedAreaTable sat;
  std::vector<Quadtree *> planes, nonPlanes;
  std::vector<std::vector<double> > validCoordinates;
  std::vector<std::vector<int> > validPixels;
  bool isPlane;

private:

  double maxPlaneThickness_;
  int minSamplesInNode_;
};

#endif // QUADTREE_HPP

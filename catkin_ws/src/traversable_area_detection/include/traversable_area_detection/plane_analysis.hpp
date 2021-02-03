#ifndef PLANE_ANALYSIS_HPP
#define PLANE_ANALYSIS_HPP

#include "traversable_area_detection/plane.hpp"
#include <traversable_area_detection/HoughPlaneTransform.h>

namespace PlaneAnalysis {
void   removeSmallPlanes(std::vector<Plane>& planes);
bool   isGround(const Plane* currentFloor, const Plane& plane,
                const float cameraHeight);
bool   isWall(const Plane& plane);
bool   isBetterWall(const Plane& currentWall, const Plane& plane);
bool   isCeiling(const Plane* currentCeil, const Plane& plane);
void   assignPlaneType(std::vector<Plane>& planes,
                       const float         cameraHeight = 0.0);
double getDistanceToNode(const cv::Mat& m, const Quadtree& node,
                         const int r, const int c, const cv::Vec3d p);
Plane  computePlanePoints(std::vector<Plane>& planes,
                          const CameraData  & cameraData,
                          const double        cameraHeight = 0.0);
bool   isFaultyObject(const cv::Mat& m,
                      const double   dist,
                      const int      r,
                      const int      c);
cv::Vec2i getTopViewCoordinates(const cv::Vec3d& p);
void      convert2dTo3d(const cv::Vec3d& p, Plane& plane);
cv::Vec2d convertTopViewToMeters(const cv::Point& p);
void      calculateNormalAndStandardDeviation(Plane                  & plane,
                                              std::vector<Quadtree *>& unfitNodes);
void      mergeSimilarPlanes(std::vector<Plane>     & planes,
                             std::vector<Quadtree *>& unfitNodes);
void      matchUnfitNodes(std::vector<Plane>     & planes,
                          std::vector<Quadtree *>& unfitNodes);
void      removeStandAloneNodes(std::vector<Plane>& planes);
void      computePlaneContour(std::vector<Plane>& planes,
                              Plane             & nonPlanePoints);
void      printPlanesInformation(const std::vector<Plane>& planes);
void      printPlaneInformation(const Plane& plane);
void      cleanUpHeightLimitedAreas(Plane& nonPlanePoints, std::vector<Plane>& planes);
void      insertPlanePublisherInformation(
  traversable_area_detection::HoughPlaneTransform& msg,
  const std::vector<Plane>                       & planes,
  const bool                                       includeNodeInformation = false);
};
#endif // PLANE_ANALYSIS_HPP

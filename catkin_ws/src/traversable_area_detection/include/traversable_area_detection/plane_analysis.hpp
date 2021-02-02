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
double getAngleDifference(const Plane& plane1, const Plane& plane2);
bool   hasSimilarDistance(const Plane& plane1, const Plane& plane2);
bool   hasSimilarDistance(const Plane& plane, const Quadtree& node);
double getDistanceDifference(const Plane& plane1, const Plane& plane2);
bool   hasSimilarNormal(const Plane& plane1, const Plane& plane2);
bool   hasSimilarNormal(const Plane& plane, const Quadtree& node);
void   transferNodes(Plane& plane1, Plane& plane2);
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
bool      isWithinTwoStandardDeviations(const Plane& plane1,
                                        const Plane& plane2);
bool      isWithinTwoStandardDeviations(const Plane   & plane,
                                        const Quadtree& node);
float     getThetaDifference(const Plane& plane1, const Plane& plane2);
float     getThetaDifference(const Plane& plane, const Quadtree& node);
void      mergeSimilarPlanes(std::vector<Plane>     & planes,
                             std::vector<Quadtree *>& unfitNodes);
void      matchUnfitNodes(std::vector<Plane>     & planes,
                          std::vector<Quadtree *>& unfitNodes);
void      removeStandAloneNodes(std::vector<Plane>& planes);
void      computePlaneContour(std::vector<Plane>& planes,
                              Plane             & nonPlanePoints);
float     leastSquareError(const Plane& plane, const Quadtree& node);
void      printPlanesInformation(const std::vector<Plane>& planes);
void      printPlaneInformation(const Plane& plane);
void      cleanUpHeightLimitedAreas(Plane& nonPlanePoints, Plane& floor);
void      insertPlanePublisherInformation(
  traversable_area_detection::HoughPlaneTransform& msg,
  const std::vector<Plane>                       & planes,
  const bool                                       includeNodeInformation = false);
};
#endif // PLANE_ANALYSIS_HPP

#ifndef PLANE_ANALYSIS_HPP
#define PLANE_ANALYSIS_HPP

#define MAX_ANGLE_DIFFERENCE 5.0 * PI / 180.0 // [radians]
#define MAX_DISTANCE_DIFFERENCE 0.1           // [m]

#include "master_project/plane.hpp"

namespace PlaneAnalysis {
void    removeSmallPlanes(std::vector<Plane>& planes);
Plane   getGroundPlane(std::vector<Plane>& planes);
bool    hasSimilarAngle(const Plane& plane1, const Plane& plane2);
double  getAngleDifference(const Plane& plane1, const Plane& plane2);
bool    hasSimilarDistance(const Plane& plane1, const Plane& plane2);
double  getDistanceDifference(const Plane& plane1, const Plane& plane2);
bool    isSimilar(const Plane& plane1, const Plane& plane2);
bool    hasSimilarNormal(const Plane& plane1, const Plane& plane2);
void    transferNodes(Plane& plane1, Plane& plane2);
cv::Mat computePlanePoints(std::vector<Plane>& planes,
                           const CameraData  & cameraData);
void    calculateNewNormal(Plane& plane);
void    mergeSimilarPlanes(std::vector<Plane>& planes);
float   leastSquareError(const Plane& plane, const Quadtree& node);
void 		printPlanesInformation(const std::vector<Plane>& planes);
void 		printPlaneInformation(const Plane& plane);
};
#endif // PLANE_ANALYSIS_HPP

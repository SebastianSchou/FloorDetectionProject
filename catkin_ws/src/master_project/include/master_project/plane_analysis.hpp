#ifndef PLANE_ANALYSIS_HPP
#define PLANE_ANALYSIS_HPP

#define MAX_ANGLE_DIFFERENCE 10 * PI / 180 // [radians]
#define MAX_DISTANCE_DIFFERENCE 0.1      // [m]

#include "master_project/plane.hpp"

namespace PlaneAnalysis {
	Plane getGroundPlane(std::vector<Plane> &planes);
	bool hasSimilarAngle(const Plane &plane1, const Plane &plane2);
	double getAngleDifference(const Plane &plane1, const Plane &plane2);
	bool hasSimilarDistance(const Plane &plane1, const Plane &plane2);
	double getDistanceDifference(const Plane &plane1, const Plane &plane2);
	bool isSimilar(const Plane &plane1, const Plane &plane2);
	void transferNodes(Plane &plane1, Plane &plane2);
};
#endif // PLANE_ANALYSIS_HPP
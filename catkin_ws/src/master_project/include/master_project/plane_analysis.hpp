#ifndef PLANE_ANALYSIS_HPP
#define PLANE_ANALYSIS_HPP

#include "master_project/hough_plane_transform.hpp"

namespace PlaneAnalysis {
	Plane getGroundPlane(std::vector<Plane> &planes);
};
#endif // PLANE_ANALYSIS_HPP
#ifndef DRAWING_FUNCTIONS_HPP
#define DRAWING_FUNCTIONS_HPP

#include <opencv4/opencv2/opencv.hpp>
#include "traversable_area_detection/quadtree.hpp"
#include "traversable_area_detection/camera_data.hpp"
#include "traversable_area_detection/accumulator.hpp"
#include "traversable_area_detection/plane.hpp"

namespace DrawingFunctions {
void    drawQuadtreeBorders(cv::Mat& image, Quadtree& node,
                            CameraData& cameraData);
void    drawQuadtreeBorders(cv::Mat& image, Quadtree& node);
void    drawOnlyPlaneQuadtreeBorders(cv::Mat           & image,
                                     std::vector<Plane>& planes,
                                     CameraData        & cameraData);
cv::Mat drawAccumulatorCellVotes(const int height, const int width,
                                 const Accumulator& accumulator,
                                 const int minRho = 0, const int maxRho = 0);
void    drawPlanesInQuadtree(cv::Mat& image, Quadtree& node,
                             CameraData& cameraData);
void    assignColorToPlane(Plane& plane, int r, int g, int b);
void    assignColorToPlanes(std::vector<Plane>& planes);
void    drawPlanePoints(cv::Mat          & image,
                        std::vector<Plane> planes,
                        CameraData       & cameraData);
void    drawPlanes(cv::Mat& image, const std::vector<Plane>& planes);
cv::Mat drawTopView(const CameraData        & cameraData,
                    const std::vector<Plane>& planes,
                    const cv::Mat           & nonPlanePoints);
cv::Mat drawSideView(const CameraData& cameraData, const Plane& plane);
};

#endif // DRAWING_FUNCTIONS_HPP

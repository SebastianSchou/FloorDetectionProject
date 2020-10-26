#ifndef DRAWING_FUNCTIONS_HPP
#define DRAWING_FUNCTIONS_HPP

#include <opencv4/opencv2/opencv.hpp>
#include "master_project/quadtree.hpp"
#include "master_project/camera_data.hpp"
#include "master_project/accumulator.hpp"

namespace DrawingFunctions {
void    drawQuadtreeBorders(cv::Mat& image, Quadtree& node,
                            CameraData& cameraData);
void    drawQuadtreeBorders(cv::Mat& image, Quadtree& node);
cv::Mat drawAccumulatorCellVotes(const int height, const int width,
                                 const Accumulator& accumulator,
                                 const int minRho = 0, const int maxRho = 0);
};

#endif // DRAWING_FUNCTIONS_HPP

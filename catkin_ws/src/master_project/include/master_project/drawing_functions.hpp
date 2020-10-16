#ifndef DRAWING_FUNCTIONS_HPP
#define DRAWING_FUNCTIONS_HPP

#include <opencv4/opencv2/opencv.hpp>
#include "master_project/quadtree.hpp"
#include "master_project/camera_data.hpp"

namespace DrawingFunctions {
void drawQuadtreeBorders(cv::Mat& image, Quadtree& node,
                         CameraData& cameraData);
void drawQuadtreeBorders(cv::Mat& image, Quadtree& node);
};

#endif // DRAWING_FUNCTIONS_HPP

#include "master_project/drawing_functions.hpp"

void DrawingFunctions::drawQuadtreeBorders(cv::Mat   & image,
                                           Quadtree  & node,
                                           CameraData& cameraData)
{
  if (node.isPlane) {
    cv::rectangle(image,
                  node.minBounds * cameraData.filterVariables.decimationScaleFactor,
                  node.maxBounds * cameraData.filterVariables.decimationScaleFactor,
                  (255, 255, 0),
                  3);
  } else {
    cv::rectangle(image,
                  node.minBounds * cameraData.filterVariables.decimationScaleFactor,
                  node.maxBounds * cameraData.filterVariables.decimationScaleFactor,
                  (255, 0, 0),
                  1);
  }
  if (node.children != NULL) {
    for (int i = 0; i < 4; i++) {
      drawQuadtreeBorders(image, node.children[i], cameraData);
    }
  }
}

void DrawingFunctions::drawQuadtreeBorders(cv::Mat& image, Quadtree& node)
{
  if (node.isPlane) {
    cv::rectangle(image, node.minBounds, node.maxBounds, (255, 255, 0), 3);
  } else {
    cv::rectangle(image, node.minBounds, node.maxBounds, (255, 0, 0), 1);
  }
  if (node.children != NULL) {
    for (int i = 0; i < 4; i++) {
      drawQuadtreeBorders(image, node.children[i]);
    }
  }
}

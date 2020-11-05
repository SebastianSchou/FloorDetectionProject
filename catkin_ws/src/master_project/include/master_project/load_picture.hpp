#include <stdio.h>
#include <opencv4/opencv2/core/ocl.hpp>
#include "master_project/camera_data.hpp"

#define EXIT_SUCCESS 0
#define EXIT_ERROR -1

float leastSquareError(const float v1, const float v2)
{
  return square(v1 - v2);
}

int loadPicture(int argc, char **argv, int width, int height, int fps,
                const std::string& filePath)
{
  // Initialize ros
  ros::init(argc, argv, "take_picture");
  ros::NodeHandle nh;

  std::string picture;
  if (!nh.getParam("/file", picture)) {
    ROS_ERROR("No file given.");
    return EXIT_ERROR;
  } else if (picture == "all") {
    ROS_ERROR("Loading all images is currently not supported");
    return EXIT_ERROR;
  }

  // Init Intel RealSense camera
  CameraData cameraData(width, height, fps);
  if (!cameraData.initializeCamera()) {
    return EXIT_ERROR;
  }

  std::string filename = filePath + picture;

  if (!cameraData.loadImage(filename)) {
    return EXIT_ERROR;
  }

  auto start = std::chrono::steady_clock::now();

  // Proces planes
  HoughPlaneTransform houghPlaneTransform(cameraData);
  std::vector<Plane>  planes = houghPlaneTransform.planes;
  Plane floor = PlaneAnalysis::getGroundPlane(planes);

  if (floor.rho != 0.0) {
    houghPlaneTransform.assignColorToPlane(floor, 0, 0, 255);
  }

  printf("Processing time: %.3f ms\n", msUntilNow(start));

  // Draw
  DrawingFunctions::drawQuadtreeBorders(cameraData.colorizedDepthImage,
                                        houghPlaneTransform.root);
  houghPlaneTransform.assignColorToPlanes();
  DrawingFunctions::drawPlanesInQuadtree(cameraData.depthAlignedColorImage,
                                         houghPlaneTransform.root,
                                         cameraData);
  cv::Mat accumDrawing = DrawingFunctions::drawAccumulatorCellVotes(
    width, height, *houghPlaneTransform.accumulator);

  // Show data
  cv::imshow("Accumulator", accumDrawing);
  cv::imshow("Realsense color", cameraData.depthAlignedColorImage);
  cv::imshow("Realsense color normal", cameraData.normalColorImage);
  cv::waitKey(0);

  // Shutdown
  cameraData.pipe.stop();
  ros::shutdown();
  return EXIT_SUCCESS;
}

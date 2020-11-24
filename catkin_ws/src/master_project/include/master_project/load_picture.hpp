#include <stdio.h>
#include <opencv4/opencv2/core/ocl.hpp>
#include "master_project/camera_data.hpp"

#define EXIT_SUCCESS 0
#define EXIT_ERROR -1

int loadPicture(int argc, char **argv, const std::string& filePath)
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
  CameraData cameraData;
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
  Plane   floor = PlaneAnalysis::getGroundPlane(planes);
  cv::Mat nonPlanePoints = PlaneAnalysis::computePlanePoints(planes,
                                                             cameraData);

  printf("Processing time: %.3f ms\n", msUntilNow(start));


  if (floor.rho != 0.0) {
    DrawingFunctions::assignColorToPlane(floor, 0, 0, 255);
  }

  // Draw
  cv::Mat onlyQuadtree = cameraData.depthAlignedColorImage.clone();
  cv::Mat cleanedPlanePoints = cameraData.depthAlignedColorImage.clone();
  DrawingFunctions::assignColorToPlanes(planes);
  DrawingFunctions::drawQuadtreeBorders(cameraData.colorizedDepthImage,
                                        houghPlaneTransform.root);
  DrawingFunctions::drawPlanesInQuadtree(onlyQuadtree,
                                         houghPlaneTransform.root,
                                         cameraData);
  DrawingFunctions::drawOnlyPlaneQuadtreeBorders(cleanedPlanePoints,
                                                 planes,
                                                 cameraData);
  DrawingFunctions::drawPlanes(cleanedPlanePoints, planes);
  DrawingFunctions::drawPlanesInQuadtree(cameraData.depthAlignedColorImage,
                                         houghPlaneTransform.root,
                                         cameraData);
  DrawingFunctions::drawPlanePoints(cameraData.depthAlignedColorImage,
                                    planes,
                                    cameraData);

  // cv::Mat accumDrawing = DrawingFunctions::drawAccumulatorCellVotes(
  //  width, height, *houghPlaneTransform.accumulator);

  // Show data
  // cv::imshow("Accumulator", accumDrawing);
  cv::imshow("Realsense non plane points", nonPlanePoints);
  cv::imshow("Realsense plane points cleaned", cleanedPlanePoints);
  cv::imshow("Realsense plane quadtree", onlyQuadtree);
  cv::imshow("Realsense plane points rough", cameraData.depthAlignedColorImage);
  cv::imshow("Realsense color normal", cameraData.normalColorImage);
  cv::waitKey(0);

  // Shutdown
  cameraData.pipe.stop();
  ros::shutdown();
  return EXIT_SUCCESS;
}

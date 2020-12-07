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

  // Load image
  CameraData  cameraData;
  std::string filename = filePath + picture;
  if (!cameraData.loadImage(filename)) {
    return EXIT_ERROR;
  }

  auto start = std::chrono::steady_clock::now();

  // Process planes
  HoughPlaneTransform houghPlaneTransform(cameraData, true);
  std::vector<Plane>  planes = houghPlaneTransform.planes;
  if (planes.empty()) {
    ROS_INFO("No planes found. Processing time: %.3f ms", msUntilNow(start));
    cv::imshow("Realsense color normal", cameraData.normalColorImage);
    cv::waitKey(0);

    // Shutdown
    ros::shutdown();
    return EXIT_SUCCESS;
  }
  auto  timePlanePoints = std::chrono::steady_clock::now();
  Plane nonPlanePoints = PlaneAnalysis::computePlanePoints(planes,
                                                           cameraData);
  PlaneAnalysis::computePlaneContour(planes, nonPlanePoints);
  ROS_INFO("Processing time: %.3f ms. Computing plane points time: %.3f ms\n",
           msUntilNow(start), msUntilNow(timePlanePoints));

  // PlaneAnalysis::printPlanesInformation(planes);

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
  cv::Mat topView =
    DrawingFunctions::drawTopView(cameraData, planes, nonPlanePoints.topView);
  cv::Mat sideView;
  for (Plane& plane : planes) {
    if (plane.type == PLANE_TYPE_FLOOR) {
      sideView = DrawingFunctions::drawSideView(cameraData, plane);
      cv::imshow("Side view", sideView);
      break;
    }
  }

  // cv::Mat accumDrawing = DrawingFunctions::drawAccumulatorCellVotes(
  //  width, height, *houghPlaneTransform.accumulator);

  // Show data
  // cv::imshow("Accumulator", accumDrawing);
  cv::imshow("Top view", topView);
  cv::imshow("Realsense non plane points", nonPlanePoints.image2dPoints);
  cv::imshow("Realsense plane points cleaned", cleanedPlanePoints);
  cv::imshow("Realsense plane quadtree", onlyQuadtree);
  cv::imshow("Realsense plane points rough", cameraData.depthAlignedColorImage);
  cv::imshow("Realsense color normal", cameraData.normalColorImage);
  cv::waitKey(0);

  // Shutdown
  ros::shutdown();
  return EXIT_SUCCESS;
}

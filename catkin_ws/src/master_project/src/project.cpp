#include <stdio.h>
#include <opencv4/opencv2/core/ocl.hpp>
#include "master_project/drawing_functions.hpp"
#include "master_project/hough_plane_transform.hpp"
#include "master_project/plane_analysis.hpp"
#include "master_project/take_picture.hpp"
#include "master_project/load_picture.hpp"


#define EXIT_SUCCESS 0
#define EXIT_ERROR -1
#define FILE_PATH "/home/sebastian/Master/images/"

int main(int argc, char **argv)
{
  if (std::string(argv[1]) == "--take_picture") {
    return takePicture(argc, argv, FILE_PATH);
  } else if (std::string(argv[1]) == "--load_picture") {
    return loadPicture(argc, argv, FILE_PATH);
  }

  // Initialize ros
  ros::init(argc, argv, "master_project");
  ros::NodeHandle nh;

  // Init Intel RealSense camera
  CameraData cameraData;
  if (!cameraData.initializeCamera()) {
    return EXIT_ERROR;
  }

  // Main loop
  float timeSum = 0.0;
  int   iteration = 0;
  char  key = ' ';
  while (key != 'q') {
    // Record processing time
    auto start = std::chrono::steady_clock::now();

    // Process frame
    if (!cameraData.processFrames()) {
      return EXIT_ERROR;
    }

    // Process planes
    HoughPlaneTransform houghPlaneTransform(cameraData);
    std::vector<Plane>  planes = houghPlaneTransform.planes;
    if (planes.empty()) {
      ROS_INFO("No planes found");
      cv::imshow("Realsense color normal", cameraData.normalColorImage);
      key = cv::waitKey(1);
    } else {
      auto timePlanePoints = std::chrono::steady_clock::now();
      Plane nonPlanePoints = PlaneAnalysis::computePlanePoints(planes,
                                                               cameraData);
      PlaneAnalysis::computePlaneContour(planes, nonPlanePoints);

      // Calculate average itteration time
      timeSum += msUntilNow(start);
      iteration++;

      // Draw
      cv::Mat cleanedPlanePoints = cameraData.depthAlignedColorImage.clone();
      DrawingFunctions::assignColorToPlanes(planes);
      DrawingFunctions::drawOnlyPlaneQuadtreeBorders(cleanedPlanePoints,
                                                     planes,
                                                     cameraData);
      DrawingFunctions::drawPlanes(cleanedPlanePoints, planes);
      cv::Mat topView =
        DrawingFunctions::drawTopView(cameraData, planes,
                                      nonPlanePoints.topView);

      // Show data
      cv::imshow("Top view", topView);
      cv::imshow("Realsense plane points cleaned", cleanedPlanePoints);
      cv::imshow("Realsense color normal", cameraData.normalColorImage);
      key = cv::waitKey(1);
    }
  }
  ROS_INFO("Script ended. Shutting down. Avg. processing time: %.3f ms",
           timeSum / (float)iteration);

  // Shutdown
  cameraData.pipe.stop();
  ros::shutdown();
  return EXIT_SUCCESS;
}

#include <stdio.h>
#include <opencv4/opencv2/core/ocl.hpp>
#include "master_project/drawing_functions.hpp"
#include "master_project/hough_plane_transform.hpp"
#include "master_project/plane_analysis.hpp"


#define EXIT_SUCCESS 0
#define EXIT_ERROR -1
#define IMAGE_WIDTH_DEPTH 848
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define CAPTURED_FRAMES_PER_SECONDS 15

int main(int argc, char **argv)
{
  // Initialize ros
  ros::init(argc, argv, "master_project");
  ros::NodeHandle nh;

  // Init Intel RealSense camera
  CameraData cameraData(IMAGE_WIDTH_DEPTH, IMAGE_HEIGHT,
                        CAPTURED_FRAMES_PER_SECONDS);
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

    HoughPlaneTransform houghPlaneTransform(cameraData);
    std::vector<Plane>  planes = houghPlaneTransform.planes;
    Plane floor = PlaneAnalysis::getGroundPlane(planes);

    if (floor.rho != 0.0) {
      houghPlaneTransform.assignColorToPlane(floor, 0, 0, 255);
    }

    // houghPlaneTransform.printTimePartition();
    // houghPlaneTransform.printPlaneInformation(floor);
    // houghPlaneTransform.printPlanesInformation();

    // Calculate average itteration time
    timeSum += msUntilNow(start);
    iteration++;

    // Draw illustrations
    DrawingFunctions::drawQuadtreeBorders(cameraData.colorizedDepthImage,
                                          houghPlaneTransform.root);
    houghPlaneTransform.assignColorToPlanes();
    DrawingFunctions::drawPlanesInQuadtree(cameraData.depthAlignedColorImage,
                                           houghPlaneTransform.root,
                                           cameraData);
    cv::Mat accumDrawing = DrawingFunctions::drawAccumulatorCellVotes(
      IMAGE_WIDTH_DEPTH, IMAGE_HEIGHT, *houghPlaneTransform.accumulator);

    // Show data
    cv::imshow("Realsense depth", cameraData.colorizedDepthImage);
    cv::imshow("Accumulator", accumDrawing);
    cv::imshow("Realsense color", cameraData.depthAlignedColorImage);
    cv::imshow("Realsense color normal", cameraData.normalColorImage);
    cv::imshow("Realsense IR", cameraData.irImage);
    key = cv::waitKey(1);
  }
  ROS_INFO("Script ended. Shutting down. Avg. processing time: %.3f ms",
           timeSum / (float)iteration);

  // Shutdown
  cameraData.pipe.stop();
  ros::shutdown();
  return EXIT_SUCCESS;
}

#include <stdio.h>
#include <opencv4/opencv2/core/ocl.hpp>
#include "traversable_area_detection/drawing_functions.hpp"
#include "traversable_area_detection/hough_plane_transform.hpp"
#include "traversable_area_detection/plane_analysis.hpp"
#include "traversable_area_detection/take_picture.hpp"
#include "traversable_area_detection/load_picture.hpp"


#define EXIT_SUCCESS 0
#define EXIT_ERROR -1
#define FILE_PATH "/home/sebastian/Master/images/"

static ros::Publisher pubHoughPlaneTransformLive;

static int planePubCallback(CameraData& cameraData, float& timeSum,
                            int& iteration, char& key)
{
  // Create publisher
  traversable_area_detection::HoughPlaneTransform msg;

  // Record processing time
  auto start = std::chrono::steady_clock::now();

  // Process frame
  if (!cameraData.processFrames()) {
    msg.success = false;
    pubHoughPlaneTransform.publish(msg);
    return EXIT_ERROR;
  }

  // Process planes
  HoughPlaneTransform houghPlaneTransform(cameraData);
  std::vector<Plane>  planes = houghPlaneTransform.planes;
  if (planes.empty()) {
    msg.computation_time = msUntilNow(start);
  } else {
    Plane nonPlanePoints = PlaneAnalysis::computePlanePoints(planes,
                                                             cameraData);
    PlaneAnalysis::computePlaneContour(planes, nonPlanePoints);

    // Set publisher message
    msg.computation_time = msUntilNow(start);
    PlaneAnalysis::insertPlanePublisherInformation(msg, planes);

    // Calculate average itteration time
    timeSum += msg.computation_time;
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
  }
  msg.success = true;
  pubHoughPlaneTransformLive.publish(msg);
  cv::imshow("Realsense color normal", cameraData.normalColorImage);
  key = cv::waitKey(1);
  return 0;
}

int main(int argc, char **argv)
{
  if (std::string(argv[1]) == "--take_picture") {
    return takePicture(argc, argv, FILE_PATH);
  } else if (std::string(argv[1]) == "--load_picture") {
    return loadPicture(argc, argv, FILE_PATH);
  }

  // Initialize ros
  ros::init(argc, argv, "traversable_area_detection");
  ros::NodeHandle nh;

  // Init Intel RealSense camera
  CameraData cameraData;
  if (!cameraData.initializeCamera()) {
    return EXIT_ERROR;
  }

  // Init publisher
  pubHoughPlaneTransformLive =
    nh.advertise<traversable_area_detection::HoughPlaneTransform>("hough_plane_transform",
                                                      QUEUE_SIZE);

  // Main loop
  float timeSum = 0.0;
  int   iteration = 0;
  char  key = ' ';
  ros::Rate rate(20); // [Hz]
  while (key != 'q') {
    if (planePubCallback(cameraData, timeSum, iteration, key) == EXIT_ERROR) {
      return EXIT_ERROR;
    }
    rate.sleep();
  }
  ROS_INFO("Script ended. Shutting down. Avg. processing time: %.3f ms",
           timeSum / (float)iteration);

  // Shutdown
  cameraData.pipe.stop();
  ros::shutdown();
  return EXIT_SUCCESS;
}

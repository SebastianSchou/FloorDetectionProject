#include <stdio.h>
#include <algorithm>
#include <math.h>
#include <librealsense2/rs.hpp>
#include <opencv4/opencv2/core/ocl.hpp>
#include "ros/ros.h"
#include "master_project/helper_function.hpp"
#include "master_project/summed_area_table.hpp"
#include "master_project/quadtree.hpp"
#include "master_project/camera_data.hpp"

#define EXIT_SUCCESS 0
#define EXIT_ERROR -1
#define IMAGE_WIDTH_DEPTH 848
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define CAPTURED_FRAMES_PER_SECONDS 15

using namespace cv;

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

    // Show data
    imshow("Realsense depth", cameraData.colorizedDepthImage);
    imshow("Realsense color", cameraData.depthAlignedColorImage);
    imshow("Realsense color normal", cameraData.normalColorImage);
    imshow("Realsense IR", cameraData.irImage);

    timeSum += msUntilNow(start);
    iteration++;
    key = waitKey(1);
  }
  ROS_INFO("Script ended. Shutting down. Avg. processing time: %.3f ms",
           timeSum / (float)iteration);

  // Shutdown
  cameraData.pipe.stop();
  ros::shutdown();
  return EXIT_SUCCESS;
}

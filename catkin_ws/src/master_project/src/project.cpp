#include <stdio.h>
#include <algorithm>
#include <math.h>
#include <librealsense2/rs.hpp>
#include <opencv4/opencv2/core/ocl.hpp>
#include "master_project/helper_function.hpp"
#include "master_project/summed_area_table.hpp"
#include "master_project/quadtree.hpp"
#include "master_project/camera_data.hpp"
#include "master_project/accumulator.hpp"
#include "master_project/voting.hpp"
#include "master_project/drawing_functions.hpp"

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

    // Data extraction from matrices should be in the form (row, col)
    // unless otherwise stated. This means that points (x, y) are (col, row).

    Quadtree root;
    root.initializeRoot(cameraData);
    root.divideIntoQuadrants();
    Accumulator *accumulator = new Accumulator(root.maxDistance, 80, 30);
    std::vector<Bin> usedBins;
    std::vector<Kernel> usedKernels;
    voting(root, *accumulator, usedBins, usedKernels);

    // Accumulator
    // Hough transform

    // Calculate average itteration time
    timeSum += msUntilNow(start);
    iteration++;

    // Draw illustrations
    DrawingFunctions::drawQuadtreeBorders(cameraData.colorizedDepthImage,
                                          root);
    DrawingFunctions::drawQuadtreeBorders(cameraData.depthAlignedColorImage,
                                          root, cameraData);
    cv::Mat accumDrawing = DrawingFunctions::drawAccumulatorCellVotes(
      IMAGE_WIDTH_DEPTH, IMAGE_HEIGHT, *accumulator);

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

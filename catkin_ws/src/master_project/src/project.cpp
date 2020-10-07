#include <stdio.h>
#include <algorithm>
#include <librealsense2/rs.hpp>
#include <opencv4/opencv2/core/ocl.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include "ros/ros.h"
#include "master_project/helper_function.hpp"

#define EXIT_SUCCESS 0
#define EXIT_ERROR -1
#define IMAGE_WIDTH_DEPTH 848
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define CAPTURED_FRAMES_PER_SECONDS 15
#define DECIMATION_SCALE_FACTIOR 2
#define SPATIAL_HOLE_FILLING_MODE 4
#define TEMPORAL_SMOOTH_ALPHA 0.4
#define TEMPORAL_SMOOTH_DELTA 20
#define TEMPORAL_PERSITENCY_MODE 4 // Last 2/8 images
#define CAMERA_TIMEOUT 5000        // ms

using namespace cv;

int main(int argc, char **argv)
{
  // Initialize ros
  ros::init(argc, argv, "master_project");
  ros::NodeHandle nh;

  // Check if Realsense camera is connected
  rs2_error   *e = 0;
  rs2_context *context = rs2_create_context(RS2_API_VERSION, &e);
  rs2_device_list *deviceList = rs2_query_devices(context, &e);
  int noOfDevices = rs2_get_device_count(deviceList, &e);
  if (noOfDevices == 0) {
    ROS_ERROR("No camera is connected. Exiting script");
    ros::shutdown();
    return EXIT_ERROR;
  }

  // Start rs2 pipe with config
  ROS_INFO("Initializing Realsense D435 camera.");

  rs2::pipeline pipe;
  rs2::config   cfg;
  cfg.enable_stream(RS2_STREAM_COLOR,
                    IMAGE_WIDTH_DEPTH,
                    IMAGE_HEIGHT,
                    RS2_FORMAT_BGR8,
                    CAPTURED_FRAMES_PER_SECONDS);
  cfg.enable_stream(RS2_STREAM_DEPTH,
                    IMAGE_WIDTH_DEPTH,
                    IMAGE_HEIGHT,
                    RS2_FORMAT_Z16,
                    CAPTURED_FRAMES_PER_SECONDS);
  cfg.enable_stream(RS2_STREAM_INFRARED,
                    IMAGE_WIDTH_DEPTH,
                    IMAGE_HEIGHT,
                    RS2_FORMAT_Y8,
                    CAPTURED_FRAMES_PER_SECONDS);
  rs2::pipeline_profile pipeProfile = pipe.start(cfg);

  // Get depth scale
  rs2::device device = pipeProfile.get_device();
  rs2::depth_sensor depthSensor = device.query_sensors().front().as<
    rs2::depth_sensor>();
  depthSensor.set_option(RS2_OPTION_VISUAL_PRESET,
                         RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
  float depthScale = depthSensor.get_depth_scale();

  // Colorizer for color in depth image
  rs2::colorizer color_map;

  // Depth filters
  // Decimation filter reduces depth frame density
  rs2::decimation_filter decimationFilter;
  decimationFilter.set_option(RS2_OPTION_FILTER_MAGNITUDE,
                              DECIMATION_SCALE_FACTIOR);
  if ((IMAGE_WIDTH_DEPTH % DECIMATION_SCALE_FACTIOR != 0) ||
      (IMAGE_HEIGHT % DECIMATION_SCALE_FACTIOR != 0)) {
    ROS_WARN("The image width and/or height are not multiples of the"
             " decimation scale factor. This can result in errors in shown"
             " images, but has no effect on the result.");
  }

  // Edge-preserving spatial smoothing
  rs2::spatial_filter spatialFilter;
  spatialFilter.set_option(RS2_OPTION_HOLES_FILL,
                           SPATIAL_HOLE_FILLING_MODE);

  // Temporal filter reduces temporal noise by using knowledge from
  // previous frames
  rs2::temporal_filter temporalFilter(TEMPORAL_SMOOTH_ALPHA,
                                      TEMPORAL_SMOOTH_DELTA,
                                      TEMPORAL_PERSITENCY_MODE);

  // Hole filling filter (fills holes, duh)
  rs2::hole_filling_filter holeFillingFilter;

  // Initialize frames
  rs2::frameset frames;

  // Align depth and color frames
  rs2::align align_to_depth(RS2_STREAM_DEPTH);
  rs2::align align_to_color(RS2_STREAM_COLOR);

  ROS_INFO("Realsense D435 camera initialized. Starting main script.");

  // Allow autoexposure to settle
  try {
    for (auto i = 0; i < CAPTURED_FRAMES_PER_SECONDS; ++i) {
      pipe.wait_for_frames(CAMERA_TIMEOUT);
    }
  } catch (...) {
    ROS_ERROR("Script timed out while waiting for frames (%d ms)."
              " Exiting script.", CAMERA_TIMEOUT);
    ros::shutdown();
    return EXIT_ERROR;
  }

  // Main loop
  float timeSum = 0.0;
  int   iteration = 0;
  char  key = ' ';
  while (key != 'q') {
    // Read camera data
    try {
      frames = pipe.wait_for_frames(CAMERA_TIMEOUT);
    } catch (...) {
      deviceList = rs2_query_devices(context, &e);
      if (rs2_get_device_count(deviceList, &e) == 0) {
        ROS_ERROR("Camera has been disconnected. Exiting script.");
      } else {
        ROS_ERROR("Script timed out while waiting for frames (%d ms)."
                  " Exiting script.", CAMERA_TIMEOUT);
      }
      ros::shutdown();
      return EXIT_ERROR;
    }

    // Check if data is valid
    if (frames == 0) {
      ROS_ERROR("Camera returned an invalid frame. Exiting script.");
      ros::shutdown();
      return EXIT_ERROR;
    }

    // Record processing time
    auto start = std::chrono::steady_clock::now();

    // Get regular color image and align data to depth
    rs2::video_frame colorNormal = frames.get_color_frame();
    frames = align_to_depth.process(frames);

    // Get data
    rs2::depth_frame depth = frames.get_depth_frame();
    rs2::depth_frame depthFrameFiltered = decimationFilter.process(depth);
    depthFrameFiltered = spatialFilter.process(depthFrameFiltered);
    depthFrameFiltered = temporalFilter.process(depthFrameFiltered);
    depthFrameFiltered = holeFillingFilter.process(depthFrameFiltered);
    rs2::video_frame depthColor = color_map.colorize(depthFrameFiltered);
    rs2::video_frame color = frames.get_color_frame();
    rs2::video_frame ir = frames.first(RS2_STREAM_INFRARED);

    // Convert to Mat for OpenCV use
    // Color image aligned to depth
    Mat colorMat(Size(IMAGE_WIDTH_DEPTH, IMAGE_HEIGHT),
                 CV_8UC3,
                 (void *)color.get_data(),
                 Mat::AUTO_STEP);

    // Regular color image
    Mat colorNormalMat(Size(IMAGE_WIDTH_DEPTH, IMAGE_HEIGHT),
                       CV_8UC3,
                       (void *)colorNormal.get_data(),
                       Mat::AUTO_STEP);

    // Colorized depth image
    Mat depthColorMat(Size(depthFrameFiltered.get_width(),
                           depthFrameFiltered.get_height()),
                      CV_8UC3,
                      (void *)depthColor.get_data(),
                      Mat::AUTO_STEP);

    // IR image
    Mat irMat(Size(IMAGE_WIDTH_DEPTH, IMAGE_HEIGHT),
              CV_8UC1,
              (void *)ir.get_data(),
              Mat::AUTO_STEP);;

    // Get depth data as a matrix
    const int width = (int)depthFrameFiltered.get_width();
    const int height = (int)depthFrameFiltered.get_height();
    auto depthArray = reinterpret_cast<const uint16_t *>(
      depthFrameFiltered.get_data());
    float distanceArray[width * height];
    for (int i = 0; i < width * height; i++) {
      distanceArray[i] = depthArray[i] * depthScale;
    }
    Mat distance(Size(width, height),
                 CV_32F,
                 distanceArray,
                 Mat::AUTO_STEP);

    // Data can be extracted by distance.at<float>(row, col)

    // Equalize IR frame
    equalizeHist(irMat, irMat);
    applyColorMap(irMat, irMat, COLORMAP_JET);

    // Show data
    imshow("Realsense depth", depthColorMat);
    imshow("Realsense color", colorMat);
    imshow("Realsense color normal", colorNormalMat);
    imshow("Realsense IR", irMat);

    timeSum += msUntilNow(start);
    iteration++;
    key = waitKey(1);
  }
  ROS_INFO("Script ended. Shutting down. Avg. processing time: %.3f ms",
           timeSum / (float)iteration);

  // Shutdown
  pipe.stop();
  ros::shutdown();
  return EXIT_SUCCESS;
}

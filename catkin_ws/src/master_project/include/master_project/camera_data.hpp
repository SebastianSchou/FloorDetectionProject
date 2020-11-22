#ifndef CAMERA_DATA_HPP
#define CAMERA_DATA_HPP

#include <opencv4/opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include "ros/ros.h"

#define IMAGE_SCALE 4    // For decimation filter
#define IMAGE_WIDTH 848  // [pixel]
#define IMAGE_HEIGHT 480 // [pixel]
#define DEPTH_WIDTH IMAGE_WIDTH / IMAGE_SCALE
#define DEPTH_HEIGHT IMAGE_HEIGHT / IMAGE_SCALE

struct FilterVariables {
  int decimationScaleFactor = IMAGE_SCALE,
      spatialHoleFillingMode = 4,
      temporalSmoothDelta = 20,
      temporalPersistencyMode = 2;
  float temporalSmoothAlpha = 0.4;
};

class CameraData {
public:

  CameraData();
  bool initializeCamera();
  bool processFrames();
  bool loadImage(const std::string& filename);

  rs2::pipeline pipe;
  rs2::pipeline_profile profile;
  int width, height;
  rs2_intrinsics intrinsics;
  cv::Mat normalColorImage, colorizedDepthImage, depthAlignedColorImage,
          irImage, depthData, data3d;
  FilterVariables filterVariables;

private:

  rs2_error *e;
  rs2_context *context;
  rs2_device_list *deviceList;
  rs2::config config;
  rs2::spatial_filter spatialFilter;
  rs2::temporal_filter temporalFilter;
  rs2::decimation_filter decimationFilter;
  rs2::colorizer colorMap;
  rs2::frameset frames;
  int timeout = 5000;
  float depthScale;
};

#endif // CAMERA_DATA_HPP

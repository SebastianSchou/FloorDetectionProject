#ifndef CAMERA_DATA_HPP
#define CAMERA_DATA_HPP

#include <opencv4/opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include "ros/ros.h"

struct FilterVariables {
  int decimationScaleFactor = 4,
      spatialHoleFillingMode = 4,
      temporalSmoothDelta = 20,
      temporalPersistencyMode = 2;
  float temporalSmoothAlpha = 0.4;
};

class CameraData {
public:

  CameraData(int width, int height, int fps);
  bool initializeCamera();
  bool processFrames();

  rs2::pipeline pipe;
  rs2::pipeline_profile profile;
  float depthScale;
  int width, height;
  rs2_intrinsics intrinsics;
  cv::Mat normalColorImage, colorizedDepthImage, depthAlignedColorImage,
          irImage;
  const uint16_t *depthArray;
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
  int imageWidth, imageHeight, cameraFps, timeout = 5000;
};

#endif // CAMERA_DATA_HPP

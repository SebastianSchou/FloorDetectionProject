#ifndef CAMERA_DATA_HPP
#define CAMERA_DATA_HPP

#include <opencv4/opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include "ros/ros.h"

#define IMAGE_SCALE 4    // For decimation filter
#define IMAGE_WIDTH 848  // [pixel]
#define IMAGE_HEIGHT 480 // [pixel]
#define USUAL_FY 422.772369
#define USUAL_FX 422.772369
#define USUAL_PPX 424.636292
#define USUAL_PPY 239.201035

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
  bool initializeCamera(const bool loadColorImage = true);
  bool processFrames();
  bool loadImage(const std::string& filename);

  rs2::pipeline pipe;
  rs2::pipeline_profile profile;
  int width, height;
  double fx, fy, ppx, ppy;
  cv::Mat normalColorImage, depthAlignedColorImage, depthData, data3d;
  FilterVariables filterVariables;
  bool loadColorImage_;

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

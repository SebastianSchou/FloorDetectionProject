#include "traversable_area_detection/camera_data.hpp"
#include "ros/ros.h"

#define FRAMES_PER_SECOND 15

CameraData::CameraData()
{
}

bool CameraData::initializeCamera()
{
  // Check if Realsense camera is connected
  e = 0;
  context = rs2_create_context(RS2_API_VERSION, &e);
  deviceList = rs2_query_devices(context, &e);

  if (rs2_get_device_count(deviceList, &e) == 0) {
    ROS_ERROR("No camera is connected. Exiting script");
    ros::shutdown();
    return false;
  }

  // Start rs2 pipe with config
  ROS_INFO("Initializing Realsense D435 camera.");

  config.enable_stream(RS2_STREAM_COLOR,
                       IMAGE_WIDTH,
                       IMAGE_HEIGHT,
                       RS2_FORMAT_BGR8,
                       FRAMES_PER_SECOND);
  config.enable_stream(RS2_STREAM_DEPTH,
                       IMAGE_WIDTH,
                       IMAGE_HEIGHT,
                       RS2_FORMAT_Z16,
                       FRAMES_PER_SECOND);
  try {
    profile = pipe.start(config);
  } catch (rs2::error errorMsg) {
    ROS_ERROR("Could not start Realsense camera pipe: %s", errorMsg.what());
    return false;
  }

  // Get depth scale
  rs2::device device = profile.get_device();
  rs2::depth_sensor depthSensor = device.query_sensors().front().as<
    rs2::depth_sensor>();
  depthSensor.set_option(RS2_OPTION_VISUAL_PRESET,
                         RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
  depthScale = depthSensor.get_depth_scale();

  // Depth filters
  // Decimation filter reduces depth frame density
  decimationFilter.set_option(RS2_OPTION_FILTER_MAGNITUDE,
                              filterVariables.decimationScaleFactor);
  if ((IMAGE_WIDTH % filterVariables.decimationScaleFactor != 0) ||
      (IMAGE_HEIGHT % filterVariables.decimationScaleFactor != 0)) {
    ROS_WARN("The image width and/or height are not multiples of the"
             " decimation scale factor. This can result in errors in shown"
             " images, but has no effect on the result.");
  }

  // Edge-preserving spatial smoothing
  spatialFilter.set_option(RS2_OPTION_HOLES_FILL,
                           filterVariables.spatialHoleFillingMode);

  // Temporal filter reduces temporal noise by using knowledge from
  // previous frames
  temporalFilter = rs2::temporal_filter(filterVariables.temporalSmoothAlpha,
                                        filterVariables.temporalSmoothDelta,
                                        filterVariables.temporalPersistencyMode);

  // Allow autoexposure to settle
  try {
    for (auto i = 0; i < FRAMES_PER_SECOND; i++) {
      pipe.wait_for_frames(timeout);
    }
  } catch (...) {
    ROS_ERROR("Script timed out while waiting for frames (%d ms)."
              " Exiting script.", timeout);
    ros::shutdown();
    return false;
  }

  // Get depth frame intrinsics
  rs2_intrinsics intrinsics =
    profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().
    get_intrinsics();
  fx = intrinsics.fx; fy = intrinsics.fy;
  ppx = intrinsics.ppx; ppy = intrinsics.ppy;

  ROS_INFO("Realsense D435 camera initialized.");
  return true;
}

bool CameraData::processFrames()
{
  // Read camera data
  try {
    frames = pipe.wait_for_frames(timeout);
  } catch (rs2::error error) {
    deviceList = rs2_query_devices(context, &e);
    if (rs2_get_device_count(deviceList, &e) == 0) {
      ROS_ERROR("Camera has been disconnected. Exiting script.");
    } else {
      ROS_ERROR("Script timed out while waiting for frames (%d ms)."
                " Exiting script. %s", timeout, error.what());
    }
    ros::shutdown();
    return false;
  }

  // Check if data is valid
  if (frames == 0) {
    ROS_ERROR("Camera returned an invalid frame. Exiting script.");
    ros::shutdown();
    return false;
  }

  // Get regular color image and align data to depth
  rs2::video_frame colorNormal = frames.get_color_frame();

  // Align depth and color frames
  rs2::align alignToDepth(RS2_STREAM_DEPTH);
  frames = alignToDepth.process(frames);

  // Get data
  rs2::depth_frame depth = frames.get_depth_frame();
  rs2::depth_frame depthFrameFiltered = decimationFilter.process(depth);
  depthFrameFiltered = spatialFilter.process(depthFrameFiltered);
  depthFrameFiltered = temporalFilter.process(depthFrameFiltered);
  rs2::video_frame depthColor = colorMap.colorize(depthFrameFiltered);
  rs2::video_frame color = frames.get_color_frame();

  // Get decimated depth image size
  width = (int)depthFrameFiltered.get_width();
  height = (int)depthFrameFiltered.get_height();

  // Convert to Mat for OpenCV use
  // Color image aligned to depth
  depthAlignedColorImage = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
                                   CV_8UC3,
                                   (void *)color.get_data(),
                                   cv::Mat::AUTO_STEP);

  // Regular color image
  normalColorImage = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
                             CV_8UC3,
                             (void *)colorNormal.get_data(),
                             cv::Mat::AUTO_STEP);

  // Colorized depth image
  colorizedDepthImage = cv::Mat(cv::Size(width, height),
                                CV_8UC3,
                                (void *)depthColor.get_data(),
                                cv::Mat::AUTO_STEP);

  // Get depth data as a matrix
  cv::Mat depthDataTemp = cv::Mat(cv::Size(width, height), CV_16UC1,
                                  (ushort *)depthFrameFiltered.get_data());
  depthDataTemp.convertTo(depthData, CV_32F, depthScale);

  return true;
}

bool CameraData::loadImage(const std::string& filename)
{
  width = DEPTH_WIDTH;
  height = DEPTH_HEIGHT;
  cv::FileStorage file(filename + "_depth.xml", cv::FileStorage::READ);
  if (!file.isOpened()) {
    ROS_ERROR("File %s could not be opened.",
              (filename + "_depth.xml").c_str());
    return false;
  }

  file["depth"] >> depthData;
  depthData.convertTo(depthData, CV_32F);
  cv::medianBlur(depthData, depthData, 5);
  cv::GaussianBlur(depthData, depthData, cv::Size(5, 5), 0);

  fx = USUAL_FX; fy = USUAL_FY;
  ppx = USUAL_PPX; ppy = USUAL_PPY;

  try {
    normalColorImage = cv::imread(filename + "_color.png", cv::IMREAD_COLOR);
    depthAlignedColorImage = cv::imread(filename + "_color_depth.png",
                                        cv::IMREAD_COLOR);
  } catch (...) {
    ROS_ERROR("File %s or %s could not be opened.",
              (filename + "_color.png").c_str(),
              (filename + "_color_depth.png").c_str());
    return false;
  }
  return true;
}

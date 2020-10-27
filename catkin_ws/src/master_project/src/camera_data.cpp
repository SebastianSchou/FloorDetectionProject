#include "master_project/camera_data.hpp"
#include "ros/ros.h"

CameraData::CameraData(int width, int height, int fps)
{
  imageWidth = width;
  imageHeight = height;
  cameraFps = fps;
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
                       imageWidth,
                       imageHeight,
                       RS2_FORMAT_BGR8,
                       cameraFps);
  config.enable_stream(RS2_STREAM_DEPTH,
                       imageWidth,
                       imageHeight,
                       RS2_FORMAT_Z16,
                       cameraFps);
  config.enable_stream(RS2_STREAM_INFRARED,
                       imageWidth,
                       imageHeight,
                       RS2_FORMAT_Y8,
                       cameraFps);
  profile = pipe.start(config);

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
  if ((imageWidth % filterVariables.decimationScaleFactor != 0) ||
      (imageHeight % filterVariables.decimationScaleFactor != 0)) {
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
    for (auto i = 0; i < cameraFps; ++i) {
      pipe.wait_for_frames(timeout);
    }
  } catch (...) {
    ROS_ERROR("Script timed out while waiting for frames (%d ms)."
              " Exiting script.", timeout);
    ros::shutdown();
    return false;
  }

  // Get depth frame intrinsics
  intrinsics =
    profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().
    get_intrinsics();

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
  rs2::video_frame ir = frames.first(RS2_STREAM_INFRARED);

  // Get decimated depth image size
  width = (int)depthFrameFiltered.get_width();
  height = (int)depthFrameFiltered.get_height();

  // Convert to Mat for OpenCV use
  // Color image aligned to depth
  depthAlignedColorImage = cv::Mat(cv::Size(imageWidth, imageHeight),
                                   CV_8UC3,
                                   (void *)color.get_data(),
                                   cv::Mat::AUTO_STEP);

  // Regular color image
  normalColorImage = cv::Mat(cv::Size(imageWidth, imageHeight),
                             CV_8UC3,
                             (void *)colorNormal.get_data(),
                             cv::Mat::AUTO_STEP);

  // Colorized depth image
  colorizedDepthImage = cv::Mat(cv::Size(width, height),
                                CV_8UC3,
                                (void *)depthColor.get_data(),
                                cv::Mat::AUTO_STEP);

  // IR image
  irImage = cv::Mat(cv::Size(imageWidth, imageHeight),
                    CV_8UC1,
                    (void *)ir.get_data(),
                    cv::Mat::AUTO_STEP);;

  // Equalize IR frame
  cv::equalizeHist(irImage, irImage);
  cv::applyColorMap(irImage, irImage, cv::COLORMAP_JET);

  // Get depth data as an array
  depthArray = reinterpret_cast<const uint16_t *>(
    depthFrameFiltered.get_data());

  return true;
}
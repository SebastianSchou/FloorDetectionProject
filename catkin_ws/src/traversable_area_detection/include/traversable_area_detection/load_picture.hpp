#include <stdio.h>
#include <opencv4/opencv2/core/ocl.hpp>
#include "traversable_area_detection/camera_data.hpp"
#include <traversable_area_detection/LoadPictureSrv.h>

#define EXIT_SUCCESS 0
#define EXIT_ERROR -1
#define QUEUE_SIZE 10

static ros::ServiceServer srvLoadPicture;
static ros::Publisher     pubHoughPlaneTransform;
static bool shutdownRos = false;
static std::string path = "";

static void planePubCallback(const std::string& file, const int waitTime)
{
  // Create publisher
  traversable_area_detection::HoughPlaneTransform msg;
  msg.filename = file;

  // Load image
  CameraData  cameraData;
  std::string filename = path + file;
  if (!cameraData.loadImage(filename)) {
    msg.success = false;
    pubHoughPlaneTransform.publish(msg);
    return;
  }

  auto start = std::chrono::steady_clock::now();

  // Process planes
  HoughPlaneTransform houghPlaneTransform(cameraData);
  std::vector<Plane>  planes = houghPlaneTransform.planes;
  if (planes.empty()) {
    msg.computation_time = msUntilNow(start);
  } else {
    Plane nonPlanePoints = PlaneAnalysis::computePlanePoints(planes,
                                                             cameraData);
    PlaneAnalysis::computePlaneContour(planes, nonPlanePoints);
    msg.computation_time = msUntilNow(start);
    PlaneAnalysis::insertPlanePublisherInformation(msg, planes, true);

    if (waitTime == 1) {
      msg.success = true;
      pubHoughPlaneTransform.publish(msg);
      return;
    }

    // Draw
    cv::Mat onlyQuadtree = cameraData.depthAlignedColorImage.clone();
    cv::Mat cleanedPlanePoints = cameraData.depthAlignedColorImage.clone();
    DrawingFunctions::assignColorToPlanes(planes);
    DrawingFunctions::drawPlanesInQuadtree(onlyQuadtree,
                                           houghPlaneTransform.root,
                                           cameraData);
    DrawingFunctions::drawOnlyPlaneQuadtreeBorders(cleanedPlanePoints,
                                                   planes,
                                                   cameraData);
    DrawingFunctions::drawPlanes(cleanedPlanePoints, planes);
    DrawingFunctions::drawPlanesInQuadtree(cameraData.depthAlignedColorImage,
                                           houghPlaneTransform.root,
                                           cameraData);
    DrawingFunctions::drawPlanePoints(cameraData.depthAlignedColorImage,
                                      planes,
                                      cameraData);
    cv::Mat topView =
      DrawingFunctions::drawTopView(cameraData, planes, nonPlanePoints.topView);
    cv::Mat sideView;
    for (Plane& plane : planes) {
      if (plane.type == PLANE_TYPE_FLOOR) {
        sideView = DrawingFunctions::drawSideView(cameraData, plane);
        cv::imshow("Side view", sideView);
        break;
      }
    }

    // cv::Mat accumDrawing = DrawingFunctions::drawAccumulatorCellVotes(
    //  width, height, *houghPlaneTransform.accumulator);

    // Show data
    // cv::imshow("Accumulator", accumDrawing);
    cv::imshow("Top view", topView);
    cv::imshow("Realsense non plane points", nonPlanePoints.image2dPoints);
    cv::imshow("Realsense plane points cleaned", cleanedPlanePoints);
    cv::imshow("Realsense plane quadtree", onlyQuadtree);
    cv::imshow("Realsense plane points rough",
               cameraData.depthAlignedColorImage);
  }
  msg.success = true;
  pubHoughPlaneTransform.publish(msg);
  cv::imshow("Realsense color normal", cameraData.normalColorImage);
  cv::waitKey(waitTime);
  cv::destroyAllWindows();
}

static bool loadPictureCallback(
  traversable_area_detection::LoadPictureSrv::Request & req,
  traversable_area_detection::LoadPictureSrv::Response& rsp)
{
  if (req.filename.empty()) {
    shutdownRos = true;
    rsp.accepted = false;
    return true;
  }

  planePubCallback(req.filename, req.imageTime);

  rsp.accepted = true;
  return true;
}

int loadPicture(int argc, char **argv, const std::string& filePath)
{
  // Initialize ros
  path = filePath;
  ros::init(argc, argv, "load_picture");
  ros::NodeHandle nh;
  srvLoadPicture = nh.advertiseService("load_picture_srv", loadPictureCallback);
  pubHoughPlaneTransform =
    nh.advertise<traversable_area_detection::HoughPlaneTransform>(
      "hough_plane_transform",
      QUEUE_SIZE);
  ros::spin();
  ros::shutdown();
  return EXIT_SUCCESS;
}

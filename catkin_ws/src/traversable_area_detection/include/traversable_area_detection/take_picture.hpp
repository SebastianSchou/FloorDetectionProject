#include <stdio.h>
#include <opencv4/opencv2/core/ocl.hpp>
#include "traversable_area_detection/camera_data.hpp"


#define EXIT_SUCCESS 0
#define EXIT_ERROR -1

void saveDepthData(const std::string& filename,
                   cv::Mat depthData)
{
  cv::FileStorage file(filename, cv::FileStorage::WRITE);
  file << "depth" << depthData;
  file.release();
}

int takePicture(int argc, char **argv, const std::string& filepath)
{
  // Initialize ros
  ros::init(argc, argv, "take_picture");
  ros::NodeHandle nh;

  // Init Intel RealSense camera
  CameraData cameraData;
  if (!cameraData.initializeCamera()) {
    return EXIT_ERROR;
  }

  // Main loop
  char  key = ' ';
  while (key != 'q') {
    // Process frame
    if (!cameraData.processFrames()) {
      return EXIT_ERROR;
    }

    // Show image
    cv::imshow("Realsense color", cameraData.normalColorImage);
    key = cv::waitKey(1);
    if (key == 'p') {
      printf("Type in the name of the picture (without extension): ");
      // Type in file name
      std::string name;
      std::cin >> name;
      if (name == "p") {
        printf("Image discarded.\n");
        continue;
      }

      const std::string filenameDepthColor = filepath + name + "_color_depth.png";
      const std::string filenameColor = filepath + name + "_color.png";
      const std::string filenameDepth = filepath + name + "_depth.xml";
      std::vector<int> compression_params;
      compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(9);

      // Save images
      cv::imwrite(filenameColor, cameraData.normalColorImage, compression_params);
      cv::imwrite(filenameDepthColor, cameraData.depthAlignedColorImage, compression_params);

      // Save depth data
      saveDepthData(filenameDepth, cameraData.depthData);

      printf("Image saved as '%s'\n", name.c_str());
    }
  }

  // Shutdown
  cameraData.pipe.stop();
  ros::shutdown();
  return EXIT_SUCCESS;
}

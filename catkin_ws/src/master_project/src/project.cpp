#include<stdio.h>
// Intel Realsense library
#include <librealsense2/rs.hpp>
// OpenCV libraries
#include <opencv4/opencv2/core/ocl.hpp>
#include <opencv4/opencv2/opencv.hpp>
// Ros library
#include "ros/ros.h"

using namespace cv;

int main(int argc, char** argv)
{
	// Initialize ros
	ros::init(argc, argv, "master_project");
	ros::NodeHandle nh;

	// Test code. Should be removed
	printf("Hello! This is a test program.\n");
	std::cout << "OpenCV version : " << CV_VERSION << "\n";

	return 0;
}
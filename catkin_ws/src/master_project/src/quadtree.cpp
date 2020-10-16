#include "master_project/quadtree.hpp"
#include <thread>
#include <chrono>

#define MIN_SAMPLES 25

// Assuming normal distribution, 95% of the value is within the
// interval [-1.96, 1.96]
#define CONFIDENCE_INTERVAL_95_PERCENTAGE 1.96
#define MAX_PLANE_THICKNESS 0.36
#define X 0
#define Y 1
#define Z 2
#define MIN_EIGENVALUE_INDEX 2

Quadtree::Quadtree()
{
  root = NULL;
  children = NULL;
  mean = { 0, 0, 0 };
  covariance = cv::Mat::zeros(3, 3, CV_64F);
  normal = cv::Mat::zeros(3, 1, CV_64F);
  areaThickness = 0;
  minVariance = 0;
  samples = 0;
  isPlane = false;
  maxDistance = 0;
}

Quadtree::~Quadtree()
{
  if (children != NULL) {
    for (int i = 0; i < 4; i++) {
      children[i].~Quadtree();
    }
  }
  children = NULL;
}

void Quadtree::divideIntoQuadrants()
{
  // If there are too few samples in the image to analyze, ignore it
  samples = root->sat.satSamples.getArea(minBounds, maxBounds);
  if (samples < MIN_SAMPLES) {
    root->nonPlanes.push_back(this);
    return;
  }

  PCA();

  // Get the thickness of the data within the 95 percentage interval
  areaThickness = CONFIDENCE_INTERVAL_95_PERCENTAGE *
                  std::sqrt(minVariance);
  if (areaThickness < MAX_PLANE_THICKNESS) {
    isPlane = true;
    root->planes.push_back(this);
    return;
  }

  // Split image into four
  children = new Quadtree[4];
  cv::Point parentCenter(((maxBounds.x - minBounds.x) / 2) + minBounds.x,
                         ((maxBounds.y - minBounds.y) / 2) + minBounds.y);
  children[0].minBounds = cv::Point(minBounds.x, minBounds.y);
  children[0].maxBounds = cv::Point(parentCenter.x, parentCenter.y);

  children[1].minBounds = cv::Point(parentCenter.x, minBounds.y);
  children[1].maxBounds = cv::Point(maxBounds.x, parentCenter.y);

  children[2].minBounds = cv::Point(minBounds.x, parentCenter.y);
  children[2].maxBounds = cv::Point(parentCenter.x, maxBounds.y);

  children[3].minBounds = cv::Point(parentCenter.x, parentCenter.y);
  children[3].maxBounds = cv::Point(maxBounds.x, maxBounds.y);

  for (int i = 0; i < 4; i++) {
    children[i].root = root;
    children[i].level = level + 1;
    children[i].divideIntoQuadrants();
  }
}

void Quadtree::PCA()
{
  // Compute mean and covariance
  mean = computeMean();
  covariance = computeCovariance();

  // Get eigenvalues and eigenvectors. They are sorted such that eigenvalue 1
  // is largest and eigenvalue 3 is smallest. Value and vector correspond
  // w.r.t. index
  cv::Mat eigenvalues, eigenvectors;
  cv::eigen(covariance, eigenvalues, eigenvectors);

  // Get the smallest variance
  minVariance = eigenvalues.at<double>(MIN_EIGENVALUE_INDEX) /
                eigenvalues.at<double>(1);

  // Get normal vector, which is the eigenvector with the smallest eigenvalue
  normal.at<double>(0, 0) = eigenvectors.at<double>(MIN_EIGENVALUE_INDEX, 0);
  normal.at<double>(0, 1) = eigenvectors.at<double>(MIN_EIGENVALUE_INDEX, 1);
  normal.at<double>(0, 2) = eigenvectors.at<double>(MIN_EIGENVALUE_INDEX, 2);
}

std::vector<double> Quadtree::computeMean()
{
  SummedAreaTable *sat = &(root->sat);

  std::vector<double> m{0, 0, 0 };
  m[X] = sat->satX.getArea(minBounds, maxBounds) / samples;
  m[Y] = sat->satY.getArea(minBounds, maxBounds) / samples;
  m[Z] = sat->satZ.getArea(minBounds, maxBounds) / samples;
  return m;
}

cv::Mat Quadtree::computeCovariance()
{
  SummedAreaTable *sat = &(root->sat);

  cv::Mat cov = cv::Mat::zeros(3, 3, CV_64F);
  cov.at<double>(X, X) = (sat->satXX.getArea(minBounds, maxBounds) -
                          2 * mean[X] *
                          sat->satX.getArea(minBounds, maxBounds) +
                          samples * mean[X] * mean[X]) / (samples - 1);
  cov.at<double>(Y, Y) = (sat->satYY.getArea(minBounds, maxBounds) -
                          2 * mean[Y] *
                          sat->satY.getArea(minBounds, maxBounds) +
                          samples * mean[Y] * mean[Y]) / (samples - 1);
  cov.at<double>(Z, Z) = (sat->satZZ.getArea(minBounds, maxBounds) -
                          2 * mean[Z] *
                          sat->satZ.getArea(minBounds, maxBounds) +
                          samples * mean[Z] * mean[Z]) / (samples - 1);
  cov.at<double>(X, Y) = (sat->satXY.getArea(minBounds, maxBounds) -
                          mean[X] * sat->satY.getArea(minBounds, maxBounds) -
                          mean[Y] * sat->satX.getArea(minBounds, maxBounds) +
                          samples * mean[X] * mean[Y]) / (samples - 1);
  cov.at<double>(X, Z) = (sat->satXZ.getArea(minBounds, maxBounds) -
                          mean[X] * sat->satZ.getArea(minBounds, maxBounds) -
                          mean[Z] * sat->satX.getArea(minBounds, maxBounds) +
                          samples * mean[X] * mean[Z]) / (samples - 1);
  cov.at<double>(Y, Z) = (sat->satYZ.getArea(minBounds, maxBounds) -
                          mean[Y] * sat->satZ.getArea(minBounds, maxBounds) -
                          mean[Z] * sat->satY.getArea(minBounds, maxBounds) +
                          samples * mean[Y] * mean[Z]) / (samples - 1);
  cov.at<double>(Y, X) = cov.at<double>(X, Y);
  cov.at<double>(Z, X) = cov.at<double>(X, Z);
  cov.at<double>(Z, Y) = cov.at<double>(Y, Z);

  return cov;
}

void Quadtree::initializeRoot(const CameraData& cameraData)
{
  root = this;
  level = 0;
  minBounds = cv::Point(0, 0);
  maxBounds = cv::Point(cameraData.width - 1, cameraData.height - 1);

  // Make summed-area table with x, y, z coordinates and their products
  sat = SummedAreaTable(cameraData.height, cameraData.width);

  for (int r = 0; r < cameraData.height; r++) {
    for (int c = 0; c < cameraData.width; c++) {
      // Depth array index
      int index = c + r * (cameraData.width);

      // Get distance
      double z = cameraData.depthArray[index] * cameraData.depthScale;

      // Ignore points where distance is 0
      if (z > 0) {
        // Calculate x and y using z and the depth cameras intrinsics
        double x = (c - cameraData.intrinsics.ppx) * z /
                   cameraData.intrinsics.fx;
        double y = (r - cameraData.intrinsics.ppy) * z /
                   cameraData.intrinsics.fy;

        // Set values
        sat.satX.set(r, c, x);
        sat.satY.set(r, c, y);
        sat.satZ.set(r, c, z);
        sat.satXX.set(r, c, x * x);
        sat.satXY.set(r, c, x * y);
        sat.satXZ.set(r, c, x * z);
        sat.satYY.set(r, c, y * y);
        sat.satYZ.set(r, c, y * z);
        sat.satZZ.set(r, c, z * z);
        sat.satSamples.set(r, c, 1);

        // Save valid points
        std::vector<double> validCoordinate { x, y, z };
        validCoordinates.push_back(validCoordinate);
        std::vector<int> validPixel { r, c };
        validPixels.push_back(validPixel);
        maxDistance = std::max(maxDistance, std::sqrt(x * x + y * y + z * z));
      }

      // Calculate sum table for this point
      sat.satX.setSumValue(r, c);
      sat.satY.setSumValue(r, c);
      sat.satZ.setSumValue(r, c);
      sat.satXX.setSumValue(r, c);
      sat.satXY.setSumValue(r, c);
      sat.satXZ.setSumValue(r, c);
      sat.satYY.setSumValue(r, c);
      sat.satYZ.setSumValue(r, c);
      sat.satZZ.setSumValue(r, c);
      sat.satSamples.setSumValue(r, c);
    }
  }
}

#include "traversable_area_detection/quadtree.hpp"
#include "traversable_area_detection/helper_function.hpp"
#include <mutex>

// Assuming normal distribution, 95% of the value is within the
// interval [-1.96, 1.96]
#define CONFIDENCE_INTERVAL_95_PERCENTAGE 1.96
#define MIN_SAMPLE_DENSITY 0.9 // [%]
#define ROWS_REMOVE 12 / IMAGE_SCALE
#define COLS_REMOVE 12 / IMAGE_SCALE
#define MIN_SAMPLES 400 / square(IMAGE_SCALE)

Quadtree::Quadtree()
{
  root = NULL;
  children = NULL;
  areaThickness = 0.0;
  minVariance = 0.0;
  samples = 0;
  isPlane = false;
  visited = false;
  maxPlaneDistance = 0.0;
  maxPhiAngle = 0.0;
  rootRepresentativeness = 0.0;
  sampleDensity = 0.0;
  rho = 0.0; phi = 0.0; theta = 0.0;
  id = 0;
  idNo = 0;
  setMinSamplesInNode(MIN_SAMPLES);
  setMaxPlaneThickness(0.3);
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
  // Get amount of samples in node
  samples = root->sat.satSamples.getArea(minBounds, maxBounds);

  // Only perform PCA analysis if sample density is above its limit and
  // the sum of the gradient does not exceed its limit
  computeSampleDensity();
  if ((sampleDensity > MIN_SAMPLE_DENSITY)) {
    PCA();

    // Get the thickness of the data within the 95 percentage interval
    areaThickness = CONFIDENCE_INTERVAL_95_PERCENTAGE *
                    std::sqrt(minVariance);
    if (areaThickness < root->getMaxPlaneThickness()) {
      isPlane = true;
      root->planes.push_back(this);

      // If the distance from plane to camera center is negative,
      // inverse the normal, and make certain that arccos can be taken
      // to the z direction of the normal vector
      if (cv::normalize(mean).dot(normal) < 0) {
        normal *= -1.0;
      }
      if (normal[Z] > 1.0) {
        normal[Z] = 1.0;
      }
      theta = std::atan2(normal[Y], normal[X]);
      phi = std::acos(normal[Z]);
      rho = mean.dot(normal);

      // Find max plane distance and max phi angle
      root->maxPlaneDistance = std::max(root->maxPlaneDistance, rho);
      root->maxPhiAngle = std::max(root->maxPhiAngle, phi);
      return;
    }
  }

  // Do not split into quadrants if children are below minimum samples size
  if (samples / 4 < root->getMinSamplesInNode()) {
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
    children[i].id = root->idNo++;
    children[i].divideIntoQuadrants();
  }
}

void Quadtree::computeSampleDensity()
{
  int width = maxBounds.x - minBounds.x + 1;
  int height = maxBounds.y - minBounds.y + 1;

  sampleDensity = (float)samples / (float)(width * height);
}

void Quadtree::PCA()
{
  // Compute mean and covariance
  computeMean();
  computeCovariance();

  // Get eigenvalues and eigenvectors. They are sorted such that eigenvalue 1
  // is largest and eigenvalue 3 is smallest. Value and vector correspond
  // w.r.t. index
  cv::Vec3d   eigenvalues;
  cv::Matx33d eigenvectors;
  cv::eigen(covariance, eigenvalues, eigenvectors);

  // Get the smallest variance
  minVariance = eigenvalues[MIN_EIGENVALUE_INDEX] / eigenvalues[1];

  // Get normal vector, which is the eigenvector with the smallest eigenvalue
  normal[X] = eigenvectors(MIN_EIGENVALUE_INDEX, X);
  normal[Y] = eigenvectors(MIN_EIGENVALUE_INDEX, Y);
  normal[Z] = eigenvectors(MIN_EIGENVALUE_INDEX, Z);
}

void Quadtree::computeMean()
{
  SummedAreaTable *sat = &(root->sat);

  mean[X] = sat->satX.getArea(minBounds, maxBounds) / samples;
  mean[Y] = sat->satY.getArea(minBounds, maxBounds) / samples;
  mean[Z] = sat->satZ.getArea(minBounds, maxBounds) / samples;
}

void Quadtree::computeCovariance()
{
  SummedAreaTable *sat = &(root->sat);

  cv::Matx33d cov(3, 3, CV_64F);

  cov(X, X) = (sat->satXX.getArea(minBounds, maxBounds) -
               2 * mean[X] * sat->satX.getArea(minBounds, maxBounds) +
               samples * mean[X] * mean[X]) / (samples - 1);
  cov(Y, Y) = (sat->satYY.getArea(minBounds, maxBounds) -
               2 * mean[Y] * sat->satY.getArea(minBounds, maxBounds) +
               samples * mean[Y] * mean[Y]) / (samples - 1);
  cov(Z, Z) = (sat->satZZ.getArea(minBounds, maxBounds) -
               2 * mean[Z] * sat->satZ.getArea(minBounds, maxBounds) +
               samples * mean[Z] * mean[Z]) / (samples - 1);
  cov(X, Y) = (sat->satXY.getArea(minBounds, maxBounds) -
               mean[X] * sat->satY.getArea(minBounds, maxBounds) -
               mean[Y] * sat->satX.getArea(minBounds, maxBounds) +
               samples * mean[X] * mean[Y]) / (samples - 1);
  cov(X, Z) = (sat->satXZ.getArea(minBounds, maxBounds) -
               mean[X] * sat->satZ.getArea(minBounds, maxBounds) -
               mean[Z] * sat->satX.getArea(minBounds, maxBounds) +
               samples * mean[X] * mean[Z]) / (samples - 1);
  cov(Y, Z) = (sat->satYZ.getArea(minBounds, maxBounds) -
               mean[Y] * sat->satZ.getArea(minBounds, maxBounds) -
               mean[Z] * sat->satY.getArea(minBounds, maxBounds) +
               samples * mean[Y] * mean[Z]) / (samples - 1);
  cov(Y, X) = cov(X, Y);
  cov(Z, X) = cov(X, Z);
  cov(Z, Y) = cov(Y, Z);
  covariance = cov;
}

void Quadtree::initializeRoot(CameraData& cameraData)
{
  root = this;
  level = 0;
  id = 0;
  idNo++;
  minBounds = cv::Point(0, 0);
  maxBounds = cv::Point(cameraData.width - 1, cameraData.height - 1);
  cameraData.data3d =
    cv::Mat(cameraData.depthData.size(), CV_64FC3, cv::Scalar::all(0));

  // Remove borders of depth data, as it is unreliable
  int rows = cameraData.height, cols = cameraData.width;
  cameraData.depthData.rowRange(0, ROWS_REMOVE) = 0.0;
  cameraData.depthData.rowRange(rows - 1 - ROWS_REMOVE, rows - 1) = 0.0;
  cameraData.depthData.colRange(0, COLS_REMOVE) = 0.0;
  cameraData.depthData.colRange(cols - 1 - COLS_REMOVE, cols - 1) = 0.0;

  // Scale intrinsics depending on image decimation
  double scaleSize = cameraData.filterVariables.decimationScaleFactor;
  double ppx = cameraData.ppx / scaleSize;
  double ppy = cameraData.ppy / scaleSize;
  double fx = cameraData.fx / scaleSize;
  double fy = cameraData.fy / scaleSize;

  // Make summed-area table with x, y, z coordinates and their products
  sat = SummedAreaTable(cameraData.height, cameraData.width);

  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      // Get distance
      double z = cameraData.depthData.at<float>(r, c);

      // Ignore points where distance is 0 or above 10 m
      if ((z > MIN_DISTANCE) && (z < MAX_DISTANCE)) {
        // Calculate x and y using z and the depth cameras intrinsics
        double x = (c - ppx) * z / fx;
        double y = (r - ppy) * z / fy;
        cameraData.data3d.at<cv::Vec3d>(r, c) = cv::Vec3d(x, y, z);

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

void Quadtree::setMaxPlaneThickness(const double maxPlaneThickness)
{
  maxPlaneThickness_ = maxPlaneThickness;
}

const double Quadtree::getMaxPlaneThickness() const
{
  std::unique_lock<std::mutex> lck(std::mutex);
  return maxPlaneThickness_;
}

void Quadtree::setMinSamplesInNode(const int minSamplesInNode)
{
  minSamplesInNode_ = minSamplesInNode;
}

const int Quadtree::getMinSamplesInNode() const
{
  std::unique_lock<std::mutex> lck(std::mutex);
  return minSamplesInNode_;
}

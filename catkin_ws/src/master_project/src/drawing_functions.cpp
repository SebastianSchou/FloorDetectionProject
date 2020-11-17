#include "master_project/drawing_functions.hpp"

void DrawingFunctions::drawQuadtreeBorders(cv::Mat   & image,
                                           Quadtree  & node,
                                           CameraData& cameraData)
{
  if (node.isPlane) {
    cv::rectangle(image,
                  node.minBounds * cameraData.filterVariables.decimationScaleFactor,
                  node.maxBounds * cameraData.filterVariables.decimationScaleFactor,
                  cv::Scalar(0, 0, 255),
                  3);
  } else {
    cv::rectangle(image,
                  node.minBounds * cameraData.filterVariables.decimationScaleFactor,
                  node.maxBounds * cameraData.filterVariables.decimationScaleFactor,
                  cv::Scalar(0, 0, 255),
                  1);
  }
  if (node.children != NULL) {
    for (int i = 0; i < 4; i++) {
      drawQuadtreeBorders(image, node.children[i], cameraData);
    }
  }
}

void DrawingFunctions::drawQuadtreeBorders(cv::Mat& image, Quadtree& node)
{
  if (node.isPlane) {
    cv::rectangle(image,
                  node.minBounds,
                  node.maxBounds,
                  cv::Scalar(0, 0, 255),
                  3);
  } else {
    cv::rectangle(image,
                  node.minBounds,
                  node.maxBounds,
                  cv::Scalar(0, 0, 255),
                  1);
  }
  if (node.children != NULL) {
    for (int i = 0; i < 4; i++) {
      drawQuadtreeBorders(image, node.children[i]);
    }
  }
}

void DrawingFunctions::drawOnlyPlaneQuadtreeBorders(cv::Mat           & image,
                                                    std::vector<Plane>& planes,
                                                    CameraData        & cameraData)
{
  int scale = cameraData.filterVariables.decimationScaleFactor;

  for (const Plane& plane : planes) {
    for (const Quadtree *node : plane.nodes) {
      cv::rectangle(image, node->minBounds * scale, node->maxBounds * scale,
                    cv::Scalar(200, 200, 200), 1);
    }
  }
}

cv::Mat DrawingFunctions::drawAccumulatorCellVotes(const int          height,
                                                   const int          width,
                                                   const Accumulator& accumulator,
                                                   const int          minRho,
                                                   const int          maxRho)
{
  // Make image
  int offset = 20;

  cv::Mat accumDrawing = cv::Mat(cv::Size(width + 2 * offset,
                                          height + 2 * offset),
                                 CV_8UC3, cv::Scalar::all(255));

  // Find phi width from image width / no of phi coloumns
  int phiWidth = std::round((accumDrawing.cols - 2 * offset - 1) /
                            accumulator.data.size());

  // Loop over phi
  for (int phi = 0; phi < accumulator.data.size(); phi++) {
    // Find theta height from image height / no of theta rows
    double thetaHeight = (double)(accumDrawing.rows - 2 * offset - 1) /
                         (double)accumulator.data[phi].size();

    // Loop over theta
    for (int theta = 0; theta < accumulator.data[phi].size(); theta++) {
      // Get cell borders
      cv::Point from(offset + phiWidth *phi,
                     offset + std::round(thetaHeight * (double)theta));
      cv::Point to(offset + phiWidth * (phi + 1),
                   offset + std::round(thetaHeight * (double)(theta + 1)));

      // Determine cell color based on no of votes
      double votes;
      if (maxRho == 0) {
        votes = accumulator.data[phi][theta]->getVoteSum();
      } else {
        votes = accumulator.data[phi][theta]->getVoteSumSlice(minRho, maxRho);
      }
      votes = votes > 500 ? 500 : votes;
      double maxVotes = accumulator.maxVotes > 500 ? 500 : accumulator.maxVotes;
      int    votingColor = maxVotes == 0 ?
                           255 : 255 - std::round(votes / maxVotes * 255);

      // Draw cell color and borders
      cv::rectangle(accumDrawing, from, to,
                    cv::Scalar(255, votingColor, votingColor), cv::FILLED);
      cv::rectangle(accumDrawing, from, to, cv::Scalar(0, 0, 0), 1);
    }
  }

  // Draw graph arrows for theta and phi
  cv::arrowedLine(accumDrawing, cv::Point(10, height + offset + 10),
                  cv::Point(10, offset), cv::Scalar(0, 0, 255), 1, 8, 0, 0.01);
  cv::arrowedLine(accumDrawing, cv::Point(10, height + offset + 10),
                  cv::Point(width + 10, height + offset + 10),
                  cv::Scalar(0, 0, 255), 1, 8, 0, 0.01);
  cv::putText(accumDrawing, "theta", cv::Point(5, offset - 2),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(0));
  cv::putText(accumDrawing, "phi", cv::Point(width + 10, height + offset + 10),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(0));

  return accumDrawing;
}

void DrawingFunctions::drawPlanesInQuadtree(cv::Mat   & image,
                                            Quadtree  & node,
                                            CameraData& cameraData)
{
  int scale = cameraData.filterVariables.decimationScaleFactor;
  double r = node.color.at<uchar>(0), g = node.color.at<uchar>(1),
         b = node.color.at<uchar>(2);

  if (node.isPlane && ((r != 0) || (g != 0) || (b != 0))) {
    cv::Mat rect(image.rows, image.cols, CV_8UC3, cv::Scalar::all(0));


    cv::rectangle(rect,
                  node.minBounds * scale,
                  node.maxBounds * scale,
                  cv::Scalar(r, g, b),
                  cv::FILLED);
    cv::rectangle(rect,
                  node.minBounds * scale,
                  node.maxBounds * scale,
                  cv::Scalar::all(125),
                  1);
    cv::addWeighted(image, 1.0, rect, 0.8, 0.0, image);
  }
  if (node.children != NULL) {
    for (int i = 0; i < 4; i++) {
      drawPlanesInQuadtree(image, node.children[i], cameraData);
    }
  }
}

void DrawingFunctions::assignColorToPlane(Plane& plane, int r, int g, int b)
{
  cv::Mat color(cv::Size(3, 1), CV_8U, cv::Scalar(0));

  // Saturate the color values between 0 and 255
  r = std::max(0, std::min(255, r));
  g = std::max(0, std::min(255, g));
  b = std::max(0, std::min(255, b));

  // Assign to each node in plane
  color.at<uchar>(0) = r;
  color.at<uchar>(1) = g;
  color.at<uchar>(2) = b;
  plane.color = color;
  for (Quadtree *node : plane.nodes) {
    node->color = color;
  }
}

void DrawingFunctions::assignColorToPlanes(std::vector<Plane>& planes)
{
  for (unsigned int i = 0; i < planes.size(); i++) {
    int colorValue = (int)(255 / (int)(i / 6 + 1));
    cv::Mat color(cv::Size(1, 3), CV_8U, cv::Scalar::all(0));
    if ((planes[i].color.at<uchar>(0) != 0) ||
        (planes[i].color.at<uchar>(1) != 0) ||
        (planes[i].color.at<uchar>(2) != 0)) {
      continue;
    }
    switch (i % 6) {
      case 0:
        assignColorToPlane(planes[i], colorValue, 0, 0);
        break;
      case 1:
        assignColorToPlane(planes[i], 0, colorValue, 0);
        break;
      case 2:
        assignColorToPlane(planes[i], 0, 0, colorValue);
        break;
      case 3:
        assignColorToPlane(planes[i], 0, colorValue, colorValue);
        break;
      case 4:
        assignColorToPlane(planes[i], colorValue, 0, colorValue);
        break;
      case 5:
        assignColorToPlane(planes[i], colorValue, colorValue, colorValue);
        break;
    }
  }
}

void DrawingFunctions::drawPlanePoints(cv::Mat          & image,
                                       std::vector<Plane> planes,
                                       CameraData       & cameraData)
{
  int scale = cameraData.filterVariables.decimationScaleFactor;

  cv::Mat rect(image.rows, image.cols, CV_8UC3, cv::Scalar::all(0));
  for (const Plane& plane : planes) {
    uchar r = plane.color.at<uchar>(0),
                    g = plane.color.at<uchar>(1),
                    b = plane.color.at<uchar>(2);
    for (const cv::Vec2i point : plane.points2d) {
      int col = point[0] * scale, row = point[1] * scale;
      cv::Point p1(row, col), p2(row + scale * 2, col + scale * 2);
      cv::rectangle(rect, p1, p2, cv::Scalar(r, g, b), cv::FILLED);
    }
  }
  cv::addWeighted(image, 1.0, rect, 0.8, 0.0, image);
}

void DrawingFunctions::drawPlanes(cv::Mat                 & image,
                                  const std::vector<Plane>& planes)
{
  cv::Mat im;
  if (planes.size() == 0) {
    return;
  }
  im = cv::Mat::zeros(planes[0].image2dPoints.size(), CV_8UC3);
  for (const Plane& plane : planes) {
    uchar r = plane.color.at<uchar>(0),
                    g = plane.color.at<uchar>(1),
                    b = plane.color.at<uchar>(2);
    im.setTo(cv::Scalar(r, g, b), plane.image2dPoints);
  }
  cv::resize(im, im, im.size() * 4);
  cv::addWeighted(image, 1.0, im, 0.8, 0.0, image);
}

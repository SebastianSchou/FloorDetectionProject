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

  if (node.isPlane && (node.color != cv::Scalar::all(0))) {
    cv::Mat rect(image.rows, image.cols, CV_8UC3, cv::Scalar::all(0));


    cv::rectangle(rect,
                  node.minBounds * scale,
                  node.maxBounds * scale,
                  node.color,
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
  plane.color = cv::Scalar(r, g, b);
  for (Quadtree *node : plane.nodes) {
    node->color = plane.color;
  }
}

void DrawingFunctions::assignColorToPlanes(std::vector<Plane>& planes)
{
  for (unsigned int i = 0; i < planes.size(); i++) {
    int colorValue = (int)(255 / (int)(i / 6 + 1));
    if (planes[i].color != cv::Scalar::all(0)) {
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

  cv::Mat rect(image.size(), CV_8UC3, cv::Scalar::all(0));
  for (const Plane& plane : planes) {
    for (const cv::Vec2i point : plane.points2d) {
      int col = point[0] * scale, row = point[1] * scale;
      cv::Point p1(row, col), p2(row + scale * 2, col + scale * 2);
      cv::rectangle(rect, p1, p2, plane.color, cv::FILLED);
    }
  }
  cv::addWeighted(image, 1.0, rect, 0.8, 0.0, image);
}

void DrawingFunctions::drawPlanes(cv::Mat                 & image,
                                  const std::vector<Plane>& planes)
{
  if (planes.size() == 0) {
    return;
  }
  cv::Mat im(planes[0].image2dPoints.size(), CV_8UC3, cv::Scalar::all(0));
  for (const Plane& plane : planes) {
    im.setTo(plane.color, plane.image2dPoints);
  }
  cv::resize(im, im, im.size() * 4);
  cv::addWeighted(image, 1.0, im, 0.8, 0.0, image);
}

cv::Mat DrawingFunctions::drawTopView(const CameraData        & cameraData,
                                      const std::vector<Plane>& planes,
                                      const cv::Mat           & nonPlanePoints)
{
  cv::Mat im(cv::Size(TOP_VIEW_WIDTH, TOP_VIEW_HEIGHT), CV_8UC3,
             cv::Scalar::all(0));

  for (const Plane& plane : planes) {
    if (plane.type == PLANE_TYPE_CEILING) {
      continue;
    }
    if (((plane.type == PLANE_TYPE_FLOOR) ||
         (plane.type == PLANE_TYPE_OTHER)) &&
        (plane.traversableAreas.size() != 0)) {
      cv::drawContours(im, plane.traversableAreas, -1, plane.color, cv::FILLED);
      if (plane.restrictedAreas.size() != 0) {
        cv::drawContours(im, plane.restrictedAreas, -1, cv::Scalar::all(0),
                         cv::FILLED);
      }
      if (plane.type == PLANE_TYPE_FLOOR) {
        for (const auto& heightArea : plane.heightLimitedAreas) {
          for (size_t i = 0; i < heightArea.second.size(); i++) {
            cv::drawContours(im, heightArea.second, i, cv::Scalar::all(125),
                             cv::FILLED);
            auto m = cv::moments(heightArea.second[i]);
            cv::Point center(int(m.m10 / m.m00), int(m.m01 / m.m00));
            center.y += 3;
            center.x -= 5;
            int h = heightArea.first * 100;
            cv::putText(im, std::to_string(h), center,
                        cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar::all(0));
          }
        }
      }
    } else {
      im.setTo(plane.color, plane.topView);
    }
  }
  im.setTo(cv::Scalar::all(255), nonPlanePoints);
  return im;
}

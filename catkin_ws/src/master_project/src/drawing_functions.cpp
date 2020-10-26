#include "master_project/drawing_functions.hpp"

void DrawingFunctions::drawQuadtreeBorders(cv::Mat   & image,
                                           Quadtree  & node,
                                           CameraData& cameraData)
{
  if (node.isPlane) {
    cv::rectangle(image,
                  node.minBounds * cameraData.filterVariables.decimationScaleFactor,
                  node.maxBounds * cameraData.filterVariables.decimationScaleFactor,
                  (255, 255, 0),
                  3);
  } else {
    cv::rectangle(image,
                  node.minBounds * cameraData.filterVariables.decimationScaleFactor,
                  node.maxBounds * cameraData.filterVariables.decimationScaleFactor,
                  (255, 0, 0),
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
    cv::rectangle(image, node.minBounds, node.maxBounds, (255, 255, 0), 3);
  } else {
    cv::rectangle(image, node.minBounds, node.maxBounds, (255, 0, 0), 1);
  }
  if (node.children != NULL) {
    for (int i = 0; i < 4; i++) {
      drawQuadtreeBorders(image, node.children[i]);
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

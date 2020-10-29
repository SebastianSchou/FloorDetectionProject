#ifndef KERNEL_HPP
#define KERNEL_HPP

#include "master_project/helper_function.hpp"
#include "master_project/quadtree.hpp"

#define X 0
#define Y 1
#define Z 2
#define MIN_EIGENVALUE_INDEX 2
#define NONZERO 0.00001

class Kernel {
public:

  Quadtree *node;

  double rho, theta, phi, gaussianPdfConstant, votingLimit, thetaIndex;
  int phiIndex, rhoIndex, votes;
  bool visited;

  cv::Mat covarianceSpherical, covarianceSphericalInversed;

  void computeKernelParameters()
  {
    cv::Mat jacobian = cv::Mat::zeros(cv::Size(3, 3), CV_64F);

    cv::Mat n = normalizeVector(node->normal);

    // Jacobian Matrix calculation
    double  epsilon = NONZERO;
    cv::Mat p = n * rho;
    double  pX = p.at<double>(X), pY = p.at<double>(Y), pZ = p.at<double>(Z);
    double  w = square(pX) + square(pY);
    double  p2 = w + square(pZ);
    double  sqrtW = std::sqrt(w);

    // Derivatives of pX*μx + pY*μy + pZ*μz
    jacobian.at<double>(X, X) = n.at<double>(X);
    jacobian.at<double>(X, Y) = n.at<double>(Y);
    jacobian.at<double>(X, Z) = n.at<double>(Z);

    // Derivatives of arrcos(pZ)
    jacobian.at<double>(Y, X) = (sqrtW < epsilon) ? (pX * pZ) / epsilon :
                                (pX * pZ) / (sqrtW * p2);
    jacobian.at<double>(Y, Y) = (sqrtW < epsilon) ? (pY * pZ) / epsilon :
                                (pY * pZ) / (sqrtW * p2);
    jacobian.at<double>(Y, Z) = (p2 < epsilon) ? -sqrtW / epsilon :
                                -sqrtW / p2;

    // Derivatives of atan2(pY, pX)
    jacobian.at<double>(Z, X) = (w < epsilon) ? -pY / epsilon : -pY / w;
    jacobian.at<double>(Z, Y) = (w < epsilon) ? pX / epsilon : pX / w;
    jacobian.at<double>(Z, Z) = 0.0;

    // Covariance matrix for the gaussian kernel
    covarianceSpherical = jacobian * node->covariance * jacobian.t();

    // Cluster representativeness
    covarianceSpherical.at<double>(X, X) += NONZERO;
    covarianceSphericalInversed = covarianceSpherical.inv();
    gaussianPdfConstant = SQRTOFCUBIC2PI *
                          std::sqrt(std::abs(cv::determinant(
                                               covarianceSpherical)));
    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(covarianceSpherical, eigenvalues, eigenvectors);

    // Weights for area importance (wa) and sample density (wd)
    double wa = 0.75;
    double wd = 1 - wa;
    double nodeSize = (node->maxBounds.x - node->minBounds.x) *
                      (node->maxBounds.y - node->minBounds.y);
    double rootSize = (node->root->maxBounds.x - node->root->minBounds.x) *
                      (node->root->maxBounds.y - node->root->minBounds.y);

    // Compute how much the node represents the root node
    node->rootRepresentativeness = nodeSize * wa / rootSize +
                                   (double)node->samples /
                                   (double)node->root->samples * wd;

    // Compute the voting limit for this kernel
    double radius = 2.0 *
                    std::sqrt(eigenvalues.at<double>(MIN_EIGENVALUE_INDEX));
    votingLimit =
      gaussianPdf(eigenvectors.at<double>(X, MIN_EIGENVALUE_INDEX) * radius,
                  eigenvectors.at<double>(Y, MIN_EIGENVALUE_INDEX) * radius,
                  eigenvectors.at<double>(Z, MIN_EIGENVALUE_INDEX) * radius);
  }

  double gaussianPdf(const double rho, const double phi,
                     const double theta)
  {
    double temp[3] = { rho, phi, theta };

    cv::Mat displacement(cv::Size(1, 3), CV_64F, (void *)temp,
                         cv::Mat::AUTO_STEP);
    cv::Mat d = displacement.t() * covarianceSphericalInversed * displacement;
    return std::exp(-0.5 * d.at<double>(0)) / gaussianPdfConstant;
  }
};

#endif // ifndef KERNEL_HPP

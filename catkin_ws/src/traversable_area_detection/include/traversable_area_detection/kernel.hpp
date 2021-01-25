#ifndef KERNEL_HPP
#define KERNEL_HPP

#include "traversable_area_detection/helper_function.hpp"
#include "traversable_area_detection/quadtree.hpp"

class Kernel {
public:

  Quadtree *node;

  double rho, theta, phi, gaussianPdfConstant, votingLimit, thetaIndex;
  int phiIndex, rhoIndex, votes;
  bool visited;

  cv::Matx33d covarianceSpherical, covarianceSphericalInversed;

  void computeKernelParameters()
  {
    cv::Matx33d jacobian;
    cv::Vec3d n = cv::normalize(node->normal);

    // Jacobian Matrix calculation
    double  epsilon = NONZERO;
    cv::Vec3d p = n * rho;
    double  w = square(p[X]) + square(p[Y]);
    double  p2 = w + square(p[Z]);
    double  sqrtW = std::sqrt(w);

    // Derivatives of p[X]*μx + p[Y]*μy + p[Z]*μz
    jacobian(X, X) = n[X];
    jacobian(X, Y) = n[Y];
    jacobian(X, Z) = n[Z];

    // Derivatives of arrcos(p[Z])
    jacobian(Y, X) = (sqrtW < epsilon) ?
                     (p[X] * p[Z]) / epsilon : (p[X] * p[Z]) / (sqrtW * p2);
    jacobian(Y, Y) = (sqrtW < epsilon) ?
                     (p[Y] * p[Z]) / epsilon : (p[Y] * p[Z]) / (sqrtW * p2);
    jacobian(Y, Z) = (p2 < epsilon) ? -sqrtW / epsilon : -sqrtW / p2;

    // Derivatives of atan2(p[Y], p[X])
    jacobian(Z, X) = (w < epsilon) ? -p[Y] / epsilon : -p[Y] / w;
    jacobian(Z, Y) = (w < epsilon) ? p[X] / epsilon : p[X] / w;
    jacobian(Z, Z) = 0.0;

    // Covariance matrix for the gaussian kernel
    covarianceSpherical = jacobian * node->covariance * jacobian.t();

    // Cluster representativeness
    covarianceSpherical(X, X) += NONZERO;
    covarianceSphericalInversed = covarianceSpherical.inv();
    gaussianPdfConstant = SQRTOFCUBIC2PI *
                          std::sqrt(std::abs(cv::determinant(
                                               covarianceSpherical)));
    cv::Vec3d eigenvalues;
    cv::Matx33d eigenvectors;
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
    double radius = 2.0 * std::sqrt(eigenvalues[MIN_EIGENVALUE_INDEX]);
    votingLimit = gaussianPdf(eigenvectors(X, MIN_EIGENVALUE_INDEX) * radius,
                              eigenvectors(Y, MIN_EIGENVALUE_INDEX) * radius,
                              eigenvectors(Z, MIN_EIGENVALUE_INDEX) * radius);
  }

  double gaussianPdf(const double rho, const double phi,
                     const double theta)
  {
    cv::Vec3d displacement(rho, phi, theta);
    cv::Scalar d = displacement.t() * covarianceSphericalInversed * displacement;
    return std::exp(-0.5 * d[0]) / gaussianPdfConstant;
  }
};

#endif // ifndef KERNEL_HPP

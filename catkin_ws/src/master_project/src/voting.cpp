#include "master_project/voting.hpp"
#include <ctime>
#include <limits>

void voting(Quadtree           & root,
            Accumulator        & accumulator,
            std::vector<Bin>   & usedBins,
            std::vector<Kernel>& usedKernels)
{
  int i = 0;

  for (Quadtree *node : root.planes) {
    computeKernel(*node, accumulator, usedKernels);

    // Voting is done in two steps: First in phi, theta dimension (based on
    // plane angle), and then in rho direction (based on distance to plane).
    // That way, planes are first found which differs in direction,
    // and then in position.
    gaussianVoting3d(usedKernels[i++], accumulator, usedBins);
  }
}

// Calculates gaussian kernel parameters
void computeKernel(Quadtree           & node,
                   Accumulator        & accum,
                   std::vector<Kernel>& usedKernels)
{
  // Init kernel
  Kernel kernel;
  kernel.node = &node;

  // Make certain that rho, the distance from plane to camera center,
  // is positive, and that arccos can be taken to the z direction of
  // the normal vector
  if (normalizeVector(node.mean).dot(node.normal) < 0) {
    node.normal *= -1.0;
  }
  if (node.normal.at<double>(2) > 1.0) {
    node.normal.at<double>(2) = 1.0;
  }

  // Set phi, theta, and rho values. Phi and theta are the rotation of the
  // plane in x and y directions
  kernel.phi = std::acos(node.normal.at<double>(2));
  kernel.theta =
    std::atan2(node.normal.at<double>(1), node.normal.at<double>(0));
  kernel.rho = node.mean.dot(node.normal);

  // Get indeces for the accumulator and make certain that they are
  // correct values
  accum.processIndexLimits(kernel.thetaIndex, kernel.phiIndex, kernel.rhoIndex);
  accum.getIndex(kernel.theta, kernel.phi, kernel.rho, kernel.thetaIndex,
                 kernel.phiIndex, kernel.rhoIndex);
  kernel.thetaIndex = accum.fixTheta(kernel.thetaIndex, kernel.phiIndex);

  // Process kernel
  kernel.computeKernelParameters();
  usedKernels.push_back(kernel);
}

// Voting in 3 dimensions (theta, phi and rho)
void gaussianVoting3d(Kernel          & kernel,
                      Accumulator     & accum,
                      std::vector<Bin>& usedBins)
{
  accum.at(kernel.thetaIndex, kernel.phiIndex, kernel.rhoIndex).top = true;

  // Vote in positive phi direction
  gaussianVoting2d(accum, kernel, usedBins, kernel.thetaIndex, kernel.phiIndex,
                   kernel.rhoIndex, 0, +1);

  int phiIndex = kernel.phiIndex - 1;
  double thetaIndex = kernel.thetaIndex;
  accum.processPhi(thetaIndex, phiIndex);

  // Vote in negative phi direction
  gaussianVoting2d(accum, kernel, usedBins, thetaIndex, phiIndex,
                   kernel.rhoIndex, -accum.phiDelta, -1);
}

// Voting in 2 dimensions (theta, phi)
void gaussianVoting2d(Accumulator     & accum,
                      Kernel          & kernel,
                      std::vector<Bin>& usedBins,
                      const double      thetaIndexStart,
                      int               phiIndexStart,
                      int               rhoIndexStart,
                      const double      phiStart,
                      int               phiIndexIncrement)
{
  double phiIncrement = (double)phiIndexIncrement * accum.phiDelta;
  int    phiIndex, rhoIndex = rhoIndexStart;
  bool   voted, endVoting = false;
  double thetaIndex, theta, phi, thetaIndexInit;

  // Voting in the PHI array in the direction "phiIndexIncrement"
  for (phiIndex = phiIndexStart, phi = phiStart; !endVoting;
       phiIndex += phiIndexIncrement, phi += phiIncrement) {
    thetaIndex = thetaIndexStart;
    if (accum.processPhi(thetaIndex, phiIndex)) {
      phiIndexIncrement *= -1;
    }
    thetaIndexInit = accum.fixTheta(thetaIndex, phiIndex);

    double thetaIncrement = accum.deltaTheta(phiIndexStart);
    double thetaIndexIncrement = accum.deltaThetaIndex(phiIndex);
    double currentThetaIndex;
    voted = false;
    endVoting = true;

    // Voting in the THETA array in the direction "positive"
    currentThetaIndex = thetaIndexInit;
    theta = 0.0;
    while (true) {
      accum.processTheta(currentThetaIndex);
      accum.initialize(currentThetaIndex, phiIndex);
      voted = gaussianVoting1d(accum, kernel, usedBins, currentThetaIndex,
                               phiIndex, rhoIndex, theta, phi);
      if (voted) {
        endVoting = false;
      } else {
        break;
      }
      currentThetaIndex += thetaIndexIncrement;
      theta += thetaIncrement;
    }

    // Voting in the THETA array in the direction "negative"
    currentThetaIndex = thetaIndexInit - thetaIndexIncrement;
    theta = -thetaIncrement;
    while (true) {
      accum.processTheta(currentThetaIndex);
      accum.initialize(currentThetaIndex, phiIndex);
      voted = gaussianVoting1d(accum, kernel, usedBins, currentThetaIndex,
                               phiIndex, rhoIndex, theta, phi);
      if (voted) {
        endVoting = false;
      } else {
        break;
      }
      currentThetaIndex -= thetaIndexIncrement;
      theta -= thetaIncrement;
    }
  }
}

// Voting in 1 dimensions (rho)
bool gaussianVoting1d(Accumulator     & accum,
                             Kernel          & kernel,
                             std::vector<Bin>& usedBins,
                             const double      thetaIndex,
                             const int         phiIndex,
                             const int         rhoIndexStart,
                             const double      theta,
                             const double      phi)
{
  int rhoIndex, p, rhoIndexIncrement;
  double rho, t, rhoIncrement, gauss, votes;
  bool   voted = false;

  p = phiIndex;
  t = thetaIndex;
  rhoIncrement = accum.rhoDelta;

  // Voting in the rho cell array in positive direction
  rhoIndexIncrement = 1;
  rhoIndex = rhoIndexStart;
  rho = 0.0;
  while (true) {
    if (rhoIndex < 0) {
      rhoIndexIncrement *= -1;
    }
    if (accum.processRho(t, p, rhoIndex) == false) {
      break;
    }
    gauss = kernel.gaussianPdf(rho, phi, theta);

    if (gauss < kernel.votingLimit) {
      break;
    }
    votes = gauss;
    if (votes >= 0.0) {
      voted = vote(usedBins, accum.at(t, p, rhoIndex), kernel, votes, t, p,
                   rhoIndex, accum);
    } else {
      break;
    }
    rhoIndex += rhoIndexIncrement;
    rho += rhoIncrement;
  }

  // Voting in the rho cell array in negative direction
  rhoIndexIncrement = -1;
  rhoIndex = rhoIndexStart - 1;
  rho = -rhoIncrement;
  while (true) {
    if (rhoIndex < 0) {
      rhoIndexIncrement *= -1;
    }
    if (accum.processRho(t, p, rhoIndex) == false) {
      break;
    }

    gauss = kernel.gaussianPdf(rho, phi, theta);
    if (gauss < kernel.votingLimit) {
      break;
    }

    if ((votes = gauss) >= 0.0) {
      voted = vote(usedBins, accum.at(t, p, rhoIndex), kernel, votes, t, p,
                   rhoIndex, accum);
    } else {
      break;
    }
    rhoIndex += rhoIndexIncrement;
    rho -= rhoIncrement;
  }
  return voted;
}

bool vote(std::vector<Bin>& usedBins,
                 AccumulatorCell & cell,
                 Kernel          & kernel,
                 double            votes,
                 double            t,
                 int               p,
                 int               r,
                 Accumulator     & accum)
{
  // Cluster representativeness
  votes = votes * kernel.node->rootRepresentativeness;
  accum.maxVotes = std::max(accum.maxVotes, votes);

  // Test if the cell has already been voted by this kernel
  if (cell.hasBeenVotedForBefore(kernel.node)) {
    // Test if the previous vote was smaller than the current
    if (cell.lastVoteCount < votes) {
      // Remove
      cell.bin += -cell.lastVoteCount + votes;
      cell.lastVoteCount = votes;
    } else {
      return false;
    }

    // First node vote
  } else {
    // Store how many votes will be cast
    cell.lastVoteCount = votes;

    // Increment votes
    cell.bin += votes;

    // Add reference of the node that votes for this cell
    cell.addNodeReference(kernel.node);

    if (!cell.voted) {
      usedBins.push_back(Bin(t, p, r));
      cell.voted = true;
    }

    // Track last node that votes for this cell
    cell.setAsVotedFor(kernel.node);
  }
  return true;
}

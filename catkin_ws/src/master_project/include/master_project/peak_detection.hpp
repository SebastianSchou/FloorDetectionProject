#include "master_project/accumulator.hpp"
#include "master_project/kernel.hpp"
#include "master_project/plane.hpp"
#include "master_project/plane_analysis.hpp"

inline void peakDetection(std::vector<Plane> & planes,
                          Accumulator        & accum,
                          std::vector<Kernel>& usedKernels,
                          std::vector<Bin>   & usedBins)
{
  // Smooth out votes with a lowpass filter
  for (Bin& bin : usedBins) {
    AccumulatorCell& cell = accum.at(bin.thetaIndex, bin.phiIndex,
                                     bin.rhoIndex);
    bin.votes = accum.convolutionValue(bin.thetaIndex, bin.phiIndex,
                                       bin.rhoIndex);

    cell.votes = bin.votes;
    cell.thetaIndex = bin.thetaIndex;
    cell.rhoIndex = bin.rhoIndex;
    cell.phiIndex = bin.phiIndex;
  }

  // Apply hillclimbing peak detection
  std::set<AccumulatorCell *> peakCells;
  std::set<AccumulatorCell *> neighbors;
  std::vector<Quadtree *>     nodes;
  for (Kernel kernel : usedKernels) {
    AccumulatorCell *nextCell = &accum.at(kernel.thetaIndex,
                                          kernel.phiIndex,
                                          kernel.rhoIndex);
    double nextCellVotes = nextCell->votes;
    double currentCellVotes = 0.0;

    // Check if any neighboring cell has more votes. If so, continue
    // to check neighboring cells until a peak has been detected
    do {
      AccumulatorCell *currentCell = nextCell;
      currentCellVotes = currentCell->votes;
      nextCellVotes = 0.0;

      // Get all connected neighbors and check their votes. If it is
      // higher than the current vote count, check that cells
      // neighbors next
      neighbors = accum.getNeighborCells(currentCell->thetaIndex,
                                         currentCell->phiIndex,
                                         currentCell->rhoIndex,
                                         27);
      std::move(currentCell->votedNotes.begin(),
                currentCell->votedNotes.end(),
                std::inserter(nodes, nodes.end()));
      for (AccumulatorCell *neighbor : neighbors) {
        if (neighbor->votes > nextCellVotes) {
          nextCellVotes = neighbor->votes;
          nextCell = neighbor;
        }
      }
    } while (nextCellVotes > currentCellVotes);
    std::move(nodes.begin(), nodes.end(),
              std::inserter(nextCell->votedNotes, nextCell->votedNotes.end()));
    peakCells.insert(nextCell);
    nodes.clear();
    neighbors.clear();
  }

  // Compute plane for every peak cell
  int planeId = 0;
  for (AccumulatorCell *cell : peakCells) {
    Plane plane;

    accum.getValues(plane.theta, plane.phi, plane.rho, cell->thetaIndex,
                    cell->phiIndex, cell->rhoIndex);

    // Get normal vector and plane position
    plane.normal.at<double>(0) = std::sin(plane.phi) * std::cos(plane.theta);
    plane.normal.at<double>(1) = std::sin(plane.phi) * std::sin(plane.theta);
    plane.normal.at<double>(2) = std::cos(plane.phi);
    plane.position = plane.normal * plane.rho;

    // Calculate phi and theta for absolute values of the normal vector
    plane.thetaAbs = (std::abs(plane.theta) < MAX_ANGLE_DIFFERENCE) ?
                     std::atan2(std::abs(plane.normal.at<double>(1)),
                                std::abs(plane.normal.at<double>(0))) :
                     plane.theta;
    plane.phiAbs = (std::abs(plane.phi) < MAX_ANGLE_DIFFERENCE) ?
                   std::acos(std::abs(plane.normal.at<double>(2))) :
                   plane.phi;

    std::move(cell->votedNotes.begin(), cell->votedNotes.end(),
              std::inserter(plane.nodes, plane.nodes.end()));
    plane.rootRepresentativeness = 0;

    plane.votes = cell->votes;
    plane.thetaIndex = cell->thetaIndex;
    plane.phiIndex = cell->phiIndex;
    plane.rhoIndex = cell->rhoIndex;
    bool skip = false;
    for (Plane& prevPlane : planes) {
      if (PlaneAnalysis::isSimilar(prevPlane, plane)) {
        PlaneAnalysis::transferNodes(prevPlane, plane);
        skip = true;
      }
    }
    if (skip) {
      continue;
    }
    plane.id = planeId++;
    plane.computePlaneParameters(planes);
    if (plane.samples > 500) {
      planes.push_back(plane);
    }
  }
}

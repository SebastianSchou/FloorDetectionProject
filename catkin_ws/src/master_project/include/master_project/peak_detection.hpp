#include "master_project/accumulator.hpp"
#include "master_project/bin.hpp"
#include "master_project/kernel.hpp"
#include "master_project/plane.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

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
  for (AccumulatorCell *cell : peakCells) {
    Plane plane;

    accum.getValues(plane.theta, plane.phi, plane.rho, cell->thetaIndex,
                    cell->phiIndex, cell->rhoIndex);

    std::move(cell->votedNotes.begin(), cell->votedNotes.end(),
              std::inserter(plane.nodes, plane.nodes.end()));
    plane.rootRepresentativeness = 0;

    plane.votes = cell->votes;
    plane.thetaIndex = cell->thetaIndex;
    plane.phiIndex = cell->phiIndex;
    plane.rhoIndex = cell->rhoIndex;
    plane.computePlaneParameters();
    if (plane.samples > 100) {
      planes.push_back(plane);
    }
  }
}

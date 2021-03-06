#include "traversable_area_detection/accumulator.hpp"
#include "traversable_area_detection/kernel.hpp"
#include "traversable_area_detection/plane.hpp"
#include "traversable_area_detection/plane_analysis.hpp"

inline void peakDetection(std::vector<Plane>       & planes,
                          Accumulator              & accum,
                          std::vector<Bin>         & usedBins,
                          const std::vector<Kernel>& usedKernels)
{
  // Smooth out votes with a lowpass filter
  for (Bin& bin : usedBins) {
    AccumulatorCell& cell = accum.at(bin.thetaIndex, bin.phiIndex,
                                     bin.rhoIndex);
    bin.votes = accum.convolutionValue(bin.thetaIndex, bin.phiIndex,
                                       bin.rhoIndex);
    accum.maxVotes = std::max(accum.maxVotes, bin.votes);

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
      std::move(currentCell->votedNodes.begin(),
                currentCell->votedNodes.end(),
                std::inserter(nodes, nodes.end()));
      for (AccumulatorCell *neighbor : neighbors) {
        if (neighbor->votes > nextCellVotes) {
          nextCellVotes = neighbor->votes;
          nextCell = neighbor;
        }
      }
    } while (nextCellVotes > currentCellVotes);
    std::move(nodes.begin(), nodes.end(),
              std::inserter(nextCell->votedNodes, nextCell->votedNodes.end()));
    peakCells.insert(nextCell);
    nodes.clear();
    neighbors.clear();
  }

  // Compute plane for every peak cell
  int planeId = 0;
  for (AccumulatorCell *cell : peakCells) {
    Plane plane;

    // Get spherical coordinate values
    accum.getValues(plane.theta, plane.phi, plane.rho, cell->thetaIndex,
                    cell->phiIndex, cell->rhoIndex);

    // Move all nodes from the cell to the plane
    std::move(cell->votedNodes.begin(), cell->votedNodes.end(),
              std::inserter(plane.nodes, plane.nodes.end()));

    // Set plane values from cell
    plane.votes = cell->votes;
    plane.thetaIndex = cell->thetaIndex;
    plane.phiIndex = cell->phiIndex;
    plane.rhoIndex = cell->rhoIndex;
    plane.id = planeId++;
    plane.rootRepresentativeness = 0;
    if (plane.computePlaneParameters(planes)) {
      planes.push_back(plane);
    }
  }
}

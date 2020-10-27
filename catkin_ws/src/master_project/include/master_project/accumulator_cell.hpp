#ifndef ACCUMULATOR_CELL_HPP
#define ACCUMULATOR_CELL_HPP

#include "master_project/quadtree.hpp"

// A cell (θ, φ, ρ) of the accumulator which stores the votes
class AccumulatorCell {
public:

  AccumulatorCell()
  {
    lastVoteCount = 0.0;
    lastVotedNote = NULL;
    visited = false;
    top = false;
    voted = false;
    peak = false;
    votes = 0.0;
  }

  ~AccumulatorCell()
  {
  }

  bool hasBeenVotedForBefore(Quadtree *node)
  {
    return lastVotedNote == node;
  }

  void setAsVotedFor(Quadtree *node)
  {
    lastVotedNote = node;
    this->votes++;
  }

  void addNodeReference(Quadtree *node)
  {
    votedNotes.insert(node);
  }

  std::set<Quadtree *> votedNotes;

  Quadtree *lastVotedNote;
  bool peak, visited, voted, top;
  double lastVoteCount, votes;
  double thetaIndex;
  int phiIndex, rhoIndex;
};

// The accumulator consists of cells (phi, theta), whick contains an array of
// rho cells
class AccumulatorCellArray {
public:

  AccumulatorCellArray(int size)
  {
    cells = new AccumulatorCell[size]();
    size_ = size;
  }

  ~AccumulatorCellArray()
  {
    delete[] cells;
  }

  double getVoteSum()
  {
    double sum = 0.0;

    if (this == NULL) {
      return 0.0;
    }
    for (int i = 0; i < size_; i++) {
      sum += cells[i].lastVoteCount;
    }
    return sum;
  }

  double getVoteSumSlice(const int min, const int max)
  {
    double sum = 0.0;

    if (this == NULL) {
      return 0.0;
    }
    for (int i = min; i < size_ && i <= max; i++) {
      sum += cells[i].lastVoteCount;
    }
    return sum;
  }

  AccumulatorCell *cells;

private:

  AccumulatorCellArray(const AccumulatorCellArray&);
  int size_;
};

#endif // ACCUMULATOR_CELL_HPP

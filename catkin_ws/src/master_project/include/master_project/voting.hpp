#ifndef VOTING_HPP
#define VOTING_HPP

#include "master_project/accumulator.hpp"
#include "master_project/kernel.hpp"
#include "master_project/bin.hpp"


void voting(Quadtree           & root,
            Accumulator        & accum,
            std::vector<Bin>   & usedBins,
            std::vector<Kernel>& usedKernels);
void computeKernel(Quadtree           & node,
                   Accumulator        & accum,
                   std::vector<Kernel>& usedKernels);
void gaussianVoting3d(Kernel          & kernel,
                      Accumulator     & accum,
                      std::vector<Bin>& usedBins);
void gaussianVoting2d(Accumulator     & accum,
                      Kernel          & kernel,
                      std::vector<Bin>& usedBins,
                      const double      thetaIndexStart,
                      int               phiIndexStart,
                      int               rhoIndexStart,
                      const double      phiStart,
                      int               phiIndexIncrement);
bool gaussianVoting1d(Accumulator     & accum,
                      Kernel          & kernel,
                      std::vector<Bin>& usedBins,
                      const double      thetaIndex,
                      const int         phiIndex,
                      const int         rhoIndexStart,
                      const double      theta,
                      const double      phi);
bool vote(std::vector<Bin>& usedBins,
          AccumulatorCell & cell,
          Kernel          & kernel,
          double            votes,
          double            t,
          int               p,
          int               r,
          Accumulator     & accum);

#endif // VOTING_HPP

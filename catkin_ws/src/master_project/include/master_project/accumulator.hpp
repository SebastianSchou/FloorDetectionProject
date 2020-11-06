#ifndef ACCUMULATOR_HPP
#define ACCUMULATOR_HPP

#include "master_project/accumulator_cell.hpp"
#include "master_project/bin.hpp"
#include "master_project/helper_function.hpp"

class Accumulator {
public:

  Accumulator(const double maxDistance, const double maxPhiAngle,
              const float rhoDeltaValue, const float phiDeltaValue);
  ~Accumulator();

  std::set<Quadtree *>        convolutionNodes(const double thetaIndex,
                                               const short  phiIndex,
                                               const short  rhoIndex);
  std::set<AccumulatorCell *> getNeighborCells(const double thetaIndex,
                                               const short  phiIndex,
                                               const short  rhoIndex,
                                               const int    neighborhoodSize);
  AccumulatorCell& at(const double theta,
                      const short  phi,
                      const short  rho);

  void   setVisited(const double thetaIndex,
                    const short  phiIndex,
                    const short  rhoIndex);
  void   initialize(double thetaIndex, int phiIndex);
  double convolutionValue(const double thetaIndex,
                          const int    phiIndex,
                          const int    rhoIndex);
  void   setVisited(std::vector<AccumulatorCell *>& neighbors);
  double deltaTheta(const double& phiIndex);
  double deltaThetaIndex(const double& phiIndex);
  void   processTheta(double& thetaIndex);
  bool   processPhi(double& thetaIndex, int& phiIndex);
  bool   processRho(double& thetaIndex,
                    int   & phiIndex,
                    int   & rhoIndex);
  bool   processIndexLimits(double& thetaIndex,
                            int   & phiIndex,
                            int   & rhoIndex);

  int  getThetaIndex(const double& theta, const int& phiIndex);
  void getIndex(const double& theta, const double& phi,
                const double& rho, double& thetaIndex,
                int& phiIndex, int& rhoIndex);
  void getValues(double& theta, double& phi, double& rho,
                 const double& thetaIndex, const int& phiIndex,
                 const int& rhoIndex);
  void sphericalToCartesianCoordinates(cv::Mat     & normal,
                                       const double& theta,
                                       const double& phi,
                                       const double& rho);
  double fixTheta(double theta, const int phi);

  std::vector<std::vector<AccumulatorCellArray *> > data;
  int neighborSize;
  double thetaMax;
  double phiMax;
  double maxVotes;

  // Accumulator dimensions
  short phiLength;
  short rhoLength;
  short thetaLength;

  // Discretization step size
  double rhoDelta;
  double phiDelta;
};

#endif // ACCUMULATOR_HPP

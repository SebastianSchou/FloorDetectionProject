#ifndef BIN_HPP
#define BIN_HPP

// A structure that stores the coordinates of an accumulator cell
struct Bin {
  Bin()
  {
  }

  Bin(double theta, short phi, short rho) : thetaIndex(theta), phiIndex(phi),
    rhoIndex(rho)
  {
  }

  double thetaIndex;
  short  phiIndex;
  short  rhoIndex;
  double votes;

  bool operator<(const Bin bin) const
  {
    return votes > bin.votes;
  }
};

#endif // BIN_HPP

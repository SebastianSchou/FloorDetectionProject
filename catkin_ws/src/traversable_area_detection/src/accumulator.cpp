#include "traversable_area_detection/accumulator.hpp"

static const short neighborOffsetTheta[26] =
{ 0,   0,  0,  0, +1, -1,                         // directly linked
  +1, -1, +1, -1, +1, -1, -1, +1, 0, 0, 0, 0,     // semi directly linked
  +1, +1, +1, -1, +1, -1, -1, -1 };               // diagonally linked
static const short neighborOffsetPhi[26] =
{ +1, -1,  0,  0,  0,  0,                         // directly linked
  +1, -1, -1, +1,  0,  0,  0, 0, +1, -1, -1, +1,  // semi directly linked
  +1, +1, -1, +1, -1, -1, +1, -1 };               // diagonally linked
static const short neighborOffsetRho[26] =
{ 0,   0, +1, -1,  0,  0,                         // directly linked
  0,   0,  0,  0, +1, -1, +1, -1, +1, -1, +1, -1, // semi directly linked
  +1, -1, +1, +1, -1, +1, -1, -1 };               // diagonally linked

Accumulator::Accumulator(const double maxDistance,
                         const double maxPhiAngle,
                         const float  rhoDeltaValue,
                         const float  phiDeltaValue)
{
  neighborSize = 27;
  thetaMax = CV_PI * 2.0;
  phiMax = CV_PI;
  maxVotes = 0.0;
  rhoDelta = rhoDeltaValue;
  phiDelta = phiDeltaValue;

  // Set rho length to be dependent on plane distance and the desired
  // distance difference planes should be voted in, and the length of
  // phi to be dependent on max plane angle and the desired angle
  // difference planes should be voted in. Multiply max distance and angle
  // with 1.1 to avoid having the extremes exceed the length
  rhoLength = std::ceil(maxDistance * 1.1 / rhoDelta);
  phiLength = std::ceil(maxPhiAngle * 1.1 / phiDelta);
  data.resize(phiLength + 1);
  for (int p = 0; p <= phiLength; p++) {
    double phi = (double)p / (double)phiLength * phiMax;
    int    length = std::max(1, (int)round((double)phiLength * 2.0 * sin(phi)));
    data[p].resize(length, NULL);
  }
}

Accumulator::~Accumulator()
{
  for (int i = 0; i < data.size(); i++) {
    for (int j = 0; j < data[i].size(); j++) {
      if (data[i][j] != NULL) {
        delete data[i][j];
      }
    }
  }
}

void Accumulator::initialize(double thetaIndex, int phiIndex)
{
  int t = getThetaIndex(thetaIndex, phiIndex);

  if (data[phiIndex][t] == NULL) {
    data[phiIndex][t] = new AccumulatorCellArray(rhoLength);
  }
}

double Accumulator::convolutionValue(const double thetaIndex,
                                     const int    phiIndex,
                                     const int    rhoIndex)
{
  double accumulatorValue = 0.0;

  std::set<AccumulatorCell *> neighbors = getNeighborCells(thetaIndex, phiIndex,
                                                           rhoIndex, 6);
  AccumulatorCell *center = &at(thetaIndex, phiIndex, rhoIndex);
  accumulatorValue = center->votes * 0.2002;
  for (AccumulatorCell *n : neighbors) {
    accumulatorValue += n->votes * 0.1333;
  }
  return accumulatorValue;
}

double Accumulator::deltaTheta(const double& phiIndex)
{
  return thetaMax / (double)(data[phiIndex].size());
}

double Accumulator::deltaThetaIndex(const double& phiIndex)
{
  return 1.0 / (double)(data[phiIndex].size());
}

void Accumulator::processTheta(double& thetaIndex)
{
  thetaIndex = (thetaIndex + 1.0) - (int)(thetaIndex + 1.0);
}

bool Accumulator::processPhi(double& thetaIndex, int& phiIndex)
{
  processTheta(thetaIndex);
  if (phiIndex < 0) {
    phiIndex = std::abs(phiIndex);
    thetaIndex = (thetaIndex + .5) - (int)(thetaIndex + .5);
    return true;
  } else if (phiIndex > phiLength) {
    phiIndex = phiLength + phiLength - phiIndex;
    thetaIndex = (thetaIndex + .5) - (int)(thetaIndex + .5);
    return true;
  }
  return false;
}

bool Accumulator::processRho(double& thetaIndex, int& phiIndex, int& rhoIndex)
{
  if (rhoIndex < 0) {
    rhoIndex = std::abs(rhoIndex);
    phiIndex = phiLength - phiIndex;
    thetaIndex = (thetaIndex + .5) - (int)(thetaIndex + .5);
    return true;
  } else if (rhoIndex >= rhoLength) {
    return false;
  }
  return true;
}

std::set<AccumulatorCell *> Accumulator::getNeighborCells(
  const double thetaIndex, const short phiIndex, const short rhoIndex,
  const int neighborhoodSize)
{
  std::set<AccumulatorCell *> result;

  int p, r;
  double t, theta;

  if ((phiIndex == 0) || (phiIndex == phiLength)) {
    theta = 0.5;
  } else {
    theta = thetaIndex;
  }

  for (short i = 0; i < neighborhoodSize; ++i) {
    t = theta;
    p = phiIndex + neighborOffsetPhi[i];
    r = rhoIndex + neighborOffsetRho[i];
    processPhi(t, p);
    t = fixTheta(t, p);
    t += (deltaThetaIndex(p) * (double)((neighborOffsetTheta[i])));

    if (!processIndexLimits(t, p, r)) {
      continue;
    }

    AccumulatorCell *cell = &at(t, p, r);
    result.insert(cell);
  }
  return result;
}

bool Accumulator::processIndexLimits(double& thetaIndex,
                                     int   & phiIndex,
                                     int   & rhoIndex)
{
  processPhi(thetaIndex, phiIndex);
  return processRho(thetaIndex, phiIndex, rhoIndex);
}

AccumulatorCell& Accumulator::at(const double theta,
                                 const short  phi,
                                 const short  rho)
{
  int t = getThetaIndex(theta, phi);

  if (data[phi][t] == NULL) {
    data[phi][t] = new AccumulatorCellArray(rhoLength);
  }
  return data[phi][t]->cells[rho];
}

int Accumulator::getThetaIndex(const double& theta, const int& phiIndex)
{
  return (int)(round(theta * (double)(data[phiIndex].size()))) %
         data[phiIndex].size();
}

void Accumulator::getIndex(const double& theta, const double& phi,
                           const double& rho, double& thetaIndex,
                           int& phiIndex, int& rhoIndex)
{
  thetaIndex = (theta / PI2) + 0.5;
  phiIndex = round(phi / phiDelta);
  rhoIndex = round(rho / rhoDelta);
}

void Accumulator::getValues(double& theta, double& phi, double& rho,
                            const double& thetaIndex, const int& phiIndex,
                            const int& rhoIndex)
{
  theta = (thetaIndex - 0.5) * PI2;
  phi = (double)(phiIndex) * phiDelta;
  rho = (double)(rhoIndex) * rhoDelta;
}

double Accumulator::fixTheta(double theta, const int phi)
{
  double thetaSize = data[phi].size();
  int    t = round((theta) * thetaSize);

  return (t == 1) ? (0.0) : t / thetaSize;
}

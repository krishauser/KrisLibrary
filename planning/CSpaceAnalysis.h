#ifndef ROBOTICS_CSPACE_ANALYSIS_H
#define ROBOTICS_CSPACE_ANALYSIS_H

#include "CSpace.h"
#include <vector>

class CSpaceAnalysis
{
 public:
  void AnalyzeNeighborhood(CSpace* cspace,const Config& x,Real radius,int numSamples);

  struct SetCharacteristics
  {
    //source data
    Config center;
    Real sampleRadius;
    std::vector<Config> points;

    //model of space: vol(Ball(r)) = c*r^d 
    //where c is an arbitrary constant, d is intrinsicDims
    Real intrinsicDims;

    //local models of space (localIntrinsicDims[i] is intrinsic dimension
    //at ball of radius localScales[i]
    std::vector<Real> localIntrinsicDims;
    std::vector<Real> localScales;
  };

  SetCharacteristics space,feasible,infeasible;
  Real feasibleVolume;
  std::vector<Real> feasibleEigenvector;
};

#endif

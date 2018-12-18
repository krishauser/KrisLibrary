#ifndef ROBOTICS_PERTURBATION_CSPACE_H
#define ROBOTICS_PERTURBATION_CSPACE_H

#include "CSpace.h"
#include <vector>

class PerturbationCSpace : public CSpace 
{
public:
  PerturbationCSpace(CSpace* baseSpace,const std::vector<Vector>& perturbations);
  virtual ~PerturbationCSpace();
  virtual void Sample(Config& q);
  virtual Config Perturb(const Config& q,const Vector& perturbation);
  virtual bool IsFeasible(const Config& q,int obstacle);
  virtual EdgePlannerPtr LocalPlanner(const Config& a,const Config& b,int obstacle);
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b,int obstacle);
  virtual int NumObstacles();
  virtual std::string ObstacleName(int obstacle);
  virtual Real ObstacleDistance(const Config& a,int obstacle);
  virtual void Properties(PropertyMap& map) const;

  CSpace* baseSpace;
  std::vector<Vector> perturbations;
};

#endif

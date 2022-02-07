#ifndef ROBOTICS_PERTURBATION_CSPACE_H
#define ROBOTICS_PERTURBATION_CSPACE_H

#include "CSpace.h"
#include <vector>

/** @brief A C-space that ensures that a configuration + perturbation[i]
 * is feasible, for all i=1,...,N.
 */
class PerturbationCSpace : public CSpace 
{
public:
  PerturbationCSpace(CSpace* baseSpace,const std::vector<Vector>& perturbations);
  virtual ~PerturbationCSpace();
  virtual void Sample(Config& q) override;
  virtual Config Perturb(const Config& q,const Vector& perturbation);
  virtual bool IsFeasible(const Config& q,int obstacle) override;
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b,int obstacle) override;
  virtual int NumConstraints() override;
  virtual std::string ConstraintName(int obstacle) override;
  virtual Real ObstacleDistance(const Config& a) override;
  virtual void Properties(PropertyMap& map) override;

  virtual EdgePlannerPtr LocalPlanner(const Config& a,const Config& b,int obstacle);
  virtual Real ObstacleDistance(const Config& a,int obstacle);

  CSpace* baseSpace;
  std::vector<Vector> perturbations;
};

#endif

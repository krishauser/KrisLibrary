#include <KrisLibrary/Logger.h>
#include "PerturbationCSpace.h"
#include "Interpolator.h"
using namespace std;

PerturbationCSpace::PerturbationCSpace(CSpace* _baseSpace,const vector<Vector>& _perturbations)
  :baseSpace(_baseSpace),perturbations(_perturbations)
{}

PerturbationCSpace::~PerturbationCSpace()
{}

void PerturbationCSpace::Sample(Config& q)
{
  baseSpace->Sample(q);
}

Config PerturbationCSpace::Perturb(const Config& q,const Vector& perturbation)
{
  return q + perturbation;
}

bool PerturbationCSpace::IsFeasible(const Config& q,int obstacle)
{
  return baseSpace->IsFeasible(Perturb(q,perturbations[obstacle]));
}

EdgePlannerPtr PerturbationCSpace::LocalPlanner(const Config& a,const Config& b,int obstacle)
{
  return baseSpace->LocalPlanner(Perturb(a,perturbations[obstacle]),Perturb(b,perturbations[obstacle]));
}

EdgePlannerPtr PerturbationCSpace::PathChecker(const Config& a,const Config& b,int obstacle)
{
  return baseSpace->PathChecker(a+perturbations[obstacle],b+perturbations[obstacle]);
}

int PerturbationCSpace::NumObstacles() { return (int)perturbations.size(); }

std::string PerturbationCSpace::ObstacleName(int obstacle)
{
  char buf[64];
  snprintf(buf,64,"perturbation[%d]",obstacle);
  return buf;
}

Real PerturbationCSpace::ObstacleDistance(const Config& a,int obstacle)
{
  return baseSpace->ObstacleDistance(Perturb(a,perturbations[obstacle]));
}

void PerturbationCSpace::Properties(PropertyMap& map) const
{
  baseSpace->Properties(map);
}

#ifndef PLANNING_KINODYNAMIC_OPTIMIZER_H
#define PLANNING_KINODYNAMIC_OPTIMIZER_H

#include "KinodynamicMotionPlanner.h"

class KinodynamicLocalOptimizer : public KinodynamicPlannerBase
{
public:
  KinodynamicLocalOptimizer(KinodynamicSpace* s,std::shared_ptr<ObjectiveFunctionalBase> objective);
  virtual ~KinodynamicLocalOptimizer();
  void Init(const KinodynamicMilestonePath& path,CSet* goalSet);
  virtual void Init(const State& xinit,CSet* goalSet);
  virtual bool Plan(int maxIters);
  virtual bool Done() const;
  virtual bool GetPath(KinodynamicMilestonePath& path);
  void ComputeCosts();
  bool DoShortcut();
  bool DoRandomDescent(Real perturbationSize);
  bool DoGradientDescent();
  bool DoDDP();

  enum { Shortcut, RandomDescent, GradientDescent, DDP };

  std::shared_ptr<ObjectiveFunctionalBase> objective;
  KinodynamicMilestonePath bestPath;
  Real bestPathCost;
  std::vector<double> cumulativeCosts;

  //stats for adaptive method
  std::vector<bool> methodAvailable;
  std::vector<int> methodCounts;
  std::vector<Real> methodRewards;
  std::vector<Real> methodCosts;
  //temporary
  KinodynamicMilestonePath tempPath;
};

#endif
#include "AnalyticIK.h"
using namespace std;

AnalyticIKSolution::AnalyticIKSolution()
  :infinite(false)
{}

void AnalyticIKSolution::DeleteMinimalDistance()
{
  for(size_t i=0;i<solutions.size();i++) {
    if(minimalDistance[i]) {
      solutions.erase(solutions.begin()+i);
      minimalDistance.erase(minimalDistance.begin()+i);
      i--;
    }
  }
}

void AnalyticIKMap::GetIndices(const RobotKinematics3D& robot,
		const IKGoal& goal,
		vector<int>& indices) const
{
  assert(goal.destLink == -1);
  int numdofs = IKGoal::NumDims(goal.posConstraint)+IKGoal::NumDims(goal.rotConstraint);
  assert(numdofs == 3 || numdofs == 6);

  indices.resize(1);
  indices[0] = goal.link;
  int base = goal.link;
  for(int i=0;i+1<numdofs;i++) {
    base = robot.parents[base];
    indices.push_back(base);
  }
  reverse(indices.begin(),indices.end());
}

bool AnalyticIKMap::Solve(RobotKinematics3D& robot,
	   const IKGoal& goal,
	   AnalyticIKSolution& solution) const 
{
  assert(goal.destLink == -1);
  int numdofs = IKGoal::NumDims(goal.posConstraint)+IKGoal::NumDims(goal.rotConstraint);
  assert(numdofs == 3 || numdofs == 6);
  //find the appropriate solver
  int link = goal.link;
  int base = link;
  for(int i=0;i+1<numdofs;i++) {
    base = robot.parents[base];
  }
  std::pair<int,int> ind(base,goal.link);
  auto solver = solvers.find(ind);
  if(solver == solvers.end())
    return false;

  RigidTransform Trel,Tworld,TbaseInv;
  if(numdofs == 6)
    goal.GetFixedGoalTransform(Tworld);
  else {
    Tworld.R.setIdentity();
    Tworld.t = goal.endPosition;
  }
  TbaseInv.setInverse(robot.links[robot.parents[base]].T_World*robot.links[base].T0_Parent);
  Trel = TbaseInv * Tworld;
  return solver->second->Solve(base,goal.link,Trel,solution);
}

bool AnalyticIKMap::Solve(int base,int ee,
			    const RigidTransform& T,
			    AnalyticIKSolution& solution) const
{
  std::pair<int,int> ind(base,ee);
  auto solver = solvers.find(ind);
  if(solver == solvers.end())
    return false;
  return solver->second->Solve(base,ee,T,solution);
}

void AnalyticIKMap::UpdateRobot(int base,int ee,
				  const AnalyticIKSolution& solution,
				  int index,RobotKinematics3D& robot) const
{
  assert(index >= 0 && index < (int)solution.solutions.size());
  int n=solution.solutions[index].n;
  while(n >= 0) {
    robot.q[ee] = solution.solutions[index][n-1];
    n--;
    ee = robot.parents[ee];
  }
  assert(ee == robot.parents[base]);
}


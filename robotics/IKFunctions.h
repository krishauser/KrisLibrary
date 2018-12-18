#ifndef ROBOTICS_IK_FUNCTIONS_H
#define ROBOTICS_IK_FUNCTIONS_H

#include "RobotKinematics3D.h"
#include "IK.h"
#include <KrisLibrary/math/vectorfunction.h>
#include <KrisLibrary/optimization/Newton.h>
#include <KrisLibrary/utils/ArrayMapping.h>
#include <KrisLibrary/utils/DirtyData.h>

/** @file IKFunctions.h
 * @ingroup Kinematics
 * @brief Helper classes and functions for solving iterative inverse
 * kinematics problems.
 */

/** @addtogroup Kinematics */
/*@{*/

/** @brief A vector field function class defining C(q)=0 for IK.
 *
 * Composes several individual IK functions together using their
 * cartesian product.  Add a function to C using the UseX() methods.
 *
 * Optionally, this can use only a subset of the dimensions of q 
 * which might accelerate the computation of Jacobians in the IK iteration.
 * This is done using the activeDofs member, which provides a mapping from
 * q' (the reduced configuration) to q (the full configuration).
 *
 * Warning: the UseIK functions store POINTERS to IKGoal's, so you must maintain
 * the structures throughout the life of this struct.
 */
struct RobotIKFunction : public CompositeVectorFieldFunction
{
  RobotIKFunction(RobotKinematics3D&);
  ~RobotIKFunction();

  void UseIK(const std::vector<IKGoal>& g);
  void UseIK(const IKGoal& g);
  void UseCOM(const Vector2& comGoal);
  void Clear();

  void SetState(const Vector& x) const;
  void GetState(Vector& x) const;
  virtual void PreEval(const Vector& x);

  RobotKinematics3D& robot;

  ArrayMapping activeDofs;
  //vector<Real> scaleDofs; TODO? enable scaling of dofs
};

/** @brief A Newton-Raphson robot IK solver.
 * 
 * Joint limits are optionally included if the UseJointLimits() functions
 * are called.  Typically, the first version is called.
 *
 * A bias configuration can be explicitly set using the UseBiasConfiguration()
 * function.  The solver will seek toward this configuration during the solve.
 *
 * Optionally, a Real parameter may specify a threshold t such that
 * a revolute joint's limits are included only if the angular range is less
 * than t. Specifying a value less than 2pi is useful to avoid local minima
 * for joints with wide ranges, because it allows the joint angle to pass from
 * -pi to pi, and vice versa.
 */
struct RobotIKSolver
{
  RobotIKSolver(RobotIKFunction& function);
  void UseJointLimits(Real revJointThreshold = Inf);
  void UseJointLimits(const Vector& qmin,const Vector& qmax);
  void UseBiasConfiguration(const Vector& qdesired);
  void ClearJointLimits();
  void RobotToState();
  void StateToRobot();
  bool Solve(Real tolerance,int& iters);
  void PrintStats();

  Optimization::NewtonRoot solver;
  RobotIKFunction& function;
  RobotKinematics3D& robot;
};

/// Computes the ArrayMapping that only includes ancestor links of the 
/// goals in ik.  This reduces the size of a RobotIKFunction's Jacobian matrix 
void GetDefaultIKDofs(const RobotKinematics3D& robot,const std::vector<IKGoal>& ik,ArrayMapping& m);

/// Computes the ArrayMapping that contains a nonredundant subset of links
/// terminating in the link of ikGoal.
void GetPassiveChainDOFs(const RobotKinematics3D& robot,const IKGoal& ikGoal,ArrayMapping& passiveDofs);

/// The same as above, but the goal is specified only by the link index
/// and the number of operational space dof's.
void GetPassiveChainDOFs(const RobotKinematics3D& robot,int link,int numTerms,ArrayMapping& passiveDofs);

/// Helper function that tries to solve the given ik function using the
/// Newton-Raphson solver.  Returns true if successful
bool SolveIK(RobotIKFunction& f,
	     Real tolerance,int& iters,int verbose=1);

/// Same as above, constructing the ik function from an IK problem
bool SolveIK(RobotKinematics3D&,const std::vector<IKGoal>& problem,
	     Real tolerance,int& iters,int verbose=1);

/// Same as above, constructing the ik function from an IK problem and a list of
/// active dofs
bool SolveIK(RobotKinematics3D&,const std::vector<IKGoal>& problem,
	     const std::vector<int>& activeDofs,
	     Real tolerance,int& iters,int verbose=1);

/// Same as above, constructing the ik function from a single IK goal and only
/// using the required number of degrees of freedom to solve for the given
/// goal.
bool SolvePassiveChainIK(RobotKinematics3D&,const IKGoal& goal,
			 Real tolerance,int& iters,int verbose=0);

/// For the goal link's transformation T, computes the error of the IK goal g
void EvalIKError(const IKGoal& g,const RigidTransform& T,Real* poserr,Real* orierr);
void EvalIKError(const IKGoal& g,const RigidTransform& T,Vector& err);
Real RobotIKError(const RobotKinematics3D& robot,const IKGoal& goal);

/// For the goal link's transformaiton T, with rotational velocity dw and
/// linear velocity dv, computes the derivative of g's error
void EvalIKGoalDeriv(const IKGoal& g,const RigidTransform& T,const Vector3& dw,const Vector3& dv,Vector& derr);

/// Constructs a new goal c from the intersection of two goals a and b which are
/// required to have the same link and destLink.  False is returned if a and b are
/// incompatible.
bool IntersectGoals(const IKGoal& a,const IKGoal& b,IKGoal& c,Real tolerance=1e-5);
/// Adds a new constraint to a goalSet while merging it in intelligently
/// to minimize the number of resulting constraints.  False is returned if goal is
/// incompatible with the previous goals. 
bool AddGoalNonredundant(const IKGoal& goal,std::vector<IKGoal>& goalSet,Real tolerance=1e-5);


/** @brief Function class that returns the world-space position of a 
 * point on the robot.
 *
 * An individual component of a RobotIKFunction.
 * Requres robot.q to be set, and the chain updated before PreEval is called.
 */
struct WorldPositionFunction : public VectorFieldFunction
{
  WorldPositionFunction(RobotKinematics3D&,const Vector3& pi,int i,const ArrayMapping& activeDofs);
  int GetDOF(int dim) const;
  virtual std::string Label() const { return "WorldPos"; }
  virtual int NumDimensions() const { return 3; }
  virtual void Eval(const Vector& x, Vector& v);
  virtual Real Eval_i(const Vector& x, int i);
  virtual void Jacobian(const Vector& x, Matrix& J);
  //virtual void Jacobian_i(const Vector& x, int i, Vector& Ji);
  //virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);

  RobotKinematics3D& robot;
  Vector3 ploc;
  int link;
  const ArrayMapping& activeDofs;
};

/** @brief Function class that returns the position/rotation errors
 * associated with a given IKGoal.
 *
 * An individual component of a RobotIKFunction.
 * Requres robot.q to be set, and the chain updated before PreEval is called.
 */
struct IKGoalFunction : public VectorFieldFunction
{
  IKGoalFunction(RobotKinematics3D&,const IKGoal&,const ArrayMapping& activeDofs);
  int GetDOF(int dim) const;
  virtual std::string Label() const;
  virtual std::string Label(int i) const;
  virtual int NumDimensions() const;
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x, Vector& r);
  virtual Real Eval_i(const Vector& x, int i);
  virtual void Jacobian(const Vector& x, Matrix& J);
  virtual void Jacobian_i(const Vector& x, int i, Vector& Ji);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);

  void UpdateEEPos();
  void UpdateEERot();

  RobotKinematics3D& robot;
  const IKGoal& goal;
  const ArrayMapping& activeDofs;
  Real positionScale, rotationScale;
  //position,rotation error of end effector
  DirtyData<Vector3> eepos;
  DirtyData<Matrix3> eerot;
  DirtyData<std::vector<Matrix> > H;
};

/** @brief Function class that measures the difference between the robot's
 * center of mass and a desired COM.
 *
 * An individual component of a RobotIKFunction.
 * Requres robot.q to be set, and the chain updated before PreEval is called.
 */
struct RobotCOMFunction : public VectorFieldFunction
{
  RobotCOMFunction(RobotKinematics3D&,const Vector2& com,const ArrayMapping& activeDofs);
  int GetDOF(int dim) const;
  virtual std::string Label() const;
  virtual std::string Label(int i) const;
  virtual int NumDimensions() const { return 2; }
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x, Vector& r);
  virtual Real Eval_i(const Vector& x, int i);
  virtual void Jacobian(const Vector& x, Matrix& J);
  virtual void Jacobian_i(const Vector& x, int i, Vector& Ji);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);

  RobotKinematics3D& robot;
  Vector2 comGoal;
  const ArrayMapping& activeDofs;
  Real comScale;
  //temp
  Real mtotal;
  DirtyData<Matrix> Hx;
  DirtyData<Matrix> Hy;
};

/// Returns false if the goals a and b cannot be mutually reached because
/// of the length of the robot links.  Joint limits do not factor into this
/// computation.
bool IsReachableGoal(const RobotKinematics3D&,const IKGoal& a,const IKGoal& b);

/// Returns false if the goals a and b cannot be mutually reached because
/// of the length of the robot links, whose maximum distance is given as
/// jointDistance.
bool IsReachableGoal(const IKGoal& a,const IKGoal& b,Real jointDistance);

/*@}*/

#endif

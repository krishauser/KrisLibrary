#ifndef TORQUE_SOLVER_H
#define TORQUE_SOLVER_H

#include <KrisLibrary/Logger.h>
#include "RobotDynamics3D.h"
#include "Contact.h"
#include <KrisLibrary/optimization/MinNormProblem.h>

/** @brief A class that solves minimum norm of torque saturation
 *        (e.g. smallest L-inf norm (default), L-2 norm, etc)
 *
 * Typical calling sequence is
 * @verbatim
 * TorqueSolver solver(robot,contacts);
 * //setup
 * solver.SetGravity(Vector3(0,0,-9.8));
 * [if dynamics are needed] solver.SetDynamics();
 * //initialize the LP
 * solver.Init([optional: set the number of friction cone edges])
 * [add any other internal forces]
 * [add any force limits]
 * //solve
 * if(!solver.Solve())
 *   LOG4CXX_ERROR(KrisLibrary::logger(),"Torque solver failed with error"<<"\n");
 * LOG4CXX_INFO(KrisLibrary::logger(),"Torques: "<<solver.t<<"\n");
 * LOG4CXX_INFO(KrisLibrary::logger(),"Contact forces: "<<solver.f<<"\n");
 * @verbatim
 *
 * If you want to just test whether torque limits are satisfied, the
 * InTorqueBounds() function can be used rather than Solve().  It will
 * return true if feasible torques are found.
 *
 * When testing new configurations, nothing needs to be changed unless
 * dynamics are being used.  In that case, a SetDynamics call is needed
 * before the next call to Solve or InTorqueBounds.
 *
 * The solution method uses an LP, and more details can be found in eq 
 * (1) in this paper http://www.cs.indiana.edu/~hauserk/papers/Hauser-2005-NonGaited.pdf.  
 */
class TorqueSolver
{
 public:
  TorqueSolver(RobotDynamics3D& robot, const ContactFormation& contacts,int nFrictionConeEdges=4);
  TorqueSolver(RobotDynamics3D& robot, const CustomContactFormation& contacts);
  void SetGravity(const Vector3& gravity) { gravityVector=gravity; }
  void SetNorm(Real norm) { problem.norm = norm; }
  void SetDynamics(const Vector& ddq);

  ///If weighted is true, performs a conditioning step
  void Init(bool weighted=true);
  void Clear();
  ///Add extra limits on contact forces
  void LimitContactForce(int i,Real maximum,const Vector3& dir);
  void LimitContactForceSum(const std::vector<int>& indices,Real maximum,const Vector3& dir);
  ///Solves torques for the current robot configuration.
  ///Returns false on error.
  bool Solve();
  ///Just like solve, but returns true if the soln is feasible
  bool InTorqueBounds();

  ///Helper: fills the problem with the current robot config
  void FillProblem();

  ///settings
  RobotDynamics3D& robot;
  CustomContactFormation contacts;
  Vector3 gravityVector;  ///< If nonzero, specifies the gravity direction
  Vector internalForces;  ///< If nonempty, specifies the internal generalized forces.  Typically nonempty when solving for dynamics, in this case set to B(q)q''+C(q,q')

  ///output: vector of torques t and contact forces f
  Vector t,f;

  //temp
  std::vector<int> active,passive;
  Optimization::MinNormProblem_Sparse problem;
  Real tmaxMin;
  Vector lhs;
};

#endif

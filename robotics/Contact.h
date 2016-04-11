#ifndef CONTACT_H
#define CONTACT_H

#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math/matrix.h>
#include <KrisLibrary/math/sparsematrix.h>
#include <vector>
#include <KrisLibrary/errors.h>
using namespace Math3D;

/** @ingroup Robotics
 * @brief A single contact point, i.e. a position, normal, and friction
 * coefficient.
 */
struct ContactPoint
{
  inline Real frictionConeHalfAngle() const { return Atan(kFriction); }
  inline void setfrictionConeHalfAngle(Real phi) { kFriction=Tan(phi); }
  /// Returns true if the force lies within the friction cone
  bool isValidForce(const Vector3& f) const;
  /// Returns the min friction coefficient value required for f to be
  /// produced (for normal held constant).  The result is negative for
  /// invalid forces.
  Real minFriction(const Vector3& f) const;
  /// Returns the quadratic form of the FC constraint f'*A*f + b*f + c >= 0
  void quadraticForm(Matrix3& A,Vector3& b,Real& c) const;

  Vector3 x;
  Vector3 n;
  Real kFriction;
};

/** @ingroup Robotics
 * @brief A 2D contact point, i.e. a position, normal, and friction
 * coefficient.
 */
struct ContactPoint2D
{
  inline Real frictionConeHalfAngle() const { return Atan(kFriction); }
  inline void setfrictionConeHalfAngle(Real phi) { kFriction=Tan(phi); }
  /// Returns true if the force lies within the friction cone
  bool isValidForce(const Vector2& f) const;
  /// Returns the min friction coefficient value required for f to be
  /// produced (for normal held constant).  The result is negative for
  /// invalid forces.
  Real minFriction(const Vector2& f) const;
  /// Returns the quadratic form of the constraint f'*A*f + b*f + c >= 0
  void quadraticForm(Matrix2& A,Vector2& b,Real& c) const;

  Vector2 x;
  Vector2 n;
  Real kFriction;
};

/** @brief A robot-environment contact formation.
 * The list of contacts contacts[i] is associated with links[i]
 * (pointing from the opposing body into the robot).
 * Usually the opposing body is the environment, but if targets is
 * non-empty, it gives the index of the body for which contact is made.
 */
struct ContactFormation
{
  ///Sets the contact formation to a single-link set of contacts
  void set(int link,const std::vector<ContactPoint>& contacts,int target=-1);
  ///Adds another formation onto this one
  void concat(const ContactFormation& formation);
  ///Collapses this into a single list of links and cps
  void flatten(std::vector<int>& flatLinks,std::vector<ContactPoint>& cps) const;
  void flatten(std::vector<int>& flatLinks,std::vector<ContactPoint>& cps,std::vector<int>& flattargets) const;
  ///Returns the number of contact points
  int numContactPoints() const;
  ///Returns the total number of force variables (3*numContactPoints())
  int numForceVariables() const;

  std::vector<int> links;
  std::vector<std::vector<ContactPoint> > contacts;
  std::vector<int> targets;
};

/** @ingroup Robotics
 * @brief Polygonal pyramid representation of the friction cone.
 *
 * There are two representations: the positive span of FC edges, or
 * forces on the positive side of all halfspaces. 
 * That is, the halfspace plane normals point inward.
 */
struct FrictionConePolygon
{
	void set(int k,const Vector3& n,Real kFriction);
	bool contains(const Vector3& f) const;
	bool onBoundary(const Vector3& f) const;

	std::vector<Vector3> edges;
	std::vector<Vector3> planes;
};

/** @ingroup Robotics
 * @brief Converts contact points with friction to many frictionless contacts.
 *
 * It does so by putting a frictionless contact on each edge of the
 * polygonalized friction cone.
 * @param k is the discretization of the friction cone
 */
void FrictionToFrictionlessContacts(const std::vector<ContactPoint>& c1,int k,std::vector<ContactPoint>& c2);

/** @ingroup Robotics
 * @brief Converts n contact points with friction to 2n frictionless contacts.
 *
 * It does so by putting a frictionless contact on each edge of the
 * friction cone.
 */
void FrictionToFrictionlessContacts(const std::vector<ContactPoint2D>& c1,std::vector<ContactPoint2D>& c2);

/** @ingroup Robotics
 * @brief Sets the k x 3 matrix A such that A*f <= 0 defines
 * the friction cone volume.
 *
 * f is the 3d force vector applied at the contact.
 *
 * The friction cone is discretized with k edges.
 */
void GetFrictionConePlanes(const ContactPoint& contact,int k,Matrix& A);

/** @ingroup Robotics
 * @brief Sets the 2 x 2 matrix A such that A*f <= 0 defines
 * the friction cone volume.
 *
 * f is the 2d force vector applied at the contact.
 */
void GetFrictionConePlanes(const ContactPoint2D& contact,Matrix2& A);

/** @brief Retreives the 6 x NumContacts(s) matrix of wrenches
 * with moments measured about cm.
 *
 * The first 3 rows give the force, while the last 3 give the moment.
 *
 * If A is nonempty, it must be at least size 6 x NumContacts(s).
 */
void GetWrenchMatrix(const std::vector<ContactPoint>& s,const Vector3& cm,Matrix& A);
void GetWrenchMatrix(const std::vector<ContactPoint>& s,const Vector3& cm,SparseMatrix& A);
void GetWrenchMatrix(const ContactFormation& s,const Vector3& cm,SparseMatrix& A);

/** @ingroup Robotics
 * @brief Sets the matrix A such that A*f <= 0 defines the friction
 * cone planes.
 *
 * f is a vector of size contacts.size()*3, such that
 * f[i*3],f[i*3+1],f[i*3+2] is the force applied at contact[i].
 *
 * The friction cones are discretized with k edges.
 */
void GetFrictionConePlanes(const std::vector<ContactPoint>& contacts,int k,Matrix& A);
void GetFrictionConePlanes(const std::vector<ContactPoint>& contacts,int k,SparseMatrix& A);
void GetFrictionConePlanes(const ContactFormation& s,int k,SparseMatrix& A);

/** @ingroup Robotics
 * @brief Sets the matrix A such that A*f <= 0 defines the friction
 * cone planes.
 *
 * f is a vector of size contacts.size()*2, such that
 * f[i*2],f[i*2+1] is the force applied at contact[i].
 */
void GetFrictionConePlanes(const std::vector<ContactPoint2D>& contacts,Matrix& A);
void GetFrictionConePlanes(const std::vector<ContactPoint2D>& contacts,SparseMatrix& A);





/** @ingroup Robotics
* @brief A contact point with custom force / wrench constraints.
*
* Custom constraints are in the form A*f <= b for forces
* or A*w <= b for wrenches.
*/
struct CustomContactPoint
{
  CustomContactPoint();
  CustomContactPoint(const ContactPoint& cp,int numFCEdges=4);
  ///Copies from a ContactPoint, and uses a polyhedral friction cone approximation
  void set(const ContactPoint& cp,int numFCEdges=4);
  ///Offsets the friction cone by the given offset
  void setRobustnessFactor(Real offset);
  ///Bounds the minimum / maximum normal force
  void addNormalForceBounds(Real minimum,Real maximum);
  ///Re-calculates the force matrix from the values of x, n, kFriction
  void calculateForceMatrix(int numFCEdges=4);
  ///Re-calculates the wrench matrix from the values of x, n, kFriction
  ///OR the force matrix if it exists
  void calculateWrenchMatrix(int numFCEdges=4);
  ///Returns the number of force degrees of freedom: 1 means frictionless,
  ///3 means frictional contact, 6 means wrench
  int numForceVariables() const;
  ///Returns the number of constraints
  int numConstraints() const;

  Vector3 x;
  Vector3 n;
  Real kFriction;
  Matrix forceMatrix;
  Vector forceOffset;
  Matrix wrenchMatrix;
  Vector wrenchOffset;
};

/** @ingroup Robotics A contact point with custom force / wrench constraints.
 *
 * Custom constraints are in the form A*f <= b for forces
 * or A*w <= b for wrenches.
 */
struct CustomContactPoint2D
{
  CustomContactPoint2D();
  CustomContactPoint2D(const ContactPoint2D& cp);
  ///Copies from a ContactPoint
  void set(const ContactPoint2D& cp);
  ///Offsets the friction cone by the given offset
  void setRobustnessFactor(Real offset);
  ///Bounds the minimum / maximum normal force
  void addNormalForceBounds(Real minimum,Real maximum);
  ///Re-calculates the force matrix from the values of x, n, kFriction
  void calculateForceMatrix();
  ///Re-calculates the wrench matrix from the values of x, n, kFriction
  ///OR the force matrix if it exists
  void calculateWrenchMatrix();
  ///Returns the number of force degrees of freedom: 1 means frictionless,
  ///2 means frictional contact, 3 means wrench
  int numForceVariables() const;
  ///Returns the number of constraints
  int numConstraints() const;

  Vector2 x;
  Vector2 n;
  Real kFriction;
  Matrix forceMatrix;
  Vector forceOffset;
  Matrix wrenchMatrix;
  Vector wrenchOffset;
};

/** @brief A more advanced ContactFormation that
 * accepts custom contact points and custom constraints.
 *
 * The contact contacts[i] is associated with links[i]
 * (pointing from the opposing body into the robot).
 * Usually the opposing body is the environment, but if targets is
 * non-empty, it gives the index of the body for which contact is made.
 */
struct CustomContactFormation
{
  ///Resets to an empty formation
  void clear();
  ///Sets this to a list of contacts on a given link
  void set(int link,const std::vector<ContactPoint>& contacts,int numFCEdges);
  ///Sets this to aa list of contacts on a given link
  void set(int link,const std::vector<CustomContactPoint>& contacts);
  ///Sets this to a plain ContactFormation 
  void set(const ContactFormation& formation,int numFCEdges);
  ///Adds another formation onto this one
  void concat(const CustomContactFormation& formation);
  ///Convenience function: limits the extent of the sum of forces on the link
  ///direction^T force <= maximum
  void addLinkForceLimit(int link,const Vector3& direction,Real maximum);
  ///Convenience function: limits the extent of the sum of forces on the link
  ///fdirection^T force + mdirection^T moment <= maximum
  void addLinkWrenchLimit(int link,const Vector3& fdirection,const Vector3& mdirection,Real maximum);
  ///Convenience function: limits the extent of the sum of forces direction^T sum fi <= maximum
  void addForceLimit(const std::vector<int>& contacts,const Vector3& direction,Real maximum);
  ///Convenience function: limits the extent of the sum of forces
  ///fdirection^T sum fi + mdirection^T sum mi <= maximum
  void addWrenchLimit(const std::vector<int>& contacts,const Vector3& fdirection,const Vector3& mdirection,Real maximum);
  ///Adds a constraint that the sum of all forces on a given link 
  ///satisfy the condition A*f <= b (or if equality=true, A*f=b).
  void addLinkForceConstraint(int link,const Matrix& A,const Vector& b,bool equality=false);
  ///Adds a constraint that the sum of all forces/moments on a given link 
  ///satisfy the condition A*w <= b (or if equality=true, A*w=b).
  void addLinkWrenchConstraint(int link,const Matrix& A,const Vector& b,bool equality=false);
  ///Adds a constraint that the forces on the given contacts, when stacked
  ///into a big vector f=[f1,...,fn], must satisfy the constraint
  ///A*f <= b (or if equality=true, A*f=b).
  void addForceConstraint(const std::vector<int>& contacts,const Matrix& A,const Vector& b,bool equality=false);
  ///Adds a constraint that the forces on the given contacts, must satisfy
  ///the constraint sum Ai*fi <= b (or if equality=true, sum Ai*fi=b).
  void addForceConstraint(const std::vector<int>& contacts,const std::vector<Matrix>& A,const Vector& b,bool equality=false);
  ///Adds a constraint that the wrenches on the given contacts, when stacked
  ///into a big vector w=[w1,...,wn], must satisfy the constraint
  ///A*f <= b (or if equality=true, A*f=b).
  void addWrenchConstraint(const std::vector<int>& contacts,const Matrix& A,const Vector& b,bool equality=false);
  ///Adds a constraint that the wrenches on the given contacts, must satisfy
  ///the constraint sum Ai*wi <= b (or if equality=true, sum Ai*wi=b).
  void addWrenchConstraint(const std::vector<int>& contacts,const std::vector<Matrix>& A,const Vector& b,bool equality=false);
  ///Returns the total number of force variables
  int numForceVariables() const;
  ///Returns the number of constraints
  int numConstraints() const;
 
  std::vector<int> links;
  std::vector<CustomContactPoint> contacts;
  std::vector<int> targets;
  std::vector<std::vector<int> > constraintGroups;
  std::vector<std::vector<Matrix> > constraintMatrices;
  std::vector<Vector> constraintOffsets;
  std::vector<bool> constraintEqualities;
};

/** @ingroup Robotics
 * @brief: Derives the matrix that produces the force when multiplying
 * the contact force variables at the given contacts.
 */
void GetForceMatrix(const std::vector<CustomContactPoint>& contacts,SparseMatrix& A);
void GetForceMatrix(const CustomContactFormation& contacts,SparseMatrix& A);


/** @ingroup Robotics
 * @brief: Derives the matrix that produces the wrench about the center of mass cm, when multiplying
 * the contact force variables at the given contacts.
 *
 * If A is nonempty, it must have size at least 6 x contact.numForceVariables().
 */
void GetWrenchMatrix(const std::vector<CustomContactPoint>& contacts,const Vector3& cm,Matrix& A);
void GetWrenchMatrix(const std::vector<CustomContactPoint>& contacts,const Vector3& cm,SparseMatrix& A);
void GetWrenchMatrix(const CustomContactFormation& contacts,const Vector3& cm,SparseMatrix& A);


/** @ingroup Robotics
 * @brief Sets the matrix A and vector b such that A*f <= b defines the friction
 * cone planes.
 *
 * f is a vector of size contacts.size()*3, such that
 * f[i*3],f[i*3+1],f[i*3+2] is the force applied at contact[i].
 *
 * The friction cones are discretized with k edges.
 */
void GetFrictionConePlanes(const std::vector<CustomContactPoint>& contacts,Matrix& A,Vector& b);
void GetFrictionConePlanes(const std::vector<CustomContactPoint>& contacts,SparseMatrix& A,Vector& b);
void GetFrictionConePlanes(const CustomContactFormation& s,SparseMatrix& A,Vector& b);

/** @ingroup Robotics
 * @brief Sets the matrix A and vector b such that A*f <= b defines the friction
 * cone planes.
 *
 * f is a vector of size contacts.size()*2, such that
 * f[i*2],f[i*2+1] is the force applied at contact[i].
 */
void GetFrictionConePlanes(const std::vector<CustomContactPoint2D>& contacts,Matrix& A,Vector& b);
void GetFrictionConePlanes(const std::vector<CustomContactPoint2D>& contacts,SparseMatrix& A,Vector& b);



#endif

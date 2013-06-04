#ifndef CONTACT_H
#define CONTACT_H

#include <math3d/primitives.h>
#include <math/matrix.h>
#include <math/sparsematrix.h>
#include <vector>
#include <errors.h>
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
 * (pointing from the environment into the robot).
 */
struct ContactFormation
{
  //make into a single list of links and cps
  void flatten(std::vector<int>& flatLinks,std::vector<ContactPoint>& cps) const;

  std::vector<int> links;
  std::vector<std::vector<ContactPoint> > contacts;
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

///@ingroup Robotics
///Returns the total number of contacts in s
int NumContactPoints(const ContactFormation& s);

/** @brief Retreives the 6 x NumContacts(s)*3 matrix of wrenches
 * with moments measured about cm.
 *
 * The first 3 rows give the force, while the last 3 give the moment.
 */
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


#endif

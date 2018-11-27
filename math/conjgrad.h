#ifndef MATH_CONJUGATE_GRADIENT_H
#define MATH_CONJUGATE_GRADIENT_H

/**@file math/conjgrad.h
 * @ingroup Math
 * @brief Template routine using preconditioned conjugate gradient
 * to solve a linear system.
 */


#include <KrisLibrary/Logger.h>
#include "vector.h"

namespace Math {

/** @ingroup Math
 * @brief Identity precondtioner 
 */
template < class Matrix >
struct NullPreconditioner 
{
  void set(const Matrix& m) {}

  void solve(const Vector& r, Vector& x) const {
    x = r;
  }
};

/** @ingroup Math
 * @brief Jacobi preconditioning (inverse of diagonal)
 */
template < class Matrix >
struct JacobiPreconditioner 
{
  JacobiPreconditioner() : isSet(false) {}
  JacobiPreconditioner(const Matrix& m) { set(m); }

  void set(const Matrix& m)
  {
    assert(m.m == m.n);
    diagInv.resize(m.m);
    isSet = true;
    for(int i=0; i<m.m; i++) {
      if(m(i,i) == Zero) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Preconditioning won't work, there's a zero entry on diagonal!\n");
	abort();
	isSet = false;
      }
      diagInv(i) = Inv(m(i,i));
    }
  }

  void solve(const Vector& r, Vector& x) const {
    if(!isSet) {
      x = r;
      return;
    }
    x.resize(r.n);
    for(int i=0; i<r.n; i++)
      x(i) = r(i)*diagInv(i);
  }

  bool isSet;
  Vector diagInv;
};


/** @ingroup Math
 * Solves the symmetric positive definite linear
 * system Ax=b using the Conjugate Gradient method.
 * Uses any type of matrix A with a method A.mul(x,b) (b=Ax) where 
 * b,x are vectors, and any preconditioner P with method P.solve(r,x)
 * x = P*r where r,x are vectors.
 *
 * CG follows the algorithm described on p. 15 in the 
 * SIAM Templates book.
 *
 * The return value indicates convergence within max_iter (input)
 * iterations (0), or no convergence within max_iter iterations (1).
 *
 * Upon successful return, output arguments have the following values:
 *  
 *        x  --  approximate solution to Ax = b
 * max_iter  --  the number of iterations performed before the
 *               tolerance was reached
 *      tol  --  the residual after the final iteration 
 */
template < class Matrix, class Preconditioner >
int 
CG(const Matrix &A, Vector &x, const Vector &b,
   const Preconditioner &M, int &max_iter, Real &tol)
{
  Real resid;
  static Vector p, z, q;
  Real alpha, beta, rho, rho_1;

  p.resize(x.n);
  z.resize(x.n);
  q.resize(x.n);

  Real normb = norm(b);
  static Vector r;
  //r = b - A*x;
  r.resize(x.n);
    A.mul(x,p);
    r.sub(b,p);

  if (normb == 0.0) 
    normb = 1;
  
  if ((resid = norm(r) / normb) <= tol) {
    tol = resid;
    max_iter = 0;
    return 0;
  }

  for (int i = 1; i <= max_iter; i++) {
    //z = M.solve(r);
      M.solve(r,z);
    rho = dot(r, z);

    if (i == 1)
      p = z;
    else {
      beta = rho / rho_1;
      //p = z + beta * p;
        p.mul(p,beta);
        p.add(p,z);
    }

    //q = A*p;
      A.mul(p,q);


    Real d = dot(p, q);
    if(d == Zero) {
      LOG4CXX_INFO(KrisLibrary::logger(),"matrix is not SPD\n");
      max_iter = 0;
      return 0;
    }
      
    alpha = rho / d;
    
    //x += alpha * p;
    //r -= alpha * q;
      x.madd(p,alpha);
      r.madd(q,-alpha);
 
    if ((resid = norm(r) / normb) <= tol) {
      tol = resid;
      max_iter = i;
      return 0;     
    }

    rho_1 = rho;
  }
  
  tol = resid;
  return 1;
}


/*
template < class Matrix, class Vector, class Preconditioner, class Real >
int 
CG(const Matrix &A, Vector &x, const Vector &b,
   const Preconditioner &M, int &max_iter, Real &tol)
{
  Real resid;
  Vector p, z, q;
  Vector alpha(1), beta(1), rho(1), rho_1(1);

  Real normb = norm(b);
  Vector r = b - A*x;

  if (normb == 0.0) 
    normb = 1;
  
  if ((resid = norm(r) / normb) <= tol) {
    tol = resid;
    max_iter = 0;
    return 0;
  }

  for (int i = 1; i <= max_iter; i++) {
    z = M.solve(r);
    rho(0) = dot(r, z);
    
    if (i == 1)
      p = z;
    else {
      beta(0) = rho(0) / rho_1(0);
      p = z + beta(0) * p;
    }
    
    q = A*p;
    alpha(0) = rho(0) / dot(p, q);
    
    x += alpha(0) * p;
    r -= alpha(0) * q;
    
    if ((resid = norm(r) / normb) <= tol) {
      tol = resid;
      max_iter = i;
      return 0;     
    }

    rho_1(0) = rho(0);
  }
  
  tol = resid;
  return 1;
}
*/

} //namespace Math


#endif

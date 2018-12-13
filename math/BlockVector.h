#ifndef MATH_BLOCK_VECTOR_H
#define MATH_BLOCK_VECTOR_H

#include "vector.h"
#include <vector>

namespace Math {

class BlockVector : public std::vector<Vector>
{
public:
  typedef std::vector<Vector> BaseT;

  BlockVector();
  BlockVector(int numBlocks);
  BlockVector(int numBlocks,int vecSize);
  BlockVector(int numBlocks,int vecSize,Real initVal);
  BlockVector(int numBlocks,const Vector& initVal);

  int numBlocks() const { return (int)BaseT::size(); }

  void resize(int numBlocks);
  void resize(int numBlocks,int vecSize);
  void resize(int numBlocks,int vecSize,Real initVal);
  void resizeSimilar(const BlockVector&);

  inline void operator += (const BlockVector& a) { inc(a); }
  inline void operator -= (const BlockVector& a) { dec(a); }
  inline void operator *= (Real c) { inplaceMul(c); }
  inline void operator /= (Real c) { inplaceDiv(c); }

  void copy(const BlockVector&);
  void swap(BlockVector&);
  void add(const BlockVector&,const BlockVector&);
  void sub(const BlockVector&,const BlockVector&);
  void mul(const BlockVector&,Real c);
  void div(const BlockVector&,Real c);
  void inc(const BlockVector&);
  void dec(const BlockVector&);
  void madd(const BlockVector&,Real c);

  void setZero();
  void set(Real c);
  void set(const BlockVector&);
  void setNegative(const BlockVector&);

  void inplaceNegative();
  void inplaceMul(Real c);
  void inplaceDiv(Real c);

  Real dot(const BlockVector&) const;
  Real normSquared() const;
  inline Real norm() const { return Sqrt(normSquared()); }

  bool hasDims(const BlockVector&) const;
  bool hasDims(int numBlocks,int vecSize) const;
};

} //namespace Math


#endif

#ifndef MATH_BLOCK_TRIDIAGONAL_MATRIX
#define MATH_BLOCK_TRIDIAGONAL_MATRIX

#include "matrix.h"
#include "BlockVector.h"

namespace Math {

class BlockTridiagonalMatrix
{
public:
  typedef BlockTridiagonalMatrix MyT;

  BlockTridiagonalMatrix();
  BlockTridiagonalMatrix(int nBlock);
  BlockTridiagonalMatrix(int nBlock,int nMat);
  BlockTridiagonalMatrix(int nBlock,int nMat,Real initVal);

  inline int numBlockRows() const { return (int)diagonal.size(); }
  inline int numBlockCols() const { return (int)diagonal.size(); }
  int numTotalRows() const;
  int numTotalCols() const;
  void resize(int nBlock);
  void resize(int nBlock,int nMat);
  void resize(int nBlock,int nMat,Real initVal);
  void resizeSimilar(const MyT&);
  void clear();

  Matrix& operator()(int i,int j);
  const Matrix& operator()(int i,int j) const;

  inline void operator += (const MyT& a) { inc(a); }
  inline void operator -= (const MyT& a) { dec(a); }
  void add(const MyT&,const MyT&);
  void sub(const MyT&,const MyT&);
  void mul(const MyT&,Real c);
  void div(const MyT&,Real c);
  void inc(const MyT&);
  void dec(const MyT&);
  void madd(const MyT&,Real c);
  void mul(const BlockVector& a,BlockVector& x) const;
  void mulTranspose(const BlockVector& a,BlockVector& x) const;
  void madd(const BlockVector& a,BlockVector& x) const;
  void maddTranspose(const BlockVector& a,BlockVector& x) const;

  void setZero();
  void set(Real);
  void setIdentity();
  void setTranspose(const MyT&);
  bool solveInverse_LU(const BlockVector& b,BlockVector& x) const;
  bool solveInverse_Cholesky(const BlockVector& b,BlockVector& x) const; //valid only if positive definite

  void inplaceTranspose();

  void getMatrix(Matrix&) const;

  inline bool isEmpty() const { return diagonal.empty(); }
  bool isValid() const;
  bool hasDims(const MyT& m) const;
  bool hasDims(int nBlock,int nMat) const;

  std::vector<Matrix> diagonal,upperDiagonal,lowerDiagonal;
};

} //namespace Math


#endif

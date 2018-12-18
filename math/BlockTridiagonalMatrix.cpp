#include <KrisLibrary/Logger.h>
#include "BlockTridiagonalMatrix.h"
#include "LUDecomposition.h"
#include <errors.h>
using namespace Math;
using namespace std;

BlockTridiagonalMatrix::BlockTridiagonalMatrix()
{}

BlockTridiagonalMatrix::BlockTridiagonalMatrix(int nBlock)
{
  resize(nBlock);
}

BlockTridiagonalMatrix::BlockTridiagonalMatrix(int nBlock,int nMat)
{
  resize(nBlock,nMat);
}

BlockTridiagonalMatrix::BlockTridiagonalMatrix(int nBlock,int nMat,Real initVal)
{
  resize(nBlock,nMat,initVal);
}

int BlockTridiagonalMatrix::numTotalRows() const
{
  int m=0;
  for(size_t i=0;i<diagonal.size();i++)
    m += diagonal[i].m;
  return m;
}

int BlockTridiagonalMatrix::numTotalCols() const
{
  int n=0;
  for(size_t i=0;i<diagonal.size();i++)
    n += diagonal[i].n;
  return n;
}

void BlockTridiagonalMatrix::resize(int nBlock)
{
  diagonal.resize(nBlock);
  upperDiagonal.resize(nBlock-1);
  lowerDiagonal.resize(nBlock-1);
}

void BlockTridiagonalMatrix::resize(int nBlock,int nMat)
{
  resize(nBlock);
  for(size_t i=0;i<diagonal.size();i++) diagonal[i].resize(nMat,nMat);
  for(size_t i=0;i<upperDiagonal.size();i++) {
    upperDiagonal[i].resize(nMat,nMat);
    lowerDiagonal[i].resize(nMat,nMat);
  }
}

void BlockTridiagonalMatrix::resize(int nBlock,int nMat,Real initVal)
{
  resize(nBlock);
  for(size_t i=0;i<diagonal.size();i++) diagonal[i].resize(nMat,nMat,initVal);
  for(size_t i=0;i<upperDiagonal.size();i++) {
    upperDiagonal[i].resize(nMat,nMat,initVal);
    lowerDiagonal[i].resize(nMat,nMat,initVal);
  }
}


void BlockTridiagonalMatrix::resizeSimilar(const MyT& m)
{
  Assert(m.isValid());
  if(!m.isEmpty()) {
    int nMat = m.diagonal[0].m;
    resize((int)m.diagonal.size(),nMat);
  }
  else clear();
}

void BlockTridiagonalMatrix::clear()
{
  diagonal.clear();
  upperDiagonal.clear();
  lowerDiagonal.clear();
}

Matrix& BlockTridiagonalMatrix::operator()(int i,int j)
{
  if(i==j) return diagonal[i];
  if(i==j-1) return upperDiagonal[i];
  if(i-1==j) return lowerDiagonal[j];
  AssertNotReached();
  return diagonal[0];
}

const Matrix& BlockTridiagonalMatrix::operator()(int i,int j) const
{
  if(i==j) return diagonal[i];
  if(i==j-1) return upperDiagonal[i];
  if(i-1==j) return lowerDiagonal[j];
  AssertNotReached();
  return diagonal[0];
}

void BlockTridiagonalMatrix::add(const MyT& a,const MyT& b)
{
  Assert(a.hasDims(b));
  resizeSimilar(a);
  for(size_t i=0;i<diagonal.size();i++) 
    diagonal[i].add(a.diagonal[i],b.diagonal[i]);
  for(size_t i=0;i<upperDiagonal.size();i++) {
    upperDiagonal[i].add(a.upperDiagonal[i],b.upperDiagonal[i]);
    lowerDiagonal[i].add(a.lowerDiagonal[i],b.lowerDiagonal[i]);
  }
}

void BlockTridiagonalMatrix::sub(const MyT& a,const MyT& b)
{
  Assert(a.hasDims(b));
  resizeSimilar(a);
  for(size_t i=0;i<diagonal.size();i++) 
    diagonal[i].sub(a.diagonal[i],b.diagonal[i]);
  for(size_t i=0;i<upperDiagonal.size();i++) {
    upperDiagonal[i].sub(a.upperDiagonal[i],b.upperDiagonal[i]);
    lowerDiagonal[i].sub(a.lowerDiagonal[i],b.lowerDiagonal[i]);
  }
}

void BlockTridiagonalMatrix::mul(const MyT& a,Real c)
{
  resizeSimilar(a);
  for(size_t i=0;i<diagonal.size();i++) 
    diagonal[i].mul(a.diagonal[i],c);
  for(size_t i=0;i<upperDiagonal.size();i++) {
    upperDiagonal[i].mul(a.upperDiagonal[i],c);
    lowerDiagonal[i].mul(a.lowerDiagonal[i],c);
  }
}

void BlockTridiagonalMatrix::div(const MyT& a,Real c)
{
  resizeSimilar(a);
  for(size_t i=0;i<diagonal.size();i++) 
    diagonal[i].div(a.diagonal[i],c);
  for(size_t i=0;i<upperDiagonal.size();i++) {
    upperDiagonal[i].div(a.upperDiagonal[i],c);
    lowerDiagonal[i].div(a.lowerDiagonal[i],c);
  }
}

void BlockTridiagonalMatrix::inc(const MyT& a)
{
  Assert(hasDims(a));
  for(size_t i=0;i<diagonal.size();i++) 
    diagonal[i] += a.diagonal[i];
  for(size_t i=0;i<upperDiagonal.size();i++) {
    upperDiagonal[i] += a.upperDiagonal[i];
    lowerDiagonal[i] += a.lowerDiagonal[i];
  }
}

void BlockTridiagonalMatrix::dec(const MyT& a)
{
  Assert(hasDims(a));
  for(size_t i=0;i<diagonal.size();i++) 
    diagonal[i] -= a.diagonal[i];
  for(size_t i=0;i<upperDiagonal.size();i++) {
    upperDiagonal[i] -= a.upperDiagonal[i];
    lowerDiagonal[i] -= a.lowerDiagonal[i];
  }
}

void BlockTridiagonalMatrix::madd(const MyT& a,Real c)
{
  Assert(hasDims(a));
  for(size_t i=0;i<diagonal.size();i++) 
    diagonal[i].madd(a.diagonal[i],c);
  for(size_t i=0;i<upperDiagonal.size();i++) {
    upperDiagonal[i].madd(a.upperDiagonal[i],c);
    lowerDiagonal[i].madd(a.lowerDiagonal[i],c);
  }
}

void BlockTridiagonalMatrix::mul(const BlockVector& a,BlockVector& x) const
{
  Assert(a.numBlocks() == numBlockRows());
  x.resize(numBlockCols());
  for(size_t i=0;i<diagonal.size();i++) {
    diagonal[i].mul(a[i],x[i]);
    if(i>=1)
      lowerDiagonal[i-1].madd(a[i-1],x[i]);
    if(i+1<diagonal.size())
      upperDiagonal[i].madd(a[i+1],x[i]);
  }
}

void BlockTridiagonalMatrix::mulTranspose(const BlockVector& a,BlockVector& x) const
{
  Assert(a.numBlocks() == numBlockCols());
  x.resize(numBlockRows());
  for(size_t i=0;i<diagonal.size();i++) {
    diagonal[i].mulTranspose(a[i],x[i]);
    if(i>=1)
      upperDiagonal[i-1].maddTranspose(a[i-1],x[i]);
    if(i+1<diagonal.size())
      lowerDiagonal[i].maddTranspose(a[i+1],x[i]);
  }
}

void BlockTridiagonalMatrix::madd(const BlockVector& a,BlockVector& x) const
{
  Assert(a.numBlocks() == numBlockRows());
  Assert(x.numBlocks() == numBlockCols());
  for(size_t i=0;i<diagonal.size();i++) {
    diagonal[i].madd(a[i],x[i]);
    if(i>=1)
      lowerDiagonal[i-1].madd(a[i-1],x[i]);
    if(i+1<diagonal.size())
      upperDiagonal[i].madd(a[i+1],x[i]);
  }
}

void BlockTridiagonalMatrix::maddTranspose(const BlockVector& a,BlockVector& x) const
{
  Assert(a.numBlocks() == numBlockCols());
  Assert(x.numBlocks() == numBlockRows());
  for(size_t i=0;i<diagonal.size();i++) {
    diagonal[i].maddTranspose(a[i],x[i]);
    if(i>=1)
      upperDiagonal[i-1].maddTranspose(a[i-1],x[i]);
    if(i+1<diagonal.size())
      lowerDiagonal[i].maddTranspose(a[i+1],x[i]);
  }
}

void BlockTridiagonalMatrix::setZero()
{
  for(size_t i=0;i<diagonal.size();i++) diagonal[i].setZero();
  for(size_t i=0;i<upperDiagonal.size();i++) {
    upperDiagonal[i].setZero();
    lowerDiagonal[i].setZero();
  }
}

void BlockTridiagonalMatrix::set(Real c)
{
  for(size_t i=0;i<diagonal.size();i++) diagonal[i].set(c);
  for(size_t i=0;i<upperDiagonal.size();i++) {
    upperDiagonal[i].set(c);
    lowerDiagonal[i].set(c);
  }
}

void BlockTridiagonalMatrix::setIdentity()
{
  for(size_t i=0;i<diagonal.size();i++) diagonal[i].setIdentity();
  for(size_t i=0;i<upperDiagonal.size();i++) {
    upperDiagonal[i].setZero();
    lowerDiagonal[i].setZero();
  }
}

void BlockTridiagonalMatrix::setTranspose(const MyT& m)
{
  resizeSimilar(m);
  for(size_t i=0;i<diagonal.size();i++) 
    diagonal[i].setTranspose(m.diagonal[i]);
  for(size_t i=0;i<upperDiagonal.size();i++) {
    upperDiagonal[i].setTranspose(m.lowerDiagonal[i]);
    lowerDiagonal[i].setTranspose(m.upperDiagonal[i]);
  }
}

bool BlockTridiagonalMatrix::solveInverse_LU(const BlockVector& b,BlockVector& x) const
{
  Assert(b.numBlocks() == numBlockRows());
  x.resize(numBlockCols());

  Matrix mtemp,mtemp2;
  Vector vtemp,vtemp2;
  vector<Matrix> Binv (diagonal.size());
  LUDecomposition<Real> lud;
  //backsub
  for(size_t i=0;i<diagonal.size();i++) {
    if(i == 0) {
      if(!lud.set(diagonal[i])) {
        LOG4CXX_INFO(KrisLibrary::logger(),"diagonal block "<<i);
        return false;
      }
      lud.getInverse(Binv[i]);
      //solve z[i] in x[i]
      Binv[i].mul(b[i],x[i]);
    }
    else {
      //Bi = Aii - Ai,i-1*Bi-1*Ai-1,i
      mtemp.mul(Binv[i-1],lowerDiagonal[i-1]);
      mtemp2.mul(upperDiagonal[i-1],mtemp);
      mtemp.sub(diagonal[i],mtemp2);
      if(!lud.set(mtemp)) {
        LOG4CXX_INFO(KrisLibrary::logger(),"diagonal block "<<i);
        return false;
      }
      lud.getInverse(Binv[i]);
      //solve z[i] in x[i]
      lowerDiagonal[i-1].mul(x[i-1],vtemp2);
      vtemp.sub(b[i],vtemp2);
      Binv[i].mul(vtemp,x[i]);
    }
  }

  //forward sub (x[n-1] already solved)
  for(int i=(int)diagonal.size()-2;i>=0;i--) {
    //x[i] = x[i]-Binv[i]*upperDiagonal[i]*x[i+1];
    upperDiagonal[i].mul(x[i+1],vtemp);
    Binv[i].mul(vtemp,vtemp2);
    x[i] -= vtemp2;
  }
  return true;
}

bool BlockTridiagonalMatrix::solveInverse_Cholesky(const BlockVector& b,BlockVector& v) const
{
  AssertNotReached();
  return false;
}

void BlockTridiagonalMatrix::inplaceTranspose()
{
  for(size_t i=0;i<diagonal.size();i++) diagonal[i].inplaceTranspose();
  Matrix temp;
  for(size_t i=0;i<upperDiagonal.size();i++) {
    temp.setTranspose(upperDiagonal[i]);
    upperDiagonal[i].setTranspose(lowerDiagonal[i]);
    lowerDiagonal[i] = temp;
  }
}

void BlockTridiagonalMatrix::getMatrix(Matrix& mat) const
{
  int m=numTotalRows(),n=numTotalCols();
  mat.resize(m,n,Zero);
  int mcur=0,ncur=0;
  for(size_t i=0;i<diagonal.size();i++) {
    mat.copySubMatrix(mcur,ncur,diagonal[i]);
    if(i+1<diagonal.size())
      mat.copySubMatrix(mcur,ncur+diagonal[i].n,upperDiagonal[i]);
    if(i>0)
      mat.copySubMatrix(mcur,ncur-lowerDiagonal[i].n,lowerDiagonal[i-1]);
    mcur += diagonal[i].m;
    ncur += diagonal[i].n;
  }
}

bool BlockTridiagonalMatrix::isValid() const
{
  if(diagonal.empty() && upperDiagonal.empty() && lowerDiagonal.empty()) return true;
  if(upperDiagonal.size()+1 != diagonal.size()) return false;
  if(lowerDiagonal.size()+1 != diagonal.size()) return false;
  int nmat = diagonal[0].m;
  for(size_t i=0;i<diagonal.size();i++)
    if(!diagonal[i].hasDims(nmat,nmat)) return false;
  for(size_t i=0;i<upperDiagonal.size();i++) {
    if(!upperDiagonal[i].hasDims(nmat,nmat)) return false;
    if(!lowerDiagonal[i].hasDims(nmat,nmat)) return false;
  }
  return true;
}

bool BlockTridiagonalMatrix::hasDims(const MyT& m) const
{
  Assert(isValid());
  Assert(m.isValid());
  if(m.isEmpty() && isEmpty()) return true;
  if(m.diagonal.size() != diagonal.size()) return false;
  if(m.diagonal[0].m != diagonal[0].m) return false;
  return true;
}

bool BlockTridiagonalMatrix::hasDims(int nBlock,int nMat) const
{
  Assert(isValid());
  if(isEmpty()) return (nBlock==0);
  if((int)diagonal.size() != nBlock) return false;
  return (diagonal[0].m == nMat);
}


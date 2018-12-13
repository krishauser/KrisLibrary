#include "statistics.h"
using namespace std;

namespace Statistics {

void Sum(const vector<Vector>& data,Vector& sum)
{
  if(data.empty()) sum.clear();
  else {
    sum.resize(data[0].n);
    sum.setZero();
    for(size_t i=0;i<data.size();i++) {
      assert(sum.n==data[i].n);
      sum += data[i];
    }
  }
}

void Sum(const Matrix& data,Vector& sum)
{
  if(data.isEmpty()) sum.clear();
  else {
    sum.resize(data.n);
    sum.setZero();
    for(int i=0;i<data.n;i++) {
      Vector datai;
      data.getRowRef(i,datai);
      sum += datai;
    }
  }
}

Real Mean(const vector<Real>& data)
{
  return Sum(data)/(Real)data.size();
}

Real Mean(const Vector& data)
{
  return Sum(data)/(Real)data.n;
}

void Mean(const vector<Vector>& data,Vector& mean)
{
  if(data.empty()) {
    mean.clear();
    return;
  }
  Sum(data,mean);
  mean.inplaceDiv((Real)data.size());
}

void Mean(const Matrix& data,Vector& mean)
{
  if(data.isEmpty()) {
    mean.clear();
    return;
  }
  Sum(data,mean);
  mean.inplaceDiv(data.m);
}



Real Variance(const vector<Real>& data)
{
  Real mean=Mean(data);
  Real var=0;
  for(size_t i=0;i<data.size();i++)
    var += Sqr(data[i]-mean);
  return var/(Real)data.size();
}

Real Variance(const Vector& data)
{
  Real mean=Mean(data);
  Real var=0;
  for(int i=0;i<data.n;i++)
    var += Sqr(data(i)-mean);
  return var/(Real)data.n;
}

void Variance(const vector<Vector>& data,Vector& var)
{
  Vector mean;
  Mean(data,mean);
  var.resize(mean.n);
  var.setZero();
  for(size_t i=0;i<data.size();i++) {
    for(int j=0;j<var.n;j++)
      var(j) += Sqr(data[i](j) - mean(j));
  }
  var /= (Real)data.size();
}

void Variance(const Matrix& data,Vector& var)
{
  Vector mean;
  Mean(data,mean);
  var.resize(mean.n);
  var.setZero();
  for(int i=0;i<data.m;i++) {
    for(int j=0;j<var.n;j++)
      var(j) += Sqr(data(i,j) - mean(j));
  }
  var /= (Real)data.m;
}


Real StdDev(const vector<Real>& data)
{
  return Sqrt(Variance(data));
}

Real StdDev(const Vector& data)
{
  return Sqrt(Variance(data));
}

void StdDev(const vector<Vector>& data,Vector& stddev)
{
  Variance(data,stddev);
  for(int i=0;i<stddev.n;i++)
    stddev(i) = Sqrt(stddev(i));
}

void StdDev(const Matrix& data,Vector& stddev)
{
  Variance(data,stddev);
  for(int i=0;i<stddev.n;i++)
    stddev(i) = Sqrt(stddev(i));
}


Real StdDev_Robust(const vector<Real>& data)
{
  Real mean=Mean(data);
  Real max=0;  //avoid overflow by dividing by max value
  for(size_t i=0;i<data.size();i++)
    max = Max(max,Abs(data[i]-mean));
  Real var=0;
  for(size_t i=0;i<data.size();i++)
    var += Sqr((data[i]-mean)/max);
  return max*Sqrt(var/(Real)data.size());
}

Real StdDev_Robust(const Vector& data)
{
  Real mean=Mean(data);
  Real max=0;
  for(int i=0;i<data.n;i++)
    max = Max(max,Abs(data[i]-mean));
  Real var=0;
  for(int i=0;i<data.n;i++)
    var += Sqr((data[i]-mean)/max);
  return max*Sqrt(var/(Real)data.n); 
}

void StdDev_Robust(const vector<Vector>& data,Vector& stddev)
{
  Vector mean;
  Mean(data,mean);

  Vector vmax(mean.n,Zero);
  for(size_t i=0;i<data.size();i++)
    for(int j=0;j<vmax.n;j++)
      vmax(j) = Max(vmax(j),Abs(data[i](j)-mean(j)));

  stddev.resize(mean.n);
  stddev.setZero();
  for(size_t i=0;i<data.size();i++) {
    for(int j=0;j<stddev.n;j++)
      stddev(j) += Sqr((data[i](j) - mean(j))/vmax(j));
  }
  for(int j=0;j<stddev.n;j++) {
    if(vmax(j) == 0)
      stddev(j) = 0;
    else
      stddev(j) = vmax(j) * Sqrt(stddev(j)/(Real)data.size());
  }
}

void StdDev_Robust(const Matrix& data,Vector& stddev)
{
  Vector mean;
  Mean(data,mean);

  Vector vmax(mean.n,Zero);
  for(int i=0;i<data.m;i++)
    for(int j=0;j<vmax.n;j++)
      vmax(j) = Max(vmax(j),Abs(data(i,j)-mean(j)));

  stddev.resize(mean.n);
  stddev.setZero();
  for(int i=0;i<data.m;i++) {
    for(int j=0;j<stddev.n;j++)
      stddev(j) += Sqr((data(i,j) - mean(j))/vmax(j));
  }
  for(int j=0;j<stddev.n;j++)
    stddev(j) = vmax(j) * Sqrt(stddev(j)/(Real)data.m);
}


Real WeightedSum(const Vector& data,const Vector& w)
{
  assert(data.n == w.n);
  return data.dot(w);
}

void WeightedSum(const Matrix& data,const Vector& w,Vector& sum)
{
  data.mulTranspose(w,sum);
}

Real WeightedMean(const Vector& data,const Vector& w)
{
  return WeightedSum(data,w)/Sum(w);
}

void WeightedMean(const Matrix& data,const Vector& w,Vector& mean)
{
  WeightedSum(data,w,mean);
  mean /= Sum(w);
}

Real WeightedVariance(const Vector& data,const Vector& w)
{
  Real mean=WeightedMean(data,w);
  Real var=0;
  for(int i=0;i<data.n;i++)
    var += w(i)*Sqr(data(i)-mean);
  return var / Sum(w);
}

void WeightedVariance(const Matrix& data,const Vector& w,Vector& var)
{
  Vector mean;
  WeightedMean(data,w,mean);
  var.resize(mean.n);
  var.setZero();
  for(int i=0;i<data.m;i++) {
    for(int j=0;j<var.n;j++)
      var(j) += w(i)*Sqr(data(i,j) - mean(j));
  }
  var /= Sum(w);
}

Real WeightedStdDev(const Vector& data,const Vector& w)
{
  return Sqrt(WeightedVariance(data,w));
}

void WeightedStdDev(const Matrix& data,const Vector& w,Vector& stddev)
{
  WeightedVariance(data,w,stddev);
  Real (*sqrt)(Real) = Sqrt;
  stddev.inplaceComponentOp(sqrt);
  //for(int i=0;i<stddev.n;i++)
    //stddev(i) = Sqrt(stddev(i));
}




} //namespace Statistics

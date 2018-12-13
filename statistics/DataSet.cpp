#include "DataSet.h"
using namespace Statistics;

void DataSet::GetObservations(std::vector<Vector>& output)
{
  output.resize(0);
  output.resize(data.m);
  for(int i=0;i<data.m;i++)
    data.getRowRef(i,output[i]);
}

void DataSet::SetObservations(const std::vector<Vector>& input)
{
  if(input.empty()) {
    data.clear();
    return;
  }
  data.resize(input.size(),input[0].n);
  for(int i=0;i<data.m;i++) {
    assert(input[i].n == input[0].n);
    data.copyRow(i,input[i]);
  }
}

/*
void DataSet::SetObservation(int i,std::vector<Real>& in);
void DataSet::SetObservation(int i,Vector& in);

void DataSet::GetObservation(int i,std::vector<Real>& out);
void DataSet::GetObservation(int i,Vector& out);

void DataSet::SetElement(int j,std::vector<Real>& in);
void DataSet::SetElement(int j,Vector& in);
*/

void DataSet::GetElement(int j,std::vector<Real>& out)
{
  out.resize(data.m);
  for(int i=0;i<data.m;i++)
    out[i] = data(i,j);
}

void DataSet::GetElement(int j,Vector& out)
{
  data.getColCopy(j,out);
}

#include <KrisLibrary/Logger.h>
#include "IntervalSet.h"
#include <algorithm>
#include <iostream>
#include <errors.h>
using namespace Math;
using namespace std;

struct LeftPointLess
{
  bool operator() (const Interval& a,const Interval& b) const
  { return a.a < b.a; }
};

void OpenIntervalSet::Union(const BaseT& set)
{
  //sort all the segments
  BaseT x=*this,y=set;
  LeftPointLess cmp;
  sort(x.begin(),x.end(),cmp);
  sort(y.begin(),y.end(),cmp);
  for(size_t i=0;i+1<x.size();i++)
    Assert(x[i].b <= x[i+1].a);
  for(size_t i=0;i+1<y.size();i++)
    Assert(y[i].b <= y[i+1].a);

  size_t i=0,j=0;
  OpenInterval temp;
  resize(0);
  while(i != x.size() && j != y.size()) {
    if(x[i].a < y[j].a) {    //shift y intervals
      while(j != y.size() && y[j].b <= x[i].b) j++;
      temp.a = x[i].a;
      if(j != y.size() && y[j].a < x[i].b) {
	temp.b = y[j].b;
	j++;
      }
      else {
	temp.b=x[i].b;
      }
      i++;
    }
    else {   //shift x intervals
      while(i != x.size() && x[i].b <= y[j].b) i++;
      temp.a = y[j].a;
      if(i != x.size() && x[i].a < y[j].b) {
	temp.b = x[i].b;
	i++;
      }
      else {
	temp.b=y[j].b;
      }
      j++;
    }
    push_back(temp);
  }
  //one of the two (or both) are at the end
  for(;j<y.size();j++)
    push_back(y[j]);
  for(;i<x.size();i++)
    push_back(x[i]);
}

/*
  void Intersect(const BaseT&);
//  void Subtract(const ClosedBaseT&);
  void Union(const OpenInterval&);
*/

void OpenIntervalSet::Intersect(const OpenInterval& interval)
{
  for(size_t i=0;i<size();i++) {
    (*this)[i].setIntersection((*this)[i],interval);
    if((*this)[i].isEmpty()) {
      erase(begin()+i);
      i--;
    }
  }
}

//  void Subtract(const ClosedInterval&);


void ClosedIntervalSet::Union(const BaseT& set)
{
  //sort all the segments
  BaseT x=*this,y=set;
  LeftPointLess cmp;
  sort(x.begin(),x.end(),cmp);
  sort(y.begin(),y.end(),cmp);
  for(size_t i=0;i+1<x.size();i++)
    Assert(x[i].b < x[i+1].a);
  for(size_t i=0;i+1<y.size();i++)
    Assert(y[i].b < y[i+1].a);

  size_t i=0,j=0;
  ClosedInterval temp;
  resize(0);
  while(i != x.size() && j != y.size()) {
    if(x[i].a < y[j].a) {  //shift y intervals
      while(j != y.size() && y[j].b <= x[i].b) j++;
      temp.a = x[i].a;
      if(j != y.size() && y[j].a <= x[i].b) {
	temp.b = y[j].b;
	j++;
      }
      else {
	temp.b=x[i].b;
      }
      i++;
    }
    else {   //shift x intervals
      while(i != x.size() && x[i].b <= y[j].b) i++;
      temp.a = y[j].a;
      if(i != x.size() && x[i].a <= y[j].b) {
	temp.b = x[i].b;
	i++;
      }
      else {
	temp.b=y[j].b;
      }
      j++;
    }
    push_back(temp);
  }
  //one of the two (or both) are at the end
  for(;j<y.size();j++)
    push_back(y[j]);
  for(;i<x.size();i++)
    push_back(x[i]);
}

void ClosedIntervalSet::Intersect(const BaseT& set)
{
  //sort all the segments
  BaseT x=*this,y=set;
  LeftPointLess cmp;
  sort(x.begin(),x.end(),cmp);
  sort(y.begin(),y.end(),cmp);
  for(size_t i=0;i+1<x.size();i++)
    Assert(x[i].b <= x[i+1].a);
  for(size_t i=0;i+1<y.size();i++)
    Assert(y[i].b <= y[i+1].a);

  /*
  LOG4CXX_INFO(KrisLibrary::logger(),"Intersecting intervals: ");
  for(size_t i=0;i<x.size();i++)
    LOG4CXX_INFO(KrisLibrary::logger(),"["<<x[i].a<<","<<x[i].b<<"] ");
  LOG4CXX_INFO(KrisLibrary::logger(),"\n");
  for(size_t i=0;i<y.size();i++)
    LOG4CXX_INFO(KrisLibrary::logger(),"["<<y[i].a<<","<<y[i].b<<"] ");
  LOG4CXX_INFO(KrisLibrary::logger(),"\n");
  */

  size_t i=0,j=0;
  ClosedInterval temp;
  resize(0);
  while(i != x.size() && j != y.size()) {
    if(x[i].b < y[j].a) {  //shift x
      i++;
    }
    else if(y[j].b < x[i].a) {  //shift y
      j++;
    }
    else {
      //intersection
      temp.setIntersection(x[i],y[j]);
      Assert(!temp.isEmpty());
      push_back(temp);
      if(x[i].a < y[j].a) i++;
      else j++;
    }
  }

  /*
  LOG4CXX_INFO(KrisLibrary::logger(),"Res: ");
  for(size_t i=0;i<size();i++)
    LOG4CXX_INFO(KrisLibrary::logger(),"["<<(*this)[i].a<<","<<(*this)[i].b<<"] ");
  LOG4CXX_INFO(KrisLibrary::logger(),"\n");
  */
}
/*
  void Subtract(const OpenBaseT&);
  void Union(const ClosedInterval&);
  void Intersect(const ClosedInterval&);
  void Subtract(const OpenInterval&);

*/

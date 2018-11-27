#include <KrisLibrary/Logger.h>
#include "Kinematics.h"
using namespace std;

Real MaxLinkParentDistance(const RobotKinematics3D& robot,int link)
{
  if(robot.links[link].type == RobotLink3D::Revolute) return robot.links[link].T0_Parent.t.norm();
  else {
    Vector3 p1 = robot.links[link].T0_Parent.t + robot.qMin(link)*robot.links[link].w;
    Vector3 p2 = robot.links[link].T0_Parent.t + robot.qMax(link)*robot.links[link].w;
    return Max(p1.norm(),p2.norm());
  }
}

Real MaxLinkSiblingDistance(const RobotKinematics3D& robot,int link1,int link2)
{
  if(robot.links[link1].type == RobotLink3D::Revolute) {
    if(robot.links[link2].type == RobotLink3D::Revolute) 
      return robot.links[link1].T0_Parent.t.distance(robot.links[link2].T0_Parent.t);
    else {
      Vector3 p1 = robot.links[link2].T0_Parent.t + robot.qMin(link2)*robot.links[link2].w;
      Vector3 p2 = robot.links[link2].T0_Parent.t + robot.qMax(link2)*robot.links[link2].w;
      return Max(robot.links[link1].T0_Parent.t.distance(p1),robot.links[link1].T0_Parent.t.distance(p2));
    }
  }
  else {
    if(robot.links[link2].type == RobotLink3D::Revolute) {
      Vector3 p1 = robot.links[link1].T0_Parent.t + robot.qMin(link1)*robot.links[link1].w;
      Vector3 p2 = robot.links[link1].T0_Parent.t + robot.qMax(link1)*robot.links[link1].w;
      return Max(robot.links[link2].T0_Parent.t.distance(p1),robot.links[link2].T0_Parent.t.distance(p2));
    }
    else {
      Vector3 p1 = robot.links[link1].T0_Parent.t + robot.qMin(link1)*robot.links[link1].w;
      Vector3 p2 = robot.links[link1].T0_Parent.t + robot.qMax(link1)*robot.links[link1].w;
      Vector3 p3 = robot.links[link2].T0_Parent.t + robot.qMin(link2)*robot.links[link2].w;
      Vector3 p4 = robot.links[link2].T0_Parent.t + robot.qMax(link2)*robot.links[link2].w;
      return Sqrt(Max(Max(p1.distanceSquared(p3),p1.distanceSquared(p4)),Max(p2.distanceSquared(p3),p2.distanceSquared(p4))));
    }
  }
}

Real MaxJointDistance(const RobotKinematics3D& robot,int link1,int link2)
{
  int p = robot.LCA(link1,link2);
  if(p < 0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"MaxJointDistance Error: joints don't have a common parent?\n");
    Abort();
    return Inf;
  }
  //if link1 or link2 is an ancestor of the other, don't include the
  //length of the branch
  Real d1=0,d2=0;
  if(link1 != p) {
    while(robot.parents[link1]!=p) {
      d1 += MaxLinkParentDistance(robot,link1);
      link1 = robot.parents[link1];
    }
    Assert(link1 != p);
  }
  if(link2 != p) {
    while(robot.parents[link2]!=p) {
      d2 += MaxLinkParentDistance(robot,link2);
      link2 = robot.parents[link2];
    }
    Assert(link2 != p);
  }
  if(link1 == p) {
    if(link2 == p) return 0; //same link
    else 
      return d2 + MaxLinkParentDistance(robot,link2);
  }
  else {
    if(link2 == p) //2 is an ancestor of 1
      return d1 + MaxLinkParentDistance(robot,link1);
    else
      return d1 + d2 + MaxLinkSiblingDistance(robot,link1,link2);
  } 
}

void ComputeJointDistances(const RobotKinematics3D& robot,vector<vector<Real> >& dist)
{
  vector<vector<int> > childList;
  robot.GetChildList(childList);

  dist.resize(robot.links.size());
  for(size_t i=0;i<robot.links.size();i++)
    dist[i].resize(robot.links.size(),Inf);

  //floyd's algorithm
  //initialization: edge distances
  for(size_t i=0;i<robot.links.size();i++) {
    dist[i][i] = 0;
    if(robot.parents[i] >= 0) {
      dist[i][robot.parents[i]] = dist[robot.parents[i]][i] = MaxLinkParentDistance(robot,i);
    }
    if(childList[i].size() >= 2) {
      for(size_t j=1;j<childList[i].size();j++) {
	for(size_t k=0;k<j;k++) {
	  Real d = MaxLinkSiblingDistance(robot,j,k);
	  if(d < dist[j][k]) dist[j][k] = dist[k][j] = d;
	}
      }
    }
  }
  
  for(size_t k=0;k<robot.links.size();k++) {
    //kth iteration: dist holds the lowest cost to go from i to j using only
    //links 0...k-1.
    //Consider adding vertex k.  If it results in a shorter path, store it
    for(size_t i=0;i<robot.links.size();i++) {
      for(size_t j=0;j<robot.links.size();j++) {
	if(dist[i][k] + dist[k][j] < dist[i][j])
	  dist[i][j] = dist[i][k] + dist[k][j];
      }
    }
  }
}

Real MaxLimbSpan(const RobotKinematics3D& robot)
{
  vector<vector<Real> > dist;
  ComputeJointDistances(robot,dist);

  Real max=0;
  for(size_t i=0;i<robot.links.size();i++)
    for(size_t j=0;j<robot.links.size();j++)
      if(!IsInf(dist[i][j]) && dist[i][j] > max)
	max = dist[i][j];
  return max;
}

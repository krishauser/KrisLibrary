#include "ClosestPoint.h"
#include <errors.h>
#include <list>
#include <set>
#include "structs/FastFindHeap.h"
using namespace std;

namespace Meshing {

int ClosestPointDescent(const TriMeshWithTopology& m,const Vector3& p,int tri,Vector3& cp)
{
  Assert(m.tris.size() == m.triNeighbors.size());
  Assert(tri >= 0 && tri < (int)m.tris.size());
  Triangle3D t;
  Vector3 temp;
  Real closestd=Inf;
  FastFindHeap<int,Real> q;  //sorted from highest to lowest
  set<int> visited;
  q.push(tri,-Inf);
  while(!q.empty()) {
    int curtri = q.top(); q.pop();
    visited.insert(curtri);

    m.GetTriangle(curtri,t);
    temp = t.closestPoint(p);
    Real d=temp.distance(p);
    if(d < closestd) {
      tri = curtri;
      closestd = d;
      cp = temp;
      //traverse to neighbors
      for(int i=0;i<3;i++) {
	int neighbor = m.triNeighbors[curtri][i];
	if(neighbor >= 0) {
	  if(visited.find(neighbor)==visited.end())
	    q.adjust(neighbor,-d);
	}
      }
    }
  } 
  return tri;
}

} //namespace Meshing

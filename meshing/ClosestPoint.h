#ifndef MESHING_CLOSEST_POINT_H
#define MESHING_CLOSEST_POINT_H

#include "TriMeshTopology.h"

namespace Meshing { 

///Finds the closest point to p on m by a gradient descent
///starting from triangle tri
int ClosestPointDescent(const TriMeshWithTopology& m,const Vector3& p,int tri,Vector3& cp);

} //namespace Meshing

#endif

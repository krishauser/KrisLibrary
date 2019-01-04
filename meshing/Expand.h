#ifndef MESHING_EXPAND_H
#define MESHING_EXPAND_H

#include "TriMeshTopology.h"

namespace Meshing {

void Expand(const TriMeshWithTopology& in,Real distance,int divs,TriMesh& m);
void Expand2Sided(const TriMeshWithTopology& in,Real distance,int divs,TriMesh& m);

} //namespace Meshing

#endif


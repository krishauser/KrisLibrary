#ifndef MESHING_IO_H
#define MESHING_IO_H

#include <iostream>
#include "TriMesh.h"

namespace Meshing {

///Import will try to determine the file type via the file extension
bool Import(const char* fn,TriMesh& tri);
///Export will try to determine the file type via the file extension
bool Export(const char* fn,const TriMesh& tri);

///Loads from VRML file format
bool LoadVRML(std::istream& in,TriMesh& tri);
///Saves to VRML file format
bool SaveVRML(std::ostream& out,const TriMesh& tri);

///Loads from the GeomView Object File Format (OFF)
bool LoadOFF(std::istream& in,TriMesh& tri);
///Saves to the GeomView Object File Format (OFF)
bool SaveOFF(std::ostream& out,const TriMesh& tri);

} //namespace Meshing

#endif

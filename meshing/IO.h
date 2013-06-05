#ifndef MESHING_IO_H
#define MESHING_IO_H

#include <iostream>
#include "TriMesh.h"

namespace Meshing {

///Import will try to determine the file type via the file extension
bool Import(const char* fn,TriMesh& tri);
///Export will try to determine the file type via the file extension
bool Export(const char* fn,const TriMesh& tri);

///Returns true if the extension is a file type that we can load from
bool CanLoadTriMeshExt(const char* ext);
///Returns true if the extension is a file type that we can save to 
bool CanSaveTriMeshExt(const char* ext);

///Loads from VRML file format
bool LoadVRML(std::istream& in,TriMesh& tri);
///Saves to VRML file format
bool SaveVRML(std::ostream& out,const TriMesh& tri);

///Loads from the GeomView Object File Format (OFF)
bool LoadOFF(std::istream& in,TriMesh& tri);
///Saves to the GeomView Object File Format (OFF)
bool SaveOFF(std::ostream& out,const TriMesh& tri);

///Loads using Assimp if available on your system
bool LoadAssimp(const char* fn,TriMesh& tri);
bool LoadAssimp(const char* fn,vector<TriMesh>& meshes);
///Saves using Assimp if available on your system
bool SaveAssimp(const char* fn,const TriMesh& tri);

} //namespace Meshing

#endif

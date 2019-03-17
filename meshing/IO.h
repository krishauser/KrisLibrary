#ifndef MESHING_IO_H
#define MESHING_IO_H

#include <iosfwd>
#include "TriMesh.h"

namespace GLDraw {
  //forward declaration
  class GeometryAppearance;
}; 

namespace Meshing {

///Import will try to determine the file type via the file extension
bool Import(const char* fn,TriMesh& tri);
///Import will try to determine the file type via the file extension.
///If the file format does not contain any appearance information,
///then the appearance argument will be untouched.
bool Import(const char* fn,TriMesh& tri,GLDraw::GeometryAppearance& appearance);
///Export will try to determine the file type via the file extension
bool Export(const char* fn,const TriMesh& tri);
///Export will try to determine the file type via the file extension
bool Export(const char* fn,const TriMesh& tri,const GLDraw::GeometryAppearance& appearance);

///Returns true if the extension is a file type that we can load from
bool CanLoadTriMeshExt(const char* ext);
///Returns true if the extension is a file type that we can save to 
bool CanSaveTriMeshExt(const char* ext);

///Loads from VRML file format: not implemented.
bool LoadVRML(std::istream& in,TriMesh& tri);
///Saves to VRML file format: not implemented.
bool SaveVRML(std::ostream& out,const TriMesh& tri);

///Loads from the GeomView Object File Format (OFF)
bool LoadOFF(std::istream& in,TriMesh& tri);
///Saves to the GeomView Object File Format (OFF)
bool SaveOFF(std::ostream& out,const TriMesh& tri);

///Loads from the Wavefront OBJ file format
bool LoadOBJ(const char* fn,TriMesh& tri);
///Loads from the Wavefront OBJ file format (can get per-vertex colors)
bool LoadOBJ(const char* fn,TriMesh& tri,GLDraw::GeometryAppearance& app);
///Saves to the Wavefront OBJ file format
bool SaveOBJ(const char* fn,const TriMesh& tri);
///Saves to the Wavefront OBJ file format (per-vertex colors not supported yet)
bool SaveOBJ(const char* fn,const TriMesh& tri,const GLDraw::GeometryAppearance& app);

///Loads using Assimp if available on your system
bool LoadAssimp(const char* fn,TriMesh& tri);
///Loads using Assimp if available on your system
bool LoadAssimp(const char* fn,TriMesh& tri,GLDraw::GeometryAppearance& appearance);
///Loads using Assimp if available on your system, extracts individual meshes
bool LoadAssimp(const char* fn,vector<TriMesh>& meshes);
///Loads using Assimp if available on your system, extracts individual meshes
bool LoadAssimp(const char* fn,vector<TriMesh>& meshes,vector<GLDraw::GeometryAppearance>& appearances);
///Saves using Assimp if available on your system 
bool SaveAssimp(const char* fn,const TriMesh& tri);
///Saves using Assimp if available on your system (vertex color output is experimental)
bool SaveAssimp(const char* fn,const TriMesh& tri,const GLDraw::GeometryAppearance& appearance);

} //namespace Meshing

#endif

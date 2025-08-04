#include <KrisLibrary/Logger.h>
#include "IO.h"
#include <locale.h>
#include <KrisLibrary/utils/AnyValue.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/fileutils.h>
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <KrisLibrary/image/io.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <KrisLibrary/utils/SimpleFile.h>
#include <string.h>
#include <errors.h>

#if HAVE_ASSIMP
#if ASSIMP_MAJOR_VERSION==2
#include <assimp/assimp.hpp>      // C++ importer interface
#include <assimp/aiScene.h>           // Output data structure
#include <assimp/aiPostProcess.h>     // Post processing flags
#else
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/Exporter.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

///HACK BECAUSE SOME VERSIONS OF ASSIMP DONT PROVIDE FUNCTIONALITY TO DELETE SCENES
///UNCOMMENT IF YOU ARE GETTING ERRORS LIKE "undefined reference to aiScene::~aiScene"

#ifdef _WIN32

namespace Assimp {
  struct ScenePrivateData {

    ScenePrivateData()
      : mOrigImporter()
      , mPPStepsApplied()
    {}

    // Importer that originally loaded the scene though the C-API
    // If set, this object is owned by this private data instance.
    Assimp::Importer* mOrigImporter;

    // List of postprocessing steps already applied to the scene.
    unsigned int mPPStepsApplied;
  };
};

// ------------------------------------------------------------------------------------------------
aiScene::aiScene()
  : mFlags()
  , mRootNode()
  , mNumMeshes()
  , mMeshes()
  , mNumMaterials()
  , mMaterials()
  , mNumAnimations()
  , mAnimations()
  , mNumTextures()
  , mTextures()
  , mNumLights()
  , mLights()
  , mNumCameras()
  , mCameras()
  , mPrivate(new Assimp::ScenePrivateData())
{
}

// ------------------------------------------------------------------------------------------------
aiScene::~aiScene()
{
  // delete all sub-objects recursively
  delete mRootNode;

  // To make sure we won't crash if the data is invalid it's
  // much better to check whether both mNumXXX and mXXX are
  // valid instead of relying on just one of them.
  if (mNumMeshes && mMeshes)
    for (unsigned int a = 0; a < mNumMeshes; a++)
      delete mMeshes[a];
  delete[] mMeshes;

  if (mNumMaterials && mMaterials)
    for (unsigned int a = 0; a < mNumMaterials; a++)
      delete mMaterials[a];
  delete[] mMaterials;

  if (mNumAnimations && mAnimations)
    for (unsigned int a = 0; a < mNumAnimations; a++)
      delete mAnimations[a];
  delete[] mAnimations;

  if (mNumTextures && mTextures)
    for (unsigned int a = 0; a < mNumTextures; a++)
      delete mTextures[a];
  delete[] mTextures;

  if (mNumLights && mLights)
    for (unsigned int a = 0; a < mNumLights; a++)
      delete mLights[a];
  delete[] mLights;

  if (mNumCameras && mCameras)
    for (unsigned int a = 0; a < mNumCameras; a++)
      delete mCameras[a];
  delete[] mCameras;

  delete static_cast<Assimp::ScenePrivateData*>(mPrivate);
}

#endif //_WIN32

#endif //ASSIMP_MAJOR_VERSION
using namespace Assimp;
#endif //HAVE_ASSIMP

using namespace std;
using namespace Math3D;
using namespace GLDraw;

namespace Meshing {

static string gTexturePath;

///Returns true if the extension is a file type that we can load from
bool CanLoadTriMeshExt(const char* ext)
{
  if(0==strcmp(ext,"tri")) return true;
  else if(0==strcmp(ext,"off")) return true;
  else {
#if HAVE_ASSIMP
    Assimp::Importer importer;
    string dext = "."+string(ext);
    return importer.IsExtensionSupported(dext.c_str());
#endif //HAVE_ASSIMP
  }
  return false;
}

void LoadTriMeshExtensions(std::vector<std::string>& exts)
{
  exts.resize(0);
  exts.push_back("tri");
  exts.push_back("off");
#if HAVE_ASSIMP
  Assimp::Importer importer;
  string extlist;
  importer.GetExtensionList(extlist);
  vector<string> assimpexts = Split(extlist,";");  //still have format *.EXT
  for(auto s:assimpexts) {
    exts.push_back(s.substr(2));
  }
#endif //HAVE_ASSIMP
}

void SaveTriMeshExtensions(std::vector<std::string>& exts)
{
  exts.resize(0);
  exts.push_back("tri");
  exts.push_back("off");
  exts.push_back("wrl");
#if HAVE_ASSIMP
  Assimp::Exporter exporter;
  for(size_t i=0;i<exporter.GetExportFormatCount();i++) 
    exts.push_back(exporter.GetExportFormatDescription(i)->fileExtension);
#endif //HAVE_ASSIMP
}


///Returns true if the extension is a file type that we can save to
bool CanSaveTriMeshExt(const char* ext)
{
  if(0==strcmp(ext,"tri")) return true;
  else if(0==strcmp(ext,"off")) return true;
  else if(0==strcmp(ext,"obj")) return true;
  else {
#if HAVE_ASSIMP
    Assimp::Exporter exporter;
    for(size_t i=0;i<exporter.GetExportFormatCount();i++) {
      if(0 == strcmp(ext,exporter.GetExportFormatDescription(i)->fileExtension))
        return true;
    }
    return false;
#endif
  }
  return false;
}

bool CanSaveTriMeshAppearanceExt(const char* ext)
{
  if(0==strcmp(ext,"obj")) return true; //Wavefront OBJ supports per-vertex colors
  else if(0==strcmp(ext,"off")) return true; //GeomView OFF supports per-face colors
  else if(0==strcmp(ext,"tri")) return false;
  else if(0==strcmp(ext,"stl")) return false;
  else if(0==strcmp(ext,"ply")) return false;
  else return CanSaveTriMeshExt(ext);  //assume Assimp supports appearance data
}

bool CanSaveMultipleTriMeshExt(const char* ext)
{
  if(0==strcmp(ext,"tri")) return false; 
  else if(0==strcmp(ext,"off")) return false; 
  else if(0==strcmp(ext,"obj")) return false; //Wavefront OBJ does not support multiple meshes
  else if(0==strcmp(ext,"stl")) return false; //STL does not support multiple meshes
  else return CanSaveTriMeshExt(ext);  //assume Assimp supports multiple meshes
}


///Import will try to determine the file type via the file extension
bool Import(const char* fn,TriMesh& tri)
{
  const char* ext=FileExtension(fn);
  if(!ext) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Import(TriMesh): file "<<fn<<" has no extension, cannot determine type");
    return false;
  }
  if(0==strcmp(ext,"tri")) {
    return LoadMultipleTriMeshes(fn,tri);
  }
  else if(0==strcmp(ext,"off")) {
    ifstream in(fn,ios::in);
    if(!in) return false;
    return LoadOFF(in,tri);
  }
  else {
#if HAVE_ASSIMP
    if(!LoadAssimp(fn,tri)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Import(TriMesh): file "<<fn);
      return false;
    }
    else {
      return true;
    }
#else
    if(0==strcmp(ext,"wrl")) {
      ifstream in(fn,ios::in);
      if(!in) return false;
      return LoadVRML(in,tri);
    }

    else {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Import(TriMesh): file extension "<<ext);
      return false;
    }
#endif
  }
}

///Import will try to determine the file type via the file extension
bool Import(const char* fn,TriMesh& tri,GeometryAppearance& app)
{
  const char* ext=FileExtension(fn);
  if(!ext) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Import(TriMesh): file "<<fn<<" has no extension, cannot determine type");
    return false;
  }
  if(0==strcmp(ext,"tri")) {
    return LoadMultipleTriMeshes(fn,tri);
  }
  else {
#if HAVE_ASSIMP
    //do the loading
    if(!LoadAssimp(fn,tri,app)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Import(TriMesh): file "<<fn);
      return false;
    }
    else {
      return true;
    }
#else
    if(0==strcmp(ext,"obj")) {
      if(LoadOBJ(fn,tri,app))
        return true;
    }
    else if(0==strcmp(ext,"off")) {
      ifstream in(fn,ios::in);
      if(!in) return false;
      return LoadOFF(in,tri,app);
    }
    if(0==strcmp(ext,"wrl")) {
      ifstream in(fn,ios::in);
      if(!in) return false;
      return LoadVRML(in,tri);
    }

    else {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Import(TriMesh): file extension "<<ext);
      return false;
    }
#endif
  }
}

bool Import(const char* fn,vector<TriMesh>& meshes,vector<GLDraw::GeometryAppearance>& appearances)
{
  const char* ext=FileExtension(fn);
  if(!ext) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Import(TriMesh): file "<<fn<<" has no extension, cannot determine type");
    return false;
  }
  if(0==strcmp(ext,"tri")) {
    meshes.resize(0);
    appearances.resize(0);
    ifstream in(fn);
    if(!in) return false;
    while(in) {
      TriMesh tri;
      in>>tri;
      if(in.bad()) {
        if(meshes.empty()) {
          LOG4CXX_ERROR(KrisLibrary::logger(),"Import(TriMesh): file "<<fn<<" is not a valid .tri file");
          return false;
        }
        break; //EOF or error
      }
      meshes.push_back(tri);
      appearances.push_back(GLDraw::GeometryAppearance());
    }
    return true;
  }
  else {
#if HAVE_ASSIMP
    //do the loading
    if(!LoadAssimp(fn,meshes,appearances)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Import(TriMesh): file "<<fn);
      return false;
    }
    else {
      return true;
    }
#else
    meshes.resize(1);
    appearances.resize(1);
    return Import(fn,meshes[0],appearances[0]);
#endif
  }
}


bool Export(const char* fn,const TriMesh& tri)
{
  const char* ext=FileExtension(fn);
  if(!ext) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Export(TriMesh): file "<<fn<<" has no extension, cannot determine type");
    return false;
  }
  if(0==strcmp(ext,"tri")) {
    ofstream out(fn,ios::out);
    if(!out) return false;
    out<<tri;
    return true;
  }
  else if(0==strcmp(ext,"off")) {
    ofstream out(fn,ios::out);
    if(!out) return false;
    return SaveOFF(out,tri);
  }
  else if(0==strcmp(ext,"obj")) {
    return SaveOBJ(fn,tri);
  }
  else {
#if HAVE_ASSIMP
    //right now SaveAssimp is not working yet...
    if(!SaveAssimp(fn,tri)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Export(TriMesh): file "<<fn<<" could not be saved to type "<<ext);
      return false;
    }
    else {
      return true;
    }
#else
    if(0==strcmp(ext,"wrl")) {
      ofstream out(fn,ios::out);
      if(!out) return false;
      return SaveVRML(out,tri);
    }
    else {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Export(TriMesh): file extension "<<ext);
      return false;
    }
#endif
  }
}

bool Export(const char* fn,const TriMesh& tri,const GeometryAppearance& app)
{
  const char* ext=FileExtension(fn);
  if(!ext) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Export(TriMesh): file "<<fn<<" has no extension, cannot determine type");
    return false;
  }
  if(0==strcmp(ext,"tri")) {
    ofstream out(fn,ios::out);
    if(!out) return false;
    out<<tri;
    return true;
  }
  else if(0==strcmp(ext,"off")) {
    ofstream out(fn,ios::out);
    if(!out) return false;
    return SaveOFF(out,tri,app);
  }
  else if(0==strcmp(ext,"obj")) {
    return SaveOBJ(fn,tri,app);
  }
  else {
#if HAVE_ASSIMP
    //right now SaveAssimp is not working yet...
    if(!SaveAssimp(fn,tri,app)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Export(TriMesh): file "<<fn<<" could not be saved to type "<<ext);
      return false;
    }
    else {
      return true;
    }
#else
    if(0==strcmp(ext,"wrl")) {
      ofstream out(fn,ios::out);
      if(!out) return false;
      return SaveVRML(out,tri);
    }
    else {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Export(TriMesh): file extension "<<ext);
      return false;
    }
#endif
  }
}

bool Export(const char* fn,const vector<TriMesh>& meshes,const vector<GeometryAppearance>& appearances)
{
  assert(meshes.size() == appearances.size());
  const char* ext=FileExtension(fn);
  if(!ext) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Export(TriMesh): file "<<fn<<" has no extension, cannot determine type");
    return false;
  }
  if(!CanSaveMultipleTriMeshExt(ext)) {
    //merge the meshes
    if(meshes.size() == 1) {
      return Export(fn,meshes[0],appearances[0]);
    }
    TriMesh tri;
    tri.Union(meshes);
    //too bad, can't merge appearances
    return Export(fn,tri,appearances[0]);
  }
  else {
    //use Assimp to save multiple meshes
#if HAVE_ASSIMP
    //right now SaveAssimp is not working yet...
    if(!SaveAssimp(fn,meshes,appearances)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Export(TriMesh): file "<<fn<<" could not be saved to type "<<ext);
      return false;
    }
    else {
      return true;
    }
#else
    return false;
#endif
  }
}


///Loads from VRML file format
bool LoadVRML(std::istream& in,TriMesh& tri)
{
    LOG4CXX_ERROR(KrisLibrary::logger(),"LoadVRML not implemented yet\n");
  return false;
}

///Saves to VRML file format
bool SaveVRML(std::ostream& out,const TriMesh& tri)
{
    LOG4CXX_ERROR(KrisLibrary::logger(),"SaveVRML not implemented yet\n");
  return false;
}

///Loads from the GeomView Object File Format (OFF)
bool LoadOFF(std::istream& in,TriMesh& tri,vector<GLDraw::GLColor>& faceColors)
{
  string tag;
  in>>tag;
  if(tag != "OFF") {
        LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOFF: not a proper OFF file\n");
    return false;
  }
  int mode = 0; //0: waiting for sizes, 1: reading verts, 2: reading tris
  int numFaces = 0;
  int vertIndex=0,faceIndex=0;
  int lineno = 0;
  string line;
  while(in) {
    lineno++;
    getline(in,line);
    if(in.bad()) {
      if(faceIndex == numFaces) return true;
            LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOFF: error reading line "<<lineno);
      return false;
    }
    if(line.length() == 0) continue;
    if(line[0] == '#') continue; //comment line
    if(mode == 0) {
      stringstream ss(line);
      int nv,nf,ne;
      ss>>nv>>nf>>ne;
      if(ss.bad()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOFF: unable to read first line\n");
        in.setstate(ios::badbit);
        return false;
      }
      tri.verts.resize(nv);
      tri.tris.resize(0);
      faceColors.resize(0);
      numFaces = nf;
      mode = 1;
      if((int)tri.verts.size() == vertIndex) mode=2;
    }
    else if(mode == 1) {
      stringstream ss(line);
      ss >> tri.verts[vertIndex];
      if(ss.bad()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOFF: unable to read vertex from line "<<lineno<<" = "<<line);
        in.setstate(ios::badbit);
        return false;
      }
      //ignore RGBA colors
      vertIndex++;
      if((int)tri.verts.size() == vertIndex) mode=2;
      if(faceIndex == numFaces) return true;
    }
    else if(mode == 2) {
      stringstream ss(line);
      int nv;
      vector<int> face;
      ss >> nv;
      if(ss.bad()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOFF: unable to load number of vertices on face line "<<lineno<<" = "<<line);
        in.setstate(ios::badbit);
        return false;
      }
      for(int i=0;i<nv;i++) {
        int v;
        ss>>v;
        if(v < 0 || v >= int(tri.verts.size())) {
          LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOFF: invalid vertex reference on line "<<lineno<<" = "<<line);
          return false;
        }
        face.push_back(v);
      }
      if(ss.bad()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOFF: incorrect number of vertices on line "<<lineno<<" = "<<line);
        in.setstate(ios::badbit);
        return false;
      }
      //triangulate faces, assume convex
      for(int i=1;i+1<nv;i++) {
        tri.tris.push_back(IntTriple(face[0],face[i],face[i+1]));
      }
            //get optional RGBA colors
      GLColor color;
      color.rgba[3] = 1;
      if(ss >> color.rgba[0] >> color.rgba[1] >> color.rgba[2]) {
        color.rgba[0] /= 255.0f;
        color.rgba[1] /= 255.0f;
        color.rgba[2] /= 255.0f;
        //RGBA color specified
        while(faceColors.size() <= tri.tris.size()) {
          faceColors.push_back(color);
        }
      }

      faceIndex++;
      if(faceIndex == numFaces) return true;
    }
  }
  if(faceIndex == numFaces) return true;
    LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOFF: Unexpected end of file on line "<<lineno<<", face "<<faceIndex<<" / "<<numFaces);
  return false;
}

bool LoadOFF(std::istream& in,TriMesh& tri)
{
  vector<GLDraw::GLColor> faceColors;
  return LoadOFF(in,tri,faceColors);
}

bool LoadOFF(std::istream& in,TriMesh& tri,GLDraw::GeometryAppearance& app)
{
  if(!LoadOFF(in,tri,app.faceColors)) return false;
  //if all face colors are the same, then set the appearance color
  if(app.faceColors.size() > 0) {
    bool all_same = true;
    for(size_t i=1;i<app.faceColors.size();i++) {
      if(app.faceColors[i] != app.faceColors[0]) {
        all_same = false;
      }
    }
    if(all_same) {
      app.faceColor = app.faceColors[0];
      app.faceColors.resize(0); //clear the per-face colors
    }
  }
  return true;
}


///Saves to the GeomView Object File Format (OFF)
bool SaveOFF(std::ostream& out,const TriMesh& tri)
{
  out<<"OFF"<<endl;
  out<<tri.verts.size()<<" "<<tri.tris.size()<<" 0"<<endl;
  for(size_t i=0;i<tri.verts.size();i++)
    out<<tri.verts[i]<<endl;
  for(size_t i=0;i<tri.tris.size();i++)
    out<<"3  "<<tri.tris[i]<<endl;
  return true;
}

///Saves to the GeomView Object File Format (OFF)
bool SaveOFF(std::ostream& out,const TriMesh& tri,const GeometryAppearance& app)
{
  out<<"OFF"<<endl;
  out<<tri.verts.size()<<" "<<tri.tris.size()<<" 0"<<endl;
  for(size_t i=0;i<tri.verts.size();i++)
    out<<tri.verts[i]<<endl;
  for(size_t i=0;i<tri.tris.size();i++) {
    const GLColor& c = app.faceColors.empty() ? app.faceColor : app.faceColors[i];
    out<<"3  "<<tri.tris[i]<<"   "<<int(c.rgba[0]*255.0)<<" "<<int(c.rgba[1]*255.0)<<" "<<int(c.rgba[2]*255.0)<<endl;
  }
  return true;
}


//scans until endc is read
bool fscanto(FILE* f, char endc)
{
  //read until end of line
  int c;
  while(true) {
    c = fgetc(f);
    if(c == EOF) return false;
    if(c == endc) return true;
  }
  return false;
}

//scans until end of line is read
bool fgetline(FILE* f, char* buf,int bufsize)
{
  //read until end of line
  int c;
  int i=0;
  while(true) {
    c = fgetc(f);
    if(c == EOF) {
      buf[i] = 0;
      return false;
    }
    if(c == '\n') {
      buf[i] = 0;
      return true;
    }
    buf[i] = c;
    i++;
  }
  return false;
}

bool LoadOBJMaterial(const char* path,const char* file,GeometryAppearance& app)
{
  string fn = string(path)+string(file);
  SimpleFile sf;
  if(!sf.Load(fn.c_str())) return false;
  if(sf.entries.count("Kd") != 0) {
    GLColor diffuse;
    diffuse.rgba[3] = 1;
    for(size_t i=0;i<sf.entries["Kd"].size();i++) {
      diffuse.rgba[i] = (float)sf.entries["Kd"][i].AsDouble();
    }
    app.SetColor(diffuse);
  }
  if(sf.entries.count("Ks") != 0) {
    GLColor specular;
    specular.rgba[3] = 1;
    for(size_t i=0;i<sf.entries["Ks"].size();i++) {
      specular.rgba[i] = (float)sf.entries["Ks"][i].AsDouble();
    }
    app.specularColor = specular;
  }
  if(sf.entries.count("Ka") != 0) {
    GLColor emissive;
    emissive.rgba[3] = 1;
    for(size_t i=0;i<sf.entries["Ka"].size();i++) {
      emissive.rgba[i] = (float)sf.entries["Ka"][i].AsDouble();
    }
    app.emissiveColor = emissive;
  }
  if(sf.entries.count("d") != 0) {  //opacity
    app.faceColor.rgba[3] = (float)sf.entries["d"][0].AsDouble();
    app.vertexColor.rgba[3] = app.faceColor.rgba[3];
  }
  if(sf.entries.count("Ns") != 0) {
    app.shininess = (float)sf.entries["Ns"][0].AsDouble();
  }
  if(sf.entries.count("map_Kd") != 0) {
    string textureMap = sf.entries["map_Kd"][0].AsString();
    string fullpath = string(path)+Strip(textureMap);
    app.tex2D.reset(new Image);
    if(!ImportImage(fullpath.c_str(),*app.tex2D)) {
      app.tex2D = NULL;
      LOG4CXX_INFO(KrisLibrary::logger(),"Unable to load image file "<<fullpath);
      return false;
    }
  }
  return true;
}

///Loads from the Wavefront OBJ format, with per-vertex colors?
bool LoadOBJ(const char* fn,FILE* f,TriMesh& tri,GeometryAppearance& app)
{
  setlocale(LC_NUMERIC, "en_US.UTF-8");
  tri.verts.resize(0);
  tri.tris.resize(0);
  vector<IntTriple> tri_texcoord;
  app.vertexColors.resize(0);
  char buf[1025];
  buf[1024]=0;
  string line,item,vmf;
  vector<char*> elements;
  vector<int> face;
  vector<int> face_texcoord;
  int lineno = 0;
  Vector3 pt;
  GLColor col;
  col.rgba[3] = 1;
  int c;
  while((c=fgetc(f)) != EOF) {
    lineno++;
    if(c == '\n') continue;
    else if(c == '#') fscanto(f,'\n');
    else if(c == 'v') {
      c = fgetc(f);
      if(c == 'n')  fscanto(f,'\n'); ///normal
      else if(c == 't') { //texture coordinate
        int n=fscanf(f," %lf %lf",&pt.x,&pt.y);
        if(n != 2) {
          LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOBJ: erroneous vt line on line "<<lineno);
          return false;
        }
        app.texcoords.push_back(Vector2(pt.x,pt.y));
      }
      else if(isspace(c)) { ///just vertex
        int n=fscanf(f," %lf %lf %lf",&pt.x,&pt.y,&pt.z);
        if(n != 3) {
          LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOBJ: erroneous v line on line "<<lineno);
          return false;
        }
        tri.verts.push_back(pt);
        fgetline(f,buf,1024);
        n=sscanf(buf,"%f %f %f %f",&col.rgba[0],&col.rgba[1],&col.rgba[2],&col.rgba[3]);
        if(n == 3) col.rgba[3]=1;
        if(n >= 3) {
          app.vertexColors.push_back(col);
          if(app.vertexColors.size() != tri.verts.size()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOBJ: number of vertex colors not equal to number of vertices\n");
            return false;
          }
        }
      }
    }
    else if(c=='f') {
      if(tri.tris.empty()) tri.tris.reserve(tri.verts.size()/3);
      if(tri_texcoord.empty()) tri_texcoord.reserve(tri.tris.size());
      face.resize(0);
      face_texcoord.resize(0);
      elements.resize(0);
      fgetline(f,buf,1024);
      bool readingspace = true;
      for(int i=0;i<1024;i++) {
        if(!buf[i]) break;
        if(readingspace) {
          if(isspace(buf[i])) buf[i]=0;
          else {
            readingspace = false;
            elements.push_back(&buf[i]);
          }
        }
        else {
          if(isspace(buf[i])) {
            buf[i]=0;
            readingspace = true;
          }
        }
      }
      //elements can be of form %d, %d/%d, or %d/%d/%d.
      //We only care about the first and possibly the second
      face.resize(elements.size());
      face_texcoord.resize(elements.size(),-1);
      int f;
      for(size_t i=0;i<elements.size();i++) {
        char* c = elements[i];
        char* ctexcoord = NULL;
        while(*c) { if(*c=='/') { *c=0; ctexcoord=c+1; break; }c++; }
        int n=sscanf(elements[i],"%d",&f);
        if(n != 1) {
          LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOBJ: invalid vertex on f line "<<lineno<<", element "<<i);
          return false;
        }
        f-=1;   //1 based
        if(f < 0 || f >= (int)tri.verts.size()) {
          LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOBJ: vertex "<<f<<" on f line "<<lineno<<" is out of bounds 0,...,"<<(int)tri.verts.size());
          return false;
        }
        face[i] = f;
        if(ctexcoord) {
          c=ctexcoord;
          while(*c) { if(*c=='/') { *c=0; break; }c++; }
          int n=sscanf(ctexcoord,"%d",&f);
          if(n != 1) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOBJ: invalid texcoord index on f line "<<lineno<<", element "<<i);
            return false;
          }
          f-=1;   //1 based
          if(f < 0 || f >= (int)app.texcoords.size()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOBJ: texcoord index "<<f<<" on f line "<<lineno<<" is out of bounds 0,...,"<<(int)app.texcoords.size());
            return false;
          }
          face_texcoord[i] = f;
        }
      }
      if(face.size() < 3) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOBJ: invalid f line "<<lineno);
        return false;
      }
      for(size_t i=2;i<face.size();i++)
        tri.tris.push_back(IntTriple(face[0],face[i-1],face[i]));
      if(face_texcoord[0] >= 0) {
        for(size_t i=2;i<face.size();i++)
          tri_texcoord.push_back(IntTriple(face_texcoord[0],face_texcoord[i-1],face_texcoord[i]));
      }
    }
    else if(c=='m') {
      int i=0;
      while(!isspace(c) && i < 63) {
        buf[i] = c;
        i++;
        c = fgetc(f);
      }
      if(i>=63) {
        buf[64]=0;
        LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOBJ: unsupported command \""<<buf<<"\" on line "<<lineno);
        return false;
      }
      buf[i]=0;
      if(0==strcmp(buf,"mtllib")) {
        fgetline(f,buf,1024);
        char* path = new char [strlen(fn)];
        GetFilePath(fn,path);
        if(!LoadOBJMaterial(path,buf,app)) {
          LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOBJ: error loading material file \""<<(string(path)+"/"+string(buf))<<"\" on line "<<lineno);
          return false;
        }
      }
      else {
        LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOBJ: unsupported command \""<<buf<<"\" on line "<<lineno);
        return false;
      }
    }
    else {
      LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOBJ: unsupported start character \""<<c<<"\" on line "<<lineno);
      return false;
    }
  }
  LOG4CXX_ERROR(KrisLibrary::logger(),"LoadOBJ: Read "<<tri.verts.size()<<" vertices, "<<app.texcoords.size()<<" texcoords, "<<app.vertexColors.size()<<" vertex colors");
  if(!tri_texcoord.empty()) {
    //need to remap per-vertex-face texcoords to per-vertex texcoords
    vector<Vector2> merged_texcoords(tri.verts.size(),Vector2(0.0));
    vector<int> texcoord_count(tri.verts.size(),0);
    for(size_t i=0;i<tri_texcoord.size();i++) {
      const IntTriple& t=tri.tris[i];
      const IntTriple& tx=tri_texcoord[i];
      /*
      texcoord_count[t.a] += 1;
      texcoord_count[t.b] += 1;
      texcoord_count[t.c] += 1;
      merged_texcoords[t.a] += app.texcoords[tx.a];
      merged_texcoords[t.b] += app.texcoords[tx.b];
      merged_texcoords[t.c] += app.texcoords[tx.c];
      */
      texcoord_count[t.a] = 1;
      texcoord_count[t.b] = 1;
      texcoord_count[t.c] = 1;
      merged_texcoords[t.a] = app.texcoords[tx.a];
      merged_texcoords[t.b] = app.texcoords[tx.b];
      merged_texcoords[t.c] = app.texcoords[tx.c];
    }
    for(size_t i=0;i<tri.verts.size();i++)
      if(texcoord_count[i] > 1)
        merged_texcoords[i] /= texcoord_count[i];
    app.texcoords = merged_texcoords;
    app.faceColor.set(1,1,1,app.faceColor.rgba[3]);
  }
  return true;
}

bool LoadOBJ(const char* fn,TriMesh& tri)
{
  GeometryAppearance app;
  return LoadOBJ(fn,tri,app);
}

bool LoadOBJ(const char* fn,TriMesh& tri,GeometryAppearance& app)
{
  FILE* f=fopen(fn,"r");
  if(!f) return false;
  bool res = LoadOBJ(fn,f,tri,app);
  fclose(f);
  return res;
}

bool SaveOBJ(const char* fn,const TriMesh& tri)
{
  FILE* f=fopen(fn,"w");
  if(!f) return false;
  fprintf(f,"#Written by KrisLibrary TriMesh exporter. %d vertices and %d faces\n",(int)tri.verts.size(),(int)tri.tris.size());
  for(size_t i=0;i<tri.verts.size();i++)
    fprintf(f,"v %f %f %f\n",tri.verts[i].x,tri.verts[i].y,tri.verts[i].z);
  for(size_t i=0;i<tri.tris.size();i++)
    fprintf(f,"f %d %d %d\n",tri.tris[i].a+1,tri.tris[i].b+1,tri.tris[i].c+1);
  fclose(f);
  return true;
}

bool SaveOBJ(const char* fn,const TriMesh& tri,const GeometryAppearance& app)
{
  if(!app.vertexColors.empty()) {
    Assert(app.vertexColors.size() == tri.verts.size());
    FILE* f=fopen(fn,"w");
    if(!f) return false;
    fprintf(f,"#Written by KrisLibrary TriMesh exporter. %d vertices and %d faces\n",(int)tri.verts.size(),(int)tri.tris.size());
    for(size_t i=0;i<tri.verts.size();i++)
      fprintf(f,"v %f %f %f %f %f %f\n",tri.verts[i].x,tri.verts[i].y,tri.verts[i].z,app.vertexColors[i].rgba[0],app.vertexColors[i].rgba[1],app.vertexColors[i].rgba[2]);
    for(size_t i=0;i<tri.tris.size();i++)
      fprintf(f,"f %d %d %d\n",tri.tris[i].a+1,tri.tris[i].b+1,tri.tris[i].c+1);
    fclose(f);
    return true;
  }
  else {
    LOG4CXX_WARN(KrisLibrary::logger(),"SaveOBJ: Can't save materials yet");
    return SaveOBJ(fn,tri);
  }
}


#if HAVE_ASSIMP


bool LoadAssimp(const char* fn, TriMesh& mesh)
{
        vector<TriMesh> models;
        if(!LoadAssimp(fn,models)) return false;
        mesh.Union(models);
        LOG4CXX_INFO(KrisLibrary::logger(),"LoadAssimp: Loaded model "<<fn<<" ("<<mesh.verts.size()<<" verts, "<<mesh.tris.size()<<" tris)");
        return true;
}


bool LoadAssimp(const char* fn, TriMesh& mesh, GeometryAppearance& app)
{
  vector<TriMesh> models;
  vector<GeometryAppearance> apps;
  if(!LoadAssimp(fn,models,apps)) return false;
  mesh.Union(models);
  if(!apps.empty()) {
    //need to merge appearance information
    app = apps[0];
    size_t numVerts = models[0].verts.size();
    size_t numTris = models[0].tris.size();
    for(size_t i=1;i<apps.size();i++) {
      if(app.texcoords.empty() != apps[i].texcoords.empty()) {
        if(app.texcoords.empty()) {
          app.texcoords.resize(numVerts,Vector2(0.0));
        }
        else {
          apps[i].texcoords.resize(models[i].verts.size(),Vector2(0.0));
        }
      }
      if(!app.texcoords.empty())
        app.texcoords.insert(app.texcoords.end(),apps[i].texcoords.begin(),apps[i].texcoords.end());
      if(app.tex2D != apps[i].tex2D) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"LoadAssimp: Warning, merging textured / non textured surfaces");
        if(app.tex2D == NULL)
          app.tex2D = apps[i].tex2D;
      }
      if(app.tex1D != apps[i].tex1D) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"LoadAssimp: Warning, merging textured / non textured surfaces");
        if(app.tex1D == NULL)
          app.tex1D = apps[i].tex1D;
      }
      if(app.vertexColors.empty() != apps[i].vertexColors.empty()) {
        if(app.vertexColors.empty())
          app.vertexColors.resize(numVerts,app.vertexColor);
        else
          apps[i].vertexColors.resize(models[i].verts.size(),apps[i].vertexColor);
      }
      if(!app.vertexColors.empty())
        app.vertexColors.insert(app.vertexColors.end(),apps[i].vertexColors.begin(),apps[i].vertexColors.end());
      if(app.faceColors.empty() != apps[i].faceColors.empty()) {
        if(app.faceColors.empty())
          app.faceColors.resize(numTris,app.faceColor);
        else
          apps[i].faceColors.resize(models[i].tris.size(),apps[i].faceColor);
      }
      if(!app.faceColors.empty())
        app.faceColors.insert(app.faceColors.end(),apps[i].faceColors.begin(),apps[i].faceColors.end());
      else {
        if(app.faceColor != apps[i].faceColor) {
          //need to construct per-face colors
          app.faceColors.resize(numTris,app.faceColor);
          app.faceColors.resize(numTris+models[i].tris.size(),apps[i].faceColor);
        }
      }
      numVerts += models[i].verts.size();
      numTris += models[i].tris.size();
    }
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"LoadAssimp: Loaded model "<<fn<<" ("<<mesh.verts.size()<<" verts, "<<mesh.tris.size()<<" tris)");
  return true;
}

void Cast(const aiMatrix4x4& a,Matrix4& out)
{
  out(0,0) = a.a1;
  out(0,1) = a.a2;
  out(0,2) = a.a3;
  out(0,3) = a.a4;
  out(1,0) = a.b1;
  out(1,1) = a.b2;
  out(1,2) = a.b3;
  out(1,3) = a.b4;
  out(2,0) = a.c1;
  out(2,1) = a.c2;
  out(2,2) = a.c3;
  out(2,3) = a.c4;
  out(3,0) = a.d1;
  out(3,1) = a.d2;
  out(3,2) = a.d3;
  out(3,3) = a.d4;
}

void AssimpMaterialToAppearance(aiMaterial* mat,const aiMesh* mesh,const aiScene* scene,GeometryAppearance& app)
{
  if(mat->GetName().length == 0 || mat->GetName() == aiString(AI_DEFAULT_MATERIAL_NAME))
    return;
  if(mesh->mColors[0]) {
    //per-vertex coloring
    app.vertexColors.resize(mesh->mNumVertices);
    for(unsigned int i=0;i<mesh->mNumVertices;i++)
      app.vertexColors[i].set(mesh->mColors[0][i].r,
                              mesh->mColors[0][i].g,
                              mesh->mColors[0][i].b,
                              mesh->mColors[0][i].a);
  }
  if(mesh->mTextureCoords[0]) {
    //texture coordinates
    app.texcoords.resize(mesh->mNumVertices);
    for(unsigned int i=0;i<mesh->mNumVertices;i++)
      app.texcoords[i].set(mesh->mTextureCoords[0][i].x,
                            mesh->mTextureCoords[0][i].y);
  }
  aiColor4D col;
  if(aiGetMaterialColor(mat,AI_MATKEY_COLOR_DIFFUSE,&col) == aiReturn_SUCCESS) {
    app.faceColor.set(col.r,col.g,col.b,1.0);
  }
  if(aiGetMaterialColor(mat,AI_MATKEY_COLOR_EMISSIVE,&col) == aiReturn_SUCCESS) {
    if(col.r == 1 && col.g == 1 && col.b == 1)
      app.lightFaces = false;
    app.emissiveColor.set(col.r,col.g,col.b,1.0);
  }
  if(aiGetMaterialFloat(mat,AI_MATKEY_SHININESS,&app.shininess) == aiReturn_SUCCESS) {
    //read in already
  }
  else {
    app.shininess = 0;
  }
  if(aiGetMaterialColor(mat,AI_MATKEY_COLOR_SPECULAR,&col) == aiReturn_SUCCESS) {
    app.specularColor.set(col.r,col.g,col.b,1.0);
  }
  float opacity = 1.0;
  if(aiGetMaterialFloat(mat,AI_MATKEY_OPACITY,&opacity) == aiReturn_SUCCESS) {
    if(opacity == 0) //interpret as transparency?
      opacity = 1.0;
    app.faceColor.rgba[3] = opacity;
  }
  #ifdef AI_MATKEY_TRANSPARENCYFACTOR
  if(aiGetMaterialFloat(mat,AI_MATKEY_TRANSPARENCYFACTOR,&opacity) == aiReturn_SUCCESS) {
    app.faceColor.rgba[3] = 1.0-opacity;
  }
  #endif // AI_MATKEY_TRANSPARENCYFACTOR
  aiString str;
  if(aiGetMaterialString(mat,AI_MATKEY_TEXTURE_DIFFUSE(0),&str) == aiReturn_SUCCESS) {
    if(str.data[0] == '*') {
      //this is an embedded texture
      int index=atoi(&str.data[1]);
      if(index < 0 || index >= (int)scene->mNumTextures) {
        LOG4CXX_INFO(KrisLibrary::logger(),"AssimpMaterialToAppearance: invalid texture "<<str.data);
      }
      else {
        aiTexture* tex = scene->mTextures[index];
        shared_ptr<Image> img(new Image);
        if(tex->mHeight == 0) {
          if(0==strcmp(tex->achFormatHint,"jpg") || 0==strcmp(tex->achFormatHint,"png") || 0==strcmp(tex->achFormatHint,"tif") || 0==strcmp(tex->achFormatHint,"bmp")) {
            stringstream ss;
            ss<<"_assimp_img_temp."<<tex->achFormatHint;
            string tempfn = ss.str();
            FILE* out = fopen(tempfn.c_str(),"wb");
            fwrite(tex->pcData,tex->mWidth,1,out);
            fclose(out);
            if(ImportImage(tempfn.c_str(),*img)) {
              app.tex2D = img;
              FileUtils::Delete(tempfn.c_str());
            }
            else {
              LOG4CXX_INFO(KrisLibrary::logger(),"AssimpMaterialToAppearance: can't load compressed embedded texture, saved to "<<tempfn<<" for inspection");
            }
            ///this is a hack for facebook Habitat scenes
            app.lightFaces = false;
          }
          else
            LOG4CXX_INFO(KrisLibrary::logger(),"AssimpMaterialToAppearance: can't load compressed embedded textures yet, format is "<<tex->achFormatHint);
        }
        else {
          Image::PixelFormat format=Image::None;
          if(0==strcmp(tex->achFormatHint,"rgba8888"))
            format=Image::R8G8B8A8;
          else if(0==strcmp(tex->achFormatHint,"argb8888"))
            format=Image::R8G8B8A8;
          else if(0==strcmp(tex->achFormatHint,"rgba5650"))
            format=Image::R5G6B5;
          else
            LOG4CXX_INFO(KrisLibrary::logger(),"AssimpMaterialToAppearance: Don't know yet how to handle texture of type "<<tex->achFormatHint);
          if(format != Image::None) {
            img->initialize(tex->mWidth,tex->mHeight,format);
            memcpy(img->data,tex->pcData,4*tex->mWidth*tex->mHeight);
            app.tex2D = img;
          }
        }
      }
    }
    else {
      //string filename = gTexturePath+str.C_str();
      string filename = gTexturePath+Strip(string(str.data));
      shared_ptr<Image> img(new Image);
      if(ImportImage(filename.c_str(),*img)) {
        app.tex2D = img;
        //TODO detect whether the texture should replace or modulate the face color
        app.faceColor.set(1,1,1,app.faceColor[3]);
      }
      else {
        LOG4CXX_INFO(KrisLibrary::logger(),"AssimpMaterialToAppearance: couldn't load image "<<filename);
      }
    }
  }
}

bool WalkAssimpNodes(const char* fn,const aiScene* scene,const aiNode* node,const Matrix4& Tparent,vector<TriMesh>& models,vector<GeometryAppearance>& apps)
{
  //LOG4CXX_INFO(KrisLibrary::logger(),node->mName.C_Str()<<" transform: ");
  Matrix4 T;
  Cast(node->mTransformation,T);
  //LOG4CXX_INFO(KrisLibrary::logger(),T);
  T = Tparent * T;
  //LOG4CXX_INFO(KrisLibrary::logger(),"final: "<<T);
  //if(node->mNumMeshes != 0)
  //KrisLibrary::loggerWait();


  for (unsigned int i = 0; i < node->mNumMeshes; i++) {
    unsigned int m = node->mMeshes[i];
    if (scene->mMeshes && scene->mMeshes[m]) {
      if (scene->mMeshes[m]->HasFaces()) {
        int nfaces = scene->mMeshes[m]->mNumFaces;
        int nverts = scene->mMeshes[m]->mNumVertices;
        apps.resize(apps.size()+1);
        AssimpMaterialToAppearance(scene->mMaterials[scene->mMeshes[m]->mMaterialIndex],scene->mMeshes[m],scene,apps.back());
        models.resize(models.size()+1);
        models.back().tris.resize(nfaces);
        models.back().verts.resize(nverts);
        for (int j = 0; j < nfaces; j++) {
          const aiFace& face = scene->mMeshes[m]->mFaces[j];
          if(face.mNumIndices != 3)
            LOG4CXX_ERROR(KrisLibrary::logger(),"FATAL: the triangle mesh has a triangle with "<<face.mNumIndices<<" vertices");
          Assert(face.mNumIndices == 3);
          models.back().tris[j].set(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
        }
        for (int j = 0; j < nverts; j++) {
          const aiVector3D& vert = scene->mMeshes[m]->mVertices[j];
          T.mulPoint(Vector3((double)vert.x, (double)vert.y, (double)vert.z),models.back().verts[j]);
        }
        if(!models.back().IsValid()) {
          LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: the triangle mesh is invalid or has degenerate triangles.");
          LOG4CXX_ERROR(KrisLibrary::logger(),"Continuing may have unexpected results.");
          //KrisLibrary::loggerWait();
        }

      } else {
        LOG4CXX_WARN(KrisLibrary::logger(), "AssimpImporter: Warning, " << fn << ", mesh "<<m<<" has no faces");
      }
    } else {
      LOG4CXX_WARN(KrisLibrary::logger(), "AssimpImporter: Warning, " << fn << " has no mesh");
    }
  }
  for(unsigned int i=0;i<node->mNumChildren;i++)
    if(!WalkAssimpNodes(fn,scene,node->mChildren[i],T,models,apps)) return false;
  return true;
}

bool LoadAssimp(const char* fn, vector<TriMesh>& models)
{
  vector<GeometryAppearance> apps;
  return LoadAssimp(fn,models,apps);
}

bool LoadAssimp(const char* fn, vector<TriMesh>& models,vector<GeometryAppearance>& apps)
{
  //setup texture path to same directory as fn
  char* buf = new char[strlen(fn)+1];
  GetFilePath(fn,buf);
  gTexturePath = buf;
  delete [] buf;
  
  Assimp::Importer importer;
  importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_LINE | aiPrimitiveType_POINT);
  importer.SetPropertyInteger(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, 1);
  const aiScene* scene = importer.ReadFile(fn, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_FindDegenerates | aiProcess_SortByPType | aiProcess_GenUVCoords );
  // If the import failed, report it
  if (!scene) {
    LOG4CXX_ERROR(KrisLibrary::logger(), "AssimpImporter error: "<<importer.GetErrorString() << " while loading "<< fn);
    return false;
  }
  if(scene->mNumMeshes == 0) {
    LOG4CXX_ERROR(KrisLibrary::logger(), "AssimpImporter: Error processing " << fn << "!");
    return false;
  }
  models.resize(0);
  int upAxis=2,upAxisSign=1,frontAxis=1,frontAxisSign=1,coordAxis=0,coordAxisSign=1;
  #if ASSIMP_MAJOR_VERSION>3
    /* for(int i=0;i<scene->mMetaData->mNumProperties;i++) {
      printf("ASSIMP: Property %s\n",scene->mMetaData->mKeys[i].C_Str());
    }
    */
    /* //some versions of assimp > 3 don't have mMetaData... this doesn't appear to be helpful anyways so ¯\_(ツ)_/¯
    if(scene->mMetaData){
      scene->mMetaData->Get<int>("UpAxis",upAxis);
      scene->mMetaData->Get<int>("UpAxisSign",upAxisSign);
      scene->mMetaData->Get<int>("FrontAxis",frontAxis);
      scene->mMetaData->Get<int>("FrontAxisSign",frontAxisSign);
      scene->mMetaData->Get<int>("CoordAxis",coordAxis);
      scene->mMetaData->Get<int>("CoordAxisSign",coordAxisSign);
    }
    */
  #endif //ASSIMP_MAJOR_VERSION>3
  if(upAxis == frontAxis || upAxis == coordAxis || frontAxis == coordAxis) {
    LOG4CXX_WARN(KrisLibrary::logger(), "AssimpImporter: Warning, axis metadata is erroneous in " << fn << "!");
    upAxis = 2;
    frontAxis = 1;
    coordAxis = 0;
  }
  if(abs(upAxisSign)!= 1 || abs(frontAxisSign)!= 1 || abs(coordAxisSign)!= 1) {
    LOG4CXX_WARN(KrisLibrary::logger(), "AssimpImporter: Warning, axis sign metadata is erroneous in " << fn << "!");
    upAxisSign = 1;
    frontAxisSign = 1;
    coordAxisSign = 1;
  }

  Matrix4 Troot; Troot.setZero();
  Troot(0,coordAxis) = coordAxisSign;
  Troot(1,frontAxis) = frontAxisSign;
  Troot(2,upAxis) = upAxisSign;
  Troot(3,3) = 1;
  if(!WalkAssimpNodes(fn,scene,scene->mRootNode,Troot,models,apps)) {
    LOG4CXX_ERROR(KrisLibrary::logger(), "AssimpImporter: Error Processing " << fn << "!");
    return false;
  }
  return true;
}

 
aiMesh* MeshToAssimp(const TriMesh& model)
{
  aiMesh* mesh = new aiMesh();
  mesh->mMaterialIndex = 0; //default material
  mesh->mVertices = new aiVector3D[ model.verts.size() ];
  mesh->mNumVertices = model.verts.size();

  for ( size_t i=0;i<model.verts.size();i++)
    mesh->mVertices[ i ] = aiVector3D( (float)model.verts[i].x, (float)model.verts[i].y, (float)model.verts[i].z );

  mesh->mFaces = new aiFace[ model.tris.size() ];
  mesh->mNumFaces = model.tris.size();

  mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
  for(size_t i=0;i<model.tris.size();i++) {
    aiFace& face = mesh->mFaces[i];
    face.mIndices = new unsigned int[ 3 ];
    face.mNumIndices = 3;

    face.mIndices[ 0 ] = model.tris[i][ 0 ];
    face.mIndices[ 1 ] = model.tris[i][ 1 ];
    face.mIndices[ 2 ] = model.tris[i][ 2 ];
  }
  return mesh;
}

bool SaveAssimp(const char* fn, const TriMesh& model)
{
  aiScene scene;

  scene.mMetaData = new aiMetadata();
  scene.mRootNode = new aiNode();

  scene.mMaterials = new aiMaterial*[ 1 ];
  scene.mNumMaterials = 1;
  scene.mMaterials[ 0 ] = new aiMaterial();

  scene.mMeshes = new aiMesh*[ 1 ];
  scene.mNumMeshes = 1;

  scene.mMeshes[ 0 ] = MeshToAssimp(model);
  scene.mRootNode->mMeshes = new unsigned int[ 1 ];
  scene.mRootNode->mMeshes[ 0 ] = 0;
  scene.mRootNode->mNumMeshes = 1;

  Assimp::Exporter exporter;
  const char* exporter_id = NULL;
  const char* ext = FileExtension(fn);
  for(int i=0;i<exporter.GetExportFormatCount();i++) {
    const aiExportFormatDesc* desc = exporter.GetExportFormatDescription(i);
    if(0==strcmp(desc->fileExtension,ext)) {
      exporter_id = desc->id;
      break;
    }
  }
  if(exporter_id == NULL) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"SaveAssimp: No exporter found for file extension "<<ext);
    return false;
  }
  auto res = exporter.Export(&scene,exporter_id,fn);
  if(res != AI_SUCCESS)
    LOG4CXX_WARN(KrisLibrary::logger(),"Assimp Exporter failed!  Error: "<<exporter.GetErrorString());
  return (res == AI_SUCCESS);
}

bool SaveAssimp(const char* fn, const TriMesh& model,const GeometryAppearance& app)
{
  aiScene scene;

  scene.mMetaData = new aiMetadata();
  scene.mRootNode = new aiNode();

  scene.mMaterials = new aiMaterial*[ 1 ];
  scene.mNumMaterials = 1;
  scene.mMaterials[ 0 ] = new aiMaterial();

  scene.mMeshes = new aiMesh*[ 1 ];
  scene.mNumMeshes = 1;

  scene.mMeshes[ 0 ] = MeshToAssimp(model);

  scene.mRootNode->mMeshes = new unsigned int[ 1 ];
  scene.mRootNode->mMeshes[ 0 ] = 0;
  scene.mRootNode->mNumMeshes = 1;


  if(!app.vertexColors.empty()) {
    auto pMesh = scene.mMeshes[ 0 ];
    pMesh->mColors[0] = new aiColor4D[ model.verts.size() ];
    for ( size_t i=0;i<model.verts.size();i++) {
      pMesh->mColors[0][i].r = app.vertexColors[i].rgba[0];
      pMesh->mColors[0][i].g = app.vertexColors[i].rgba[1];
      pMesh->mColors[0][i].b = app.vertexColors[i].rgba[2];
      pMesh->mColors[0][i].a = app.vertexColors[i].rgba[3];
    }
  }

  scene.mMaterials[0]->AddProperty( &app.faceColor.rgba, 1, AI_MATKEY_COLOR_DIFFUSE );

  if(!app.faceColors.empty()) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Can't export per-face colors yet");
  }
  if(!app.tex1D || !app.tex2D) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Can't export textures yet");
  }

  Assimp::Exporter exporter;
  const char* exporter_id = NULL;
  const char* ext = FileExtension(fn);
  for(int i=0;i<exporter.GetExportFormatCount();i++) {
    const aiExportFormatDesc* desc = exporter.GetExportFormatDescription(i);
    if(0==strcmp(desc->fileExtension,ext)) {
      exporter_id = desc->id;
      //break;
    }
  }
  if(exporter_id == NULL) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"SaveAssimp: No exporter found for file extension "<<ext);
    return false;
  }
  auto res = exporter.Export(&scene,exporter_id,fn);
  if(res != AI_SUCCESS)
    LOG4CXX_WARN(KrisLibrary::logger(),"Assimp Exporter failed!  Error: "<<exporter.GetErrorString());
  return (res == AI_SUCCESS);
}

bool SaveAssimp(const char* fn, const vector<TriMesh>& models,const vector<GeometryAppearance>& appearances)
{
  if(models.size() == 0) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"SaveAssimp: No models to save");
    return false;
  }
  aiScene scene;

  scene.mMetaData = new aiMetadata();
  scene.mRootNode = new aiNode();

  scene.mNumMaterials = Max(1,(int)appearances.size());
  scene.mMaterials = new aiMaterial*[ scene.mNumMaterials ];
  for(size_t i=0;i<scene.mNumMaterials;i++)
    scene.mMaterials[i] = new aiMaterial();

  scene.mMeshes = new aiMesh*[ models.size() ];
  scene.mNumMeshes = models.size();

  for(size_t i=0;i<models.size();i++) {
    scene.mMeshes[i] = MeshToAssimp(models[i]);
  }

  scene.mRootNode->mMeshes = new unsigned int[ models.size() ];
  for(size_t i=0;i<models.size();i++) {
    scene.mRootNode->mMeshes[i] = i;
  }
  scene.mRootNode->mNumMeshes = models.size();

  for(size_t i=0;i<models.size();i++) {
    if(i < appearances.size()) {
      const auto& app = appearances[i];
      if(!app.vertexColors.empty()) {
        auto pMesh = scene.mMeshes[i];
        pMesh->mColors[0] = new aiColor4D[ models[i].verts.size() ];
        for ( size_t j=0;j<models[i].verts.size();j++) {
          pMesh->mColors[0][j].r = app.vertexColors[j].rgba[0];
          pMesh->mColors[0][j].g = app.vertexColors[j].rgba[1];
          pMesh->mColors[0][j].b = app.vertexColors[j].rgba[2];
          pMesh->mColors[0][j].a = app.vertexColors[j].rgba[3];
        }
      }
      scene.mMaterials[i]->AddProperty( &app.faceColor.rgba, 1, AI_MATKEY_COLOR_DIFFUSE );

      if(!app.faceColors.empty()) {
        LOG4CXX_WARN(KrisLibrary::logger(),"Can't export per-face colors yet");
      }
      if(!app.tex1D || !app.tex2D) {
        LOG4CXX_WARN(KrisLibrary::logger(),"Can't export textures yet");
      }
    }
  }  
  
  Assimp::Exporter exporter;
  const char* exporter_id = NULL;
  const char* ext = FileExtension(fn);
  for(int i=0;i<exporter.GetExportFormatCount();i++) {
    const aiExportFormatDesc* desc = exporter.GetExportFormatDescription(i);
    if(0==strcmp(desc->fileExtension,ext)) {
      exporter_id = desc->id;
      break;
    }
  }
  if(exporter_id == NULL) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"SaveAssimp: No exporter found for file extension "<<ext);
    return false;
  }
  auto res = exporter.Export(&scene,exporter_id,fn);
  if(res != AI_SUCCESS)
    LOG4CXX_WARN(KrisLibrary::logger(),"Assimp Exporter failed!  Error: "<<exporter.GetErrorString());
  return (res == AI_SUCCESS);
}


#else

bool LoadAssimp(const char* fn, TriMesh& mesh)
{
  LOG4CXX_INFO(KrisLibrary::logger(),"No Assimp Importer defined!");
  return false;
}

bool LoadAssimp(const char* fn, vector<TriMesh>& meshes)
{
  LOG4CXX_INFO(KrisLibrary::logger(),"No Assimp Importer defined!");
  return false;
}

bool SaveAssimp(const char* fn, const TriMesh& mesh)
{
  LOG4CXX_INFO(KrisLibrary::logger(),"No Assimp Exporter defined!");
  return false;
}

bool SaveAssimp(const char* fn, const TriMesh& mesh,const GeometryAppearance& app)
{
  LOG4CXX_INFO(KrisLibrary::logger(),"No Assimp Exporter defined!");
  return false;
}

bool SaveAssimp(const char* fn, const vector<TriMesh>& meshes,const vector<GeometryAppearance>& appearances)
{
  LOG4CXX_INFO(KrisLibrary::logger(),"No Assimp Exporter defined!");
  return false;
}



#endif



} //namespace Meshing

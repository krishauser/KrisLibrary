#include "IO.h"
#include <utils/stringutils.h>
#include <sstream>
#include <fstream>
#include <string.h>
#include <errors.h>

#if HAVE_ASSIMP
#if ASSIMP_MAJOR_VERSION==2
#include <assimp/assimp.hpp>      // C++ importer interface
#include <assimp/aiScene.h>           // Output data structure
#include <assimp/aiPostProcess.h>     // Post processing flags
#else
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags
#endif //ASSIMP_MAJOR_VERSION
using namespace Assimp;
#endif //HAVE_ASSIMP


namespace Meshing {

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
#endif
  }
  return false;
}

///Returns true if the extension is a file type that we can save to 
bool CanSaveTriMeshExt(const char* ext)
{
  if(0==strcmp(ext,"tri")) return true;
  else if(0==strcmp(ext,"off")) return true;
  else {
#if HAVE_ASSIMP
    //TODO: check exporter
#endif
  }
  return false;
}


///Import will try to determine the file type via the file extension
bool Import(const char* fn,TriMesh& tri)
{
  const char* ext=FileExtension(fn);
  if(0==strcmp(ext,"tri")) {
    return LoadMultipleTriMeshes(fn,tri);
  }
  else {
#if HAVE_ASSIMP
    if(!LoadAssimp(fn,tri)) {
      fprintf(stderr,"Import(TriMesh): file %s could not be loaded\n",fn);
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
    else if(0==strcmp(ext,"off")) {
      ifstream in(fn,ios::in);
      if(!in) return false;
      return LoadOFF(in,tri);
    }
    else {
      fprintf(stderr,"Import(TriMesh): file extension %s not recognized\n",ext);
      return false;
    }
#endif
  }
}

bool Export(const char* fn,const TriMesh& tri)
{
  const char* ext=FileExtension(fn);
  if(0==strcmp(ext,"tri")) {
    ofstream out(fn,ios::out);
    if(!out) return false;
    out<<tri;
    return true;
  }
  else {
#if HAVE_ASSIMP
    if(!SaveAssimp(fn,tri)) {
      fprintf(stderr,"Export(TriMesh): file %s could not be saved to type %s\n",fn,ext);
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
    else if(0==strcmp(ext,"off")) {
      ofstream out(fn,ios::out);
      if(!out) return false;
      return SaveOFF(out,tri);
    }
    else {
      fprintf(stderr,"Export(TriMesh): file extension %s not recognized\n",ext);
      return false;
    }
#endif
  }
}

///Loads from VRML file format
bool LoadVRML(std::istream& in,TriMesh& tri)
{
  fprintf(stderr,"LoadVRML not implemented yet\n");
  return false;
}

///Saves to VRML file format
bool SaveVRML(std::ostream& out,const TriMesh& tri)
{
  fprintf(stderr,"SaveVRML not implemented yet\n");
  return false;
}

///Loads from the GeomView Object File Format (OFF)
bool LoadOFF(std::istream& in,TriMesh& tri)
{
  string tag;
  in>>tag;
  if(tag != "OFF") {
    fprintf(stderr,"LoadOFF: not a proper OFF file\n");
    return false;
  }
  int mode = 0; //0: waiting for sizes, 1: reading verts, 2: reading tris
  int numFaces = 0;
  int vertIndex=0,faceIndex=0;
  string line;
  while(in) {
    getline(in,line);
    if(in.bad()) return false;
    if(line.length() == 0) continue;
    if(line[0] == '#') continue; //comment line
    if(mode == 0) {
      stringstream ss(line);
      int nv,nf,ne;
      ss>>nv>>nf>>ne;
      if(ss.bad()) {
	in.setstate(ios::badbit);
	return false;
      }
      tri.verts.resize(nv);
      tri.tris.resize(0);
      numFaces = nf;
      mode = 1;
    }
    if(mode == 1) {
      if((int)tri.verts.size() == vertIndex) mode=2;
      else {
	stringstream ss(line);
	ss >> tri.verts[vertIndex];
	if(ss.bad()) {
	  in.setstate(ios::badbit);
	  return false;
	}
	//ignore RGBA colors
	vertIndex++;
      }
    }
    if(mode == 2) {
      if(faceIndex == numFaces) return true;
      else {
	stringstream ss(line);
	int nv;
	vector<int> face;
	ss >> nv;
	if(ss.bad()) {
	  in.setstate(ios::badbit);
	  return false;
	}
	for(int i=0;i<nv;i++) {
	  int v;
	  ss>>v;
	  face.push_back(v);
	}
	if(ss.bad()) {
	  in.setstate(ios::badbit);
	  return false;
	}
	//ignore RGBA colors

	//triangulate faces, assume convex
	for(int i=1;i+1<nv;i++) {
	  tri.tris.push_back(IntTriple(face[0],face[i],face[i+1]));
	}

	faceIndex++;
      }
    }
  }
  return false;
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



#if HAVE_ASSIMP


bool LoadAssimp(const char* fn, TriMesh& mesh)
{
	vector<TriMesh> models;
	if(!LoadAssimp(fn,models)) return false;
	mesh.Merge(models);
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

bool WalkAssimpNodes(const char* fn,const aiScene* scene,const aiNode* node,const Matrix4& Tparent,vector<TriMesh>& models)
{
  //cout<<node->mName.C_Str()<<" transform: "<<endl;
  Matrix4 T;
  Cast(node->mTransformation,T);
  //cout<<T<<endl;
  T = Tparent * T;
  //cout<<"final: "<<T<<endl;
  //if(node->mNumMeshes != 0)
  //getchar();


  for (unsigned int i = 0; i < node->mNumMeshes; i++) {
    unsigned int m = node->mMeshes[i];
    if (scene->mMeshes && scene->mMeshes[m]) {
      if (scene->mMeshes[m]->HasFaces()) {
	int nfaces = scene->mMeshes[m]->mNumFaces;
	int nverts = scene->mMeshes[m]->mNumVertices;
	models.resize(models.size()+1);
	models.back().tris.resize(nfaces);
	models.back().verts.resize(nverts);
	for (int j = 0; j < nfaces; j++) {
	  const aiFace& face = scene->mMeshes[m]->mFaces[j];
	  Assert(face.mNumIndices == 3);
	  models.back().tris[j].set(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
	}
	for (int j = 0; j < nverts; j++) {
	  const aiVector3D& vert = scene->mMeshes[m]->mVertices[j];
	  T.mulPoint(Vector3((double)vert.x, (double)vert.y, (double)vert.z),models.back().verts[j]);
	}
	if(!models.back().IsValid()) {
	  cerr<<"Warning: the triangle mesh is invalid or has degenerate triangles."<<endl;
	  cerr<<"Continuing may have unexpected results."<<endl;
	  cerr<<"Press enter to continue."<<endl;
	  getchar();
	}
	
      } else {
	cout << "AssimpImporter: Warning, " << fn << ", mesh "<<m<<" has no faces" << endl;
      }
    } else {
      cout << "AssimpImporter: Warning, " << fn << " has no mesh" << endl;
    }
  }
  for(unsigned int i=0;i<node->mNumChildren;i++)
    if(!WalkAssimpNodes(fn,scene,node->mChildren[i],T,models)) return false;
  return true;
}

bool LoadAssimp(const char* fn, vector<TriMesh>& models)
{
	Assimp::Importer importer;
	const aiScene* scene = importer.ReadFile(fn, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
	// If the import failed, report it
	if (!scene || scene->mNumMeshes == 0) {
		std::cout << "AssimpImporter:"<<importer.GetErrorString() << std::endl;
		cout << "AssimpImporter:"<<"Error Processing " << fn << "!" << endl;
		return false;
	}
	models.resize(0);
	Matrix4 Tident; Tident.setIdentity();
	/*
	Matrix4 Tyz; 
	Tyz.setZero();
	Tyz(0,0) = 1;
	Tyz(1,2) = -1;
	Tyz(2,1) = 1;
	Tyz(3,3) = 1;
	*/
	//if(!WalkAssimpNodes(fn,scene,scene->mRootNode,Tyz,models)) {
	if(!WalkAssimpNodes(fn,scene,scene->mRootNode,Tident,models)) {
	  cout << "AssimpImporter:"<<"Error Processing " << fn << "!" << endl;
	  return false;
	}
	return true;
}

bool SaveAssimp(const char* fn, const TriMesh& model)
{
  cout<<"Assimp saving not defined yet"<<endl;
  return false;
}

#else

bool LoadAssimp(const char* fn, TriMesh& mesh)
{
	cout<<"No Assimp Importer defined!"<<endl;
	return false;
}

bool LoadAssimp(const char* fn, vector<TriMesh>& meshes)
{
	cout<<"No Assimp Importer defined!"<<endl;
	return false;
}

bool SaveAssimp(const char* fn, const TriMesh& mesh)
{
	cout<<"No Assimp Exporter defined!"<<endl;
	return false;
}


#endif



} //namespace Meshing

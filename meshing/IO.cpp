#include "IO.h"
#include <utils/AnyValue.h>
#include <utils/stringutils.h>
#include <GLdraw/GeometryAppearance.h>
#include <image/import.h>
#include <iostream>
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

using namespace std;
using namespace Math3D;
using namespace GLDraw;

namespace Meshing {

static string gTexturePath;
bool LoadOBJ(FILE* f,TriMesh& tri,GeometryAppearance& app);

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

///Import will try to determine the file type via the file extension
bool Import(const char* fn,TriMesh& tri,GeometryAppearance& app)
{
  const char* ext=FileExtension(fn);
  if(0==strcmp(ext,"tri")) {
    return LoadMultipleTriMeshes(fn,tri);
  }
  else {
    if(0==strcmp(ext,"obj")) {
      FILE* f = fopen(fn,"r");
      if(f && LoadOBJ(f,tri,app)) return true;
    }
#if HAVE_ASSIMP
    //setup texture path to same directory as fn
    char* buf = new char[strlen(fn)+1];
    GetFilePath(fn,buf);
    gTexturePath = buf;
    delete [] buf;
    //do the loading
    if(!LoadAssimp(fn,tri,app)) {
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

bool Export(const char* fn,const TriMesh& tri,const GeometryAppearance& app)
{
  return Export(fn,tri);
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
bool LoadOFF(std::istream& in,TriMesh& tri,GeometryAppearance& app)
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


///Loads from the Wavefront OBJ format, with per-vertex colors?
bool LoadOBJ(FILE* f,TriMesh& tri,GeometryAppearance& app)
{
  tri.verts.resize(0);
  tri.tris.resize(0);
  app.vertexColors.resize(0);
  char buf[1025];
  buf[1024]=0;
  string line,item,vmf;
  vector<char*> elements;
  vector<int> face;
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
      if(c == 'n')  fscanto(f,'\n');
      else if(isspace(c)) { ///just vertex
	int n=fscanf(f," %lf %lf %lf",&pt.x,&pt.y,&pt.z);
	if(n != 3) {
	  fprintf(stderr,"LoadOBJ: erroneous v line on line %d\n",lineno);
	  return false;
	}
	tri.verts.push_back(pt);
	fgetline(f,buf,1024);
	n=sscanf(buf,"%f %f %f %f",&col.rgba[0],&col.rgba[1],&col.rgba[2],&col.rgba[3]);
	if(n == 3) col.rgba[3]=1;
	if(n >= 3) {
	  app.vertexColors.push_back(col);
	  if(app.vertexColors.size() != tri.verts.size()) {
	    fprintf(stderr,"LoadOBJ: number of vertex colors not equal to number of vertices\n");
	    return false;
	  }
	}
      }
    }
    else if(c=='f') {
      if(tri.tris.size()==0) tri.tris.reserve(tri.verts.size()/3);
      face.resize(0);
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
      //We only care about the first
      face.resize(elements.size());
      int f;
      for(size_t i=0;i<elements.size();i++) {
	char* c = elements[i];
	while(*c) { if(*c=='/') { *c=0; break; }c++; }
	int n=sscanf(elements[i],"%d",&f);
	if(n != 1) {
	  fprintf(stderr,"LoadOBJ: invalid vertex on f line %d, element %d\n",lineno,i);
	  return false;
	}
	f-=1;   //1 based
	if(f < 0 || f >= (int)tri.verts.size()) {
	  fprintf(stderr,"LoadOBJ: vertex %d on f line %d is out of bounds 0,...,%d\n",f,lineno,tri.verts.size());
	  return false;
	}
	face[i] = f;
      }
      if(face.size() < 3) {
	fprintf(stderr,"LoadOBJ: invalid f line %d\n",lineno);
	return false;
      }
      for(size_t i=2;i<face.size();i++) 
	tri.tris.push_back(IntTriple(face[0],face[i-1],face[i]));
    }
    else {
      fprintf(stderr,"LoadOBJ: unsupported start character \"%c\" on line %d\n",c,lineno);
      return false;
    }
  }
  return true;
}



#if HAVE_ASSIMP


bool LoadAssimp(const char* fn, TriMesh& mesh)
{
	vector<TriMesh> models;
	if(!LoadAssimp(fn,models)) return false;
	mesh.Merge(models);
	cout<<"LoadAssimp: Loaded model with "<<mesh.verts.size()<<" verts and "<<mesh.tris.size()<<" tris"<<endl;
	return true;
}


bool LoadAssimp(const char* fn, TriMesh& mesh, GeometryAppearance& app)
{
	vector<TriMesh> models;
	vector<GeometryAppearance> apps;
	if(!LoadAssimp(fn,models,apps)) return false;
	mesh.Merge(models);
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
	      fprintf(stderr,"LoadAssimp: Warning, merging textured / non textured surfaces\n");
	      if(app.tex2D == NULL)
		app.tex2D = apps[i].tex2D;
	    }
	    if(app.tex1D != apps[i].tex1D) {
	      fprintf(stderr,"LoadAssimp: Warning, merging textured / non textured surfaces\n");
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
	cout<<"LoadAssimp: Loaded model with "<<mesh.verts.size()<<" verts and "<<mesh.tris.size()<<" tris"<<endl;
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

void AssimpMaterialToAppearance(const aiMaterial* mat,const aiMesh* mesh,GeometryAppearance& app)
{
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
    app.faceColor.set(col.r,col.g,col.b,col.a);
  }
  aiString str;
  if(aiGetMaterialString(mat,AI_MATKEY_TEXTURE_DIFFUSE(0),&str) == aiReturn_SUCCESS) {
    string filename = gTexturePath+str.C_Str();
    SmartPointer<Image> img = new Image;
    if(ImportImage(filename.c_str(),*img)) {
      app.tex2D = img;
    }
    else {
      printf("AssimpMaterialToAppearance: couldn't load image %s\n",filename.c_str());
    }
  }
}

bool WalkAssimpNodes(const char* fn,const aiScene* scene,const aiNode* node,const Matrix4& Tparent,vector<TriMesh>& models,vector<GeometryAppearance>& apps)
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
	apps.resize(apps.size()+1);
	AssimpMaterialToAppearance(scene->mMaterials[scene->mMeshes[m]->mMaterialIndex],scene->mMeshes[m],apps.back());
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
	if(!WalkAssimpNodes(fn,scene,scene->mRootNode,Tident,models,apps)) {
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

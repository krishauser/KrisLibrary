#include "IO.h"
#include <utils/stringutils.h>
#include <sstream>
#include <fstream>
#include <string.h>
#include <errors.h>

#if HAVE_ASSIMP
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

using namespace Assimp;
#endif


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

bool LoadAssimp(const char* fn, vector<TriMesh>& models)
{
	Assimp::Importer importer;
	const aiScene* scene = importer.ReadFile(fn, aiProcess_Triangulate);
	// If the import failed, report it
	if (!scene || scene->mNumMeshes == 0) {
		std::cout << "AssimpImporter:"<<importer.GetErrorString() << std::endl;
		cout << "AssimpImporter:"<<"Error Processing " << fn << "!" << endl;
		return false;
	}
	models.resize(scene->mNumMeshes);
	for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
		if (scene->mMeshes && scene->mMeshes[i]) {
			if (scene->mMeshes[i]->HasFaces()) {
				int nfaces = scene->mMeshes[i]->mNumFaces;
				int nverts = scene->mMeshes[i]->mNumVertices;
				models[i].tris.resize(nfaces);
				models[i].verts.resize(nverts);
				for (int j = 0; j < nfaces; j++) {
					const aiFace& face = scene->mMeshes[i]->mFaces[j];
					Assert(face.mNumIndices == 3);
					models[i].tris[j].set(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
				}
				for (int j = 0; j < nverts; j++) {
					const aiVector3D& vert = scene->mMeshes[i]->mVertices[j];
					models[i].verts[j].set((double)vert.x, (double)vert.y, (double)vert.z);
				}
			  if(!models[i].IsValid()) {
				cerr<<"Warning: the triangle mesh is invalid or has degenerate triangles."<<endl;
				cerr<<"Continuing may have unexpected results."<<endl;
				cerr<<"Press enter to continue."<<endl;
				getchar();
			  }

			} else {
				cout << "Error processing " << *fn << ", no faces!" << endl;
			}
		} else {
			cout << "Error processing " << *fn << ", no mesh!" << endl;
		}
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

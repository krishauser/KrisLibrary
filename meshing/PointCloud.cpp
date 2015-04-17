#include "PointCloud.h"
#include <math3d/AABB3D.h>
#include <utils/SimpleParser.h>
#include <utils/stringutils.h>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <string.h>

using namespace Meshing;

class PCLParser : public SimpleParser
{
public:
  enum {NORMAL, READING_FIELDS, READING_TYPES, READING_SIZES, READING_COUNTS };
  PCLParser(istream& in,PointCloud3D& _pc)
    :SimpleParser(in),pc(_pc),state(NORMAL),numPoints(-1)
  {}
  virtual bool IsComment(char c) const { return c=='#'; }
  virtual bool IsToken(char c) const { return !IsSpace(c) && !IsComment(c); }
  virtual bool IsPunct(char c) const { return !IsSpace(c) && !IsComment(c) && !IsToken(c); }
  virtual Result InputToken(const string& word) {
    if(state == READING_FIELDS) {
      pc.propertyNames.push_back(word);
    }
    else if(state == READING_TYPES) {
      if(word != "F" && word != "U" && word != "I") {
	fprintf(stderr,"PCD parser: Invalid PCD TYPE %s\n",word.c_str());
	return Error;
      }
      types.push_back(word);
    }
    else if(state == READING_COUNTS) {
      if(!IsValidInteger(word.c_str())) {
	fprintf(stderr,"PCD parser: Invalid PCD COUNT string %s, must be integer\n",word.c_str());
	return Error;
      }
      stringstream ss(word);
      int count;
      ss>>count;
      if(count != 1) {
	fprintf(stderr,"PCD parser: Invalid PCD COUNT %s, we only handle counts of 1\n",word.c_str());
	return Error;
      }
      counts.push_back(count);
    }
    else if(state == READING_SIZES) {
      if(!IsValidInteger(word.c_str())) {
	fprintf(stderr,"PCD parser: Invalid PCD SIZE string %s, must be integer\n",word.c_str());
	return Error;
      }
      stringstream ss(word);
      int size;
      ss>>size;
      if(size <= 0) {
	fprintf(stderr,"PCD parser: Invalid PCD SIZE %s, must be positive\n",word.c_str());
	return Error;
      }
      sizes.push_back(size);
    }
    else {
      if(word == "POINTS") {
	string points;
	ReadLine(points);
	stringstream ss(points);
	ss>>numPoints;
	if(!ss) {
	  fprintf(stderr,"PCD parser: Unable to read integer POINTS\n");
	  return Error;
	}
      }
      else if(word == "FIELDS") {
	state = READING_FIELDS;
      }
      else if(word == "COUNT") {
	state = READING_COUNTS;
      }
      else if(word == "TYPE") {
	state = READING_TYPES;
      }
      else if(word == "SIZE") {
	state = READING_SIZES;
      }
      else if(word == "DATA") {
	string datatype;
	if(!ReadLine(datatype)) return Error;
	datatype = Strip(datatype);
	if(datatype == "binary") {
	  //pull in the endline
	  int c = in.get();
	  assert(c == '\n');
	  //read in binary data
	  if(numPoints < 0) {
	    fprintf(stderr,"PCD parser: DATA specified before POINTS element\n");
	    return Error;
	  }
	  if(sizes.size() != pc.propertyNames.size()) {
	    fprintf(stderr,"PCD parser: Invalid number of SIZE elements\n");
	    return Error;
	  }
	  if(types.size() != pc.propertyNames.size()) {
	    fprintf(stderr,"PCD parser: Invalid number of TYPE elements\n");
	    return Error;
	  }
	  int pointsize = 0;
	  for(size_t i=0;i<sizes.size();i++)
	    pointsize += sizes[i];
	  vector<char> buffer(pointsize);
	  for(int i=0;i<numPoints;i++) {
	    in.read(&buffer[0],pointsize);
	    if(!in) {
	      fprintf(stderr,"PCD parser: Error reading data for point %d\n",i);
	      return Error;
	    }
	    //parse the point and add it
	    Vector v(pc.propertyNames.size());
	    int ofs = 0;
	    for(size_t j=0;j<sizes.size();j++) {
	      if(types[j] == "F") {
		if(sizes[j] == 4) {
		  float f;
		  memcpy(&f,&buffer[ofs],sizes[j]);
		  v[j] = f;
		}
		else if(sizes[j] == 8) {
		  double f;
		  memcpy(&f,&buffer[ofs],sizes[j]);
		  v[j] = f;
		}
		else {
		  fprintf(stderr,"PCD parser: Invalid float size %d\n",sizes[j]);
		  return Error;
		}
	      }
	      else if(types[j] == "U") {
		if(sizes[j] > 4) {
		  fprintf(stderr,"PCD parser: Invalid unsigned int size %d\n",sizes[j]);
		  return Error;		  
		}
		unsigned i=0;
		memcpy(&i,&buffer[ofs],sizes[j]);
		v[j] = Real(i);
	      }
	      else if(types[j] == "I") {
		fprintf(stderr,"PCD parser: Invalid int size %d\n",sizes[j]);
		int i=0;
		memcpy(&i,&buffer[ofs],sizes[j]);
		v[j] = Real(i);
	      }
	      else {
		fprintf(stderr,"PCD parser: Invalid type %s\n",types[i].c_str());
		return Error;
	      }
	      ofs += sizes[j];
	    }
	    pc.properties.push_back(v);
	  }
	}
	else if(datatype == "ascii") {
	  if(numPoints < 0) {
	    fprintf(stderr,"PCD parser: DATA specified before POINTS element\n");
	    return Error;
	  }
	  string line;
	  for(int i=0;i<numPoints;i++) {
	    int c = in.get();
	    assert(c=='\n' || c==EOF);
	    lineno++;
	    if(c==EOF) {
	      fprintf(stderr,"PCD parser: Premature end of DATA element\n");
	      return Error;
	    }
	    if(!ReadLine(line)) {
	      fprintf(stderr,"PCD parser: Error reading point %d\n",i);
	      return Error;
	    }
	    vector<string> elements = Split(line," ");
	    if(elements.size() != pc.propertyNames.size()) {
	      fprintf(stderr,"PCD parser: DATA element %d has length %d, but %d properties specified\n",i,elements.size(),pc.propertyNames.size());
	      return Error;
	    }
	    Vector v(elements.size());
	    for(size_t k=0;k<elements.size();k++) {
	      stringstream ss(elements[k]);
	      ss>>v[k];
	    }
	    pc.properties.push_back(v);
	  }
	}
	else {
	  fprintf(stderr,"PCD parser: DATA is not spcified as ascii or binary\n");
	  return Error;
	}
      }
      else {
	string value;
	if(!ReadLine(value)) return Error;
	pc.settings[word] = Strip(value);

	if(word == "VERSION") {
	  if(pc.settings[word] != "0.7" && pc.settings[word] != ".7") {
	    fprintf(stderr,"PCD parser: Warning, PCD version 0.7 expected, got %s\n",pc.settings[word].c_str());
	  }
	}
	else {
	  printf("PCD parser: Read property \"%s\" = \"%s\"\n",word.c_str(),pc.settings[word].c_str());
	}
      }
    }
    return Continue;
  }
  virtual Result InputPunct(const string& punct) { return Continue; }
  virtual Result InputEndLine()
  {
    if(state != NORMAL) state=NORMAL;
    return Continue;
  }

  PointCloud3D& pc;
  int state;
  int numPoints;
  //for binary data
  vector<string> types;
  vector<int> sizes;
  vector<int> counts;
};


void PointCloud3D::Clear()
{
  points.clear();
  propertyNames.clear();
  properties.clear();
  settings.clear();
}

bool PointCloud3D::LoadPCL(const char* fn)
{
  ifstream in(fn,ios::in);
  if(!in) return false;
  if(!LoadPCL(in)) return false;
  in.close();
  return true;
}

bool PointCloud3D::SavePCL(const char* fn) const
{
  ofstream out(fn,ios::out);
  if(!out) return false;
  if(!SavePCL(out)) return false;
  out.close();
  return true;
}

bool PointCloud3D::LoadPCL(istream& in)
{
  PCLParser parser(in,*this);
  if(!parser.Read()) {
    fprintf(stderr,"PCD parser: Unable to parse PCD file\n");
    return false;
  }
  int elemIndex[3] = {-1,-1,-1};
  for(size_t i=0;i<propertyNames.size();i++) {
    if(propertyNames[i]=="x") elemIndex[0] = (int)i;
    if(propertyNames[i]=="y") elemIndex[1] = (int)i;
    if(propertyNames[i]=="z") elemIndex[2] = (int)i;
  }
  if(elemIndex[0]<0 || elemIndex[1]<0 || elemIndex[2]<0) {
    fprintf(stderr,"PCD parser: Warning, PCD file does not have x, y or z\n");
    fprintf(stderr,"  Properties:");
    for(size_t i=0;i<propertyNames.size();i++)
      fprintf(stderr," \"%s\"",propertyNames[i].c_str());
    fprintf(stderr,"\n");
    return true;
  }
  //parse out the points
  points.resize(properties.size());
  for(size_t i=0;i<properties.size();i++) {
    points[i].set(properties[i][elemIndex[0]],properties[i][elemIndex[1]],properties[i][elemIndex[2]]);
  }
  printf("PCD parser: %d points read\n",points.size());

  if(properties.size()==3 && elemIndex[0]==0 && elemIndex[1]==1 && elemIndex[2]==2) {
    //x,y,z are the only properties, go ahead and take them out
    propertyNames.resize(0);
    properties.resize(0);
  }
  return true;
}

bool PointCloud3D::SavePCL(ostream& out) const
{
  out<<"# .PCD v0.7 - Point Cloud Data file format"<<endl;
  if(settings.find("VERSION") != settings.end())
    out<<"VERSION "<<settings.find("VERSION")->second<<endl;
  else
    out<<"VERSION 0.7"<<endl;
  bool addxyz = false;
  if(!points.empty()) {
    int elemIndex[3] = {-1,-1,-1};
    for(size_t i=0;i<propertyNames.size();i++) {
      if(propertyNames[i]=="x") elemIndex[0] = (int)i;
      if(propertyNames[i]=="y") elemIndex[1] = (int)i;
      if(propertyNames[i]=="z") elemIndex[2] = (int)i;
    }
    if(elemIndex[0]<0 || elemIndex[1]<0 || elemIndex[2]<0) {
      addxyz = true;
    }
  }

  out<<"FIELDS";
  if(addxyz)
    out<<" x y z";
  for(size_t i=0;i<propertyNames.size();i++)
    out<<" "<<propertyNames[i];
  out<<endl;
  if(!properties.empty())
    out<<"POINTS "<<properties.size()<<endl;
  else 
    out<<"POINTS "<<points.size()<<endl;

  for(map<string,string>::const_iterator i=settings.begin();i!=settings.end();i++) {
    if(i->first == "VERSION") continue;
    out<<i->first<<" "<<i->second<<endl;
  }
  out<<"DATA ascii"<<endl;  
  if(propertyNames.empty()) {
    for(size_t i=0;i<points.size();i++) 
      out<<points[i]<<endl;
  }
  else {
    for(size_t i=0;i<properties.size();i++) {
      if(addxyz)
	out<<points[i]<<" ";
      for(int j=0;j<properties[i].n;j++)
	out<<properties[i][j]<<" ";
      out<<endl;
    }
  }
  return true;
}

void PointCloud3D::Transform(const Matrix4& mat)
{
  bool hasNormals = false;
  //names of the normal properties in the PCD file
  static const char* nxprop = "normal_x", *nyprop = "normal_y", *nzprop = "normal_z";
  int nxind = -1, nyind = -1, nzind = -1;
  for(size_t i=0;i<propertyNames.size();i++) {
    if(propertyNames[i] == nxprop) {
      nxind = (int)i;
      continue;
    }
    if(propertyNames[i] == nyprop) {
      nyind = (int)i;
      continue;
    }
    if(propertyNames[i] == nzprop) {
      nzind = (int)i;
      continue;
    }
  }
  hasNormals = (nxind >= 0 && nyind >= 0 && nzind >= 0);

  for(size_t i=0;i<points.size();i++) {
    Vector3 temp=points[i];
    mat.mulPoint(temp,points[i]);
    //transform normals if this has them
    if(hasNormals) {
      Vector3 temp2;
      temp.set(properties[i][nxind],properties[i][nyind],properties[i][nzind]);
      mat.mulVector(temp,temp2);
      temp2.get(properties[i][nxind],properties[i][nyind],properties[i][nzind]);
    }
  }
}

int PointCloud3D::PropertyIndex(const string& name) const
{
  for(size_t i=0;i<propertyNames.size();i++) {
    if(propertyNames[i] == name) return (int)i;
  }
  return -1;
}

bool PointCloud3D::GetProperty(const string& name,vector<Real>& items) const
{
  int i = PropertyIndex(name);
  if(i < 0) return false;
  items.resize(properties.size());
  for(size_t k=0;k<properties.size();k++)
    items[k] = properties[k][i];
  return true;
}

void PointCloud3D::SetProperty(const string& name,const vector<Real>& items)
{
  int i = PropertyIndex(name);
  if(i >= 0) {
    for(size_t k=0;k<properties.size();k++) {
      properties[k][i] = items[k];
    }
    return;
  }
  else {
    //add it
    propertyNames.push_back(name);
    for(size_t k=0;k<properties.size();k++) {
      Vector oldval = properties[k];
      properties[k].resize(propertyNames.size());
      properties[k].copySubVector(0,oldval);
      properties[k][propertyNames.size()-1] = items[k];
    }
  }
}
void PointCloud3D::RemoveProperty(const string& name)
{
  int i = PropertyIndex(name);
  if(i >= 0) {
    for(size_t k=0;k<properties.size();k++) {
      for(size_t j=i+1;j<propertyNames.size();j++)
	properties[k][j-1] = properties[k][j];
      properties[k].n--;
    }
    propertyNames.erase(propertyNames.begin()+i);
    return;
  }
  else
    fprintf(stderr,"PointCloud3D::RemoveProperty: warning, property %s does not exist\n",name.c_str());
}

void PointCloud3D::GetSubCloud(const Vector3& bmin,const Vector3& bmax,PointCloud3D& subcloud)
{
  AABB3D bb(bmin,bmax);
  subcloud.Clear();
  subcloud.propertyNames = propertyNames;
  subcloud.settings = settings;
  for(size_t i=0;i<points.size();i++)
    if(bb.contains(points[i])) {
      subcloud.points.push_back(points[i]);
      subcloud.properties.push_back(properties[i]);
    }
}

void PointCloud3D::GetSubCloud(const string& property,Real value,PointCloud3D& subcloud)
{
  GetSubCloud(property,value,value,subcloud);
}

void PointCloud3D::GetSubCloud(const string& property,Real minValue,Real maxValue,PointCloud3D& subcloud)
{
  subcloud.Clear();
  subcloud.propertyNames = propertyNames;
  subcloud.settings = settings;
  if(property == "x") {
    for(size_t i=0;i<points.size();i++)
      if(minValue <= points[i].x && points[i].x <= maxValue) {
	subcloud.points.push_back(points[i]);
	subcloud.properties.push_back(properties[i]);
      }
  }
  else if(property == "y") {
    for(size_t i=0;i<points.size();i++)
      if(minValue <= points[i].y && points[i].y <= maxValue) {
	subcloud.points.push_back(points[i]);
	subcloud.properties.push_back(properties[i]);
      }
  }
  else if(property == "z") {
    for(size_t i=0;i<points.size();i++)
      if(minValue <= points[i].z && points[i].z <= maxValue) {
	subcloud.points.push_back(points[i]);
	subcloud.properties.push_back(properties[i]);
      }
  }
  else {
    int i=PropertyIndex(property);
    if(i < 0) {
      fprintf(stderr,"PointCloud3D::GetSubCloud: warning, property %s does not exist\n",property.c_str());
      return;
    }
    for(size_t k=0;k<properties.size();k++)
      if(minValue <= properties[k][i] && properties[k][i] <= maxValue) {
	subcloud.points.push_back(points[k]);
	subcloud.properties.push_back(properties[k]);
      }
  }
}

#include "PointCloud.h"
#include <utils/SimpleParser.h>
#include <utils/stringutils.h>
#include <sstream>
#include <fstream>
#include <stdlib.h>

using namespace Meshing;

class PCLParser : public SimpleParser
{
public:
  enum {NORMAL, READING_FIELDS };
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
    else {
      if(word == "POINTS") {
	string points;
	ReadLine(points);
	stringstream ss(points);
	ss>>numPoints;
	if(!ss) {
	  fprintf(stderr,"Unable to read integer POINTS\n");
	  return Error;
	}
      }
      else if(word == "FIELDS") {
	state = READING_FIELDS;
      }
      else if(word == "DATA") {
	string datatype;
	if(!ReadLine(datatype)) return Error;
	datatype = Strip(datatype);
	if(datatype != "ascii") {
	  fprintf(stderr,"DATA is not ascii, can't parse binary yet\n");
	  return Error;
	}
	if(numPoints < 0) {
	  fprintf(stderr,"DATA specified before POINTS element\n");
	  return Error;
	}
	string line;
	for(int i=0;i<numPoints;i++) {
	  int c = in.get();
	  assert(c=='\n' || c==EOF);
	  lineno++;
	  if(c==EOF) {
	    fprintf(stderr,"Premature end of DATA element\n");
	    return Error;
	  }
	  if(!ReadLine(line)) {
	    fprintf(stderr,"Error reading point %d\n",i);
	    return Error;
	  }
	  vector<string> elements = Split(line," ");
	  if(elements.size() != pc.propertyNames.size()) {
	    fprintf(stderr,"DATA element %d has length %d, but %d properties specified\n",i,elements.size(),pc.propertyNames.size());
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
	string value;
	if(!ReadLine(value)) return Error;
	pc.settings[word] = Strip(value);

	if(word == "VERSION") {
	  if(pc.settings[word] != "0.7") {
	    fprintf(stderr,"Warning, PCL version 0.7 expected, got %s\n",pc.settings[word].c_str());
	  }
	}
      }
    }
    return Continue;
  }
  virtual Result InputPunct(const string& punct) { return Continue; }
  virtual Result InputEndLine()
  {
    if(state == READING_FIELDS) state=NORMAL;
    return Continue;
  }

  PointCloud3D& pc;
  int state;
  int numPoints;
};


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
    fprintf(stderr,"Unable to parse PCL file\n");
    return false;
  }
  int elemIndex[3] = {-1,-1,-1};
  for(size_t i=0;i<propertyNames.size();i++) {
    if(propertyNames[i]=="x") elemIndex[0] = (int)i;
    if(propertyNames[i]=="y") elemIndex[1] = (int)i;
    if(propertyNames[i]=="z") elemIndex[2] = (int)i;
  }
  if(elemIndex[0]<0 || elemIndex[1]<0 || elemIndex[2]<0) {
    fprintf(stderr,"Warning, PCL file does not have x, y or z\n");
    fprintf(stderr,"Properties:");
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

  if(properties.size()==3 && elemIndex[0]==0 && elemIndex[1]==1 && elemIndex[2]==2) {
    //go ahead and take out the properties
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
  //TODO: anything with normals?
  for(size_t i=0;i<points.size();i++) {
    Vector3 temp=points[i];
    mat.mulPoint(temp,points[i]);
  }
}

bool PointCloud3D::GetProperty(const string& name,vector<Real>& items) const
{
  for(size_t i=0;i<propertyNames.size();i++) {
    if(propertyNames[i] == name) {
      items.resize(properties.size());
      for(size_t k=0;k<properties.size();k++)
	items[k] = properties[k][i];
      return true;
    }
  }
  return false;
}

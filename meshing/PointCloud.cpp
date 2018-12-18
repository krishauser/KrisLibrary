#include <KrisLibrary/Logger.h>
#include "PointCloud.h"
#include <iostream>
#include <KrisLibrary/math3d/AABB3D.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/utils/SimpleParser.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/Timer.h>
#include <errors.h>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <string.h>

using namespace Meshing;
const static Real one_over_255 = 1.0/255.0;

string IntToStr(int i)
{
  stringstream ss;
  ss<<i;
  return ss.str();
}

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
        LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Invalid PCD TYPE "<<word.c_str());
        return Error;
      }
      types.push_back(word);
    }
    else if(state == READING_COUNTS) {
      if(!IsValidInteger(word.c_str())) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Invalid PCD COUNT string "<<word.c_str()<<", must be integer");
        return Error;
      }
      stringstream ss(word);
      int count;
      ss>>count;
      if(ss.bad() || count <= 0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Invalid PCD COUNT "<<word.c_str()<<", must be a positive integer");
        return Error;
      }
      counts.push_back(count);
    }
    else if(state == READING_SIZES) {
      if(!IsValidInteger(word.c_str())) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Invalid PCD SIZE string "<<word.c_str()<<", must be integer");
        return Error;
      }
      stringstream ss(word);
      int size;
      ss>>size;
      if(size <= 0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Invalid PCD SIZE "<<word.c_str()<<" must be positive");
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
          LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Unable to read integer POINTS");
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
        ConvertToSingleCounts();
        /*
        cout<<"property names, type, size"<<endl;
        for(size_t i=0;i<pc.propertyNames.size();i++)
          cout<<pc.propertyNames[i]<<", "<<types[i]<<", "<<sizes[i]<<endl;
        */
        string datatype;
        if(!ReadLine(datatype)) return Error;
        datatype = Strip(datatype);
        if(datatype == "binary") {
          //pull in the endline
          int c = in.get();
          if(c != '\n') {
            LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: DATA binary not followed immediately by endline");
            return Error;
          }
          //read in binary data
          if(numPoints < 0) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: DATA specified before POINTS element");
            return Error;
          }
          if(sizes.size() != pc.propertyNames.size()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Invalid number of SIZE elements");
            return Error;
          }
          if(types.size() != pc.propertyNames.size()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Invalid number of TYPE elements");
            return Error;
          }
          int pointsize = 0;
          for(size_t i=0;i<sizes.size();i++)
            pointsize += sizes[i];

          streampos fcur = in.tellg();
          in.seekg( 0, std::ios::end );
          streampos fsize = in.tellg() - fcur;
          in.seekg( fcur );
          if(fsize-streampos(pointsize*numPoints) != 0) {
            cout<<"Size of point "<<pointsize<<" x "<<numPoints<<" points"<<endl;
            cout<<"Remaining bytes left: "<<fsize<<", we'll probably have "<<fsize-streampos(pointsize*numPoints)<<" left?"<<endl;
            for(int i=0;i<int(fsize)-pointsize*numPoints;i++) {
              if(in.peek() != 0) {
                cout<<"Hmmm... what's this wasted space? stopped on "<<i<<endl;
              }
              in.get();
            }
          }

          vector<char> buffer(pointsize);
          for(int i=0;i<numPoints;i++) {
            in.read(&buffer[0],pointsize);
            if(!in) {
              LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Error reading data for point "<<i);
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
                  /*
                  if(i % 10000 == 0)
                    printf("%s Buffer %x => float %f\n",pc.propertyNames[j].c_str(),*((unsigned int*)&buffer[ofs]),f);
                    */
                }
                else if(sizes[j] == 8) {
                  double f;
                  memcpy(&f,&buffer[ofs],sizes[j]);
                  v[j] = f;
                }
                else {
                  LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Invalid float size "<<sizes[j]);
                  return Error;
                }
              }
              else if(types[j] == "U") {
                if(sizes[j] > 4) {
                  LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Invalid unsigned int size "<<sizes[j]);
                  return Error;       
                }
                unsigned i=0;
                memcpy(&i,&buffer[ofs],sizes[j]);
                v[j] = Real(i);
              }
              else if(types[j] == "I") {
                LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Invalid int size "<<sizes[j]);
                int i=0;
                memcpy(&i,&buffer[ofs],sizes[j]);
                v[j] = Real(i);
              }
              else {
                LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Invalid type "<<types[i].c_str());
                return Error;
              }
              ofs += sizes[j];
            }
            pc.properties.push_back(v);
          }
          return Stop;
        }
        else if(datatype == "ascii") {
          if(numPoints < 0) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: DATA specified before POINTS element");
            return Error;
          }
          string line;
          for(int i=0;i<numPoints;i++) {
            int c = in.get();
            assert(c=='\n' || c==EOF);
            lineno++;
            if(c==EOF) {
              LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Premature end of DATA element");
              return Error;
            }
            if(!ReadLine(line)) {
              LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Error reading point "<<i);
              return Error;
            }
            vector<string> elements = Split(line," ");
            if(elements.size() != pc.propertyNames.size()) {
              LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: DATA element "<<i<<" has length "<<elements.size()<<", but "<<pc.propertyNames.size());
              return Error;
            }
            Vector v(elements.size());
            for(size_t k=0;k<elements.size();k++) {
              stringstream ss(elements[k]);
              SafeInputFloat(ss,v[k]);
            }
            pc.properties.push_back(v);
          }
        }
        else {
          LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: DATA is not spcified as ascii or binary");
          return Error;
        }
      }
      else {
        string value;
        if(!ReadLine(value)) return Error;
        value = Strip(value);

        if(word == "VERSION") {
          if(value != "0.7" && value != ".7") {
                        LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Warning, PCD version 0.7 expected, got \""<<value.c_str()<<"\"");
          }
          pc.settings["pcd_version"] = value;
        }
        else {
          //LOG4CXX_INFO(KrisLibrary::logger(),"PCD parser: Read property \""<<word.c_str()<<"\" = \""<<pc.settings[word].c_str());
          string key=word;
          Lowercase(key);
          pc.settings[key] = Strip(value);
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

  void ConvertToSingleCounts()
  {
    size_t i=0;
    while(i < counts.size()) {
      if(counts[i] > 1) {
        int n=counts[i];
        LOG4CXX_INFO(KrisLibrary::logger(),"PCD parser: converting element "<<pc.propertyNames[i]<<" into "<<counts[i]<<" sub-elements");
        string basename = pc.propertyNames[i];
        pc.propertyNames[i] = basename + '_' + IntToStr(0);
        counts[i] = 1;
        for(int j=1;j<n;j++) {
          pc.propertyNames.insert(pc.propertyNames.begin()+i+1,basename + '_' + IntToStr(j));
          types.insert(types.begin()+i+1,types[i]);
          sizes.insert(sizes.begin()+i+1,sizes[i]);
          counts.insert(counts.begin()+i+1,1);
        }
        i += n;
      }
      else
        i++;
    }
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
  settings["file"] = fn;
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
        LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Unable to parse PCD file");
    return false;
  }
  int elemIndex[3] = {-1,-1,-1};
  for(size_t i=0;i<propertyNames.size();i++) {
    if(propertyNames[i]=="x") elemIndex[0] = (int)i;
    if(propertyNames[i]=="y") elemIndex[1] = (int)i;
    if(propertyNames[i]=="z") elemIndex[2] = (int)i;
  }
  if(elemIndex[0]<0 || elemIndex[1]<0 || elemIndex[2]<0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"PCD parser: Warning, PCD file does not have x, y or z");
        LOG4CXX_ERROR(KrisLibrary::logger(),"  Properties:");
    for(size_t i=0;i<propertyNames.size();i++)
            LOG4CXX_ERROR(KrisLibrary::logger()," \""<<propertyNames[i].c_str());
        LOG4CXX_ERROR(KrisLibrary::logger(),"");
    return true;
  }

  //HACK: for float RGB and RGBA elements, convert float bytes
  //to integer via memory cast
  Assert(propertyNames.size() == parser.types.size());
  for(size_t k=0;k<propertyNames.size();k++) {
    if(parser.types[k] == "F" && (propertyNames[k] == "rgb" || propertyNames[k] == "rgba")) { 
      bool docast = false;
      for(size_t i=0;i<properties.size();i++) {
    Vector& v = properties[i];
    float f = float(v[k]);
    if(f < 1.0 && f > 0.0) {
      docast=true;
      break;
    }
      }
      if(docast) {
        //LOG4CXX_ERROR(KrisLibrary::logger(),"PointCloud::LoadPCL: Warning, casting RGB colors to integers via direct memory cast");
    for(size_t i=0;i<properties.size();i++) {
      Vector& v = properties[i];
      float f = float(v[k]);
      int rgb = *((int*)&f);
      v[k] = (Real)rgb;
    }
      }
    }
  }

  //parse out the points
  points.resize(properties.size());
  for(size_t i=0;i<properties.size();i++) {
    points[i].set(properties[i][elemIndex[0]],properties[i][elemIndex[1]],properties[i][elemIndex[2]]);
  }
  //LOG4CXX_INFO(KrisLibrary::logger(),"PCD parser: "<<points.size());

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
  if(settings.find("pcd_version") != settings.end())
    out<<"VERSION "<<settings.find("pcd_version")->second<<endl;
  else
    out<<"VERSION 0.7"<<endl;
  bool addxyz = !HasXYZAsProperties();

  out<<"FIELDS";
  if(addxyz)
    out<<" x y z";
  for(size_t i=0;i<propertyNames.size();i++)
    out<<" "<<propertyNames[i];
  out;
  out<<"TYPE";
  if(addxyz)
    out<<" F F F";
  for(size_t i=0;i<propertyNames.size();i++)
    out<<" F";
  out;

  if(!properties.empty())
    out<<"POINTS "<<properties.size();
  else 
    out<<"POINTS "<<points.size();

  for(map<string,string>::const_iterator i=settings.begin();i!=settings.end();i++) {
    if(i->first == "pcd_version" || i->first == "file") continue;
    string key = i->first;
    Uppercase(key);
    out<<key<<" "<<i->second;
  }
  out<<"DATA ascii";  
  if(propertyNames.empty()) {
    for(size_t i=0;i<points.size();i++) 
      out<<points[i];
  }
  else {
    for(size_t i=0;i<properties.size();i++) {
      if(addxyz)
    out<<points[i]<<" ";
      for(int j=0;j<properties[i].n;j++)
    out<<properties[i][j]<<" ";
      out;
    }
  }
  return true;
}

void PointCloud3D::FromDepthImage(int w,int h,float wfov,float hfov,const std::vector<float>& depths,const std::vector<unsigned int>& rgb,float invalidDepth)
{
  SetStructured(w,h);
  Real xscale = Tan(wfov/2)*(2.0/w);
  Real yscale = Tan(hfov/2)*(2.0/h);
  Assert(depths.size()==points.size());
  Real xc = Real(w)/2;
  Real yc = Real(h)/2;
  int k=0;
  Real fi=0,fj=0;
  for(int j=0;j<h;j++,fj+=1.0) {
    fi = 0;
    for(int i=0;i<w;i++,k++,fi+=1.0) {
      if(depths[k] == invalidDepth)
        points[k].setZero();
      else {
        Real x = (fi-xc)*xscale;
        Real y = (fj-yc)*yscale;
        points[k].x = depths[k]*x;
        points[k].y = depths[k]*y;
        points[k].z = depths[k];
      }
    }
  }
  if(!rgb.empty()) {
    Assert(rgb.size()==depths.size());
    propertyNames.resize(1);
    propertyNames[0] = "rgb";
    properties.resize(points.size());
    for(size_t i=0;i<points.size();i++) {
      properties[i].resize(1);
      properties[i][0] = Real(rgb[i]);
    }
  }
}

void PointCloud3D::FromDepthImage(int w,int h,float wfov,float hfov,float depthscale,const std::vector<unsigned short>& depths,const std::vector<unsigned int>& rgb,unsigned short invalidDepth)
{
  vector<float> fdepth(depths.size());
  for(size_t i=0;i<depths.size();i++)
    fdepth[i] = depths[i]*depthscale;
  FromDepthImage(w,h,wfov,hfov,fdepth,rgb,invalidDepth*depthscale);
}

bool PointCloud3D::IsStructured() const
{
  return GetStructuredWidth() >= 1 && GetStructuredHeight() > 1;
}

int PointCloud3D::GetStructuredWidth() const
{
  return settings.getDefault("width",0);
}

int PointCloud3D::GetStructuredHeight() const
{
  return settings.getDefault("height",0);
}

void PointCloud3D::SetStructured(int w,int h)
{
  settings.set("width",w);
  settings.set("height",h);
  points.resize(w*h);
}

Vector3 PointCloud3D::GetOrigin() const
{
  string viewpoint;
  if(!settings.get("viewpoint",viewpoint)) return Vector3(0.0);
  stringstream ss(viewpoint);
  Vector3 o;
  ss>>o;
  return o;
}

void PointCloud3D::SetOrigin(const Vector3& origin)
{
  string viewpoint;
  if(!settings.get("viewpoint",viewpoint)) {
    stringstream ss;
    ss<<origin<<" 1 0 0 0";
    settings.set("viewpoint",ss.str());
    return;
  }
  stringstream ss(viewpoint);
  Vector3 o;
  Vector4 q;
  ss>>o>>q;
  stringstream ss2;
  ss2<<origin<<" "<<q;
  settings.set("viewpoint",ss2.str());
}

RigidTransform PointCloud3D::GetViewpoint() const
{
  RigidTransform T;
  string viewpoint;
  if(!settings.get("viewpoint",viewpoint)) {
    T.setIdentity();
    return T;
  }
  stringstream ss(viewpoint);
  QuaternionRotation q;
  ss>>T.t>>q;
  q.getMatrix(T.R);
  return T;
}

void PointCloud3D::SetViewpoint(const RigidTransform& T)
{
  QuaternionRotation q;
  q.setMatrix(T.R);
  stringstream ss;
  ss<<T.t<<" "<<q;
  settings.set("viewpoint",ss.str());
}

void PointCloud3D::GetAABB(Vector3& bmin,Vector3& bmax) const
{
  AABB3D bb;
  bb.minimize();
  for(size_t i=0;i<points.size();i++)
    bb.expand(points[i]);
  bmin = bb.bmin;
  bmax = bb.bmax;
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

bool PointCloud3D::HasXYZAsProperties() const
{
  if(points.empty()) return false;
  int elemIndex[3] = {-1,-1,-1};
  for(size_t i=0;i<propertyNames.size();i++) {
    if(propertyNames[i]=="x") elemIndex[0] = (int)i;
    if(propertyNames[i]=="y") elemIndex[1] = (int)i;
    if(propertyNames[i]=="z") elemIndex[2] = (int)i;
  }
  if(elemIndex[0]<0 || elemIndex[1]<0 || elemIndex[2]<0) 
    return false;
  return true;
}

void PointCloud3D::SetXYZAsProperties(bool isprop)
{
  if(HasXYZAsProperties() == isprop)  return;
  int elemIndex[3] = {-1,-1,-1};
  const char* elementNames[3] = {"x","y","z"};
  for(size_t i=0;i<propertyNames.size();i++) {
    if(propertyNames[i]=="x") elemIndex[0] = (int)i;
    if(propertyNames[i]=="y") elemIndex[1] = (int)i;
    if(propertyNames[i]=="z") elemIndex[2] = (int)i;
  }
  if(isprop) { //add
    int numprops = propertyNames.size();
    for(int i=0;i<3;i++)
      if(elemIndex[i] < 0) {
    propertyNames.push_back(elementNames[i]);
    elemIndex[i] = numprops;
    numprops++;
      }
    if(properties.empty()) 
      properties.resize(points.size());
    for(size_t i=0;i<points.size();i++) {
      Vector oldprops = properties[i];
      properties[i].resize(numprops);
      properties[i].copySubVector(0,oldprops);
      properties[i][elemIndex[0]] = points[i].x;
      properties[i][elemIndex[1]] = points[i].y;
      properties[i][elemIndex[2]] = points[i].z;
    }
  }
  else {
    //remove from properties
    if(elemIndex[2] >= 0) RemoveProperty("z");
    if(elemIndex[1] >= 0) RemoveProperty("y");
    if(elemIndex[0] >= 0) RemoveProperty("x");
  }
}

bool PointCloud3D::HasNormals() const
{
  return HasProperty("normal_x") && HasProperty("normal_y") && HasProperty("normal_z") ;
}

bool PointCloud3D::GetNormals(vector<Vector3>& normals) const
{
  int nx=PropertyIndex("normal_x"),ny=PropertyIndex("normal_y"),nz=PropertyIndex("normal_z");
  if(nx < 0 || ny < 0 || nz < 0) return false;
  normals.resize(properties.size());
  for(size_t i=0;i<properties.size();i++)
    normals[i].set(properties[i][nx],properties[i][ny],properties[i][nz]);
  return true;
}

void PointCloud3D::SetNormals(const vector<Vector3>& normals)
{
  int nx=PropertyIndex("normal_x"),ny=PropertyIndex("normal_y"),nz=PropertyIndex("normal_z");
  if(nx < 0) {
    vector<Real> items(points.size());
    SetProperty("normal_x",items);
  }
  if(ny < 0) {
    vector<Real> items(points.size());
    SetProperty("normal_y",items);
  }
  if(nz < 0) {
    vector<Real> items(points.size());
    SetProperty("normal_z",items);
  }
  for(size_t i=0;i<properties.size();i++)
    normals[i].get(properties[i][nx],properties[i][ny],properties[i][nz]);
}
bool PointCloud3D::HasColor() const
{
  return HasProperty("c") || HasProperty("rgba") || HasProperty("rgb") || HasProperty("opacity") || (HasProperty("r") && HasProperty("g") && HasProperty("b"));
}

bool PointCloud3D::HasOpacity() const
{
  return HasProperty("c") || HasProperty("opacity");
}

bool PointCloud3D::HasRGB() const
{
  return HasProperty("rgb") || HasProperty("rgba") || (HasProperty("r") && HasProperty("g") && HasProperty("b"));
}

bool PointCloud3D::HasRGBA() const
{
  return HasProperty("rgba") || (HasProperty("r") && HasProperty("g") && HasProperty("b") && HasProperty("a"));
}

bool PointCloud3D::UnpackColorChannels(bool alpha)
{
  if(HasProperty("rgb")) {
    vector<Real> r,g,b,a;
    GetColors(r,g,b,a);
    SetProperty("r",r);
    SetProperty("g",g);
    SetProperty("b",b);
    if(alpha)
      SetProperty("a",a);
    RemoveProperty("rgb");
    return true;
  }
  else if(HasProperty("rgba")) {
    vector<Real> r,g,b,a;
    GetColors(r,g,b,a);
    SetProperty("r",r);
    SetProperty("g",g);
    SetProperty("b",b);
    SetProperty("a",a);
    RemoveProperty("rgba");
    return true;
  }
  return false;
}

bool PointCloud3D::PackColorChannels(const char* fmt)
{
  vector<Real> r,g,b,a;
  if(!GetProperty("r",r)) return false;
  if(!GetProperty("g",g)) return false;
  if(!GetProperty("b",b)) return false;
  if(0==strcmp(fmt,"rgb")) {
    SetColors(r,g,b,false);
    RemoveProperty("r");
    RemoveProperty("g");
    RemoveProperty("b");
    if(HasProperty("a")) RemoveProperty("a");
    return true;
  }
  else if(0==strcmp(fmt,"rgba")) {
    if(!GetProperty("a",a)) {
      a.resize(points.size());
      fill(a.begin(),a.end(),1.0);
    }
    SetColors(r,g,b,a,true);
    RemoveProperty("r");
    RemoveProperty("g");
    RemoveProperty("b");
    RemoveProperty("a");
    return true;
  }
  return false;
}

bool PointCloud3D::GetColors(vector<Vector4>& out) const
{
  Timer timer;
  vector<Real> rgb;
  if(GetProperty("rgb",rgb)) {
    //convert real to hex to GLcolor
    out.resize(rgb.size());
    for(size_t i=0;i<rgb.size();i++) {
      unsigned int col = (unsigned int)rgb[i];
      Real r=((col&0xff0000)>>16) *one_over_255;
      Real g=((col&0xff00)>>8) *one_over_255;
      Real b=(col&0xff) *one_over_255;
      out[i].set(r,g,b,1.0);
    }
    return true;
  }
  else if(GetProperty("rgba",rgb)) {
    //convert real to hex to GLcolor
    //following PCD, this is actuall A-RGB
    out.resize(rgb.size());
    for(size_t i=0;i<rgb.size();i++) {
      unsigned int col = (unsigned int)rgb[i];
      Real r = ((col&0xff0000)>>16) *one_over_255;
      Real g = ((col&0xff00)>>8) *one_over_255;
      Real b = (col&0xff) *one_over_255;
      Real a = ((col&0xff000000)>>24) *one_over_255;
      out[i].set(r,g,b,a);
    }
    return true;
  }
  else if(GetProperty("opacity",rgb)) {
    out.resize(rgb.size());
    for(size_t i=0;i<rgb.size();i++) 
      out[i].set(1,1,1,rgb[i]);
    return true;
  }
  else if(GetProperty("c",rgb)) {
    out.resize(rgb.size());
    for(size_t i=0;i<rgb.size();i++) 
      out[i].set(1,1,1,rgb[i]*one_over_255);
    return true;
  }
  return false;
}

bool PointCloud3D::GetColors(vector<Real>& r,vector<Real>& g,vector<Real>& b,vector<Real>& a) const
{
  vector<Real> rgb;
  if(GetProperty("rgb",rgb)) {
    //convert real to hex to GLcolor
    r.resize(rgb.size());
    g.resize(rgb.size());
    b.resize(rgb.size());
    a.resize(rgb.size());
    fill(a.begin(),a.end(),1.0);
    for(size_t i=0;i<rgb.size();i++) {
      int col = (int)rgb[i];
      r[i]=((col&0xff0000)>>16) *one_over_255;
      g[i]=((col&0xff00)>>8) *one_over_255;
      b[i]=(col&0xff) *one_over_255;
    }
    return true;
  }
  else if(GetProperty("rgba",rgb)) {
    //convert real to hex to GLcolor
    //following PCD, this is actuall A-RGB
    r.resize(rgb.size());
    g.resize(rgb.size());
    b.resize(rgb.size());
    a.resize(rgb.size());
    for(size_t i=0;i<rgb.size();i++) {
      int col = (int)rgb[i];
      r[i] = ((col&0xff0000)>>16) * one_over_255;
      g[i] = ((col&0xff00)>>8) * one_over_255;
      b[i] = (col&0xff) * one_over_255;
      a[i] = ((col&0xff000000)>>24) * one_over_255;
    }
    return true;
  }
  else if(GetProperty("opacity",rgb)) {
    r.resize(rgb.size(),1.0);
    g.resize(rgb.size(),1.0);
    b.resize(rgb.size(),1.0);
    a = rgb;
    return true;
  }
  else if(GetProperty("c",rgb)) {
    r.resize(rgb.size(),1.0);
    g.resize(rgb.size(),1.0);
    b.resize(rgb.size(),1.0);
    a.resize(rgb.size());
    for(size_t i=0;i<rgb.size();i++) 
      a[i] = rgb[i]*one_over_255;
    return true;
  }
  return false;
}

void PointCloud3D::SetColors(const vector<Real>& r,const vector<Real>& g,const vector<Real>& b,bool includeAlpha)
{
  if(!includeAlpha) {
    vector<Real> a;
    SetColors(r,g,b,a,includeAlpha);
  }
  else {
    vector<Real> a(points.size(),1.0);
    SetColors(r,g,b,a,includeAlpha);
  }
}

void PointCloud3D::SetColors(const vector<Real>& r,const vector<Real>& g,const vector<Real>& b,const vector<Real>& a,bool includeAlpha)
{
  if(!includeAlpha) {
    //pack it
    vector<Real> rgb(r.size());
    for(size_t i=0;i<r.size();i++) {
      int col = ((int(r[i]*255.0) & 0xff) << 16) |
    ((int(g[i]*255.0) & 0xff) << 8) |
    (int(b[i]*255.0) & 0xff);
      rgb[i] = Real(col);
    }
    SetProperty("rgb",rgb);
  }
  else {
    //pack it
    vector<Real> rgba(r.size());
    for(size_t i=0;i<r.size();i++) {
      int col = ((int(a[i]*255.0) & 0xff) << 24) |
    ((int(r[i]*255.0) & 0xff) << 16) |
    ((int(g[i]*255.0) & 0xff) << 8) |
    (int(b[i]*255.0) & 0xff);
      rgba[i] = Real(col);
    }
    SetProperty("rgba",rgba);
  }
}

void PointCloud3D::SetColors(const vector<Vector4>& rgba,bool includeAlpha)
{
  Assert(points.size()==rgba.size());
  Real r,g,b,a;
  if(!includeAlpha) {
    //pack it
    vector<Real> packed(rgba.size());
    for(size_t i=0;i<rgba.size();i++) {
      rgba[i].get(r,g,b,a);
      int col = ((int(r*255.0) & 0xff) << 16) |
        ((int(g*255.0) & 0xff) << 8) |
        (int(b*255.0) & 0xff);
      packed[i] = Real(col);
    }
    SetProperty("rgb",packed);
  }
  else {
    //pack it
    vector<Real> packed(rgba.size());
    for(size_t i=0;i<rgba.size();i++) {
      rgba[i].get(r,g,b,a);
      int col = ((int(a*255.0) & 0xff) << 24) |
        ((int(r*255.0) & 0xff) << 16) |
        ((int(g*255.0) & 0xff) << 8) |
        (int(b*255.0) & 0xff);
      packed[i] = Real(col);
    }
    SetProperty("rgba",packed);
  }
}

void PointCloud3D::SetUV(const vector<Vector2>& uvs)
{
  Assert(points.size()==uvs.size());
  vector<Real> u(uvs.size()),v(uvs.size());
  for(size_t i=0;i<uvs.size();i++) {
    uvs[i].get(u[i],v[i]);
  }
  SetProperty("u",u);
  SetProperty("v",v);
}

bool PointCloud3D::GetUV(vector<Vector2>& uvs) const
{
  vector<Real> u,v;
  if(GetProperty("u",u) && GetProperty("v",v)) {
    //convert real to hex to GLcolor
    uvs.resize(u.size());
    for(size_t i=0;i<uvs.size();i++) {
      uvs[i].set(u[i],v[i]);
    }
    return true;
  }
  return false;
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
        LOG4CXX_ERROR(KrisLibrary::logger(),"PointCloud3D::RemoveProperty: warning, property "<<name.c_str());
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
            LOG4CXX_ERROR(KrisLibrary::logger(),"PointCloud3D::GetSubCloud: warning, property "<<property.c_str());
      return;
    }
    for(size_t k=0;k<properties.size();k++)
      if(minValue <= properties[k][i] && properties[k][i] <= maxValue) {
    subcloud.points.push_back(points[k]);
    subcloud.properties.push_back(properties[k]);
      }
  }
}

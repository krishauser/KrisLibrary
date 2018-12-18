#include <KrisLibrary/Logger.h>
#include "StatCollector.h"
#include <limits>
#include <iostream>
#include <string.h>
#include "ioutils.h"
#include "errors.h"
using namespace std;

StatCollector::StatCollector()
  :n(0),xmin(numeric_limits<double>::infinity()),xmax(-numeric_limits<double>::infinity()),sum(0),sumsquared(0)
{}

void StatCollector::clear()
{
  n=0;
  xmin=numeric_limits<double>::infinity();
  xmax=-numeric_limits<double>::infinity();
  sum=0;
  sumsquared=0;
}
  
void StatCollector::collect(double x)
{
  if(x<xmin) xmin=x;
  if(x>xmax) xmax=x;
  sum += x;
  sumsquared += x*x;
  n+=1;
}

void StatCollector::weightedCollect(double x,double weight)
{
  if(x<xmin) xmin=x;
  if(x>xmax) xmax=x;
  sum += x*weight;
  sumsquared += x*x*weight;
  n+=weight;
}

string StatDatabase::Concat(const string& s1,const string& s2) const
{
  string d; d+=delim;
  return s1+d+s2;
}

string StatDatabase::Concat(const string& s1,const string& s2,const string& s3) const
{
  string d; d+=delim;
  return s1+d+s2+d+s3;
}

string StatDatabase::Concat(const string& s1,const string& s2,const string& s3,const string& s4) const
{
  string d; d+=delim;
  return s1+d+s2+d+s3+d+s4;
}

string StatDatabase::Concat(const vector<string>& s) const
{
  if(s.empty()) return "";
  string d; d+=delim;
  string temp = s[0];
  for(size_t i=1;i<s.size();i++) {
    temp += d;
    temp += s[i];
  }
  return temp;
}



typedef map<string,StatDatabase::Data>::const_iterator ConstIterator;

StatDatabase::StatDatabase()
  :delim(':')
{
}

void StatDatabase::Clear()
{
  root.count=0;
  root.value.clear(); 
  root.children.clear(); 
}

void StatDatabase::NameToPath(const string& name,vector<string>& path) const
{
  size_t pos=0;
  size_t len=name.length();
  string temp;
  path.clear();
  while(pos < len) {
    if(name[pos] == delim) {
      path.push_back(temp);
      temp.erase();
    }
    else temp += name[pos];
    pos++;
  }
  path.push_back(temp);
}

StatDatabase::Data& StatDatabase::AddData(const string& name)
{
  vector<string> path;
  NameToPath(name,path);
  Data* curNode = &root;
  for(size_t j=0;j<path.size();j++) {
    curNode = &curNode->children[path[j]];
  }
  return *curNode;
}

const StatDatabase::Data* StatDatabase::GetData(const string& name) const
{
  vector<string> path;
  NameToPath(name,path);
  const Data* curNode = &root;
  for(size_t j=0;j<path.size();j++) {
    ConstIterator i = curNode->children.find(path[j]);
    if(i == curNode->children.end()) return NULL;
    curNode = &i->second;
  }
  return curNode;
}

void StatDatabase::Increment(const string& name,int inc)
{
  Data& d=AddData(name);
  d.count += inc;
}

void StatDatabase::AddValue(const string& name,double val)
{
  Data& d=AddData(name);
  d.value << val;
}


int StatDatabase::GetCount(const string& name) const
{
  const Data* d=GetData(name);
  if(!d) return 0;
  return d->count;
}

const StatCollector& StatDatabase::GetValue(const string& name) const
{
  static StatCollector nullStats;
  const Data* d=GetData(name);
  if(!d) return nullStats;
  return d->value;
}


void PrintRecurse(ostream& out,const string& name,const StatDatabase::Data& data,int indent)
{
  //indent
  for(int i=0;i<indent;i++)
    out<<' ';

  //write the name, its data
  out<<name<<" -- ";
  bool nothingWritten = false;
  if(data.count != 0 && data.value.number() != 0) {
    out<<"(count) "<<data.count<<", (value) ";
    data.value.Print(out);
  }
  else if(data.count != 0) out<<data.count;
  else if(data.value.number() != 0) {
    data.value.Print(out);
  }
  else nothingWritten = true;

  if(data.children.empty()) {
    out<<endl;
    return;
  }

  //if all the children are leaves and don't have values, we can write them all on the same line
  bool writeOnSameLine = true;
  for(ConstIterator i=data.children.begin();i!=data.children.end();i++) {
    if(!i->second.children.empty()) {
      writeOnSameLine=false;
      break;
    }
    if(i->second.value.number() != 0) {
      writeOnSameLine=false;
      break;
    }
  }
  if(writeOnSameLine) {
    if(!nothingWritten) {
      out<<endl;
      //indent the next line
      for(int i=0;i<indent+2;i++)
	out<<' ';
    }
    for(ConstIterator i=data.children.begin();i!=data.children.end();i++) {
      out<<i->first<<" "<<i->second.count;
      if(i != --data.children.end()) out<<", ";
    }
    out<<endl;
  }
  else {
    if(nothingWritten) out<<endl;
    for(ConstIterator i=data.children.begin();i!=data.children.end();i++) {
      PrintRecurse(out,i->first,i->second,indent+2);
    }
  }
}

string XMLTranslateString(const string& name)
{
  string str;
  size_t n=name.length();
  for(size_t i=0;i<n;i++) {
    if(isalnum(name[i]) || strchr("_-.",name[i])!=NULL)
      str += name[i];
    else
      str += '_';
  }
  return str;
}

void SaveRecurse(ostream& out,const string& name,const StatDatabase::Data& data,int indent)
{
  /*
  for(int i=0;i<indent;i++)
    out<<' ';
  out<<"<"<<XMLTranslateString(name)<<" count=\""<<data.count<<"\" n=\""<<data.value.n<<"\" xmin=\""<<data.value.xmin<<"\" xmax=\""<<data.value.xmax<<"\" sum=\""<<data.value.sum<<"\" sumsquared=\""<<data.value.sumsquared<<"\" ";

  if(data.children.empty())
    out<<"/>"<<endl;
  else {
    out<<">"<<endl;
    for(ConstIterator i=data.children.begin();i!=data.children.end();i++) {
      SaveRecurse(out,i->first,i->second,indent+2);
    }
    for(int i=0;i<indent;i++)
      out<<' ';
    out<<"</"<<XMLTranslateString(name)<<">"<<endl;
  }
  */

  //indent
  for(int i=0;i<indent;i++)
    out<<' ';

  //write the name, its data
  SafeOutputString(out,name);
  out<<" "<<data.count<<" ";
  data.value.Save(out);
  out<<" {";
  if(data.children.empty()) {  //done, close bracket
    out<<" }"<<endl;
  }
  else {
    out<<endl;
    for(ConstIterator i=data.children.begin();i!=data.children.end();i++) {
      SaveRecurse(out,i->first,i->second,indent+2);
    }
    
    //indent
    for(int i=0;i<indent;i++)
      out<<' ';
    out<<"}"<<endl;
  }
}

bool LoadRecurse(istream& in,string& name,StatDatabase::Data& data,int indent)
{
  if(!InputQuotedString(in,name)) { LOG4CXX_INFO(KrisLibrary::logger(),"Failed to load name"); return false; }
  in>>data.count;
  if(!data.value.Load(in))  { LOG4CXX_INFO(KrisLibrary::logger(),"Failed to load value for "<<name); return false; }

  EatWhitespace(in);
  int c = in.get();
  if(!in) { LOG4CXX_INFO(KrisLibrary::logger(),"Couldnt load open brace for "<<name.c_str() <<", end of file\n" );} 
  if(c != '{') { LOG4CXX_INFO(KrisLibrary::logger(),"Couldn't load open brace for "<<name.c_str()<<", got "<<c); return false; }
  EatWhitespace(in);
  c = in.peek();
  while(c != '}') {
    string childname;
    StatDatabase::Data child;
    if(!LoadRecurse(in,childname,child,indent+2)) return false;
    EatWhitespace(in);
    c = in.peek();
  }
  //close brace read
  c=in.get();
  return true;
}

void StatDatabase::Print(ostream& out) const
{
  PrintRecurse(out,"Database root",root,0);
}

bool StatDatabase::Save(ostream& out) const
{
  SaveRecurse(out,"StatDatabase",root,0);
  return (bool)out;
}

bool StatDatabase::Load(istream& in)
{
  string temp;
  if(LoadRecurse(in,temp,root,0)) {
    if(temp != "StatDatabase") {
      LOG4CXX_ERROR(KrisLibrary::logger(),"StatDatabase::Load(): Warning, root is not set to \"StatDatabase\"");
    }
    return true;
  }
  return false;
}


void StatCollector::Print(ostream& out) const
{
  out<<"Num: "<<n<<" min: "<<minimum()<<" max: "<<maximum()<<" avg: "<<average()<<" std: "<<stddev();
}

bool StatCollector::Save(ostream& out) const
{
  out<<n<<" "<<xmin<<" "<<xmax<<" "<<sum<<" "<<sumsquared;
  return (bool)out;
}

bool StatCollector::Load(istream& in)
{
  //in>>n>>xmin>>xmax>>sum>>sumsquared;
  SafeInputFloat(in,n);
  SafeInputFloat(in,xmin);
  SafeInputFloat(in,xmax);
  SafeInputFloat(in,sum);
  SafeInputFloat(in,sumsquared);
  return (bool)in;
}

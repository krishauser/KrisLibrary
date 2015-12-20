#include "SimpleFile.h"
#include "stringutils.h"
#include <sstream>
#include <fstream>
using namespace std;

SimpleFile::SimpleFile(const char* fn)
{
  Load(fn);
}

SimpleFile::SimpleFile(istream& in)
{
  Load(in);
}

SimpleFile::SimpleFile()
{
  loaded = false;
}

void SimpleFile::AllowItem(const std::string& str,bool caseSensitive)
{
  if(!caseSensitive) {
    string s = str;
    Lowercase(s);
    validItems[s] = caseSensitive;
  }
  else
    validItems[str] = caseSensitive;
}

bool SimpleFile::Load(const char* fn)
{
  ifstream in(fn);
  if(!in) {
    fprintf(stderr,"SimpleFile::Load(): Unable to open file %s\n",fn);
    loaded = false;
    return false;
  }
  return Load(in);
}

bool SimpleFile::Load(istream& in)
{
  string name;
  string line;
  char buf[1025];
  int lineno=1;
  while(in >> name) {
    //read the rest of the line
    lineno++;
    buf[1024]=0;
    line.clear();
    bool foundEndline=false,comment=false;
    while(!foundEndline) {
      for(int i=0;i<1024;i++) {
	int c=in.get();
	if(c == '\n') {
	  buf[i] = 0;
	  foundEndline = true;
	  break;
	}
	else if(c == '#' || comment) {
	  c = 0;
	  comment = true;
	}
	buf[i] = c;
      }
      line += buf;
    }
    if(name[0]=='#') continue;  //ignore comment up to end of line
    if(!validItems.empty()) {
      if(validItems.count(name) == 0) {
	string temp=name;
	Lowercase(temp);
	if(validItems.count(temp) != 0) {
	  if(validItems[temp] == true) { //case sensitive
	    fprintf(stderr,"SimpleFile::Load: unsupported item %s on line %d (case sensitive)\n",name.c_str(),lineno);
	    return false;
	  }
	}
	else {
	  fprintf(stderr,"SimpleFile::Load: unsupported item %s on line %d\n",name.c_str(),lineno);
	  return false;
	}
      }
    }

    stringstream ss;
    ss.str(line);

    vector<PrimitiveValue>& items=entries[name];
    items.clear();
    PrimitiveValue value;
    while(ss>>value) {
      items.push_back(value);
    }
  }
  loaded=true;
  if(in.bad()) {
    fprintf(stderr,"SimpleFile::Load() Bad value on line %d\n",lineno);
    loaded=false;
  }
  return !in.bad();
}

bool SimpleFile::Save(const char* fn)
{
  ofstream out(fn);
  if(!out) return false;
  return Save(out);
}

bool SimpleFile::Save(ostream& out)
{
  for(map<string,vector<PrimitiveValue> >::const_iterator i=entries.begin();i!=entries.end();i++) {
    out<<i->first<<"\t";
    for(size_t j=0;j<i->second.size();j++)
      out<<i->second[j]<<" ";
    out<<endl;
  }
  return true;
}

bool SimpleFile::CheckSize(const string& name,int size,const char* errorString)
{
  const char* title=errorString;
  if(errorString==NULL) title = "Untitled file";
  if(entries.count(name) == 0) {
    fprintf(stderr,"SimpleFile(%s): Wrong number of items in %s, entry not present\n",title,name.c_str());
    return false;
  }
  if((int)entries[name].size() != size) {
    fprintf(stderr,"SimpleFile(%s): Wrong number of items in %s.  Need %d, have %d.\n",title,name.c_str(),size,entries[name].size());
    return false;
  }
  return true;
}

bool SimpleFile::CheckType(const string& name,int type,const char* errorString)
{
  const char* title=errorString;
  if(errorString==NULL) title = "Untitled file";
  if(entries.count(name) == 0) {
    fprintf(stderr,"SimpleFile(%s): Wrong entry type in %s, entry not present\n",title,name.c_str());
    return false;
  }
  vector<PrimitiveValue>& items = entries[name];
  for(size_t i=0;i<items.size();i++) {
    if(!items[i].CanCast(type)) {
      fprintf(stderr,"SimpleFile(%s): Wrong entry type in %s.  Need %d, have %d.\n",title,name.c_str(),type,items[i].type);
      cerr<<"   Item "<<i<<": "<<items[i]<<endl;
      return false;
    }
  }
  return true;
}

vector<int> SimpleFile::AsInteger(const string& name)
{
  vector<PrimitiveValue>& items = entries[name];
  vector<int> res(items.size());
  for(size_t i=0;i<res.size();i++)
    res[i] = items[i].AsInteger();
  return res;
}

vector<double> SimpleFile::AsDouble(const string& name)
{
  vector<PrimitiveValue>& items = entries[name];
  vector<double> res(items.size());
  for(size_t i=0;i<res.size();i++)
    res[i] = items[i].AsDouble();
  return res;
}

vector<string> SimpleFile::AsString(const string& name)
{
  vector<PrimitiveValue>& items = entries[name];
  vector<string> res(items.size());
  for(size_t i=0;i<res.size();i++)
    res[i] = items[i].AsString();
  return res;
}

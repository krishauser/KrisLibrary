#include "DataCollector.h"
using namespace std;

string DataDatabase::Concat(const string& s1,const string& s2) const
{
  string d; d+=delim;
  return s1+d+s2;
}

string DataDatabase::Concat(const string& s1,const string& s2,const string& s3) const
{
  string d; d+=delim;
  return s1+d+s2+d+s3;
}

string DataDatabase::Concat(const string& s1,const string& s2,const string& s3,const string& s4) const
{
  string d; d+=delim;
  return s1+d+s2+d+s3+d+s4;
}

string DataDatabase::Concat(const vector<string>& s) const
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




DataDatabase::DataDatabase()
  :delim(':')
{
}

void DataDatabase::Clear()
{
  root.Clear();
  root.children.clear(); 
}

void DataDatabase::NameToPath(const string& name,vector<string>& path) const
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

DataDatabase::Entry& DataDatabase::Insert(const string& name)
{
  vector<string> path;
  NameToPath(name,path);
  Entry* curNode = &root;
  for(size_t j=0;j<path.size();j++) {
    curNode = &curNode->children[path[j]];
  }
  return *curNode;
}

const DataDatabase::Entry* DataDatabase::Find(const string& name) const
{
  vector<string> path;
  NameToPath(name,path);
  const Entry* curNode = &root;
  for(size_t j=0;j<path.size();j++) {
    map<string,Entry>::const_iterator i=curNode->children.find(path[j]);
    if(i == curNode->children.end()) {
      return NULL;
    }
    curNode = &i->second;
  }
  return curNode;
}

DataDatabase::Entry* DataDatabase::Find(const string& name)
{
  vector<string> path;
  NameToPath(name,path);
  Entry* curNode = &root;
  for(size_t j=0;j<path.size();j++) {
    map<string,Entry>::iterator i=curNode->children.find(path[j]);
    if(i == curNode->children.end()) {
      return NULL;
    }
    curNode = &i->second;
  }
  return curNode;
}


#include "ParamList.h"
#include "stringutils.h"
#include "ioutils.h"
#include <sstream>
#include <errors.h>
using namespace std;

ParamList::ParamList()
{}

ParamList::ParamList(const PrimitiveValue& a1)
  :args(1)
{
  args[0] = a1;
}

ParamList::ParamList(const PrimitiveValue& a1,const PrimitiveValue& a2)
  :args(2)
{
  args[0] = a1;
  args[1] = a2;
}

ParamList::ParamList(const PrimitiveValue& a1,const PrimitiveValue& a2,const PrimitiveValue& a3)
  :args(3)
{
  args[0] = a1;
  args[1] = a2;
  args[2] = a3;
}

ParamList::ParamList(const PrimitiveValue& a1,const PrimitiveValue& a2,const PrimitiveValue& a3,const PrimitiveValue& a4)
  :args(3)
{
  args[0] = a1;
  args[1] = a2;
  args[2] = a3;
  args[3] = a4;
}

ParamList::ParamList(const vector<PrimitiveValue>& _args)
  :args(_args)
{}

ParamList::ParamList(const map<string,PrimitiveValue>& _args)
{
  for(map<string,PrimitiveValue>::const_iterator i=_args.begin();i!=_args.end();i++) {
    names[i->first] = args.size();
    args.push_back(i->second);
  }
}

string ParamList::write() const
{
  stringstream temp;
  map<int,string> nameMap;
  for(map<string,int>::const_iterator i=names.begin();i!=names.end();i++)
    nameMap[i->second] = i->first;
  for(size_t i=0;i<args.size();i++) {
    if(nameMap.count(i) != 0) {
      temp<<nameMap[i]<<"=";
	}
	temp<<args[i];
    if(i+1 != args.size()) temp<<", ";
  }
  return temp.str();
}

bool ParamList::parse(const string& str)
{
  vector<string> argText;
  size_t index=0;
  while(index != string::npos) {
    size_t index2 = str.find_first_of(',',index);
    if(index2 == string::npos) {
      argText.push_back(str.substr(index));
      index = string::npos;
    }
    else {
      argText.push_back(str.substr(index,index2-index));
      index = index2+1;
    }
  }
  names.clear();
  args.resize(argText.size());
  for(size_t i=0;i<argText.size();i++) {
    //strip whitespace
    argText[i] = Strip(argText[i]);
    if(argText[i].empty()) {
      return false;
    }
    if(argText[i][0] == '\"' && argText[i][argText[i].length()-1] == '\"') {
      //quoted string
      args[i] = TranslateEscapes(argText[i].substr(1,argText[i].length()-2));
    }
    else if(argText[i].find('=') != string::npos) {  //an assignment
      size_t index=argText[i].find('=');
      string name=Strip(argText[i].substr(0,index-1));
      string arg=Strip(argText[i].substr(index+1));
      if(name.empty()) return false;
      if(arg.empty()) return false;
      if(names.count(name) != 0) return false;  //duplicate named argument
      names[name] = i;
      stringstream ss(arg);
      ss>>args[i];
      if(!ss) return false;
    }
    else {
      stringstream ss(argText[i]);
      ss>>args[i];
      if(!ss) return false;
    }
  }
  return true;
}

PrimitiveValue& ParamList::operator [] (const string& name)
{
  map<string,int>::iterator i = names.find(name);
  if(i == names.end()) {
    names[name] = args.size();
    args.resize(args.size()+1);
    return args.back();
  }
  return args[i->second];
}

const PrimitiveValue& ParamList::operator [] (const string& name) const
{
  map<string,int>::const_iterator i = names.find(name);
  Assert(i != names.end());
  return args[i->second];
}

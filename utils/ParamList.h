#ifndef PARAM_LIST_H
#define PARAM_LIST_H

#include "PrimitiveValue.h"
#include <vector>
#include <map>
#include <string>

struct ParamList
{
  ParamList();
  ParamList(const PrimitiveValue& a1);
  ParamList(const PrimitiveValue& a1,const PrimitiveValue& a2);
  ParamList(const PrimitiveValue& a1,const PrimitiveValue& a2,const PrimitiveValue& a3);
  ParamList(const PrimitiveValue& a1,const PrimitiveValue& a2,const PrimitiveValue& a3,const PrimitiveValue& a4);
  ParamList(const std::vector<PrimitiveValue>& args);
  ParamList(const std::map<string,PrimitiveValue>& args);
  bool parse(const string& str);
  string write() const;
  inline size_t size() const { return args.size(); }
  inline bool empty() const { return args.empty(); }
  inline void clear() { args.clear(); names.clear(); }
  inline bool contains(const std::string& name) const { return names.find(name) != names.end(); }
  inline PrimitiveValue& operator [] (int index) { return args[index]; }
  inline const PrimitiveValue& operator [] (int index) const { return args[index]; }
  PrimitiveValue& operator [] (const std::string& name);
  const PrimitiveValue& operator [] (const std::string& name) const;
  
  std::vector<PrimitiveValue> args;
  std::map<std::string,int> names;
};

#endif

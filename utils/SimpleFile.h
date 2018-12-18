#ifndef SIMPLE_FILE_H
#define SIMPLE_FILE_H

#include <map>
#include <vector>
#include <string>
#include <iosfwd>
#include "PrimitiveValue.h"

/** @brief A simple file format that has named items and whitespace-delimited
 * values.
 *
 * Files are a list of endline-separated item-values pairs.  All values
 * on a line are input into the value.
 * Values beginning with " are read as a string until a closing ".
 * Comments are denoted by # and run until an endline.
 */
class SimpleFile
{
 public:
  SimpleFile(const char* fn);
  SimpleFile(istream& in);
  SimpleFile();
  bool Load(const char* fn);
  bool Load(istream& in);
  bool Save(const char* fn);
  bool Save(ostream& out);
  operator bool () const { return loaded; }
  void AllowItem(const std::string& str,bool caseSensitive=false);

  //accessors
  inline bool empty() { return entries.empty(); }
  inline size_t count(const string& name) { return entries.count(name); }
  inline void erase(const string& name) { entries.erase(entries.find(name)); }
  inline std::vector<PrimitiveValue>& operator [] (const string& name) { return entries[name]; }
  inline std::vector<PrimitiveValue>& operator [] (const char* name) { return entries[string(name)]; }

  //size and type-checking utilities
  bool CheckSize(const string& name,int size,const char* errorString=NULL);
  bool CheckType(const string& name,int type,const char* errorString=NULL);
  std::vector<int> AsInteger(const string& name);
  std::vector<double> AsDouble(const string& name);
  std::vector<std::string> AsString(const string& name);

  bool loaded;
  std::map<std::string,std::vector<PrimitiveValue> > entries;
  std::map<std::string,bool> validItems;
};

#endif

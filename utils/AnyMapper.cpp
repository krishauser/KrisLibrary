#include <KrisLibrary/Logger.h>
#include "AnyMapper.h"
#include <iostream>
using namespace std;

bool Match(const AnyCollection& item,const AnyCollection& schema)
{
  AnyCollection temp;
  return Match(item,schema,temp);
}

bool Match(const AnyCollection& item,const AnyCollection& schema,AnyCollection& wildcardValues)
{
  if(schema.depth()==0) { //it's a primitive value
    if(item.size()==0) { //it's empty
      return false;
    }
    Wildcard val(0);
    if(schema.as<Wildcard>(val)) {
      if(val.id == -1) //match everything
	return true;
      else {
	if(wildcardValues.find(val.id) != NULL) {
	  //item must be equal to the existing value
	  if(wildcardValues[val.id] != item) {
	    LOG4CXX_INFO(KrisLibrary::logger(),"Match: Invalid match on existing wildcard "<<val.id);
	    return false;
	  }
	  return true;
	}
	else {
	  wildcardValues[val.id] = item;
	  return true;
	}
      }
    }
    if(LexicalCast((const AnyValue&)schema) != LexicalCast((const AnyValue&)item)) {
      return false;
    }
    return true;
  }
  if(item.size() != schema.size()) {
    return false;
  }
  vector<AnyKeyable> itemkeys;
  vector<AnyKeyable> schemakeys;
  item.enumerate_keys(itemkeys);
  schema.enumerate_keys(schemakeys);
  if(itemkeys != schemakeys) {
    return false;
  }
  for(size_t i=0;i<itemkeys.size();i++)
    if(!Match(*item.find(itemkeys[i]),*schema.find(itemkeys[i]),wildcardValues)) {
      return false;
    }
  return true;
}

bool Fill(const AnyCollection& schema,const AnyCollection& wildcardValues,AnyCollection& out)
{
  if(schema.depth()==0) {
    if(schema.size()==0) //it's empty
      return true;

    Wildcard val(0);
    if(schema.as<Wildcard>(val)) {
      if(wildcardValues.find(val.id) == NULL) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Fill: wildcard "<<val.id<<" was not matched");
	return false;
      }
      out = *wildcardValues.find(val.id);
      return true;
    }
    out = schema;
    return true;
  }
  vector<AnyKeyable> schemakeys;
  schema.enumerate_keys(schemakeys);
  for(size_t i=0;i<schemakeys.size();i++) {
    AnyCollection val;
    if(!Fill(*schema.find(schemakeys[i]),wildcardValues,val)) 
      return false;
    out[schemakeys[i]] = val;
  }
  return true;
}

bool MatchAndFill(const AnyCollection& in,const AnyCollection& inschema,const AnyCollection& outschema,AnyCollection& out)
{
  AnyCollection matches;
  if(!Match(in,inschema,matches)) return false;
  if(!Fill(outschema,matches,out)) return false;
  return true;
}

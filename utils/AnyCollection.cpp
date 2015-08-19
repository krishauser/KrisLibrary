#include "AnyCollection.h"
#include "stringutils.h"
#include "ioutils.h"
#include "IndexSet.h"
#include <utils.h>
#include <errors.h>
#include <sstream>

bool ReadValue(AnyValue& value,std::istream& in,const std::string& delims)
{
  EatWhitespace(in);
  if(!in) {
    printf("ReadValue: hit end of file\n");
    return false;
  }
  if(in.peek() == '"') {
    //beginning of string
    std::string str;
    if(!InputQuotedString(in,str)) {
      printf("ReadValue: unable to read quoted string\n");
      return false;
    }
    value = str;
    return true;
  }
  else if(in.peek() == '\'') {
    //character: TODO: translate escapes properly
    in.get();
    char c=in.get();
    value = c;
    char end=in.get();
    if(end != '\'') {
      printf("ReadValue: character not delimited properly\n");
      return false;
    }
    return true;
  }
  else {
    std::string str;
    if(delims.empty())
      in >> str;
    else {
      while(in && delims.find(in.peek()) == std::string::npos && !isspace(in.peek())) {
	str += in.get();
      }
    }
    if(str.empty()) {
      printf("ReadValue: read an empty string\n");
      return false;
    }
    if(IsValidInteger(str.c_str())) {
      int val;
      std::stringstream ss(str);
      ss>>val;
      value = val;
      return true;
    }
    else if(IsValidFloat(str.c_str())) {
      double val;
      std::stringstream ss(str);
      ss>>val;
      value = val;
      return true;
    }
    std::string lstr=str;
    Lowercase(lstr);
    if(lstr=="true") {
      value = true;
      return true;
    }
    else if(lstr=="false") {
      value = false;
      return true;
    }
    else {
      //check for invalid values
      for(size_t i=0;i<str.length();i++) {
	if(!(isalnum(str[i])||str[i]=='_')) {
	  std::cerr<<"ReadValue: Invalid basic data type \""<<str<<"\""<<std::endl;
	  return false;
	}
      }
      //identifier
      value = str;
      return true;
    }
  }
}

void WriteValue(const AnyValue& value,std::ostream& out)
{
  const std::type_info* type = &value.type();
  if(type == &typeid(bool)) {
    if(*AnyCast<bool>(&value)==true)
      out<<"true";
    else
      out<<"false";
  }
  else if(type == &typeid(char))
    out<<*AnyCast<char>(&value);
  else if(type == &typeid(unsigned char))
    out<<*AnyCast<unsigned char>(&value);
  else if(type == &typeid(int))
    out<<*AnyCast<int>(&value);
  else if(type == &typeid(unsigned int))
    out<<*AnyCast<unsigned int>(&value);
  else if(type == &typeid(float))
    out<<*AnyCast<float>(&value);
  else if(type == &typeid(double))
    out<<*AnyCast<double>(&value);
  else if(type == &typeid(std::string))
    OutputQuotedString(out,*AnyCast<std::string>(&value));
  else out<<"UNKNOWN_TYPE("<<type->name()<<")";
}

template <class T>
bool BasicNumericalCoerceCast(const AnyValue& value,T& result)
{
  const std::type_info* type = &value.type();
  if(type == &typeid(bool)) result=T(*AnyCast<bool>(&value)); 
  else if(type == &typeid(char)) result=T(*AnyCast<char>(&value));
  else if(type == &typeid(unsigned char)) result=T(*AnyCast<unsigned char>(&value));
  else if(type == &typeid(int)) result=T(*AnyCast<int>(&value));
  else if(type == &typeid(unsigned int)) result=T(*AnyCast<unsigned int>(&value));
  else if(type == &typeid(float)) result=T(*AnyCast<float>(&value));
  else if(type == &typeid(double)) result=T(*AnyCast<double>(&value));
  else {
    return false;
  }
  return true;
}

template <> bool CoerceCast<bool>(const AnyValue& value,bool& result) { return BasicNumericalCoerceCast<bool>(value,result); }
template <> bool CoerceCast<char>(const AnyValue& value,char& result) { return BasicNumericalCoerceCast<char>(value,result); }
template <> bool CoerceCast<unsigned char>(const AnyValue& value,unsigned char& result) { return BasicNumericalCoerceCast<unsigned char>(value,result); }
template <> bool CoerceCast<int>(const AnyValue& value,int& result) { return BasicNumericalCoerceCast<int>(value,result); }
template <> bool CoerceCast<unsigned int>(const AnyValue& value,unsigned int& result) { return BasicNumericalCoerceCast<unsigned int>(value,result); }
template <> bool CoerceCast<float>(const AnyValue& value,float& result) { return BasicNumericalCoerceCast<float>(value,result); }
template <> bool CoerceCast<double>(const AnyValue& value,double& result) { return BasicNumericalCoerceCast<double>(value,result); }

template <> bool LexicalCast(const AnyValue& value,std::string& result)
{
  const std::type_info* type = &value.type();
  if(type == &typeid(bool)) return LexicalCast(*AnyCast<bool>(&value),result); 
  else if(type == &typeid(char)) return LexicalCast(*AnyCast<char>(&value),result);
  else if(type == &typeid(unsigned char)) return LexicalCast(*AnyCast<unsigned char>(&value),result);
  else if(type == &typeid(int)) return LexicalCast(*AnyCast<int>(&value),result);
  else if(type == &typeid(unsigned int)) return LexicalCast(*AnyCast<unsigned int>(&value),result);
  else if(type == &typeid(float)) return LexicalCast(*AnyCast<float>(&value),result);
  else if(type == &typeid(double)) return LexicalCast(*AnyCast<double>(&value),result);
  else if(type == &typeid(std::string)) { result=*AnyCast<std::string>(&value); return true; }
  else {
    return false;
  }
  return true;
}
template <> bool LexicalCast(const std::string& value,AnyValue& result)
{
  std::stringstream ss(value);
  return ReadValue(result,ss,"");
}


AnyKeyable::AnyKeyable()
{}

AnyKeyable::AnyKeyable(const AnyKeyable& rhs)
  :value(rhs.value)
{}

AnyKeyable::AnyKeyable(bool _value)
  :value(_value)
{}

AnyKeyable::AnyKeyable(char _value)
  :value(_value)
{}

AnyKeyable::AnyKeyable(unsigned char _value)
  :value(_value)
{}

AnyKeyable::AnyKeyable(int _value)
  :value(_value)
{}

AnyKeyable::AnyKeyable(unsigned int _value)
  :value(_value)
{}

AnyKeyable::AnyKeyable(float _value)
  :value(_value)
{}

AnyKeyable::AnyKeyable(double _value)
  :value(_value)
{}

AnyKeyable::AnyKeyable(const std::string& _value)
  :value(_value)
{}

AnyKeyable::AnyKeyable(const char* _value)
  :value(std::string(_value))
{}

bool AnyKeyable::operator == (const AnyKeyable& rhs) const
{
  const std::type_info* type = &value.type();
  const std::type_info* rhstype = &rhs.value.type();
  if(type != rhstype) return false;
  if(value.empty()) return true;  
  if(type == &typeid(bool)) 
    return *AnyCast<bool>(&value) == *AnyCast<bool>(&rhs.value);
  if(type == &typeid(char)) 
    return *AnyCast<char>(&value) == *AnyCast<char>(&rhs.value);
  if(type == &typeid(unsigned char)) 
    return *AnyCast<unsigned char>(&value) == *AnyCast<unsigned char>(&rhs.value);
  else if(type == &typeid(int)) 
    return *AnyCast<int>(&value) == *AnyCast<int>(&rhs.value);
  else if(type == &typeid(unsigned int)) 
    return *AnyCast<unsigned int>(&value) == *AnyCast<unsigned int>(&rhs.value);
  else if(type == &typeid(float)) 
    return *AnyCast<float>(&value) == *AnyCast<float>(&rhs.value);
  else if(type == &typeid(double)) 
    return *AnyCast<double>(&value) == *AnyCast<double>(&rhs.value);
  else if(type == &typeid(std::string)) 
    return *AnyCast<std::string>(&value) == *AnyCast<std::string>(&rhs.value);
  else
    FatalError("Equality testing of objects of type %s not supported",type->name());
  return 0;
}

size_t AnyKeyable::hash() const
{
  if(value.empty()) return 0;
  const std::type_info* type = &value.type();
  if(type == &typeid(bool)) 
    return HASH_NAMESPACE::hash<bool>()(*AnyCast<bool>(&value));
  if(type == &typeid(char)) 
    return HASH_NAMESPACE::hash<char>()(*AnyCast<char>(&value));
  if(type == &typeid(unsigned char)) 
    return HASH_NAMESPACE::hash<unsigned char>()(*AnyCast<unsigned char>(&value));
  else if(type == &typeid(int)) 
    return HASH_NAMESPACE::hash<int>()(*AnyCast<int>(&value));
  else if(type == &typeid(unsigned int)) 
    return HASH_NAMESPACE::hash<unsigned int>()(*AnyCast<unsigned int>(&value));
  else if(type == &typeid(float)) 
    return HASH_NAMESPACE::hash<float>()(*AnyCast<float>(&value));
  else if(type == &typeid(double)) 
    return HASH_NAMESPACE::hash<double>()(*AnyCast<double>(&value));
  else if(type == &typeid(std::string)) 
    return HASH_NAMESPACE::hash<std::string>()(*AnyCast<std::string>(&value));
  else
    FatalError("Hash keying of objects of type %s not supported",type->name());
  return 0;
}

AnyCollection::AnyCollection()
  :type(None)
{}

AnyCollection::AnyCollection(AnyValue _value)
  :type(Value),value(_value)
{}

AnyCollection::operator const AnyValue& () const
{
  if(type != Value) FatalError("Not of basic value type");
  return value;
}

AnyCollection::operator AnyValue& ()
{
  if(type != Value) FatalError("Not of basic value type");
  return value;
}

bool AnyCollection::asvector(std::vector<AnyValue>& values) const
{
  if(type != Array) return false;
  if(depth() != 1) return false;
  values.resize(array.size());
  for(size_t i=0;i<array.size();i++)
    values[i] = (const AnyValue&)(*array[i]);
  return true;
}

void AnyCollection::resize(size_t n)
{
  if(type == Value) {
    //TODO: check to see if there are integer entries
    FatalError("AnyCollection::resize(): Cannot resize value without clearing first\n");
  }
  if(type == Map) {
    std::cout<<*this<<std::endl;
    //TODO: check to see if there are integer entries
    FatalError("AnyCollection::resize(): Cannot resize map without clearing first\n");
  }
  type = Array;
  array.resize(n,NULL);
  for(size_t i=0;i<n;i++) {
    if(!array[i])
      array[i] = new AnyCollection;
  }
}

void AnyCollection::clear()
{
  type = None;
  array.clear();
  map.clear();
}

SmartPointer<AnyCollection> AnyCollection::lookup(const std::vector<std::string>& path,bool doinsert)
{
  if(path.empty()) {
    SmartPointer<AnyCollection> res = new AnyCollection;
    res->shallow_copy(*this);
    return res;
  }
  SmartPointer<AnyCollection> entry;
  if(type == Array) {
    int index;
    if(!LexicalCast(path[0],index)) {
      if(doinsert) {
	entry = insert(path[0]);
      }
      else {
	return NULL;
      }
    }
    else {
      entry = find(index);
      if(!entry) {
	if(doinsert) {
	  entry = insert(index);
	  Assert(entry != NULL); 
	}
	else return NULL;
      }
    }
  }
  else if(type == Map || type == None) {
    entry = find(path[0]);
    if(!entry) {
      //try casting to int
      int index;
      if(LexicalCast(path[0],index)) {
	entry = find(index);
	if(!entry) {
	  if(doinsert) {
	    entry = insert(index);
	  }
	}
      }
      else {
	if(doinsert) {
	  entry = insert(path[0]);
	}
      }
    }
  }
  else {
    if(doinsert) 
      FatalError("Trying to insert \"%s\" into a primitive value type\n",path[0].c_str());
  }

  if(!entry) {
    Assert(!doinsert);
    return NULL;
  }

  for(size_t i=1;i<path.size();i++) {
    SmartPointer<AnyCollection> child;
    if(entry->type == Array) {
      int index;
      if(!LexicalCast(path[i],index)) {
	return NULL;
      }
      child = entry->find(index);
      if(!child) {
	if(doinsert) {
	  child = entry->insert(index);
	  Assert(child != NULL); 
	}
	else return NULL;
      }
    }
    else if(entry->type == Map || entry->type == None) {
      child = entry->find(path[i]);
      if(!child) {
	//try casting to int
	int index;
	if(LexicalCast(path[i],index)) {
	  child = entry->find(index);
	  if(!child) {
	    if(doinsert) {
	      child = entry->insert(index);
	      Assert(child != NULL);
	    }
	  }
	}
	else {
	  if(doinsert) {
	    child = entry->insert(path[i]);
	  }
	}
      }
    }
    else {
      if(doinsert) 
	FatalError("Trying to insert \"%s\" into a primitive value type\n",path[i].c_str());
    }
    if(!child) {
      Assert(!doinsert);
      return NULL;
    }
    entry = child;
  }
  if(!entry)
    Assert(!doinsert);
  return entry;
}

SmartPointer<AnyCollection> AnyCollection::lookup(const std::vector<AnyKeyable>& path,bool doinsert)
{
  if(path.empty()) {
    SmartPointer<AnyCollection> res = new AnyCollection;
    res->shallow_copy(*this);
    return res;
  }
  SmartPointer<AnyCollection> entry;
  if(doinsert) entry = insert(path[0]);
  else entry = find(path[0]);
  if(!entry) {
    Assert(!doinsert);
    return NULL;
  }
  for(size_t i=1;i<path.size();i++) {
    if(doinsert) entry = entry->insert(path[i]);
    else entry = entry->find(path[i]);
    if(!entry) {
      Assert(!doinsert);
      return NULL;
    }
  }
  return entry;
}

bool AnyCollection::parse_reference(const std::string& reference,std::vector<std::string>& path,char delim,char lbracket,char rbracket)
{
  path.resize(0);
  //can have a . at the beginning of the reference
  int mode = 0; //0 = waiting, 1 = reading member key, 2 = reading array key, 3 = reading quoted member key, 4 = reading quoted array key, 5 = read quoted array key
  size_t start = 0;
  for(size_t i=0;i<reference.size();i++) {
    switch(mode) {
    case 0:
      if(reference[i]==delim) {
	mode = 1;
	start = (int)i+1;
      }
      else if(reference[i]==lbracket) {
	mode = 2;
	start = (int)i+1;
      }
      else {
	if(reference[i]==rbracket) {
	  fprintf(stderr,"AnyCollection::parse_reference: unexpected right bracket in position %d\n",(int)i);
	  return false;
	}
	if(i!=0) {
	  fprintf(stderr,"AnyCollection::parse_reference: unexpected string in position %d\n",(int)i);
	  return false;
	}
	mode = 1;
	start = 0;
      }
      break;
    case 1: //reading member key
      if(reference[i]=='\"') {
	if(i>start+1) {
	  fprintf(stderr,"AnyCollection::parse_reference: unexpected quotation at position %d\n",(int)i);
	  return false;
	}
	mode = 3;
	start = i+1;
      }
      else if(reference[i]==delim || reference[i]==lbracket) { 
	//read out path entry
	if(i<start+1) {
	  fprintf(stderr,"AnyCollection::parse_reference: empty reference string in position %d, start of ref in position %d\n",(int)i,(int)start);
	  return false;
	}
	path.push_back(reference.substr(start,i-start));
	start = (int)i+1;
	if(reference[i]==lbracket) mode = 2;  //ending at lbracket
	else mode = 1; //ending at delim
      }
      else if(reference[i]==rbracket) {
	fprintf(stderr,"AnyCollection::parse_reference: unexpected right bracket in position %d\n",(int)i);
	return false;
      }
      break;
    case 2: //reading array key
      if(reference[i]=='\"') {
	if(i>start+1) {
	  fprintf(stderr,"AnyCollection::parse_reference: unexpected quotation in key at position %d\n",(int)i);
	  return false;
	}
	mode = 4;
	start = i+1;
      }
      else if(reference[i]==rbracket) {
	path.push_back(reference.substr(start,i-start));
	mode = 0;
      }
      else if(reference[i]==lbracket || reference[i]==delim) {
	fprintf(stderr,"AnyCollection::parse_reference: unexpected character %c in key in position %d\n",reference[i],(int)i);
	return false;
      }
      break;
    case 3: //reading quoted member key
      if(reference[i]=='\"') {
	path.push_back(reference.substr(start,i-start));
	mode = 0;
      }
      break;
    case 4: //reading quoted array key
      if(reference[i]=='\"') {
	path.push_back(reference.substr(start,i-start));
	mode = 5;
      }
      break;
    case 5: //end of quoted array key
      if(reference[i]!=rbracket) {
	fprintf(stderr,"AnyCollection::parse_reference: quoted array key must be followed by right bracket at position %d\n",(int)i);
	return false;
      }
      mode = 0;
      break;
    }
  }
  //invalid end modes: 2,3,4,5
  if(mode == 1) { // .member_name
    path.push_back(reference.substr(start,reference.length()-start));
  }
  if(mode >= 2) {
    fprintf(stderr,"AnyCollection::parse_reference: unexpected termination of reference\n");
    return false;
  }
  return true;
}

bool AnyCollection::match_path(const std::vector<std::string>& path,std::vector<AnyKeyable>& key_path) const
{
  key_path.resize(path.size());
  if(path.empty()) return true;
  SmartPointer<AnyCollection> entry;
  if(type == Array) {
    int index;
    if(!LexicalCast(path[0],index)) {
      fprintf(stderr,"AnyCollection::lookup(): invalid array index %s\n",path[0].c_str());
      return false;
    }
    key_path[0] = AnyKeyable(index);
    entry = find(index);
    if(!entry) return false;
  }
  else if(type == Map) {
    entry = find(path[0]);
    if(!entry) {
      //try casting to int
      int index;
      if(LexicalCast(path[0],index)) {
	entry = find(index);
	key_path[0] = AnyKeyable(index);
      }
    }
    else
      key_path[0] = AnyKeyable(path[0]);
    if(!entry) return false;
  }
  else return false;

  for(size_t i=1;i<path.size();i++) {
    SmartPointer<AnyCollection> child;
    if(type == Array) {
      int index;
      if(!LexicalCast(path[i],index)) {
	fprintf(stderr,"AnyCollection::lookup(): invalid array index %s\n",path[i].c_str());
	return false;
      }
      key_path[i] = AnyKeyable(index);
      child = entry->find(index);
    }
    else if(type == Map) {
      child = entry->find(path[i]);
      if(!child) {
	//try casting to int
	int index;
	if(LexicalCast(path[i],index)) {
	  child = entry->find(index);
	  key_path[i] = AnyKeyable(index);
	}
      }
      else
	key_path[i] = AnyKeyable(path[i]);
    }
    if(!child) return false;
    entry = child;
  }
  return true;
}

SmartPointer<AnyCollection> AnyCollection::lookup(const std::string& reference,bool insert,char delim,char lbracket,char rbracket)
{
  std::vector<std::string> path;
  if(!parse_reference(reference,path,delim,lbracket,rbracket)) {
    fprintf(stderr,"AnyCollection::lookup: unable to parse reference string %s\n",reference.c_str());
    return NULL;
  }
  return lookup(path,insert);
}

SmartPointer<AnyCollection> AnyCollection::slice(const std::string& reference,const char* delims)
{
  SmartPointer<AnyCollection> res;
  if(reference.empty()) {
    res = new AnyCollection;
    res->shallow_copy(*this);
    return res;
  }
  char member = delims[0];
  char lbracket = delims[1];
  char rbracket = delims[2];
  char colon = delims[3];
  char comma = delims[4];
  if(reference[0]==member) {
    if(type != Map) {
      fprintf(stderr,"AnyCollection: slice reference %s in a non-map type\n",reference.c_str());
      return NULL;
    }
    //parse out the key lookup
    int pos = reference.length();
    for(size_t i=1;i<reference.length();i++)
      if(reference[i] == lbracket || reference[i] == member) {
	pos = (int)i;
      }
    std::string key = reference.substr(1,pos-1);
    SmartPointer<AnyCollection> res=find(key);
    if(res) return res->slice(reference.substr(pos,reference.length()-pos),delims);
    return NULL;
  }
  else if(reference[0]==lbracket) {
    //parse until rbracket is found
    int pos = -1;
    for(size_t i=1;i<reference.length();i++)
      if(reference[i] == rbracket) {
	pos = (int)i;
      }
    if(pos < 0) {
      fprintf(stderr,"AnyCollection: lookup reference %s has unterminated bracket\n",reference.c_str());
      return NULL;
    }
    std::string key = reference.substr(1,pos-1);
    bool complexKey = false;
    //TODO: determine whether the key is complex or simple
    if(!complexKey) {
      if(type == Array) {
	//cast to an integer
	if(!IsValidInteger(key.c_str())) {
	  fprintf(stderr,"AnyCollection: lookup index %s is not a valid integer\n",key.c_str());
	  return NULL;
	}
	int index;
	std::stringstream ss(key);
	ss>>index;
	SmartPointer<AnyCollection> res=find(index);
	if(res) return res->slice(reference.substr(pos+1,reference.length()-pos-1),delims);
	return NULL;
      }
      else {
	//lookup 
	SmartPointer<AnyCollection> res=find(key);
	if(res) return res->slice(reference.substr(pos+1,reference.length()-pos-1),delims);
	return NULL;
      }
    }
    else {
      //complex key
      res = new AnyCollection;
      //TODO split keys into subsets
      std::string slice1,slice2;
      std::vector<std::string> elements;
      if(type == Array) {
	res->type = Array;
	int islice1=-1,islice2=1;
	std::vector<int> ielements;
	//TODO parse into integers
	if(ielements.empty()) {
	  if(islice1 < 0) {
	    islice1 = islice1 + (int)array.size();
	    if(islice1 < 0) {
	      fprintf(stderr,"AnyCollection: Invalid array index %d\n",islice1-(int)array.size());
	      return NULL;
	    }
	  }
	  else if (islice1 >= (int)array.size()) {
	    fprintf(stderr,"AnyCollection: Invalid array index %d\n",islice1);
	    return NULL;
	  }
	  if(islice2 < 0) {
	    islice2 = islice2 + (int)array.size();
	    if(islice2 < 0) {
	      fprintf(stderr,"AnyCollection: Invalid array index %d\n",islice2-(int)array.size());
	      return NULL;
	    }
	  }
	  else if (islice2 >= (int)array.size()) {
	    fprintf(stderr,"AnyCollection: Invalid array index %d\n",islice2);
	    return NULL;
	  }
	  if(islice1 > islice2) {
	    fprintf(stderr,"AnyCollection: Invalid array slice %d:%d\n",islice1,islice2);
	    return NULL;
	  }
	  for(int i=islice1;i<islice2;i++) {
	    res->array.push_back(array[i]->slice(reference.substr(pos+1,reference.length()-pos-1),delims));
	  }
	  return res;
	}
	else {
	  //go through the elements
	  for(size_t i=0;i<ielements.size();i++) {
	    if(ielements[i] < 0 || ielements[i] >= (int)array.size()) {
	      fprintf(stderr,"AnyCollection: Invalid array index %d\n",ielements[i]);
	      return NULL;
	    }
	    res->array.push_back(array[ielements[i]]->slice(reference.substr(pos+1,reference.length()-pos-1),delims));
	  }
	}
      }
      else {
	FatalError("TODO: slice in a Map");
      }
    }
    FatalError("Not done yet");
    return NULL;
  }
  else {
    fprintf(stderr,"AnyCollection: cannot lookup reference %s in a primitive type\n",reference.c_str());
    return NULL;
  }
}

bool AnyCollection::subcollection(const std::vector<std::string>& paths,AnyCollection& subset,const char* delims)
{
  subset.clear();

  char member = delims[0];
  char lbracket = delims[1];
  char rbracket = delims[2];
  char colon = delims[3];
  char comma = delims[4];

  for(size_t i=0;i<paths.size();i++) {
    if(paths[i].empty()) //matches whole thing
      subset.deepmerge(*this);
    else {
      std::vector<std::string> path;
      if(!parse_reference(paths[i],path,member,lbracket,rbracket)) {
	fprintf(stderr,"AnyCollection::subcollection(): error parsing path %s\n",paths[i].c_str());
	return false;
      }
      //parse strings into ints if necessary
      std::vector<AnyKeyable> key_path(path.size());
      for(size_t j=0;j<path.size();j++) {
	if(IsValidInteger(path[j].c_str())) {
	  int index;
	  LexicalCast(path[j],index);
	  key_path[j] = AnyKeyable(index);
	}
	else
	  key_path[j] = AnyKeyable(path[j]);
      }
      SmartPointer<AnyCollection> item = lookup(key_path);
      if(!item) {
	fprintf(stderr,"AnyCollection::subcollection(): invalid item %s\n",paths[i].c_str());
	return false;
      }
      subset.lookup(key_path,true)->deepmerge(*item);
    }
  }   
  return true;
}

SmartPointer<AnyCollection> AnyCollection::find(int i) const
{
  if(type == Array) {
    if(i < 0 || i >= (int)array.size()) return NULL;
    return array[i];
  }
  else if(type == Map) {
    AnyKeyable key(i);
    return find(key);
  }
  return NULL;
}

SmartPointer<AnyCollection> AnyCollection::find(const char* str) const
{
  if(type != Map) return NULL;
  return find(AnyKeyable(std::string(str)));
}

SmartPointer<AnyCollection> AnyCollection::find(AnyKeyable key) const
{
  if(type == Array) {
    if(key.value.hastype<int>())
      return find(*AnyCast<int>(&key.value));
    else if(key.value.hastype<unsigned int>())
      return find(*AnyCast<unsigned int>(&key.value));
    else 
      return NULL;
  }
  else if(type == Map) {
    MapType::const_iterator i=map.find(key);
    if(i == map.end()) return NULL;
    return i->second;
  }
  return NULL;
}

SmartPointer<AnyCollection> AnyCollection::insert(int index)
{
  if(type == None) {
    if(index==0) { // first array reference
      type = Array;
      array.resize(0);
    }
    else {  // first map reference
      type = Map;
      map.clear();
    }
  }
  if(type == Array) {
    if(index == (int)array.size()) { //resize by one
      size_t start = array.size();
      array.resize(index+1);
      for(int i=start;i<(int)array.size();i++)
	array[i] = new AnyCollection();
    }
    else if(index > (int)array.size()) {
      //convert to a map
      //cast array to a map
      type = Map;
      map.clear();
      for(size_t i=0;i<array.size();i++)
	map[(int)i] = array[i];
      array.clear();
      map[index] = new AnyCollection();
      return map[index];
    }
    return array[index];
  }
  else if(type == Map) {
    AnyKeyable key(index);
    return insert(key);
  }
  FatalError("AnyCollection: Can't insert into non-collection types");
  return NULL;
}

SmartPointer<AnyCollection> AnyCollection::insert(const char* str)
{
  return insert(AnyKeyable(std::string(str)));
}

SmartPointer<AnyCollection> AnyCollection::insert(AnyKeyable key)
{
  if(type == None) {
    //if the key is an integer, try turning it into an array
    if(key.value.hastype<int>())
      return insert(*AnyCast<int>(&key.value));
    else if(key.value.hastype<unsigned int>())
      return insert((int)(*AnyCast<unsigned int>(&key.value)));

    //coerce to a map type
    type = Map;
    map.clear();
  }
  if(type == Array) {
    if(!key.value.hastype<int>() && !key.value.hastype<unsigned int>()) {
      //cast array to a map
      type = Map;
      map.clear();
      for(size_t i=0;i<array.size();i++)
	map[(int)i] = array[i];
      array.clear();
    }
  }

  if(type == Array) {
    int index;
    if(key.value.hastype<int>())
      return insert(*AnyCast<int>(&key.value));
    else if(key.value.hastype<unsigned int>())
      return insert((int)(*AnyCast<unsigned int>(&key.value)));
    else {
      FatalError("AnyCollection: can't lookup arrays with non-integer types");
      return NULL;
    }
    if(index >= (int)array.size()) {
      size_t start = array.size();
      array.resize(index+1);
      for(size_t i=start;i<array.size();i++)
	array[i] = new AnyCollection();
    }
    return array[index];
  }
  else if(type == Map) {
    MapType::iterator i=map.find(key);
    if(i == map.end()) {
      map[key] = new AnyCollection;
      return map[key];
    }
    return i->second;
  }
  FatalError("AnyCollection: Can't lookup non-collection types");
  return NULL;
}

AnyCollection& AnyCollection::operator[](int index)
{
  if(type == None) {
    if(index==0) { // first array reference
      type = Array;
      array.resize(0);
    }
    else {  // first map reference
      type = Map;
      map.clear();
    }
  }

  if(type == Array) {
    if(index >= (int)array.size()) { //resize 
      size_t start = array.size();
      array.resize(index+1);
      for(size_t i=start;i<array.size();i++)
	array[i] = new AnyCollection();
    }
    return *array[index];
  }
  else if(type == Map) {
    AnyKeyable key(index);
    return operator[](key);
  }
  FatalError("AnyCollection: Can't index into non-collection types");
  return *this;
}

const AnyCollection& AnyCollection::operator[](int i) const
{
  if(type == Array)
    return *array[i];
  else if(type == Map) {
    AnyKeyable key(i);
    return operator[](key);
  }
  FatalError("AnyCollection: Can't index into non-collection types");
  return *this;
}

AnyCollection& AnyCollection::operator[](const char* str)
{
  return operator [](AnyKeyable(std::string(str)));
}

const AnyCollection& AnyCollection::operator[](const char* str) const
{
  return operator [](AnyKeyable(std::string(str)));
}


AnyCollection& AnyCollection::operator[](AnyKeyable key)
{
  if(type == None) {
    //coerce to a map type
    type = Map;
    map.clear();
  }
  if(type == Array) {
    if(!key.value.hastype<int>() && !key.value.hastype<unsigned int>()) {
      //cast array to a map
      type = Map;
      map.clear();
      for(size_t i=0;i<array.size();i++)
	map[(int)i] = array[i];
      array.clear();
    }
  }

  if(type == Array) {
    int index;
    if(key.value.hastype<int>())
      index = *AnyCast<int>(&key.value);
    else if(key.value.hastype<unsigned int>())
      index = (int)(*AnyCast<unsigned int>(&key.value));
    else {
      FatalError("AnyCollection: can't lookup arrays with non-integer types");
      return *this;
    }
    if(index >= (int)array.size()) {
      size_t start = array.size();
      array.resize(index+1);
      for(int i=start;i<(int)array.size();i++)
	array[i] = new AnyCollection();
    }
    return *array[index];
  }
  else if(type == Map) {
    MapType::iterator i=map.find(key);
    if(i == map.end()) {
      map[key] = new AnyCollection;
      return *map[key];
    }
    return *i->second;
  }
  FatalError("AnyCollection: Can't lookup non-collection types");
  return *this;
}

const static AnyCollection nullCollection;

const AnyCollection& AnyCollection::operator[](AnyKeyable key) const
{
  if(type == Array) {
    if(key.value.hastype<int>())
      return *array[*AnyCast<int>(&key.value)];
    else if(key.value.hastype<unsigned int>())
      return *array[*AnyCast<unsigned int>(&key.value)];
    else {
      return nullCollection;
      FatalError("AnyCollection: can't lookup arrays with non-integer types");
      return *this;
    }
  }
  else if(type == Map) {
    MapType::const_iterator i=map.find(key);
    if(i == map.end()) {
      return nullCollection;
      std::cerr<<"AnyCollection: const [] accessor can't find key";
      WriteValue(key.value,std::cerr);
      std::cerr<<std::endl;
      Abort();
      return *this;
    }
    return *i->second;
  }
  return nullCollection;
  FatalError("AnyCollection: Can't lookup non-collection types");
  return *this;

}

void AnyCollection::shallow_copy(const AnyCollection& rhs)
{
  type = rhs.type;
  value = rhs.value;
  array = rhs.array;
  map = rhs.map;
}

void AnyCollection::deep_copy(const AnyCollection& rhs)
{
  clear();
  type = rhs.type;
  if(type == Value) {
    value = rhs.value;
  }
  else if(type == Array) {
    array.resize(rhs.array.size());
    for(size_t i=0;i<rhs.array.size();i++) {
      array[i] = new AnyCollection;
      array[i]->deep_copy(*rhs.array[i]);
    }
  }
  else if(type == Map) {
    for(MapType::const_iterator i=rhs.map.begin();i!=rhs.map.end();i++) {
      map[i->first] = new AnyCollection;
      map[i->first]->deep_copy(*i->second);
    }
  }
}

AnyCollection& AnyCollection::operator = (const AnyCollection& rhs)
{
  shallow_copy(rhs);
  return *this;
}

AnyCollection& AnyCollection::operator = (AnyValue _value)
{
  type = Value;
  value = _value;
  return *this;
}

size_t AnyCollection::size() const
{
  if(type == Value) return 1;
  else if(type == Array) return array.size();
  else if(type == Map) return map.size();
  else return 0;
}

bool AnyCollection::null() const
{
  return type == None; 
}

bool AnyCollection::collection() const
{
  return (type == Array || type == Map);
}

bool AnyCollection::isvalue() const
{
  return type == Value;
}

bool AnyCollection::isarray() const
{
  return type == Array;
}

bool AnyCollection::ismap() const
{
  return type == Map;
}

size_t AnyCollection::depth() const
{
  if(type == Value) return 0;
  else if(type == Array) {
    size_t maxD = 0;
    for(size_t i=0;i<array.size();i++)
      maxD = Max(maxD,array[i]->depth());
    return maxD + 1;
  }
  else if(type == Map) {
    size_t maxD = 0;
    for(MapType::const_iterator i=map.begin();i!=map.end();i++)
      maxD = Max(maxD,i->second->depth());
    return maxD + 1;
  }
  return -1;
}

void AnyCollection::enumerate(std::vector<SmartPointer<AnyCollection> >& collections) const
{
  collections.resize(0);
  if(type == Array) {
    collections = array;
  }
  else if(type == Map) {
    collections.resize(map.size());
    int k=0;
    for(MapType::const_iterator i=map.begin();i!=map.end();i++,k++)
      collections[k] = i->second;
  }
}

void AnyCollection::enumerate_keys(std::vector<AnyKeyable>& keys) const
{
  if(type == Array) {
    for(size_t i=0;i<array.size();i++)
      keys.push_back((int)i);
  }
  else if(type == Map) {
    for(MapType::const_iterator i=map.begin();i!=map.end();i++)
      keys.push_back(i->first);
  }
}

void AnyCollection::enumerate_values(std::vector<AnyValue>& elements) const
{
  if(type == Value) 
    elements.push_back(value);
  else if(type == Array) {
    for(size_t i=0;i<array.size();i++) {
      if(array[i]->type == Value)
	elements.push_back(array[i]->value);
    }
  }
  else if(type == Map) {
    for(MapType::const_iterator i=map.begin();i!=map.end();i++)
      if(i->second->type == Value)
	elements.push_back(i->second->value);
  }
}

void AnyCollection::enumerate_keys_dfs(std::vector<std::vector<AnyKeyable> >& paths) const
{
  if(type == Value) {
    paths.resize(1);
    paths[0].clear();
  }
  else if(type == Array) {
    for(size_t i=0;i<array.size();i++) {
      std::vector<std::vector<AnyKeyable> > subpaths;
      array[i]->enumerate_keys_dfs(subpaths);
      for(size_t j=0;j<subpaths.size();j++) {
	paths.resize(paths.size()+1);
	paths.back().resize(subpaths[j].size()+1);
	paths.back()[0] = AnyKeyable(int(i));
	copy(subpaths[j].begin(),subpaths[j].end(),paths.back().begin()+1);
      }
    }
  }
  else if(type == Map) {
    for(MapType::const_iterator i=map.begin();i!=map.end();i++) {
      std::vector<std::vector<AnyKeyable> > subpaths;
      i->second->enumerate_keys_dfs(subpaths);
      for(size_t j=0;j<subpaths.size();j++) {
	paths.resize(paths.size()+1);
	paths.back().resize(subpaths[j].size()+1);
	paths.back()[0] = AnyKeyable(i->first);
	copy(subpaths[j].begin(),subpaths[j].end(),paths.back().begin()+1);
      }
    }
  }
}

void AnyCollection::enumerate_values_dfs(std::vector<AnyValue>& elements) const
{
  if(type == Value) 
    elements.push_back(value);
  else if(type == Array) {
    for(size_t i=0;i<array.size();i++)
      array[i]->enumerate_values_dfs(elements);
  }
  else if(type == Map) {
    for(MapType::const_iterator i=map.begin();i!=map.end();i++)
      i->second->enumerate_values_dfs(elements);
  }
}

void AnyCollection::merge(const AnyCollection& other)
{
  Assert(collection() && other.collection());
  if(type == Array) {
    if(other.type == Map) {
      //convert this to a map
      for(size_t i=0;i<array.size();i++)
	map[(int)i] = array[i];
      array.clear();
      type = Map;
      for(MapType::const_iterator i=other.map.begin();i!=other.map.end();i++)
	map[i->first] = i->second;
    }
    else {
      array = other.array;
    }
  }
  else {
    if(other.type == Array) {
      for(size_t i=0;i<other.array.size();i++)
	map[(int)i] = other.array[i];
    }
    else {
      for(MapType::const_iterator i=other.map.begin();i!=other.map.end();i++)
	map[i->first] = i->second;
    }
  }
}

void AnyCollection::deepmerge(const AnyCollection& other)
{
  Assert(collection() && other.collection());
  if(type == Array) {
    if(other.type == Map) {
      //convert this to a map
      for(size_t i=0;i<array.size();i++) 
	map[(int)i] = array[i];
      array.clear();
      type = Map;
      for(MapType::const_iterator i=other.map.begin();i!=other.map.end();i++) {
	SmartPointer<AnyCollection>& lhs = map[i->first];
	if(i->second->collection() && lhs->collection())
	  lhs->deepmerge(*i->second);
	else
	  lhs = i->second;
      }
    }
    else {
      if(other.array.size() > array.size()) 
	array.resize(other.array.size());
      for(size_t i=0;i<other.array.size();i++) {
	if(array[i]->collection() && other.array[i]->collection())
	  array[i]->deepmerge(*other.array[i]);
	else
	  array[i] = other.array[i];
      }
    }
  }
  else {
    if(other.type == Array) {
      for(size_t i=0;i<other.array.size();i++) {
	SmartPointer<AnyCollection>& lhs = map[(int)i];
	if(other.array[i]->collection() && lhs->collection())
	  lhs->deepmerge(*other.array[i]);
	else
	  lhs = other.array[i];
      }
    }
    else {
      for(MapType::const_iterator i=other.map.begin();i!=other.map.end();i++) {
	SmartPointer<AnyCollection>& lhs = map[i->first];
	if(i->second->collection() && lhs->collection())
	  lhs->deepmerge(*i->second);
	else
	  lhs = i->second;
      }
    }
  }
}

bool AnyCollection::fill(AnyCollection& universe,bool checkSuperset)
{
  if(!collection()) {
    *this = universe;
    return true;
  }
  if(type == Array) {
    if(universe.type != Array) return false;
    if(universe.array.size() < array.size()) {
      if(checkSuperset) return false;
      for(size_t i=0;i<universe.array.size();i++) {
	if(!array[i]->fill(*universe.array[i],checkSuperset)) return false;
      }
    }
    else {
      for(size_t i=0;i<array.size();i++) {
	if(!array[i]->fill(*universe.array[i],checkSuperset)) return false;
      }
    }
  }
  else {
    if(universe.type != Map) return false;
    for(MapType::iterator i=map.begin();i!=map.end();i++) {
      MapType::iterator j = universe.map.find(i->first);
      if(j == universe.map.end()) {
	if(checkSuperset) return false;
      }
      else {
	if(!i->second->fill(*j->second,checkSuperset)) return false;
      }
    }
  }
  return true;
}

bool AnyCollection::read(const char* data)
{
  std::stringstream ss(data);
  return read(ss);
}

bool AnyCollection::read(std::istream& in)
{
  clear();
  EatWhitespace(in);
  if(in.peek()=='[') {
    in.get();
    type = Array;
    EatWhitespace(in);
    if(in.peek()==']') {
      //empty array
      in.get();
      return true;
    }
    //read list
    SmartPointer<AnyCollection> value;
    value = new AnyCollection();
    if(!value->read(in)) {
      fprintf(stderr,"AnyCollection(): read failed on array item %d\n",(int)array.size());
      return false;
    }
    array.push_back(value);
    EatWhitespace(in);
    while(in.peek() != ']') {
      if(in.get() != ',') {
	std::cerr<<"List not separated by commas"<<std::endl;
	return false;
      }
      value = new AnyCollection();
      if(!value->read(in)) {
	fprintf(stderr,"AnyCollection(): read failed on array item %d\n",(int)array.size());
	return false;
      }
      array.push_back(value);
      EatWhitespace(in);
    }
    if(!in) {
      fprintf(stderr,"AnyCollection(): file ended before end-of-list item %d\n",(int)array.size());
      return false;
    }
    in.get();
    return true;
  }
  else if(in.peek()=='{') {
    in.get();
    type = Map;
    EatWhitespace(in);
    if(in.peek() == '}') {
      //empty map
      in.get();
      return true;
    }
    //read list
    AnyKeyable key;
    SmartPointer<AnyCollection> value;
    while(true) {
      if(!ReadValue(key.value,in,":")) {
	fprintf(stderr,"AnyCollection(): read failed on map item %d\n",(int)map.size());
	return false;
      }
      EatWhitespace(in);
      if(in.peek() != ':') {
	std::cerr<<"Map missing a colon-separator between key-value pair ";
	WriteValue(key.value,std::cerr);
	std::cerr<<std::endl;
	return false;
      }
      in.get();
      value = new AnyCollection();
      if(!value->read(in)) {
	std::cerr<<"AnyCollection(): couldn't read map value for key ";
	WriteValue(key.value,std::cerr);
	std::cerr<<std::endl;
	return false;
      }
      map[key] = value;
      EatWhitespace(in);
      char c = in.get();
      if(c == '}') return true;
      if(c != ',') {
	std::cerr<<"Map entries not separated by commas"<<std::endl;
	return false;
      }
    }
  }
  else {
    //could be part of a list or map
    type = Value;
    if(!ReadValue(value,in,",]}")) {
      std::cerr<<"AnyCollection: Unable to read primitive value"<<std::endl;
      return false;
    }
    return true;
  }
  return false;
}

void AnyCollection::write_inline(std::ostream& out) const
{
  if(type == None) out<<"null";
  else if(type == Value) {
    WriteValue(value,out);
  }
  else if(type == Array) {
    out<<"[";
    for(size_t i=0;i<array.size();i++) {
      if(i!=0) out<<", ";
      array[i]->write_inline(out);
    }
    out<<"]";
  }
  else {
    //map
    out<<"{";
    for(MapType::const_iterator i=map.begin();i!=map.end();i++) {
      if(i!=map.begin()) out<<", ";
      WriteValue(i->first.value,out);
      out<<":";
      i->second->write_inline(out);
    }
    out<<"}";
  }
}

void AnyCollection::write(std::ostream& out,int indent) const
{
  if(type == None) out<<"null";
  else if(type == Value) {
    WriteValue(value,out);
  }
  else if(type == Array) {
    bool write_inline = (depth() == 1); //raw array, write as inline
    out<<"[";
    for(size_t i=0;i<array.size();i++) {
      if(i!=0) out<<", ";
      if(!write_inline)	out<<std::endl<<std::string(indent+2,' ');
      array[i]->write(out,indent+2);
    }
    if(!write_inline)	out<<std::endl<<std::string(indent,' ');
    out<<"]";
  }
  else {
    bool write_inline = (depth() == 1); //raw map, write as inline
    //map
    out<<"{";
    for(MapType::const_iterator i=map.begin();i!=map.end();i++) {
      if(i!=map.begin()) out<<", ";
      if(!write_inline) out<<std::endl<<std::string(indent+2,' ');
      WriteValue(i->first.value,out);
      out<<": ";
      i->second->write(out,indent+2);
    }
    if(!write_inline) out<<std::endl<<std::string(indent,' ');
    out<<"}";
  }
}

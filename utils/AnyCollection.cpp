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
	  std::cerr<<"Invalid basic data type \""<<str<<"\""<<std::endl;
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
  if(value.type() == typeid(bool)) {
    if(*AnyCast<bool>(&value)==true)
      out<<"true";
    else
      out<<"false";
  }
  else if(value.type() == typeid(char))
    out<<*AnyCast<char>(&value);
  else if(value.type() == typeid(unsigned char))
    out<<*AnyCast<unsigned char>(&value);
  else if(value.type() == typeid(int))
    out<<*AnyCast<int>(&value);
  else if(value.type() == typeid(unsigned int))
    out<<*AnyCast<unsigned int>(&value);
  else if(value.type() == typeid(float))
    out<<*AnyCast<float>(&value);
  else if(value.type() == typeid(double))
    out<<*AnyCast<double>(&value);
  else if(value.type() == typeid(std::string))
    OutputQuotedString(out,*AnyCast<std::string>(&value));
  else out<<"UNKNOWN_TYPE("<<value.type().name()<<")";
}

template <class T>
bool BasicNumericalCoerceCast(const AnyValue& value,T& result)
{
  if(value.type() == typeid(bool)) result=T(*AnyCast<bool>(&value)); 
  else if(value.type() == typeid(char)) result=T(*AnyCast<char>(&value));
  else if(value.type() == typeid(unsigned char)) result=T(*AnyCast<unsigned char>(&value));
  else if(value.type() == typeid(int)) result=T(*AnyCast<int>(&value));
  else if(value.type() == typeid(unsigned int)) result=T(*AnyCast<unsigned int>(&value));
  else if(value.type() == typeid(float)) result=T(*AnyCast<float>(&value));
  else if(value.type() == typeid(double)) result=T(*AnyCast<double>(&value));
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
  if(value.type() == typeid(bool)) return LexicalCast(*AnyCast<bool>(&value),result); 
  else if(value.type() == typeid(char)) return LexicalCast(*AnyCast<char>(&value),result);
  else if(value.type() == typeid(unsigned char)) return LexicalCast(*AnyCast<unsigned char>(&value),result);
  else if(value.type() == typeid(int)) return LexicalCast(*AnyCast<int>(&value),result);
  else if(value.type() == typeid(unsigned int)) return LexicalCast(*AnyCast<unsigned int>(&value),result);
  else if(value.type() == typeid(float)) return LexicalCast(*AnyCast<float>(&value),result);
  else if(value.type() == typeid(double)) return LexicalCast(*AnyCast<double>(&value),result);
  else if(value.type() == typeid(std::string)) { result=*AnyCast<std::string>(&value); return true; }
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

bool AnyKeyable::operator == (const AnyKeyable& rhs) const
{
  if(value.type() != rhs.value.type()) return false;
  if(value.empty()) return true;  
  if(value.type() == typeid(bool)) 
    return *AnyCast<bool>(&value) == *AnyCast<bool>(&rhs.value);
  if(value.type() == typeid(char)) 
    return *AnyCast<char>(&value) == *AnyCast<char>(&rhs.value);
  if(value.type() == typeid(unsigned char)) 
    return *AnyCast<unsigned char>(&value) == *AnyCast<unsigned char>(&rhs.value);
  else if(value.type() == typeid(int)) 
    return *AnyCast<int>(&value) == *AnyCast<int>(&rhs.value);
  else if(value.type() == typeid(unsigned int)) 
    return *AnyCast<unsigned int>(&value) == *AnyCast<unsigned int>(&rhs.value);
  else if(value.type() == typeid(float)) 
    return *AnyCast<float>(&value) == *AnyCast<float>(&rhs.value);
  else if(value.type() == typeid(double)) 
    return *AnyCast<double>(&value) == *AnyCast<double>(&rhs.value);
  else if(value.type() == typeid(std::string)) 
    return *AnyCast<std::string>(&value) == *AnyCast<std::string>(&rhs.value);
  else
    FatalError("Equality testing of objects of type %s not supported",value.type().name());
  return 0;
}

size_t AnyKeyable::hash() const
{
  if(value.empty()) return 0;
  if(value.type() == typeid(bool)) 
    return std::tr1::hash<bool>()(*AnyCast<bool>(&value));
  if(value.type() == typeid(char)) 
    return std::tr1::hash<char>()(*AnyCast<char>(&value));
  if(value.type() == typeid(unsigned char)) 
    return std::tr1::hash<unsigned char>()(*AnyCast<unsigned char>(&value));
  else if(value.type() == typeid(int)) 
    return std::tr1::hash<int>()(*AnyCast<int>(&value));
  else if(value.type() == typeid(unsigned int)) 
    return std::tr1::hash<unsigned int>()(*AnyCast<unsigned int>(&value));
  else if(value.type() == typeid(float)) 
    return std::tr1::hash<float>()(*AnyCast<float>(&value));
  else if(value.type() == typeid(double)) 
    return std::tr1::hash<double>()(*AnyCast<double>(&value));
  else if(value.type() == typeid(std::string)) 
    return std::tr1::hash<std::string>()(*AnyCast<std::string>(&value));
  else
    FatalError("Hash keying of objects of type %s not supported",value.type().name());
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
  type = Array;
  array.resize(n);
  for(size_t i=0;i<n;i++)
    array[i] = new AnyCollection;
}

void AnyCollection::clear()
{
  type = None;
  array.clear();
  map.clear();
}

AnyCollection* AnyCollection::lookup(const std::string& reference,char delim,char lbracket,char rbracket)
{
  if(reference.empty()) return this;
  if(reference[0]==delim) {
    if(type != Map) {
      fprintf(stderr,"AnyCollection: lookup reference %s in a non-map type\n",reference.c_str());
      return NULL;
    }
    //parse out the key lookup
    int pos = reference.length();
    for(size_t i=1;i<reference.length();i++)
      if(reference[i] == lbracket || reference[i] == delim) {
	pos = (int)i;
      }
    std::string key = reference.substr(1,pos-1);
    AnyCollection* res=find(key);
    if(res) return res->lookup(reference.substr(pos,reference.length()-pos),delim,lbracket,rbracket);
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
    if(type == Array) {
      //cast to an integer
      if(!IsValidInteger(key.c_str())) {
	fprintf(stderr,"AnyCollection: lookup index %s is not a valid integer\n",key.c_str());
	return NULL;
      }
      int index;
      std::stringstream ss(key);
      ss>>index;
      AnyCollection* res=find(index);
      if(res) return res->lookup(reference.substr(pos+1,reference.length()-pos-1),delim,lbracket,rbracket);
      return NULL;
    }
    else {
      //lookup 
      AnyCollection* res=find(key);
      if(!res) {
	//try parsing res into a keyable type
	AnyKeyable keyable;
	std::stringstream ss(key);
	if(ReadValue(keyable.value,ss,""))
	  res=find(keyable);
      }
      if(res) return res->lookup(reference.substr(pos+1,reference.length()-pos-1),delim,lbracket,rbracket);
      fprintf(stderr,"AnyCollection: lookup index %s doesnt exist\n",key.c_str());
      return NULL;
    }
  }
  else {
    fprintf(stderr,"AnyCollection: cannot lookup reference %s in a primitive type\n",reference.c_str());
    return NULL;
  }
}

SmartPointer<AnyCollection> AnyCollection::slice(const std::string& reference,const char* delims)
{
  SmartPointer<AnyCollection> res;
  if(reference.empty()) {
    res = new AnyCollection;
    res->shallow_copy(*this);
    return res;
  }
  char delim = delims[0];
  char lbracket = delims[1];
  char rbracket = delims[2];
  char colon = delims[3];
  char comma = delims[4];
  if(reference[0]==delim) {
    if(type != Map) {
      fprintf(stderr,"AnyCollection: slice reference %s in a non-map type\n",reference.c_str());
      return NULL;
    }
    //parse out the key lookup
    int pos = reference.length();
    for(size_t i=1;i<reference.length();i++)
      if(reference[i] == lbracket || reference[i] == delim) {
	pos = (int)i;
      }
    std::string key = reference.substr(1,pos-1);
    AnyCollection* res=find(key);
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
	AnyCollection* res=find(index);
	if(res) return res->slice(reference.substr(pos+1,reference.length()-pos-1),delims);
	return NULL;
      }
      else {
	//lookup 
	AnyCollection* res=find(key);
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
	int islice1,islice2;
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
    if(key.value.type() == typeid(int)) 
      return find(*AnyCast<int>(&key.value));
    else if(key.value.type() == typeid(unsigned int))
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

AnyCollection& AnyCollection::operator[](int i)
{
  if(type == None) {
    if(i==0) { // first array reference
      type = Array;
      array.resize(0);
    }
    else {  // first map reference
      type = Map;
      map.clear();
    }
  }

  if(type == Array) {
    if(i == (int)array.size()) { //resize by 1
      array.resize(i+1);
      array[i] = new AnyCollection();
    }
    return *array[i];
  }
  else if(type == Map) {
    AnyKeyable key(i);
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
    if(key.value.type() != typeid(int) && key.value.type() != typeid(unsigned int)) {
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
    if(key.value.type() == typeid(int))
      index = *AnyCast<int>(&key.value);
    else if(key.value.type() == typeid(unsigned int))
      index = (int)(*AnyCast<unsigned int>(&key.value));
    else {
      FatalError("AnyCollection: can't lookup arrays with non-integer types");
      return *this;
    }
    if(index == (int)array.size()) {
      array.resize(index+1);
      array[index] = new AnyCollection();
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

const AnyCollection& AnyCollection::operator[](AnyKeyable key) const
{
  if(type == Array) {
    if(key.value.type() == typeid(int))
      return *array[*AnyCast<int>(&key.value)];
    else if(key.value.type() == typeid(unsigned int))
      return *array[*AnyCast<unsigned int>(&key.value)];
    else {
      FatalError("AnyCollection: can't lookup arrays with non-integer types");
      return *this;
    }
  }
  else if(type == Map) {
    MapType::const_iterator i=map.find(key);
    if(i == map.end()) {
      FatalError("AnyCollection: Can't find key\n");
      return *this;
    }
    return *i->second;
  }
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

bool AnyCollection::collection() const
{
  return (type == Array || type == Map);
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
    if (in.peek() == ']') {
      //empty list
      in.get();
      return true;
    }
    //read list
    SmartPointer<AnyCollection> value;
    value = new AnyCollection();
    if(!value->read(in)) {
      fprintf(stderr,"AnyCollection(): read failed on array item %d\n",array.size());
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
	fprintf(stderr,"AnyCollection(): read failed on array item %d\n",array.size());
	return false;
      }
      array.push_back(value);
      EatWhitespace(in);
    }
    if(!in) {
      fprintf(stderr,"AnyCollection(): file ended before end-of-list item %d\n",array.size());
      return false;
    }
    in.get();
    return true;
  }
  else if(in.peek()=='{') {
    in.get();
    type = Map;
    EatWhitespace(in);
    if (in.peek() == '}') {
      //empty map
      in.get();
      return true;
    }
    //read list
    AnyKeyable key;
    SmartPointer<AnyCollection> value;
    while(true) {
      if(!ReadValue(key.value,in,":")) {
	fprintf(stderr,"AnyCollection(): read failed on map item %d\n",map.size());
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

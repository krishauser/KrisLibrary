#include "ioutils.h"
#include "stringutils.h"
#include <Logger.h>
#include <iostream>
#include <math/infnan.h>
#include <fstream>
#include <limits>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>

//if c is preceded by a \, returns the translated ascii character
int TranslateEscape(int c)
{
  switch(c) {
  case 'a':  return '\a';
  case 'b':  return '\b';
  case 'n':  return '\n';
  case 'r':  return '\r';
  case 't':  return '\t';
  case 'v':  return '\v';
  default:   return c;
  }
}

std::string TranslateEscapes(const std::string& str)
{
  std::string res;
  for(size_t i=0;i+1<str.length();i++) {
    if(str[i]=='\\') {
      res += TranslateEscape(str[i+1]);
      i++;
    }
    else res += str[i];
  }
  res += str[str.length()-1];
  return res;
}

void EatWhitespace(std::istream& in)
{
  while(in && isspace(in.peek())) in.get();
}

bool InputToken(std::istream& in,const char* characterSet,std::string& str)
{
  str.erase();
  int c;
  while(in && (c=in.peek())!=EOF) {
    if(strchr(characterSet,c) != NULL)
      str += c;
    c=in.get();
  }
  return !in.bad();
}

bool InputQuotedString(std::istream& in, char* out, int n)
{
  int c;
  int state=0;  //0 = not yet read begin ", 1 is reading chars until end "
  int i=0;
  while((c=in.peek())!=EOF) {
    switch (state) {
    case 0:
      if(c=='\"')
	state = 1;
      else if(!isspace(c)) return false;
      break;
    case 1:
      if(c=='\"') {
	in.get();
	out[i]='\0';
	return true;
      }
      else if(c=='\\') {
	c=in.get();
	c=in.peek();
	out[i] = c;
      }
      else {
	if(i>=n) return false;
	out[i] = c;
	i++;
      }
      break;
    }
    c=in.get();
  }
  return false;
}

bool InputQuotedString(std::istream& in, std::string& out)
{
  int c;
  int state=0;
  out.erase();
  while((c=in.peek())!=EOF) {
    switch (state) {
      case 0:
	if(c=='\"')
	  state = 1;
	else if(!isspace(c))
	  return false;
	break;
    case 1:
      if(c=='\"') { in.get(); return true; }
      else if(c=='\\') {
	c=in.get();
	c=in.peek();
	out+= c;
      }
      else
	out+= c;
      break;
    }
    c = in.get();
  }
  return false;
}

void OutputQuotedString(std::ostream& out, const char* str)
{
  if(StringContainsQuote(str)) {
    //output quotes using \"
    out<<'\"';
    const char* c=str;
    while(*c) {
      if(*c == '\"') out<<"\\\"";
      else out<<*c;
      c++;
    }
    out<<'\"';
  }
  else
    out<<'\"'<<str<<'\"';
}

void OutputQuotedString(std::ostream& out, const std::string& str)
{
  OutputQuotedString(out,str.c_str());
}

bool StringContainsQuote(const char* str)
{
	return strchr(str,'\"')!=NULL;
}

bool StringContainsQuote(const std::string& str)
{
	return str.rfind('\"')!=str.npos;
}

bool StringRequiresQuoting(const char* str)
{
  if(*str == 0) return true;
  while(str) {
    if(!isgraph(*str)) return true;
    if(*str == '\"') return true;
    str++;
  }
  return false;
}

bool StringRequiresQuoting(const std::string& str)
{
  return StringRequiresQuoting(str.c_str());
}

bool SafeInputString(std::istream& in, char* str,int n)
{
  EatWhitespace(in);
  if(!in) return false;
  if(in.peek() == EOF) return false;
  if(in.peek() == '\"') return InputQuotedString(in,str,n);
  else { //read until new whitespace
    int i;
    for(i=0;i<n;i++) {
      str[i] = in.get();
      if(isspace(str[i]) || in.eof()) {
	str[i] = 0;
	return true;
      }
      if(!in) return false;
    }
    //buffer overflowed
    return false; 
  }
}

bool SafeInputString(std::istream& in, std::string& str)
{
  //first eat up white space
  str.erase();
  while(in && isspace(in.peek())) in.get();
  if(!in) return false;
  if(in.peek() == EOF) return false;
  if(in.peek() == '\"') return InputQuotedString(in,str);
  else { //read until new whitespace
    in >> str;
    /*
    while(in) {
      char c = in.get();
      if(isspace(c) || c==EOF) 
	return true;
      str += c;
    }
    */
    if(in.eof()) return true;
    if(!in) return false;
    return true; 
  }
}

void SafeOutputString(std::ostream& out, const char* str)
{
  if(StringRequiresQuoting(str)) OutputQuotedString(out,str);
  else out<<str;
}

void SafeOutputString(std::ostream& out, const std::string& str)
{
  if(StringRequiresQuoting(str)) OutputQuotedString(out,str);
  else out<<str;
}


bool SafeInputFloat(std::istream& in, float& f)
{
  EatWhitespace(in);
  int c=in.peek();
  bool neg=false;
  if(c=='-') {
    in.get();
    c=in.peek();
    neg=true;
  }
  if(isdigit(c) || c=='.') { in>>f; }
  else if(tolower(c) == 'n' || tolower(c) == 'i') {
    std::string str;
    in>>str;
    Lowercase(str);
    if(str == "inf" || str == "infinity")
      f = Math::fInf;
    else if(str=="nan")
      f = std::numeric_limits<float>::quiet_NaN();
    else return false;
  }
  else return false;
  if(neg) f = -f;
  return (bool)in;
}

bool SafeInputFloat(std::istream& in, double& f)
{
  EatWhitespace(in);
  int c=in.peek();
  bool neg=false;
  if(c=='-') {
    in.get();
    c=in.peek();
    neg=true;
  }
#if _WIN32
  if(isdigit(c) || c=='.') { in>>f; }
#else
  if(std::isdigit(c) || c=='.') { in>>f; }
#endif
  else if(tolower(c) == 'n' || tolower(c) == 'i') {
    std::string str;
    in>>str;
    Lowercase(str);
    if(str == "inf" || str == "infinity")
      f = Math::dInf;
    else if(str=="nan")
      f = std::numeric_limits<double>::quiet_NaN();
    else return false;
  }
  else return false;
  if(neg) f = -f;
  return (bool)in;
}

bool SafeOutputFloat(std::ostream& out, float f)
{
  out<<f;
  return true;
}

bool SafeOutputFloat(std::ostream& out, double f)
{
  out<<f;
  return true;
}


bool GetFileContents(const char *filename,std::string& contents)
{
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (in)
  {
    in.seekg(0, std::ios::end);
    contents.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&contents[0], contents.size());
    in.close();
    return true;
  }
  return false;
}

#if HAVE_CURL

size_t write_file(void *ptr, size_t size, size_t nmemb, FILE *stream) {
    size_t written = fwrite(ptr, size, nmemb, stream);
    return written;
}

size_t write_data(void *ptr, size_t size, size_t nmemb, File *stream) {
  stream->WriteData(ptr,size);
  return size;
}

#include <curl/curl.h>
/* For older cURL versions you will also need 
#include <curl/types.h>
#include <curl/easy.h>
*/

bool GetURLDownload(const char* url,const char* filename)
{
    CURL *curl;
    FILE *fp;
    CURLcode res;
    fp = fopen(filename,"wb");
    if(!fp) {
      LOG4CXX_WARN(KrisLibrary::logger(),"GetURLDownload: could not open file "<<filename<<" for writing");
      return false;
    }
    curl = curl_easy_init();
    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_file);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
        curl_easy_setopt(curl, CURLOPT_FAILONERROR, true);
        res = curl_easy_perform(curl);
        if(res != CURLE_OK) 
          LOG4CXX_WARN(KrisLibrary::logger(),"GetURLContents: libcurl error "<<curl_easy_strerror(res));
        /* always cleanup */
        curl_easy_cleanup(curl);
        fclose(fp);
        if(res != CURLE_OK) {
          return false;
        }
        return true;
    }
    LOG4CXX_WARN(KrisLibrary::logger(),"GetURLContents: libcurl could not be initialized");
    fclose(fp);
    return false;
}

bool GetURLContents(const char* url,std::string& contents)
{

    CURL *curl;
    CURLcode res;
    curl = curl_easy_init();
    if (curl) {
        File f;
        f.OpenData();
        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &f);
        curl_easy_setopt(curl, CURLOPT_FAILONERROR, true);
        res = curl_easy_perform(curl);
        if(res != CURLE_OK) 
          LOG4CXX_WARN(KrisLibrary::logger(),"GetURLContents: libcurl error "<<curl_easy_strerror(res));
        /* always cleanup */
        curl_easy_cleanup(curl);
        if(res != CURLE_OK) 
          return false;
        contents.resize(f.Length()+1);
        contents.copy((char*)f.GetDataBuffer(),f.Length());
        contents[f.Length()] = 0;
        return true;
    }
    LOG4CXX_WARN(KrisLibrary::logger(),"GetURLContents: libcurl could not be initialized");
    return false;
}

#else

bool GetURLDownload(const char* url,const char* filename)
{
  LOG4CXX_WARN(KrisLibrary::logger(),"libcurl is not available on your system, cant use GetURLDownload");
  return false;
}

bool GetURLContents(const char* url,std::string& contents)
{
  LOG4CXX_WARN(KrisLibrary::logger(),"libcurl is not available on your system, cant use GetURLContents");
  return false;  
}

#endif //HAVE_CURL

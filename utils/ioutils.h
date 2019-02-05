#ifndef IO_UTILITIES_H
#define IO_UTILITIES_H

#include <iostream>
#include <string>
#include <vector>
#include <KrisLibrary/File.h>

/** @file utils/ioutils.h
 * @ingroup Utils
 * @brief Utilities for I/O.
 */

/** @addtogroup Utils */
/*@{*/

///Eats up whitespace at beginning of stream
void EatWhitespace(std::istream& in);

///Inputs a "token" consisting of only the characters in characterSet.
///Does not ignore whitespace!
bool InputToken(std::istream& in,const char* characterSet,std::string&);

///Gets a quoted string from the istream into a char buffer (or string)
bool InputQuotedString(std::istream& in, char* str, int n);
bool InputQuotedString(std::istream& in, std::string&);
///Outputs "str".  Outputs \" for quote characters in str.
void OutputQuotedString(std::ostream& out, const char* str);
void OutputQuotedString(std::ostream& out, const std::string&);
///Returns true if str has a quote character
bool StringContainsQuote(char* str);
bool StringContainsQuote(const std::string& str);
///Returns true if outputting string requires quotations
bool StringRequiresQuoting(char* str);
bool StringRequiresQuoting(const std::string& str);

///Inputs the string with quotes if necessary
bool SafeInputString(std::istream& in, char* str,int n);
bool SafeInputString(std::istream& in, std::string&);
///Outputs the string with quotes if necessary
void SafeOutputString(std::ostream& out, const char* str);
void SafeOutputString(std::ostream& out, const std::string&);

/// I/O with denormalized floats (infinity, NaN)
bool SafeInputFloat(std::istream& in, float& f);
bool SafeInputFloat(std::istream& in, double& f);
bool SafeOutputFloat(std::ostream& out, float f);
bool SafeOutputFloat(std::ostream& out, double f);

///If c is preceded by a \, returns the translated ascii character.
///e.g. n->\n, t->\t, etc.
int TranslateEscape(int c);
//translates escape sequences (e.g. \n, \t, etc) over str
std::string TranslateEscapes(const std::string& str);

///Returns the entire contents of a file as a string
bool GetFileContents(const char *filename,std::string& contents);

///Returns the contents of a URL as a string.
///Requires libcurl to be available when CMake is run.
bool GetURLContents(const char* url,std::string& contents);

///Downloads the contents of a URL to a file
///Requires libcurl to be available when CMake is run.
bool GetURLDownload(const char* url,const char* filename);

///ReadFile() for STL vectors.  See File.h
template <class type>
bool ReadFile(File& f, std::vector<type>& v)
{
	size_t size;
	if(!ReadFile(f,size)) return false;
	v.resize(size);
	for(size_t i=0;i<size;i++)
		if(!ReadFile(f,v[i])) return false;
	return true;
}

///WriteFile() for STL vectors.  See File.h
template <class type>
bool WriteFile(File& f, const std::vector<type>& v)
{
	size_t size=v.size();
	if(!WriteFile(f,size)) return false;
	for(size_t i=0;i<size;i++)
		if(!WriteFile(f,v[i])) return false;
	return true;
}

///Inputs a vector from an iostream using the format 
///size  v[0] v[1] ... v[size-1]
template <class type>
bool InputVector(std::istream& in, std::vector<type>& v)
{
	size_t size;
	in>>size;
	if(in.bad()) return false;
	v.resize(size);
	for(size_t i=0;i<size;i++) {
	  in>>v[i];
	  if(in.bad()) return false;
	}
	return true;
}

///Outputs a vector to an iostream using the format 
///size  v[0] v[1] ... v[size-1]
template <class type>
bool OutputVector(std::ostream& out, const std::vector<type>& v)
{
  out<<v.size()<<'\t';
  for(size_t i=0;i<v.size();i++) {
    out<<v[i]<<" ";
  }
  return true;
}

/*@}*/

#endif

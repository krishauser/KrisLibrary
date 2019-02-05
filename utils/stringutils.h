#ifndef UTILS_STRING_H
#define UTILS_STRING_H

#include <string>
#include <vector>

/** @file utils/stringutils.h
 * @ingroup Utils
 * @brief Utilities for string manipulation.
 */

/** @addtogroup Utils */
/*@{*/

///Returns a "close bracket" character opposite c
char CloseBracket(char c);

///Turns the string into lower/uppercase
void Lowercase(char* str);
void Uppercase(char* str);
void Lowercase(std::string& str);
void Uppercase(std::string& str);

//strips whitespace from str
std::string Strip(const std::string& str);
std::string LStrip(const std::string& str);
std::string RStrip(const std::string& str);

//divides a string into multiple strings given a string containing deliminators
std::vector<std::string> Split(const std::string& str, const std::string& delim);

///Replace all instances of strfind with strreplace in str
int ReplaceAll(std::string& str,const char* strfind,const char* strreplace);

///Returns true if the beginning of the given string matches prefix
bool StartsWith(const char* str,const char* prefix);
///Returns true if the end of the given string matches prefix
bool EndsWith(const char* str,const char* suffix);

bool IsValidCToken(const char* str);
bool IsValidInteger(const char* str);
bool IsValidFloat(const char* str);

///Detects a pattern in str = [prefix][digits][suffix].
///Returns the number specified by [digits], or -1 if no such pattern is found.
int DetectNumericalPattern(const char* str,char prefix[],char suffix[],int& numDigits);

//For a string in the pattern XXXX####XXXX, increments the #### part.
//The number of digits can be 1-4.  For a string with no digits,
//leaves the string unchanged.
void IncrementStringDigits(char* str);
void IncrementStringDigits(std::string& str);

void ToBase64(const std::string& in,std::string& out);
void ToBase64(const char* in,int length,std::string& out);
void FromBase64(const std::string& in,std::string& out);
void FromBase64(const char* in,std::string& out);

std::string ToBase64(const std::string& in);
std::string ToBase64(const char* in,int length);
std::string FromBase64(const std::string& in);
std::string FromBase64(const char* in);

///Dos-unix endline conversion
int LengthWithDOSEndlines(const char* str);
bool EndlinesToDOS(const char* str,char* out,int max);
bool EndlinesFromDOS(const char* str,char* out,int max);
void EndlinesToDOS(std::string& str);
void EndlinesFromDOS(std::string& str);

///Returns pointer to "ext" for str="filename.ext"
const char* FileExtension (const char* str);
///Replaces the file extension of str with ext, or concatenates .ext onto str
void ChangeFileExtension (char* str, const char* ext);
///Returns "file.ext" for the str="dir1/dir2/.../file.ext"
const char* GetFileName(const char* str);
///Extracts the path from str (formatted as above) into buf, not including the trailing '/'
void GetFilePath(const char* str, char* buf);
///Removes the file extension of str
void StripExtension(char* str);

///Returns "ext" for str="filename.ext"
std::string FileExtension (const std::string& str);
///Replaces the file extension of str with ext, or concatenates .ext onto str
void ChangeFileExtension (std::string& str, const char* ext);
///Returns "file.ext" for the str="dir1/dir2/.../file.ext"
std::string GetFileName(const std::string& str);
///Extracts the path from str (formatted as above), not including the trailing '/'
std::string GetFilePath(const std::string& str);
///Removes the file extension of str
void StripExtension(std::string& str);

///For a path dir1/dir2/.../file, splits the path into dir1, dir2, ..., file
///in a cross platform manner
void SplitPath(const std::string& path,std::vector<std::string>& elements);
///For a path that might contain . and .. references this will take out
///the "." and non-leading ".." elements.
///For example, if the input is "dir/./subdir/../" this will return "dir".
std::string ReducePath(const std::string& path);
///For elements dir1, dir2, ..., file, returns dir1/dir2/.../file
///in a cross-platform manner.  The elements may also have leading/trailing
///path delimiters which are ignored
std::string JoinPath(const std::vector<std::string>& path,char delim=0);
std::string JoinPath(const std::string& path1,const std::string& path2);

#ifdef WIN32
#ifndef WCHAR
typedef wchar_t WCHAR;    // wc,   16-bit UNICODE character
#endif //WCHAR
void ToWideChar(const char* str, WCHAR* buf, int maxBuf);
#endif //WIN32

/*@}*/

#endif

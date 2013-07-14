#include "fileutils.h"
#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#endif
#include <errors.h>

namespace FileUtils {


std::string SafeFileName(const std::string& str)
{
  std::string temp;
  for(size_t i=0;i<str.length();i++) {
    char c=str[i];
    if(isalnum(c) || c=='.' || c=='-' || c=='_') temp+=c;
    else temp+='_';
  }
  return temp;
}

void SafeFileName(char* str)
{
  while(*str) {
    char c=*str;
    if(isalnum(c) || c=='.' || c=='-' || c=='_') {}
    else *str='_';
    str++;
  }
}

bool Exists(const char* fn)
{
#ifdef WIN32
	HANDLE f = CreateFile((LPCTSTR)fn,GENERIC_READ,FILE_SHARE_READ,NULL,OPEN_ALWAYS,0,0);
	if(f == INVALID_HANDLE_VALUE) return false;
	CloseHandle(f);
	return true;
#else
	FILE* f = fopen(fn,"r");
	if(!f) return false;
	fclose(f);
	return true;
#endif
}

bool Delete(const char* fn)
{
#ifdef WIN32
	return DeleteFile((LPCTSTR)fn) != FALSE;
#else
	return (unlink(fn)==0);
#endif
}

bool Rename(const char* from,const char* to)
{
#ifdef WIN32
	return MoveFile(from,to) != FALSE;
#else
	return (rename(from,to)==0);
	/*
	size_t len = strlen(from) + strlen(to) + 5;
	char* buf = new char[len];
	sprintf(buf,"mv %s %s",from,to);
	int res = system(buf);
	delete [] buf;
	return res == 0;
	*/
#endif
}

bool Copy(const char* from,const char* to,bool override)
{
#ifdef WIN32
	return CopyFile(from,to,(override?FALSE:TRUE)) != FALSE;
#else
	size_t len = strlen(from) + strlen(to) + 5;
	char* buf = new char[len];
	sprintf(buf,"cp %s %s %s",(override?"-f":""),from,to);
	int res = system(buf);
	delete [] buf;
	return res == 0;
#endif
}

bool TempName(char* out,const char* directory,const char* prefix)
{
#ifdef WIN32
	if(directory == NULL) directory = ".";
	UINT res = GetTempFileName(directory,prefix,0,out);
	return (res!=0);
#else
	char* fn=tempnam(directory,prefix);
	if(!fn) return false;
	strcpy(out,fn);
	return true;
#endif
}

bool IsDirectory(const char* path)
{
#if WIN32
	bool res = ((GetFileAttributes(path) & FILE_ATTRIBUTE_DIRECTORY) != 0);
	return res;
#else
  struct stat buf;
  stat(path,&buf);
  if(S_ISDIR(buf.st_mode)) return true;
  return false;
#endif
}

bool CreateDirectory(const char* path)
{
#if WIN32
	HANDLE f = CreateFile(path,FILE_LIST_DIRECTORY,FILE_SHARE_READ,NULL,CREATE_NEW,FILE_ATTRIBUTE_DIRECTORY,NULL);
	if(f == INVALID_HANDLE_VALUE) return false;
	CloseHandle(f);
	return true;
#else
  return mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)==0;
#endif
}

bool ListDirectory(const char* path,std::vector<std::string>& files)
{
#if WIN32
  FatalError("TODO: ListDirectory on windows");
  return false;
#else
  struct dirent *de=NULL;
  DIR *d=NULL;

  d=opendir(path);
  if(d == NULL)
  {
    return false;
  }

  // Loop while not NULL
  files.resize(0);
  while((de = readdir(d))) 
    files.push_back(de->d_name);

  closedir(d);
  return true;
#endif
}

} // namespace FileUtils

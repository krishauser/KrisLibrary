#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "fileutils.h"
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <strsafe.h>
#include <shlobj.h>    // for SHCreateDirectoryEx
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#define GetCurrentDir getcwd
#endif //_WIN32
#ifdef __APPLE__
#include <unistd.h>
#endif //__APPLE__
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
#ifdef _WIN32
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
#ifdef _WIN32
	return DeleteFile((LPCTSTR)fn) != FALSE;
#else
	return (unlink(fn)==0);
#endif
}

bool Rename(const char* from,const char* to)
{
#ifdef _WIN32
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
#ifdef _WIN32
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
#ifdef _WIN32
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
#ifdef _WIN32
	bool res = ((GetFileAttributes(path) & FILE_ATTRIBUTE_DIRECTORY) != 0);
	return res;
#else
  struct stat buf;
  stat(path,&buf);
  if(S_ISDIR(buf.st_mode)) return true;
  return false;
#endif
}

bool MakeDirectory(const char* path)
{
#ifdef _WIN32
	HANDLE f = CreateFile(path,FILE_LIST_DIRECTORY,FILE_SHARE_READ,NULL,CREATE_NEW,FILE_ATTRIBUTE_DIRECTORY,NULL);
	if(f == INVALID_HANDLE_VALUE) return false;
	CloseHandle(f);
	return true;
#else
  return mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)==0;
#endif
}


bool MakeDirectoryRecursive(const char* path)
{
#ifdef _WIN32
  return SHCreateDirectoryEx( NULL, path, NULL ) == ERROR_SUCCESS;
#else
        size_t len = strlen(path);
        char* tmp = new char[len+1];
        char *p = NULL;
 
        strcpy(tmp, path);
        if(tmp[len - 1] == '/')
                tmp[len - 1] = 0;
	p = tmp;
	if(*p == '/') p++;
        for(; *p; p++)
                if(*p == '/') {
                        *p = 0;
                        mkdir(tmp, S_IRWXU);
                        *p = '/';
                }
        bool res=( mkdir(tmp, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0);
	delete [] tmp;
	return res;
#endif
}

bool ListDirectory(const char* path,std::vector<std::string>& files)
{
#ifdef _WIN32
  files.resize(0);
  WIN32_FIND_DATA ffd;
  TCHAR szDir[MAX_PATH];
  size_t length_of_arg;
  HANDLE hFind = INVALID_HANDLE_VALUE;
  DWORD dwError=0;

  StringCchLength(path, MAX_PATH, &length_of_arg);

  if (length_of_arg > (MAX_PATH - 3)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Directory path "<<path);
    return false;
  }

   // Prepare string for use with FindFile functions.  First, copy the
   // string to a buffer, then append '\*' to the directory name.

   StringCchCopy(szDir, MAX_PATH, path);
   StringCchCat(szDir, MAX_PATH, TEXT("\\*"));

   // Find the first file in the directory.

   hFind = FindFirstFile(szDir, &ffd);

   if (INVALID_HANDLE_VALUE == hFind) 
   {
     //either no items or not a directory -- should we return true if it's
     //an empty directory?
     return false;
   } 
   
   // List all the files in the directory with some info about them.

   do
   {
     files.push_back(ffd.cFileName);
   }
   while (FindNextFile(hFind, &ffd) != 0);
 
   dwError = GetLastError();
   if (dwError != ERROR_NO_MORE_FILES) 
   {
          LOG4CXX_ERROR(KrisLibrary::logger(),"Error while reading files from "<<path);
     return false;
   }

   FindClose(hFind);
   return true;
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

static char cCurrentPath[FILENAME_MAX];

std::string GetWorkingDirectory()
{
  if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
  {
    return "";
  }
  cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */
  return cCurrentPath;
}

} // namespace FileUtils

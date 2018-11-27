#include <KrisLibrary/Logger.h>
#include "apputils.h"
#include "fileutils.h"
#include <string.h>
#include <fstream>
#ifdef _WIN32
#include <shlobj.h>    // for SHGetFolderPath
#endif
using namespace std;

namespace AppUtils {

#ifdef _WIN32
std::string GetApplicationDataPath(const char* applicationName,const char* version)
{
  string versionSuffix;
  if(version && strlen(version) > 0)
    versionSuffix = string("\\")+string(version);
  TCHAR szPath[MAX_PATH];
  if(SUCCEEDED(SHGetFolderPath(NULL,CSIDL_APPDATA,NULL,0,szPath))) {
    return string(szPath)+string("\\")+string(applicationName)+versionSuffix;
  }
    LOG4CXX_ERROR(KrisLibrary::logger(),"GetApplicationDataPath: SHGetFolderPath failed? returning local folder\n");
  return string(applicationName)+versionSuffix;
}
#else 
//assuming unix variant
#include <unistd.h>
std::string GetApplicationDataPath(const char* applicationName,const char* version)
{
  string versionSuffix;
  if(version && strlen(version) > 0)
    versionSuffix = string("\\")+string(version);
  const char* homedir = getenv("HOME");
  if(homedir == NULL)
    return string(applicationName)+versionSuffix;
  return string(homedir)+"/."+string(applicationName)+versionSuffix;
}

#endif

ProgramSettings::ProgramSettings(const char* _applicationName,const char* _version)
    :applicationName(_applicationName),version(_version)
{ 
}

bool ProgramSettings::read(const char* fn)
{
  string appDataPath = GetApplicationDataPath(applicationName.c_str(),version.c_str());
  ifstream in(((appDataPath+"/")+fn).c_str(),ios::in);
  if(!in) return false;
  AnyCollection newEntries;
  if(!newEntries.read(in)) return false;
  merge(newEntries);
  return true;
}

bool ProgramSettings::write(const char* fn)
{
  string appDataPath = GetApplicationDataPath(applicationName.c_str(),version.c_str());
  if(!FileUtils::MakeDirectoryRecursive(appDataPath.c_str())) return false;
  ofstream out(((appDataPath+"/")+fn).c_str(),ios::out);
  if(!out) return false;
  AnyCollection::write(out);
  out.close();
  return true;
}

} //namespace AppUtils

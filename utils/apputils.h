#ifndef UTILS_APPUTILS_H
#define UTILS_APPUTILS_H

#include "AnyCollection.h"

namespace AppUtils {

///Returns a cross-platform location for saving application data
std::string GetApplicationDataPath(const char* applicationName,const char* version=NULL);

///A convenience class for reading/writing settings to the application data
///folder
class ProgramSettings : public AnyCollection
{
public:
  ProgramSettings(const char* applicationName,const char* version="");
  bool read(const char* fn);
  bool write(const char* fn);
  std::string applicationName;
  std::string version;
};

} //namespace AppUtils

#endif

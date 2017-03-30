#ifndef UTILS_FILEUTILS_H
#define UTILS_FILEUTILS_H

#include <stdlib.h>
#include <string>
#include <vector>

/** @file utils/fileutils.h
 * @ingroup Utils
 * @brief Cross-platform wrapper for file utilities.
 *
 * File names are given as unix-formatted strings (directories separated by
 * a forward slash '/').
 */

/** @addtogroup Utils */
/*@{*/

namespace FileUtils {

void SafeFileName(char* str);
std::string SafeFileName(const std::string& str);

/// Returns true if the file fn exists.
bool Exists(const char* fn);

/// Deletes the file. Returns true if successful.
bool Delete(const char* fn);

/// Renames the file. Returns true if successful.
bool Rename(const char* from,const char* to);

/// Copies the file. Returns true if successful.
bool Copy(const char* from,const char* to);

/// Returns the name of a temporary file in out. Optionally takes a
/// directory and prefix for the file name.  Returns true if successful.
bool TempName(char* out,const char* directory=NULL,const char* prefix=NULL);

/// Returns true if the file is a directory.
bool IsDirectory(const char* path);

/// Creates the directory, if it doesn't exist.  Returns true if successful.
bool MakeDirectory(const char* path);

/// Creates all subdirectories leading up to the directory, if they don't
///exist.  Returns true if successful.
bool MakeDirectoryRecursive(const char* path);

/// Returns a list of filenames in the given directory
bool ListDirectory(const char* path,std::vector<std::string>& items);

/// Returns the current working directory
std::string GetWorkingDirectory();

} //namespace FileUtils

/*@}*/

#endif

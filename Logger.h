#ifndef KRISLIBRARY_LOGGER_H
#define KRISLIBRARY_LOGGER_H

/** @file Logger.h
 * @brief The logging system used in KrisLibrary.
 * 
 * Log4cxx will be used if present.  Otherwise, LOG4CXX macros will be defined just to print to stdout
 * (INFO and WARN) or stderr
 */


#if HAVE_LOG4CXX

#include <log4cxx/logger.h>

namespace KrisLibrary {

typedef log4cxx::LoggerPtr LoggerType;

/** @brief Retrieves the base logger. 
 * 
 * On first call, will attempt to load a configuration from log4cxx.xml in the current directory.
 * If this doesn't exist, then a default configuration printing to stderr will be used.
 *
 * To log, call LOG4CXX_INFO(KrisLibrary::logger(),msg...), or LOG4CXX_WARN, LOG4CXX_ERROR, etc.
 */
extern LoggerType logger();

/** @brief Retrieves a named logger. 
 * 
 * On first call, will attempt to load a configuration from log4cxx.xml in the current directory.
 * If this doesn't exist, then a default configuration printing to stderr will be used.
 *
 * It can be used by calling LOG4CXX_INFO(KrisLibrary::logger(name),msg...), or LOG4CXX_WARN, LOG4CXX_ERROR, etc.
 * However, this is not the most efficient method, since logger lookup is performed for each message.  Instead
 * you should save the LoggerPtr and use it multiple times.  The DEFINE_LOGGER and GET_LOGGER macros are
 * better for this, since they only perform lookup of the logger on the first call.
 */
extern LoggerType logger(const char* name);

/** @brief If the root logger is enabled for debug level, this will cause a getchar() to be called.
 */
extern void loggerWait();

/** @brief If logger is enabled for debug level, this will cause a getchar() to be called.
 */
extern void loggerWait(LoggerType logger);

///Use this inside a cpp file to define a fast logger 
#define DEFINE_LOGGER(name) \
  DECLARE_LOGGER(name) \
  namespace KrisLibrary { \
    LoggerType _logger_##name; \
  }

///Use to declare that you will use a fast logger (only needed if you will share a fast logger between cpp files)
#define DECLARE_LOGGER(name) \
  namespace KrisLibrary { \
    extern LoggerType _logger_##name; \
    inline LoggerType _get_logger_##name() { \
        if (_logger_##name == NULL)  \
            _logger_##name = logger(#name); \
        return _logger_##name; \
      } \
  }

///Use this to retrieve a fast logger
#define GET_LOGGER(name) KrisLibrary::_get_logger_##name()

} //namespace KrisLibrary

#else

#include <iostream>
#include <stdlib.h>

namespace KrisLibrary {

typedef const char* LoggerType;

/** @brief Retrieves the base logger. 
 *
 * To log, call LOG4CXX_INFO(KrisLibrary::logger(),msg...), or LOG4CXX_WARN, LOG4CXX_ERROR, etc.
 */
inline LoggerType logger() { return NULL; }

/** @brief Retrieves a named logger. 
 *
 * It can be used by calling LOG4CXX_INFO(KrisLibrary::logger(name),msg...), or LOG4CXX_WARN, LOG4CXX_ERROR, etc.
 * However, this is not the most efficient method, since logger lookup is performed for each message.  Instead
 * you should save the LoggerPtr and use it multiple times.  The DEFINE_LOGGER and GET_LOGGER macros are
 * better for this, since they only perform lookup of the logger on the first call.
 */
inline LoggerType logger(const char* name) { return name; }

/** @brief If the root logger is enabled for debug level, this will cause a getchar() to be called.
 */
inline void loggerWait() { printf("Press enter to continue...\n"); getchar(); }

/** @brief If logger is enabled for debug level, this will cause a getchar() to be called.
 */
inline void loggerWait(LoggerType logger) { printf("Press enter to continue...\n"); getchar(); }

#define LOG4CXX_DEBUG(logger,data) { \
  if(logger) std::cout<<logger<<": "<<data<<std::endl; \
  else std::cout<<data<<std::endl; }

#define LOG4CXX_INFO(logger,data) { \
  if(logger) std::cout<<logger<<": "<<data<<std::endl; \
  else std::cout<<data<<std::endl; }

#define LOG4CXX_WARN(logger,data) { \
  if(logger) std::cout<<logger<<": "<<data<<std::endl; \
  else std::cout<<data<<std::endl; }

#define LOG4CXX_ERROR(logger,data) { \
  if(logger) std::cerr<<logger<<": "<<data<<std::endl; \
  else std::cerr<<data<<std::endl; }

#define LOG4CXX_FATAL(logger,data) { \
  if(logger) std::cerr<<logger<<": "<<data<<std::endl; \
  else std::cerr<<data<<std::endl; }

///Use this inside a cpp file to define a fast logger 
#define DEFINE_LOGGER(name) \
  DECLARE_LOGGER(name) \
  namespace KrisLibrary { \
    LoggerType _logger_##name; \
  }

///Use to declare that you will use a fast logger (only needed if you will share a fast logger between cpp files)
#define DECLARE_LOGGER(name) \
  namespace KrisLibrary { \
    extern LoggerType _logger_##name; \
    inline LoggerType _get_logger_##name() { \
        if (_logger_##name == NULL)  \
            _logger_##name = logger(#name); \
        return _logger_##name; \
      } \
  }

///Use this to retrieve a fast logger
#define GET_LOGGER(name) KrisLibrary::_get_logger_##name()

} //namespace KrisLibrary


#endif // HAVE_LOG4CXX
  

#endif // KRISLIBRARY_LOGGER_H
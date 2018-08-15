#ifndef KRISLIBRARY_LOGGER_H
#define KRISLIBRARY_LOGGER_H

#include <log4cxx/logger.h>

/** @file Logger.h
 * @brief The logging system used in KrisLibrary.
 */


namespace KrisLibrary {

/** @brief Retrieves the base logger. 
 *
 * To log, call LOG4CXX_INFO(KrisLibrary::logger(),msg...), or LOG4CXX_WARN, LOG4CXX_ERROR, etc.
 */
extern log4cxx::LoggerPtr logger();

/** @brief Retrieves a named logger. 
 *
 * It can be used by calling LOG4CXX_INFO(KrisLibrary::logger(name),msg...), or LOG4CXX_WARN, LOG4CXX_ERROR, etc.
 * However, this is not the most efficient method, since logger lookup is performed for each message.  Instead
 * you should save the LoggerPtr and use it multiple times.  The DEFINE_LOGGER and GET_LOGGER macros are
 * better for this, since they only perform lookup of the logger on the first call.
 */
extern log4cxx::LoggerPtr logger(const char* name);

/** @brief If the root logger is enabled for debug level, this will cause a getchar() to be called.
 */
extern void loggerWait();

/** @brief If logger is enabled for debug level, this will cause a getchar() to be called.
 */
extern void loggerWait(log4cxx::LoggerPtr logger);

///Use this inside a cpp file to define a fast logger 
#define DEFINE_LOGGER(name) \
  DECLARE_LOGGER(name) \
  namespace KrisLibrary { \
    log4cxx::LoggerPtr _logger_##name; \
  }

///Use to declare that you will use a fast logger (only needed if you will share a fast logger between cpp files)
#define DECLARE_LOGGER(name) \
  namespace KrisLibrary { \
    extern log4cxx::LoggerPtr _logger_##name; \
    inline log4cxx::LoggerPtr _get_logger_##name() { \
        if (_logger_##name == NULL)  \
            _logger_##name = logger(#name); \
        return _logger_##name; \
      } \
  }

///Use this to retrieve a fast logger
#define GET_LOGGER(name) KrisLibrary::_get_logger_##name()

} //namespace KrisLibrary

  

#endif // KRISLIBRARY_LOGGER_H
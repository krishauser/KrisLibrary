
#ifndef __LOGGER_H_INCLUDED__
#define __LOGGER_H_INCLUDED__

#include <log4cxx/logger.h>

namespace KrisLibrary{
	
	extern log4cxx::LoggerPtr logger();
	extern log4cxx::LoggerPtr logger(const char*);
	extern void loggerWait();
}

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

  

#endif // __LOGGER_H_INCLUDED__
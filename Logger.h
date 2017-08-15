
#ifndef __LOGGER_H_INCLUDED__
#define __LOGGER_H_INCLUDED__

#include <log4cxx/logger.h>

namespace KrisLibrary{
	
	extern log4cxx::LoggerPtr logger();
	extern log4cxx::LoggerPtr logger(const char*);
	extern void loggerWait();
	//getLogger();
}

#endif // __LOGGER_H_INCLUDED__
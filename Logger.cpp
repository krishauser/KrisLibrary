#if HAVE_LOG4CXX

#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/xml/domconfigurator.h>
#include <log4cxx/level.h>
#include <log4cxx/helpers/objectptr.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/consoleappender.h>
#include "Logger.h"
#include <stdio.h>
#include <sstream>

namespace KrisLibrary {

//static log4cxx::LoggerPtr logger(log4cxx::Logger::getLogger("KrisLibrary"));
bool rootLoggerDefined = false;
log4cxx::LoggerPtr rootLogger = 0;
log4cxx::LoggerPtr myLogger = 0;

log4cxx::LoggerPtr logger()
{
	if (rootLoggerDefined){
		return rootLogger;
	}

	rootLogger = log4cxx::Logger::getRootLogger();
	//non-additive logging by default
	rootLogger->setAdditivity(false);
	try{
		log4cxx::xml::DOMConfigurator::configure("./log4cxx.xml");
		if (rootLogger->getAllAppenders().empty()){
			log4cxx::LayoutPtr defaultLayout   = new log4cxx::PatternLayout("%m %n");
       		log4cxx::AppenderPtr defaultAppender = new log4cxx::ConsoleAppender(defaultLayout);
       		rootLogger->addAppender(defaultAppender);
       		rootLogger->setLevel(log4cxx::Level::getInfo());
			printf("KrisLibrary::logger(): configured as default\n");
			//log4cxx::BasicConfigurator::configure();
		}

	}
	catch(...){
		log4cxx::LayoutPtr defaultLayout   = new log4cxx::PatternLayout("%m %n");
       	log4cxx::AppenderPtr defaultAppender = new log4cxx::ConsoleAppender(defaultLayout);
       	rootLogger->addAppender(defaultAppender);
       	rootLogger->setLevel(log4cxx::Level::getInfo());
		printf("KrisLibrary::logger(): configured as default\n");
		//log4cxx::BasicConfigurator::configure();
	}
	
	rootLoggerDefined = true;
	return rootLogger;
}

log4cxx::LoggerPtr logger(const char* s)
{
	
	if(rootLoggerDefined){
		//we have read the file or at least set a root logger previously
		try{
			printf("KrisLibrary::logger(): trying to find logger %s\n", s);
			myLogger = log4cxx::Logger::getLogger(s);
			if(myLogger->getAllAppenders().empty()){
				printf("  Logger %s has no appenders. Returning root logger.\n", s);
				return rootLogger;
			}
			return myLogger;
		}
		catch(...){
			//logger i
			printf("  Logger %s is not defined. Returning root logger.\n", s);
			return rootLogger;
		}
	}else{
		// otherwise, logger needs to be configured
		try{
			log4cxx::xml::DOMConfigurator::configure("./log4cxx.xml");
			myLogger = log4cxx::Logger::getLogger(s);

			if (myLogger->getAllAppenders().empty()){
				//sets up basic setup if logger doesn't have appender set up.
				std::stringstream ss;
				ss<<s<<": %m %n";
				log4cxx::LayoutPtr defaultLayout   = new log4cxx::PatternLayout(ss.str().c_str());
			    log4cxx::AppenderPtr defaultAppender = new log4cxx::ConsoleAppender(defaultLayout);
			    myLogger->addAppender(defaultAppender);
			    myLogger->setLevel(log4cxx::Level::getInfo());
				printf("KrisLibrary::logger(): configured as default\n");
				//log4cxx::BasicConfigurator::configure();
			}


		}
		catch(...){
			//catches null logger errors
			rootLogger = log4cxx::Logger::getRootLogger();
			printf("KrisLibrary::logger(): Hit error. Setting up default logger\n");
			log4cxx::LayoutPtr defaultLayout   = new log4cxx::PatternLayout("%m %n");
       		log4cxx::AppenderPtr defaultAppender = new log4cxx::ConsoleAppender(defaultLayout);
       		rootLogger->addAppender(defaultAppender);
       		rootLogger->setLevel(log4cxx::Level::getInfo());  // Log level set to INFO
			//log4cxx::BasicConfigurator::configure();
			rootLoggerDefined = true;
			return logger(s);
		}
		return myLogger;
	}
}

void loggerWait()
{
	loggerWait(KrisLibrary::logger());
}

void loggerWait(log4cxx::LoggerPtr logger)
{
	if(logger->isEnabledFor(log4cxx::Level::getDebug())){
		LOG4CXX_ERROR(logger,"   Press enter to continue...");
		getchar();	
	} 
}

} //namespace KrisLibrary

#endif //HAVE_LOG4CXX

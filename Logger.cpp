#include "Logger.h"

#if HAVE_LOG4CXX 

#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/xml/domconfigurator.h>
#include <log4cxx/level.h>
//#include <log4cxx/helpers/objectptr.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/consoleappender.h>
#include <stdio.h>
#include <string.h>
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
			log4cxx::LayoutPtr defaultLayout(new log4cxx::PatternLayout("%m %n"));
       		log4cxx::AppenderPtr defaultAppender(new log4cxx::ConsoleAppender(defaultLayout));
       		rootLogger->addAppender(defaultAppender);
       		rootLogger->setLevel(log4cxx::Level::getInfo());
			printf("KrisLibrary::logger(): configured as default\n");
			//log4cxx::BasicConfigurator::configure();
		}

	}
	catch(...){
		log4cxx::LayoutPtr defaultLayout(new log4cxx::PatternLayout("%m %n"));
       	log4cxx::AppenderPtr defaultAppender(new log4cxx::ConsoleAppender(defaultLayout));
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
			myLogger = log4cxx::Logger::getLogger(s);
			if(myLogger->getAllAppenders().empty()){
				printf("KrisLibrary::logger(): Logger %s has no appenders, using root logger.\n", s);
				return rootLogger;
			}
			return myLogger;
		}
		catch(...){
			//logger i
			printf("KrisLibrary::logger(): Logger %s is not defined, using root logger.\n", s);
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
				log4cxx::LayoutPtr defaultLayout(new log4cxx::PatternLayout(ss.str().c_str()));
			    log4cxx::AppenderPtr defaultAppender(new log4cxx::ConsoleAppender(defaultLayout));
			    myLogger->addAppender(defaultAppender);
			    myLogger->setLevel(log4cxx::Level::getInfo());
				printf("KrisLibrary::logger(): %s configured as default\n",s);
				//log4cxx::BasicConfigurator::configure();
			}


		}
		catch(...){
			//catches null logger errors
			rootLogger = log4cxx::Logger::getRootLogger();
			printf("KrisLibrary::logger(): Hit error. Setting up default logger\n");
			log4cxx::LayoutPtr defaultLayout(new log4cxx::PatternLayout("%m %n"));
       		log4cxx::AppenderPtr defaultAppender(new log4cxx::ConsoleAppender(defaultLayout));
       		rootLogger->addAppender(defaultAppender);
       		rootLogger->setLevel(log4cxx::Level::getInfo());  // Log level set to INFO
			//log4cxx::BasicConfigurator::configure();
			rootLoggerDefined = true;
			return logger(s);
		}
		return myLogger;
	}
}

void setLogLevel(const char* level)
{
	setLogLevel(KrisLibrary::logger(),level);
}

void setLogLevel(LoggerType logger, const char* level)
{
	if(0 == strcmp(level,"DEBUG")) logger->setLevel(log4cxx::Level::getDebug());
	else if(0 == strcmp(level,"INFO")) logger->setLevel(log4cxx::Level::getInfo());
	else if(0 == strcmp(level,"WARN")) logger->setLevel(log4cxx::Level::getWarn());
	else if(0 == strcmp(level,"ERROR")) logger->setLevel(log4cxx::Level::getError());
	else if(0 == strcmp(level,"FATAL")) logger->setLevel(log4cxx::Level::getFatal());
	else logger->setLevel(log4cxx::Level::getInfo());
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

#else //HAVE_LOG4CXX

#include <stdio.h>
#include <string.h>

namespace KrisLibrary
{

	enum LogLevel { LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_FATAL };
	LogLevel currentLogLevel = LOG_INFO;

	LogLevel logLevelToEnum(const char* level) {
		if(0 == strcmp(level,"DEBUG")) return LOG_DEBUG;
		else if(0 == strcmp(level,"INFO")) return LOG_INFO;
		else if(0 == strcmp(level,"WARN")) return LOG_WARN;
		else if(0 == strcmp(level,"ERROR")) return LOG_ERROR;
		else if(0 == strcmp(level,"FATAL")) return LOG_FATAL;
		else return LOG_INFO;
	}

	void setLogLevel(const char* level) {
		currentLogLevel = logLevelToEnum(level);
	}

	void loggerWait() {
		if(currentLogLevel <= LOG_DEBUG) { 
			printf("Press enter to continue...\n");
			getchar();
		}
	}

	void loggerWait(LoggerType logger) { 
		loggerWait();
	}

	bool _shouldLog(LoggerType logger,const char* level) {
		return logLevelToEnum(level) >= currentLogLevel;
	}


} //namespace KrisLibrary

#endif //HAVE_LOG4CXX

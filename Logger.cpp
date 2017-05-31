#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/xml/domconfigurator.h>
#include "Logger.h"
#include <stdio.h>

//static log4cxx::LoggerPtr logger(log4cxx::Logger::getLogger("KrisLibrary"));
bool myLoggerDefined = false;
log4cxx::LoggerPtr rootLogger;
namespace KrisLibrary{
	log4cxx::LoggerPtr logger(){
		if (myLoggerDefined){
			return rootLogger;
		}

		rootLogger = log4cxx::Logger::getRootLogger();
		try{
			log4cxx::xml::DOMConfigurator::configure("./log4cxx.xml");
			if (rootLogger->getAllAppenders().empty()){
				log4cxx::BasicConfigurator::configure();
			}

		}
		catch(...){
			log4cxx::BasicConfigurator::configure();
		}
		
		myLoggerDefined = true;
		return rootLogger;
	}

}


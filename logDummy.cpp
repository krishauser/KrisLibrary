#pragma once
#include <log4cxx/logger.h>
#include "logDummy.h"

//static log4cxx::LoggerPtr logger(log4cxx::Logger::getLogger("KrisLibrary"));
static log4cxx::LoggerPtr logger(log4cxx::Logger::getRootLogger());
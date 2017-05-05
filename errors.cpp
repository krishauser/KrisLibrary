#include <log4cxx/logger.h>
#include <KrisLibrary/logDummy.cpp>
#include "errors.h"
#include <stdlib.h>

#ifdef CYGWIN

int counter=0;

void Abort_Cygwin()
{
  counter++;
}

void Abort()
{
    LOG4CXX_ERROR(logger,"Abort() called, aborting...\n");
    LOG4CXX_ERROR(logger,"To debug, re-run the program under gdb and enter `break Abort_Cygwin' before running\n");
  Abort_Cygwin();
  abort();
}

#endif

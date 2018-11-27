#include <KrisLibrary/Logger.h>
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
    LOG4CXX_FATAL(KrisLibrary::logger(),"Abort() called, aborting...");
    LOG4CXX_FATAL(KrisLibrary::logger(),"To debug, re-run the program under gdb and enter `break Abort_Cygwin' before running");
  Abort_Cygwin();
  abort();
}

#endif

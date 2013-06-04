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
  fprintf(stderr,"Abort() called, aborting...\n");
  fprintf(stderr,"To debug, re-run the program under gdb and enter `break Abort_Cygwin' before running\n");
  Abort_Cygwin();
  abort();
}

#endif

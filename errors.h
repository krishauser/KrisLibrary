#ifndef ERRORS_H
#define ERRORS_H

#include <KrisLibrary/Logger.h>
#include <stdio.h>
#include <stdarg.h>

/* Aborts and asserts don't give a proper stack trace on cygwin. */
#ifdef CYGWIN

#ifdef NDEBUG           /* required by ANSI standard */
  #define Assert(p)  	((void)0)
#else

  #ifdef __STDC__
  #define Assert(e)       ((e) ? (void)0 : __Assert(__FILE__, __LINE__, #e))
  #else   /* PCC */
  #define Assert(e)       ((e) ? (void)0 : __Assert(__FILE__, __LINE__, "e"))
  #endif

#endif // NDEBUG

void Abort();

inline void __Assert(const char * file, int line, const char * e)
{
    LOG4CXX_FATAL(KrisLibrary::logger(),"Assertion \""<<e<<"\" failed: file \""<<file<<"\", line "<<line);
  Abort();
}


#else

  #include "assert.h"
  #include "stdlib.h"
  #define Assert assert
  #define Abort abort
  #define __Assert __assert

#endif //CYGWIN

inline void RaiseErrorFmt(const char* func, const char* file, int line, const char* fmt, ...)
{
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error in "<< func<<" ("<<file<<":"<<line); 
  va_list args;
	va_start(args, fmt);
  char buf[1024];
	vsnprintf(buf, 1024, fmt, args);
  va_end(args);
    LOG4CXX_FATAL(KrisLibrary::logger(),buf);
  Abort();
}

inline void RaiseErrorFmt(const char* fmt,...)
{
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error (unknown function): ");
  va_list args;
	va_start(args, fmt);
	char buf[1024];
  vsnprintf(buf, 1024, fmt, args);
  va_end(args);
    LOG4CXX_FATAL(KrisLibrary::logger(),buf);
  Abort();
}


inline void RaiseError(const char* func, const char* file, int line, const char* text)
{
    LOG4CXX_FATAL(KrisLibrary::logger(),"Error in "<< func<<" ("<<file<<":"<<line<<"): "<<text); 
  Abort();
}



//the following is bending over backwards to support MS's lack of 
//variable argument macro support

#ifdef HAVE_PRETTY_FUNCTION
#define WHERE_AM_I __PRETTY_FUNCTION__, __FILE__, __LINE__
#else
#define WHERE_AM_I __FUNCTION__, __FILE__, __LINE__
#endif

//Error1 is guaranteed to print line numbers with a single-argument error
#define FatalError1(text) RaiseError(WHERE_AM_I,text)

#if HAVE_VARARGS_MACROS
#define FatalError(fmt,...) RaiseErrorFmt(WHERE_AM_I,fmt,__VA_ARGS__)
#else
//if no variable arguments, can't get any line info 
#define FatalError RaiseErrorFmt
#endif

#define PrintLocation(file)  fprintf(file,"%s (%s:%d): ", WHERE_AM_I)
#define AssertNotReached() RaiseError(WHERE_AM_I,"Code should not be reached")

#endif


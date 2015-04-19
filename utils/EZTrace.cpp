#include "EZTrace.h"
#include <errors.h>
using namespace std;

#define MAXBUF 4096

EZTrace::EZTrace()
  :dumpStats(true),dumpTrace(true)
{
  Assert(curTrace == NULL);
  curTrace = &myTrace;
}

EZTrace::~EZTrace()
{
  cout<<"Destroying EZTrace object..."<<endl;
  Assert(curTrace != NULL);
  Assert(curTrace == &myTrace);
  if(dumpTrace) {
    cout<<"********* Program execution trace: **********"<<endl;
    curTrace->DumpTrace(cout);
    cout<<"*********************************************"<<endl;
  }
  if(dumpStats) {
    cout<<"********** Function call stats: *************"<<endl;
    curTrace->DumpStats(cout);
    cout<<"*********************************************"<<endl;
  }
  curTrace = NULL;
}

Trace* EZTrace::curTrace=NULL;

EZCallTrace::EZCallTrace(const char* name)
{
  if(EZTrace::curTrace != NULL) {
    EZTrace::curTrace->Call(name);
    this->name = name;
    this->retval = "void";
  }
}

EZCallTrace::EZCallTrace(const char* name,const char* fmt,...)
{
  if(EZTrace::curTrace != NULL) {
    char buf [MAXBUF];
    va_list args;
    va_start(args, fmt);
#ifdef _WIN32
    _vsnprintf(buf, MAXBUF, fmt, args);
#else
    vsnprintf(buf, MAXBUF, fmt, args);
#endif
    EZTrace::curTrace->Call(name,buf);
    this->name = name;
    this->retval = "void";
  }
}

EZCallTrace::~EZCallTrace()
{
  if(EZTrace::curTrace) 
    EZTrace::curTrace->EndCall(name.c_str(),retval.c_str());
}

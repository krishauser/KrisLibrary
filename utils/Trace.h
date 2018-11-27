#ifndef UTILS_TRACE_H
#define UTILS_TRACE_H

#include <KrisLibrary/Logger.h>
#include <vector>
#include <list>
#include <string>
#include <iostream>
#include <KrisLibrary/Timer.h>
#include "StatCollector.h"
class File;

struct TraceFunctionCall;

struct TraceFunction
{
  std::string name;
  StatCollector stats;
};

struct TraceItem
{
  TraceItem();
  TraceFunctionCall* call;
  std::string text;
};

struct TraceFunctionCall
{
  TraceFunctionCall();
  TraceFunctionCall(TraceFunction* t,TraceFunctionCall* p);
  ~TraceFunctionCall();
  void ClearChildren();

  TraceFunction* type;
  TraceFunctionCall* parent;
  std::string args;
  std::vector<TraceItem> children;
  std::string ret;
  double calltime,endtime;
};

/** @brief Allows detailed tracing of program execution
 *
 * Controlled by the user using a sequence of Call/EndCall/Log commands.
 * Call should be called on entry of a function, and EndCall on exit.
 *
 * Logs 
 * 1) Program execution trace
 * 2) Function call statistics: number of calls, time spent in function
 *
 * @sa EZTrace
 * @sa EZCallTrace
 */
class Trace
{
public:
  Trace();
  ~Trace();
  void Clear();
  void ResetTrace();
  bool Load(const char* fn);
  bool Save(const char* fn);
  void DumpTrace(std::ostream& out=std::cout) const;
  void DumpStats(std::ostream& out=std::cout) const;

  void Call(const char* function,const char* args=NULL);
  void CallFmt(const char* function,const char* fmt,...);
  void EndCall(const char* function,const char* ret=NULL);
  void EndCallFmt(const char* function,const char* fmt,...);
  void Log(const char* txt);

private:
  bool LoadIter(File& f,TraceItem& item,TraceFunctionCall* parent);
  bool SaveIter(File& f,const TraceItem& item);
  void DumpIter(std::ostream& out,const TraceFunctionCall* call,int depth) const;
  TraceFunction* FindFunction(const char* func);
  TraceFunctionCall* FindParentIter(TraceFunctionCall* call,TraceFunction* func);
  void BeginCall(TraceFunctionCall* call);
  void EndCall(TraceFunctionCall* call);

public:
  TraceFunctionCall root;
  TraceFunctionCall* cur;
  std::list<TraceFunction> funcs;
  Timer timer;
};

#endif

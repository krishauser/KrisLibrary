#ifndef UTILS_EZTRACE_H
#define UTILS_EZTRACE_H

#include "Trace.h"
#include <sstream>

/** @ingroup Utils
 * @brief A Trace that dumps on destruction.  Only one may be active at
 * any time.
 *
 * All EZCallTrace functions log to the current trace.
 */
struct EZTrace
{
  EZTrace();
  ~EZTrace();

  bool dumpStats,dumpTrace;
  Trace myTrace;

  static Trace* curTrace;
};

/** @ingroup Utils
 * @brief Helper class that eases function call tracing
 *
 * On entry to a function, instantiate an EZCallTrace object.
 * On exit, optionally pass the return value to Return().
 * If this isn't done, "void" is passed to EZTrace as the 
 * return value.
 * e.g.
 *
 * int foo() {
 *   EZCallTrace tr("foo");
 *   ...
 *   return tr.Return(bar);
 * }
 */
struct EZCallTrace
{
  EZCallTrace(const char* name);
  EZCallTrace(const char* name,const char* args,...);
  ~EZCallTrace();

  template <class T>
  const T& Return(const T& val) {
    std::stringstream ss;
    ss<<val;
    retval = ss.str();
    return val;
  }

  std::string name;
  std::string retval;
};

#endif

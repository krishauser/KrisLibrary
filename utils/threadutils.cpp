#include "threadutils.h"

#ifdef WIN32 
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
void ThreadSleep(double duration) { Sleep(int(duration*1000)); }
#endif
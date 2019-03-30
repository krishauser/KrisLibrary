#include "Timer.h"
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
typedef DWORD TimerCounterType;
#else
#include <sys/time.h>
#ifdef  _POSIX_MONOTONIC_CLOCK
typedef timespec TimerCounterType;
#else
typedef timeval TimerCounterType;
#endif //_POSIX_MONOTONIC_CLOCK
#endif //_WIN32

struct TimerImpl
{
  TimerCounterType start;
  TimerCounterType current;
};


#ifdef _WIN32
#define GETCURRENTTIME(x) x=timeGetTime()
#else
#ifdef  _POSIX_MONOTONIC_CLOCK
#define GETCURRENTTIME(x) clock_gettime(CLOCK_MONOTONIC,&x)
#else
#define GETCURRENTTIME(x) gettimeofday(&x,NULL)
#endif //_POSIX_MONOTONIC_CLOCK
#endif //_WIN32

// Sadly, timersub isn't defined in Solaris. :(
// So we use this instead. (added by Ryan)

#if defined (__SVR4) && defined (__sun)
#include "timersub.h"
#endif

Timer::Timer()
  :impl(new TimerImpl)
{
  Reset();
}

Timer::Timer(const Timer& rhs)
  :impl(new TimerImpl)
{
  *this = rhs;
}

const Timer& Timer::operator =(const Timer& rhs)
{
  *impl = *rhs.impl;
  return *this;
}

Timer::~Timer()
{
  delete impl;
}

void Timer::Reset()
{
  GETCURRENTTIME(impl->start);
  impl->current=impl->start;
}

long long Timer::ElapsedTicks()
{
  GETCURRENTTIME(impl->current);
  return LastElapsedTicks();
}

long long Timer::LastElapsedTicks() const
{
#ifdef _WIN32
  return impl->current-impl->start;
#else
#ifdef  _POSIX_MONOTONIC_CLOCK
  long long ticks = (impl->current.tv_sec-impl->start.tv_sec)*1000 + (impl->current.tv_nsec-impl->start.tv_nsec)/1000000;
  return ticks;
#else
  timeval delta;
  timersub(&impl->current,&impl->start,&delta);
  long long ticks = delta.tv_sec*1000 + delta.tv_usec/1000;
  return ticks;
#endif //_POSIX_MONOTONIC_CLOCK
#endif //_WIN32
}
    
double Timer::ElapsedTime()
{
  GETCURRENTTIME(impl->current);
  return LastElapsedTime();
}

double Timer::LastElapsedTime() const
{
#ifdef _WIN32
  return double(impl->current-impl->start)/1000.0;
#else
#ifdef  _POSIX_MONOTONIC_CLOCK
  double secs=double(impl->current.tv_sec-impl->start.tv_sec);
  secs += double(impl->current.tv_nsec-impl->start.tv_nsec)/1000000000.0;
  return secs;
#else
  timeval delta;
  timersub(&impl->current,&impl->start,&delta);
  double secs=double(delta.tv_sec);
  secs += double(delta.tv_usec)/1000000.0;
  return secs;
#endif //_POSIX_MONOTONIC_CLOCK
#endif //_WIN32
}

/*
clock_t Timer::ElapsedTicks()
{
  current = clock();
  return (current-start);
}

double Timer::ElapsedTime()
{
  current = clock();
  return double(current-start)/CLOCKS_PER_SEC;
}

clock_t Timer::LastElapsedTicks() const
{
  return current-start;
}

double Timer::LastElapsedTime() const
{
  return double(current-start)/CLOCKS_PER_SEC;
}
*/

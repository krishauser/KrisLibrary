#ifndef MY_TIMER_H
#define MY_TIMER_H

struct TimerImpl;

class Timer
{
 public:
  Timer();
  Timer(const Timer& rhs);
  ~Timer();
  const Timer& operator = (const Timer&);
  /// Resets the timer
  void Reset();

  /// Returns elapsed time since constructor or last Reset() call, in milliseconds
  long long ElapsedTicks();
  /// Returns elapsed time since constructor or last Reset() call, in seconds
  double ElapsedTime();

  /// Returns elapsed time cached on prior ElapsedX call, in milliseconds 
  long long LastElapsedTicks() const;
  /// Returns elapsed time cached on prior ElapsedX call, in seconds
  double LastElapsedTime() const;

 private:
  TimerImpl* impl;
};

#endif

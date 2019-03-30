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
  void Reset();

  // Returns elapsed time in milliseconds,seconds respectively
  long long ElapsedTicks();
  double ElapsedTime();

  // Doesn't refresh the current time
  long long LastElapsedTicks() const;
  double LastElapsedTime() const;

 private:
  TimerImpl* impl;
};

#endif

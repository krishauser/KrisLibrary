#ifndef UTILS_SIGNAL_HANDLER_H
#define UTILS_SIGNAL_HANDLER_H

/** @ingroup Utils
 * @brief A base class for an object-oriented signal handler.
 * Properly restores old signal handlers once the class is destroyed.
 *
 * Override the OnRaise() member in your subclass.
 *
 * Set handler for a signal number with SetCurrent, unset it with
 * UnsetCurrent.
 */
class SignalHandler
{
public:
  virtual ~SignalHandler();

  void SetCurrent(int signum);
  bool IsCurrent(int signum) const;
  void UnsetCurrent(int signum);
  static SignalHandler* GetCurrent(int signum);

  virtual void OnRaise(int signum)=0;
};


#endif

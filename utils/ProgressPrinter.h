#ifndef UTILS_PROGRESS_PRINTER_H
#define UTILS_PROGRESS_PRINTER_H

#include <KrisLibrary/Logger.h>
#include <iosfwd>

/** @ingroup Utils
 * @brief Prints the progress of an iterative, long computation.
 *
 * The user calls Update() each iteration from 0...max.
 * Print() is called "increments" times at evenly-spaced
 * intervals.  By default, prints the percentage complete to
 * "out"
 */
class ProgressPrinter
{
public:
  ProgressPrinter(std::ostream& out,int max,int increments=100);
  ProgressPrinter(int max,int increments=100);
  void Update();
  void Update(int iter);
  virtual void Print(float fraction);
  virtual void Done();

  std::ostream& out;
  int max;
  int increments;
  int iter;
};

#endif

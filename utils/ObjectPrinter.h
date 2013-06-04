#ifndef OBJECT_PRINTER_H
#define OBJECT_PRINTER_H

#include <iostream>

/**  @ingroup Utils
 * @brief Provides a unified interface to "pretty print" objects to
 * the console, or other ostreams
 *
 * Just override the template and fill in the right Print function. The
 * default implementation uses T.Print(ostream& out) const.
 */
template <class T>
class ObjectPrinter
{
 public:
  ObjectPrinter(const T& _obj) : obj(_obj),width(-1) {}
  void Print(std::ostream& out) const { obj.Print(out); }
  void PrettyPrint(std::ostream& out) const { 
    if(width < 0) Print(out);
    else {
      //TODO: use the width to make nice output
      Print(out);
    }
  }

  const T& obj;
  int width;
};

template <class T>
inline std::ostream& operator << (std::ostream& out, const ObjectPrinter<T>& printer)
{
  printer.PrettyPrint(out);
  return out;
}

#endif

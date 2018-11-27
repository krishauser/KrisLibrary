#ifndef UTILS_CONTEXT_STACK_H
#define UTILS_CONTEXT_STACK_H

#include <KrisLibrary/Logger.h>
#include <string>
#include <list>

/** @brief A helper for maintaining and printing context.
 * 
 * A context frame is pushed (with a name) when it is entered, and popped
 * when it is exited.  The string cast operator returns the context string,
 * where the name of each frame is delimited with delim (default ':').
 */
class ContextStack
{
 public:
  void Push(const std::string& item) {
    frames.push_back(item);
    str += delim;
    str += item;
  }
  void Pop() {
    frames.erase(--frames.end());
    str.clear();
    for(std::list::const_iterator i=frames.begin();i!=frames.end();i++) {
      if(i!=frames.begin()) str+=delim;
      str += *i;
    }
  }
  const std::string& GetLocal() const { return frames.back(); }
  operator const std::string& () const { return str; }

 private:
  char delim;
  std::list<std::string> frames;
  std::string str;
};

#endif

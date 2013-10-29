#ifndef GL_DISPLAY_LIST_H
#define GL_DISPLAY_LIST_H

#include <utils/SmartPointer.h>

namespace GLDraw {

struct GLDisplayList
{
  GLDisplayList(int count=1);
  ~GLDisplayList();
  operator bool() const { return isCompiled(); }
  bool isCompiled() const;
  void beginCompile(int index=0);
  void endCompile();
  void call(int index=0) const;
  void callAll() const;
  void erase();

  SmartPointer<int> id;
  int count;
};

} //namespace GLDraw

#endif



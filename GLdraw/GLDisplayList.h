#ifndef GL_DISPLAY_LIST_H
#define GL_DISPLAY_LIST_H

#include <memory>

namespace GLDraw {

struct GLDisplayList
{
  explicit GLDisplayList(int count=1);
  GLDisplayList(const GLDisplayList& rhs)=default;
  ~GLDisplayList();
  operator bool() const { return isCompiled(); }
  GLDisplayList& operator = (const GLDisplayList& rhs)=default;
  bool isCompiled() const;
  void beginCompile(int index=0);
  void endCompile();
  void call(int index=0) const;
  void callAll() const;
  void erase();

  std::shared_ptr<int> id;
  int count;
};

} //namespace GLDraw

#endif



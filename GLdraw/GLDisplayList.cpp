#include <KrisLibrary/Logger.h>
#include "GLDisplayList.h"
#include "GL.h"
#include <errors.h>
#include <stdio.h>

using namespace GLDraw;

static int gNumDisplayLists = 0;

GLDisplayList::GLDisplayList(int _count)
  :count(_count)
{}

GLDisplayList::~GLDisplayList()
{
  erase();
}


bool GLDisplayList::isCompiled() const 
{
  return bool(id);
}

void GLDisplayList::beginCompile(int index)
{
  if(!id) {
    id = std::make_shared<int>();
    *id = glGenLists(count);
    gNumDisplayLists += count;
    if(gNumDisplayLists > 3000)
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning, compiling new OpenGL display list id "<<*id<<", total number "<<gNumDisplayLists);
  }
  glNewList(*id+index,GL_COMPILE);
}

void GLDisplayList::endCompile()
{
  if(!id) return;
  //LOG4CXX_INFO(KrisLibrary::logger(),"End compile,  list "<<*id);
  glEndList();
}

void GLDisplayList::call(int index) const
{
  if(!id) return;
  //LOG4CXX_INFO(KrisLibrary::logger(),"Calling list "<<*id+index);
  glCallList(*id+index);
}

void GLDisplayList::callAll() const
{
  if(!id) return;
  for(int i=0;i<count;i++)
    glCallList(*id+i);
}

void GLDisplayList::erase()
{
  if(id && id.use_count()==1) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Erasing OpenGL display list "<<*id);
    glDeleteLists(*id,count);
    gNumDisplayLists -= count;
  }
  //else if(id)
    //LOG4CXX_INFO(KrisLibrary::logger(),"Not yet erasing OpenGL display list "<<*id<<" has ref count "<<id.getRefCount());

  id=NULL;
}

#include <KrisLibrary/Logger.h>
#include "GLDisplayList.h"
#include "GL.h"
#include <errors.h>
#include <stdio.h>
#include <vector>

using namespace GLDraw;

static int gNumDisplayLists = 0;
const static int MAX_STALE_DISPLAY_LISTS = 100;

class DisplayListManager
{
public:
  DisplayListManager();
  ~DisplayListManager();
  int Allocate(int count=1);
  void Deallocate(int id,int count=1);

  std::vector<int> freedDisplayLists;
  std::vector<int> freedDisplayListCounts;
};

DisplayListManager::DisplayListManager()
{}

DisplayListManager::~DisplayListManager()
{
  for(size_t i=0;i<freedDisplayLists.size();i++) {
    glDeleteLists(freedDisplayLists[i],freedDisplayListCounts[i]);
    gNumDisplayLists -= freedDisplayListCounts[i];
  }
}

int DisplayListManager::Allocate(int count)
{
  for(size_t i=0;i<freedDisplayLists.size();i++) {
    if(freedDisplayListCounts[i] == count) {
      int id = freedDisplayLists[i];
      freedDisplayLists[i] = freedDisplayLists.back();
      freedDisplayListCounts[i] = freedDisplayListCounts.back();
      freedDisplayLists.resize(freedDisplayLists.size()-1);
      freedDisplayListCounts.resize(freedDisplayListCounts.size()-1);
      return id;
    }
  }
  int id = glGenLists(count);
  gNumDisplayLists += count;
  if(gNumDisplayLists > 3000)
    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, compiling new OpenGL display list id "<<id<<", total number "<<gNumDisplayLists);
  return id;
}

void DisplayListManager::Deallocate(int id,int count)
{
  if(freedDisplayLists.size() >= MAX_STALE_DISPLAY_LISTS) {
    glDeleteLists(id,count);
    gNumDisplayLists -= count;
  }
  else {
    freedDisplayLists.push_back(id);
    freedDisplayListCounts.push_back(count);
  }
}

static DisplayListManager gDisplayListManager;

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
  if(id == NULL) {
    id = std::make_shared<int>();
    *id = gDisplayListManager.Allocate(count);
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
    gDisplayListManager.Deallocate(*id,count);
  }
  //else if(id)
    //LOG4CXX_INFO(KrisLibrary::logger(),"Not yet erasing OpenGL display list "<<*id<<" has ref count "<<id.use_count());

  id=NULL;
}

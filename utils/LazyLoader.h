#ifndef UTILS_LAZY_LOADER_H
#define UTILS_LAZY_LOADER_H

#include <utils.h>
#include "FileLoader.h"
#include "SmartPointer.h"

/** @ingroup Utils
 * @brief A utility that allows objects from files to be loaded from and
 * saved to disk on demand.
 *
 * The easiest interface to create an object is by Set(), or the constructor.
 * The object will be loaded on the * or -> operators.
 *
 * SetObject() takes ownership of the pointer.
 * ReleaseObject() releases ownership of the pointer.
 * Load() loads the object only if it is not already loaded.  If loading 
 *        is successful, erases the dirty bit.
 * Unload() unloads the object (deletes it) if loaded.  Erases the dirty bit.
 * ForceLoad() is the equivalent of Unload(); Load()
 *
 * If this is marked as dirty and saveOnDestruction is true, then
 * the object is saved to disk on destruction.
 * This must be done manually using SetAutoSave() and SetDirty().
 * Save() erases the dirty flag.
 */
template <class T>
class LazyLoader
{
 public:
  LazyLoader();
  LazyLoader(const string& fn);
  ~LazyLoader();
  operator T* () { Load(); return object; }
  T* operator->() { Load(); return object; }
  operator SmartPointer<T> () { Load(); return object; }

  void Set(const string& fn);
  void SetObject(const SmartPointer<T>& ptr);
  SmartPointer<T> GetObject() const { return object; }
  void SetFileName(const string& fn) { fileName=fn; }
  const string& GetFileName() const { return fileName; }
  bool IsLoaded() const { return object!=NULL; }
  bool Load();
  bool ForceLoad();
  void Unload();
  bool Save();
  void SetDirty(bool val) { dirty=val; }
  void SetAutoSave(bool val) { saveOnDestruction=val; }
  bool IsDirty() const { return dirty; }

 protected:
  string fileName;
  SmartPointer<T> object;
  bool dirty;
  bool saveOnDestruction;
};

template <class T>
LazyLoader<T>::LazyLoader()
  :object(NULL),dirty(false),saveOnDestruction(false)
{}

template <class T>
LazyLoader<T>::LazyLoader(const string& fn)
  :fileName(fn),object(NULL),dirty(false),saveOnDestruction(false)
{}

template <class T>
LazyLoader<T>::~LazyLoader()
{
  if(object && dirty && saveOnDestruction) Save();
}

template <class T>
void LazyLoader<T>::SetObject(const SmartPointer<T>& ptr)
{
  object = ptr;
}

template <class T>
void LazyLoader<T>::Set(const string& fn)
{
  if(fileName != fn) object=NULL;
  fileName = fn;
}

template <class T>
bool LazyLoader<T>::Load()
{
  if(object) return true;
  object = new T;
  if(FileLoader<T>::Load(fileName.c_str(),*object)) {
    dirty = false;
    return true;
  }
  else return false;
}

template <class T>
bool LazyLoader<T>::ForceLoad()
{
  object = NULL;
  return Load();
}

template <class T>
void LazyLoader<T>::Unload()
{
  object = NULL;
  dirty=false;
}

template <class T>
bool LazyLoader<T>::Save()
{
  if(!object) return false;
  if(FileLoader<T>::Save(fileName.c_str(),*object)) {
    dirty = false;
    return true;
  }
  else return false;
}


#endif

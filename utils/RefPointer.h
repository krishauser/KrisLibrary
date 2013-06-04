#ifndef REF_POINTER_H
#define REF_POINTER_H

#include <stdlib.h>

//NOTE: this is superceded by the SmartPointer class

class RefObjectBase
{
public:
	RefObjectBase();
	void Ref();
	void Unref();
	void UnrefNoDelete();
	inline int NumRefs() const { return numRefs; }

protected:
	//protected so you can't delete this automatically
	virtual ~RefObjectBase();

private:
	int numRefs;
};

//obj must define the methods Ref and Unref
template <class obj>
class RefPointer
{
public:
	typedef RefPointer<obj> Pointer;

	RefPointer()
	:object(NULL)
	{}

	RefPointer(obj* obj_in)
	:object(obj_in)
	{ if(object != NULL) object->Ref();	}

	RefPointer(const RefPointer& rhs)
	:object(rhs.object)
	{ if(object != NULL) object->Ref(); }

	~RefPointer()
	{ if(object != NULL) object->Unref(); }

	inline operator obj* () const { return object; }
	inline obj* operator ->() const { return object; }
	inline const Pointer& operator = (const Pointer& rhs) { return operator=(rhs.object); }
	inline const Pointer& operator = (obj* obj_in) {
		if(object) object->Unref();
		object=obj_in;
		if(object) object->Ref();
		return *this;
	}
	inline bool operator == (const Pointer& rhs) const { return (object==rhs.object); }
	inline bool operator != (const Pointer& rhs) const { return (object!=rhs.object); }
	inline bool isNull() const { return (object==NULL); }

private:
	obj* object;
};

//RefStorage
//stores a RefPointer that keeps an object persistent
//obj must define the methods Ref and Unref
template <class obj>
class RefStorage
{
public:
	typedef RefStorage<obj> Storage;
	typedef RefPointer<obj> Pointer;

	RefStorage()
	:pointer(new obj)
	{}

	RefStorage(const Pointer& ptr)
	:pointer(ptr)
	{}

	RefStorage(const Storage& rhs)
	:pointer(rhs.pointer)
	{}

	Pointer GetPointer() const { return pointer; }

	const Storage& operator = (const Storage& storage)
	{ pointer=storage.pointer; return *this; }
	const Storage& operator = (const Pointer& ptr)
	{ pointer=ptr; return *this; }

	operator const obj& () const { return *pointer; }
	void Copy(const Storage& storage) { pointer->Copy(*storage.pointer); }
	void Copy(const obj& storage) { pointer->Copy(storage); }

private:
	Pointer pointer;
};



#endif

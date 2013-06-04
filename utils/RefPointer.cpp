//#include "stdafx.h"
#include "RefPointer.h"
#include <iostream>

RefObjectBase::RefObjectBase()
:numRefs(0)
{}

RefObjectBase::~RefObjectBase()
{
	if(numRefs != 0) { std::cerr<<"Deleting object without releasing refs"<<std::endl; abort(); }
}

void RefObjectBase::Ref()
{
	numRefs++;
}

void RefObjectBase::Unref()
{
	UnrefNoDelete();
	if(numRefs == 0) delete this;
}

void RefObjectBase::UnrefNoDelete()
{
	if(numRefs <= 0) { std::cerr<<"Unreffing with 0 refs!"<<std::endl; abort(); }
	numRefs--;
}

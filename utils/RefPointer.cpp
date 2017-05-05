#include <log4cxx/logger.h>
#include <KrisLibrary/logDummy.cpp>
//#include "stdafx.h"
#include "RefPointer.h"
#include <iostream>

RefObjectBase::RefObjectBase()
:numRefs(0)
{}

RefObjectBase::~RefObjectBase()
{
	if(numRefs != 0) { LOG4CXX_ERROR(logger,"Deleting object without releasing refs"<<"\n"); abort(); }
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
	if(numRefs <= 0) { LOG4CXX_ERROR(logger,"Unreffing with 0 refs!"<<"\n"); abort(); }
	numRefs--;
}

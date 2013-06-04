#include "set.h"
#include <stdincludes.h>
#include <assert.h>

Mapping::Mapping()
: _map(NULL), _size(0)
{}

Mapping::Mapping(int size)
  :_map(NULL)_size(0)
{
  init(size);
}

Mapping::Mapping(int size, int initVal)
: _map(NULL), _size(0)
{
	init(size, initVal);
}

Mapping::~Mapping()
{
	cleanup();
}

void Mapping::init(int size)
{
	cleanup();
	_size = size;
	_map = new int[_size];
}

void Mapping::init(int size, int initVal)
{
	init(size);
	clear(initVal);
}

void Mapping::resize(int size)
{
	if(size != _size)
		init(size);
}

void Mapping::resize(int size, int initVal)
{
	resize(size);
	clear(initVal);
}

void Mapping::cleanup()
{
	SafeArrayDelete(_map);
	_size = 0;
}

void Mapping::clear(int initVal)
{
	for(int i=0; i<_size; i++)
		_map[i] = initVal;
}


int Set_Membership::size() const
{
	int num = 0;
	for(int i=0; i<Mapping::getSize(); i++)
		if(Mapping::getMap(i) != 0)
			num++;
	return num;
}

bool Set_Membership::Iterator::next()
{
	while(cur < mySet->maxSize())
	{
		cur++;
		if(mySet->contains(cur))
			return true;
	}
	return false;
}


Set_List::Set_List()
: _numItems(0)
{}

Set_List::Set_List(int size)
: Mapping(size), _numItems(0)
{}


void Set_List::init(int size)
{
	Mapping::init(size);
	clear();
}

void Set_List::resize(int size)
{
	Mapping::resize(size);
	clear();
}

//void Set_List::cleanup();

void Set_List::add(int item)
{
	assert(item < maxSize());
	if(!contains(item)) {
		assert(_numItems < maxSize());
		Mapping::setMap(_numItems, item);
		_numItems++;
	}
}

void Set_List::remove(int item)
{
	int ind = getIndex(item);
	if(ind == -1) return;

	int replacement = -1;
	if(_numItems >= 1)
	{
		replacement = Mapping::getMap(_numItems-1);
		Mapping::setMap(_numItems-1, -1);
	}
	Mapping::setMap(ind, replacement);
	_numItems--;
}

void Set_List::clear()
{
	_numItems = 0;
}

bool Set_List::contains(int item) const
{
	int ind = getIndex(item);
	return ind != -1;
}

int Set_List::getIndex(int item) const
{
	for(int i=0; i<_numItems; i++)
		if(Mapping::getMap(i) == item) return i;
	return -1;

}



void Set_Dual::init(int size)
{
	list.init(size);
	listIndex.init(size, -1);
	_numItems = 0;
}

void Set_Dual::resize(int size)
{
	list.resize(size);
	listIndex.resize(size, -1);
	_numItems = 0;
}

void Set_Dual::cleanup()
{
	list.cleanup();
	listIndex.cleanup();
	_numItems = 0;
}

void Set_Dual::add(int item)
{
	if(!contains(item))
	{
		listIndex.setMap(item, _numItems);
		list.setMap(_numItems, item);
		_numItems++;
	}
}

void Set_Dual::remove(int item)
{
	if(contains(item))
	{
		int ind = listIndex.getMap(item);
		listIndex.setMap(item, -1);
		int replacement = -1;
		if(_numItems > 0)
		{
			replacement = list.getMap(_numItems-1);
			listIndex.setMap(replacement, ind);
		}
		list.setMap(ind, replacement);
		_numItems--;
	}
}

void Set_Dual::clear()
{
	for(int i=0; i<_numItems; i++)
		listIndex.setMap(list.getMap(i), -1);
	_numItems = 0;
}

#ifndef UTILS_SET_H
#define UTILS_SET_H

//maps integers in the range [0,size)
class Mapping
{
public:
	Mapping();
	Mapping(int size);
	Mapping(int size, int initVal);
	~Mapping();

	void init(int size);
	void init(int size, int initVal);
	void resize(int size);
	void resize(int size, int initVal);
	void cleanup();

	void clear(int initVal = -1);

	//maps a to b
	inline void setMap(int a, int b) { _map[a] = b; }
	inline int getMap(int a) const { return _map[a]; }

	inline int* getData() const { return _map; }
	inline int getSize() const { return _size; }

private:
	int* _map;
	int _size;
};

/*********************************************************
 * Set guidelines:
 *
 * Contains integers from int the range [0,maxSize())
 * Support the following operations:
 * init, resize, cleanup
 * add, remove, clear
 * contains
 * size, maxsize
 *
 * have an Iterator class with the operations
 * next, reset, operator int ()
 *
 *********************************************************/

/*********************************************************
 * Set_Membership
 *
 * Holds a boolean flag if the item is in the set.
 *
 * O(1) operations for add, remove, contains
 * O(n) operations for resize, size, iterate, clear
 *
 *********************************************************/

class Set_Membership : private Mapping
{
public:
	inline Set_Membership() {}
	inline Set_Membership(int size) : Mapping(size,0) {}

	inline void init(int size) { Mapping::init(size, 0); }
	inline void resize(int size)  { Mapping::resize(size, 0); }
	inline void cleanup() { Mapping::cleanup(); }

	inline void add(int item) { Mapping::setMap(item,1); }
	inline void remove(int item)  { Mapping::setMap(item,0); }
	inline void clear() { Mapping::clear(0); }
	inline bool contains(int item) const { return Mapping::getMap(item) == 1; }

	int size() const;
	inline int maxSize() const { return Mapping::getSize(); }

	struct Iterator
	{
	public:
		inline Iterator(const Set_Membership* s) : mySet(s), cur(-1) {}
		inline void reset() { cur = -1; }
		bool next();
		inline operator int () const { return cur; }

	private:
		const Set_Membership* mySet;
		int cur;
	};

	inline Iterator iterator() const { return Iterator(this); }
};

/*********************************************************
 * Set_List
 *
 * Holds a list of contained items
 *
 * O(1) operations for size, iterate, clear
 * O(k) operations for add, remove, contains
 * O(n) operations for resize
 *
 *********************************************************/
class Set_List : private Mapping
{
public:
	Set_List();
	Set_List(int size);

	void init(int size);
	void resize(int size);
	void cleanup();

	void add(int item);
	void remove(int item);
	void clear();
	bool contains(int item) const;

	inline int size() const { return _numItems; }
	inline int maxSize() const { return Mapping::getSize(); }

	int getIndex(int item) const;
	inline int getItem(int index) const { return Mapping::getMap(index); }

	struct Iterator
	{
	public:
		inline Iterator(const Set_List* s) :mySet(s), cur(-1) {}
		inline void reset() { cur = -1; }
		inline bool next() { cur++; return cur < mySet->size(); }
		inline operator int () const { return mySet->getItem(cur); }

	private:
		const Set_List* mySet;
		int cur;
	};

	inline Iterator iterator() const { return Iterator(this); }

private:
	int _numItems;
};

/*********************************************************
 * Set_Dual
 *
 * Combines the best of both worlds with the membership
 * and list sets, by storing a list mapping
 *
 * O(1) operations for add, remove, size, contains, iterate
 * O(k) operations for clear
 * O(n) operations for resize
 *
 *********************************************************/
class Set_Dual
{
public:
	void init(int size);
	void resize(int size);
	void cleanup();

	void add(int item);
	void remove(int item);
	void clear();
	inline bool contains(int item) const { return listIndex.getMap(item) != -1; }

	inline int size() const { return _numItems; }
	inline int maxSize() const { return list.getSize(); }

private:
	Mapping list;
	Mapping listIndex;
	int _numItems;
};

#endif

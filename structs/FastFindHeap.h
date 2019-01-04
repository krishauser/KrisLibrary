#ifndef FAST_FIND_HEAP_H
#define FAST_FIND_HEAP_H

#include <KrisLibrary/Logger.h>
#include <vector>
#include <map>
#include <iostream>
#include <assert.h>
using namespace std;

/** 
 * @brief Implements a heap that stores objects of type type, with priority
 * keys of type ptype.
 *
 * The highest priority item is always on top.
 *
 * pop(), push(), and _adjust() run in worst-case time O(log^2 n)
 * find() runs in worst-case time O(log n)
 */
template <class type,class ptype>
class FastFindHeap
{
public:
  FastFindHeap() : h(1) 
  {}

  inline const type& top() const { return h[1].x; }

  void pop()
  {
    assert(!empty());
    objectToHeapIndex.erase(objectToHeapIndex.find(h[1].x));
    h[1]=h.back();
    h.resize(h.size()-1);
    if(h.size()>1) heapifyDown(1);
  }

  void push(const type& x,const ptype& p)
  {
    assert(objectToHeapIndex.find(x) == objectToHeapIndex.end());
    objectToHeapIndex[x] = (int)h.size();
    item it;
    it.x=x;
    it.p=p;
    h.push_back(it);
    heapifyUp((int)h.size()-1);
  }

  int find(const type& x) const
  {
    typename map<type,int>::const_iterator i=objectToHeapIndex.find(x);
    if(i==objectToHeapIndex.end()) return 0;
    return i->second;
  }

  void adjust(const type& x, const ptype& p)
  {
    int i=find(x);
    if(i) _adjust(i,p);
    else push(x,p);
  }

  //NOTE: i must be an index in the array
  void _adjust(int i,const ptype& p)
  {
    assert(i>=1 && i<=size());
    if(h[i].p < p) { //increase
      h[i].p=p;
      heapifyUp(i);
    }
    else {  //decrease
      h[i].p=p;
      heapifyDown(i);
    }
  }
  
  inline void clear() { h.resize(1); objectToHeapIndex.clear(); }
  inline bool empty() const { return h.size()==1; }
  inline int size() const { return (int)h.size()-1; }

  bool isHeap() const
  {
    for(int i=2;i<=size();i++)
      if(h[parent(i)].p < h[i].p) return false;
    return true;
  }

  void print() const {
    int level=1;
    for(int i=1;i<=size();i++) {
      if(i == (1<<level)) {
        LOG4CXX_INFO(KrisLibrary::logger(),"\n");
        level++;
      }
      LOG4CXX_INFO(KrisLibrary::logger(),"("<<h[i].x<<","<<h[i].p<<")"<<" ");
    }
    LOG4CXX_INFO(KrisLibrary::logger(),"\n");
  }
  
private:
  struct item
  {
    type x;
    ptype p;
  };

  //assume 1-based array
  inline int parent(int i) const { return i>>1; }
  inline int child1(int i) const { return i<<1; }
  inline int child2(int i) const { return (i<<1)+1; }

  void heapifyUp(int i)
  {
    item it=h[i];
    while(i>1) {
      int par=parent(i);
      if(it.p>h[par].p) {
        h[i]=h[par];
	objectToHeapIndex[h[i].x]=i;
      }
      else break;
      i=par;
    }
    h[i]=it;
    objectToHeapIndex[h[i].x]=i;
  }

  void heapifyDown(int i)
  {
    item it=h[i];
    int child;
    int size = (int)h.size();
    while(child1(i)<size) {
      child = child1(i);
      if(child+1<size && h[child+1].p > h[child].p)
        child++;
      if(it.p < h[child].p) {
        h[i]=h[child];
	objectToHeapIndex[h[i].x]=i;
      }
      else break;
      i=child;
    }
    h[i]=it;
    objectToHeapIndex[h[i].x]=i;
  }


  map<type,int> objectToHeapIndex;
  vector<item> h;
};


#endif

#ifndef HEAP_H
#define HEAP_H

#include <KrisLibrary/Logger.h>
#include <vector>
#include <assert.h>
using namespace std;

/** 
 * @brief Implements a heap that stores objects of type type, with priority
 * keys of type ptype.
 *
 * The highest priority item is always on top.
 *
 * pop(), push(), and _adjust() run in worst-case time O(log n)
 * adjust() and find() run in worst-case time O(n)
 */
template <class type,class ptype>
class Heap
{
public:
  Heap() : h(1) 
  {}

  inline const type& top() const { return h[1].x; }
  inline const ptype& topPriority() const { return h[1].p; }

  void pop()
  {
    assert(!empty());
    h[1]=h.back();
    h.resize(h.size()-1);
    if(h.size()>1) heapifyDown(1);
  }

  void push(const type& x,const ptype& p)
  {
    item it;
    it.x=x;
    it.p=p;
    h.push_back(it);
    heapifyUp((int)h.size()-1);
  }

  int find(const type& x) const
  {
    for(size_t i=1;i<h.size();i++)
      if(h[i].x==x) return (int)i;
    return 0;
  }

  int findByPriority(const type& x,const ptype& p) const
  {
    return findElement(1,x,p);
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

  inline void clear() { h.resize(1); }
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
      if(it.p>h[par].p)
        h[i]=h[par];
      else break;
      i=par;
    }
    h[i]=it;
  }

  void heapifyDown(int i)
  {
    item it=h[i];
    int size = (int)h.size();
    while(child1(i)<size) {
      int child = child1(i);
      if(child+1<size && h[child+1].p > h[child].p)
        child++;
      if(it.p < h[child].p)
        h[i]=h[child];
      else break;
      i=child;
    }
    h[i]=it;
  }

  int findElement(int i,const type& x,const ptype& p) const {
    if(p > h[i].p) return 0;
    if(p == h[i].p && x == h[i].x) return i;
    int res=findElement(child1(i),x,p);
    if(res != 0) return res;
    res=findElement(child2(i),x,p);
    return res;
  }

  vector<item> h;
};


#endif

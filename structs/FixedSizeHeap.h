#ifndef FIXED_SIZE_HEAP_H
#define FIXED_SIZE_HEAP_H

#include <KrisLibrary/Logger.h>
#include <vector>
#include <iostream>
#include <KrisLibrary/errors.h>

/** @brief A heap of fixed maximum size N.  Each element is indexed by 
 * an integer 0...N-1.  The priority key of each element is of type ptype.
 */
template <class ptype>
class FixedSizeHeap
{
public:
  FixedSizeHeap(int maxValue)
    :objectToHeapIndex(maxValue),h(1)
  {
    for(int i=0;i<maxValue;i++)
      objectToHeapIndex[i]=0;
    h.reserve(maxValue+1);
  }

  FixedSizeHeap()
    :h(1)
  {
  }

  void init(int maxValue)
  {
    objectToHeapIndex.resize(maxValue);
    std::fill(objectToHeapIndex.begin(),objectToHeapIndex.end(),0);
    h.resize(1);
    h.reserve(maxValue+1);
  }

  void increaseCapacity(int maxValue)
  {
    objectToHeapIndex.resize(maxValue,0);
    h.reserve(maxValue+1);
  }

  inline int top() const { return h[1].x; }

  inline ptype topPriority() const { return h[1].p; }

  inline ptype priority(int item) const { return h[objectToHeapIndex[item]].p; }

  void pop()
  {
    Assert(!empty());
    objectToHeapIndex[h[1].x]=0;
    h[1]=h.back();
    h.resize(h.size()-1);
    if(h.size()>1) heapifyDown(1);
  }

  void push(int x,const ptype& p)
  {
    Assert(objectToHeapIndex[x]==0);
    objectToHeapIndex[x] = (int)h.size();
    item it;
    it.x=x;
    it.p=p;
    h.push_back(it);
    heapifyUp((int)h.size()-1);
  }

  int find(const int x) const
  {
    Assert(x >= 0 && x < (int)objectToHeapIndex.size());
    return objectToHeapIndex[x];
  }


  void adjust(const int x, const ptype& p)
  {
    int i=find(x);
    if(i) adjustByHeapIndex(i,p);
    else push(x,p);
  }
  
  void adjustByHeapIndex(int i,const ptype& p)
  {
    Assert(i>=1 && i<=size());
    if(h[i].p < p) { //increase
      h[i].p=p;
      heapifyUp(i);
    }
    else {  //decrease
      h[i].p=p;
      heapifyDown(i);
    }
  }

  inline void clear() { h.resize(1); std::fill(objectToHeapIndex.begin(),objectToHeapIndex.end(),0); }
  inline bool empty() const { return h.size()==1; }
  inline int size() const { return (int)h.size()-1; }
  inline int maxObjects() const { return (int)objectToHeapIndex.size(); }

  bool isHeap() const
  {
    for(size_t i=1;i<h.size();i++) 
      Assert(objectToHeapIndex[h[i].x] == (int)i);
    for(size_t i=0;i<objectToHeapIndex.size();i++)
      if(objectToHeapIndex[i]!=0)
	Assert(h[objectToHeapIndex[i]].x == i);

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
    int x;   //the object index
    ptype p; //the priority key
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

  //stores the mapping from object identifiers to heap items
  std::vector<int> objectToHeapIndex;
  //stores the items in heap order, from indices 1 to N
  std::vector<item> h;
};


#endif

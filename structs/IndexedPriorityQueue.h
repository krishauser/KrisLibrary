#ifndef INDEXED_PRIORITY_QUEUE_H
#define INDEXED_PRIORITY_QUEUE_H

#include <set>
#include <map>

/** @brief Contains a priority queue with an associated index, allowing
 * updates of the priority value.  Priority defined with lowest value first.
 *
 * An index is provided with each priority queue entry, allowing a priority
 * value to be located from an index in logarithmic time.  
 * 
 * IT is the type of the index.
 * PT is the type of the priority value. 
 * It is assumed that the < operator has been defined for both types.
 *
 * Most members operate like STL's priority_queue<pair<PT,IT> >.  Be sure
 * to remember that the returned pair has PT as its first type, and IT
 * as its second!
 *
 * The new members are as follows:
 * - iterator insert(const IT& index,const PT& p): inserts a priority queue
 *   entry p, indexed by index.  Returns the inserted iterator.
 * - iterator find(const IT& index): returns the iterator corresponding
 *   to index.
 * - void refresh(const IT& index,const PT& p): sets the priority value
 *   of index to p.  If index does not exist, it is created.
 * - bool is_valid() const: Returns true if the internal structures are valid.
 *
 * One index value can only map to one priority value (that is, 
 * insert(2,0.5) and insert(2,0.7) is an invalid operation).
 *
 * Undefined behavior will result if any iterator is modified.
 */
template <class IT,class PT>
class IndexedPriorityQueue
{
 public:
  typedef std::pair<PT,IT> value_type;
  typedef typename std::set<std::pair<PT,IT> >::iterator iterator;
  typedef typename std::set<std::pair<PT,IT> >::const_iterator const_iterator;

  iterator begin() { return q.begin(); }
  const_iterator begin() const { return q.begin(); }
  iterator end() { return q.end(); }
  const iterator end() const { return q.end(); }
  const value_type& front() const { return *q.begin(); }
  const value_type& back() const { const_iterator i=q.end(); --i; return *i; }
  bool empty() const { return q.empty(); }
  void clear() { q.clear(); indices.clear(); }
  size_t size() const { return q.size(); }
  const value_type& top() const { return *q.begin(); }
  void erase(iterator i) {
    size_t n=indices.erase(i->second);
    Assert(n==1);
    q.erase(i);
  }
  void pop() { 
    erase(q.begin());
  }
  iterator insert(const value_type& item) {
    iterator i=q.insert(item).first;
    Assert(indices.count(item.second)==0);
    indices[item.second] = i;
    return i;
  }
  template <class InputIterator>
  void insert(InputIterator i,InputIterator j) {
    for(InputIterator k=i;k!=j;++k)
      insert(*k);
  }
  iterator insert(const IT& index,const PT& p) {
    iterator i=q.insert(value_type(p,index)).first;
    Assert(indices.count(index)==0);
    indices[index] = i;
    return i;
  }
  iterator find(const IT& index) {
    auto i=indices.find(index);
    if(i==indices.end()) return end();
    return i->second;
  }
  iterator refresh(const IT& index,const PT& p) {
    iterator i=find(index);
    if(i != q.end()) q.erase(i);
    i = q.insert(value_type(p,index)).first;
    indices[index] = i;
    return i;
  }
  bool is_valid() const {
    for(auto i=indices.begin();i!=indices.end();++i) {
      if(i->second->second != i->first) return false;
    }
    return true;
  }

 private:
  std::map<IT,iterator> indices;
  std::set<std::pair<PT,IT> > q;
};

#endif

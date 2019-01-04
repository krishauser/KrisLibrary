#ifndef PRIORITY_QUEUE_2_H
#define PRIORITY_QUEUE_2_H

#include <vector>

namespace std {

/** @brief A better priority queue class than the STL version
 *
 * Allows iterating and defining new priority orderings
 * (no need to overload <)
 */
template <class T,class Cmp=less<T> >
class priority_queue2 : private vector<T>
{
 private:
  typedef priority_queue2<T,Cmp> MyT;
  typedef vector<T> ParentT;
  Cmp compareObject;

 public:
  typedef T value_type;
  typedef Cmp value_compare;
  typedef typename vector<T>::const_iterator const_iterator;
  typedef typename vector<T>::const_reverse_iterator const_reverse_iterator;

  priority_queue2() {}
  priority_queue2(const MyT& other) :ParentT(other) {}
 
  bool empty() const { return ParentT::empty(); }
  void clear() { ParentT::clear(); }
  const T& top() const { return ParentT::front(); }
  const T& front() const { return ParentT::front(); }
  const T& back() const { return ParentT::back(); }
  void push(const T& obj) { ParentT::push_back(obj); push_heap(ParentT::begin(),ParentT::end(),compareObject); }
  void pop() { pop_heap(ParentT::begin(),ParentT::end(),compareObject); ParentT::resize(ParentT::size()-1); }
  const_iterator begin() const { return ParentT::begin(); }
  const_iterator end() const { return ParentT::end(); }

  void swap(MyT& other) { swap((ParentT&)*this,other); }
};

template <class T,class Cmp>
void swap(priority_queue2<T,Cmp>& a,priority_queue2<T,Cmp>& b)
{
  a.swap(b);
}

} //namespace std

#endif

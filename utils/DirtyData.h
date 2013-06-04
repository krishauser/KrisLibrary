#ifndef DIRTY_DATA_H
#define DIRTY_DATA_H

template <class type>
struct DirtyData : public type
{
  DirtyData() : dirty(true) {}
  DirtyData(const type& t) : type(t),dirty(true) {}

  bool dirty;
};

#endif

#include "Subset.h"
#include <algorithm>
#include <errors.h>
#include <assert.h>
using namespace std;

Subset::Subset(int _maxItem):maxItem(_maxItem) {}
Subset::Subset(const Subset& s):maxItem(s.maxItem),items(s.items) {}
Subset::Subset(const vector<bool>& bits)
{
  maxItem=(int)bits.size();
  /*
  for(size_t i=0;i<bits.size();i++)
    if(bits[i]) items.insert(items.end(),(int)i);
  */
  for(size_t i=0;i<bits.size();i++)
    if(bits[i]) items.push_back((int)i);
}

void Subset::insert(int item)
{
  assert(item < maxItem);
  vector<int>::iterator it = lower_bound(items.begin(),items.end(),item);
  if(it != items.end()) {
    if(*it == item) return;
  }
  items.insert(it,item);
}

void Subset::insert_end(int item)
{
  assert(item < maxItem);
  if(!items.empty())
    assert(item > items.back());
  //items.insert(--items.end(),item);
  items.push_back(item);
}

void Subset::remove(int item)
{
  vector<int>::iterator it=lower_bound(items.begin(),items.end(),item);
  if(it == items.end()) return;
  if(*it != item) return;
  items.erase(it);
}

Subset::iterator Subset::find(int item)
{
  vector<int>::iterator it=lower_bound(items.begin(),items.end(),item);
  if(it == items.end()) return items.end();
  if(*it != item) return items.end();
  return it;
}

Subset::const_iterator Subset::find(int item) const
{
  vector<int>::const_iterator it=lower_bound(items.begin(),items.end(),item);
  if(it == items.end()) return items.end();
  if(*it != item) return items.end();
  return it;
}

bool Subset::operator < (const Subset& s) const
{
  if(maxItem < s.maxItem) return true;
  if(maxItem > s.maxItem) return false;
  return lexicographical_compare(items.begin(),items.end(),s.items.begin(),s.items.end());
}

bool Subset::operator > (const Subset& s) const
{
  return s < *this;
}

bool Subset::operator == (const Subset& s) const
{
  return maxItem == s.maxItem && items==s.items;
}

bool Subset::operator != (const Subset& s) const
{
  return !(s==*this);
}

Subset Subset::operator + (const Subset& s) const
{
  Subset res(std::max(maxItem,s.maxItem));
  /*
  set_union(items.begin(),items.end(),s.items.begin(),s.items.end(),inserter(res.items, res.items.end()));
  */
  res.items.resize(res.maxItem);
  vector<int>::iterator i=set_union(items.begin(),items.end(),s.items.begin(),s.items.end(),res.items.begin());
  res.items.resize(i-res.items.begin());
  return res;
}

Subset Subset::operator - (const Subset& s) const
{
  Subset res(maxItem);
  //set_difference(items.begin(),items.end(),s.items.begin(),s.items.end(),inserter(res.items, res.items.end()));
  res.items.resize(items.size());
  vector<int>::iterator i=set_difference(items.begin(),items.end(),s.items.begin(),s.items.end(),res.items.begin());
  res.items.resize(i-res.items.begin());
  return res;
}

Subset Subset::operator & (const Subset& s) const
{
  Subset res(std::min(maxItem,s.maxItem));
  //set_intersection(items.begin(),items.end(),s.items.begin(),s.items.end(),inserter(res.items, res.items.end()));
  res.items.resize(res.maxItem);
  vector<int>::iterator i=set_intersection(items.begin(),items.end(),s.items.begin(),s.items.end(),res.items.begin());
  res.items.resize(i-res.items.begin());
  return res;
}

Subset Subset::operator - () const
{
  Subset res(maxItem);
  /*
  for(int i=0;i<maxItem;i++)
    if(items.count(i)==0)
      res.items.insert(res.items.end(),i);
  */
  int last=0;
  for(size_t i=0;i<items.size();i++) {
    for(int j=last;j<items[i];j++)
      res.items.push_back(j);
    last = items[i]+1;
  }
  for(int j=last;j<maxItem;j++)
    res.items.push_back(j);
  return res;
}

bool Subset::is_subset(const Subset& s) const
{
  if(maxItem > s.maxItem) return false;
  return includes(s.items.begin(),s.items.end(),items.begin(),items.end());
}

ostream& operator << (ostream& out,const Subset& s)
{
  out<<"{";
  //for(set<int>::const_iterator i=s.items.begin();i!=s.items.end();i++)
  for(vector<int>::const_iterator i=s.items.begin();i!=s.items.end();i++)
    out<<*i<<" ";
  out<<"}";
  return out;
}

#include <KrisLibrary/Logger.h>
#include "CoverSet.h"
#include <iostream>
#include <algorithm>
#include <set>
#include <list>
#include <errors.h>

/*
//returns true if it spills over the max value
template <class T>
bool increment(vector<T>& counter,const T& max,const T inc=1)
{
  T carry=inc;
  for(int k=(int)counter.size()-1;k>=0;k--) {
    Assert(counter[k] < max);
    T temp=counter[k]+carry;
    if(temp >= max) {
      counter[k] = temp%max;
      carry = temp/max;
    }
    else {
      counter[k] = temp;
      return false;
    }
  }
  Assert(carry != 0);
  if(carry >= max) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Final carry is greater than max, is the increment > max?  if so, this isn't done yet");
    Abort();
  }
  bool res=increment(counter,max,carry);
  Assert(res==false);
  return true;
}
*/

template <class T>
void first_subset(vector<T>& subset,const T& max)
{
  Assert(subset.size() < max);
  T val=0;
  typename vector<T>::iterator i;
  for(i=subset.begin();i!=subset.end();i++) {
    *i = val;
    val+=1;
  }
}

//calculates the next lexicographically ordered subset
//returns true if done
template <class T>
bool next_subset(vector<T>& subset,const T& max)
{
  if(subset.empty()) return true;
  for(size_t k=0;k<subset.size();k++)
    Assert(subset[k] < max);

  int k=(int)subset.size()-1;
  for(;k>=0;k--) {
    //the max potential value for item k is max-(subset.size()-1-k)
    T maxval = max-(T)(subset.size()-1-k);
    if(subset[k]+1 < maxval) {
      subset[k]++;
      break;
    }
  }
  if(k < 0) { //cycled through the entire possible subsets
    for(size_t i=0;i<subset.size();i++)
      subset[i]=i;
    return true;
  }
  //set the first lexicographical subset starting with subset[k]
  for(int j=k+1;j<(int)subset.size();j++)
    subset[j]=subset[k]+j-k;
  return false;
}

bool empty_set(const vector<bool>& A)
{
  for(size_t i=0;i<A.size();i++) if(A[i]) return false;
  return true;
}

int cardinality(const vector<bool>& a)
{
  size_t sa=0;
  for(size_t i=0;i<a.size();i++) if(a[i]) sa++;
  return sa;
}

//S must be a STL container class (with an int-like value type)
template <class T>
bool contains_subset(const vector<bool>& A,const T& S)
{
  for(typename T::const_iterator i=S.begin();i!=S.end();i++)
    if(!A[*i]) return false;
  return true;
}

//S must be a STL container class (with an int-like value type)
template <class T>
bool intersects_subset(const vector<bool>& A,const T& S)
{
  for(typename T::const_iterator i=S.begin();i!=S.end();i++)
    if(A[*i]) return true;
  return false;
}

vector<bool> CalculateCoverset_BruteForce(const vector<vector<bool> >& sets)
{
  if(sets.empty()) return vector<bool>();
  const size_t N=sets.size();
  const size_t M=sets[0].size();
  if(M==0) return vector<bool>();
  for(size_t i=0;i<N;i++) Assert(sets[i].size()==M);

  vector<bool> potential_values(M,false);
  for(size_t i=0;i<N;i++) {
    if(empty_set(sets[i])) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Empty set "<<i<<" given to coverset!");
      return vector<bool>();
    }
    for(size_t j=0;j<M;j++)
      if(sets[i][j]) potential_values[j]=true;
  }
  vector<size_t> potential_items;
  for(size_t j=0;j<M;j++)
    if(potential_values[j]) potential_items.push_back(j);
  if(potential_items.empty()) return vector<bool>();
  size_t P=potential_items.size();

  //go through 2^P combinations of subsets
  for(size_t k=1;k<=P;k++) {
    if(k > 20) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: couldn't find a cover set of size less than 20, quitting");
      return vector<bool>();
    }
    //go through all subsets of size k
    vector<size_t> order(k);
    vector<size_t> subset(k);
    //start with the first subset
    for(size_t i=0;i<k;i++)
      order[i]=i;
    bool done=false;
    while(!done) {
      //map the subset order to the subset items
      for(size_t i=0;i<k;i++)
	subset[i] = potential_items[order[i]];

      //check if the subset misses any set
      bool anyMissed=false;
      for(size_t i=0;i<N;i++) {
	if(!intersects_subset(sets[i],subset)) {
	  anyMissed=true;
	  break;
	}
      }
      if(!anyMissed) {
	//return the subset as a bit vector
	vector<bool> bvsubset(M,false);
	for(size_t j=0;j<k;j++)
	  bvsubset[subset[j]]=true;
	return bvsubset;
      }

      //check the next lexicographical subset
      done=next_subset(order,P);
    }
  }
  AssertNotReached();
  return vector<bool>();
}

vector<bool> CalculateCoverset_IncrementalSorted(const vector<vector<bool> >& sets)
{
  if(sets.empty()) return vector<bool>();
  const size_t N=sets.size();
  const size_t M=sets[0].size();
  for(size_t i=0;i<N;i++) Assert(sets[i].size()==M);
  
  //start with a set of potential minimal sets -- the items in set 0
  size_t min_set_size=0;
  list<set<int> > minimal_sets;
  for(size_t k=0;k<M;k++)
    if(sets[0][k]) {
      minimal_sets.push_back(set<int>());
      minimal_sets.back().insert(k);
    }
  if(minimal_sets.empty()) return vector<bool>();
  min_set_size = 1;

  //now proceed to incrementally find the minimal covering subsets for 
  //sets 1...i by updating the minimal set with set i
  for(size_t i=1;i<N;i++) {
    vector<size_t> potential_values;
    for(size_t k=0;k<M;k++) 
      if(sets[i][k]) potential_values.push_back(k);
    Assert(!potential_values.empty());
    
    //if any minimal_set doesn't intersect set i
    //then it must be augmented with an additional value from set i
    list<set<int> > newsets;
    int m=0;
    for(list<set<int> >::iterator j=minimal_sets.begin();j!=minimal_sets.end();j++) {
      if(!intersects_subset(sets[i],*j)) {
	for(size_t k=0;k<potential_values.size();k++) {
	  if(j->count(potential_values[k]) == 0) {
	    set<int> newset=*j;
	    newset.insert(potential_values[k]);
	    newsets.push_back(newset);
	  }
	  else {
	    //duplicate j
	    newsets.push_back(*j);
	    //break because we don't need any of the other subsets
	    break;
	  }
	}
      }
      else {
	newsets.push_back(*j);
      }
      m++;
    }
    swap(minimal_sets,newsets);

    //erase the non-minimal sets
    min_set_size++;
    for(list<set<int> >::iterator j=minimal_sets.begin();j!=minimal_sets.end();j++) 
      if(j->size() < min_set_size) min_set_size=j->size();
    for(list<set<int> >::iterator j=minimal_sets.begin();j!=minimal_sets.end();j++) {
      if(j->size()!=min_set_size) {
	list<set<int> >::iterator prev=j; prev--;
	minimal_sets.erase(j);
	j=prev;
      }
    }
  }
  Assert(!minimal_sets.empty());
  vector<bool> bvsubset(M,false);
  for(set<int>::iterator i=minimal_sets.front().begin();i!=minimal_sets.front().end();i++)
    bvsubset[*i]=true;
  return bvsubset;
}

//returns true if |a| < |b|
bool set_size_less(const vector<bool>& a,const vector<bool>& b)
{
  return cardinality(a) < cardinality(b);
}

vector<bool> CalculateCoverset_Incremental(const vector<vector<bool> >& sets)
{
  //order the sets by increasing cardinality
  vector<vector<bool> > sorted_sets=sets;
  std::sort(sorted_sets.begin(),sorted_sets.end(),set_size_less);
  return CalculateCoverset_IncrementalSorted(sorted_sets);
}

vector<bool> CalculateCoverset_Greedy(const vector<vector<bool> >& sets)
{
  if(sets.empty()) return vector<bool>();
  const size_t N=sets.size();
  const size_t M=sets[0].size();
  vector<bool> covered(N,false);
  vector<bool> set_cover(M,false);
  size_t numCovered=0;
  while(numCovered < N) {
    //pick the item with the largest cover
    size_t largest=0;
    size_t largest_index=0;
    for(size_t i=0;i<M;i++) {
      size_t size=0;
      for(size_t k=0;k<N;k++)
	if(!covered[k] && sets[k][i]) size++;
      if(size > largest) {
	largest=size;
	largest_index=i;
      }
    }
    if(largest == 0) {
      //no possible cover
      return vector<bool>();
    }

    //add largest_index to the set
    set_cover[largest_index]=true;
    for(size_t k=0;k<N;k++)
      if(sets[k][largest_index]) covered[k]=true;

    numCovered=0;
    for(size_t k=0;k<N;k++)
      if(covered[k]) numCovered++;
  }
  return set_cover;
}




vector<int> CalculateSetCover_Greedy(const vector<vector<bool> >& sets)
{
  if(sets.empty()) return vector<int>();
  const size_t N=sets.size();
  const size_t M=sets[0].size();
  vector<bool> covered(M,false);
  vector<int> set_cover;
  size_t numCovered=0;
  while(numCovered < M) {
    //pick the subset with the largest cover
    size_t largest=0;
    size_t largest_index=0;
    for(size_t i=0;i<N;i++) {
      size_t size=0;
      for(size_t k=0;k<M;k++)
	if(!covered[k] && sets[i][k]) size++;
      if(size > largest) {
	largest=size;
	largest_index=i;
      }
    }
    if(largest == 0) {
      //no possible cover
      return vector<int>();
    }

    //add largest_index to the set
    set_cover.push_back(largest_index);
    for(size_t k=0;k<M;k++)
      if(sets[largest_index][k]) covered[k]=true;

    numCovered=0;
    for(size_t k=0;k<M;k++)
      if(covered[k]) numCovered++;
  }
  return set_cover;
}


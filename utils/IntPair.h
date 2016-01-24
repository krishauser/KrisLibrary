#ifndef INT_PAIR_H
#define INT_PAIR_H

#include <iosfwd>

/** @ingroup Utils
 * @brief A lightweight integer 2-tuple class
 */
struct IntPair
{
  inline IntPair() {}
  inline IntPair(int _a,int _b) :a(_a),b(_b) {}
  inline IntPair(const IntPair& t) :a(t.a),b(t.b) {}
  inline bool operator == (const IntPair& t) const
  { return a==t.a&&b==t.b; }
  inline bool operator != (const IntPair& t) const { return !operator==(t); }
  inline bool operator < (const IntPair& t) const
  { return a<t.a || (a==t.a && b<t.b); }
  inline void set(int _a,int _b) { a=_a; b=_b; }
  inline int operator[](int i) const { return data[i]; }  //i={0,1}
  inline int& operator[](int i) { return data[i]; }  //i={0,1}
  inline int getIndex(int x) const {
    for(int i=0;i<2;i++) if(x==data[i]) return i;
    return -1;
  }
  inline bool contains(int x) const { return a==x||b==x; }
  inline bool contains(int x,int& index) const { 
    index=getIndex(x);
    return (index>=0);
  }
  inline int count(int x) const { 
    int n=0;
    for(int i=0;i<2;i++) if(x==data[i]) n++;
    return n;
  }
  inline void operator += (int ofs) { a+=ofs; b+=ofs; }
  inline void operator -= (int ofs) { a-=ofs; b-=ofs; }

  union {
    int data[2];
    struct { int a,b; };
  };
};

std::ostream& operator << (std::ostream& out,const IntPair& t);
std::istream& operator >> (std::istream& in,IntPair& t);

#endif

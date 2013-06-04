#ifndef INT_TRIPLE_H
#define INT_TRIPLE_H

#include <iostream>

/** @ingroup Utils
 * @brief A lightweight integer 3-tuple class 
*/
struct IntTriple
{
  inline IntTriple() {}
  inline IntTriple(int _a,int _b, int _c) :a(_a),b(_b),c(_c) {}
  inline IntTriple(const IntTriple& t) :a(t.a),b(t.b),c(t.c) {}
  inline bool operator == (const IntTriple& t) const
  { return a==t.a&&b==t.b&&c==t.c; }
  inline bool operator != (const IntTriple& t) const { return !operator==(t); }
  inline bool operator < (const IntTriple& t) const
  { return a<t.a || (a==t.a && (b<t.b || (b==t.b && c<t.c))); }
  inline void set(int _a,int _b,int _c) { a=_a; b=_b; c=_c; }
  inline int operator[](int i) const { return data[i]; }  //i={0,1,2}
  inline int& operator[](int i) { return data[i]; }  //i={0,1,2}
  inline int getIndex(int x) const {
    for(int i=0;i<3;i++) if(x==data[i]) return i;
    return -1;
  }
  inline bool contains(int x) const { return a==x||b==x||c==x; }
  inline bool contains(int x,int& index) const { 
    index=getIndex(x);
    return (index>=0);
  }
  inline int count(int x) const { 
    int n=0;
    for(int i=0;i<3;i++) if(x==data[i]) n++;
    return n;
  }
  inline void getCompliment(int i,int& v1,int& v2) const {
    v1=data[(i+1)%3];
    v2=data[(i+2)%3];
  }
  inline int getCompliment(int i1,int i2) const {
    return data[3-i1-i2];
  }
  inline void operator += (int ofs) { a+=ofs; b+=ofs; c+=ofs; }
  inline void operator -= (int ofs) { a-=ofs; b-=ofs; c-=ofs; }

  union {
    int data[3];
    struct { int a,b,c; };
  };
};

inline std::ostream& operator << (std::ostream& out,const IntTriple& t)
{
  out<<t.a<<" "<<t.b<<" "<<t.c;
  return out;
}

inline std::istream& operator >> (std::istream& in,IntTriple& t)
{
  in >> t.a >> t.b >> t.c;
  return in;
}

#endif

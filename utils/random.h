#ifndef UTILS_RANDOM_H
#define UTILS_RANDOM_H

#include <stdlib.h>
#include <limits.h>

/** @addtogroup Utils */
/*@{*/

/** @brief Interface for a random number generator.
 *
 * These RNG's have the following structure:
 * @code
 * struct RNG { 
 *   void seed(unsigned long);
 *   long int maxValue() const;
 *   long int randInt();  //random long in [0,max]
 *   float randFloat();   //floating point value in [0,1]
 *   double randDouble();  //double fp value in [0,1]
 * };
 * @endcode
 *
 * This RNG uses the rand() function declared in stdlib.h.
 */
struct StandardRNG 
{
  static inline void seed(unsigned long n) { srand(n); }
  static inline long int maxValue() { return RAND_MAX; }
  static inline long int randInt() { return rand(); }
  static inline float randFloat() { return float(rand())/float(RAND_MAX); }
  static inline double randDouble() { return double(rand())/double(RAND_MAX); }

  ///helpers
  static inline long int randInt(long int n) { return randInt()%n; }
  static inline float randFloat(float a, float b) {
    float t = randFloat();
    return a + t*(b-a); 
  }
  static inline double randDouble(double a, double b) {
    double t = randDouble();
    return a + t*(b-a);
  }
};


#if 0
///This RNG generates a lot of warnings
///generates a random 32 bit random number using a linear congruential generator
struct RNG32 
{
  inline void seed(unsigned long n) { state = n; }
  inline unsigned int maxValue() const { return UINT_MAX; }
  inline unsigned long randLong()
  {
    state = 1664525L*state + 1013904223L;
    return state;
  }
  inline long int randInt() { return (long int)randLong(); }
  inline float randFloat()
  {
    unsigned long itemp;
    static unsigned long jflone = 0x3f800000;
    static unsigned long jflmsk = 0x007fffff;
    
    itemp = jflone | (jflmsk & randLong());
    return (*(float *)&itemp)-1.0f;
  }

  ///returns a random double in the range [0,1]
  inline double randDouble()
  {
    unsigned long r[2];
    r[0] = randLong();
    r[1] = randLong();
    static unsigned long jflone = 0x3ff00000;
    static unsigned long jflmsk = 0x000fffff;
    r[0] = jflone | (jflmsk & r[0]);
    double d = *(double*)&r[0];
    return d-1.0;
  }

  //helpers
  inline long int randInt(int n) { return randInt()%n; }
  inline float randFloat(float a, float b) {
    float t = randFloat();
    return a + t*(b-a); 
  }
  inline double randDouble(double a, double b) {
    double t = randDouble();
    return a + t*(b-a);
  }

  unsigned long state;
};
#endif // 0

///uses the ANSI C rand48 functions
#ifndef _WIN32
struct RNG48
{
  static inline void seed(unsigned long n) { srand48(n); }
  static inline long int maxValue() { return INT_MAX; }
  static inline long int randInt() { return lrand48(); }
  static inline float randFloat() { return (float)drand48(); }
  static inline double randDouble() { return drand48(); }

  static inline long int randInt(int n) { return randInt()%n; }
  static inline float randFloat(float a, float b) {
    float t = randFloat();
    return a + t*(b-a); 
  }
  static inline double randDouble(double a, double b) {
    double t = randDouble();
    return a + t*(b-a);
  }
};
#endif

/*@}*/

#endif

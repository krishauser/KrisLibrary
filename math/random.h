#ifndef MATH_RANDOM_H
#define MATH_RANDOM_H

#include <KrisLibrary/utils/random.h>
#include "math.h"

/** @file math/random.h
 * @ingroup Math
 * @brief Defines a standard method for random floating-point number
 * generation.
 *
 * The random number generator is defined as Math::rng.
 */

/** @addtogroup Math
 * @brief Generates random gaussian distributed floating point values.
 *
 * Uses a given uniform random number generator, generates values
 * in a gaussian with 0 mean and standard deviation 1.
 */
template <class RNG>
float frand_gaussian(RNG& rng);
template <class RNG>
double drand_gaussian(RNG& rng);

namespace Math {
  /** @addtogroup Math */
  /*@{*/

extern StandardRNG rng;
inline void Srand(unsigned long seed) { rng.seed(seed); }
inline long int RandInt() { return rng.randInt(); }
inline long int RandInt(long int n) { return rng.randInt(n); }

#ifdef MATH_DOUBLE
inline Real Rand() { return rng.randDouble(); }
inline Real Rand(Real a,Real b) { return rng.randDouble(a,b); }
inline Real RandGaussian() { return drand_gaussian(rng); }
inline Real RandGaussian(Real mean, Real stddev) { return RandGaussian()*stddev+mean; }
#else
inline Real Rand() { return rng.randFloat(); }
inline Real Rand(Real a,Real b) { return rng.randFloat(a,b); }
inline Real RandGaussian() { return frand_gaussian(rng); }
inline Real RandGaussian(Real mean, Real stddev) { return RandGaussian()*stddev+mean; }
#endif //MATH_DOUBLE

inline bool RandBool() { return rng.randInt() < (rng.maxValue()/2); }
inline bool RandBool(Real p) { return Rand() < p; }

/** @fn Srand()
 * \brief Seeds the Math random number generator.
 */

/** @fn Rand() 
 * @brief Generates a random Real uniformly in [0,1].
 */

/** @fn Rand(Real,Real)
 * @brief Generates a random Real uniformly in [a,b].
 */

/** @fn RandInt() 
 * @brief Generates a random int.
 */

/** @fn RandInt(int) 
 * @brief Generates a random int in [0,n).
 */

/** @fn RandGaussian(Real,Real) 
 * @brief Generates a random Real in a Gaussian distribution.
 */

/** @fn RandBool() 
 * @brief Generates a random boolean that is true with probability 0.5
 */

/** @fn RandBool(Real) 
 * @brief Generates a random boolean that is true with probability p
 */

/*@}*/
} //namespace Math



//definition of rand_gaussian functions

template <class RNG>
float frand_gaussian(RNG& rng)
{
  static float t = 0.0f;
  float x,v1,v2,r;
  if (t == 0) {
    do {
      v1 = 2.0f * rng.randFloat() - 1.0f;
      v2 = 2.0f * rng.randFloat() - 1.0f;
      r = v1 * v1 + v2 * v2;
    } while (r>=1.0f);
    r = sqrtf((-2.0f*logf(r))/r);
    t = v2*r;
    return (v1*r);
  }
  else {
    x = t;
    t = 0.0f;
    return (x);
  }
}

template <class RNG>
double drand_gaussian(RNG& rng)
{
  static double t = 0.0;
  double x,v1,v2,r;
  if (t == 0) {
    do {
      v1 = 2.0 * rng.randDouble() - 1.0;
      v2 = 2.0 * rng.randDouble() - 1.0;
      r = v1 * v1 + v2 * v2;
    } while (r>=1.0);
    r = sqrt((-2.0*log(r))/r);
    t = v2*r;
    return (v1*r);
  }
  else {
    x = t;
    t = 0.0;
    return (x);
  }
}

#endif

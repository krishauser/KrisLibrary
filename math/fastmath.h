#ifndef MATH_FASTMATH_H
#define MATH_FASTMATH_H

/** @file fastmath.h
 * @ingroup Math
 * @brief Miscellaneous lightweight math routines.
 */

unsigned int fastlog2(unsigned int v)
{
  const unsigned int b[] = {0x2, 0xC, 0xF0, 0xFF00, 0xFFFF0000};
  const unsigned int S[] = {1, 2, 4, 8, 16};
  int i;

  register unsigned int r = 0; // result of log2(v) will go here
  for (i = 4; i >= 0; i--) { // unroll for speed...
    if (v & b[i]) {
      v >>= S[i];
      r |= S[i];
    } 
  }
  return r;
}

#endif

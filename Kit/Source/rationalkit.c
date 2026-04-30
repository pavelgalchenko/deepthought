/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "rationalkit.h"

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

/**********************************************************************/
static signed long _gcd(long a, long b)
{
   if (a == 0)
      return b;
   if (b == 0)
      return a;
   a = labs(a);
   b = labs(b);
   while (b) {
      long t = b;
      b      = a % b;
      a      = t;
   }
   return a;
}
/**********************************************************************/
static void _reduce_by_gcd(long *const a, long *const b)
{
   signed long gcd = _gcd(*a, *b);
   *a              = *a / gcd;
   *b              = *b / gcd;
}
/**********************************************************************/
/*  Multiply integer by rational, returning integer whole part and    */
/*  Rational fractional part                                          */
Rational IntegerRationalMult(const long mul, const Rational rat)
{
   Rational out;
   __int128_t product = (__int128_t)(mul * rat.p);
   out.whole          = (long)(product / rat.q) + (rat.whole * mul);
   out.p              = (long)(product % rat.q);
   out.q              = rat.q;
   _reduce_by_gcd(&out.p, &out.q);
   return out;
}
/**********************************************************************/
/*  Compute a / b where a and b are both Rationals                    */
Rational RationalDivide(const Rational a, const Rational b)
{
   Rational out;
   long num_a = a.whole * a.q + a.p;
   long den_a = a.q;
   long num_b = b.whole * b.q + b.p;
   long den_b = b.q;
   _reduce_by_gcd(&num_a, &num_b);
   _reduce_by_gcd(&den_a, &den_b);
   long num = num_a * den_b;
   long den = den_a * num_b;

   if (den < 0) {
      num = -num;
      den = -den;
   }
   _reduce_by_gcd(&num, &den);
   out.whole = num / den;
   out.p     = num % den;
   out.q     = den;

   return out;
}
/**********************************************************************/
/*  Convert the double precision value 'val' to an exact rational p/q */
Rational double2rational(const double val)
{
   Rational rat;
   unsigned long bits;
   memcpy(&bits, &val, sizeof(bits));

   // very slight risk of this not being portable if doubles are different on
   // different architectures
   signed long mantissa =
       (bits & 0x000FFFFFFFFFFFFFULL) | 0x0010000000000000ULL;
   int exponent = (int)((bits >> 52) & 0x7FF) - 1023 - 52;

   // val = mantissa * 2^exponent exactly
   if (exponent >= 0) {
      rat.whole = mantissa << exponent; // exact integer, q=1
      rat.q     = 0;
      rat.q     = 1;
   }
   else {
      rat.p     = mantissa;
      rat.q     = (signed long)1 << (-exponent);
      rat.whole = rat.p / rat.q;
      rat.p     = rat.p % rat.q;
      _reduce_by_gcd(&rat.p, &rat.q);
   }
   return rat;
}
/**********************************************************************/
double rational2double(Rational rat)
{
   return ((double)rat.p / rat.q) + rat.whole;
}
/**********************************************************************/
long RationalRoundUp(Rational rat)
{
   if (rat.whole > 0)
      return rat.whole + 1;
   else
      return rat.whole;
}
/**********************************************************************/
long RationalRoundDown(Rational rat)
{
   if (rat.whole > 0)
      return rat.whole;
   else
      return rat.whole - 1;
}

/* #ifdef __cplusplus
** }
** #endif
*/
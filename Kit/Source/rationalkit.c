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

#define STR2(x) #x
#define STR(X)  STR2(X)

// Use 'long long int' if its larger than 'long int' or if we have nothing
// better, otherwise use int128 if we have it
#if (__SIZEOF_LONG__ < __SIZEOF_LONG_LONG__)
typedef long long int longlong;
#define _SIZEOF_LONGLONG_ __SIZEOF_LONG_LONG__
#elif __SIZEOF_LONG__ == 8
typedef __int128_t longlong;
#define _SIZEOF_LONGLONG_ __SIZEOF_INT128__
#elif
typedef __INT64_TYPE__ longlong
#define _SIZEOF_LONGLONG_ 8
#endif

/**********************************************************************/
static void _positive_denom(Rational *const rat)
{
   rat->num = (rat->den > 0) ? rat->num : -rat->num;
   rat->den = (rat->den > 0) ? rat->den : -rat->den;
}
/**********************************************************************/
/*  Find Greatest Common Divisor of two integers by Eulcidean         */
/*  Algorithm                                                         */
static signed long _gcd(long a, long b)
{
   if (a == 0L)
      return b;
   if (b == 0L)
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
/*  Find Greatest Common Divisor of two integers by Eulcidean         */
/*  Algorithm for 128 bit integers                                    */
static longlong _llgcd(longlong a, longlong b)
{
   if (a == 0LL)
      return b;
   if (b == 0LL)
      return a;
   a = llabs(a);
   b = llabs(b);
   while (b) {
      longlong t = b;
      b          = a % b;
      a          = t;
   }
   return a;
}
/**********************************************************************/
/*  Find the Least Common Multiple of two integers by the identity    */
/*  lcm = (a * b) / gcd(a, b)                                         */
static signed long _lcm(long a, long b)
{
   const long gcd = _gcd(a, b);
   if (gcd) {
      if (b > a)
         b /= gcd;
      else
         a /= gcd;
      return (a * b);
   }
   return 0;
}
/**********************************************************************/
/*  Reduce two longs by their greatest common divisor                 */
static void _reduce_by_gcd(long *const a, long *const b)
{
   const signed long gcd = _gcd(*a, *b);
   if (gcd >= 0) {
      *a = *a / gcd;
      *b = *b / gcd;
   }
}
/**********************************************************************/
/*  Reduce two longs by their greatest common divisor                 */
static void _llreduce_by_gcd(longlong *const a, longlong *const b)
{
   const longlong gcd = _llgcd(*a, *b);
   if (gcd >= 0LL) {
      *a = *a / gcd;
      *b = *b / gcd;
   }
}
/**********************************************************************/
static void _cleanup(Rational *const rat)
{
   _reduce_by_gcd(&rat->num, &rat->den);
   _positive_denom(rat);
   if (labs(rat->num) > rat->den) {
      rat->whole += rat->num / rat->den;
      rat->num    = rat->num % rat->den;
   }
}
/**********************************************************************/
/*  Multiply integer by rational, returning integer whole part and    */
/*  Rational fractional part                                          */
Rational IntegerRationalMult(const long mul, const Rational rat)
{
#ifndef _IGNORE_LONG_
   _Static_assert(
       sizeof(long) < sizeof(longlong),
       "Size of 'long' is " STR(
           __SIZEOF_LONG__) ", which is not less than the size of 'long "
                            "long', " STR(_SIZEOF_LONGLONG_) ". To ignore, "
                                                             "reconfigure with "
                                                             "-DIGNORE_LONG="
                                                             "TRUE.");
#endif
   Rational out;
   longlong product = ((longlong)mul * rat.num);
   out.whole        = (long)((product / rat.den) + ((longlong)mul * rat.whole));
   out.num          = (long)(product % rat.den);
   out.den          = rat.den;
   _cleanup(&out);
   return out;
}
/**********************************************************************/
Rational RationalMult(const Rational a, const Rational b)
{
   Rational out;
   longlong num_a = (longlong)a.whole * a.den + a.num;
   longlong den_a = (longlong)a.den;
   longlong num_b = (longlong)b.whole * b.den + b.num;
   longlong den_b = (longlong)b.den;

   // TODO: shouldn't need to do this step if a and b are already reduced
   _llreduce_by_gcd(&num_a, &den_a);
   _llreduce_by_gcd(&num_b, &den_b);

   longlong den           = den_a * den_b;
   const longlong a_gcd   = _llgcd(num_a, out.den);
   const longlong com_gcd = _llgcd(a_gcd, num_b);
   if (com_gcd >= 0) {
      num_a /= com_gcd;
      num_b /= com_gcd;
      den   /= com_gcd;
   }
   longlong product = num_a * num_b;
   out.whole        = (long)(product / den);
   out.num          = (long)(product % den);
   out.den          = (long)den;
   _cleanup(&out);
   return out;
}
/**********************************************************************/
/*  Compute a / b where a and b are both Rationals                    */
Rational RationalDivide(const Rational a, const Rational b)
{
   Rational out   = {0};
   longlong num_a = (longlong)a.whole * a.den + a.num;
   longlong den_a = (longlong)a.den;
   longlong num_b = (longlong)b.whole * b.den + b.num;
   longlong den_b = (longlong)b.den;
   _llreduce_by_gcd(&num_a, &num_b);
   _llreduce_by_gcd(&den_a, &den_b);
   out.num = (long)(num_a * den_b);
   out.den = (long)(den_a * num_b);
   _cleanup(&out);
   return out;
}
/**********************************************************************/
Rational RationalAdd(const Rational a, const Rational b)
{
   Rational out;
   out.whole      = a.whole + b.whole;
   longlong den   = (longlong)a.den * b.den;
   longlong num_a = (longlong)a.num * b.den;
   longlong num_b = (longlong)b.num * a.den;

   const longlong a_gcd   = _llgcd(num_a, den);
   const longlong com_gcd = _llgcd(a_gcd, num_b);
   if (com_gcd >= 0) {
      num_a /= com_gcd;
      num_b /= com_gcd;
      den   /= com_gcd;
   }
   longlong sum  = num_a + num_b;
   out.whole    += (long)(sum / den);
   out.num       = (long)(sum % den);
   out.den       = (long)den;
   _cleanup(&out);
   return out;
}
/**********************************************************************/
Rational RationalSub(const Rational a, const Rational b)
{
   Rational out;
   out.whole      = a.whole - b.whole;
   longlong den   = (longlong)a.den * b.den;
   longlong num_a = (longlong)a.num * b.den;
   longlong num_b = (longlong)b.num * a.den;

   const longlong a_gcd   = _llgcd(num_a, den);
   const longlong com_gcd = _llgcd(a_gcd, num_b);
   if (com_gcd >= 0) {
      num_a /= com_gcd;
      num_b /= com_gcd;
      den   /= com_gcd;
   }
   longlong diff  = num_a - num_b;
   out.whole     += (long)(diff / den);
   out.num        = (long)(diff % den);
   out.den        = (long)den;
   _cleanup(&out);
   return out;
}
/**********************************************************************/
/*  Convert a double precision value to an exact rational             */
/*  whole + num/den                                                   */

// Number of bits in the stored significand (no implicit bit)
#define DBL_FRAC_BITS (__DBL_MANT_DIG__ - 1)

// Masks derived portably
#define DBL_FRAC_MASK ((__UINT64_C(1) << DBL_FRAC_BITS) - 1)
#define DBL_IMPLICIT  (__UINT64_C(1) << DBL_FRAC_BITS)
#define DBL_EXP_BIAS  (__DBL_MAX_EXP__ - 1)
#define DBL_EXP_SHIFT DBL_FRAC_BITS
#define DBL_EXP_MASK  ((__UINT64_C(1) << (64 - DBL_FRAC_BITS - 1)) - 1)

Rational double2rational(const double val)
{
   Rational out;
   union {
      double x;
      __UINT64_TYPE__ bits;
   } u;

   _Static_assert(sizeof(__UINT64_TYPE__) == sizeof(double),
                  "Size of unsigned long not equal to size of double.");

   u.x = val;
   if (u.bits == 0) {
      out.whole = 0;
      out.num   = 0;
      out.den   = 1;
      return out;
   }
   // Extract and remove sign bit before processing
   const int sign  = (u.bits >> 63) & 1;
   u.bits         &= ~(__UINT64_C(1) << 63); // clear sign bit

   signed long mantissa = (u.bits & DBL_FRAC_MASK) | DBL_IMPLICIT;
   int exponent         = (int)((u.bits >> DBL_EXP_SHIFT) & DBL_EXP_MASK) -
                          DBL_EXP_BIAS - DBL_FRAC_BITS;

   // val = mantissa * 2^exponent exactly
   if (exponent >= 0) {
      out.num = mantissa << exponent; // exact integer, den=1
      out.den = 1;
   }
   else {
      out.num = mantissa;
      out.den = __INT64_C(1) << (-exponent);
      _reduce_by_gcd(&out.num, &out.den);
   }
   out.num   = (sign) ? -out.num : out.num;
   out.whole = out.num / out.den;
   out.num   = out.num % out.den;
   _cleanup(&out);
   return out;
}
/**********************************************************************/
double rational2double(const Rational rat)
{
   return ((double)rat.num / rat.den) + (double)rat.whole;
}
/**********************************************************************/
long RationalRoundUp(const Rational rat)
{
   return rat.whole + ((labs(rat.num) > 0 && rat.whole > 0) ? 1 : 0);
}
/**********************************************************************/
long RationalRoundDown(const Rational rat)
{
   return rat.whole - ((labs(rat.num) > 0 && rat.whole < 0) ? 1 : 0);
}
/**********************************************************************/
int isequal_rational(Rational a, Rational b)
{
   _cleanup(&a);
   _cleanup(&b);
   return ((a.whole == b.whole) && (a.num == b.num) && (a.den == b.den));
}
/**********************************************************************/
int isless_rational(Rational a, Rational b)
{
   _cleanup(&a);
   _cleanup(&b);
   const long lcm  = _lcm(a.den, b.den);
   a.num          *= (lcm / a.den);
   b.num          *= (lcm / b.den);
   return (a.whole < b.whole) || ((a.whole == b.whole) && (a.num < b.num));
}
/**********************************************************************/
int isgreater_rational(Rational a, Rational b)
{
   _cleanup(&a);
   _cleanup(&b);
   const long lcm  = _lcm(a.den, b.den);
   a.num          *= (lcm / a.den);
   b.num          *= (lcm / b.den);
   return (a.whole > b.whole) || ((a.whole == b.whole) && (a.num > b.num));
}

/* #ifdef __cplusplus
** }
** #endif
*/
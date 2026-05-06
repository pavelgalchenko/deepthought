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

// TODO: ensure den <= __LONG_MAX__
//   plan: if den gets too large, find closest representable rational

// TODO: do wrappers for defines to be portable between, e.g., gcc and clang

#define STR2(x) #x
#define STR(X)  STR2(X)

// Use 'long long int' if its larger than 'long int'. If not use int128 if we
// have it
#ifdef __SIZEOF_INT128__
typedef __int128_t Rat_LongLong;
#define _SIZEOF_LONGLONG_ (__SIZEOF_INT128__)
static unsigned int _ctzll(__uint128_t v)
{
   if (!v)
      return _SIZEOF_LONGLONG_ * __CHAR_BIT__;
   // do binary search to find the first set bit
   //    From Stanford's Bit Twiddling Hacks
   //    https://graphics.stanford.edu/%7Eseander/bithacks.html#ZerosOnRightParallel
   unsigned int c;
   if (v & 0x1) {
      c = 0;
   }
   else {
      c = 1;
      if ((v & 0xffffffffffffffff) == 0) {
         v >>= 64;
         c  += 64;
      }
      if ((v & 0xffffffff) == 0) {
         v >>= 32;
         c  += 32;
      }
      if ((v & 0xffff) == 0) {
         v >>= 16;
         c  += 16;
      }
      if ((v & 0xff) == 0) {
         v >>= 8;
         c  += 8;
      }
      if ((v & 0xf) == 0) {
         v >>= 4;
         c  += 4;
      }
      if ((v & 0x3) == 0) {
         v >>= 2;
         c  += 2;
      }
      c -= v & 0x1;
   }

   return c;
}

static __uint128_t _absll(Rat_LongLong x)
{
   if (x >= 0)
      return x;
   return -x;
}
#elif (__SIZEOF_LONG_LONG__ > __SIZEOF_RATLONG__)
typedef signed long long int Rat_LongLong;
#define _SIZEOF_RATLONGLONG_ (__SIZEOF_LONG_LONG__)
#define _ctzll               (__builtin_ctzll)
#define _absll(x)            (llabs(x))
#else
_Static_assert(
    0, "Configuration does not support rationalkit. Two different sizes of "
       "integer types are required, preferrably 64-bit/128-bit.");
#endif

#if (__SIZEOF_DOUBLE__ == __SIZEOF_LONG_LONG__)
typedef unsigned long long int Rat_Dbl_Cmp;
#elif (__SIZEOF_DOUBLE__ == __SIZEOF_LONG__)
typedef unsigned long int Rat_Dbl_Cmp;
#elif (__SIZEOF_DOUBLE__ == __SIZEOF_INT__)
typedef unsigned int Rat_Dbl_Cmp;
#else
_Static_assert(0, "Configuration does not support rationalkit. Unable to find "
                  "an integer type the same size as double.");
#endif

#ifndef MAX
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif

/**********************************************************************/
static void _positive_denom(Rational *const rat)
{
   rat->num = (rat->den > 0) ? rat->num : -rat->num;
   rat->den = (rat->den > 0) ? rat->den : -rat->den;
}
/**********************************************************************/
static Rat_LongLong _gcdll(Rat_LongLong a, Rat_LongLong b)
{
   if (a == 0L)
      return b;
   if (b == 0L)
      return a;
   a = _absll(a);
   b = _absll(b);

   // TODO: __builtin_ctzl is a gcc builtin for determining the number of
   // trailing zeros in an unsigned integer type. Will need to do our own
   // defines/wrappers to get compatibility with other compilers
   const int shift   = _ctzll(a | b);
   a               >>= _ctzll(a);
   do {
      b >>= _ctzll(b);
      if (a > b) {
         Rat_LongLong t = a;
         a              = b;
         b              = t;
      }
   } while (b -= a);
   return a << shift;
}
/**********************************************************************/
/*  From Stack Overflow user Maxim Egorushkin                         */
/*  Computes the Greatest Common Divisior of two longs using the      */
/*  Binary GCD algorithm, with speedups from builtins                 */
static Rat_Long _gcdl(Rat_Long a, Rat_Long b)
{
   if (a == 0L)
      return b;
   if (b == 0L)
      return a;
   a = labs(a);
   b = labs(b);

   // TODO: __builtin_ctzl is a gcc builtin for determining the number of
   // trailing zeros in an unsigned integer type. Will need to do our own
   // defines/wrappers to get compatibility with other compilers
   const int shift   = __builtin_ctzl(a | b);
   a               >>= __builtin_ctzl(a);
   do {
      b >>= __builtin_ctzl(b);
      if (a > b) {
         // swap the values without needing memory for a third
         a = a ^ b;
         b = a ^ b;
         a = b ^ a;
      }
   } while (b -= a);
   return a << shift;
}
/**********************************************************************/
/*  Find the Least Common Multiple of two integers by the identity    */
/*  lcm = (a * b) / gcd(a, b)                                         */
static Rat_Long _lcml(Rat_Long a, Rat_Long b)
{
   const Rat_Long gcd = _gcdl(a, b);
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
static void _reduce_by_gcd(Rat_Long *const a, Rat_Long *const b)
{
   const Rat_Long gcd = _gcdl(*a, *b);
   if (gcd > 0) {
      *a = *a / gcd;
      *b = *b / gcd;
   }
}
/**********************************************************************/
/*  Reduce two longs by their greatest common divisor                 */
static void _reduce_by_gcdll(Rat_LongLong *const a, Rat_LongLong *const b)
{
   const Rat_LongLong gcd = _gcdll(*a, *b);
   if (gcd > 0) {
      *a = *a / gcd;
      *b = *b / gcd;
   }
}
/**********************************************************************/
static void _validate(Rational *const rat)
{
   if (rat->den == 0)
      rat->den = 1;
}
/**********************************************************************/
static void _cleanup(Rational *const rat)
{
   _validate(rat);
   _reduce_by_gcd(&rat->num, &rat->den);
   _positive_denom(rat);
   if (labs(rat->num) > rat->den) {
      rat->whole += rat->num / rat->den;
      rat->num   %= rat->den;
   }
}
/**********************************************************************/
/*  Multiply integer by rational, returning integer whole part and    */
/*  Rational fractional part                                          */
Rational IntegerRationalMult(const Rat_Long mul, Rational rat)
{
#ifndef _IGNORE_LONG_
   _Static_assert(
       sizeof(Rat_Long) < sizeof(Rat_LongLong),
       "Size of 'long' is " STR(
           __SIZEOF_LONG__) ", which is not less than the size of 'long "
                            "long', " STR(
                                _SIZEOF_RATLONGLONG_) ". To ignore, "
                                                      "reconfigure with "
                                                      "-DIGNORE_LONG="
                                                      "TRUE.");
#endif
   _validate(&rat);

   Rational out;
   Rat_LongLong product = ((Rat_LongLong)mul * rat.num);
   out.whole =
       (Rat_Long)((product / rat.den) + ((Rat_LongLong)mul * rat.whole));
   out.num = (Rat_Long)(product % rat.den);
   out.den = rat.den;
   _cleanup(&out);
   return out;
}
/**********************************************************************/
/*  Multiply integer by rational, returning integer whole part and    */
/*  Rational fractional part                                          */
/*  This version sets:  '*carry = out.whole / mod'                    */
/*                and:  'out.whole %= mod'                            */
Rational IntegerRationalMultMod(const Rat_Long mul, Rational rat, Rat_Long mod,
                                Rat_Long *const carry)
{
   _validate(&rat);
   if (!mod)
      mod = 1;

   Rational out;
   Rat_LongLong product  = ((Rat_LongLong)mul * rat.num);
   Rat_LongLong wholea   = product / rat.den;
   Rat_LongLong wholeb   = (Rat_LongLong)mul * rat.whole;
   *carry                = (Rat_Long)((wholea / mod) + (wholeb / mod));
   out.whole             = (Rat_Long)((wholea % mod) + (wholeb % mod));
   *carry               += out.whole / mod;
   out.whole            %= mod;
   out.num               = (Rat_Long)(product % rat.den);
   out.den               = rat.den;
   _cleanup(&out);
   return out;
}
/**********************************************************************/
Rational RationalMult(Rational a, Rational b)
{
   _validate(&a);
   _validate(&b);

   Rational out;
   Rat_LongLong num_a = (Rat_LongLong)a.whole * a.den + a.num;
   Rat_LongLong den_a = (Rat_LongLong)a.den;
   Rat_LongLong num_b = (Rat_LongLong)b.whole * b.den + b.num;
   Rat_LongLong den_b = (Rat_LongLong)b.den;

   // TODO: shouldn't need to do this step if a and b are already reduced
   _reduce_by_gcdll(&num_a, &den_a);
   _reduce_by_gcdll(&num_b, &den_b);

   Rat_LongLong den           = den_a * den_b;
   const Rat_LongLong a_gcd   = _gcdll(num_a, den);
   const Rat_LongLong com_gcd = _gcdll(a_gcd, num_b);
   if (com_gcd > 0) {
      num_a /= com_gcd;
      num_b /= com_gcd;
      den   /= com_gcd;
   }
   Rat_LongLong product = num_a * num_b;
   out.whole            = (Rat_Long)(product / den);
   out.num              = (Rat_Long)(product % den);
   out.den              = (Rat_Long)den;
   _cleanup(&out);
   return out;
}
/**********************************************************************/
/*  Compute a / b where a and b are both Rationals                    */
Rational RationalDivide(Rational a, Rational b)
{
   _validate(&a);
   _validate(&b);

   Rational out       = {0};
   Rat_LongLong num_a = (Rat_LongLong)a.whole * a.den + a.num;
   Rat_LongLong den_a = (Rat_LongLong)a.den;
   Rat_LongLong num_b = (Rat_LongLong)b.whole * b.den + b.num;
   Rat_LongLong den_b = (Rat_LongLong)b.den;
   _reduce_by_gcdll(&num_a, &num_b);
   _reduce_by_gcdll(&den_a, &den_b);
   out.num = (Rat_Long)(num_a * den_b);
   out.den = (Rat_Long)(den_a * num_b);
   _cleanup(&out);
   return out;
}
/**********************************************************************/
Rational RationalAdd(Rational a, Rational b)
{
   _validate(&a);
   _validate(&b);

   Rational out;
   out.whole          = a.whole + b.whole;
   Rat_LongLong den   = (Rat_LongLong)a.den * b.den;
   Rat_LongLong num_a = (Rat_LongLong)a.num * b.den;
   Rat_LongLong num_b = (Rat_LongLong)b.num * a.den;

   const Rat_LongLong a_gcd   = _gcdll(num_a, den);
   const Rat_LongLong com_gcd = _gcdll(a_gcd, num_b);
   if (com_gcd > 0) {
      num_a /= com_gcd;
      num_b /= com_gcd;
      den   /= com_gcd;
   }
   Rat_LongLong sum  = num_a + num_b;
   out.whole        += (Rat_Long)(sum / den);
   out.num           = (Rat_Long)(sum % den);
   out.den           = (Rat_Long)den;
   _cleanup(&out);
   return out;
}
/**********************************************************************/
Rational RationalSub(Rational a, Rational b)
{
   _validate(&a);
   _validate(&b);

   Rational out;
   out.whole          = a.whole - b.whole;
   Rat_LongLong den   = (Rat_LongLong)a.den * b.den;
   Rat_LongLong num_a = (Rat_LongLong)a.num * b.den;
   Rat_LongLong num_b = (Rat_LongLong)b.num * a.den;

   const Rat_LongLong a_gcd   = _gcdll(num_a, den);
   const Rat_LongLong com_gcd = _gcdll(a_gcd, num_b);
   if (_absll(com_gcd) > 0) {
      num_a /= com_gcd;
      num_b /= com_gcd;
      den   /= com_gcd;
   }
   Rat_LongLong diff  = num_a - num_b;
   out.whole         += (Rat_Long)(diff / den);
   out.num            = (Rat_Long)(diff % den);
   out.den            = (Rat_Long)den;
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
   Rational out = {.whole = 0, .num = 0, .den = 1};
   union {
      double x;
      Rat_Dbl_Cmp bits;
   } u;

   u.x = val;

   // grab sign bit (need to do it this way due to -0)
   const int sign = (u.bits >> 63) & 1;
   u.x            = (sign) ? -u.x : u.x; // make u.x positive
   if (u.bits == 0) {
      out.whole = 0;
      out.num   = 0;
      out.den   = 1;
      return out;
   }
   Rat_Long mantissa = (u.bits & DBL_FRAC_MASK) | DBL_IMPLICIT;
   int exponent      = (int)((u.bits >> DBL_EXP_SHIFT) & DBL_EXP_MASK) -
                       DBL_EXP_BIAS - DBL_FRAC_BITS;

   // TODO: this needs testing, especially with small numbers
   if (exponent >= 0) {
      // val = mantissa * 2^exponent exactly
      const int max_shift = 63 - __DBL_MANT_DIG__; // = 10 for IEEE 754 double
      if (exponent > max_shift) {
         // exact value overflows int64_t; shift down (low bits are zero for
         // exact ints)
         out.num = mantissa << max_shift;
         out.den = 1;
      }
      else {
         out.num = mantissa << exponent; // exact integer, den=1
         out.den = 1;
      }
   }
   else if (-exponent <= 62) {
      out.num = mantissa;
      out.den = __INT64_C(1) << (-exponent);
      _reduce_by_gcd(&out.num, &out.den);
   }
   else {
      // -exponent > 62: num would overflow
      const int excess = (-exponent) - 62;
      if (excess >= __DBL_MANT_DIG__) {
         // Value too small to represent; round to zero
         out.num = 0;
         out.den = 1;
      }
      else {
         // Best approximation: scale both down to fit in int64_t
         out.num = mantissa >> excess;
         out.den = __INT64_C(1) << 62;
      }
   }
   out.num   = (sign) ? -out.num : out.num;
   out.whole = out.num / out.den;
   out.num   = out.num % out.den;
   _cleanup(&out);
   return out;
}
/**********************************************************************/
double rational2double(Rational rat)
{
   _validate(&rat);
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
   const Rat_Long lcm  = _lcml(a.den, b.den);
   a.num              *= (lcm / a.den);
   b.num              *= (lcm / b.den);
   return (a.whole < b.whole) || ((a.whole == b.whole) && (a.num < b.num));
}
/**********************************************************************/
int isgreater_rational(Rational a, Rational b)
{
   _cleanup(&a);
   _cleanup(&b);
   const Rat_Long lcm  = _lcml(a.den, b.den);
   a.num              *= (lcm / a.den);
   b.num              *= (lcm / b.den);
   return (a.whole > b.whole) || ((a.whole == b.whole) && (a.num > b.num));
}

/* #ifdef __cplusplus
** }
** #endif
*/
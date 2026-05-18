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

#define STR2(x) #x
#define STR(X)  STR2(X)

// Use 'long long int' if its larger than 'long int'. If not use int128 if we
// have it
#if (__SIZEOF_LONG_LONG__ == _SIZEOF_RATLONGLONG_)
typedef unsigned long long int Rat_ULongLong;
#define _absll (llabs)
#elif (__SIZEOF_INT128__ == _SIZEOF_RATLONGLONG_)
typedef __uint128_t Rat_ULongLong;
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

#define CONCAT_PRIMATIVE(a, b) a##b
#define CONCAT_EXPAND(a, b)    CONCAT_PRIMATIVE(a, b)
#if defined(__INT64_C)
#define INT64_MACRO __INT64_C
#elif defined(__INT64_C_SUFFIX__)
#define INT64_MACRO(c) CONCAT_EXPAND(c, __INT64_C_SUFFIX__)
#else
#define INT64_MACRO(c) CONCAT_EXPAND(c, L)
#endif

#if defined(__UINT64_C)
#define UINT64_MACRO __UINT64_C
#elif defined(__UINT64_C_SUFFIX__)
#define UINT64_MACRO(c) CONCAT_EXPAND(c, __UINT64_C_SUFFIX__)
#else
#define UINT64_MACRO(c) CONCAT_EXPAND(c, UL)
#endif

#ifdef __has_builtin
#if __has_builtin(__builtin_ctzl)
#define _ctzl (__builtin_ctzl)
#endif
#if (_SIZEOF_RATLONGLONG_) == (__SIZEOF_LONG_LONG__) &&                        \
    __has_builtin(__builtin_ctzll)
#define _ctzll (__builtin_ctzll)
#endif
#endif

#ifndef _ctzl
static unsigned int _ctzl(Rat_ULong v)
{
   if (!v)
      return _SIZEOF_LONG_ * __CHAR_BIT__;
   // do binary search to find the first set bit
   //    From Stanford's Bit Twiddling Hacks
   //    https://graphics.stanford.edu/%7Eseander/bithacks.html#ZerosOnRightParallel
   unsigned int c;
   if (v & 0x1) {
      c = 0;
   }
   else {
      c = 1;
#if _SIZEOF_RATLONG_ > 4
      if ((v & 0xffffffff) == 0) {
         v >>= 32;
         c  += 32;
      }
#endif
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
#endif

#ifndef _ctzll
static unsigned int _ctzll(Rat_ULongLong v)
{
   if (!v)
      return _SIZEOF_RATLONGLONG_ * __CHAR_BIT__;
   // do binary search to find the first set bit
   //    From Stanford's Bit Twiddling Hacks
   //    https://graphics.stanford.edu/%7Eseander/bithacks.html#ZerosOnRightParallel
   unsigned int c;
   if (v & 0x1) {
      c = 0;
   }
   else {
      c = 1;
#if _SIZEOF_RATLONGLONG_ > 8
      if ((v & 0xffffffffffffffff) == 0) {
         v >>= 64;
         c  += 64;
      }
#endif
#if _SIZEOF_RATLONGLONG_ > 4
      if ((v & 0xffffffff) == 0) {
         v >>= 32;
         c  += 32;
      }
#endif
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
#endif

#ifndef _absll
static Rat_ULongLong _absll(Rat_LongLong x)
{
   if (x >= 0)
      return x;
   return -x;
}
#endif

#if _SIZEOF_RATLONG_ != __SIZEOF_LONG__
static Rat_ULong _absl(Rat_Long x)
{
   if (x >= 0)
      return x;
   return -x;
}
#else
#define _absl (labs)
#endif

#define RATIONALLL_RAW(whl, n, d)                                              \
   ((RationalLL){.whole = (whl), .num = (n), .den = (d)})
#define RATIONALLL_NGCD(whl, n, d)                                             \
   RATIONALLL_RAW((whl) + (n) / (d), SIGN(d) * (n) % (d), MAX(1, ABS(d)))

/**********************************************************************/
static RationalLL _rationalll_negate(RationalLL rat)
{
   rat.whole = -rat.whole;
   rat.num   = -rat.num;
   return rat;
}
/**********************************************************************/
static void _positive_denom(Rational *const rat)
{
   if (rat->den < 0) {
      rat->num = -rat->num;
      rat->den = -rat->den;
   }
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
   a = _absl(a);
   b = _absl(b);

   const int shift   = _ctzl(a | b);
   a               >>= _ctzl(a);
   do {
      b >>= _ctzl(b);
      if (a > b) {
         Rat_Long t = a;
         a          = b;
         b          = t;
      }
   } while (b -= a);
   return a << shift;
}
/**********************************************************************/
/*  Reduce two longs by their greatest common divisor                 */
static void _reduce_by_gcdl(Rat_Long *const a, Rat_Long *const b)
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
static void _reduce(Rational *const rat)
{
   _positive_denom(rat);
   _validate(rat);
   rat->whole += rat->num / rat->den;
   rat->num   %= rat->den;
   if ((rat->whole ^ rat->num) < 0 && rat->whole != 0 && rat->num != 0) {
      if (rat->whole > 0) {
         rat->num += rat->den;
         rat->whole--;
      }
      else if (rat->whole < 0) {
         rat->num -= rat->den;
         rat->whole++;
      }
   }
   if (rat->num == 0)
      rat->den = 1;
}
/**********************************************************************/
static void _cleanup(Rational *const rat)
{
   _reduce(rat);
   _reduce_by_gcdl(&rat->num, &rat->den);
}
/**********************************************************************/
static void _positive_denomll(RationalLL *const rat)
{
   if (rat->den < 0) {
      rat->num = -rat->num;
      rat->den = -rat->den;
   }
}
/**********************************************************************/
static void _validatell(RationalLL *const rat)
{
   if (rat->den == 0)
      rat->den = 1;
}
/**********************************************************************/
static void _reducell(RationalLL *const rat)
{
   _positive_denomll(rat);
   _validatell(rat);
   rat->whole += rat->num / rat->den;
   rat->num   %= rat->den;
   if ((rat->whole ^ rat->num) < 0 && rat->whole != 0 && rat->num != 0) {
      if (rat->whole > 0) {
         rat->num += rat->den;
         rat->whole--;
      }
      else if (rat->whole < 0) {
         rat->num -= rat->den;
         rat->whole++;
      }
   }
   if (rat->num == 0)
      rat->den = 1;
}
/**********************************************************************/
static void _cleanupll(RationalLL *const rat)
{
   _reducell(rat);
   _reduce_by_gcdll(&rat->num, &rat->den);
}
/**********************************************************************/
static Rat_LongLong _rounddown_div(const Rat_LongLong a, const Rat_LongLong b)
{
   Rat_LongLong c = a / b;
   if (c <= 0 && a % b < 0)
      c--;
   return c;
}
/**********************************************************************/
/*  Limit the size of the denomiator of the fraction p/q. Checks if   */
/*  solution is the convergent or the semiconvergent of the below     */
/*  process.                                                          */
/*  Taken from python's fraction.Fraction.limit_denominator().        */
#define RAT_MAX_DEN (_RATLONG_MAX_ >> 8)
Rational _limit_denominator(Rat_LongLong p, Rat_LongLong q)
{
   const Rat_Long max_den = RAT_MAX_DEN;
   if (p == 0)
      return RATIONAL_ZERO;

   const int sign_q = SIGN(q);

   p *= sign_q;
   q *= sign_q;

   if (q <= max_den)
      return RATIONAL_NGCD(p / q, p % q, q);

   Rat_LongLong p0 = 0, q0 = 1, p1 = 1, q1 = 0;
   Rat_LongLong n = p, d = q;

   _reduce_by_gcdll(&n, &d);
   if (d <= max_den)
      return RATIONAL_NGCD(n / d, n % d, d);

   Rat_LongLong t1 = 0, t2 = 0;
   while (1) {
      Rat_LongLong a  = _rounddown_div(n, d);
      Rat_LongLong q2 = q0 + a * q1;
      if (q2 > max_den)
         break;
      t1 = p0;
      p0 = p1;
      p1 = t1 + a * p1;
      q0 = q1;
      q1 = q2;

      t1 = n;
      n  = d;
      d  = t1 - a * d;
   }
   Rat_LongLong k = _rounddown_div(max_den - q0, q1);

   t1 = 2 * d * (q0 + k * q1);
   if (t1 <= q)
      return RATIONAL_NGCD(p1 / q1, p1 % q1, q1);
   else {
      t1 = p0 + k * p1;
      t2 = q0 + k * q1;
      return RATIONAL_NGCD(t1 / t2, t1 % t2, t2);
   }
}
/**********************************************************************/
/*  Limit the size of the denomiator of the fraction p/q. Checks if   */
/*  solution is the convergent or the semiconvergent of the below     */
/*  process.                                                          */
/*  Taken from python's fraction.Fraction.limit_denominator().        */
RationalLL _limit_denominator_ll(Rat_LongLong p, Rat_LongLong q)
{
   const Rat_Long max_den = RAT_MAX_DEN;
   if (p == 0)
      return RATIONALLL_NGCD(0, 0, 1);

   const int sign_q = SIGN(q);

   p *= sign_q;
   q *= sign_q;

   if (q <= max_den)
      return RATIONALLL_NGCD(p / q, p % q, q);

   Rat_LongLong p0 = 0, q0 = 1, p1 = 1, q1 = 0;
   Rat_LongLong n = p, d = q;

   _reduce_by_gcdll(&n, &d);
   if (d <= max_den)
      return RATIONALLL_NGCD(n / d, n % d, d);

   Rat_LongLong t1 = 0, t2 = 0;
   while (1) {
      Rat_LongLong a  = _rounddown_div(n, d);
      Rat_LongLong q2 = q0 + a * q1;
      if (q2 > max_den)
         break;
      t1 = p0;
      p0 = p1;
      p1 = t1 + a * p1;
      q0 = q1;
      q1 = q2;

      t1 = n;
      n  = d;
      d  = t1 - a * d;
   }
   Rat_LongLong k = _rounddown_div(max_den - q0, q1);

   t1 = 2 * d * (q0 + k * q1);
   if (t1 <= q)
      return RATIONALLL_NGCD(p1 / q1, p1 % q1, q1);
   else {
      t1 = p0 + k * p1;
      t2 = q0 + k * q1;
      return RATIONALLL_NGCD(t1 / t2, t1 % t2, t2);
   }
}
/**********************************************************************/
Rational InitRational(const Rat_Long whole, const Rat_Long num,
                      const Rat_Long den)
{
   Rational rat = RATIONAL_NGCD(whole, num, den);
   _cleanup(&rat);
   rat        = _limit_denominator(rat.num, rat.den);
   rat.whole += whole;
   return rat;
}
/**********************************************************************/
Rat_Long RationalIntMod(Rational *const rat, const Rat_Long mod)
{
   Rat_Long old_whole  = rat->whole;
   rat->whole         %= mod;
   return old_whole / mod;
}
/**********************************************************************/
void ReduceRational(Rational *const rat)
{
   _cleanup(rat);
}
/**********************************************************************/
/*  Multiply integer by rational, returning integer whole part and    */
/*  Rational fractional part                                          */
Rational IntegerRationalMult(const Rat_Long mul, RationalLL rat)
{
   _reducell(&rat);

   RationalLL out_ll;
   Rat_LongLong product = ((Rat_LongLong)mul * rat.num);

   out_ll.whole = (product / rat.den) + ((Rat_LongLong)mul * rat.whole);
   out_ll.num   = product % rat.den;
   out_ll.den   = rat.den;
   _cleanupll(&out_ll);
   return ToRational(out_ll);
}
/**********************************************************************/
/*  Multiply integer by rational, returning integer whole part and    */
/*  Rational fractional part                                          */
/*  This version sets:  '*carry = out.whole / mod'                    */
/*                and:  'out.whole %= mod'                            */
Rational IntegerRationalMultMod(const Rat_Long mul, RationalLL rat,
                                Rat_Long mod, Rat_Long *const carry)
{
   _reducell(&rat);
   if (!mod)
      mod = 1;

   RationalLL out_ll;
   Rat_LongLong product  = ((Rat_LongLong)mul * rat.num);
   Rat_LongLong wholea   = product / rat.den;
   Rat_LongLong wholeb   = (Rat_LongLong)mul * rat.whole;
   *carry                = (wholea / mod) + (wholeb / mod);
   out_ll.whole          = (wholea % mod) + (wholeb % mod);
   *carry               += out_ll.whole / mod;
   out_ll.whole         %= mod;
   out_ll.num            = product % rat.den;
   out_ll.den            = rat.den;
   _cleanupll(&out_ll);
   return ToRational(out_ll);
}
/**********************************************************************/
RationalLL RationalMult(RationalLL a, RationalLL b)
{
   _reducell(&a);
   _reducell(&b);

   _reduce_by_gcdll(&a.num, &a.den);
   _reduce_by_gcdll(&b.num, &b.den);

   RationalLL out;
   Rat_LongLong num_a = a.whole * a.den + a.num;
   Rat_LongLong den_a = a.den;
   Rat_LongLong num_b = b.whole * b.den + b.num;
   Rat_LongLong den_b = b.den;

   // TODO: shouldn't need to do this step if a and b are already reduced
   _reduce_by_gcdll(&num_a, &den_b);
   _reduce_by_gcdll(&num_b, &den_a);

   Rat_LongLong den           = den_a * den_b;
   const Rat_LongLong a_gcd   = _gcdll(num_a, den);
   const Rat_LongLong com_gcd = _gcdll(a_gcd, num_b);
   if (com_gcd > 0) {
      num_a /= com_gcd;
      num_b /= com_gcd;
      den   /= com_gcd;
   }
   out = RATIONALLL_NGCD(0, num_a * num_b, den);
   _cleanupll(&out);
   return out;
}
/**********************************************************************/
/*  Compute a / b where a and b are both Rationals                    */
RationalLL RationalDivide(RationalLL a, RationalLL b)
{
   _reducell(&a);
   _reducell(&b);

   _reduce_by_gcdll(&a.num, &a.den);
   _reduce_by_gcdll(&b.num, &b.den);

   RationalLL out     = {0};
   Rat_LongLong num_a = a.whole * a.den + a.num;
   Rat_LongLong den_a = a.den;
   Rat_LongLong num_b = b.whole * b.den + b.num;
   Rat_LongLong den_b = b.den;
   _reduce_by_gcdll(&num_a, &num_b);
   _reduce_by_gcdll(&den_a, &den_b);
   out = RATIONALLL_NGCD(0, num_a * den_b, den_a * num_b);
   _cleanupll(&out);
   return out;
}
/**********************************************************************/
RationalLL RationalAdd(RationalLL a, RationalLL b)
{
   _reducell(&a);
   _reducell(&b);

   _reduce_by_gcdll(&a.num, &a.den);
   _reduce_by_gcdll(&b.num, &b.den);

   if (a.whole == 0 && a.num == 0)
      return b;
   if (b.whole == 0 && b.num == 0)
      return a;

   RationalLL out;
   Rat_LongLong den   = a.den * b.den;
   Rat_LongLong num_a = a.num * b.den;
   Rat_LongLong num_b = b.num * a.den;

   const Rat_LongLong a_gcd   = _gcdll(num_a, den);
   const Rat_LongLong com_gcd = _gcdll(a_gcd, num_b);
   if (com_gcd > 0) {
      num_a /= com_gcd;
      num_b /= com_gcd;
      den   /= com_gcd;
   }
   out        = _limit_denominator_ll(num_a + num_b, den);
   out.whole += a.whole + b.whole;
   _cleanupll(&out);
   return out;
}
/**********************************************************************/
RationalLL RationalSub(RationalLL a, RationalLL b)
{
   _reducell(&a);
   _reducell(&b);

   _reduce_by_gcdll(&a.num, &a.den);
   _reduce_by_gcdll(&b.num, &b.den);

   if (a.whole == 0 && a.num == 0)
      return _rationalll_negate(b);
   if (b.whole == 0 && b.num == 0)
      return a;

   RationalLL out;
   Rat_LongLong den   = a.den * b.den;
   Rat_LongLong num_a = a.num * b.den;
   Rat_LongLong num_b = b.num * a.den;

   const Rat_LongLong a_gcd   = _gcdll(num_a, den);
   const Rat_LongLong com_gcd = _gcdll(a_gcd, num_b);
   if (_absll(com_gcd) > 0) {
      num_a /= com_gcd;
      num_b /= com_gcd;
      den   /= com_gcd;
   }
   out        = _limit_denominator_ll(num_a - num_b, den);
   out.whole += a.whole - b.whole;
   _cleanupll(&out);
   return out;
}
/**********************************************************************/
// Number of bits in the stored significand (no implicit bit)
#define DBL_FRAC_BITS (__DBL_MANT_DIG__ - 1)

// Masks derived portably
#define DBL_FRAC_MASK ((UINT64_MACRO(1) << DBL_FRAC_BITS) - 1)
#define DBL_IMPLICIT  (UINT64_MACRO(1) << DBL_FRAC_BITS)
#define DBL_EXP_BIAS  (__DBL_MAX_EXP__ - 1)
#define DBL_EXP_SHIFT DBL_FRAC_BITS
#define DBL_EXP_MASK  ((UINT64_MACRO(1) << (64 - DBL_FRAC_BITS - 1)) - 1)

typedef union {
   double x;
   Rat_Dbl_Cmp bits;
} DoubleBits;

static Rat_Long _get_mantissa(double val)
{
   DoubleBits u;
   u.x = val;
   return (u.bits & DBL_FRAC_MASK) | DBL_IMPLICIT;
}

static int _get_exponent(double val)
{
   DoubleBits u;
   u.x = val;
   return ((u.bits >> DBL_EXP_SHIFT) & DBL_EXP_MASK) - DBL_EXP_BIAS -
          DBL_FRAC_BITS;
}

static int _get_clear_sign(double *val)
{
   // grab sign bit (need to do it this way due to -0)
   DoubleBits u;
   u.x      = *val;
   int sign = (u.bits >> 63) & 1;
   u.x      = (sign) ? -u.x : u.x;
   *val     = u.x;

   return sign;
}

static void _decomp_dbl(double x, int *sign, Rat_Long *mantissa, int *exponent)
{
   *sign = _get_clear_sign(&x);

   *mantissa = _get_mantissa(x);
   *exponent = _get_exponent(x);
}
/**********************************************************************/
/*  Convert a double precision value to an exact rational             */
/*  whole + num/den                                                   */

Rational double2rational(double val)
{
   if (fabs(val) > _RATLONG_MAX_) {
      // TODO: clean up the condition, also maybe just throw a warning for the
      // caller to deal with?
      fprintf(stderr,
              "In double2rational() input double %le is larger than the "
              "largest whole part of Rational. Exiting...\n",
              val);
      exit(EXIT_FAILURE);
   }

   Rational out = RATIONAL_ZERO;

   double integer = 0;
   val            = modf(val, &integer);
   Rat_Long whole = 0;
   whole          = (Rat_Long)integer;
   // something weird can happen if val equals _RATLONG_MAX_
   if (integer != whole) {
      if (integer * whole < 0) {
         if (integer < whole) {
            whole = _RATLONG_MIN_;
         }
         else if (integer > whole) {
            whole = _RATLONG_MAX_;
         }
      }
      else {
         Rat_Long err  = integer - whole;
         whole        += err;
      }
   }

   if (val == 0)
      return RATIONAL_NGCD(whole, 0, 1);

   Rat_Long mantissa;
   int sign, exponent;
   _decomp_dbl(val, &sign, &mantissa, &exponent);
   Rat_LongLong num = 0, den = 0;

   // TODO: this needs testing, especially with small numbers
   if (exponent >= 0) {
      // val = mantissa * 2^exponent exactly
      const int max_shift = 127 - __DBL_MANT_DIG__;
      if (exponent > max_shift) {
         // exact value overflows int128_t; shift down (low bits are zero for
         // exact ints)
         num = (Rat_LongLong)mantissa << max_shift;
         den = 1;
      }
      else {
         num = (Rat_LongLong)mantissa << exponent; // exact integer, den=1
         den = 1;
      }
   }
   else if (-exponent <= 126) {
      num = (Rat_LongLong)mantissa;
      den = ((Rat_LongLong)1) << (-exponent);
      _reduce_by_gcdl(&out.num, &out.den);
   }
   else {
      // -exponent > 127: num would overflow
      const int excess = (-exponent) - 126;
      if (excess >= __DBL_MANT_DIG__) {
         // Value too small to represent; round to zero
         num = 0;
         den = 1;
      }
      else {
         // Best approximation: scale both down to fit in int128_t
         num = (Rat_LongLong)mantissa >> excess;
         den = ((Rat_LongLong)1) << 126;
      }
   }
   out        = _limit_denominator((sign) ? -num : num, den);
   out.whole += whole;
   _cleanup(&out);
   return out;
}
/**********************************************************************/
double rational2double(Rational rat)
{
   _reduce(&rat);
   return ((double)rat.num / rat.den) + (double)rat.whole;
}
/**********************************************************************/
Rat_Long RationalRoundUp(const Rational rat)
{
   return rat.whole + ((_absl(rat.num) > 0 && rat.whole > 0) ? 1 : 0);
}
/**********************************************************************/
Rat_Long RationalRoundDown(const Rational rat)
{
   return rat.whole - ((_absl(rat.num) > 0 && rat.whole < 0) ? 1 : 0);
}
/**********************************************************************/
Rational RationalAbs(Rational rat)
{
   rat.whole = _absl(rat.whole);
   rat.num   = _absl(rat.num);
   return rat;
}
/**********************************************************************/
Rational RationalNegate(Rational rat)
{
   rat.whole = -rat.whole;
   rat.num   = -rat.num;
   return rat;
}
/**********************************************************************/
int ispos_rational(Rational a)
{
   _reduce(&a);
   return (a.whole > 0 || (a.whole == 0 && a.num > 0));
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
   _reduce(&a);
   _reduce(&b);
   Rat_Long a_whole = a.whole, b_whole = b.whole;
   a.whole = 0;
   b.whole = 0;
   return (a_whole < b_whole) ||
          ((a_whole == b_whole) && (rational2double(a) < rational2double(b)));
}
/**********************************************************************/
int isgreater_rational(Rational a, Rational b)
{
   _reduce(&a);
   _reduce(&b);
   Rat_Long a_whole = a.whole, b_whole = b.whole;
   a.whole = 0;
   b.whole = 0;
   return (a_whole > b_whole) ||
          ((a_whole == b_whole) && (rational2double(a) > rational2double(b)));
}
/**********************************************************************/
/* Stringify Rational WITHOUT REDUCING                                */
void rat2str(Rational rat, char str[RATIONAL_STR_LEN])
{
   snprintf(str, RATIONAL_STR_LEN, "%ld + (%ld/%ld)", rat.whole, rat.num,
            rat.den);
}
/**********************************************************************/
Rational ToRational(const RationalLL rat_ll)
{
   Rational rat            = _limit_denominator(rat_ll.num, rat_ll.den);
   Rat_LongLong test_whole = rat.whole + rat_ll.whole;
   if (!(_RATLONG_MIN_ < test_whole && test_whole < _RATLONG_MAX_)) {
      if (test_whole < 0)
         test_whole =
             (test_whole <= _RATLONG_MIN_) ? _RATLONG_MIN_ : test_whole;
      else
         test_whole =
             (test_whole >= _RATLONG_MAX_) ? _RATLONG_MAX_ : test_whole;
   }
   rat.whole = test_whole;
   return rat;
}
/**********************************************************************/
RationalLL ToRationalLL(const Rational rat)
{
   return RATIONALLL_NGCD(rat.whole, rat.num, rat.den);
}

/* #ifdef __cplusplus
** }
** #endif
*/
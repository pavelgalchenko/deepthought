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

typedef unsigned long int Rat_ULong;

#define RAT_MAX_DEN (_RATLONG_MAX_ >> 4)

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

#ifdef __has_builtin
#if (_SIZEOF_RATLONGLONG_) == (__SIZEOF_LONG_LONG__) &&                        \
    __has_builtin(__builtin_ctzll)
#define _ctzll (__builtin_ctzll)
#endif
#endif

#ifndef _ctzl
static unsigned int _ctzl(Rat_ULong v) __attribute__((const));
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
static unsigned int _ctzll(Rat_ULongLong v) __attribute__((const));
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
static Rat_ULongLong _absll(Rat_LongLong x) __attribute((const));
static Rat_ULongLong _absll(Rat_LongLong x)
{
   if (x < 0)
      return -x;
   return x;
}
#endif

#if _SIZEOF_RATLONG_ != __SIZEOF_LONG__
static Rat_ULong _absl(Rat_Long x) __attribute((const));
static Rat_ULong _absl(Rat_Long x)
{
   if (x < 0)
      return -x;
   return x;
}
#else
#define _absl labs
#endif

#define RATIONALLL_RAW(whl, n, d)                                              \
   ((RationalLL){.whole = (whl), .num = (n), .den = (d)})
#define RATIONALLL_NGCD(whl, n, d)                                             \
   RATIONALLL_RAW((whl) + ((SIGN(d) * (n)) / MAX_ABS_ONE(d)),                  \
                  ((SIGN(d) * (n)) % MAX_ABS_ONE(d)), MAX_ABS_ONE(d))

/**********************************************************************/
static RationalLL _rationalll_negate(RationalLL rat) __attribute__((const));
static RationalLL _rationalll_negate(RationalLL rat)
{
   rat.whole = -rat.whole;
   rat.num   = -rat.num;
   return rat;
}
/**********************************************************************/
static Rat_LongLong _gcdll(Rat_LongLong a, Rat_LongLong b)
    __attribute__((const));
static Rat_LongLong _gcdll(Rat_LongLong a, Rat_LongLong b)
{
   if (a == 0L)
      return b;
   if (b == 0L)
      return a;
   Rat_ULongLong au = _absll(a);
   Rat_ULongLong bu = _absll(b);
   if (au < RAT_MAX_DEN && bu < RAT_MAX_DEN)
      return 1;

   const int shift   = _ctzll(au | bu);
   au              >>= _ctzll(au);
   do {
      bu >>= _ctzll(bu);
      if (au > bu) {
         Rat_ULongLong t = au;
         au              = bu;
         bu              = t;
      }
   } while (bu -= au);
   return au << shift;
}
/**********************************************************************/
/*  From Stack Overflow user Maxim Egorushkin                         */
/*  Computes the Greatest Common Divisior of two longs using the      */
/*  Binary GCD algorithm, with speedups from builtins                 */
static Rat_Long _gcdl(Rat_Long a, Rat_Long b) __attribute__((const));
static Rat_Long _gcdl(Rat_Long a, Rat_Long b)
{
   if (a == 0L)
      return b;
   if (b == 0L)
      return a;
   Rat_ULong au = _absl(a);
   Rat_ULong bu = _absl(b);
   if (au < RAT_MAX_DEN && bu < RAT_MAX_DEN)
      return 1;

   const int shift   = _ctzl(au | bu);
   au              >>= _ctzl(au);
   do {
      bu >>= _ctzl(bu);
      if (au > bu) {
         Rat_ULong t = au;
         au          = bu;
         bu          = t;
      }
   } while (bu -= au);
   return au << shift;
}
/**********************************************************************/
/*  Reduce two longs by their greatest common divisor                 */
#define _gcd_generic(a, b)                                                     \
   _Generic(TWO_ARG_TYPES((a), (b)),                                           \
       void (*)(Rat_Long, Rat_Long): _gcdl,                                    \
       void (*)(Rat_LongLong, Rat_LongLong): _gcdll)((a), (b))
#define RAT_REDUCE_BY_GCD(a, b)                                                \
   do {                                                                        \
      __auto_type gcd = _gcd_generic((a), (b));                                \
      if (gcd > 1) {                                                           \
         (a) = (a) / gcd;                                                      \
         (b) = (b) / gcd;                                                      \
      }                                                                        \
   } while (0)
/**********************************************************************/
/* For use when it is known that at least one of num or den is a      */
/* power of 2                                                         */
#define REDUCE_BY_GCD_POW2(num, den)                                           \
   do {                                                                        \
      if ((den) < 0) {                                                         \
         num = -(num);                                                         \
         den = -(den);                                                         \
      }                                                                        \
      const int ispos = num >= 0;                                              \
      num = _Generic((num), Rat_Long: _absl, Rat_LongLong: _absll)(num);       \
      const int shift =                                                        \
          (num == 0 || den == 0)                                               \
              ? 0                                                              \
              : (_Generic((num), Rat_Long: _ctzl, Rat_LongLong: _ctzll)(       \
                    (num) | (den)));                                           \
      num = (num) >> shift;                                                    \
      den = (den) >> shift;                                                    \
      if (!ispos)                                                              \
         num = -num;                                                           \
   } while (0)
/**********************************************************************/
#define _validate_den(x)                                                       \
   _Generic((x),                                                               \
       Rational: (Rational){.whole = (x).whole,                                \
                            .num   = (x).num,                                  \
                            .den   = ((x).den != 0) ? (x).den : 1},            \
       RationalLL: (RationalLL){.whole = (x).whole,                            \
                                .num   = (x).num,                              \
                                .den   = ((x).den != 0) ? (x).den : 1})
#define _positive_denom(x)                                                     \
   _Generic((x),                                                               \
       Rational: (Rational){.whole = (x).whole,                                \
                            .num   = ((x).den > 0) ? (x).num : -(x).num,       \
                            .den   = ((x).den > 0) ? (x).den : -(x).den},      \
       RationalLL: (RationalLL){.whole = (x).whole,                            \
                                .num   = ((x).den > 0) ? (x).num : -(x).num,   \
                                .den   = ((x).den > 0) ? (x).den : -(x).den})
/**********************************************************************/
#define _reduce(rat)                                                           \
   do {                                                                        \
      rat        = _validate_den(rat);                                         \
      rat        = _positive_denom(rat);                                       \
      rat.whole += rat.num / rat.den;                                          \
      rat.num   %= rat.den;                                                    \
      if ((rat.whole ^ rat.num) < 0 && rat.whole != 0 && rat.num != 0) {       \
         if (rat.whole > 0) {                                                  \
            rat.num += rat.den;                                                \
            rat.whole--;                                                       \
         }                                                                     \
         else if (rat.whole < 0) {                                             \
            rat.num -= rat.den;                                                \
            rat.whole++;                                                       \
         }                                                                     \
      }                                                                        \
      if (rat.num == 0)                                                        \
         rat.den = 1;                                                          \
   } while (0)
/**********************************************************************/
#define _cleanup_body(rat)                                                     \
   _reduce(rat);                                                               \
   if (rat.den > RAT_MAX_DEN)                                                  \
      RAT_REDUCE_BY_GCD(rat.num, rat.den);                                     \
   return rat;
/**********************************************************************/
static inline Rational _cleanupl(Rational rat) __attribute__((const));
static inline Rational _cleanupl(Rational rat)
{
   _cleanup_body(rat)
}
/**********************************************************************/
static inline RationalLL _cleanupll(RationalLL rat) __attribute__((const));
static inline RationalLL _cleanupll(RationalLL rat){_cleanup_body(rat)}
/**********************************************************************/
#define _cleanup(rat)                                                          \
   _Generic((rat), Rational: _cleanupl, RationalLL: _cleanupll)(rat)
/**********************************************************************/
static Rat_LongLong _rounddown_div(const Rat_LongLong a, const Rat_LongLong b)
    __attribute__((const));
static Rat_LongLong _rounddown_div(const Rat_LongLong a, const Rat_LongLong b)
{
   const Rat_LongLong p = (b < 0) ? -a : a;
   const Rat_LongLong q = (b < 0) ? -b : b;
   Rat_LongLong c       = p / q;
   if ((c <= 0) && ((p % q) < 0))
      c--;
   return c;
}
/**********************************************************************/
/*  Limit the size of the denomiator of the fraction p/q. Checks if   */
/*  solution is the convergent or the semiconvergent of the below     */
/*  process.                                                          */
/*  Taken from python's fractions.Fraction.limit_denominator().       */
/* making a macro to keep consistency between Rational and RationalLL */
#define LIM_DENOM_HELPER(p, q)                                                 \
   do {                                                                        \
      const Rat_Long max_den = RAT_MAX_DEN;                                    \
      if ((p) == 0) {                                                          \
         (q) = 1;                                                              \
         break;                                                                \
      }                                                                        \
      if ((q) < 0) {                                                           \
         (p) = -(p);                                                           \
         (q) = -(q);                                                           \
      }                                                                        \
                                                                               \
      /* calculating gcd is expensive, dont do unless needed */                \
      if ((q) <= max_den)                                                      \
         break;                                                                \
                                                                               \
      RAT_REDUCE_BY_GCD((p), (q));                                             \
                                                                               \
      if ((q) <= max_den)                                                      \
         break;                                                                \
                                                                               \
      Rat_LongLong p0 = 0, q0 = 1, p1 = 1, q1 = 0;                             \
      Rat_LongLong n = (p), d = (q);                                           \
      Rat_LongLong t1 = 0;                                                     \
      while (1) {                                                              \
         Rat_LongLong a        = _rounddown_div(n, d);                         \
         const Rat_LongLong q2 = q0 + a * q1;                                  \
         if (q2 > max_den)                                                     \
            break;                                                             \
         t1 = p0;                                                              \
         p0 = p1;                                                              \
         p1 = t1 + a * p1;                                                     \
         q0 = q1;                                                              \
         q1 = q2;                                                              \
                                                                               \
         t1 = n;                                                               \
         n  = d;                                                               \
         d  = t1 - a * d;                                                      \
      }                                                                        \
      Rat_LongLong k = _rounddown_div(max_den - q0, q1);                       \
                                                                               \
      t1 = 2 * d * (q0 + k * q1);                                              \
      if (t1 <= (q)) {                                                         \
         (p) = p1;                                                             \
         (q) = q1;                                                             \
         break;                                                                \
      }                                                                        \
      else {                                                                   \
         (p) = p0 + k * p1;                                                    \
         (q) = q0 + k * q1;                                                    \
         break;                                                                \
      }                                                                        \
   } while (0)
/**********************************************************************/
Rational _limit_denominator(Rat_LongLong p, Rat_LongLong q)
{
   LIM_DENOM_HELPER(p, q);
   return RATIONAL_NGCD(p / q, p % q, q);
}
/**********************************************************************/
/*  Limit the size of the denomiator of the fraction p/q. Checks if   */
/*  solution is the convergent or the semiconvergent of the below     */
/*  process.                                                          */
/*  Taken from python's fraction.Fraction.limit_denominator().        */
RationalLL _limit_denominator_ll(Rat_LongLong p, Rat_LongLong q)
{
   LIM_DENOM_HELPER(p, q);
   return RATIONALLL_NGCD(p / q, p % q, q);
}
/**********************************************************************/
Rational InitRational(const Rat_Long whole, const Rat_Long num,
                      const Rat_Long den)
{
   Rational rat  = RATIONAL_NGCD(whole, num, den);
   rat           = _cleanup(rat);
   rat           = _limit_denominator(rat.num, rat.den);
   rat.whole    += whole;
   return rat;
}
/**********************************************************************/
Rat_Long _rat_int_mod_rat(Rational *const rat, const Rat_Long mod)
{
   Rat_Long old_whole  = rat->whole;
   rat->whole         %= mod;
   return old_whole / mod;
}
/**********************************************************************/
Rat_LongLong _rat_int_mod_ratll(RationalLL *const rat, const Rat_LongLong mod)
{
   Rat_LongLong old_whole  = rat->whole;
   rat->whole             %= mod;
   return old_whole / mod;
}
/**********************************************************************/
Rational ReduceRational(Rational rat)
{
   _reduce(rat);
   if (rat.den > RAT_MAX_DEN)
      RAT_REDUCE_BY_GCD(rat.num, rat.den);
   return rat;
}
/**********************************************************************/
/*  Multiply integer by rational, returning integer whole part and    */
/*  Rational fractional part                                          */
Rational _int_rat_mult(const Rat_LongLong mul, RationalLL rat)
{
   _reduce(rat);
   RationalLL out_ll;
   Rat_LongLong product = ((Rat_LongLong)mul * rat.num);

   out_ll.whole = (product / rat.den) + ((Rat_LongLong)mul * rat.whole);
   out_ll.num   = product % rat.den;
   out_ll.den   = rat.den;
   out_ll       = _cleanup(out_ll);
   return ToRational(out_ll);
}
/**********************************************************************/
/*  Multiply integer by rational, returning integer whole part and    */
/*  Rational fractional part                                          */
/*  This version sets:  '*carry = out.whole / mod'                    */
/*                and:  'out.whole %= mod'                            */
Rational _int_rat_mult_mod(const Rat_LongLong mul, RationalLL rat, Rat_Long mod,
                           Rat_Long *const carry)
{
   _reduce(rat);
   if (!mod)
      mod = 1;

   RationalLL out_ll;
   Rat_LongLong product  = mul * rat.num;
   Rat_LongLong wholea   = product / rat.den;
   Rat_LongLong wholeb   = mul * rat.whole;
   *carry                = (wholea / mod) + (wholeb / mod);
   out_ll.whole          = (wholea % mod) + (wholeb % mod);
   *carry               += out_ll.whole / mod;
   out_ll.whole         %= mod;
   out_ll.num            = product % rat.den;
   out_ll.den            = rat.den;
   out_ll                = _cleanup(out_ll);
   return ToRational(out_ll);
}
/**********************************************************************/
/*  Compute a * b where a and b are both Rationals                    */
RationalLL _rat_mult(RationalLL a, RationalLL b)
{
   _reduce(a);
   _reduce(b);

   if ((a.whole == 0 && a.num == 0) || (b.whole == 0 && b.num == 0))
      return RATIONALLL_RAW(0, 0, 1);

   RationalLL out;
   Rat_LongLong num_a = a.whole * a.den + a.num;
   Rat_LongLong den_a = a.den;
   Rat_LongLong num_b = b.whole * b.den + b.num;
   Rat_LongLong den_b = b.den;

   RAT_REDUCE_BY_GCD(num_a, den_b);
   RAT_REDUCE_BY_GCD(num_b, den_a);
   out = RATIONALLL_RAW(0, num_a * num_b, den_a * den_b);
   out = _cleanup(out);
   return out;
}
/**********************************************************************/
/*  Compute a / b where a and b are both Rationals                    */
RationalLL _rat_divide(RationalLL a, RationalLL b)
{
   _reduce(a);
   _reduce(b);

   if (a.whole == 0 && a.num == 0)
      return RATIONALLL_RAW(0, 0, 1);

   if (b.whole == 0 && b.num == 0) {
      fprintf(stderr, "Divide by zero in _rat_divide. Exiting...\n");
      exit(EXIT_FAILURE);
   }

   RationalLL out     = {0};
   Rat_LongLong num_a = a.whole * a.den + a.num;
   Rat_LongLong den_a = a.den;
   Rat_LongLong num_b = b.whole * b.den + b.num;
   Rat_LongLong den_b = b.den;

   RAT_REDUCE_BY_GCD(num_a, num_b);
   RAT_REDUCE_BY_GCD(den_a, den_b);
   out = RATIONALLL_RAW(0, num_a * den_b, den_a * num_b);
   out = _cleanup(out);
   return out;
}
/**********************************************************************/
RationalLL _rat_add(RationalLL a, RationalLL b)
{
   _reduce(a);
   _reduce(b);

   if (a.whole == 0 && a.num == 0)
      return b;
   if (b.whole == 0 && b.num == 0)
      return a;

   RationalLL out = a;
   if (a.num == 0) {
      a   = b;
      b   = out;
      out = a;
   }

   if (b.num != 0) {
      Rat_LongLong den   = a.den * b.den;
      Rat_LongLong num_a = a.num * b.den;
      Rat_LongLong num_b = b.num * a.den;

      const Rat_LongLong a_gcd = _gcdll(num_a, den);
      Rat_LongLong com_gcd     = 1;
      if (a_gcd != 1)
         com_gcd = _gcdll(a_gcd, num_b);
      if (com_gcd > 1) {
         num_a /= com_gcd;
         num_b /= com_gcd;
         den   /= com_gcd;
      }

      out = RATIONALLL_RAW(a.whole + b.whole, num_a + num_b, den);
   }
   else
      out.whole += b.whole;

   out = _cleanup(out);
   return out;
}
/**********************************************************************/
RationalLL _rat_sub(RationalLL a, RationalLL b)
{
   _reduce(a);
   _reduce(b);

   if (a.whole == 0 && a.num == 0)
      return _rationalll_negate(b);
   if (b.whole == 0 && b.num == 0)
      return a;

   RationalLL out = a;
   if (a.num == 0) {
      a   = (RationalLL){.whole = -b.whole, .num = -b.num, .den = b.den};
      b   = (RationalLL){.whole = -out.whole, .num = -out.num, .den = out.den};
      out = a;
   }

   if (b.num != 0) {
      Rat_LongLong den   = a.den * b.den;
      Rat_LongLong num_a = a.num * b.den;
      Rat_LongLong num_b = b.num * a.den;

      const Rat_LongLong a_gcd = _gcdll(num_a, den);
      Rat_LongLong com_gcd     = 1;
      if (a_gcd != 1)
         com_gcd = _gcdll(a_gcd, num_b);
      if (com_gcd > 1) {
         num_a /= com_gcd;
         num_b /= com_gcd;
         den   /= com_gcd;
      }

      Rat_LongLong num  = num_a - num_b;
      out               = _limit_denominator_ll(num, den);
      out.whole        += a.whole - b.whole;
   }
   else
      out.whole -= b.whole;

   out = _cleanup(out);
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

static Rat_Long _get_mantissa(DoubleBits u)
{
   return (u.bits & DBL_FRAC_MASK) | DBL_IMPLICIT;
}

static int _get_exponent(DoubleBits u)
{
   return ((u.bits >> DBL_EXP_SHIFT) & DBL_EXP_MASK) - DBL_EXP_BIAS -
          DBL_FRAC_BITS;
}

static int _get_clear_sign(DoubleBits *u)
{
   // grab sign bit (need to do it this way due to -0)
   const int sign_bit = (u->bits >> 63) & 1;
   u->x               = (sign_bit) ? -u->x : u->x;

   return sign_bit;
}

static void _decomp_dbl(double x, int *sign, Rat_Long *mantissa, int *exponent)
{
   DoubleBits u;
   u.x       = x;
   *sign     = _get_clear_sign(&u);
   *mantissa = _get_mantissa(u);
   *exponent = _get_exponent(u);
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
      REDUCE_BY_GCD_POW2(num, den);
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
         REDUCE_BY_GCD_POW2(num, den);
      }
   }
   out        = _limit_denominator((sign) ? -num : num, den);
   out.whole += whole;
   out        = _cleanup(out);
   return out;
}
/**********************************************************************/
double rational2double(Rational rat)
{
   _reduce(rat);
   return ((double)rat.num / rat.den) + (double)rat.whole;
}
/**********************************************************************/
Rat_Long _rat_round_up(const Rational rat)
{
   return rat.whole + ((_absl(rat.num) > 0 && rat.whole > 0) ? 1 : 0);
}
/**********************************************************************/
Rat_Long _rat_round_down(const Rational rat)
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
   _reduce(a);
   return (a.whole > 0 || (a.whole == 0 && a.num > 0));
}
/**********************************************************************/
int isequal_rational(Rational a, Rational b)
{
   // just going to do this the easy way to help avoid reduction
   return (rational2double(a) == rational2double(b));
}
/**********************************************************************/
int isless_rational(Rational a, Rational b)
{
   _reduce(a);
   _reduce(b);
   Rat_Long a_whole = a.whole, b_whole = b.whole;
   a.whole = 0;
   b.whole = 0;
   return (a_whole < b_whole) ||
          ((a_whole == b_whole) && (rational2double(a) < rational2double(b)));
}
/**********************************************************************/
int isgreater_rational(Rational a, Rational b)
{
   _reduce(a);
   _reduce(b);
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
   const char *rat_str_fmt = "%ld + (%ld/%ld)";
   snprintf(str, RATIONAL_STR_LEN, rat_str_fmt, rat.whole, rat.num, rat.den);
}
/**********************************************************************/
Rational _rat_to_rational(const RationalLL rat_ll)
{
   Rational rat            = _limit_denominator(rat_ll.num, rat_ll.den);
   Rat_LongLong test_whole = ((Rat_LongLong)rat.whole) + rat_ll.whole;
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
RationalLL _rat_to_rationalll(const Rational rat)
{
   return RATIONALLL_NGCD(rat.whole, rat.num, rat.den);
}

/* #ifdef __cplusplus
** }
** #endif
*/
/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __RATIONALKIT_H__
#define __RATIONALKIT_H__

#include "defineskit.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef signed long int Rat_Long;
#define _SIZEOF_RATLONG_ (__SIZEOF_LONG__)
#define _RATLONG_MAX_    (__LONG_MAX__)
#define _RATLONG_MIN_    (-_RATLONG_MAX_ - 1)
#define RATIONAL_STR_LEN (64)
#define RATIONAL_STR_FMT "%64s"

// Use 'long long int' if its larger than 'long int'. If not use int128 if we
// have it
#if (__SIZEOF_LONG_LONG__ > _SIZEOF_RATLONG_)
typedef signed long long int Rat_LongLong;
#define _SIZEOF_RATLONGLONG_ (__SIZEOF_LONG_LONG__)
#elif defined(__SIZEOF_INT128__)
typedef __int128_t Rat_LongLong;
#define _SIZEOF_RATLONGLONG_ (__SIZEOF_INT128__)
#else
_Static_assert(
    0, "Configuration does not support rationalkit. Two different sizes of "
       "integer types are required, preferrably 64-bit/128-bit.");
#endif

// represents a number using the form "whole + (num/denom)"
typedef struct Rational {
   Rat_Long whole;
   Rat_Long num;
   Rat_Long den;
} Rational;

// TODO: would rather this just be a type hidden in the .c; will be thinking
// about other ways to put rationalkit together
// used to hold intermediate calculations, Rational is the interface to use
typedef struct RationalLL {
   Rat_LongLong whole;
   Rat_LongLong num;
   Rat_LongLong den;
} RationalLL;

// Create rational without gcd reduction
#define MAX_ABS_ONE(d) (MAX(1, ABS(d)))
#define RATIONAL_RAW(whl, n, d)                                                \
   ((Rational){.whole = (whl), .num = (n), .den = (d)})
#define RATIONAL_NGCD(whl, n, d)                                               \
   RATIONAL_RAW((whl) + ((SIGN(d) * (n)) / MAX_ABS_ONE(d)),                    \
                ((SIGN(d) * (n)) % MAX_ABS_ONE(d)), MAX_ABS_ONE(d))
#define RATIONAL_ZERO RATIONAL_RAW(0, 0, 1)

Rational InitRational(const Rat_Long whole, const Rat_Long num,
                      const Rat_Long den) __attribute__((const));
Rat_Long RationalIntMod(Rational *const rat, const Rat_Long mod)
    __attribute__((pure));
Rat_LongLong RationalIntMod_ll(RationalLL *const rat, const Rat_LongLong mod)
    __attribute__((pure));
Rational ReduceRational(Rational rat) __attribute__((const));
Rational IntegerRationalMult(const Rat_LongLong mul, RationalLL rat)
    __attribute__((const));
Rational IntegerRationalMultMod(const Rat_LongLong mul, RationalLL rat,
                                Rat_Long mod, Rat_Long *const carry)
    __attribute__((pure));
RationalLL RationalMult(RationalLL a, RationalLL b) __attribute__((const));
RationalLL RationalDivide(RationalLL a, RationalLL b) __attribute__((const));
RationalLL RationalAdd(RationalLL a, RationalLL b) __attribute__((const));
RationalLL RationalSub(RationalLL a, RationalLL b) __attribute__((const));
Rational double2rational(const double val) __attribute__((const));
double rational2double(const Rational rat) __attribute__((const));
Rat_Long RationalRoundUp(const Rational rat) __attribute__((const));
Rat_Long RationalRoundDown(const Rational rat) __attribute__((const));
Rational RationalAbs(Rational rat) __attribute__((const));
Rational RationalNegate(Rational rat) __attribute__((const));
int ispos_rational(Rational a) __attribute__((const));
int isequal_rational(const Rational a, const Rational b) __attribute__((const));
int isless_rational(const Rational a, const Rational b) __attribute__((const));
int isgreater_rational(const Rational a, const Rational b)
    __attribute__((const));
void rat2str(Rational rat, char str[RATIONAL_STR_LEN]);
Rational ToRational(const RationalLL rat_ll) __attribute__((const));
RationalLL ToRationalLL(const Rational rat) __attribute__((const));

#endif /* __RATIONALKIT_H__ */
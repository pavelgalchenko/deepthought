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

#include <math.h>
#include <stdlib.h>

#ifdef __INT64_MAX__
typedef int64_t Rat_Long;
#define __SIZEOF_RATLONG__ (8)
#else
typedef signed long int Rat_Long;
#define __SIZEOF_RATLONG__ (__SIZEOF_LONG__)
#endif

// represents a number using the form "whole + (num/denom)"
typedef struct Rational {
   Rat_Long whole;
   Rat_Long num;
   Rat_Long den;
} Rational;

#define RATIONAL_ZERO ((Rational){.whole = 0, .num = 0, .den = 1})

Rational InitRational(const Rat_Long whole, const Rat_Long num,
                      const Rat_Long den);
Rat_Long RationalIntMod(Rational *const rat, const Rat_Long mod);
void ReduceRational(Rational *const);
Rational IntegerRationalMult(const Rat_Long mul, Rational rat);
Rational IntegerRationalMultMod(const Rat_Long mul, Rational rat, Rat_Long mod,
                                Rat_Long *const carry);
Rational RationalMult(Rational a, Rational b);
Rational RationalDivide(Rational a, Rational b);
Rational RationalAdd(Rational a, Rational b);
Rational RationalSub(Rational a, Rational b);
Rational double2rational(const double val);
double rational2double(const Rational rat);
Rat_Long RationalRoundUp(const Rational rat);
Rat_Long RationalRoundDown(const Rational rat);
Rational RationalAbs(Rational rat);
Rational RationalNegate(Rational rat);
int isequal_rational(const Rational a, const Rational b);
int isless_rational(const Rational a, const Rational b);
int isgreater_rational(const Rational a, const Rational b);

#endif /* __RATIONALKIT_H__ */
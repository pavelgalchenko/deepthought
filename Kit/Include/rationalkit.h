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

#include <stdlib.h>

// represents a number using the form "whole + (num/denom)"
typedef struct Rational {
   signed long whole;
   signed long num;
   signed long den;
} Rational;

Rational IntegerRationalMult(const long mul, Rational rat);
Rational IntegerRationalMultMod(const long mul, Rational rat, long mod,
                                long *const carry);
Rational RationalMult(Rational a, Rational b);
Rational RationalDivide(Rational a, Rational b);
Rational RationalAdd(Rational a, Rational b);
Rational RationalSub(Rational a, Rational b);
Rational double2rational(const double val);
double rational2double(const Rational rat);
long RationalRoundUp(const Rational rat);
long RationalRoundDown(const Rational rat);
int isequal_rational(const Rational a, const Rational b);
int isless_rational(const Rational a, const Rational b);
int isgreater_rational(const Rational a, const Rational b);

#endif /* __RATIONALKIT_H__ */
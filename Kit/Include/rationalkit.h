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
#include <string.h>

// represents a number using the form "whole + p/q"
typedef struct Rational {
   signed long whole;
   signed long p;
   signed long q;
} Rational;

Rational IntegerRationalMult(const long mul, const Rational rat);
Rational RationalDivide(const Rational a, const Rational b);
Rational double2rational(const double val);
double rational2double(Rational rat);
long RationalRoundUp(Rational rat);
long RationalRoundDown(Rational rat);

#endif /* __RATIONALKIT_H__ */
/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __TEST_LIB_H__
#define __TEST_LIB_H__

#include "42constants.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TRUE  1
#define FALSE 0

#define ASSERT(actual, expected)                                               \
   if ((actual) != (expected)) {                                               \
      return (FALSE);                                                          \
   }

#define TEST(actual, expected) ((actual) == (expected))
#define TEST_DOUBLE(actual, expected, tol)                                     \
   (fabs((double)(actual) - (double)(expected)) <= (double)(tol))

long TEST_MAT(const long n, const long m, const double actual[n][m],
              const double expected[n][m], const double tol);
long TEST_MATP(const long n, const long m, double **actual, double **expected,
               const double tol);
long TEST_VEC(const long n, const double actual[n], const double expected[n],
              const double tol);
void print_hdr(const char *str, long len, const int nTabs);
int print_result(const long result, const char *str, long len, const int nTabs,
                 const char *trialInfo, const long isOkay,
                 const long printIfPass);

#endif
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
#include <criterion/criterion.h>
#include <criterion/new/assert.h>
#include <criterion/parameterized.h>
#include <criterion/theories.h>

#define SUITE_NAME rationalkit

#define ULP_THRESH     (4) // acceptable Units in Last Place variation
#define DBL_THRESH     (ULP_THRESH * __DBL_EPSILON__)
#define COND_STR(cond) (cond) ? ("TRUE") : ("FALSE")

#define RATIONAL_DATAPOINTS                                                    \
   DataPoints(                                                                 \
       Rational *, &RATIONAL_ZERO, &RATIONAL_RAW(1, 0, 1),                     \
       &RATIONAL_RAW(-1, 0, 1), &RATIONAL_RAW(32, 184, 1000),                  \
       &RATIONAL_RAW(32, 23, 125), &RATIONAL_RAW(-32, -184, 1000),             \
       &RATIONAL_RAW(-32, -23, 125),                                           \
       &RATIONAL_RAW(3, 1415926535, 10000000000),                              \
       &RATIONAL_RAW(3, 283185307, 2000000000),                                \
       &RATIONAL_RAW(-3, -1415926535, 10000000000),                            \
       &RATIONAL_RAW(-3, -283185307, 2000000000),                              \
       &RATIONAL_RAW(0, 1, _RATLONG_MAX_),                                     \
       &RATIONAL_RAW(0, 1, -_RATLONG_MAX_),                                    \
       &RATIONAL_RAW(0, _RATLONG_MAX_, 1),                                     \
       &RATIONAL_RAW(0, -_RATLONG_MAX_, 1), &RATIONAL_RAW(0, 1, (2L << 32)),   \
       &RATIONAL_RAW(0, 1, -(2L << 32)), &RATIONAL_RAW(0, (2L << 32), 1),      \
       &RATIONAL_RAW(0, -(2L << 32), 1), &RATIONAL_RAW(0, 1, (2L << 48)),      \
       &RATIONAL_RAW(0, 1, -(2L << 48)), &RATIONAL_RAW(0, (2L << 48), 1),      \
       &RATIONAL_RAW(0, -(2L << 48), 1), &RATIONAL_RAW(0, 1, (2L << 56)),      \
       &RATIONAL_RAW(0, 1, -(2L << 56)), &RATIONAL_RAW(0, (2L << 56), 1),      \
       &RATIONAL_RAW(0, -(2L << 56), 1), &RATIONAL_RAW(0, 1, (2L << 60)),      \
       &RATIONAL_RAW(0, 1, -(2L << 60)), &RATIONAL_RAW(0, (2L << 60), 1),      \
       &RATIONAL_RAW(0, -(2L << 60), 1),                                       \
       &RATIONAL_RAW(0, _RATLONG_MAX_, _RATLONG_MAX_),                         \
       &RATIONAL_RAW(0, -_RATLONG_MAX_, -_RATLONG_MAX_),                       \
       &RATIONAL_RAW(0, _RATLONG_MAX_, -_RATLONG_MAX_),                        \
       &RATIONAL_RAW(0, -_RATLONG_MAX_, _RATLONG_MAX_),                        \
       &RATIONAL_RAW(0, _RATLONG_MAX_, _RATLONG_MAX_ / 2),                     \
       &RATIONAL_RAW(0, -_RATLONG_MAX_, -_RATLONG_MAX_ / 2),                   \
       &RATIONAL_RAW(0, _RATLONG_MAX_, -_RATLONG_MAX_ / 2),                    \
       &RATIONAL_RAW(0, -_RATLONG_MAX_, _RATLONG_MAX_ / 2),                    \
       &RATIONAL_RAW(0, _RATLONG_MAX_ / 2, _RATLONG_MAX_),                     \
       &RATIONAL_RAW(0, -_RATLONG_MAX_ / 2, -_RATLONG_MAX_),                   \
       &RATIONAL_RAW(0, _RATLONG_MAX_ / 2, -_RATLONG_MAX_),                    \
       &RATIONAL_RAW(0, -_RATLONG_MAX_ / 2, _RATLONG_MAX_),                    \
       &RATIONAL_RAW(0, 1, 10), &RATIONAL_RAW(0, -1, 10),                      \
       &RATIONAL_RAW(0, 1, 1000000000000),                                     \
       &RATIONAL_RAW(0, -1, 1000000000000), &RATIONAL_RAW(42, 1, 42),          \
       &RATIONAL_RAW(-42, -1, 42))

/* Test rat2str                                                       */
TheoryDataPoints(SUITE_NAME, rat2str) = {RATIONAL_DATAPOINTS};

Theory((Rational * a), SUITE_NAME, rat2str)
{
   char a_str[RATIONAL_STR_LEN];
   rat2str(*a, a_str);
   long whl, n, d;
   sscanf(a_str, "%ld + (%ld/%ld)", &whl, &n, &d);

   cr_expect(all(a->whole == whl, a->num == n, a->den == d),
             "rat2str(%ld + (%ld/%ld)) returns " RATIONAL_STR_FMT
             "; this is not correct",
             a->whole, a->num, a->den, a_str);
}

/* Test conditionals                                                  */
struct ratcond_tuple {
   Rational a;
   Rational b;
   int iscond; // 0=equal, -1=isless, 1=isgreater
};

ParameterizedTestParameters(SUITE_NAME, conditional)
{
   // static is required as each element of val is passed as a
   // pointer to ParameterizedTest
   static struct ratcond_tuple vals[] = {
       {RATIONAL_ZERO, RATIONAL_ZERO, 0},
       {RATIONAL_NGCD(1, 0, 1), RATIONAL_NGCD(1, 0, 1), 0},
       {RATIONAL_NGCD(-1, 0, 1), RATIONAL_NGCD(-1, 0, 1), 0},
       {RATIONAL_NGCD(1, 0, 1), RATIONAL_ZERO, 1},
       {RATIONAL_NGCD(-1, 0, 1), RATIONAL_ZERO, -1},
       {RATIONAL_ZERO, RATIONAL_NGCD(1, 0, 1), -1},
       {RATIONAL_ZERO, RATIONAL_NGCD(-1, 0, 1), 1},
       {RATIONAL_NGCD(1, 0, 1), RATIONAL_NGCD(-1, 0, 1), 1},
       {RATIONAL_NGCD(-1, 0, 1), RATIONAL_NGCD(1, 0, 1), -1},
       {RATIONAL_ZERO, RATIONAL_NGCD(0, 1, _RATLONG_MAX_), -1},
       {RATIONAL_ZERO, RATIONAL_NGCD(0, -1, _RATLONG_MAX_), 1},
       {RATIONAL_NGCD(0, 1, _RATLONG_MAX_), RATIONAL_ZERO, 1},
       {RATIONAL_NGCD(0, -1, _RATLONG_MAX_), RATIONAL_ZERO, -1},
       {RATIONAL_NGCD(0, 1, _RATLONG_MAX_), RATIONAL_NGCD(0, 1, _RATLONG_MAX_),
        0},
       {RATIONAL_NGCD(0, -1, _RATLONG_MAX_),
        RATIONAL_NGCD(0, -1, _RATLONG_MAX_), 0},
       {RATIONAL_NGCD(0, 1, _RATLONG_MAX_), RATIONAL_NGCD(0, -1, _RATLONG_MAX_),
        1},
       {RATIONAL_NGCD(0, -1, _RATLONG_MAX_), RATIONAL_NGCD(0, 1, _RATLONG_MAX_),
        -1},
       {RATIONAL_NGCD(0, 1, -_RATLONG_MAX_),
        RATIONAL_NGCD(0, 1, -_RATLONG_MAX_), 0},
       {RATIONAL_NGCD(0, -1, -_RATLONG_MAX_),
        RATIONAL_NGCD(0, -1, -_RATLONG_MAX_), 0},
       {RATIONAL_NGCD(0, 1, -_RATLONG_MAX_),
        RATIONAL_NGCD(0, -1, -_RATLONG_MAX_), -1},
       {RATIONAL_NGCD(0, -1, -_RATLONG_MAX_),
        RATIONAL_NGCD(0, 1, -_RATLONG_MAX_), 1},
       {RATIONAL_NGCD(0, 1, _RATLONG_MAX_), RATIONAL_NGCD(0, 1, -_RATLONG_MAX_),
        1},
       {RATIONAL_NGCD(0, -1, _RATLONG_MAX_),
        RATIONAL_NGCD(0, -1, -_RATLONG_MAX_), -1},
       {RATIONAL_NGCD(0, 1, -_RATLONG_MAX_),
        RATIONAL_NGCD(0, -1, _RATLONG_MAX_), 0},
       {RATIONAL_NGCD(0, -1, -_RATLONG_MAX_),
        RATIONAL_NGCD(0, 1, _RATLONG_MAX_), 0},
       {RATIONAL_NGCD(0, _RATLONG_MAX_, -_RATLONG_MAX_),
        RATIONAL_NGCD(0, -_RATLONG_MAX_, _RATLONG_MAX_), 0},
       {RATIONAL_NGCD(0, -_RATLONG_MAX_, -_RATLONG_MAX_),
        RATIONAL_NGCD(0, _RATLONG_MAX_, _RATLONG_MAX_), 0},
       {RATIONAL_NGCD(0, -_RATLONG_MAX_, _RATLONG_MAX_),
        RATIONAL_NGCD(0, _RATLONG_MAX_, _RATLONG_MAX_), -1},
       {RATIONAL_NGCD(0, _RATLONG_MAX_, _RATLONG_MAX_),
        RATIONAL_NGCD(0, -_RATLONG_MAX_, _RATLONG_MAX_), 1},
       {RATIONAL_NGCD(0, _RATLONG_MAX_, -1),
        RATIONAL_NGCD(0, -_RATLONG_MAX_, 1), 0},
       {RATIONAL_NGCD(0, -_RATLONG_MAX_, -1),
        RATIONAL_NGCD(0, _RATLONG_MAX_, 1), 0},
       {RATIONAL_NGCD(0, -_RATLONG_MAX_, 1), RATIONAL_NGCD(0, _RATLONG_MAX_, 1),
        -1},
       {RATIONAL_NGCD(0, _RATLONG_MAX_, 1), RATIONAL_NGCD(0, -_RATLONG_MAX_, 1),
        1},
       {RATIONAL_NGCD(0, 1, _RATLONG_MAX_), RATIONAL_ZERO, 1},
       {RATIONAL_NGCD(0, -1, _RATLONG_MAX_), RATIONAL_ZERO, -1},
       {RATIONAL_NGCD(42, 1, 42), RATIONAL_ZERO, 1},
       {RATIONAL_NGCD(-42, -1, 42), RATIONAL_ZERO, -1},
       {RATIONAL_ZERO, RATIONAL_NGCD(42, 1, 42), -1},
       {RATIONAL_ZERO, RATIONAL_NGCD(-42, -1, 42), 1},
       {RATIONAL_NGCD(42, 1, 42), RATIONAL_NGCD(42, 1, 42), 0},
       {RATIONAL_NGCD(-42, -1, 42), RATIONAL_NGCD(-42, -1, 42), 0},
       {RATIONAL_NGCD(42, 1, 42), RATIONAL_NGCD(-42, -1, 42), 1},
       {RATIONAL_NGCD(-42, -1, 42), RATIONAL_NGCD(42, 1, 42), -1},
   };
   const size_t size = sizeof(vals) / sizeof(struct ratcond_tuple);
   return cr_make_param_array(struct ratcond_tuple, vals, size);
}

ParameterizedTest(struct ratcond_tuple *val, SUITE_NAME, conditional)
{
   char a_str[RATIONAL_STR_LEN] = {'\0'}, b_str[RATIONAL_STR_LEN] = {'\0'};
   rat2str(val->a, a_str);
   rat2str(val->b, b_str);

   if (val->iscond == 0)
      cr_expect(isequal_rational(val->a, val->b),
                "(%s != %s) when they should be equal", a_str, b_str);
   else
      cr_expect(not(isequal_rational(val->a, val->b)),
                "(%s == %s) when they should not be equal", a_str, b_str);

   if (val->iscond == -1)
      cr_expect(isless_rational(val->a, val->b),
                "(%s < %s) is not true, when it should be false.", a_str,
                b_str);
   else
      cr_expect(not(isless_rational(val->a, val->b)),
                "(%s < %s) is not false, when it should be true.", a_str,
                b_str);

   if (val->iscond == 1)
      cr_expect(isgreater_rational(val->a, val->b),
                "(%s > %s) is not true, when it should be false.", a_str,
                b_str);
   else
      cr_expect(not(isgreater_rational(val->a, val->b)),
                "(%s > %s) is not false, when it should be true.", a_str,
                b_str);
}

/* Test math operation properties */
//*** Reduction
TheoryDataPoints(SUITE_NAME, reduce) = {RATIONAL_DATAPOINTS};

Theory((Rational * a), SUITE_NAME, reduce)
{
   char a_str[RATIONAL_STR_LEN] = {'\0'};
   rat2str(*a, a_str);
   Rational c;
   char c_str[RATIONAL_STR_LEN] = {'\0'};

   double a_dbl = rational2double(*a);

   // reduction invariance
   memcpy(&c, a, sizeof(Rational));
   c = ReduceRational(c);
   rat2str(c, c_str);
   cr_expect(isequal_rational(*a, c), "reducing %s to %s is not equal", a_str,
             c_str);
}

//*** Addition
TheoryDataPoints(SUITE_NAME, add) = {RATIONAL_DATAPOINTS, RATIONAL_DATAPOINTS};

Theory((Rational * a, Rational *b), SUITE_NAME, add)
{
   char a_str[RATIONAL_STR_LEN] = {'\0'}, b_str[RATIONAL_STR_LEN] = {'\0'};
   rat2str(*a, a_str);
   rat2str(*b, b_str);

   const double a_dbl = rational2double(*a);
   const double b_dbl = rational2double(*b);

   Rational a_red, b_red;
   memcpy(&a_red, a, sizeof(Rational));
   memcpy(&b_red, b, sizeof(Rational));
   a_red = ReduceRational(a_red);
   b_red = ReduceRational(b_red);

   cr_assert(isequal_rational(*a, a_red) && isequal_rational(*b, b_red),
             "reducing %s and %s is not equal and did not preserve them", a_str,
             b_str);

   // commutative addition
   cr_expect(isequal_rational(
                 ToRational(RationalAdd(ToRationalLL(*a), ToRationalLL(*b))),
                 ToRational(RationalAdd(ToRationalLL(*b), ToRationalLL(*a)))),
             "(%s) + (%s) != (%s) + (%s)", a_str, b_str, b_str, a_str);

   // TODO: limit_denominator means isequal won't work in general
   // addition inversion
   Rational rat_check = ToRational(RationalAdd(
       RationalSub(ToRationalLL(*a), ToRationalLL(*b)), ToRationalLL(*b)));
   if (ABS(a_red.den) > (_RATLONG_MAX_ >> 8) ||
       ABS(b_red.den) > (_RATLONG_MAX_ >> 8)) {
      double check = rational2double(rat_check);
      if (a_dbl == 0 || check == 0) {
         cr_expect(epsilon_eq(dbl, a_dbl, check, DBL_THRESH),
                   "Rational addition is not approximately invertible (a=(%s), "
                   "b=(%s), err=%le)",
                   a_str, b_str, fabs(a_dbl - check));
      }
      else {
         cr_expect(ieee_ulp_eq(dbl, a_dbl, check, ULP_THRESH),
                   "Rational addition is not approximately invertible (a=(%s), "
                   "b=(%s), err=%le)",
                   a_str, b_str, fabs(a_dbl - check));
      }
   }
   else {
      double error = a_dbl - rational2double(rat_check);
      cr_expect(isequal_rational(*a, rat_check),
                "Rational addition is not invertible (a=(%s), "
                "b=(%s), err=%le)",
                a_str, b_str, fabs(error));
   }
}

//*** Multiplication
TheoryDataPoints(SUITE_NAME, mult) = {RATIONAL_DATAPOINTS, RATIONAL_DATAPOINTS};

Theory((Rational * a, Rational *b), SUITE_NAME, mult)
{
   char a_str[RATIONAL_STR_LEN] = {'\0'}, b_str[RATIONAL_STR_LEN] = {'\0'};
   rat2str(*a, a_str);
   rat2str(*b, b_str);

   const double a_dbl = rational2double(*a);
   const double b_dbl = rational2double(*b);

   Rational a_red, b_red;
   memcpy(&a_red, a, sizeof(Rational));
   memcpy(&b_red, b, sizeof(Rational));
   a_red = ReduceRational(a_red);
   b_red = ReduceRational(b_red);

   cr_assert(isequal_rational(*a, a_red) && isequal_rational(*b, b_red),
             "reducing %s and %s is not equal did not preserve them", a_str,
             b_str);

   // commutative multiplication
   cr_expect(isequal_rational(
                 ToRational(RationalMult(ToRationalLL(*a), ToRationalLL(*b))),
                 ToRational(RationalMult(ToRationalLL(*b), ToRationalLL(*a)))),
             "(%s) * (%s) != (%s) * (%s)", a_str, b_str, b_str, a_str);

   // TODO: limit_denominator means isequal won't work in general
   //  multiplication inversion
   cr_assume(!isequal_rational(*b, RATIONAL_ZERO));
   Rational rat_check = ToRational(RationalMult(
       RationalDivide(ToRationalLL(*a), ToRationalLL(*b)), ToRationalLL(*b)));
   if (ABS(a_red.den) > (_RATLONG_MAX_ >> 8) ||
       ABS(b_red.num) > (_RATLONG_MAX_ >> 8) ||
       ABS(b_red.whole) > (_RATLONG_MAX_ >> 8)) {
      double check = rational2double(rat_check);
      if (a_dbl == 0 || check == 0) {
         cr_expect(epsilon_eq(dbl, a_dbl, check, DBL_THRESH),
                   "Rational multiplication is not approximately invertible "
                   "(a=(%s), b=(%s), err=%le)",
                   a_str, b_str, fabs(a_dbl - check));
      }
      else {
         cr_expect(ieee_ulp_eq(dbl, a_dbl, check, ULP_THRESH),
                   "Rational multiplication is not approximately invertible "
                   "(a=(%s), b=(%s), err=%le)",
                   a_str, b_str, fabs(a_dbl - check));
      }
   }
   else {
      double error = a_dbl - rational2double(rat_check);
      cr_expect(isequal_rational(*a, rat_check),
                "Rational multiplication is not invertible (a=(%s), "
                "b=(%s), err=%le)",
                a_str, b_str, fabs(error));
   }
}

//*** rational2double inverse of double2rational
TheoryDataPoints(SUITE_NAME, ratdblinv) = {RATIONAL_DATAPOINTS};

Theory((Rational * a), SUITE_NAME, ratdblinv)
{
   char a_str[RATIONAL_STR_LEN] = {'\0'}, ret_str[RATIONAL_STR_LEN] = {'\0'};
   rat2str(*a, a_str);

   double a_dbl      = rational2double(*a);
   Rational returned = double2rational(a_dbl);
   double ret_dbl    = rational2double(returned);
   rat2str(returned, ret_str);
   Rational rat_err =
       ToRational(RationalSub(ToRationalLL(*a), ToRationalLL(returned)));
   double err = rational2double(rat_err);

   if (isequal_rational(returned, RATIONAL_ZERO)) {
      cr_expect(epsilon_eq(dbl, a_dbl, ret_dbl, DBL_EPSILON),
                "double2rational does not approximately invert rational2double "
                "for %s (double: %le, returned: %s, err: %le)",
                a_str, a_dbl, ret_str, err);
   }
   else {
      cr_expect(ieee_ulp_eq(dbl, a_dbl, ret_dbl, ULP_THRESH),
                "double2rational does not approximately invert rational2double "
                "for %s (double: %le, returned: %s, err: %le)",
                a_str, a_dbl, ret_str, err);
   }
}

/* Test rational2double and double2rational */
struct ratdbl_tuple {
   Rational rat;
   double dbl;
   int isequal;
};

ParameterizedTestParameters(SUITE_NAME, dbl)
{
   static struct ratdbl_tuple vals[] = {
       {RATIONAL_NGCD(0, 0, 1), 0, 1},
       {RATIONAL_NGCD(1, 0, 1), 1, 1},
       {RATIONAL_NGCD(-1, 0, 1), -1, 1},
       {RATIONAL_NGCD(-1, 0, 1), 1, 0},
       {RATIONAL_NGCD(1, 0, 1), -1, 0},
       {RATIONAL_NGCD(0, 1, 10), 0.1, 1},
       {RATIONAL_NGCD(32, 184, 1000), 32.184, 1},
       {RATIONAL_NGCD(32, 23, 125), 32.184, 1},
       {RATIONAL_NGCD(-32, -184, 1000), -32.184, 1},
       {RATIONAL_NGCD(-32, -23, 125), -32.184, 1},
       {RATIONAL_NGCD(3, 1415926535, 10000000000), 3.1415926535, 1},
       {RATIONAL_NGCD(3, 283185307, 2000000000), 3.1415926535, 1},
       {RATIONAL_NGCD(-3, -1415926535, 10000000000), -3.1415926535, 1},
       {RATIONAL_NGCD(-3, -283185307, 2000000000), -3.1415926535, 1},
       {RATIONAL_NGCD(42, 1, 42), 42.023809523809523808, 1},
       {RATIONAL_NGCD(-42, -1, 42), -42.023809523809523808, 1},
   };
   const size_t size = sizeof(vals) / sizeof(struct ratdbl_tuple);
   return cr_make_param_array(struct ratdbl_tuple, vals, size);
}

ParameterizedTest(struct ratdbl_tuple *val, SUITE_NAME, dbl)
{
   char ratstr[RATIONAL_STR_LEN] = {'\0'};
   rat2str(val->rat, ratstr);
   double rat_dbl = rational2double(val->rat);

   double check = val->dbl - rational2double(val->rat);
   if (val->isequal) {
      cr_expect(
          ieee_ulp_eq(dbl, val->dbl, rational2double(val->rat), ULP_THRESH),
          "%le - rational2double(%s) has an error of %le, larger than "
          "the threshold of %i ULP when it should be smaller.",
          val->dbl, ratstr, fabs(check), ULP_THRESH);
   }
   else {
      cr_expect(
          ieee_ulp_ne(dbl, val->dbl, rational2double(val->rat), ULP_THRESH),
          "%le - rational2double(%s) has an error of %le, smaller than "
          "the threshold of %i ULP when it should be larger.",
          val->dbl, ratstr, fabs(check), ULP_THRESH);
   }

   // not all tested numbers are exact for doubles in the first place
   double thresh    = ULP_THRESH * (nextafter(val->dbl, INFINITY) - val->dbl);
   Rational dbl_rat = double2rational(val->dbl);
   double dbl_conv  = rational2double(dbl_rat);
   Rational ratchk =
       ToRational(RationalSub(ToRationalLL(dbl_rat), ToRationalLL(val->rat)));
   char ratchk_str[RATIONAL_STR_LEN];
   rat2str(ratchk, ratchk_str);
   check = rational2double(ratchk);
   if (val->isequal) {
      cr_expect(epsilon_eq(dbl, check, 0, thresh),
                "double2rational(%le) - %s has an error of %s (%le), larger "
                "than the threshold of %le (%i eps) when it should be smaller.",
                val->dbl, ratstr, ratchk_str, fabs(check), thresh, ULP_THRESH);
   }
   else {
      cr_expect(epsilon_ne(dbl, check, 0, thresh),
                "double2rational(%le) - %s has an error of %s (%le), smaller "
                "than the threshold of %le (%i eps) when it should be larger.",
                val->dbl, ratstr, ratchk_str, fabs(check), thresh, ULP_THRESH);
   }
}

/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "42constants.h"
#include "mathkit.h"
#include <criterion/criterion.h>
#include <criterion/new/assert.h>
#include <criterion/parameterized.h>
#include <criterion/theories.h>
#include <stdio.h>

#define ULP_THRESH (4) // acceptable Units in Last Place variation
#define DBL_THRESH (ULP_THRESH * __DBL_EPSILON__)

#define MAT3STRLEN (512)
static inline void mat3x32str(mat3x3_t a, const char *whitespace,
                              char str[MAT3STRLEN])
{
   char dummy[MAT3STRLEN];
   sprintf(str, "[");
   for (int i = 0; i < 3; i++) {
      if (i > 0) {
         sprintf(dummy, "%s  ", whitespace);
         strcat(str, dummy);
      }
      sprintf(dummy, "[%+.16le, %+.16le, %+.16le]", a.rows[i].x, a.rows[i].y,
              a.rows[i].z);
      strcat(str, dummy);
      if (i < 2)
         strcat(str, ",\n");
   }
   strcat(str, "]");
}

#define SQRT3_2 (SQRTTHREE / 2.0)

#define mat3x3_ieee_ulp_eq(a, b, thres)                                        \
   all(ieee_ulp_eq(dbl, (a).x.x, (b).x.x, thres),                              \
       ieee_ulp_eq(dbl, (a).x.y, (b).x.y, thres),                              \
       ieee_ulp_eq(dbl, (a).x.z, (b).x.z, thres),                              \
       ieee_ulp_eq(dbl, (a).y.x, (b).y.x, thres),                              \
       ieee_ulp_eq(dbl, (a).y.y, (b).y.y, thres),                              \
       ieee_ulp_eq(dbl, (a).y.z, (b).y.z, thres),                              \
       ieee_ulp_eq(dbl, (a).z.x, (b).z.x, thres),                              \
       ieee_ulp_eq(dbl, (a).z.y, (b).z.y, thres),                              \
       ieee_ulp_eq(dbl, (a).z.z, (b).z.z, thres))

#define mat3x3_epsilon_eq(a, b, thres)                                         \
   all(epsilon_eq(dbl, (a).x.x, (b).x.x, thres),                               \
       epsilon_eq(dbl, (a).x.y, (b).x.y, thres),                               \
       epsilon_eq(dbl, (a).x.z, (b).x.z, thres),                               \
       epsilon_eq(dbl, (a).y.x, (b).y.x, thres),                               \
       epsilon_eq(dbl, (a).y.y, (b).y.y, thres),                               \
       epsilon_eq(dbl, (a).y.z, (b).y.z, thres),                               \
       epsilon_eq(dbl, (a).z.x, (b).z.x, thres),                               \
       epsilon_eq(dbl, (a).z.y, (b).z.y, thres),                               \
       epsilon_eq(dbl, (a).z.z, (b).z.z, thres))

#define mat3x3_iden_epsilon_eq(a, ulpthresh)                                   \
   all(ieee_ulp_eq(dbl, (a).x.x, 1.0, (ulpthresh)),                            \
       epsilon_eq(dbl, (a).x.y, 0.0, (ulpthresh) * __DBL_EPSILON__),           \
       epsilon_eq(dbl, (a).x.z, 0.0, (ulpthresh) * __DBL_EPSILON__),           \
       epsilon_eq(dbl, (a).y.x, 0.0, (ulpthresh) * __DBL_EPSILON__),           \
       ieee_ulp_eq(dbl, (a).y.y, 1.0, (ulpthresh)),                            \
       epsilon_eq(dbl, (a).y.z, 0.0, (ulpthresh) * __DBL_EPSILON__),           \
       epsilon_eq(dbl, (a).z.x, 0.0, (ulpthresh) * __DBL_EPSILON__),           \
       epsilon_eq(dbl, (a).z.y, 0.0, (ulpthresh) * __DBL_EPSILON__),           \
       ieee_ulp_eq(dbl, (a).z.z, 1.0, (ulpthresh)))

#define MAT3X3_DATAPOINTS                                                      \
   DataPoints(                                                                 \
       mat3x3_t *, &MAT3X3_ZERO, &MAT3X3_EYE, &MAT3X3_ONES,                    \
       &MAT3X3_SETROWS(VEC3_NXAXIS, VEC3_NYAXIS, VEC3_NZAXIS),                 \
       &MAT3X3_SET(-1, -1, -1, -1, -1, -1, -1, -1, -1),                        \
       &MAT3X3_SETROWS(VEC3_NXAXIS, VEC3_NYAXIS, VEC3_NZAXIS),                 \
                                                                               \
       &MAT3X3_SETROWS(VEC3_ONES, VEC3_ZERO, VEC3_ZERO),                       \
       &MAT3X3_SETROWS(VEC3_ZERO, VEC3_ONES, VEC3_ZERO),                       \
       &MAT3X3_SETROWS(VEC3_ZERO, VEC3_ZERO, VEC3_ONES),                       \
       &MAT3X3_SETROWS(VEC3_ONES, VEC3_ONES, VEC3_ZERO),                       \
       &MAT3X3_SETROWS(VEC3_ONES, VEC3_ZERO, VEC3_ONES),                       \
       &MAT3X3_SETROWS(VEC3_ZERO, VEC3_ONES, VEC3_ONES),                       \
                                                                               \
       &MAT3X3_SET(-1, -1, -1, 0, 0, 0, 0, 0, 0),                              \
       &MAT3X3_SET(0, 0, 0, -1, -1, -1, 0, 0, 0),                              \
       &MAT3X3_SET(0, 0, 0, 0, 0, 0, -1, -1, -1),                              \
       &MAT3X3_SET(-1, -1, -1, -1, -1, -1, 0, 0, 0),                           \
       &MAT3X3_SET(-1, -1, -1, 0, 0, 0, -1, -1, -1),                           \
       &MAT3X3_SET(0, 0, 0, -1, -1, -1, -1, -1, -1),                           \
                                                                               \
       &MAT3X3_SETROWS(VEC3_PXAXIS, VEC3_PZAXIS, VEC3_ZERO),                   \
       &MAT3X3_SETROWS(VEC3_PXAXIS, VEC3_ZERO, VEC3_PYAXIS),                   \
       &MAT3X3_SETROWS(VEC3_PYAXIS, VEC3_PZAXIS, VEC3_ZERO),                   \
       &MAT3X3_SETROWS(VEC3_PYAXIS, VEC3_ZERO, VEC3_PXAXIS),                   \
       &MAT3X3_SETROWS(VEC3_PZAXIS, VEC3_PYAXIS, VEC3_ZERO),                   \
       &MAT3X3_SETROWS(VEC3_PZAXIS, VEC3_ZERO, VEC3_PXAXIS),                   \
                                                                               \
       &MAT3X3_SETROWS(VEC3_PXAXIS, VEC3_PZAXIS, VEC3_PYAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PZAXIS, VEC3_PYAXIS, VEC3_PXAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PYAXIS, VEC3_PXAXIS, VEC3_PZAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PZAXIS, VEC3_PXAXIS, VEC3_PYAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PYAXIS, VEC3_PZAXIS, VEC3_PZAXIS),                 \
                                                                               \
       &MAT3X3_SETROWS(VEC3_NXAXIS, VEC3_PZAXIS, VEC3_PYAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PZAXIS, VEC3_NYAXIS, VEC3_PXAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PYAXIS, VEC3_PXAXIS, VEC3_NZAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PZAXIS, VEC3_PXAXIS, VEC3_NYAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PYAXIS, VEC3_NZAXIS, VEC3_PYAXIS),                 \
                                                                               \
       &MAT3X3_SETROWS(VEC3_PZAXIS, VEC3_NXAXIS, VEC3_NYAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PYAXIS, VEC3_NZAXIS, VEC3_NZAXIS),                 \
       &MAT3X3_SETROWS(VEC3_NZAXIS, VEC3_PXAXIS, VEC3_NYAXIS),                 \
       &MAT3X3_SETROWS(VEC3_NYAXIS, VEC3_PZAXIS, VEC3_NZAXIS),                 \
                                                                               \
       &MAT3X3_SETROWS(VEC3_NZAXIS, VEC3_NXAXIS, VEC3_NYAXIS),                 \
       &MAT3X3_SETROWS(VEC3_NYAXIS, VEC3_NZAXIS, VEC3_NZAXIS),                 \
                                                                               \
       &MAT3X3_SETROWS(VEC3_PZAXIS, VEC3_PYAXIS, VEC3_NXAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PZAXIS, VEC3_NYAXIS, VEC3_PXAXIS),                 \
                                                                               \
       &MAT3X3_SET(1, 0, 0, 0, SQRTHALF, -SQRTHALF, 0, SQRTHALF, SQRTHALF),    \
       &MAT3X3_SET(SQRTHALF, 0, SQRTHALF, 0, 1, 0, -SQRTHALF, 0, SQRTHALF),    \
       &MAT3X3_SET(SQRTHALF, -SQRTHALF, 0, SQRTHALF, SQRTHALF, 0, 0, 0, 1),    \
       &MAT3X3_SET(1, 0, 0, 0, SQRTHALF, SQRTHALF, 0, -SQRTHALF, SQRTHALF),    \
       &MAT3X3_SET(SQRTHALF, 0, -SQRTHALF, 0, 1, 0, SQRTHALF, 0, SQRTHALF),    \
       &MAT3X3_SET(SQRTHALF, SQRTHALF, 0, -SQRTHALF, SQRTHALF, 0, 0, 0, 1),    \
                                                                               \
       &MAT3X3_SET(1, 0, 0, 0, SQRT3_2, -0.5, 0, 0.5, SQRT3_2),                \
       &MAT3X3_SET(SQRT3_2, 0, 0.5, 0, 1, 0, -0.5, 0, SQRT3_2),                \
       &MAT3X3_SET(SQRT3_2, -0.5, 0, 0.5, SQRT3_2, 0, 0, 0, 1),                \
       &MAT3X3_SET(1, 0, 0, 0, 0.5, SQRT3_2, 0, -SQRT3_2, 0.5),                \
       &MAT3X3_SET(0.5, 0, -SQRT3_2, 0, 1, 0, SQRT3_2, 0, 0.5),                \
       &MAT3X3_SET(0.5, SQRT3_2, 0, -SQRT3_2, 0.5, 0, 0, 0, 1),                \
                                                                               \
       &MAT3X3_SET(2, 7, 6, 9, 5, 1, 4, 3, 8),                                 \
       &MAT3X3_SET(9, 5, 1, 2, 7, 6, 4, 3, 8),                                 \
       &MAT3X3_SET(2, 9, 4, 7, 5, 3, 6, 1, 8),                                 \
       &MAT3X3_SET(7, 5, 3, 2, 9, 4, 6, 1, 8),                                 \
                                                                               \
       &MAT3X3_SET(-2, 7, -6, 9, -5, 1, -4, 3, -8),                            \
       &MAT3X3_SET(9, -5, 1, -2, 7, -6, -4, 3, -8),                            \
       &MAT3X3_SET(-2, 9, -4, 7, -5, 3, -6, 1, -8),                            \
       &MAT3X3_SET(7, -5, 3, -2, 9, -4, -6, 1, -8),                            \
                                                                               \
       &MAT3X3_SET(2, -7, 6, -9, 5, -1, 4, -3, 8),                             \
       &MAT3X3_SET(-9, 5, -1, 2, -7, 6, 4, -3, 8),                             \
       &MAT3X3_SET(2, -9, 4, -7, 5, -3, 6, -1, 8),                             \
       &MAT3X3_SET(-7, 5, -3, 2, -9, 4, 6, -1, 8),                             \
                                                                               \
       &MAT3X3_SET(-2, -7, -6, -9, -5, -1, -4, -3, -8),                        \
       &MAT3X3_SET(-9, -5, -1, -2, -7, -6, -4, -3, -8),                        \
       &MAT3X3_SET(-2, -9, -4, -7, -5, -3, -6, -1, -8),                        \
       &MAT3X3_SET(-7, -5, -3, -2, -9, -4, -6, -1, -8),                        \
       &MAT3X3_SET(1, 2, 3, 4, 5, 6, 7, 8, 9),                                 \
       &MAT3X3_SET(1, -2, 3, -4, 5, -6, 7, -8, 9),                             \
       &MAT3X3_SET(-1, 2, -3, 4, -5, 6, -7, 8, -9),                            \
       &MAT3X3_SET(1, 4, 7, 2, 5, 8, 3, 6, 9),                                 \
       &MAT3X3_SET(1, -4, 7, -2, 5, -8, 3, -6, 9),                             \
       &MAT3X3_SET(-1, 4, -7, 2, -5, 8, -3, 6, -9), )

/**********************************************************************/
#define SUITE_NAME mathkit_mat3x3

// Testing stringify
TheoryDataPoints(SUITE_NAME, tostr) = {MAT3X3_DATAPOINTS};
Theory((mat3x3_t * ap), SUITE_NAME, tostr)
{
   const mat3x3_t a      = *ap;
   char astr[MAT3STRLEN] = {'\0'};
   mat3x32str(a, "\t    ", astr);
   mat3x3_t b;

   sscanf(astr, "[[%lf, %lf, %lf], [%lf, %lf, %lf], [%lf, %lf, %lf]]", &b.x.x,
          &b.x.y, &b.x.z, &b.y.x, &b.y.y, &b.y.z, &b.z.x, &b.z.y, &b.z.z);
   cr_expect(mat3x3_ieee_ulp_eq(a, b, ULP_THRESH),
             "3x3 Matrix stringify did not preseve the vector with "
             "param:\n\ta = %s",
             astr);
}

// only conditional currently is equivalence
TheoryDataPoints(SUITE_NAME, conditional) = {MAT3X3_DATAPOINTS,
                                             MAT3X3_DATAPOINTS};
Theory((mat3x3_t * ap, mat3x3_t *bp), SUITE_NAME, conditional)
{
   const mat3x3_t a = *ap, b = *bp;
   char astr[MAT3STRLEN] = {'\0'}, bstr[MAT3STRLEN] = {'\0'};
   mat3x32str(a, "\t    ", astr);
   mat3x32str(b, "\t    ", bstr);

   if (a.x.x == b.x.x && a.x.y == b.x.y && a.x.z == b.x.z && a.y.x == b.y.x &&
       a.y.y == b.y.y && a.y.z == b.y.z && a.z.x == b.z.x && a.z.y == b.z.y &&
       a.z.z == b.z.z) {
      cr_assert(_isequal_mat3x3(a, b),
                "3x3 Matrix 'a' and 'b' were not equal when they should be "
                "equal with params:\n\ta = %s\n\tb = %s",
                astr, bstr);
   }
   else {
      cr_assert(not(_isequal_mat3x3(a, b)),
                "3x3 Matrix 'a' and 'b' were equal when they should not be "
                "equal with params:\n\ta = %s\n\tb = %s",
                astr, bstr);
   }
}
#undef SUITE_NAME
/**********************************************************************/

/**********************************************************************/
// .. START UNARY FUNCTION TESTS
/**********************************************************************/
#define SUITE_NAME mathkit_mat3x3_unary
// Matrix transpose
TheoryDataPoints(SUITE_NAME, transpose) = {MAT3X3_DATAPOINTS};
Theory((mat3x3_t * ap), SUITE_NAME, transpose)
{
   const mat3x3_t a      = *ap;
   const mat3x3_t aT     = MT(a);
   char astr[MAT3STRLEN] = {'\0'};
   mat3x32str(a, "\t    ", astr);
   mat3x3_t b, eyecheck;

   // transpose is its own inverse function
   cr_assert(
       mat3x3_ieee_ulp_eq(a, MT(aT), ULP_THRESH),
       "Matrix transpose is not its own inverse function with param:\n\ta = %s",
       astr);
}

// Matrix inverse
TheoryDataPoints(SUITE_NAME, inverse) = {MAT3X3_DATAPOINTS};
Theory((mat3x3_t * ap), SUITE_NAME, inverse)
{
   const mat3x3_t a      = *ap;
   const mat3x3_t aT     = MT(a);
   char astr[MAT3STRLEN] = {'\0'}, atstr[MAT3STRLEN] = {'\0'},
        ainvstr[MAT3STRLEN] = {'\0'}, outstr[MAT3STRLEN] = {'\0'};
   mat3x3_t b, out;
   const mat3x3_t eye = MAT3X3_EYE;

   // matrix inverse
   const double deta = det3x3(a);
   cr_assume(fabs(det3x3(a)) > __DBL_EPSILON__);
   mat3x3_t ainv = MINV3(a);
   out           = MxM(a, ainv);
   mat3x32str(a, "\t       ", astr);
   mat3x32str(ainv, "\t       ", ainvstr);
   mat3x32str(out, "\t       ", ainvstr);
   cr_assert(
       mat3x3_iden_epsilon_eq(out, ULP_THRESH),
       "Multiplying with the precomputed inverse does not result in identity "
       "matrix with params:\n\ta    = %s\n\tainv = %s\n\tout  = %s",
       astr, ainvstr, outstr);

   MINVxM3(a, 3, eye.rows, ainv.rows);
   mat3x32str(a, "\t      ", astr);
   mat3x32str(out, "\t      ", ainvstr);
   out = MxMT(a, ainv);
   cr_assert(mat3x3_iden_epsilon_eq(out, ULP_THRESH),
             "Using MINVxM3 with an identity matrix argument did not result in "
             "identity with params:\n\ta   = %s\n\tout = %s",
             astr, outstr);
}

// unary rotation matrix
TheoryDataPoints(SUITE_NAME, unrotation) = {MAT3X3_DATAPOINTS};
Theory((mat3x3_t * ap), SUITE_NAME, unrotation)
{
   const mat3x3_t a      = *ap;
   const mat3x3_t aT     = MT(a);
   char astr[MAT3STRLEN] = {'\0'}, Cstr[MAT3STRLEN] = {'\0'},
        Rstr[MAT3STRLEN] = {'\0'}, RTstr[MAT3STRLEN] = {'\0'},
        Rinvstr[MAT3STRLEN] = {'\0'}, outstr[MAT3STRLEN] = {'\0'};
   mat3x32str(a, "\t    ", astr);
   mat3x3_t b, eyecheck, out;

   // get a rotation matrix from first 2 rows via Rodrigues' formula
   const double magx = MAGV(a.x), magy = MAGV(a.y);
   cr_assume(magx > 0 && magy > 0);
   mat3x3_t C          = MAT3X3_SETROWS(UNITV(a.x).v, UNITV(a.y).v, VEC3_ZERO);
   const double absxoy = fabs(VoV(C.x, C.y));
   cr_assume(absxoy < 0.9999999);

   mat3x3_t R  = RodriguesRotation(C.x, C.y);
   mat3x3_t RT = MT(R), Rinv = MINV3(R);

   // inverse is transpose matrix
   // TODO: surprisingly, transpose does worse than the inverse calculation
   mat3x32str(a, "\t        ", astr);
   mat3x32str(C, "\t        ", Cstr);
   mat3x32str(R, "\t        ", Rstr);
   mat3x32str(RT, "\t        ", RTstr);
   mat3x32str(out, "\t        ", outstr);
   cr_expect(mat3x3_iden_epsilon_eq(MxM(R, RT), ULP_THRESH + 2),
             "Rotation matrix multiplied with its transpose does not result in "
             "identity with params:\n\ta      = %s\n\tdet(a) = %le\n\tC      = "
             "%s\n\tR      = %s\n\tRT     = %s\n\tout    = %s",
             astr, Cstr, Rstr, RTstr, outstr);

   // inverse is inverse matrix
   mat3x32str(a, "\t           ", astr);
   mat3x32str(C, "\t           ", Cstr);
   mat3x32str(R, "\t           ", Rstr);
   mat3x32str(Rinv, "\t           ", Rinvstr);
   mat3x32str(out, "\t           ", outstr);
   cr_expect(mat3x3_iden_epsilon_eq(MxM(R, Rinv), ULP_THRESH),
             "Rotation matrix multiplied with its inverse does not result in "
             "identity with params:\n\ta      = %s\n\tdet(a) = %le\n\tC      = "
             "%s\n\tR      = %s\n\tRT     = %s\n\tout    = %s",
             astr, Cstr, Rstr, Rinvstr, outstr);
}
#undef SUITE_NAME
/**********************************************************************/
// .. END UNARY FUNCTION TESTS
/**********************************************************************/
/**********************************************************************/
// .. START BINARY FUNCTION TESTS
/**********************************************************************/
#define SUITE_NAME mathkit_mat3x3_binary
#undef SUITE_NAME
/**********************************************************************/
// .. END BINARY FUNCTION TESTS
/**********************************************************************/

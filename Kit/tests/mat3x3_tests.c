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
#include <stdarg.h>
#include <stdio.h>

#define ULP_THRESH (4) // acceptable Units in Last Place variation
#define DBL_THRESH (ULP_THRESH * __DBL_EPSILON__)

#define VEC3STRLEN (128)
static inline void vec32str(vec3_t a, char str[VEC3STRLEN])
{
   sprintf(str, "[%+.16le, %+.16le, %+.16le]", a.x, a.y, a.z);
}

#define MAT3STRLEN (512)
static inline void mat3x32str(mat3x3_t a, const char *whitespace,
                              char str[MAT3STRLEN])
{
   char dummy[MAT3STRLEN];
   sprintf(str, "[");
   for (int i = 0; i < 3; i++) {
      if (i > 0) {
         sprintf(dummy, "%s ", whitespace);
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

#define vec3_ieee_ulp_eq(a, b, ulpthresh)                                      \
   all(ieee_ulp_eq(dbl, (a).x, (b).x, (ulpthresh)),                            \
       ieee_ulp_eq(dbl, (a).y, (b).y, (ulpthresh)),                            \
       ieee_ulp_eq(dbl, (a).z, (b).z, (ulpthresh)))

#define vec3_eps_vec_eq(a, b, vec_dblthres)                                    \
   all(epsilon_eq(dbl, (a).x, (b).x, (vec_dblthres).x),                        \
       epsilon_eq(dbl, (a).y, (b).y, (vec_dblthres).y),                        \
       epsilon_eq(dbl, (a).z, (b).z, (vec_dblthres).z))

#define vec3_eps_eq(a, b, ulpthresh, dblthresh)                                \
   all(epsilon_eq(dbl, (a).x, (b).x, (dblthresh)),                             \
       epsilon_eq(dbl, (a).y, (b).y, (dblthresh)),                             \
       epsilon_eq(dbl, (a).z, (b).z, (dblthresh)))

#define vec3_x_z_eq(a, b, ulpthresh, dblthresh)                                \
   all(epsilon_eq(dbl, (a).x, (b).x, (dblthresh)),                             \
       ieee_ulp_eq(dbl, (a).y, (b).y, (ulpthresh)),                            \
       ieee_ulp_eq(dbl, (a).z, (b).z, (ulpthresh)))
#define vec3_y_z_eq(a, b, ulpthresh, dblthresh)                                \
   all(ieee_ulp_eq(dbl, (a).x, (b).x, (ulpthresh)),                            \
       epsilon_eq(dbl, (a).y, (b).y, (dblthresh)),                             \
       ieee_ulp_eq(dbl, (a).z, (b).z, (ulpthresh)))
#define vec3_z_z_eq(a, b, ulpthresh, dblthresh)                                \
   all(ieee_ulp_eq(dbl, (a).x, (b).x, (ulpthresh)),                            \
       ieee_ulp_eq(dbl, (a).y, (b).y, (ulpthresh)),                            \
       epsilon_eq(dbl, (a).z, (b).z, (dblthresh)))
#define vec3_xy_z_eq(a, b, ulpthresh, dblthresh)                               \
   all(epsilon_eq(dbl, (a).x, (b).x, (dblthresh)),                             \
       epsilon_eq(dbl, (a).y, (b).y, (dblthresh)),                             \
       ieee_ulp_eq(dbl, (a).z, (b).z, (ulpthresh)))
#define vec3_xz_z_eq(a, b, ulpthresh, dblthresh)                               \
   all(epsilon_eq(dbl, (a).x, (b).x, (dblthresh)),                             \
       ieee_ulp_eq(dbl, (a).y, (b).y, (ulpthresh)),                            \
       epsilon_eq(dbl, (a).z, (b).z, (dblthresh)))
#define vec3_yz_z_eq(a, b, ulpthresh, dblthresh)                               \
   all(ieee_ulp_eq(dbl, (a).x, (b).x, (ulpthresh)),                            \
       epsilon_eq(dbl, (a).y, (b).y, (dblthresh)),                             \
       epsilon_eq(dbl, (a).z, (b).z, (dblthresh)))

#define vec3_eq_zchk(a, b, ulpthresh, dblthresh, str)                          \
   do {                                                                        \
      if (_isequal_vec3((b), VEC3_ZERO))                                       \
         cr_expect(vec3_eps_eq((a), (b), (ulpthresh), (dblthresh)),            \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else if (_isequal_vec3((a), VEC3_ZERO))                                  \
         cr_expect(vec3_eps_eq((b), (a), (ulpthresh), (dblthresh)),            \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else if ((b).x == 0 && (b).y == 0)                                       \
         cr_expect(vec3_xy_z_eq((a), (b), (ulpthresh), (dblthresh)),           \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else if ((b).x == 0 && (b).z == 0)                                       \
         cr_expect(vec3_xz_z_eq((a), (b), (ulpthresh), (dblthresh)),           \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else if ((b).y == 0 && (b).z == 0)                                       \
         cr_expect(vec3_yz_z_eq((a), (b), (ulpthresh), (dblthresh)),           \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else if ((a).x == 0 && (a).y == 0)                                       \
         cr_expect(vec3_xy_z_eq((b), (a), (ulpthresh), (dblthresh)),           \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else if ((a).x == 0 && (a).z == 0)                                       \
         cr_expect(vec3_xz_z_eq((b), (a), (ulpthresh), (dblthresh)),           \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else if ((a).y == 0 && (a).z == 0)                                       \
         cr_expect(vec3_yz_z_eq((b), (a), (ulpthresh), (dblthresh)),           \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else if ((b).x == 0)                                                     \
         cr_expect(vec3_x_z_eq((a), (b), (ulpthresh), (dblthresh)),            \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else if ((b).y == 0)                                                     \
         cr_expect(vec3_y_z_eq((a), (b), (ulpthresh), (dblthresh)),            \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else if ((b).z == 0)                                                     \
         cr_expect(vec3_z_z_eq((a), (b), (ulpthresh), (dblthresh)),            \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else if ((a).x == 0)                                                     \
         cr_expect(vec3_x_z_eq((b), (a), (ulpthresh), (dblthresh)),            \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else if ((a).y == 0)                                                     \
         cr_expect(vec3_y_z_eq((a), (b), (ulpthresh), (dblthresh)),            \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else if ((a).z == 0)                                                     \
         cr_expect(vec3_z_z_eq((a), (b), (ulpthresh), (dblthresh)),            \
                   "%s\nulp = %i\neps = %le", (str), (ulpthresh),              \
                   (dblthresh));                                               \
      else                                                                     \
         cr_expect(vec3_ieee_ulp_eq((a), (b), (ulpthresh)), "%s\nulp = %i",    \
                   (str), (ulpthresh));                                        \
   } while (0)

#define VEC3_DATAPOINTS                                                        \
   DataPoints(vec3_t *, &VEC3_ZERO, &VEC3_PXAXIS, &VEC3_PYAXIS, &VEC3_PZAXIS,  \
              &VEC3_NXAXIS, &VEC3_NYAXIS, &VEC3_NZAXIS, &VEC3_ONES,            \
              &VEC3_SET(1234567.89, 0, 0), &VEC3_SET(0, 1234567.89, 0),        \
              &VEC3_SET(0, 0, 1234567.89), &VEC3_SET(-1234567.89, 0, 0),       \
              &VEC3_SET(0, -1234567.89, 0), &VEC3_SET(0, 0, -1234567.89),      \
              &VEC3_SET(0, 1234567.89, 1234567.89),                            \
              &VEC3_SET(1234567.89, 0, 1234567.89),                            \
              &VEC3_SET(1234567.89, 1234567.89, 0),                            \
              &VEC3_SET(0, 1234567.89, -1234567.89),                           \
              &VEC3_SET(1234567.89, 0, -1234567.89),                           \
              &VEC3_SET(1234567.89, -1234567.89, 0),                           \
              &VEC3_SET(0, -1234567.89, 1234567.89),                           \
              &VEC3_SET(-1234567.89, 0, 1234567.89),                           \
              &VEC3_SET(-1234567.89, 1234567.89, 0),                           \
              &VEC3_SET(0, -1234567.89, -1234567.89),                          \
              &VEC3_SET(-1234567.89, 0, -1234567.89),                          \
              &VEC3_SET(-1234567.89, -1234567.89, 0),                          \
              &VEC3_SET(1234567.89, 1234567.89, 1234567.89),                   \
              &VEC3_SET(1234567.89, 1234567.89, -1234567.89),                  \
              &VEC3_SET(1234567.89, -1234567.89, 1234567.89),                  \
              &VEC3_SET(-1234567.89, 1234567.89, 1234567.89),                  \
              &VEC3_SET(1234567.89, -1234567.89, -1234567.89),                 \
              &VEC3_SET(-1234567.89, 1234567.89, -1234567.89),                 \
              &VEC3_SET(-1234567.89, -1234567.89, 1234567.89),                 \
              &VEC3_SET(-1234567.89, -1234567.89, -1234567.89),                \
              &VEC3_SET(PI, 0, 0), &VEC3_SET(0, PI, 0), &VEC3_SET(0, 0, PI),   \
              &VEC3_SET(-PI, 0, 0), &VEC3_SET(0, -PI, 0),                      \
              &VEC3_SET(0, 0, -PI))

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

#define mat3x3_epsilon_mat_eq(a, b, thres)                                     \
   all(epsilon_eq(dbl, (a).x.x, (b).x.x, (thres).x.x),                         \
       epsilon_eq(dbl, (a).x.y, (b).x.y, (thres).x.y),                         \
       epsilon_eq(dbl, (a).x.z, (b).x.z, (thres).x.z),                         \
       epsilon_eq(dbl, (a).y.x, (b).y.x, (thres).y.x),                         \
       epsilon_eq(dbl, (a).y.y, (b).y.y, (thres).y.y),                         \
       epsilon_eq(dbl, (a).y.z, (b).y.z, (thres).y.z),                         \
       epsilon_eq(dbl, (a).z.x, (b).z.x, (thres).z.x),                         \
       epsilon_eq(dbl, (a).z.y, (b).z.y, (thres).z.y),                         \
       epsilon_eq(dbl, (a).z.z, (b).z.z, (thres).z.z))

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

#define mat3x3_iden_eq(a, ulpthresh, dblthresh)                                \
   all(ieee_ulp_eq(dbl, (a).x.x, 1.0, (ulpthresh)),                            \
       epsilon_eq(dbl, (a).x.y, 0.0, (dblthresh)),                             \
       epsilon_eq(dbl, (a).x.z, 0.0, (dblthresh)),                             \
       epsilon_eq(dbl, (a).y.x, 0.0, (dblthresh)),                             \
       ieee_ulp_eq(dbl, (a).y.y, 1.0, (ulpthresh)),                            \
       epsilon_eq(dbl, (a).y.z, 0.0, (dblthresh)),                             \
       epsilon_eq(dbl, (a).z.x, 0.0, (dblthresh)),                             \
       epsilon_eq(dbl, (a).z.y, 0.0, (dblthresh)),                             \
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
       &MAT3X3_SET(1, 0, 0, 0, 2, 0, 0, 0, 3),                                 \
       &MAT3X3_SET(0, 1, 2, 3, 0, -3, -2, -1, 0),                              \
       &MAT3X3_SET(1, 4, 5, 6, 2, 6, 5, 4, 3),                                 \
       &MAT3X3_SET(1, 4, 5, 6, 2, -6, -5, -4, 3),                              \
       &MAT3X3_SET(-1, 4, 5, 6, -2, 6, 5, 4, -3),                              \
       &MAT3X3_SET(-1, 4, 5, 6, -2, -6, -5, -4, -3),                           \
                                                                               \
       &MAT3X3_SET(1234567.89, 0, 0, 0, 2345678.91, 0, 0, 0, 34567891.23),     \
       &MAT3X3_SET(0, 1234567.89, 2345678.91, 34567891.23, 0, -34567891.23,    \
                   -2345678.91, -1234567.89, 0),                               \
       &MAT3X3_SET(1234567.89, 4567891.23, 5678912.34, 678912.34, 2345678.91,  \
                   678912.34, 5678912.34, 4567891.23, 34567891.23),            \
       &MAT3X3_SET(1234567.89, 4567891.23, 5678912.34, 678912.34, 2345678.91,  \
                   -678912.34, -5678912.34, -4567891.23, 34567891.23),         \
       &MAT3X3_SET(-1234567.89, 4567891.23, 5678912.34, 678912.34,             \
                   -2345678.91, 678912.34, 5678912.34, 4567891.23,             \
                   -34567891.23),                                              \
       &MAT3X3_SET(-1234567.89, 4567891.23, 5678912.34, 678912.34,             \
                   -2345678.91, -678912.34, -5678912.34, -4567891.23,          \
                   -34567891.23),                                              \
                                                                               \
       &MAT3X3_SET(PI, 0, 0, 0, PI, 0, 0, 0, PI),                              \
       &MAT3X3_SET(-PI, 0, 0, 0, -PI, 0, 0, 0, -PI),                           \
       &MAT3X3_SET(PI, 0, 0, 0, -PI, 0, 0, 0, PI),                             \
       &MAT3X3_SET(-PI, 0, 0, 0, PI, 0, 0, 0, -PI),                            \
                                                                               \
       &MAT3X3_SET(PI, HALFPI, SQRT3_2, D2R, PI, D2R, SQRT3_2, HALFPI, PI),    \
       &MAT3X3_SET(PI, HALFPI, SQRT3_2, D2R, PI, -D2R, -SQRT3_2, -HALFPI, PI), \
       &MAT3X3_SET(PI, -HALFPI, -SQRT3_2, -D2R, PI, -D2R, -SQRT3_2, -HALFPI,   \
                   PI),                                                        \
       &MAT3X3_SET(-PI, HALFPI, SQRT3_2, D2R, -PI, D2R, SQRT3_2, HALFPI, -PI), \
       &MAT3X3_SET(PI, HALFPI, SQRT3_2, D2R, -PI, D2R, SQRT3_2, HALFPI, PI),   \
       &MAT3X3_SET(PI, -HALFPI, -SQRT3_2, -D2R, -PI, D2R, SQRT3_2, HALFPI,     \
                   PI),                                                        \
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

#define ROT3_DATAPOINTS                                                        \
   DataPoints(                                                                 \
       mat3x3_t *, &MAT3X3_EYE,                                                \
       &MAT3X3_SETROWS(VEC3_NXAXIS, VEC3_NYAXIS, VEC3_PZAXIS),                 \
       &MAT3X3_SETROWS(VEC3_NXAXIS, VEC3_PYAXIS, VEC3_NZAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PXAXIS, VEC3_NYAXIS, VEC3_NZAXIS),                 \
                                                                               \
       &MAT3X3_SETROWS(VEC3_PYAXIS, VEC3_PZAXIS, VEC3_PXAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PZAXIS, VEC3_PXAXIS, VEC3_PYAXIS),                 \
                                                                               \
       &MAT3X3_SETROWS(VEC3_PXAXIS, VEC3_PZAXIS, VEC3_NYAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PXAXIS, VEC3_NZAXIS, VEC3_PYAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PYAXIS, VEC3_NXAXIS, VEC3_PZAXIS),                 \
       &MAT3X3_SETROWS(VEC3_NYAXIS, VEC3_PXAXIS, VEC3_PZAXIS),                 \
       &MAT3X3_SETROWS(VEC3_PZAXIS, VEC3_PYAXIS, VEC3_NXAXIS),                 \
       &MAT3X3_SETROWS(VEC3_NZAXIS, VEC3_PYAXIS, VEC3_PXAXIS),                 \
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
       &MAT3X3_SET(0.5, SQRT3_2, 0, -SQRT3_2, 0.5, 0, 0, 0, 1))

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
   char astr[MAT3STRLEN] = {'\0'}, ainvstr[MAT3STRLEN] = {'\0'},
        outstr[MAT3STRLEN] = {'\0'};
   mat3x3_t b, out;
   const mat3x3_t eye = MAT3X3_EYE;

   // matrix inverse
   const double deta = det3x3(a);
   cr_assume(fabs(det3x3(a)) > __DBL_EPSILON__);
   mat3x3_t ainv = MINV3(a);
   out           = MxM(a, ainv);
   mat3x32str(a, "\t       ", astr);
   mat3x32str(ainv, "\t       ", ainvstr);
   mat3x32str(out, "\t       ", outstr);
   cr_assert(
       mat3x3_iden_eq(out, 5 * ULP_THRESH, 5 * DBL_THRESH),
       "Multiplying with the precomputed inverse does not result in identity "
       "matrix with params:\n\ta    = %s\n\tainv = %s\n\tout  = %s",
       astr, ainvstr, outstr);

   MINVxM3(a, 3, eye.rows, ainv.rows);
   mat3x32str(a, "\t      ", astr);
   mat3x32str(out, "\t      ", outstr);
   out = MxMT(a, ainv);
   cr_assert(mat3x3_iden_eq(out, 5 * ULP_THRESH, 5 * DBL_THRESH),
             "Using MINVxM3 with an identity matrix argument did not result in "
             "identity with params:\n\ta   = %s\n\tout = %s",
             astr, outstr);
}

// unary rotation matrix
TheoryDataPoints(SUITE_NAME, unrotation) = {ROT3_DATAPOINTS};
Theory((mat3x3_t * ap), SUITE_NAME, unrotation)
{
   const mat3x3_t a      = *ap;
   char astr[MAT3STRLEN] = {'\0'}, Cstr[MAT3STRLEN] = {'\0'},
        Rstr[MAT3STRLEN] = {'\0'}, aTstr[MAT3STRLEN] = {'\0'},
        ainvstr[MAT3STRLEN] = {'\0'}, outstr[MAT3STRLEN] = {'\0'};
   mat3x3_t aT, ainv, out;

   // verify Rodrigues yields the original x-axis
   // mat3x3_t R = RodriguesRotation(a.x, a.y);
   // mat3x32str(a, "\t    ", astr);
   // mat3x32str(R, "\t    ", Rstr);
   // cr_expect(vec3_ieee_ulp_eq(a.z, R.z, ULP_THRESH),
   //           "Rodrigues' Rotation Formula did not yield the original z-row "
   //           "with params:\n\ta = %s\n\tR = %s",
   //           astr, Rstr);

   // inverse is transpose matrix
   // TODO: surprisingly, transpose does worse than the inverse calculation
   aT  = MT(a);
   out = MxM(a, aT);
   mat3x32str(a, "\t      ", astr);
   mat3x32str(aT, "\t      ", aTstr);
   mat3x32str(out, "\t      ", outstr);
   cr_expect(mat3x3_iden_eq(out, ULP_THRESH, (ULP_THRESH)*__DBL_EPSILON__),
             "Rotation matrix multiplied with its transpose does not result in "
             "identity with params:\n\ta   = %s\n\taT  = %s\n\tout = %s",
             astr, aTstr, outstr);

   // inverse is inverse matrix
   ainv = MINV3(a);
   out  = MxM(a, aT);
   mat3x32str(a, "\t       ", astr);
   mat3x32str(ainv, "\t       ", ainvstr);
   mat3x32str(out, "\t       ", outstr);
   cr_expect(mat3x3_iden_eq(out, ULP_THRESH, DBL_THRESH),
             "Rotation matrix multiplied with its inverse does not result in "
             "identity with params:\n\ta    = %s\n\tainv = %s\n\tout  = %s",
             astr, ainvstr, outstr);
}

/* .. expmso3 and logso3 */
// starting from vec3
TheoryDataPoints(SUITE_NAME, so3explog_vec3) = {VEC3_DATAPOINTS};
Theory((vec3_t * vp), SUITE_NAME, so3explog_vec3)
{
   const vec3_t v        = *vp;
   char vstr[VEC3STRLEN] = {'\0'}, logRstr[VEC3STRLEN] = {'\0'};
   char Rstr[MAT3STRLEN] = {'\0'};
   vec32str(v, vstr);

   const mat3x3_t R  = expmso3(v);
   const vec3_t logR = logso3(R);

   vec32str(v, vstr);
   mat3x32str(R, "\t    ", Rstr);
   // check R is rotation matrix
   cr_assert(mat3x3_iden_eq(MxMT(R, R), ULP_THRESH, DBL_THRESH),
             "Transpose of expmso3 from vec3 is not a valid rotation matrix "
             "with params:\n\tv = %s\n\tR = %s",
             vstr, Rstr);

   // cr_assume();
   mat3x32str(R, "\t           ", Rstr);
   vec32str(logR, logRstr);

   // calculate rotation vector geodesic distance
   const double th       = WrapToPMPi(MAGV(VSubV_Elem(v, logR)));
   const double geo_dist = (th < (TWOPI - th)) ? th : (TWOPI - th);

   double maxval = -INFINITY;
   for (int i = 0; i < 3; i++)
      if (fabs(v.v[i]) > maxval)
         maxval = fabs(v.v[i]);
   const double threshhld = nextafter(maxval, INFINITY) - maxval;

   cr_assume(MTrace(R) + 1 > __DBL_EPSILON__);
   // compare v to logR
   char str[4096] = {'\0'};
   sprintf(str,
           "logso3 does not return the original vector with params:\n\tv       "
           " = %s\n\tR        = %s\n\tlogR     =  %s\n\tgeo_dist = %le",
           vstr, Rstr, logRstr, geo_dist);
   cr_expect(epsilon_eq(dbl, 0, geo_dist, 4 * threshhld), "%s", str);
}

// starting from mat3x3
TheoryDataPoints(SUITE_NAME, so3explog_mat3x3) = {ROT3_DATAPOINTS};
Theory((mat3x3_t * ap), SUITE_NAME, so3explog_mat3x3)
{
   const mat3x3_t a      = *ap;
   char astr[MAT3STRLEN] = {'\0'}, Rstr[MAT3STRLEN] = {'\0'};
   char logastr[VEC3STRLEN] = {'\0'};

   // logso3 can not distinguish between 180 degree rotations about the
   // principle axes
   vec3_t sums = VEC3_ZERO;
   for (int i = 0; i < 3; i++) {
      sums.x += fabs(a.x.v[i]);
      sums.y += fabs(a.y.v[i]);
      sums.z += fabs(a.z.v[i]);
   }
   cr_assume(MTrace(a) + 1.0 > 0);

   vec3_t loga = logso3(a);
   mat3x3_t R  = expmso3(loga);
   mat3x32str(a, "\t       ", astr);
   vec32str(loga, logastr);
   mat3x32str(R, "\t       ", Rstr);
   cr_expect(mat3x3_iden_eq(MxMT(a, R), ULP_THRESH, DBL_THRESH),
             "Taking the SO(3) expm of the so(3) log of a rotation matrix did "
             "not return the original matrix with params:\n\ta    = %s\n\tR    "
             "= %s\n\tloga =  %s",
             astr, Rstr, logastr);
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

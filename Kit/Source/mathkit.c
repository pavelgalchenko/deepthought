/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "mathkit.h"
#include "42constants.h"
#include "defineskit.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

/**********************************************************************/
int any_int(const long n, const int *const vec)
{
   for (long i = 0; i < n; i++)
      if (vec[i])
         return 1;
   return 0;
}
/**********************************************************************/
int all_int(const long n, const int *const vec)
{
   for (long i = 0; i < n; i++)
      if (!vec[i])
         return 0;
   return 1;
}
/**********************************************************************/
int any_isnan(const long n, const double *const v)
{
   for (long i = 0; i < n; i++)
      if (isnan(v[i]))
         return 1;
   return 0;
}
/**********************************************************************/
double signum(const double x)
{
   return (x >= 0 ? 1.0 : -1.0);
}
/**********************************************************************/
double sin_deg(double x)
{
   return sin(x * D2R);
}
/**********************************************************************/
double cos_deg(double x)
{
   return cos(x * D2R);
}
/**********************************************************************/
/* sinc(x) = sin(x)/x                                                 */
/*  Series expansion: sinc(x) = 1 - x^2/3! + x^4/5! - x^6/7!...       */
/*  Enough terms kept to be within 2E-10 for x in [-pi:pi]            */
double sinc(const double x)
{
   double x2;

   if (x < -PI || x > PI) {
      return (sin(x) / x);
   }
   else {
      x2 = x * x;
      return (1.0 -
              x2 / 6.0 *
                  (1.0 -
                   x2 / 20.0 *
                       (1.0 -
                        x2 / 42.0 *
                            (1.0 -
                             x2 / 72.0 *
                                 (1.0 -
                                  x2 / 110.0 *
                                      (1.0 -
                                       x2 / 156.0 *
                                           (1.0 -
                                            x2 / 210.0 *
                                                (1.0 -
                                                 x2 / 272.0 *
                                                     (1.0 - x2 / 342.0)))))))));
   }
}
/**********************************************************************/
double smootherstep(const double x)
{
   return x * x * x * (x * (6.0 * x - 15.0) + 10.0);
}
/**********************************************************************/
double Limit(double x, double min, double max)
{
   return (x < min ? min : (x > max ? max : x));
}
/**********************************************************************/
/*   3x3 Matrix Product                                               */
mat3x3_t MxM(const mat3x3_t A, const mat3x3_t B)
{
   mat3x3_t C;
   C.mat[0][0] = A.mat[0][0] * B.mat[0][0] + A.mat[0][1] * B.mat[1][0] +
                 A.mat[0][2] * B.mat[2][0];
   C.mat[0][1] = A.mat[0][0] * B.mat[0][1] + A.mat[0][1] * B.mat[1][1] +
                 A.mat[0][2] * B.mat[2][1];
   C.mat[0][2] = A.mat[0][0] * B.mat[0][2] + A.mat[0][1] * B.mat[1][2] +
                 A.mat[0][2] * B.mat[2][2];
   C.mat[1][0] = A.mat[1][0] * B.mat[0][0] + A.mat[1][1] * B.mat[1][0] +
                 A.mat[1][2] * B.mat[2][0];
   C.mat[1][1] = A.mat[1][0] * B.mat[0][1] + A.mat[1][1] * B.mat[1][1] +
                 A.mat[1][2] * B.mat[2][1];
   C.mat[1][2] = A.mat[1][0] * B.mat[0][2] + A.mat[1][1] * B.mat[1][2] +
                 A.mat[1][2] * B.mat[2][2];
   C.mat[2][0] = A.mat[2][0] * B.mat[0][0] + A.mat[2][1] * B.mat[1][0] +
                 A.mat[2][2] * B.mat[2][0];
   C.mat[2][1] = A.mat[2][0] * B.mat[0][1] + A.mat[2][1] * B.mat[1][1] +
                 A.mat[2][2] * B.mat[2][1];
   C.mat[2][2] = A.mat[2][0] * B.mat[0][2] + A.mat[2][1] * B.mat[1][2] +
                 A.mat[2][2] * B.mat[2][2];
   return C;
}
/**********************************************************************/
/* 3x3 Matrix times Transpose of Matrix                               */
mat3x3_t MxMT(const mat3x3_t A, const mat3x3_t B)
{
   mat3x3_t C;
   C.mat[0][0] = A.mat[0][0] * B.mat[0][0] + A.mat[0][1] * B.mat[0][1] +
                 A.mat[0][2] * B.mat[0][2];
   C.mat[0][1] = A.mat[0][0] * B.mat[1][0] + A.mat[0][1] * B.mat[1][1] +
                 A.mat[0][2] * B.mat[1][2];
   C.mat[0][2] = A.mat[0][0] * B.mat[2][0] + A.mat[0][1] * B.mat[2][1] +
                 A.mat[0][2] * B.mat[2][2];
   C.mat[1][0] = A.mat[1][0] * B.mat[0][0] + A.mat[1][1] * B.mat[0][1] +
                 A.mat[1][2] * B.mat[0][2];
   C.mat[1][1] = A.mat[1][0] * B.mat[1][0] + A.mat[1][1] * B.mat[1][1] +
                 A.mat[1][2] * B.mat[1][2];
   C.mat[1][2] = A.mat[1][0] * B.mat[2][0] + A.mat[1][1] * B.mat[2][1] +
                 A.mat[1][2] * B.mat[2][2];
   C.mat[2][0] = A.mat[2][0] * B.mat[0][0] + A.mat[2][1] * B.mat[0][1] +
                 A.mat[2][2] * B.mat[0][2];
   C.mat[2][1] = A.mat[2][0] * B.mat[1][0] + A.mat[2][1] * B.mat[1][1] +
                 A.mat[2][2] * B.mat[1][2];
   C.mat[2][2] = A.mat[2][0] * B.mat[2][0] + A.mat[2][1] * B.mat[2][1] +
                 A.mat[2][2] * B.mat[2][2];
   return C;
}
/**********************************************************************/
/*  3x3 Transpose of Matrix times Matrix                              */
mat3x3_t MTxM(const mat3x3_t A, const mat3x3_t B)
{
   mat3x3_t C;
   C.mat[0][0] = A.mat[0][0] * B.mat[0][0] + A.mat[1][0] * B.mat[1][0] +
                 A.mat[2][0] * B.mat[2][0];
   C.mat[0][1] = A.mat[0][0] * B.mat[0][1] + A.mat[1][0] * B.mat[1][1] +
                 A.mat[2][0] * B.mat[2][1];
   C.mat[0][2] = A.mat[0][0] * B.mat[0][2] + A.mat[1][0] * B.mat[1][2] +
                 A.mat[2][0] * B.mat[2][2];
   C.mat[1][0] = A.mat[0][1] * B.mat[0][0] + A.mat[1][1] * B.mat[1][0] +
                 A.mat[2][1] * B.mat[2][0];
   C.mat[1][1] = A.mat[0][1] * B.mat[0][1] + A.mat[1][1] * B.mat[1][1] +
                 A.mat[2][1] * B.mat[2][1];
   C.mat[1][2] = A.mat[0][1] * B.mat[0][2] + A.mat[1][1] * B.mat[1][2] +
                 A.mat[2][1] * B.mat[2][2];
   C.mat[2][0] = A.mat[0][2] * B.mat[0][0] + A.mat[1][2] * B.mat[1][0] +
                 A.mat[2][2] * B.mat[2][0];
   C.mat[2][1] = A.mat[0][2] * B.mat[0][1] + A.mat[1][2] * B.mat[1][1] +
                 A.mat[2][2] * B.mat[2][1];
   C.mat[2][2] = A.mat[0][2] * B.mat[0][2] + A.mat[1][2] * B.mat[1][2] +
                 A.mat[2][2] * B.mat[2][2];
   return C;
}
/**********************************************************************/
/*  3x3 Transpose of Matrix times Transpose of Matrix                 */
mat3x3_t MTxMT(const mat3x3_t A, const mat3x3_t B)
{
   mat3x3_t C;
   C.mat[0][0] = A.mat[0][0] * B.mat[0][0] + A.mat[1][0] * B.mat[0][1] +
                 A.mat[2][0] * B.mat[0][2];
   C.mat[0][1] = A.mat[0][0] * B.mat[1][0] + A.mat[1][0] * B.mat[1][1] +
                 A.mat[2][0] * B.mat[1][2];
   C.mat[0][2] = A.mat[0][0] * B.mat[2][0] + A.mat[1][0] * B.mat[2][1] +
                 A.mat[2][0] * B.mat[2][2];
   C.mat[1][0] = A.mat[0][1] * B.mat[0][0] + A.mat[1][1] * B.mat[0][1] +
                 A.mat[2][1] * B.mat[0][2];
   C.mat[1][1] = A.mat[0][1] * B.mat[1][0] + A.mat[1][1] * B.mat[1][1] +
                 A.mat[2][1] * B.mat[1][2];
   C.mat[1][2] = A.mat[0][1] * B.mat[2][0] + A.mat[1][1] * B.mat[2][1] +
                 A.mat[2][1] * B.mat[2][2];
   C.mat[2][0] = A.mat[0][2] * B.mat[0][0] + A.mat[1][2] * B.mat[0][1] +
                 A.mat[2][2] * B.mat[0][2];
   C.mat[2][1] = A.mat[0][2] * B.mat[1][0] + A.mat[1][2] * B.mat[1][1] +
                 A.mat[2][2] * B.mat[1][2];
   C.mat[2][2] = A.mat[0][2] * B.mat[2][0] + A.mat[1][2] * B.mat[2][1] +
                 A.mat[2][2] * B.mat[2][2];
   return C;
}
/**********************************************************************/
/*  1x3 Vector times 3x3 Matrix                                       */
vec3_t VxM(const vec3_t V, const mat3x3_t M)
{
   vec3_t W;
   W.x = V.x * M.mat[0][0] + V.y * M.mat[1][0] + V.z * M.mat[2][0];
   W.y = V.x * M.mat[0][1] + V.y * M.mat[1][1] + V.z * M.mat[2][1];
   W.z = V.x * M.mat[0][2] + V.y * M.mat[1][2] + V.z * M.mat[2][2];
   return W;
}
/**********************************************************************/
/*  Transpose of 3x3 Matrix times 3x1 Vector                          */
/*  Equivalent to the transpose problem, VxM                          */
vec3_t MTxV(const mat3x3_t M, const vec3_t V)
{
   // vec3_t W;
   // W.x = V.x * M.mat[0][0] + V.y * M.mat[1][0] + V.z * M.mat[2][0];
   // W.y = V.x * M.mat[0][1] + V.y * M.mat[1][1] + V.z * M.mat[2][1];
   // W.z = V.x * M.mat[0][2] + V.y * M.mat[1][2] + V.z * M.mat[2][2];
   return VxM(V, M);
}
/**********************************************************************/
/*  3x3 Matrix times 3x1 Vector                                       */
vec3_t MxV(const mat3x3_t M, const vec3_t V)
{
   vec3_t W;
   W.x = V.x * M.mat[0][0] + V.y * M.mat[0][1] + V.z * M.mat[0][2];
   W.y = V.x * M.mat[1][0] + V.y * M.mat[1][1] + V.z * M.mat[1][2];
   W.z = V.x * M.mat[2][0] + V.y * M.mat[2][1] + V.z * M.mat[2][2];
   return W;
}
/**********************************************************************/
/*  1x3 Vector times transpose of 3x3 Matrix                          */
/*  Equivalent to the transpose problem, MxV                          */
vec3_t VxMT(const vec3_t V, const mat3x3_t M)
{
   // vec3_t W;
   // W.x = V.x * M.mat[0][0] + V.y * M.mat[0][1] + V.z * M.mat[0][2];
   // W.y = V.x * M.mat[1][0] + V.y * M.mat[1][1] + V.z * M.mat[1][2];
   // W.z = V.x * M.mat[2][0] + V.y * M.mat[2][1] + V.z * M.mat[2][2];
   return MxV(M, V);
}
/**********************************************************************/
/*  Scalar times 3x1 Vector                                           */
vec3_t SxV(const double S, const vec3_t V)
{
   vec3_t W;
   W.x = S * V.x;
   W.y = S * V.y;
   W.z = S * V.z;
   return W;
}
/**********************************************************************/
vec3_t NegV_Elem(const vec3_t A)
{
   vec3_t out;
   out.x = -A.x;
   out.y = -A.y;
   out.z = -A.z;
   return out;
}
/**********************************************************************/
vec3_t VAddV_Elem(const vec3_t A, const vec3_t B)
{
   vec3_t out  = A;
   out.x      += B.x;
   out.y      += B.y;
   out.z      += B.z;
   return out;
}
/**********************************************************************/
vec3_t VSubV_Elem(const vec3_t A, const vec3_t B)
{
   vec3_t out  = A;
   out.x      -= B.x;
   out.y      -= B.y;
   out.z      -= B.z;
   return out;
}
/**********************************************************************/
vec3_t VMulV_Elem(const vec3_t A, const vec3_t B)
{
   vec3_t out  = A;
   out.x      *= B.x;
   out.y      *= B.y;
   out.z      *= B.z;
   return out;
}
/**********************************************************************/
vec3_t VDivV_Elem(const vec3_t A, const vec3_t B)
{
   vec3_t out  = A;
   out.x      /= (fabs(B.x) > __DBL_EPSILON__) ? B.x : 0.0;
   out.y      /= (fabs(B.y) > __DBL_EPSILON__) ? B.y : 0.0;
   out.z      /= (fabs(B.z) > __DBL_EPSILON__) ? B.z : 0.0;
   return out;
}
/**********************************************************************/
vec3_t LimitElem_bidir(vec3_t x, const vec3_t lim)
{
   if (lim.x > 0)
      x.x = Limit(x.x, -lim.x, lim.x);
   if (lim.y > 0)
      x.y = Limit(x.y, -lim.y, lim.y);
   if (lim.z > 0)
      x.z = Limit(x.z, -lim.z, lim.z);
   return x;
}
/**********************************************************************/
int _isequal_vec3(const vec3_t a, const vec3_t b)
{
   return a.x == b.x && a.y == b.y && a.z == b.z;
}
/**********************************************************************/
mat3x3_t MAddM_Elem(const mat3x3_t A, const mat3x3_t B)
{
   mat3x3_t out  = A;
   out.flat[0]  += B.flat[0];
   out.flat[1]  += B.flat[1];
   out.flat[2]  += B.flat[2];
   out.flat[3]  += B.flat[3];
   out.flat[4]  += B.flat[4];
   out.flat[5]  += B.flat[5];
   out.flat[6]  += B.flat[6];
   out.flat[7]  += B.flat[7];
   out.flat[8]  += B.flat[8];
   return out;
}
/**********************************************************************/
mat3x3_t MSubM_Elem(const mat3x3_t A, const mat3x3_t B)
{
   mat3x3_t out  = A;
   out.flat[0]  -= B.flat[0];
   out.flat[1]  -= B.flat[1];
   out.flat[2]  -= B.flat[2];
   out.flat[3]  -= B.flat[3];
   out.flat[4]  -= B.flat[4];
   out.flat[5]  -= B.flat[5];
   out.flat[6]  -= B.flat[6];
   out.flat[7]  -= B.flat[7];
   out.flat[8]  -= B.flat[8];
   return out;
}
/**********************************************************************/
mat3x3_t MMulM_Elem(const mat3x3_t A, const mat3x3_t B)
{
   mat3x3_t out  = A;
   out.flat[0]  *= B.flat[0];
   out.flat[1]  *= B.flat[1];
   out.flat[2]  *= B.flat[2];
   out.flat[3]  *= B.flat[3];
   out.flat[4]  *= B.flat[4];
   out.flat[5]  *= B.flat[5];
   out.flat[6]  *= B.flat[6];
   out.flat[7]  *= B.flat[7];
   out.flat[8]  *= B.flat[8];
   return out;
}
/**********************************************************************/
mat3x3_t MDivM_Elem(const mat3x3_t A, const mat3x3_t B)
{
   mat3x3_t out  = A;
   out.flat[0]  /= B.flat[0];
   out.flat[1]  /= B.flat[1];
   out.flat[2]  /= B.flat[2];
   out.flat[3]  /= B.flat[3];
   out.flat[4]  /= B.flat[4];
   out.flat[5]  /= B.flat[5];
   out.flat[6]  /= B.flat[6];
   out.flat[7]  /= B.flat[7];
   out.flat[8]  /= B.flat[8];
   return out;
}
/**********************************************************************/
double MTrace(const mat3x3_t A)
{
   return A.mat[0][0] + A.mat[1][1] + A.mat[2][2];
}
/**********************************************************************/
int _isequal_mat3x3(const mat3x3_t a, const mat3x3_t b)
{
   return _isequal_vec3(a.rows[0], b.rows[0]) &&
          _isequal_vec3(a.rows[1], b.rows[1]) &&
          _isequal_vec3(a.rows[2], b.rows[2]);
}
/**********************************************************************/
/*              Cofactor of 3x3 matrix                                */
mat3x3_t cof3x3(const mat3x3_t A)
{
   mat3x3_t B;
   B.mat[0][0] = (A.mat[1][1] * A.mat[2][2] - A.mat[2][1] * A.mat[1][2]);
   B.mat[0][1] = (A.mat[2][0] * A.mat[1][2] - A.mat[1][0] * A.mat[2][2]);
   B.mat[0][2] = (A.mat[1][0] * A.mat[2][1] - A.mat[2][0] * A.mat[1][1]);
   B.mat[1][0] = (A.mat[2][1] * A.mat[0][2] - A.mat[0][1] * A.mat[2][2]);
   B.mat[1][1] = (A.mat[0][0] * A.mat[2][2] - A.mat[2][0] * A.mat[0][2]);
   B.mat[1][2] = (A.mat[2][0] * A.mat[0][1] - A.mat[0][0] * A.mat[2][1]);
   B.mat[2][0] = (A.mat[0][1] * A.mat[1][2] - A.mat[1][1] * A.mat[0][2]);
   B.mat[2][1] = (A.mat[1][0] * A.mat[0][2] - A.mat[0][0] * A.mat[1][2]);
   B.mat[2][2] = (A.mat[0][0] * A.mat[1][1] - A.mat[1][0] * A.mat[0][1]);
   return B;
}
/**********************************************************************/
/* Transpose of Cofactor of 3x3 matrix                                */
mat3x3_t cofT3x3(const mat3x3_t A)
{
   mat3x3_t B;
   B.mat[0][0] = (A.mat[1][1] * A.mat[2][2] - A.mat[2][1] * A.mat[1][2]);
   B.mat[0][1] = (A.mat[2][1] * A.mat[0][2] - A.mat[0][1] * A.mat[2][2]);
   B.mat[0][2] = (A.mat[0][1] * A.mat[1][2] - A.mat[1][1] * A.mat[0][2]);
   B.mat[1][0] = (A.mat[2][0] * A.mat[1][2] - A.mat[1][0] * A.mat[2][2]);
   B.mat[1][1] = (A.mat[0][0] * A.mat[2][2] - A.mat[2][0] * A.mat[0][2]);
   B.mat[1][2] = (A.mat[1][0] * A.mat[0][2] - A.mat[0][0] * A.mat[1][2]);
   B.mat[2][0] = (A.mat[1][0] * A.mat[2][1] - A.mat[2][0] * A.mat[1][1]);
   B.mat[2][1] = (A.mat[2][0] * A.mat[0][1] - A.mat[0][0] * A.mat[2][1]);
   B.mat[2][2] = (A.mat[0][0] * A.mat[1][1] - A.mat[1][0] * A.mat[0][1]);
   return B;
}
/**********************************************************************/
/*  Scalar times 3x3 Matrix                                           */
mat3x3_t SxM(const double S, const mat3x3_t A)
{
   mat3x3_t B;
   B.mat[0][0] = S * A.mat[0][0];
   B.mat[0][1] = S * A.mat[0][1];
   B.mat[0][2] = S * A.mat[0][2];
   B.mat[1][0] = S * A.mat[1][0];
   B.mat[1][1] = S * A.mat[1][1];
   B.mat[1][2] = S * A.mat[1][2];
   B.mat[2][0] = S * A.mat[2][0];
   B.mat[2][1] = S * A.mat[2][1];
   B.mat[2][2] = S * A.mat[2][2];
   return B;
}
/******************************************************************************/
double det3x3(const mat3x3_t M)
{
   return M.mat[0][0] *
              (M.mat[1][1] * M.mat[2][2] - M.mat[1][2] * M.mat[2][1]) -
          M.mat[0][1] *
              (M.mat[1][0] * M.mat[2][2] - M.mat[1][2] * M.mat[2][0]) +
          M.mat[0][2] * (M.mat[1][0] * M.mat[2][1] - M.mat[1][1] * M.mat[2][0]);
}
/******************************************************************************/
/* Inverse of a 4x4 Matrix                                                    */
void MINV4(const double A[4][4], double B[4][4])
{
   double DET = 0.0;
   long r, s, i, j, k, x, y, z;

   for (r = 0; r < 4; r++) {
      for (s = 0; s < 4; s++) {
         i       = (r + 1) % 4;
         j       = (r + 2) % 4;
         k       = (r + 3) % 4;
         x       = (s + 1) % 4;
         y       = (s + 2) % 4;
         z       = (s + 3) % 4;
         B[s][r] = A[i][x] * (A[j][y] * A[k][z] - A[j][z] * A[k][y]) +
                   A[i][y] * (A[j][z] * A[k][x] - A[j][x] * A[k][z]) +
                   A[i][z] * (A[j][x] * A[k][y] - A[j][y] * A[k][x]);
         if ((r + s) % 2 == 1)
            B[s][r] = -B[s][r];
      }
   }
   for (r = 0; r < 4; r++)
      DET += A[0][r] * B[r][0];

   if (DET == 0.0) {
      fprintf(
          stderr,
          "Attempted inversion of singular matrix in MINV4.  Bailing out.\n");
      exit(EXIT_FAILURE);
   }
   else {
      for (r = 0; r < 4; r++) {
         for (s = 0; s < 4; s++) {
            B[r][s] /= DET;
         }
      }
   }
}
/******************************************************************************/
/*  Inverse of a 3x3 Matrix                                                   */
mat3x3_t MINV3(const mat3x3_t A)
{
   mat3x3_t B;
   double DET;

   DET = det3x3(A);

   if (DET == 0.0) {
      fprintf(
          stderr,
          "Attempted inversion of singular matrix in MINV3.  Bailing out.\n");
      exit(EXIT_FAILURE);
   }
   else
      B = SxM(1.0 / DET, cofT3x3(A));

   return B;
}
/******************************************************************************/
/*  Inverse of a 2x2 Matrix                                                   */
void MINV2(const double A[2][2], double B[2][2])
{
   double DET;

   DET = A[0][0] * A[1][1] - A[1][0] * A[0][1];

   if (DET == 0.0) {
      fprintf(
          stderr,
          "Attempted inversion of singular matrix in MINV2.  Bailing out.\n");
      exit(EXIT_FAILURE);
   }
   else {
      B[0][0] = A[1][1] / DET;
      B[0][1] = -A[0][1] / DET;
      B[1][0] = -A[1][0] / DET;
      B[1][1] = A[0][0] / DET;
   }
}
/**********************************************************************/
/*  Pseudo-inverse of a 4x3 matrix                                    */
void PINV4x3(const double A[4][3], double Aplus[3][4])
{
   mat3x3_t AtA = {0};

   AtA.mat[0][0] = A[0][0] * A[0][0] + A[1][0] * A[1][0] + A[2][0] * A[2][0] +
                   A[3][0] * A[3][0];
   AtA.mat[0][1] = A[0][0] * A[0][1] + A[1][0] * A[1][1] + A[2][0] * A[2][1] +
                   A[3][0] * A[3][1];
   AtA.mat[0][2] = A[0][0] * A[0][2] + A[1][0] * A[1][2] + A[2][0] * A[2][2] +
                   A[3][0] * A[3][2];
   AtA.mat[1][0] = A[0][1] * A[0][0] + A[1][1] * A[1][0] + A[2][1] * A[2][0] +
                   A[3][1] * A[3][0];
   AtA.mat[1][1] = A[0][1] * A[0][1] + A[1][1] * A[1][1] + A[2][1] * A[2][1] +
                   A[3][1] * A[3][1];
   AtA.mat[1][2] = A[0][1] * A[0][2] + A[1][1] * A[1][2] + A[2][1] * A[2][2] +
                   A[3][1] * A[3][2];
   AtA.mat[2][0] = A[0][2] * A[0][0] + A[1][2] * A[1][0] + A[2][2] * A[2][0] +
                   A[3][2] * A[3][0];
   AtA.mat[2][1] = A[0][2] * A[0][1] + A[1][2] * A[1][1] + A[2][2] * A[2][1] +
                   A[3][2] * A[3][1];
   AtA.mat[2][2] = A[0][2] * A[0][2] + A[1][2] * A[1][2] + A[2][2] * A[2][2] +
                   A[3][2] * A[3][2];

   mat3x3_t AtAi = MINV3(AtA);

   Aplus[0][0] = AtAi.mat[0][0] * A[0][0] + AtAi.mat[0][1] * A[0][1] +
                 AtAi.mat[0][2] * A[0][2];
   Aplus[0][1] = AtAi.mat[0][0] * A[1][0] + AtAi.mat[0][1] * A[1][1] +
                 AtAi.mat[0][2] * A[1][2];
   Aplus[0][2] = AtAi.mat[0][0] * A[2][0] + AtAi.mat[0][1] * A[2][1] +
                 AtAi.mat[0][2] * A[2][2];
   Aplus[0][3] = AtAi.mat[0][0] * A[3][0] + AtAi.mat[0][1] * A[3][1] +
                 AtAi.mat[0][2] * A[3][2];
   Aplus[1][0] = AtAi.mat[1][0] * A[0][0] + AtAi.mat[1][1] * A[0][1] +
                 AtAi.mat[1][2] * A[0][2];
   Aplus[1][1] = AtAi.mat[1][0] * A[1][0] + AtAi.mat[1][1] * A[1][1] +
                 AtAi.mat[1][2] * A[1][2];
   Aplus[1][2] = AtAi.mat[1][0] * A[2][0] + AtAi.mat[1][1] * A[2][1] +
                 AtAi.mat[1][2] * A[2][2];
   Aplus[1][3] = AtAi.mat[1][0] * A[3][0] + AtAi.mat[1][1] * A[3][1] +
                 AtAi.mat[1][2] * A[3][2];
   Aplus[2][0] = AtAi.mat[2][0] * A[0][0] + AtAi.mat[2][1] * A[0][1] +
                 AtAi.mat[2][2] * A[0][2];
   Aplus[2][1] = AtAi.mat[2][0] * A[1][0] + AtAi.mat[2][1] * A[1][1] +
                 AtAi.mat[2][2] * A[1][2];
   Aplus[2][2] = AtAi.mat[2][0] * A[2][0] + AtAi.mat[2][1] * A[2][1] +
                 AtAi.mat[2][2] * A[2][2];
   Aplus[2][3] = AtAi.mat[2][0] * A[3][0] + AtAi.mat[2][1] * A[3][1] +
                 AtAi.mat[2][2] * A[3][2];
}
/**********************************************************************/
/*  Transpose of a 3x3 Matrix                                         */
mat3x3_t MT(const mat3x3_t A)
{
   mat3x3_t B;
   B.mat[0][0] = A.mat[0][0];
   B.mat[0][1] = A.mat[1][0];
   B.mat[0][2] = A.mat[2][0];
   B.mat[1][0] = A.mat[0][1];
   B.mat[1][1] = A.mat[1][1];
   B.mat[1][2] = A.mat[2][1];
   B.mat[2][0] = A.mat[0][2];
   B.mat[2][1] = A.mat[1][2];
   B.mat[2][2] = A.mat[2][2];
   return B;
}
/**********************************************************************/
/*  Vector Dot Product                                                */
double VoV(const vec3_t A, const vec3_t B)
{
   return (A.x * B.x + A.y * B.y + A.z * B.z);
}
/**********************************************************************/
mat3x3_t VOuterV(const vec3_t A, const vec3_t B)
{
   mat3x3_t out;
   out.x.x = A.x * B.x;
   out.x.y = A.x * B.y;
   out.x.z = A.x * B.z;
   out.y.x = A.y * B.x;
   out.y.y = A.y * B.y;
   out.y.z = A.y * B.z;
   out.z.x = A.z * B.x;
   out.z.y = A.z * B.y;
   out.z.z = A.z * B.z;
   return out;
}
/**********************************************************************/
/*  Vector Cross Product                                              */
vec3_t VxV(const vec3_t A, const vec3_t B)
{
   vec3_t C;
   C.v[0] = A.v[1] * B.v[2] - A.v[2] * B.v[1];
   C.v[1] = A.v[2] * B.v[0] - A.v[0] * B.v[2];
   C.v[2] = A.v[0] * B.v[1] - A.v[1] * B.v[0];
   return C;
}
/**********************************************************************/
/*  Vector cross Matrix dot Vector                                    */
vec3_t vxMov(const vec3_t w, const mat3x3_t M)
{
   vec3_t Mow = MxV(M, w);
   return VxV(w, Mow);
}
/**********************************************************************/
/*  Magnitude of a 3-vector                                           */
double MAGV(const vec3_t V)
{
   return sqrt(VoV(V, V));
}
/**********************************************************************/
/*  Normalize a 3-vector.  Return its (pre-normalization) magnitude   */
magvec3_t UNITV(vec3_t V)
{
   magvec3_t A;

   A.v = V;
   A.m = MAGV(A.v);
   if (A.m > 0.0) {
      A.v.v[0] /= A.m;
      A.v.v[1] /= A.m;
      A.v.v[2] /= A.m;
   }
   else {
      printf("Attempted divide by zero in UNITV (Line %d of mathkit.c)\n",
             __LINE__);
      A.v.v[0] = 0.0;
      A.v.v[1] = 0.0;
      A.v.v[2] = 0.0;
   }
   return (A);
}
/**********************************************************************/
/*  Copy and normalize a 3-vector.  Return its magnitude              */
// double CopyUnitV(const vec3_t V, vec3_t *W)
// {
//    *W       = V;
//    double A = UNITV(W);
//    return (A);
// }
/**********************************************************************/
/*  Form a skew-symmetric matrix M from a vector V such that the      */
/*  product MxA equals the cross product VxA for any vector A.        */
mat3x3_t V2CrossM(const vec3_t V)
{
   mat3x3_t M;
   M.mat[0][0] = 0.0;
   M.mat[1][1] = 0.0;
   M.mat[2][2] = 0.0;
   M.mat[2][1] = V.v[0];
   M.mat[0][2] = V.v[1];
   M.mat[1][0] = V.v[2];
   M.mat[1][2] = -V.v[0];
   M.mat[2][0] = -V.v[1];
   M.mat[0][1] = -V.v[2];
   return M;
}
/**********************************************************************/
/*  Form a symmetric matrix M from a vector V such that the           */
/*  product M*A equals the product Vx(VxA) for any vector A.          */
mat3x3_t V2DoubleCrossM(const vec3_t V)
{
   mat3x3_t M;
   M.mat[0][0] = -V.v[1] * V.v[1] - V.v[2] * V.v[2];
   M.mat[1][1] = -V.v[2] * V.v[2] - V.v[0] * V.v[0];
   M.mat[2][2] = -V.v[0] * V.v[0] - V.v[1] * V.v[1];
   M.mat[2][1] = V.v[2] * V.v[1];
   M.mat[0][2] = V.v[0] * V.v[2];
   M.mat[1][0] = V.v[1] * V.v[0];
   M.mat[1][2] = V.v[1] * V.v[2];
   M.mat[2][0] = V.v[2] * V.v[0];
   M.mat[0][1] = V.v[0] * V.v[1];
   return M;
}
/**********************************************************************/
/*  Save a step.  Form a skew matrix from V, then multiply by M       */
mat3x3_t VcrossM(const vec3_t V, const mat3x3_t M)
{
   mat3x3_t A;
   A.mat[0][0] = V.v[1] * M.mat[2][0] - V.v[2] * M.mat[1][0];
   A.mat[0][1] = V.v[1] * M.mat[2][1] - V.v[2] * M.mat[1][1];
   A.mat[0][2] = V.v[1] * M.mat[2][2] - V.v[2] * M.mat[1][2];
   A.mat[1][0] = V.v[2] * M.mat[0][0] - V.v[0] * M.mat[2][0];
   A.mat[1][1] = V.v[2] * M.mat[0][1] - V.v[0] * M.mat[2][1];
   A.mat[1][2] = V.v[2] * M.mat[0][2] - V.v[0] * M.mat[2][2];
   A.mat[2][0] = V.v[0] * M.mat[1][0] - V.v[1] * M.mat[0][0];
   A.mat[2][1] = V.v[0] * M.mat[1][1] - V.v[1] * M.mat[0][1];
   A.mat[2][2] = V.v[0] * M.mat[1][2] - V.v[1] * M.mat[0][2];
   return A;
}
/**********************************************************************/
/*  Save a step.  Form a skew matrix from V, then multiply by MT      */
mat3x3_t VcrossMT(const vec3_t V, const mat3x3_t M)
{
   mat3x3_t A;
   A.mat[0][0] = V.v[1] * M.mat[0][2] - V.v[2] * M.mat[0][1];
   A.mat[0][1] = V.v[1] * M.mat[1][2] - V.v[2] * M.mat[1][1];
   A.mat[0][2] = V.v[1] * M.mat[2][2] - V.v[2] * M.mat[2][1];
   A.mat[1][0] = V.v[2] * M.mat[0][0] - V.v[0] * M.mat[0][2];
   A.mat[1][1] = V.v[2] * M.mat[1][0] - V.v[0] * M.mat[1][2];
   A.mat[1][2] = V.v[2] * M.mat[2][0] - V.v[0] * M.mat[2][2];
   A.mat[2][0] = V.v[0] * M.mat[0][1] - V.v[1] * M.mat[0][0];
   A.mat[2][1] = V.v[0] * M.mat[1][1] - V.v[1] * M.mat[1][0];
   A.mat[2][2] = V.v[0] * M.mat[2][1] - V.v[1] * M.mat[2][0];
   return A;
}
/**********************************************************************/
/*  Quaternion product                                                */
quat_t QxQ(const quat_t A, const quat_t B)
{
   quat_t C;
   C.q[0] =
       A.q[3] * B.q[0] + A.q[2] * B.q[1] - A.q[1] * B.q[2] + A.q[0] * B.q[3];
   C.q[1] =
       -A.q[2] * B.q[0] + A.q[3] * B.q[1] + A.q[0] * B.q[2] + A.q[1] * B.q[3];
   C.q[2] =
       A.q[1] * B.q[0] - A.q[0] * B.q[1] + A.q[3] * B.q[2] + A.q[2] * B.q[3];
   C.q[3] =
       -A.q[0] * B.q[0] - A.q[1] * B.q[1] - A.q[2] * B.q[2] + A.q[3] * B.q[3];
   return C;
}
/**********************************************************************/
/* Product of the Complement of a Quaternion (A) with a Quaternion (B)*/
quat_t QTxQ(const quat_t A, const quat_t B)
{
   quat_t C;
   C.q[0] =
       A.q[3] * B.q[0] - A.q[2] * B.q[1] + A.q[1] * B.q[2] - A.q[0] * B.q[3];
   C.q[1] =
       A.q[2] * B.q[0] + A.q[3] * B.q[1] - A.q[0] * B.q[2] - A.q[1] * B.q[3];
   C.q[2] =
       -A.q[1] * B.q[0] + A.q[0] * B.q[1] + A.q[3] * B.q[2] - A.q[2] * B.q[3];
   C.q[3] =
       A.q[0] * B.q[0] + A.q[1] * B.q[1] + A.q[2] * B.q[2] + A.q[3] * B.q[3];
   return C;
}
/**********************************************************************/
/* Product of a Quaternion (A) with the Complement of a Quaternion (B)*/
quat_t QxQT(const quat_t A, const quat_t B)
{
   quat_t C;
   C.q[0] =
       -A.q[3] * B.q[0] - A.q[2] * B.q[1] + A.q[1] * B.q[2] + A.q[0] * B.q[3];
   C.q[1] =
       A.q[2] * B.q[0] - A.q[3] * B.q[1] - A.q[0] * B.q[2] + A.q[1] * B.q[3];
   C.q[2] =
       -A.q[1] * B.q[0] + A.q[0] * B.q[1] - A.q[3] * B.q[2] + A.q[2] * B.q[3];
   C.q[3] =
       A.q[0] * B.q[0] + A.q[1] * B.q[1] + A.q[2] * B.q[2] + A.q[3] * B.q[3];
   return C;
}
/**********************************************************************/
/* Find components of V in B, given components of V in A, and qab     */
vec3_t VxQ(const vec3_t Va, const quat_t QAB)
{
   vec3_t Vb;
   double qq[4][4];
   long i, j;

   for (i = 0; i < 4; i++) {
      for (j = i; j < 4; j++)
         qq[i][j] = QAB.q[i] * QAB.q[j];
   }

   Vb.v[0] = (qq[0][0] - qq[1][1] - qq[2][2] + qq[3][3]) * Va.v[0] +
             2.0 * ((qq[0][1] - qq[2][3]) * Va.v[1] +
                    (qq[0][2] + qq[1][3]) * Va.v[2]);
   Vb.v[1] = (-qq[0][0] + qq[1][1] - qq[2][2] + qq[3][3]) * Va.v[1] +
             2.0 * ((qq[1][2] - qq[0][3]) * Va.v[2] +
                    (qq[0][1] + qq[2][3]) * Va.v[0]);
   Vb.v[2] = (-qq[0][0] - qq[1][1] + qq[2][2] + qq[3][3]) * Va.v[2] +
             2.0 * ((qq[0][2] - qq[1][3]) * Va.v[0] +
                    (qq[1][2] + qq[0][3]) * Va.v[1]);
   return Vb;
}
/**********************************************************************/
/* Find components of V in A, given components of V in B, and qab     */
vec3_t QxV(const quat_t QAB, const vec3_t Vb)
{
   vec3_t Va;
   double qq[4][4];
   long i, j;

   for (i = 0; i < 4; i++) {
      for (j = i; j < 4; j++)
         qq[i][j] = QAB.q[i] * QAB.q[j];
   }

   Va.v[0] = (qq[0][0] - qq[1][1] - qq[2][2] + qq[3][3]) * Vb.v[0] +
             2.0 * ((qq[0][1] + qq[2][3]) * Vb.v[1] +
                    (qq[0][2] - qq[1][3]) * Vb.v[2]);
   Va.v[1] = (-qq[0][0] + qq[1][1] - qq[2][2] + qq[3][3]) * Vb.v[1] +
             2.0 * ((qq[1][2] + qq[0][3]) * Vb.v[2] +
                    (qq[0][1] - qq[2][3]) * Vb.v[0]);
   Va.v[2] = (-qq[0][0] - qq[1][1] + qq[2][2] + qq[3][3]) * Vb.v[2] +
             2.0 * ((qq[0][2] + qq[1][3]) * Vb.v[0] +
                    (qq[1][2] - qq[0][3]) * Vb.v[1]);
   return Va;
}
/**********************************************************************/
/* Find components of V in B, given components of V in A, and qab     */
vec3_t QTxV(const quat_t QAB, const vec3_t Va)
{
   vec3_t Vb;
   double qq[4][4];
   long i, j;

   for (i = 0; i < 4; i++) {
      for (j = i; j < 4; j++)
         qq[i][j] = QAB.q[i] * QAB.q[j];
   }

   Vb.v[0] = (qq[0][0] - qq[1][1] - qq[2][2] + qq[3][3]) * Va.v[0] +
             2.0 * ((qq[0][1] - qq[2][3]) * Va.v[1] +
                    (qq[0][2] + qq[1][3]) * Va.v[2]);
   Vb.v[1] = (-qq[0][0] + qq[1][1] - qq[2][2] + qq[3][3]) * Va.v[1] +
             2.0 * ((qq[1][2] - qq[0][3]) * Va.v[2] +
                    (qq[0][1] + qq[2][3]) * Va.v[0]);
   Vb.v[2] = (-qq[0][0] - qq[1][1] + qq[2][2] + qq[3][3]) * Va.v[2] +
             2.0 * ((qq[0][2] - qq[1][3]) * Va.v[0] +
                    (qq[1][2] + qq[0][3]) * Va.v[1]);
   return Vb;
}
/**********************************************************************/
/*  Normalize a quaternion                                            */
quat_t UNITQ(quat_t Q)
{
   double A = sqrt(VoV(Q.qv, Q.qv) + Q.qs * Q.qs);
   if (A == 0.0) {
      fprintf(stderr,
              "Divide by zero in UNITQ (Line %d of mathkit.c).  You'll want to "
              "fix that.\n",
              __LINE__);
      exit(EXIT_FAILURE);
   }
   else {
      Q.q[0] /= A;
      Q.q[1] /= A;
      Q.q[2] /= A;
      Q.q[3] /= A;
   }
   return Q;
}
/**********************************************************************/
/*  Rectify a quaternion, forcing q[3] to be positive                 */
quat_t RECTIFYQ(quat_t Q)
{
   if (Q.q[3] < 0.0) {
      Q.q[0] = -Q.q[0];
      Q.q[1] = -Q.q[1];
      Q.q[2] = -Q.q[2];
      Q.q[3] = -Q.q[3];
   }
   return Q;
}
/*********************************************************************/
int _isequal_vec4(const vec4_t a, const vec4_t b)
{
   return _isequal_vec3(a.qv, b.qv) && a.qs == b.qs;
}
/*********************************************************************/
/* Given vector A, find vectors B, C to form orthogonal basis        */
pair_vec3_t PerpBasis(const vec3_t A)
{
   long i;
   magvec3_t uv;
   pair_vec3_t V = {.first = VEC3_ZERO, .second = VEC3_ZERO};
   double Amin;

   Amin = fabs(A.v[0]);
   i    = 0;
   if (fabs(A.v[1]) < Amin) {
      Amin = A.v[1];
      i    = 1;
   }
   if (fabs(A.v[2]) < Amin) {
      i = 2;
   }

   V.first.v[i] = 1.0;
   V.first      = VxV(A, V.first);
   uv           = UNITV(V.first);
   V.first      = uv.v;
   V.second     = VxV(A, V.first);
   uv           = UNITV(V.second);
   V.second     = uv.v;
   return V;
}
/**********************************************************************/
double fact(long const n)
{
   double F = 1.0;
   long i;

   for (i = 1; i <= n; i++)
      F *= i;

   return F;
}
/**********************************************************************/
double oddfact(long const n)
{
   static double *memo   = NULL;
   static long memo_size = 0;
   if (n < 0) {
      fprintf(stderr, "oddfact: argument out of range. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   if (memo == NULL && n > 0) {
      memo_size = 2;
      memo      = calloc(memo_size, sizeof(double));
      if (memo == NULL) {
         fprintf(stderr, "oddfact: memory allocation failed. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      memo[0] = 1.0;
      memo[1] = 1.0;
   }

   if (n >= memo_size) {
      double *tmp = realloc(memo, (n + 1) * sizeof(double));
      if (tmp == NULL) {
         fprintf(stderr, "oddfact: memory allocation failed. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      memo = tmp;
      for (long i = memo_size; i <= n; i++) {
         if (i % 2 == 0)
            memo[i] = 1.0;
         else
            memo[i] = i * memo[i - 2];
      }
      memo_size = n + 1;
   }
   return memo[n];
}
/**********************************************************************/
/*  Compute fact(n)/fact(m), where n > m                              */
double factDfact(long const n, long const m)
{
   double out = 1.0;
   for (long i = MAX(m + 1, 1); i <= n; i++)
      out *= i;
   return out;
}

/**********************************************************************/
/*  Legendre Functions P(x) and sdP(x), up to Degree N and Order M    */
/*  Neumann normalization (see Battin p.390+, Wertz App. G)           */
/*  Note that dP[n][1] are singular at x = +/- 1.0.                   */
/*  sdP = sqrt(1-x^2)*dP are not singular, and are how dP's are used  */
/*  in SphericalHarmonics.                                            */
void Legendre(const long N, const long M, const double x,
              double P[N + 1][M + 1], double sdP[N + 1][M + 1])
{
   double Ps[N + 1][M + 1];
   long n, m;

   /* .. Order can't be greater than Degree */
   if (M > N) {
      fprintf(stderr, "Order %ld can't be greater than Degree %ld\n", M, N);
      exit(EXIT_FAILURE);
   }

   for (n = 0; n <= N; n++) {
      for (m = 0; m <= M; m++) {
         P[n][m]   = 0.0;
         Ps[n][m]  = 0.0;
         sdP[n][m] = 0.0;
      }
   }

   const double s = sqrt(1.0 - x * x);

   /* .. Some terms are easy */
   P[0][0]   = 1.0;
   P[1][0]   = x;
   sdP[0][0] = 0.0;
   sdP[1][0] = s;

   /* .. m=0 terms are not too bad */
   for (n = 2; n <= N; n++) {
      P[n][0]   = ((2.0 * n - 1.0) / n) * x * P[n - 1][0] -
                  ((n - 1.0) / n) * P[n - 2][0];
      sdP[n][0] = x * sdP[n - 1][0] + n * s * P[n - 1][0];
   }

   double powsm  = s;
   double powsm1 = 1.0;
   /* .. Then there are the rest... */
   for (m = 1; m <= M; m++) {
      double oddf = oddfact(2 * m - 1);
      P[m][m]     = oddf * powsm;
      Ps[m][m]    = oddf * powsm1;
      sdP[m][m]   = m * (x * Ps[m][m] - 2.0 * P[m][m - 1]);
      if (m < N) {
         P[m + 1][m]  = x * (2 * m + 1) * P[m][m];
         Ps[m + 1][m] = x * (2 * m + 1) * Ps[m][m];
         sdP[m + 1][m] =
             m * x * Ps[m + 1][m] - 2.0 * (2 * m + 1) * P[m + 1][m - 1];
      }
      for (n = m + 2; n <= N; n++) {
         P[n][m] = (x * (2 * n - 1) * P[n - 1][m] - (n + m - 1) * P[n - 2][m]) /
                   (n - m);
         Ps[n][m] =
             (x * (2 * n - 1) * Ps[n - 1][m] - (n + m - 1) * Ps[n - 2][m]) /
             (n - m);
         sdP[n][m] = m * x * Ps[n][m] - ((n + m) * (n - m + 1)) * P[n][m - 1];
      }
      powsm1  = powsm;
      powsm  *= s;
   }
}
/**********************************************************************/
/* Finds gradient of the potential V, which is parameterized by       */
/* Legendre coefficient matrices C and S, using the Neumann           */
/* ("conventional") normalization.                                    */
/* gradV[0] = Radial (positive outward)                               */
/* gradV[1] = Latitudinal (positive south)                            */
/* gradV[2] = Longitudinal (positive east)                            */
vec3_t SphericalHarmonics(const long N, const long M,
                          const sphere_coord_t coord, const double Re,
                          const double K, double **C, double **S, double **Norm)
{

   double P[N + 1][M + 1], sdP[N + 1][M + 1];
   long n, m;
   double cphi[M + 1], sphi[M + 1];
   double Rern1[N + 1], CcSs, ScCs;
   double dVdr, dVdphi, dVdtheta;

   const double r   = coord.r;
   const double cth = coord.cth;
   const double sth = coord.sth;

   /* .. Order can't be greater than Degree */
   if (M > N) {
      fprintf(stderr, "Order %ld can't be greater than Degree %ld\n", M, N);
      exit(EXIT_FAILURE);
   }

   /* .. Find Legendre functions */
   Legendre(N, M, cth, P, sdP);

   /* .. Build cos(m*phi) and sin(m*phi) */
   cphi[0] = 1.0;
   sphi[0] = 0.0;
   cphi[1] = coord.cph;
   sphi[1] = coord.sph;
   for (m = 2; m <= M; m++) {
      cphi[m] = cphi[m - 1] * cphi[1] - sphi[m - 1] * sphi[1];
      sphi[m] = sphi[m - 1] * cphi[1] + cphi[m - 1] * sphi[1];
   }

   /* .. Find gradient of V */
   dVdr     = 0.0;
   dVdphi   = 0.0;
   dVdtheta = 0.0;
   /* .. Rern1[n] = (Re/r)^(n+1) */
   Rern1[0] = Re / r;
   for (n = 1; n <= N; n++)
      Rern1[n] = Rern1[n - 1] * Rern1[0];

   // Accumulate from smallest component to largest
   for (n = N; n >= 1; n--) {
      for (m = MIN(n, M); m >= 0; m--) {
         double Pbar  = P[n][m] * Norm[n][m];
         CcSs         = C[n][m] * cphi[m] + S[n][m] * sphi[m];
         ScCs         = S[n][m] * cphi[m] - C[n][m] * sphi[m];
         dVdr        -= (CcSs * Rern1[n]) * ((n + 1) * Pbar);
         dVdphi      += (ScCs * Rern1[n]) * (m * Pbar);
         dVdtheta    -= (CcSs * Rern1[n]) * (sdP[n][m] * Norm[n][m]);
      }
   }
   dVdr     *= K / r;
   dVdphi   *= K;
   dVdtheta *= K;

   vec3_t gradV;
   gradV.v[0] = dVdr;
   gradV.v[1] = dVdtheta / r;
   if (sth == 0.0)
      gradV.v[2] = 0.0;
   else
      gradV.v[2] = dVdphi / (r * sth);
   return gradV;
}
/**********************************************************************/
/*  A is NxK, B is KxM, C is NxM                                      */
void MxMG(double **A, double **B, double **C, const long N, const long K,
          const long M)
{

   // transpose B for better cache locality
   double **BT = CreateMatrix(M, K);
   for (int i = 0; i < M; i++)
      for (int j = 0; j < K; j++)
         BT[i][j] = B[j][i];

   for (int i = 0; i < N; i++) {
      for (int j = 0; j < M; j++) {
         C[i][j] = 0.0;
         for (int k = 0; k < K; k++) {
            C[i][j] += A[i][k] * BT[j][k];
         }
      }
   }
   DestroyMatrix(BT);
}
/**********************************************************************/
/*  A is NxK, B is MxK, C is NxM                                      */
void MxMTG(double **A, double **B, double **C, const long N, const long K,
           const long M)
{
   long i, j, k;

   for (i = 0; i < N; i++) {
      for (j = 0; j < M; j++) {
         C[i][j] = 0.0;
         for (k = 0; k < K; k++) {
            C[i][j] += A[i][k] * B[j][k];
         }
      }
   }
}
/**********************************************************************/
/*  A is KxN, B is KxM, C is NxM                                      */
void MTxMG(double **A, double **B, double **C, const long N, const long K,
           const long M)
{
   long i, j, k;

   for (i = 0; i < N; i++) {
      for (j = 0; j < M; j++) {
         C[i][j] = 0.0;
         for (k = 0; k < K; k++) {
            C[i][j] += A[k][i] * B[k][j];
         }
      }
   }
}
/**********************************************************************/
void CopyVG(double *const dest, const double *const src, const long n)
{
   memcpy(dest, src, n * sizeof(double));
}
/**********************************************************************/
void SxVG(const double S, const double *V, double *W, const long n)
{
   for (long i = 0; i < n; i++)
      W[i] = S * V[i];
}
/**********************************************************************/
/* the operation y := a * x + y for an n-dimensional vec              */
void axpy(const double a, const double *const x, double *const y, const long n)
{
   for (long i = 0; i < n; i++)
      y[i] += a * x[i];
}
/**********************************************************************/
void MxVG(double **M, double *v, double *w, const long n, const long m)
{
   long i, j;

   for (i = 0; i < n; i++) {
      w[i] = 0.0;
      for (j = 0; j < m; j++) {
         w[i] += M[i][j] * v[j];
      }
   }
}
/**********************************************************************/
/*  Product of scalar S with NxM matrix A                             */
void SxMG(double s, double **A, double **B, const long N, const long M)
{
   long i, j;

   for (i = 0; i < N; i++) {
      for (j = 0; j < M; j++) {
         B[i][j] = s * A[i][j];
      }
   }
}
/**********************************************************************/
/*                  GENERAL MATRIX INVERSE                            */
/* Inverse of an NxN matrix                                           */
void MINVG(double **A, double **AI, const long N)
{
   long I, J, ROW;
   long IPIVOT = 0;
   double **M;
   double PIVOT, K, *TA, *TB;

   M  = CreateMatrix(N, N);
   TA = (double *)calloc(N, sizeof(double));
   TB = (double *)calloc(N, sizeof(double));

   for (I = 0; I < N; I++) {
      for (J = 0; J < N; J++) {
         M[I][J]  = A[I][J];
         AI[I][J] = 0.0;
      }
      AI[I][I] = 1.0;
   }

   for (ROW = 0; ROW < N; ROW++) {
      PIVOT  = M[ROW][ROW];
      IPIVOT = ROW;
      for (I = ROW + 1; I < N; I++) {
         if (fabs(M[I][ROW]) > fabs(PIVOT)) {
            PIVOT  = M[I][ROW];
            IPIVOT = I;
         }
      }
      if (PIVOT == 0.0) {
         fprintf(stderr, "Matrix is singular in MINVG\n");
         exit(EXIT_FAILURE);
      }

      for (J = 0; J < N; J++) {
         TA[J]         = M[IPIVOT][J];
         TB[J]         = AI[IPIVOT][J];
         M[IPIVOT][J]  = M[ROW][J];
         AI[IPIVOT][J] = AI[ROW][J];
         M[ROW][J]     = TA[J] / PIVOT;
         AI[ROW][J]    = TB[J] / PIVOT;
      }
      for (I = ROW + 1; I < N; I++) {
         K = M[I][ROW];
         for (J = 0; J < N; J++) {
            M[I][J]  = M[I][J] - K * M[ROW][J];
            AI[I][J] = AI[I][J] - K * AI[ROW][J];
         }
      }
   }

   /*    M is now upper diagonal */

   for (ROW = N - 1; ROW > 0; ROW--) {
      for (I = 0; I < ROW; I++) {
         K = M[I][ROW];
         for (J = 0; J < N; J++) {
            M[I][J]  = M[I][J] - K * M[ROW][J];
            AI[I][J] = AI[I][J] - K * AI[ROW][J];
         }
      }
   }

   DestroyMatrix(M);
   free(TA);
   free(TB);
}
/******************************************************************************/
/* For Order-N dynamics, we need to invert matrices of size 1 <= N <= 6       */
/* This specialized function avoids mallocs to save time                      */
void FastMINV6(const double A[6][6], double AI[6][6], const long N)
{
   long I, J, ROW;
   long IPIVOT = 0;
   double M[6][6];
   double PIVOT, K, TA[6], TB[6];

   for (I = 0; I < N; I++) {
      for (J = 0; J < N; J++) {
         M[I][J]  = A[I][J];
         AI[I][J] = 0.0;
      }
      AI[I][I] = 1.0;
   }

   for (ROW = 0; ROW < N; ROW++) {
      PIVOT  = M[ROW][ROW];
      IPIVOT = ROW;
      for (I = ROW + 1; I < N; I++) {
         if (fabs(M[I][ROW]) > fabs(PIVOT)) {
            PIVOT  = M[I][ROW];
            IPIVOT = I;
         }
      }
      if (PIVOT == 0.0) {
         fprintf(stderr, "Matrix is singular in FastMINV6\n");
         exit(EXIT_FAILURE);
      }

      for (J = 0; J < N; J++) {
         TA[J]         = M[IPIVOT][J];
         TB[J]         = AI[IPIVOT][J];
         M[IPIVOT][J]  = M[ROW][J];
         AI[IPIVOT][J] = AI[ROW][J];
         M[ROW][J]     = TA[J] / PIVOT;
         AI[ROW][J]    = TB[J] / PIVOT;
      }
      for (I = ROW + 1; I < N; I++) {
         K = M[I][ROW];
         for (J = 0; J < N; J++) {
            M[I][J]  = M[I][J] - K * M[ROW][J];
            AI[I][J] = AI[I][J] - K * AI[ROW][J];
         }
      }
   }

   /*    M is now upper diagonal */

   for (ROW = N - 1; ROW > 0; ROW--) {
      for (I = 0; I < ROW; I++) {
         K = M[I][ROW];
         for (J = 0; J < N; J++) {
            M[I][J]  = M[I][J] - K * M[ROW][J];
            AI[I][J] = AI[I][J] - K * AI[ROW][J];
         }
      }
   }
}
/**********************************************************************/
/*  Find the pseudo-inverse of an n-by-m matrix A                     */
void PINVG(double **A, double **Ai, const long n, const long m)
{
   double **AtA, **AtAi;
   double **AAt, **AAti;

   if (n == m) {
      MINVG(A, Ai, n);
   }
   else if (n > m) {
      AtA  = CreateMatrix(m, m);
      AtAi = CreateMatrix(m, m);
      MTxMG(A, A, AtA, m, n, m);
      MINVG(AtA, AtAi, m);
      MxMTG(AtAi, A, Ai, m, m, n);
      DestroyMatrix(AtA);
      DestroyMatrix(AtAi);
   }
   else {
      AAt  = CreateMatrix(n, n);
      AAti = CreateMatrix(n, n);
      MxMTG(A, A, AAt, n, m, n);
      MINVG(AAt, AAti, n);
      MTxMG(A, AAti, Ai, m, n, n);
      DestroyMatrix(AAt);
      DestroyMatrix(AAti);
   }
}
/**********************************************************************/
double **CreateMatrix(const long n, const long m)
{
   double **A;
   long i;

   // Throw warning??
   // this will happen sometimes with the graphics
   if (n == 0 || m == 0)
      return NULL;

   // Guarantee the allocation for A is a contiguous block
   A = (double **)malloc(sizeof(double *) * n);
   if (A == NULL) {
      fprintf(stderr, "malloc failed in CreateMatrix.  Bailing out.\n");
      exit(EXIT_FAILURE);
   }
   A[0] = (double *)calloc(n * m, sizeof(double));
   if (A[0] == NULL) {
      fprintf(stderr, "calloc failed in CreateMatrix.  Bailing out.\n");
      exit(EXIT_FAILURE);
   }
   for (i = 1; i < n; i++)
      A[i] = A[0] + m * i;

   return (A);
}
/**********************************************************************/
void DestroyMatrix(double **A)
{
   if (A == NULL)
      return;
   free(A[0]);
   free(A);
   A = NULL;
}
/**********************************************************************/
/*   Solution of NxN system      A * x = b                            */
/*   by Gaussian Elimination and Back Substitution, with pivoting     */
void LINSOLVE(double **A, double *x, double *b, const long n)
{
   long i, j, k, l, m;
   double mm, *a1, b1;

   if (n == 1) {
      x[0] = b[0] / A[0][0];
      return;
   }

   a1 = (double *)calloc(n, sizeof(double));

   for (j = 0; j < n - 1; j++) {
      mm = fabs(A[j][j]);
      l  = j;
      for (i = j + 1; i < n; i++) {
         if (fabs(A[i][j]) >= mm) {
            l  = i;
            mm = fabs(A[i][j]);
         }
      }
      if (l != j) {
         for (i = 0; i < n; i++) {
            a1[i] = A[j][i];
         }
         b1 = b[j];
         for (i = j; i < n; i++) {
            A[j][i] = A[l][i] / A[l][j];
         }
         b[j] = b[l] / A[l][j];
         for (i = 0; i < n; i++) {
            A[l][i] = a1[i];
         }
         b[l] = b1;
      }
      else {
         b[j] = b[j] / A[j][j];
         for (i = n - 1; i >= j; i--) {
            A[j][i] = A[j][i] / A[j][j];
         }
      }
      for (k = j + 1; k < n; k++) {
         b[k] -= A[k][j] * b[j];
      }
      for (k = j + 1; k < n; k++) {
         for (m = n - 1; m >= j; m--) {
            A[k][m] -= A[k][j] * A[j][m];
         }
      }
   }
   x[n - 1] = b[n - 1] / A[n - 1][n - 1];
   for (i = n - 2; i >= 0; i--) {
      x[i] = b[i];
      for (k = i + 1; k < n; k++) {
         x[i] -= A[i][k] * x[k];
      }
   }

   free(a1);
}
/**********************************************************************/
/*  Solution of the linear equations A*x=b by Cholesky Decomposition  */
/*  This method can only be used if A is positive definite            */
/*  (symmetric), but it is roughly twice as fast as Gaussian          */
/*  Elimination.                                                      */
/*  In testing, this didn't live up to the hype, being slightly       */
/*  slower than LINSOLVE.  I must have an inefficiency.               */
void CholeskySolve(double **A, double *x, double *b, const long n)
{
   double **L, *D, **LD;
   double *y;
   long i, j, k;

   L  = CreateMatrix(n, n);
   D  = (double *)calloc(n, sizeof(double));
   LD = CreateMatrix(n, n);
   y  = (double *)calloc(n, sizeof(double));

   /* .. Find L, D */
   for (j = 0; j < n; j++) {
      D[j] = A[j][j];
      for (k = 0; k < j; k++)
         D[j] -= L[j][k] * L[j][k] * D[k];
      for (i = 0; i < n; i++) {
         L[i][j] = A[i][j];
         for (k = 0; k < j; k++)
            L[i][j] -= L[i][k] * L[j][k] * D[k];
         L[i][j] /= D[j];
      }
   }

   /* .. Find LD */
   for (i = 0; i < n; i++) {
      LD[i][i] = D[i];
      for (j = i + 1; j < n; j++)
         LD[j][i] = L[j][i] * D[i];
   }

   /* .. Solve LD*y = b */
   for (i = 0; i < n; i++) {
      y[i] = b[i];
      for (k = 0; k < i; k++)
         y[i] -= LD[i][k] * y[k];
      y[i] /= LD[i][i];
   }
   /* .. Solve Lt*x = y */
   for (i = n - 1; i >= 0; i--) {
      x[i] = y[i];
      for (k = n - 1; k > i; k--)
         x[i] -= L[k][i] * x[k];
   }

   DestroyMatrix(L);
   DestroyMatrix(LD);
   free(D);
   free(y);
}
/**********************************************************************/
/* Solution of linear equations by Conjugate Gradient method.         */
/* See "An Introduction to the Conjugate Gradient Method              */
/* Without the Agonizing Pain", by Jonathan Richard Shewchuk          */
void ConjGradSolve(double **A, double *x, double *b, const long n,
                   const double errtol, const long maxiter)
{
   double *r, *d, *q;
   double DeltaNew, DeltaOld, Err2D0, alpha, Beta, dq;
   long i, j, Iter;

   r = (double *)calloc(n, sizeof(double));
   d = (double *)calloc(n, sizeof(double));
   q = (double *)calloc(n, sizeof(double));

   /* .. r = b - A*x, d = r, DeltaNew = r'*r */
   DeltaNew = 0.0;
   for (i = 0; i < n; i++) {
      r[i] = b[i];
      for (j = 0; j < n; j++)
         r[i] -= A[i][j] * x[j];
      d[i]      = r[i];
      DeltaNew += r[i] * r[i];
   }

   /* .. Error tolerance end condition */
   Err2D0 = errtol * errtol * DeltaNew;

   Iter = 0;
   while (Iter < maxiter && DeltaNew > Err2D0) {
      /* q = A*d */
      dq = 0.0;
      for (i = 0; i < n; i++) {
         q[i] = 0.0;
         for (j = 0; j < n; j++)
            q[i] += A[i][j] * d[j];
         dq += d[i] * q[i];
      }
      /* alpha = DeltaNew/d'*q */
      alpha = DeltaNew / dq;

      /* x += alpha*d */
      for (i = 0; i < n; i++)
         x[i] += alpha * d[i];

      if (Iter % 50 == 0) {
         for (i = 0; i < n; i++) {
            r[i] = b[i];
            for (j = 0; j < n; j++)
               r[i] -= A[i][j] * x[j];
         }
      }
      else {
         for (i = 0; i < n; i++)
            r[i] -= alpha * q[i];
      }
      /* DeltaOld = DeltaNew */
      DeltaOld = DeltaNew;

      /* DeltaNew = r'*r */
      DeltaNew = 0.0;
      for (i = 0; i < n; i++)
         DeltaNew += r[i] * r[i];

      /* Beta = DeltaNew/DeltaOld */
      Beta = DeltaNew / DeltaOld;

      /* d = r + beta*d */
      for (i = 0; i < n; i++)
         d[i] = r[i] + Beta * d[i];

      Iter++;
   }

   free(r);
   free(d);
   free(q);
}
/************************************************************************/
/*  Find Roots of a Polynomial using Bairstow's Method                  */
/*  a = Coefficients of polynomial (length n+1)                         */
/*  Real = Real parts of roots (length n)                               */
/*  Imag = Imaginary parts of roots (length n)                          */
void Bairstow(long n, double *a, const double Tol, double *Real, double *Imag)
{

   double *b, *c;
   double r, s, dr, ds, Disc, Det;
   long Done;
   long i;

   b = (double *)calloc(n + 1, sizeof(double));
   c = (double *)calloc(n + 1, sizeof(double));

   while (n > 2) {

      /* Initial Guesses */
      if (a[n - 2] != 0.0) {
         r = -a[n - 1] / a[n - 2];
         s = -a[n] / a[n - 2];
      }
      else {
         r = -a[1] / a[0];
         s = -a[2] / a[0];
      }

      b[0] = a[0];
      c[0] = b[0];

      /* Search for quadratic factor */
      Done = 0;
      while (!Done) {
         b[1] = a[1] + b[0] * r;
         c[1] = b[1] + c[0] * r;
         for (i = 2; i < n + 1; i++) {
            b[i] = a[i] + r * b[i - 1] + s * b[i - 2];
            c[i] = b[i] + r * c[i - 1] + s * c[i - 2];
         }
         Det = c[n - 1] * c[n - 3] - c[n - 2] * c[n - 2];
         dr  = (b[n - 1] * c[n - 2] - b[n] * c[n - 3]) / Det;
         ds  = (b[n] * c[n - 2] - b[n - 1] * c[n - 1]) / Det;
         r   = r + dr;
         s   = s + ds;

         if (fabs(dr) < Tol * MAX(1.0, r) && fabs(ds) < Tol * MAX(1.0, s))
            Done = 1;
      }

      /* Store roots of quadratic factor */
      Disc = r * r + 4.0 * s;
      if (Disc < 0.0) {
         Real[n - 1] = 0.5 * r;
         Imag[n - 1] = 0.5 * sqrt(-Disc);
         Real[n - 2] = Real[n - 1];
         Imag[n - 2] = -Imag[n - 1];
      }
      else {
         Real[n - 1] = 0.5 * (r + sqrt(Disc));
         Imag[n - 1] = 0.0;
         Real[n - 2] = 0.5 * (r - sqrt(Disc));
         Imag[n - 2] = 0.0;
      }

      /* Deflate polynomial */
      n -= 2;
      for (i = 0; i < n + 1; i++)
         a[i] = b[i];

   } /* End deflation */

   /* Find roots of remaining first/second order polynomial */
   if (n == 1) {
      Real[0] = -a[1] / a[0];
      Imag[0] = 0.0;
   }
   else {
      r    = -a[1] / a[0];
      s    = -a[2] / a[0];
      Disc = r * r + 4 * s;
      if (Disc < 0.0) {
         Real[1] = 0.5 * r;
         Imag[1] = 0.5 * sqrt(-Disc);
         Real[0] = Real[1];
         Imag[0] = -Imag[1];
      }
      else {
         Real[1] = 0.5 * (r + sqrt(Disc));
         Imag[1] = 0.0;
         Real[0] = 0.5 * (r - sqrt(Disc));
         Imag[0] = 0.0;
      }
   }
   free(b);
   free(c);
}
/**********************************************************************/
/*  Minimize a cost function by Downhill Simplex Method               */
/*  See Numerical Recipes 10.4 for description of method              */
/*                                                                    */
/*  N = Number of dimensions                                          */
/*  P = Vector which minimizes cost function                          */
/*      (Initial guess in, result out)                                */
/*  CostFunction must take an N-vector as argument,                   */
/*               must return a double                                 */
/*  scale = Size of initial amoeba                                    */
/*  Tol = Tolerance on cost function to declare convergence           */
double Amoeba(const long N, double *P,
              double CostFunction(double *p, double *Parm), double *CostParm,
              const double scale, const double Tol)
{

   long Converged = 0;
   double **p, *pc, *f;
   long high, nexthigh, low;
   double Coef1, Coef2, *pn, fn, StepSize;
   double MinCost;
   long i, j;

   p = (double **)calloc(N + 1, sizeof(double *));
   for (i = 0; i < N + 1; i++)
      p[i] = (double *)calloc(N, sizeof(double));

   pc = (double *)calloc(N, sizeof(double));
   f  = (double *)calloc(N + 1, sizeof(double));
   pn = (double *)calloc(N, sizeof(double));

   /* Simplex */
   for (j = 0; j < N; j++)
      p[0][j] = P[j];
   for (i = 1; i < N + 1; i++) {
      for (j = 0; j < N; j++) {
         p[i][j] = p[0][j];
      }
      p[i][i - 1] += scale;
   }

   /* Simplex Centroid */
   for (j = 0; j < N; j++) {
      pc[j] = 0.0;
      for (i = 0; i < N + 1; i++) {
         pc[j] += p[i][j];
      }
      pc[j] /= (N + 1.0);
   }

   /* Evaluate cost function */
   for (i = 0; i < N + 1; i++) {
      f[i] = CostFunction(p[i], CostParm);
   }

   /* Find high, next-to-high, low cost */
   low  = 0;
   high = 0;
   for (i = 1; i < N + 1; i++) {
      if (f[i] < f[low])
         low = i;
      if (f[i] > f[high])
         high = i;
   }
   nexthigh = low;
   for (i = 0; i < N + 1; i++) {
      if (f[i] > f[nexthigh] && f[i] < f[high])
         nexthigh = i;
   }

   while (!Converged) {

      /* Try Reflection */
      StepSize = -1.0;
      Coef1    = (N + 1.0) / N * (1.0 - StepSize);
      Coef2    = -(1.0 - (N + 1.0) * StepSize) / N;
      for (j = 0; j < N; j++) {
         pn[j] = Coef1 * pc[j] + Coef2 * p[high][j];
      }
      fn = CostFunction(pn, CostParm);
      if (fn < f[high]) {
         f[high] = fn;
         for (j = 0; j < N; j++) {
            pc[j]      += (pn[j] - p[high][j]) / (N + 1);
            p[high][j]  = pn[j];
         }
      }
      if (f[high] < f[low]) { /* Worked so well, try longer step */
         StepSize = 2.0;
         Coef1    = (N + 1.0) / N * (1.0 - StepSize);
         Coef2    = -(1.0 - (N + 1.0) * StepSize) / N;
         for (j = 0; j < N; j++) {
            pn[j] = Coef1 * pc[j] + Coef2 * p[high][j];
         }
         fn = CostFunction(pn, CostParm);
         if (fn < f[high]) {
            f[high] = fn;
            for (j = 0; j < N; j++) {
               pc[j]      += (pn[j] - p[high][j]) / (N + 1);
               p[high][j]  = pn[j];
            }
         }
      }
      else if (f[high] >
               f[nexthigh]) { /* Worked not so well, so try contraction */
         StepSize = 0.5;
         Coef1    = (N + 1.0) / N * (1.0 - StepSize);
         Coef2    = -(1.0 - (N + 1.0) * StepSize) / N;
         for (j = 0; j < N; j++) {
            pn[j] = Coef1 * pc[j] + Coef2 * p[high][j];
         }
         fn = CostFunction(pn, CostParm);
         if (fn < f[high]) {
            f[high] = fn;
            for (j = 0; j < N; j++) {
               pc[j]      += (pn[j] - p[high][j]) / (N + 1);
               p[high][j]  = pn[j];
            }
         }
         if (f[high] > f[nexthigh]) { /* Stuck.  Contract about lowest point */
            for (i = 0; i < N + 1; i++) {
               if (i != low) {
                  for (j = 0; j < N; j++) {
                     p[i][j] = 0.5 * (p[i][j] + p[low][j]);
                  }
                  f[i] = CostFunction(p[i], CostParm);
               }
            }
            /* Find centroid */
            for (j = 0; j < N; j++) {
               pc[j] = 0.0;
               for (i = 0; i < N + 1; i++) {
                  pc[j] += p[i][j];
               }
               pc[j] /= (N + 1.0);
            }
         }
      }

      /* Find high, next-to-high, low cost */
      low  = 0;
      high = 0;
      for (i = 1; i < N + 1; i++) {
         if (f[i] < f[low])
            low = i;
         if (f[i] > f[high])
            high = i;
      }
      nexthigh = low;
      for (i = 0; i < N + 1; i++) {
         if (f[i] > f[nexthigh] && f[i] < f[high])
            nexthigh = i;
      }

      /* Termination Condition */
      if ((fabs(f[low] / f[high] - 1.0) < Tol) ||
          (fabs(f[low] - f[high]) < Tol)) {
         Converged = 1;
      }
   }
   for (j = 0; j < N; j++)
      P[j] = p[low][j];
   MinCost = f[low];

   for (i = 0; i < N + 1; i++)
      free(p[i]);
   free(p);
   free(pc);
   free(f);
   free(pn);

   return (MinCost);
}
/**********************************************************************/
/*  Find unit normal vector to plane defined by points V1, V2, V3     */
vec3_t FindNormal(const vec3_t V1, const vec3_t V2, const vec3_t V3)
{
   long i;
   vec3_t D1, D2;

   for (i = 0; i < 3; i++) {
      D1.v[i] = V2.v[i] - V1.v[i];
      D2.v[i] = V3.v[i] - V2.v[i];
   }
   magvec3_t N;
   N.v = VxV(D1, D2);
   N   = UNITV(N.v);
   return N.v;
}
/**********************************************************************/
/*  Output clamped at ends of interval                                */
double LinInterp(const double *X, const double *Y, const double x, const long n)
{
   double dx, dxn, y;
   long i, i1, i2;

   dx  = x - X[0];
   dxn = X[n - 1] - X[0];
   if (fabs(dxn) < fabs(dx)) {
      printf("LinInterp clamped to 'right' end of interval\n");
      y = Y[n - 1];
   }
   else if (dx * dxn < 0.0) {
      printf("LinInterp clamped to 'left' end of interval\n");
      y = Y[0];
   }
   else {
      /* Binary Search */
      i1 = 0;
      i2 = n - 1;
      while (i1 + 1 < i2) {
         i = (i1 + i2) / 2;
         if (fabs(X[i] - X[0]) < fabs(dx))
            i1 = i;
         else
            i2 = i;
      }
      y = (Y[i2] - Y[i1]) / (X[i2] - X[i1]) * (x - X[i1]) + Y[i1];
   }
   return (y);
}
/**********************************************************************/
/*  SLERP = Spherical Linear Interpolation                            */
/*  A constant-rate interpolation for quaternions                     */
/*  Ref: Ken Shoemake, "Animating Rotation with Quaternion Curves"    */
/*  q(u=0.0) = q1, q(u=1.0) = q2                                      */
quat_t SphereInterp(quat_t q1, quat_t q2, const double u)
{
   quat_t q = QUAT_ZERO;
   double Theta, CosTheta, SinTheta;
   double SinU, Sin1mU;
   long k;

   CosTheta = VoV(q1.qv, q2.qv) + q1.qs * q2.qs;
   if (CosTheta >= 1.0) {
      for (k = 0; k < 4; k++)
         q.q[k] = q1.q[k];
   }
   else {
      SinTheta = sqrt(1.0 - CosTheta * CosTheta);
      Theta    = asin(SinTheta);
      SinU     = sin(u * Theta);
      Sin1mU   = sin((1.0 - u) * Theta);
      for (k = 0; k < 4; k++)
         q.q[k] = (SinU * q2.q[k] + Sin1mU * q1.q[k]) / SinTheta;
   }
   return q;
}
/**********************************************************************/
double CubicInterp1D(double f0, double f1, double x)
{
   double x1 = 1.0 - x;
   return ((3.0 - 2.0 * x1) * x1 * x1 * f0 + (3.0 - 2.0 * x) * x * x * f1);
}
/**********************************************************************/
double CubicInterp2D(double f00, double f10, double f01, double f11, double x,
                     double y)
{
   double f0 = CubicInterp1D(f00, f10, x);
   double f1 = CubicInterp1D(f01, f11, x);
   return (CubicInterp1D(f0, f1, y));
}
/**********************************************************************/
double CubicInterp3D(double f000, double f100, double f010, double f110,
                     double f001, double f101, double f011, double f111,
                     double x, double y, double z)
{
   double f0 = CubicInterp2D(f000, f100, f010, f110, x, y);
   double f1 = CubicInterp2D(f001, f101, f011, f111, x, y);
   return (CubicInterp1D(f0, f1, z));
}
/**********************************************************************/
double DistanceToLine(vec3_t LineEnd1, vec3_t LineEnd2, vec3_t Point,
                      vec3_t *VecToLine)
{
   vec3_t Vec;
   magvec3_t uAxis;
   vec3_t *const Axis = &uAxis.v;
   double VoA;
   long i;

   for (i = 0; i < 3; i++) {
      Axis->v[i] = LineEnd2.v[i] - LineEnd1.v[i];
      Vec.v[i]   = Point.v[i] - LineEnd1.v[i];
   }
   uAxis = UNITV(*Axis);
   VoA   = VoV(Vec, *Axis);

   for (i = 0; i < 3; i++)
      VecToLine->v[i] = VoA * Axis->v[i] - Vec.v[i];
   return (MAGV(*VecToLine));
}
/**********************************************************************/
long ProjectPointOntoPoly(vec3_t Point, vec3_t DirVec, vec3_t *Vtx, long Nvtx,
                          vec3_t *ProjPoint, double *Distance)
{
   vec3_t Axis, a1, a2, S1xS2;
   magvec3_t uNorm, us1, us2;
   vec3_t *const Norm = &uNorm.v;
   vec3_t *const s1   = &us1.v;
   vec3_t *const s2   = &us2.v;
   static double **COEF, *RHS, *x;
   double SumAng, SinAng, CosAng;
   long i, j, Iv, Nwrap;
   static long First = 1;
   long OnEdge;

   if (First) {
      First = 0;
      COEF  = CreateMatrix(4, 4);
      RHS   = (double *)calloc(4, sizeof(double));
      x     = (double *)calloc(4, sizeof(double));
   }

   Axis = UNITV(DirVec).v;
   for (i = 0; i < 3; i++) {
      a1.v[i] = Vtx[1].v[i] - Vtx[0].v[i];
      a2.v[i] = Vtx[2].v[i] - Vtx[0].v[i];
   }
   COEF[0][0] = a1.y * a2.z - a1.z * a2.y;
   COEF[0][1] = a1.z * a2.x - a1.x * a2.z;
   COEF[0][2] = a1.x * a2.y - a1.y * a2.x;
   COEF[0][3] = 0.0;
   RHS[0] =
       COEF[0][0] * Vtx[0].x + COEF[0][1] * Vtx[0].y + COEF[0][2] * Vtx[0].z;
   COEF[1][0] = 1.0;
   COEF[1][1] = 0.0;
   COEF[1][2] = 0.0;
   COEF[1][3] = -Axis.x;
   COEF[2][0] = 0.0;
   COEF[2][1] = 1.0;
   COEF[2][2] = 0.0;
   COEF[2][3] = -Axis.y;
   COEF[3][0] = 0.0;
   COEF[3][1] = 0.0;
   COEF[3][2] = 1.0;
   COEF[3][3] = -Axis.z;
   VEC3_TO_DBL(RHS, Point);
   LINSOLVE(COEF, x, RHS, 4);
   for (i = 0; i < 3; i++)
      ProjPoint->v[i] = x[i];
   *Distance = x[3];

   /* Find whether ProjPoint lies in polygon */
   *Norm  = VxV(a1, a2);
   uNorm  = UNITV(*Norm);
   SumAng = 0.0;
   OnEdge = 0;
   for (Iv = 0; Iv < Nvtx; Iv++) {
      for (j = 0; j < 3; j++) {
         s1->v[j] = Vtx[Iv].v[j] - ProjPoint->v[j];
         s2->v[j] = Vtx[(Iv + 1) % Nvtx].v[j] - ProjPoint->v[j];
      }
      us1    = UNITV(*s1);
      us2    = UNITV(*s2);
      S1xS2  = VxV(*s1, *s2);
      SinAng = VoV(S1xS2, *Norm);
      CosAng = VoV(*s1, *s2);
      if (fabs(SinAng) < 1.0E-6 && CosAng < -0.9)
         OnEdge = 1;
      SumAng += atan2(SinAng, CosAng);
   }
   Nwrap = (long)(SumAng / 6.283 + 0.5);

   /* If Nwrap is odd, then ProjPoint lies within polygon */
   return (Nwrap % 2 || OnEdge);
}
/*********************************************************************/
/* Given Triangle ABC, a point Pt, and a direction vector,           */
/* find the projection of Pt onto ABC.  Barycentric coords have      */
/* fourth element, so that                                           */
/* Pt = Bary[0]*A + Bary[1]*B + Bary[2]*C + Bary[3]*DirVec           */
long ProjectPointOntoTriangle(vec3_t A, vec3_t B, vec3_t C, vec3_t DirVec,
                              vec3_t Pt, vec3_t *ProjPt, vec4_t *Bary)
{
   double Den, NumA, NumB, NumC, NumD;
   vec3_t AxB, CxD, PxB, AxP, CxP, PxD;
   double M[4][3], Mplus[3][4];
   long InPoly, i;

   AxB = VxV(A, B);
   CxD = VxV(C, DirVec);

   Den = (A.v[0] - B.v[0]) * CxD.v[0] + (A.v[1] - B.v[1]) * CxD.v[1] +
         (A.v[2] - B.v[2]) * CxD.v[2] - DirVec.v[0] * AxB.v[0] -
         DirVec.v[1] * AxB.v[1] - DirVec.v[2] * AxB.v[2];

   if (fabs(Den) < 1.0E-12) {
      /* If DirVec is in plane of ABC, then problem reduces to... */
      for (i = 0; i < 3; i++) {
         M[i][0] = A.v[i];
         M[i][1] = B.v[i];
         M[i][2] = C.v[i];
         M[3][i] = 1.0;
      }
      PINV4x3(M, Mplus);
      Bary->q[0] =
          Mplus[0][0] * Pt.v[0] + Mplus[0][1] * Pt.v[1] + Mplus[0][2] * Pt.v[2];
      Bary->q[1] =
          Mplus[1][0] * Pt.v[0] + Mplus[1][1] * Pt.v[1] + Mplus[1][2] * Pt.v[2];
      Bary->q[2] =
          Mplus[2][0] * Pt.v[0] + Mplus[2][1] * Pt.v[1] + Mplus[2][2] * Pt.v[2];
      Bary->q[3] = 0.0;
   }
   else {
      PxB = VxV(Pt, B);
      AxP = VxV(A, Pt);
      CxP = VxV(C, Pt);
      PxD = VxV(Pt, DirVec);

      NumA = (Pt.v[0] - B.v[0]) * CxD.v[0] + (Pt.v[1] - B.v[1]) * CxD.v[1] +
             (Pt.v[2] - B.v[2]) * CxD.v[2] - DirVec.v[0] * PxB.v[0] -
             DirVec.v[1] * PxB.v[1] - DirVec.v[2] * PxB.v[2];

      NumB = (A.v[0] - Pt.v[0]) * CxD.v[0] + (A.v[1] - Pt.v[1]) * CxD.v[1] +
             (A.v[2] - Pt.v[2]) * CxD.v[2] - DirVec.v[0] * AxP.v[0] -
             DirVec.v[1] * AxP.v[1] - DirVec.v[2] * AxP.v[2];

      NumC = (A.v[0] - B.v[0]) * PxD.v[0] + (A.v[1] - B.v[1]) * PxD.v[1] +
             (A.v[2] - B.v[2]) * PxD.v[2] - DirVec.v[0] * AxB.v[0] -
             DirVec.v[1] * AxB.v[1] - DirVec.v[2] * AxB.v[2];

      NumD = (A.v[0] - B.v[0]) * CxP.v[0] + (A.v[1] - B.v[1]) * CxP.v[1] +
             (A.v[2] - B.v[2]) * CxP.v[2] - (Pt.v[0] - C.v[0]) * AxB.v[0] -
             (Pt.v[1] - C.v[1]) * AxB.v[1] - (Pt.v[2] - C.v[2]) * AxB.v[2];

      Bary->q[0] = NumA / Den;
      Bary->q[1] = NumB / Den;
      Bary->q[2] = NumC / Den;
      Bary->q[3] = NumD / Den;
   }

   ProjPt->v[0] =
       Bary->q[0] * A.v[0] + Bary->q[1] * B.v[0] + Bary->q[2] * C.v[0];
   ProjPt->v[1] =
       Bary->q[0] * A.v[1] + Bary->q[1] * B.v[1] + Bary->q[2] * C.v[1];
   ProjPt->v[2] =
       Bary->q[0] * A.v[2] + Bary->q[1] * B.v[2] + Bary->q[2] * C.v[2];

   InPoly =
       (Bary->q[0] >= 0.0 && Bary->q[1] >= 0.0 && Bary->q[2] >= 0.0 ? 1 : 0);

   return (InPoly);
}
/**********************************************************************/
double CubicSpline(double x, double X[4], double Y[4])
{
   double DY0, DY2, DY3;
   double Det, u0, u3, u;
   double z0, z3, u02, u32;
   double a, b, c, d;

   u = (x - X[1]) / (X[2] - X[1]);

   if (isnan(u)) {
      fprintf(stderr, "Bad spline interval in CubicSpline.\n");
      exit(EXIT_FAILURE);
   }
   if (u < 0.0 || u > 1.0) {
      fprintf(stderr, "Interpolant out of range in CubicSpline.\n");
      exit(EXIT_FAILURE);
   }

   DY0 = Y[0] - Y[1];
   DY2 = Y[2] - Y[1];
   DY3 = Y[3] - Y[1];

   u0 = (X[0] - X[1]) / (X[2] - X[1]);
   u3 = (X[3] - X[1]) / (X[2] - X[1]);

   z0  = u0 - 1.0;
   z3  = u3 - 1.0;
   u02 = u0 * u0;
   u32 = u3 * u3;

   Det = (u3 - 1.0) * (u0 - 1.0) * (u3 - u0) * u0 * u3;
   if (fabs(Det) < 1.0E-9) {
      fprintf(stderr, "Matrix is close to singular in CubicSpline.\n");
      exit(EXIT_FAILURE);
   }
   a = Y[1];
   b = (-z3 * u32 * DY0 + (u3 - u0) * u02 * u32 * DY2 + z0 * u02 * DY3) / Det;
   c = (z3 * (u3 + 1.0) * u3 * DY0 - (u3 - u0) * (u3 + u0) * u0 * u3 * DY2 -
        z0 * (u0 + 1.0) * u0 * DY3) /
       Det;
   d = (-z3 * u3 * DY0 + (u3 - u0) * u0 * u3 * DY2 + z0 * u0 * DY3) / Det;

   return (a + u * (b + u * (c + u * d)));
}
/******************************************************************************/
/* Compute Chebyshev polynomials of first kind (T) and second kind (U)        */
void ChebyPolys(double u, long n, double T[20], double U[20])
{
   long k;

   if (u < -1.0 || u > 1.0) {
      fprintf(stderr, "u out of range in ChebPolys.  Bailing out.\n");
      exit(EXIT_FAILURE);
   }
   if (n > 20) {
      fprintf(stderr, "n out of range in ChebPolys.  Bailing out.\n");
      exit(EXIT_FAILURE);
   }

   T[0] = 1.0;
   T[1] = u;
   U[0] = 1.0;
   U[1] = 2.0 * u;
   for (k = 1; k < n - 1; k++) {
      T[k + 1] = 2.0 * u * T[k] - T[k - 1];
      U[k + 1] = 2.0 * u * U[k] - U[k - 1];
   }
}
/******************************************************************************/
/* Using ChebyPolys, find "position" (P) and scaled velocity (dPdu)           */
void ChebyInterp(double T[20], double U[20], double Coef[20], long n, double *P,
                 double *dPdu)
{
   long k;

   if (n > 20) {
      fprintf(stderr, "n out of range in ChebyInterp.  Bailing out.\n");
      exit(EXIT_FAILURE);
   }

   *P    = Coef[0] * T[0];
   *dPdu = 0.0;
   for (k = 1; k < n; k++) {
      *P    += Coef[k] * T[k];
      *dPdu += Coef[k] * ((double)k) * U[k - 1];
   }
}
/******************************************************************************/
void FindChebyCoefs(double *u, double *P, long Nu, long Nc, double Coef[20])
{
   long i, j, k;
   double T[20], U[20];
   double **AtA, *x, *Atb;

   if (Nc > 20) {
      fprintf(stderr, "Nc out of range in FindChebyCoefs.  Bailing out.\n");
      exit(EXIT_FAILURE);
   }

   AtA = CreateMatrix(Nc, Nc);
   x   = (double *)calloc(Nc, sizeof(double));
   Atb = (double *)calloc(Nc, sizeof(double));

   for (k = 0; k < Nu; k++) {
      ChebyPolys(u[k], Nc, T, U);
      for (i = 0; i < Nc; i++) {
         for (j = 0; j < Nc; j++) {
            AtA[i][j] += T[i] * T[j];
         }
         Atb[i] += T[i] * P[k];
      }
   }
   LINSOLVE(AtA, x, Atb, Nc);
   for (i = 0; i < Nc; i++)
      Coef[i] = x[i];
   for (i = Nc; i < 20; i++)
      Coef[i] = 0.0;

   DestroyMatrix(AtA);
   free(x);
   free(Atb);
}
/******************************************************************************/
void VecToLngLat(vec3_t A, double *lng, double *lat)
{
   vec3_t B;

   if (MAGV(A) > 0.0) {
      B = UNITV(A).v;

      *lng = atan2(B.v[1], B.v[0]);

      if (fabs(B.v[2]) < 1.0)
         *lat = asin(B.v[2]);
      else if (B.v[2] > 0.0)
         *lat = HALFPI;
      else
         *lat = -HALFPI;
   }
   else {
      *lng = 0.0;
      *lat = 0.0;
   }
}
/******************************************************************************/
double WrapTo2Pi(double n)
{
   double OrbVar = fmod(n, TWOPI);
   if (OrbVar < 0.0)
      OrbVar += TWOPI;
   return (OrbVar);
}
/******************************************************************************/
/* Simple Newton-Raphson method for function given by f/dfdx = fdf            */
/* Iterates until tolerance or max iterations are reached; maximum stepsize   */
/* governed by maxStep. Use params to pass parameters to fdf                  */
// TODO: This has difficulty with high multiplicity roots and starting on the
// "other" side of a section of a function with zero derivative. E.g
// function 12.23459071*x^3 + 54.9176*x^2 - 23.39456*x + 97.1235 and x0 = 15
double NewtonRaphson(double x0, double tol, long nMax, double maxStep,
                     long breakOnZeroF, double (*fdf)(const double, double *),
                     double *params)
{
   if (maxStep < 0)
      maxStep = -maxStep;
   double x = x0;
   double dx;
   double f = 0.0;
   long k   = 0;
   do {
      // TODO: what to do if fp=f' is small? break or perturb??
      dx = fdf(x, params);
      if (fabs(dx) > maxStep)
         dx = signum(dx) * maxStep;
      x -= dx;
   } while ((!breakOnZeroF || fabs(f) > tol) && fabs(dx) > tol && k++ < nMax);
   return x;
}
/******************************************************************************/
/* Helper for Brent's Method                                                  */
static double _inv_quad_int(double a, double fa, double fb, double fc)
{
   return a * fb * fc / ((fa - fb) * (fa - fc));
}
/******************************************************************************/
/* Find root for function f in the domain [a0, b0] by Brent's Method          */
double BrentsMethod(double a, double b, const double tol,
                    double (*f)(const double, double *), double *params)
{
   if (a == b)
      return a;

   const double tol_abs = fabs(tol);

   double fa = f(a, params);
   double fb = f(b, params);

   if (fa * fb >= 0) {
      // fa and fb are same sign (or zero)
      //    return the value associated with the smaller one
      if (fa == 0)
         return fa;
      if (fb == 0)
         return fb;
      const double mag_fa = fabs(fa);
      const double mag_fb = fabs(fb);
      if (mag_fa < mag_fb)
         return a;
      else
         return b;
   }

   if (fabs(fa) < fabs(fb)) {
      double t = a;
      a        = b;
      b        = t;

      t  = fa;
      fa = fb;
      fb = t;
   }

   double c = a, d = 0.0;
   double fc = fa;
   int mflag = 1;

   double err = fabs(b - a);
   while (fb != 0 && err > tol_abs) {
      double s = 0;
      if (fa != fc && fb != fc)
         // inverse quadratic interpolation
         s = _inv_quad_int(a, fa, fb, fc) + _inv_quad_int(b, fb, fc, fa) +
             _inv_quad_int(c, fc, fa, fb);
      else
         // secant method
         s = b - fb * (b - a) / (fb - fa);

      const double tmp = (3.0 * a + b) / 4.0;
      const int cond_1 = !((tmp > b) ? (b < s && s < tmp) : (tmp < s && s < b));
      const int cond_2 = (mflag) && (fabs(s - b) >= (fabs(b - c) / 2.0));
      const int cond_3 = (!mflag) && (fabs(s - b) >= (fabs(c - d) / 2.0));
      const int cond_4 = (mflag) && (fabs(b - c) < tol_abs);
      const int cond_5 = (!mflag) && (fabs(c - d) < tol_abs);
      if (cond_1 || cond_2 || cond_3 || cond_4 || cond_5) {
         // bisection method
         s     = (a + b) / 2;
         mflag = 1;
      }
      else
         mflag = 0;

      d  = c;
      c  = b;
      fc = fb;
      // determine what sign fs is, and replace one of the brackets with it
      double fs = f(s, params);
      if (fa * fs < 0) {
         b  = s;
         fb = fs;
      }
      else {
         a  = s;
         fa = fs;
      }
      if (fabs(fa) < fabs(fb)) {
         double t = a;
         a        = b;
         b        = t;

         t  = fa;
         fa = fb;
         fb = t;
      }

      err = fabs(b - a);
      if (fabs(a) > __DBL_EPSILON__)
         err /= a;
   }
   return b;
}
/******************************************************************************/
/* Get Trigonometric values of Azimuth and Elevation and magnitude from 3D    */
/* vector                                                                     */
sphere_coord_t getTrigSphericalCoords(const vec3_t pbe)
{
   sphere_coord_t out;
   out.r              = MAGV(pbe);
   const double denom = sqrt(pbe.y * pbe.y + pbe.x * pbe.x);
   out.cth            = pbe.z / out.r;                 // cos(theta)
   out.sth            = sqrt(1.0 - out.cth * out.cth); // sin(theta);
   out.cph            = pbe.x / denom;                 // cos(phi);
   out.sph            = pbe.y / denom;                 // sin(phi);
   return out;
}
/******************************************************************************/
// Calculate SO(3) adjoint operation: for rotation matrix C and matrix A,
// calculate C*A*C^T
mat3x3_t Adjoint(const mat3x3_t C, const mat3x3_t A)
{
   mat3x3_t CACT;
   long i, j, k, l;
   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++) {
         CACT.mat[i][j] = 0.0;
         for (l = 0; l < 3; l++)
            for (k = 0; k < 3; k++)
               CACT.mat[i][j] += C.mat[i][l] * A.mat[l][k] * C.mat[j][k];
      }
   return CACT;
}
/******************************************************************************/
// Calculate SO(3) adjoint operation for transpose rotation: for rotation matrix
// C and matrix A, calculate C^T*A*C
mat3x3_t AdjointT(const mat3x3_t C, const mat3x3_t A)
{
   mat3x3_t CTAC;
   long i, j, k, l;
   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++) {
         CTAC.mat[i][j] = 0.0;
         for (l = 0; l < 3; l++)
            for (k = 0; k < 3; k++)
               CTAC.mat[i][j] += C.mat[k][j] * A.mat[l][k] * C.mat[l][i];
      }
   return CTAC;
}
/******************************************************************************/
/* Invert 3x3 matrix A and right multiply by 3xm matrix B, returning 3xm      */
/* matrix C.                                                                  */
/*    Note that the aguments `BT` and `BT` are the transpose of the           */
/*    relevant matricies.                                                     */
void MINVxM3(mat3x3_t A, long m, vec3_t BT[m], vec3_t CT[m])
{
   long I, J, ROW;
   long IPIVOT = 0;
   mat3x3_t M;
   double PIVOT, K;

   M = A;
   for (J = 0; J < m; J++)
      CT[J] = BT[J];

   for (ROW = 0; ROW < 3; ROW++) {
      PIVOT  = M.mat[ROW][ROW];
      IPIVOT = ROW;
      for (I = ROW + 1; I < 3; I++) {
         if (fabs(M.mat[I][ROW]) >= fabs(PIVOT)) {
            PIVOT  = M.mat[I][ROW];
            IPIVOT = I;
         }
      }
      if (PIVOT == 0.0) {
         printf("Matrix is singular in MINVxM3\n");
         exit(EXIT_FAILURE);
      }

      vec3_t tv      = M.rows[IPIVOT];
      M.rows[IPIVOT] = M.rows[ROW];
      M.rows[ROW]    = SxV(1.0 / PIVOT, tv);
      for (J = 0; J < m; J++) {
         double t        = CT[J].v[IPIVOT];
         CT[J].v[IPIVOT] = CT[J].v[ROW];
         CT[J].v[ROW]    = t / PIVOT;
      }
      for (I = ROW + 1; I < 3; I++) {
         K = M.mat[I][ROW];
         for (J = 3 - 1; J >= ROW; J--)
            M.mat[I][J] -= K * M.mat[ROW][J];
         for (J = 0; J < m; J++)
            CT[J].v[I] -= K * CT[J].v[ROW];
      }
   }

   /*    M is now upper triangular */
   for (ROW = 3 - 1; ROW >= 0; ROW--) {
      for (I = 0; I < ROW; I++) {
         K = M.mat[I][ROW];
         for (J = 0; J < 3; J++)
            M.mat[I][J] -= K * M.mat[ROW][J];
         for (J = 0; J < m; J++)
            CT[J].v[I] -= K * CT[J].v[ROW];
      }
   }
}
/******************************************************************************/
// Invert NxN matrix A and right-multiply by Nxm matrix B, returning Nxm matrix
// C
void MINVxMG(double **A, double **B, double **C, long N, long m)
{
   long I, J, ROW;
   long IPIVOT = 0;
   double M[N][N];
   double PIVOT, K;

   if (N == 1) {
      for (I = 0; I < m; I++)
         C[0][I] = B[0][I] / A[0][0];
   }

   for (I = 0; I < N; I++) {
      for (J = 0; J < N; J++)
         M[I][J] = A[I][J];
      for (J = 0; J < m; J++)
         C[I][J] = B[I][J];
   }

   for (ROW = 0; ROW < N; ROW++) {
      PIVOT  = M[ROW][ROW];
      IPIVOT = ROW;
      for (I = ROW + 1; I < N; I++) {
         if (fabs(M[I][ROW]) >= fabs(PIVOT)) {
            PIVOT  = M[I][ROW];
            IPIVOT = I;
         }
      }
      if (PIVOT == 0.0) {
         printf("Matrix is singular in MINVxMG\n");
         exit(EXIT_FAILURE);
      }

      for (J = 0; J < N; J++) {
         double t     = M[IPIVOT][J];
         M[IPIVOT][J] = M[ROW][J];
         M[ROW][J]    = t / PIVOT;
      }
      for (J = 0; J < m; J++) {
         double t     = C[IPIVOT][J];
         C[IPIVOT][J] = C[ROW][J];
         C[ROW][J]    = t / PIVOT;
      }
      for (I = ROW + 1; I < N; I++) {
         K = M[I][ROW];
         for (J = N - 1; J >= ROW; J--)
            M[I][J] -= K * M[ROW][J];
         for (J = 0; J < m; J++)
            C[I][J] -= K * C[ROW][J];
      }
   }

   /*    M is now upper triangular */
   for (ROW = N - 1; ROW >= 0; ROW--) {
      for (I = 0; I < ROW; I++) {
         K = M[I][ROW];
         for (J = 0; J < N; J++)
            M[I][J] -= K * M[ROW][J];
         for (J = 0; J < m; J++)
            C[I][J] -= K * C[ROW][J];
      }
   }
}
/******************************************************************************/
// Invert mxm matrix B and left-multiply by Nxm matrix A, returning Nxm matrix C
// looks at the transpose of the problem and uses MINVxMG to solve
void MxMINVG(double **A, double **B, double **C, long N, long m)
{
   // TODO: actually do this, not the transpose of the problem. Less memory
   // overhead
   double **AT, **BT, **CT;
   long i, j;

   AT = CreateMatrix(m, N);
   BT = CreateMatrix(m, m);
   CT = CreateMatrix(m, N);
   for (i = 0; i < m; i++) {
      for (j = 0; j < N; j++)
         AT[i][j] = A[j][i];
      for (j = 0; j < m; j++)
         BT[i][j] = B[j][i];
   }
   MINVxMG(BT, AT, CT, m, N);
   for (i = 0; i < m; i++)
      for (j = 0; j < N; j++)
         C[j][i] = CT[i][j];

   DestroyMatrix(AT);
   DestroyMatrix(BT);
   DestroyMatrix(CT);
}
/******************************************************************************/
// Matrix Exponential for Special Orthogonal Group of dimension 3 (SO(3))
mat3x3_t expmso3(vec3_t theta)
{
   double tMag, sTMag, cTMagM1;
   mat3x3_t R = MAT3X3_EYE, tCross, tCrossCross;
   long i, j;

   tMag = MAGV(theta);
   if (tMag >= __DBL_EPSILON__) {
      vec3_t thetaHat;
      for (i = 0; i < 3; i++)
         thetaHat.v[i] = theta.v[i] / tMag;
      sTMag   = sin(tMag);
      cTMagM1 = cos(tMag) - 1.0;

      tCross      = V2CrossM(thetaHat);
      tCrossCross = V2DoubleCrossM(thetaHat);

      for (i = 0; i < 3; i++)
         for (j = 0; j < 3; j++)
            R.mat[i][j] +=
                sTMag * tCross.mat[i][j] - cTMagM1 * tCrossCross.mat[i][j];
   }
   return R;
}
/******************************************************************************/
// Matrix Logarithm for SO(3)
vec3_t logso3(mat3x3_t const R)
{
   double tMag, dSincTMag;

   tMag      = acos((R.mat[0][0] + R.mat[1][1] + R.mat[2][2] - 1) / 2.0);
   dSincTMag = 2.0;
   if (tMag > __DBL_EPSILON__)
      dSincTMag *= sinc(tMag);
   vec3_t theta;
   theta.v[0] = (R.mat[2][1] - R.mat[1][2]) / dSincTMag;
   theta.v[1] = (R.mat[0][2] - R.mat[2][0]) / dSincTMag;
   theta.v[2] = (R.mat[1][0] - R.mat[0][1]) / dSincTMag;
   return theta;
}
/******************************************************************************/
// Calculate matrix exponential on two-frames-group (SO(3)xR^((n+m)x3))
void expmTFG(const vec3_t theta, long const n, long const m, vec3_t x[n],
             vec3_t xbar[m], mat3x3_t *R)
{
   mat3x3_t tCross, tCrossCross, intmR, intR = MAT3X3_EYE;
   double scTMag, cTMagM1;
   long i, j;

   magvec3_t utheta;
   utheta.v = theta;

   *R = expmso3(utheta.v);

   utheta = UNITV(utheta.v);
   if (utheta.m > __DBL_EPSILON__) {
      scTMag  = sinc(utheta.m);
      cTMagM1 = cos(utheta.m) - 1.0;

      tCross      = V2CrossM(utheta.v);
      tCrossCross = V2DoubleCrossM(utheta.v);

      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++) {
            intR.mat[i][j] += (1.0 - scTMag) * tCrossCross.mat[i][j];
            intmR.mat[i][j] =
                intR.mat[i][j] + cTMagM1 * tCross.mat[i][j] / utheta.m;
            intR.mat[i][j] -= cTMagM1 * tCross.mat[i][j] / utheta.m;
         }
      }
      for (j = 0; j < n; j++)
         x[j] = MxV(intR, x[j]);

      for (j = 0; j < m; j++)
         xbar[j] = MxV(intmR, xbar[j]);
   }
}
/******************************************************************************/
// Calculate the induced matrix 1-norm of nxm matrix A
double M1NormG(double **A, long const n, long const m)
{
   double p1Norm = 0.0;
   long i, j;

   for (i = 0; i < n; i++) {
      double colSum = 0.0;
      for (j = 0; j < m; j++)
         colSum += fabs(A[i][j]);

      if (colSum > p1Norm)
         p1Norm = colSum;
   }
   return (p1Norm);
}
/******************************************************************************/
// Calculate the square of the induced matrix 2-norm of th nxm matrix A by
// calculation of the largest eigenvalue of ATA or AAT, depending on which is
// smaller dimension
double M2Norm2G(double **A, long const n, long const m)
{
   double **ATA;
   double d[n];
   const long maxIter = 100;

   long majDim = n, minDim = m;

   if (m > n) {
      majDim = m;
      minDim = n;
   }
   ATA = CreateMatrix(minDim, minDim);

   if (minDim == n)
      MxMTG(A, A, ATA, minDim, majDim, minDim);
   else
      MTxMG(A, A, ATA, minDim, majDim, minDim);

   jacobiEValue(ATA, minDim, maxIter, d);
   DestroyMatrix(ATA);

   double p2Norm2 = d[0];
   for (int i = 0; i < minDim; i++)
      if (d[i] > p2Norm2)
         p2Norm2 = d[i];
   return (p2Norm2);
}
/******************************************************************************/
// Downdates the nxn, lower triangular, Cholesky matrix S by the vector u.
// Returns 0 if failure, 1 if success
int cholDowndate(double **S, double u[], long const n)
{
   long i, k;

   for (k = 0; k < n; k++) {
      double r = S[k][k] * S[k][k] - u[k] * u[k];
      if (r < 0.0)
         return 0;
      r        = sqrt(r);
      double c = r / S[k][k];
      double s = u[k] / S[k][k];
      S[k][k]  = r;
      for (i = k + 1; i < n; i++) {
         S[i][k] = (S[i][k] - s * u[i]) / c;
         u[i]    = c * u[i] - s * S[i][k];
      }
   }
   return 1;
}
/******************************************************************************/
// Calculates the nxn, lower triangular, Cholesky matrix S from nxn, real,
// positive definite matrix A
void chol(double **A, double **S, long const n)
{
   long i, j, k;

   for (i = 0; i < n; i++) {
      for (j = 0; j <= i; j++) {
         double s = 0.0;
         for (k = 0; k < j; k++)
            s += S[i][k] * S[j][k];
         S[i][j] = (i == j) ? sqrt(A[i][i] - s) : ((A[i][j] - s) / S[j][j]);
      }
   }
}
/******************************************************************************/
double houseGen(double x[], double u[], long const n)
{
   long i;
   double nu        = 0.0;
   double const rt2 = SQRTTWO;
   for (i = 0; i < n; i++)
      nu += x[i] * x[i];
   if (nu < __DBL_EPSILON__) {
      nu   = 0.0;
      u[0] = rt2;
   }
   else {
      nu = sqrt(nu);
      for (i = 0; i < n; i++)
         u[i] = x[i] / (nu);
      if (u[0] >= 0.0) {
         u[0] += 1.0;
         nu    = -nu;
      }
      else {
         u[0] -= 1.0;
      }
      double a = sqrt(fabs(u[0]));
      for (i = 0; i < n; i++)
         u[i] /= a;
   }
   return (nu);
}
/******************************************************************************/
// QR decomposition of nxm matrix A by Householder transformations
void hqrd(double **A, double **U, double **R, long const n, long const m)
{
   double X[n][m];
   long k, i, j;

   for (i = 0; i < n; i++) {
      for (j = 0; j < m; j++) {
         X[i][j] = A[i][j];
      }
   }

   for (k = 0; k < ((n < m) ? n : m); k++) {
      long size = n - k;
      double x[size], u[size], v[m - k - 1];
      for (i = 0; i < size; i++) {
         x[i] = X[k + i][k];
         u[i] = 0.0;
      }

      R[k][k] = houseGen(x, u, size);

      for (j = k + 1; j < m; j++) {
         v[j - k - 1] = 0.0;
         for (i = 0; i < size; i++)
            v[j - k - 1] += u[i] * X[k + i][j];
      }
      for (i = 0; i < size; i++) {
         U[i + k][k] = u[i];
         for (j = k + 1; j < m; j++) {
            X[k + i][j] -= u[i] * v[j - k - 1];
         }
      }
      for (j = k + 1; j < m; j++) {
         R[k][j] = X[k][j];
      }
   }
   for (i = 0; i < n; i++) {
      for (j = 0; j < m; j++) {
         A[i][j] = X[i][j];
      }
   }
}
/******************************************************************************/
// Helper function for bhqrd
void utu(double **U, double **T, long const n, long const m)
{
   long i, j, k;
   for (k = 0; k < m; k++) {
      T[k][k] = 1.0;

      for (j = 0; j < k; j++) {
         T[j][k] = 0.0;
         for (i = 0; i < n; i++)
            T[j][k] += U[i][j] * U[i][k];
         double a = 0.0;
         for (i = 0; i < k; i++)
            a -= T[j][i] * T[i][k];
         T[j][k] = a;
      }
   }
}
/******************************************************************************/
// QR decomposition of nxm matrix A by blocked Householder transforms, using
// blocksize bSize (WIP)
// TODO: fix
void bhqrd(double **A, double **U, double **R, long const n, long const m,
           long const bSize)
{
   // long q = 0;
   long i, j, k;
   double **Xb = NULL, **Ub = NULL, **Rb = NULL, **Tq = NULL, **V = NULL,
          **tmp = NULL;
   for (k = 0; k < ((n < m) ? n : m); k += bSize) {
      // q++;
      long ell = ((m < (k + bSize)) ? m : (k + bSize));
      long a = n - k, b = ell - k;
      Xb = CreateMatrix(a, b);
      Ub = CreateMatrix(a, b);
      Rb = CreateMatrix(b, b);
      Tq = CreateMatrix(b, b);
      for (j = 0; j < b; j++) {
         for (i = 0; i < a; i++)
            Xb[i][j] = A[k + i][k + j];
      }
      hqrd(Xb, Ub, Rb, a, b);
      for (j = 0; j < b; j++) {
         for (i = 0; i < a; i++) {
            A[k + i][k + j] = Xb[i][j];
            U[k + i][k + j] = Ub[i][j];
         }
      }
      for (i = 0; i < b; i++) {
         for (j = 0; j < b; j++)
            R[k + i][k + j] = Rb[i][j];
      }
      utu(Ub, Tq, a, b);
      if (ell < m) {
         V   = CreateMatrix(b, m - ell);
         tmp = CreateMatrix(a, m - ell);
         for (i = 0; i < a; i++) {
            for (j = 0; j < (m - ell); j++)
               tmp[i][j] = A[k + i][ell + j];
         }
         MTxMG(Ub, tmp, V, b, a, m - ell);
         DestroyMatrix(tmp);
         tmp = CreateMatrix(b, m - ell);
         MTxMG(Tq, V, tmp, b, b, m - ell);
         for (i = 0; i < b; i++) {
            for (j = 0; j < m - ell; j++)
               V[i][j] = tmp[i][j];
         }
         DestroyMatrix(tmp);
         tmp = CreateMatrix(a, m - ell);
         MxMG(Ub, V, tmp, a, b, m - ell);
         for (i = k; i < n; i++) {
            for (j = ell; j < m; j++)
               A[i][j] -= tmp[i - k][j - ell];
         }
         for (i = k; i < ell; i++) {
            for (j = ell; j < m; j++)
               R[i][j] = A[i][j];
         }
         DestroyMatrix(V);
         DestroyMatrix(tmp);
      }

      DestroyMatrix(Xb);
      DestroyMatrix(Ub);
      DestroyMatrix(Rb);
      DestroyMatrix(Tq);
   }
}
/******************************************************************************/
// Algorithm for calculating integer powers of doubles
double ipow(double base, long exp)
{
   double result = 1.0;
   do {
      if (exp & 1)
         result *= base;
      exp >>= 1;
      if (!exp)
         break;
      base *= base;
   } while (1);
   return result;
}
/******************************************************************************/
// Taylor series method for Matrix Exponential, adapted from John Burkardt
// https://people.sc.fsu.edu/~jburkardt
void expm(double **A, double **e, long const n)
{
   int const maxIter = 25;
   int k             = 1;
   double **B, **C;
   B = CreateMatrix(n, n);
   C = CreateMatrix(n, n);

   for (int i = 0; i < n; i++)
      for (int j = 0; j < n; j++)
         e[i][j] = 0.0;

   for (int i = 0; i < n; i++)
      B[i][i] = 1.0;

   while (k <= maxIter && isSignificant(n, n, e, B)) {
      for (int i = 0; i < n; i++)
         for (int j = 0; j < n; j++)
            e[i][j] += B[i][j];

      MxMG(B, A, C, n, n, n);
      SxMG(1.0 / ((double)(k)), C, B, n, n);
      k++;
   }

   DestroyMatrix(B);
   DestroyMatrix(C);
}
/******************************************************************************/
// Test if matrix B is significant relative to matrix A, adapted from John
// Burkardt https://people.sc.fsu.edu/~jburkardt
long isSignificant(int const m, int const n, double **A, double **B)
{
   double t, tol;
   long isSignificant = 0;

   for (int i = 0; i < n; i++) {
      for (int j = 0; j < m; j++) {
         t   = A[i][j] + B[i][j];
         tol = nextafter(fabs(A[i][j]), INFINITY) - fabs(A[i][j]);
         if (tol < (fabs(A[i][j] - t))) {
            isSignificant = 1;
            break;
         }
      }
   }

   return (isSignificant);
}
/******************************************************************************/
// Calculates eigenvalues of real symmetric matrix by Jacobi iteration,
// adapted from John Burkardt https://people.sc.fsu.edu/~jburkardt
// eigenvalues are in descending order
void jacobiEValue(double **A, int const n, int const maxIter, double d[n])
{
   double *bw, *zw, c, g, gapq, h;
   long i, j, k, l, m, p, q;
   double s, t, tau, term, termp, termq, theta, thresh;
   long iterNum = 0;

   bw = calloc(n, sizeof(double));
   zw = calloc(n, sizeof(double));
   for (i = 0; i < n; i++) {
      d[i]  = A[i][i];
      bw[i] = d[i];
      zw[i] = 0.0;
   }

   while (iterNum < maxIter) {
      iterNum++;

      /*
      The convergence threshold is based on the size of the elements in
      the strict upper triangle of the matrix.
      */
      thresh = 0.0;
      for (j = 0; j < n; j++) {
         for (i = 0; i < j; i++)
            thresh += A[i][j] * A[i][j];
      }

      thresh = sqrt(thresh) / (double)(4.0 * n);
      if (thresh < __DBL_EPSILON__)
         break;

      for (p = 0; p < n; p++) {
         for (q = p + 1; q < n; q++) {
            gapq  = 10.0 * fabs(A[p][q]);
            termp = gapq + fabs(d[p]);
            termq = gapq + fabs(d[q]);
            /*
            Annihilate tiny offdiagonal elements.
            */
            if (4 < iterNum && termp == fabs(d[p]) && termq == fabs(d[q])) {
               A[p][q] = 0.0;
            }
            /*
            Otherwise, apply a rotation.
            */
            else if (thresh <= fabs(A[p][q])) {
               h    = d[q] - d[p];
               term = fabs(h) + gapq;

               if (term == fabs(h)) {
                  t = A[p][q] / h;
               }
               else {
                  theta = 0.5 * h / A[p][q];
                  t     = 1.0 / (fabs(theta) + sqrt(1.0 + theta * theta));
                  if (theta < 0.0)
                     t = -t;
               }
               c   = 1.0 / sqrt(1.0 + t * t);
               s   = t * c;
               tau = s / (1.0 + c);
               h   = t * A[p][q];
               /*
               Accumulate corrections to diagonal elements.
               */
               zw[p] -= h;
               zw[q] += h;
               d[p]  -= h;
               d[q]  += h;

               A[p][q] = 0.0;
               /*
               Rotate, using information from the upper triangle of A only.
               */
               for (j = 0; j < p; j++) {
                  g       = A[j][p];
                  h       = A[j][q];
                  A[j][p] = g - s * (h + g * tau);
                  A[j][q] = h + s * (g - h * tau);
               }

               for (j = p + 1; j < q; j++) {
                  g       = A[p][j];
                  h       = A[j][q];
                  A[p][j] = g - s * (h + g * tau);
                  A[j][q] = h + s * (g - h * tau);
               }

               for (j = q + 1; j < n; j++) {
                  g       = A[p][j];
                  h       = A[q][j];
                  A[p][j] = g - s * (h + g * tau);
                  A[q][j] = h + s * (g - h * tau);
               }
            }
         }
      }

      for (i = 0; i < n; i++) {
         bw[i] = bw[i] + zw[i];
         d[i]  = bw[i];
         zw[i] = 0.0;
      }
   }
   /*
   Restore upper triangle of input matrix.
   */
   for (j = 0; j < n; j++) {
      for (i = 0; i < j; i++) {
         A[i][j] = A[j][i];
      }
   }
   /*
   Descending sort the eigenvalues and eigenvectors.
   */
   for (k = 0; k < n - 1; k++) {
      m = k;
      for (l = k + 1; l < n; l++) {
         if (d[l] > d[m]) {
            m = l;
         }
      }

      if (m != k) {
         t    = d[m];
         d[m] = d[k];
         d[k] = t;
      }
   }

   free(bw);
   free(zw);
}
/******************************************************************************/
// Calculates eigenvalues & eigenvectors of real symmetric matrix by Jacobi
// iteration, adapted from John Burkardt https://people.sc.fsu.edu/~jburkardt
// eigenvalues & eigenvectors are in descending order
void jacobiEValueEVector(double **A, int const n, int const maxIter, double **V,
                         double d[n])
{
   double *bw, c, g, gapq, h;
   long i, j, k, l, m, p, q;
   double s, t, tau, term, termp, termq, theta, thresh, w;
   double *zw;
   long iterNum = 0;

   bw = calloc(n, sizeof(double));
   zw = calloc(n, sizeof(double));
   for (i = 0; i < n; i++) {
      for (j = 0; j < n; j++)
         V[i][j] = 0.0;
      V[i][i] = 1.0;
      d[i]    = A[i][i];
      bw[i]   = d[i];
      zw[i]   = 0.0;
   }

   while (iterNum < maxIter) {
      iterNum++;

      /*
      The convergence threshold is based on the size of the elements in
      the strict upper triangle of the matrix.
      */
      thresh = 0.0;
      for (j = 0; j < n; j++) {
         for (i = 0; i < j; i++)
            thresh += A[i][j] * A[i][j];
      }

      thresh = sqrt(thresh) / ((double)4 * n);
      if (thresh < __DBL_EPSILON__)
         break;

      for (p = 0; p < n; p++) {
         for (q = p + 1; q < n; q++) {
            gapq  = 10.0 * fabs(A[p][q]);
            termp = gapq + fabs(d[p]);
            termq = gapq + fabs(d[q]);
            /*
            Annihilate tiny offdiagonal elements.
            */
            if (4 < iterNum && termp == fabs(d[p]) && termq == fabs(d[q])) {
               A[p][q] = 0.0;
            }
            /*
            Otherwise, apply a rotation.
            */
            else if (thresh <= fabs(A[p][q])) {
               h    = d[q] - d[p];
               term = fabs(h) + gapq;

               if (term == fabs(h)) {
                  t = A[p][q] / h;
               }
               else {
                  theta = 0.5 * h / A[p][q];
                  t     = 1.0 / (fabs(theta) + sqrt(1.0 + theta * theta));
                  if (theta < 0.0)
                     t = -t;
               }
               c   = 1.0 / sqrt(1.0 + t * t);
               s   = t * c;
               tau = s / (1.0 + c);
               h   = t * A[p][q];
               /*
               Accumulate corrections to diagonal elements.
               */
               zw[p] = zw[p] - h;
               zw[q] = zw[q] + h;
               d[p]  = d[p] - h;
               d[q]  = d[q] + h;

               A[p][q] = 0.0;
               /*
               Rotate, using information from the upper triangle of A only.
               */
               for (j = 0; j < p; j++) {
                  g       = A[j][p];
                  h       = A[j][q];
                  A[j][p] = g - s * (h + g * tau);
                  A[j][q] = h + s * (g - h * tau);
               }

               for (j = p + 1; j < q; j++) {
                  g       = A[p][j];
                  h       = A[j][q];
                  A[p][j] = g - s * (h + g * tau);
                  A[j][j] = h + s * (g - h * tau);
               }

               for (j = q + 1; j < n; j++) {
                  g       = A[p][j];
                  h       = A[q][j];
                  A[p][j] = g - s * (h + g * tau);
                  A[q][j] = h + s * (g - h * tau);
               }
               /*
               Accumulate information in the eigenvector matrix.
               */
               for (j = 0; j < n; j++) {
                  g       = V[j][p];
                  h       = V[j][q];
                  V[j][p] = g - s * (h + g * tau);
                  V[j][q] = h + s * (g - h * tau);
               }
            }
         }
      }

      for (i = 0; i < n; i++) {
         bw[i] = bw[i] + zw[i];
         d[i]  = bw[i];
         zw[i] = 0.0;
      }
   }
   /*
   Restore upper triangle of input matrix.
   */
   for (j = 0; j < n; j++) {
      for (i = 0; i < j; i++) {
         A[i][j] = A[j][i];
      }
   }
   /*
   Descending sort the eigenvalues and eigenvectors.
   */
   for (k = 0; k < n - 1; k++) {
      m = k;
      for (l = k + 1; l < n; l++) {
         if (d[l] > d[m]) {
            m = l;
         }
      }

      if (m != k) {
         t    = d[m];
         d[m] = d[k];
         d[k] = t;
         for (i = 0; i < n; i++) {
            w       = V[i][m];
            V[i][m] = V[i][k];
            V[i][k] = w;
         }
      }
   }
   /*
   Normalize eigenvectors
   */
   for (k = 0; k < n; k++) {
      double norm = 0;
      for (l = 0; l < n; l++)
         norm += V[k][l] * V[k][l];
      norm = sqrt(norm);
      if (norm > __DBL_EPSILON__) {
         for (l = 0; l < n; l++)
            V[k][l] /= norm;
      }
   }

   free(bw);
   free(zw);
}
/******************************************************************************/
double **matPow(const long n, double **A, const unsigned long p)
// computes the positive pth power of nxn matrix A
{
   double **out = CreateMatrix(n, n);
   double **B   = CreateMatrix(n, n);
   double **C   = CreateMatrix(n, n);
   for (int i = 0; i < n; i++)
      for (int j = 0; j < n; j++)
         C[i][j] = A[i][j];

   unsigned long iterator = p;
   for (long i = 0; i < n; i++)
      out[i][i] = 1.0;

   while (iterator > 0) {
      if (iterator & 1) {
         MxMG(out, C, B, n, n, n);
         for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
               out[i][j] = B[i][j];
      }
      // swap pointers for temporaries; should be faster than assigning elements
      // of C to B
      double **t = C;
      C          = B;
      B          = t;

      MxMG(B, B, C, n, n, n);
      iterator = iterator >> 1;
   }
   DestroyMatrix(B);
   DestroyMatrix(C);
   return out;
}
/******************************************************************************/
/* compute the positive pth integer power of nxn matrix A with precomputed    */
/* positive sth power of A                                                    */
void QuickMatPow(const long n, double **A, double **As, const long s,
                 double **Ap, const long p)
{
   double **Ar = NULL;
   if (s < 2) {
      double **tmp = matPow(n, A, p);
      for (long i = 0; i < n; i++)
         for (long j = 0; j < n; j++)
            Ap[i][j] = tmp[i][j];
      return;
   }

   long q       = p / s;
   const long r = p % s;
   if (s - r < r) {
      q++;
      double **Ainv = CreateMatrix(n, n);
      MINVG(A, Ainv, n);
      Ar = matPow(n, Ainv, s - r);
      DestroyMatrix(Ainv);
   }
   else {
      Ar = matPow(n, A, r);
   }
   double **Asq = matPow(n, As, q);
   MxMG(Asq, Ar, Ap, n, n, n);
   DestroyMatrix(Ar);
   DestroyMatrix(Asq);
}

/* #ifdef __cplusplus
** }
** #endif
*/

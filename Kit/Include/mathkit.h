/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __MATHKIT_H__
#define __MATHKIT_H__

#include "42constants.h"
#include "defineskit.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

typedef union {
   struct {
      double x;
      double y;
      double z;
   };
   double v[3];
} vec3;

#define VEC3_ZERO          ((vec3){.x = 0, .y = 0, .z = 0})
#define VEC3_PXAXIS        ((vec3){.x = 1, .y = 0, .z = 0})
#define VEC3_PYAXIS        ((vec3){.x = 0, .y = 1, .z = 0})
#define VEC3_PZAXIS        ((vec3){.x = 0, .y = 0, .z = 1})
#define VEC3_NXAXIS        ((vec3){.x = -1, .y = 0, .z = 0})
#define VEC3_NYAXIS        ((vec3){.x = 0, .y = -1, .z = 0})
#define VEC3_NZAXIS        ((vec3){.x = 0, .y = 0, .z = -1})
#define VEC3_INIT(a, b, c) ((vec3){.x = (a), .y = (b), .z = (c)})
#define DBL_TO_VEC3(dbl)   ((vec3){.x = (dbl)[0], .y = (dbl)[1], .z = (dbl)[2]})
#define VEC3_TO_DBL(dbl, vec)                                                  \
   do {                                                                        \
      (dbl)[0] = vec.x;                                                        \
      (dbl)[1] = vec.y;                                                        \
      (dbl)[2] = vec.z;                                                        \
   } while (0)

// Row-Major 3x3 Matrix
typedef union {
   vec3 rows[3];
   double flat[9];
   double mat[3][3];
} mat3x3;
#define MAT3X3_ZERO                                                            \
   ((mat3x3){.rows = {(vec3){.x = 0, .y = 0, .z = 0},                          \
                      (vec3){.x = 0, .y = 0, .z = 0},                          \
                      (vec3){.x = 0, .y = 0, .z = 0}}})
#define MAT3X3_EYE ((mat3x3){.rows = {VEC3_PXAXIS, VEC3_PYAXIS, VEC3_PZAXIS}})

// Vector first Quaternion
typedef union {
   struct {
      double x;
      double y;
      double z;
      double s;
   };
   struct {
      vec3 qv;
      double qs;
   };
   double q[4];
} quat;
typedef quat vec4;

#define QUAT_ZERO ((quat){.qv = {.x = 0, .y = 0, .z = 0}, .qs = 0})
#define QUAT_EYE  ((quat){.qv = {.x = 0, .y = 0, .z = 0}, .qs = 1})
#define DBL_TO_QUAT(dbl)                                                       \
   ((quat){.qv = {.x = (dbl)[0], .y = (dbl)[1], .z = (dbl)[2]}, .qs = (dbl)[3]})
#define QUAT_TO_DBL(dbl, qua)                                                  \
   do {                                                                        \
      (dbl)[0] = qua.x;                                                        \
      (dbl)[1] = qua.y;                                                        \
      (dbl)[2] = qua.z;                                                        \
      (dbl)[3] = qua.s;                                                        \
   } while (0)

__attribute__((pure)) int any_int(const long n, const int *const vec);
__attribute__((pure)) int all_int(const long n, const int *const vec);
__attribute__((pure)) int any_isnan(const long n, const double *const v);
__attribute__((const)) double signum(const double x);
__attribute__((const)) double sin_deg(double x);
__attribute__((const)) double cos_deg(double x);
__attribute__((const)) double sinc(const double x);
__attribute__((const)) double smootherstep(const double x);
__attribute__((const)) double Limit(double x, double min, double max);
__attribute__((const)) mat3x3 MxM(const mat3x3 A, const mat3x3 B);
__attribute__((const)) mat3x3 MxMT(const mat3x3 A, const mat3x3 B);
__attribute__((const)) mat3x3 MTxM(const mat3x3 A, const mat3x3 B);
__attribute__((const)) mat3x3 MTxMT(const mat3x3 A, const mat3x3 B);
__attribute__((const)) vec3 VxM(const vec3 V, const mat3x3 M);
__attribute__((const)) vec3 MxV(const mat3x3 M, const vec3 V);
__attribute__((const)) vec3 VxMT(const vec3 V, const mat3x3 M);
__attribute__((const)) vec3 MTxV(const mat3x3 M, const vec3 V);
__attribute__((const)) vec3 SxV(const double S, const vec3 V);

__attribute__((const)) vec3 VNegElem(const vec3 A);
__attribute__((const)) vec3 VpVElem(const vec3 A, const vec3 B);
__attribute__((const)) vec3 VmVElem(const vec3 A, const vec3 B);
__attribute__((const)) vec3 VxVElem(const vec3 A, const vec3 B);
__attribute__((const)) vec3 VdVElem(const vec3 A, const vec3 B);
__attribute__((const)) vec3 LimitElem_bidir(vec3 x, const vec3 lim);

__attribute__((const)) mat3x3 SxM(const double S, const mat3x3 A);
__attribute__((const)) double det3x3(const mat3x3 M);
void MINV4(const double A[4][4], double B[4][4]);
__attribute__((const)) mat3x3 MINV3(const mat3x3 A);
void MINV2(const double A[2][2], double B[2][2]);
void PINV4x3(const double A[4][3], double Aplus[3][4]);
mat3x3 MT(const mat3x3 A);
__attribute__((const)) double VoV(const vec3 A, const vec3 B);
__attribute__((const)) vec3 VxV(const vec3 A, const vec3 B);
__attribute__((const)) vec3 vxMov(const vec3 w, const mat3x3 M);
__attribute__((const)) double MAGV(const vec3 V);
double UNITV(vec3 *V);
double CopyUnitV(const vec3 V, vec3 *W);
__attribute__((const)) mat3x3 V2CrossM(const vec3 V);
__attribute__((const)) mat3x3 V2DoubleCrossM(const vec3 V);
__attribute__((const)) mat3x3 VcrossM(const vec3 V, const mat3x3 M);
__attribute__((const)) mat3x3 VcrossMT(const vec3 V, const mat3x3 M);
__attribute__((const)) quat QxQ(const quat A, const quat B);
__attribute__((const)) quat QTxQ(const quat A, const quat B);
__attribute__((const)) quat QxQT(const quat A, const quat B);
__attribute__((const)) vec3 VxQ(const vec3 Va, const quat QAB);
__attribute__((const)) vec3 QxV(const quat QAB, const vec3 Vb);
__attribute__((const)) vec3 QTxV(const quat QAB, const vec3 Va);
__attribute__((const)) quat UNITQ(quat Q);
__attribute__((const)) quat RECTIFYQ(quat Q);
vec3 PerpBasis(const vec3 A, vec3 *B);
__attribute__((const)) double fact(long const n);
__attribute__((const)) double oddfact(long const n);
__attribute__((const)) double factDfact(long const n, long const m);
void Legendre(const long N, const long M, const double x,
              double P[N + 1][M + 1], double sdP[N + 1][M + 1]);
vec3 SphericalHarmonics(const long N, const long M, const double r,
                        const double trigs[4], const double Re, const double K,
                        double **C, double **S, double **Norm);
void MxMG(double **A, double **B, double **C, const long N, const long K,
          const long M);
void MxMTG(double **A, double **B, double **C, const long N, const long K,
           const long M);
void MTxMG(double **A, double **B, double **C, const long N, const long K,
           const long M);
void CopyVG(double *const dest, const double *const src, const long n);
void SxVG(const double S, const double *V, double *W, const long n);
void axpy(const double a, const double *const x, double *const y, const long n);
void MxVG(double **M, double *v, double *w, const long n, const long m);
void SxMG(double s, double **A, double **B, const long N, const long M);
void MINVG(double **A, double **AI, const long N);
void FastMINV6(const double A[6][6], double AI[6][6], const long N);
void PINVG(double **A, double **Ai, const long n, const long m);
__attribute__((malloc)) double **CreateMatrix(const long n, const long m);
void DestroyMatrix(double **A);
void LINSOLVE(double **A, double *x, double *b, const long n);
void CholeskySolve(double **A, double *x, double *b, const long n);
void ConjGradSolve(double **A, double *x, double *b, const long n,
                   const double errtol, const long maxiter);
void Bairstow(long n, double *a, const double Tol, double *Real, double *Imag);
double Amoeba(const long N, double *P,
              double CostFunction(double *p, double *Parm), double *CostParm,
              const double scale, const double Tol);
__attribute__((const)) vec3 FindNormal(const vec3 V1, const vec3 V2,
                                       const vec3 V3);
__attribute__((pure)) double LinInterp(const double *X, const double *Y,
                                       const double x, const long n);
__attribute__((const)) quat SphereInterp(quat q1, quat q2, const double u);
__attribute__((const)) double CubicInterp1D(double f0, double f1, double x);
__attribute__((const)) double CubicInterp2D(double f00, double f10, double f01,
                                            double f11, double x, double y);
__attribute__((const)) double CubicInterp3D(double f000, double f100,
                                            double f010, double f110,
                                            double f001, double f101,
                                            double f011, double f111, double x,
                                            double y, double z);
double DistanceToLine(vec3 LineEnd1, vec3 LineEnd2, vec3 Point,
                      vec3 *VecToLine);
long ProjectPointOntoPoly(vec3 Point, vec3 DirVec, vec3 *Vtx, long Nvtx,
                          vec3 *ProjPoint, double *Distance);
long ProjectPointOntoTriangle(vec3 A, vec3 B, vec3 C, vec3 DirVec, vec3 Pt,
                              vec3 *ProjPt, vec4 *Bary);
__attribute__((pure)) double CubicSpline(double x, double X[4], double Y[4]);
void ChebyPolys(double u, long n, double T[20], double U[20]);
void ChebyInterp(double T[20], double U[20], double Coef[20], long n, double *P,
                 double *dPdu);
void FindChebyCoefs(double *u, double *P, long Nu, long Nc, double Coef[20]);
void VecToLngLat(vec3 A, double *lng, double *lat);
__attribute__((const)) double WrapTo2Pi(double OrbVar);
__attribute__((pure)) double BrentsMethod(double a, double b, const double tol,
                                          double (*f)(const double, double *),
                                          double *params);
__attribute__((pure)) double
NewtonRaphson(double x0, double tol, long nMax, double maxStep,
              long breakOnZero, double (*fdf)(const double, double *),
              double *params);
void getTrigSphericalCoords(const vec3 pbe, double *cth, double *sth,
                            double *cph, double *sph, double *r);
__attribute__((const)) mat3x3 Adjoint(const mat3x3 C, const mat3x3 A);
__attribute__((const)) mat3x3 AdjointT(const mat3x3 C, const mat3x3 A);
void MINVxM3(mat3x3 A, long m, double B[3][m], double C[3][m]);
void MINVxMG(double **A, double **B, double **C, long N, long m);
void MxMINVG(double **A, double **B, double **C, long N, long m);
__attribute__((const)) mat3x3 expmso3(vec3 const theta);
__attribute__((const)) vec3 logso3(mat3x3 const R);
void expmTFG(vec3 *theta, long const n, long const m, vec3 x[n], vec3 xbar[m],
             mat3x3 *R);
__attribute__((pure)) double M1NormG(double **A, long const n, long const m);
__attribute__((pure)) double M2Norm2G(double **A, long const n, long const m);
int cholDowndate(double **S, double u[], long const n);
void chol(double **A, double **S, long const n);
void hqrd(double **A, double **U, double **R, long const n, long const m);
void bhqrd(double **A, double **U, double **R, long const n, long const m,
           long const bSize);

__attribute__((const)) double ipow(double base, long exp);
void expm(double **A, double **e, long const n);
__attribute__((pure)) long isSignificant(int const m, int const n, double **A,
                                         double **B);
void jacobiEValue(double **A, int const n, int const maxIter, double d[n]);
void jacobiEValueEVector(double **A, int const n, int const maxIter, double **V,
                         double d[n]);
__attribute__((malloc)) double **matPow(const long n, double **A,
                                        const unsigned long p);
void QuickMatPow(const long n, double **A, double **As, const long s,
                 double **Ap, const long p);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __MATHKIT_H__ */

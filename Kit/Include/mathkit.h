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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

#ifndef __MINGW32__
#ifdef WIN32
#ifndef isnan(x)
#define isnan(x) ((x) != (x))
#endif
#endif
#endif

double signum(double x);
double sinc(double x);
void MxM(const double A[3][3], const double B[3][3], double C[3][3]);
void MxMT(const double A[3][3], const double B[3][3], double C[3][3]);
void MTxM(const double A[3][3], const double B[3][3], double C[3][3]);
void MTxMT(const double A[3][3], const double B[3][3], double C[3][3]);
void VxM(const double V[3], const double M[3][3], double W[3]);
void MxV(const double M[3][3], const double V[3], double W[3]);
void VxMT(const double V[3], const double M[3][3], double W[3]);
void MTxV(const double M[3][3], const double V[3], double W[3]);
void SxV(const double S, const double V[3], double W[3]);
void SxM(const double S, const double A[3][3], double B[3][3]);
void MINV4(const double A[4][4], double B[4][4]);
void MINV3(const double A[3][3], double B[3][3]);
void MINV2(const double A[2][2], double B[2][2]);
void PINV4x3(const double A[4][3], double Aplus[3][4]);
void MT(const double A[3][3], double B[3][3]);
double VoV(const double A[3], const double B[3]);
void VxV(const double A[3], const double B[3], double C[3]);
void vxMov(const double w[3], const double M[3][3], double wxMow[3]);
double MAGV(const double V[3]);
double UNITV(double V[3]);
double CopyUnitV(const double V[3], double W[3]);
void V2CrossM(const double V[3], double M[3][3]);
void V2DoubleCrossM(const double V[3], double M[3][3]);
void VcrossM(const double V[3], const double M[3][3], double A[3][3]);
void VcrossMT(const double V[3], const double M[3][3], double A[3][3]);
void QxQ(const double A[4], const double B[4], double C[4]);
void QTxQ(const double A[4], const double B[4], double C[4]);
void QxQT(const double A[4], const double B[4], double C[4]);
void VxQ(const double Va[3], const double QAB[4], double Vb[3]);
void QxV(const double QAB[4], const double Vb[3], double Va[3]);
void QTxV(const double QAB[4], const double Va[3], double Vb[3]);
void UNITQ(double Q[4]);
void RECTIFYQ(double Q[4]);
void PerpBasis(const double A[3], double B[3], double C[3]);
double fact(const long n);
double oddfact(const long n);
void Legendre(const long N, const long M, const double x, double P[19][19],
              double sdP[19][19]);
void SphericalHarmonics(const long N, const long M, double r,
                        const double pbe[3], const double Re, const double K,
                        const double C[19][19], const double S[19][19],
                        double gradV[3]);
void MxMG(double **A, double **B, double **C, const long N, const long K,
          const long M);
void MxMTG(double **A, double **B, double **C, const long N, const long K,
           const long M);
void MTxMG(double **A, double **B, double **C, const long N, const long K,
           const long M);
void MxVG(double **M, double *v, double *w, const long n, const long m);
void SxMG(double s, double **A, double **B, const long N, const long M);
void MINVG(double **A, double **AI, const long N);
void FastMINV6(const double A[6][6], double AI[6][6], const long N);
void PINVG(double **A, double **Ai, const long n, const long m);
double **CreateMatrix(const long n, const long m);
void DestroyMatrix(double **A);
void LINSOLVE(double **A, double *x, double *b, const long n);
void CholeskySolve(double **A, double *x, double *b, const long n);
void ConjGradSolve(double **A, double *x, double *b, const long n,
                   const double errtol, const long maxiter);
void Bairstow(long n, double *a, const double Tol, double *Real, double *Imag);
double Amoeba(const long N, double *P,
              double CostFunction(double *p, double *Parm), double *CostParm,
              const double scale, const double Tol);
void FindNormal(const double V1[3], const double V2[3], const double V3[3],
                double N[3]);
double LinInterp(double *X, double *Y, const double x, const long n);
void SphereInterp(double q1[4], double q2[4], const double u, double q[4]);
double CubicInterp1D(double f0, double f1, double x);
double CubicInterp2D(double f00, double f10, double f01, double f11, double x,
                     double y);
double CubicInterp3D(double f000, double f100, double f010, double f110,
                     double f001, double f101, double f011, double f111,
                     double x, double y, double z);
double DistanceToLine(double LineEnd1[3], double LineEnd2[3], double Point[3],
                      double VecToLine[3]);
long ProjectPointOntoPoly(double Point[3], double DirVec[3], double **Vtx,
                          long Nvtx, double ProjPoint[3], double *Distance);
long ProjectPointOntoTriangle(double A[3], double B[3], double C[3],
                              double DirVec[3], double Pt[3], double ProjPt[3],
                              double Bary[4]);
double CubicSpline(double x, double X[4], double Y[4]);
void ChebyPolys(double u, long n, double T[20], double U[20]);
void ChebyInterp(double T[20], double U[20], double Coef[20], long n, double *P,
                 double *dPdu);
void FindChebyCoefs(double *u, double *P, long Nu, long Nc, double Coef[20]);
void VecToLngLat(double A[3], double *lng, double *lat);
double WrapTo2Pi(double OrbVar);
double NewtonRaphson(double x0, double tol, long nMax, double maxStep,
                     void (*fdf)(const double, double *, double *, double *),
                     double *params);

void Adjoint(const double C[3][3], const double A[3][3], double CACT[3][3]);
void AdjointT(const double C[3][3], const double A[3][3], double CACT[3][3]);
void MINVxM3(double A[3][3], long m, double B[3][m], double C[3][m]);
void MINVxMG(double **A, double **B, double **C, long N, long m);
void MxMINVG(double **A, double **B, double **C, long N, long m);
void expmso3(double theta[3], double R[3][3]);
void logso3(double const R[3][3], double theta[3]);
void expmTFG(double theta[3], long const n, long const m, double x[n][3],
             double xbar[m][3], double R[3][3]);
void expm(double **A, double **e, long const n);
long isSignificant(int const m, int const n, double **A, double **B);
double M1NormG(double **A, long const n, long const m);
double M2Norm2G(double **A, long const n, long const m);
void jacobiEValue(double **A, int const n, int const maxIter, double d[n]);
void jacobiEValueEVector(double **A, int const n, int const maxIter, double **V,
                         double d[n]);
int cholDowndate(double **S, double u[], long const n);
void chol(double **A, double **S, long const n);
void hqrd(double **A, double **U, double **R, long const n, long const m);
void bhqrd(double **A, double **U, double **R, long const n, long const m,
           long const bSize);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __MATHKIT_H__ */

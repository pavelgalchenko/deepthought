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
#include "dcmkit.h"
#include "defineskit.h"
#include "mathkit.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef __FSWKIT_H__
#define __FSWKIT_H__

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

struct KFMeasType {
   long Ny; /* 1, 2, or 3 */

   double *y;   /* Ny by 1 */
   double **Rv; /* Ny by Ny */
   double **H;  /* Ny by Nx */
   double **L;  /* Nx by Ny */

   /* Workspace variables */
   double **HP;        /* Ny by Nx */
   double *Hx;         /* Ny by 1 */
   double **HPHtRv;    /* Ny by Ny */
   double **HPHtRvInv; /* Ny by Ny */
};

struct KalmanFilterType {
   long Nx;
   long Nu;
   long Nw;
   long Nm;

   double *x;     /* Nx by 1 */
   double *u;     /* Nu by 1 */
   double **Phi;  /* Nx by Nx */
   double **Gam;  /* Nx by Nu */
   double **Gamw; /* Nx by Nw */

   double **P; /* Covariance after measurement (P is for Plus), Nx by Nx */
   double **M; /* Covariance before measurement (M is for Minus), Nx by Nx */

   double **Rw; /* Nw by Nw */

   /* Measurement structures, Nm by 1 */
   struct KFMeasType *Meas;

   /* Workspace variables */
   double *PhiX;   /* Nx by 1 */
   double *GamU;   /* Nx by 1 */
   double **PhiP;  /* Nx by Nx */
   double **GRwGt; /* Gamw*Rw*GamwT, Nx by Nx */
};

void FindPDGains(double I, double w, double z, double *Kr, double *Kp);
double SpinGainCostFunction(double p[2], double CostParm[2]);
void FindSpinnerGains(double J, double It, double Tc, double OrbPer,
                      double alpha, double *SpinRate, double *Knute,
                      double *Kprec);
__attribute__((const)) mat3x3 TRIAD(vec3 Va, vec3 Wa, vec3 Vb, vec3 Wb);
__attribute__((pure)) quat Quest(long n, double *Weight, vec3 *Ref, vec3 *Meas);
quat FilterQuest(long n, double *Weight, vec3 *Ref, vec3 *Meas, double dt,
                 double memory, vec3 wbn);
__attribute__((const)) vec3 PointGimbalToTarget(long Seq, mat3x3 CGiBi,
                                                mat3x3 CBoGo, vec3 tvi,
                                                vec3 bvo);
__attribute__((const)) vec3 CollisionAvoidanceLaw(vec3 x, vec3 v, vec3 xg,
                                                  vec3 xa, double Ra,
                                                  double vmax, double amax,
                                                  double wc, double zc);
__attribute__((const)) double BangBangSettle(double x, double v, double w0,
                                             double amax, double vmax);
__attribute__((const)) double RampCoastGlide(double x, double v, double w0,
                                             double amax, double vmax);
__attribute__((const)) double RateControl(double v, double amax, double w0);
__attribute__((const)) vec3 VectorRampCoastGlide(vec3 Xvec, vec3 Vvec,
                                                 double w0, double amax,
                                                 double vmax);
__attribute__((const)) double SolarBeta(vec3 svn, vec3 psn, vec3 vsn);
double ThrusterSelection(double **A, double *f, double *t, double tmax, long m,
                         long n, long OffPulse);
void StateEstimator(double **PHI, double **GAMMA, double **H, double **L,
                    double *u, double *y, double *x, long Nx, long Nu, long Ny);
void UDUFactor(double **P, double **U, long N);
void UDMeasUpdate(double *x, double **U, double y, double *H, double Rv,
                  long Ns);
void UDTimeUpdate(double *x, double **U, double **phi, double **gam, double *y,
                  double *Rw, long Ns, long Nw);
void AllocKalmanFilterMeasurement(struct KFMeasType *M, long Nx, long Ny);
struct KalmanFilterType *CreateKalmanFilter(long Nx, long Nu, long Nw, long Nm);
void PopulateKalmanFilterWorkspace(struct KalmanFilterType *KF);
void KalmanFilterMeasUpdate(struct KalmanFilterType *KF, struct KFMeasType *M);
void KalmanFilterTimeUpdate(struct KalmanFilterType *KF);
double CMGLaw4x1DOF(vec3 Tcmd, vec3 Axis[4], vec3 Gim[4], quat h,
                    quat *AngRateCmd);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __FSWKIT_H__ */

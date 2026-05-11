/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __RKKIT_H__
#define __RKKIT_H__

#include "jdkit.h"
#include "mathkit.h"
#include <string.h>

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

/*  Methods for integration of odes using explicit Runge Kutta forms */

// doing this in an attempt to make a potential later change easier
// TODO: different t type? JDType, or just Rational?
typedef JDType RKIndType; // independent variable data type

// used to initialized the RungeKutta Struct
typedef enum RKType {
   EULER_RK = 0, // Euler Integration
   THE_RK4_RK,   // classic RK44
   RK4_RK,       // 3/8-rule RK4
   RK89_RK,      // RK8(9)
} RKType;

typedef struct RKParams {
   long dim;
} RKParams;

// TODO: starting from somewhere similar to GMAT's implementation
typedef struct RungeKutta {
   // TODO: add a placeholder void * params object?
   void (*ode)(RKIndType t, double *x, RKParams *const params, double *xdot);
   double (*errorCalc)(const double *const errEst,
                       const double *const candState, const double *cur_state,
                       const double relErrThreshold, const long dim);

   RKIndType t;
   double *inState;
   double *outState;
   double *stateDot;
   double *stageState;
   double *candidateState;
   double *errorEsts;
   RKParams params;

   int stages;
   int order;
   int dim;

   double **ki;
   double *ci;
   double **aij;
   double *bj;

   double *ee;

   double tol;
   double relErrThreshold;
   double decPower;
   double incPower;

   RKIndType curTime;
   RKIndType stepSize;
   RKIndType smallestTime;

   RKIndType minStep;
   RKIndType maxStep;
   int stepAttempts;
   int maxStepAttempts;
   RKIndType StepTaken;

   int hasErrorControl;
   int isInitialized;
   double sigma;
} RungeKutta;

RungeKutta GetRungeKutta(
    RKType type, const double tol, const double relErrThresh,
    const int dimension, const double minStep, const double maxStep,
    void (*const ode)(RKIndType t, double *x, RKParams *const params,
                      double *xdot),
    double (*const errorCalc)(const double *const errEst,
                              const double *const candState,
                              const double *cur_state,
                              const double relErrThreshold, const long dim));
void RungeKuttaStep(RungeKutta *const rk, RKIndType t0, double dt_seconds,
                    double *x);
double RKErrorCalc(const double *const errEst, const double *const candState,
                   const double *cur_state, const double relErrThreshold,
                   const long dim);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __RKKIT_H__ */

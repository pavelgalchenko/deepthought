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

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

// doing this in an attempt to make a potential later change easier
// TODO: different t type? JDType, or just Rational?
typedef double RKIndType; // independent variable data type

// used to initialized the RungeKutta Struct
typedef enum RKType {
   THE_RK4_RK, // classic RK44
   RK4_RK,     // 3/8-rule RK4
   RK89_RK,    // RK8(9)
} RKType;
// RK8(9) coeffs from "Explicit Runge-Kutta Methods with Estimates of the Local
// Truncation Error", SIAM Journal on Numerical Analysis, vol 15, no 4, 1978

typedef enum TolType {
   REL_TOL,
   ABS_TOL,
} TolType;

// TODO: starting from somewhere similar to GMAT's implementation
typedef struct RungeKutta {
   // TODO: add a placeholder void * params object?
   void (*ode)(RKIndType t, double *x, double *xdot);

   RKIndType t;
   double *x;

   int stages;
   int order;
   int dim;

   double **ki;
   double *ai;
   double **bij;
   double *cj;

   double *ee;

   double tol;
   TolType tolType;

   RKIndType minStep;
   RKIndType maxStep;
   int stepAttempts;
   int maxStepAttempts;
   RKIndType StepTaken;

   int hasErrorControl;
   int isInitialized;
} RungeKutta;

double estimateError(RungeKutta rk);
/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __RKKIT_H__ */

/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "rkkit.h"

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

static void _allocrk(RungeKutta *const rk)
{
   if (rk->stages <= 0) {
      fprintf(stderr,
              "RungeKutta::stages is not set in _allocrk. Exiting...\n");
      exit(EXIT_FAILURE);
   }

   if (rk->isInitialized) {
      DestroyMatrix(rk->ki);
      DestroyMatrix(rk->aij);
      free(rk->ci);
      free(rk->bj);
      free(rk->ee);
      free(rk->inState);
      free(rk->stateDot);
      free(rk->stageState);
      free(rk->candidateState);
      free(rk->errorEsts);
   }

   rk->ci             = calloc(rk->stages, sizeof(double));
   rk->bj             = calloc(rk->stages, sizeof(double));
   rk->ee             = calloc(rk->stages, sizeof(double));
   rk->stateDot       = calloc(rk->dim, sizeof(double));
   rk->stageState     = calloc(rk->dim, sizeof(double));
   rk->candidateState = calloc(rk->dim, sizeof(double));
   rk->errorEsts      = calloc(rk->dim, sizeof(double));
   if (rk->ci == NULL || rk->bj == NULL || rk->ee == NULL ||
       rk->stateDot == NULL || rk->stageState == NULL ||
       rk->candidateState == NULL || rk->errorEsts == NULL) {
      fprintf(stderr, "calloc failed in _allocrk. Exiting...\n");
      exit(EXIT_FAILURE);
   }

   rk->aij = CreateMatrix(rk->stages, rk->stages);
   rk->ki  = CreateMatrix(rk->stages, rk->dim);
}

static void _initcommonrk(RungeKutta *const rk)
{
   rk->hasErrorControl = 1;
   rk->sigma           = 0.9;
   rk->tol             = 1.0e-11;
   rk->relErrThreshold = 0.1;
   rk->errorCalc       = RKErrorCalc;
   rk->maxStepAttempts = 50;
   rk->stepAttempts    = 0;
   rk->smallestTime =
       InitJD(JD_ZERO.system, JD_ZERO.epoch, 0,
              (Rational){.whole = 0, .num = 1, .den = 1000000000000});
   rk->incPower = 1.0 / rk->order;
   rk->decPower = 1.0 / (rk->order - 1);
   _allocrk(rk);
}

static void _initeuler(RungeKutta *const rk, const int dimension)
{
   // Euler integration
   rk->stages = 1;
   rk->order  = 1;
   rk->dim    = dimension;
   _initcommonrk(rk);
   rk->hasErrorControl = 0;

   rk->ci[0]     = 0.0;
   rk->aij[0][0] = 0;
   rk->bj[0]     = 1.0;
   rk->ee[0]     = 0.0;
}

static void _inittherk4(RungeKutta *const rk, const int dimension)
{
   // the classic RK4
   rk->stages = 4;
   rk->order  = 4;
   rk->dim    = dimension;
   _initcommonrk(rk);
   rk->hasErrorControl = 0;

   rk->ci[0] = 0.0;
   rk->ci[1] = 0.5;
   rk->ci[2] = 0.5;
   rk->ci[3] = 1.0;

   rk->aij[0][0] = 0.0;

   rk->aij[1][0] = 0.5;

   rk->aij[2][0] = 0.0;
   rk->aij[2][1] = 0.5;

   rk->aij[3][0] = 0.0;
   rk->aij[3][1] = 0.0;
   rk->aij[3][2] = 1.0;

   rk->bj[0] = 1.0 / 6.0;
   rk->bj[1] = 1.0 / 3.0;
   rk->bj[2] = 1.0 / 3.0;
   rk->bj[3] = 1.0 / 6.0;

   for (int i = 0; i < rk->stages; i++)
      rk->ee[i] = 0.0;
}

static void _init38rk4(RungeKutta *const rk, const int dimension)
{
   // 3/8ths rule RK4
   rk->stages = 4;
   rk->order  = 4;
   rk->dim    = dimension;
   _initcommonrk(rk);
   rk->hasErrorControl = 0;

   rk->ci[0] = 0.0;
   rk->ci[1] = 1.0 / 3.0;
   rk->ci[2] = 2.0 / 3.0;
   rk->ci[3] = 1.0;

   rk->aij[0][0] = 0.0;

   rk->aij[1][0] = 1.0 / 3.0;

   rk->aij[2][0] = -1.0 / 3.0;
   rk->aij[2][1] = 1.0;

   rk->aij[3][0] = 1.0;
   rk->aij[3][1] = -1.0;
   rk->aij[3][2] = 1.0;

   rk->bj[0] = 1.0 / 8.0;
   rk->bj[1] = 3.0 / 8.0;
   rk->bj[2] = 3.0 / 8.0;
   rk->bj[3] = 1.0 / 8.0;

   for (int i = 0; i < rk->stages; i++)
      rk->ee[i] = 0.0;
}

static void _initrk89(RungeKutta *const rk, const int dimension)
{
   // RK8(9)
   rk->stages = 16;
   rk->order  = 9;
   rk->dim    = dimension;
   _initcommonrk(rk);

   // RK8(9) coeffs taken directly from GMAT, but overall from "Explicit
   // Runge-Kutta Methods with Estimates of the Local Truncation Error", SIAM
   // Journal on Numerical Analysis, vol 15, no 4, 1978

   const double rt6 = sqrt(6.0);

   rk->ci[0]  = 0.0;
   rk->ci[1]  = 1.0 / 12.0;
   rk->ci[2]  = 1.0 / 9.0;
   rk->ci[3]  = 1.0 / 6.0;
   rk->ci[4]  = (2.0 + 2.0 * rt6) / 15.0;
   rk->ci[5]  = (6.0 + rt6) / 15.0;
   rk->ci[6]  = (6.0 - rt6) / 15.0;
   rk->ci[7]  = 2.0 / 3.0;
   rk->ci[8]  = 1.0 / 2.0;
   rk->ci[9]  = 1.0 / 3.0;
   rk->ci[10] = 1.0 / 4.0;
   rk->ci[11] = 4.0 / 3.0;
   rk->ci[12] = 5.0 / 6.0;
   rk->ci[13] = 1.0;
   rk->ci[14] = 1.0 / 6.0;
   rk->ci[15] = 1.0;

   rk->aij[0][0] = 0.0;

   rk->aij[1][0] = 1.0 / 12.0;
   rk->aij[1][1] = 0.0;

   rk->aij[2][0] = 1.0 / 27.0;
   rk->aij[2][1] = 2.0 / 27.0;
   rk->aij[2][2] = 0.0;

   rk->aij[3][0] = 1.0 / 24.0;
   rk->aij[3][1] = 0.0;
   rk->aij[3][2] = 1.0 / 8.0;
   rk->aij[3][3] = 0.0;

   rk->aij[4][0] = (4.0 + 94.0 * rt6) / 375.0;
   rk->aij[4][1] = 0.0;
   rk->aij[4][2] = (-94.0 - 84.0 * rt6) / 125.0;
   rk->aij[4][3] = (328.0 + 208.0 * rt6) / 375.0;
   rk->aij[4][4] = 0.0;

   rk->aij[5][0] = (9.0 - rt6) / 150.0;
   rk->aij[5][1] = 0.0;
   rk->aij[5][2] = 0.0;
   rk->aij[5][3] = (312.0 + 32.0 * rt6) / 1425.0;
   rk->aij[5][4] = (69.0 + 29.0 * rt6) / 570.0;
   rk->aij[5][5] = 0.0;

   rk->aij[6][0] = (927.0 - 347.0 * rt6) / 1250.0;
   rk->aij[6][1] = 0.0;
   rk->aij[6][2] = 0.0;
   rk->aij[6][3] = (-16248.0 + 7328.0 * rt6) / 9375.0;
   rk->aij[6][4] = (-489.0 + 179.0 * rt6) / 3750.0;
   rk->aij[6][5] = (14268.0 - 5798.0 * rt6) / 9375.0;
   rk->aij[6][6] = 0.0;

   rk->aij[7][0] = 2.0 / 27.0;
   rk->aij[7][1] = 0.0;
   rk->aij[7][2] = 0.0;
   rk->aij[7][3] = 0.0;
   rk->aij[7][4] = 0.0;
   rk->aij[7][5] = (16.0 - rt6) / 54.0;
   rk->aij[7][6] = (16.0 + rt6) / 54.0;
   rk->aij[7][7] = 0.0;

   rk->aij[8][0] = 19.0 / 256.0;
   rk->aij[8][1] = 0.0;
   rk->aij[8][2] = 0.0;
   rk->aij[8][3] = 0.0;
   rk->aij[8][4] = 0.0;
   rk->aij[8][5] = (118.0 - 23.0 * rt6) / 512.0;
   rk->aij[8][6] = (118.0 + 23.0 * rt6) / 512.0;
   rk->aij[8][7] = -9.0 / 256.0;
   rk->aij[8][8] = 0.0;

   rk->aij[9][0] = 11.0 / 144.0;
   rk->aij[9][1] = 0.0;
   rk->aij[9][2] = 0.0;
   rk->aij[9][3] = 0.0;
   rk->aij[9][4] = 0.0;
   rk->aij[9][5] = (266.0 - rt6) / 864.0;
   rk->aij[9][6] = (266.0 + rt6) / 864.0;
   rk->aij[9][7] = -1.0 / 16.0;
   rk->aij[9][8] = -8.0 / 27.0;
   rk->aij[9][9] = 0.0;

   rk->aij[10][0]  = (5034.0 - 271.0 * rt6) / 61440.0;
   rk->aij[10][1]  = 0.0;
   rk->aij[10][2]  = 0.0;
   rk->aij[10][3]  = 0.0;
   rk->aij[10][4]  = 0.0;
   rk->aij[10][5]  = 0.0;
   rk->aij[10][6]  = (7859.0 - 1626.0 * rt6) / 10240.0;
   rk->aij[10][7]  = (-2232.0 + 813.0 * rt6) / 20480.0;
   rk->aij[10][8]  = (-594.0 + 271.0 * rt6) / 960.0;
   rk->aij[10][9]  = (657.0 - 813.0 * rt6) / 5120.0;
   rk->aij[10][10] = 0.0;

   rk->aij[11][0]  = (5996.0 - 3794.0 * rt6) / 405.0;
   rk->aij[11][1]  = 0.0;
   rk->aij[11][2]  = 0.0;
   rk->aij[11][3]  = 0.0;
   rk->aij[11][4]  = 0.0;
   rk->aij[11][5]  = (-4342.0 - 338.0 * rt6) / 9.0;
   rk->aij[11][6]  = (154922.0 - 40458.0 * rt6) / 135.0;
   rk->aij[11][7]  = (-4176.0 + 3794.0 * rt6) / 45.0;
   rk->aij[11][8]  = (-340864.0 + 242816.0 * rt6) / 405.0;
   rk->aij[11][9]  = (26304.0 - 15176.0 * rt6) / 45.0;
   rk->aij[11][10] = -26624.0 / 81.0;
   rk->aij[11][11] = 0.0;

   rk->aij[12][0]  = (3793.0 + 2168.0 * rt6) / 103680.0;
   rk->aij[12][1]  = 0.0;
   rk->aij[12][2]  = 0.0;
   rk->aij[12][3]  = 0.0;
   rk->aij[12][4]  = 0.0;
   rk->aij[12][5]  = (4042.0 + 2263.0 * rt6) / 13824.0;
   rk->aij[12][6]  = (-231278.0 + 40717.0 * rt6) / 69120.0;
   rk->aij[12][7]  = (7947.0 - 2168.0 * rt6) / 11520.0;
   rk->aij[12][8]  = (1048.0 - 542.0 * rt6) / 405.0;
   rk->aij[12][9]  = (-1383.0 + 542.0 * rt6) / 720.0;
   rk->aij[12][10] = 2624.0 / 1053.0;
   rk->aij[12][11] = 3.0 / 1664.0;
   rk->aij[12][12] = 0.0;

   rk->aij[13][0]  = -137.0 / 1296.0;
   rk->aij[13][1]  = 0.0;
   rk->aij[13][2]  = 0.0;
   rk->aij[13][3]  = 0.0;
   rk->aij[13][4]  = 0.0;
   rk->aij[13][5]  = (5642.0 - 337.0 * rt6) / 864.0;
   rk->aij[13][6]  = (5642.0 + 337.0 * rt6) / 864.0;
   rk->aij[13][7]  = -299.0 / 48.0;
   rk->aij[13][8]  = 184.0 / 81.0;
   rk->aij[13][9]  = -44.0 / 9.0;
   rk->aij[13][10] = -5120.0 / 1053.0;
   rk->aij[13][11] = -11.0 / 468.0;
   rk->aij[13][12] = 16.0 / 9.0;
   rk->aij[13][13] = 0.0;

   rk->aij[14][0]  = (33617.0 - 2168.0 * rt6) / 518400.0;
   rk->aij[14][1]  = 0.0;
   rk->aij[14][2]  = 0.0;
   rk->aij[14][3]  = 0.0;
   rk->aij[14][4]  = 0.0;
   rk->aij[14][5]  = (-3846.0 + 31.0 * rt6) / 13824.0;
   rk->aij[14][6]  = (155338.0 - 52807.0 * rt6) / 345600.0;
   rk->aij[14][7]  = (-12537.0 + 2168.0 * rt6) / 57600.0;
   rk->aij[14][8]  = (92.0 + 542.0 * rt6) / 2025.0;
   rk->aij[14][9]  = (-1797.0 - 542.0 * rt6) / 3600.0;
   rk->aij[14][10] = 320.0 / 567.0;
   rk->aij[14][11] = -1.0 / 1920.0;
   rk->aij[14][12] = 4.0 / 105.0;
   rk->aij[14][13] = 0.0;
   rk->aij[14][14] = 0.0;

   rk->aij[15][0]  = (-36487.0 - 30352.0 * rt6) / 279600.0;
   rk->aij[15][1]  = 0.0;
   rk->aij[15][2]  = 0.0;
   rk->aij[15][3]  = 0.0;
   rk->aij[15][4]  = 0.0;
   rk->aij[15][5]  = (-29666.0 - 4499.0 * rt6) / 7456.0;
   rk->aij[15][6]  = (2779182.0 - 615973.0 * rt6) / 186400.0;
   rk->aij[15][7]  = (-94329.0 + 91056.0 * rt6) / 93200.0;
   rk->aij[15][8]  = (-232192.0 + 121408.0 * rt6) / 17475.0;
   rk->aij[15][9]  = (101226.0 - 22764.0 * rt6) / 5825.0;
   rk->aij[15][10] = -169984.0 / 9087.0;
   rk->aij[15][11] = -87.0 / 30290.0;
   rk->aij[15][12] = 492.0 / 1165.0;
   rk->aij[15][13] = 0.0;
   rk->aij[15][14] = 1260.0 / 233.0;
   rk->aij[15][15] = 0.0;

   rk->bj[0]  = 23.0 / 525.0;
   rk->bj[1]  = 0.0;
   rk->bj[2]  = 0.0;
   rk->bj[3]  = 0.0;
   rk->bj[4]  = 0.0;
   rk->bj[5]  = 0.0;
   rk->bj[6]  = 0.0;
   rk->bj[7]  = 171.0 / 1400.0;
   rk->bj[8]  = 86.0 / 525.0;
   rk->bj[9]  = 93.0 / 280.0;
   rk->bj[10] = -2048.0 / 6825.0;
   rk->bj[11] = -3.0 / 18200.0;
   rk->bj[12] = 39.0 / 175.0;
   rk->bj[13] = 0.0;
   rk->bj[14] = 9.0 / 25.0;
   rk->bj[15] = 233.0 / 4200.0;

   rk->ee[0]  = -7.0 / 400.0;
   rk->ee[1]  = 0.0;
   rk->ee[2]  = 0.0;
   rk->ee[3]  = 0.0;
   rk->ee[4]  = 0.0;
   rk->ee[5]  = 0.0;
   rk->ee[6]  = 0.0;
   rk->ee[7]  = 63.0 / 200.0;
   rk->ee[8]  = -14.0 / 25.0;
   rk->ee[9]  = 21.0 / 20.0;
   rk->ee[10] = -1024.0 / 975.0;
   rk->ee[11] = -21.0 / 36400.0;
   rk->ee[12] = -3.0 / 25.0;
   rk->ee[13] = -9.0 / 280.0;
   rk->ee[14] = 9.0 / 25.0;
   rk->ee[15] = 233.0 / 4200.0;
}

static double _estimateError(RungeKutta *const rk)
{
   for (int i = 0; i < rk->dim; i++) {
      rk->errorEsts[i] = 0.0;
      for (int j = 0; j < rk->stages; j++)
         rk->errorEsts[i] += rk->ee[j] * rk->ki[j][i];
   }
   return rk->errorCalc(rk->errorEsts, rk->candidateState, rk->inState,
                        rk->relErrThreshold, rk->dim);
}

static void _rawstep(RungeKutta *const rk)
{
   for (int i = 0; i < rk->stages; i++) {
      CopyVG(rk->stageState, rk->inState, rk->dim);
      RKIndType time = rk->curTime;
      if (i > 0) {
         time = JDaxpy(rk->ci[i], rk->stepSize, time);
         for (int j = 0; j < i; j++)
            axpy(rk->aij[i][j], rk->ki[j], rk->stageState, rk->dim);
      }
      rk->ode(time, rk->stageState, rk->params, rk->stateDot);

      SxVG(JDToSeconds(rk->stepSize), rk->stateDot, rk->ki[i], rk->dim);
   }

   CopyVG(rk->candidateState, rk->inState, rk->dim);
   for (int i = 0; i < rk->stages; i++)
      axpy(rk->bj[i], rk->ki[i], rk->candidateState, rk->dim);
}

static int _adaptstep(RungeKutta *const rk, const double maxErr)
{
   if (maxErr > rk->tol) {
      rk->stepSize = JDaxpy(rk->sigma * pow(rk->tol / maxErr, rk->decPower),
                            rk->stepSize, JD_ZERO);
      if (isless_jd(JDAbs(rk->stepSize), rk->minStep))
         rk->stepSize =
             ispos_jd(rk->stepSize) ? rk->minStep : JDNegate(rk->minStep);
      return 0;
   }

   rk->stepSize = JDaxpy(rk->sigma * pow(rk->tol / maxErr, rk->incPower),
                         rk->stepSize, JD_ZERO);
   return 1;
}

static void _step(RungeKutta *const rk)
{
   if ((isless_jd(JDAbs(rk->stepSize), rk->minStep)))
      rk->stepSize =
          (ispos_jd(rk->stepSize) ? rk->minStep : JDNegate(rk->minStep));
   if (isgreater_jd(JDAbs(rk->stepSize), rk->maxStep))
      rk->stepSize =
          (ispos_jd(rk->stepSize) ? rk->maxStep : JDNegate(rk->maxStep));

   int goodStep = -1;
   do {
      _rawstep(rk);
      rk->StepTaken = rk->stepSize;
      rk->stepAttempts++;
      if (!rk->hasErrorControl || _adaptstep(rk, _estimateError(rk))) {
         CopyVG(rk->outState, rk->candidateState, rk->dim);
         rk->stepAttempts = 0;
         goodStep         = 1;
      }

      if (rk->stepAttempts >= rk->maxStepAttempts) {
         fprintf(stderr, "%d step attempts taken; max is %d. Exiting...\n",
                 rk->stepAttempts, rk->maxStepAttempts);
         exit(EXIT_FAILURE);
      }

   } while (!goodStep);
   rk->curTime = JDaxpy(1.0, rk->StepTaken, rk->curTime);
}

void RungeKuttaStep(RungeKutta *const rk, RKIndType t0, double dt_seconds,
                    double *x)
{
   int stepFinished = 0;
   RKIndType timeLeft =
       JDFromSeconds(dt_seconds, JD_ZERO.system, JD_ZERO.epoch);
   int attemptsTaken = 0;
   rk->stepAttempts  = 0;

   rk->curTime  = t0;
   rk->inState  = x;
   rk->outState = rk->inState;
   do {
      if (attemptsTaken > rk->maxStepAttempts) {
         fprintf(stderr,
                 "Integrator attempted too many steps! (%d attempts taken). "
                 "Exiting...\n",
                 attemptsTaken);
         exit(EXIT_FAILURE);
      }

      rk->stepSize = timeLeft;
      _step(rk);
      JDType time_diff_abs = JDAbs(JDSub(timeLeft, rk->StepTaken));
      if (islessequal_jd(time_diff_abs, rk->smallestTime))
         stepFinished = 1;

      timeLeft = JDSub(timeLeft, rk->StepTaken);
      attemptsTaken++;
   } while (!stepFinished);
}

RungeKutta GetRungeKutta(
    RKType type, const double tol, const double relErrThreshold,
    const int dimension, const double minStep, const double maxStep,
    RKParams *params,
    void (*const ode)(RKIndType t, double *x, RKParams *const params,
                      double *xdot),
    double (*const errorCalc)(const double *const errEst,
                              const double *const candidateState,
                              const double *cur_state,
                              const double relErrThreshold, const long dim))
{
   RungeKutta rk;
   rk.isInitialized = 0;
   switch (type) {
      case EULER_RK:
         _initeuler(&rk, dimension);
         break;
      case THERK4_RK:
         _inittherk4(&rk, dimension);
         break;
      case RK4_RK:
         _init38rk4(&rk, dimension);
         break;
      case RK89_RK:
         _initrk89(&rk, dimension);
         break;
      default:
         fprintf(stderr,
                 "Unknown Runge Kutta type in GetRungeKutta(). Exiting...\n");
         exit(EXIT_FAILURE);
   }
   rk.minStep = JDFromSeconds(fabs(minStep), TT_TIME, ZERO_EPOCH);
   rk.maxStep = JDFromSeconds(fabs(maxStep), TT_TIME, ZERO_EPOCH);
   if (ode == NULL) {
      fprintf(
          stderr,
          "The ode input to GetRungeKutta is required to be set. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   rk.ode = ode;
   if (errorCalc != NULL)
      rk.errorCalc = errorCalc;
   if (tol > 0)
      rk.tol = tol;
   if (relErrThreshold > 0)
      rk.relErrThreshold = relErrThreshold;
   rk.isInitialized = 1;
   rk.params        = params; //
   return rk;
}

double RKErrorCalc(const double *const errEst,
                   const double *const candidateState, const double *cur_state,
                   const double relErrThreshold, const long dim)
{
   double retval = 0.0;
   for (int i = 0; i < dim; i++) {
      double err;
      double delta = candidateState[i] - cur_state[i];
      if (fabs(delta) > relErrThreshold)
         err = fabs(errEst[i] / delta);
      else
         err = fabs(errEst[i]);
      if (err > retval)
         retval = err;
   }
   return retval;
}

/* #ifdef __cplusplus
** }
** #endif
*/
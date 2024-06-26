/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __NAVKIT_H__
#define __NAVKIT_H__

#include "42.h"
#include "42types.h"
#include "AcTypes.h"
#include "DSMTypes.h"

enum matType {
   Q_DAT,
   P0_DAT,
   IC_DAT,
};

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

void InitMeasList(struct DSMMeasListType *list);
void appendMeas(struct DSMMeasListType *list, struct DSMMeasType *newRef);
void appendList(struct DSMMeasListType *list1, struct DSMMeasListType *list2);
void DestroyMeas(struct DSMMeasType *meas);
void push(struct DSMMeasListType *list, struct DSMMeasType *meas);
struct DSMMeasType *pop_DSMMeas(struct DSMMeasListType *list);
void DestroyMeasList(struct DSMMeasListType *list);
struct DSMMeasType *swap_DSMMeas(struct DSMMeasType *ptr1,
                                 struct DSMMeasType *ptr2);
void bubbleSort(struct DSMMeasListType *list);
struct DSMMeasType *CreateMeas(struct DSMNavType *const Nav,
                               enum sensorType const type,
                               long const sensorNum);
int comparator_DSMMeas(const void *v1, const void *v2);

void updateNavTime(struct DateType *Time, const double dSeconds);
double gpsTime2J2000Sec(const long gpsRollover, const long gpsWeek,
                        const double gpsSec);

/*--------------------------------------------------------------------*/
/*                    Navigation Filter Functions                     */
/*--------------------------------------------------------------------*/
double **GetStateLinTForm(struct SCType *const S);
void configureRefFrame(struct SCType *const S, const double dLerpAlpha,
                       const long reset);
void getForceAndTorque(struct AcType *const AC, struct DSMNavType *const Nav,
                       double const CRB[3][3], double const *whlH);
void PropagateNav(struct SCType *const S, const long dSubStep,
                  const long dStep);
void KalmanFilt(struct SCType *const S);

/*--------------------------------------------------------------------*/
/*                       Measurement Jacobians                        */
/*--------------------------------------------------------------------*/
double **gyroJacobianFun(struct SCType *const S, const long sensorNum);
double **magJacobianFun(struct SCType *const S, const long sensorNum);
double **cssJacobianFun(struct SCType *const S, const long sensorNum);
double **fssJacobianFun(struct SCType *const S, const long sensorNum);
double **startrackJacobianFun(struct SCType *const S, const long sensorNum);
double **gpsJacobianFun(struct SCType *const S, const long sensorNum);
double **accelJacobianFun(struct SCType *const S, const long sensorNum);

double *gyroFun(struct SCType *const S, const long sensorNum);
double *magFun(struct SCType *const S, const long sensorNum);
double *cssFun(struct SCType *const S, const long sensorNum);
double *fssFun(struct SCType *const S, const long sensorNum);
double *startrackFun(struct SCType *const S, const long sensorNum);
double *gpsFun(struct SCType *const S, const long sensorNum);
double *accelFun(struct SCType *const S, const long sensorNum);

/*--------------------------------------------------------------------*/
/*                          RIEKF functions                           */
/*--------------------------------------------------------------------*/

void eomRIEKFJacobianFun(struct SCType *const S, const struct DateType *date,
                         double const CRB[3][3], double const qbr[4],
                         double const PosR[3], double const VelR[3],
                         double const wbr[3], double const whlH[S->AC.Nwhl],
                         const double AtmoDensity);
void RIEKFUpdateLaw(struct DSMNavType *const Nav);

/*--------------------------------------------------------------------*/
/*                          LIEKF functions                           */
/*--------------------------------------------------------------------*/

void eomLIEKFJacobianFun(struct SCType *const S, const struct DateType *date,
                         double const CRB[3][3], double const qbr[4],
                         double const PosR[3], double const VelR[3],
                         double const wbr[3], double const whlH[S->AC.Nwhl],
                         const double AtmoDensity);
void LIEKFUpdateLaw(struct DSMNavType *const Nav);

/*--------------------------------------------------------------------*/
/*                          MEKF functions                           */
/*--------------------------------------------------------------------*/

void eomMEKFJacobianFun(struct SCType *const S, const struct DateType *date,
                        double const CRB[3][3], double const qbr[4],
                        double const PosR[3], double const VelR[3],
                        double const wbr[3], double const whlH[S->AC.Nwhl],
                        const double AtmoDensity);
void MEKFUpdateLaw(struct DSMNavType *const Nav);

/******************************************************************************/
//                          Auxillary Math Functions
/******************************************************************************/
void subMatAdd(double **A, double **B, long const iN, long const iM,
               long const n, long const M);
double mahalonobis2(double **A, double *x, long const n);
double chi2InvLookup(double const pGate, long const dim);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __NAVKIT_H__ */
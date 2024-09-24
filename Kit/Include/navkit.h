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
                               enum SensorType const type,
                               long const sensorNum);
int comparator_DSMMeas(const void *v1, const void *v2);

void updateNavTime(struct DateType *Time, const double dSeconds);
double gpsTime2J2000Sec(const long gpsRollover, const long gpsWeek,
                        const double gpsSec);

/*--------------------------------------------------------------------*/
/*                    Navigation Filter Functions                     */
/*--------------------------------------------------------------------*/
double **GetStateLinTForm(struct DSMNavType *const Nav);
void UnscentedStateTForm(struct DSMNavType *const Nav, double *mean,
                         double **P);
void configureRefFrame(struct DSMNavType *const Nav,
                       const struct OrbitType *refOrb, const double dLerpAlpha,
                       const long reset);
void getForceAndTorque(struct AcType *const AC, struct DSMNavType *const Nav,
                       double const CRB[3][3], double const *whlH);
void PropagateNav(struct AcType *const AC, struct DSMType *const DSM,
                  const long dSubStep, const long dStep);
void KalmanFilt(struct AcType *const AC, struct DSMType *const DSM);

/*--------------------------------------------------------------------*/
/*                       Measurement Jacobians                        */
/*--------------------------------------------------------------------*/
double **gyroJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                         const long sensorNum);
double **magJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                        const long sensorNum);
double **cssJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                        const long sensorNum);
double **fssJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                        const long sensorNum);
double **startrackJacobianFun(struct AcType *const AC,
                              struct DSMType *const DSM, const long sensorNum);
double **gpsJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                        const long sensorNum);
double **accelJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                          const long sensorNum);

double *gyroFun(struct AcType *const AC, struct DSMType *const DSM,
                const long sensorNum);
double *magFun(struct AcType *const AC, struct DSMType *const DSM,
               const long sensorNum);
double *cssFun(struct AcType *const AC, struct DSMType *const DSM,
               const long sensorNum);
double *fssFun(struct AcType *const AC, struct DSMType *const DSM,
               const long sensorNum);
double *startrackFun(struct AcType *const AC, struct DSMType *const DSM,
                     const long sensorNum);
double *gpsFun(struct AcType *const AC, struct DSMType *const DSM,
               const long sensorNum);
double *accelFun(struct AcType *const AC, struct DSMType *const DSM,
                 const long sensorNum);

/*--------------------------------------------------------------------*/
/*                          RIEKF functions                           */
/*--------------------------------------------------------------------*/

void eomRIEKFJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                         const struct DateType *date, double const CRB[3][3],
                         double const qbr[4], double const PosR[3],
                         double const VelR[3], double const wbr[3],
                         double const whlH[AC->Nwhl], const double AtmoDensity);
void RIEKFUpdateLaw(struct DSMNavType *const Nav);

/*--------------------------------------------------------------------*/
/*                          LIEKF functions                           */
/*--------------------------------------------------------------------*/

void eomLIEKFJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                         const struct DateType *date, double const CRB[3][3],
                         double const qbr[4], double const PosR[3],
                         double const VelR[3], double const wbr[3],
                         double const whlH[AC->Nwhl], const double AtmoDensity);
void LIEKFUpdateLaw(struct DSMNavType *const Nav);

/*--------------------------------------------------------------------*/
/*                          MEKF functions                           */
/*--------------------------------------------------------------------*/

void eomMEKFJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                        const struct DateType *date, double const CRB[3][3],
                        double const qbr[4], double const PosR[3],
                        double const VelR[3], double const wbr[3],
                        double const whlH[AC->Nwhl], const double AtmoDensity);
void MEKFUpdateLaw(struct DSMNavType *const Nav);

/******************************************************************************/
//                          Auxillary Math Functions
/******************************************************************************/
void subMatAdd(double **A, double **B, long const iN, long const iM,
               long const n, long const M);
double mahalonobis2(double **A, double *x, double *y, long const n);
double chi2InvLookup(double const pGate, long const dim);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __NAVKIT_H__ */
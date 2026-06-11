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

#include "42types.h"
#include "AcTypes.h"
#include "DSMTypes.h"
#include "defineskit.h"

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
__attribute__((malloc)) struct DSMMeasType *
CreateMeas(struct DSMNavType *const Nav, enum SensorType const type,
           long const sensorNum);
int comparator_DSMMeas(const void *v1, const void *v2);

void updateNavCCSDS(CCSDSTime ccsds_time, const double dSeconds);
__attribute__((const)) double gpsTime2J2000Sec(const long gpsRollover,
                                               const long gpsWeek,
                                               const double gpsSec);

/*--------------------------------------------------------------------*/
/*                    Navigation Filter Functions                     */
/*--------------------------------------------------------------------*/
__attribute__((malloc)) double **GetStateLinTForm(struct DSMNavType *const Nav);
void UnscentedStateTForm(struct DSMNavType *const Nav, double *mean,
                         double **P);
void configureRefFrame(struct DSMNavType *const Nav, double *const lerp_alpha,
                       const struct OrbitType *refOrb, const double dLerpAlpha,
                       const long reset);
void getForceAndTorque(struct AcType *const AC, struct DSMNavType *const Nav,
                       const mat3x3_t CRB, const double *whlH);
void PropagateNav(struct AcType *const AC, struct DSMType *const DSM,
                  CCSDSTime *const cur_ccsds, const CCSDSTime next_ccsds,
                  const long init);
void KalmanFilt(struct AcType *const AC, struct DSMType *const DSM);

/*--------------------------------------------------------------------*/
/*                       Measurement Jacobians                        */
/*--------------------------------------------------------------------*/
__attribute__((malloc)) double **gyroJacobianFun(struct AcType *const AC,
                                                 struct DSMType *const DSM,
                                                 const long sensorNum,
                                                 double **N);
__attribute__((malloc)) double **magJacobianFun(struct AcType *const AC,
                                                struct DSMType *const DSM,
                                                const long sensorNum,
                                                double **N);
__attribute__((malloc)) double **cssJacobianFun(struct AcType *const AC,
                                                struct DSMType *const DSM,
                                                const long sensorNum,
                                                double **N);
__attribute__((malloc)) double **fssJacobianFun(struct AcType *const AC,
                                                struct DSMType *const DSM,
                                                const long sensorNum,
                                                double **N);
__attribute__((malloc)) double **startrackJacobianFun(struct AcType *const AC,
                                                      struct DSMType *const DSM,
                                                      const long sensorNum,
                                                      double **N);
__attribute__((malloc)) double **gpsJacobianFun(struct AcType *const AC,
                                                struct DSMType *const DSM,
                                                const long sensorNum,
                                                double **N);
__attribute__((malloc, unused)) double **
accelJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                 const long sensorNum, double **N);

__attribute__((malloc)) double *gyroFun(struct AcType *const AC,
                                        struct DSMType *const DSM,
                                        const long sensorNum);
__attribute__((malloc)) double *magFun(struct AcType *const AC,
                                       struct DSMType *const DSM,
                                       const long sensorNum);
__attribute__((malloc)) double *cssFun(struct AcType *const AC,
                                       struct DSMType *const DSM,
                                       const long sensorNum);
__attribute__((malloc)) double *fssFun(struct AcType *const AC,
                                       struct DSMType *const DSM,
                                       const long sensorNum);
__attribute__((malloc)) double *startrackFun(struct AcType *const AC,
                                             struct DSMType *const DSM,
                                             const long sensorNum);
__attribute__((malloc)) double *gpsFun(struct AcType *const AC,
                                       struct DSMType *const DSM,
                                       const long sensorNum);
__attribute__((malloc, unused)) double *accelFun(struct AcType *const AC,
                                                 struct DSMType *const DSM,
                                                 const long sensorNum);

/*--------------------------------------------------------------------*/
/*                          RIEKF functions                           */
/*--------------------------------------------------------------------*/

void eomRIEKFJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                         const DateType *date, const mat3x3_t CRB,
                         const quat_t qbr, const vec3_t PosR, const vec3_t VelR,
                         const vec3_t wbr, const double whlH[AC->Nwhl],
                         const double AtmoDensity, double **jacobian);
void RIEKFUpdateLaw(struct DSMNavType *const Nav);

/*--------------------------------------------------------------------*/
/*                          LIEKF functions                           */
/*--------------------------------------------------------------------*/

void eomLIEKFJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                         const DateType *date, const mat3x3_t CRB,
                         const quat_t qbr, const vec3_t PosR, const vec3_t VelR,
                         const vec3_t wbr, const double whlH[AC->Nwhl],
                         const double AtmoDensity, double **jacobian);
void LIEKFUpdateLaw(struct DSMNavType *const Nav);

/*--------------------------------------------------------------------*/
/*                          MEKF functions                           */
/*--------------------------------------------------------------------*/

void eomMEKFJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                        const DateType *date, const mat3x3_t CRB,
                        const quat_t qbr, const vec3_t PosR, const vec3_t VelR,
                        const vec3_t wbr, const double whlH[AC->Nwhl],
                        const double AtmoDensity, double **jacobian);
void MEKFUpdateLaw(struct DSMNavType *const Nav);

/******************************************************************************/
//                          Auxillary Math Functions
/******************************************************************************/
void subMatAdd(double **A, double **B, long const iN, long const iM,
               long const n, long const M);
__attribute__((pure)) double mahalonobis2(double **A, double *x, double *y,
                                          long const n);
__attribute__((const)) double chi2InvLookup(double const pGate, long const dim);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __NAVKIT_H__ */
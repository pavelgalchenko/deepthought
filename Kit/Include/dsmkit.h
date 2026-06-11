/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "AcTypes.h"
#include "DSMTypes.h"
#include "mathkit.h"

#ifndef __DSMKIT_H__
#define __DSMKIT_H__

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

#ifdef __cplusplus
extern "C" {
#endif

vec3_t DSM_RelMotionToAngRate(vec3_t RelPosN, vec3_t RelVelN);
void DSM_WheelProcessing(struct AcType *AC);
void DSM_MtbProcessing(struct AcType *AC);
__attribute__((malloc)) struct DSMMeasListType *
DSM_GyroProcessing(struct AcType *const AC, struct DSMType *const DSM);
__attribute__((malloc)) struct DSMMeasListType *
DSM_MagnetometerProcessing(struct AcType *const AC, struct DSMType *const DSM);
__attribute__((malloc)) struct DSMMeasListType *
DSM_CssProcessing(struct AcType *const AC, struct DSMType *const DSM);
__attribute__((malloc)) struct DSMMeasListType *
DSM_FssProcessing(struct AcType *const AC, struct DSMType *const DSM);
__attribute__((malloc)) struct DSMMeasListType *
DSM_StarTrackerProcessing(struct AcType *const AC, struct DSMType *const DSM);
__attribute__((malloc)) struct DSMMeasListType *
DSM_GpsProcessing(struct AcType *const AC, struct DSMType *const DSM);
__attribute__((malloc)) struct DSMMeasListType *
DSM_AccelProcessing(struct AcType *const AC, struct DSMType *const DSM);
__attribute__((const)) struct DSMStateType
DSM_CommStateProcessing(struct DSMStateType state);

#ifdef __cplusplus
}
#endif

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __FSWKIT_H__ */

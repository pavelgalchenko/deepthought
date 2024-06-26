/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "42defines.h"
#include "AcTypes.h"
#include "DSMTypes.h"
#include "dcmkit.h"
#include "fswkit.h"
#include "mathkit.h"

#ifndef __DSMKIT_H__
#define __DSMKIT_H__

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

void DSM_RelMotionToAngRate(double RelPosN[3], double RelVelN[3], double wn[3]);
void DSM_WheelProcessing(struct AcType *AC);
void DSM_MtbProcessing(struct AcType *AC);
struct DSMMeasListType *DSM_GyroProcessing(struct AcType *const AC,
                                           struct DSMType *const DSM);
struct DSMMeasListType *DSM_MagnetometerProcessing(struct AcType *const AC,
                                                   struct DSMType *const DSM);
struct DSMMeasListType *DSM_CssProcessing(struct AcType *const AC,
                                          struct DSMType *const DSM);
struct DSMMeasListType *DSM_FssProcessing(struct AcType *const AC,
                                          struct DSMType *const DSM);
struct DSMMeasListType *DSM_StarTrackerProcessing(struct AcType *const AC,
                                                  struct DSMType *const DSM);
struct DSMMeasListType *DSM_GpsProcessing(struct AcType *const AC,
                                          struct DSMType *const DSM);
struct DSMMeasListType *DSM_AccelProcessing(struct AcType *const AC,
                                            struct DSMType *const DSM);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __FSWKIT_H__ */

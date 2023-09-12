/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */


#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "42defines.h"
#include "AcTypes.h"
#include "mathkit.h"
#include "fswkit.h"
#include "dcmkit.h"

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
void DSM_GyroProcessing(struct AcType *AC);
void DSM_MagnetometerProcessing(struct AcType *AC);
void DSM_CssProcessing(struct AcType *AC);
void DSM_FssProcessing(struct AcType *AC);
void DSM_StarTrackerProcessing(struct AcType *AC);
void DSM_GpsProcessing(struct AcType *AC);
void DSM_AccelProcessing(struct AcType *AC);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __FSWKIT_H__ */

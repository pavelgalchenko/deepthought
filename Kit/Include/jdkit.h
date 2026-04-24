/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __JDKIT_H__
#define __JDKIT_H__

#include "42constants.h"
#include <ctype.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

/**********************************************************************/
/**********************************************************************/
// NOTE: IN THIS CODE,
//    'MJD' USES EPOCH GD 05 Jan 1941 12:00:00.000 (JD 2,430,000.0 TT)
/**********************************************************************/
/**********************************************************************/

typedef enum Epoch {
   ZERO_EPOCH = 0,
   J2000_EPOCH,
   MJD_EPOCH,
   GMAT_MJD_EPOCH,
   GD_CONV_EPOCH,
   TCB_TDB_CONV_EPOCH,
   // TODO: other epochs?
} Epoch; // TODO: type different name?

typedef enum TimeSystem {
   UTC_TIME = 0, // Coordinated Universal Time
   TAI_TIME,     // International Atomic Time
   TCB_TIME,     // Barycentric Coordinate Time
   TDB_TIME,     // Barycentric Dynamical Time
   TT_TIME,      // Terrestrial Time, sometimes uses old TDT term
   // TODO: add TT(BIPM)? others?
} TimeSystem;

// some strict typing to enforce correct timing interpretation
typedef struct {
   // Julian day representation
   // while the value of day changes with the time system, the value indicated
   // by 'epoch' will always be in the TT system
   TimeSystem system;
   double day;
   Epoch epoch;

   // TODO: enforce epoch as TT?
   // Typical values for epoch are:
   //    - 0.0 TT for typical Julian Day
   //    - 2,430,000.0 TT for GMAT-like modified Julian Day
   //    - 2,451,545.0 TT for J2000 epoch
   // Epoch epoch; ?
} JDType;

double GetLeapSec(const JDType jd);
double EpochValueTT(Epoch epoch);
void ChangeEpoch(const Epoch new_epoch, JDType *const jd);
void ChangeSystem(const TimeSystem new_system, JDType *const jd);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __TIMEKIT_H__ */

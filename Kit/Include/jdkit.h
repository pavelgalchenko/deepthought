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

typedef enum EpochTT {
   ZERO_EPOCH = 0,     // Jan  1, -4712, 12:00:00
   GD_CONV_EPOCH,      // Nov 18, -0001, 00:00:00, in GD->JD
   TCB_TDB_CONV_EPOCH, // Jan  1,  1977, 00:00:00, in tcb->tdb
   MJD_EPOCH,          // Nov 17,  1858, 00:00:00
   J1900_EPOCH,        // Dec 31,  1899, 00:00:00, in JD->GD
   GMAT_MJD_EPOCH,     // Jan  5,  1941, 12:00:00
   CCSDS_EPOCH,        // Jan 1,   1958, 00:00:00
   J2000_EPOCH,        // Jan 1 ,  2000, 12:00:00, J2000 epoch
   // TODO: other epochs?
} EpochTT;
// TODO: add CUSTOM_EPOCH, but then epoch in JDType will need to be a struct
// with members of EpochTT and a value that is used only for the custom value

typedef enum TimeSystem {
   UTC_TIME = 0, // Coordinated Universal Time
   TAI_TIME,     // International Atomic Time
   TCB_TIME,     // Barycentric Coordinate Time
   TDB_TIME,     // Barycentric Dynamical Time
   TT_TIME,      // Terrestrial Time, sometimes referred as old TDT term
   // TODO: add TT(BIPM)? others?
} TimeSystem;

// some strict typing to enforce correct timing interpretation
typedef struct {
   // Julian day representation
   // while the value of day changes with the time system, the value indicated
   // by 'epoch' will always be in the TT system
   TimeSystem system;
   double day;
   EpochTT epoch;
   // TODO: track an integer day and a double part of day independently?
   // GMAT uses an integer day, integer seconds of day, and a double fractional
   // second
} JDType;

double GetLeapSec(const JDType jd);
double EpochValueTT(EpochTT epoch);
void ChangeEpoch(const EpochTT new_epoch, JDType *const jd);
void ChangeSystem(const TimeSystem new_system, JDType *const jd);
void ChangeSystemEpoch(const TimeSystem new_system, const EpochTT new_epoch,
                       JDType *const jd);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __TIMEKIT_H__ */

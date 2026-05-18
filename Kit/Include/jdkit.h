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
#include "rationalkit.h"
#include <ctype.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
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

#define JD_STR_LEN       (138)
#define JDEPOCH_STR_LEN  (19)
#define JDSYSTEM_STR_LEN (4)

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
   TT_TIME,      // Terrestrial Time, sometimes referred as old TDT term
   TCB_TIME,     // Barycentric Coordinate Time
   TDB_TIME,     // Barycentric Dynamical Time
   // TODO: add TT(BIPM)? others?
} TimeSystem;

// some strict typing to enforce correct timing interpretation
typedef struct JDType {
   // Julian day representation
   // while the value of day changes with the time system, the value indicated
   // by 'epoch' will always be in the TT system
   TimeSystem system;
   EpochTT epoch;
   long whole_days;
   Rational seconds;
} JDType;

#define JD_RAW(sys, epc, day, sec)                                             \
   ((JDType){.system     = (sys),                                              \
             .epoch      = (epc),                                              \
             .whole_days = (day),                                              \
             .seconds    = (sec)})
#define JD_ZERO JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO)
#define JD_NREDUCE(sys, epc, day)                                              \
   JD_RAW(sys, epc, day, RATIONAL_NGCD((day - ((long)day)) * 86400.0, 0, 1))

JDType InitJD(const TimeSystem system, const EpochTT epoch, const long days,
              const Rational seconds);
double GetLeapSec(const JDType jd);
double EpochValueTT(EpochTT epoch);
void JDChangeEpoch(const EpochTT new_epoch, JDType *const jd);
void JDChangeSystem(const TimeSystem new_system, JDType *const jd);
void JDChangeSystemEpoch(const TimeSystem new_system, const EpochTT new_epoch,
                         JDType *const jd);
double JDToDays(const JDType jd);
JDType JDFromDays(const double days, const TimeSystem system,
                  const EpochTT new_epoch);
JDType JDFromSeconds(const double seconds, const TimeSystem system,
                     const EpochTT new_epoch);
JDType JDFromRationalSeconds(const Rational seconds, const TimeSystem system,
                             const EpochTT new_epoch);
double JDToSeconds(JDType jd);
Rational JDToRationalSeconds(JDType jd);
double JDToTime(JDType jd);
double JDToDynTime(JDType JD);

JDType JDAdd(const JDType a, const JDType b);
JDType JDAddDays(const JDType a, const double b);
JDType JDAddSeconds(const JDType a, const double b);
JDType JDAddRationalSeconds(const JDType a, const Rational b);
JDType JDAddMultRatSecs(const JDType jd, const long mul, const Rational rat);
JDType JDSub(const JDType a, const JDType b);
JDType JDSubDays(const JDType a, const double b);
JDType JDSubSeconds(const JDType a, const double b);
JDType JDSubRationalSeconds(const JDType a, const Rational b);
JDType JDaxpy(const double a, const JDType x, JDType y);
double JDAddToDays(const JDType a, const JDType b);
double JDAddToSeconds(const JDType a, const JDType b);
double JDSubToDays(const JDType a, const JDType b);
double JDSubToSeconds(const JDType a, const JDType b);

JDType JDAbs(JDType jd);
int ispos_jd(JDType jd);
JDType JDNegate(JDType jd);

int isequal_jd(const JDType a, const JDType b);
int isless_jd(const JDType a, const JDType b);
int islessequal_jd(const JDType a, const JDType b);
int isgreater_jd(const JDType a, const JDType b);
int isgreaterequal_jd(const JDType a, const JDType b);

EpochTT str2epoch(char str[JDEPOCH_STR_LEN]);
TimeSystem str2system(char str[JDSYSTEM_STR_LEN]);
void epoch2str(EpochTT epoch, char str[JDEPOCH_STR_LEN]);
void system2str(TimeSystem system, char str[JDSYSTEM_STR_LEN]);
void jd2str(JDType jd, char str[JD_STR_LEN]);
void jddays2str(JDType jd, char str[JD_STR_LEN]);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __JDKIT_H__ */

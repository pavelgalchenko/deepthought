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
#include "defineskit.h"
#include "mathkit.h"
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

#define JD_STR_LEN        (138)
#define JDEPOCH_STR_LEN   (19)
#define JDSYSTEM_STR_LEN  (4)
#define JDDAY_PER_CENTURY (36525)

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

TimeSystem GetTimeSystem(const char *s) __attribute__((pure));

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
   JD_RAW((sys), (epc), (day),                                                 \
          RATIONAL_NGCD(((day) - ((long)(day))) * 86400.0, 0, 1))

JDType InitJD(const TimeSystem system, const EpochTT epoch, const long days,
              const Rational seconds) __attribute__((const));
double GetLeapSec(const JDType jd) __attribute__((const));
double EpochValueTT(EpochTT epoch) __attribute__((const));
JDType JDChangeEpoch(const EpochTT new_epoch, JDType jd) __attribute__((const));
JDType JDChangeSystem(const TimeSystem new_system, JDType jd)
    __attribute__((const));
JDType JDChangeSystemEpoch(const TimeSystem new_system, const EpochTT new_epoch,
                           JDType jd) __attribute__((const));
double JDToDays(const JDType jd) __attribute__((const));
JDType JDFromDays(const double days, const TimeSystem system,
                  const EpochTT new_epoch) __attribute__((const));
JDType JDFromSeconds(const double seconds, const TimeSystem system,
                     const EpochTT new_epoch) __attribute__((const));
JDType JDFromRationalSeconds(const Rational seconds, const TimeSystem system,
                             const EpochTT new_epoch) __attribute__((const));
double JDToSeconds(JDType jd) __attribute__((const));
Rational JDToRationalSeconds(JDType jd) __attribute__((const));
double JDToTime(JDType jd) __attribute__((const));
double JDToDynTime(JDType JD) __attribute__((const));

JDType JDAdd(const JDType a, const JDType b) __attribute__((const));
JDType JDAddDays(const JDType a, const double b) __attribute__((const));
JDType JDAddSeconds(const JDType a, const double b) __attribute__((const));
JDType JDAddRationalSeconds(const JDType a, const Rational b)
    __attribute__((const));
JDType JDAddIntegerMultRatSecs(const JDType jd, const long mul,
                               const Rational rat) __attribute__((const));
JDType JDAddRationalMult(const JDType a, Rational mul, const JDType b)
    __attribute__((const));
JDType JDSub(const JDType a, const JDType b) __attribute__((const));
JDType JDSubDays(const JDType a, const double b) __attribute__((const));
JDType JDSubSeconds(const JDType a, const double b) __attribute__((const));
JDType JDSubRationalSeconds(const JDType a, const Rational b)
    __attribute__((const));
JDType JDSubRationalMult(const JDType a, Rational mul, const JDType b)
    __attribute__((const));
JDType JDaxpy(const double a, const JDType x, JDType y) __attribute__((const));
double JDAddToDays(const JDType a, const JDType b) __attribute__((const));
double JDAddToSeconds(const JDType a, const JDType b) __attribute__((const));
double JDSubToDays(const JDType a, const JDType b) __attribute__((const));
double JDSubToSeconds(const JDType a, const JDType b) __attribute__((const));

JDType JDAbs(JDType jd) __attribute__((const));
int ispos_jd(JDType jd) __attribute__((const));
JDType JDNegate(JDType jd) __attribute__((const));

int isequal_jd_systemepoch(const JDType a, const JDType b)
    __attribute__((const));
int isequal_jd(const JDType a, const JDType b) __attribute__((const));
int isless_jd(const JDType a, const JDType b) __attribute__((const));
int islessequal_jd(const JDType a, const JDType b) __attribute__((const));
int isgreater_jd(const JDType a, const JDType b) __attribute__((const));
int isgreaterequal_jd(const JDType a, const JDType b) __attribute__((const));

EpochTT str2epoch(char str[JDEPOCH_STR_LEN]) __attribute__((pure));
TimeSystem str2system(char str[JDSYSTEM_STR_LEN]) __attribute__((pure));
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

/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __TIMEKIT_H__
#define __TIMEKIT_H__

#include "42constants.h"
#include "jdkit.h"
#include <math.h>
#include <stdint.h>
#if defined(_WIN32)
#include <Windows.h>
#elif defined(__APPLE__)
/*      #include <CoreServices/Timer.h> */ /* For Microseconds high-precision
                                              timer */
#endif
#include <time.h>
#ifndef _WIN32
#include <sys/time.h>
#endif

/***************************** CCSDS CONFIGURATION ****************************/
// TODO: 24 bit words for Coarse and Fine time fields; 0 bit field for fine
#define CCSDS_COARSE_TIME_BYTES 4 // 1, 2, or 4 bytes long
#define CCSDS_FINE_TIME_BYTES   2 // 1 or 2 bytes long

#if CCSDS_COARSE_TIME_BYTES == 1
typedef uint8_t ccsdsCoarse;
#define CCSDS_COARSE_MAX (256)
#elif CCSDS_COARSE_TIME_BYTES == 2
typedef uint16_t ccsdsCoarse;
#define CCSDS_COARSE_MAX (65536)
#elif CCSDS_COARSE_TIME_BYTES == 4
typedef uint32_t ccsdsCoarse;
#define CCSDS_COARSE_MAX (4294967296)
#endif

#if CCSDS_FINE_TIME_BYTES == 1
typedef uint8_t ccsdsFine;
#define CCSDS_FINE_MAX (256)
#elif CCSDS_FINE_TIME_BYTES == 2
typedef uint16_t ccsdsFine;
#define CCSDS_FINE_MAX (65536)
#endif

#define CCSDS_STEP_SIZE ((1.0) / (CCSDS_FINE_MAX))
/*************************** END CCSDS CONFIGURATION **************************/

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

#define SEC_PER_DAY (86400.0)

typedef struct CCSDSTime {
   ccsdsCoarse coarse;
   ccsdsFine fine;
} CCSDSTime;

CCSDSTime CCSDSSub(const CCSDSTime a_ccsds_time, const CCSDSTime b_ccsds_time);
CCSDSTime CCSDSAdd(const CCSDSTime a_ccsds_time, const CCSDSTime b_ccsds_time);
CCSDSTime CCSDSAddSeconds(const CCSDSTime time, const double dt);
double ccsds2seconds(const CCSDSTime ccsds);
CCSDSTime seconds2ccsds(const double);
CCSDSTime jdutc2ccsds(const double UTC);
CCSDSTime jdtt2ccsds(const double TT, const double LeapSec);
double ccsds2time(const CCSDSTime ccsds_time);
int isequal_ccsds(const CCSDSTime a_ccsds_time, const CCSDSTime b_ccsds_time);
int isless_ccsds(const CCSDSTime a_ccsds, const CCSDSTime b_ccsds);

typedef struct {
   // TODO: I think remove JD, add a time system property
   TimeSystem system;

   JDType JD;
   // double MJD;
   // double JulDay;
   double tdbTime; // TDB_TIME, J2000_EPOCH
   long Year;
   long Month;
   long Day;
   long doy;
   long Hour;
   long Minute;
   double Second;
} DateType;

// double TDB_JDtoTT(double TDB_JD);
// double TTtoTDB_JD(double SecSinceJ2000);
// double TTtoTDB_Time(double SecSinceJ2000);
// double JDToMJD(double JD);
// double MJDToJD(double MJD);
// double DateToMJD(long Year, long Month, long Day, long Hour, long Minute,
//                  double Second);

CCSDSTime date2ccsds(const DateType date);
DateType ccsds2date(const CCSDSTime ccsds_time);

JDType TimeToJD(double SecSince, TimeSystem system, EpochTT epoch);
double JDToTime(const JDType JD);
double JDToDynTime(const JDType JD);
double DateToTime(const DateType date);
JDType DateToJD(const DateType date, const EpochTT epoch,
                const TimeSystem system);
CCSDSTime TimeToCCSDS(double UTC);
DateType CCSDSToDate(CCSDSTime ccsds_time);
DateType JDToDate(const JDType jd, const TimeSystem system);
DateType TimeToDate(double Time, TimeSystem system, double LSB);
long MD2DOY(long Year, long Month, long Day);
void DOY2MD(long Year, long DayOfYear, long *Month, long *Day);
double JD2GMST(JDType JD);
void GpsTimeToGpsDate(double GpsTime, long *GpsRollover, long *GpsWeek,
                      double *GpsSecond);
double GpsDateToGpsTime(long GpsRollover, long GpsWeek, double GpsSecond);
double usec(void);
void RealSystemTime(DateType *const date, double LSB);
double RealRunTime(double *RealTimeDT, double LSB);
void updateTime(DateType *Time, const double dSeconds);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __TIMEKIT_H__ */

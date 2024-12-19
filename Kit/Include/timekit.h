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
#define CCSDS_FINE_TIME_BYTES   2 // 1, or 2 bytes long

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

struct DateType {
   double JulDay;
   long Year;
   long Month;
   long Day;
   long doy;
   long Hour;
   long Minute;
   double Second;
};

#ifdef __cplusplus
extern "C" {
#endif

double TimeToJD(double Time);
double JDToTime(double JD);
double DateToTime(long Year, long Month, long Day, long Hour, long Minute,
                  double Second);
double DateToJD(long Year, long Month, long Day, long Hour, long Minute,
                double Second);
void DateToCCSDS(struct DateType date, ccsdsCoarse *ccsdsSeconds,
                 ccsdsFine *ccsdsSubSeconds);
void TimeToCCSDS(double UTC, ccsdsCoarse *ccsdsSeconds,
                 ccsdsFine *ccsdsSubSeconds);
void CCSDSToDate(ccsdsCoarse ccsdsSeconds, ccsdsFine ccsdsSubSeconds,
                 struct DateType *date);
double CCSDSAdd(const ccsdsCoarse a_coarse, const ccsdsFine a_fine,
                const ccsdsCoarse b_coarse, const ccsdsFine b_fine);
double CCSDSSub(const ccsdsCoarse a_coarse, const ccsdsFine a_fine,
                const ccsdsCoarse b_coarse, const ccsdsFine b_fine);
void JDToDate(double JD, long *Year, long *Month, long *Day, long *Hour,
              long *Minute, double *Second);
void TimeToDate(double Time, long *Year, long *Month, long *Day, long *Hour,
                long *Minute, double *Second, double LSB);
long MD2DOY(long Year, long Month, long Day);
void DOY2MD(long Year, long DayOfYear, long *Month, long *Day);
double JD2GMST(double JD);
void GpsTimeToGpsDate(double GpsTime, long *GpsRollover, long *GpsWeek,
                      double *GpsSecond);
double GpsDateToGpsTime(long GpsRollover, long GpsWeek, double GpsSecond);
double usec(void);
void RealSystemTime(long *Year, long *DOY, long *Month, long *Day, long *Hour,
                    long *Minute, double *Second, double LSB);
double RealRunTime(double *RealTimeDT, double LSB);
void updateTime(struct DateType *Time, const double dSeconds);

#ifdef __cplusplus
}
#endif

#endif /* __TIMEKIT_H__ */

/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "timekit.h"

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

/**********************************************************************/
/* Convert CCSDS Seconds and subseconds */
/* Convert UTC Date to CCSDS Seconds and Subseconds with epoch        */
/* midnight, Jan 1st, 1958                                            */
double ccsds2time(const CCSDSTime ccsds_time)
{
   const DateType date = ccsds2date(ccsds_time, TT_TIME);
   return Date2Time(date);
}
/**********************************************************************/
/* Convert CCSDS Seconds and Subseconds to TT Date with epoch        */
/* midnight, Jan 1st, 1958                                            */
DateType ccsds2date(const CCSDSTime ccsds_time, TimeSystem system)
{
   JDType jd = JDFromSeconds(ccsds_time.coarse, TAI_TIME, CCSDS_EPOCH);
   jd        = JDAddSeconds(jd, ccsds_time.fine * CCSDS_STEP_SIZE);

   DateType date = JDToDate(jd, system);
   return date;
}
/**********************************************************************/
CCSDSTime CCSDSSub(const CCSDSTime a_ccsds, const CCSDSTime b_ccsds)
{
   CCSDSTime dccsds;

   dccsds.coarse = a_ccsds.coarse - b_ccsds.coarse;
   dccsds.fine   = a_ccsds.fine - b_ccsds.fine;

   if (a_ccsds.fine < b_ccsds.fine)
      dccsds.coarse--;

   return dccsds;
}
/**********************************************************************/
CCSDSTime CCSDSAdd(const CCSDSTime a_ccsds, const CCSDSTime b_ccsds)
{
   CCSDSTime dccsds;

   dccsds.coarse = a_ccsds.coarse + b_ccsds.coarse;
   dccsds.fine   = a_ccsds.fine + b_ccsds.fine;

   if (dccsds.fine < a_ccsds.fine)
      dccsds.coarse++;

   return dccsds;
}
/**********************************************************************/
CCSDSTime CCSDSAddSeconds(const CCSDSTime time, const double dt)
{
   if (dt >= 0.0) {
      const CCSDSTime sec_ccsds = seconds2ccsds(dt);
      return CCSDSAdd(time, sec_ccsds);
   }
   else {
      const CCSDSTime sec_ccsds = seconds2ccsds(-dt);
      return CCSDSSub(time, sec_ccsds);
   }
}
/**********************************************************************/
double ccsds2seconds(const CCSDSTime ccsds)
{
   return (double)ccsds.coarse + (double)ccsds.fine * CCSDS_STEP_SIZE;
}
/**********************************************************************/
CCSDSTime seconds2ccsds(const double sec)
{
   double integral = 0;
   CCSDSTime ccsds = {0};
   ccsds.fine      = modf(sec, &integral) * CCSDS_FINE_MAX + 0.5;
   ccsds.coarse    = integral;
   return ccsds;
}
/**********************************************************************/
int isequal_ccsds(const CCSDSTime a_ccsds, const CCSDSTime b_ccsds)
{
   return (a_ccsds.fine == b_ccsds.fine) && (a_ccsds.coarse == b_ccsds.coarse);
}
/**********************************************************************/
int isless_ccsds(const CCSDSTime a_ccsds, const CCSDSTime b_ccsds)
{
   return (a_ccsds.coarse < b_ccsds.coarse) ||
          ((a_ccsds.coarse == b_ccsds.coarse) && a_ccsds.fine < b_ccsds.fine);
}
/**********************************************************************/
DateType DateTypeInit(const TimeSystem system, const long Year,
                      const long Month, const long Day, const long Hour,
                      const long Minute, const Rational Second)
{
   DateType date = {.Year   = Year,
                    .Month  = Month,
                    .Day    = Day,
                    .Hour   = Hour,
                    .Minute = Minute,
                    .Second = Second,
                    .system = system};
   if (!date.Second.den)
      date.Second.den = 1;
   date.doy = MD2DOY(date.Year, date.Month, date.Day);
   return date;
}
/**********************************************************************/
/*   Convert Gregorian Date to a Julian Date format. Uses the same    */
/*   time system as 'date' and returns desired 'epoch'                */
/*     Adapted From: Vallado, Fundamentals of Astrodyanmics and       */
/*     Applications, 4th Ed, Microcosm Press, El Segundo CA, 2013,    */
/*     p. 183                                                         */
JDType Date2JD(const DateType date, const EpochTT epoch)
{
   const long Y = date.Year;
   const long M = date.Month;
   const long D = date.Day;
   const long H = date.Hour;
   const long m = date.Minute;
   Rational s   = date.Second;

   const long c = (M + 9.0) / 12.0;
   const long b = (275.0 * M / 9.0);
   const long a = (7.0 * (Y + c)) / 4.0;

   const long day = 367 * Y - a + b + D;
   JDType jd      = JDFromDays(day, date.system, GD_CONV_EPOCH);
   ChangeEpoch(epoch, &jd);
   s.whole += 60 * (m + 60 * H);
   jd       = JDAddRationalSeconds(jd, s);

   return jd;
}
/**********************************************************************/
static int _is_leap_yr(long yr)
{
   return (yr % 4 == 0 && yr % 100 != 0) || yr % 400 == 0;
}
/**********************************************************************/
/*  Convert Year, Month, Day, Hour, Minute and Second to              */
/*  "Time", i.e. seconds elapsed since J2000 epoch.                   */
/*  This function is agnostic to the TT-to-UTC offset.  You get out   */
/*  what you put in.                                                  */
double Date2Time(const DateType date)
{
   JDType jd = Date2JD(date, J2000_EPOCH);
   return JDToTime(jd);
}
/**********************************************************************/
/*  Same as above, but also converts the output time system           */
double Date2TimeSystem(const DateType date, const TimeSystem system)
{
   JDType jd = Date2JD(date, J2000_EPOCH);
   ChangeSystem(system, &jd);
   return JDToTime(jd);
}
/**********************************************************************/
/*  Year, Month, Day assumed in Gregorian calendar. (Not true < 1582) */
/*  Ref. Jean Meeus, 'Astronomical Algorithms', QB51.3.E43M42, 1991.  */

double DateToTime(const DateType date)
{
   long A, B;
   double Days;
   long Year  = date.Year;
   long Month = date.Month;

   if (Month < 3) {
      Year--;
      Month += 12;
   }

   A = Year / 100;
   B = 2 - A + A / 4;

   /* Days since J2000 Epoch (01 Jan 2000 12:00:00.0) */
   Days = floor(365.25 * (Year - 2000)) + floor(30.6001 * (Month + 1)) +
          date.Day + B - 50.5;

   /* Add fractional day */
   return (SEC_PER_DAY * Days + 3600.0 * ((double)date.Hour) +
           60.0 * ((double)date.Minute) + rational2double(date.Second));
}
/**********************************************************************/
/*  Convert Year, Month, Day, Hour, Minute and Second to Julian Day   */
/*  Valid for any positive JD.                                        */
/*  Year, Month, Day assumed in Gregorian calendar. (Not true < 1582) */
/*  Ref. Jean Meeus, 'Astronomical Algorithms', QB51.3.E43M42, 1991.  */
/*  This function is agnostic to the TT-to-UTC offset.  You get out   */
/*  what you put in.                                                  */
JDType DateToJD(const DateType date, const TimeSystem system,
                const EpochTT epoch)
{
   long A, B;
   long Year  = date.Year;
   long Month = date.Month;

   if (date.Month < 3) {
      Year--;
      Month += 12;
   }

   A = Year / 100;
   B = 2 - A + A / 4;

   const double day = floor(365.25 * (Year + 4716)) +
                      floor(30.6001 * (Month + 1)) + date.Day + B - 1524.5;

   JDType jd   = JDFromDays(day, date.system, ZERO_EPOCH);
   Rational s  = date.Second;
   s.whole    += 60 * (date.Minute + 60 * date.Hour);
   jd          = JDAddRationalSeconds(jd, s);

   ChangeSystemEpoch(system, epoch, &jd);
   return (jd);
}
/**********************************************************************/
/* Convert UTC Date to CCSDS Seconds and Subseconds with epoch        */
/* midnight, Jan 1st, 1958                                            */
CCSDSTime date2ccsds(const DateType date)
{
   // TODO: SOMETHING WRONG IN HERE
   CCSDSTime ccsds_time;

   JDType jd = Date2JD(date, CCSDS_EPOCH);
   ChangeSystem(TAI_TIME, &jd);
   ccsds_time.coarse = JDToTime(jd);
   jd.seconds.whole  = 0;
   ccsds_time.fine   = (rational2double(jd.seconds) * CCSDS_FINE_MAX) + 0.5;
   return ccsds_time;
}
/**********************************************************************/
/* Convert UTC Date to CCSDS Seconds and Subseconds with epoch        */
/* midnight, Jan 1st, 1958                                            */
CCSDSTime TimeToCCSDS(double UTC)
{
   DateType date = TimeToDate(UTC, UTC_TIME);
   return date2ccsds(date);
}
/**********************************************************************/
/*   Convert Julian Day to Year, Month, Day, Hour, Minute, and Second */
/*     Adapted From: Vallado, Fundamentals of Astrodyanmics and       */
/*     Applications, 4th Ed, Microcosm Press, El Segundo CA, 2013,    */
/*     p. 202                                                         */
DateType JDToDate(const JDType jd, const TimeSystem system)
{
   double lp_yrs, days, T_1900, tmp;
   long l_month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

   DateType date = {0};
   date.system   = system;

   JDType jd_sys_1900 = jd;
   ChangeSystemEpoch(system, J1900_EPOCH, &jd_sys_1900);
   const double jd_1900_days = JDToDays(jd_sys_1900);

   T_1900    = jd_1900_days / 365.25;
   date.Year = 1900 + trunc(T_1900);
   if (date.Year >= 2100) {
      fprintf(stderr, "JDToDate is not valid for years greater than or equal "
                      "2100 due to unhandled leap years. Exiting...\n");
      exit(EXIT_FAILURE);
      // TODO: change the lp_yrs calculation to handle more leap years
   }
   lp_yrs = trunc((date.Year - 1900 - 1) * 0.25);
   days   = jd_1900_days - ((date.Year - 1900) * 365 + lp_yrs);
   if (days < 1.0) {
      date.Year--;
      lp_yrs = trunc((date.Year - 1900 - 1) * 0.25);
      days   = jd_1900_days - ((date.Year - 1900) * 365 + lp_yrs);
   }
   // modification from ref to cover leap years beyond the year % 4==0 rule
   if (_is_leap_yr(date.Year))
      l_month[1] = 29;
   date.doy = trunc(days);

   int i    = 0;
   long sum = 0;
   for (i = 0; i < 11; i++) {
      if (sum + l_month[i + 1] > date.doy)
         break;
      sum += l_month[i];
   }
   date.Month     = i + 1;
   date.Day       = date.doy - sum;
   tmp            = (days - date.doy) * 24;
   date.Hour      = trunc(tmp);
   date.Minute    = trunc((tmp - date.Hour) * 60);
   Rational hours = double2rational(tmp - date.Hour - (double)date.Minute / 60);
   date.Second    = IntegerRationalMult(3600, hours);
   return date;
}
/**********************************************************************/
/*   Convert Time to Year, Month, Day, Hour, Minute, and Second       */
DateType TimeToDate(double Time, TimeSystem system)
{
   JDType jd     = JDFromSeconds(Time, system, J2000_EPOCH);
   DateType date = JDToDate(jd, system);
   return date;
}
/**********************************************************************/
/*  Find Day of Year, given Month, Day                                */
/*  Ref. Jean Meeus, 'Astronomical Algorithms', QB51.3.E43M42, 1991.  */
/*   This function is agnostic to the TT-to-UTC offset.  You get out  */
/*   what you put in.                                                 */
long MD2DOY(long Year, long Month, long Day)
{
   long K;

   if (Year % 4 == 0) {
      K = 1;
   }
   else {
      K = 2;
   }

   return (275 * Month / 9 - K * ((Month + 9) / 12) + Day - 30);
}
/**********************************************************************/
/*  Find Month, Day, given Day of Year                                */
/*  Ref. Jean Meeus, 'Astronomical Algorithms', QB51.3.E43M42, 1991.  */
/*   This function is agnostic to the TT-to-UTC offset.  You get out  */
/*   what you put in.                                                 */
void DOY2MD(long Year, long DayOfYear, long *Month, long *Day)
{
   long K;

   if (_is_leap_yr(Year)) {
      K = 1;
   }
   else {
      K = 2;
   }

   if (DayOfYear < 32) {
      *Month = 1;
   }
   else {
      *Month = (long)(9.0 * (K + DayOfYear) / 275.0 + 0.98);
   }

   *Day = DayOfYear - 275 * (*Month) / 9 + K * (((*Month) + 9) / 12) + 30;
}
/**********************************************************************/
/*  Find Greenwich Mean Sidereal Time (GMST)                          */
/*  Ref. Jean Meeus, 'Astronomical Algorithms', QB51.3.E43M42, 1991.  */
/*  GMST is output in units of days.                                  */
double JD2GMST(JDType jd)
{
   double T, JD0, GMST0, GMST;

   ChangeSystemEpoch(UTC_TIME, J2000_EPOCH, &jd);
   const double JD = JDToDays(jd);

   JD0 = floor(JD) + 0.5;

   T = JD0 / 36525.0;

   /* .. GMST at UT=0h, in deg */
   GMST0 =
       100.46061837 + T * (36000.770053608 + T * (3.87933E-4 - T / 3.871E7));

   /* .. Convert to days */
   GMST0 /= 360.0;

   GMST  = GMST0 + 1.00273790935 * (JD - JD0);
   GMST -= (long)GMST;
   return (GMST);
}
/**********************************************************************/
/* GPS Epoch is 6 Jan 1980 00:00:00.0 UTC                             */
/* which is 6 Jan 1980 00:00:19.0 TAI                                 */
/* J2000 is 1 Jan 2000 12:00:00.0 TT                                  */
/* which is 1 Jan 2000 11:59:27.816 TAI                               */
/* so J2000-GPS epoch is 7300.5 days minus (19+32.184) sec            */
void GpsTimeToGpsDate(double GpsTime, long *GpsRollover, long *GpsWeek,
                      double *GpsSecond)
{
   double DaysSinceEpoch, DaysSinceRollover, DaysSinceWeek;

   DaysSinceEpoch    = 7300.5 + GpsTime / SEC_PER_DAY;
   *GpsRollover      = (long)(DaysSinceEpoch / 7168.0);
   DaysSinceRollover = DaysSinceEpoch - 7168.0 * ((double)*GpsRollover);
   *GpsWeek          = (long)(DaysSinceRollover / 7.0);
   DaysSinceWeek     = DaysSinceRollover - 7.0 * ((double)*GpsWeek);
   *GpsSecond        = DaysSinceWeek * SEC_PER_DAY;
}
/**********************************************************************/
/* GPS Epoch is 6 Jan 1980 00:00:00.0 UTC                             */
/* which is 6 Jan 1980 00:00:19.0 TAI                                 */
/* J2000 is 1 Jan 2000 12:00:00.0 TT                                  */
/* which is 1 Jan 2000 11:59:27.816 TAI                               */
/* so J2000-GPS epoch is 7300.5 days minus (19+32.184) sec            */
double GpsDateToGpsTime(long GpsRollover, long GpsWeek, double GpsSecond)
{
   return (((GpsRollover * 1024.0 + GpsWeek) * 7.0 - 7300.5) * SEC_PER_DAY +
           GpsSecond);
}

/**********************************************************************/
/* This function returns the number of microseconds since the Unix    */
/* epoch, 00:00:00.0 Jan 1 1970.  Typically used as a tick/tock       */
/* duration measurement.                                              */
double usec(void)
{
#if defined(_WIN32)

   static LARGE_INTEGER SysFreq;
   LARGE_INTEGER SysCtr;
   static long First = 1;

   if (First) {
      First = 0;
      QueryPerformanceFrequency(&SysFreq);
   }

   QueryPerformanceCounter(&SysCtr);
   return ((1.0E6 * ((double)SysCtr.QuadPart)) / ((double)SysFreq.QuadPart));

#elif (defined(__APPLE__) || defined(__linux__))
   struct timeval now;

   gettimeofday(&now, NULL);
   return (1.0E6 * now.tv_sec + now.tv_usec);
#else
#error "This OS not supported by usec function"
   fprintf(stderr, "This OS not supported by usec function.  Bailing out.\n");
   exit(EXIT_FAILURE);
   return (0.0);
#endif
}
/**********************************************************************/
/* Get time from operating system, and convert to compatible format.  */
/* Returns the UTC date.                                              */
DateType RealSystemTime()
{
#if (defined(__APPLE__) || defined(__linux__))
   struct timeval now;
   double UnixTime, Time;

   /* Unix Time is since 00:00:00.0 Jan 1 1970 */
   gettimeofday(&now, NULL);
   UnixTime = now.tv_sec + 1.0E-6 * now.tv_usec;

   /* Time is since J2000 */
   Time = UnixTime - 946728000.0;

   return TimeToDate(Time, UTC_TIME);
#endif
}
/**********************************************************************/
double RealRunTime(double *RealTimeDT, double LSB)
{
   static double RunTime = 0.0;
   static long First     = 1;

#if defined(_WIN32)

   static LARGE_INTEGER SysFreq;
   static LARGE_INTEGER OldSysCtr;
   LARGE_INTEGER SysCtr;

   if (First) {
      First = 0;
      QueryPerformanceFrequency(&SysFreq);
      QueryPerformanceCounter(&OldSysCtr);
   }

   QueryPerformanceCounter(&SysCtr);
   *RealTimeDT = ((double)(SysCtr.QuadPart - OldSysCtr.QuadPart)) /
                 ((double)SysFreq.QuadPart);
   OldSysCtr   = SysCtr;

#elif (defined(__APPLE__) || defined(__linux__))

   static double OldSysTime;
   double SysTime;
   DateType date = {0};

   if (First) {
      First      = 0;
      date       = RealSystemTime();
      OldSysTime = Date2Time(date);
   }

   date        = RealSystemTime();
   SysTime     = Date2Time(date);
   *RealTimeDT = SysTime - OldSysTime;
   OldSysTime  = SysTime;
#else
#error "Unknown operating system in RealRunTime.  Fix that!"
   fprintf(stderr, "Unknown operating system in RealRunTime.  Bailing out.\n");
   exit(EXIT_FAILURE);
#endif

   if (*RealTimeDT < 0.0)
      *RealTimeDT = 0.001;
   /*if (*RealTimeDT > 1.0) *RealTimeDT = 1.0;*/
   RunTime += *RealTimeDT;

   return (RunTime);
}

void updateTime(DateType *Time, const double dSeconds)
{
   JDType jd = Date2JD(*Time, GMAT_MJD_EPOCH);

   if (fabs(dSeconds) > 0.0) {
      Rational rat_dseconds = double2rational(dSeconds);

      Time->Second  = RationalAdd(Time->Second, rat_dseconds);
      long quotient = RationalIntMod(&Time->Second, 60);
      if (Time->Second.whole < 0) {
         Time->Second.whole += 60;
         quotient--;
      }

      if (quotient != 0.0) {
         Time->Minute += quotient;
         quotient      = Time->Minute / 60;
         Time->Minute %= 60;
         if (Time->Minute < 0) {
            Time->Minute += 60;
            quotient--;
         }

         if (quotient != 0) {
            Time->Hour += quotient;
            quotient    = Time->Hour / 24;
            Time->Hour %= 24;
            if (Time->Hour < 0) {
               Time->Hour += 24;
               quotient--;
            }

            if (quotient != 0) {
               jd                = JDAddDays(jd, quotient);
               DateType date_day = JDToDate(jd, Time->system);
               Time->Year        = date_day.Year;
               Time->Month       = date_day.Month;
               Time->Day         = date_day.Day;
               Time->doy         = MD2DOY(Time->Year, Time->Month, Time->Day);
               /* ALTERNATIVELY
               long l_month[12] = {31, 28, 31, 30, 31, 30,
                                   31, 31, 30, 31, 30, 31};

               Time->doy = Time->Day + quotient;
               int mnth  = Time->Month;

               int l_yr = 365;

               if (_is_leap_yr(Time->Year)) {
                  l_yr       = 366;
                  l_month[1] = 29;
               }

               for (int i = 0; i < mnth - 1; i++) {
                  Time->doy += l_month[i];
               }

               long yr = Time->Year;
               while (Time->doy < 0) {
                  Time->Year--;
                  l_yr = 365;
                  if (_is_leap_yr(Time->Year)) {
                     l_yr = 366;
                  }
                  Time->doy += l_yr;
               }
               while (Time->doy > l_yr) {
                  Time->Year++;
                  l_yr = 365;
                  if (_is_leap_yr(Time->Year)) {
                     l_yr = 366;
                  }
                  Time->doy -= l_yr;
               }
               DOY2MD(Time->Year, Time->doy, &Time->Month, &Time->Day);
               */
            }
         }
      }
      // Time->JulDay = Date2JD(Time->Year, Time->Month, Time->Day, Time->Hour,
      //                         Time->Minute, Time->Second);
   }
}

/* #ifdef __cplusplus
** }
** #endif
*/

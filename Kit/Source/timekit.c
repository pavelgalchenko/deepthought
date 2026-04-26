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

double TTtoTDB_JD(double SecSinceJ2000)
{
   double T_TT, m_E, deltaTDB, JD_TDB;
   static double TDB_COEFF1             = 0.00165;
   static double TDB_COEFF2             = 0.00001385;
   static double M_E_OFFSET             = 357.5277233;
   static double M_E_COEFF1             = 35999.05034;
   static double SEC_PER_JULIAN_CENTURY = 3155760000.00;

   T_TT = SecSinceJ2000 / SEC_PER_JULIAN_CENTURY;

   m_E      = fmod((M_E_OFFSET + (M_E_COEFF1 * T_TT)), 360.0) * D2R;
   deltaTDB = (TDB_COEFF1 * sin(m_E) + TDB_COEFF2 * sin(2.0 * m_E));
   JD_TDB   = (SecSinceJ2000 + deltaTDB) / 86400.0 + 2451545.0;

   return (JD_TDB);
}
double TTtoTDB_Time(double SecSinceJ2000)
{
   double T_TT, m_E, deltaTDB, time_TDB;
   static double TDB_COEFF1             = 0.001658;
   static double TDB_COEFF2             = 0.00001385;
   static double M_E_OFFSET             = 357.5277233;
   static double M_E_COEFF1             = 35999.05034;
   static double SEC_PER_JULIAN_CENTURY = 3155760000.00;

   T_TT = SecSinceJ2000 / SEC_PER_JULIAN_CENTURY;

   m_E      = fmod((M_E_OFFSET + (M_E_COEFF1 * T_TT)), 360.0) * D2R;
   deltaTDB = (TDB_COEFF1 * sin(m_E) + TDB_COEFF2 * sin(2.0 * m_E));
   time_TDB = SecSinceJ2000 + deltaTDB;

   return (time_TDB);
}
/**********************************************************************/
/*    ignores date.JD                                                 */
/*    GMAT Math Spec 2026, page 12                                    */
JDType Date2JD(const DateType date, const Epoch epoch)
{
   JDType jd = {.day = 0, .epoch = epoch, .system = GD_CONV_EPOCH};

   const double Y = date.Year;
   const double M = date.Month;
   const double D = date.Day;
   const double H = date.Hour;
   const double m = date.Minute;
   const double s = date.Second;

   const long a = 367 * Y;
   const long b = 7.0 * (Y + trunc((M + 9.0) / 12.0)) / 4.0;
   const long c = 275.0 * M / 9.0;

   jd.day = a - b + c + D;
   ChangeSystem(GMAT_MJD_EPOCH, &jd);
   jd.day += ((s / 60.0 + m) / 60.0 + H) / 24.0;

   return jd;
}
/**********************************************************************/
/*  Converts seconds since 'epoch' in 'system' to a JDType format     */
JDType TimeToJD(double SecSince, TimeSystem system, Epoch epoch)
{
   JDType jd = {
       .day = SecSince / SEC_PER_DAY, .system = system, .epoch = epoch};
   return jd;
}
/**********************************************************************/
/* Time is elapsed seconds since J2000 epoch                          */
/*  This function is agnostic to the TT-to-UTC offset.  You get out   */
/*  what you put in.                                                  */
double JDToTime(JDType jd)
{
   ChangeEpoch(J2000_EPOCH, &jd);
   return (jd.day * 86400.0);
}
/**********************************************************************/
/* Time is elapsed seconds since J2000 epoch in TT time               */
double JDToDynTime(JDType jd)
{
   ChangeSystem(TT_TIME, &jd);
   return JDToTime(jd);
}
/**********************************************************************/
/*  Convert Year, Month, Day, Hour, Minute and Second to              */
/*  "Time", i.e. seconds elapsed since J2000 epoch.                   */
/*  Year, Month, Day assumed in Gregorian calendar. (Not true < 1582) */
/*  Ref. Jean Meeus, 'Astronomical Algorithms', QB51.3.E43M42, 1991.  */
/*  This function is agnostic to the TT-to-UTC offset.  You get out   */
/*  what you put in.                                                  */

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
           60.0 * ((double)date.Minute) + date.Second);
}
/**********************************************************************/
/*  Convert Year, Month, Day, Hour, Minute and Second to Julian Day   */
/*  Valid for any positive JD.                                        */
/*  Year, Month, Day assumed in Gregorian calendar. (Not true < 1582) */
/*  Ref. Jean Meeus, 'Astronomical Algorithms', QB51.3.E43M42, 1991.  */
/*  This function is agnostic to the TT-to-UTC offset.  You get out   */
/*  what you put in.                                                  */
JDType DateToJD(const DateType date, const TimeSystem old_system,
                const Epoch epoch, const TimeSystem system)
{
   long A, B;
   long Year  = date.Year;
   long Month = date.Month;

   JDType jd = {.day = 0, .epoch = ZERO_EPOCH, .system = old_system};

   if (date.Month < 3) {
      Year--;
      Month += 12;
   }

   A = Year / 100;
   B = 2 - A + A / 4;

   jd.day = floor(365.25 * (Year + 4716)) + floor(30.6001 * (Month + 1)) +
            date.Day + B - 1524.5;

   jd.day += ((double)date.Hour) / 24.0 + ((double)date.Minute) / 1440.0 +
             date.Second / SEC_PER_DAY;

   ChangeSystemEpoch(system, epoch, &jd);

   return (jd);
}
/**********************************************************************/
/*  Convert Y, M, D, H, m and s to Modified Julian Day (MJD)          */
/*  Year, Month, Day assumed in Gregorian calendar. (Not true < 1900) */
/*  Ref. GMAT Math Spec.                                              */
double DateToMJD(long Year, long Month, long Day, long Hour, long Minute,
                 double Second)
{
   int A, B, C;
   double JD, MJD;
   double partofday;

   C = (int)((Month + 9) / 12);
   B = (int)(275 * Month / 9);
   A = (int)(7 * (Year + C) / 4);

   partofday = (((Second / 60) + Minute) / 60 + Hour) / 24;

   JD = 367 * Year - A + B + Day + 1721013.5;

   MJD = (JD - 2430000.0) + partofday;

   return (MJD);
}
double JDToMJD(double JD)
{
   return (JD - 2430000.0);
}
double MJDToJD(double MJD)
{
   return (MJD + 2430000.0);
}
/**********************************************************************/
/* Convert UTC Date to CCSDS Seconds and Subseconds with epoch        */
/* midnight, Jan 1st, 1958                                            */
void DateToCCSDS(DateType date, ccsdsCoarse *ccsdsSeconds,
                 ccsdsFine *ccsdsSubSeconds)
{
   DateType date_day = {0};
   date_day.Year     = date.Year;
   date_day.Month    = date.Month;
   date_day.Day      = date.Day;

   JDType jd = DateToJD(date_day, UTC_TIME, CCSDS_EPOCH, TT_TIME);

   double jd_i   = 0;
   double jd_dbl = modf(jd.day, &jd_i);

   double integral;
   *ccsdsSubSeconds = (modf(date.Second, &integral) * CCSDS_FINE_MAX) + 0.5;
   *ccsdsSeconds =
       jd_i * SEC_PER_DAY + date.Hour * 3600 + date.Minute * 60 + integral;
   // TODO: handle jd_dbl
}
/**********************************************************************/
/* Convert UTC Date to CCSDS Seconds and Subseconds with epoch        */
/* midnight, Jan 1st, 1958                                            */
void TimeToCCSDS(double UTC, ccsdsCoarse *ccsdsSeconds,
                 ccsdsFine *ccsdsSubSeconds)
{
   const double LSB = CCSDS_STEP_SIZE;
   DateType date    = TimeToDate(UTC, LSB);
   DateToCCSDS(date, ccsdsSeconds, ccsdsSubSeconds);
}
/**********************************************************************/
/* Convert CCSDS Seconds and Subseconds to UTC Date with epoch        */
/* midnight, Jan 1st, 1958                                            */
void CCSDSToDate(ccsdsCoarse ccsdsSeconds, ccsdsFine ccsdsSubSeconds,
                 DateType *date)
{
   const double ccsdsStep = CCSDS_STEP_SIZE;
   const double subDay    = fmod((double)ccsdsSeconds, 86400.0);
   const long days        = ccsdsSeconds / 86400;
   JDType jd = {.day = days, .epoch = CCSDS_EPOCH, .system = TT_TIME};

   *date = JDToDate(jd);
   updateTime(date, subDay + (double)ccsdsSubSeconds * ccsdsStep);
}
/**********************************************************************/
double CCSDSAdd(const ccsdsCoarse a_coarse, const ccsdsFine a_fine,
                const ccsdsCoarse b_coarse, const ccsdsFine b_fine)
{
   const double ccsdsStepSize = CCSDS_STEP_SIZE;

   double out =
       (double)(a_coarse + b_coarse) + (a_fine + b_fine) * ccsdsStepSize;

   ccsdsFine test;
   if (__builtin_add_overflow(a_fine, b_fine, &test))
      out += 1.0;

   return out;
}
/**********************************************************************/
double CCSDSSub(const ccsdsCoarse a_coarse, const ccsdsFine a_fine,
                const ccsdsCoarse b_coarse, const ccsdsFine b_fine)
{
   const double ccsdsStepSize = CCSDS_STEP_SIZE;

   double out =
       (double)(a_coarse - b_coarse) + (a_fine - b_fine) * ccsdsStepSize;

   ccsdsFine test;
   if (__builtin_sub_overflow(a_fine, b_fine, &test))
      out -= 1.0;

   return out;
}
/**********************************************************************/
/*   Convert Julian Day to Year, Month, Day, Hour, Minute, and Second */
/*   Ref. Jean Meeus, 'Astronomical Algorithms', QB51.3.E43M42, 1991. */
/*  This function is agnostic to the TT-to-UTC offset.  You get out   */
/*  what you put in.                                                  */
DateType JDToDate(const JDType jd)
{
   double Z, F, A, B, C, D, E, alpha;
   double FD;

   DateType date = {0};

   JDType jd_tt_z = jd;
   ChangeSystem(TT_TIME, &jd_tt_z);
   ChangeEpoch(ZERO_EPOCH, &jd_tt_z);

   Z = floor(jd_tt_z.day + 0.5);
   F = (jd_tt_z.day + 0.5) - Z;

   if (Z < 2299161.0) {
      A = Z;
   }
   else {
      alpha = floor((Z - 1867216.25) / 36524.25);
      A     = Z + 1.0 + alpha - floor(alpha / 4.0);
   }

   B = A + 1524.0;
   C = floor((B - 122.1) / 365.25);
   D = floor(365.25 * C);
   E = floor((B - D) / 30.6001);

   FD       = B - D - floor(30.6001 * E) + F;
   date.Day = (long)FD;

   if (E < 14.0) {
      date.Month = (long)(E - 1.0);
      date.Year  = (long)(C - 4716.0);
   }
   else {
      date.Month = (long)(E - 13.0);
      date.Year  = (long)(C - 4715.0);
   }

   FD          = FD - floor(FD);
   FD          = FD * 24.0;
   date.Hour   = (long)FD;
   FD          = FD - floor(FD);
   FD          = FD * 60.0;
   date.Minute = (long)FD;
   FD          = FD - floor(FD);
   date.Second = FD * 60.0;
   return date;
}
/**********************************************************************/
/*   Convert Time to Year, Month, Day, Hour, Minute, and Second       */
/*   Time is seconds since J2000 epoch (01 Jan 2000 12:00:00.0)       */
/*   Outputs are rounded to LSB to avoid loss of precision            */
/*   Ref. Jean Meeus, 'Astronomical Algorithms', QB51.3.E43M42, 1991. */
/*   This function is agnostic to the TT-to-UTC offset.  You get out  */
/*   what you put in.                                                 */
DateType TimeToDate(double Time, double LSB)
{
   double Z, F, A, B, C, D, E, alpha;
   double FD, JD;

   DateType date = {0};

   JD = Time / SEC_PER_DAY + EpochValueTT(J2000_EPOCH);

   Z = floor(JD + 0.5);
   F = (JD + 0.5) - Z;

   if (Z < 2299161.0) {
      A = Z;
   }
   else {
      alpha = floor((Z - 1867216.25) / 36524.25);
      A     = Z + 1.0 + alpha - floor(alpha / 4.0);
   }

   B = A + 1524.0;
   C = floor((B - 122.1) / 365.25);
   D = floor(365.25 * C);
   E = floor((B - D) / 30.6001);

   FD       = B - D - floor(30.6001 * E) + F;
   date.Day = (long)FD;

   if (E < 14.0) {
      date.Month = (long)(E - 1.0);
      date.Year  = (long)(C - 4716.0);
   }
   else {
      date.Month = (long)(E - 13.0);
      date.Year  = (long)(C - 4715.0);
   }

   FD = Time - 43200.0 + 0.5 * LSB;
   FD = FD - ((long)(FD / 86400.0)) * 86400.0;
   if (FD < 0.0)
      FD += 86400.0;

   date.Hour = (long)(FD / 3600.0);

   FD -= 3600.0 * (date.Hour);

   date.Minute = (long)(FD / 60.0);

   date.Second = FD - 60.0 * (date.Minute);

   /* Clean up roundoff */
   date.Second = ((long)(date.Second / LSB)) * LSB;
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

   if (Year % 4 == 0) {
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

   ChangeSystemEpoch(UTC_TIME, ZERO_EPOCH, &jd);
   const double JD = jd.day;

   JD0 = floor(JD) + 0.5;

   T = (JD0 - 2451545.0) / 36525.0;

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

   DaysSinceEpoch    = 7300.5 + GpsTime / 86400.0;
   *GpsRollover      = (long)(DaysSinceEpoch / 7168.0);
   DaysSinceRollover = DaysSinceEpoch - 7168.0 * ((double)*GpsRollover);
   *GpsWeek          = (long)(DaysSinceRollover / 7.0);
   DaysSinceWeek     = DaysSinceRollover - 7.0 * ((double)*GpsWeek);
   *GpsSecond        = DaysSinceWeek * 86400.0;
}
/**********************************************************************/
/* GPS Epoch is 6 Jan 1980 00:00:00.0 UTC                             */
/* which is 6 Jan 1980 00:00:19.0 TAI                                 */
/* J2000 is 1 Jan 2000 12:00:00.0 TT                                  */
/* which is 1 Jan 2000 11:59:27.816 TAI                               */
/* so J2000-GPS epoch is 7300.5 days minus (19+32.184) sec            */
double GpsDateToGpsTime(long GpsRollover, long GpsWeek, double GpsSecond)
{
   return (((GpsRollover * 1024.0 + GpsWeek) * 7.0 - 7300.5) * 86400.0 +
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
/* TODO:  Is this date returned in TT or UTC?                         */
void RealSystemTime(DateType *const date, double DT)
{
#if (defined(__APPLE__) || defined(__linux__))
   struct timeval now;
   double UnixTime, Time;

   /* Unix Time is since 00:00:00.0 Jan 1 1970 */
   gettimeofday(&now, NULL);
   UnixTime = now.tv_sec + 1.0E-6 * now.tv_usec;

   /* Time is since J2000 */
   Time = UnixTime - 946728000.0;

   *date     = TimeToDate(Time, DT);
   date->doy = MD2DOY(date->Year, date->Month, date->Day);
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
      First = 0;
      RealSystemTime(&date, LSB);
      OldSysTime = DateToTime(date);
   }

   RealSystemTime(&date, LSB);
   SysTime     = DateToTime(date);
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
   if (fabs(dSeconds) > 0.0) {
      Time->Second  += dSeconds;
      long quotient  = Time->Second / 60.0;
      Time->Second   = fmod(Time->Second, 60.0);
      if (Time->Second < 0) {
         Time->Second += 60;
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
               // Time->JulDay =
               //     floor(Time->JulDay + 0.5 + (double)quotient) - 0.5;
               // long tmpHr = 0, tmpMin = 0;
               // double tmpSec = 0.0;
               // JDToDate(Time->JulDay, &Time->Year, &Time->Month, &Time->Day,
               //          &tmpHr, &tmpMin, &tmpSec);
               Time->doy = MD2DOY(Time->Year, Time->Month, Time->Day);
            }
         }
      }
      // Time->JulDay = DateToJD(Time->Year, Time->Month, Time->Day, Time->Hour,
      //                         Time->Minute, Time->Second);
   }
}

/* #ifdef __cplusplus
** }
** #endif
*/

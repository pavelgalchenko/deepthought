/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "jdkit.h"
#include "defineskit.h"
#include "earthorikit.h"
#include "mathkit.h"
#include <ctype.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

// NOTE: Uses the global 'ModelPath'
// __attribute__((const, unused)) static JDSecond
// _epoch_pod_seconds(const EpochTT epoch);
__attribute__((const)) static JDType _epoch_lookup(const TimeSystem system,
                                                   const EpochTT epoch);
__attribute__((const)) static JDType
_epoch_diff_tt(const TimeSystem system, const EpochTT a, const EpochTT b);
__attribute__((const)) static JDType _jdtt(JDType);
__attribute__((const)) static JDType _jd_tdb2tcb(JDType tdb_jd);
__attribute__((const)) static inline double
_sec_dbl_d_tt_tdb(double secs_tt_j2000, double secs_tdb_j2000);
__attribute__((const)) static JDType _jd_tt2tdb(JDType tt_jd);
__attribute__((pure)) static double _tdb2ttF(const double secs_tt_j2000,
                                             double params[1]);
__attribute__((const)) static JDType _jd_tdb2tt(JDType tdb_jd);
__attribute__((const)) static inline JDType _jd_tt2tai(const JDType jd_tt);
__attribute__((const)) static inline JDType _jd_tai2tt(const JDType jd_tai);
__attribute__((const)) static JDType _jd_utc2tai(JDType utc_jd);
__attribute__((const)) static JDType _jd_tai2utc(JDType tai_jd);
__attribute__((const)) static JDType _jd_utc2ut1(JDType tai_jd);
__attribute__((const)) static JDType _jd_ut12utc(JDType ut1_jd);
__attribute__((const)) static JDType _reduce_jd_no_seconds(JDType jd);
__attribute__((const)) static JDType _reduce_jd(JDType jd);
__attribute__((const)) static double _d_tcb_tdb(const double jd);

#ifdef _USE_RATIONAL_
#define ispos_jdsec            ispos_rational
#define isless_jdsec           isless_rational
#define isgreater_jdsec        isgreater_rational
#define Rational2JDSecondLL(a) (a)
#define JDSecIntegerMultMod    IntegerRationalMultMod
#define JDSecondAbs            RationalAbs
#define jdsecond2double        rational2double
// #define jdsecond2double(x)          (x)

JDSecond sec_double2JDSecond(const double dbl)
{
   return double2rational(dbl);
}
JDSecondLL day_double2JDSecond(const double dbl)
{
   const JDSecond part_day = double2rational(dbl);
   return JDSecLLMult(SEC_PER_DAY, part_day);
}

#else
// Begin Function Declarations
__attribute__((const)) static inline int ispos_jdsec(JDSecond sec);
__attribute__((const)) static inline int isless_jdsec(JDSecond a, JDSecond b);
__attribute__((const)) static inline int isgreater_jdsec(JDSecond a,
                                                         JDSecond b);
__attribute__((const)) static inline JDSecond JDSecondAbs(JDSecond a);
__attribute__((const)) static inline Rational jdsecond2rational(JDSecond sec);
__attribute__((const)) static inline JDSecond
JDSecIntegerMultMod(const Rat_LongLong mul, JDSecondLL sec, long mod,
                    long *const carry);
__attribute__((const)) static inline JDSecond
JDSecDblMultMod(const double mul, JDSecondLL sec, long mod, long *const carry);
__attribute__((const)) static inline int
isequal_dbl(const double a, const double b, const double epsilon);

#define JDSecToRational(a)                                                     \
   _Generic((a),                                                               \
       JDSecond: jdsecond2rational,                                            \
       Rational: _iden_rational,                                               \
       RationalLL: _rat_to_rational,                                           \
       double: double2rational)(a)

#define _reduce_jdsec(sec)                                                     \
   do {                                                                        \
      double _red_integral;                                                    \
      sec.frac_sec  = modf(sec.frac_sec, &_red_integral);                      \
      sec.whole    += _red_integral;                                           \
      if ((sec.frac_sec * sec.whole) < 0) {                                    \
         if (sec.whole > 0) {                                                  \
            sec.frac_sec += 1.0;                                               \
            sec.whole    -= 1.0;                                               \
         }                                                                     \
         else if (sec.whole < 0) {                                             \
            sec.frac_sec -= 1.0;                                               \
            sec.whole    += 1.0;                                               \
         }                                                                     \
      }                                                                        \
   } while (0);

// Begin Function Definitions
JDType JDFromJDSeconds(const JDSecond sec, const TimeSystem system,
                       const EpochTT epoch)
{
   JDType jd  = {0};
   jd.epoch   = epoch;
   jd.system  = system;
   jd.seconds = sec;

   double integral;
   jd.seconds.frac_sec  = modf(jd.seconds.frac_sec, &integral);
   jd.seconds.whole    += integral;

   jd.whole_days  = jd.seconds.whole / SEC_PER_DAY;
   jd.whole_days += jd.seconds.frac_sec / SEC_PER_DAY;

   jd.seconds.whole %= SEC_PER_DAY;

   return jd;
}
int isequal_jdsec(JDSecond a, JDSecond b, const double epsilon)
{
   a = ReduceJDSecond(a);
   b = ReduceJDSecond(b);
   return (a.whole == b.whole && isequal_dbl(a.frac_sec, b.frac_sec, epsilon));
}
JDSecondLL rational2jdsecondLL(const Rational rat)
{
   Rational ratout = ReduceRational(rat);
   return JDSECONDLL_RAW(ratout.whole, ratout.num, ratout.den);
}
JDSecondLL rationalll2jdsecondLL(const RationalLL ratll)
{
   JDSecondLL out = {.whole    = ratll.whole,
                     .frac_sec = ((double)ratll.num) / ((double)ratll.den)};
   _reduce_jdsec(out);
   return out;
}
JDSecond ReduceJDSecond(JDSecond sec)
{
   _reduce_jdsec(sec);
   return sec;
}
JDSecondLL ReduceJDSecondLL(JDSecondLL sec)
{
   _reduce_jdsec(sec);
   return sec;
}
JDSecond sec_double2JDSecond(const double dbl)
{
   JDSecond out;
   double integral;
   out.frac_sec = modf(dbl, &integral);
   out.whole    = integral;
   return out;
}
JDSecond day_double2JDSecond(const double dbl)
{
   return sec_double2JDSecond(dbl * SEC_PER_DAY);
}
JDSecondLL sec_double2JDSecondLL(const double dbl)
{
   JDSecondLL out;
   double integral;
   out.frac_sec = modf(dbl, &integral);
   out.whole    = integral;
   return out;
}
JDSecondLL day_double2JDSecondLL(const double dbl)
{
   return sec_double2JDSecondLL(dbl * SEC_PER_DAY);
}
JDSecondLL _JDSecondAdd(JDSecondLL a, JDSecondLL b)
{
   JDSecondLL out = {.whole    = a.whole + b.whole,
                     .frac_sec = a.frac_sec + b.frac_sec};
   _reduce_jdsec(out);
   return out;
}
JDSecondLL _JDSecondSub(JDSecondLL a, JDSecondLL b)
{
   JDSecondLL out = {.whole    = a.whole - b.whole,
                     .frac_sec = a.frac_sec - b.frac_sec};
   _reduce_jdsec(out);
   return out;
}
JDSecondLL RatLLJDSecLLMult(Rat_LongLong mult, JDSecondLL sec)
{
   JDSecondLL out;
   double integral;
   out.frac_sec = modf(sec.frac_sec * mult, &integral);
   out.whole    = integral + (mult * sec.whole);
   _reduce_jdsec(out);
   return out;
}
JDSecondLL RationalLLJDSecLLMult(RationalLL mult, JDSecondLL a)
{
   if ((mult.whole == 0 && mult.num == 0) || (a.whole == 0 && a.frac_sec == 0))
      return JDSECONDLL_ZERO;

   JDSecondLL out     = JDSecLLMult(mult.whole, a);
   mult.whole         = 0;
   Rat_LongLong prod  = a.whole * mult.num;
   out.whole         += prod / mult.den;

   mult.num     = prod % mult.den;
   out.frac_sec = (a.frac_sec * mult.num) / mult.den;
   _reduce_jdsec(out);
   return out;
}
JDSecondLL IntegerJDSecLLMult(long mult, JDSecondLL sec)
{
   JDSecondLL out;
   double integral;
   out.frac_sec = modf(sec.frac_sec * mult, &integral);
   out.whole    = integral + (mult * sec.whole);
   _reduce_jdsec(out);
   return out;
}
JDSecondLL DblJDSecLLMult(double dbl, JDSecondLL sec)
{
   JDSecondLL out;
   double integral;
   out.frac_sec  = modf(sec.frac_sec * dbl, &integral);
   out.whole     = integral;
   out.frac_sec += modf(sec.whole * dbl, &integral);
   out.whole    += integral;

   _reduce_jdsec(out);
   return out;
}
JDSecondLL JDSecJDSecLLMult(JDSecond mult, JDSecondLL a)
{
   JDSecondLL out = JDSecLLMult(mult.whole, a);
   out            = JDSecondAdd(out, JDSecLLMult(mult.frac_sec, a));

   _reduce_jdsec(out);
   return out;
}
JDSecondLL JDSecLLJDSecLLMult(JDSecondLL mult, JDSecondLL a)
{
   JDSecondLL out = JDSecLLMult(mult.whole, a);
   out            = JDSecondAdd(out, JDSecLLMult(mult.frac_sec, a));

   _reduce_jdsec(out);
   return out;
}
JDSecond JDSecondNegate(JDSecond a)
{
   a          = ReduceJDSecond(a);
   a.whole    = -a.whole;
   a.frac_sec = -a.frac_sec;
   return a;
}
JDSecond str2jdsec(const char *str)
{
   JDSecond sec = JDSECOND_ZERO;

   // can get trailing non digits sometimes
   // int len = strlen(buffer);
   // while (!isdigit(buffer[len - 1])) {
   //    buffer[len - 1] = '\0';
   //    len             = strlen(buffer);
   // }

   char frac_sec[JDSECOND_STR_LEN] = {"0."};
   sscanf(str, JDSECOND_STR_FMT_READ, &sec.whole, &frac_sec[2]);
   sec.frac_sec = atof(frac_sec);
   return sec;
}
JDType JDAddIntegerMultJDSecs(const JDType jd, const long mul,
                              const JDSecond sec)
{
   JDType jdb     = jd;
   jdb.whole_days = 0;

   jdb.seconds = JDSecIntegerMultMod(mul, ToJDSecondLL(sec), SEC_PER_DAY,
                                     &jdb.whole_days);

   return JDAddDays(jd, jdb);
}
JDType JDAddJDSecondSeconds(const JDType a, const JDSecond b)
{
   JDType jdb = a;
   double integral;

   jdb.seconds.frac_sec = modf(b.frac_sec, &integral);

   jdb.seconds.whole  = fmod(integral, SEC_PER_DAY);
   jdb.seconds.whole += b.whole % SEC_PER_DAY;

   jdb.whole_days     = integral / SEC_PER_DAY;
   jdb.whole_days    += b.whole / SEC_PER_DAY;
   jdb.whole_days    += jdb.seconds.whole / SEC_PER_DAY;
   jdb.seconds.whole %= SEC_PER_DAY;

   return JDAddDays(a, _reduce_jd(jdb));
}
JDType JDSubJDSecondSeconds(const JDType a, const JDSecond b)
{
   JDType jdb = a;
   double integral;

   jdb.seconds.frac_sec = modf(b.frac_sec, &integral);

   jdb.seconds.whole = fmod(integral, SEC_PER_DAY);
   jdb.seconds.whole = b.whole % SEC_PER_DAY;

   jdb.whole_days     = integral / SEC_PER_DAY;
   jdb.whole_days    += b.whole / SEC_PER_DAY;
   jdb.whole_days    += jdb.seconds.whole / SEC_PER_DAY;
   jdb.seconds.whole %= SEC_PER_DAY;

   return JDSubDays(a, _reduce_jd(jdb));
}
long JDSecondIntMod(JDSecond *const secs, long mod)
{
   long old_whole  = secs->whole;
   secs->whole    %= mod;
   return old_whole / mod;
}
double jdsecond2double(JDSecond sec)
{
   return sec.whole + sec.frac_sec;
}
void jdsec2str(JDSecond sec, char str[JDSECOND_STR_LEN])
{
   JDSecond abssec                = JDSecondAbs(ReduceJDSecond(sec));
   char sec_str[JDSECOND_STR_LEN] = {'\0'};
   snprintf(sec_str, JDSECOND_STR_LEN, "%." STR(JDSECOND_DECIMAL_DIGITS) "lf",
            fabs(abssec.frac_sec));
   const char *decpt = strstr(sec_str, ".");

   int len = strlen(sec_str);
   for (char *p = &sec_str[len - 1]; p > (decpt + 1); p--) {
      if (*p == '0')
         *p = '\0';
      else
         break;
   }

   const char sign_str = (ispos_jdsec(sec)) ? ' ' : '-';
   snprintf(str, JDSECOND_STR_LEN, "%c" JDSECOND_STR_FMT_WRITE, sign_str,
            abssec.whole, &decpt[1]);
}

static inline int ispos_jdsec(JDSecond sec)
{
   sec = ReduceJDSecond(sec);
   return (sec.whole > 0 || (sec.whole == 0 && sec.frac_sec >= 0));
}
static inline int isequal_dbl(const double a, const double b,
                              const double epsilon)
{
   double eps = epsilon;

   if (eps < 0) {
      // compute eps from nextafter and treat epsilon as -ULP
      eps = (-eps) * fabs(nextafter(a, b) - a);
   }
   return fabs(a - b) <= eps;
}
static inline int isless_jdsec(JDSecond a, JDSecond b)
{
   a = ReduceJDSecond(a);
   b = ReduceJDSecond(b);
   return (a.whole < b.whole) ||
          ((a.whole == b.whole) && (a.frac_sec < b.frac_sec));
}
static inline int isgreater_jdsec(JDSecond a, JDSecond b)
{
   a = ReduceJDSecond(a);
   b = ReduceJDSecond(b);
   return (a.whole > b.whole) ||
          ((a.whole == b.whole) && (a.frac_sec > b.frac_sec));
}
static inline JDSecond JDSecIntegerMultMod(const Rat_LongLong mul,
                                           JDSecondLL sec, long mod,
                                           long *const carry)
{
   JDSecondLL out;

   double wholea_dbl;
   out.frac_sec        = modf(mul * sec.frac_sec, &wholea_dbl);
   Rat_LongLong wholea = wholea_dbl;
   Rat_LongLong wholeb = mul * sec.whole;

   *carry     = (wholea / mod) + (wholeb / mod);
   out.whole  = (wholea % mod) + (wholeb % mod);
   *carry    += out.whole / mod;
   out.whole %= mod;

   _reduce_jdsec(out);
   return ToJDSecond(out);
}
static inline JDSecond JDSecDblMultMod(const double mul, JDSecondLL sec,
                                       long mod, long *const carry)
{
   JDSecondLL out;

   double wholea_dbl, wholeb_dbl;
   out.frac_sec         = modf(mul * sec.frac_sec, &wholea_dbl);
   out.frac_sec        += modf(mul * sec.whole, &wholeb_dbl);
   Rat_LongLong wholea  = wholea_dbl;
   Rat_LongLong wholeb  = wholeb_dbl;

   *carry     = (wholea / mod) + (wholeb / mod);
   out.whole  = (wholea % mod) + (wholeb % mod);
   *carry    += out.whole / mod;
   out.whole %= mod;

   _reduce_jdsec(out);
   return ToJDSecond(out);
}
static inline JDSecond JDSecondAbs(JDSecond a)
{
   return (JDSecond){.whole = labs(a.whole), .frac_sec = fabs(a.frac_sec)};
}
static inline Rational jdsecond2rational(JDSecond sec)
{
   Rational out  = double2rational(sec.frac_sec);
   out.whole    += sec.whole;
   return ReduceRational(out);
}
#endif

static void _error_epoch_system(const JDType a, const JDType b,
                                const char *call_func)
{
   if (!isequal_jd_systemepoch(a, b)) {
      fprintf(stderr,
              "In function %s, both input JDTypes must have the same system "
              "and epoch. Exiting...\n",
              call_func);
      exit(EXIT_FAILURE);
   }
}

#define TAI_FACTOR RATIONAL_RAW(32, 23, 125)

// TODO: do we want these in the header?
#define DAY_ZERO_EPOCH_TT     (0.0)       // Jan  1, -4712, 12:00:00
#define DAY_GD_JD_EPOCH_TT    (1721013.5) // Nov 18, -0001, 00:00:00, in GD->JD
#define DAY_MJD_EPOCH_TT      (2400000.5) // Nov 17,  1858, 00:00:00
#define DAY_J1900_EPOCH_TT    (2415019.5) // Dec 31,  1899, 00:00:00, in JD->GD
#define DAY_GMAT_MJD_EPOCH_TT (2430000.0) // Jan  5,  1941, 12:00:00
#define DAY_CCSDS_EPOCH_TT    (2436204.5) // Jan  1,  1958, 00:00:00
#define DAY_TCB_TDB_EPOCH_TT  (2443144.5) // Jan  1, 1977, 00:00:00, in tcb->tdb
#define DAY_J2000_EPOCH_TT    (2451545.0) // Jan  1, 2000, 12:00:00, J2000 epoch

#ifdef _USE_RATIONAL_
#define JD_ZERO_EPOCH_TT JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO)
#define JD_GD_JD_EPOCH_TT                                                      \
   JD_RAW(TT_TIME, ZERO_EPOCH, 1721013, RATIONAL_RAW(43200, 0, 1))
#define JD_MJD_EPOCH_TT                                                        \
   JD_RAW(TT_TIME, ZERO_EPOCH, 2400000, RATIONAL_RAW(43200, 0, 1))
#define JD_J1900_EPOCH_TT                                                      \
   JD_RAW(TT_TIME, ZERO_EPOCH, 2415019, RATIONAL_RAW(43200, 0, 1))
#define JD_GMAT_MJD_EPOCH_TT JD_RAW(TT_TIME, ZERO_EPOCH, 2430000, RATIONAL_ZERO)
#define JD_CCSDS_EPOCH_TT                                                      \
   JD_RAW(TT_TIME, ZERO_EPOCH, 2436204, RATIONAL_RAW(43200, 0, 1))
#define JD_TCB_TDB_EPOCH_TT                                                    \
   JD_RAW(TT_TIME, ZERO_EPOCH, 2443144, RATIONAL_RAW(43200, 0, 1))
#define JD_J2000_EPOCH_TT JD_RAW(TT_TIME, ZERO_EPOCH, 2451545, RATIONAL_ZERO)
#else
#define JD_ZERO_EPOCH_TT JD_RAW(TT_TIME, ZERO_EPOCH, 0, JDSECOND_ZERO)
#define JD_GD_JD_EPOCH_TT                                                      \
   JD_RAW(TT_TIME, ZERO_EPOCH, 1721013, JDSECOND_RAW(43200, 0, 1))
#define JD_MJD_EPOCH_TT                                                        \
   JD_RAW(TT_TIME, ZERO_EPOCH, 2400000, JDSECOND_RAW(43200, 0, 1))
#define JD_J1900_EPOCH_TT                                                      \
   JD_RAW(TT_TIME, ZERO_EPOCH, 2415019, JDSECOND_RAW(43200, 0, 1))
#define JD_GMAT_MJD_EPOCH_TT JD_RAW(TT_TIME, ZERO_EPOCH, 2430000, JDSECOND_ZERO)
#define JD_CCSDS_EPOCH_TT                                                      \
   JD_RAW(TT_TIME, ZERO_EPOCH, 2436204, JDSECOND_RAW(43200, 0, 1))
#define JD_TCB_TDB_EPOCH_TT                                                    \
   JD_RAW(TT_TIME, ZERO_EPOCH, 2443144, JDSECOND_RAW(43200, 0, 1))
#define JD_J2000_EPOCH_TT JD_RAW(TT_TIME, ZERO_EPOCH, 2451545, JDSECOND_ZERO)
#endif

// Make sure that we covered everything EXPLICITLY
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wswitch"
#pragma GCC diagnostic error "-Wswitch-enum"

/**********************************************************************/
TimeSystem GetTimeSystem(const char *s)
{
   if (!strncmp(s, "UT1", 3))
      return UTC_TIME;
   else if (!strncmp(s, "UTC", 3))
      return UTC_TIME;
   else if (!strncmp(s, "TAI", 3))
      return TAI_TIME;
   else if (!strncmp(s, "TCB", 3))
      return TCB_TIME;
   else if (!strncmp(s, "TDB", 3))
      return TDB_TIME;
   else if (!strncmp(s, "TT", 2))
      return TT_TIME;
   fprintf(stderr, "Bogus input %s in GetTimeSystem (jdkit.c:%d)\n", s,
           __LINE__);
   exit(EXIT_FAILURE);
}
/**********************************************************************/
static double epoch_tbl[N_TIME][N_EPOCH];

static once_flag epoch_base_init_flag = ONCE_FLAG_INIT;
// Compute TT_TIME, TAI_TIME, UTC_TIME, and TDB_TIME epochs
static void fill_epoch_base_tbl()
{
   // just precompute the Leapseconds, I can't be bothered
   double leap_secs[] = {1.4228180, 1.4228180, 1.4228180, 1.4228180,
                         1.4228180, 1.4228180, 16,        32};
   // leap_secs[TCB_TDB_CONV_EPOCH] is 16 seconds if
   // _TCB_TDB_EPOCH_TT is indeed TT

   epoch_tbl[TT_TIME][ZERO_EPOCH]         = DAY_ZERO_EPOCH_TT;
   epoch_tbl[TT_TIME][GD_CONV_EPOCH]      = DAY_GD_JD_EPOCH_TT;
   epoch_tbl[TT_TIME][MJD_EPOCH]          = DAY_MJD_EPOCH_TT;
   epoch_tbl[TT_TIME][J1900_EPOCH]        = DAY_J1900_EPOCH_TT;
   epoch_tbl[TT_TIME][GMAT_MJD_EPOCH]     = DAY_GMAT_MJD_EPOCH_TT;
   epoch_tbl[TT_TIME][CCSDS_EPOCH]        = DAY_CCSDS_EPOCH_TT;
   epoch_tbl[TT_TIME][TCB_TDB_CONV_EPOCH] = DAY_TCB_TDB_EPOCH_TT;
   epoch_tbl[TT_TIME][J2000_EPOCH]        = DAY_J2000_EPOCH_TT;

   for (EpochTT epo = ZERO_EPOCH; epo < N_EPOCH; epo++) {
      epoch_tbl[TAI_TIME][epo] = epoch_tbl[TT_TIME][epo] - 32.184 / SEC_PER_DAY;

      epoch_tbl[UTC_TIME][epo] =
          epoch_tbl[TAI_TIME][epo] - leap_secs[epo] / SEC_PER_DAY;

      // const double jdday_tt_j2000 =
      //     epoch_tbl[TT_TIME][epo] - epoch_tbl[TT_TIME][J2000_EPOCH];
      // const double sec_ttmtdb = _sec_dbl_d_tt_tdb(jdday_tt_j2000 *
      // SEC_PER_DAY);
      // epoch_tbl[TDB_TIME][epo] =
      //     epoch_tbl[TT_TIME][epo] + sec_ttmtdb / SEC_PER_DAY;
   }
}

static once_flag epoch_init_flag = ONCE_FLAG_INIT;
// Compute UT1_TIME and TCB_TIME epochs
static void fill_epoch_tbl()
{
   call_once(&epoch_base_init_flag, fill_epoch_base_tbl);

   for (EpochTT epo = ZERO_EPOCH; epo < N_EPOCH; epo++) {
      const double jdays_tai_mjd =
          epoch_tbl[TAI_TIME][epo] - epoch_tbl[TAI_TIME][MJD_EPOCH];
      const double dut1        = GetUt1UtcOffset(jdays_tai_mjd);
      epoch_tbl[UT1_TIME][epo] = epoch_tbl[UTC_TIME][epo] + dut1 / SEC_PER_DAY;
   }

   // TODO: this is an assumption, though should work out with the linear
   // approximation we have
   epoch_tbl[TCB_TIME][TCB_TDB_CONV_EPOCH] =
       epoch_tbl[TDB_TIME][TCB_TDB_CONV_EPOCH];
   for (EpochTT epo = ZERO_EPOCH; epo < N_EPOCH; epo++) {
      if (epo == TCB_TDB_CONV_EPOCH)
         continue;

      const double jdays_tt_conv =
          epoch_tbl[TT_TIME][epo] - epoch_tbl[TT_TIME][TCB_TDB_CONV_EPOCH];
      const double d_tcb_tdb = _d_tcb_tdb(jdays_tt_conv);
      epoch_tbl[TCB_TIME][epo] =
          epoch_tbl[TDB_TIME][epo] + d_tcb_tdb / SEC_PER_DAY;
   }
}

static double _epoch_lookup_days(const TimeSystem system, const EpochTT epoch)
    __attribute__((unused));
static double _epoch_lookup_days(const TimeSystem system, const EpochTT epoch)
{
   call_once(&epoch_base_init_flag, fill_epoch_base_tbl);
   switch (system) {
      case UTC_TIME:
      case TAI_TIME:
      case TT_TIME:
      case TDB_TIME:
         break;
      case UT1_TIME:
      case TCB_TIME:
         call_once(&epoch_init_flag, fill_epoch_tbl);
      case N_TIME:
      default:
         break;
   }

   return epoch_tbl[system][epoch];
}
static JDType _epoch_lookup(const TimeSystem system __attribute__((unused)),
                            const EpochTT epoch)
{
   // return JDFromDoubleDays(_epoch_lookup_days(system, epoch), system,
   // ZERO_EPOCH);
   switch (epoch) {
      case ZERO_EPOCH:
         return JD_ZERO_EPOCH_TT;
      case GD_CONV_EPOCH:
         return JD_GD_JD_EPOCH_TT;
      case MJD_EPOCH:
         return JD_MJD_EPOCH_TT;
      case J1900_EPOCH:
         return JD_J1900_EPOCH_TT;
      case GMAT_MJD_EPOCH:
         return JD_GMAT_MJD_EPOCH_TT;
      case CCSDS_EPOCH:
         return JD_CCSDS_EPOCH_TT;
      case TCB_TDB_CONV_EPOCH:
         return JD_TCB_TDB_EPOCH_TT;
      case J2000_EPOCH:
         return JD_J2000_EPOCH_TT;
      case N_EPOCH:
      default:
         return JD_ZERO;
   }
}
/**********************************************************************/
static JDType _epoch_diff_tt(const TimeSystem system, const EpochTT a,
                             const EpochTT b)
{
   // handle the easy cases here
   if (a == b)
      return JD_ZERO;

   return JDSubDays(_epoch_lookup(system, a), _epoch_lookup(system, b));
}
/**********************************************************************/
//  time system low level conversion helpers
/**********************************************************************/
#define L_B (1.550505e-8)
static inline double _d_tcb_tdb(const double jdays_tt_conv)
{
   return L_B * jdays_tt_conv;
}
#undef L_B
static JDType _jd_tcb2tdb(const JDType tcb_jd __attribute__((unused)))
{
   // JDType jd_tt_conv = _jdtt(tcb_jd);// nope, do not
   // JDChangeEpoch(TCB_TDB_CONV_EPOCH, &jd_tt_conv);
   // TODO: pretending that we don't need this for now
   fprintf(stderr, "Julian Day conversion from TCB to TDB is not implemented. "
                   "Exiting...\n");
   exit(EXIT_FAILURE);
}
static JDType _jd_tdb2tcb(JDType tdb_jd)
{
   const JDType jd_tt_conv =
       JDChangeSystemEpoch(TT_TIME, TCB_TDB_CONV_EPOCH, tdb_jd);
   const double d_tcb_tdb = _d_tcb_tdb(JDToDays(jd_tt_conv));

   tdb_jd.system = TCB_TIME;
   return JDAddSeconds(tdb_jd, d_tcb_tdb);
}
/**********************************************************************/
#define TDB_COEFF1 (0.00165)
#define TDB_COEFF2 (2.2e-05)
#define M_E_OFFSET (357.5277233)
#define M_E_COEFF1 (35999.05034)
// Astronomical Almanac, 2012 // TODO: lookup, current reference is Vallado
// Includes Jovian effects through dlambda_mean
static inline double _sec_dbl_d_tt_tdb(double secs_tt_j2000,
                                       double secs_tdb_j2000)
{
   const double T_TDB = secs_tdb_j2000 / (DAY_PER_JULIAN_CENTURY * SEC_PER_DAY);
   const double m_E   = fmod((M_E_OFFSET + (M_E_COEFF1 * T_TDB)), 360.0);

   const double dlambda_mean =
       (246.11 + 0.90251792 * secs_tt_j2000 / SEC_PER_DAY) * D2R;

   return TDB_COEFF1 * sin(m_E) + TDB_COEFF2 * sin(dlambda_mean);
}
#undef M_E_OFFSET
#undef M_E_COEFF1
/**********************************************************************/
static double _tt2tdbF(const double secs_tdb_j2000, double params[1])
{
   const double secs_tt_j2000 = params[0];
   return secs_tdb_j2000 -
          (secs_tt_j2000 + _sec_dbl_d_tt_tdb(secs_tt_j2000, secs_tdb_j2000));
}

/**********************************************************************/
static inline JDType _jd_tt2tdb(JDType tt_jd)
{
   JDType jd_tt_j2000         = JDChangeEpoch(J2000_EPOCH, tt_jd);
   const double secs_tt_j2000 = JDToSeconds(jd_tt_j2000);
   double params[1]           = {secs_tt_j2000};

   const double max_width      = fabs(TDB_COEFF1 + TDB_COEFF2);
   const double secs_tdb_j2000 = BrentsMethod(
       secs_tt_j2000 - 2.0 * max_width, secs_tt_j2000 + 2.0 * max_width,
       __DBL_EPSILON__, &_tt2tdbF, params);
   JDType jd_tdb_out = JDFromSeconds(secs_tdb_j2000, TDB_TIME, J2000_EPOCH);
   jd_tdb_out        = JDChangeEpoch(tt_jd.epoch, jd_tdb_out);

   jd_tdb_out.system = TDB_TIME;
   return jd_tdb_out;
}
/**********************************************************************/
static double _tdb2ttF(const double secs_tt_j2000, double params[1])
{
   const double secs_tdb_j2000 = params[0];
   return secs_tt_j2000 + _sec_dbl_d_tt_tdb(secs_tt_j2000, secs_tdb_j2000) -
          secs_tdb_j2000;
}
// *******************
static JDType _jd_tdb2tt(JDType tdb_jd)
{
   // Use Brent's Method to approximate inverse of _jd_tt2tdb
   JDType jd_tdb_j2000 = JDChangeEpoch(J2000_EPOCH, tdb_jd);

   const double secs_tdb_j2000 = JDToSeconds(jd_tdb_j2000);
   double params[1]            = {secs_tdb_j2000};

   const double max_width     = fabs(TDB_COEFF1 + TDB_COEFF2);
   const double secs_tt_j2000 = BrentsMethod(
       secs_tdb_j2000 - 2.0 * max_width, secs_tdb_j2000 + 2.0 * max_width,
       __DBL_EPSILON__, &_tdb2ttF, params);
   JDType jd_tt_out = JDFromSeconds(secs_tt_j2000, TT_TIME, J2000_EPOCH);

   jd_tt_out = JDChangeEpoch(tdb_jd.epoch, jd_tt_out);

   jd_tt_out.system = TT_TIME; // just to make sure
   return jd_tt_out;
}
#undef TDB_COEFF1
#undef TDB_COEFF2
/**********************************************************************/
static inline JDType _jd_tt2tai(const JDType jd_tt)
{
   JDType out = jd_tt;
   out.system = TAI_TIME;
   return JDSubSeconds(out, TAI_FACTOR);
}
/**********************************************************************/
static inline JDType _jd_tai2tt(const JDType jd_tai)
{
   JDType out = jd_tai;
   out.system = TT_TIME;
   return JDAddSeconds(out, TAI_FACTOR);
}
/**********************************************************************/
// UTC headaches
static JDType _jd_utc2tai(JDType utc_jd)
{
   const double leap_sec = GetLeapSec(utc_jd);

   utc_jd.system = TAI_TIME;
   return JDAddSeconds(utc_jd, leap_sec);
}
static JDType _jd_tai2utc(JDType jd_tai)
{
   // IF 'GetLeapSec()' GETS BACK HERE, WE'LL HAVE INFINITE RECURSION. AVOID!!

   // sketch of the idea is:
   //    - tell GetLeapSec that this TAI JD is actually UTC
   //    - use the resulting leap sec to actually convert TAI to UTC
   //    - use this actually converted UTC to check for leap sec again
   //       - if this leap sec matches the old one, exellent! We're done!
   //       - if this leap sec does not match the old one, use the newest to do
   //       the conversion and finish

   // NEEDED SO THAT GetLeapSec() DOES NOT GET BACK HERE
   jd_tai.system = UTC_TIME;
   JDType jd_utc = jd_tai;

   const double leap_sec = GetLeapSec(jd_utc);
   jd_utc                = JDSubSeconds(jd_tai, leap_sec);
   const double test_ls  = GetLeapSec(jd_utc);
   if (test_ls != leap_sec)
      jd_utc = JDSubSeconds(jd_tai, test_ls);

   return jd_utc;
}
/**********************************************************************/
static JDType _jd_utc2ut1(JDType utc_jd)
{
   const JDType jd_utc_mjd = JDChangeSystemEpoch(UTC_TIME, MJD_EPOCH, utc_jd);
   const double dut1       = GetUt1UtcOffset(JDToDays(jd_utc_mjd));
   JDType jd_ut1           = JDAddSeconds(utc_jd, dut1);
   jd_ut1.system           = UT1_TIME;
   return jd_ut1;
}
/**********************************************************************/
static double _jd_ut12utcF(const JDType jd_utc, JDType params[1])
{
   const JDType jd_utc_mjd = JDChangeSystemEpoch(UTC_TIME, MJD_EPOCH, jd_utc);
   const double dut1       = GetUt1UtcOffset(JDToDays(jd_utc_mjd));
   JDType jd_ut1_mjd       = params[0];
   jd_ut1_mjd.system       = UTC_TIME;
   return JDSubToSeconds(JDAddSeconds(jd_utc_mjd, dut1), jd_ut1_mjd);
}
// *******************
static JDType _jd_inv_quad_int(JDType a, double fa, double fb, double fc)
{
   return JDMultDbl(a, fb * fc / ((fa - fb) * (fa - fc)));
}
// *******************
static JDType JDBrentsMethod(JDType a, JDType b, const JDType tol,
                             double (*f)(const JDType, JDType *),
                             JDType *params)
{
   _error_epoch_system(a, b, "JDBrentsMethod");

   if (isequal_jd(a, b, __DBL_EPSILON__))
      return a;

   JDType tol_abs = JDAbs(tol);
   tol_abs.system = a.system;
   tol_abs.epoch  = a.epoch;

   double fa = f(a, params);
   double fb = f(b, params);

   if (!(fa * fb < 0)) {
      // fa and fb are same sign (or zero)
      //    return the value associated with the smaller one
      if (fa == 0)
         return a;
      if (fb == 0)
         return b;
      const double mag_fa = fabs(fa);
      const double mag_fb = fabs(fb);
      if (mag_fa < mag_fb)
         return a;
      else
         return b;
   }

   if (fabs(fa) < fabs(fb)) {
      JDType t = a;
      a        = b;
      b        = t;

      double ft = fa;
      fa        = fb;
      fb        = ft;
   }

   JDType c = a, d = JD_ZERO_LIKE(a);
   double fc = fa;
   int mflag = 1;

   JDType err = JDAbs(JDSubDays(b, a));
   while (fb != 0 && isgreater_jd(err, tol_abs)) {
      JDType s = JD_ZERO_LIKE(a);

      if (fa != fc && fb != fc)
         // inverse quadratic interpolation
         s = JDAddDays(JDAddDays(_jd_inv_quad_int(a, fa, fb, fc),
                                 _jd_inv_quad_int(b, fb, fc, fa)),
                       _jd_inv_quad_int(c, fc, fa, fb));
      else
         // secant method
         s = JDaxpy(-fb / (fb - fa), JDSubDays(b, a), b);

      const JDType tmp = JDMultDbl(JDaxpy(3.0, a, b), 0.25);
      const int cond_1 =
          !(isgreater_jd(tmp, b) ? (isless_jd(b, s) && isless_jd(s, tmp))
                                 : (isless_jd(tmp, s) && isless_jd(s, b)));
      const int cond_2 = (mflag) && (fabs(JDSubToSeconds(s, b)) >=
                                     (fabs(JDSubToSeconds(b, c)) / 2.0));
      const int cond_3 = (!mflag) && (fabs(JDSubToSeconds(s, b)) >=
                                      (fabs(JDSubToSeconds(c, d)) / 2.0));
      const int cond_4 = (mflag) && isless_jd(JDAbs(JDSubDays(b, c)), tol_abs);
      const int cond_5 = (!mflag) && isless_jd(JDAbs(JDSubDays(c, d)), tol_abs);
      if (cond_1 || cond_2 || cond_3 || cond_4 || cond_5) {
         // bisection method
         s     = JDMultDbl(JDAddDays(a, b), 0.5);
         mflag = 1;
      }
      else
         mflag = 0;

      d  = c;
      c  = b;
      fc = fb;
      // determine what sign fs is, and replace one of the brackets with it
      double fs = f(s, params);
      if (fa * fs < 0) {
         b  = s;
         fb = fs;
      }
      else {
         a  = s;
         fa = fs;
      }
      if (fabs(fa) < fabs(fb)) {
         JDType t = a;
         a        = b;
         b        = t;

         double ft = fa;
         fa        = fb;
         fb        = ft;
      }

      err = JDAbs(JDSubDays(b, a));
   }
   return b;
}
/**********************************************************************/
static JDType _jd_ut12utc(JDType ut1_jd)
{
   JDType jd_ut1_mjd = JDChangeEpoch(MJD_EPOCH, ut1_jd);
   jd_ut1_mjd.system = UTC_TIME;

   JDType jd_utc_mjd = jd_ut1_mjd;
   // Use Brent's Method to approximate inverse of _jd_tai2ut1

   // currently looking for correct tai around ut1, should be looking around
   // tai. take off leap seconds and look there
   // JDType guess_jd_tai = JDChangeSystem(TAI_TIME, jd_utc_mjd);
   const double dut1 = GetUt1UtcOffset(JDToDays(jd_utc_mjd));

   JDType params[1] = {jd_ut1_mjd};
   // get interval
   double lowertest, uppertest;
   JDType lower, upper;
   int i = 1;
   do {
      double half_width  = i * dut1 / 2.0;
      lower              = JDSubSeconds(jd_utc_mjd, half_width);
      upper              = JDAddSeconds(jd_utc_mjd, half_width);
      lowertest          = _jd_ut12utcF(lower, params);
      uppertest          = _jd_ut12utcF(upper, params);
      i                 *= 2;
   } while ((lowertest * uppertest >= 0) && i < 512);

   jd_utc_mjd = JDBrentsMethod(
       lower, upper, JD_RAW_LIKE(lower, 0, JDSECOND_RAW(0, __DBL_EPSILON__, 1)),
       &_jd_ut12utcF, params);

   JDType jd_utc_out = JDChangeEpoch(ut1_jd.epoch, jd_utc_mjd);

   jd_utc_out.system = TAI_TIME; // just to make sure
   return jd_utc_out;
}
/**********************************************************************/
//  end time system low level conversion helpers
/**********************************************************************/

/**********************************************************************/
//  time system high level conversion helpers
/**********************************************************************/
static JDType _jdutc(const JDType jd)
{
   JDType jd_out = jd;
   switch (jd.system) {
      case TCB_TIME:
         jd_out = _jd_tcb2tdb(jd_out);
         [[fallthrough]];
      case TDB_TIME:
         jd_out = _jd_tdb2tt(jd_out);
         [[fallthrough]];
      case TT_TIME:
         jd_out = _jd_tt2tai(jd_out);
         [[fallthrough]];
      case TAI_TIME:
         jd_out = _jd_tai2utc(jd_out);
         break;
      case UT1_TIME:
         jd_out = _jd_ut12utc(jd_out);
         break;
      case UTC_TIME:
         break;
      case N_TIME:
      default:
         break;
   }
   jd_out.system = UTC_TIME;
   return jd_out;
}
static JDType _jdut1(const JDType jd)
{
   JDType jd_out = jd;
   switch (jd.system) {
      case TCB_TIME:
         jd_out = _jd_tcb2tdb(jd_out);
         [[fallthrough]];
      case TDB_TIME:
         jd_out = _jd_tdb2tt(jd_out);
         [[fallthrough]];
      case TT_TIME:
         jd_out = _jd_tt2tai(jd_out);
         [[fallthrough]];
      case TAI_TIME:
         jd_out = _jd_tai2utc(jd_out);
         [[fallthrough]];
      case UTC_TIME:
         jd_out = _jd_utc2ut1(jd_out);
         break;
      case UT1_TIME:
         break;
      case N_TIME:
      default:
         break;
   }
   jd_out.system = UT1_TIME;
   return jd_out;
}
static JDType _jdtai(const JDType jd)
{
   JDType jd_out = jd;
   switch (jd.system) {
      case UT1_TIME:
         jd_out = _jd_ut12utc(jd_out);
         [[fallthrough]];
      case UTC_TIME:
         jd_out = _jd_utc2tai(jd_out);
         break;
      case TCB_TIME:
         jd_out = _jd_tcb2tdb(jd_out);
         [[fallthrough]];
      case TDB_TIME:
         jd_out = _jd_tdb2tt(jd_out);
         [[fallthrough]];
      case TT_TIME:
         jd_out = _jd_tt2tai(jd_out);
         break;
      case TAI_TIME:
         break;
      case N_TIME:
      default:
         break;
   }
   jd_out.system = TAI_TIME;
   return jd_out;
}
static JDType _jdtcb(const JDType jd)
{
   JDType jd_out = jd;
   switch (jd.system) {
      case UT1_TIME:
         jd_out = _jd_ut12utc(jd_out);
         [[fallthrough]];
      case UTC_TIME:
         jd_out = _jd_utc2tai(jd_out);
         [[fallthrough]];
      case TAI_TIME:
         jd_out = _jd_tai2tt(jd_out);
         [[fallthrough]];
      case TT_TIME:
         jd_out = _jd_tt2tdb(jd_out);
         [[fallthrough]];
      case TDB_TIME:
         jd_out = _jd_tdb2tcb(jd_out);
         break;
      case TCB_TIME:
         break;
      case N_TIME:
      default:
         break;
   }
   jd_out.system = TCB_TIME;
   return jd_out;
}
static JDType _jdtdb(const JDType jd)
{
   JDType jd_out = jd;
   switch (jd.system) {
      case UT1_TIME:
         jd_out = _jd_ut12utc(jd_out);
         [[fallthrough]];
      case UTC_TIME:
         jd_out = _jd_utc2tai(jd_out);
         [[fallthrough]];
      case TAI_TIME:
         jd_out = _jd_tai2tt(jd_out);
         [[fallthrough]];
      case TT_TIME:
         jd_out = _jd_tt2tdb(jd_out);
         break;
      case TCB_TIME:
         jd_out = _jd_tcb2tdb(jd_out);
      case TDB_TIME:
         break;
      case N_TIME:
      default:
         break;
   }
   jd_out.system = TDB_TIME;
   return jd_out;
}
static JDType _jdtt(const JDType jd)
{
   JDType jd_out = jd;
   switch (jd.system) {
      case UT1_TIME:
         jd_out = _jd_ut12utc(jd_out);
         [[fallthrough]];
      case UTC_TIME:
         jd_out = _jd_utc2tai(jd_out);
         [[fallthrough]];
      case TAI_TIME:
         jd_out = _jd_tai2tt(jd_out);
         break;
      case TCB_TIME:
         jd_out = _jd_tcb2tdb(jd_out);
         [[fallthrough]];
      case TDB_TIME:
         jd_out = _jd_tdb2tt(jd_out);
         break;
      case TT_TIME:
         break;
      case N_TIME:
      default:
         break;
   }
   jd_out.system = TT_TIME;
   return jd_out;
}
/**********************************************************************/
//  end time system high level conversion helpers
/**********************************************************************/

// return the desired epoch in JD TT
double EpochValueTT(EpochTT epoch)
{
   switch (epoch) {
      case ZERO_EPOCH:
         return DAY_ZERO_EPOCH_TT;
      case GD_CONV_EPOCH:
         return DAY_GD_JD_EPOCH_TT;
      case MJD_EPOCH:
         return DAY_MJD_EPOCH_TT;
      case J1900_EPOCH:
         return DAY_J1900_EPOCH_TT;
      case GMAT_MJD_EPOCH:
         return DAY_GMAT_MJD_EPOCH_TT;
      case CCSDS_EPOCH:
         return DAY_CCSDS_EPOCH_TT;
      case TCB_TDB_CONV_EPOCH:
         return DAY_TCB_TDB_EPOCH_TT;
      case J2000_EPOCH:
         return DAY_J2000_EPOCH_TT;
      case N_EPOCH:
      default:
         break;
   }
   return 0.0;
}

static int isLineBlank(char *const line)
{
   char *ch;
   int is_blank = 1;
   for (ch = line; *ch != '\0'; ++ch)
      if (!isspace(*ch)) {
         is_blank = 0;
         break;
      }
   return is_blank;
}

struct LeapSecFileEntry {
   JDType jd_mjd_utc; // JD in UTC with MJD epoch

   // TAI-UTC = offset_1 + (MJD - offset_2) x offset_3
   double offset_1;
   double offset_2;
   double offset_3;
};
static struct LeapSecFileTbl {
   long n_entries;
   struct LeapSecFileEntry *entries;
} leapSecTbl = {.n_entries = 0, .entries = NULL};

static __once_flag leapsec_flag = __ONCE_FLAG_INIT;
static void load_leapsec_file()
{
   extern char ModelPath[1000];
   char f_path[1064] = {'\0'};
   strcpy(f_path, ModelPath);
   strcat(f_path, "/data_files/tai-utc.dat");
   FILE *file = fopen(f_path, "rt");
   if (file == NULL) {
      fprintf(stderr, "Error opening tai-utc.dat file '%s'. Exiting...\n",
              f_path);
      exit(EXIT_FAILURE);
   }

   leapSecTbl = (struct LeapSecFileTbl){.n_entries = 0, .entries = NULL};

   // loop over file to find the number of nonempty lines
   char line[512] = {'\0'};
   while (fgets(line, 512, file) != NULL)
      if (!isLineBlank(line))
         leapSecTbl.n_entries++;

   // use number of nonempty lines to allocate the data locations
   leapSecTbl.entries =
       calloc(leapSecTbl.n_entries, sizeof(struct LeapSecFileEntry));

   // rewind file and start parsing for the actual data
   rewind(file);
   int i = 0;
   while (fgets(line, 512, file) != NULL) {
      struct LeapSecFileEntry *const entry = &leapSecTbl.entries[i];
      int y, d;
      char mon[16]           = {'\0'};
      double jd_mjd_utc_days = 0;
      int sscanf_check =
          sscanf(line, "%i %s %i =JD %lf TAI-UTC= %lf S + (MJD - %lf) X %lf S",
                 &y, mon, &d, &jd_mjd_utc_days, &entry->offset_1,
                 &entry->offset_2, &entry->offset_3);

      if (sscanf_check) {
         entry->jd_mjd_utc = DaysToJD(jd_mjd_utc_days, UTC_TIME, ZERO_EPOCH);
         entry->jd_mjd_utc = JDChangeEpoch(MJD_EPOCH, entry->jd_mjd_utc);
         i++;
      }
   }
   fclose(file);
}

// returns the number of leap seconds for specified JD
double GetLeapSec(const JDType jd)
{
   // TODO: this and other functions do not handle the time being *during* a
   // leap second

   // TODO: use spice instead if available?

   call_once(&leapsec_flag, load_leapsec_file);

   // ensure jd is UTC with MJD epoch
   // dug through GMAT source code, JD in 'tai-utc.dat' is UTC
   // TODO: this causes infinite recursion due to the conversion to TT_TIME
   // embeded within
   const JDType jd_mjd_utc = JDChangeSystemEpoch(UTC_TIME, MJD_EPOCH, jd);
   double jd_mjd_utc_days  = JDToDays(jd_mjd_utc);

   // read through the table backwards (we're probably doing a sim closer to the
   // end of the table)
   struct LeapSecFileEntry *entry =
       &leapSecTbl.entries[leapSecTbl.n_entries - 1];
   for (; entry >= leapSecTbl.entries; entry--) {
      if (isgreaterequal_jd(jd_mjd_utc, entry->jd_mjd_utc, __DBL_EPSILON__))
         return entry->offset_1 +
                ((jd_mjd_utc_days - entry->offset_2) * entry->offset_3);
   }
   return 0;
}

// ensure everything in JDType is reduced, and that if whole_days < 0, then so
// are seconds.whole and seconds.num, and vice-versa
static JDType _reduce_jd_no_seconds(JDType jd)
{
   const JDSecondLL jdsec_day  = JDSECONDLL_RAW(SEC_PER_DAY, 0, 1);
   jd.whole_days              += JDSecondIntMod(&jd.seconds, SEC_PER_DAY);
   if (((jd.whole_days < 0 && ispos_jdsec(jd.seconds)) ||
        (jd.whole_days > 0 && !ispos_jdsec(jd.seconds))) &&
       jd.whole_days != 0 &&
       !isequal_jdsec(jd.seconds, JDSECOND_ZERO, __DBL_EPSILON__)) {
      if (jd.whole_days > 0) {
         jd.seconds = ToJDSecond(JDSecondAdd(jd.seconds, jdsec_day));
         jd.whole_days--;
      }
      else if (jd.whole_days < 0) {
         jd.seconds = ToJDSecond(JDSecondSub(jd.seconds, jdsec_day));
         jd.whole_days++;
      }
   }
   return jd;
}
static JDType _reduce_jd(JDType jd)
{
   jd         = _reduce_jd_no_seconds(jd);
   jd.seconds = ReduceJDSecond(jd.seconds);
   return jd;
}

// chages the time system of JD
JDType JDChangeSystem(const TimeSystem new_system, JDType jd)
{
   if (jd.system == new_system)
      return jd;
   switch (new_system) {
      case UTC_TIME:
         return _jdutc(jd);
      case UT1_TIME:
         return _jdut1(jd);
      case TAI_TIME:
         return _jdtai(jd);
      case TCB_TIME:
         return _jdtcb(jd);
      case TDB_TIME:
         return _jdtdb(jd);
      case TT_TIME:
         return _jdtt(jd);
      case N_TIME:
      default:
         fprintf(stderr,
                 "Unkown desired system %u in JDChangeSystem. Exiting...\n",
                 new_system);
         exit(EXIT_FAILURE);
   }
}

// changes the epoch of JD
JDType JDChangeEpoch(const EpochTT new_epoch, JDType jd)
{
   if (new_epoch == jd.epoch)
      return jd; // nothing to do

   // TODO: What kind of care needs to be taken with non TT_TIME systems, since
   // the epochs are currently defined as TT?
   // Current idea is convert to TT_TIME first, change the epoch, then change
   // back?? honestly, is this even correct?
   // Without changing anything, this  causes and infinite recursion since
   // GetLeapSec() changes to MJD_EPOCH

   // In GMAT, *it looks like* they don't bother with this
   // could come from, in part, GMAT being more reserved with what epochs to use
   // regularly

   // Upon furthur reading in Vallado, algorithms from there assume Julian Dates
   // are in UT1 unless otherwise specified

   /* jd_diff.seconds will either be zero or +/- 43200 seconds */
   JDType jd_diff = _epoch_diff_tt(jd.system, jd.epoch, new_epoch);

   // JDType jd_tt = _jdtt(*jd);
   jd                = JDAddDays(jd, jd_diff.whole_days);
   jd.seconds.whole += jd_diff.seconds.whole;
   jd.epoch          = new_epoch;

   // JDChangeSystem(jd->system, &jd_tt);
   // *jd = jd_tt;
   return _reduce_jd_no_seconds(jd);
}

JDType JDChangeSystemEpoch(const TimeSystem new_system, const EpochTT new_epoch,
                           JDType jd)
{
   // This whole kit needs some testing, but this is currently how I prefer to
   // change the jd formats due to the limitations currently in JDChangeEpoch()
   jd = JDChangeEpoch(new_epoch, jd);
   return JDChangeSystem(new_system, jd);
}

// returns the number of Julian days from 'jd.epoch' according to 'jd.system'
double JDToDays(const JDType jd)
{
   return (double)jd.whole_days + (jdsecond2double(jd.seconds)) / SEC_PER_DAY;
}

JDType JDFromDoubleDays(const double days, const TimeSystem system,
                        const EpochTT new_epoch)
{
   JDType jd       = {0};
   double integral = 0;
   jd.epoch        = new_epoch;
   jd.system       = system;
   jd.seconds      = day_double2JDSecond(modf(days, &integral));
   jd.whole_days   = integral;
   return jd;
}
/**********************************************************************/
/*  Converts rational seconds since 'epoch' in 'system' to a JDType   */
/*  format                                                            */
JDType JDFromRationalSeconds(const Rational seconds, const TimeSystem system,
                             const EpochTT new_epoch)
{
   JDType jd     = {0};
   jd.epoch      = new_epoch;
   jd.system     = system;
   jd.whole_days = seconds.whole / SEC_PER_DAY;
   jd.seconds    = ToJDSecond(
       RATIONAL_NGCD(seconds.whole % SEC_PER_DAY, seconds.num, seconds.den));
   return jd;
} /**********************************************************************/
/*  Converts seconds since 'epoch' in 'system' to a JDType format     */
JDType JDFromSeconds(const double seconds, const TimeSystem system,
                     const EpochTT epoch)
{
   JDType jd     = {0};
   jd.epoch      = epoch;
   jd.system     = system;
   jd.whole_days = seconds / SEC_PER_DAY;
   jd.seconds    = sec_double2JDSecond(fmod(seconds, SEC_PER_DAY));
   return jd;
}
/**********************************************************************/
/* Time is elapsed seconds since input epoch                          */
/*  This function returns the seconds in whatever system the input    */
/*  'jd' uses                                                         */
double JDToSeconds(JDType jd)
{
   return ((double)jd.whole_days * SEC_PER_DAY + jdsecond2double(jd.seconds));
}
/**********************************************************************/
Rational JDToRationalSeconds(JDType jd)
{
   // this will be ~292,271,023,045 years for a 64-bit system
   const Rat_Long max_days = _RATLONG_MAX_ / SEC_PER_DAY;
   if (jd.whole_days > max_days) {
      fprintf(stderr,
              "How in goodness name do you have Julian Days > %li in "
              "JDToRationalSeconds!?! Exiting...\n",
              max_days);
      exit(EXIT_FAILURE);
   }
   Rational out  = jdsecond2rational(jd.seconds);
   out.whole    += jd.whole_days * SEC_PER_DAY;
   return out;
}
/**********************************************************************/
/* Time is elapsed seconds since J2000 epoch                          */
/*  This function returns the seconds in whatever system the input    */
/*  'jd' uses                                                         */
double JDToTime(JDType jd)
{
   return JDToSeconds(JDChangeEpoch(J2000_EPOCH, jd));
}
/**********************************************************************/
/* Time is elapsed seconds since J2000 epoch in TT time               */
double JDToDynTime(JDType jd)
{
   return JDToTime(JDChangeSystem(TT_TIME, jd));
}

JDType InitJD(const TimeSystem system, const EpochTT epoch, const long days,
              const JDSecond seconds)
{
   JDType jd;
   jd.epoch      = epoch;
   jd.system     = system;
   jd.whole_days = days;
   jd.seconds    = seconds;

   return _reduce_jd(jd);
}

static JDType _jd_rational_mult_helper(Rational mul, const JDType jd)
{
   JDType jd_out = jd;

   jd_out.seconds     = ToJDSecond(JDSecLLMult(mul, jd.seconds));
   Rational jd_whole  = ToRational(IntegerRationalMult(jd.whole_days, mul));
   jd_out.whole_days  = jd_whole.whole;
   jd_whole.whole     = 0;
   RationalLL jd_secs = IntegerRationalMult(SEC_PER_DAY, jd_whole);

   jd_out.seconds =
       ToJDSecond(JDSecondAdd(ToJDSecond(jd_secs), jd_out.seconds));

   return _reduce_jd(jd_out);
}

JDType JDAddJD(JDType a, JDType b)
{
   EpochTT out_epoch = a.epoch;
   if (a.epoch == ZERO_EPOCH && b.epoch != ZERO_EPOCH)
      out_epoch = b.epoch;
   if ((a.epoch == ZERO_EPOCH) != (b.epoch == ZERO_EPOCH)) {
      a.epoch = out_epoch;
      b.epoch = out_epoch;
   }
   if (isequal_jd(a, JD_ZERO_LIKE(a), 0))
      return b;
   else if (isequal_jd(b, JD_ZERO_LIKE(b), 0))
      return a;

   _error_epoch_system(a, b, "JDAddJD");
   JDType jdout      = a;
   jdout.whole_days += b.whole_days;
   jdout.seconds     = ToJDSecond(JDSecondAdd(jdout.seconds, b.seconds));

   return _reduce_jd(jdout);
}
/**********************************************************************/
/*  Add (mul * b) seconds to the Julian Date in jd using an integer   */
/*  arithmetic multiplication algorithm                               */
JDType JDAddIntegerMultRatSecs(const JDType jd, const long mul,
                               const Rational rat)
{
   JDType jdb = jd;

   jdb.whole_days = 0;
   jdb.seconds    = JDSecIntegerMultMod(mul, ToJDSecondLL(rat), SEC_PER_DAY,
                                        &jdb.whole_days);

   return JDAddDays(jd, jdb);
}

JDType JDAddRationalMult(const JDType a, const Rational mul, JDType b)
{
   b.system = a.system;
   b.epoch  = a.epoch;
   b        = _jd_rational_mult_helper(mul, b);

   return JDAddDays(a, b);
}

JDType JDSubJD(JDType a, JDType b)
{
   if (isequal_jd(a, JD_ZERO, __DBL_EPSILON__))
      return JDNegate(b);
   else if (isequal_jd(b, JD_ZERO, __DBL_EPSILON__))
      return a;
   EpochTT out_epoch = a.epoch;
   if (a.epoch == ZERO_EPOCH && b.epoch != ZERO_EPOCH)
      out_epoch = b.epoch;
   if ((a.epoch == ZERO_EPOCH) != (b.epoch == ZERO_EPOCH)) {
      a.epoch = out_epoch;
      b.epoch = out_epoch;
   }

   _error_epoch_system(a, b, "JDSubJD");
   JDType jdout      = a;
   jdout.whole_days -= b.whole_days;
   jdout.seconds     = ToJDSecond(JDSecondSub(jdout.seconds, b.seconds));

   return _reduce_jd(jdout);
}

JDType JDSubRationalMult(const JDType a, const Rational mul, JDType b)
{
   b.system = a.system;
   b.epoch  = a.epoch;
   b        = _jd_rational_mult_helper(mul, b);

   return JDSubDays(a, b);
}
/**********************************************************************/
static inline int _ispos_dbl(const double x)
{
   return SIGN(x);
}
#define _gen_ispos(x)                                                          \
   _Generic((x),                                                               \
       double: _ispos_dbl,                                                     \
       JDSecond: ispos_jdsec,                                                  \
       Rational: _ispos_rat,                                                   \
       RationalLL: _ispos_ratll)(x)

#define _gen_abs(x)                                                            \
   _Generic((x),                                                               \
       double: fabs,                                                           \
       JDSecond: JDSecondAbs,                                                  \
       Rational: _rat_abs,                                                     \
       RationalLL: _ratll_abs)(x)

/**********************************************************************/
// handle the multiplication of a JDType by a double, JDSecond, or
// JDSecondLL
// #define _jdmult_body(a, x)
//    const int sign             = SIGN(_gen_ispos((a)));
//    const JDSecondLL mult_sec  = ToJDSecondLL(_gen_abs((a)));
//    JDSecondLL day_mult        = JDSecLLMult(x.whole_days, mult_sec);
//    (x).whole_days             = day_mult.whole;
//    day_mult.whole             = 0;
//    day_mult                   = JDSecLLMult(SEC_PER_DAY, day_mult);
//    JDSecondLL secs            = JDSecLLMult(mult_sec, (x).seconds);
//    (x).seconds                = ToJDSecond(JDSecondAdd(secs, day_mult));
//    (x)                        = _reduce_jd((x));
//    (x).whole_days            *= sign;
//    if (sign < 0)
//       (x).seconds = JDSecondNegate((x).seconds);
//
//    return x;

JDType JDMultDbl(JDType x, const double a)
{
   JDType out  = JD_ZERO_LIKE(x);
   out.seconds = JDSecDblMultMod(a, ToJDSecondLL(x.seconds), SEC_PER_DAY,
                                 &out.whole_days);
   double integral;
   double day_rem  = modf(a * x.whole_days, &integral);
   out.seconds     = ToJDSecond(JDSecondAdd(out, day_rem * SEC_PER_DAY));
   out.whole_days += integral;

   return _reduce_jd(out);
}
// JDType _jdmult_rat(JDType x, const Rational a)
// {
//    _jdmult_body(a, x);
// }
// JDType _jdmult_ratll(JDType x, const RationalLL a)
// {
//    _jdmult_body(a, x);
// }
/**********************************************************************/
double JDAddToDays(const JDType a, const JDType b)
{
   return JDToDays(JDAddDays(a, b));
}

double JDAddToSeconds(const JDType a, const JDType b)
{
   return JDToSeconds(JDAddDays(a, b));
}

double JDSubToDays(const JDType a, const JDType b)
{
   return JDToDays(JDSubDays(a, b));
}

double JDSubToSeconds(const JDType a, const JDType b)
{
   return JDToSeconds(JDSubDays(a, b));
}

double JDPODSeconds(const JDType jd)
{
   return jdsecond2double(jd.seconds);
}

double JDPODDays(const JDType jd)
{
   return JDPODSeconds(jd) / SEC_PER_DAY;
}

JDType JDAbs(JDType jd)
{
   jd.whole_days = labs(jd.whole_days);
   jd.seconds    = JDSecondAbs(jd.seconds);
   return jd;
}

int ispos_jd(JDType jd)
{
   jd = _reduce_jd(JDChangeEpoch(ZERO_EPOCH, jd));
   return (jd.whole_days > 0 ||
           (jd.whole_days == 0 && ispos_jdsec(jd.seconds)));
}

JDType JDNegate(JDType jd)
{
   jd            = _reduce_jd(jd);
   jd.whole_days = -jd.whole_days;
   jd.seconds    = JDSecondNegate(jd.seconds);
   return jd;
}

int isequal_jd_systemepoch(const JDType a, const JDType b)
{
   return (a.system == b.system) && (a.epoch == b.epoch);
}

int isequal_jd(const JDType a, const JDType b, const double epsilon)
{
   return ((a.system == b.system) && (a.epoch == b.epoch) &&
           (a.whole_days == b.whole_days) &&
           isequal_jdsec(a.seconds, b.seconds, epsilon));
}

int isless_jd(const JDType a, const JDType b)
{
   _error_epoch_system(a, b, "isless_jd");
   const int is_day_less = a.whole_days < b.whole_days;
   const int is_sec_less =
       (a.whole_days == b.whole_days) && isless_jdsec(a.seconds, b.seconds);
   return is_day_less || is_sec_less;
}

int islessequal_jd(const JDType a, const JDType b, const double epsilon)
{
   _error_epoch_system(a, b, "islessequal_jd");
   return isless_jd(a, b) || isequal_jd(a, b, epsilon);
}

int isgreater_jd(const JDType a, const JDType b)
{
   _error_epoch_system(a, b, "isgreater_jd");
   const int is_day_greater = a.whole_days > b.whole_days;
   const int is_sec_greater =
       (a.whole_days == b.whole_days) && isgreater_jdsec(a.seconds, b.seconds);
   return is_day_greater || is_sec_greater;
}

int isgreaterequal_jd(const JDType a, const JDType b, const double epsilon)
{
   _error_epoch_system(a, b, "isgreaterequal_jd");
   return isgreater_jd(a, b) || isequal_jd(a, b, epsilon);
}

EpochTT str2epoch(char str[JDEPOCH_STR_LEN])
{

   if (!strncmp(str, "Zero", 5))
      return ZERO_EPOCH;
   else if (!strncmp(str, "JD/GD", 6))
      return GD_CONV_EPOCH;
   else if (!strncmp(str, "TCB/TDB", 8))
      return TCB_TDB_CONV_EPOCH;
   else if (!strncmp(str, "MJD", 4))
      return MJD_EPOCH;
   else if (!strncmp(str, "J1900", 6))
      return J1900_EPOCH;
   else if (!strncmp(str, "GMAT-MJD", 9))
      return GMAT_MJD_EPOCH;
   else if (!strncmp(str, "CCSDS", 6))
      return CCSDS_EPOCH;
   else if (!strncmp(str, "J2000", 6))
      return J2000_EPOCH;

   fprintf(stderr, "Invalid string '%s' in str2epoch. Exiting...\n", str);
   exit(EXIT_FAILURE);
}

TimeSystem str2system(char str[JDSYSTEM_STR_LEN])
{
   if (!strncmp(str, "UTC", 4))
      return UTC_TIME;
   else if (!strncmp(str, "UT1", 4))
      return UT1_TIME;
   else if (!strncmp(str, "TAI", 4))
      return TAI_TIME;
   else if (!strncmp(str, "TT", 3))
      return TT_TIME;
   else if (!strncmp(str, "TCB", 4))
      return TCB_TIME;
   else if (!strncmp(str, "TDB", 4))
      return TDB_TIME;

   fprintf(stderr, "Invalid string '%s' in str2system. Exiting...\n", str);
   exit(EXIT_FAILURE);
}

void epoch2str(EpochTT epoch, char str[JDEPOCH_STR_LEN])
{
   switch (epoch) {
      case ZERO_EPOCH:
         strcpy(str, "Zero");
         break;
      case GD_CONV_EPOCH:
         strcpy(str, "JD/GD");
         break;
      case TCB_TDB_CONV_EPOCH:
         strcpy(str, "TCB/TDB");
         break;
      case MJD_EPOCH:
         strcpy(str, "MJD");
         break;
      case J1900_EPOCH:
         strcpy(str, "J1900");
         break;
      case GMAT_MJD_EPOCH:
         strcpy(str, "GMAT-MJD");
         break;
      case CCSDS_EPOCH:
         strcpy(str, "CCSDS");
         break;
      case J2000_EPOCH:
         strcpy(str, "J2000");
         break;
      case N_EPOCH:
      default:
         break;
   }
}

void system2str(TimeSystem system, char str[JDSYSTEM_STR_LEN])
{
   switch (system) {
      case UTC_TIME:
         strcpy(str, "UTC");
         break;
      case UT1_TIME:
         strcpy(str, "UT1");
         break;
      case TAI_TIME:
         strcpy(str, "TAI");
         break;
      case TT_TIME:
         strcpy(str, "TT");
         break;
      case TCB_TIME:
         strcpy(str, "TCB");
         break;
      case TDB_TIME:
         strcpy(str, "TDB");
         break;
      case N_TIME:
      default:
         break;
   }
}

void jd2str(JDType jd, char str[JD_STR_LEN])
{
   const char *jdday_str_fmt = JD_STR_FMT_WRITE;

   char epoch_str[JDEPOCH_STR_LEN]   = {'\0'};
   char system_str[JDSYSTEM_STR_LEN] = {'\0'};
   epoch2str(jd.epoch, epoch_str);
   system2str(jd.system, system_str);

   char sec_str[JDSECOND_STR_LEN] = {'\0'};
   jdsec2str(jd.seconds, sec_str);
   snprintf(str, JD_STR_LEN, jdday_str_fmt, jd.whole_days, sec_str, system_str,
            epoch_str);
}

void jdays2str(JDType jd, char str[JD_STR_LEN])
{
   const char *jdday_str_fmt         = JDAY_STR_FMT_WRITE;
   char epoch_str[JDEPOCH_STR_LEN]   = {'\0'};
   char system_str[JDSYSTEM_STR_LEN] = {'\0'};
   epoch2str(jd.epoch, epoch_str);
   system2str(jd.system, system_str);

   const double jd_pod = jdsecond2double(jd.seconds) / SEC_PER_DAY;
   char sec_str[28]    = {'\0'};
   snprintf(sec_str, 28, "%.20lf", jd_pod);
   snprintf(str, JD_STR_LEN, jdday_str_fmt, jd.whole_days, &sec_str[2],
            system_str, epoch_str);
}

#pragma GCC diagnostic pop
#undef TAI_FACTOR

/* #ifdef __cplusplus
** }
** #endif
*/
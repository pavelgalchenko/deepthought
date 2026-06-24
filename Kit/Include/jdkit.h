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

#define JD_STR_LEN        (256)
#define JDEPOCH_STR_LEN   (10)
#define JDSYSTEM_STR_LEN  (4)
#define JDDAY_PER_CENTURY (36525)

#define EPOCH_STR_FMT_READ "%10[-a-zA-Z/0-9]"

#ifdef _USE_RATIONAL_
#define JD_STR_FMT_WRITE "JD % 9li days, (%-10.64s) seconds, %s Time, %s Epoch"
#define JD_STR_FMT_READ                                                        \
   "JD %li days, (%[0-9. +()/]) seconds, %s Time, " EPOCH_STR_FMT_READ " Epoc" \
   "h"
#else
#define JD_STR_FMT_WRITE "JD % 9li days, %-12s seconds, %s Time, %s Epoch"
#define JD_STR_FMT_READ                                                        \
   "JD %li days, %[0-9.] seconds, %s Time, " EPOCH_STR_FMT_READ " Epoch"
#endif

#define JDAY_STR_FMT_WRITE "JD % 9li.%s days, %s Time, %s Epoch"
#define JDAY_STR_FMT_READ                                                      \
   "JD %li.%[0-9] days, %s Time, " EPOCH_STR_FMT_READ " Epoch"

typedef enum EpochTT {
   ZERO_EPOCH = 0,     // Jan  1, -4712, 12:00:00
   GD_CONV_EPOCH,      // Nov 18, -0001, 00:00:00, in GD->JD
   MJD_EPOCH,          // Nov 17,  1858, 00:00:00
   J1900_EPOCH,        // Dec 31,  1899, 00:00:00, in JD->GD
   GMAT_MJD_EPOCH,     // Jan  5,  1941, 12:00:00
   CCSDS_EPOCH,        // Jan 1,   1958, 00:00:00
   TCB_TDB_CONV_EPOCH, // Jan  1,  1977, 00:00:00, in tcb->tdb
   J2000_EPOCH,        // Jan 1 ,  2000, 12:00:00, J2000 epoch
   N_EPOCH,
   // TODO: other epochs?
} EpochTT;
// TODO: add CUSTOM_EPOCH, but then epoch in JDType will need to be a struct
// with members of EpochTT and a value that is used only for the custom value

typedef enum TimeSystem {
   UTC_TIME = 0, // Coordinated Universal Time
   UT1_TIME,     // UT1
   TAI_TIME,     // International Atomic Time
   TT_TIME,      // Terrestrial Time, sometimes referred as old TDT term
   TCB_TIME,     // Barycentric Coordinate Time
   TDB_TIME,     // Barycentric Dynamical Time
   N_TIME,
   // TODO: add TT(BIPM)? others?
} TimeSystem;

__attribute__((pure)) TimeSystem GetTimeSystem(const char *s);

// some strict typing to enforce correct timing interpretation

#ifdef _USE_RATIONAL_
typedef Rational JDSecond;
typedef RationalLL JDSecondLL;
#define ToJDSecond   ToRational
#define ToJDSecondLL ToRationalLL

#define JDSECOND_RAW                RATIONAL_RAW
#define JDSECOND_NRED               RATIONAL_NGCD
#define JDSECONDLL_RAW              RATIONALLL_RAW
#define JDSECONDLL_NRED             RATIONALLL_NGCD
#define JDSecondAdd                 RationalAdd
#define JDSecondSub                 RationalSub
#define jdsecond2double             rational2double
#define RationalJDSecLLMult(a, b)   RationalMult((a), (b))
#define RationalLLJDSecLLMult(a, b) RationalMult((a), (b))
#define JDSecJDSecLLMult(a, b)      RationalMult((a), (b))
#define JDSecLLJDSecLLMult(a, b)    RationalMult((a), (b))
#define IntegerJDSecLLMult          IntegerRationalMult
#define DblJDSecLLMult(a, b)        RationalMult((a), (b))
#define ReduceJDSecond              ReduceRational
#define JDSecondIntMod              RationalIntMod

#define JDSECOND_STR_LEN       RATIONAL_STR_LEN
#define JDSECOND_STR_FMT_WRITE RATIONAL_STR_FMT
#define JDSECOND_STR_FMT_READ  RATIONAL_STR_FMT

#define isequal_jdsec(a, b, none) isequal_rational(a, b);
#define JDSecondNegate            RationalNegate
#define jdsec2str                 rat2str
#define str2jdsec                 str2rat

// Second argument is always JDSecond, returns RationalLL
#define JDSecMult RationalMult
#else
typedef struct JDSecond_s {
   // this approach caps the error at __DBL_EPSILON__ = 2^(-52)
   Rat_Long whole;
   double frac_sec;
} JDSecond;
typedef struct JDSecondLL_s {
   Rat_LongLong whole;
   double frac_sec;
} JDSecondLL;
#define JDSECOND_RAW(whl, frac, den)                                           \
   ((JDSecond){.whole    = (whl),                                              \
               .frac_sec = (((double)(frac)) / ((double)(den)))})
#define JDSECOND_NRED(whl, frac, den)                                          \
   JDSECOND_RAW((whl) + ((long)(frac)) / (den), ((frac) - ((long)(frac))),     \
                (den))
#define JDSECONDLL_RAW(whl, frac, den)                                         \
   ((JDSecondLL){.whole    = (whl),                                            \
                 .frac_sec = (((double)(frac)) / ((double)(den)))})
#define JDSECONDLL_NRED(whl, frac, den)                                        \
   JDSECONDLL_RAW((whl) + ((long)(frac)) / (den), ((frac) - ((long)(frac))),   \
                  (den))

#define JDSECOND_STR_LEN        (64)
#define JDSECOND_DECIMAL_DIGITS 18
#define JDSECOND_STR_FMT_WRITE  "%5ld.%.12s"
#define JDSECOND_STR_FMT_READ   "%ld.%47[0-9]"

#define JDSecondAdd(a, b) _JDSecondAdd(ToJDSecondLL((a)), ToJDSecondLL((b)))
#define JDSecondSub(a, b) _JDSecondSub(ToJDSecondLL((a)), ToJDSecondLL((b)))

__attribute__((const)) JDSecondLL RationalLLJDSecLLMult(RationalLL mult,
                                                        JDSecondLL a);
__attribute__((const)) static inline JDSecondLL
RationalJDSecLLMult(Rational mult, JDSecondLL a)
{
   return RationalLLJDSecLLMult(ToRationalLL(mult), a);
}
__attribute__((const)) JDSecondLL JDSecJDSecLLMult(JDSecond, JDSecondLL);
__attribute__((const)) JDSecondLL JDSecLLJDSecLLMult(JDSecondLL mult,
                                                     JDSecondLL a);
__attribute__((const)) JDSecondLL DblJDSecLLMult(double, JDSecondLL);
__attribute__((const)) JDSecondLL IntegerJDSecLLMult(long mult, JDSecondLL sec);
__attribute__((const)) JDSecondLL RatLLJDSecLLMult(Rat_LongLong mult,
                                                   JDSecondLL sec);
__attribute__((const)) JDSecondLL _JDSecondAdd(JDSecondLL a, JDSecondLL b);
__attribute__((const)) JDSecondLL _JDSecondSub(JDSecondLL a, JDSecondLL b);
__attribute__((const)) double jdsecond2double(JDSecond sec);
__attribute__((const)) int isequal_jdsec(JDSecond a, JDSecond b,
                                         const double epsilon);
__attribute__((const)) JDSecond JDSecondNegate(JDSecond a);
void jdsec2str(JDSecond sec, char str[JDSECOND_STR_LEN]);
__attribute__((pure)) JDSecond str2jdsec(const char *str);
__attribute__((const)) JDSecond ReduceJDSecond(JDSecond sec);
__attribute__((const)) long JDSecondIntMod(JDSecond *const secs, long mod);
#endif

#define JDSECOND_ZERO   JDSECOND_RAW(0, 0, 1)
#define JDSECONDLL_ZERO JDSECONDLL_RAW(0, 0, 1)

static inline JDSecondLL jdsecondll_iden(const JDSecondLL sec)
{
   return sec;
}
static inline JDSecondLL jdsec2jdsecll(const JDSecond sec)
{
   return JDSECONDLL_RAW(sec.whole, sec.frac_sec, 1);
}
static inline JDSecond jdsecll2jdsec(const JDSecondLL sec)
{
   return JDSECOND_RAW(sec.whole, sec.frac_sec, 1);
}
static inline JDSecondLL long2jdsecondll(long x)
{
   return JDSECONDLL_RAW(x, 0, 1);
}

typedef struct JDType {
   // Julian day representation
   // while the value of day changes with the time system, the value indicated
   // by 'epoch' will always be in the TT system
   TimeSystem system;
   EpochTT epoch;
   long whole_days;
   JDSecond seconds;
} JDType;

#define JD_RAW(sys, epc, day, sec)                                             \
   ((JDType){.system     = (sys),                                              \
             .epoch      = (epc),                                              \
             .whole_days = (day),                                              \
             .seconds    = (sec)})
#define JD_RAW_LIKE(a, day, sec)                                               \
   ((JDType){.system     = (a).system,                                         \
             .epoch      = (a).epoch,                                          \
             .whole_days = (day),                                              \
             .seconds    = (sec)})
#define JD_ZERO         JD_RAW(TT_TIME, ZERO_EPOCH, 0, JDSECOND_ZERO)
#define JD_ZERO_LIKE(a) JD_RAW_LIKE((a), 0, JDSECOND_ZERO)
#define JD_NREDUCE(sys, epc, day)                                              \
   JD_RAW((sys), (epc), (day),                                                 \
          JDSECOND_NRED(((day) - ((long)(day))) * 86400.0, 0, 1))

static inline JDType _jdtype_iden(const JDType a,
                                  __attribute__((unused))
                                  const TimeSystem system,
                                  __attribute__((unused)) const EpochTT epoch)
{
   return a;
}
static inline JDType JDFromIntegerDays(const long day, const TimeSystem system,
                                       const EpochTT epoch)
{
   return JD_RAW(system, epoch, day, JDSECOND_ZERO);
}
#define DaysToJD(a, sys, epo)                                                  \
   _Generic((a),                                                               \
       JDType: _jdtype_iden,                                                   \
       int: JDFromIntegerDays,                                                 \
       long: JDFromIntegerDays,                                                \
       double: JDFromDoubleDays)((a), (sys), (epo))

static inline JDType JDFromIntegerSeconds(const long sec,
                                          const TimeSystem system,
                                          const EpochTT epoch)
{
   JDType jd        = {0};
   jd.system        = system;
   jd.epoch         = epoch;
   jd.whole_days    = sec / SEC_PER_DAY;
   jd.seconds.whole = sec % SEC_PER_DAY;
   return jd;
}

#ifdef _USE_RATIONAL_
#define SecondsToJD(a, sys, epo)                                               \
   _Generic((a),                                                               \
       Rational: JDFromRationalSeconds,                                        \
       int: JDFromIntegerSeconds,                                              \
       long: JDFromIntegerSeconds,                                             \
       double: JDFromSeconds)((a), (sys), (epo))

#define JDAddIntegerMultSeconds JDAddIntegerMultRatSecs

static inline Rational jd2jdsecond(JDType a)
{
   Rational out;
   out        = a.seconds;
   out.whole += a.whole_days * SEC_PER_DAY;
   return out;
}
static inline RationalLL jd2jdsecondll(JDType a)
{
   RationalLL out;
   out        = a.seconds;
   out.whole += a.whole_days * SEC_PER_DAY;
   return out;
}
#define ToJDSecond(a)                                                          \
   _Generic((a),                                                               \
       JDType: jd2jdsecond,                                                    \
       Rational: _iden_rational,                                               \
       RationalLL: _rat_to_rational,                                           \
       double: double2rational,                                                \
       int: int2rational,                                                      \
       long: long2rational)(a)
#define ToJDSecondLL(a)                                                        \
   _Generic((a),                                                               \
       JDType: jd2jdsecondll,                                                  \
       Rational: _rat_to_rationalll,                                           \
       RationalLL: _iden_rationalll,                                           \
       double: double2rationalll,                                              \
       int: int2rationalll,                                                    \
       long: long2rationalll)(a)

#else
#define SecondsToJD(a, sys, epo)                                               \
   _Generic((a),                                                               \
       Rational: JDFromRationalSeconds,                                        \
       JDSecond: JDFromJDSeconds,                                              \
       int: JDFromIntegerSeconds,                                              \
       long: JDFromIntegerSeconds,                                             \
       double: JDFromSeconds)((a), (sys), (epo))

__attribute__((const)) JDType JDFromJDSeconds(const JDSecond sec,
                                              const TimeSystem system,
                                              const EpochTT epoch);
__attribute__((const)) JDType JDAddIntegerMultJDSecs(const JDType jd,
                                                     const long mul,
                                                     const JDSecond sec);
__attribute__((const)) JDType JDAddJDSecondSeconds(const JDType a,
                                                   const JDSecond b);
__attribute__((const)) JDType JDSubJDSecondSeconds(const JDType a,
                                                   const JDSecond b);

// Second argument is always JDSecondLL, returns JDSecondLL
#define JDSecLLMult(a, b)                                                      \
   _Generic((a),                                                               \
       Rational: RationalJDSecLLMult,                                          \
       RationalLL: RationalLLJDSecLLMult,                                      \
       JDSecond: JDSecJDSecLLMult,                                             \
       JDSecondLL: JDSecLLJDSecLLMult,                                         \
       int: IntegerJDSecLLMult,                                                \
       long: IntegerJDSecLLMult,                                               \
       Rat_LongLong: RatLLJDSecLLMult,                                         \
       double: DblJDSecLLMult)((a), ToJDSecondLL(b))
// Second argument is always JDSecond, returns JDSecond
#define JDSecMult(a, b) ToJDSecond(JDSecLLMult(a, ToJDSecondLL(b)))

#define JDAddIntegerMultSeconds(a, b, c)                                       \
   _Generic((c),                                                               \
       Rational: JDAddIntegerMultRatSecs,                                      \
       JDSecond: JDAddIntegerMultJDSecs)(a, b, c)

static inline JDSecondLL jd2jdsecondLL(JDType a)
{
   JDSecondLL out  = jdsec2jdsecll(a.seconds);
   out.whole      += a.whole_days * SEC_PER_DAY;
   return out;
}
#define ToJDSecondLL(a)                                                        \
   _Generic((a),                                                               \
       JDType: jd2jdsecondLL,                                                  \
       JDSecond: jdsec2jdsecll,                                                \
       JDSecondLL: jdsecondll_iden,                                            \
       Rational: rational2jdsecondLL,                                          \
       RationalLL: rationalll2jdsecondLL,                                      \
       int: long2jdsecondll,                                                   \
       long: long2jdsecondll,                                                  \
       double: sec_double2JDSecondLL)(a)
#endif
#define ToJDSecond(a) jdsecll2jdsec(ToJDSecondLL(a))

// first argument is always JDType
// #define JDMult(x, a)
//    _Generic((a),
//        double: _jdmult_dbl,
//        Rational: _jdmult_rat, \ RationalLL: _jdmult_ratll)(x, a)

// The operation 'z = a * x + y' for 'x' and 'y' being JDType and 'a' being
// a scalar Rational, RationalLL, or double
#define JDaxpy(a, x, y)                                                        \
   JDAddDays(                                                                  \
       JDMultDbl(JD_RAW((y).system, (y).epoch, (x).whole_days, (x).seconds),   \
                 a),                                                           \
       y)

__attribute__((const)) JDSecondLL sec_double2JDSecondLL(const double dbl);
__attribute__((const)) JDSecondLL day_double2JDSecondLL(const double dbl);
__attribute__((const)) JDSecond sec_double2JDSecond(const double dbl);
__attribute__((const)) JDSecond day_double2JDSecond(const double dbl);
__attribute__((const)) JDSecond jdsecond_iden(const JDSecond dbl);
__attribute__((const)) JDSecondLL rational2jdsecondLL(const Rational rat);
__attribute__((const)) JDSecondLL rationalll2jdsecondLL(const RationalLL ratll);
__attribute__((const)) JDType JDAddIntegerMultRatSecs(const JDType jd,
                                                      const long mul,
                                                      const Rational rat);
__attribute__((const)) JDType InitJD(const TimeSystem system,
                                     const EpochTT epoch, const long days,
                                     const JDSecond seconds);
__attribute__((pure)) double GetLeapSec(const JDType jd);
__attribute__((const)) double EpochValueTT(EpochTT epoch);
__attribute__((const)) JDType JDChangeEpoch(const EpochTT new_epoch, JDType jd);
__attribute__((const)) JDType JDChangeSystem(const TimeSystem new_system,
                                             JDType jd);
__attribute__((const)) JDType JDChangeSystemEpoch(const TimeSystem new_system,
                                                  const EpochTT new_epoch,
                                                  JDType jd);
__attribute__((const)) double JDToDays(const JDType jd);
__attribute__((const)) JDType JDFromDoubleDays(const double days,
                                               const TimeSystem system,
                                               const EpochTT new_epoch);
__attribute__((const)) JDType JDFromSeconds(const double seconds,
                                            const TimeSystem system,
                                            const EpochTT new_epoch);
__attribute__((const)) JDType JDFromRationalSeconds(const Rational seconds,
                                                    const TimeSystem system,
                                                    const EpochTT new_epoch);
__attribute__((const)) double JDToSeconds(JDType jd);
__attribute__((const)) Rational JDToRationalSeconds(JDType jd);
__attribute__((const)) double JDToTime(JDType jd);
__attribute__((const)) double JDToDynTime(JDType JD);

// First argument is always JDType
#define JDAddSeconds(a, b) JDAddJD((a), SecondsToJD((b), (a).system, (a).epoch))
// .. presume second argument is in units of days if not JDType
#define JDAddDays(a, b) JDAddJD((a), DaysToJD((b), (a).system, (a).epoch))
__attribute__((const)) JDType JDAddJD(const JDType a, const JDType b);

__attribute__((const)) JDType JDAddRationalMult(const JDType a,
                                                const Rational mul, JDType b);

// First argument is always JDType
#define JDSubSeconds(a, b) JDSubJD((a), SecondsToJD((b), (a).system, (a).epoch))
// .. presume second argument is in units of days if not JDType
#define JDSubDays(a, b) JDSubJD((a), DaysToJD((b), (a).system, (a).epoch))
__attribute__((const)) JDType JDSubJD(const JDType a, const JDType b);

__attribute__((const)) JDType JDSubRationalMult(const JDType a,
                                                const Rational mul, JDType b);

__attribute__((const)) JDType JDMultDbl(const JDType x, const double a);
// __attribute__((const)) JDType _jdmult_rat(const JDType x, const Rational a);
// __attribute__((const)) JDType _jdmult_ratll(const JDType x, const RationalLL
// a);
__attribute__((const)) double JDAddToDays(const JDType a, const JDType b);
__attribute__((const)) double JDAddToSeconds(const JDType a, const JDType b);
__attribute__((const)) double JDSubToDays(const JDType a, const JDType b);
__attribute__((const)) double JDSubToSeconds(const JDType a, const JDType b);

__attribute__((const)) double JDPODSeconds(const JDType);

__attribute__((const)) JDType JDAbs(JDType jd);
__attribute__((const)) int ispos_jd(JDType jd);
__attribute__((const)) JDType JDNegate(JDType jd);

__attribute__((const)) int isequal_jd_systemepoch(const JDType a,
                                                  const JDType b);
__attribute__((const)) int isequal_jd(const JDType a, const JDType b,
                                      const double epsilon);
__attribute__((const)) int isless_jd(const JDType a, const JDType b);
__attribute__((const)) int islessequal_jd(const JDType a, const JDType b,
                                          const double epsilon);
__attribute__((const)) int isgreater_jd(const JDType a, const JDType b);
__attribute__((const)) int isgreaterequal_jd(const JDType a, const JDType b,
                                             const double epsilon);

__attribute__((pure)) EpochTT str2epoch(char str[JDEPOCH_STR_LEN]);
__attribute__((pure)) TimeSystem str2system(char str[JDSYSTEM_STR_LEN]);
void epoch2str(EpochTT epoch, char str[JDEPOCH_STR_LEN]);
void system2str(TimeSystem system, char str[JDSYSTEM_STR_LEN]);
void jd2str(JDType jd, char str[JD_STR_LEN]);
void jdays2str(JDType jd, char str[JD_STR_LEN]);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __JDKIT_H__ */

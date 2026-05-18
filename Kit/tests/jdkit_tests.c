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
#include <criterion/criterion.h>
#include <criterion/new/assert.h>
#include <criterion/parameterized.h>
#include <criterion/theories.h>

#define SUITE_NAME jdkit

#define ULP_THRESH     (4) // acceptable Units in Last Place variation
#define DBL_THRESH     (ULP_THRESH * __DBL_EPSILON__)
#define COND_STR(cond) (cond) ? ("TRUE") : ("FALSE")

#define JD_DATAPOINTS                                                          \
   DataPoints(                                                                 \
       JDType *, &JD_ZERO,                                                     \
                                                                               \
       &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),                         \
       &JD_RAW(TT_TIME, GD_CONV_EPOCH, 0, RATIONAL_ZERO),                      \
       &JD_RAW(TT_TIME, TCB_TDB_CONV_EPOCH, 0, RATIONAL_ZERO),                 \
       &JD_RAW(TT_TIME, MJD_EPOCH, 0, RATIONAL_ZERO),                          \
       &JD_RAW(TT_TIME, J1900_EPOCH, 0, RATIONAL_ZERO),                        \
       &JD_RAW(TT_TIME, GMAT_MJD_EPOCH, 0, RATIONAL_ZERO),                     \
       &JD_RAW(TT_TIME, CCSDS_EPOCH, 0, RATIONAL_ZERO),                        \
       &JD_RAW(TT_TIME, J2000_EPOCH, 0, RATIONAL_ZERO),                        \
                                                                               \
       &JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),                        \
       &JD_RAW(TAI_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),                        \
       &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),                         \
       &JD_RAW(TCB_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),                        \
       &JD_RAW(TDB_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),                        \
                                                                               \
       &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(32, 184, 1000)),          \
       &JD_RAW(TT_TIME, GD_CONV_EPOCH, 0, RATIONAL_NGCD(32, 184, 1000)),       \
       &JD_RAW(TT_TIME, TCB_TDB_CONV_EPOCH, 0, RATIONAL_NGCD(32, 184, 1000)),  \
       &JD_RAW(TT_TIME, MJD_EPOCH, 0, RATIONAL_NGCD(32, 184, 1000)),           \
       &JD_RAW(TT_TIME, J1900_EPOCH, 0, RATIONAL_NGCD(32, 184, 1000)),         \
       &JD_RAW(TT_TIME, GMAT_MJD_EPOCH, 0, RATIONAL_NGCD(32, 184, 1000)),      \
       &JD_RAW(TT_TIME, CCSDS_EPOCH, 0, RATIONAL_NGCD(32, 184, 1000)),         \
       &JD_RAW(TT_TIME, J2000_EPOCH, 0, RATIONAL_NGCD(32, 184, 1000)),         \
                                                                               \
       &JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(32, 184, 1000)),         \
       &JD_RAW(TAI_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(32, 184, 1000)),         \
       &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(32, 184, 1000)),          \
       &JD_RAW(TCB_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(32, 184, 1000)),         \
       &JD_RAW(TDB_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(32, 184, 1000)),         \
                                                                               \
       &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(51, 184, 1000)),          \
       &JD_RAW(TT_TIME, GD_CONV_EPOCH, 0, RATIONAL_NGCD(51, 184, 1000)),       \
       &JD_RAW(TT_TIME, TCB_TDB_CONV_EPOCH, 0, RATIONAL_NGCD(51, 184, 1000)),  \
       &JD_RAW(TT_TIME, MJD_EPOCH, 0, RATIONAL_NGCD(51, 184, 1000)),           \
       &JD_RAW(TT_TIME, J1900_EPOCH, 0, RATIONAL_NGCD(51, 184, 1000)),         \
       &JD_RAW(TT_TIME, GMAT_MJD_EPOCH, 0, RATIONAL_NGCD(51, 184, 1000)),      \
       &JD_RAW(TT_TIME, CCSDS_EPOCH, 0, RATIONAL_NGCD(51, 184, 1000)),         \
       &JD_RAW(TT_TIME, J2000_EPOCH, 0, RATIONAL_NGCD(51, 184, 1000)),         \
                                                                               \
       &JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(51, 184, 1000)),         \
       &JD_RAW(TAI_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(51, 184, 1000)),         \
       &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(51, 184, 1000)),          \
       &JD_RAW(TCB_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(51, 184, 1000)),         \
       &JD_RAW(TDB_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(51, 184, 1000)),         \
                                                                               \
       &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),               \
       &JD_RAW(TT_TIME, GD_CONV_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),            \
       &JD_RAW(TT_TIME, TCB_TDB_CONV_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),       \
       &JD_RAW(TT_TIME, MJD_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),                \
       &JD_RAW(TT_TIME, J1900_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),              \
       &JD_RAW(TT_TIME, GMAT_MJD_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),           \
       &JD_RAW(TT_TIME, CCSDS_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),              \
       &JD_RAW(TT_TIME, J2000_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),              \
                                                                               \
       &JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),              \
       &JD_RAW(TAI_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),              \
       &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),               \
       &JD_RAW(TCB_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),              \
       &JD_RAW(TDB_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),              \
                                                                               \
       &JD_NREDUCE(ZERO_EPOCH, TT_TIME, 0.0),                                  \
       &JD_NREDUCE(ZERO_EPOCH, TT_TIME, 1721013.5),                            \
       &JD_NREDUCE(ZERO_EPOCH, TT_TIME, 2443144.5),                            \
       &JD_NREDUCE(ZERO_EPOCH, TT_TIME, 2400000.5),                            \
       &JD_NREDUCE(ZERO_EPOCH, TT_TIME, 2415019.5),                            \
       &JD_NREDUCE(ZERO_EPOCH, TT_TIME, 2430000.0),                            \
       &JD_NREDUCE(ZERO_EPOCH, TT_TIME, 2436204.5),                            \
       &JD_NREDUCE(ZERO_EPOCH, TT_TIME, 2451545.0),                            \
                                                                               \
       &JD_NREDUCE(ZERO_EPOCH, UTC_TIME, 0.0),                                 \
       &JD_NREDUCE(ZERO_EPOCH, UTC_TIME, 1721013.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, UTC_TIME, 2443144.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, UTC_TIME, 2400000.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, UTC_TIME, 2415019.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, UTC_TIME, 2430000.0),                           \
       &JD_NREDUCE(ZERO_EPOCH, UTC_TIME, 2436204.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, UTC_TIME, 2451545.0),                           \
                                                                               \
       &JD_NREDUCE(ZERO_EPOCH, TAI_TIME, 0.0),                                 \
       &JD_NREDUCE(ZERO_EPOCH, TAI_TIME, 1721013.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TAI_TIME, 2443144.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TAI_TIME, 2400000.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TAI_TIME, 2415019.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TAI_TIME, 2430000.0),                           \
       &JD_NREDUCE(ZERO_EPOCH, TAI_TIME, 2436204.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TAI_TIME, 2451545.0),                           \
                                                                               \
       &JD_NREDUCE(ZERO_EPOCH, TCB_TIME, 0.0),                                 \
       &JD_NREDUCE(ZERO_EPOCH, TCB_TIME, 1721013.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TCB_TIME, 2443144.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TCB_TIME, 2400000.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TCB_TIME, 2415019.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TCB_TIME, 2430000.0),                           \
       &JD_NREDUCE(ZERO_EPOCH, TCB_TIME, 2436204.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TCB_TIME, 2451545.0),                           \
                                                                               \
       &JD_NREDUCE(ZERO_EPOCH, TDB_TIME, 0.0),                                 \
       &JD_NREDUCE(ZERO_EPOCH, TDB_TIME, 1721013.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TDB_TIME, 2443144.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TDB_TIME, 2400000.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TDB_TIME, 2415019.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TDB_TIME, 2430000.0),                           \
       &JD_NREDUCE(ZERO_EPOCH, TDB_TIME, 2436204.5),                           \
       &JD_NREDUCE(ZERO_EPOCH, TDB_TIME, 2451545.0),                           \
                                                                               \
       &JD_NREDUCE(GD_CONV_EPOCH, TT_TIME, 0.0),                               \
       &JD_NREDUCE(GD_CONV_EPOCH, TT_TIME, 1721013.5),                         \
       &JD_NREDUCE(GD_CONV_EPOCH, TT_TIME, 2443144.5),                         \
       &JD_NREDUCE(GD_CONV_EPOCH, TT_TIME, 2400000.5),                         \
       &JD_NREDUCE(GD_CONV_EPOCH, TT_TIME, 2415019.5),                         \
       &JD_NREDUCE(GD_CONV_EPOCH, TT_TIME, 2430000.0),                         \
       &JD_NREDUCE(GD_CONV_EPOCH, TT_TIME, 2436204.5),                         \
       &JD_NREDUCE(GD_CONV_EPOCH, TT_TIME, 2451545.0),                         \
                                                                               \
       &JD_NREDUCE(TCB_TDB_CONV_EPOCH, TT_TIME, 0.0),                          \
       &JD_NREDUCE(TCB_TDB_CONV_EPOCH, TT_TIME, 1721013.5),                    \
       &JD_NREDUCE(TCB_TDB_CONV_EPOCH, TT_TIME, 2443144.5),                    \
       &JD_NREDUCE(TCB_TDB_CONV_EPOCH, TT_TIME, 2400000.5),                    \
       &JD_NREDUCE(TCB_TDB_CONV_EPOCH, TT_TIME, 2415019.5),                    \
       &JD_NREDUCE(TCB_TDB_CONV_EPOCH, TT_TIME, 2430000.0),                    \
       &JD_NREDUCE(TCB_TDB_CONV_EPOCH, TT_TIME, 2436204.5),                    \
       &JD_NREDUCE(TCB_TDB_CONV_EPOCH, TT_TIME, 2451545.0),                    \
                                                                               \
       &JD_NREDUCE(MJD_EPOCH, TT_TIME, 0.0),                                   \
       &JD_NREDUCE(MJD_EPOCH, TT_TIME, 1721013.5),                             \
       &JD_NREDUCE(MJD_EPOCH, TT_TIME, 2443144.5),                             \
       &JD_NREDUCE(MJD_EPOCH, TT_TIME, 2400000.5),                             \
       &JD_NREDUCE(MJD_EPOCH, TT_TIME, 2415019.5),                             \
       &JD_NREDUCE(MJD_EPOCH, TT_TIME, 2430000.0),                             \
       &JD_NREDUCE(MJD_EPOCH, TT_TIME, 2436204.5),                             \
       &JD_NREDUCE(MJD_EPOCH, TT_TIME, 2451545.0),                             \
                                                                               \
       &JD_NREDUCE(J1900_EPOCH, TT_TIME, 0.0),                                 \
       &JD_NREDUCE(J1900_EPOCH, TT_TIME, 1721013.5),                           \
       &JD_NREDUCE(J1900_EPOCH, TT_TIME, 2443144.5),                           \
       &JD_NREDUCE(J1900_EPOCH, TT_TIME, 2400000.5),                           \
       &JD_NREDUCE(J1900_EPOCH, TT_TIME, 2415019.5),                           \
       &JD_NREDUCE(J1900_EPOCH, TT_TIME, 2430000.0),                           \
       &JD_NREDUCE(J1900_EPOCH, TT_TIME, 2436204.5),                           \
       &JD_NREDUCE(J1900_EPOCH, TT_TIME, 2451545.0),                           \
                                                                               \
       &JD_NREDUCE(GMAT_MJD_EPOCH, TT_TIME, 0.0),                              \
       &JD_NREDUCE(GMAT_MJD_EPOCH, TT_TIME, 1721013.5),                        \
       &JD_NREDUCE(GMAT_MJD_EPOCH, TT_TIME, 2443144.5),                        \
       &JD_NREDUCE(GMAT_MJD_EPOCH, TT_TIME, 2400000.5),                        \
       &JD_NREDUCE(GMAT_MJD_EPOCH, TT_TIME, 2415019.5),                        \
       &JD_NREDUCE(GMAT_MJD_EPOCH, TT_TIME, 2430000.0),                        \
       &JD_NREDUCE(GMAT_MJD_EPOCH, TT_TIME, 2436204.5),                        \
       &JD_NREDUCE(GMAT_MJD_EPOCH, TT_TIME, 2451545.0),                        \
                                                                               \
       &JD_NREDUCE(CCSDS_EPOCH, TT_TIME, 0.0),                                 \
       &JD_NREDUCE(CCSDS_EPOCH, TT_TIME, 1721013.5),                           \
       &JD_NREDUCE(CCSDS_EPOCH, TT_TIME, 2443144.5),                           \
       &JD_NREDUCE(CCSDS_EPOCH, TT_TIME, 2400000.5),                           \
       &JD_NREDUCE(CCSDS_EPOCH, TT_TIME, 2415019.5),                           \
       &JD_NREDUCE(CCSDS_EPOCH, TT_TIME, 2430000.0),                           \
       &JD_NREDUCE(CCSDS_EPOCH, TT_TIME, 2436204.5),                           \
       &JD_NREDUCE(CCSDS_EPOCH, TT_TIME, 2451545.0),                           \
                                                                               \
       &JD_NREDUCE(J2000_EPOCH, TT_TIME, 0.0),                                 \
       &JD_NREDUCE(J2000_EPOCH, TT_TIME, 1721013.5),                           \
       &JD_NREDUCE(J2000_EPOCH, TT_TIME, 2443144.5),                           \
       &JD_NREDUCE(J2000_EPOCH, TT_TIME, 2400000.5),                           \
       &JD_NREDUCE(J2000_EPOCH, TT_TIME, 2415019.5),                           \
       &JD_NREDUCE(J2000_EPOCH, TT_TIME, 2430000.0),                           \
       &JD_NREDUCE(J2000_EPOCH, TT_TIME, 2436204.5),                           \
       &JD_NREDUCE(J2000_EPOCH, TT_TIME, 2451545.0), )

// define needed globals
//      path of model directory relative to executable
const char ModelPath[1000] = "Model/";

/* Test jd2str funcs */
TheoryDataPoints(SUITE_NAME, jd2str_test) = {JD_DATAPOINTS};

Theory((JDType * a), SUITE_NAME, jd2str_test)
{
   char jd_str[JD_STR_LEN]           = {'\0'};
   char epoch_str[JDEPOCH_STR_LEN]   = {'\0'};
   char system_str[JDSYSTEM_STR_LEN] = {'\0'};
   char sec_str[RATIONAL_STR_LEN]    = {'\0'};

   char *end_ptr;

   JDType jd_test = JD_ZERO;

   sec_str[0] = '0';
   sec_str[1] = '.';
   jddays2str(*a, jd_str);
   sscanf(jd_str, "JD %li.%[0-9] %[^,], Epoch %[^\n]\n", &jd_test.whole_days,
          &sec_str[2], system_str, epoch_str);
   jd_test.system  = str2system(system_str);
   jd_test.epoch   = str2epoch(epoch_str);
   double sec      = strtod(sec_str, &end_ptr) * 86400.0;
   jd_test.seconds = double2rational(sec);

   cr_assert(a->epoch == jd_test.epoch && a->system == jd_test.system &&
                 a->whole_days == jd_test.whole_days,
             "jddays2str() whole part error");
   if (sec == 0) {
      cr_assert(epsilon_eq(dbl, sec, rational2double(a->seconds), ULP_THRESH),
                "jddays2str() second error near zero");
   }
   else {
      cr_assert(ieee_ulp_eq(dbl, sec, rational2double(a->seconds), ULP_THRESH),
                "jddays2str() second error");
   }

   jd2str(*a, jd_str);
   sscanf(jd_str, "JD %li days, (%s) seconds, %s, Epoch %s",
          &jd_test.whole_days, sec_str, system_str, epoch_str);
   jd_test.system = str2system(system_str);
   jd_test.epoch  = str2epoch(epoch_str);
   sscanf(sec_str, "%ld + (%ld/%ld)", &jd_test.seconds.whole,
          &jd_test.seconds.num, &jd_test.seconds.den);

   cr_assert(a->epoch == jd_test.epoch && a->system == jd_test.system &&
                 a->whole_days == jd_test.whole_days &&
                 a->seconds.whole == jd_test.seconds.whole &&
                 a->seconds.num == jd_test.seconds.num &&
                 a->seconds.den == jd_test.seconds.den,
             "jd2str() error");
}

/* Test Conditionals */
struct jdcond_tuple {
   JDType a;
   JDType b;
   int iscond; // 0=equal, -1=isless, 1=isgreater
};

ParameterizedTestParameters(SUITE_NAME, conditional_test)
{
#define SIZE 13
   static int first                         = 0;
   static struct jdcond_tuple vals[SIZE]    = {0};
   const struct jdcond_tuple vals_nstatic[] = {
       {JD_ZERO, JD_ZERO, 0},
       {JD_RAW(TT_TIME, GD_CONV_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, GD_CONV_EPOCH, 0, RATIONAL_ZERO), 0},
       {JD_RAW(TT_TIME, GD_CONV_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, TCB_TDB_CONV_EPOCH, 0, RATIONAL_ZERO), 0},
       {JD_RAW(TT_TIME, TCB_TDB_CONV_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, MJD_EPOCH, 0, RATIONAL_ZERO), 1},
       {JD_RAW(TT_TIME, MJD_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, J1900_EPOCH, 0, RATIONAL_ZERO), 1},
       {JD_RAW(TT_TIME, J1900_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, GMAT_MJD_EPOCH, 0, RATIONAL_ZERO), 1},
       {JD_RAW(TT_TIME, GMAT_MJD_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, CCSDS_EPOCH, 0, RATIONAL_ZERO), 1},
       {JD_RAW(TT_TIME, CCSDS_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, J2000_EPOCH, 0, RATIONAL_ZERO), 1},
       {JD_RAW(TT_TIME, J2000_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 1},
       {JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 0},
       {JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TAI_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 0},
       {JD_RAW(TAI_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 1},
       {JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TCB_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 1},
       {JD_RAW(TCB_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TDB_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 1},
       {JD_RAW(TDB_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 1},
       {JD_RAW(TT_TIME, ZERO_EPOCH, 0.0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, ZERO_EPOCH, 1721013.5, RATIONAL_ZERO), -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 1721013.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2400000.5), -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2400000.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2415019.5), -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2415019.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2430000.0), -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2430000.0),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2436204.5), -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2436204.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2443144.5), -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2443144.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2451545.0), -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2451545.0),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 0.0), 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 0.0),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2451545.0), -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 1721013.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 0.0), 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2400000.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 1721013.5), 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2415019.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2400000.5), 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2430000.0),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2415019.5), 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2436204.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2430000.0), 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2443144.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2436204.5), 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2451545.0),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2443144.5), 1},

   };
   if (!first) {
      first = 1;
      for (size_t i = 0; i < SIZE; i++) {
         vals[i] = vals[i];
      }
   }
   return cr_make_param_array(struct jdcond_tuple, vals, SIZE);
#undef SIZE
}

ParameterizedTest(struct jdcond_tuple *val, SUITE_NAME, conditional_test)
{
   char a_str[JD_STR_LEN] = {'\0'}, b_str[JD_STR_LEN] = {'\0'};
   jd2str(val->a, a_str);
   jd2str(val->b, b_str);

   if (val->iscond == 0)
      cr_assert(isequal_jd(val->a, val->b),
                "(%s != %s) when they should be equal", a_str, b_str);
   else
      cr_assert(not(isequal_jd(val->a, val->b)),
                "(%s == %s) when they should not be equal", a_str, b_str);

   // TODO: make conditionals only need the same system
   cr_assume(val->a.system == val->b.system && val->a.epoch == val->b.epoch);
   if (val->iscond == -1)
      cr_assert(isless_jd(val->a, val->b),
                "(%s < %s) is not true, when it should be false.", a_str,
                b_str);
   else
      cr_assert(isgreaterequal_jd(val->a, val->b),
                "(%s >= %s) is not true, when it should be false.", a_str,
                b_str);

   if (val->iscond == 1)
      cr_assert(isgreater_jd(val->a, val->b),
                "(%s > %s) is not true, when it should be false.", a_str,
                b_str);
   else
      cr_assert(islessequal_jd(val->a, val->b),
                "(%s <= %s) is not true, when it should be false.", a_str,
                b_str);
}

/* Test math operation properties */
//*** Addition
TheoryDataPoints(SUITE_NAME, add_test) = {JD_DATAPOINTS, JD_DATAPOINTS};

Theory((JDType * a, JDType *b), SUITE_NAME, add_test) {}

//*** Multiplication
TheoryDataPoints(SUITE_NAME, mult_test) = {JD_DATAPOINTS, JD_DATAPOINTS};

Theory((JDType * a, JDType *b), SUITE_NAME, mult_test) {}
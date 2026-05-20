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
#include "utilkit.h"
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
       &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),            \
       &JD_RAW(TT_TIME, GD_CONV_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),         \
       &JD_RAW(TT_TIME, TCB_TDB_CONV_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),    \
       &JD_RAW(TT_TIME, MJD_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),             \
       &JD_RAW(TT_TIME, J1900_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),           \
       &JD_RAW(TT_TIME, GMAT_MJD_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),        \
       &JD_RAW(TT_TIME, CCSDS_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),           \
       &JD_RAW(TT_TIME, J2000_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),           \
                                                                               \
       &JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),           \
       &JD_RAW(TAI_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),           \
       &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),            \
       &JD_RAW(TCB_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),           \
       &JD_RAW(TDB_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),           \
                                                                               \
       &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),            \
       &JD_RAW(TT_TIME, GD_CONV_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),         \
       &JD_RAW(TT_TIME, TCB_TDB_CONV_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),    \
       &JD_RAW(TT_TIME, MJD_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),             \
       &JD_RAW(TT_TIME, J1900_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),           \
       &JD_RAW(TT_TIME, GMAT_MJD_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),        \
       &JD_RAW(TT_TIME, CCSDS_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),           \
       &JD_RAW(TT_TIME, J2000_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),           \
                                                                               \
       &JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),           \
       &JD_RAW(TAI_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),           \
       &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),            \
       &JD_RAW(TCB_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),           \
       &JD_RAW(TDB_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),           \
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
       &JD_NREDUCE(TT_TIME, ZERO_EPOCH, 0.0),                                  \
       &JD_NREDUCE(TT_TIME, ZERO_EPOCH, 1721013.5),                            \
       &JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2443144.5),                            \
       &JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2400000.5),                            \
       &JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2415019.5),                            \
       &JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2430000.0),                            \
       &JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2436204.5),                            \
       &JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2451545.0),                            \
                                                                               \
       &JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 0.0),                                 \
       &JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 1721013.5),                           \
       &JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2443144.5),                           \
       &JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2400000.5),                           \
       &JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2415019.5),                           \
       &JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2430000.0),                           \
       &JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2436204.5),                           \
       &JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2451545.0),                           \
                                                                               \
       &JD_NREDUCE(TAI_TIME, ZERO_EPOCH, 0.0),                                 \
       &JD_NREDUCE(TAI_TIME, ZERO_EPOCH, 1721013.5),                           \
       &JD_NREDUCE(TAI_TIME, ZERO_EPOCH, 2443144.5),                           \
       &JD_NREDUCE(TAI_TIME, ZERO_EPOCH, 2400000.5),                           \
       &JD_NREDUCE(TAI_TIME, ZERO_EPOCH, 2415019.5),                           \
       &JD_NREDUCE(TAI_TIME, ZERO_EPOCH, 2430000.0),                           \
       &JD_NREDUCE(TAI_TIME, ZERO_EPOCH, 2436204.5),                           \
       &JD_NREDUCE(TAI_TIME, ZERO_EPOCH, 2451545.0),                           \
                                                                               \
       &JD_NREDUCE(TCB_TIME, ZERO_EPOCH, 0.0),                                 \
       &JD_NREDUCE(TCB_TIME, ZERO_EPOCH, 1721013.5),                           \
       &JD_NREDUCE(TCB_TIME, ZERO_EPOCH, 2443144.5),                           \
       &JD_NREDUCE(TCB_TIME, ZERO_EPOCH, 2400000.5),                           \
       &JD_NREDUCE(TCB_TIME, ZERO_EPOCH, 2415019.5),                           \
       &JD_NREDUCE(TCB_TIME, ZERO_EPOCH, 2430000.0),                           \
       &JD_NREDUCE(TCB_TIME, ZERO_EPOCH, 2436204.5),                           \
       &JD_NREDUCE(TCB_TIME, ZERO_EPOCH, 2451545.0),                           \
                                                                               \
       &JD_NREDUCE(TDB_TIME, ZERO_EPOCH, 0.0),                                 \
       &JD_NREDUCE(TDB_TIME, ZERO_EPOCH, 1721013.5),                           \
       &JD_NREDUCE(TDB_TIME, ZERO_EPOCH, 2443144.5),                           \
       &JD_NREDUCE(TDB_TIME, ZERO_EPOCH, 2400000.5),                           \
       &JD_NREDUCE(TDB_TIME, ZERO_EPOCH, 2415019.5),                           \
       &JD_NREDUCE(TDB_TIME, ZERO_EPOCH, 2430000.0),                           \
       &JD_NREDUCE(TDB_TIME, ZERO_EPOCH, 2436204.5),                           \
       &JD_NREDUCE(TDB_TIME, ZERO_EPOCH, 2451545.0),                           \
                                                                               \
       &JD_NREDUCE(TT_TIME, GD_CONV_EPOCH, 0.0),                               \
       &JD_NREDUCE(TT_TIME, GD_CONV_EPOCH, 1721013.5),                         \
       &JD_NREDUCE(TT_TIME, GD_CONV_EPOCH, 2443144.5),                         \
       &JD_NREDUCE(TT_TIME, GD_CONV_EPOCH, 2400000.5),                         \
       &JD_NREDUCE(TT_TIME, GD_CONV_EPOCH, 2415019.5),                         \
       &JD_NREDUCE(TT_TIME, GD_CONV_EPOCH, 2430000.0),                         \
       &JD_NREDUCE(TT_TIME, GD_CONV_EPOCH, 2436204.5),                         \
       &JD_NREDUCE(TT_TIME, GD_CONV_EPOCH, 2451545.0),                         \
                                                                               \
       &JD_NREDUCE(TT_TIME, TCB_TDB_CONV_EPOCH, 0.0),                          \
       &JD_NREDUCE(TT_TIME, TCB_TDB_CONV_EPOCH, 1721013.5),                    \
       &JD_NREDUCE(TT_TIME, TCB_TDB_CONV_EPOCH, 2443144.5),                    \
       &JD_NREDUCE(TT_TIME, TCB_TDB_CONV_EPOCH, 2400000.5),                    \
       &JD_NREDUCE(TT_TIME, TCB_TDB_CONV_EPOCH, 2415019.5),                    \
       &JD_NREDUCE(TT_TIME, TCB_TDB_CONV_EPOCH, 2430000.0),                    \
       &JD_NREDUCE(TT_TIME, TCB_TDB_CONV_EPOCH, 2436204.5),                    \
       &JD_NREDUCE(TT_TIME, TCB_TDB_CONV_EPOCH, 2451545.0),                    \
                                                                               \
       &JD_NREDUCE(TT_TIME, MJD_EPOCH, 0.0),                                   \
       &JD_NREDUCE(TT_TIME, MJD_EPOCH, 1721013.5),                             \
       &JD_NREDUCE(TT_TIME, MJD_EPOCH, 2443144.5),                             \
       &JD_NREDUCE(TT_TIME, MJD_EPOCH, 2400000.5),                             \
       &JD_NREDUCE(TT_TIME, MJD_EPOCH, 2415019.5),                             \
       &JD_NREDUCE(TT_TIME, MJD_EPOCH, 2430000.0),                             \
       &JD_NREDUCE(TT_TIME, MJD_EPOCH, 2436204.5),                             \
       &JD_NREDUCE(TT_TIME, MJD_EPOCH, 2451545.0),                             \
                                                                               \
       &JD_NREDUCE(TT_TIME, J1900_EPOCH, 0.0),                                 \
       &JD_NREDUCE(TT_TIME, J1900_EPOCH, 1721013.5),                           \
       &JD_NREDUCE(TT_TIME, J1900_EPOCH, 2443144.5),                           \
       &JD_NREDUCE(TT_TIME, J1900_EPOCH, 2400000.5),                           \
       &JD_NREDUCE(TT_TIME, J1900_EPOCH, 2415019.5),                           \
       &JD_NREDUCE(TT_TIME, J1900_EPOCH, 2430000.0),                           \
       &JD_NREDUCE(TT_TIME, J1900_EPOCH, 2436204.5),                           \
       &JD_NREDUCE(TT_TIME, J1900_EPOCH, 2451545.0),                           \
                                                                               \
       &JD_NREDUCE(TT_TIME, GMAT_MJD_EPOCH, 0.0),                              \
       &JD_NREDUCE(TT_TIME, GMAT_MJD_EPOCH, 1721013.5),                        \
       &JD_NREDUCE(TT_TIME, GMAT_MJD_EPOCH, 2443144.5),                        \
       &JD_NREDUCE(TT_TIME, GMAT_MJD_EPOCH, 2400000.5),                        \
       &JD_NREDUCE(TT_TIME, GMAT_MJD_EPOCH, 2415019.5),                        \
       &JD_NREDUCE(TT_TIME, GMAT_MJD_EPOCH, 2430000.0),                        \
       &JD_NREDUCE(TT_TIME, GMAT_MJD_EPOCH, 2436204.5),                        \
       &JD_NREDUCE(TT_TIME, GMAT_MJD_EPOCH, 2451545.0),                        \
                                                                               \
       &JD_NREDUCE(TT_TIME, CCSDS_EPOCH, 0.0),                                 \
       &JD_NREDUCE(TT_TIME, CCSDS_EPOCH, 1721013.5),                           \
       &JD_NREDUCE(TT_TIME, CCSDS_EPOCH, 2443144.5),                           \
       &JD_NREDUCE(TT_TIME, CCSDS_EPOCH, 2400000.5),                           \
       &JD_NREDUCE(TT_TIME, CCSDS_EPOCH, 2415019.5),                           \
       &JD_NREDUCE(TT_TIME, CCSDS_EPOCH, 2430000.0),                           \
       &JD_NREDUCE(TT_TIME, CCSDS_EPOCH, 2436204.5),                           \
       &JD_NREDUCE(TT_TIME, CCSDS_EPOCH, 2451545.0),                           \
                                                                               \
       &JD_NREDUCE(TT_TIME, J2000_EPOCH, 0.0),                                 \
       &JD_NREDUCE(TT_TIME, J2000_EPOCH, 1721013.5),                           \
       &JD_NREDUCE(TT_TIME, J2000_EPOCH, 2443144.5),                           \
       &JD_NREDUCE(TT_TIME, J2000_EPOCH, 2400000.5),                           \
       &JD_NREDUCE(TT_TIME, J2000_EPOCH, 2415019.5),                           \
       &JD_NREDUCE(TT_TIME, J2000_EPOCH, 2430000.0),                           \
       &JD_NREDUCE(TT_TIME, J2000_EPOCH, 2436204.5),                           \
       &JD_NREDUCE(TT_TIME, J2000_EPOCH, 2451545.0), )

#define JD_SECONDS_DATAPOINTS                                                  \
   DataPoints(JDType *,                                                        \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(51, 23, 125)),     \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(-51, -23, 125)),   \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(32, 23, 125)),     \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(-32, -23, 125)),   \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(19, 0, 1)),        \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(-19, 0, 1)),       \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, 1, 10)),        \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, -1, 10)),       \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, 1, 50)),        \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, -1, 50)),       \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, 1, 100)),       \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, -1, 100)),      \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, 1, 10000)),     \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, -1, 10000)),    \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, 1, 64)),        \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, -1, 64)),       \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, 1, 1024)),      \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, -1, 1024)),     \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, 1, 4096)),      \
              &JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_NGCD(0, -1, 4096)), )

/* Configure Suite                                                    */
// define needed globals
//      path of model directory relative to executable
char ModelPath[1000] = {'\0'};
void modelpath_init(void)
{
   // configure ModelPath
   GetExecDir(ModelPath);
   const char *rel_model_path = "/../../Model/\0";
   strcat(ModelPath, rel_model_path);
}

TestSuite(SUITE_NAME, .init = modelpath_init);

/* Test jd2str funcs */
TheoryDataPoints(SUITE_NAME, jd2str) = {JD_DATAPOINTS};

Theory((JDType * a), SUITE_NAME, jd2str)
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
      cr_expect(epsilon_eq(dbl, sec, rational2double(a->seconds), DBL_THRESH),
                "jddays2str() second error near zero");
   }
   else {
      cr_expect(ieee_ulp_eq(dbl, sec, rational2double(a->seconds), ULP_THRESH),
                "jddays2str() second error");
   }

   jd2str(*a, jd_str);
   sscanf(jd_str, "JD %li days, (%[0-9 +()/]) seconds, %s, Epoch %s",
          &jd_test.whole_days, sec_str, system_str, epoch_str);
   jd_test.system = str2system(system_str);
   jd_test.epoch  = str2epoch(epoch_str);
   sscanf(sec_str, "%ld + (%ld/%ld)", &jd_test.seconds.whole,
          &jd_test.seconds.num, &jd_test.seconds.den);

   cr_expect(a->epoch == jd_test.epoch && a->system == jd_test.system &&
                 a->whole_days == jd_test.whole_days &&
                 a->seconds.whole == jd_test.seconds.whole &&
                 a->seconds.num == jd_test.seconds.num &&
                 a->seconds.den == jd_test.seconds.den,
             "jd2str() error for string: %s", jd_str);
}

/* Test Conditionals                                                  */
struct jdcond_tuple {
   JDType a;
   JDType b;
   int isequal; // 1 = equal
   int islegr;  // -1 = isless, 1 = isgreater
};

ParameterizedTestParameters(SUITE_NAME, conditional)
{
#define SIZE 31
   static int first                         = 0;
   static struct jdcond_tuple vals[SIZE]    = {0};
   const struct jdcond_tuple vals_nstatic[] = {
       {JD_ZERO, JD_ZERO, 1, 0},
       {JD_RAW(TT_TIME, GD_CONV_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, GD_CONV_EPOCH, 0, RATIONAL_ZERO), 1, 0},
       {JD_RAW(TT_TIME, GD_CONV_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, TCB_TDB_CONV_EPOCH, 0, RATIONAL_ZERO), 0, 0},
       {JD_RAW(TT_TIME, TCB_TDB_CONV_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, MJD_EPOCH, 0, RATIONAL_ZERO), 0, 0},
       {JD_RAW(TT_TIME, MJD_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, J1900_EPOCH, 0, RATIONAL_ZERO), 0, 0},
       {JD_RAW(TT_TIME, J1900_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, GMAT_MJD_EPOCH, 0, RATIONAL_ZERO), 0, 0},
       {JD_RAW(TT_TIME, GMAT_MJD_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, CCSDS_EPOCH, 0, RATIONAL_ZERO), 0, 0},
       {JD_RAW(TT_TIME, CCSDS_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, J2000_EPOCH, 0, RATIONAL_ZERO), 0, 0},
       {JD_RAW(TT_TIME, J2000_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 0, 0},
       {JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 1, 0},
       {JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TAI_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 0, 0},
       {JD_RAW(TAI_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 0, 0},
       {JD_RAW(TT_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TCB_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 0, 0},
       {JD_RAW(TCB_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(TDB_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 0, 0},
       {JD_RAW(TDB_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO),
        JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 0, 0},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 0.0),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 1721013.5), 0, -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 1721013.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2400000.5), 0, -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2400000.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2415019.5), 0, -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2415019.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2430000.0), 0, -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2430000.0),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2436204.5), 0, -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2436204.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2443144.5), 0, -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2443144.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2451545.0), 0, -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2451545.0),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 0.0), 0, 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 0.0),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2451545.0), 0, -1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 1721013.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 0.0), 0, 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2400000.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 1721013.5), 0, 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2415019.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2400000.5), 0, 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2430000.0),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2415019.5), 0, 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2436204.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2430000.0), 0, 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2443144.5),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2436204.5), 0, 1},
       {JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2451545.0),
        JD_NREDUCE(TT_TIME, ZERO_EPOCH, 2443144.5), 0, 1},
   };
   if (!first) {
      first = 1;
      for (size_t i = 0; i < SIZE; i++)
         vals[i] = vals_nstatic[i];
   }
   return cr_make_param_array(struct jdcond_tuple, vals, SIZE);
#undef SIZE
}

ParameterizedTest(struct jdcond_tuple *val, SUITE_NAME, conditional)
{
   char a_str[JD_STR_LEN] = {'\0'}, b_str[JD_STR_LEN] = {'\0'};
   jd2str(val->a, a_str);
   jd2str(val->b, b_str);

   if (val->isequal == 1) {
      cr_expect(isequal_jd(val->a, val->b),
                "(%s != %s) when they should be equal", a_str, b_str);
      cr_expect(islessequal_jd(val->a, val->b),
                "(%s <= %s) is not true, when it should be false.", a_str,
                b_str);
      cr_expect(isgreaterequal_jd(val->a, val->b),
                "(%s >= %s) is not true, when it should be false.", a_str,
                b_str);
   }
   else
      cr_expect(not(isequal_jd(val->a, val->b)),
                "(%s == %s) when they should not be equal", a_str, b_str);

   // TODO: make conditionals only need the same system
   if (val->a.system == val->b.system && val->a.epoch == val->b.epoch) {
      if (val->islegr == -1) {
         cr_expect(isless_jd(val->a, val->b),
                   "(%s < %s) is not true, when it should be false.", a_str,
                   b_str);
         cr_expect(islessequal_jd(val->a, val->b),
                   "(%s <= %s) is not true, when it should be false.", a_str,
                   b_str);
      }

      if (val->islegr == 1) {
         cr_expect(isgreater_jd(val->a, val->b),
                   "(%s > %s) is not true, when it should be false.", a_str,
                   b_str);
         cr_expect(isgreaterequal_jd(val->a, val->b),
                   "(%s >= %s) is not true, when it should be false.", a_str,
                   b_str);
      }
   }
}

/* Test math operation properties                                     */
//*** JD Addition
TheoryDataPoints(SUITE_NAME, add) = {JD_DATAPOINTS, JD_DATAPOINTS};

Theory((JDType * a, JDType *b), SUITE_NAME, add)
{
   char a_str[JD_STR_LEN] = {'\0'}, b_str[JD_STR_LEN] = {'\0'};
   jd2str(*a, a_str);
   jd2str(*b, b_str);

   // addition inversion
   cr_assume((b->epoch == ZERO_EPOCH || a->epoch == b->epoch) &&
             (a->system == b->system));
   cr_expect(isequal_jd(*a, JDAdd(JDSub(*a, *b), *b)),
             "JDAdd is not invertable (a: (%s); b: (%s))", a_str, b_str);

   // commutative addition
   cr_assert(isequal_jd(JDAdd(*a, *b), JDAdd(*b, *a)),
             "JDAdd is not commutative (a: (%s); b:( %s))", a_str, b_str);
}

//*** conversion to days
TheoryDataPoints(SUITE_NAME, jd2dayinv) = {JD_DATAPOINTS};

Theory((JDType * a), SUITE_NAME, jd2dayinv)
{
   char a_str[JD_STR_LEN] = {'\0'};
   jd2str(*a, a_str);

   double jdday  = JDToDays(*a);
   double thresh = 1 * (nextafter(jdday, INFINITY) - jdday);
   cr_expect(
       epsilon_eq(dbl,
                  JDToDays(JDSub(*a, JDFromDays(jdday, a->system, a->epoch))),
                  0, thresh),
       "JDToDays/JDFromDays is not invertible (a: (%s))", a_str);
}

//*** JD Addition with days
TheoryDataPoints(SUITE_NAME, dayadd) = {JD_DATAPOINTS, JD_DATAPOINTS};

Theory((JDType * a, JDType *b), SUITE_NAME, dayadd)
{
   char a_str[JD_STR_LEN] = {'\0'}, b_str[JD_STR_LEN] = {'\0'};
   jd2str(*a, a_str);
   jd2str(*b, b_str);

   double a_days = JDToDays(*a);
   double b_days = JDToDays(*b);

   // addition day inversion
   cr_expect(
       isequal_jd(*a, JDAddDays(JDSubDays(*a, b_days), b_days)),
       "JDAddDays is not invertable with JDSubDays (a: (%s); b:( %s), %lf)",
       a_str, b_str, b_days);

   // commutative day addition
   cr_assume((a->epoch == b->epoch || a->epoch == ZERO_EPOCH ||
              b->epoch == ZERO_EPOCH) &&
             a->system == b->system);
   double thresh = MAX(nextafter(a_days, INFINITY) - a_days,
                       nextafter(b_days, INFINITY) - b_days);
   cr_assert(epsilon_eq(
                 dbl, JDSubToDays(JDAddDays(*a, b_days), JDAddDays(*b, a_days)),
                 0, thresh),
             "JDAddDays is not commutative (a: (%s), %lf; b:( %s), %lf)", a_str,
             a_days, b_str, b_days);
}

//*** conversion to seconds
TheoryDataPoints(SUITE_NAME, jd2secinv) = {JD_DATAPOINTS};

Theory((JDType * a), SUITE_NAME, jd2secinv)
{
   char a_str[JD_STR_LEN] = {'\0'};
   jd2str(*a, a_str);

   double jdsec  = JDToSeconds(*a);
   double thresh = 1 * (nextafter(jdsec, INFINITY) - jdsec);
   cr_expect(
       epsilon_eq(dbl,
                  JDSubToSeconds(*a, JDFromSeconds(jdsec, a->system, a->epoch)),
                  0, thresh),
       "JDToSeconds/JDFromSeconds is not invertible (a: (%s))", a_str);
}

//*** JD Addition with seconds
TheoryDataPoints(SUITE_NAME, secadd) = {JD_DATAPOINTS, JD_DATAPOINTS};

Theory((JDType * a, JDType *b), SUITE_NAME, secadd)
{
   char a_str[JD_STR_LEN] = {'\0'}, b_str[JD_STR_LEN] = {'\0'};
   jd2str(*a, a_str);
   jd2str(*b, b_str);

   double a_sec = JDToSeconds(*a);
   double b_sec = JDToSeconds(*b);

   // addition second inversion
   cr_expect(isequal_jd(*a, JDAddSeconds(JDSubSeconds(*a, b_sec), b_sec)),
             "JDAddSeconds is not invertable with JDSubSeconds (a: (%s); b:( "
             "%s), %lf)",
             a_str, b_str, b_sec);

   cr_assume((a->epoch == b->epoch || a->epoch == ZERO_EPOCH ||
              b->epoch == ZERO_EPOCH) &&
             a->system == b->system);
   double thresh = MAX(nextafter(a_sec, INFINITY) - a_sec,
                       nextafter(b_sec, INFINITY) - b_sec);
   // commutative second addition
   cr_assert(epsilon_eq(dbl,
                        JDSubToSeconds(JDAddSeconds(*a, b_sec),
                                       JDAddSeconds(*b, a_sec)),
                        0, thresh),
             "JDAddSeconds is not commutative (a: (%s), %lf; b:(%s), %lf)",
             a_str, a_sec, b_str, b_sec);
}

//*** conversion to rational seconds
TheoryDataPoints(SUITE_NAME, jd2ratsecinv) = {JD_DATAPOINTS};

Theory((JDType * a), SUITE_NAME, jd2ratsecinv)
{
   char a_str[JD_STR_LEN] = {'\0'};
   jd2str(*a, a_str);

   Rational jdratsec = JDToRationalSeconds(*a);
   cr_expect(
       isequal_jd(*a, JDFromRationalSeconds(jdratsec, a->system, a->epoch)),
       "JDToSeconds/JDFromSeconds is not invertible (a: (%s))", a_str);
}

//*** JD Addition with Rational seconds
TheoryDataPoints(SUITE_NAME, secratadd) = {JD_DATAPOINTS, JD_DATAPOINTS};

Theory((JDType * a, JDType *b), SUITE_NAME, secratadd)
{
   char a_str[JD_STR_LEN] = {'\0'}, b_str[JD_STR_LEN] = {'\0'};
   jd2str(*a, a_str);
   jd2str(*b, b_str);

   Rational a_sec_rat = JDToRationalSeconds(*a);
   Rational b_sec_rat = JDToRationalSeconds(*b);

   // addition rational second inversion
   cr_expect(
       isequal_jd(*a, JDAddRationalSeconds(JDSubRationalSeconds(*a, b_sec_rat),
                                           b_sec_rat)),
       "JDAddRationalSeconds is not invertable with JDSubRationalSeconds (a: "
       "(%s); b:( %s))",
       a_str, b_str);

   // commutative rational second addition
   cr_assume(a->epoch == b->epoch && a->system == b->system);
   cr_assert(isequal_jd(JDAddRationalSeconds(*a, b_sec_rat),
                        JDAddRationalSeconds(*b, a_sec_rat)),
             "JDAddRationalSeconds is not commutative (a: (%s); b:( %s))",
             a_str, b_str);
}

/* Epoch/System Conversion                                            */

//*** Epoch conversions
TheoryDataPoints(SUITE_NAME, epoch) = {JD_DATAPOINTS};

Theory((JDType * a), SUITE_NAME, epoch)
{
   char a_str[JD_STR_LEN] = {'\0'};
   jd2str(*a, a_str);

   EpochTT new_epoch = (a->epoch + 2) % (J2000_EPOCH + 1);

   // ensure epoch changes are reversible
   JDType jd = *a;
   JDChangeEpoch(new_epoch, &jd);
   JDChangeEpoch(a->epoch, &jd);

   cr_expect(isequal_jd(*a, jd),
             "Converting to different epoch and back did not preserve %s",
             a_str);
}

//*** System conversions
TheoryDataPoints(SUITE_NAME, system) = {JD_DATAPOINTS};

Theory((JDType * a), SUITE_NAME, system)
{
   // converting out of TCB is not implemented
   cr_assume(a->system != TCB_TIME);
   // TDB2TCB algorithm is approximate, ESPECIALLY when going back and forth
   cr_assume(a->system != TDB_TIME);

   char a_str[JD_STR_LEN] = {'\0'};
   jd2str(*a, a_str);

   TimeSystem new_system = (a->system + 2) % (TDB_TIME + 1);
   while (new_system == TCB_TIME || new_system == TDB_TIME)
      new_system = (new_system + 1) % (TDB_TIME + 1);

   // ensure system changes are approximately reversible
   JDType jd = *a;
   JDChangeSystem(new_system, &jd);
   double sec_tmp = JDToSeconds(jd);
   JDChangeSystem(a->system, &jd);

   const double a_sec  = JDToSeconds(*a);
   const double jd_sec = JDToSeconds(jd);

   if (a_sec == 0) {

      cr_expect(epsilon_eq(dbl, jd_sec, a_sec, DBL_THRESH),
                "Converting to different system and back did not preserve %s",
                a_str);
   }
   else {
      cr_expect(ieee_ulp_eq(dbl, jd_sec, a_sec, ULP_THRESH),
                "Converting to different system and back did not preserve %s",
                a_str);
   }
}

//*** both together
TheoryDataPoints(SUITE_NAME, epochsystem) = {JD_DATAPOINTS};

Theory((JDType * a), SUITE_NAME, epochsystem)
{
   // converting out of TCB is not implemented
   cr_assume(a->system != TCB_TIME);
   char a_str[JD_STR_LEN] = {'\0'};
   jd2str(*a, a_str);

   EpochTT new_epoch     = (a->epoch + 2) % (J2000_EPOCH + 1);
   TimeSystem new_system = (a->system + 2) % (TDB_TIME + 1);

   // check system/epoch changes work in either order
   JDType epsys_test = *a;
   JDChangeEpoch(new_epoch, &epsys_test);
   JDChangeSystem(new_system, &epsys_test);

   JDType sysep_test = *a;
   JDChangeSystem(new_system, &sysep_test);
   JDChangeEpoch(new_epoch, &sysep_test);

   cr_expect(isequal_jd(epsys_test, sysep_test),
             "Switching the order of changing system/epoch matters for %s",
             a_str);

   // test that the combined function also stays the same
   JDType combined_test = *a;
   JDChangeSystemEpoch(new_system, new_epoch, &combined_test);
   cr_assert(isequal_jd(epsys_test, combined_test) &&
                 isequal_jd(sysep_test, combined_test),
             "The combined function does not match individual epoch/system "
             "changing behavior for %s",
             a_str);
}

//*** Negation
TheoryDataPoints(SUITE_NAME, negation) = {JD_DATAPOINTS};

Theory((JDType * a), SUITE_NAME, negation)
{
   char a_str[JD_STR_LEN] = {'\0'};
   jd2str(*a, a_str);

   cr_assume(a->whole_days >= -_RATLONG_MAX_ &&
             a->seconds.whole >= -_RATLONG_MAX_);

   // correct
   JDType jd             = JDNegate(*a);
   JDType a_test         = *a;
   a_test.whole_days    *= -1;
   a_test.seconds.whole *= -1;
   a_test.seconds.num   *= -1;
   cr_expect(isequal_jd(jd, a_test), "JDNegate is not correct for %s", a_str);

   // inversion
   jd         = JDNegate(*a);
   JDType jd2 = JDNegate(jd);
   cr_expect(isequal_jd(*a, jd2), "JDNegate is not its own inverse for %s",
             a_str);
}

//*** JDaxpy
TheoryDataPoints(SUITE_NAME, jdaxpy) = {
    JD_SECONDS_DATAPOINTS, JD_DATAPOINTS,
    DataPoints(double, 0, 1, 2, -1, -2, 0.5, -0.5, 1 / 64, -1 / 64, 1 / 1024,
               -1 / 1024, _RATLONG_MAX_, -_RATLONG_MAX_, _RATLONG_MIN_, 32.184,
               -32.184, 19, -19, 51.184, -51.184)};

Theory((JDType * x, JDType *y, long a), SUITE_NAME, jdaxpy)
{
   char x_str[JD_STR_LEN] = {'\0'}, y_str[JD_STR_LEN] = {'\0'};
   jd2str(*x, x_str);
   jd2str(*y, y_str);

   // check
   JDType z = JDaxpy(a, *x, *y);
   if (a == 0) {
      cr_expect(isequal_jd(*y, z), "JDaxpy with a=0 did not preserve %s",
                y_str);
   }
   else {
      // idk here
   }

   // commutative
   JDType zero = JD_ZERO;
   zero.system = y->system;
   zero.epoch  = y->epoch;
   JDType ax   = JDaxpy(a, *x, zero);
   JDType ypax = JDaxpy(1, *y, ax);
   cr_expect(
       isequal_jd(z, ypax),
       "JDaxpy is not commutative with params:\n\ta = %lf\n\tx = %s\n\ty = %s",
       a, x_str, y_str);

   // inversion
   cr_assume(a >= -_RATLONG_MAX_); // -_RATLONG_MIN_ > _RATLONG_MAX_
   JDType z_inv = JDaxpy(-a, *x, z);
   cr_expect(
       isequal_jd(*y, z_inv),
       "JDaxpy is not invertible with params:\n\ta = %lf\n\tx = %s\n\ty = %s",
       a, x_str, y_str);
}

//*** JDAddMultRatSecs
TheoryDataPoints(SUITE_NAME, addmultratsecs) = {
    JD_DATAPOINTS, JD_SECONDS_DATAPOINTS,
    DataPoints(long, 0, 1, 2, -1, -2, _RATLONG_MAX_, -_RATLONG_MAX_,
               _RATLONG_MAX_ / 2, -_RATLONG_MAX_ / 2, (((Rat_Long)1) << 32),
               -(((Rat_Long)1) << 32), (((Rat_Long)1) << 48),
               -(((Rat_Long)1) << 48), (((Rat_Long)1) << 60),
               -(((Rat_Long)1) << 60), (((Rat_Long)1) << 32) + 1,
               -(((Rat_Long)1) << 32) - 1, (((Rat_Long)1) << 48) + 1,
               -(((Rat_Long)1) << 48) - 1, (((Rat_Long)1) << 60) + 1,
               -(((Rat_Long)1) << 60) - 1, _RATLONG_MIN_)};

Theory((JDType * jd, JDType *jd_secs, long a), SUITE_NAME, addmultratsecs)
{
   char jd_str[JD_STR_LEN] = {'\0'}, sec_str[RATIONAL_STR_LEN] = {'\0'};
   jd2str(*jd, jd_str);
   Rational seconds = jd_secs->seconds;
   rat2str(seconds, sec_str);

   // check
   JDType z = JDAddMultRatSecs(*jd, a, seconds);
   if (a == 0) {
      cr_expect(isequal_jd(*jd, z),
                "JDAddMultRatSecs with a=0 did not preserve %s", jd_str);
   }
   else {
      // idk here
   }

   // associative
   JDType zero    = JD_ZERO;
   zero.system    = jd->system;
   zero.epoch     = jd->epoch;
   JDType a_sec   = JDAddMultRatSecs(zero, a, seconds);
   JDType z_prime = JDAdd(a_sec, *jd);
   cr_expect(isequal_jd(z_prime, z),
             "JDAddMultRatSecs is not associative with params:\n\ta   = "
             "%li\n\tjd  = %s\n\tsec = %s",
             a, jd_str, sec_str);

   // invertible
   cr_assume(a >= -_RATLONG_MAX_); // -_RATLONG_MIN_ > _RATLONG_MAX_
   JDType z_inv = JDAddMultRatSecs(z, -a, seconds);
   cr_expect(isequal_jd(*jd, z_inv),
             "JDAddMultRatSecs is not invertible with params:\n\ta   = "
             "%li\n\tjd  = %s\n\tsec = %s",
             a, jd_str, sec_str);
}

/* Leap Seconds                                                       */
struct leapsec_tuple {
   JDType jd;
   double leapsec;
};

ParameterizedTestParameters(SUITE_NAME, leapsec)
{
#define SIZE 56
   static int first                          = 0;
   static struct leapsec_tuple vals[SIZE]    = {0};
   const struct leapsec_tuple vals_nstatic[] = {
       {JD_RAW(UTC_TIME, ZERO_EPOCH, 0, RATIONAL_ZERO), 0},

       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2441317.5 + 0.5), 10.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2441499.5 + 0.5), 11.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2441683.5 + 0.5), 12.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2442048.5 + 0.5), 13.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2442413.5 + 0.5), 14.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2442778.5 + 0.5), 15.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2443144.5 + 0.5), 16.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2443509.5 + 0.5), 17.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2443874.5 + 0.5), 18.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2444239.5 + 0.5), 19.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2444786.5 + 0.5), 20.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2445151.5 + 0.5), 21.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2445516.5 + 0.5), 22.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2446247.5 + 0.5), 23.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2447161.5 + 0.5), 24.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2447892.5 + 0.5), 25.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2448257.5 + 0.5), 26.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2448804.5 + 0.5), 27.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2449169.5 + 0.5), 28.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2449534.5 + 0.5), 29.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2450083.5 + 0.5), 30.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2450630.5 + 0.5), 31.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2451179.5 + 0.5), 32.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2453736.5 + 0.5), 33.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2454832.5 + 0.5), 34.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2456109.5 + 0.5), 35.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2457204.5 + 0.5), 36.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2457754.5 + 0.5), 37.0},

       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2441499.5 - 0.5), 10.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2441683.5 - 0.5), 11.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2442048.5 - 0.5), 12.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2442413.5 - 0.5), 13.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2442778.5 - 0.5), 14.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2443144.5 - 0.5), 15.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2443509.5 - 0.5), 16.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2443874.5 - 0.5), 17.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2444239.5 - 0.5), 18.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2444786.5 - 0.5), 19.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2445151.5 - 0.5), 20.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2445516.5 - 0.5), 21.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2446247.5 - 0.5), 22.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2447161.5 - 0.5), 23.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2447892.5 - 0.5), 24.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2448257.5 - 0.5), 25.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2448804.5 - 0.5), 26.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2449169.5 - 0.5), 27.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2449534.5 - 0.5), 28.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2450083.5 - 0.5), 29.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2450630.5 - 0.5), 30.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2451179.5 - 0.5), 31.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2453736.5 - 0.5), 32.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2454832.5 - 0.5), 33.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2456109.5 - 0.5), 34.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2457204.5 - 0.5), 35.0},
       {JD_NREDUCE(UTC_TIME, ZERO_EPOCH, 2457754.5 - 0.5), 36.0},
   };
   if (!first) {
      first = 1;
      for (size_t i = 0; i < SIZE; i++)
         vals[i] = vals_nstatic[i];
   }
   return cr_make_param_array(struct leapsec_tuple, vals, SIZE);
#undef SIZE
}

ParameterizedTest(struct leapsec_tuple *a, SUITE_NAME, leapsec)
{
   char a_str[JD_STR_LEN] = {'\0'};
   jd2str(a->jd, a_str);

   double calc_leapsec = GetLeapSec(a->jd);

   cr_expect(calc_leapsec == a->leapsec,
             "GetLeapSec(%s) = %lf sec did not match expected "
             "value of %lf sec",
             a->jd, calc_leapsec, a->leapsec);
}

// TODO: test epoch/system change results are correct (this one will take a bit
//       of work)
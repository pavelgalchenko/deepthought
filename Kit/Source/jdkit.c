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

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

// NOTE: Uses the global 'ModelPath'

// TODO: do we want these in the header?
#define _ZERO_EPOCH_TT     (0.0)       // Jan  1, -4712, 12:00:00
#define _GD_JD_EPOCH_TT    (1721013.5) // Nov 18, -0001, 00:00:00, in GD->JD
#define _MJD_EPOCH_TT      (2400000.5) // Nov 17,  1858, 00:00:00
#define _J1900_EPOCH_TT    (2415019.5) // Dec 31,  1899, 00:00:00, in JD->GD
#define _GMAT_MJD_EPOCH_TT (2430000.0) // Jan  5,  1941, 12:00:00
#define _CCSDS_EPOCH_TT    (2436204.5) // Jan 1,   1958, 00:00:00
#define _TCB_TDB_EPOCH_TT  (2443144.5) // Jan  1,  1977, 00:00:00, in tcb->tdb
#define _J2000_EPOCH_TT    (2451545.0) // Jan 1 ,  2000, 12:00:00, J2000 epoch

// Make sure that we covered everything EXPLICITLY
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wswitch"
#pragma GCC diagnostic error "-Wswitch-enum"

/**********************************************************************/
TimeSystem GetTimeSystem(const char *s)
{
   if (!strncmp(s, "UTC", 3))
      return UTC_TIME;
   else if (!strncmp(s, "TAI", 3))
      return TAI_TIME;
   else if (!strncmp(s, "TCB", 3))
      return TCB_TIME;
   else if (!strncmp(s, "TDB", 3))
      return TDB_TIME;
   else if (!strncmp(s, "TT", 2))
      return TT_TIME;
   fprintf(stderr, "Bogus input %s in GetTimeSystem (42init.c:%d)\n", s,
           __LINE__);
   exit(EXIT_FAILURE);
}
/**********************************************************************/
static Rational _epoch_pod_seconds(const EpochTT epoch) __attribute__((const));
static Rational _epoch_pod_seconds(const EpochTT epoch)
{
   // either zero or 43200 seconds
   switch (epoch) {
      case ZERO_EPOCH:
      case GMAT_MJD_EPOCH:
      case J2000_EPOCH:
      default:
         break;
      case GD_CONV_EPOCH:
      case TCB_TDB_CONV_EPOCH:
      case MJD_EPOCH:
      case J1900_EPOCH:
      case CCSDS_EPOCH:
         return RATIONAL_RAW(SEC_PER_DAY / 2, 0, 1);
   }
   return RATIONAL_ZERO;
}
/**********************************************************************/
static JDType _epoch_diff_tt(const EpochTT a, const EpochTT b)
    __attribute__((const));
static JDType _epoch_diff_tt(const EpochTT a, const EpochTT b)
{
   JDType jd_diff = JD_ZERO;
   // handle the easy cases here
   if (a == b) {
      jd_diff.whole_days = 0;
      jd_diff.seconds    = RATIONAL_ZERO;
      return jd_diff;
   }

   // determine part of day value
   const Rational a_pod = _epoch_pod_seconds(a);
   const Rational b_pod = _epoch_pod_seconds(b);

   jd_diff.seconds.whole = a_pod.whole - b_pod.whole;
   jd_diff.seconds       = RationalAbs(jd_diff.seconds);

   if (b == ZERO_EPOCH)
      jd_diff.whole_days = (long)(EpochValueTT(a));
   else if (a == ZERO_EPOCH)
      jd_diff.whole_days = (long)(-1.0 * EpochValueTT(b));

   // do the switches (have to do all of them due to the pragma rules)
   // TODO: some testing on how the compiler does this, hope it precomputes
   //    i.e. hoping for a lookup table by the time we get to execution time
   // maybe see about going to C23 to have constexpr?
   switch (a) {
      case J1900_EPOCH: {
         switch (b) {
            case J2000_EPOCH: {
               jd_diff.whole_days = (long)(_J1900_EPOCH_TT - _J2000_EPOCH_TT);
               break;
            }
            case GMAT_MJD_EPOCH: {
               jd_diff.whole_days =
                   (long)(_J1900_EPOCH_TT - _GMAT_MJD_EPOCH_TT);
               break;
            }
            case MJD_EPOCH: {
               jd_diff.whole_days = (long)(_J1900_EPOCH_TT - _MJD_EPOCH_TT);
               break;
            }
            case GD_CONV_EPOCH: {
               jd_diff.whole_days = (long)(_J1900_EPOCH_TT - _GD_JD_EPOCH_TT);
               break;
            }
            case TCB_TDB_CONV_EPOCH: {
               jd_diff.whole_days = (long)(_J1900_EPOCH_TT - _TCB_TDB_EPOCH_TT);
               break;
            }
            case CCSDS_EPOCH: {
               jd_diff.whole_days = (long)(_J1900_EPOCH_TT - _CCSDS_EPOCH_TT);
               break;
            }
            case ZERO_EPOCH:
            case J1900_EPOCH:
               break;
         }
      } break;
      case J2000_EPOCH: {
         switch (b) {
            case GMAT_MJD_EPOCH: {
               jd_diff.whole_days =
                   (long)(_J2000_EPOCH_TT - _GMAT_MJD_EPOCH_TT);
               break;
            }
            case MJD_EPOCH: {
               jd_diff.whole_days = (long)(_J2000_EPOCH_TT - _MJD_EPOCH_TT);
               break;
            }
            case GD_CONV_EPOCH: {
               jd_diff.whole_days = (long)(_J2000_EPOCH_TT - _GD_JD_EPOCH_TT);
               break;
            }
            case TCB_TDB_CONV_EPOCH: {
               jd_diff.whole_days = (long)(_J2000_EPOCH_TT - _TCB_TDB_EPOCH_TT);
               break;
            }
            case CCSDS_EPOCH: {
               jd_diff.whole_days = (long)(_J2000_EPOCH_TT - _CCSDS_EPOCH_TT);
               break;
            }
            case J1900_EPOCH: {
               jd_diff.whole_days = (long)(_J2000_EPOCH_TT - _J1900_EPOCH_TT);
               break;
            }
            case ZERO_EPOCH:
            case J2000_EPOCH:
               break;
         }
      } break;
      case MJD_EPOCH: {
         switch (b) {
            case J2000_EPOCH: {
               jd_diff.whole_days = (long)(_MJD_EPOCH_TT - _J2000_EPOCH_TT);
               break;
            }
            case GMAT_MJD_EPOCH: {
               jd_diff.whole_days = (long)(_MJD_EPOCH_TT - _GMAT_MJD_EPOCH_TT);
               break;
            }
            case GD_CONV_EPOCH: {
               jd_diff.whole_days = (long)(_MJD_EPOCH_TT - _GD_JD_EPOCH_TT);
               break;
            }
            case TCB_TDB_CONV_EPOCH: {
               jd_diff.whole_days = (long)(_MJD_EPOCH_TT - _TCB_TDB_EPOCH_TT);
               break;
            }
            case CCSDS_EPOCH: {
               jd_diff.whole_days = (long)(_MJD_EPOCH_TT - _CCSDS_EPOCH_TT);
               break;
            }
            case J1900_EPOCH: {
               jd_diff.whole_days = (long)(_MJD_EPOCH_TT - _J1900_EPOCH_TT);
               break;
            }
            case ZERO_EPOCH:
            case MJD_EPOCH:
               break;
         }
      } break;
      case GMAT_MJD_EPOCH: {
         switch (b) {
            case J2000_EPOCH: {
               jd_diff.whole_days =
                   (long)(_GMAT_MJD_EPOCH_TT - _J2000_EPOCH_TT);
               break;
            }
            case MJD_EPOCH: {
               jd_diff.whole_days = (long)(_GMAT_MJD_EPOCH_TT - _MJD_EPOCH_TT);
               break;
            }
            case GD_CONV_EPOCH: {
               jd_diff.whole_days =
                   (long)(_GMAT_MJD_EPOCH_TT - _GD_JD_EPOCH_TT);
               break;
            }
            case TCB_TDB_CONV_EPOCH: {
               jd_diff.whole_days =
                   (long)(_GMAT_MJD_EPOCH_TT - _TCB_TDB_EPOCH_TT);
               break;
            }
            case CCSDS_EPOCH: {
               jd_diff.whole_days =
                   (long)(_GMAT_MJD_EPOCH_TT - _CCSDS_EPOCH_TT);
               break;
            }
            case J1900_EPOCH: {
               jd_diff.whole_days =
                   (long)(_GMAT_MJD_EPOCH_TT - _J1900_EPOCH_TT);
               break;
            }
            case ZERO_EPOCH:
            case GMAT_MJD_EPOCH:
               break;
         }
      } break;
      case GD_CONV_EPOCH: {
         switch (b) {
            case J2000_EPOCH: {
               jd_diff.whole_days = (long)(_GD_JD_EPOCH_TT - _J2000_EPOCH_TT);
               break;
            }
            case MJD_EPOCH: {
               jd_diff.whole_days = (long)(_GD_JD_EPOCH_TT - _MJD_EPOCH_TT);
               break;
            }
            case GMAT_MJD_EPOCH: {
               jd_diff.whole_days =
                   (long)(_GD_JD_EPOCH_TT - _GMAT_MJD_EPOCH_TT);
               break;
            }
            case TCB_TDB_CONV_EPOCH: {
               jd_diff.whole_days = (long)(_GD_JD_EPOCH_TT - _TCB_TDB_EPOCH_TT);
               break;
            }
            case CCSDS_EPOCH: {
               jd_diff.whole_days = (long)(_GD_JD_EPOCH_TT - _CCSDS_EPOCH_TT);
               break;
            }
            case J1900_EPOCH: {
               jd_diff.whole_days = (long)(_GD_JD_EPOCH_TT - _J1900_EPOCH_TT);
               break;
            }
            case ZERO_EPOCH:
            case GD_CONV_EPOCH:
               break;
         }
      } break;
      case TCB_TDB_CONV_EPOCH: {
         switch (b) {
            case J2000_EPOCH: {
               jd_diff.whole_days = (long)(_TCB_TDB_EPOCH_TT - _J2000_EPOCH_TT);
               break;
            }
            case MJD_EPOCH: {
               jd_diff.whole_days = (long)(_TCB_TDB_EPOCH_TT - _MJD_EPOCH_TT);
               break;
            }
            case GMAT_MJD_EPOCH: {
               jd_diff.whole_days =
                   (long)(_TCB_TDB_EPOCH_TT - _GMAT_MJD_EPOCH_TT);
               break;
            }
            case GD_CONV_EPOCH: {
               jd_diff.whole_days = (long)(_TCB_TDB_EPOCH_TT - _GD_JD_EPOCH_TT);
               break;
            }
            case CCSDS_EPOCH: {
               jd_diff.whole_days = (long)(_TCB_TDB_EPOCH_TT - _CCSDS_EPOCH_TT);
               break;
            }
            case J1900_EPOCH: {
               jd_diff.whole_days = (long)(_TCB_TDB_EPOCH_TT - _J1900_EPOCH_TT);
               break;
            }
            case ZERO_EPOCH:
            case TCB_TDB_CONV_EPOCH:
               break;
         }
      } break;
      case CCSDS_EPOCH: {
         switch (b) {
            case J2000_EPOCH: {
               jd_diff.whole_days = (long)(_CCSDS_EPOCH_TT - _J2000_EPOCH_TT);
               break;
            }
            case MJD_EPOCH: {
               jd_diff.whole_days = (long)(_CCSDS_EPOCH_TT - _MJD_EPOCH_TT);
               break;
            }
            case GMAT_MJD_EPOCH: {
               jd_diff.whole_days =
                   (long)(_CCSDS_EPOCH_TT - _GMAT_MJD_EPOCH_TT);
               break;
            }
            case GD_CONV_EPOCH: {
               jd_diff.whole_days = (long)(_CCSDS_EPOCH_TT - _GD_JD_EPOCH_TT);
               break;
            }
            case TCB_TDB_CONV_EPOCH: {
               jd_diff.whole_days = (long)(_CCSDS_EPOCH_TT - _TCB_TDB_EPOCH_TT);
               break;
            }
            case J1900_EPOCH: {
               jd_diff.whole_days = (long)(_CCSDS_EPOCH_TT - _J1900_EPOCH_TT);
               break;
            }
            case ZERO_EPOCH:
            case CCSDS_EPOCH:
               break;
         }
      } break;
      case ZERO_EPOCH:
         break;
   }

   if (jd_diff.whole_days < 0) {
      jd_diff.seconds = RationalNegate(jd_diff.seconds);
   }
   return jd_diff;
}

#define _jd_tt2tai(x) JDSubRationalSeconds((x), RATIONAL_NGCD(32, 23, 125))
#define _jd_tai2tt(x) JDAddRationalSeconds((x), RATIONAL_NGCD(32, 23, 125))

/**********************************************************************/
//  time system low level conversion helpers
/**********************************************************************/
#define L_B (1.550505e-8)
static JDType _jd_tcb2tdb(const JDType tcb_jd __attribute__((unused)))
{
   // JDType jd_tt_conv = _jdtt(tcb_jd);// nope, do not
   // JDChangeEpoch(TCB_TDB_CONV_EPOCH, &jd_tt_conv);
   // TODO: pretending that we don't need this for now
   fprintf(stderr, "Julian Day conversion from TCB to TDB is not implemented. "
                   "Exiting...\n");
   exit(EXIT_FAILURE);
}
static JDType _jdtt(JDType) __attribute__((const));
static JDType _jd_tdb2tcb(JDType tdb_jd) __attribute__((const));
static JDType _jd_tdb2tcb(JDType tdb_jd)
{
   const JDType jd_tt_conv =
       JDChangeSystemEpoch(TT_TIME, TCB_TDB_CONV_EPOCH, tdb_jd);
   const double d_tcb_tdb = L_B * JDToDays(jd_tt_conv);

   tdb_jd.system = TCB_TIME;
   return JDAddSeconds(tdb_jd, d_tcb_tdb);
}
#undef L_B
/**********************************************************************/
#define TDB_COEFF1             (0.00165)
#define TDB_COEFF2             (0.00001385)
#define M_E_OFFSET             (357.5277233)
#define M_E_COEFF1             (35999.05034)
#define DAY_PER_JULIAN_CENTURY (36525.0)
static double _sec_dbl_d_tt_tdb(double secs_tt_j2000) __attribute__((const));
static double _sec_dbl_d_tt_tdb(double secs_tt_j2000)
{
   const double T_TT = secs_tt_j2000 / (DAY_PER_JULIAN_CENTURY * SEC_PER_DAY);
   const double m_E  = fmod((M_E_OFFSET + (M_E_COEFF1 * T_TT)), 360.0);
   return (TDB_COEFF1 * sin_deg(m_E) + TDB_COEFF2 * sin_deg(2.0 * m_E));
}
#undef M_E_OFFSET
#undef M_E_COEFF1
#undef DAY_PER_JULIAN_CENTURY
/**********************************************************************/
static double _d_tt_tdb(JDType jd)
{
   // TODO: use spice instead if available?
   // Approximation from GMAT 2026 Mathematical Specification, p10
   // assuming input jd is tt already
   jd = JDChangeEpoch(J2000_EPOCH, jd);
   return _sec_dbl_d_tt_tdb(JDToSeconds(jd));
}
/**********************************************************************/
static double _tdb2ttF(const double x, double params[1]) __attribute__((pure));
static double _tdb2ttF(const double x, double params[1])
{
   return x + _sec_dbl_d_tt_tdb(x) - params[0];
}
/**********************************************************************/
static JDType _jd_tt2tdb(JDType tt_jd) __attribute__((const));
static JDType _jd_tt2tdb(JDType tt_jd)
{
   tt_jd.system = TDB_TIME;
   return JDAddSeconds(tt_jd, _d_tt_tdb(tt_jd));
}
/**********************************************************************/
static JDType _jd_tdb2tt(JDType tdb_jd)
{
   // Use Newton Method to approximate inverse of _jd_tt2tdb;
   JDType jd_tdb_j2000 = JDChangeEpoch(J2000_EPOCH, tdb_jd);

   const double secs_tdb_j2000 = JDToSeconds(jd_tdb_j2000);
   double params[1]            = {secs_tdb_j2000};

   const double max_width     = TDB_COEFF1 + TDB_COEFF2;
   const double secs_tt_j2000 = BrentsMethod(secs_tdb_j2000 - 10.0 * max_width,
                                             secs_tdb_j2000 + 10.0 * max_width,
                                             1e-14, &_tdb2ttF, params);
   JDType jd_tt_out = JDFromSeconds(secs_tt_j2000, TT_TIME, J2000_EPOCH);
   jd_tt_out        = JDChangeEpoch(tdb_jd.epoch, jd_tt_out);

   jd_tt_out.system = TT_TIME;
   return jd_tt_out;
}
#undef TDB_COEFF1
#undef TDB_COEFF2
// UTC headaches
static JDType _jd_utc2tai(JDType utc_jd) __attribute__((const));
static JDType _jd_utc2tai(JDType utc_jd)
{
   const double leap_sec = GetLeapSec(utc_jd);

   utc_jd.system = TAI_TIME;
   return JDAddSeconds(utc_jd, leap_sec);
}
static JDType _jd_tai2utc(JDType tai_jd) __attribute__((const));
static JDType _jd_tai2utc(JDType tai_jd)
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
   tai_jd.system     = UTC_TIME;
   JDType tai_utc_jd = tai_jd;

   const double leap_sec = GetLeapSec(tai_utc_jd);
   tai_utc_jd            = JDSubSeconds(tai_jd, leap_sec);
   const double test_ls  = GetLeapSec(tai_utc_jd);
   if (test_ls != leap_sec)
      tai_utc_jd = JDSubSeconds(tai_jd, test_ls);

   return tai_utc_jd;
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
      case UTC_TIME:
         break;
   }
   jd_out.system = UTC_TIME;
   return jd_out;
}
static JDType _jdtai(const JDType jd)
{
   JDType jd_out = jd;
   switch (jd.system) {
      case UTC_TIME: {
         jd_out = _jd_utc2tai(jd_out);
      } break;
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
         break;
   }
   jd_out.system = TAI_TIME;
   return jd_out;
}
static JDType _jdtcb(const JDType jd)
{
   JDType jd_out = jd;
   switch (jd.system) {
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
   }
   jd_out.system = TCB_TIME;
   return jd_out;
}
static JDType _jdtdb(const JDType jd)
{
   JDType jd_out = jd;
   switch (jd.system) {
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
   }
   jd_out.system = TDB_TIME;
   return jd_out;
}
static JDType _jdtt(const JDType jd)
{
   JDType jd_out = jd;
   switch (jd.system) {
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
         return _ZERO_EPOCH_TT;
      case GD_CONV_EPOCH:
         return _GD_JD_EPOCH_TT;
      case TCB_TDB_CONV_EPOCH:
         return _TCB_TDB_EPOCH_TT;
      case MJD_EPOCH:
         return _MJD_EPOCH_TT;
      case GMAT_MJD_EPOCH:
         return _GMAT_MJD_EPOCH_TT;
      case CCSDS_EPOCH:
         return _CCSDS_EPOCH_TT;
      case J2000_EPOCH:
         return _J2000_EPOCH_TT;
      case J1900_EPOCH:
         return _J1900_EPOCH_TT;
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

// returns the number of leap seconds for specified JD
double GetLeapSec(const JDType jd)
{
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

   // TODO: this and other functions do not handle the time being *during* a
   // leap second

   // TODO: use spice instead if available?

   // ensure jd is UTC with MJD epoch
   // dug through GMAT source code, JD in 'tai-utc.dat' is UTC
   // TODO: this causes infinite recursion due to the conversion to TT_TIME
   // embeded within
   const JDType jd_mjd_utc = JDChangeSystemEpoch(UTC_TIME, MJD_EPOCH, jd);

   if (leapSecTbl.n_entries == 0) {
      // initalize data
      extern char ModelPath[1000];
      char f_path[1064] = {'\0'};
      strcpy(f_path, ModelPath);
      strcat(f_path, "/tai-utc.dat");
      FILE *file = fopen(f_path, "rt");
      if (file == NULL) {
         fprintf(stderr, "Error opening tai-utc file '%s'. Exiting...\n",
                 f_path);
         exit(EXIT_FAILURE);
      }

      // loop over file to find the number of nonempty lines
      char line[512] = {'\0'};
      while (fgets(line, 512, file))
         if (!isLineBlank(line))
            leapSecTbl.n_entries++;

      // use number of nonempty lines to allocate the data locations
      leapSecTbl.entries =
          calloc(leapSecTbl.n_entries, sizeof(struct LeapSecFileEntry));

      // rewind file and start parsing for the actual data
      rewind(file);
      int i = 0;
      while (fgets(line, 512, file)) {
         struct LeapSecFileEntry *const entry = &leapSecTbl.entries[i];
         int y, d;
         char mon[16]           = {'\0'};
         double jd_mjd_utc_days = 0;
         int sscanf_check       = sscanf(
             line, "%i %s %i =JD %lf TAI-UTC= %lf S + (MJD - %lf) X %lf S", &y,
             mon, &d, &jd_mjd_utc_days, &entry->offset_1, &entry->offset_2,
             &entry->offset_3);

         if (sscanf_check) {
            entry->jd_mjd_utc =
                JDFromDays(jd_mjd_utc_days, UTC_TIME, ZERO_EPOCH);
            entry->jd_mjd_utc = JDChangeEpoch(MJD_EPOCH, entry->jd_mjd_utc);

            i++;
         }
      }
      fclose(file);
   }

   double jd_mjd_utc_days = JDToDays(jd_mjd_utc);
   struct LeapSecFileEntry *const start_entry =
       &leapSecTbl.entries[leapSecTbl.n_entries - 1];
   for (struct LeapSecFileEntry *entry = start_entry;
        entry >= leapSecTbl.entries; entry--) {
      if (isgreaterequal_jd(jd_mjd_utc, entry->jd_mjd_utc))
         return entry->offset_1 +
                ((jd_mjd_utc_days - entry->offset_2) * entry->offset_3);
   }
   return 0;
}

// ensure everything in JDType is reduced, and that if whole_days < 0, then so
// are seconds.whole and seconds.num, and vice-versa
static JDType _reduce_jd_no_rational(JDType jd) __attribute__((const));
static JDType _reduce_jd_no_rational(JDType jd)
{
   const RationalLL rat_day =
       (RationalLL){.whole = SEC_PER_DAY, .num = 0, .den = 1};
   jd.whole_days += RationalIntMod(&jd.seconds, SEC_PER_DAY);
   if (((jd.whole_days < 0 && ispos_rational(jd.seconds)) ||
        (jd.whole_days > 0 && !ispos_rational(jd.seconds))) &&
       jd.whole_days != 0 && !isequal_rational(jd.seconds, RATIONAL_ZERO)) {
      if (jd.whole_days > 0) {
         jd.seconds = ToRational(RationalAdd(jd.seconds, rat_day));
         jd.whole_days--;
      }
      else if (jd.whole_days < 0) {
         jd.seconds = ToRational(RationalSub(jd.seconds, rat_day));
         jd.whole_days++;
      }
   }
   return jd;
}
static JDType _reduce_jd(JDType jd) __attribute__((const));
static JDType _reduce_jd(JDType jd)
{
   jd         = _reduce_jd_no_rational(jd);
   jd.seconds = ReduceRational(jd.seconds);
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
      case TAI_TIME:
         return _jdtai(jd);
      case TCB_TIME:
         return _jdtcb(jd);
      case TDB_TIME:
         return _jdtdb(jd);
      case TT_TIME:
         return _jdtt(jd);
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
   JDType jd_diff = _epoch_diff_tt(jd.epoch, new_epoch);

   // JDType jd_tt = _jdtt(*jd);
   jd                = JDAddDays(jd, jd_diff.whole_days);
   jd.seconds.whole += jd_diff.seconds.whole;
   jd.epoch          = new_epoch;

   // JDChangeSystem(jd->system, &jd_tt);
   // *jd = jd_tt;
   return _reduce_jd_no_rational(jd);
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
   return (double)jd.whole_days + (rational2double(jd.seconds)) / SEC_PER_DAY;
}

JDType JDFromDays(const double days, const TimeSystem system,
                  const EpochTT new_epoch)
{
   JDType jd               = {0};
   jd.epoch                = new_epoch;
   jd.system               = system;
   jd.whole_days           = days;
   const Rational part_day = double2rational(days - jd.whole_days);
   jd.seconds              = IntegerRationalMult(SEC_PER_DAY, part_day);
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
   jd.seconds =
       RATIONAL_NGCD(seconds.whole % SEC_PER_DAY, seconds.num, seconds.den);
   return jd;
} /**********************************************************************/
/*  Converts seconds since 'epoch' in 'system' to a JDType format     */
JDType JDFromSeconds(const double seconds, const TimeSystem system,
                     const EpochTT new_epoch)
{
   JDType jd     = {0};
   jd.epoch      = new_epoch;
   jd.system     = system;
   jd.whole_days = seconds / SEC_PER_DAY;
   jd.seconds    = double2rational(fmod(seconds, SEC_PER_DAY));
   return jd;
}
/**********************************************************************/
/* Time is elapsed seconds since input epoch                          */
/*  This function returns the seconds in whatever system the input    */
/*  'jd' uses                                                         */
double JDToSeconds(JDType jd)
{
   return ((double)jd.whole_days * SEC_PER_DAY + rational2double(jd.seconds));
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
   Rational out  = jd.seconds;
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

JDType InitJD(const TimeSystem system, const EpochTT epoch, const long days,
              const Rational seconds)
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

   jd_out.seconds    = ToRational(RationalMult(mul, jd.seconds));
   Rational jd_whole = IntegerRationalMult(jd.whole_days, mul);
   jd_out.whole_days = jd_whole.whole;
   jd_whole.whole    = 0;
   Rational jd_secs  = IntegerRationalMult(SEC_PER_DAY, jd_whole);

   jd_out.seconds = ToRational(RationalAdd(jd_secs, jd_out.seconds));

   return _reduce_jd(jd_out);
}

JDType JDAdd(JDType a, JDType b)
{
   EpochTT out_epoch = a.epoch;
   if (a.epoch == ZERO_EPOCH && b.epoch != ZERO_EPOCH)
      out_epoch = b.epoch;
   if ((a.epoch == ZERO_EPOCH) != (b.epoch == ZERO_EPOCH)) {
      a.epoch = out_epoch;
      b.epoch = out_epoch;
   }

   _error_epoch_system(a, b, "JDAdd");
   JDType jdout      = a;
   jdout.whole_days += b.whole_days;
   jdout.seconds     = ToRational(RationalAdd(jdout.seconds, b.seconds));

   return _reduce_jd(jdout);
}
JDType JDAddDays(const JDType a, const double b)
{
   JDType jdb = JDFromDays(b, a.system, a.epoch);
   return JDAdd(a, jdb);
}
JDType JDAddSeconds(const JDType a, const double b)
{
   JDType jdb = JDFromSeconds(b, a.system, a.epoch);
   return JDAdd(a, jdb);
}
JDType JDAddRationalSeconds(const JDType a, const Rational b)
{
   JDType jdb  = {0};
   jdb.system  = a.system;
   jdb.epoch   = a.epoch;
   jdb.seconds = b;

   return JDAdd(a, _reduce_jd(jdb));
}
/**********************************************************************/
/*  Add (mul * b) seconds to the Julian Date in jd using an integer   */
/*  arithmetic multiplication algorithm                               */
JDType JDAddIntegerMultRatSecs(const JDType jd, const long mul,
                               const Rational rat)
{
   JDType jdb = jd;

   jdb.whole_days = 0;
   jdb.seconds    = IntegerRationalMultMod(mul, _rat_to_rationalll(rat),
                                           SEC_PER_DAY, &jdb.whole_days);

   return JDAdd(jd, jdb);
}

JDType JDAddRationalMult(const JDType a, Rational mul, JDType b)
{
   b.system = a.system;
   b.epoch  = a.epoch;
   b        = _jd_rational_mult_helper(mul, b);

   return JDAdd(a, b);
}

JDType JDSub(JDType a, JDType b)
{
   EpochTT out_epoch = a.epoch;
   if (a.epoch == ZERO_EPOCH && b.epoch != ZERO_EPOCH)
      out_epoch = b.epoch;
   if ((a.epoch == ZERO_EPOCH) != (b.epoch == ZERO_EPOCH)) {
      a.epoch = out_epoch;
      b.epoch = out_epoch;
   }

   _error_epoch_system(a, b, "JDSub");
   JDType jdout      = a;
   jdout.whole_days -= b.whole_days;
   jdout.seconds     = ToRational(RationalSub(jdout.seconds, b.seconds));

   return _reduce_jd(jdout);
}

JDType JDSubDays(const JDType a, const double b)
{
   JDType jdb = JDFromDays(b, a.system, a.epoch);
   return JDSub(a, jdb);
}

JDType JDSubSeconds(const JDType a, const double b)
{
   JDType jdb = JDFromSeconds(b, a.system, a.epoch);
   return JDSub(a, jdb);
}

JDType JDSubRationalSeconds(const JDType a, const Rational b)
{
   JDType jdb  = {0};
   jdb.system  = a.system;
   jdb.epoch   = a.epoch;
   jdb.seconds = b;

   return JDSub(a, _reduce_jd(jdb));
}

JDType JDSubRationalMult(const JDType a, Rational mul, JDType b)
{
   b.system = a.system;
   b.epoch  = a.epoch;
   b        = _jd_rational_mult_helper(mul, b);

   return JDSub(a, b);
}

JDType JDaxpy(const double a, JDType x, JDType y)
{
   // The operation 'z = a * x + y' for 'x' and 'y' being JDType and 'a' being
   // a scalar double
   // TODO: this is quite hacky to "just work" for its usage in rkkit
   const int sign       = (a >= 0) ? 1 : -1;
   const double mult    = fabs(a);
   RationalLL mult_rat  = _rat_to_rationalll(double2rational(mult));
   Rational day_mult    = IntegerRationalMult(x.whole_days, mult_rat);
   x.whole_days         = day_mult.whole;
   day_mult.whole       = 0;
   day_mult.num        *= SEC_PER_DAY;

   RationalLL secs = RationalMult(mult_rat, x.seconds);
   x.seconds       = ToRational(RationalAdd(secs, day_mult));

   x.epoch  = y.epoch;
   x.system = y.system;

   x                = _reduce_jd(x);
   x.whole_days    *= sign;
   x.seconds.whole *= sign;
   x.seconds.num   *= sign;

   return JDAdd(x, y);
}

double JDAddToDays(const JDType a, const JDType b)
{
   return JDToDays(JDAdd(a, b));
}

double JDAddToSeconds(const JDType a, const JDType b)
{
   return JDToSeconds(JDAdd(a, b));
}

double JDSubToDays(const JDType a, const JDType b)
{
   return JDToDays(JDSub(a, b));
}

double JDSubToSeconds(const JDType a, const JDType b)
{
   return JDToSeconds(JDSub(a, b));
}

JDType JDAbs(JDType jd)
{
   jd.whole_days = labs(jd.whole_days);
   jd.seconds    = RationalAbs(jd.seconds);
   return jd;
}

int ispos_jd(JDType jd)
{
   jd = _reduce_jd(JDChangeEpoch(ZERO_EPOCH, jd));
   return (jd.whole_days > 0 ||
           (jd.whole_days == 0 && ispos_rational(jd.seconds)));
}

JDType JDNegate(JDType jd)
{
   jd            = _reduce_jd(jd);
   jd.whole_days = -jd.whole_days;
   jd.seconds    = RationalNegate(jd.seconds);
   return jd;
}

int isequal_jd_systemepoch(const JDType a, const JDType b)
{
   return (a.system == b.system) && (a.epoch == b.epoch);
}

int isequal_jd(const JDType a, const JDType b)
{
   return ((a.system == b.system) && (a.epoch == b.epoch) &&
           (a.whole_days == b.whole_days) &&
           isequal_rational(a.seconds, b.seconds));
}

int isless_jd(const JDType a, const JDType b)
{
   _error_epoch_system(a, b, "isless_jd");
   const int is_day_less = a.whole_days < b.whole_days;
   const int is_sec_less =
       (a.whole_days == b.whole_days) && isless_rational(a.seconds, b.seconds);
   return is_day_less || is_sec_less;
}

int islessequal_jd(const JDType a, const JDType b)
{
   _error_epoch_system(a, b, "islessequal_jd");
   return isequal_jd(a, b) || isless_jd(a, b);
}

int isgreater_jd(const JDType a, const JDType b)
{
   _error_epoch_system(a, b, "isgreater_jd");
   const int is_day_greater = a.whole_days > b.whole_days;
   const int is_sec_greater = (a.whole_days == b.whole_days) &&
                              isgreater_rational(a.seconds, b.seconds);
   return is_day_greater || is_sec_greater;
}

int isgreaterequal_jd(const JDType a, const JDType b)
{
   _error_epoch_system(a, b, "isgreaterequal_jd");
   return isequal_jd(a, b) || isgreater_jd(a, b);
}

EpochTT str2epoch(char str[JDEPOCH_STR_LEN])
{

   if (!strncmp(str, "Zero", 5))
      return ZERO_EPOCH;
   else if (!strncmp(str, "GD Conversion", 14))
      return GD_CONV_EPOCH;
   else if (!strncmp(str, "TCB/TDB Conversion", 19))
      return TCB_TDB_CONV_EPOCH;
   else if (!strncmp(str, "MJD", 4))
      return MJD_EPOCH;
   else if (!strncmp(str, "J1900", 6))
      return J1900_EPOCH;
   else if (!strncmp(str, "GMAT MJD", 9))
      return GMAT_MJD_EPOCH;
   else if (!strncmp(str, "CCSDS", 6))
      return CCSDS_EPOCH;
   else if (!strncmp(str, "J2000", 6))
      return J2000_EPOCH;

   fprintf(stderr, "Invalid string in str2epoch. Exiting...\n");
   exit(EXIT_FAILURE);
}

TimeSystem str2system(char str[JDSYSTEM_STR_LEN])
{
   if (!strncmp(str, "UTC", 4))
      return UTC_TIME;
   else if (!strncmp(str, "TAI", 4))
      return TAI_TIME;
   else if (!strncmp(str, "TT", 3))
      return TT_TIME;
   else if (!strncmp(str, "TCB", 4))
      return TCB_TIME;
   else if (!strncmp(str, "TDB", 4))
      return TDB_TIME;

   fprintf(stderr, "Invalid string in str2system. Exiting...\n");
   exit(EXIT_FAILURE);
}

void epoch2str(EpochTT epoch, char str[JDEPOCH_STR_LEN])
{
   switch (epoch) {
      case ZERO_EPOCH:
         strcpy(str, "Zero");
         break;
      case GD_CONV_EPOCH:
         strcpy(str, "GD Conversion");
         break;
      case TCB_TDB_CONV_EPOCH:
         strcpy(str, "TCB/TDB Conversion");
         break;
      case MJD_EPOCH:
         strcpy(str, "MJD");
         break;
      case J1900_EPOCH:
         strcpy(str, "J1900");
         break;
      case GMAT_MJD_EPOCH:
         strcpy(str, "GMAT MJD");
         break;
      case CCSDS_EPOCH:
         strcpy(str, "CCSDS");
         break;
      case J2000_EPOCH:
         strcpy(str, "J2000");
         break;
   }
}

void system2str(TimeSystem system, char str[JDSYSTEM_STR_LEN])
{
   switch (system) {
      case UTC_TIME:
         strcpy(str, "UTC");
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
   }
}

void jd2str(JDType jd, char str[JD_STR_LEN])
{
   const char *jdday_str_fmt       = "JD %li days, (%s) seconds, %s, Epoch %s";
   char epoch_str[JDEPOCH_STR_LEN] = {'\0'};
   char system_str[JDSYSTEM_STR_LEN] = {'\0'};
   epoch2str(jd.epoch, epoch_str);
   system2str(jd.system, system_str);

   char sec_str[RATIONAL_STR_LEN] = {'\0'};
   rat2str(jd.seconds, sec_str);
   snprintf(str, JD_STR_LEN, jdday_str_fmt, jd.whole_days, sec_str, system_str,
            epoch_str);
}

void jddays2str(JDType jd, char str[JD_STR_LEN])
{
   const char *jdday_str_fmt         = "JD %li.%s %s, Epoch %s";
   char epoch_str[JDEPOCH_STR_LEN]   = {'\0'};
   char system_str[JDSYSTEM_STR_LEN] = {'\0'};
   epoch2str(jd.epoch, epoch_str);
   system2str(jd.system, system_str);

   const Rational rat_SEC_PER_DAY = RATIONAL_NGCD(SEC_PER_DAY, 0, 1);
   Rational rat_jd_pod =
       ToRational(RationalDivide(jd.seconds, rat_SEC_PER_DAY));
   const double jd_pod = rational2double(rat_jd_pod);
   char sec_str[28]    = {'\0'};
   snprintf(sec_str, 28, "%.20lf", jd_pod);
   snprintf(str, JD_STR_LEN, jdday_str_fmt, jd.whole_days, &sec_str[2],
            system_str, epoch_str);
}

#pragma GCC diagnostic pop
#undef _jd_tai2tt
#undef _jd_tt2tai

/* #ifdef __cplusplus
** }
** #endif
*/
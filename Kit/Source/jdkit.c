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

#define sec_per_day (86400)

static Rational _epoch_pod_seconds(const EpochTT epoch)
{
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
         return InitRational(sec_per_day / 2, 0, 1);
   }
   return RATIONAL_ZERO;
}

static void _epoch_diff_tt(const EpochTT a, const EpochTT b, long *const day,
                           Rational *const part_of_day)
{
   // handle the easy cases here
   if (a == b) {
      *day         = 0;
      *part_of_day = RATIONAL_ZERO;
      return;
   }

   // determine part of day value
   *part_of_day = ToRational(RationalSub(ToRationalLL(_epoch_pod_seconds(a)),
                                         ToRationalLL(_epoch_pod_seconds(b))));
   *part_of_day = RationalAbs(*part_of_day);

   if (b == ZERO_EPOCH)
      *day = (long)(EpochValueTT(a));
   else if (a == ZERO_EPOCH)
      *day = (long)(-1.0 * EpochValueTT(b));

   // do the switches (have to do all of them due to the pragma rules)
   // TODO: some testing on how the compiler does this, hope it precomputes
   //    i.e. hoping for a lookup table by the time we get to execution time
   // maybe see about going to C23 to have constexpr?
   switch (a) {
      case J1900_EPOCH: {
         switch (b) {
            case J2000_EPOCH: {
               *day = (long)(_J1900_EPOCH_TT - _J2000_EPOCH_TT);
               break;
            }
            case GMAT_MJD_EPOCH: {
               *day = (long)(_J1900_EPOCH_TT - _GMAT_MJD_EPOCH_TT);
               break;
            }
            case MJD_EPOCH: {
               *day = (long)(_J1900_EPOCH_TT - _MJD_EPOCH_TT);
               break;
            }
            case GD_CONV_EPOCH: {
               *day = (long)(_J1900_EPOCH_TT - _GD_JD_EPOCH_TT);
               break;
            }
            case TCB_TDB_CONV_EPOCH: {
               *day = (long)(_J1900_EPOCH_TT - _TCB_TDB_EPOCH_TT);
               break;
            }
            case CCSDS_EPOCH: {
               *day = (long)(_J1900_EPOCH_TT - _CCSDS_EPOCH_TT);
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
               *day = (long)(_J2000_EPOCH_TT - _GMAT_MJD_EPOCH_TT);
               break;
            }
            case MJD_EPOCH: {
               *day = (long)(_J2000_EPOCH_TT - _MJD_EPOCH_TT);
               break;
            }
            case GD_CONV_EPOCH: {
               *day = (long)(_J2000_EPOCH_TT - _GD_JD_EPOCH_TT);
               break;
            }
            case TCB_TDB_CONV_EPOCH: {
               *day = (long)(_J2000_EPOCH_TT - _TCB_TDB_EPOCH_TT);
               break;
            }
            case CCSDS_EPOCH: {
               *day = (long)(_J2000_EPOCH_TT - _CCSDS_EPOCH_TT);
               break;
            }
            case J1900_EPOCH: {
               *day = (long)(_J2000_EPOCH_TT - _J1900_EPOCH_TT);
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
               *day = (long)(_MJD_EPOCH_TT - _J2000_EPOCH_TT);
               break;
            }
            case GMAT_MJD_EPOCH: {
               *day = (long)(_MJD_EPOCH_TT - _GMAT_MJD_EPOCH_TT);
               break;
            }
            case GD_CONV_EPOCH: {
               *day = (long)(_MJD_EPOCH_TT - _GD_JD_EPOCH_TT);
               break;
            }
            case TCB_TDB_CONV_EPOCH: {
               *day = (long)(_MJD_EPOCH_TT - _TCB_TDB_EPOCH_TT);
               break;
            }
            case CCSDS_EPOCH: {
               *day = (long)(_MJD_EPOCH_TT - _CCSDS_EPOCH_TT);
               break;
            }
            case J1900_EPOCH: {
               *day = (long)(_MJD_EPOCH_TT - _J1900_EPOCH_TT);
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
               *day = (long)(_GMAT_MJD_EPOCH_TT - _J2000_EPOCH_TT);
               break;
            }
            case MJD_EPOCH: {
               *day = (long)(_GMAT_MJD_EPOCH_TT - _MJD_EPOCH_TT);
               break;
            }
            case GD_CONV_EPOCH: {
               *day = (long)(_GMAT_MJD_EPOCH_TT - _GD_JD_EPOCH_TT);
               break;
            }
            case TCB_TDB_CONV_EPOCH: {
               *day = (long)(_GMAT_MJD_EPOCH_TT - _TCB_TDB_EPOCH_TT);
               break;
            }
            case CCSDS_EPOCH: {
               *day = (long)(_GMAT_MJD_EPOCH_TT - _CCSDS_EPOCH_TT);
               break;
            }
            case J1900_EPOCH: {
               *day = (long)(_GMAT_MJD_EPOCH_TT - _J1900_EPOCH_TT);
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
               *day = (long)(_GD_JD_EPOCH_TT - _J2000_EPOCH_TT);
               break;
            }
            case MJD_EPOCH: {
               *day = (long)(_GD_JD_EPOCH_TT - _MJD_EPOCH_TT);
               break;
            }
            case GMAT_MJD_EPOCH: {
               *day = (long)(_GD_JD_EPOCH_TT - _GMAT_MJD_EPOCH_TT);
               break;
            }
            case TCB_TDB_CONV_EPOCH: {
               *day = (long)(_GD_JD_EPOCH_TT - _TCB_TDB_EPOCH_TT);
               break;
            }
            case CCSDS_EPOCH: {
               *day = (long)(_GD_JD_EPOCH_TT - _CCSDS_EPOCH_TT);
               break;
            }
            case J1900_EPOCH: {
               *day = (long)(_GD_JD_EPOCH_TT - _J1900_EPOCH_TT);
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
               *day = (long)(_TCB_TDB_EPOCH_TT - _J2000_EPOCH_TT);
               break;
            }
            case MJD_EPOCH: {
               *day = (long)(_TCB_TDB_EPOCH_TT - _MJD_EPOCH_TT);
               break;
            }
            case GMAT_MJD_EPOCH: {
               *day = (long)(_TCB_TDB_EPOCH_TT - _GMAT_MJD_EPOCH_TT);
               break;
            }
            case GD_CONV_EPOCH: {
               *day = (long)(_TCB_TDB_EPOCH_TT - _GD_JD_EPOCH_TT);
               break;
            }
            case CCSDS_EPOCH: {
               *day = (long)(_TCB_TDB_EPOCH_TT - _CCSDS_EPOCH_TT);
               break;
            }
            case J1900_EPOCH: {
               *day = (long)(_TCB_TDB_EPOCH_TT - _J1900_EPOCH_TT);
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
               *day = (long)(_CCSDS_EPOCH_TT - _J2000_EPOCH_TT);
               break;
            }
            case MJD_EPOCH: {
               *day = (long)(_CCSDS_EPOCH_TT - _MJD_EPOCH_TT);
               break;
            }
            case GMAT_MJD_EPOCH: {
               *day = (long)(_CCSDS_EPOCH_TT - _GMAT_MJD_EPOCH_TT);
               break;
            }
            case GD_CONV_EPOCH: {
               *day = (long)(_CCSDS_EPOCH_TT - _GD_JD_EPOCH_TT);
               break;
            }
            case TCB_TDB_CONV_EPOCH: {
               *day = (long)(_CCSDS_EPOCH_TT - _TCB_TDB_EPOCH_TT);
               break;
            }
            case J1900_EPOCH: {
               *day = (long)(_CCSDS_EPOCH_TT - _J1900_EPOCH_TT);
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

   if (*day < 0) {
      *part_of_day = RationalNegate(*part_of_day);
   }
}

#define _jd_tt2tai(x) JDSubRationalSeconds((x), RATIONAL_NGCD(32, 23, 125))
#define _jd_tai2tt(x) JDAddRationalSeconds((x), RATIONAL_NGCD(32, 23, 125))

/**********************************************************************/
//  time system low level conversion helpers
/**********************************************************************/
#define L_B (1.550505e-8)
static JDType _jd_tcb2tdb(const JDType tcb_jd)
{
   // JDType jd_tt_conv = _jdtt(tcb_jd);// nope, do not
   // JDChangeEpoch(TCB_TDB_CONV_EPOCH, &jd_tt_conv);
   // TODO: pretending that we don't need this for now
   fprintf(stderr, "Julian Day conversion from TCB to TDB is not implemented. "
                   "Exiting...\n");
   exit(EXIT_FAILURE);
}
static JDType _jdtt(JDType);
static JDType _jd_tdb2tcb(JDType tdb_jd)
{
   JDType jd_tt_conv = _jdtt(tdb_jd);
   JDChangeEpoch(TCB_TDB_CONV_EPOCH, &jd_tt_conv);
   const double d_tcb_tdb = L_B * JDToDays(jd_tt_conv);

   tdb_jd.system = TCB_TIME;
   return JDAddSeconds(tdb_jd, d_tcb_tdb);
}
#undef L_B

#define TDB_COEFF1             (0.00165)
#define TDB_COEFF2             (0.00001385)
#define M_E_OFFSET             (357.5277233)
#define M_E_COEFF1             (35999.05034)
#define DAY_PER_JULIAN_CENTURY (36525.0)
static double _d_tt_tdb(JDType jd)
{
   // TODO: use spice instead if available?
   // Approximation from GMAT 2026 Mathematical Specification, p10
   // assuming input jd is tt already
   JDChangeEpoch(J2000_EPOCH, &jd);
   const double T_TT = JDToDays(jd) / DAY_PER_JULIAN_CENTURY;
   const double m_E  = fmod((M_E_OFFSET + (M_E_COEFF1 * T_TT)), 360.0) * D2R;
   return (TDB_COEFF1 * sin(m_E) + TDB_COEFF2 * sin(2.0 * m_E));
}
#undef TDB_COEFF1
#undef TDB_COEFF2
#undef M_E_OFFSET
#undef M_E_COEFF1
#undef DAY_PER_JULIAN_CENTURY

static JDType _jd_tt2tdb(JDType tt_jd)
{
   const double delta_TDB = _d_tt_tdb(tt_jd);

   tt_jd.system = TDB_TIME;
   return JDAddDays(tt_jd, delta_TDB);
}
static JDType _jd_tdb2tt(JDType tdb_jd)
{
   // Using the same approximation as GMAT, i.e. pretend we're already in TT
   JDType tdb_tt_jd       = tdb_jd;
   tdb_tt_jd.system       = TT_TIME;
   const double delta_TDB = _d_tt_tdb(tdb_tt_jd);

   tdb_jd.system = TT_TIME;
   return JDSubDays(tdb_jd, delta_TDB);
}

// UTC headaches
static JDType _jd_utc2tai(JDType utc_jd)
{
   const double leap_sec = GetLeapSec(utc_jd);

   utc_jd.system = TAI_TIME;
   return JDAddSeconds(utc_jd, leap_sec);
}
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
   JDType tai_utc_jd = tai_jd;
   tai_utc_jd.system = UTC_TIME;

   const double leap_sec = GetLeapSec(tai_utc_jd);
   tai_utc_jd            = JDSubSeconds(tai_utc_jd, leap_sec);
   const double test_ls  = GetLeapSec(tai_utc_jd);
   if (test_ls != leap_sec)
      tai_utc_jd = JDSubSeconds(tai_jd, test_ls);

   tai_utc_jd.system = UTC_TIME;
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
      case TDB_TIME:
         jd_out = _jd_tdb2tt(jd_out);
      case TT_TIME:
         jd_out = _jd_tt2tai(jd_out);
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
      case TDB_TIME:
         jd_out = _jd_tdb2tt(jd_out);
      case TT_TIME:
         jd_out = _jd_tt2tai(jd_out);
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
      case TAI_TIME:
         jd_out = _jd_tai2tt(jd_out);
      case TT_TIME:
         jd_out = _jd_tt2tdb(jd_out);
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
      case TAI_TIME:
         jd_out = _jd_tai2tt(jd_out);
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
      case TAI_TIME:
         jd_out = _jd_tai2tt(jd_out);
         break;
      case TCB_TIME:
         jd_out = _jd_tcb2tdb(jd_out);
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
   // TODO: this and other functions does not handle the time being *during* a
   // leap second

   // TODO: use spice instead if available?

   // ensure jd is UTC with MJD epoch
   // dug through GMAT source code, JD in tai-utc.dat is UTC
   JDType jd_mjd_utc = _jdutc(jd);
   JDChangeEpoch(MJD_EPOCH,
                 &jd_mjd_utc); // TODO: this causes infinite recursion due to
                               // the conversion to TT_TIME embeded within

   static long n_entries          = 0;
   static JDType *jd_list_mjd_utc = NULL; // list of JD in UTC with MJD epoch

   // TAI-UTC = offset_1 + (MJD - offset_2) x offset_3
   static double *offset_1 = NULL;
   static double *offset_2 = NULL;
   static double *offset_3 = NULL;

   if (n_entries == 0) {
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
            n_entries++;
      // use number of nonempty lines to allocate that data locations
      jd_list_mjd_utc = malloc(n_entries * sizeof(JDType));
      offset_1        = calloc(n_entries, sizeof(double));
      offset_2        = calloc(n_entries, sizeof(double));
      offset_3        = calloc(n_entries, sizeof(double));

      // rewind file and start parsing for the actual data
      rewind(file);
      int i = 0;
      while (fgets(line, 512, file)) {
         int y, d;
         char mon[16]           = {'\0'};
         double jd_mjd_utc_days = 0;
         int sscanf_check       = sscanf(
             line, "%i %s %i =JD %lf TAI-UTC= %lf S + (MJD - %lf) X %lf S", &y,
             mon, &d, &jd_mjd_utc_days, &offset_1[i], &offset_2[i],
             &offset_3[i]);

         if (sscanf_check) {
            jd_list_mjd_utc[i] =
                JDFromDays(jd_mjd_utc_days, UTC_TIME, ZERO_EPOCH);
            JDChangeEpoch(MJD_EPOCH, &jd_list_mjd_utc[i]);

            i++;
         }
      }
      fclose(file);
   }

   // TODO: double check
   for (int i = n_entries - 1; i >= 0; i--)
      if (isgreaterequal_jd(jd_mjd_utc, jd_list_mjd_utc[i]))
         return offset_1[i] +
                ((JDToDays(jd_mjd_utc) - offset_2[i]) * offset_3[i]);

   return 0;
}

// chages the time system of JD
void JDChangeSystem(const TimeSystem new_system, JDType *const jd)
{
   if (jd->system == new_system)
      return;
   switch (new_system) {
      case UTC_TIME:
         *jd = _jdutc(*jd);
         return;
      case TAI_TIME:
         *jd = _jdtai(*jd);
         return;
      case TCB_TIME:
         *jd = _jdtcb(*jd);
         return;
      case TDB_TIME:
         *jd = _jdtdb(*jd);
         return;
      case TT_TIME:
         *jd = _jdtt(*jd);
         return;
   }
}

// changes the epoch of JD
void JDChangeEpoch(const EpochTT new_epoch, JDType *const jd)
{
   if (new_epoch == jd->epoch)
      return; // nothing to do

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

   long epoch_diff_l = 0;
   Rational epoch_diff_pod_s;
   _epoch_diff_tt(jd->epoch, new_epoch, &epoch_diff_l, &epoch_diff_pod_s);

   // JDType jd_tt = _jdtt(*jd);
   *jd       = JDAddDays(*jd, epoch_diff_l);
   *jd       = JDAddRationalSeconds(*jd, epoch_diff_pod_s);
   jd->epoch = new_epoch;

   // JDChangeSystem(jd->system, &jd_tt);
   // *jd = jd_tt;
}

void JDChangeSystemEpoch(const TimeSystem new_system, const EpochTT new_epoch,
                         JDType *const jd)
{
   // This whole kit needs some testing, but this is currently how I prefer to
   // change the jd formats due to the limitations currently in JDChangeEpoch()
   JDChangeEpoch(new_epoch, jd);
   JDChangeSystem(new_system, jd);
}

// returns the number of Julian days from 'jd.epoch' according to 'jd.system'
double JDToDays(const JDType jd)
{
   return (double)jd.whole_days + (rational2double(jd.seconds)) / sec_per_day;
}

JDType JDFromDays(const double days, const TimeSystem system,
                  const EpochTT new_epoch)
{
   JDType jd               = {0};
   jd.epoch                = new_epoch;
   jd.system               = system;
   jd.whole_days           = days;
   const Rational part_day = double2rational(days - jd.whole_days);
   jd.seconds = IntegerRationalMult(sec_per_day, ToRationalLL(part_day));
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
   jd.whole_days = seconds.whole / sec_per_day;
   jd.seconds =
       RATIONAL_NGCD(seconds.whole % sec_per_day, seconds.num, seconds.den);
   return jd;
} /**********************************************************************/
/*  Converts seconds since 'epoch' in 'system' to a JDType format     */
JDType JDFromSeconds(const double seconds, const TimeSystem system,
                     const EpochTT new_epoch)
{
   JDType jd     = {0};
   jd.epoch      = new_epoch;
   jd.system     = system;
   jd.whole_days = seconds / sec_per_day;
   jd.seconds    = double2rational(fmod(seconds, sec_per_day));
   return jd;
}
/**********************************************************************/
/* Time is elapsed seconds since J2000 epoch                          */
/*  This function returns the seconds in whatever system the input    */
/*  'jd' uses                                                         */
double JDToSeconds(JDType jd)
{
   return ((double)jd.whole_days * sec_per_day + rational2double(jd.seconds));
}
Rational JDToRationalSeconds(JDType jd)
{
   // this will be ~292,271,023,045 years for a 64-bit system
   const Rat_Long max_days = _RATLONG_MAX_ / sec_per_day;
   if (jd.whole_days > max_days) {
      fprintf(stderr,
              "How in goodness name do you have Julian Days > %li in "
              "JDToRationalSeconds!?! Exiting...\n",
              max_days);
      exit(EXIT_FAILURE);
   }
   Rational out  = jd.seconds;
   out.whole    += jd.whole_days * sec_per_day;
   return out;
}
double JDToTime(JDType jd)
{
   JDChangeEpoch(J2000_EPOCH, &jd);
   return JDToSeconds(jd);
}
/**********************************************************************/
/* Time is elapsed seconds since J2000 epoch in TT time               */
double JDToDynTime(JDType jd)
{
   JDChangeSystem(TT_TIME, &jd);
   return JDToTime(jd);
}

static void _error_epoch_system(const JDType a, const JDType b,
                                const char *call_func)
{
   if (a.epoch != b.epoch || a.system != b.system) {
      fprintf(stderr,
              "In function %s, both input JDTypes must have the same system "
              "and epoch. Exiting...\n",
              call_func);
      exit(EXIT_FAILURE);
   }
}

// ensure everything in JDType is reduced, and that if whole_days < 0, then so
// are seconds.whole and seconds.num, and vice-versa
static void _reduce(JDType *const jd)
{
   jd->whole_days += RationalIntMod(&jd->seconds, sec_per_day);
   if (jd->whole_days * jd->seconds.whole < 0) {
      if (jd->whole_days > 0) {
         jd->seconds.whole += sec_per_day;
         jd->whole_days--;
      }
      else if (jd->whole_days < 0) {
         jd->seconds.whole -= sec_per_day;
         jd->whole_days++;
      }
   }
   ReduceRational(&jd->seconds);
}

JDType InitJD(const TimeSystem system, const EpochTT epoch, const long days,
              const Rational seconds)
{
   JDType jd;
   jd.epoch      = epoch;
   jd.system     = system;
   jd.whole_days = days;
   jd.seconds    = seconds;
   _reduce(&jd);
   return jd;
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
   jdout.seconds     = ToRational(
       RationalAdd(ToRationalLL(jdout.seconds), ToRationalLL(b.seconds)));
   _reduce(&jdout);
   return jdout;
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
   _reduce(&jdb);
   return JDAdd(a, jdb);
}
/**********************************************************************/
/*  Add (mul * b) seconds to the Julian Date in jd using an integer   */
/*  arithmetic multiplication algorithm                               */
JDType JDAddMultRatSecs(const JDType jd, const long mul, const Rational rat)
{
   JDType jdb = jd;

   jdb.seconds = IntegerRationalMultMod(mul, ToRationalLL(rat), sec_per_day,
                                        &jdb.whole_days);

   return JDAdd(jd, jdb);
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
   jdout.seconds     = ToRational(
       RationalSub(ToRationalLL(jdout.seconds), ToRationalLL(b.seconds)));
   _reduce(&jdout);
   return jdout;
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
   _reduce(&jdb);
   return JDSub(a, jdb);
}

JDType JDaxpy(const double a, JDType x, JDType y)
{
   // The operation 'z = a * x + y' for 'x' and 'y' being JDType and 'a' being
   // a scalar double
   // TODO: this is quite hacky to "just work" for its usage in rkkit
   x.whole_days   *= a;
   Rational a_rat  = double2rational(a);
   x.seconds =
       ToRational(RationalMult(ToRationalLL(a_rat), ToRationalLL(x.seconds)));

   x.epoch  = y.epoch;
   x.system = y.system;
   _reduce(&x);

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
   JDChangeEpoch(ZERO_EPOCH, &jd);
   _reduce(&jd);
   return (jd.whole_days > 0 ||
           (jd.whole_days == 0 && ispos_rational(jd.seconds)));
}

JDType JDNegate(JDType jd)
{
   JDChangeEpoch(ZERO_EPOCH, &jd);
   _reduce(&jd);
   jd.whole_days = -jd.whole_days;
   jd.seconds    = RationalNegate(jd.seconds);
   return jd;
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

   const Rational rat_sec_per_day = RATIONAL_NGCD(sec_per_day, 0, 1);
   Rational rat_jd_pod            = ToRational(
       RationalDivide(ToRationalLL(jd.seconds), ToRationalLL(rat_sec_per_day)));
   const double jd_pod = rational2double(rat_jd_pod);
   char sec_str[28]    = {'\0'};
   snprintf(sec_str, 28, "%.20lf", jd_pod);
   snprintf(str, JD_STR_LEN, jdday_str_fmt, jd.whole_days, &sec_str[2],
            system_str, epoch_str);
}

#pragma GCC diagnostic pop
#undef _jd_tai2tt
#undef _jd_tt2tai
#undef sec_per_day

/* #ifdef __cplusplus
** }
** #endif
*/
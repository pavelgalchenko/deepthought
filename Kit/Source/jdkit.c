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
#define _TCB_TDB_EPOCH_TT  (2443144.5) // Jan  1,  1977, 00:00:00, in tcb->tdb
#define _MJD_EPOCH_TT      (2400000.5) // Nov 17,  1858, 00:00:00
#define _J1900_EPOCH_TT    (2415019.5) // Dec 31,  1899, 00:00:00, in JD->GD
#define _GMAT_MJD_EPOCH_TT (2430000.0) // Jan  5,  1941, 12:00:00
#define _CCSDS_EPOCH_TT    (2436204.5) // Jan 1,   1958, 00:00:00
#define _J2000_EPOCH_TT    (2451545.0) // Jan 1 ,  2000, 12:00:00, J2000 epoch

// Make sure that we covered everything EXPLICITLY
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wswitch"
#pragma GCC diagnostic error "-Wswitch-enum"

static void _epoch_diff_tt(const EpochTT a, const EpochTT b, long *const day,
                           double *const part_of_day)
{
   // handle the easy cases here
   if (a == b) {
      *day         = 0;
      *part_of_day = 0.0;
      return;
   }

   // determine part of day value
   const int a_nonzero_d =
       (a == GD_CONV_EPOCH || a == TCB_TDB_CONV_EPOCH || a == MJD_EPOCH ||
        a == CCSDS_EPOCH || a == J1900_EPOCH);
   const int b_nonzero_d =
       (b == GD_CONV_EPOCH || b == TCB_TDB_CONV_EPOCH || b == MJD_EPOCH ||
        b == CCSDS_EPOCH || b == J1900_EPOCH);
   if ((a_nonzero_d && !b_nonzero_d) || (!a_nonzero_d && b_nonzero_d))
      *part_of_day = 0.5;
   else
      *part_of_day = 0.0;

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
               *day = (long)(_J1900_EPOCH_TT - CCSDS_EPOCH);
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
               *day = (long)(_GD_JD_EPOCH_TT - CCSDS_EPOCH);
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
               *day = (long)(_TCB_TDB_EPOCH_TT - CCSDS_EPOCH);
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

   if (*day < 0)
      *part_of_day *= -1.0;
}

#define sec_per_day    (86400.0)
#define _day_tt2tai(x) (x) - ((32.184) / sec_per_day)
#define _day_tai2tt(x) (x) + ((32.184) / sec_per_day)

/**********************************************************************/
//  time system low level conversion helpers
/**********************************************************************/
#define L_B (1.550505e-8)
static double _jd_tcb2tdb(const JDType tcb_jd)
{
   // JDType tt_jd_conv = _jdtt(tcb_jd);// nope, do not
   // ChangeEpoch(TCB_TDB_CONV_EPOCH, &tt_jd_conv);
   // TODO: pretending that we don't need this for now
   fprintf(stderr, "Julian Day conversion from TCB to TDB is not implemented. "
                   "Exiting...\n");
   exit(EXIT_FAILURE);
}
static JDType _jdtt(const JDType);
static double jd_tdb2tcb(const JDType tdb_jd)
{
   JDType tt_jd_conv = _jdtt(tdb_jd);
   ChangeEpoch(TCB_TDB_CONV_EPOCH, &tt_jd_conv);
   const double d_tcb_tdb = L_B * tt_jd_conv.day * sec_per_day;
   return (tdb_jd.day + d_tcb_tdb);
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
   ChangeEpoch(J2000_EPOCH, &jd);
   const double T_TT = jd.day / DAY_PER_JULIAN_CENTURY;
   const double m_E  = fmod((M_E_OFFSET + (M_E_COEFF1 * T_TT)), 360.0) * D2R;
   return (TDB_COEFF1 * sin(m_E) + TDB_COEFF2 * sin(2.0 * m_E));
}
#undef TDB_COEFF1
#undef TDB_COEFF2
#undef M_E_OFFSET
#undef M_E_COEFF1
#undef DAY_PER_JULIAN_CENTURY

static double _jd_tt2tdb(const JDType tt_jd)
{
   const double deltaTDB = _d_tt_tdb(tt_jd);
   return (tt_jd.day + deltaTDB);
}
static double _jd_tdb2tt(const JDType tdb_jd)
{
   // Using the same approximation as GMAT, i.e. pretend we're already in TT
   JDType tdb_tt_jd       = tdb_jd;
   tdb_tt_jd.system       = TT_TIME;
   const double delta_TDB = _d_tt_tdb(tdb_tt_jd);
   return (tdb_jd.day - delta_TDB);
}

// UTC headaches
static double _jd_utc2tai(const JDType utc_jd)
{
   const double leap_sec = GetLeapSec(utc_jd);

   return utc_jd.day + (leap_sec / sec_per_day);
}
static double _jd_tai2utc(const JDType tai_jd)
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

   const double leap_sec  = GetLeapSec(tai_utc_jd);
   tai_utc_jd.day        -= (leap_sec / sec_per_day);
   const double test_ls   = GetLeapSec(tai_utc_jd);
   if (test_ls != leap_sec)
      tai_utc_jd.day = tai_jd.day - (test_ls / sec_per_day);

   return tai_utc_jd.day;
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
         jd_out.day    = _jd_tcb2tdb(jd_out);
         jd_out.system = TDB_TIME;
      case TDB_TIME:
         jd_out.day    = _jd_tdb2tt(jd_out);
         jd_out.system = TT_TIME;
      case TT_TIME:
         jd_out.day    = _day_tt2tai(jd_out.day);
         jd_out.system = TAI_TIME;
      case TAI_TIME:
         jd_out.day = _jd_tai2utc(jd_out);
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
         jd_out.day = _jd_utc2tai(jd_out);
      } break;
      case TCB_TIME:
         jd_out.day    = _jd_tcb2tdb(jd_out);
         jd_out.system = TDB_TIME;
      case TDB_TIME:
         jd_out.day    = _jd_tdb2tt(jd_out);
         jd_out.system = TT_TIME;
      case TT_TIME:
         jd_out.day = _day_tt2tai(jd_out.day);
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
         jd_out.day    = _jd_utc2tai(jd_out);
         jd_out.system = TAI_TIME;
      case TAI_TIME:
         jd_out.day    = _day_tai2tt(jd_out.day);
         jd_out.system = TT_TIME;
      case TT_TIME:
         jd_out.day    = _jd_tt2tdb(jd_out);
         jd_out.system = TDB_TIME;
      case TDB_TIME:
         jd_out.day = jd_tdb2tcb(jd_out);
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
         jd_out.day    = _jd_utc2tai(jd_out);
         jd_out.system = TAI_TIME;
      case TAI_TIME:
         jd_out.day    = _day_tai2tt(jd_out.day);
         jd_out.system = TT_TIME;
      case TT_TIME:
         jd_out.day = _jd_tt2tdb(jd_out);
         break;
      case TCB_TIME:
         jd_out.day = _jd_tcb2tdb(jd_out);
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
         jd_out.day    = _jd_utc2tai(jd_out);
         jd_out.system = TAI_TIME;
      case TAI_TIME:
         jd_out.day = _day_tai2tt(jd_out.day);
         break;
      case TCB_TIME:
         jd_out.day    = _jd_tcb2tdb(jd_out);
         jd_out.system = TDB_TIME;
      case TDB_TIME:
         jd_out.day = _jd_tdb2tt(jd_out);
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
   // TODO: use spice instead if available?

   // ensure jd is UTC with MJD epoch
   // dug through GMAT source code, JD in tai-utc.dat is UTC
   JDType jd_mjd_utc = _jdutc(jd);
   ChangeEpoch(MJD_EPOCH,
               &jd_mjd_utc); // TODO: this causes infinite recursion due to the
                             // conversion to TT_TIME embeded within

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
         char mon[16] = {'\0'};

         int sscanf_check = sscanf(
             line, "%i %s %i =JD %lf TAI-UTC= %lf S + (MJD - %lf) X %lf S", &y,
             mon, &d, &jd_list_mjd_utc[i].day, &offset_1[i], &offset_2[i],
             &offset_3[i]);

         if (sscanf_check) {
            jd_list_mjd_utc[i].epoch  = ZERO_EPOCH;
            jd_list_mjd_utc[i].system = UTC_TIME;
            ChangeEpoch(MJD_EPOCH, &jd_list_mjd_utc[i]);

            i++;
         }
      }
      fclose(file);
   }

   // TODO: double check
   for (int i = n_entries - 1; i >= 0; i--)
      if (jd_mjd_utc.day >= jd_list_mjd_utc[i].day)
         return offset_1[i] + ((jd_mjd_utc.day - offset_2[i]) * offset_3[i]);

   return 0;
}

// chages the time system of JD
void ChangeSystem(const TimeSystem new_system, JDType *const jd)
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
void ChangeEpoch(const EpochTT new_epoch, JDType *const jd)
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

   long epoch_diff_l   = 0;
   double epoch_diff_d = 0.0;
   _epoch_diff_tt(jd->epoch, new_epoch, &epoch_diff_l, &epoch_diff_d);

   // JDType jd_tt = _jdtt(*jd);
   double jd_l = 0.0, jd_d = 0.0;
   jd_d      = modf(jd->day, &jd_l);
   jd->day   = (double)((long)jd_l + epoch_diff_l) + (jd_d + epoch_diff_d);
   jd->epoch = new_epoch;

   // ChangeSystem(jd->system, &jd_tt);
   // *jd = jd_tt;
}

void ChangeSystemEpoch(const TimeSystem new_system, const EpochTT new_epoch,
                       JDType *const jd)
{
   // This whole kit needs some testing, but this is currently how I prefer to
   // change the jd formats due to the limitations currently in ChangeEpoch()
   ChangeEpoch(new_epoch, jd);
   ChangeSystem(new_system, jd);
}

#pragma GCC diagnostic pop
#undef tdt2tdb
#undef tdb2tdt
#undef sec_per_day

/* #ifdef __cplusplus
** }
** #endif
*/
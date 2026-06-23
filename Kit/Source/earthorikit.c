/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "earthorikit.h"
#include "42constants.h"
#include "dcmkit.h"
#include "defineskit.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>

/**********************************************************************/
/* Load Earth Orientation Parameter data                              */

// Loading into memory like in GMAT
// TODO: the file is >23000 rows, do we want to do this?
//    loading all rows and just the data show is > 1.2 GB!!!!!!!!
//    each row corresponds to a single day
//    lets try storing ~once/wk (~175 MB now)
//    file is from Jan 1, 1962 to Sep 14, 2026, could cutoff before ~2000?

#define EOP_STEPOVER (1)
struct Ut1MUtcEntry {};
struct Ut1MUtcInfo {
   int n_entries;
   double *jday_tai_mjd; // TAI days since MJD
   int *jday_utc_mjd;    // UTC days since MJD (unused??)
   double *ut1_m_utc;    // UT1 - UTC (sec)
} Ut1MUtcInfo;

struct PolarMotionEntry {};
static struct PolarMotionInfo {
   int n_entries;
   double *jday_utc_mjd; // UTC days since MJD
   double *x;
   double *y;
   double *lod; // (unused??)
} PM_Info;

// TODO: Meeus AStronomical Algorithms pp 79 has a TT(?)-UT(1?) limited table
// for times before eopc04_08.62-now; could use to fill in values
static __once_flag eop_file_flag = __ONCE_FLAG_INIT;
static void init_eop_file()
{
   extern char ModelPath[1000];
   char f_path[1064] = {'\0'};
   strcpy(f_path, ModelPath);
   strcat(f_path, "/data_files/eopc04_08.62-now");

   FILE *file = fopen(f_path, "rt");
   if (file == NULL) {
      fprintf(stderr, "Error opening eopc04_08.62-now file '%s'. Exiting...\n",
              f_path);
      exit(EXIT_FAILURE);
   }

   char line[512] = {'\0'};
   // loop over file to find last line of header
   int startfound = FALSE;
   while (!startfound && fgets(line, 512, file) != NULL) {
      // NOTE: this line being unchanged is assumed
      if (strstr(line, "(0h UTC)")) {
         startfound = TRUE;
         if (fgets(line, 512, file) == NULL) {
            fprintf(stderr, "No Data in '%s' file. Exiting...\n", f_path);
            exit(EXIT_FAILURE);
         }
      }
   }
   if (!startfound) {
      fprintf(stderr, "Unable to read '%s' file. Exiting...\n", f_path);
      exit(EXIT_FAILURE);
   }

   Ut1MUtcInfo.n_entries    = 0;
   Ut1MUtcInfo.ut1_m_utc    = NULL;
   Ut1MUtcInfo.jday_utc_mjd = NULL;
   PM_Info.n_entries        = 0;
   PM_Info.x                = NULL;
   PM_Info.y                = NULL;
   PM_Info.lod              = NULL;
   while (fgets(line, 512, file) != NULL) {
      int year, month, day, jday_utc_mjd;
      double x, y, ut1_m_utc, lod;

      if (sscanf(line, "%i %i %i %i %lf %lf %lf %lf", &year, &month, &day,
                 &jday_utc_mjd, &x, &y, &ut1_m_utc, &lod) != 8) {
         fprintf(stderr,
                 "Could not find required eop data in data line '%i' of file "
                 "'%s'. Exiting...\n",
                 Ut1MUtcInfo.n_entries, f_path);
         exit(EXIT_FAILURE);
      }

      if ((jday_utc_mjd % EOP_STEPOVER) == 0) {
         Ut1MUtcInfo.jday_tai_mjd =
             realloc(Ut1MUtcInfo.jday_tai_mjd,
                     (Ut1MUtcInfo.n_entries + 1) * sizeof(double));
         Ut1MUtcInfo.ut1_m_utc =
             realloc(Ut1MUtcInfo.ut1_m_utc,
                     (Ut1MUtcInfo.n_entries + 1) * sizeof(double));
         Ut1MUtcInfo.jday_utc_mjd =
             realloc(Ut1MUtcInfo.jday_utc_mjd,
                     (Ut1MUtcInfo.n_entries + 1) * sizeof(int));

         PM_Info.jday_utc_mjd = realloc(
             PM_Info.jday_utc_mjd, (PM_Info.n_entries + 1) * sizeof(double));
         PM_Info.x =
             realloc(PM_Info.x, (PM_Info.n_entries + 1) * sizeof(double));
         PM_Info.y =
             realloc(PM_Info.y, (PM_Info.n_entries + 1) * sizeof(double));
         PM_Info.lod =
             realloc(PM_Info.lod, (PM_Info.n_entries + 1) * sizeof(double));

         Ut1MUtcInfo.jday_utc_mjd[Ut1MUtcInfo.n_entries] = jday_utc_mjd;
         const JDType jd_utc_mjd = DaysToJD(jday_utc_mjd, UTC_TIME, MJD_EPOCH);

         Ut1MUtcInfo.jday_tai_mjd[Ut1MUtcInfo.n_entries] =
             JDToDays(JDChangeSystem(TAI_TIME, jd_utc_mjd));
         Ut1MUtcInfo.jday_utc_mjd[Ut1MUtcInfo.n_entries] = jday_utc_mjd;
         Ut1MUtcInfo.ut1_m_utc[Ut1MUtcInfo.n_entries]    = ut1_m_utc;

         PM_Info.jday_utc_mjd[PM_Info.n_entries] = jday_utc_mjd;
         PM_Info.x[PM_Info.n_entries]            = x;
         PM_Info.y[PM_Info.n_entries]            = y;
         PM_Info.lod[PM_Info.n_entries]          = lod;

         Ut1MUtcInfo.n_entries++;
         PM_Info.n_entries++;
      }
   }
}
/**********************************************************************/
/* Read ut1 - utc data and clampled cubic spline interpolation        */
/* between stored data points                                         */
double GetUt1UtcOffset(const double jday_tai_mjd)
{
   call_once(&eop_file_flag, init_eop_file);
   static int ind = 0;

   const int n_entries              = Ut1MUtcInfo.n_entries;
   const double *const tbl_jday_tai = Ut1MUtcInfo.jday_tai_mjd;

   // if 'jday_tai_mjd' is outside the table, return the nearest extent
   if (tbl_jday_tai[0] >= jday_tai_mjd)
      return Ut1MUtcInfo.ut1_m_utc[0];
   else if (jday_tai_mjd >= tbl_jday_tai[n_entries - 1])
      return Ut1MUtcInfo.ut1_m_utc[n_entries - 1];

   // From the above, 'jday_tai_mjd' will be in the open interval
   //    (tbl_jday[0], tbl_jday[n_entries - 1])
   // thus, 'ind' will be on the closed interval
   //    [0, n_entries - 2]
   ind = FindIndex(jday_tai_mjd, tbl_jday_tai, Ut1MUtcInfo.n_entries, ind);

   double yp[2];

   yp[0] =
       CentralDifference(ind, n_entries, tbl_jday_tai, Ut1MUtcInfo.ut1_m_utc);
   yp[1] = CentralDifference(ind + 1, n_entries, tbl_jday_tai,
                             Ut1MUtcInfo.ut1_m_utc);

   const double *const dut1 = &Ut1MUtcInfo.ut1_m_utc[ind];
   const double *const X    = &tbl_jday_tai[ind];

   return ClampedCubicSpline(jday_tai_mjd, X[0], X[1], dut1[0], dut1[1], yp[0],
                             yp[1]);
}
/**********************************************************************/
/* Yields x and y in arcseconds                                       */
void GetPolarMotionData(const double jday_utc_mjd, double *const xp,
                        double *const yp, double *const lodp)
{
   call_once(&eop_file_flag, init_eop_file);
   static int ind = 0;

   const int n_entries              = PM_Info.n_entries;
   const double *const tbl_jday_utc = PM_Info.jday_utc_mjd;
   ind = FindIndex(jday_utc_mjd, tbl_jday_utc, n_entries, ind);

   const double *const x   = &PM_Info.x[ind];
   const double *const y   = &PM_Info.y[ind];
   const double *const lod = &PM_Info.lod[ind];
   const double Yi2[3]     = {x[1], y[1], lod[1]};
   const double Yi1[3]     = {x[0], y[0], lod[0]};
   double vals[3]          = {0};
   lerpV(tbl_jday_utc[ind + 1], tbl_jday_utc[ind], Yi2, Yi1, jday_utc_mjd, 3,
         vals);
   *xp   = vals[0];
   *yp   = vals[1];
   *lodp = vals[2];

   // const double *const T = &tbl_jday_utc[ind];

   // double yprime[2];
   // yprime[0] = CentralDifference(ind, n_entries, tbl_jday_utc, PM_Info.x);
   // yprime[1] = CentralDifference(ind + 1, n_entries, tbl_jday_utc,
   // PM_Info.x); *xp       = ClampedCubicSpline(jday_utc_mjd, T[0], T[1],
   // PM_Info.x[ind],
   //                                PM_Info.x[ind + 1], yp[0], yp[1]);
   // yprime[0] = CentralDifference(ind, n_entries, tbl_jday_utc, PM_Info.y);
   // yprime[1] = CentralDifference(ind + 1, n_entries, tbl_jday_utc,
   // PM_Info.y); *yp       = ClampedCubicSpline(jday_utc_mjd, T[0], T[1],
   // PM_Info.y[ind],
   //                                PM_Info.y[ind + 1], yp[0], yp[1]);
   // yprime[0] = CentralDifference(ind, n_entries, tbl_jday_utc, PM_Info.lod);
   // yprime[1] = CentralDifference(ind + 1, n_entries, tbl_jday_utc,
   // PM_Info.lod); *lodp     = ClampedCubicSpline(jday_utc_mjd, T[0], T[1],
   // PM_Info.lod[ind],
   //                                PM_Info.lod[ind + 1], yp[0], yp[1]);
}
/**********************************************************************/
/* IAU Nutation data                                                  */

// using IAU 1980 or IAU 1996
// default to IAU 1980
#define N_NUT_MAX (263)

#define NUT_N_1950            (69)
#define NUT_ORDER_1950        (3)
#define NUT_MULT_1950         (1.0e-04) // arcseconds
#define NUT_FIRST_PHRASE_1950 ("1950 IAU")

#define NUT_N_1980            (106)
#define NUT_ORDER_1980        (3)
#define NUT_MULT_1980         (1.0e-04) // arcseconds
#define NUT_FIRST_PHRASE_1980 ("1980 IAU")

#define NUT_N_1996            (263)
#define NUT_ORDER_1996        (4)
#define NUT_MULT_1996         (1.0e-07) // arcseconds
#define NUT_FIRST_PHRASE_1996 ("1996 IAU")

struct NutEntry {
   int a[5];
   double A;
   double B;
   double C;
   double D;
   double E;
   double F;
   int index;
};

static struct NutInfo {
   long n_entries;
   double mult;
   char first_phrase[10];
   int order;
   struct NutEntry entries[N_NUT_MAX];
} Nut_Info;

// Default to using ITRF 1980 data
// static const enum NutEnum NutSelection = NUT_ITRF_1950;
static const enum NutEnum NutSelection = NUT_ITRF_1980;
// static const enum NutEnum NutSelection = NUT_ITRF_1996;

static __once_flag iau_file_flag = __ONCE_FLAG_INIT;
static void init_iau_file()
{
   extern char ModelPath[1000];
   char f_path[1064] = {'\0'};
   strcpy(f_path, ModelPath);
   strcat(f_path, "/data_files/NUTATION.DAT");

   switch (NutSelection) {
      case NUT_ITRF_1950:
         Nut_Info.n_entries = NUT_N_1950;
         Nut_Info.mult      = NUT_MULT_1950;
         Nut_Info.order     = NUT_ORDER_1950;
         strcpy(Nut_Info.first_phrase, NUT_FIRST_PHRASE_1950);
         break;
      case NUT_ITRF_1980:
         Nut_Info.n_entries = NUT_N_1980;
         Nut_Info.mult      = NUT_MULT_1980;
         Nut_Info.order     = NUT_ORDER_1980;
         strcpy(Nut_Info.first_phrase, NUT_FIRST_PHRASE_1980);
         break;
      case NUT_ITRF_1996:
         Nut_Info.n_entries = NUT_N_1996;
         Nut_Info.mult      = NUT_MULT_1996;
         Nut_Info.order     = NUT_ORDER_1996;
         strcpy(Nut_Info.first_phrase, NUT_FIRST_PHRASE_1996);
         break;
      default:
         break;
   }

   FILE *file = fopen(f_path, "rt");
   if (file == NULL) {
      fprintf(stderr, "Error opening NUTATION.DAT file '%s'. Exiting...\n",
              f_path);
      exit(EXIT_FAILURE);
   }

   // loop over file to find start of desired IAU section
   char line[512] = {'\0'};
   int foundline  = FALSE;
   while (fgets(line, 512, file) != NULL) {
      if (strstr(line, Nut_Info.first_phrase)) {
         foundline = TRUE;
         break;
      }
   }

   if (!foundline) {
      fprintf(stderr, "Unable to find '%s' in '%s'. Exiting...\n",
              Nut_Info.first_phrase, f_path);
      exit(EXIT_FAILURE);
   }

   // skip column headings
   fgets(line, 512, file);
   if (strstr(line, "a2") == NULL) {
      fprintf(stderr,
              "ITRF nutation file '%s' not in expected format. Exiting...\n",
              f_path);
      exit(EXIT_FAILURE);
   }
   for (int i = 0; i < Nut_Info.n_entries; i++) {
      if (fgets(line, 512, file) == NULL) {
         fprintf(stderr,
                 "ITRF nutation file '%s' ended before reading all expected "
                 "values. Exiting...\n",
                 f_path);
         exit(EXIT_FAILURE);
      }
      struct NutEntry *entry = &Nut_Info.entries[i];
      switch (NutSelection) {
         case NUT_ITRF_1950: // no E or F terms
         case NUT_ITRF_1980: // no E or F terms
            sscanf(line, "%i %i %i %i %i %lf %lf %lf %lf %i", &entry->a[0],
                   &entry->a[1], &entry->a[2], &entry->a[3], &entry->a[4],
                   &entry->A, &entry->B, &entry->C, &entry->D, &entry->index);
            entry->E = 0;
            entry->F = 0;
            break;
         default:
            sscanf(line, "%i %i %i %i %i %lf %lf %lf %lf %lf %lf %i",
                   &entry->a[0], &entry->a[1], &entry->a[2], &entry->a[3],
                   &entry->a[4], &entry->A, &entry->B, &entry->C, &entry->D,
                   &entry->E, &entry->F, &entry->index);
            break;
      }
   }
   fclose(file);
}
/**********************************************************************/
__attribute__((const)) static mat3x3_t EarthPrecessionMatrix(const double TTDB);
static mat3x3_t EarthPrecessionMatrix(const double TTDB)
{
   const double zeta  = (2306.2181 + (0.30188 + 0.017998 * TTDB) * TTDB) * TTDB;
   const double Theta = (2004.3109 - (0.42665 + 0.041833 * TTDB) * TTDB) * TTDB;
   const double z     = zeta + (0.7928 + 0.000205 * TTDB) * TTDB * TTDB;

   mat3x3_t PREC = A2C(323, -z * A2R, Theta * A2R, -zeta * A2R);
   PREC = MxM(ROT3(-z * A2R), MxM(ROT2(Theta * A2R), ROT3(-zeta * A2R)));

   return PREC;
}
/**********************************************************************/
/* The coefficients in each row are in the order:                     */
/*    meanAnomLuna                                                    */
/*    meanAnomSol                                                     */
/*    argLatLuna                                                      */
/*    meanElongSol                                                    */
/*    longAscNodeLuna                                                 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
__attribute__((const)) static inline const double *const
NutSolarLunarPosition_R_Rot(const enum NutEnum selection)
{
   static double zero_rs[5] = {0, 0, 0, 0, 0};
   switch (selection) {
      case NUT_ITRF_1950:
      case NUT_ITRF_1980: {
         static double r_rot[5] = {1325, 99, 1342, 1236, -5};
         return r_rot;
      } break;
      case NUT_ITRF_1996: {
         static double r_rot[5] = {1325, 99, 1342, 1236, -5};
         return r_rot;
      } break;
      default:
         break;
   }
   return zero_rs;
}
__attribute__((const)) static inline const double *const
NutSolarLunarPosition_ArcSec(const enum NutEnum selection, const int order_i)
{
   static double zero_coeffs[5] = {0, 0, 0, 0, 0};
   switch (selection) {
      case NUT_ITRF_1950:
      case NUT_ITRF_1980: {
         static double coeffs_asec[NUT_ORDER_1980 + 1][5] = {
             {485866.733, 1287099.804, 335778.877, 1072261.307, 450160.280},
             {715922.633, 1292581.224, 295263.137, 1105601.328, -482890.539},
             {31.310, -0.577, -13.257, -6.891, 7.455},
             {0.064, -0.012, 0.011, 0.019, 0.008}};
         if (order_i >= NUT_ORDER_1980 + 1)
            return zero_coeffs;
         return coeffs_asec[order_i];
      } break;
      case NUT_ITRF_1996: {
         static double coeffs_asec[NUT_ORDER_1996 + 1][5] = {
             {485868.249036, 1287104.793048, 335779.526232, 1072260.703692,
              450160.398036},
             {715923.2178, 1292581.0481, 295262.8478, 1105601.2090,
              -482890.2665},
             {31.8792, -0.5532, -12.7512, -6.3706, 7.4722},
             {0.051635, -0.000136, 0.001037, 0.006593, 0.007702},
             {-0.00024470, -0.00001149, 0.00000417, -0.00003169, -0.00005939}};
         if (order_i >= NUT_ORDER_1996 + 1)
            return zero_coeffs;
         return coeffs_asec[order_i];
      } break;
      default:
         break;
   }
   return zero_coeffs;
}
#pragma GCC diagnostic pop
//**********************
static mat3x3_t EarthNutationMatrix(const double TTDB, double *const dPsi,
                                    double *const longAscNodeLuna_ret,
                                    double *const cosEps)
{
   call_once(&iau_file_flag, init_iau_file);

   double nut_angles[5] = {0};

   /* Mean anomaly of the Moon's orbit (rad)                            */
   double *const meanAnomLuna = &nut_angles[0];
   /* Mean anomaly of the Sun's orbit (rad)                             */
   double *const meanAnomSol = &nut_angles[1];
   /* Mean argument of latitude of the Moon's orbit (rad)                */
   double *const argLatLuna = &nut_angles[2];
   /* Difference between the mean longitude of the Sun and Moon (rad)    */
   double *const meanElongSol = &nut_angles[3];
   /* Mean longitude of the ascending node of the Moon's orbit (rad)    */
   double *const longAscNodeLuna = &nut_angles[4];

   // .. Accumulate the various Sun & Moon position angles
   double x = 1;
   for (int order_i = 0; order_i < Nut_Info.order + 1; order_i++) {
      const double *const coeffs_asec =
          NutSolarLunarPosition_ArcSec(NutSelection, order_i);

      for (int j = 0; j < 5; j++)
         nut_angles[j] += fmod(x * coeffs_asec[j], 360 * D2A);
      x *= TTDB;
   }

   // .. Accumulate the constant and complete rotation terms, then map to 2 pi
   const double *const r_rot = NutSolarLunarPosition_R_Rot(NutSelection);
   for (int j = 0; j < 5; j++) {
      nut_angles[j] += fmod(r_rot[j] * TTDB, 1.0) * 360 * D2A;
      nut_angles[j]  = fmod(nut_angles[j], 360 * D2A);
   }

   *longAscNodeLuna_ret = *longAscNodeLuna * A2R;

   /* Mean Obliquity of the ecliptic at J2000 epoch (arcsec)                */
   const double Epsbar =
       (84381.448 + (-46.8150 + (-0.00059 + 0.001813 * TTDB) * TTDB) * TTDB) *
       A2R;

   *dPsi       = 0;
   double dEps = 0;
   // start at end since list is sorted with descending A coefficients
   struct NutEntry *entry = &Nut_Info.entries[Nut_Info.n_entries - 1];
   for (; entry >= Nut_Info.entries; entry--) {
      const double apNut =
          (entry->a[0] * (*meanAnomLuna) + entry->a[1] * (*meanAnomSol) +
           entry->a[2] * (*argLatLuna) + entry->a[3] * (*meanElongSol) +
           entry->a[4] * (*longAscNodeLuna)) *
          A2R;
      const double cosAp = cos(apNut);
      const double sinAp = sin(apNut);

      if (NutSelection != NUT_ITRF_1980) {
         *dPsi += (entry->A + entry->B * TTDB) * sinAp + entry->E * cosAp;
         dEps  += (entry->C + entry->D * TTDB) * cosAp + entry->F * sinAp;
      }
      else {
         *dPsi += (entry->A + entry->B * TTDB) * sinAp;
         dEps  += (entry->C + entry->D * TTDB) * cosAp;
      }
   }
   *dPsi *= A2R * Nut_Info.mult;
   dEps  *= A2R * Nut_Info.mult;

   const double Eps = Epsbar + dEps;
   *cosEps          = cos(Epsbar);
   mat3x3_t NUT     = A2C(131, -Eps, -*dPsi, Epsbar);

   NUT = MxM(ROT1(-Eps), MxM(ROT3(-*dPsi), ROT1(Epsbar)));
   return NUT;
}
/**********************************************************************/
/* Greenwich Mean Sidereal Time at UT=0 (sec)                         */
double GMAT_JD2GMST0(const JDType jd)
{
   const JDType jd_ut1_j2000 = JDChangeSystemEpoch(UT1_TIME, J2000_EPOCH, jd);

   const double T0UT1 =
       (floor(JDToDays(jd_ut1_j2000)) + 0.5) / JDDAY_PER_CENTURY;

   // NOTE: 1 sec = 15"; 1 hour (= 15 deg) = 54000"
   const double hr2deg  = 15.0;
   const double sec2deg = hr2deg / (SEC_PER_HOUR);

   const double out =
       (24110.54841 +
        (8640184.812866 + (0.093104 - 0.0000062 * T0UT1) * T0UT1) * T0UT1) *
       sec2deg;
   return fmod(out, 360);
}
/**********************************************************************/
/* Greenwich Mean Sidereal Time (sec)                                 */
double GMAT_JD2GMST(const JDType jd)
{
   const JDType jd_ut1_j2000 = JDChangeSystemEpoch(UT1_TIME, J2000_EPOCH, jd);

   // TODO: should be ut1
   const JDType t0_jd_ut1_j2000 =
       JD_RAW(UT1_TIME, J2000_EPOCH, jd_ut1_j2000.whole_days,
              JDSECOND_RAW(SEC_PER_DAY / 2, 0, 1));

   const double T0UT1      = JDToDays(t0_jd_ut1_j2000) / JDDAY_PER_CENTURY;
   const double TUT1       = JDToDays(jd_ut1_j2000) / JDDAY_PER_CENTURY;
   const double sec_ut1day = JDSubToSeconds(jd_ut1_j2000, t0_jd_ut1_j2000);

   // NOTE: 1 sec = 15"; 1 hour (= 15 deg) = 54000"
   const double secperdeg = 240.0;

   // 'Satellite Orbits: Models, Methods, Applications' by Montenbruck and Gill,
   // Eq (5.19)
   // const double sec_GMST = (24110.54841 + 1.002737909350795 * sec_ut1day +
   //                          (fmod(8640184.812866 * T0UT1, SEC_PER_DAY) +
   //                           (9.3104e-02 - 6.2e-06 * TUT1) * TUT1 * TUT1));
   const double sec_GMST =
       (67310.548 + (fmod((3155760000.0 + 8640184.812866) * TUT1, 86400) +
                     (9.3104e-02 - 6.2e-06 * TUT1) * TUT1 * TUT1));
   const double rad_GMST = WrapTo2Pi((sec_GMST / secperdeg) * D2R);

   // 'Astronomical Algorithms' by Meeus, Eq (12.4)
   // const double deg_GMST =
   //     (280.4661837 + (360.98564736629 * JDToDays(jd_ut1_j2000) +
   //                     TUT1 * TUT1 * (3.87933E-4 - TUT1 / 3.871E7)));
   // const double rad_GMST = WrapTo2Pi(deg_GMST * D2R);
   return rad_GMST;
}
/**********************************************************************/
__attribute__((const)) static double JD2GAST(const JDType jd, const double dPsi,
                                             const double longAscNodeLuna,
                                             const double cosEps);
static double JD2GAST(const JDType jd, const double dPsi,
                      const double longAscNodeLuna, const double cosEps)
{
   // jdEQThresh is Jan 1, 1997 UTC
   const JDType jdEQThresh   = JD_RAW(UT1_TIME, J2000_EPOCH, -1095,
                                      JDSECOND_RAW(SEC_PER_DAY / 2, 0, 1));
   const JDType jd_utc_j2000 = JDChangeSystemEpoch(UT1_TIME, J2000_EPOCH, jd);

   double eq_equinox = dPsi * cosEps;
   if (isgreater_jd(jd_utc_j2000, jdEQThresh))
      eq_equinox +=
          (2.64 * sin(longAscNodeLuna) + 6.3e-02 * sin(2.0 * longAscNodeLuna)) *
          1.0e-3 * A2R;

   const double GMST = GMAT_JD2GMST(jd);
   return GMST + eq_equinox; //- 1.18 * A2R;
   // return GMST - 1.16 * A2R;
}
/**********************************************************************/
__attribute__((const)) mat3x3_t EarthPolarMotion(const JDType jd);
mat3x3_t EarthPolarMotion(const JDType jd)
{
   JDType jd_utc_mjd = JDChangeSystemEpoch(UTC_TIME, MJD_EPOCH, jd);
   double x, y, lod = 0;
   GetPolarMotionData(JDToDays(jd_utc_mjd), &x, &y, &lod);

   mat3x3_t PM = A2C(21, -x * A2R, -y * A2R, 0);
   PM          = MxM(ROT2(-x * A2R), ROT1(-y * A2R));

   return PM;
}
/**********************************************************************/
/* returns Apparent Sidereal Time and Earth CWN                       */
pair_dbl_mat3x3_t HiFiEarthCWN(const JDType jd)
{
   double dPsi, longAscNodeLuna, cosEps;
   pair_dbl_mat3x3_t out;
   double *const GAST  = &out.dbl;
   mat3x3_t *const CWN = &out.mat;

   const JDType jd_tdb_j2000 = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);
   const JDType jd_utc_j2000 = JDChangeSystemEpoch(UTC_TIME, J2000_EPOCH, jd);
   const double TTDB         = JDToDays(jd_tdb_j2000) / JDDAY_PER_CENTURY;

   const mat3x3_t PREC = EarthPrecessionMatrix(TTDB);
   const mat3x3_t NUT =
       EarthNutationMatrix(TTDB, &dPsi, &longAscNodeLuna, &cosEps);
   *GAST = JD2GAST(jd_utc_j2000, dPsi, longAscNodeLuna, cosEps);

   mat3x3_t ST       = ROT3(*GAST);
   const mat3x3_t PM = EarthPolarMotion(jd_utc_j2000);

   // *CWN = MxM(PM, MxM(ST, MxM(NUT, PREC)));
   *CWN = MxM(ST, MxM(NUT, PREC));
   return out;
}
/**********************************************************************/
/*  Find Greenwich Mean Sidereal Time (GMST)                          */
/*  Ref. Jean Meeus, 'Astronomical Algorithms', QB51.3.E43M42, 1991.  */
/*  GMST is output in units of days.                                  */
double JD2GMST(JDType jd)
{
   double T, JD0, GMST0, GMST;

   jd              = JDChangeSystemEpoch(UTC_TIME, J2000_EPOCH, jd);
   const double JD = JDToDays(jd);

   JD0 = floor(JD) + 0.5;

   T = JD0 / 36525.0;

   /* .. GMST at UT=0h, in deg */
   GMST0 =
       100.46061837 + T * (36000.770053608 + T * (3.87933E-4 - T / 3.871E7));

   /* .. Convert to days */
   GMST0 /= 360.0;

   GMST = GMST0 + 1.00273790935 * (JD - JD0);
   GMST = fmod(GMST, 1.0);
   if (GMST < 0)
      GMST += 1.0;
   return (GMST);
}
/**********************************************************************/
/* Find coordinate transformation from True Equator True Equinox      */
/* (TETE) frame to J2000 frame.  Ref "The Astronomical Almanac",      */
/* QB8.U5, 2003, p. B18,B20.                                          */
/* TEME to TETE rotation from Montenbruck adn Gill (TL1080.M66)       */
pair_mat3x3_t SimpleEarthPrecNute(JDType jd_tt_j2000)
{
   double d, arg1, arg2, dpsi, deps, eps;
   double T, z, theta, zeta;
   mat3x3_t N, P;
   double c1, s1, c2, s2, c3, s3;
   double dR;
   pair_mat3x3_t pair;
   mat3x3_t *const C_TEME_TETE  = &pair.first;
   mat3x3_t *const C_TETE_J2000 = &pair.second;

   jd_tt_j2000 = JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, jd_tt_j2000);
   const double days_tt_j2000 = JDToDays(jd_tt_j2000);

   /* TETE to MEME (Nutation) */
   d    = days_tt_j2000 - 1094.5;
   arg1 = (67.1 - 0.053 * d) * D2R;
   arg2 = (198.5 + 1.971 * d) * D2R;
   dpsi = (-0.0048 * sin(arg1) - 0.0004 * sin(arg2)) * D2R;
   deps = (0.0026 * cos(arg1) + 0.0002 * cos(arg2)) * D2R;
   eps  = 23.44 * D2R;

   N.mat[0][0] = 1.0;
   N.mat[1][1] = 1.0;
   N.mat[2][2] = 1.0;
   N.mat[2][1] = deps;
   N.mat[2][0] = dpsi * sin(eps);
   N.mat[1][0] = dpsi * cos(eps);
   N.mat[1][2] = -N.mat[2][1];
   N.mat[0][2] = -N.mat[2][0];
   N.mat[0][1] = -N.mat[1][0];

   /* MEME to J2000 (Precession) */
   T     = days_tt_j2000 / 36525.0;
   z     = D2R * ((0.6406161 + (3.041E-4 + 5.10E-6 * T) * T) * T);
   theta = D2R * ((0.5567530 - (1.185E-4 + 1.16E-5 * T) * T) * T);
   zeta  = D2R * ((0.6406161 + (8.390E-5 + 5.00E-6 * T) * T) * T);

   c1          = cos(-zeta);
   s1          = sin(-zeta);
   c2          = cos(theta);
   s2          = sin(theta);
   c3          = cos(-z);
   s3          = sin(-z);
   P.mat[0][0] = c1 * c2 * c3 - s3 * s1;
   P.mat[1][0] = -c1 * c2 * s3 - c3 * s1;
   P.mat[2][0] = c1 * s2;
   P.mat[0][1] = s1 * c2 * c3 + s3 * c1;
   P.mat[1][1] = -s1 * c2 * s3 + c3 * c1;
   P.mat[2][1] = s1 * s2;
   P.mat[0][2] = -s2 * c3;
   P.mat[1][2] = s2 * s3;
   P.mat[2][2] = c2;

   /* TETE to J2000 (Precession, then Nutation) */
   C_TETE_J2000->mat[0][0] = N.mat[0][0] * P.mat[0][0] +
                             N.mat[0][1] * P.mat[1][0] +
                             N.mat[0][2] * P.mat[2][0];
   C_TETE_J2000->mat[0][1] = N.mat[0][0] * P.mat[0][1] +
                             N.mat[0][1] * P.mat[1][1] +
                             N.mat[0][2] * P.mat[2][1];
   C_TETE_J2000->mat[0][2] = N.mat[0][0] * P.mat[0][2] +
                             N.mat[0][1] * P.mat[1][2] +
                             N.mat[0][2] * P.mat[2][2];
   C_TETE_J2000->mat[1][0] = N.mat[1][0] * P.mat[0][0] +
                             N.mat[1][1] * P.mat[1][0] +
                             N.mat[1][2] * P.mat[2][0];
   C_TETE_J2000->mat[1][1] = N.mat[1][0] * P.mat[0][1] +
                             N.mat[1][1] * P.mat[1][1] +
                             N.mat[1][2] * P.mat[2][1];
   C_TETE_J2000->mat[1][2] = N.mat[1][0] * P.mat[0][2] +
                             N.mat[1][1] * P.mat[1][2] +
                             N.mat[1][2] * P.mat[2][2];
   C_TETE_J2000->mat[2][0] = N.mat[2][0] * P.mat[0][0] +
                             N.mat[2][1] * P.mat[1][0] +
                             N.mat[2][2] * P.mat[2][0];
   C_TETE_J2000->mat[2][1] = N.mat[2][0] * P.mat[0][1] +
                             N.mat[2][1] * P.mat[1][1] +
                             N.mat[2][2] * P.mat[2][1];
   C_TETE_J2000->mat[2][2] = N.mat[2][0] * P.mat[0][2] +
                             N.mat[2][1] * P.mat[1][2] +
                             N.mat[2][2] * P.mat[2][2];

   /* TEME to TETE (Projection) */
   dR                     = atan(tan(dpsi) * cos(eps));
   c1                     = cos(dR);
   s1                     = sin(dR);
   C_TEME_TETE->mat[0][0] = c1;
   C_TEME_TETE->mat[1][0] = -s1;
   C_TEME_TETE->mat[2][0] = 0.0;
   C_TEME_TETE->mat[0][1] = s1;
   C_TEME_TETE->mat[1][1] = c1;
   C_TEME_TETE->mat[2][1] = 0.0;
   C_TEME_TETE->mat[0][2] = 0.0;
   C_TEME_TETE->mat[1][2] = 0.0;
   C_TEME_TETE->mat[2][2] = 1.0;
   return pair;
}
/**********************************************************************/
/* Ref Montenbruck and Gill, "Satellite Orbits: Models, Methods,      */
/* Applications", TL1080.M66                                          */
pair_mat3x3_t HiFiEarthPrecNute(JDType jd_tt_j2000)
{

   mat3x3_t P, N;
   long i;
   double T, zeta, z, theta;
   double cos_zeta, sin_zeta, cos_theta, sin_theta, cos_z, sin_z;
   double dpsi, de, l, lp, F, D, Om, phi, e, ep;
   double cos_e, sin_e, cos_ep, sin_ep, cos_dpsi, sin_dpsi;
   double dR, cos_dR, sin_dR;
   pair_mat3x3_t pair;
   mat3x3_t *const C_TEME_TETE  = &pair.first;
   mat3x3_t *const C_TETE_J2000 = &pair.second;

   static const double pl[106] = {
       0, 0,  -2, 2,  -2, 1,  0,  2, 0,  0,  0,  0,  0, 2,  0,  0,  0, 0,
       0, -2, 0,  2,  0,  1,  2,  0, 0,  0,  -1, 0,  0, 1,  0,  1,  1, -1,
       0, 1,  -1, -1, 1,  0,  2,  1, 2,  0,  -1, -1, 1, -1, 1,  0,  0, 1,
       1, 2,  0,  0,  1,  0,  1,  2, 0,  1,  0,  1,  1, 1,  -1, -2, 3, 0,
       1, -1, 2,  1,  3,  0,  -1, 1, -2, -1, 2,  1,  1, -2, -1, 1,  2, 2,
       1, 0,  3,  1,  0,  -1, 0,  0, 0,  1,  0,  1,  1, 2,  0,  0};
   static const double plp[106] = {
       0,  0, 0,  0, 0, -1, -2, 0, 0, 1, 1,  -1, 0, 0,  0,  2,  1,  2,
       -1, 0, -1, 0, 1, 0,  1,  0, 1, 1, 0,  1,  0, 0,  0,  0,  0,  0,
       0,  0, 0,  0, 0, 0,  0,  0, 0, 0, 0,  0,  0, 0,  1,  1,  -1, 0,
       0,  0, 0,  0, 0, 0,  -1, 0, 1, 0, 0,  1,  0, -1, -1, 0,  0,  -1,
       1,  0, 0,  0, 0, 0,  0,  0, 0, 0, 0,  1,  0, 0,  0,  -1, 0,  0,
       0,  0, 0,  0, 1, -1, 0,  0, 1, 0, -1, 1,  0, 0,  0,  1};
   static const double pD[106] = {
       0, 0, 2, -2, 2,  0, 2, -2, 2,  0, 2, 2,  2, 0, 2,  0, 0, 2, 0, 0,  2, 0,
       2, 0, 0, -2, -2, 0, 0, 2,  2,  0, 2, 2,  0, 2, 0,  0, 0, 2, 2, 2,  0, 2,
       2, 2, 2, 0,  0,  2, 0, 2,  2,  2, 0, 2,  0, 2, 2,  0, 0, 2, 0, -2, 0, 0,
       2, 2, 2, 0,  2,  2, 2, 2,  0,  0, 0, 2,  0, 0, 2,  2, 0, 2, 2, 2,  4, 0,
       2, 2, 0, 4,  2,  2, 2, 0,  -2, 2, 0, -2, 2, 0, -2, 0, 2, 0};
   static const double pF[106] = {
       0, 0,  0,  0,  0,  -1, -2, 0,  -2, 0,  -2, -2, -2, -2, -2, 0,  0,  -2,
       0, 2,  -2, -2, -2, -1, -2, 2,  2,  0,  1,  -2, 0,  0,  0,  0,  -2, 0,
       2, 0,  0,  2,  0,  2,  0,  -2, 0,  0,  0,  2,  -2, 2,  -2, 0,  0,  2,
       0, -2, 2,  2,  -2, -2, 0,  0,  -2, 0,  1,  0,  0,  0,  2,  0,  0,  2,
       0, -2, 0,  0,  0,  1,  0,  -4, 2,  4,  -4, -2, 2,  4,  0,  -2, -2, 2,
       2, -2, -2, -2, 0,  2,  0,  -1, 2,  -2, 0,  -2, 2,  2,  4,  1};
   static const double pOm[106] = {
       1, 2, 1, 0, 2, 0, 1, 1, 2, 0, 2, 2, 1, 0, 0, 0, 1, 2, 1, 1, 1, 1,
       1, 0, 0, 1, 0, 2, 1, 0, 2, 0, 1, 2, 0, 2, 0, 1, 1, 2, 1, 2, 0, 2,
       2, 0, 1, 1, 1, 1, 0, 2, 2, 2, 0, 2, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0,
       0, 2, 2, 1, 2, 2, 2, 1, 1, 2, 0, 2, 2, 0, 2, 2, 0, 2, 1, 2, 2, 0,
       1, 2, 1, 2, 2, 0, 1, 1, 1, 2, 0, 0, 1, 1, 0, 0, 2, 0};
   static const double dp0[106] = {
       -171996, 2062, 46,  11,  -3,  -3,  -2,    1,   -13187, 1426, -517, 217,
       129,     48,   -22, 17,  -15, -16, -12,   -6,  -5,     4,    4,    -4,
       1,       1,    -1,  1,   1,   -1,  -2274, 712, -386,   -301, -158, 123,
       63,      63,   -58, -59, -51, -38, 29,    29,  -31,    26,   21,   16,
       -13,     -10,  -7,  7,   -7,  -8,  6,     6,   -6,     -7,   6,    -5,
       5,       -5,   -4,  4,   -4,  -3,  3,     -3,  -3,     -2,   -3,   -3,
       2,       -2,   2,   -2,  2,   2,   1,     -1,  1,      -2,   -1,   1,
       -1,      -1,   1,   1,   1,   -1,  -1,    1,   1,      -1,   1,    1,
       -1,      -1,   -1,  -1,  -1,  -1,  -1,    1,   -1,     1};
   static const double dp1[40] = {
       -174.2, 0.2,  0,    0,   0,    0, 0, 0, -1.6, -3.4, 1.2,  -0.5, 0.1, 0,
       0,      -0.1, 0,    0.1, 0,    0, 0, 0, 0,    0,    0,    0,    0,   0,
       0,      0,    -0.2, 0.1, -0.4, 0, 0, 0, 0,    0.1,  -0.1, 0};
   static const double de0[106] = {
       92025, -895, -24, 0,  1,   0,   1,  0,   5736, 54,  224, -95, -70, 1,
       0,     0,    9,   7,  6,   3,   3,  -2,  -2,   0,   0,   0,   0,   0,
       0,     0,    977, -7, 200, 129, -1, -53, -2,   -33, 32,  26,  27,  16,
       -1,    -12,  13,  -1, -10, -8,  7,  5,   0,    -3,  3,   3,   0,   -3,
       3,     3,    -3,  3,  0,   3,   0,  0,   0,    0,   0,   1,   1,   1,
       1,     1,    -1,  1,  -1,  1,   0,  -1,  -1,   0,   -1,  1,   0,   -1,
       1,     1,    0,   0,  -1,  0,   0,  0,   0,    0,   0,   0,   0,   0,
       0,     0,    0,   0,  0,   0,   0,  0};
   static const double de1[40] = {8.9,  0.5, 0, 0,    0, 0, 0, 0, -3.1, -0.1,
                                  -0.6, 0.3, 0, 0,    0, 0, 0, 0, 0,    0,
                                  0,    0,   0, 0,    0, 0, 0, 0, 0,    0,
                                  -0.5, 0,   0, -0.1, 0, 0, 0, 0, 0,    0};

   static const double Al  = 134.0 * 3600.0 + 57.0 * 60.0 + 46.733;
   static const double Bl  = 477198.0 * 3600.0 + 52 * 60.0 + 2.633;
   static const double Alp = 357.0 * 3600.0 + 31.0 * 60.0 + 39.804;
   static const double Blp = 35999.0 * 3600.0 + 3.0 * 60.0 + 1.224;
   static const double AF  = 93.0 * 3600.0 + 16.0 * 60.0 + 18.877;
   static const double BF  = 483202.0 * 3600.0 + 1.0 * 60.0 + 3.137;
   static const double AD  = 297.0 * 3600.0 + 51.0 * 60.0 + 1.307;
   static const double BD  = 445267.0 * 3600.0 + 6.0 * 60.0 + 41.328;
   static const double AOm = 125.0 * 3600.0 + 2.0 * 60 + 40.280;
   static const double BOm = -(1934.0 * 3600.0 + 8.0 * 60.0 + 10.539);

   jd_tt_j2000 = JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, jd_tt_j2000);

   T     = JDToDays(jd_tt_j2000) / 36525.0;
   zeta  = (2306.2181 + (0.30188 + 0.017998 * T) * T) * T * A2R;
   theta = (2004.3109 - (0.42665 + 0.041833 * T) * T) * T * A2R;
   z     = zeta + (0.79280 + 0.000205 * T) * T * T * A2R;

   cos_zeta    = cos(zeta);
   sin_zeta    = sin(zeta);
   cos_theta   = cos(theta);
   sin_theta   = sin(theta);
   cos_z       = cos(z);
   sin_z       = sin(z);
   P.mat[0][0] = -sin_z * sin_zeta + cos_zeta * cos_theta * cos_z;
   P.mat[1][0] = cos_z * sin_zeta + sin_z * cos_theta * cos_zeta;
   P.mat[2][0] = sin_theta * cos_zeta;
   P.mat[0][1] = -sin_z * cos_zeta - cos_z * cos_theta * sin_zeta;
   P.mat[1][1] = cos_z * cos_zeta - sin_z * cos_theta * sin_zeta;
   P.mat[2][1] = -sin_theta * sin_zeta;
   P.mat[0][2] = -cos_z * sin_theta;
   P.mat[1][2] = -sin_z * sin_theta;
   P.mat[2][2] = cos_theta;

   dpsi = 0.0;
   de   = 0.0;
   l    = Al + (Bl + (31.310 + 0.064 * T) * T) * T;
   lp   = Alp + (Blp + (-0.577 - 0.012 * T) * T) * T;
   F    = AF + (BF + (-13.257 + 0.011 * T) * T) * T;
   D    = AD + (BD + (-6.891 + 0.019 * T) * T) * T;
   Om   = AOm + (BOm + (7.455 + 0.008 * T) * T) * T;
   for (i = 0; i < 40; i++) {
      phi =
          (pl[i] * l + plp[i] * lp + pD[i] * D + pF[i] * F + pOm[i] * Om) * A2R;
      dpsi += (dp0[i] + dp1[i] * T) * sin(phi);
      de   += (de0[i] + de1[i] * T) * cos(phi);
   }
   for (i = 40; i < 106; i++) {
      phi =
          (pl[i] * l + plp[i] * lp + pF[i] * F + pD[i] * D + pOm[i] * Om) * A2R;
      dpsi += dp0[i] * sin(phi);
      de   += de0[i] * cos(phi);
   }
   dpsi *= 1.0E-4 * A2R;
   de   *= 1.0E-4 * A2R;

   e = (23.43929111 + (-46.8150 + (-0.00059 + 0.001813 * T) * T) * T / 3600.0) *
       D2R;
   ep = e + de;

   cos_e       = cos(e);
   sin_e       = sin(e);
   cos_ep      = cos(ep);
   sin_ep      = sin(ep);
   cos_dpsi    = cos(dpsi);
   sin_dpsi    = sin(dpsi);
   N.mat[0][0] = cos_dpsi;
   N.mat[1][0] = cos_ep * sin_dpsi;
   N.mat[2][0] = sin_ep * sin_dpsi;
   N.mat[0][1] = -cos_e * sin_dpsi;
   N.mat[1][1] = cos_e * cos_ep * cos_dpsi + sin_e * sin_ep;
   N.mat[2][1] = cos_e * sin_ep * cos_dpsi - sin_e * cos_ep;
   N.mat[0][2] = -sin_e * sin_dpsi;
   N.mat[1][2] = sin_e * cos_ep * cos_dpsi - cos_e * sin_ep;
   N.mat[2][2] = sin_e * sin_ep * cos_dpsi + cos_e * cos_ep;

   *C_TETE_J2000 = MxM(N, P);

   /* TEME to TETE (Projection) */
   dR     = atan(sin_dpsi / cos_dpsi * cos_ep);
   cos_dR = cos(dR);
   sin_dR = sin(dR);

   *C_TEME_TETE           = MAT3X3_EYE;
   C_TEME_TETE->mat[0][0] = cos_dR;
   C_TEME_TETE->mat[1][0] = -sin_dR;
   C_TEME_TETE->mat[0][1] = sin_dR;
   C_TEME_TETE->mat[1][1] = cos_dR;
   return pair;
}
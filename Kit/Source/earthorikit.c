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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>
#define _GNU_SOURCE
#include <math.h>

/**********************************************************************/
/* Load Earth Orientation Parameter data                              */

// Loading into memory like in GMAT
// TODO: the file is >23000 rows, do we want to do this?
//    loading all rows of just the data desireed is > 1.1 GB!!!!!!!!
//    each row corresponds to a single day
//    lets try storing ~once/wk (~162 MB now)
//    file is from Jan 1, 1962 to Sep 14, 2026, could cutoff before ~2000?

#define EOP_STEPOVER (7)
struct Ut1MUtcInfo {
   int n_entries;
   double *jday_utc_mjd; // UTC days since MJD
   double *ut1_m_utc;    // UT1 - UTC (sec)
} Ut1MUtcInfo;

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
   extern char DataFilePath[1000];
   char f_path[1064] = {'\0'};
   strcpy(f_path, DataFilePath);
   strcat(f_path, "/eopc04_08.62-now");

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
         Ut1MUtcInfo.ut1_m_utc =
             realloc(Ut1MUtcInfo.ut1_m_utc,
                     (Ut1MUtcInfo.n_entries + 1) * sizeof(double));
         Ut1MUtcInfo.jday_utc_mjd =
             realloc(Ut1MUtcInfo.jday_utc_mjd,
                     (Ut1MUtcInfo.n_entries + 1) * sizeof(double));

         PM_Info.jday_utc_mjd = realloc(
             PM_Info.jday_utc_mjd, (PM_Info.n_entries + 1) * sizeof(double));
         PM_Info.x =
             realloc(PM_Info.x, (PM_Info.n_entries + 1) * sizeof(double));
         PM_Info.y =
             realloc(PM_Info.y, (PM_Info.n_entries + 1) * sizeof(double));
         PM_Info.lod =
             realloc(PM_Info.lod, (PM_Info.n_entries + 1) * sizeof(double));

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
double GetUt1UtcOffset(const double jday_utc_mjd)
{
   call_once(&eop_file_flag, init_eop_file);
   static int ind = 0;

   const int n_entries              = Ut1MUtcInfo.n_entries;
   const double *const tbl_jday_utc = Ut1MUtcInfo.jday_utc_mjd;

   // if 'jday_utc_mjd' is outside the table, return the nearest extent
   if (tbl_jday_utc[0] >= jday_utc_mjd)
      return Ut1MUtcInfo.ut1_m_utc[0];
   else if (jday_utc_mjd >= tbl_jday_utc[n_entries - 1])
      return Ut1MUtcInfo.ut1_m_utc[n_entries - 1];

   // From the above, 'jday_utc_mjd' will be in the open interval
   //    (tbl_jday[0], tbl_jday[n_entries - 1])
   // thus, 'ind' will be on the closed interval
   //    [0, n_entries - 2]
   ind = FindIndex(jday_utc_mjd, tbl_jday_utc, Ut1MUtcInfo.n_entries, ind);

   double yp[2];

   yp[0] =
       CentralDifference(ind, n_entries, tbl_jday_utc, Ut1MUtcInfo.ut1_m_utc);
   yp[1] = CentralDifference(ind + 1, n_entries, tbl_jday_utc,
                             Ut1MUtcInfo.ut1_m_utc);

   const double *const dut1 = &Ut1MUtcInfo.ut1_m_utc[ind];
   const double *const X    = &tbl_jday_utc[ind];

   return ClampedCubicSpline(jday_utc_mjd, X[0], X[1], dut1[0], dut1[1], yp[0],
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
}
/**********************************************************************/
/* IAU Nutation data                                                  */

// using IAU 1950, IAU 1980, or IAU 1996
// default to IAU 1980
#define N_NUT_MAX (1320)

#define NUT_N_1950            (69)
#define NUT_ORDER_1950        (3)
#define NUT_N_PLANETS_1950    (5)
#define NUT_MULT_1950         (1.0e-04) // arcseconds
#define NUT_FIRST_PHRASE_1950 ("1950 IAU")

#define NUT_N_1980            (106)
#define NUT_ORDER_1980        (3)
#define NUT_N_PLANETS_1980    (5)
#define NUT_MULT_1980         (1.0e-04) // arcseconds
#define NUT_FIRST_PHRASE_1980 ("1980 IAU")

#define NUT_N_1996            (263)
#define NUT_ORDER_1996        (4)
#define NUT_N_PLANETS_1996    (5)
#define NUT_MULT_1996         (1.0e-07) // arcseconds
#define NUT_FIRST_PHRASE_1996 ("1996 IAU")

#define NUT_N_2000            (106)
#define NUT_ORDER_2000        (4)
#define NUT_N_PLANETS_2000    (5)
#define NUT_MULT_2000         (1.0e-04) // arcseconds
#define NUT_FIRST_PHRASE_2000 ("2000 IAU")

// IAU2000_R06 adapted from United States Naval Observatory Circular No. 179
// 'The IAU Resolutions on Astronomical Reference Systems, Time Scales, and
// Earth Rotation Models: Explanation and Implementation' by George H. Kaplan,
// 2005, Oct 20
#define NUT_N_2000R06            (1320)
#define NUT_ORDER_2000R06        (4)
#define NUT_N_PLANETS_2000R06    (14)
#define NUT_MULT_2000R06         (1.0e-06) // arcseconds
#define NUT_FIRST_PHRASE_2000R06 ("2000R06 IAU")

struct NutEntry {
   char a[NUT_N_PLANETS_2000R06];
   double A;
   double B;
   double C;
   double D;
   double E;
   double F;
   double G;
   double H;
   char index;
};

static struct NutInfo {
   long n_entries;
   double mult;
   char first_phrase[10];
   int order;
   int n_planets;
   struct NutEntry *entries;
} Nut_Info;

// Default to using ITRF 1980 data
// static const enum NutEnum NutSelection = NUT_ITRF_1950;
static const enum NutEnum NutSelection = NUT_ITRF_1980;
// static const enum NutEnum NutSelection = NUT_ITRF_1996;
// static const enum NutEnum NutSelection = NUT_ITRF_2000;
// static const enum NutEnum NutSelection = NUT_IAU_2000R06;

static __once_flag iau_file_flag = __ONCE_FLAG_INIT;
static void init_iau_file()
{
   extern char DataFilePath[1000];
   char f_path[1064] = {'\0'};
   strcpy(f_path, DataFilePath);
   strcat(f_path, "/NUTATION.DAT");

   switch (NutSelection) {
      case NUT_ITRF_1950:
         Nut_Info.n_entries = NUT_N_1950;
         Nut_Info.mult      = NUT_MULT_1950;
         Nut_Info.order     = NUT_ORDER_1950;
         Nut_Info.n_planets = NUT_N_PLANETS_1950;
         strcpy(Nut_Info.first_phrase, NUT_FIRST_PHRASE_1950);
         break;
      case NUT_ITRF_1980:
         Nut_Info.n_entries = NUT_N_1980;
         Nut_Info.mult      = NUT_MULT_1980;
         Nut_Info.order     = NUT_ORDER_1980;
         Nut_Info.n_planets = NUT_N_PLANETS_1980;
         strcpy(Nut_Info.first_phrase, NUT_FIRST_PHRASE_1980);
         break;
      case NUT_ITRF_1996:
         Nut_Info.n_entries = NUT_N_1996;
         Nut_Info.mult      = NUT_MULT_1996;
         Nut_Info.order     = NUT_ORDER_1996;
         Nut_Info.n_planets = NUT_N_PLANETS_1996;
         strcpy(Nut_Info.first_phrase, NUT_FIRST_PHRASE_1996);
         break;
      case NUT_ITRF_2000:
         Nut_Info.n_entries = NUT_N_2000;
         Nut_Info.mult      = NUT_MULT_2000;
         Nut_Info.order     = NUT_ORDER_2000;
         Nut_Info.n_planets = NUT_N_PLANETS_2000;
         strcpy(Nut_Info.first_phrase, NUT_FIRST_PHRASE_2000);
         break;
      case NUT_IAU_2000R06:
         Nut_Info.n_entries = NUT_N_2000R06;
         Nut_Info.mult      = NUT_MULT_2000R06;
         Nut_Info.order     = NUT_ORDER_2000R06;
         Nut_Info.n_planets = NUT_N_PLANETS_2000R06;
         strcpy(Nut_Info.first_phrase, NUT_FIRST_PHRASE_2000R06);
         break;
      default:
         break;
   }
   Nut_Info.entries = calloc(Nut_Info.n_entries, sizeof(struct NutEntry));

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
   // start at end since list is sorted with descending A coefficients
   // forward iteration later will sum from smallest coeffs to largest
   struct NutEntry *entry = &Nut_Info.entries[Nut_Info.n_entries - 1];
   for (; entry >= Nut_Info.entries; entry--) {
      if (fgets(line, 512, file) == NULL) {
         fprintf(stderr,
                 "ITRF nutation file '%s' ended before reading all expected "
                 "values. Exiting...\n",
                 f_path);
         exit(EXIT_FAILURE);
      }
      switch (NutSelection) {
         case NUT_ITRF_1950: // no E, F, G, or H terms
         case NUT_ITRF_1980: // no E, F, G, or H terms
         default:
            sscanf(line, "%hhi %hhi %hhi %hhi %hhi %lf %lf %lf %lf %hhi",
                   &entry->a[0], &entry->a[1], &entry->a[2], &entry->a[3],
                   &entry->a[4], &entry->A, &entry->B, &entry->C, &entry->D,
                   &entry->index);
            entry->E = 0;
            entry->F = 0;
            entry->G = 0;
            entry->H = 0;
            break;
         case NUT_ITRF_1996: // no G or H terms
         case NUT_ITRF_2000: // no G or H terms
            sscanf(line,
                   "%hhi %hhi %hhi %hhi %hhi %lf %lf %lf %lf %lf %lf %hhi",
                   &entry->a[0], &entry->a[1], &entry->a[2], &entry->a[3],
                   &entry->a[4], &entry->A, &entry->B, &entry->C, &entry->D,
                   &entry->E, &entry->F, &entry->index);
            entry->G = 0;
            entry->H = 0;
            break;
         case NUT_IAU_2000R06:
            sscanf(line,
                   "%hhi %hhi %hhi %hhi %hhi %hhi %hhi %hhi %hhi %hhi %hhi "
                   "%hhi %hhi %hhi %lf %lf %lf %lf %lf %lf %lf %lf %hhi",
                   &entry->a[0], &entry->a[1], &entry->a[2], &entry->a[3],
                   &entry->a[4], &entry->a[5], &entry->a[6], &entry->a[7],
                   &entry->a[8], &entry->a[9], &entry->a[10], &entry->a[11],
                   &entry->a[12], &entry->a[13], &entry->A, &entry->B,
                   &entry->C, &entry->D, &entry->E, &entry->F, &entry->G,
                   &entry->H, &entry->index);
            break;
      }
   }
   fclose(file);
}
/**********************************************************************/
__attribute__((const)) static inline mat3x3_t EarthJ2000ICRSBiasMatrix()
{
   const double da0 = (-14.6 * 1e-03 / D2A) * D2R;
   const double e0  = (-16.6170 * 1e-03 / D2A) * D2R;
   const double n0  = (-6.8192 * 1e-03 / D2A) * D2R;

   const double da02 = da0 * da0;
   const double e02  = e0 * e0;
   const double n02  = n0 * n0;

   // approximation of MxM(MxM(ROT1(-n0), ROT2(e0)), ROT3(da0))
   return (mat3x3_t){
       .mat = {{1.0 - 0.5 * (da02 + e02), da0, -e0},
               {-da0 - n0 * e0, 1.0 - 0.5 * (da02 + n02), -n0},
               {e0 - n0 * da0, n0 + e0 * da0, 1.0 - 0.5 * (n02 + e02)}}};
}
/**********************************************************************/
__attribute__((const)) static mat3x3_t EarthPrecessionMatrix(const double TTDB);
static mat3x3_t EarthPrecessionMatrix(const double TTDB)
{
   mat3x3_t PREC = MAT3X3_EYE;
   switch (NutSelection) {
      case NUT_ITRF_1950:
      case NUT_ITRF_1980:
      case NUT_ITRF_1996: {
         double zeta  = (2306.2181 + (0.30188 + 0.017998 * TTDB) * TTDB) * TTDB;
         double Theta = (2004.3109 - (0.42665 + 0.041833 * TTDB) * TTDB) * TTDB;
         double z     = zeta + (0.7928 + 0.000205 * TTDB) * TTDB * TTDB;

         zeta  = WrapArcSec(zeta);
         Theta = WrapArcSec(Theta);
         z     = WrapArcSec(z);

         const double S1 = sin(-zeta * A2R), C1 = cos(-zeta * A2R);
         const double S2 = sin(Theta * A2R), C2 = cos(Theta * A2R);
         const double S3 = sin(-z * A2R), C3 = cos(-z * A2R);

         //  PREC = A2C(323, -z * A2R, Theta * A2R, -zeta * A2R);
         PREC.x = (vec3_t){.x = C1 * C2 * C3 - S1 * S3,
                           .y = C1 * S3 + C2 * C3 * S1,
                           .z = -C3 * S2};
         PREC.y = (vec3_t){.x = -C1 * C2 * S3 - C3 * S1,
                           .y = C1 * C3 - C2 * S1 * S3,
                           .z = S2 * S3};
         PREC.z = (vec3_t){.x = C1 * S2, .y = S1 * S2, .z = C2};
      } break;
      case NUT_ITRF_2000:
      case NUT_IAU_2000R06: {
         const double eps0 = 84381.406 * A2R;
         const double psia =
             ((((-0.0000000951 * TTDB + 0.000132851) * TTDB - 0.00114045) *
                   TTDB -
               1.0790069) *
                  TTDB +
              5038.481507) *
             TTDB * A2R;
         const double omga =
             ((((0.0000003337 * TTDB - 0.000000467) * TTDB - 0.00772503) *
                   TTDB +
               0.0512623) *
                  TTDB -
              0.025754) *
                 TTDB * A2R +
             eps0;
         const double chia =
             ((((-0.0000000560 * TTDB + 0.000170663) * TTDB - 0.00121197) *
                   TTDB -
               2.3814292) *
                  TTDB +
              10.556403) *
             TTDB * A2R;
         const double S1 = sin(eps0), C1 = cos(eps0);
         const double S2 = sin(-psia), C2 = cos(-psia);
         const double S3 = sin(-omga), C3 = cos(-omga);
         const double S4 = sin(chia), C4 = cos(chia);

         PREC.x = (vec3_t){.x = C2 * C4 - C3 * S2 * S4,
                           .y = C1 * (C2 * C3 * S4 + C4 * S2) - S1 * S3 * S4,
                           .z = C1 * S3 * S4 + S1 * (C2 * C3 * S4 + C4 * S2)};
         PREC.y = (vec3_t){.x = -C2 * S4 - C3 * C4 * S2,
                           .y = C1 * (C2 * C3 * C4 - S2 * S4) - C4 * S1 * S3,
                           .z = C1 * C4 * S3 + S1 * (C2 * C3 * C4 - S2 * S4)};
         PREC.z = (vec3_t){.x = S2 * S3,
                           .y = -C1 * C2 * S3 - C3 * S1,
                           .z = C1 * C3 - C2 * S1 * S3};
      } break;
      default:
         break;
   }

   return PREC;
}
/**********************************************************************/
/* The coefficients in each row are in the order:                     */
/*    1 - meanAnomLuna                                                */
/*    2 - meanAnomSol                                                 */
/*    3 - argLatLuna                                                  */
/*    4 - meanElongSol                                                */
/*    5 - longAscNodeLuna                                             */
/*    6 - meanEclipLongMercury                                        */
/*    7 - meanEclipLongVenus                                          */
/*    8 - meanEclipLongEarth                                          */
/*    9 - meanEclipLongMars                                           */
/*   10 - meanEclipLongJupiter                                        */
/*   11 - meanEclipLongSaturn                                         */
/*   12 - meanEclipLongUranus                                         */
/*   13 - meanEclipLongNeptune                                        */
/*   14 - generalPrecLong                                             */
/* NOTE: elements 6-14 are used (non-zero) only for NUT_IAU_2000R06    */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
__attribute__((const)) static inline const double *const
NutSolarLunarPosition_R_Rot(const enum NutEnum selection)
{
   static double zero_rs[NUT_N_PLANETS_2000R06] = {0, 0, 0, 0, 0, 0, 0,
                                                   0, 0, 0, 0, 0, 0, 0};
   switch (selection) {
      case NUT_ITRF_1950:
      case NUT_ITRF_1980:
      case NUT_ITRF_1996:
      case NUT_ITRF_2000:
      case NUT_IAU_2000R06: {
         static double r_rot[NUT_N_PLANETS_2000R06] = {
             1325, 99, 1342, 1236, -5, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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
   static double zero_coeffs[NUT_N_PLANETS_2000R06] = {0, 0, 0, 0, 0, 0, 0,
                                                       0, 0, 0, 0, 0, 0, 0};
   switch (selection) {
      case NUT_ITRF_1950:
      case NUT_ITRF_1980: {
         static double coeffs_asec[NUT_ORDER_1980 + 1][NUT_N_PLANETS_2000R06] =
             {{485866.733, 1287099.804, 335778.877, 1072261.307, 450160.280, 0,
               0, 0, 0, 0, 0, 0, 0, 0},
              {715922.633, 1292581.224, 295263.137, 1105601.328, -482890.539, 0,
               0, 0, 0, 0, 0, 0, 0, 0},
              {31.310, -0.577, -13.257, -6.891, 7.455, 0, 0, 0, 0, 0, 0, 0, 0,
               0},
              {0.064, -0.012, 0.011, 0.019, 0.008, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
         if (order_i >= NUT_ORDER_1980 + 1)
            return zero_coeffs;
         return coeffs_asec[order_i];
      } break;
      case NUT_ITRF_1996:
      case NUT_ITRF_2000: {
         static double coeffs_asec[NUT_ORDER_1996 + 1][NUT_N_PLANETS_2000R06] =
             {{485868.249036, 1287104.793048, 335779.526232, 1072260.703692,
               450160.398036, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {715923.2178, 1292581.0481, 295262.8478, 1105601.2090,
               -482890.5431, 0, 0, 0, 0, 0, 0, 0, 0, 0},
              {31.8792, -0.5532, -12.7512, -6.3706, 7.4722, 0, 0, 0, 0, 0, 0, 0,
               0, 0},
              {0.051635, 0.000136, -0.001037, 0.006593, 0.007702, 0, 0, 0, 0, 0,
               0, 0, 0, 0},
              {-0.00024470, -0.00001149, 0.00000417, -0.00003169, -0.00005939,
               0, 0, 0, 0, 0, 0, 0, 0, 0}};
         if (order_i >= NUT_ORDER_1996 + 1)
            return zero_coeffs;
         return coeffs_asec[order_i];
      } break;
      case NUT_IAU_2000R06: {

         // From United States Naval Observatory Circular No. 179 'The IAU
         // Resolutions on Astronomical Reference Systems, Time Scales, and
         // Earth Rotation Models: Explanation and Implementation'
         // by George H. Kaplan, 2005, Oct 20
         static double
             coeffs_asec[NUT_ORDER_2000R06 + 1][NUT_N_PLANETS_2000R06] = {
                 {485868.249036, 1287104.79305, 335779.526232, 1072260.70369,
                  450160.398036, 908103.259872, 655127.283060, 361679.244588,
                  1279558.798488, 123665.467464, 180278.799480, 1130598.018396,
                  1095655.195728, 0},
                 {715923.2178, 1292581.0481, 295262.8478, 1105601.209,
                  -482890.5431, 538101628.688982, 210664136.433548,
                  129597742.283429, 68905077.493988, 10925660.377991,
                  4399609.855732, 1542481.193933, 786550.320744, 5028.82},
                 {31.8792, -0.5532, -12.7512, -6.3706, 7.4722, 0, 0, 0, 0, 0, 0,
                  0, 0, 1.112022},
                 {51635e-02, 1.36e-04, -1.037e-03, 6.593e-03, 7.702e-03, 0, 0,
                  0, 0, 0, 0, 0, 0, 0},
                 {-2.4470e-04, -1.149e-05, 4.17e-06, -3.169e-05, -5.939e-05, 0,
                  0, 0, 0, 0, 0, 0, 0, 0}};
         if (order_i >= NUT_ORDER_2000R06 + 1)
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

   double nut_angles[NUT_N_PLANETS_2000R06] = {0};

   /* // Descriptions of each index
   // .. Mean anomaly of the Moon's orbit (rad)
   double *const meanAnomLuna = &nut_angles[0];
   // .. Mean anomaly of the Sun's orbit (rad)
   double *const meanAnomSol = &nut_angles[1];
   // .. Mean argument of latitude of the Moon's orbit (rad)
   double *const argLatLuna = &nut_angles[2];
   // .. Difference between the mean longitude of the Sun and Moon (rad)
   double *const meanElongSol = &nut_angles[3];

   // .. Mean Heliocentric Ecliptic Longitude of Mercury (rad)
   double *const meanEclipLongMercury = &nut_angles[5];
   // .. Mean Heliocentric Ecliptic Longitude of Venus (rad)
   double *const meanEclipLongVenus = &nut_angles[6];
   // .. Mean Heliocentric Ecliptic Longitude of Earth (rad)
   double *const meanEclipLongEarth = &nut_angles[7];
   // .. Mean Heliocentric Ecliptic Longitude of Mars (rad)
   double *const meanEclipLongMars = &nut_angles[8];
   // .. Mean Heliocentric Ecliptic Longitude of Jupiter (rad)
   double *const meanEclipLongJupiter = &nut_angles[9];
   // .. Mean Heliocentric Ecliptic Longitude of Saturn (rad)
   double *const meanEclipLongSaturn = &nut_angles[10];
   // .. Mean Heliocentric Ecliptic Longitude of Uranus (rad)
   double *const meanEclipLongUranus = &nut_angles[11];
   // .. Mean Heliocentric Ecliptic Longitude of Neptune (rad)
   double *const meanEclipLongNeptune = &nut_angles[12];

   // .. Approximation to the General Precession in Longitude (rad)
   double *const generalPrecLong = &nut_angles[13];
   */

   // .. Mean longitude of the ascending node of the Moon's orbit (rad)
   double *const longAscNodeLuna = &nut_angles[4];

   // .. Accumulate the various Sun & Moon position angles
   double x = 1;
   for (int order_i = 0; order_i < Nut_Info.order + 1; order_i++) {
      const double *const coeffs_asec =
          NutSolarLunarPosition_ArcSec(NutSelection, order_i);

      for (int j = 0; j < Nut_Info.n_planets; j++)
         nut_angles[j] += WrapArcSec(x * coeffs_asec[j]);
      x *= TTDB;
   }

   // .. Accumulate the complete rotation terms, then map to 360 deg
   const double *const r_rot = NutSolarLunarPosition_R_Rot(NutSelection);
   double dummy;
   for (int j = 0; j < Nut_Info.n_planets; j++) {
      nut_angles[j] += modf(r_rot[j] * TTDB, &dummy) * (360 * D2A);
      nut_angles[j]  = WrapArcSec(nut_angles[j]);
   }

   *longAscNodeLuna_ret = *longAscNodeLuna * A2R;

   /* Mean Obliquity of the ecliptic at J2000 epoch (arcsec)                */
   double Epsbar = 0;

   switch (NutSelection) {
      case NUT_ITRF_1950:
      case NUT_ITRF_1980:
      case NUT_ITRF_1996: {
         Epsbar = (84381.448 +
                   (-46.8150 + (-0.00059 + 0.001813 * TTDB) * TTDB) * TTDB) *
                  A2R;
      } break;
      case NUT_ITRF_2000:
      case NUT_IAU_2000R06: {
         const double eps0 = 84381.406;
         Epsbar =
             (((((4.34e-08 * TTDB + 5.76e-7) * TTDB + 2.00340e-03) * TTDB) -
               1.831e-4) *
                  TTDB -
              46.836769) *
                 TTDB +
             eps0;
         Epsbar *= A2R;
      } break;
      default:
         break;
   }

   *dPsi          = 0;
   double dPsidot = 0;

   double dEps    = 0;
   double dEpsdot = 0;

   struct NutEntry *entry = Nut_Info.entries;
   for (; entry <= &Nut_Info.entries[Nut_Info.n_entries - 1]; entry++) {
      double apNut = 0;
      for (int i = 0; i < Nut_Info.n_planets; i++)
         apNut += entry->a[i] * nut_angles[i];
      apNut            *= A2R;
      const double CAp  = cos(apNut);
      const double SAp  = sin(apNut);

      switch (NutSelection) {
         case NUT_ITRF_1950: // no E, F, G, or H terms
         case NUT_ITRF_1980: // no E, F, G, or H terms
         default:
            *dPsi   += entry->A * SAp;
            dPsidot += entry->B * SAp;
            dEps    += entry->C * CAp;
            dEpsdot += entry->D * CAp;
            break;
         case NUT_ITRF_1996: // no G or H terms
         case NUT_ITRF_2000: // no G or H terms
            *dPsi   += entry->A * SAp + entry->E * CAp;
            dPsidot += entry->B * SAp;
            dEps    += entry->C * CAp + entry->F * SAp;
            dEpsdot += entry->D * CAp;
            break;
         case NUT_IAU_2000R06:
            *dPsi   += entry->A * SAp + entry->E * CAp;
            dPsidot += entry->B * SAp + entry->G * CAp;
            dEps    += entry->C * CAp + entry->F * SAp;
            dEpsdot += entry->D * CAp + entry->H * SAp;
            break;
      }
   }
   *dPsi += dPsidot * TTDB;
   dEps  += dEpsdot * TTDB;
   *dPsi *= A2R * Nut_Info.mult;
   dEps  *= A2R * Nut_Info.mult;

   const double Eps = Epsbar + dEps;
   *cosEps          = cos(Epsbar);

   const double S1 = sin(Epsbar), C1 = cos(Epsbar);
   const double S2 = sin(-*dPsi), C2 = cos(-*dPsi);
   const double S3 = sin(-Eps), C3 = cos(-Eps);

   mat3x3_t NUT = MAT3X3_EYE;

   NUT.x = (vec3_t){.x = C2, .y = C1 * S2, .z = S1 * S2};
   NUT.y = (vec3_t){
       .x = -C3 * S2, .y = C1 * C2 * C3 - S1 * S3, .z = C1 * S3 + C2 * C3 * S1};
   NUT.z = (vec3_t){
       .x = S2 * S3, .y = -C1 * C2 * S3 - C3 * S1, .z = C1 * C3 - C2 * S1 * S3};
   return NUT;
}
/**********************************************************************/
static mat3x3_t EarthNutPrecMatrix(const double TTDB, double *const dPsi,
                                   double *const longAscNodeLuna,
                                   double *const cosEps)
{
   const mat3x3_t PREC = EarthPrecessionMatrix(TTDB);
   const mat3x3_t NUT =
       EarthNutationMatrix(TTDB, dPsi, longAscNodeLuna, cosEps);
   const mat3x3_t B = EarthJ2000ICRSBiasMatrix();

   return MxM(MxM(NUT, PREC), B);
}
/**********************************************************************/
double EarthERA(JDType jd)
{
   const double sidereal_add = 2.73781191135448e-03;
   JDType jd_ut1_j2000       = JDChangeSystemEpoch(UT1_TIME, J2000_EPOCH, jd);
   const double jd_frac_day =
       jdsecond2double(jd_ut1_j2000.seconds) / SEC_PER_DAY;
   return (0.7790572732640 + jd_ut1_j2000.whole_days +
           sidereal_add * jd_ut1_j2000.whole_days + jd_frac_day +
           sidereal_add * jd_frac_day) *
          TWOPI;
}
/**********************************************************************/
/* Greenwich Mean Sidereal Time (sec)                                 */
double HiFiJD2GMST(const JDType jd)
{
   const JDType jd_ut1_j2000 = JDChangeSystemEpoch(UT1_TIME, J2000_EPOCH, jd);

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
   const double sec_GMST = (24110.54841 + 1.002737909350795 * sec_ut1day +
                            (WrapDaySec(8640184.812866 * T0UT1) +
                             (9.3104e-02 - 6.2e-06 * TUT1) * TUT1 * TUT1));
   const double rad_GMST = WrapTo2Pi((sec_GMST / secperdeg) * D2R);
   return rad_GMST;
}
/**********************************************************************/
// Astromical Almanac 2017, pg B10
// if dPsi and cosEps are good, then this is accurate to < 2e-6 seconds
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
          (2.64 * sin(longAscNodeLuna) + 6.0e-02 * sin(2.0 * longAscNodeLuna)) *
          1.0e-3 * A2R;

   const double GMST = HiFiJD2GMST(jd);
   return GMST + eq_equinox;
}
/**********************************************************************/
__attribute__((const)) mat3x3_t EarthPolarMotion(const JDType jd);
mat3x3_t EarthPolarMotion(const JDType jd)
{
   JDType jd_utc_mjd = JDChangeSystemEpoch(UTC_TIME, MJD_EPOCH, jd);
   double x, y, lod = 0;
   GetPolarMotionData(JDToDays(jd_utc_mjd), &x, &y, &lod);

   const double S2 = sin(y * A2R), C2 = cos(y * A2R);
   const double S1 = sin(x * A2R), C1 = cos(x * A2R);

   mat3x3_t PM = MAT3X3_EYE;
   PM.x        = (vec3_t){.x = C1, .y = 0, .z = -S1};
   PM.y        = (vec3_t){.x = S1 * S2, .y = C2, .z = C1 * S2};
   PM.z        = (vec3_t){.x = S1 * C2, .y = -S2, .z = C1 * C2};

   return MT(PM);
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

   const mat3x3_t NPB =
       EarthNutPrecMatrix(TTDB, &dPsi, &longAscNodeLuna, &cosEps);
   *GAST = JD2GAST(jd_utc_j2000, dPsi, longAscNodeLuna, cosEps);

   mat3x3_t ST       = ROT3(*GAST);
   const mat3x3_t PM = EarthPolarMotion(jd_utc_j2000);

   *CWN = MxM(PM, MxM(ST, NPB));
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
   GMST = modf(GMST, &GMST0);
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
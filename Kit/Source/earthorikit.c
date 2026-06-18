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

// using IAU 1980 or IAU 1996
// default to IAU 1980
#define N_NUT_MAX (263)

#define N_NUT_1996            (263)
#define MULT_NUT_1980         (1.0e-04) // arcseconds
#define FIRST_PHRASE_NUT_1980 ("1980 IAU")

#define N_NUT_1980            (180)
#define MULT_NUT_1996         (1.0e-04) // arcseconds
#define FIRST_PHRASE_NUT_1996 ("1996 IAU")

struct NutEntry {
   int a[5];
   double A;
   double B;
   double C;
   double D;
   double E;
   double F;
};

static struct NutInfo {
   long n_entries;
   double mult;
   char first_phrase[10];
   struct NutEntry entries[N_NUT_MAX];
} Nut_Info;

// Default to using ITRF 1980 data
static const enum NutEnum NutSelection = NUT_ITRF_1980;

static __once_flag iau_flag = __ONCE_FLAG_INIT;
static void init_iau_file()
{
   extern char ModelPath[1000];
   char f_path[1064] = {'\0'};
   strcpy(f_path, ModelPath);
   strcat(f_path, "/data_files/NUTATION.DAT");

   switch (NutSelection) {
      case NUT_ITRF_1980:
         Nut_Info.n_entries = N_NUT_1980;
         Nut_Info.mult      = MULT_NUT_1980;
         strcpy(Nut_Info.first_phrase, FIRST_PHRASE_NUT_1980);
         break;
      case NUT_ITRF_1996:
         Nut_Info.n_entries = N_NUT_1996;
         Nut_Info.mult      = MULT_NUT_1996;
         strcpy(Nut_Info.first_phrase, FIRST_PHRASE_NUT_1996);
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

   // loop over file to find start of 1996 IAU section
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
         case NUT_ITRF_1980: // no E or F terms
            sscanf(line, "%i %i %i %i %i %lf %lf %lf %lf", &entry->a[0],
                   &entry->a[1], &entry->a[2], &entry->a[3], &entry->a[4],
                   &entry->A, &entry->B, &entry->C, &entry->D);
            entry->E = 0;
            entry->F = 0;
            break;
         default:
            sscanf(line, "%i %i %i %i %i %lf %lf %lf %lf %lf %lf", &entry->a[0],
                   &entry->a[1], &entry->a[2], &entry->a[3], &entry->a[4],
                   &entry->A, &entry->B, &entry->C, &entry->D, &entry->E,
                   &entry->F);
            break;
      }
      entry->a[0] *= Nut_Info.mult;
      entry->a[1] *= Nut_Info.mult;
      entry->a[2] *= Nut_Info.mult;
      entry->a[3] *= Nut_Info.mult;
      entry->a[4] *= Nut_Info.mult;
      entry->A    *= Nut_Info.mult;
      entry->B    *= Nut_Info.mult;
      entry->C    *= Nut_Info.mult;
      entry->D    *= Nut_Info.mult;
      entry->E    *= Nut_Info.mult;
      entry->F    *= Nut_Info.mult;
   }
   fclose(file);
}
/**********************************************************************/
/* Mean longitude of the ascending node of the Moon's orbit (rad)    */
__attribute__((const)) static inline double
_earth_nut_Omega(const double TTDB, const double TTDB2, const double TTDB3,
                 const double TTDB4);
static inline double _earth_nut_Omega(const double TTDB, const double TTDB2,
                                      const double TTDB3, const double TTDB4)
{
   switch (NutSelection) {
      case NUT_ITRF_1980:
         return (125.04452222 * D2R) +
                (TTDB * -6962890.5390 + TTDB2 * 7.455 + TTDB3 * 0.008) * A2R;
      case NUT_ITRF_1996:
         return (125.04455501 * D2R) +
                (TTDB * -6962890.2665 + TTDB2 * 7.4722 + TTDB3 * 0.007702 +
                 TTDB4 * -0.00005939) *
                    A2R;
   }
}
/**********************************************************************/
/* Mean anomaly of the Moon's orbit (rad)                             */
__attribute__((const)) static inline double _earth_nut_l(const double TTDB,
                                                         const double TTDB2,
                                                         const double TTDB3,
                                                         const double TTDB4);
static inline double _earth_nut_l(const double TTDB, const double TTDB2,
                                  const double TTDB3, const double TTDB4)
{
   switch (NutSelection) {
      case NUT_ITRF_1980:
         return (134.96298139 * D2R) +
                (TTDB * 1717915922.6330 + TTDB2 * 31.310 + TTDB3 * 0.064) * A2R;
      case NUT_ITRF_1996:
         return (134.96340251 * D2R) +
                (TTDB * 1717915923.2178 + TTDB2 * 31.8792 + TTDB3 * 0.051635 +
                 TTDB4 * -0.00024470) *
                    A2R;
   }
}
/**********************************************************************/
/* Mean anomaly of the Sun's orbit (rad)                              */
__attribute__((const)) static inline double _earth_nut_lp(const double TTDB,
                                                          const double TTDB2,
                                                          const double TTDB3,
                                                          const double TTDB4);
static inline double _earth_nut_lp(const double TTDB, const double TTDB2,
                                   const double TTDB3, const double TTDB4)
{
   switch (NutSelection) {
      case NUT_ITRF_1980:
         return (357.52772333 * D2R) +
                (TTDB * 129596581.2240 + TTDB2 * -0.577 + TTDB3 * -0.012) * A2R;
      case NUT_ITRF_1996:
         return (357.52910918 * D2R) +
                (TTDB * 129596581.0481 + TTDB2 * -0.5532 + TTDB3 * -0.000136 +
                 TTDB4 * -0.00001149) *
                    A2R;
   }
}
/**********************************************************************/
/* Mean argument of latitude of the Moon's orbit (rad)                */
__attribute__((const)) static inline double _earth_nut_F(const double TTDB,
                                                         const double TTDB2,
                                                         const double TTDB3,
                                                         const double TTDB4);
static inline double _earth_nut_F(const double TTDB, const double TTDB2,
                                  const double TTDB3, const double TTDB4)
{
   switch (NutSelection) {
      case NUT_ITRF_1980:
         return (93.27191028 * D2R) +
                (TTDB * 1739527263.1370 + TTDB2 * -13.257 + TTDB3 * 0.011) *
                    A2R;
      case NUT_ITRF_1996:
         return (93.27209062 * D2R) +
                (TTDB * 1739527262.8478 + TTDB2 * -12.7512 + TTDB3 * 0.001037 +
                 TTDB4 * 0.00000417) *
                    A2R;
   }
}
/**********************************************************************/
/* Difference between the mean longitude of the Sun and Moon (rad)    */
__attribute__((const)) static inline double _earth_nut_D(const double TTDB,
                                                         const double TTDB2,
                                                         const double TTDB3,
                                                         const double TTDB4);
static inline double _earth_nut_D(const double TTDB, const double TTDB2,
                                  const double TTDB3, const double TTDB4)
{
   switch (NutSelection) {
      case NUT_ITRF_1980:
         return (297.85036306 * D2R) +
                (TTDB * 1602961601.3280 + TTDB2 * -6.891 + TTDB3 * 0.019) * A2R;
      case NUT_ITRF_1996:
         return (297.85019547 * D2R) +
                (TTDB * 1602961601.2090 + TTDB2 * -6.3706 + TTDB3 * 0.006593 +
                 TTDB4 * -0.00003169) *
                    A2R;
   }
}
/**********************************************************************/
/* Mean Obliquity of the ecliptic at J2000 epoch (deg)                */
__attribute__((const)) static inline double
_earth_nut_eps(const double TTDB, const double TTDB2, const double TTDB3);
static inline double _earth_nut_eps(const double TTDB, const double TTDB2,
                                    const double TTDB3)
{
   return (84381.448 + TTDB * -46.8150 + TTDB2 * -0.00059 + TTDB3 * 0.001813) *
          A2R;
}
/**********************************************************************/
/* Greenwich Mean Sidereal Time (rad)                                 */
double GMAT_JD2GMST(const JDType jd)
{
   const JDType jd_utc_j2000 = JDChangeSystemEpoch(UTC_TIME, J2000_EPOCH, jd);
   const double TUT1         = JDToDays(jd_utc_j2000) / JDDAY_PER_CENTURY;
   const double TUT12        = TUT1 * TUT1;
   const double TUT13        = TUT12 * TUT1;
   return (1.00965822615e6 + 4.746600277219299e10 * TUT1 + 1.3956560 * TUT12 +
           9.3e-5 * TUT13) *
          A2R;
}
/**********************************************************************/
__attribute__((const)) static pair_dbl_mat3x3_t
EarthNutationSTMatricies(const JDType jd);
pair_dbl_mat3x3_t EarthNutationSTMatricies(const JDType jd)
{
   call_once(&iau_flag, init_iau_file);

   pair_dbl_mat3x3_t out;

   const JDType jdEQThresh = JD_RAW(UTC_TIME, J2000_EPOCH, -1095,
                                    RATIONAL_RAW(SEC_PER_DAY / 2, 0, 1));

   JDType jd_tdb_j2000       = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);
   const JDType jd_utc_j2000 = JDChangeSystemEpoch(UTC_TIME, J2000_EPOCH, jd);

   const double TTDB = JDToDays(jd_tdb_j2000) / JDDAY_PER_CENTURY;

   const double TTDB2 = TTDB * TTDB;
   const double TTDB3 = TTDB2 * TTDB;
   double TTDB4       = 0;
   if (NutSelection != NUT_ITRF_1980)
      TTDB4 = TTDB2 * TTDB2;

   const double longAscNodeLunar =
       WrapTo2Pi(_earth_nut_Omega(TTDB, TTDB2, TTDB3, TTDB4));
   const double meanAnomLuna =
       WrapTo2Pi(_earth_nut_l(TTDB, TTDB2, TTDB3, TTDB4));
   const double meanAnomSol =
       WrapTo2Pi(_earth_nut_lp(TTDB, TTDB2, TTDB3, TTDB4));
   const double argLatLuna = WrapTo2Pi(_earth_nut_F(TTDB, TTDB2, TTDB3, TTDB4));
   const double meanElonSol =
       WrapTo2Pi(_earth_nut_D(TTDB, TTDB2, TTDB3, TTDB4));

   const double Epsbar    = _earth_nut_eps(TTDB, TTDB2, TTDB3);
   const double cosEpsbar = cos(Epsbar);

   double dPsi = 0, dEps = 0;
   for (int i = Nut_Info.n_entries - 1; i >= 0; i--) {
      const struct NutEntry *entry = &Nut_Info.entries[i];
      const double apNut =
          entry->a[0] * meanAnomLuna + entry->a[1] * meanAnomSol +
          entry->a[2] * argLatLuna + entry->a[3] * meanElonSol +
          entry->a[4] * longAscNodeLunar;
      const double cosAp = cos(apNut);
      const double sinAp = sin(apNut);

      if (NutSelection == NUT_ITRF_1980) {
         dPsi += (entry->A + entry->B * TTDB) * sinAp;
         dEps += (entry->C + entry->D * TTDB) * cosAp;
      }
      else {
         dPsi += (entry->A + entry->B * TTDB) * sinAp + entry->E * cosAp;
         dEps += (entry->C + entry->D * TTDB) * cosAp + entry->F * sinAp;
      }
   }
   dPsi *= A2R;
   dEps *= A2R;

   const double Eps = Epsbar + dEps;

   // Compute useful trigonometric quantities
   const double cosdPsi   = cos(dPsi);
   const double cosEps    = cos(Eps);
   const double sindPsi   = sin(dPsi);
   const double sinEpsbar = sin(Epsbar);
   const double sinEps    = sin(Eps);

   const mat3x3_t NUT = MAT3X3_SET(
       cosdPsi, -sindPsi * cosEpsbar, -sindPsi * sinEpsbar, sindPsi * cosEps,
       cosEps * cosdPsi * cosEpsbar + sinEps * sinEpsbar,
       sinEpsbar * cosEps * cosdPsi - sinEps * cosEpsbar, sinEps * sindPsi,
       sinEps * cosdPsi * cosEpsbar - sinEpsbar * cosEps,
       sinEps * sinEpsbar * cosdPsi + cosEps * cosEpsbar);

   double eq_equinox = dPsi * cosEps;
   if (isgreater_jd(jd_utc_j2000, jdEQThresh))
      eq_equinox += (0.00264 * sin(longAscNodeLunar) +
                     0.000063 * sin(2.0 * longAscNodeLunar)) *
                    A2R;

   out.dbl           = GMAT_JD2GMST(jd_utc_j2000) + eq_equinox;
   const mat3x3_t ST = SimpRot(VEC3_PZAXIS, out.dbl);
   out.mat           = MxM(ST, NUT);
   return out;
}
/**********************************************************************/
__attribute__((const)) static mat3x3_t EarthPrecessionMatrix(const JDType jd);
mat3x3_t EarthPrecessionMatrix(const JDType jd)
{
   JDType jd_tdb_j2000 = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);
   const double TTDB   = JDToDays(jd_tdb_j2000) / JDDAY_PER_CENTURY;

   const double TTDB2 = TTDB * TTDB;
   const double TTDB3 = TTDB2 * TTDB;

   const double zeta =
       (2306.2181 * TTDB + 0.30188 * TTDB2 + 0.017998 * TTDB3) * A2R;
   const double Theta =
       (2004.3109 * TTDB - 0.42665 * TTDB2 - 0.041833 * TTDB3) * A2R;
   const double z =
       (2306.2181 * TTDB + 1.09468 * TTDB2 + 0.018203 * TTDB3) * A2R;

   // Compute trigonometric quantities
   const double cosTheta = cos(Theta);
   const double cosz     = cos(z);
   const double coszeta  = cos(zeta);
   const double sinTheta = sin(Theta);
   const double sinz     = sin(z);
   const double sinzeta  = sin(zeta);

   const mat3x3_t PREC =
       MAT3X3_SET(cosTheta * cosz * coszeta - sinz * sinzeta,
                  -sinzeta * cosTheta * cosz - sinz * coszeta, -sinTheta * cosz,
                  sinz * cosTheta * coszeta + sinzeta * cosz,
                  -sinz * sinzeta * cosTheta + cosz * coszeta, -sinTheta * sinz,
                  sinTheta * coszeta, -sinTheta * sinzeta, cosTheta);
   return PREC;
}
/**********************************************************************/
/* returns Greenwich Apparent Sidereal Time and Earth CWN             */
pair_dbl_mat3x3_t GMAT_HiFiEarthCWN(const JDType jd)
{
   pair_dbl_mat3x3_t out;
   JDType jd_tdb_j2000 = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);
   mat3x3_t P          = EarthPrecessionMatrix(jd_tdb_j2000);
   out                 = EarthNutationSTMatricies(jd_tdb_j2000);
   out.mat             = MxM(out.mat, P);
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
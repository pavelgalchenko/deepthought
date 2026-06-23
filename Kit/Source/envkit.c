/*    This file is distributed with 42,                               */

/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "envkit.h"
#include "42constants.h"
#include "42types.h"
#include "dcmkit.h"
#include "defineskit.h"
#include "geomkit.h"
#include "iokit.h"
#include "timekit.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <threads.h>

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

/**********************************************************************/
vec3_t SphericalHarmGravForce(const long N, const long M,
                              const struct WorldType *W, mat3x3_t CWN,
                              const double mass, const vec3_t pbn)
{
   double Fr, Fth, Fph;
   vec3_t pbw, Fe, gradV = VEC3_ZERO;
   const struct SphereHarmType *GravModel = &W->GravModel;

#ifdef _DEBUG_GRAV_
   // print out gravitational acceleration vector field to a file for debugging
   static int first      = 0;
   static int reporting  = 0;
   static FILE *gravFile = NULL;
   static double theta, phi;
   mat3x3_t eye3 = MAT3X3_EYE;
   if (!first && !strcmp(W->Name, "Earth")) {
      first = 1;
      extern char OutPath[1000];
      gravFile  = FileOpen(OutPath, "/GravModelTest.42", "wt");
      r         = MAGV(pbn);
      reporting = 1;
      for (theta = 0.5; theta <= 179.5; theta += 0.5) {
         for (phi = -180.0; phi < 180.0; phi += 0.5) {
            cth         = cos(theta * D2R);
            sth         = sin(theta * D2R);
            cph         = cos(phi * D2R);
            sph         = sin(phi * D2R);
            vec3_t rvec = VEC3_ZERO;
            rvec.v[0]   = r * sth * cph;
            rvec.v[1]   = r * sth * sph;
            rvec.v[2]   = r * cth;
            vec3_t out  = SphericalHarmGravForce(N, M, W, eye3, mass, rvec);
         }
      }
      reporting = 0;
      fclose(gravFile);
   }
#endif

   if (GravModel->C != NULL && GravModel->N >= 2) {
      /*    Transform p to spherical coords in World frame */
      pbw                  = MxV(CWN, pbn);
      sphere_coord_t coord = getTrigSphericalCoords(pbw);
      const double cth     = coord.cth;
      const double sth     = coord.sth;
      const double cph     = coord.cph;
      const double sph     = coord.sph;

      gradV = SphericalHarmonics(N, M, coord, GravModel->r_ref,
                                 W->mu / GravModel->r_ref, GravModel->C,
                                 GravModel->S, GravModel->Norm);
      Fr    = mass * gradV.v[0];
      Fth   = mass * gradV.v[1];
      Fph   = mass * gradV.v[2];

#ifdef _DEBUG_GRAV_
      if (reporting) {
         // double th[3] = {0};
         // logso3(CWN, th);
         fprintf(gravFile,
                 "%lf, %lf, %18.36le, %18.36le, %18.36le, %18.36le \n", theta,
                 phi, r, gradV[0], gradV[1], gradV[2]);
      }
#endif

      /*    Transform back to cartesian coords in Newtonian frame */
      Fe.v[0] = (Fr * sth + Fth * cth) * cph - Fph * sph;
      Fe.v[1] = (Fr * sth + Fth * cth) * sph + Fph * cph;
      Fe.v[2] = Fr * cth - Fth * sth;

      gradV = MTxV(CWN, Fe);
   }
   return gradV;
}
/**********************************************************************/
/*  IGRF Magnetic field model                                      *  */
#define nYears 26
static double **IGRF_C = NULL, **IGRF_S = NULL, **IGRF_Norm = NULL;
static double **IGRF_Cdat[nYears + 1] = {NULL};
static double **IGRF_Sdat[nYears + 1] = {NULL};

static char IGRF_ModelPath[1000] = {'\0'};
static long IGRF_warned          = 0;
static __once_flag igrf_flag     = __ONCE_FLAG_INIT;
static void load_igrf_file()
{
   double dum[nYears + 1];
   long k;
   long n, m;
   char gh;

   IGRF_C    = CreateMatrix(14, 14);
   IGRF_S    = CreateMatrix(14, 14);
   IGRF_Norm = CreateMatrix(14, 14);

   for (k = 0; k < nYears + 1; k++) {
      IGRF_Cdat[k] = CreateMatrix(14, 14);
      IGRF_Sdat[k] = CreateMatrix(14, 14);
   }

   /* Get data from IGRF20.txt */
   const char *file_name = "igrf14coeffs.txt";
   FILE *IGRFfile        = FileOpen(IGRF_ModelPath, file_name, "r");
   // skip first 4 lines

   char buffer[BUFSIZ] = {0};
   for (int i = 0; i < 4; i++)
      fgets(buffer, sizeof(buffer), IGRFfile);
   while (fgets(buffer, sizeof(buffer), IGRFfile) != NULL) {
      sscanf(buffer,
             "%c %ld %ld %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf "
             "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
             &gh, &n, &m, &dum[0], &dum[1], &dum[2], &dum[3], &dum[4], &dum[5],
             &dum[6], &dum[7], &dum[8], &dum[9], &dum[10], &dum[11], &dum[12],
             &dum[13], &dum[14], &dum[15], &dum[16], &dum[17], &dum[18],
             &dum[19], &dum[20], &dum[21], &dum[22], &dum[23], &dum[24],
             &dum[25], &dum[26]);
      switch (gh) {
         case 'g':
            for (k = 0; k < nYears + 1; k++)
               IGRF_Cdat[k][n][m] = dum[k];
            break;
         case 'h':
            for (k = 0; k < nYears + 1; k++)
               IGRF_Sdat[k][n][m] = dum[k];
            break;
         default:
            fprintf(stderr,
                    "Invalid leading character in IGRF file %s. Exiting...\n",
                    file_name);
            exit(EXIT_FAILURE);
            break;
      }
   }
   fclose(IGRFfile);
   /* Transform from Schmidt normalization to Neumann normalization */
   for (n = 1; n <= 13; n++) {
      for (m = 0; m <= n; m++) {
         IGRF_Norm[n][m] = 1.0;
         if (m != 0)
            IGRF_Norm[n][m] = sqrt(2.0 / factDfact(n + m, n - m));
      }
   }
}

vec3_t IGRFMagField(const char *ModelPath, const DateType UTC, const long N,
                    const long M, const vec3_t pbn, const double PriMerAng)
{
   if (IGRF_ModelPath[0] == '\0')
      strncpy(IGRF_ModelPath, ModelPath, 999);
   call_once(&igrf_flag, load_igrf_file);

   static const double t[nYears] = {
       1900.0, 1905.0, 1910.0, 1915.0, 1920.0, 1925.0, 1930.0, 1935.0, 1940.0,
       1945.0, 1950.0, 1955.0, 1960.0, 1965.0, 1970.0, 1975.0, 1980.0, 1985.0,
       1990.0, 1995.0, 2000.0, 2005.0, 2010.0, 2015.0, 2020.0, 2025.0};

   double Br, Bth, Bph;
   vec3_t pbe, gradV, BVE;
   mat3x3_t CEN;
   const double Re = 6371200.0;

#ifdef _DEBUG_MAG_
   static long First    = 1;
   static FILE *magFile = NULL;
   static int reporting = 0;
   static double theta, phi;
   if (First) {
      First = 0;
      extern char OutPath[1000];
      magFile   = FileOpen(OutPath, "/IGRFModelTest.42", "wt");
      r         = MAGV(pbn);
      reporting = 1;
      fprintf(magFile, "%ld/%02ld/%02ld %02ld:%02ld:%.6lf\n", UTC.Year,
              UTC.Month, UTC.Day, UTC.Hour, UTC.Minute,
              rational2double(UTC.Second));
      for (theta = 0.5; theta <= 179.5; theta += 0.5) {
         for (phi = -180.0; phi < 180.0; phi += 0.5) {
            cth = cos(theta * D2R);
            sth = sin(theta * D2R);
            cph = cos(phi * D2R);
            sph = sin(phi * D2R);
            vec3_t rvec;
            rvec.v[0]  = r * sth * cph;
            rvec.v[1]  = r * sth * sph;
            rvec.v[2]  = r * cth;
            vec3_t out = IGRFMagField(ModelPath, UTC, N, M, rvec, 0);
         }
      }
      reporting = 0;
      fclose(magFile);
   }
#endif

   const double doy  = (UTC.doy - 1) + (UTC.Hour - 1) / 24.0 +
                       UTC.Minute / 1440.0 +
                       (jdsecond2double(UTC.Second)) / 86400.0;
   const double year = UTC.Year + doy / (UTC.Year % 4 ? 365.0 : 366.0);
   if (year > 2020) {
      if (!IGRF_warned && year > t[nYears - 1] + 5) {
         IGRF_warned = 1;
         printf("***** WARNING: IGRF model only well defined up to %ld; "
                "IGRF values at %lf may be of reduced accuracy. *****\n",
                (long)t[nYears - 1] + 5, year);
      }

      const double dt =
          (year > t[nYears - 1] + 5.0 ? 5.0 : year - t[nYears - 1]);
      for (int n = 0; n <= 13; n++) {
         for (int m = 0; m <= n; m++) {
            IGRF_C[n][m] =
                IGRF_Cdat[nYears - 1][n][m] + IGRF_Cdat[nYears][n][m] * dt;
            IGRF_S[n][m] =
                IGRF_Sdat[nYears - 1][n][m] + IGRF_Sdat[nYears][n][m] * dt;
         }
      }
   }
   else {
      for (int n = 0; n <= 13; n++) {
         for (int m = 0; m <= n; m++) {
            double Y[nYears] = {0.0};
            for (int k = 0; k < nYears; k++)
               Y[k] = IGRF_Cdat[k][n][m];
            IGRF_C[n][m] = LinInterpTbl(t, Y, year, nYears);
            for (int k = 0; k < nYears; k++)
               Y[k] = IGRF_Sdat[k][n][m];
            IGRF_S[n][m] = LinInterpTbl(t, Y, year, nYears);
         }
      }
   }

   CEN = ROT3(PriMerAng);

   /*    Transform p to spherical coords in Earth frame */
   pbe                  = MxV(CEN, pbn);
   sphere_coord_t coord = getTrigSphericalCoords(pbe);
   const double cth     = coord.cth;
   const double sth     = coord.sth;
   const double cph     = coord.cph;
   const double sph     = coord.sph;

   /*    Find Br, Bth, Bph */
   gradV = SphericalHarmonics(N, M, coord, Re, Re, IGRF_C, IGRF_S, IGRF_Norm);
   Br    = -gradV.v[0];
   Bth   = -gradV.v[1];
   Bph   = -gradV.v[2];

#ifdef _DEBUG_MAG_
   if (reporting) {
      fprintf(magFile, "%lf, %lf, %18.36le, %18.36le, %18.36le, %18.36le\n",
              theta, phi - PriMerAng, r, Bph, -Bth, Br);
   }
#endif

   /*    Transform back to cartesian coords in Newtonian frame */
   /*    and convert from nanoTesla to Tesla */
   BVE.v[0] = 1.0E-9 * ((Br * sth + Bth * cth) * cph - Bph * sph);
   BVE.v[1] = 1.0E-9 * ((Br * sth + Bth * cth) * sph + Bph * cph);
   BVE.v[2] = 1.0E-9 * (Br * cth - Bth * sth);

   return MTxV(CEN, BVE);

   /*printf("r,phi,theta: %lf %lf %lf\n",r,phi,theta);
   **printf("Br,Bth,Bph: %lf %lf %lf\n",Br,Bth,Bph);
   **printf("BVE: %lf %lf %lf\n\n",BVE[0],BVE[1],BVE[2]);
   */
#undef nYears
}

/**********************************************************************/
/*  Computes planetary dipole magnetic field vector at S/C position.  */
vec3_t DipoleMagField(double DipoleMoment, vec3_t DipoleAxis,
                      vec3_t DipoleOffset, vec3_t p, double PriMerAng)
{
   double MoR;
   vec3_t PCN, MN;
   magvec3_t uR;
   vec3_t *const R  = &uR.v;
   double *const R3 = &uR.m;
   long i;

   mat3x3_t CEN = ROT3(PriMerAng);

   PCN = MTxV(CEN, DipoleOffset);
   for (i = 0; i < 3; i++)
      R->v[i] = p.v[i] - PCN.v[i];
   uR  = UNITV(*R);
   *R3 = (*R3) * (*R3) * (*R3);
   MN  = MTxV(CEN, DipoleAxis);
   MoR = VoV(MN, *R);
   vec3_t MagVecN;
   for (i = 0; i < 3; i++)
      MagVecN.v[i] = DipoleMoment / (*R3) * (3.0 * MoR * R->v[i] - MN.v[i]);
   return MagVecN;
}
/**********************************************************************/
/* Ref: Rostoker, "Geomagnetic Indices", Rev. of Geophysics and       */
/* Space Physics, Vol 10, No. 4, pp. 935-950, Nov 1972.               */
/* Kp is in the scale [0o 0+ 1- 1o 1+ ... 8+ 9- 9o].                  */
/* We map it to [0.0 0.33 0.67 1.0 1.33 ... 8.33 8.67 9.0] for ease   */
/* of table lookup.                                                   */
double KpToAp(double Kp)
{
   long k;
   double Ap[28] = {0,   2,   3,   4,   5,   6,   7,   9,  12, 15,
                    18,  22,  27,  32,  39,  48,  56,  67, 80, 94,
                    111, 132, 154, 179, 207, 236, 300, 400};

   k = (long)(3.0 * Kp + 0.5); /* Round to the nearest 1/3 */
   if (k < 0)
      k = 0;
   if (k > 27)
      k = 27;

   return (Ap[k]);
} /**********************************************************************/
/*                                                                    */
/* This is an atmospheric density model, described in  "Models of     */
/* Earth's Atmosphere", NASA SP-8021, May 1969.  It is a modification */
/* of the Jacchia model.  Range of validity is from 120 km to 1000 km */
/* altitude.                                                          */
double JacchiaRoberts(vec3_t pbn, vec3_t svn, double F10p7, double Ap)
{
#define ERAD 6378.145E3
   double N[5], Fbar, F, logT, sinth25;
   double W[5] = {1.6731E-24, 6.6435E-24, 4.6496E-23, 5.3104E-23, 2.6552E-23};
   double Lat, RAP, z, DS, RAS, HRA, TAU, T[5], fDD, th, A, X, S, dH, Q, P, B,
       TD, c;
   double T4_800, p;
   double density;

   /* Fbar should properly be an 81-day average.  Here it's assumed to   */
   /* equal to the instantaneous F10p7                                   */
   Fbar = F10p7;
   F    = F10p7;

   p   = MAGV(pbn);
   Lat = asin(pbn.z / p);
   if (pbn.y == 0.0 && pbn.x == 0.0)
      RAP = 0.0;
   else
      RAP = atan2(pbn.y, pbn.x);

   /*    Find Geometric Altitude, z, in km */
   z = 1.0E-3 * (p - ERAD);
   if (z < 120.0) {
      fprintf(stderr, "Altitude %f km too low for Jacchia-Roberts\n", z);
      exit(EXIT_FAILURE);
   }
   else if (z > 1000.0)
      density = 0.0; /* Beyond range of model */
   else {

      /*       Find E-W separation of sun and computation point */
      DS  = asin(svn.z);
      RAS = atan2(svn.y, svn.x);
      HRA = RAP - RAS;

      /*       Temperature Computation */
      TAU = HRA - 0.25 * PI + PI / 15.0 * sin(HRA + 0.25 * PI);
      if (TAU > PI)
         TAU -= TWOPI;
      if (TAU < -PI)
         TAU += TWOPI;

      T[0]    = 362.0 + 3.6 * Fbar;
      T[1]    = T[0] + 1.8 * (F - Fbar);
      fDD     = (0.37 + 0.14 * sin(RAS - 1.222)) * sin(2.0 * RAS + 0.723);
      T[2]    = T[1] + fDD * Fbar;
      th      = 0.5 * fabs(Lat + DS);
      sinth25 = pow(sin(th), 2.5);
      A       = 0.28 * (pow(cos(0.5 * (Lat - DS)), 2.5) - sinth25) /
                (1.0 + 0.28 * sinth25);
      c       = cos(0.5 * TAU);
      T[3]    = T[2] * (1.0 + 0.28 * sinth25) * (1.0 + A * c * c * sqrt(c));
      T[4]    = T[3] + Ap + 100.0 * (1.0 - exp(-0.08 * Ap));

      T4_800 = T[4] - 800.0;
      X      = (T4_800) / (750.0 + 1.722E-4 * T4_800 * T4_800);
      S      = 1.5E-4 + 0.0291 * exp(-0.5 * X * X);
      dH     = (z - 120.0) * (ERAD + 120.0E3) / p;

      /*       Number Density Computations */
      Q = 1.13619033 / (S * T[4]);
      P = (T[4] - 355.0) / T[4];
      B = (1.0 - P) / (1.0 - P * exp(-S * dH));
      /*       Hydrogen */
      TD   = ((((9.753963073E-16 * T[4] - 7.577509214E-12) * T[4] +
                2.341193059E-8) *
                   T[4] -
               3.62095821E-5) *
                  T[4] +
              2.844291123E-2) *
                 T[4] -
             10.48947029;
      logT = log10(T[4]);
      N[0] = pow(10.0, 73.13 - 39.4 * logT + 5.5 * logT * logT);
      if (z > 500.0)
         N[0] *= pow(B, 1.0 + TD + 1.008 * Q) * exp(-1.008 * S * Q * dH);
      /*       Helium */
      N[1] = 3.4E7 * pow(B, 0.63 + 4.002 * Q) * exp(-4.002 * S * Q * dH);
      /*       Nitrogen */
      N[2] = 4.0E11 * pow(B, 1.0 + 28.0134 * Q) * exp(-28.0134 * S * Q * dH);
      /*       Oxygen */
      N[3] = 7.5E10 * pow(B, 1.0 + 31.9988 * Q) * exp(-31.9988 * S * Q * dH);
      /*       Atomic Oxygen */
      N[4] = 7.6E10 * pow(B, 1.0 + 15.9990 * Q) * exp(-15.9990 * S * Q * dH);

      /*       Mass Density */
      density = 1.0E3 * (N[0] * W[0] + N[1] * W[1] + N[2] * W[2] + N[3] * W[3] +
                         N[4] * W[4]);
   }
   return (density);
}
/**********************************************************************/
/*  The tabulated data is taken from a table in the back of           */
/*  Wertz and Larson, "SMAD", Third Edition.  The data is based       */
/*  on MSIS-86, averaged to be a function of altitude only, and       */
/*  F10.7 chosen so that the density is according to the column.      */
/*  Col = 0 (Min), 1, (Mean), or 2 (Max) as defined in table.         */
double SimpleMSIS(vec3_t pbn, long Col)
{
   double AltTable[22]    = {0.0,   100.0,  150.0,  200.0, 250.0, 300.0,
                             350.0, 400.0,  450.0,  500.0, 550.0, 600.0,
                             650.0, 700.0,  750.0,  800.0, 850.0, 900.0,
                             950.0, 1000.0, 1250.0, 1500.0};
   double RhoTable[22][3] = {{1.2, 1.2, 1.2},
                             {4.61E-7, 4.79E-7, 5.10E-7},
                             {1.65E-9, 1.81E-9, 2.04E-9},
                             {1.78E-10, 2.53E-10, 3.52E-10},
                             {3.35E-11, 6.24E-11, 1.06E-10},
                             {8.19E-12, 1.95E-11, 3.96E-11},
                             {2.34E-12, 6.98E-12, 1.66E-11},
                             {7.32E-13, 2.72E-12, 7.55E-12},
                             {2.47E-13, 1.13E-12, 3.61E-12},
                             {8.98E-14, 4.89E-13, 1.80E-12},
                             {3.63E-14, 2.21E-13, 9.25E-13},
                             {1.68E-14, 1.04E-13, 4.89E-13},
                             {9.14E-15, 5.15E-14, 2.64E-13},
                             {5.74E-15, 2.72E-14, 1.47E-13},
                             {3.99E-15, 1.55E-14, 8.37E-14},
                             {2.96E-15, 9.63E-15, 4.39E-14},
                             {2.28E-15, 6.47E-15, 3.00E-14},
                             {1.80E-15, 4.66E-15, 1.91E-14},
                             {1.44E-15, 3.54E-15, 1.27E-14},
                             {1.17E-15, 2.79E-15, 8.84E-15},
                             {4.67E-16, 1.11E-15, 2.59E-15},
                             {2.30E-16, 5.21E-16, 1.22E-15}};

   double EarthRad = 6378.145E3;
   double Alt, a;
   long i1, i2;
   double density;

   if (Col < 0 || Col > 2) {
      fprintf(stderr, "Column %ld out of range in SimpleMSIS.  Bailing out.\n",
              Col);
      exit(EXIT_FAILURE);
   }

   Alt = 1.0E-3 * (MAGV(pbn) - EarthRad);
   if (Alt < 0.0)
      density = 1.2;
   else if (Alt >= 1500.0)
      density = 0.0;
   else {
      i2 = 1;
      while (Alt > AltTable[i2])
         i2++;
      i1 = i2 - 1;
      /* Interpolate density logarithmically */
      a = (Alt - AltTable[i1]) / (AltTable[i2] - AltTable[i1]);
      density =
          exp((1.0 - a) * log(RhoTable[i1][Col]) + a * log(RhoTable[i2][Col]));
   }
   return (density);
}
/**********************************************************************/
/* This simple model is taken from                                    */
/* http://www.grc.nasa.gov/WWW/K-12/airplane/atmosmrm.html            */
double MarsAtmosphereModel(vec3_t r)
{
   double Alt, T, p;
   double MarsRad = 3410.0E3;
   double Density;

   Alt = MAGV(r) - MarsRad;
   if (Alt < 150.0E3) {
      if (Alt < 7000.0) {
         T = -23.4 - 0.00222 * Alt;
      }
      else {
         T = -31.0 - 9.98E-4 * Alt;
      }
      p       = 1.0E3 * 0.699 * exp(-9.0E-5 * Alt); /* p in Pa */
      Density = p / (0.1921 * (T + 273.1));
   }
   else
      Density = 0.0;

   return (Density);
}
/**********************************************************************/
/* http://en.wikipedia.org/wiki/Geodetic_system#Geodetic_versus_geocentric_latitude
 */
vec3_t WGS84ToECEF(const vec3_t lla)
{
   double glat, glong, alt;
   DEAL_VEC3(lla, glat, glong, alt);

   double a  = 6378137.0;
   double f  = 1.0 / 298.257222101;
   double e2 = f * (2.0 - f);
   double X, CosLat, SinLat, CosLng, SinLng;

   CosLat = cos(glat);
   SinLat = sin(glat);
   CosLng = cos(glong);
   SinLng = sin(glong);

   X = sqrt(1.0 - e2 * SinLat * SinLat);
   vec3_t p;
   p.x = (a / X + alt) * CosLat * CosLng;
   p.y = (a / X + alt) * CosLat * SinLng;
   p.z = (a / X * (1.0 - e2) + alt) * SinLat;
   return p;
}
/**********************************************************************/
/* Returns a vec3_t in the order glat, glong, alt                     */
vec3_t ECEFToWGS84(const vec3_t p)
{
   double glat, glong, alt;
   const double a   = 6378137.0;
   const double f   = 1.0 / 298.257222101;
   const double b   = a * (1.0 - f);
   const double e2  = f * (2.0 - f);
   const double ep2 = f * (2.0 - f) / (1.0 - f) / (1.0 - f);
   double r, E2, F, G, C, S, P, Q, r0, U, V, Z0;

   double OneMinusE2, Z1, SpolyG, Qpoly;

   OneMinusE2 = 1.0 - e2;

   r = sqrt(p.x * p.x + p.y * p.y);

   E2 = a * a - b * b;

   Z1 = b * p.z;

   F = 54.0 * Z1 * Z1;

   const double zz = p.z * p.z;

   G = r * r + OneMinusE2 * zz - e2 * E2;

   Z1 = e2 * r / G;
   C  = Z1 * Z1 * F / G;

   S = pow(1.0 + C + sqrt(C * C + 2.0 * C), 1.0 / 3.0);

   SpolyG = (S + 1.0 / S + 1.0) * G;
   P      = F / (3.0 * SpolyG * SpolyG);

   Q = sqrt(1.0 + 2.0 * e2 * e2 * P);

   Qpoly = 1.0 + Q;
   r0    = -P * e2 * r / Qpoly +
           sqrt(0.5 * a * a * Qpoly / Q - P * OneMinusE2 * zz / (Q * Qpoly) -
                0.5 * P * r * r);

   Z1  = r - e2 * r0;
   Z1 *= Z1;

   U = sqrt(Z1 + zz);

   V = sqrt(Z1 + OneMinusE2 * zz);

   Z1 = b * b / a / V;
   Z0 = Z1 * p.z;

   alt   = U * (1.0 - Z1);
   glat  = atan((p.z + ep2 * Z0) / r);
   glong = atan2(p.y, p.x);
   return (vec3_t){.x = glat, .y = glong, .z = alt};
}
/**********************************************************************/
/* Ref Werner and Scheeres, "Exterior Gravitation of a Polyhedron ..." */
/* Returns 1 if PosN is outside polyhedron, 0 if inside */
long PolyhedronGravAcc(struct GeomType *G, double Density, const vec3_t PosN,
                       mat3x3_t CWN, vec3_t *const GravAccN)
{
   struct EdgeType *E;
   struct PolyType *P;
   vec3_t *V1, *V2, *V3;
   vec3_t PosW, GravAccW, re1, re2, rf1, rf2, rf3, r2xr3, Er, Fr;
   double r1, r2, r3, Num, Den, Le, wf, SumWf, Gsig;
   long PosIsOutside;
   long Ie, Ip, i;

   GravAccW = VEC3_ZERO;
   SumWf    = 0.0;

   PosW = MxV(CWN, PosN);

   for (Ie = 0; Ie < G->Nedge; Ie++) {
      E  = &G->Edge[Ie];
      V1 = &G->V[E->Vtx1];
      V2 = &G->V[E->Vtx2];
      for (i = 0; i < 3; i++) {
         re1.v[i] = V1->v[i] - PosW.v[i];
         re2.v[i] = V2->v[i] - PosW.v[i];
      }
      r1 = MAGV(re1);
      r2 = MAGV(re2);
      Le = log((r1 + r2 + E->Length) / (r1 + r2 - E->Length));
      Er = MxV(E->Dyad, re1);
      for (i = 0; i < 3; i++) {
         GravAccW.v[i] -= Er.v[i] * Le;
      }
   }

   for (Ip = 0; Ip < G->Npoly; Ip++) {
      P  = &G->Poly[Ip];
      V1 = &G->V[P->V[0]];
      V2 = &G->V[P->V[1]];
      V3 = &G->V[P->V[2]];
      for (i = 0; i < 3; i++) {
         rf1.v[i] = V1->v[i] - PosW.v[i];
         rf2.v[i] = V2->v[i] - PosW.v[i];
         rf3.v[i] = V3->v[i] - PosW.v[i];
      }
      r1    = MAGV(rf1);
      r2    = MAGV(rf2);
      r3    = MAGV(rf3);
      r2xr3 = VxV(rf2, rf3);
      Num   = VoV(rf1, r2xr3);
      Den   = r1 * r2 * r3 + r1 * VoV(rf2, rf3) + r2 * VoV(rf3, rf1) +
              r3 * VoV(rf1, rf2);
      wf    = 2.0 * atan2(Num, Den);
      Fr    = MxV(P->Dyad, rf1);
      for (i = 0; i < 3; i++) {
         GravAccW.v[i] += Fr.v[i] * wf;
      }
      SumWf += wf;
   }

   Gsig     = 6.67408E-11 * Density;
   GravAccW = SxV(Gsig, GravAccW);

   *GravAccN = MTxV(CWN, GravAccW);

   /* SumWf should be zero if Pos Is Outside, or -4*pi if Pos is Inside */
   PosIsOutside = (SumWf > -TWOPI ? 1 : 0);

   return (PosIsOutside);
}
/**********************************************************************/
/* Ref Werner and Scheeres, "Exterior Gravitation of a Polyhedron ..." */
/* Returns 1 if PosN is outside polyhedron, 0 if inside */
long PolyhedronGravGrad(struct GeomType *G, double Density, vec3_t PosN,
                        mat3x3_t CWN, mat3x3_t *GravGradN)
{
   struct EdgeType *E;
   struct PolyType *P;
   vec3_t *V1, *V2, *V3;
   mat3x3_t GravGradW, GC;
   vec3_t PosW, re1, re2, rf1, rf2, rf3, r2xr3;
   double r1, r2, r3;
   double Num, Den, Le, wf, SumWf, Gsig;
   long PosIsOutside;
   long Ie, Ip, i, j;

   GravGradW = MAT3X3_ZERO;
   SumWf     = 0.0;

   PosW = MxV(CWN, PosN);

   for (Ie = 0; Ie < G->Nedge; Ie++) {
      E  = &G->Edge[Ie];
      V1 = &G->V[E->Vtx1];
      V2 = &G->V[E->Vtx2];
      for (i = 0; i < 3; i++) {
         re1.v[i] = V1->v[i] - PosW.v[i];
         re2.v[i] = V2->v[i] - PosW.v[i];
      }
      r1 = MAGV(re1);
      r2 = MAGV(re2);
      Le = log((r1 + r2 + E->Length) / (r1 + r2 - E->Length));
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++)
            GravGradW.mat[i][j] += E->Dyad.mat[i][j] * Le;
      }
   }

   for (Ip = 0; Ip < G->Npoly; Ip++) {
      P  = &G->Poly[Ip];
      V1 = &G->V[P->V[0]];
      V2 = &G->V[P->V[1]];
      V3 = &G->V[P->V[2]];
      for (i = 0; i < 3; i++) {
         rf1.v[i] = V1->v[i] - PosW.v[i];
         rf2.v[i] = V2->v[i] - PosW.v[i];
         rf3.v[i] = V3->v[i] - PosW.v[i];
      }
      r1    = MAGV(rf1);
      r2    = MAGV(rf2);
      r3    = MAGV(rf3);
      r2xr3 = VxV(rf2, rf3);
      Num   = VoV(rf1, r2xr3);
      Den   = r1 * r2 * r3 + r1 * VoV(rf2, rf3) + r2 * VoV(rf3, rf1) +
              r3 * VoV(rf1, rf2);
      wf    = 2.0 * atan2(Num, Den);
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++)
            GravGradW.mat[i][j] -= P->Dyad.mat[i][j] * wf;
      }
      SumWf += wf;
   }

   Gsig      = 6.67408E-11 * Density;
   GravGradW = SxM(Gsig, GravGradW);

   GC         = MxM(GravGradW, CWN);
   *GravGradN = MTxM(CWN, GC);

   /* SumWf should be zero if Pos Is Outside, or -4*pi if Pos is Inside */
   PosIsOutside = (SumWf > -6.28 ? 1 : 0);

   return (PosIsOutside);
}
/**********************************************************************/
vec3_t GravGradTimesInertia(const mat3x3_t g, const mat3x3_t I)
{
   vec3_t GGxI;
   GGxI.v[0] = (I.mat[2][2] - I.mat[1][1]) * g.mat[1][2] +
               (g.mat[1][1] - g.mat[2][2]) * I.mat[1][2] +
               I.mat[0][2] * g.mat[1][0] - I.mat[0][1] * g.mat[2][0];
   GGxI.v[1] = (I.mat[0][0] - I.mat[2][2]) * g.mat[2][0] +
               (g.mat[2][2] - g.mat[0][0]) * I.mat[2][0] +
               I.mat[0][1] * g.mat[1][2] - I.mat[1][2] * g.mat[0][1];
   GGxI.v[2] = (I.mat[1][1] - I.mat[0][0]) * g.mat[0][1] +
               (g.mat[0][0] - g.mat[1][1]) * I.mat[0][1] +
               I.mat[1][2] * g.mat[0][2] - I.mat[0][2] * g.mat[1][2];
   return GGxI;
}
/* #ifdef __cplusplus
** }
** #endif
*/

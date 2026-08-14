/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "orbkit.h"
#include "42constants.h"
#include "dcmkit.h"
#include "defineskit.h"
#include "timekit.h"
#include "utilkit.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

/**********************************************************************/
void CloneWorld(struct WorldType *const destWorld,
                const struct WorldType srcWorld)
{
   memcpy(destWorld, &srcWorld, sizeof(struct WorldType));
   CloneOrbit(&destWorld->eph, srcWorld.eph);
   // TODO: should deep copy more, but all other pointers in WorldType (e.g.,
   // WorldType::GravModel::C) are not modified after initial world
   // configuration.
}
/**********************************************************************/
void CopyWorld(struct WorldType *const destWorld,
               const struct WorldType srcWorld)
{
   memcpy(destWorld, &srcWorld, sizeof(struct WorldType));
   CopyOrbit(&destWorld->eph, srcWorld.eph);
}
/**********************************************************************/
double GetWorldW(JDType jd, const struct WorldType *const world)
{
   jd                         = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);
   const double day_tdb_j2000 = JDToDays(jd);
   const AngDataType *const pm_data = &world->ang_data[0];

   return (pm_data->ang[1] + 2.0 * pm_data->ang[2] * day_tdb_j2000) * D2R /
          SEC_PER_DAY;
}
/**********************************************************************/
vec3_t GetWorldWln(JDType jd, const struct WorldType *const world)
{
   vec3_t wln;
   wln.x = 0.0;
   wln.y = 0.0;
   wln.z = GetWorldW(jd, world);
   return wln;
}
/**********************************************************************/
AngDataType CopyAngData(const AngDataType src)
{
   AngDataType dest = ANGDATATYPE_INVALID;
   dest.ang_char    = src.ang_char;
   CopyVG(dest.ang, src.ang, 3);
   dest.n_ang = src.n_ang;
   dest.n_E   = src.n_E;

   dest.nut_prec_E   = calloc(dest.n_E, sizeof(double[2]));
   dest.nut_prec_ang = calloc(dest.n_ang, sizeof(double));
   CopyVG(dest.nut_prec_E[0], src.nut_prec_E[0], 2 * dest.n_E);
   CopyVG(dest.nut_prec_ang, src.nut_prec_ang, dest.n_ang);
   return dest;
}
/**********************************************************************/
double GetWorldAng(JDType jd, const AngDataType *const ang_data)
{
   jd = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);

   const double day_tdb_j2000 = JDToDays(jd);
   const double cen_tdb_j2000 = day_tdb_j2000 / JDDAY_PER_CENTURY;

   double angle   = 0;
   double d       = 1;
   double day_mul = cen_tdb_j2000;
   if (ang_data->ang_char == 'P')
      day_mul = day_tdb_j2000;

   for (int i = 0; i < 3; i++) {
      angle += ang_data->ang[i] * d;
      d     *= day_mul;
   }

   if (ang_data->n_E && ang_data->n_ang) {
      double E[ang_data->n_E];
      double (*s_func)(double) = sin;
      if (ang_data->ang_char == 'D')
         s_func = cos;

      for (int i = 0; i < ang_data->n_E; i++) {
         E[i] = ang_data->nut_prec_E[i][0] +
                ang_data->nut_prec_E[i][1] * cen_tdb_j2000;
         E[i] = WrapDeg(E[i]) * D2R;
      }

      for (int i = 0; i < ang_data->n_ang; i++)
         angle += ang_data->nut_prec_ang[i] * s_func(E[i]);
   }

   angle = WrapDeg(angle);
   return angle * D2R;
}
/**********************************************************************/
/* Return both the Prime Meridian angle of a world and the            */
/* corresponding CWN                                                  */
pair_dbl_mat3x3_t GetWorldCWN(JDType jd, const AngDataType *const ang_data)
{
   pair_dbl_mat3x3_t pri_cwn;

   const AngDataType *pm_data = NULL;
   for (int i = 0; i < 3; i++) {
      if (ang_data[i].ang_char == 'P') {
         pm_data = &ang_data[i];
         break;
      }
   }
   if (pm_data == NULL) {
      fprintf(stderr,
              "Expected a 'P'rime Merdian angle in GetWorldCWN, got '%c', "
              "'%c', and '%c'. Exiting...\n",
              ang_data[0].ang_char, ang_data[1].ang_char, ang_data[2].ang_char);
      exit(EXIT_FAILURE);
   }
   jd = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);

   pri_cwn.dbl = GetWorldAng(jd, pm_data);
   pri_cwn.mat = ROT3(pri_cwn.dbl);
   return pri_cwn;
}
/**********************************************************************/
mat3x3_t GetWorldCNJ(JDType jd, const AngDataType *const ang_data)
{
   const AngDataType *ra_data  = NULL;
   const AngDataType *dec_data = NULL;
   for (int i = 0; i < 3; i++) {
      if (ang_data[i].ang_char == 'R') {
         ra_data = &ang_data[i];
      }
      else if (ang_data[i].ang_char == 'D') {
         dec_data = &ang_data[i];
      }
      if (ra_data != NULL && dec_data != NULL)
         break;
   }
   if (ra_data == NULL || dec_data == NULL) {
      fprintf(stderr,
              "Expected both a 'R'ight Ascension angle and a 'D'eclination "
              "angle in GetWorldCNJ, got '%c', '%c', and '%c'. Exiting...\n",
              ang_data[0].ang_char, ang_data[1].ang_char, ang_data[2].ang_char);
      exit(EXIT_FAILURE);
   }

   JDType jd_tdb_j2000 = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);
   const double ra     = GetWorldAng(jd_tdb_j2000, ra_data);
   const double dec    = GetWorldAng(jd_tdb_j2000, dec_data);

   return A2C(312, (ra + HALFPI), (HALFPI - dec), 0.0);
}
/**********************************************************************/
void CloneOrbit(struct OrbitType *const destOrb, const struct OrbitType srcOrb)
{
   memcpy(destOrb, &srcOrb, sizeof(struct OrbitType));
   if (destOrb->SplineFile)
      destOrb->SplineFile = fopen(destOrb->SplineFileName, "rt");

   if (srcOrb.Ncheb) {
      destOrb->Cheb = malloc(srcOrb.Ncheb * sizeof(struct Cheb3DType));
      for (int i = 0; i < destOrb->Ncheb; i++)
         memcpy(&destOrb->Cheb[i], &srcOrb.Cheb[i], sizeof(struct Cheb3DType));
   }
}
/**********************************************************************/
void CopyOrbit(struct OrbitType *const destOrb, const struct OrbitType srcOrb)
{
   if (destOrb->SplineFile)
      fclose(destOrb->SplineFile);

   memcpy(destOrb, &srcOrb, sizeof(struct OrbitType));
   if (destOrb->SplineFile)
      destOrb->SplineFile = fopen(destOrb->SplineFileName, "rt");

   if (srcOrb.Ncheb) {
      for (int i = 0; i < destOrb->Ncheb; i++)
         memcpy(&destOrb->Cheb[i], &srcOrb.Cheb[i], sizeof(struct Cheb3DType));
   }
}
/**********************************************************************/
WorldID GetWorldIDLenient(const char *const s)
{
   // check if incoming string `s` is either the `str_val` or the `naif_str` of
   // a world. case insensitive
   unsigned long i;
#define X(world, str_val, naif_str, parent)                                    \
   if ((dt_strcasecmp(s, str_val) == 0) || dt_strcasecmp(s, naif_str) == 0)    \
      return world;
   X_WORLD_LIST
#undef X
   if (sscanf(s, "MINORBODY_%lu", &i) == 1)
      return (NMAJORWORLD + i);
   return NULL_WORLD;
}
/**********************************************************************/
WorldID GetWorldID(const char *const s)
{
   // Check if incoming string `s` is either the `str_val` or the `naif_str` of
   // a world. case insensitive. Errors out if no match is found.
   WorldID out = GetWorldIDLenient(s);
   if (out != NULL_WORLD)
      return out;
   fprintf(stderr, "Bogus input %s in GetWorldID (42init.c:%d)\n", s, __LINE__);
   exit(EXIT_FAILURE);
}
/**********************************************************************/
const char *WorldID2NAIFString(const WorldID w_id)
{
   // TODO: what about minor bodies??
   //  Returns the NAIF names of the celestial bodies
   switch (w_id) {
#define X(world, str_val, naif_str, parent)                                    \
   case world:                                                                 \
      return naif_str;
      X_WORLD_LIST
#undef X
      default: {
         fprintf(stderr,
                 "Unknown WorldID %u in WorldID2NAIFString. Exiting...\n",
                 w_id);
         exit(EXIT_FAILURE);
      }
   }
}
/**********************************************************************/
static __once_flag naif_title_str_flag = __ONCE_FLAG_INIT;
static char *naif_title_strs[NWORLD]   = {};
static void init_naif_title()
{
   for (WorldID world = SOL; world < NMAJORWORLD; world++) {
      const char *naif_base  = WorldID2NAIFString(world);
      const int naif_len     = strlen(naif_base);
      naif_title_strs[world] = calloc(naif_len + 1, sizeof(char));
      strcpy(naif_title_strs[world], naif_base);
      totitle_str(naif_title_strs[world], naif_len);
   }
}
const char *WorldID2NAIFString_Title(const WorldID w_id)
{
   call_once(&naif_title_str_flag, init_naif_title);

   if (w_id < SOL || w_id > NMAJORWORLD) {
      fprintf(stderr,
              "Unknown WorldID %u in WorldID2NAIFString_Title. Exiting...\n",
              w_id);
      exit(EXIT_FAILURE);
   }

   return naif_title_strs[w_id];
}
/**********************************************************************/
static __once_flag world_name_str_flag = __ONCE_FLAG_INIT;
static char *world_name_strs[NWORLD]   = {};
static void init_world_names()
{
#define X(world, str_val, naif_str, parent)                                    \
   {                                                                           \
      const int name_len     = strlen(str_val);                                \
      world_name_strs[world] = calloc((name_len) + 1, sizeof(char));           \
      strncpy(world_name_strs[world], str_val, (name_len) + 1);                \
      totitle_str(world_name_strs[world], name_len);                           \
   }
   X_WORLD_LIST
#undef X
}
const char *WorldID2Name(const WorldID w_id)
{
   // TODO: what about minor bodies??
   //  Returns the names of the celestial bodies
   call_once(&world_name_str_flag, init_world_names);

   if (w_id < SOL || w_id > NMAJORWORLD) {
      fprintf(stderr, "Unknown WorldID %u in WorldID2Name. Exiting...\n", w_id);
      exit(EXIT_FAILURE);
   }

   return world_name_strs[w_id];
}
/**********************************************************************/
WorldID GetWorldParent(const WorldID w_id)
{
   switch (w_id) {
#define X(world, str_val, naif_str, parent)                                    \
   case world:                                                                 \
      return parent;
      X_WORLD_LIST
#undef X
      default: {
         fprintf(stderr, "WorldID %u has unspecified parent. Exiting...\n",
                 w_id);
         exit(EXIT_FAILURE);
      }
   }
}
/**********************************************************************/
void WorldConfigureSatellites(const WorldID w_id, long *const n_sat,
                              WorldID **const sat_list)
{
   *n_sat    = 0;
   *sat_list = NULL;
   for (WorldID Iw = SOL; Iw < NMAJORWORLD; Iw++)
      if (GetWorldParent(Iw) == w_id)
         *n_sat += 1;

   if (*n_sat > 0) {
      *sat_list = calloc(*n_sat, sizeof(WorldID));
      if (*sat_list == NULL) {
         fprintf(stderr,
                 "World[%i].Sat calloc returned null pointer . Exiting...\n",
                 w_id);
         exit(EXIT_FAILURE);
      }

      long i_sat = 0;
      for (WorldID Iw = SOL; Iw < NMAJORWORLD; Iw++) {
         if (GetWorldParent(Iw) == w_id) {
            (*sat_list)[i_sat] = Iw;
            i_sat++;
         }
      }
   }
}
/**********************************************************************/
__attribute__((pure)) static double _eccFDF(const double E, double params[2]);
static double _eccFDF(const double E, double params[2])
{
   const double f  = E - params[0] * sin(E) - params[1];
   const double fp = 1.0 - params[0] * cos(E);
   return f / fp;
}
/**********************************************************************/
double MeanAnomToTrueAnom(double MeanAnom, double ecc)
{
#define EPS (1.0E-12)
   double params[2] = {ecc, MeanAnom};
   double E = NewtonRaphson(MeanAnom, EPS, 100, 0.1, 0, &_eccFDF, params);
   return (2.0 * atan(sqrt((1.0 + ecc) / (1.0 - ecc)) * tan(0.5 * E)));
#undef EPS
}
/**********************************************************************/
__attribute__((pure)) static double _parabolFDF(const double x,
                                                double params[1]);
static double _parabolFDF(const double x, double params[1])
{
   const double f  = x * (x * x + 3.0) - 2.0 * params[0];
   const double fp = 3.0 * x * x + 3.0;
   return f / fp;
}
/**********************************************************************/
static double _hyperbolFDF(const double H, double params[2])
    __attribute__((pure));
static double _hyperbolFDF(const double H, double params[2])
{
   const double f  = params[0] * sinh(H) - H - params[1];
   const double fp = params[0] * cosh(H) - 1.0;
   return f / fp;
}
/**********************************************************************/
double TrueAnomaly(double mu, double p, double e, double t)
{
#define EPS (1.0E-12)
   double Anom;
   double p3 = p * p * p;

   if (e == 1.0) {
      double params[1] = {3.0 * sqrt(mu / p3) * t};
      double x = NewtonRaphson(0, EPS, 100, 1.0, 0, &_parabolFDF, params);
      Anom     = 2.0 * atan(x);
   }
   else if (e > 1.0) {
      double e1        = e * e - 1.0;
      double N         = sqrt(mu * e1 * e1 * e1 / p3) * t;
      double Ne        = N / e;
      double params[2] = {e, N};
      /* H0 = arcsinh(N/e); */
      double H = NewtonRaphson(log(Ne + sqrt(Ne * Ne + 1.0)), EPS, 100, 0.1, 0,
                               &_hyperbolFDF, params);
      Anom     = 2.0 * atan(sqrt((e + 1.0) / (e - 1.0)) * tanh(0.5 * H));
   }
   else {
      double a = p / (1.0 - e * e);
      double M = sqrt(mu / (a * a * a)) * t;
      M        = fmod(M + PI, TWOPI) - PI;
      Anom     = MeanAnomToTrueAnom(M, e);
   }

   return (Anom);
#undef EPS
}
/**********************************************************************/
static double _hyperradFDF(const double r, double params[7])
{
   const double rold = params[5];
   const double fold = params[6];
   const double sqX  = sqrt((2.0 - params[0] / r) / r - params[1]);
   const double f =
       r * sqX -
       params[2] * log(((sqX + 1.0 / params[2]) * r + params[2]) / params[3]) -
       params[4];
   params[5]       = r;
   params[6]       = f;
   const double fp = (f - fold) / (r - rold);
   return f / fp;
}
/**********************************************************************/
/* As a hyperbolic trajectory approaches its asymptotes, it's more    */
/* precise to find the radius rather than the true anomaly as a       */
/* function of time since periapsis passage.                          */
/* Sensitivity crossover happens when velocity is more radial than    */
/* tangential, which happens at about N = e-1 (approximation improves */
/* as e->inf.                                                         */
void FindHyperbolicRadius(double mu, double p, double e, double dt, double *R)
{

   double a, q, sqma, T, Den, alpha, sqX, r, f;

   a     = p / (1.0 - e * e);
   q     = p / (1.0 + e);
   sqma  = sqrt(-a);
   T     = -sqrt(mu) / a * fabs(dt);
   Den   = q / sqma + sqma;
   alpha = 1.0 / a;

   r   = p;
   sqX = sqrt((2.0 - p / r) / r - alpha);
   f   = r * sqX - sqma * log(((sqX + 1.0 / sqma) * r + sqma) / Den) - T;

   double params[7] = {p, alpha, sqma, Den, T, r, f};
   *R = NewtonRaphson(1.1 * p, 1.0E-3, 500, 1.0E9, 0, &_hyperradFDF, params);
}
/**********************************************************************/
double atanh(double x)
{
   if (fabs(x) < 1.0)
      return 0.5 * log((1.0 + x) / (1.0 - x));
   else
      return 0.0;
}
/**********************************************************************/
double TimeSincePeriapsis(double mu, double p, double e, double th)
{
   double x, a, B, E, H, dt;

   x = tan(0.5 * th);

   if (e == 1.0) {
      B  = 0.5 * x * (x * x + 3.0);
      dt = sqrt(p * p * p / mu) / 3.0 * B;
   }
   else if (e < 1.0) {
      a  = p / (1.0 - e * e);
      E  = 2.0 * atan(sqrt((1.0 - e) / (1.0 + e)) * x);
      dt = (E - e * sin(E)) * sqrt(a * a * a / mu);
   }
   else {
      a  = p / (1.0 - e * e);
      H  = 2.0 * atanh(sqrt((e - 1.0) / (e + 1.0)) * x);
      dt = (e * sinh(H) - H) * sqrt(-a * a * a / mu);
   }
   return (dt);
}
/**********************************************************************/
/* Find position and velocity, given initial position, velocity, and  */
/* true anomaly difference.  This routine good for all orbits.        */
/* See Battin, p.130                                                  */
void RV02RV(double mu, vec3_t xr0, vec3_t xv0, double anom, vec3_t *xr,
            vec3_t *xv)
{
   double sqmu, cth, sth, cth1, s0, p, r0, sqp, r, F, Ft, G, Gt;
   vec3_t R0xV0;
   long i;

   sqmu = sqrt(mu);
   cth  = cos(anom);
   sth  = sin(anom);
   cth1 = 1.0 - cth;

   s0    = VoV(xr0, xv0) / sqmu;
   R0xV0 = VxV(xr0, xv0);
   p     = MAGV(R0xV0) / mu;
   r0    = MAGV(xr0);
   sqp   = sqrt(p);

   r  = p * r0 / (r0 + (p - r0) * cth - sqp * s0 * sth);
   F  = 1.0 - r / p * cth1;
   G  = r * r0 * sth / (sqmu * sqp);
   Ft = sqmu / (r0 * p) * (s0 * cth1 - sqp * sth);
   Gt = 1.0 - r0 / p * cth1;

   for (i = 0; i < 3; i++) {
      xr->v[i] = F * xr0.v[i] + G * xv0.v[i];
      xv->v[i] = Ft * xr0.v[i] + Gt * xv0.v[i];
   }
}
/**********************************************************************/
/* Compute position and velocity given orbital elements.  Works for   */
/* circular, elliptical, parabolic and hyperbolic orbits.             */
void Eph2RV(double mu, double p, double e, double i, double RAAN, double ArgP,
            double dt, vec3_t *r, vec3_t *v, double *anom)
{
   mat3x3_t CPN;
   vec3_t pr, pv;
   double R, th, cth, sth, c2;
   double C1, S1, C2, S2, C3, S3;

   th  = TrueAnomaly(mu, p, e, dt);
   sth = sin(th);
   cth = cos(th);
   R   = p / (1.0 + e * cth);

   c2   = sqrt(mu / p);
   pr.x = R * cth;
   pr.y = R * sth;
   pr.z = 0.0;
   pv.x = -c2 * sth;
   pv.y = c2 * (e + cth);
   pv.z = 0.0;

   C1 = cos(RAAN);
   S1 = sin(RAAN);
   C2 = cos(i);
   S2 = sin(i);
   C3 = cos(ArgP);
   S3 = sin(ArgP);

   CPN.mat[0][0] = -S1 * C2 * S3 + C3 * C1;
   CPN.mat[1][0] = -S1 * C2 * C3 - S3 * C1;
   CPN.mat[2][0] = S1 * S2;
   CPN.mat[0][1] = C1 * C2 * S3 + C3 * S1;
   CPN.mat[1][1] = C1 * C2 * C3 - S3 * S1;
   CPN.mat[2][1] = -C1 * S2;
   CPN.mat[0][2] = S2 * S3;
   CPN.mat[1][2] = S2 * C3;
   CPN.mat[2][2] = C2;

   *r    = MTxV(CPN, pr);
   *v    = MTxV(CPN, pv);
   *anom = th;
}
/**********************************************************************/
/* Compute orbital elements, given position and velocity.  Works for  */
/* for all eccentricities.                                            */
void RV2Eph(double time, double mu, vec3_t xr, vec3_t xv, double *SMA,
            double *e, double *i, double *RAAN, double *ArgP, double *th,
            double *tp, double *SLR, double *alpha, double *rmin,
            double *MeanMotion, double *Period)
{
#define EPS (1.0E-12)

   double v, cth, cosw, sinw;
   double rohxe, h, dt;
   vec3_t xn, hxn, xh, xe, vxh, hxe;
   magvec3_t ur;
   vec3_t *const rhat = &ur.v;
   double *const r    = &ur.m;

   ur = UNITV(xr);
   v  = MAGV(xv);

   *alpha = 2.0 / *r - v * v / mu;
   *SMA   = 1.0 / (*alpha);
   if (*alpha > 0.0) {
      /* Elliptic orbit */
      *MeanMotion = sqrt(mu * (*alpha) * (*alpha) * (*alpha));
      *Period     = TWOPI / (*MeanMotion);
   }
   else {
      /* For hyperbolic orbits, these need special interpretation */
      *MeanMotion = sqrt(-mu * (*alpha) * (*alpha) * (*alpha));
      *Period     = TWOPI / (*MeanMotion);
   }

   xh = VxV(xr, xv);
   h  = MAGV(xh); /* 3D mag */

   /* Semi-Latus Rectum */
   *SLR = VoV(xh, xh) / mu;

   vxh = VxV(xv, xh);

   xe.x = vxh.x / mu - rhat->x;
   xe.y = vxh.y / mu - rhat->y;
   xe.z = vxh.z / mu - rhat->z;
   *e   = MAGV(xe);

   *rmin = *SLR / (1.0 + *e);

   if (h > EPS) {
      xh.x /= h;
      xh.y /= h;
      xh.z /= h;
   }

   h = sqrt(xh.x * xh.x + xh.y * xh.y); /* 2D mag */
   if (*e < EPS) {                      /* Circular */
      if (h < EPS) {                    /* Equatorial */
         /* Arbitrarily set RAAN, omg = 0 */
         *RAAN = 0.0;
         *i    = (xh.z > 0.0 ? 0.0 : PI);
         *ArgP = 0.0;
         xe    = VEC3_PXAXIS;
      }
      else { /* Inclined */
         /* Arbitrarily set omg = 0 */
         *RAAN = atan2(xh.x, -xh.y);
         *i    = acos(xh.z);
         *ArgP = 0.0;
         xe.x  = -xh.y / h;
         xe.y  = xh.x / h;
         xe.z  = 0.0;
      }
   }
   else { /* Eccentric */
      xe = SxV(1.0 / *e, xe);
      if (h < EPS) { /* Equatorial */
         /* Arbitrarily set RAAN = 0 */
         xn    = VEC3_PXAXIS;
         *RAAN = 0.0;
         *i    = (xh.z > 0.0 ? 0.0 : PI);
         /* Find omg */
         hxn.x = -xh.z * xn.y;
         hxn.y = xh.z * xn.x;
         hxn.z = xh.x * xn.y - xh.y * xn.x;
         cosw  = xe.x * xn.x + xe.y * xn.y;
         sinw  = VoV(xe, hxn);
         *ArgP = atan2(sinw, cosw);
      }
      else { /* Inclined */
         /* RAAN, omg both well defined */
         xn.x  = -xh.y / h;
         xn.y  = xh.x / h;
         xn.z  = 0.0;
         *RAAN = atan2(xn.y, xn.x);
         *i    = acos(xh.z);
         /* Find omg */
         hxn.x = -xh.z * xn.y;
         hxn.y = xh.z * xn.x;
         hxn.z = xh.x * xn.y - xh.y * xn.x;
         cosw  = xe.x * xn.x + xe.y * xn.y;
         sinw  = VoV(xe, hxn);
         *ArgP = atan2(sinw, cosw);
      }
   }

   cth = VoV(*rhat, xe);
   *th = acos(cth);

   hxe   = VxV(xh, xe);
   rohxe = VoV(*rhat, hxe);

   if (rohxe < 0.0)
      *th = TWOPI - *th;

   dt  = TimeSincePeriapsis(mu, *SLR, *e, *th);
   *tp = time - dt;
#undef EPS
}
/**********************************************************************/
void TLE2MeanEph(const char Line1[80], const char Line2[80], JDType jd,
                 struct OrbitType *O)
{
#define EPS (1.0E-12)

   char YearString[3];
   char DOYstring[13];
   char IncString[9];
   char RAANstring[10];
   char EccString[8];
   char omgstring[9];
   char MeanAnomString[9];
   char MeanMotionString[12];
   DateType date = {0};
   JDType jdEpoch;
   double FloatDOY, FracDay;
   double j2000_tt;
   /* Parameters quoted from SatelliteToolbox.jl's sgp4_model.jl */
   double mu = 3.986005E14;
   double Re = 6378.137E3;
   double J2 = 1.08262998905E-3;
   double Coef;

   date.system = UTC_TIME;

   strncpy(YearString, &Line1[18], 2);
   YearString[2] = 0;
   date.Year     = (long)atoi(YearString);
   if (date.Year < 57)
      date.Year += 2000;
   else
      date.Year += 1900;
   strncpy(DOYstring, &Line1[20], 12);
   DOYstring[12] = 0;
   FloatDOY      = (double)atof(DOYstring);
   date.doy      = (long)FloatDOY;
   FracDay       = FloatDOY - ((double)date.doy);
   DOY2MD(date.Year, date.doy, &date.Month, &date.Day);
   jdEpoch  = Date2JD(date, J2000_EPOCH);
   jd       = JDChangeSystem(TT_TIME, jd);
   jdEpoch  = JDAddDays(jdEpoch, FracDay);
   O->Epoch = JDToDynTime(jdEpoch);
   j2000_tt = JDToDynTime(jd);

   strncpy(IncString, &Line2[8], 8);
   IncString[8] = 0;
   O->inc       = ((double)atof(IncString)) * D2R;

   strncpy(RAANstring, &Line2[17], 9);
   RAANstring[9] = 0;
   O->RAAN0      = ((double)atof(RAANstring)) * D2R;

   strncpy(EccString, &Line2[26], 7);
   EccString[7] = 0;
   O->ecc       = ((double)atof(EccString)) * 1.0E-7;

   strncpy(omgstring, &Line2[34], 8);
   omgstring[8] = 0;
   O->ArgP0     = ((double)atof(omgstring)) * D2R;

   strncpy(MeanAnomString, &Line2[43], 8);
   MeanAnomString[8] = 0;
   O->MeanAnom0      = ((double)atof(MeanAnomString)) * D2R;

   strncpy(MeanMotionString, &Line2[52], 11);
   MeanMotionString[11] = 0;
   O->MeanMotion        = ((double)atof(MeanMotionString)) * TWOPI / 86400.0;
   O->Period            = TWOPI / (O->MeanMotion);

   /* Time of Periapsis passage given in seconds since J2000 */
   O->tp = O->Epoch - O->MeanAnom0 / (O->MeanMotion);
   while ((j2000_tt - O->tp) > O->Period)
      O->tp += O->Period;
   while ((j2000_tt - O->tp) < -(O->Period))
      O->tp -= O->Period;

   O->MeanSMA = pow(mu / (O->MeanMotion * O->MeanMotion), 1.0 / 3.0);
   O->SMA     = O->MeanSMA;
   O->alpha   = 1.0 / (O->SMA);
   O->SLR     = O->SMA * (1.0 - O->ecc * O->ecc);
   O->rmin    = O->SLR / (1.0 + O->ecc);

   O->MeanAnom = O->MeanMotion * (j2000_tt - O->tp);
   O->anom     = MeanAnomToTrueAnom(O->MeanAnom, O->ecc);

   /* Initialize J2 Drift Parameters (ref Markley and Crassidis, Ch. 10) */
   /* 10.121 */
   if (O->J2DriftEnabled) {
      Coef       = 1.5 * J2 * Re * Re / (O->SLR * O->SLR) * O->MeanMotion;
      O->RAANdot = -Coef * cos(O->inc);
      O->ArgPdot = Coef * (2.0 - 2.5 * sin(O->inc) * sin(O->inc));
      O->RAAN    = O->RAAN0 + O->RAANdot * (j2000_tt - O->Epoch);
      while (O->RAAN > PI)
         O->RAAN -= TWOPI;
      while (O->RAAN < -PI)
         O->RAAN += TWOPI;
      O->ArgP = O->ArgP0 + O->ArgPdot * (j2000_tt - O->Epoch);
      while (O->ArgP > PI)
         O->ArgP -= TWOPI;
      while (O->ArgP < -PI)
         O->ArgP += TWOPI;
      /* 10.126 */
      O->J2Rw2bya = J2 * Re * Re / O->MeanSMA;
   }
   else {
      O->RAANdot  = 0.0;
      O->ArgPdot  = 0.0;
      O->RAAN     = O->RAAN0;
      O->ArgP     = O->ArgP0;
      O->J2Rw2bya = 0.0;
   }
}
/**********************************************************************/
/* Ref: Markley and Crassidis, 10.4.3                                 */
/* Osculating elements drift from initial conditions due to J2        */
void MeanEph2RV(struct OrbitType *O, double dyntime)
{
   double e, e2, sin2i, sinw, sin2w, cosnu, g, cth, sth, R;
   mat3x3_t CPN;
   vec3_t pr, pv;
   double C1, S1, C2, S2, C3, S3;
   long i;

   /* 10.121a,b */
   if (O->J2DriftEnabled) {
      O->ArgP = O->ArgP0 + O->ArgPdot * (dyntime - O->Epoch);
      O->RAAN = O->RAAN0 + O->RAANdot * (dyntime - O->Epoch);
   }

   /* 10.122 */
   O->MeanAnom =
       fmod(O->MeanAnom0 + O->MeanMotion * (dyntime - O->Epoch) - PI, TWOPI) +
       PI;

   O->anom = MeanAnomToTrueAnom(O->MeanAnom, O->ecc);

   e     = O->ecc;
   e2    = e * e;
   sin2i = sin(O->inc) * sin(O->inc);

   /* 10.127 */
   sinw          = sin(O->ArgP + O->anom);
   sin2w         = sinw * sinw;
   cosnu         = cos(O->anom);
   double gTerm  = (1.0 + e * cosnu) / (1.0 - e2);
   double gTerm2 = gTerm * gTerm;
   g             = (gTerm2 * gTerm) * (1.0 - 3.0 * sin2i * sin2w);

   /* 10.126 */
   O->SMA = O->MeanSMA + O->J2Rw2bya * g;

   O->SLR   = O->SMA * (1.0 - e2);
   O->alpha = 1.0 / O->SMA;
   O->rmin  = O->SLR / (1.0 + O->ecc);

   sth = sin(O->anom);
   cth = cos(O->anom);
   R   = O->SLR / (1.0 + e * cth);

   pr.x = R * cth;
   pr.y = R * sth;
   pr.z = 0.0;
   if (O->J2DriftEnabled) {
      // TODO: double/triple check the pv calculations here, there is notable
      // difference between pv and finite differencing of pr, mostly in
      // periapsis direction but that may just happen (due to, ya know, finite
      // differencing)
      // TODO: this only works for elliptical orbits
      double sqrterat   = sqrt((1.0 - e) / (1.0 + e));
      double EccAnom    = 2.0 * atan(sqrterat * tan(0.5 * O->anom));
      double cE         = cos(EccAnom);
      double EccAnomDot = O->MeanMotion / (1.0 - e * cE);
      double AnomDot    = (1.0 + cth) / (1.0 + cE) * EccAnomDot / sqrterat;
      double gdot =
          -3.0 * gTerm2 *
          ((e * AnomDot * sth / (1.0 - e2)) * (1.0 - 3.0 * sin2i * sin2w) +
           2.0 * gTerm * sin2i * sinw * cos(O->ArgP + O->anom) *
               (O->ArgPdot + AnomDot));
      double SMAdot = O->J2Rw2bya * gdot;
      double Rdot =
          SMAdot * (1 - e * cE) + O->SMA * e * sin(EccAnom) * EccAnomDot;

      pv.x = Rdot * cth - R * AnomDot * sth;
      pv.y = Rdot * sth + R * AnomDot * cth;
      pv.z = 0.0;
   }
   else {
      double c2 = sqrt(O->mu / O->SLR);
      pv.x      = -c2 * sth;
      pv.y      = c2 * (e + cth);
      pv.z      = 0.0;
   }

   C1 = cos(O->RAAN);
   S1 = sin(O->RAAN);
   C2 = cos(O->inc);
   S2 = sin(O->inc);
   C3 = cos(O->ArgP);
   S3 = sin(O->ArgP);

   CPN.mat[0][0] = -S1 * C2 * S3 + C3 * C1;
   CPN.mat[1][0] = -S1 * C2 * C3 - S3 * C1;
   CPN.mat[2][0] = S1 * S2;
   CPN.mat[0][1] = C1 * C2 * S3 + C3 * S1;
   CPN.mat[1][1] = C1 * C2 * C3 - S3 * S1;
   CPN.mat[2][1] = -C1 * S2;
   CPN.mat[0][2] = S2 * S3;
   CPN.mat[1][2] = S2 * C3;
   CPN.mat[2][2] = C2;

   for (i = 0; i < 3; i++) {
      O->PosN.v[i] = pr.x * CPN.mat[0][i] + pr.y * CPN.mat[1][i];
      O->VelN.v[i] = pv.x * CPN.mat[0][i] + pv.y * CPN.mat[1][i];
   }
   if (O->J2DriftEnabled) {
      vec3_t wxr  = VEC3_ZERO;
      wxr.x      += (-pr.x * CPN.mat[0][1] - pr.y * CPN.mat[1][1]) * O->RAANdot;
      wxr.y      += (+pr.x * CPN.mat[0][0] + pr.y * CPN.mat[1][0]) * O->RAANdot;

      for (i = 0; i < 3; i++)
         wxr.v[i] +=
             (+pr.x * CPN.mat[1][i] - pr.y * CPN.mat[0][i]) * O->ArgPdot;

      for (i = 0; i < 3; i++)
         O->VelN.v[i] += wxr.v[i];
   }
}
/**********************************************************************/
/* TLEs use UTC.  42 orbits use TT.  So LeapSec are needed.           */
long LoadTleFromFile(const char *Path, const char *TleFileName,
                     const char *TleLabel, double dyntime, JDType jd,
                     struct OrbitType *O)
{
   FILE *infile;
   char line[80], line1[80], line2[80];
   char Label[25];
   long i, Nchar;
   long Success = 0;

   infile = FileOpen(Path, TleFileName, "r");

   Nchar = strlen(TleLabel);
   if (Nchar > 24)
      Nchar = 24;
   /* Pad label to 24 characters to assure unique match */
   for (i = 0; i < Nchar; i++)
      Label[i] = TleLabel[i];
   for (i = Nchar; i < 24; i++)
      Label[i] = ' ';
   Label[24] = '\0';
   while (!feof(infile) && !Success) {
      fgets(line, 80, infile);
      if (!strncmp(line, Label, Nchar)) {
         Success = 1;
         fgets(line1, 80, infile);
         fgets(line2, 80, infile);
         TLE2MeanEph(line1, line2, jd, O);
         MeanEph2RV(O, dyntime);
      }
   }
   fclose(infile);

   return (Success);
}
/**********************************************************************/
/* Periapsis position and velocity might make a useful orbit element  */
/* set for some applications.  This function finds them from the      */
/* given position and velocity.                                       */
double RV2RVp(double mu, vec3_t r, vec3_t v, vec3_t *rp, vec3_t *vp)
{
   double cth, sth;
   double Cer, Cev, Cpr, Cpv, anom;
   magvec3_t uie, uip;
   vec3_t *const ie = &uie.v;
   vec3_t *const ip = &uip.v;
   long i;

   const double magr = MAGV(r);
   const double magv = MAGV(v);

   const double E     = 0.5 * magv * magv - mu / magr;
   const vec3_t rxv   = VxV(r, v);
   const double h     = MAGV(rxv);
   const double p     = h * h / mu;
   const double e     = sqrt(1.0 + 2.0 * E * p / mu);
   const double magrp = p / (1.0 + e);
   const double magvp = h / magrp;
   if (e == 0.0) {
      cth = 1.0;
      sth = 0.0;
   }
   else {
      cth = (p / magr - 1.0) / e;
      sth = sqrt(1.0 - cth * cth);
      if (VoV(r, v) < 0.0)
         sth = -sth;
   }

   Cer = (e + cth) / p;
   Cev = -magr / h * sth;
   Cpr = sth / p;
   Cpv = magr / h * cth;
   for (i = 0; i < 3; i++) {
      ie->v[i] = Cer * r.v[i] + Cev * v.v[i];
      ip->v[i] = Cpr * r.v[i] + Cpv * v.v[i];
   }
   uie = UNITV(*ie);
   uip = UNITV(*ip);
   for (i = 0; i < 3; i++) {
      rp->v[i] = magrp * ie->v[i];
      vp->v[i] = magvp * ip->v[i];
   }
   anom = atan2(sth, cth);

   return (anom);
}
/**********************************************************************/
/*  This function finds the mean orbit of planet "i" with respect to  */
/*  the mean-equinox-of-date frame.  Ref. Chap 31 of Meeus,           */
/*  "Astronomical Algorithms", second edition, QB51.3.E43, M42, 1998. */
/*  Index 1=Mercury, 2=Venus, ... 9=Pluto.  0=Sun is not used.        */
/*  Note that the elements for Pluto are not from Meeus, but from a   */
/*  lower-fidelity data set from JPL.                                 */
void PlanetEphemerides(long i, JDType jd, double mu, double *SMA, double *ecc,
                       double *inc, double *RAAN, double *ArgP, double *tp,
                       double *anom, double *SLR, double *alpha, double *rmin,
                       double *MeanMotion, double *Period)
{

   double La0[10] = {0.0,       252.250906, 181.979801, 100.466457, 355.433,
                     34.351519, 50.077444,  314.055005, 304.348665, 238.92881};
   double La1[10] = {0.0,           149474.0722491, 58519.2130302,
                     36000.7698278, 19141.6964471,  3036.3027748,
                     1223.5110686,  429.8640561,    219.8833092,
                     145.20775};
   double La2[10] = {0.0,       0.0003035,  0.00031014, 0.00030322, 0.00031052,
                     0.0002233, 0.00051908, 0.00030390, 0.00030882, 0.0};
   double La3[10] = {0.0,     18.0E-9, 15.0E-9, 20.0E-9, 16.0E-9,
                     37.0E-9, -3.0E-8, 26.0E-9, 18.0E-9, 0.0};

   double aa0[10] = {0.0,          0.387098310, 0.723329820, 1.000001018,
                     1.523679342,  5.202603209, 9.554909192, 19.218446062,
                     30.110386869, 39.48168677};
   double aa1[10] = {0.0,      0.0,       0.0,      0.0,       0.0,
                     1.913E-7, -2.139E-6, -3.72E-8, -1.663E-7, -7.6912E-4};
   double aa2[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4E-9, 9.8E-10, 6.9E-10, 0.0};

   double ea0[10] = {0.0,        0.20563175, 0.00677192, 0.01670863,
                     0.09340065, 0.04849793, 0.05554814, 0.04638122,
                     0.00945575, 0.24880766};
   double ea1[10] = {0.0,        2.0407E-5,   -4.7765E-5, -4.2037E-5, 9.0484E-5,
                     1.63225E-4, -3.46641E-4, -2.7293E-5, 6.033E-6,   6.465E-5};
   double ea2[10] = {0.0,       -2.83E-8,  9.81E-8, -1.267E-7, -8.06E-8,
                     -4.714E-7, -6.436E-7, 7.89E-8, 0.0,       0.0};
   double ea3[10] = {0.0,      -1.8E-10, 4.6E-10, 1.4E-10,  -2.5E-10,
                     -2.01E-9, 3.4E-9,   2.4E-10, -5.0E-11, 0.0};

   double ia0[10] = {0.0,      7.004986, 3.394662, 0.0,      1.849726,
                     1.303267, 2.488879, 0.773197, 1.769953, 17.14175};
   double ia1[10] = {0.0,        0.0018215,  0.0010037, 0.0,        -0.0006011,
                     -0.0054965, -0.0037362, 0.0007744, -0.0093082, 0.003075};
   double ia2[10] = {0.0,     -1.81E-5,  -8.8E-7,  0.0,      1.276E-5,
                     4.66E-5, -1.519E-5, 3.749E-5, -7.08E-6, 0.0};
   double ia3[10] = {0.0,     5.6E-8, -7.0E-9, 0.0,    -7.0E-9,
                     -2.0E-9, 8.7E-8, -9.2E-8, 2.7E-8, 0.0};

   double Oa0[10] = {0.0,        48.330893,  76.67992,  0.0,        49.558093,
                     100.464407, 113.665503, 74.005957, 131.784057, 110.30347};
   double Oa1[10] = {0.0,       1.1861883, 0.9011206, 0.0,       0.7720959,
                     1.0209774, 0.877088,  0.5211278, 1.1022039, -0.01037};
   double Oa2[10] = {0.0,       1.7542E-4,  4.0618E-4,  0.0,       1.557E-5,
                     4.0315E-4, -1.2176E-4, 1.33947E-3, 2.5952E-4, 0.0};
   double Oa3[10] = {0.0,     2.15E-7,   -9.3E-8,   0.0,      2.267E-6,
                     4.04E-7, -2.249E-6, 1.8484E-5, -6.37E-7, 0.0};

   double pa0[10] = {0.0,       77.456119, 131.563703, 102.937348, 336.060234,
                     14.331207, 93.057237, 173.005291, 48.120276,  224.06676};
   double pa1[10] = {0.0,       1.5564776, 1.4022288, 1.7195366, 1.8410449,
                     1.6126352, 1.9637613, 1.486379,  1.4262957, -0.036736};
   double pa2[10] = {0.0,        2.9544E-4, -1.07618E-3, 4.5688E-4, 1.3477E-4,
                     1.03042E-3, 8.3753E-4, 2.1406E-4,   3.8434E-4, 0.0};
   double pa3[10] = {0.0,       9.0E-9,   -5.678E-6, -1.8E-8, 5.36E-7,
                     -4.464E-6, 4.928E-6, 4.34E-7,   2.0E-8,  0.0};
   double T, L, Pi, M, dt, SecSinceJ2000;

   double AU2m = 149597870000.0;

   jd = JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, jd);

   /* .. Time since J2000, in Julian centuries */
   T = JDToDays(jd) / 36525.0;

   /* .. Time since J2000, in seconds */
   SecSinceJ2000 = JDToDynTime(jd);

   /* .. Mean Longitude */
   L = (La0[i] + T * (La1[i] + T * (La2[i] + T * La3[i]))) * D2R;
   /* .. Semi-major axis */
   *SMA = (aa0[i] + T * (aa1[i] + T * aa2[i])) * AU2m;
   /* .. Eccentricity */
   *ecc = ea0[i] + T * (ea1[i] + T * (ea2[i] + T * ea3[i]));
   /* .. Inclination to ecliptic */
   *inc = (ia0[i] + T * (ia1[i] + T * (ia2[i] + T * ia3[i]))) * D2R;
   /* .. Right ascension of ascending node */
   *RAAN = (Oa0[i] + T * (Oa1[i] + T * (Oa2[i] + T * Oa3[i]))) * D2R;
   /* .. Right ascension of perihelion */
   Pi = (pa0[i] + T * (pa1[i] + T * (pa2[i] + T * pa3[i]))) * D2R;

   /* .. Argument of perihelion */
   *ArgP = Pi - *RAAN;
   /* .. Mean anomaly */
   M = L - Pi;
   /* .. Time since perihelion passage */
   dt = M * sqrt((*SMA) * (*SMA) * (*SMA) / mu);
   /* .. Time of perihelion passage */
   *tp = SecSinceJ2000 - dt;

   /* .. Semilatus rectum */
   *SLR = *SMA * (1.0 - (*ecc) * (*ecc));
   /* .. Alpha is inverse of axis */
   *alpha = 1.0 / (*SMA);
   /* .. Periapsis radius */
   *rmin = *SMA * (1.0 - (*ecc));

   /* .. True anomaly */
   *anom = TrueAnomaly(mu, *SLR, *ecc, dt);

   *MeanMotion = sqrt(mu / (*SMA) / (*SMA) / (*SMA));
   *Period     = TWOPI / (*MeanMotion);
}
/*********************************************************************/
/*  This function gives the location of Luna, with respect to the    */
/*  geocentric ecliptic frame.  Refer to Chap 47 of Meeus,           */
/*  "Astronomical Algorithms" QB51.3.E43 M42, 1998.                  */
vec3_t LunaPosition(const JDType jd)
{
   // dug a bit through Astronomical Algorithmsm,
   // JD is Terrestrial Dynamical Time here...

   JDType jd_tt_j2000 = JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, jd);

   double T, Lp, D, M, Mp, F, A1, A2, A3, E, E2, SumL, SumR, SumB, arg;
   double Lat, Lng, Delta;

   T = JDToDays(jd_tt_j2000) / 36525.0;

   Lp = (218.3164477 +
         T * (481267.88123421 +
              T * (-1.5786E-3 + T * (1.855835024E-6 - T / 65194000.0)))) *
        D2R;
   D  = (297.8501921 +
         T * (445267.1114034 +
              T * (-1.8819E-3 + T * (1.831944719E-6 - T / 113065000.0)))) *
        D2R;
   M  = (357.5291092 + T * (35999.0502909 + T * (-1.536E-4 + T / 24490000.0))) *
        D2R;
   Mp = (134.9633964 +
         T * (477198.8675055 +
              T * (8.7414E-3 + T * (1.434740814E-5 - T / 14712000.0)))) *
        D2R;
   F  = (93.272095 +
         T * (483202.0175233 +
              T * (-3.6539E-3 + T * (-2.836074872E-7 + T / 863310000.0)))) *
        D2R;
   A1 = (119.75 + 131.849 * T) * D2R;
   A2 = (53.09 + 479264.29 * T) * D2R;
   A3 = (313.45 + 481266.484 * T) * D2R;
   E  = 1.0 - 2.516E-3 * T - 7.4E-6 * T * T;
   E2 = E * E;

   SumL = 0.0;
   SumR = 0.0;
   SumB = 0.0;

   arg   = Mp;
   SumL += 6288774.0 * sin(arg);
   SumR += -20905355.0 * cos(arg);

   arg   = 2.0 * D - Mp;
   SumL += 1274027.0 * sin(arg);
   SumR += -3699111.0 * cos(arg);

   arg   = 2.0 * D;
   SumL += 658314.0 * sin(arg);
   SumR += -2955968.0 * cos(arg);

   arg   = 2.0 * Mp;
   SumL += 213618.0 * sin(arg);
   SumR += -569925.0 * cos(arg);

   arg   = M;
   SumL += -185116.0 * E * sin(arg);
   SumR += 48888.0 * E * cos(arg);

   arg   = 2.0 * F;
   SumL += -114332.0 * sin(arg);
   SumR += -3149.0 * cos(arg);

   arg   = 2.0 * (D - Mp);
   SumL += 58793.0 * sin(arg);
   SumR += 246158.0 * cos(arg);

   arg   = 2.0 * D - M - Mp;
   SumL += 57066.0 * E * sin(arg);
   SumR += -152138.0 * E * cos(arg);

   arg   = 2.0 * D + Mp;
   SumL += 53322.0 * sin(arg);
   SumR += -170733.0 * cos(arg);

   arg   = 2.0 * D - M;
   SumL += 45758.0 * E * sin(arg);
   SumR += -204586.0 * E * cos(arg);

   arg   = M - Mp;
   SumL += -40923.0 * E * sin(arg);
   SumR += -129620.0 * E * cos(arg);

   arg   = 2.0 * D;
   SumL += -34720.0 * sin(arg);
   SumR += 108743.0 * cos(arg);

   arg   = M + Mp;
   SumL += -30383.0 * E * sin(arg);
   SumR += 104755.0 * E * cos(arg);

   arg   = 2.0 * (D - F);
   SumL += 15327.0 * sin(arg);
   SumR += 10321.0 * cos(arg);

   arg   = Mp + 2.0 * F;
   SumL += -12528.0 * sin(arg);

   arg   = Mp - 2.0 * F;
   SumL += 10980.0 * sin(arg);
   SumR += 79661.0 * cos(arg);

   arg   = 4.0 * D - Mp;
   SumL += 10675.0 * sin(arg);
   SumR += -34782.0 * cos(arg);

   arg   = 3.0 * Mp;
   SumL += 10034.0 * sin(arg);
   SumR += -23210.0 * cos(arg);

   arg   = 4.0 * D - 2.0 * Mp;
   SumL += 8548.0 * sin(arg);
   SumR += -21636.0 * cos(arg);

   arg   = 2.0 * D + M - Mp;
   SumL += -7888.0 * E * sin(arg);
   SumR += 24208.0 * E * cos(arg);

   arg   = 2.0 * D + M;
   SumL += -6766.0 * E * sin(arg);
   SumR += 30824.0 * E * cos(arg);

   arg   = D - Mp;
   SumL += -5163.0 * sin(arg);
   SumR += -8379.0 * cos(arg);

   arg   = D + M;
   SumL += 4987.0 * E * sin(arg);
   SumR += -16675.0 * E * cos(arg);

   arg   = 2.0 * D - M + Mp;
   SumL += 4036.0 * E * sin(arg);
   SumR += -12831.0 * E * cos(arg);

   arg   = 2.0 * (D + Mp);
   SumL += 3994.0 * sin(arg);
   SumR += -10445.0 * cos(arg);

   arg   = 4.0 * D;
   SumL += 3861.0 * sin(arg);
   SumR += -11650.0 * cos(arg);

   arg   = 2.0 * D - 3.0 * Mp;
   SumL += 3665.0 * sin(arg);
   SumR += 14403.0 * cos(arg);

   arg   = M - 2.0 * Mp;
   SumL += -2689.0 * E * sin(arg);
   SumR += -7003.0 * E * cos(arg);

   arg   = 2.0 * (D + F) - Mp;
   SumL += -2602.0 * sin(arg);

   arg   = 2.0 * (D - Mp) - M;
   SumL += 2390.0 * E * sin(arg);
   SumR += 10056.0 * E * cos(arg);

   arg   = D + Mp;
   SumL += -2348.0 * sin(arg);
   SumR += 6322.0 * cos(arg);

   arg   = 2.0 * (D - M);
   SumL += 2236.0 * E2 * sin(arg);
   SumR += -9884.0 * E2 * cos(arg);

   arg   = M + 2.0 * Mp;
   SumL += -2120.0 * E * sin(arg);
   SumR += 5751.0 * E * cos(arg);

   arg   = 2.0 * M;
   SumL += -2069.0 * E2 * sin(arg);

   arg   = 2.0 * (D - M) - Mp;
   SumL += 2048.0 * E2 * sin(arg);
   SumR += -4950.0 * E2 * cos(arg);

   arg   = 2.0 * (D - F) + Mp;
   SumL += -1773.0 * sin(arg);
   SumR += 4130.0 * cos(arg);

   arg   = 2.0 * (D + F);
   SumL += -1595.0 * sin(arg);

   arg   = 4.0 * D - M - Mp;
   SumL += 1215.0 * E * sin(arg);
   SumR += -3958.0 * E * cos(arg);

   arg   = 2.0 * (Mp + F);
   SumL += -1110.0 * sin(arg);

   arg   = 3.0 * D - Mp;
   SumL += -892.0 * sin(arg);
   SumR += 3258.0 * cos(arg);

   arg   = 2.0 * D + M + Mp;
   SumL += -810.0 * E * sin(arg);
   SumR += 2616.0 * E * cos(arg);

   arg   = 4.0 * D - M - 2.0 * Mp;
   SumL += 759.0 * E * sin(arg);
   SumR += -1897.0 * E * cos(arg);

   arg   = 2.0 * M - Mp;
   SumL += -713.0 * E2 * sin(arg);
   SumR += -2117.0 * E2 * cos(arg);

   arg   = 2.0 * (D + M) - Mp;
   SumL += -700.0 * E2 * sin(arg);
   SumR += 2354.0 * E2 * cos(arg);

   arg   = 2.0 * (D - Mp) + M;
   SumL += 691.0 * E * sin(arg);

   arg   = 2.0 * (D - F) - M;
   SumL += 596.0 * E * sin(arg);

   arg   = 4.0 * D + Mp;
   SumL += 549.0 * sin(arg);
   SumR += -1423.0 * cos(arg);

   arg   = 4.0 * Mp;
   SumL += 537.0 * sin(arg);
   SumR += -1117.0 * cos(arg);

   arg   = 4.0 * D - M;
   SumL += 520.0 * E * sin(arg);
   SumR += -1571.0 * E * cos(arg);

   arg   = D - 2.0 * Mp;
   SumL += -487.0 * sin(arg);
   SumR += -1739.0 * cos(arg);

   arg   = 2.0 * (D - F) + M;
   SumL += -399.0 * E * sin(arg);

   arg   = 2.0 * (Mp - F);
   SumL += -381.0 * sin(arg);
   SumR += -4421.0 * cos(arg);

   arg   = D + M + Mp;
   SumL += 351.0 * E * sin(arg);

   arg   = 3.0 * D - 2.0 * Mp;
   SumL += -340.0 * sin(arg);

   arg   = 4.0 * D - 3.0 * Mp;
   SumL += 330.0 * sin(arg);

   arg   = 2.0 * (D + Mp) - M;
   SumL += 327.0 * E * sin(arg);

   arg   = 2.0 * M + Mp;
   SumL += -323.0 * E2 * sin(arg);
   SumR += 1165.0 * E2 * cos(arg);

   arg   = D + M - Mp;
   SumL += 299.0 * E * sin(arg);

   arg   = 2.0 * D + 3.0 * Mp;
   SumL += 294.0 * sin(arg);

   arg   = 2.0 * (D - F) - Mp;
   SumR += 8752.0 * cos(arg);

   SumB += 5128122.0 * sin(F);
   SumB += 280602.0 * sin(Mp + F);
   SumB += 277693.0 * sin(Mp - F);
   SumB += 173237.0 * sin(2.0 * D - F);
   SumB += 55413.0 * sin(2.0 * D - Mp + F);
   SumB += 46271.0 * sin(2.0 * D - Mp - F);
   SumB += 32573.0 * sin(2.0 * D + F);
   SumB += 17198.0 * sin(2.0 * Mp + F);
   SumB += 9266.0 * sin(2.0 * D + Mp - F);
   SumB += 8822.0 * sin(2.0 * Mp - F);
   SumB += 8216.0 * E * sin(2.0 * D - M - F);
   SumB += 4324.0 * sin(2.0 * (D - Mp) - F);
   SumB += 4200.0 * sin(2.0 * D + Mp + F);
   SumB += -3359.0 * E * sin(2.0 * D + M - F);
   SumB += 2463.0 * E * sin(2.0 * D - M - Mp + F);
   SumB += 2211.0 * E * sin(2.0 * D - M + F);
   SumB += 2065.0 * E * sin(2.0 * D - M - Mp - F);
   SumB += -1870.0 * E * sin(M - Mp - F);
   SumB += 1828.0 * sin(4.0 * D - Mp - F);
   SumB += -1794.0 * E * sin(M + F);
   SumB += -1749.0 * sin(3.0 * F);
   SumB += -1565.0 * E * sin(M - Mp + F);
   SumB += -1491.0 * sin(D + F);
   SumB += -1475.0 * E * sin(M + Mp + F);
   SumB += -1410.0 * E * sin(M + Mp - F);
   SumB += -1344.0 * E * sin(M - F);
   SumB += -1335.0 * sin(D - F);
   SumB += 1107.0 * sin(3.0 * Mp + F);
   SumB += 1021.0 * sin(4.0 * D - F);
   SumB += 833.0 * sin(4.0 * D - Mp + F);
   SumB += 777.0 * sin(Mp - 3.0 * F);
   SumB += 671.0 * sin(4.0 * D - 2.0 * Mp + F);
   SumB += 607.0 * sin(2.0 * D - 3.0 * F);
   SumB += 596.0 * sin(2.0 * (D + Mp) - F);
   SumB += 491.0 * E * sin(2.0 * D - M + Mp - F);
   SumB += -451.0 * sin(2.0 * (D - Mp) + F);
   SumB += 439.0 * sin(3.0 * Mp - F);
   SumB += 422.0 * sin(2.0 * (D + Mp) + F);
   SumB += 421.0 * sin(2.0 * D - 3.0 * Mp - F);
   SumB += -366.0 * E * sin(2.0 * D + M - Mp + F);
   SumB += -351.0 * E * sin(2.0 * D + M + F);
   SumB += 331.0 * sin(4.0 * D + F);
   SumB += 315.0 * E * sin(2.0 * D - M + Mp + F);
   SumB += 302.0 * E2 * sin(2.0 * (D - M) - F);
   SumB += -283.0 * sin(Mp + 3.0 * F);
   SumB += -229.0 * E * sin(2.0 * D + M + Mp - F);
   SumB += 223.0 * E * (sin(D + M - F) + sin(D + M + F));
   SumB += -220.0 * E * (sin(M - 2.0 * Mp - F) + sin(2.0 * D + M - Mp - F));
   SumB += -185.0 * sin(D + Mp + F);
   SumB += 181.0 * E * sin(2.0 * (D - Mp) - M - F);
   SumB += -177.0 * E * sin(M + 2.0 * Mp + F);
   SumB += 176.0 * sin(4.0 * D - 2.0 * Mp - F);
   SumB += 166.0 * E * sin(4.0 * D - M - Mp - F);
   SumB += -164.0 * sin(D + Mp - F);
   SumB += 132.0 * sin(4.0 * D + Mp - F);
   SumB += -119.0 * sin(D - Mp - F);
   SumB += 115.0 * E * sin(4.0 * D - M - F);
   SumB += 107.0 * E2 * sin(2.0 * (D - M) + F);

   SumL += 3958.0 * sin(A1) + 1962.0 * sin(Lp - F) + 318.0 * sin(A2);
   SumB += -2235.0 * sin(Lp) + 382.0 * sin(A3) + 175.0 * sin(A1 - F) +
           175.0 * sin(A1 + F) + 127.0 * sin(Lp - Mp) - 115.0 * sin(Lp + Mp);

   Lng   = Lp + 1.0E-6 * SumL * D2R;
   Lat   = 1.0E-6 * SumB * D2R;
   Delta = 385000.56E3 + SumR;

   vec3_t r;
   r.x = Delta * cos(Lng) * cos(Lat);
   r.y = Delta * sin(Lng) * cos(Lat);
   r.z = Delta * sin(Lat);
   return r;
}
/**********************************************************************/
/*  Ref JPL D-32296, "Lunar Constants and Models Document"            */
/*  http://ssd.jpl.nasa.gov/?lunar_doc                                */
int LoadLunarNutPrecAngle(int *n_E, double (**nut_prec_E)[2])
{
   const double nut_prec_ang_data[26] = {
       125.045, -0.0529921, 250.089, -0.1059842, 260.008, 13.0120009,
       176.625, 13.3407154, 357.529, 0.9856003,  311.589, 26.4057084,
       134.963, 13.0649930, 276.617, 0.3287146,  34.226,  1.7484877,
       15.134,  -0.1589763, 119.743, 0.0036096,  239.961, 0.1643573,
       25.053,  12.9590088};
   *n_E        = 13;
   *nut_prec_E = calloc(*n_E, sizeof(double[2]));

   for (int i = 0; i < *n_E; i++) {
      for (int j = 0; j < 2; j++) {
         (*nut_prec_E)[i][j] = nut_prec_ang_data[j + 2 * i];
      }
      (*nut_prec_E)[i][1] *= JDDAY_PER_CENTURY;
   }
   return 1;
}
/**********************************************************************/
/*  Ref JPL D-32296, "Lunar Constants and Models Document"            */
/*  http://ssd.jpl.nasa.gov/?lunar_doc                                */
/*  Finds Lunar Inertial Frame wrt J2000                              */
int LoadLunaInertialFrameData(AngDataType *const ang_data)
{
   const double ra_dat[3]  = {269.9949, 0.0031, 0.0};
   const double dec_dat[3] = {66.5392, 0.0130, 0.0};

   const double nut_prec_ra[13]  = {-3.8787, -0.1204, 0.0700, -0.0172, 0.0,
                                    0.0072,  0.0,     0.0,    0.0,     -0.0052,
                                    0.0,     0.0,     0.0043};
   const double nut_prec_dec[13] = {1.5419,  0.0239, -0.0278, 0.0068, 0.0,
                                    -0.0029, 0.0009, 0.0,     0.0,    0.0008,
                                    0.0,     0.0,    -0.0009};

   AngDataType *const ra_data  = &ang_data[1];
   AngDataType *const dec_data = &ang_data[2];

   ra_data->ang_char  = 'R';
   dec_data->ang_char = 'D';

   CopyVG(ra_data->ang, ra_dat, 3);
   CopyVG(dec_data->ang, dec_dat, 3);

   LoadLunarNutPrecAngle(&ra_data->n_E, &ra_data->nut_prec_E);
   LoadLunarNutPrecAngle(&dec_data->n_E, &dec_data->nut_prec_E);

   ra_data->n_ang         = 13;
   dec_data->n_ang        = 13;
   ra_data->nut_prec_ang  = calloc(ra_data->n_ang, sizeof(double));
   dec_data->nut_prec_ang = calloc(dec_data->n_ang, sizeof(double));
   CopyVG(ra_data->nut_prec_ang, nut_prec_ra, ra_data->n_ang);
   CopyVG(dec_data->nut_prec_ang, nut_prec_dec, dec_data->n_ang);
   return 1;
}
/**********************************************************************/
/*  Ref JPL D-32296, "Lunar Constants and Models Document"            */
/*  http://ssd.jpl.nasa.gov/?lunar_doc                                */
/*  Finds Lunar Inertial Frame wrt J2000                              */
mat3x3_t LunaInertialFrame(const JDType jd)
{
   JDType jd_tdb_j2000 = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);

   double D, T;
   double E1, E2, E3, E4, E6, E7, E10, E13;
   /* double E12; */
   double SinE1, SinE2, SinE3, SinE4, SinE6;
   double SinE10, SinE13;
   double CosE1, CosE2, CosE3, CosE4, CosE6, CosE7, CosE10, CosE13;
   double PoleRA, PoleDec;
   vec3_t PoleVec;
   magvec3_t uNodeVec, uYVec;
   vec3_t *const NodeVec = &uNodeVec.v;
   vec3_t *const YVec    = &uYVec.v;

   D = JDToDays(jd_tdb_j2000);
   T = D / 36525.0;

   E1  = WrapDeg(125.045 - 0.0529921 * D) * D2R;
   E2  = WrapDeg(250.089 - 0.1059842 * D) * D2R;
   E3  = WrapDeg(260.008 + 13.0120009 * D) * D2R;
   E4  = WrapDeg(176.625 + 13.3407154 * D) * D2R;
   E6  = WrapDeg(311.589 + 26.4057084 * D) * D2R;
   E7  = WrapDeg(134.963 + 13.0649930 * D) * D2R;
   E10 = WrapDeg(15.134 - 0.1589763 * D) * D2R;
   /* E12 = WrapDeg(239.961 + 0.1643573*D)*D2R; */
   E13 = WrapDeg(25.053 + 12.9590088 * D) * D2R;

   SinE1  = sin(E1);
   SinE2  = sin(E2);
   SinE3  = sin(E3);
   SinE4  = sin(E4);
   SinE6  = sin(E6);
   SinE10 = sin(E10);
   SinE13 = sin(E13);

   CosE1  = cos(E1);
   CosE2  = cos(E2);
   CosE3  = cos(E3);
   CosE4  = cos(E4);
   CosE6  = cos(E6);
   CosE7  = cos(E7);
   CosE10 = cos(E10);
   CosE13 = cos(E13);

   PoleRA = 269.9949 + 0.0031 * T - 3.8787 * SinE1 - 0.1204 * SinE2 +
            0.0700 * SinE3 - 0.0172 * SinE4 + 0.0072 * SinE6 - 0.0052 * SinE10 +
            0.0043 * SinE13;

   PoleDec = 66.5392 + 0.0130 * T + 1.5419 * CosE1 + 0.0239 * CosE2 -
             0.0278 * CosE3 + 0.0068 * CosE4 - 0.0029 * CosE6 + 0.0009 * CosE7 +
             0.0008 * CosE10 - 0.0009 * CosE13;

   PoleRA  *= D2R;
   PoleDec *= D2R;

   /* Derive Pole Vector in J2000 */
   PoleVec.x = cos(PoleRA) * cos(PoleDec);
   PoleVec.y = sin(PoleRA) * cos(PoleDec);
   PoleVec.z = sin(PoleDec);

   /* IAU convention puts the X axis at Z(J2000) x PoleVec */
   NodeVec->x = -PoleVec.y;
   NodeVec->y = PoleVec.x;
   NodeVec->z = 0.0;
   uNodeVec   = UNITV(*NodeVec);

   *YVec = VxV(PoleVec, *NodeVec);
   uYVec = UNITV(*YVec);

   /* Luna's N frame wrt J2000 */
   mat3x3_t out = {.rows = {*NodeVec, *YVec, PoleVec}};
   return out;
}
/**********************************************************************/
/*  Ref JPL D-32296, "Lunar Constants and Models Document"            */
/*  http://ssd.jpl.nasa.gov/?lunar_doc                                */
int LoadLunaPriMerAngData(AngDataType *const ang_data)
{
   const double pm_dat[3]        = {38.3213, 13.17635815, -1.4E-12};
   const double nut_prec_dat[13] = {3.5610,  0.1208,  -0.0642, 0.0158, 0.0252,
                                    -0.0066, -0.0047, -0.0046, 0.0028, 0.0052,
                                    0.0040,  0.0019,  -0.0044};

   AngDataType *const pm_data = &ang_data[0];
   pm_data->ang_char          = 'P';
   CopyVG(pm_data->ang, pm_dat, 3);
   LoadLunarNutPrecAngle(&pm_data->n_E, &pm_data->nut_prec_E);
   pm_data->n_ang        = 13;
   pm_data->nut_prec_ang = calloc(pm_data->n_ang, sizeof(double));
   CopyVG(pm_data->nut_prec_ang, nut_prec_dat, pm_data->n_ang);
   return 1;
}
/**********************************************************************/
/*  Ref JPL D-32296, "Lunar Constants and Models Document"            */
/*  http://ssd.jpl.nasa.gov/?lunar_doc                                */
double LunaPriMerAng(const JDType jd)
{
   double D;
   double E1, E2, E3, E4, E5, E6, E7, E8, E9, E10, E11, E12, E13;
   double SinE1, SinE2, SinE3, SinE4, SinE5, SinE6, SinE7;
   double SinE8, SinE9, SinE10, SinE11, SinE12, SinE13;
   double PriMerAng;

   JDType jd_tdb_j2000 = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);

   D = JDToDays(jd_tdb_j2000);

   E1  = WrapDeg(125.045 - 0.0529921 * D) * D2R;
   E2  = WrapDeg(250.089 - 0.1059842 * D) * D2R;
   E3  = WrapDeg(260.008 + 13.0120009 * D) * D2R;
   E4  = WrapDeg(176.625 + 13.3407154 * D) * D2R;
   E5  = WrapDeg(357.529 + 0.9856003 * D) * D2R;
   E6  = WrapDeg(311.589 + 26.4057084 * D) * D2R;
   E7  = WrapDeg(134.963 + 13.0649930 * D) * D2R;
   E8  = WrapDeg(276.617 + 0.3287146 * D) * D2R;
   E9  = WrapDeg(34.226 + 1.7484877 * D) * D2R;
   E10 = WrapDeg(15.134 - 0.1589763 * D) * D2R;
   E11 = WrapDeg(119.743 + 0.0036096 * D) * D2R;
   E12 = WrapDeg(239.961 + 0.1643573 * D) * D2R;
   E13 = WrapDeg(25.053 + 12.9590088 * D) * D2R;

   SinE1  = sin(E1);
   SinE2  = sin(E2);
   SinE3  = sin(E3);
   SinE4  = sin(E4);
   SinE5  = sin(E5);
   SinE6  = sin(E6);
   SinE7  = sin(E7);
   SinE8  = sin(E8);
   SinE9  = sin(E9);
   SinE10 = sin(E10);
   SinE11 = sin(E11);
   SinE12 = sin(E12);
   SinE13 = sin(E13);

   PriMerAng = 38.3213 + 13.17635815 * D - 1.4E-12 * D * D + 3.5610 * SinE1 +
               0.1208 * SinE2 - 0.0642 * SinE3 + 0.0158 * SinE4 +
               0.0252 * SinE5 - 0.0066 * SinE6 - 0.0047 * SinE7 -
               0.0046 * SinE8 + 0.0028 * SinE9 + 0.0052 * SinE10 +
               0.0040 * SinE11 + 0.0019 * SinE12 - 0.0044 * SinE13;

   return (PriMerAng * D2R);
}
/**********************************************************************/
void FindCLN(vec3_t r, vec3_t v, mat3x3_t *CLN, vec3_t *wln)
{
   vec3_t L1, L2, L3, h;
   double rr, hh;

   h  = VxV(r, v);
   rr = VoV(r, r);
   hh = VoV(h, h);

   *wln = SxV(1.0 / rr, h);
   L3   = NegV_Elem(r);
   L2   = NegV_Elem(h);

   L3 = SxV(1.0 / sqrt(rr), L3);

   if (hh == 0.0) { /* Rectlinear Motion */
      pair_vec3_t pair = PerpBasis(L3);
      L1               = pair.first;
      L2               = pair.second;
   }
   else {
      magvec3_t mv = UNITV(L2);
      L2           = mv.v;

      L1 = VxV(L2, L3);

      mv = UNITV(L1);
      L1 = mv.v;
   }

   CLN->rows[0] = L1;
   CLN->rows[1] = L2;
   CLN->rows[2] = L3;
}
/**********************************************************************/
/* E = Equatorial Frame.  e1 = n3, e2 = East, e3 points to axis of World */
mat3x3_t FindCEN(vec3_t r)
{
   mat3x3_t CEN  = MAT3X3_ZERO;
   CEN.rows[0]   = VEC3_PZAXIS;
   CEN.mat[2][0] = -r.v[0];
   CEN.mat[2][1] = -r.v[1];
   magvec3_t uv  = UNITV(CEN.rows[2]);
   CEN.rows[2]   = uv.v;
   CEN.rows[1]   = VxV(CEN.rows[2], CEN.rows[0]);
   return CEN;
}
/**********************************************************************/
void FindENU(vec3_t PosN, double WorldW, mat3x3_t *CLN, vec3_t *wln)
{
   vec3_t Zaxis = VEC3_PZAXIS;
   vec3_t Up;
   magvec3_t uEast, uNorth;
   vec3_t *const East  = &uEast.v;
   vec3_t *const North = &uNorth.v;

   Up     = UNITV(PosN).v;
   *East  = VxV(Zaxis, Up);
   uEast  = UNITV(*East);
   *North = VxV(Up, *East);
   uNorth = UNITV(*North);

   CLN->rows[0] = *East;
   CLN->rows[1] = *North;
   CLN->rows[2] = Up;

   wln->v[0] = 0.0;
   wln->v[1] = 0.0;
   wln->v[2] = WorldW;
}
/**********************************************************************/
static double _lagpointFDF(const double x, double params[3])
    __attribute__((pure));
static double _lagpointFDF(const double x, double params[3])
{
   double rho = params[0], rho1 = params[1];
   double xp  = x - params[0];
   double xp1 = xp + 1.0;
   long lp    = params[2];
   switch (lp) {
      case 1: {
         const double f = x + rho1 / (xp * xp) - rho / (xp1 * xp1);
         const double fp =
             1.0 - 2.0 * rho1 / (xp * xp * xp) + 2.0 * rho / (xp1 * xp1 * xp1);
         return f / fp;
      }
      case 2: {
         const double f = x + rho1 / (xp * xp) + rho / (xp1 * xp1);
         const double fp =
             1.0 - 2.0 * rho1 / (xp * xp * xp) - 2.0 * rho / (xp1 * xp1 * xp1);
         return f / fp;
      }
      case 3: {
         const double f = x - rho1 / (xp * xp) - rho / (xp1 * xp1);
         const double fp =
             1.0 + 2.0 * rho1 / (xp * xp * xp) + 2.0 * rho / (xp1 * xp1 * xp1);
         return f / fp;
      }
   }
   return 0;
}
/**********************************************************************/
LagrangeSystem LagSysFromPair(const WorldID pair[2])
{
   // returns LagrangeSystem associated with unordered pair of WorldIDs
#define X(lagsys, body1, body2, lu)                                            \
   if ((body1 == pair[0] && body2 == pair[1]) ||                               \
       (body2 == pair[0] && body1 == pair[1]))                                 \
      return lagsys;
   X_LAGSYS_LIST
#undef X
   fprintf(stderr,
           "WorldID pair for worlds %s && %s is not associated with "
           "any known LagrangeSystem. Exiting...\n",
           WorldID2Name(pair[0]), WorldID2Name(pair[1]));
   exit(EXIT_FAILURE);
}
/**********************************************************************/
void ConfigureLagSys(const LagrangeSystem lagsys_id,
                     struct LagrangeSystemType *lag_sys)
{
   switch (lagsys_id) {
#define X(lagsys, body1, body2, lu)                                            \
   case lagsys:                                                                \
      lag_sys->Body1 = body1;                                                  \
      lag_sys->Body2 = body2;                                                  \
      lag_sys->LU    = lu;                                                     \
      break;
      X_LAGSYS_LIST
#undef X
      default: {
         fprintf(stderr,
                 "Unknown LagrangeSystem %u in ConfigureLagSys. Exiting...\n",
                 lagsys_id);
         exit(EXIT_FAILURE);
      }
   }
}
/**********************************************************************/
/*  Consider the Circular Restricted Three-Body Problem, with two     */
/*  massive bodies (masses m1 and m2, m2 < m1) and a body of          */
/*  negligible mass.  The locations of the Lagrange points are        */
/*  functions of the mass ratio rho = m2/(m1+m2).                     */
/*                                                                    */
/*  Reference Bong Wie, "Space Vehicle Dynamics and Control"          */
/*  (TL1050.W52)Sec 3.7.3                                             */
/*  Also see LagModes.pdf for dimensioned derivations.                */
void FindLagPtParms(struct LagrangeSystemType *LS)
{
   struct LagrangePointType *LP;
   double rho, x, rho1;
   double eps = 2.0E-16;
   double n, D, X0rD, X0r1D, MuSum, a, b, c, s2;
   double R13, R15;
   double R23, R25;
   double alpha;

   rho                = LS->rho;
   rho1               = 1.0 - rho;
   double lpParams[3] = {rho, rho1, 1};
   MuSum              = LS->mu1 + LS->mu2;
   n                  = LS->MeanRate;
   D                  = LS->SMA;

   /* .. L1 */
   LP     = &LS->LP[0];
   x      = NewtonRaphson(-1.0, eps, 200, 100.0, 0, &_lagpointFDF, lpParams);
   LP->X0 = x * D;
   LP->Y0 = 0.0;

   X0rD  = LP->X0 - rho * D;
   X0r1D = LP->X0 + rho1 * D;

   LP->R1 = sqrt(X0rD * X0rD + LP->Y0 * LP->Y0);
   LP->R2 = sqrt(X0r1D * X0r1D + LP->Y0 * LP->Y0);
   R13    = 1.0 / (LP->R1 * LP->R1 * LP->R1);
   R15    = R13 / (LP->R1 * LP->R1);
   R23    = 1.0 / (LP->R2 * LP->R2 * LP->R2);
   R25    = R23 / (LP->R2 * LP->R2);

   LP->Kxx = MuSum * (rho1 * (R13 - 3.0 * R15 * X0rD * X0rD) +
                      rho * (R23 - 3.0 * R25 * X0r1D * X0r1D));
   LP->Kxy = 0.0;
   LP->Kyy = MuSum * (rho1 * (R13 - 3.0 * R15 * LP->Y0 * LP->Y0) +
                      rho * (R23 - 3.0 * R25 * LP->Y0 * LP->Y0));

   a         = 1.0;
   b         = LP->Kxx + LP->Kyy + 2.0 * n * n;
   c         = (LP->Kxx - n * n) * (LP->Kyy - n * n) - LP->Kxy * LP->Kxy;
   s2        = (-b - sqrt(b * b - 4 * a * c)) / (2.0 * a);
   LP->w1    = sqrt(-s2);
   LP->w2    = 0.0;
   s2        = (-b + sqrt(b * b - 4 * a * c)) / (2.0 * a);
   LP->sigma = sqrt(s2);
   LP->wz    = sqrt(MuSum * (rho1 * R13 + rho * R23));

   LP->Zw1 = LP->Kyy - n * n - LP->w1 * LP->w1;
   LP->Zw2 = 0.0;
   LP->Zs  = LP->Kyy - n * n - LP->sigma * LP->sigma;

   LP->ca1 = 1.0;
   LP->sa1 = 0.0;
   LP->ca2 = 1.0;
   LP->sa2 = 0.0;

   LP->AR1 = -(LP->Zw1) / (2.0 * n * LP->w1);
   LP->AR2 = 0.0;

   /* .. L2 */
   LP          = &LS->LP[1];
   lpParams[2] = 2;
   x      = NewtonRaphson(-1.0, eps, 200, 100.0, 0, &_lagpointFDF, lpParams);
   LP->X0 = x * D;
   LP->Y0 = 0.0;

   X0rD  = LP->X0 - rho * D;
   X0r1D = LP->X0 + rho1 * D;

   LP->R1 = sqrt(X0rD * X0rD + LP->Y0 * LP->Y0);
   LP->R2 = sqrt(X0r1D * X0r1D + LP->Y0 * LP->Y0);
   R13    = 1.0 / (LP->R1 * LP->R1 * LP->R1);
   R15    = R13 / (LP->R1 * LP->R1);
   R23    = 1.0 / (LP->R2 * LP->R2 * LP->R2);
   R25    = R23 / (LP->R2 * LP->R2);

   LP->Kxx = MuSum * (rho1 * (R13 - 3.0 * R15 * X0rD * X0rD) +
                      rho * (R23 - 3.0 * R25 * X0r1D * X0r1D));
   LP->Kxy = 0.0;
   LP->Kyy = MuSum * (rho1 * (R13 - 3.0 * R15 * LP->Y0 * LP->Y0) +
                      rho * (R23 - 3.0 * R25 * LP->Y0 * LP->Y0));

   a         = 1.0;
   b         = LP->Kxx + LP->Kyy + 2.0 * n * n;
   c         = (LP->Kxx - n * n) * (LP->Kyy - n * n) - LP->Kxy * LP->Kxy;
   s2        = (-b - sqrt(b * b - 4 * a * c)) / (2.0 * a);
   LP->w1    = sqrt(-s2);
   LP->w2    = 0.0;
   s2        = (-b + sqrt(b * b - 4 * a * c)) / (2.0 * a);
   LP->sigma = sqrt(s2);
   LP->wz    = sqrt(MuSum * (rho1 * R13 + rho * R23));

   LP->Zw1 = LP->Kyy - n * n - LP->w1 * LP->w1;
   LP->Zw2 = 0.0;
   LP->Zs  = LP->Kyy - n * n - LP->sigma * LP->sigma;

   LP->ca1 = 1.0;
   LP->sa1 = 0.0;
   LP->ca2 = 1.0;
   LP->sa2 = 0.0;

   LP->AR1 = -(LP->Zw1) / (2.0 * n * LP->w1);
   LP->AR2 = 0.0;

   /* .. L3 */
   LP          = &LS->LP[2];
   lpParams[2] = 3;
   x      = NewtonRaphson(1.0, eps, 200, 100.0, 0, &_lagpointFDF, lpParams);
   LP->X0 = x * D;
   LP->Y0 = 0.0;

   X0rD  = LP->X0 - rho * D;
   X0r1D = LP->X0 + rho1 * D;

   LP->R1 = sqrt(X0rD * X0rD + LP->Y0 * LP->Y0);
   LP->R2 = sqrt(X0r1D * X0r1D + LP->Y0 * LP->Y0);
   R13    = 1.0 / (LP->R1 * LP->R1 * LP->R1);
   R15    = R13 / (LP->R1 * LP->R1);
   R23    = 1.0 / (LP->R2 * LP->R2 * LP->R2);
   R25    = R23 / (LP->R2 * LP->R2);

   LP->Kxx = MuSum * (rho1 * (R13 - 3.0 * R15 * X0rD * X0rD) +
                      rho * (R23 - 3.0 * R25 * X0r1D * X0r1D));
   LP->Kxy = 0.0;
   LP->Kyy = MuSum * (rho1 * (R13 - 3.0 * R15 * LP->Y0 * LP->Y0) +
                      rho * (R23 - 3.0 * R25 * LP->Y0 * LP->Y0));

   a         = 1.0;
   b         = LP->Kxx + LP->Kyy + 2.0 * n * n;
   c         = (LP->Kxx - n * n) * (LP->Kyy - n * n) - LP->Kxy * LP->Kxy;
   s2        = (-b - sqrt(b * b - 4 * a * c)) / (2.0 * a);
   LP->w1    = sqrt(-s2);
   LP->w2    = 0.0;
   s2        = (-b + sqrt(b * b - 4 * a * c)) / (2.0 * a);
   LP->sigma = sqrt(s2);
   LP->wz    = sqrt(MuSum * (rho1 * R13 + rho * R23));

   LP->Zw1 = LP->Kyy - n * n - LP->w1 * LP->w1;
   LP->Zw2 = 0.0;
   LP->Zs  = LP->Kyy - n * n - LP->sigma * LP->sigma;

   LP->ca1 = 1.0;
   LP->sa1 = 0.0;
   LP->ca2 = 1.0;
   LP->sa2 = 0.0;

   LP->AR1 = -(LP->Zw1) / (2.0 * n * LP->w1);
   LP->AR2 = 0.0;

   /* .. L4 */
   LP     = &LS->LP[3];
   LP->X0 = -(0.5 - rho) * D;
   LP->Y0 = 0.5 * sqrt(3.0) * D;

   X0rD  = LP->X0 - rho * D;
   X0r1D = LP->X0 + rho1 * D;

   LP->R1 = sqrt(X0rD * X0rD + LP->Y0 * LP->Y0);
   LP->R2 = sqrt(X0r1D * X0r1D + LP->Y0 * LP->Y0);
   R13    = 1.0 / (LP->R1 * LP->R1 * LP->R1);
   R15    = R13 / (LP->R1 * LP->R1);
   R23    = 1.0 / (LP->R2 * LP->R2 * LP->R2);
   R25    = R23 / (LP->R2 * LP->R2);

   LP->Kxx = MuSum * (rho1 * (R13 - 3.0 * R15 * X0rD * X0rD) +
                      rho * (R23 - 3.0 * R25 * X0r1D * X0r1D));
   LP->Kxy = 3.0 * MuSum * LP->Y0 * (rho1 * R15 * X0rD + rho * R25 * X0r1D);
   LP->Kyy = MuSum * (rho1 * (R13 - 3.0 * R15 * LP->Y0 * LP->Y0) +
                      rho * (R23 - 3.0 * R25 * LP->Y0 * LP->Y0));

   a         = 1.0;
   b         = LP->Kxx + LP->Kyy + 2.0 * n * n;
   c         = (LP->Kxx - n * n) * (LP->Kyy - n * n) - LP->Kxy * LP->Kxy;
   s2        = (-b - sqrt(b * b - 4 * a * c)) / (2.0 * a);
   LP->w1    = sqrt(-s2);
   s2        = (-b + sqrt(b * b - 4 * a * c)) / (2.0 * a);
   LP->w2    = sqrt(-s2);
   LP->sigma = 0.0;
   LP->wz    = sqrt(MuSum * (rho1 * R13 + rho * R23));

   LP->Zw1 = LP->Kyy - n * n - LP->w1 * LP->w1;
   LP->Zw2 = LP->Kyy - n * n - LP->w2 * LP->w2;
   LP->Zs  = 0.0;

   alpha   = 0.5 * atan2(-2.0 * LP->Kxy * LP->Zw1,
                         -(LP->Zw1 * LP->Zw1 - LP->Kxy * LP->Kxy -
                           4.0 * n * n * LP->w1 * LP->w1));
   LP->ca1 = cos(alpha);
   LP->sa1 = sin(alpha);
   alpha   = 0.5 * atan2(-2.0 * LP->Kxy * LP->Zw2,
                         -(LP->Zw2 * LP->Zw2 - LP->Kxy * LP->Kxy -
                           4.0 * n * n * LP->w2 * LP->w2));
   LP->ca2 = cos(alpha);
   LP->sa2 = sin(alpha);

   LP->AR1 =
       -(LP->Zw1 * LP->ca1 + LP->Kxy * LP->sa1) / (2.0 * n * LP->w1 * LP->ca1);
   LP->AR2 =
       -(LP->Zw2 * LP->ca2 + LP->Kxy * LP->sa2) / (2.0 * n * LP->w2 * LP->ca2);

   /* .. L5 */
   LP     = &LS->LP[4];
   LP->X0 = -(0.5 - rho) * D;
   LP->Y0 = -0.5 * sqrt(3.0) * D;

   X0rD  = LP->X0 - rho * D;
   X0r1D = LP->X0 + rho1 * D;

   LP->R1 = sqrt(X0rD * X0rD + LP->Y0 * LP->Y0);
   LP->R2 = sqrt(X0r1D * X0r1D + LP->Y0 * LP->Y0);
   R13    = 1.0 / (LP->R1 * LP->R1 * LP->R1);
   R15    = R13 / (LP->R1 * LP->R1);
   R23    = 1.0 / (LP->R2 * LP->R2 * LP->R2);
   R25    = R23 / (LP->R2 * LP->R2);

   LP->Kxx = MuSum * (rho1 * (R13 - 3.0 * R15 * X0rD * X0rD) +
                      rho * (R23 - 3.0 * R25 * X0r1D * X0r1D));
   LP->Kxy = 3.0 * MuSum * LP->Y0 * (rho1 * R15 * X0rD + rho * R25 * X0r1D);
   LP->Kyy = MuSum * (rho1 * (R13 - 3.0 * R15 * LP->Y0 * LP->Y0) +
                      rho * (R23 - 3.0 * R25 * LP->Y0 * LP->Y0));

   a         = 1.0;
   b         = LP->Kxx + LP->Kyy + 2.0 * n * n;
   c         = (LP->Kxx - n * n) * (LP->Kyy - n * n) - LP->Kxy * LP->Kxy;
   s2        = (-b - sqrt(b * b - 4 * a * c)) / (2.0 * a);
   LP->w1    = sqrt(-s2);
   s2        = (-b + sqrt(b * b - 4 * a * c)) / (2.0 * a);
   LP->w2    = sqrt(-s2);
   LP->sigma = 0.0;
   LP->wz    = sqrt(MuSum * (rho1 * R13 + rho * R23));

   LP->Zw1 = LP->Kyy - n * n - LP->w1 * LP->w1;
   LP->Zw2 = LP->Kyy - n * n - LP->w2 * LP->w2;
   LP->Zs  = 0.0;

   alpha   = 0.5 * atan2(-2.0 * LP->Kxy * LP->Zw1,
                         -(LP->Zw1 * LP->Zw1 - LP->Kxy * LP->Kxy -
                           4.0 * n * n * LP->w1 * LP->w1));
   LP->ca1 = cos(alpha);
   LP->sa1 = sin(alpha);
   alpha   = 0.5 * atan2(-2.0 * LP->Kxy * LP->Zw2,
                         -(LP->Zw2 * LP->Zw2 - LP->Kxy * LP->Kxy -
                           4.0 * n * n * LP->w2 * LP->w2));
   LP->ca2 = cos(alpha);
   LP->sa2 = sin(alpha);

   LP->AR1 =
       -(LP->Zw1 * LP->ca1 + LP->Kxy * LP->sa1) / (2.0 * n * LP->w1 * LP->ca1);
   LP->AR2 =
       -(LP->Zw2 * LP->ca2 + LP->Kxy * LP->sa2) / (2.0 * n * LP->w2 * LP->ca2);
}
/**********************************************************************/
/*  Find the instantaneous locations of Lagrange Points, assuming the */
/*  elliptic restricted three-body problem.                           */
/*  Reference Bong Wie, "Space Vehicle Dynamics and Control"          */
/*  (TL1050.W52)Sec 3.7.3                                             */
void FindLagPtPosVel(double SecSinceJ2000, struct LagrangeSystemType *S,
                     long Ilp, vec3_t *PosN, vec3_t *VelN, mat3x3_t *CLN)
{

   double OnePlusEcosTH, magr, OneMinusE2;
   vec3_t R2, V2, rp, rhat, thhat, vp;
   magvec3_t uL1, uL2, uL3;
   vec3_t *const L1 = &uL1.v;
   vec3_t *const L2 = &uL2.v;
   vec3_t *const L3 = &uL3.v;
   double sth, cth;
   long i;

   Eph2RV(S->mu1, S->SLR, S->ecc, S->inc, S->RAAN, S->ArgP,
          SecSinceJ2000 - S->tp, &R2, &V2, &S->th);

   sth = sin(S->th);
   cth = cos(S->th);

   /* Find D, Ddot, Ddotdot, thdot, thdotdot */
   OnePlusEcosTH = 1.0 + S->ecc * cth;
   OneMinusE2    = 1.0 - S->ecc * S->ecc;
   S->D          = OneMinusE2 / OnePlusEcosTH;
   S->Ddot       = S->ecc / sqrt(OneMinusE2) * sth;
   S->Ddotdot    = S->ecc / OneMinusE2 * cth * OnePlusEcosTH;
   S->thdot      = OnePlusEcosTH * OnePlusEcosTH / sqrt(OneMinusE2);
   S->thdotdot =
       -2.0 * S->ecc * sth / OneMinusE2 * OnePlusEcosTH * OnePlusEcosTH;

   /* L is rotating ("synodic") frame */
   /* L1 points from Body 2 to Body 1 */
   /* L2 is in plane of rotation */
   for (i = 0; i < 3; i++) {
      L1->v[i] = -R2.v[i];
      L2->v[i] = -V2.v[i];
   }
   uL1 = UNITV(*L1);
   uL2 = UNITV(*L2);
   *L3 = VxV(*L1, *L2);
   uL3 = UNITV(*L3);
   *L2 = VxV(*L3, *L1);
   uL2 = UNITV(*L2);

   CLN->rows[0] = *L1;
   CLN->rows[1] = *L2;
   CLN->rows[2] = *L3;

   rp.x         = S->LP[Ilp].X0 * S->D;
   rp.y         = S->LP[Ilp].Y0 * S->D;
   rp.z         = 0.0;
   magvec3_t uv = UNITV(rp);
   magr         = uv.m;
   rhat         = uv.v;
   thhat.x      = -rhat.y;
   thhat.y      = rhat.x;
   thhat.z      = 0.0;
   vp.x = (S->Ddot * S->SMA * rhat.x + magr * S->thdot * thhat.x) * S->MeanRate;
   vp.y = (S->Ddot * S->SMA * rhat.y + magr * S->thdot * thhat.y) * S->MeanRate;
   vp.z = 0.0;
   *PosN = MTxV(*CLN, rp);
   *VelN = MTxV(*CLN, vp);
}
/**********************************************************************/
/*  From Lagrange System "modal" description, find position, velocity */
/*  (m, m/sec) wrt N frame of LagSys Body 1                           */
void LagModes2RV(double SecSinceJ2000, struct LagrangeSystemType *LS,
                 struct OrbitType *O, vec3_t *r, vec3_t *v)
{
   struct LagrangePointType *LP;
   double cw1t, sw1t, cw2t, sw2t, ep, em, cwzt, swzt;
   double TimeSinceEpoch;
   vec3_t rl, vl;
   long i;

   LP = &LS->LP[O->LP];

   TimeSinceEpoch = SecSinceJ2000 - O->Epoch;

   cw1t = cos(LP->w1 * TimeSinceEpoch);
   sw1t = sin(LP->w1 * TimeSinceEpoch);
   if (O->LP < 3) { /* Collinear LP */
      ep   = exp(LP->sigma * TimeSinceEpoch);
      em   = exp(-LP->sigma * TimeSinceEpoch);
      rl.x = O->Ax * cw1t + O->Bx * sw1t + O->Cx * ep + O->Dx * em;
      rl.y = O->Ay * cw1t + O->By * sw1t + O->Cy * ep + O->Dy * em;
      vl.x = LP->w1 * (-O->Ax * sw1t + O->Bx * cw1t) +
             LP->sigma * (O->Cx * ep - O->Dx * em);
      vl.y = LP->w1 * (-O->Ay * sw1t + O->By * cw1t) +
             LP->sigma * (O->Cy * ep - O->Dy * em);
   }
   else { /* Triangular LP */
      cw2t = cos(LP->w2 * TimeSinceEpoch);
      sw2t = sin(LP->w2 * TimeSinceEpoch);
      rl.x = O->Ax * cw1t + O->Bx * sw1t + O->Cx * cw2t + O->Dx * sw2t;
      rl.y = O->Ay * cw1t + O->By * sw1t + O->Cy * cw2t + O->Dy * sw2t;
      vl.x = LP->w1 * (-O->Ax * sw1t + O->Bx * cw1t) +
             LP->w2 * (-O->Cx * sw1t + O->Dx * cw1t);
      vl.y = LP->w1 * (-O->Ay * sw1t + O->By * cw1t) +
             LP->w2 * (-O->Cy * sw1t + O->Dy * cw1t);
   }
   cwzt = cos(LP->wz * TimeSinceEpoch);
   swzt = sin(LP->wz * TimeSinceEpoch);
   rl.z = O->Az * cwzt + O->Bz * swzt;
   vl.z = LP->wz * (-O->Az * swzt + O->Bz * cwzt);

   /* Do we need to keep x,y,z,xdot,ydot,zdot? */
   DEAL_VEC3(rl, O->x, O->y, O->z);
   DEAL_VEC3(vl, O->xdot, O->ydot, O->zdot);

   *r = VxM(rl, LS->CLN);
   *v = VxM(vl, LS->CLN);
   for (i = 0; i < 3; i++)
      r->v[i] += LP->PosN.v[i];

   /* Add velocity of rotating frame, at r, not at LP */
   v->x -= LS->MeanRate * r->y;
   v->y += LS->MeanRate * r->x;
}
/**********************************************************************/
/*  From position, velocity (m, m/sec) wrt N frame of LagSys Body 1,  */
/*  find Lagrange System "modal" description                          */
void RV2LagModes(double SecSinceJ2000, struct LagrangeSystemType *LS,
                 struct OrbitType *O)
{
   struct LagrangePointType *LP;
   double TimeSinceEpoch;
   double R, Rmin;
   double cw1t, sw1t, cw2t, sw2t, ep, em, cwzt, swzt;
   vec3_t rn, rl, vn, vl, LpPosN, LpVelN, wvec, wxr;
   mat3x3_t CLN;
   double **COEF, *RHS, *ParmVec;
   long i, j;

   COEF    = CreateMatrix(8, 8);
   RHS     = (double *)calloc(8, sizeof(double));
   ParmVec = (double *)calloc(8, sizeof(double));

   /* Find which LP is closest.  Assume motion is about that LP */
   O->LP = 0;
   Rmin  = 1.0E15; /* Absurdly large */
   for (i = 0; i < 5; i++) {
      FindLagPtPosVel(SecSinceJ2000, LS, i, &LpPosN, &LpVelN, &CLN);
      for (j = 0; j < 3; j++)
         rn.v[j] = O->PosN.v[j] - LpPosN.v[j];
      R = MAGV(rn);
      if (R < Rmin) {
         O->LP = i;
         Rmin  = R;
      }
   }

   LP = &LS->LP[O->LP];
   FindLagPtPosVel(SecSinceJ2000, LS, O->LP, &LpPosN, &LpVelN, &CLN);

   TimeSinceEpoch = SecSinceJ2000 - O->Epoch;

   cw1t = cos(LP->w1 * TimeSinceEpoch);
   sw1t = sin(LP->w1 * TimeSinceEpoch);
   cw2t = cos(LP->w2 * TimeSinceEpoch);
   sw2t = sin(LP->w2 * TimeSinceEpoch);

   if (O->LP < 3) { /* Collinear LPs */
      ep         = exp(LP->sigma * TimeSinceEpoch);
      em         = exp(-LP->sigma * TimeSinceEpoch);
      COEF[0][0] = cw1t;
      COEF[0][1] = sw1t;
      COEF[0][2] = ep;
      COEF[0][3] = em;
      COEF[1][4] = cw1t;
      COEF[1][5] = sw1t;
      COEF[1][6] = ep;
      COEF[1][7] = em;
      COEF[2][0] = -LP->w1 * sw1t;
      COEF[2][1] = LP->w1 * cw1t;
      COEF[2][2] = LP->sigma * ep;
      COEF[2][3] = -LP->sigma * em;
      COEF[3][4] = -LP->w1 * sw1t;
      COEF[3][5] = LP->w1 * cw1t;
      COEF[3][6] = LP->sigma * ep;
      COEF[3][7] = -LP->sigma * em;
      COEF[4][1] = 2.0 * LS->MeanRate * LP->w1;
      COEF[5][0] = -2.0 * LS->MeanRate * LP->w1;
      COEF[6][2] = 2.0 * LS->MeanRate * LP->sigma;
      COEF[7][3] = -2.0 * LS->MeanRate * LP->sigma;
      COEF[4][4] = LP->Zw1;
      COEF[5][5] = LP->Zw1;
      COEF[6][6] = LP->Zs;
      COEF[7][7] = LP->Zs;
   }
   else {
      COEF[0][0] = cw1t;
      COEF[0][1] = sw1t;
      COEF[0][2] = cw2t;
      COEF[0][3] = sw2t;
      COEF[1][4] = cw1t;
      COEF[1][5] = sw1t;
      COEF[1][6] = cw2t;
      COEF[1][7] = sw2t;
      COEF[2][0] = -LP->w1 * sw1t;
      COEF[2][1] = LP->w1 * cw1t;
      COEF[2][2] = -LP->w2 * sw2t;
      COEF[2][3] = LP->w2 * cw2t;
      COEF[3][4] = -LP->w1 * sw1t;
      COEF[3][5] = LP->w1 * cw1t;
      COEF[3][6] = -LP->w2 * sw2t;
      COEF[3][7] = LP->w2 * cw2t;
      COEF[4][0] = -LP->Kxy;
      COEF[4][1] = 2.0 * LS->MeanRate * LP->w1;
      COEF[5][0] = -2.0 * LS->MeanRate * LP->w1;
      COEF[5][1] = -LP->Kxy;
      COEF[6][2] = -LP->Kxy;
      COEF[6][3] = 2.0 * LS->MeanRate * LP->w2;
      COEF[7][2] = -2.0 * LS->MeanRate * LP->w2;
      COEF[7][3] = -LP->Kxy;
      COEF[4][4] = LP->Zw1;
      COEF[5][5] = LP->Zw1;
      COEF[6][6] = LP->Zw2;
      COEF[7][7] = LP->Zw2;
   }
   for (i = 0; i < 3; i++)
      rn.v[i] = O->PosN.v[i] - LpPosN.v[i];
   rl = MxV(CLN, rn);

   /* O's velocity wrt rotating frame, expressed in N */
   for (i = 0; i < 3; i++)
      wvec.v[i] = LS->MeanRate * CLN.mat[2][i];
   wxr = VxV(wvec, O->PosN);
   for (i = 0; i < 3; i++)
      vn.v[i] = O->VelN.v[i] - wxr.v[i];
   /* Now transform to L frame */
   vl = MxV(CLN, vn);

   RHS[0] = rl.x;
   RHS[1] = rl.y;
   RHS[2] = vl.x;
   RHS[3] = vl.y;
   LINSOLVE(COEF, ParmVec, RHS, 8);
   O->Ax = ParmVec[0];
   O->Bx = ParmVec[1];
   O->Cx = ParmVec[2];
   O->Dx = ParmVec[3];
   O->Ay = ParmVec[4];
   O->By = ParmVec[5];
   O->Cy = ParmVec[6];
   O->Dy = ParmVec[7];

   cwzt = cos(LP->wz * TimeSinceEpoch);
   swzt = sin(LP->wz * TimeSinceEpoch);

   O->Az = cwzt * rl.z - swzt * vl.z / LP->wz;
   O->Bz = swzt * rl.z + cwzt * vl.z / LP->wz;

   DestroyMatrix(COEF);
   free(RHS);
   free(ParmVec);
}
/**********************************************************************/
/*  From position (m) wrt N frame of LagSys Body 1,                   */
/*  find Lagrange System "modal" description, stable mode only        */
void R2StableLagMode(double SecSinceJ2000, struct LagrangeSystemType *LS,
                     struct OrbitType *O)
{
   struct LagrangePointType *LP;
   double TimeSinceEpoch;
   double R, Rmin;
   double cw1t, sw1t, cwzt, swzt;
   vec3_t rn, rl, vn, vl, wvec, wxr, LpPosN, LpVelN;
   mat3x3_t CLN;
   double **COEF, *RHS, *ParmVec;
   long i, j;

   COEF    = CreateMatrix(4, 4);
   RHS     = (double *)calloc(4, sizeof(double));
   ParmVec = (double *)calloc(4, sizeof(double));

   /* Find which LP is closest.  Assume motion is about that LP */
   O->LP = 0;
   Rmin  = 1.0E15; /* Absurdly large */
   for (i = 0; i < 5; i++) {
      FindLagPtPosVel(SecSinceJ2000, LS, i, &LpPosN, &LpVelN, &CLN);
      for (j = 0; j < 3; j++)
         rn.v[j] = O->PosN.v[j] - LpPosN.v[j];
      R = MAGV(rn);
      if (R < Rmin) {
         O->LP = i;
         Rmin  = R;
      }
   }

   LP = &LS->LP[O->LP];
   FindLagPtPosVel(SecSinceJ2000, LS, O->LP, &LpPosN, &LpVelN, &CLN);

   TimeSinceEpoch = SecSinceJ2000 - O->Epoch;

   cw1t = cos(LP->w1 * TimeSinceEpoch);
   sw1t = sin(LP->w1 * TimeSinceEpoch);

   if (O->LP < 3) { /* Collinear LPs */
      COEF[0][0] = cw1t;
      COEF[0][1] = sw1t;
      COEF[1][2] = cw1t;
      COEF[1][3] = sw1t;
      COEF[2][1] = 2.0 * LS->MeanRate * LP->w1;
      COEF[3][0] = -2.0 * LS->MeanRate * LP->w1;
      COEF[2][2] = LP->Zw1;
      COEF[3][3] = LP->Zw1;
   }
   else {
      COEF[0][0] = cw1t;
      COEF[0][1] = sw1t;
      COEF[1][2] = cw1t;
      COEF[1][3] = sw1t;
      COEF[2][0] = -LP->Kxy;
      COEF[2][1] = 2.0 * LS->MeanRate * LP->w1;
      COEF[3][0] = -2.0 * LS->MeanRate * LP->w1;
      COEF[3][1] = -LP->Kxy;
      COEF[2][2] = LP->Zw1;
      COEF[3][3] = LP->Zw1;
   }
   for (i = 0; i < 3; i++)
      rn.v[i] = O->PosN.v[i] - LpPosN.v[i];
   rl = MxV(CLN, rn);

   /* O's velocity wrt rotating frame, expressed in N */
   for (i = 0; i < 3; i++)
      wvec.v[i] = LS->MeanRate * CLN.mat[2][i];
   wxr = VxV(wvec, O->PosN);
   for (i = 0; i < 3; i++)
      vn.v[i] = O->VelN.v[i] - wxr.v[i];
   /* Now transform to L frame */
   vl = MxV(CLN, vn);

   RHS[0] = rl.x;
   RHS[1] = rl.y;
   LINSOLVE(COEF, ParmVec, RHS, 4);
   O->Ax = ParmVec[0];
   O->Bx = ParmVec[1];
   O->Cx = 0.0;
   O->Dx = 0.0;
   O->Ay = ParmVec[2];
   O->By = ParmVec[3];
   O->Cy = 0.0;
   O->Dy = 0.0;

   cwzt = cos(LP->wz * TimeSinceEpoch);
   swzt = sin(LP->wz * TimeSinceEpoch);

   O->Az = cwzt * rl.z - swzt * vl.z / LP->wz;
   O->Bz = swzt * rl.z + cwzt * vl.z / LP->wz;

   DestroyMatrix(COEF);
   free(RHS);
   free(ParmVec);
}
/**********************************************************************/
/*  From position, velocity (m, m/sec) wrt N frame of LagSys Body 1,  */
/*  find Lagrange System "modal" description                          */
void XYZ2LagModes(double TimeSinceEpoch, struct LagrangeSystemType *LS,
                  struct OrbitType *O)
{
   struct LagrangePointType *LP;
   double cw1t, sw1t, cw2t, sw2t, ep, em, cwzt, swzt;
   double **COEF, *RHS, *ParmVec;

   COEF    = CreateMatrix(8, 8);
   RHS     = (double *)calloc(8, sizeof(double));
   ParmVec = (double *)calloc(8, sizeof(double));

   LP = &LS->LP[O->LP];

   cw1t = cos(LP->w1 * TimeSinceEpoch);
   sw1t = sin(LP->w1 * TimeSinceEpoch);
   cw2t = cos(LP->w2 * TimeSinceEpoch);
   sw2t = sin(LP->w2 * TimeSinceEpoch);

   if (O->LP < 3) { /* Collinear LPs */
      ep         = exp(LP->sigma * TimeSinceEpoch);
      em         = exp(-LP->sigma * TimeSinceEpoch);
      COEF[0][0] = cw1t;
      COEF[0][1] = sw1t;
      COEF[0][2] = ep;
      COEF[0][3] = em;
      COEF[1][4] = cw1t;
      COEF[1][5] = sw1t;
      COEF[1][6] = ep;
      COEF[1][7] = em;
      COEF[2][0] = -LP->w1 * sw1t;
      COEF[2][1] = LP->w1 * cw1t;
      COEF[2][2] = LP->sigma * ep;
      COEF[2][3] = -LP->sigma * em;
      COEF[3][4] = -LP->w1 * sw1t;
      COEF[3][5] = LP->w1 * cw1t;
      COEF[3][6] = LP->sigma * ep;
      COEF[3][7] = -LP->sigma * em;
      COEF[4][1] = 2.0 * LS->MeanRate * LP->w1;
      COEF[5][0] = -2.0 * LS->MeanRate * LP->w1;
      COEF[6][2] = 2.0 * LS->MeanRate * LP->sigma;
      COEF[7][3] = -2.0 * LS->MeanRate * LP->sigma;
      COEF[4][4] = LP->Zw1;
      COEF[5][5] = LP->Zw1;
      COEF[6][6] = LP->Zs;
      COEF[7][7] = LP->Zs;
   }
   else {
      COEF[0][0] = cw1t;
      COEF[0][1] = sw1t;
      COEF[0][2] = cw2t;
      COEF[0][3] = sw2t;
      COEF[1][4] = cw1t;
      COEF[1][5] = sw1t;
      COEF[1][6] = cw2t;
      COEF[1][7] = sw2t;
      COEF[2][0] = -LP->w1 * sw1t;
      COEF[2][1] = LP->w1 * cw1t;
      COEF[2][2] = -LP->w2 * sw2t;
      COEF[2][3] = LP->w2 * cw2t;
      COEF[3][4] = -LP->w1 * sw1t;
      COEF[3][5] = LP->w1 * cw1t;
      COEF[3][6] = -LP->w2 * sw2t;
      COEF[3][7] = LP->w2 * cw2t;
      COEF[4][0] = -LP->Kxy;
      COEF[4][1] = 2.0 * LS->MeanRate * LP->w1;
      COEF[5][0] = -2.0 * LS->MeanRate * LP->w1;
      COEF[5][1] = -LP->Kxy;
      COEF[6][2] = -LP->Kxy;
      COEF[6][3] = 2.0 * LS->MeanRate * LP->w2;
      COEF[7][2] = -2.0 * LS->MeanRate * LP->w2;
      COEF[7][3] = -LP->Kxy;
      COEF[4][4] = LP->Zw1;
      COEF[5][5] = LP->Zw1;
      COEF[6][6] = LP->Zw2;
      COEF[7][7] = LP->Zw2;
   }

   RHS[0] = O->x;
   RHS[1] = O->y;
   RHS[2] = O->xdot;
   RHS[3] = O->ydot;
   LINSOLVE(COEF, ParmVec, RHS, 8);
   O->Ax = ParmVec[0];
   O->Bx = ParmVec[1];
   O->Cx = ParmVec[2];
   O->Dx = ParmVec[3];
   O->Ay = ParmVec[4];
   O->By = ParmVec[5];
   O->Cy = ParmVec[6];
   O->Dy = ParmVec[7];

   cwzt = cos(LP->wz * TimeSinceEpoch);
   swzt = sin(LP->wz * TimeSinceEpoch);

   O->Az = cwzt * O->z - swzt * O->zdot / LP->wz;
   O->Bz = swzt * O->z + cwzt * O->zdot / LP->wz;

   DestroyMatrix(COEF);
   free(RHS);
   free(ParmVec);
}
/**********************************************************************/
void AmpPhase2LagModes(double TimeSinceEpoch, double AmpXY1, double PhiXY1,
                       double SenseXY1, double AmpXY2, double PhiXY2,
                       double SenseXY2, double AmpZ, double PhiZ,
                       struct LagrangeSystemType *S, struct OrbitType *O)
{
   double A, B, ca, sa, cphi, sphi;
   struct LagrangePointType *LP;

   LP = &S->LP[O->LP];

   PhiXY1 -= LP->w1 * TimeSinceEpoch;
   PhiXY2 -= LP->w2 * TimeSinceEpoch;
   PhiZ   -= LP->wz * TimeSinceEpoch;

   if (SenseXY1 < 0.0)
      PhiXY1 = -PhiXY1;
   if (SenseXY2 < 0.0)
      PhiXY2 = -PhiXY2;

   if (O->LP < 3) { /* Collinear, only one stable mode */
      A     = AmpXY1;
      B     = SenseXY1 * LP->AR1 * A;
      sphi  = sin(PhiXY1);
      cphi  = cos(PhiXY1);
      O->Ax = -B * sphi;
      O->Bx = -B * cphi;
      O->Ay = A * cphi;
      O->By = -A * sphi;
      O->Cx = 0.0;
      O->Dx = 0.0;
      O->Cy = 0.0;
      O->Dy = 0.0;
   }
   else { /* Triangular, two stable modes */
      A     = AmpXY1;
      B     = SenseXY1 * LP->AR1 * A;
      sa    = LP->sa1;
      ca    = LP->ca1;
      sphi  = sin(PhiXY1);
      cphi  = cos(PhiXY1);
      O->Ax = -A * sa * cphi - B * sa * sphi;
      O->Bx = A * sa * sphi - B * ca * cphi;
      O->Ay = A * ca * cphi - B * sa * sphi;
      O->By = -A * ca * sphi - B * sa * cphi;

      A     = AmpXY2;
      B     = SenseXY2 * LP->AR2 * A;
      sa    = LP->sa2;
      ca    = LP->ca2;
      sphi  = sin(PhiXY2);
      cphi  = cos(PhiXY2);
      O->Cx = -A * sa * cphi - B * sa * sphi;
      O->Dx = A * sa * sphi - B * ca * cphi;
      O->Cy = A * ca * cphi - B * sa * sphi;
      O->Dy = -A * ca * sphi - B * sa * cphi;
   }

   O->Az = AmpZ * cos(PhiZ);
   O->Bz = AmpZ * sin(PhiZ);
}
/*************************************************************************/
/*   Conversion of state [nd] in rotating barycentric frame to iNertial  */
/*   frame [dim] (body 1 centric)                                        */
/*   Follows algorithm given in TA Pavlak's Ph.D. thesis                 */
/*   Summary of process:                                                 */
void StateRnd2StateN(struct LagrangeSystemType *LS, vec3_t W2_pos,
                     vec3_t W2_vel, vec3_t R_R_nd, vec3_t V_R_nd, vec3_t *R_N,
                     vec3_t *V_N)
{
   vec3_t bary_p, bary_v;
   vec3_t r2_from_r1, v2_from_v1, yvec;
   magvec3_t uxvec, uzvec;
   vec3_t *const xvec = &uxvec.v;
   vec3_t *const zvec = &uzvec.v;
   mat3x3_t CRN;
   double LU, VU;
   // double TU;
   double magr, theta_dot;
   double full_N_state[6], full_R_state[6];

   for (int i = 0; i < 3; i++)
      bary_p.v[i] = (LS->mu2 * W2_pos.v[i]) / (LS->mu1 + LS->mu2);
   for (int i = 0; i < 3; i++)
      bary_v.v[i] = (LS->mu2 * W2_vel.v[i]) / (LS->mu1 + LS->mu2);

   r2_from_r1 = W2_pos; // World 2 StateN is already centered on Body1
   v2_from_v1 = W2_vel;

   LU = LS->LU;
   // TU = LS->TU;
   VU = LS->VU;

   *xvec = r2_from_r1;
   uxvec = UNITV(*xvec);
   *zvec = VxV(r2_from_r1, v2_from_v1);
   uzvec = UNITV(*zvec);
   yvec  = VxV(*zvec, *xvec);

   magr = MAGV(r2_from_r1);

   theta_dot = uzvec.m / (magr * magr);

   mat3x3_t CNR = {.rows = {*xvec, yvec, *zvec}};
   CRN          = MT(CNR);

   // Transform nd Body 1 Centric to inertial
   double **StateCRN = CreateMatrix(6, 6);

   for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
         StateCRN[i][j] = 0;
   for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
         StateCRN[i][j] = CRN.mat[i][j];
   for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
         StateCRN[i + 3][j + 3] = CRN.mat[i][j];

   for (int i = 0; i < 3; i++)
      StateCRN[i + 3][0] = theta_dot * CRN.mat[i][1];
   for (int i = 0; i < 3; i++)
      StateCRN[i + 3][1] = -theta_dot * CRN.mat[i][0];

   // Transform nd barycenter to nd
   for (int i = 0; i < 3; i++)
      full_R_state[i] = R_R_nd.v[i] * LU;
   for (int i = 0; i < 3; i++)
      full_R_state[i + 3] = V_R_nd.v[i] * VU;

   MxVG(StateCRN, full_R_state, full_N_state, 6, 6);
   DestroyMatrix(StateCRN);

   // Output
   for (int i = 0; i < 3; i++)
      R_N->v[i] = full_N_state[i] + bary_p.v[i];
   for (int i = 0; i < 3; i++)
      V_N->v[i] = full_N_state[i + 3] + bary_v.v[i];
}
/*************************************************************************/
/*   Conversion of iNertial frame (body 1 centered) [dim] to state [nd]  */
/*   in rotating barycentric frame                                       */
/*   Follows algorithm given in TA Pavlak's Ph.D. thesis                 */
/*   Summary of process:                                                 */
/*    - Construct position rotation matrix from rotating to inertial     */
/*    - Construct full transformation matrix for rot. state to N state   */
/*    - Invert transformation to yield N state to rot state              */
/*    - Transform to rot state                                           */
/*    - Center at barycenter                                             */
void StateN2StateRnd(struct LagrangeSystemType *LS, vec3_t W2_pos,
                     vec3_t W2_vel, vec3_t R_N, vec3_t V_N, vec3_t *R_R_nd,
                     vec3_t *V_R_nd)
{
   vec3_t bary_p, bary_v, r2_from_r1, v2_from_v1, yvec;
   magvec3_t uxvec, uzvec;
   vec3_t *const xvec = &uxvec.v;
   vec3_t *const zvec = &uzvec.v;
   mat3x3_t CRN;
   double LU, VU;
   // double TU;
   double magr, theta_dot;
   double StateCRN[6][6], StateCNR[6][6];
   double full_N_state[6], full_R_state[6];

   for (int i = 0; i < 3; i++)
      bary_p.v[i] = (LS->mu2 * W2_pos.v[i]) / (LS->mu1 + LS->mu2);
   for (int i = 0; i < 3; i++)
      bary_v.v[i] = (LS->mu2 * W2_vel.v[i]) / (LS->mu1 + LS->mu2);

   r2_from_r1 = W2_pos;
   v2_from_v1 = W2_vel;

   LU = LS->LU;
   // TU = LS->TU;
   VU = LS->VU;

   *xvec = r2_from_r1;
   uxvec = UNITV(*xvec);
   *zvec = VxV(r2_from_r1, v2_from_v1);
   uzvec = UNITV(*zvec);
   yvec  = VxV(*zvec, *xvec);

   magr = MAGV(r2_from_r1);

   theta_dot = uzvec.m / (magr * magr);

   mat3x3_t CNR = {.rows = {*xvec, yvec, *zvec}};
   CRN          = MT(CNR);

   for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
         StateCRN[i][j] = 0;
   for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
         StateCRN[i][j] = CRN.mat[i][j];
   for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
         StateCRN[i + 3][j + 3] = CRN.mat[i][j];

   for (int i = 0; i < 3; i++)
      StateCRN[i + 3][0] = theta_dot * CRN.mat[i][1];
   for (int i = 0; i < 3; i++)
      StateCRN[i + 3][1] = -theta_dot * CRN.mat[i][0];

   // inertial to rotating is inverse
   FastMINV6(StateCRN, StateCNR, 6);

   // Assemble State N
   for (int i = 0; i < 3; i++)
      full_N_state[i] =
          R_N.v[i] - bary_p.v[i]; // transform body centric to barycentric nd
   for (int i = 0; i < 3; i++)
      full_N_state[i + 3] = V_N.v[i] - bary_v.v[i];

   double **A = CreateMatrix(6, 6);
   for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
         A[i][j] = StateCNR[i][j];

   MxVG(A, full_N_state, full_R_state, 6, 6);
   DestroyMatrix(A);
   for (int i = 0; i < 3; i++) {
      R_R_nd->v[i] = full_R_state[0 + i] / LU;
      V_R_nd->v[i] = full_R_state[3 + i] / VU;
   }
}
/**********************************************************************/
/*   Notional position and velocities for TDRS satellites             */
/*   Note that TDRS[1] (TDRS-2) was lost at launch                    */
void TDRSPosVel(double PriMerAng, double dyntime, vec3_t ptn[10],
                vec3_t vtn[10])
{

   double Lng[10] = {-49.0,  0.0,    -275.0, -46.0, -171.4,
                     -173.7, -150.0, -271.0, -62.4, -40.9}; /* deg */
   double a[10]   = {42.241E6, 42.241E6, 42.241E6, 42.241E6, 42.241E6,
                     42.241E6, 42.241E6, 42.241E6, 42.241E6, 42.241E6}; /* m */
   double e[10]   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   double i[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; /* deg */
   double LANdrift[10] = {0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0}; /* r/s */
   double omdrift[10]  = {0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0}; /* r/s */
   double anom, Li, LAN, om;
   double S1, C1, C2, S3, C3;
   long j;

   static double p[10];
   static double LAN0[10] = {0.0,   36.0,  72.0,  108.0, 144.0,
                             180.0, 216.0, 252.0, 288.0, 324.0}; /* deg */
   static double om0[10];
   static long First = 1;

   if (First) {
      First = 0;
      for (j = 0; j < 10; j++) {
         p[j] = a[j] * (1.0 - e[j] * e[j]);

         Li     = Lng[j] * D2R + PriMerAng;
         S1     = sin(LAN0[j] * D2R);
         C1     = cos(LAN0[j] * D2R);
         C2     = cos(i[j] * D2R);
         S3     = sin(Li);
         C3     = cos(Li);
         om0[j] = atan2(-(C1 * S3 - S1 * C3), (-S1 * C2 * S3 - C1 * C2 * C3));
      }
   }

   for (j = 0; j < 10; j++) {
      om  = om0[j] + omdrift[j] * dyntime;
      LAN = LAN0[j] * D2R + LANdrift[j] * dyntime;

      Eph2RV(3.986004E14, p[j], e[j], i[j] * D2R, LAN, om, dyntime, &ptn[j],
             &vtn[j], &anom);
   }
}
/**********************************************************************/
/* Find coordinate transformation from True Equator True Equinox      */
/* (TETE) frame to J2000 frame.  Ref "The Astronomical Almanac",      */
/* QB8.U5, 2003, p. B18,B20.                                          */
mat3x3_t TETE2J2000(double JD)
{

   double d, arg1, arg2, dpsi, deps, eps;
   double T, z, theta, zeta;
   double c1, s1, c2, s2, c3, s3;
   mat3x3_t CTM, CMJ, CTJ;

   /* TETE to MEME */
   d    = JD - 2452639.5;
   arg1 = (67.1 - 0.053 * d) * D2R;
   arg2 = (198.5 + 1.971 * d) * D2R;
   dpsi = (-0.0048 * sin(arg1) - 0.0004 * sin(arg2)) * D2R;
   deps = (0.0026 * cos(arg1) + 0.0002 * cos(arg2)) * D2R;
   eps  = 23.44 * D2R;

   CTM.mat[0][0] = 1.0;
   CTM.mat[1][1] = 1.0;
   CTM.mat[2][2] = 1.0;
   CTM.mat[2][1] = deps;
   CTM.mat[2][0] = dpsi * sin(eps);
   CTM.mat[1][0] = dpsi * cos(eps);
   CTM.mat[1][2] = -CTM.mat[2][1];
   CTM.mat[0][2] = -CTM.mat[2][0];
   CTM.mat[0][1] = -CTM.mat[1][0];

   /* MEME to J2000 */
   T     = (JD - 2451545.0) / 36525.0;
   z     = D2R * ((0.6406161 + (3.041E-4 + 5.10E-6 * T) * T) * T);
   theta = D2R * ((0.5567530 - (1.185E-4 + 1.16E-5 * T) * T) * T);
   zeta  = D2R * ((0.6406161 + (8.390E-5 + 5.00E-6 * T) * T) * T);

   c1            = cos(-zeta);
   s1            = sin(-zeta);
   c2            = cos(theta);
   s2            = sin(theta);
   c3            = cos(-z);
   s3            = sin(-z);
   CMJ.mat[0][0] = c1 * c2 * c3 - s3 * s1;
   CMJ.mat[1][0] = -c1 * c2 * s3 - c3 * s1;
   CMJ.mat[2][0] = c1 * s2;
   CMJ.mat[0][1] = s1 * c2 * c3 + s3 * c1;
   CMJ.mat[1][1] = -s1 * c2 * s3 + c3 * c1;
   CMJ.mat[2][1] = s1 * s2;
   CMJ.mat[0][2] = -s2 * c3;
   CMJ.mat[1][2] = s2 * s3;
   CMJ.mat[2][2] = c2;

   /* Concatenate */
   CTJ.mat[0][0] = CTM.mat[0][0] * CMJ.mat[0][0] +
                   CTM.mat[0][1] * CMJ.mat[1][0] +
                   CTM.mat[0][2] * CMJ.mat[2][0];
   CTJ.mat[0][1] = CTM.mat[0][0] * CMJ.mat[0][1] +
                   CTM.mat[0][1] * CMJ.mat[1][1] +
                   CTM.mat[0][2] * CMJ.mat[2][1];
   CTJ.mat[0][2] = CTM.mat[0][0] * CMJ.mat[0][2] +
                   CTM.mat[0][1] * CMJ.mat[1][2] +
                   CTM.mat[0][2] * CMJ.mat[2][2];
   CTJ.mat[1][0] = CTM.mat[1][0] * CMJ.mat[0][0] +
                   CTM.mat[1][1] * CMJ.mat[1][0] +
                   CTM.mat[1][2] * CMJ.mat[2][0];
   CTJ.mat[1][1] = CTM.mat[1][0] * CMJ.mat[0][1] +
                   CTM.mat[1][1] * CMJ.mat[1][1] +
                   CTM.mat[1][2] * CMJ.mat[2][1];
   CTJ.mat[1][2] = CTM.mat[1][0] * CMJ.mat[0][2] +
                   CTM.mat[1][1] * CMJ.mat[1][2] +
                   CTM.mat[1][2] * CMJ.mat[2][2];
   CTJ.mat[2][0] = CTM.mat[2][0] * CMJ.mat[0][0] +
                   CTM.mat[2][1] * CMJ.mat[1][0] +
                   CTM.mat[2][2] * CMJ.mat[2][0];
   CTJ.mat[2][1] = CTM.mat[2][0] * CMJ.mat[0][1] +
                   CTM.mat[2][1] * CMJ.mat[1][1] +
                   CTM.mat[2][2] * CMJ.mat[2][1];
   CTJ.mat[2][2] = CTM.mat[2][0] * CMJ.mat[0][2] +
                   CTM.mat[2][1] * CMJ.mat[1][2] +
                   CTM.mat[2][2] * CMJ.mat[2][2];
   return CTJ;
}
/**********************************************************************/
/*  See Battin                                                        */
double RadiusOfInfluence(double mu1, double mu2, double r)
{
   return (r * pow(mu2 / mu1, 0.4));
}
/**********************************************************************/
/*  Given Rrel and Vrel, find the Euler-Hill state vector [re, ve]    */
/*  E-H usually assumes small departures from LVLH.  I'm using        */
/*  spherical coordinates here to ensure valid solution anywhere.     */
pair_vec3_t RelRV2EHRV(double OrbRadius, double OrbRate, mat3x3_t OrbCLN,
                       vec3_t Rrel, vec3_t Vrel)
{
   double magp, alpha, beta;
   vec3_t p, b3, vn, vb;
   mat3x3_t CBL, CBN;
   double C1, S1, C2, S2;
   pair_vec3_t pair;
   vec3_t *const re = &pair.first;
   vec3_t *const ve = &pair.second;
   long i;

   for (i = 0; i < 3; i++)
      p.v[i] = Rrel.v[i] - OrbRadius * OrbCLN.mat[2][i];
   magvec3_t uv = UNITV(p);
   magp         = uv.m;
   b3           = uv.v;
   for (i = 0; i < 3; i++)
      b3.v[i] = -b3.v[i];
   alpha = atan2(-VoV(OrbCLN.rows[0], b3), VoV(OrbCLN.rows[2], b3));
   beta  = asin(-VoV(b3, OrbCLN.rows[1]));

   re->x = OrbRadius * alpha;
   re->y = OrbRadius * beta;
   re->x = OrbRadius - magp;

   C1            = cos(alpha);
   S1            = -sin(alpha);
   C2            = cos(beta);
   S2            = sin(beta);
   CBL.mat[0][0] = C1;
   CBL.mat[1][0] = S1 * S2;
   CBL.mat[2][0] = S1 * C2;
   CBL.mat[0][1] = 0.0;
   CBL.mat[1][1] = C2;
   CBL.mat[2][1] = -S2;
   CBL.mat[0][2] = -S1;
   CBL.mat[1][2] = C1 * S2;
   CBL.mat[2][2] = C1 * C2;
   CBN           = MxM(CBL, OrbCLN);
   for (i = 0; i < 3; i++)
      vn.v[i] = Vrel.v[i] + OrbRadius * OrbRate * OrbCLN.mat[0][i];
   vb = MxV(CBN, vn);

   *ve    = vb;
   ve->x -= OrbRate * magp;
   return pair;
}
/**********************************************************************/
/*  Given a circular reference, and the Euler-Hill state vector       */
/*  find the relative position and velocity (expressed in N)          */
/*  E-H usually assumes small departures from LVLH.  I'm using        */
/*  spherical coordinates here to ensure valid solution anywhere.     */
pair_vec3_t EHRV2RelRV(double OrbRadius, double OrbRate, mat3x3_t OrbCLN,
                       vec3_t re, vec3_t ve)
{
   double alpha, beta, magp;
   mat3x3_t CBL, CBN;
   vec3_t vb, vn;
   double C1, S1, C2, S2;
   pair_vec3_t pair;
   vec3_t *const Rrel = &pair.first;
   vec3_t *const Vrel = &pair.second;
   long i;

   alpha = re.x / OrbRadius;
   beta  = re.y / OrbRadius;
   magp  = OrbRadius - re.z;

   C1            = cos(alpha);
   S1            = -sin(alpha);
   C2            = cos(beta);
   S2            = sin(beta);
   CBL.mat[0][0] = C1;
   CBL.mat[1][0] = S1 * S2;
   CBL.mat[2][0] = S1 * C2;
   CBL.mat[0][1] = 0.0;
   CBL.mat[1][1] = C2;
   CBL.mat[2][1] = -S2;
   CBL.mat[0][2] = -S1;
   CBL.mat[1][2] = C1 * S2;
   CBL.mat[2][2] = C1 * C2;
   CBN           = MxM(CBL, OrbCLN);
   for (i = 0; i < 3; i++)
      Rrel->v[i] = OrbRadius * OrbCLN.mat[2][i] - magp * CBN.mat[2][i];

   vb   = ve;
   vb.x = ve.x + OrbRate * magp;
   vn   = MTxV(CBN, vb);

   for (i = 0; i < 3; i++)
      Vrel->v[i] = vn.v[i] - OrbRadius * OrbRate * OrbCLN.rows[0].v[i];
   return pair;
}
/**********************************************************************/
/*  Given Euler-Hill position and velocity, find parameters of        */
/*  Drift, Ellipse, Static, and Cross-Track modes                     */
void EHRV2EHModes(vec3_t r, vec3_t v, double n, double nt, double *A,
                  double *Bc, double *Bs, double *C, double *Dc, double *Ds)
{
   double s, c, zuterm;

   s      = sin(nt);
   c      = cos(nt);
   zuterm = 2.0 * v.v[0] / n - 3.0 * r.v[2];

   /* Drift */
   *A = 4.0 * r.v[2] - 2.0 * v.v[0] / n;

   /* Ellipse */
   *Bc = zuterm * c - v.v[2] / n * s;
   *Bs = v.v[2] / n * c + zuterm * s;

   /* Static */
   *C = r.v[0] - 6.0 * nt * r.v[2] + 3.0 * nt * v.v[0] / n + 2.0 * v.v[2] / n;

   /* Cross-Track */
   *Dc = v.v[1] / n * c + r.v[1] * s;
   *Ds = v.v[1] / n * s - r.v[1] * c;
}
/**********************************************************************/
void EHModes2EHRV(double A, double Bc, double Bs, double C, double Dc,
                  double Ds, double n, double nt, vec3_t *const r,
                  vec3_t *const v)
{
   double c, s, BCosTheta, BSinTheta, DCosTheta, DSinTheta;

   c = cos(nt);
   s = sin(nt);
   /* B*cos(nt-thp) */
   BCosTheta = Bc * c + Bs * s;
   /* B*sin(nt-thp) */
   BSinTheta = Bc * s - Bs * c;
   /* D*cos(nt-thn) */
   DCosTheta = Dc * c + Ds * s;
   /* D*sin(nt-thn) */
   DSinTheta = Dc * s - Ds * c;

   r->v[0] = 1.5 * A * nt + 2.0 * BSinTheta + C;
   r->v[1] = DSinTheta;
   r->v[2] = A + BCosTheta;
   v->v[0] = 1.5 * A * n + 2.0 * n * BCosTheta;
   v->v[1] = n * DCosTheta;
   v->v[2] = -n * BSinTheta;
}
/**********************************************************************/
/* See Battin 7.1                                                     */
double LambertTOF(double mu, double amin, double lambda, double x)
{
   double y, eta, S1, Q, T;
   double delta = 1.0;
   double u     = 1.0;
   double Sigma = 1.0;
   long n       = 1;
   double gamma;

   y   = sqrt(1.0 - lambda * lambda * (1.0 - x * x));
   eta = y - lambda * x;
   S1  = 0.5 * (1.0 - lambda - x * eta);

   /* Q = HyperQ(S1) */
   while (fabs(u) > 1.0E-8 && n < 1000) {
      if (n % 2) { /* n odd */
         gamma = ((double)((n + 2) * (n + 5))) /
                 ((double)((2 * n + 1) * (2 * n + 3)));
      }
      else { /* n even */
         gamma =
             ((double)(n * (n - 3))) / ((double)((2 * n + 1) * (2 * n + 3)));
      }
      delta  = 1.0 / (1.0 - gamma * delta * S1);
      u     *= (delta - 1.0);
      Sigma += u;
      n++;
   }
   Q = 4.0 / 3.0 * Sigma;

   T = (eta * eta * Q + 4.0 * lambda) * eta * sqrt(amin * amin * amin / mu);
   return (T);
}
/**********************************************************************/
/*  See Battin 7.1                                                    */
/*  TransferType =  1.0 for Type I  (1H, 1A, 1B) transfers            */
/*  TransferType = -1.0 for Type II (2H, 2A, 2B) transfers            */
void LambertProblem(double t0, double mu, vec3_t xr1, vec3_t xr2, double TOF,
                    double TransferType, double *SLR, double *e, double *inc,
                    double *RAAN, double *ArgP, double *tp)
{
   double r1, r2, th, c, s, amin, lambda;
   double xold, Told, x, dx, T;
   double y, eta, Coef0, Coef1, Coef2;
   vec3_t ir1, ir2, ih, dr, ihxir1, xv1;
   double a, anom, alpha, MeanMotion, Period, rmin;
   long i;
   magvec3_t uv;

   uv  = UNITV(xr1);
   r1  = uv.m;
   ir1 = uv.v;
   uv  = UNITV(xr2);
   r2  = uv.m;
   ir2 = uv.v;
   ih  = VxV(xr1, xr2);
   uv  = UNITV(ih);
   ih  = uv.v;
   th  = acos(VoV(ir1, ir2));
   for (i = 0; i < 3; i++)
      dr.v[i] = xr2.v[i] - xr1.v[i];
   c      = MAGV(dr);
   s      = 0.5 * (r1 + r2 + c);
   amin   = 0.5 * s;
   lambda = TransferType * sqrt(r1 * r2) * cos(0.5 * th) / s;

   /* Secant Search for x */
   xold = 0.0;
   Told = LambertTOF(mu, amin, lambda, xold);
   x    = 0.1;
   dx   = x - xold;
   while (fabs(dx) > 1.0E-6) {
      T    = LambertTOF(mu, amin, lambda, x);
      dx   = (TOF - T) / (T - Told) * (x - xold);
      xold = x;
      Told = T;
      if (dx < -0.1)
         dx = -0.1;
      if (dx > 0.1)
         dx = 0.1;
      x += dx;
      if (x < -1.0)
         x = -1.0;
   }

   y      = sqrt(1.0 - lambda * lambda * (1.0 - x * x));
   eta    = y - lambda * x;
   ihxir1 = VxV(ih, ir1);
   Coef0  = sqrt(mu / amin) / eta;
   Coef1  = (2.0 * lambda * amin / r1 - (lambda + x * eta)) * Coef0;
   Coef2  = sqrt(r2 / r1) * sin(0.5 * th) * Coef0;
   for (i = 0; i < 3; i++)
      xv1.v[i] = Coef1 * ir1.v[i] + Coef2 * TransferType * ihxir1.v[i];
   RV2Eph(t0, mu, xr1, xv1, &a, e, inc, RAAN, ArgP, &anom, tp, SLR, &alpha,
          &rmin, &MeanMotion, &Period);
}
/**********************************************************************/
double RendezvousCostFunction(double *InVec, double *AuxVec)
{
   double t0, TOF, mu;
   double tf, SMA, ecc, inc, RAAN, ArgP, anom, tp, p, alpha, MeanMotion, rmin;
   vec3_t r1, v1, r2, v2, r1e, v1e, r2e, v2e;
   vec3_t r1t, v1t, r2t, v2t;
   vec3_t DV1I, DV2I, DV1II, DV2II, DV1, DV2;
   double Per1, Per2, DeltaV, DeltaVII, DeltaVI;
   long i;

   t0  = InVec[0];
   TOF = InVec[1];

   mu = AuxVec[0];
   CopyVG(r1e.v, &AuxVec[1], 3);
   CopyVG(v1e.v, &AuxVec[1], 3);
   CopyVG(r2e.v, &AuxVec[7], 3);
   CopyVG(v2e.v, &AuxVec[10], 3);

   tf = t0 + TOF;

   /* .. Find r1(t0), r2(tf) */
   RV2Eph(0.0, mu, r1e, v1e, &SMA, &ecc, &inc, &RAAN, &ArgP, &anom, &tp, &p,
          &alpha, &rmin, &MeanMotion, &Per1);
   Eph2RV(mu, p, ecc, inc, RAAN, ArgP, t0 - tp, &r1, &v1, &anom);

   RV2Eph(0.0, mu, r2e, v2e, &SMA, &ecc, &inc, &RAAN, &ArgP, &anom, &tp, &p,
          &alpha, &rmin, &MeanMotion, &Per2);
   Eph2RV(mu, p, ecc, inc, RAAN, ArgP, tf - tp, &r2, &v2, &anom);

   /* .. Solve Lambert problem to find transfer orbit, transfer angle < Pi */
   LambertProblem(t0, mu, r1, r2, TOF, 1.0, &p, &ecc, &inc, &RAAN, &ArgP, &tp);

   /* .. Compute delta-V */
   Eph2RV(mu, p, ecc, inc, RAAN, ArgP, t0 - tp, &r1t, &v1t, &anom);
   Eph2RV(mu, p, ecc, inc, RAAN, ArgP, tf - tp, &r2t, &v2t, &anom);

   for (i = 0; i < 3; i++) {
      DV1I.v[i] = v1t.v[i] - v1.v[i];
      DV2I.v[i] = v2.v[i] - v2t.v[i];
   }
   DeltaVI = MAGV(DV1I) + MAGV(DV2I);

   /* .. Solve Lambert problem to find transfer orbit, transfer angle > Pi */
   LambertProblem(t0, mu, r1, r2, TOF, -1.0, &p, &ecc, &inc, &RAAN, &ArgP, &tp);

   /* .. Compute delta-V */
   Eph2RV(mu, p, ecc, inc, RAAN, ArgP, t0 - tp, &r1t, &v1t, &anom);
   Eph2RV(mu, p, ecc, inc, RAAN, ArgP, tf - tp, &r2t, &v2t, &anom);

   for (i = 0; i < 3; i++) {
      DV1II.v[i] = v1t.v[i] - v1.v[i];
      DV2II.v[i] = v2.v[i] - v2t.v[i];
   }
   DeltaVII = MAGV(DV1II) + MAGV(DV2II);

   if (DeltaVI < DeltaVII) {
      DeltaV = DeltaVI;
      DV1    = DV1I;
      DV2    = DV2I;
   }
   else {
      DeltaV = DeltaVII;
      DV1    = DV1II;
      DV2    = DV2II;
   }
   for (i = 0; i < 3; i++) {
      AuxVec[13 + i] = DV1.v[i];
      AuxVec[16 + i] = DV2.v[i];
   }

   /* Ramp penalties to constrain solution */
   if (t0 < 0.0)
      DeltaV += 10.0 * (-t0 / Per1);
   if (t0 > Per1)
      DeltaV += 10.0 * ((t0 - Per1) / Per1);
   if (TOF < 0.0)
      DeltaV += 10.0 * (-TOF / Per1);
   if (TOF > Per1 + Per2)
      DeltaV += 10.0 * ((TOF - Per1 - Per2) / (Per1 + Per2));

   return (DeltaV);
}
/**********************************************************************/
/*  Given starting state (r1e, v1e) and target state (r2e, v2e) at    */
/*  epoch t=0, find times (t1, t2) and DVs (DV1, DV2) to perform      */
/*  rendezvous.                                                       */
void PlanTwoImpulseRendezvous(double mu, vec3_t r1e, vec3_t v1e, vec3_t r2e,
                              vec3_t v2e, double *t1, double *t2, vec3_t DV1,
                              vec3_t DV2)
{
   double SMA1, SMA2, ecc, inc, RAAN, ArgP, anom1, anom2, tp, p, alpha;
   double AmP[2], AmParm[19], DeltaV;
   double MeanMotion, Period, rmin;
   long i;

   RV2Eph(0.0, mu, r1e, v1e, &SMA1, &ecc, &inc, &RAAN, &ArgP, &anom1, &tp, &p,
          &alpha, &rmin, &MeanMotion, &Period);
   RV2Eph(0.0, mu, r2e, v2e, &SMA2, &ecc, &inc, &RAAN, &ArgP, &anom2, &tp, &p,
          &alpha, &rmin, &MeanMotion, &Period);
   AmP[0] = 0.0;
   AmP[1] =
       3.1416 * sqrt(sqrt(fabs(SMA1 * SMA1 * SMA1 * SMA2 * SMA2 * SMA2)) / mu);
   AmParm[0] = mu;
   for (i = 0; i < 3; i++) {
      AmParm[1 + i]  = r1e.v[i];
      AmParm[4 + i]  = v1e.v[i];
      AmParm[7 + i]  = r2e.v[i];
      AmParm[10 + i] = v2e.v[i];
   }
   DeltaV = Amoeba(2, AmP, RendezvousCostFunction, AmParm, 0.1 * SMA1, 1.0E-5);
   *t1    = AmP[0];
   *t2    = AmP[0] + AmP[1];
   for (i = 0; i < 3; i++) {
      DV1.v[i] = AmParm[13 + i];
      DV2.v[i] = AmParm[16 + i];
   }
   printf("t1 = %7.2lf   TOF = %7.2lf  DV = %7.2lf\n", *t1, *t2 - *t1, DeltaV);
   printf("DV1: %7.2lf %7.2lf %7.2lf\n", DV1.x, DV1.y, DV1.z);
   printf("DV2: %7.2lf %7.2lf %7.2lf\n", DV2.x, DV2.y, DV2.z);
}
/**********************************************************************/
/*  Given the orbits of an Observer and a Target, find:               */
/*     PastPos: Location of Target when it sent the photons that      */
/*              are reaching the Observer at the present instant.     */
/*     FuturePos: Location of Target when the photons that are        */
/*                leaving the Observer at the present instant         */
/*                will arrive.                                        */
/*  Two iterations gives < mm accuracy for GEO-LEO distances.         */
/*  Will need more iterations for interplanetary-scale applications.  */
void FindLightLagOffsets(double dyntime, struct OrbitType *Observer,
                         struct OrbitType *Target, vec3_t PastPos,
                         vec3_t FuturePos __attribute__((unused)))
{
   vec3_t RelPos, Vel;
   double dt, anom;
   long i;

   /* .. Past */
   for (i = 0; i < 3; i++)
      RelPos.v[i] = Target->PosN.v[i] - Observer->PosN.v[i];
   dt = MAGV(RelPos) / SPEED_OF_LIGHT;
   Eph2RV(Target->mu, Target->SLR, Target->ecc, Target->inc, Target->RAAN,
          Target->ArgP, dyntime - dt - Target->tp, &PastPos, &Vel, &anom);

   for (i = 0; i < 3; i++)
      RelPos.v[i] = PastPos.v[i] - Observer->PosN.v[i];
   dt = MAGV(RelPos) / SPEED_OF_LIGHT;
   Eph2RV(Target->mu, Target->SLR, Target->ecc, Target->inc, Target->RAAN,
          Target->ArgP, dyntime - dt - Target->tp, &PastPos, &Vel, &anom);

   /* .. Future */
   for (i = 0; i < 3; i++)
      RelPos.v[i] = Target->PosN.v[i] - Observer->PosN.v[i];
   dt = MAGV(RelPos) / SPEED_OF_LIGHT;
   Eph2RV(Target->mu, Target->SLR, Target->ecc, Target->inc, Target->RAAN,
          Target->ArgP, dyntime + dt - Target->tp, &PastPos, &Vel, &anom);

   for (i = 0; i < 3; i++)
      RelPos.v[i] = PastPos.v[i] - Observer->PosN.v[i];
   dt = MAGV(RelPos) / SPEED_OF_LIGHT;
   Eph2RV(Target->mu, Target->SLR, Target->ecc, Target->inc, Target->RAAN,
          Target->ArgP, dyntime + dt - Target->tp, &PastPos, &Vel, &anom);
}
/**********************************************************************/
/* Ref: Markley and Crassidis, 10.4.3                                 */
/* Osculating elements drift from initial conditions due to J2        */
/* Use this function to initialize mean eph at sim start              */
void OscEphToMeanEph(double mu, double J2, double Rw, JDType jd,
                     struct OrbitType *O)
{
   double e, e2, sin2i, sinw, sin2w, cosnu, g, E;
   double a, p, p2, Coef;

   const double tt_j2000_sec_0 = JDToDynTime(jd);

   sin2i = sin(O->inc) * sin(O->inc);

   /* 10.127 */
   e     = O->ecc;
   e2    = e * e;
   sinw  = sin(O->ArgP + O->anom);
   sin2w = sinw * sinw;
   cosnu = cos(O->anom);
   g = pow((1.0 + e * cosnu) / (1.0 - e2), 3.0) * (1.0 - 3.0 * sin2i * sin2w);

   /* 10.128 */
   a          = O->SMA;
   O->MeanSMA = 0.5 * (a + sqrt(a * a - 4.0 * J2 * Rw * Rw * g));

   /* 10.123 */
   O->MeanMotion = sqrt(mu / O->MeanSMA) / O->MeanSMA;
   O->Period     = TWOPI / O->MeanMotion;

   /* 10.121 */
   p          = O->MeanSMA * (1.0 - O->ecc * O->ecc);
   p2         = p * p;
   Coef       = 1.5 * J2 * Rw * Rw / p2 * O->MeanMotion;
   O->RAANdot = -Coef * cos(O->inc);
   O->ArgPdot = Coef * (2.0 - 2.5 * sin2i);

   O->RAAN0 = O->RAAN - O->RAANdot * (tt_j2000_sec_0 - O->Epoch);
   O->ArgP0 = O->ArgP - O->ArgPdot * (tt_j2000_sec_0 - O->Epoch);

   /* 10.126 */
   O->J2Rw2bya = J2 * Rw * Rw / O->MeanSMA;

   E = atan2(sqrt(1.0 - O->ecc * O->ecc) * sin(O->anom), O->ecc + cos(O->anom));
   O->MeanAnom = E - O->ecc * sin(E);
   O->MeanAnom0 =
       fmod(O->MeanAnom - O->MeanMotion * (tt_j2000_sec_0 - O->Epoch), TWOPI);
}
/* #ifdef __cplusplus
** }
** #endif
*/

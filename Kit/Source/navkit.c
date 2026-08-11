/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "navkit.h"
#include "42.h"
#include "earthorikit.h"
#include "spicekit.h"

/* REQUIRED GLOBALS                                                   */
/*    WorldType World                                                 */
/*    long AtmoOption                                                 */
/*    double SchattenTable[5][410]                                    */

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

extern double EnckeFQ(double const r[3], double const delta[3]);
extern void Legendre(const long N, const long M, const double x,
                     double P[N + 1][M + 1], double sdP[N + 1][M + 1]);
#ifdef _REPORT_RESIDUALS_
extern void DSM_NAV_ResidualsReport(double time,
                                    double **residuals[FIN_SENSOR + 1]);
#endif

void InitMeasList(struct DSMMeasListType *list)
{
   list->head    = NULL;
   list->length  = 0;
   list->measDim = 0;
}

void appendMeas(struct DSMMeasListType *list, struct DSMMeasType *newMeas)
{
   struct DSMMeasType *last = list->head;
   if (newMeas == NULL) {
      return;
   }
   else if (last == NULL) {
      list->head = newMeas;
   }
   else {
      while (last->nextMeas != NULL) {
         last = last->nextMeas;
      }
      last->nextMeas = newMeas;
   }

   list->length  += 1;
   list->measDim += newMeas->errDim;
}

void appendList(struct DSMMeasListType *list1, struct DSMMeasListType *list2)
{
   if (list2 == NULL)
      return;
   struct DSMMeasType *last = list1->head;
   if (list2->head == NULL) {
      return;
   }
   else if (last == NULL) {
      list1->head = list2->head;
   }
   else {
      while (last->nextMeas != NULL) {
         last = last->nextMeas;
      }
      last->nextMeas = list2->head;
   }

   list1->length  += list2->length;
   list1->measDim += list2->measDim;
}

void DestroyMeas(struct DSMMeasType *meas)
{
   free(meas->data);
   free(meas->R);
   DestroyMatrix(meas->N);
   free(meas);
   meas = NULL;
}

void push(struct DSMMeasListType *list, struct DSMMeasType *meas)
{
   struct DSMMeasType *last = meas;
   long measDim             = last->errDim;
   long length              = 1;
   while (last->nextMeas != NULL) {
      last     = last->nextMeas;
      length  += 1;
      measDim += last->errDim;
   }

   last->nextMeas  = list->head;
   list->head      = meas;
   list->length   += length;
   list->measDim  += measDim;
}

struct DSMMeasType *pop_DSMMeas(struct DSMMeasListType *list)
{
   if (list->head == NULL)
      return NULL;
   struct DSMMeasType *meas  = list->head;
   list->head                = meas->nextMeas;
   meas->nextMeas            = NULL;
   list->length             -= 1;
   list->measDim            -= meas->errDim;
   return meas;
}

void DestroyMeasList(struct DSMMeasListType *list)
{
   struct DSMMeasType *meas = pop_DSMMeas(list);
   while (meas != NULL) {
      DestroyMeas(meas);
      meas = pop_DSMMeas(list);
   }
}

struct DSMMeasType *swap_DSMMeas(struct DSMMeasType *ptr1,
                                 struct DSMMeasType *ptr2)
{
   struct DSMMeasType *tmp = ptr2->nextMeas;
   ptr2->nextMeas          = ptr1;
   ptr1->nextMeas          = tmp;
   return ptr2;
}

// Measurements lists shouldn't be that long, maybe 20 or so per call, so bubble
// sort should be fine Likely won't need to sort anything anyway do to how lists
// are populated
void bubbleSort(struct DSMMeasListType *list)
{
   const long count = list->length;
   struct DSMMeasType **node;
   long i, j, swapped;

   for (i = 0; i <= count; i++) {
      node    = &list->head;
      swapped = 0;

      for (j = 0; j < count - i - 1; j++) {
         struct DSMMeasType *p1 = *node;
         struct DSMMeasType *p2 = p1->nextMeas;

         if (comparator_DSMMeas(&p1, &p2) == +1) {
            /* update the link after swapping */
            *node   = swap_DSMMeas(p1, p2);
            swapped = 1;
         }
         node = &(*node)->nextMeas;
      }

      /* break if the loop ended without any swap */
      if (swapped == 0)
         break;
   }
}

struct DSMMeasType *CreateMeas(struct DSMNavType *const Nav,
                               enum SensorType const type, long const sensorNum)
{
   struct DSMMeasType *meas, *sourceMeas = &Nav->measTypes[type][sensorNum];
   meas                  = malloc(sizeof *meas);
   meas->measFun         = sourceMeas->measFun;
   meas->measJacobianFun = sourceMeas->measJacobianFun;
   meas->type            = sourceMeas->type;
   meas->dim             = sourceMeas->dim;
   meas->errDim          = sourceMeas->errDim;
   meas->noiseDim        = sourceMeas->noiseDim;

   meas->data = calloc(meas->dim, sizeof(double));
   meas->R    = calloc(meas->noiseDim, sizeof(double));
   meas->N    = CreateMatrix(meas->errDim, meas->noiseDim);
   for (int i = 0; i < meas->errDim; i++) {
      meas->R[i] = sourceMeas->R[i];
      for (int j = 0; j < meas->noiseDim; j++)
         meas->N[i][j] = sourceMeas->N[i][j];
   }
   meas->sensorNum = sensorNum;
   meas->nextMeas  = NULL;
   return (meas);
}

//------------------------------------------------------------------------------
// Used to order the array of measurements
//------------------------------------------------------------------------------
int comparator_DSMMeas(const void *v1, const void *v2)
{
   const struct DSMMeasType *m1 = *(struct DSMMeasType **)v1;
   const struct DSMMeasType *m2 = *(struct DSMMeasType **)v2;
   if (m1->ccsds_time.coarse < m2->ccsds_time.coarse)
      return -1;
   else if (m1->ccsds_time.coarse > m2->ccsds_time.coarse)
      return +1;
   else if (m1->ccsds_time.fine < m2->ccsds_time.fine)
      return -1;
   else if (m1->ccsds_time.fine > m2->ccsds_time.fine)
      return +1;
   else if (m1->type < m2->type)
      return -1;
   else if (m1->type > m2->type)
      return +1;
   else if (m1->sensorNum < m2->sensorNum)
      return -1;
   else if (m1->sensorNum > m2->sensorNum)
      return +1;
   else
      return 0;
}

// Take in GPS time information, output time since J2000 TT
double gpsTime2J2000Sec(long const gpsRollover, long const gpsWk,
                        double const gpsSec)
{
   const double secPerDay       = 86400.0;
   const double dayperWk        = 7.0;
   const double daysperRollover = 7168.0;
   const double gpst0J2000      = -7300.5; // -7300.499407592695

   const double DaysSinceRollover = dayperWk * gpsWk;
   const double DaysSinceEpoch =
       DaysSinceRollover + daysperRollover * gpsRollover;
   return ((DaysSinceEpoch + gpst0J2000) * secPerDay) + gpsSec + (32.184 + 19);
}
/**********************************************************************/
/* Given a time in seconds since J2000 TT, find the orientation of    */
/* the world fixed frame relative to the world's inertial frame       */
mat3x3_t NavGetWorldCWN(const long orbCenter, const DateType date)
    __attribute__((pure));
mat3x3_t NavGetWorldCWN(const long orbCenter, const DateType date)
{
   // TODO: don't use worlds and getworldCWN directly
   struct WorldType *W = &World[orbCenter];
   JDType jd           = Date2JD(date, J2000_EPOCH);
   mat3x3_t CWN        = MAT3X3_EYE;

   switch (orbCenter) {
      case EARTH: {
         if (EphemOption == EPH_SPICE)
            CWN = SpiceGetCWJ(jd, EARTH);
         else {
            /* .. Earth rotation is a special case */
            mat3x3_t C_TETE_J2000, C_W_TETE;
            JDType jd_tt_j2000 = JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, jd);

            // double PriMerAng         = TwoPi * JD2GMST(jd_tt_j2000);
            // const pair_mat3x3_t pair = HiFiEarthPrecNute(jd_tt_j2000);
            // CWN                      = pair.first;
            // C_TETE_J2000             = pair.second;
            // C_W_TETE                 = SimpRot(ZAxis, PriMerAng);
            // CWN                      = MxM(C_W_TETE, C_TETE_J2000);
            CWN = HiFiEarthCWN(jd_tt_j2000).mat;
         }
      } break;
      default:
         CWN = GetWorldCWN(jd, W->ang_data).mat;
         break;
   }
   return CWN;
}
//------------------------------------------------------------------------------
// Acceleration perturbation functions
//------------------------------------------------------------------------------
mat3x3_t SphericalHarmonicsJacobian(const long N, const long M,
                                    const sphere_coord_t coord, const double Re,
                                    const double K, double **C, double **S,
                                    double **Norm)
{
   double P[N + 1][M + 1], sdP[N + 1][M + 1];
   long n, m;
   double cphi[M + 1], sphi[M + 1];
   double Rern1[N + 1];

   const double r   = coord.r;
   const double cth = coord.cth;
   const double sth = coord.sth;

   /* .. Order can't be greater than Degree */
   if (M > N) {
      fprintf(stderr, "Order %ld can't be greater than Degree %ld\n", M, N);
      exit(EXIT_FAILURE);
   }

   /* .. Find Legendre functions */
   const double sth2  = sth * sth;
   const double cotth = cth / sth;
   const double r2 = r * r, rsth = r * sth;
   const double rsth2 = rsth * rsth;
   Legendre(N, M, cth, P, sdP);

   /* .. Build cos(m*phi) and sin(m*phi) */
   cphi[0] = 1.0;
   sphi[0] = 0.0;
   cphi[1] = coord.cph; // cos(phi);
   sphi[1] = coord.sph; // sin(phi);
   for (m = 2; m <= M; m++) {
      cphi[m] = cphi[m - 1] * cphi[1] - sphi[m - 1] * sphi[1];
      sphi[m] = sphi[m - 1] * cphi[1] + cphi[m - 1] * sphi[1];
   }

   double d2Vdr2 = 0.0, d2Vdphi2 = 0.0, d2Vdtheta2 = 0.0, d2Vdrdphi = 0.0,
          d2Vdrdtheta = 0.0, d2Vdphidtheta = 0.0;
   /* .. Find Jacobian of V */
   /* .. Rern1[n] = (Re/r)^(n+1) */
   Rern1[0] = Re / r;
   for (n = 1; n <= N; n++)
      Rern1[n] = Rern1[n - 1] * Rern1[0];
   for (n = N; n >= 2; n--) {
      for (m = MIN(n, M); m >= 0; m--) {
         const double Pbar   = P[n][m] * Norm[n][m];
         const double sdPbar = sdP[n][m] * Norm[n][m];
         const double CcSsbar =
             (C[n][m] * cphi[m] + S[n][m] * sphi[m]) * Rern1[n];
         const double ScCsbar =
             (S[n][m] * cphi[m] - C[n][m] * sphi[m]) * Rern1[n];

         double tmp = (double)(n * (n + 1));
         if (m != 0 && sth != 0.0)
            tmp -= (m * m) / sth2;
         tmp *= Pbar;
         if (sth != 0.0)
            tmp -= sdPbar * cotth;
         d2Vdr2        += CcSsbar * (Pbar * ((n + 1) * (n + 2)));
         d2Vdtheta2    -= CcSsbar * tmp;
         d2Vdphi2      -= CcSsbar * (Pbar * (m * m));
         d2Vdrdtheta   += CcSsbar * (sdPbar * (n + 1));
         d2Vdrdphi     -= ScCsbar * (Pbar * ((n + 1) * m));
         d2Vdphidtheta -= ScCsbar * (sdPbar * m);
      }
   }
   const double Kr  = K / r;
   d2Vdr2          *= Kr / r;
   d2Vdphi2        *= K;
   d2Vdtheta2      *= K;
   d2Vdrdphi       *= Kr;
   d2Vdrdtheta     *= Kr;
   d2Vdphidtheta   *= K;

   mat3x3_t HV;
   HV.mat[0][0] = d2Vdr2;
   HV.mat[1][1] = d2Vdtheta2 / r2;
   HV.mat[2][2] = d2Vdphi2 / rsth2;
   HV.mat[0][1] = d2Vdrdtheta / r;
   HV.mat[0][2] = d2Vdrdphi / rsth;
   HV.mat[1][2] = d2Vdphidtheta / (r * rsth);
   HV.mat[1][0] = HV.mat[0][1];
   HV.mat[2][0] = HV.mat[0][2];
   HV.mat[2][1] = HV.mat[1][2];
   return HV;
}

mat3x3_t SphericalHarmonicsHessian(long N, long M, struct WorldType *W,
                                   mat3x3_t CWN, vec3_t pbn)
{
   long i, j, k;
   const struct SphereHarmType *GravModel = &W->GravModel;

   mat3x3_t HV;
   vec3_t gradV = VEC3_ZERO, pbw;

   /*    Transform p to ECEF */
   pbw = MxV(CWN, pbn);

   const double denom   = sqrt(pbw.y * pbw.y + pbw.x * pbw.x);
   sphere_coord_t coord = getTrigSphericalCoords(pbw);
   const double r       = coord.r;
   const double cth     = coord.cth;
   const double sth     = coord.sth;
   const double cph     = coord.cph;
   const double sph     = coord.sph;

   const mat3x3_t MSE = {.flat = {pbw.x / r, pbw.y / r, cth, cth * cph,
                                  cth * sph, -sth, -sph, cph, 0.0}};

   /*    Find Jacobian */
   HV = SphericalHarmonicsJacobian(N, M, coord, W->rad, W->mu / W->rad,
                                   GravModel->C, GravModel->S, GravModel->Norm);

   /*   Calculate scaled Christoffel Symbols */
   /*     sCS^k_{ij} = CS^k_{ij} * sqrt(g_{kk}) / (sqrt(g_{ii})*sqrt(g_{jj})) */
   /*     due to scaling of gradV and scaling in polar transform */
   mat3x3_t sCS[3] = {MAT3X3_ZERO};

   sCS[0].mat[1][1] = -1.0 / r;          // -r * 1 / (r*r) = -1 / r
   sCS[0].mat[2][2] = sCS[0].mat[1][1];  // -rsth*sth * 1 / (rsth*rsth) = -1/r
   sCS[1].mat[0][1] = -sCS[0].mat[1][1]; // 1/r * r / (1*r) = 1/r
   sCS[1].mat[1][0] = sCS[1].mat[0][1];
   sCS[1].mat[2][2] =
       -pbw.z / (r * denom); // -sth*cth * r / (rsth*rsth) = -cth / rsth
   sCS[2].mat[0][2] = sCS[1].mat[0][1]; // 1/r * rsth / (1*rsth) = 1 / r
   sCS[2].mat[1][2] =
       -sCS[1].mat[2][2]; // cth/sth * rsth / (r*rsth) = cth / rsth
   sCS[2].mat[2][0] = sCS[2].mat[0][2];
   sCS[2].mat[2][1] = sCS[2].mat[1][2];

   gradV = SphericalHarmonics(N, M, coord, W->rad, W->mu / W->rad, GravModel->C,
                              GravModel->S, GravModel->Norm);
   for (k = 0; k < 3; k++)
      for (i = 0; i < 3; i++)
         for (j = 0; j < 3; j++)
            HV.mat[i][j] -= gradV.v[k] * sCS[k].mat[i][j];

   /*    Transform back to cartesian coords in Newtonian frame */
   mat3x3_t CSN = MxM(MSE, CWN);
   return AdjointT(CSN, HV);
}

vec3_t getGravAccel(const double mu, const vec3_t pos) __attribute__((const));
vec3_t getGravAccel(const double mu, const vec3_t pos)
{
   int i;
   vec3_t posHat, gravFrc;

   const magvec3_t uv     = UNITV(pos);
   const double posMag    = uv.m;
   posHat                 = uv.v;
   const double gravScale = -mu / (posMag * posMag);
   for (i = 0; i < 3; i++)
      gravFrc.v[i] = posHat.v[i] * gravScale;
   return gravFrc;
}

mat3x3_t getDGravFrcDPos(const double mu, const vec3_t pos)
    __attribute__((const));
mat3x3_t getDGravFrcDPos(const double mu, const vec3_t pos)
{
   int i, j;
   magvec3_t uposHat;
   uposHat.v           = pos;
   uposHat             = UNITV(uposHat.v);
   const double posMag = uposHat.m;
   const vec3_t posHat = uposHat.v;

   const double gravScale = -mu / (posMag * posMag * posMag);

   mat3x3_t dGravFrcdPos = MAT3X3_EYE;
   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         dGravFrcdPos.mat[i][j] += -posHat.v[i] * posHat.v[j] * 3.0;
   return SxM(gravScale, dGravFrcdPos);
}

vec3_t ThirdBodyGravAccel(vec3_t p, vec3_t s, double mu) __attribute__((const));
vec3_t ThirdBodyGravAccel(vec3_t p, vec3_t s, double mu)
{
   const double magp = MAGV(p);
   const double mags = MAGV(s);
   const double p3   = magp * magp * magp;
   const double s3   = mags * mags * mags;
   vec3_t accel;
   for (long j = 0; j < 3; j++)
      accel.v[j] = mu * (s.v[j] / s3 - p.v[j] / p3);
   return accel;
}

vec3_t NavGravPertAccel(struct DSMNavType *Nav, const DateType *date,
                        const vec3_t PosR, const double mass,
                        const struct OrbitType *O) __attribute__((pure));
vec3_t NavGravPertAccel(struct DSMNavType *Nav, const DateType *date,
                        const vec3_t PosR, const double mass,
                        const struct OrbitType *O)
{
   vec3_t VelRdot = VEC3_ZERO;
   vec3_t ph, pn, pr, s, accelR;
   long Iw, Im, j;
   long OrbCenter, SecCenter;

   if (O->Regime == ORB_CENTRAL) {
      OrbCenter = O->World;
      SecCenter = -1; /* Nonsense value */
   }
   else {
      OrbCenter = O->Body1;
      SecCenter = O->Body2;
   }

   struct WorldType *WCenter = &World[OrbCenter];

   if (GravPert.ThirdBody) {
      for (Iw = SOL; Iw <= PLUTO; Iw++) {
         if (World[Iw].Exists && !(Iw == OrbCenter || Iw == SecCenter)) {
            for (j = 0; j < 3; j++)
               ph.v[j] = World[Iw].PosH.v[j] - WCenter->PosH.v[j];
            pn = MxV(WCenter->CNH, ph);
            pr = MxV(Nav->refCRN, pn);
            for (j = 0; j < 3; j++)
               s.v[j] = pr.v[j] - PosR.v[j];
            accelR = ThirdBodyGravAccel(pr, s, World[Iw].mu);
            for (j = 0; j < 3; j++)
               VelRdot.v[j] += accelR.v[j];
         }
      }
      /* Moons of OrbCenter (but not SecCenter) */
      if (OrbCenter != SOL) {
         for (Im = 0; Im < WCenter->Nsat; Im++) {
            Iw = WCenter->Sat[Im];
            if (Iw != SecCenter) {
               pr = MxV(Nav->refCRN, World[Iw].eph.PosN);
               for (j = 0; j < 3; j++)
                  s.v[j] = pr.v[j] - PosR.v[j];
               accelR = ThirdBodyGravAccel(pr, s, World[Iw].mu);
               for (j = 0; j < 3; j++)
                  VelRdot.v[j] += accelR.v[j];
            }
         }
      }
      /* Moons of SecCenter */
      if (O->Regime == ORB_THREE_BODY) {
         for (Im = 0; Im < World[SecCenter].Nsat; Im++) {
            Iw = World[SecCenter].Sat[Im];
            ph = MTxV(World[SecCenter].CNH, World[Iw].eph.PosN);
            pn = MxV(WCenter->CNH, ph);
            for (j = 0; j < 3; j++)
               pn.v[j] += World[SecCenter].eph.PosN.v[j];
            pr = MxV(Nav->refCRN, pn);
            for (j = 0; j < 3; j++)
               s.v[j] = pr.v[j] - PosR.v[j];

            accelR = ThirdBodyGravAccel(pr, s, World[Iw].mu);
            for (j = 0; j < 3; j++)
               VelRdot.v[j] += accelR.v[j];
         }
      }
   }

   // TODO: maybe make this just 2/0 if it exists
   /* Perturbations due to non-spherical gravity potential */
   if (GravPert.Harmonic) {
      const struct SphereHarmType *GravModel = &WCenter->GravModel;
      if (GravModel->N >= 2) {
         mat3x3_t CWN = NavGetWorldCWN(OrbCenter, *date);
         vec3_t fGeoN, fGeoR, PosN;
         PosN  = MTxV(Nav->refCRN, PosR);
         fGeoN = SphericalHarmGravForce(GravModel->N, GravModel->M, WCenter,
                                        CWN, mass, PosN);
         fGeoR = MxV(Nav->refCRN, fGeoN);
         for (j = 0; j < 3; j++)
            VelRdot.v[j] += fGeoR.v[j] / mass;
      }
   }
   return VelRdot;
}

mat3x3_t NavDGravPertAccelDPos(struct DSMNavType *Nav, const DateType *date,
                               vec3_t PosR, struct OrbitType const *O)
{
   mat3x3_t dGravDPos = MAT3X3_ZERO, dGdR;
   vec3_t ph, pn, pr, s;
   long Iw, Im, i, j;
   long OrbCenter, SecCenter;

   if (O->Regime == ORB_CENTRAL) {
      OrbCenter = O->World;
      SecCenter = -1; /* Nonsense value */
   }
   else {
      OrbCenter = O->Body1;
      SecCenter = O->Body2;
   }
   struct WorldType *WCenter = &World[OrbCenter];

   if (GravPert.ThirdBody) {
      for (Iw = SOL; Iw <= PLUTO; Iw++) {
         if (World[Iw].Exists && !(Iw == OrbCenter || Iw == SecCenter)) {
            for (j = 0; j < 3; j++)
               ph.v[j] = World[Iw].PosH.v[j] - WCenter->PosH.v[j];
            pn = MxV(WCenter->CNH, ph);
            pr = MxV(Nav->refCRN, pn);
            for (j = 0; j < 3; j++)
               s.v[j] = pr.v[j] - PosR.v[j];
            dGdR = getDGravFrcDPos(World[Iw].mu, s);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  dGravDPos.mat[i][j] += dGdR.mat[i][j];
         }
      }
      /* Moons of OrbCenter (but not SecCenter) */
      if (OrbCenter != SOL) {
         for (Im = 0; Im < WCenter->Nsat; Im++) {
            Iw = WCenter->Sat[Im];
            if (Iw != SecCenter) {
               pr = MxV(Nav->refCRN, World[Iw].eph.PosN);
               for (j = 0; j < 3; j++)
                  s.v[j] = pr.v[j] - PosR.v[j];
               dGdR = getDGravFrcDPos(World[Iw].mu, s);
               for (i = 0; i < 3; i++)
                  for (j = 0; j < 3; j++)
                     dGravDPos.mat[i][j] += dGdR.mat[i][j];
            }
         }
      }
      /* Moons of SecCenter */
      if (O->Regime == ORB_THREE_BODY) {
         for (Im = 0; Im < World[SecCenter].Nsat; Im++) {
            Iw = World[SecCenter].Sat[Im];
            ph = MTxV(World[SecCenter].CNH, World[Iw].eph.PosN);
            pn = MxV(WCenter->CNH, ph);
            for (j = 0; j < 3; j++)
               pn.v[j] += World[SecCenter].eph.PosN.v[j];
            pr = MxV(Nav->refCRN, pn);
            for (j = 0; j < 3; j++)
               s.v[j] = pr.v[j] - PosR.v[j];

            dGdR = getDGravFrcDPos(World[Iw].mu, s);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  dGravDPos.mat[i][j] += dGdR.mat[i][j];
         }
      }
   }

   // TODO: maybe make this just 2/0 if it exists
   /* Perturbations due to non-spherical gravity potential */
   if (GravPert.Harmonic) {
      const struct SphereHarmType *GravModel = &WCenter->GravModel;
      if (GravModel->N >= 2) {
         mat3x3_t CWN, HgeoN, HgeoR;
         vec3_t PosN;
         CWN   = NavGetWorldCWN(OrbCenter, *date);
         PosN  = MTxV(Nav->refCRN, PosR);
         HgeoN = SphericalHarmonicsHessian(GravModel->N, GravModel->M, WCenter,
                                           CWN, PosN);
         if (Nav->refFrame != FRAME_N) {
            HgeoR = Adjoint(Nav->refCRN, HgeoN);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  dGravDPos.mat[i][j] += HgeoR.mat[i][j];
         }
         else {
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  dGravDPos.mat[i][j] += HgeoN.mat[i][j];
         }
      }
   }
   return dGravDPos;
}

void getAeroForceAndTorque(struct DSMType *const DSM,
                           const mat3x3_t CRB __attribute__((unused)),
                           const vec3_t PosR, const vec3_t VelR,
                           double const worldW, double const AtmoDensity,
                           vec3_t *frcR, vec3_t *trq)
{
   // TODO: be able to choose between ballistic coef model and more accurate
   // model basllistic coef is noticeably faster and simplification doesn't
   // change much if torque is trivial
   // Higher fidelity model requires information that exists only in SCType

   const struct DSMNavType *Nav = &DSM->DsmNav;
   vec3_t worldWR, VrelR, PosRWorld, VrelRHat;
   for (int i = 0; i < 3; i++)
      PosRWorld.v[i] = PosR.v[i] + Nav->refPos.v[i];
   for (int i = 0; i < 3; i++)
      worldWR.v[i] = -Nav->refCRN.mat[i][2] * worldW;
   VrelR = VxV(worldWR, PosRWorld);
   for (int i = 0; i < 3; i++)
      VrelR.v[i] += VelR.v[i] + Nav->refVel.v[i];

   const magvec3_t uv     = UNITV(VrelR);
   const double WindSpeed = uv.m;
   VrelRHat               = uv.v;
   const double Coef1 = -0.5 * AtmoDensity * WindSpeed * WindSpeed * DSM->mass /
                        Nav->ballisticCoef;
   for (int i = 0; i < 3; i++)
      frcR->v[i] = Coef1 * VrelRHat.v[i];
   *trq = VEC3_ZERO;
}

void getDAeroFrcAndTrqDVRel(struct DSMType *const DSM,
                            const mat3x3_t CRB __attribute__((unused)),
                            const vec3_t PosR, const vec3_t VelR,
                            const double worldW, const double AtmoDensity,
                            mat3x3_t *const dAeroFrcdVRel,
                            mat3x3_t *const dAeroTrqdVRel)
{
   // TODO: be able to choose between ballistic coef model and more accurate
   // model basllistic coef is noticeably faster and simplification doesn't
   // change much if torque is trivial
   // Higher fidelity model requires information that exists only in SCType

   const struct DSMNavType *Nav = &DSM->DsmNav;
   vec3_t worldWR, VrelR, PosRWorld, VrelRHat;
   for (int i = 0; i < 3; i++)
      PosRWorld.v[i] = PosR.v[i] + Nav->refPos.v[i];
   for (int i = 0; i < 3; i++)
      worldWR.v[i] = -Nav->refCRN.mat[i][2] * worldW;
   VrelR = VxV(worldWR, PosRWorld);
   for (int i = 0; i < 3; i++)
      VrelR.v[i] += VelR.v[i] + Nav->refVel.v[i];

   const magvec3_t uv     = UNITV(VrelR);
   const double WindSpeed = uv.m;
   VrelRHat               = uv.v;
   const double Coef1 =
       -0.5 * AtmoDensity * WindSpeed * DSM->mass / Nav->ballisticCoef;

   *dAeroTrqdVRel = MAT3X3_ZERO;
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++)
         dAeroFrcdVRel->mat[i][j] = VrelRHat.v[i] * VrelRHat.v[j] * Coef1;
      dAeroFrcdVRel->mat[i][i] += Coef1;
   }
}

//------------------------------------------------------------------------------
//                               NAV FUNCTIONS
//------------------------------------------------------------------------------

double **gyroJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                         const long Igyro, double **N __attribute__((unused)))
{
   vec3_t tmp, tmp2;
   static double **B = NULL; // if its static, just need to allocate once,
                             // instead of allocate/deallocate
   const struct DSMNavType *Nav  = &DSM->DsmNav;
   const struct AcGyroType *gyro = &AC->Gyro[Igyro];
   long i;

   if (B == NULL)
      B = CreateMatrix(1, 3);

   for (i = 0; i < 3; i++)
      B[0][i] = 0.0;

   double **jacobian =
       CreateMatrix(Nav->measTypes[GYRO_SENSOR][Igyro].dim, Nav->navDim);

   switch (Nav->type) {
      case LIEKF_NAV:
         tmp = MTxV(Nav->CRB, Nav->refOmega);
         for (i = 0; i < 3; i++)
            tmp.v[i] += Nav->wbr.v[i];
         tmp2 = VxV(tmp, gyro->Axis);

         for (i = 0; i < 3; i++)
            B[0][i] = tmp2.v[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
         for (i = 0; i < 3; i++)
            B[0][i] = -gyro->Axis.v[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[OMEGA_STATE], 1, 3);
         break;
      case RIEKF_NAV: {
         vec3_t axisR = MxV(Nav->CRB, gyro->Axis);
         for (i = 0; i < 3; i++)
            B[0][i] = -axisR.v[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[OMEGA_STATE], 1, 3);

         tmp2 = VxV(Nav->refOmega, axisR);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp2.v[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
      } break;
      case MEKF_NAV:
         for (i = 0; i < 3; i++)
            B[0][i] = -gyro->Axis.v[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[OMEGA_STATE], 1, 3);
         if (Nav->refFrame != FRAME_N) {
            tmp  = QxV(Nav->qbr, Nav->refOmega);
            tmp2 = VxV(tmp, gyro->Axis);

            for (i = 0; i < 3; i++)
               B[0][i] = tmp2.v[i] * R2D;
            subMatAdd(jacobian, B, 0, Nav->navInd[QUAT_STATE], 1, 3);
         }
         break;
      default:
         fprintf(stderr, "Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double **magJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                        const long Imag, double **N __attribute__((unused)))
{
   vec3_t tmp, tmp2;
   static double **B = NULL; // if its static, just need to allocate once,
                             // instead of allocate/deallocate
   const struct DSMNavType *Nav         = &DSM->DsmNav;
   const struct AcMagnetometerType *mag = &AC->MAG[Imag];
   const double T2mG                    = 1.0e7; // tesla to milligauss
   long i;

   if (B == NULL)
      B = CreateMatrix(1, 3);

   for (i = 0; i < 3; i++)
      B[0][i] = 0.0;

   double **jacobian =
       CreateMatrix(Nav->measTypes[MAG_SENSOR][Imag].dim, Nav->navDim);

   switch (Nav->type) {
      case LIEKF_NAV:
         tmp  = MxV(Nav->refCRN, AC->bvn);
         tmp2 = MTxV(Nav->CRB, tmp);
         tmp  = VxV(tmp2, mag->Axis);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp.v[i] * T2mG;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
         break;
      case RIEKF_NAV: {
         tmp2         = MxV(Nav->refCRN, AC->bvn);
         vec3_t axisR = MxV(Nav->CRB, mag->Axis);
         tmp          = VxV(tmp2, axisR);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp.v[i] * T2mG;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
      } break;
      case MEKF_NAV:
         tmp  = MxV(Nav->refCRN, AC->bvn);
         tmp2 = QxV(Nav->qbr, tmp);
         tmp  = VxV(tmp2, mag->Axis);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp.v[i] * T2mG;
         subMatAdd(jacobian, B, 0, Nav->navInd[QUAT_STATE], 1, 3);
         break;
      default:
         fprintf(stderr, "Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double **cssJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                        const long Icss, double **N __attribute__((unused)))
{
   vec3_t tmp, svb, svr;
   static double **B = NULL; // if its static, just need to allocate once,
                             // instead of allocate/deallocate
   const struct DSMNavType *Nav = &DSM->DsmNav;
   const struct AcCssType *css  = &AC->CSS[Icss];
   long i;

   if (B == NULL)
      B = CreateMatrix(1, 3);

   for (i = 0; i < 3; i++)
      B[0][i] = 0.0;

   svr = MxV(Nav->refCRN, AC->svn);

   double **jacobian =
       CreateMatrix(Nav->measTypes[CSS_SENSOR][Icss].dim, Nav->navDim);

   switch (Nav->type) { // will need to figure something out with albedo if that
                        // is active
      case LIEKF_NAV:
         svb = MTxV(Nav->CRB, svr);
         tmp = VxV(svb, css->Axis);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp.v[i] * css->Scale;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
         break;
      case RIEKF_NAV:
         tmp = MxV(Nav->CRB, css->Axis);
         svb = VxV(svr, tmp);
         for (i = 0; i < 3; i++)
            B[0][i] = svb.v[i] * css->Scale;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
         break;
      case MEKF_NAV:
         svb = QxV(Nav->qbr, svr);
         tmp = VxV(svb, css->Axis);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp.v[i] * css->Scale;
         subMatAdd(jacobian, B, 0, Nav->navInd[QUAT_STATE], 1, 3);
         break;
      default:
         fprintf(stderr, "Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double **fssJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                        const long Ifss, double **N __attribute__((unused)))
{
   mat3x3_t B, tmp3x3, CBN;
   const struct AcFssType *fss  = &AC->FSS[Ifss];
   const struct DSMNavType *Nav = &DSM->DsmNav;
   static double **tmpAssign    = NULL;
   vec3_t svb, svs, bhat = VEC3_ZERO, hhat = VEC3_ZERO, vhat = VEC3_ZERO;
   vec3_t bxsvs, hxsvs, vxsvs;
   long i;

   const long BoreAxis = fss->BoreAxis;
   const long H_Axis   = fss->H_Axis;
   const long V_Axis   = fss->V_Axis;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(2, 3);

   memset(tmpAssign[0], 0, sizeof(double) * 6);

   CBN = MTxM(Nav->CRB, Nav->refCRN);
   svb = MxV(CBN, AC->svn);
   svs = MxV(fss->CB, svb);

   bhat.v[BoreAxis] = 1.0;
   hhat.v[H_Axis]   = 1.0;
   vhat.v[V_Axis]   = 1.0;

   bxsvs = VxV(bhat, svs);
   hxsvs = VxV(hhat, svs);
   vxsvs = VxV(vhat, svs);

   const double svsb = svs.v[BoreAxis];
   const double svsh = svs.v[H_Axis];
   const double svsv = svs.v[V_Axis];

   switch (fss->type) {
      case CONVENTIONAL_FSS: {
         const double denomA = 1.0 / (svsb * svsb + svsh * svsh);
         const double denomB = 1.0 / (svsb * svsb + svsv * svsv);
         for (i = 0; i < 3; i++) {
            B.mat[0][i] = (svsh * bxsvs.v[i] - svsb * hxsvs.v[i]) * denomA;
            B.mat[1][i] = (svsv * bxsvs.v[i] - svsb * vxsvs.v[i]) * denomB;
         }
      } break;
      case GS_FSS: {
         const double denomA = -1.0 / sqrt(1.0 - svsb * svsb);
         const double denomB = 1.0 / (svsh * svsh + svsv * svsv);
         for (i = 0; i < 3; i++) {
            B.mat[0][i] = bxsvs.v[i] * denomA;
            B.mat[1][i] = (svsh * vxsvs.v[i] - svsv * hxsvs.v[i]) * denomB;
         }
      } break;
      default:
         fprintf(stderr,
                 "Invalid FSS Type. How did it get this far? Exiting...\n");
         exit(EXIT_FAILURE);
   }

   double **jacobian =
       CreateMatrix(Nav->measTypes[FSS_SENSOR][Ifss].dim, Nav->navDim);

   switch (Nav->type) {
      case LIEKF_NAV:
         tmp3x3 = MxM(B, fss->CB);
         CopyVG(tmpAssign[0], tmp3x3.flat, 6);
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[ROTMAT_STATE], 2, 3);
         break;
      case RIEKF_NAV:
         tmp3x3 = MxM(B, fss->CB);
         B      = MxMT(tmp3x3, Nav->CRB);
         CopyVG(tmpAssign[0], B.flat, 6);
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[ROTMAT_STATE], 2, 3);
         break;
      case MEKF_NAV:
         tmp3x3 = MxM(B, fss->CB);
         CopyVG(tmpAssign[0], tmp3x3.flat, 6);
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[QUAT_STATE], 2, 3);
         break;
      default:
         fprintf(stderr, "Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double **startrackJacobianFun(struct AcType *const AC,
                              struct DSMType *const DSM, const long Ist,
                              double **N __attribute__((unused)))
{
   mat3x3_t tmpM, CSB;
   static double **tmpAssign          = NULL;
   const struct DSMNavType *Nav       = &DSM->DsmNav;
   const struct AcStarTrackerType *st = &AC->ST[Ist];

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   double **jacobian =
       CreateMatrix(Nav->measTypes[STARTRACK_SENSOR][Ist].errDim, Nav->navDim);
   CSB = Q2C(st->qb);

   switch (Nav->type) {
      case LIEKF_NAV:
      case MEKF_NAV:
         CopyVG(tmpAssign[0], SxM(-1.0, CSB).flat, 9);
         if (Nav->type == LIEKF_NAV)
            subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[ROTMAT_STATE], 3, 3);
         else
            subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[QUAT_STATE], 3, 3);
         break;
      case RIEKF_NAV:
         tmpM = MxMT(CSB, Nav->CRB);
         CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[ROTMAT_STATE], 3, 3);
         break;
      default:
         fprintf(stderr, "Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double **gpsJacobianFun(struct AcType *const AC __attribute__((unused)),
                        struct DSMType *const DSM, const long Igps,
                        double **N __attribute__((unused)))
{
   mat3x3_t tmp1, tmp2, tmp3, tmpX;
   vec3_t tmpV;
   static double **tmpAssign    = NULL;
   const struct DSMNavType *Nav = &DSM->DsmNav;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   double **jacobian =
       CreateMatrix(Nav->measTypes[GPS_SENSOR][Igps].dim, Nav->navDim);

   switch (Nav->type) {
      case LIEKF_NAV:
         tmp1 = Nav->CRB;
         if (Nav->refFrame != FRAME_N) {
            tmp1 = MTxM(Nav->refCRN, tmp1);

            tmpV = MTxV(Nav->refCRN, Nav->refOmega);
            tmpX = V2CrossM(tmpV);
            tmp2 = MTxM(Nav->refCRN, tmpX);
            tmpX = MxM(tmp2, tmp1);
            CopyVG(tmpAssign[0], SxM(-1.0, tmpX).flat, 9);
            subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[POS_STATE], 3, 3);
         }
         CopyVG(tmpAssign[0], SxM(-1.0, tmp1).flat, 9);
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[POS_STATE], 3, 3);
         subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[VEL_STATE], 3, 3);
         break;
      case RIEKF_NAV:
         tmp1 = Nav->refCRN;
         if (Nav->refFrame != FRAME_N) {
            tmpX = V2CrossM(Nav->refOmega);
            tmp2 = MTxM(Nav->refCRN, tmpX);
            CopyVG(tmpAssign[0], SxM(-1.0, tmp2).flat, 9);
            subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[POS_STATE], 3, 3);
         }
         CopyVG(tmpAssign[0], SxM(-1.0, tmp1).flat, 9);
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[POS_STATE], 3, 3);
         subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[VEL_STATE], 3, 3);

         tmpX = V2CrossM(Nav->PosR);
         tmp3 = MxM(tmp1, tmpX);
         CopyVG(tmpAssign[0], tmp3.flat, 9);
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[ROTMAT_STATE], 3, 3);
         if (Nav->refFrame != FRAME_N) {
            tmp3 = MxM(tmp2, tmpX);
            CopyVG(tmpAssign[0], tmp3.flat, 9);
            subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[ROTMAT_STATE], 3, 3);
         }

         tmpX = V2CrossM(Nav->VelR);
         tmp3 = MxM(tmp1, tmpX);
         CopyVG(tmpAssign[0], tmp3.flat, 9);
         subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[ROTMAT_STATE], 3, 3);
         break;
      case MEKF_NAV:
         tmp1 = Nav->refCRN;
         if (Nav->refFrame != FRAME_N) {
            tmpX = V2CrossM(Nav->refOmega);
            tmp2 = MTxM(Nav->refCRN, tmpX);
            CopyVG(tmpAssign[0], SxM(-1.0, tmp2).flat, 9);
            subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[POS_STATE], 3, 3);
         }
         CopyVG(tmpAssign[0], SxM(-1.0, tmp1).flat, 9);
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[POS_STATE], 3, 3);
         subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[VEL_STATE], 3, 3);
         break;
      default:
         fprintf(stderr, "Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double **accelJacobianFun(struct AcType *const AC __attribute__((unused)),
                          struct DSMType *const DSM __attribute__((unused)),
                          const long Iaccel __attribute__((unused)),
                          double **N __attribute__((unused)))
{
   const struct DSMNavType *Nav = &DSM->DsmNav;

   double **jacobian =
       CreateMatrix(Nav->measTypes[ACCEL_SENSOR][Iaccel].dim, Nav->navDim);

   switch (Nav->type) {
      case LIEKF_NAV:
         break;
      case RIEKF_NAV:
         break;
      case MEKF_NAV:
         break;
      default:
         fprintf(stderr, "Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double *gyroFun(struct AcType *const AC, struct DSMType *const DSM,
                const long Ig)
{
   const struct AcGyroType *G   = &AC->Gyro[Ig];
   const struct DSMNavType *Nav = &DSM->DsmNav;
   vec3_t wbn, wrn;
   long i;

   double *gyroEst = calloc(1, sizeof(double));

   wrn = MTxV(Nav->CRB, Nav->refOmega);
   for (i = 0; i < 3; i++)
      wbn.v[i] = Nav->wbr.v[i] + wrn.v[i];
   gyroEst[0] = VoV(G->Axis, wbn) * R2D;
   return (gyroEst);
}

double *magFun(struct AcType *const AC, struct DSMType *const DSM,
               const long Imag)
{
   const struct AcMagnetometerType *MAG = &AC->MAG[Imag];
   const struct DSMNavType *Nav         = &DSM->DsmNav;
   vec3_t bvb, bvn;
   mat3x3_t CBN;
   const double T2mG = 1.0e7; // tesla to milligauss

   double *magEst = calloc(1, sizeof(double));

   // Not to really be used.
   bvn = AC->bvn;

   CBN       = MTxM(Nav->CRB, Nav->refCRN);
   bvb       = MxV(CBN, bvn);
   magEst[0] = VoV(MAG->Axis, bvb) * T2mG;

   return (magEst);
}

double *cssFun(struct AcType *const AC, struct DSMType *const DSM,
               const long Icss)
{
   const struct AcCssType *css  = &AC->CSS[Icss];
   const struct DSMNavType *Nav = &DSM->DsmNav;
   vec3_t svb;
   mat3x3_t CBN;

   double *IllumEst = calloc(1, sizeof(double));

   CBN         = MTxM(Nav->CRB, Nav->refCRN);
   svb         = MxV(CBN, AC->svn);
   IllumEst[0] = VoV(svb, css->Axis) * css->Scale;

   return (IllumEst);
}

double *fssFun(struct AcType *const AC, struct DSMType *const DSM,
               const long Ifss)
{
   const struct AcFssType *fss  = &AC->FSS[Ifss];
   const struct DSMNavType *Nav = &DSM->DsmNav;
   vec3_t svb, svs;
   mat3x3_t CBN;

   double *SunAngEst = calloc(2, sizeof(double));

   const long BoreAxis = fss->BoreAxis;
   const long H_Axis   = fss->H_Axis;
   const long V_Axis   = fss->V_Axis;

   CBN = MTxM(Nav->CRB, Nav->refCRN);
   svb = MxV(CBN, AC->svn);
   svs = MxV(fss->CB, svb);

   switch (fss->type) {
      case CONVENTIONAL_FSS: {
         SunAngEst[0] = atan2(svs.v[H_Axis], svs.v[BoreAxis]);
         SunAngEst[1] = atan2(svs.v[V_Axis], svs.v[BoreAxis]);
      } break;
      case GS_FSS: {
         SunAngEst[0] = atan2(svs.v[V_Axis], svs.v[H_Axis]);
         SunAngEst[1] = atan2(sqrt(svs.v[V_Axis] * svs.v[V_Axis] +
                                   svs.v[H_Axis] * svs.v[H_Axis]),
                              svs.v[BoreAxis]);
      } break;
      default:
         fprintf(stderr,
                 "Invalid FSS Type. How did it get this far? Exiting...\n");
         exit(EXIT_FAILURE);
   }

   return (SunAngEst);
}

double *startrackFun(struct AcType *const AC, struct DSMType *const DSM,
                     const long Ist)
{
   const struct AcStarTrackerType *st = &AC->ST[Ist];
   const struct DSMNavType *Nav       = &DSM->DsmNav;
   quat_t qbn, qrn, qsnEst;

   double *q = calloc(4, sizeof(double));

   qrn    = C2Q(Nav->refCRN);
   qbn    = QxQ(Nav->qbr, qrn);
   qsnEst = QxQ(st->qb, qbn);
   CopyVG(q, qsnEst.q, 4);

   return (q);
}

double *gpsFun(struct AcType *const AC __attribute__((unused)),
               struct DSMType *const DSM,
               const long Igps __attribute__((unused)))
{
   const struct DSMNavType *Nav = &DSM->DsmNav;
   vec3_t tmp3V, tmpPosN, tmpVelN;
   long i;

   double *posNVelNEst = calloc(6, sizeof(double));

   for (i = 0; i < 3; i++)
      tmp3V.v[i] = Nav->PosR.v[i] + Nav->refPos.v[i];
   tmpPosN = MTxV(Nav->refCRN, tmp3V);
   for (i = 0; i < 3; i++)
      tmp3V.v[i] = Nav->VelR.v[i] + Nav->refVel.v[i];
   tmpVelN = MTxV(Nav->refCRN, tmp3V);
   if (Nav->refFrame != FRAME_N) {
      vec3_t wrn, wxr;
      wrn = MTxV(Nav->refCRN, Nav->refOmega);
      wxr = VxV(wrn, tmpPosN);
      for (i = 0; i < 3; i++)
         tmpVelN.v[i] += wxr.v[i];
   }

   for (i = 0; i < 3; i++) {
      posNVelNEst[i]     = tmpPosN.v[i];
      posNVelNEst[3 + i] = tmpVelN.v[i];
   }

   return (posNVelNEst);
}

// don't need this at the moment, WIP
double *accelFun(struct AcType *const AC __attribute__((unused)),
                 struct DSMType *const DSM __attribute__((unused)),
                 const long Ia __attribute__((unused)))
{
   return (NULL);
} /*{
   static double prevVelB[3]={0.0}, prevQBN[4]={0.0, 0.0, 0.0, 1.0};
   struct AcAccelType *A;
   struct NodeType *N;
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   double p[3];
   double r,Coef,rhatn[3],rhat[3],rhatop;
   double AccGGB[3],AccGG,Axis[3];
   double NodeQBN[4],AvgQN[4];
   double dvn[3],dvb[3],AvgAcc;
   double accelEst[1];
   double tmp3V[3], posN[3], velN[3], velB[3], CBN[3][3], qbn[4], qrn[4];
   long i;

   AC = &S->AC;
   DSM = &S->DSM;
   Nav = &DSM->DsmNav;
   N   = &S->B[0].Node[S->Accel[Ia].Node];

   MTxM(Nav->CRB,Nav->refCRN,CBN);
   for (i=0;i<3;i++) tmp3V[i] = Nav->VelR[i] + Nav->refVel[i];
   MTxV(Nav->CRB,tmp3V,velB);

   for (i=0;i<3;i++) tmp3V[i] = Nav->PosR[i] + Nav->refPos[i];
   MxV(Nav->CRB,tmp3V,posN);

   C2Q(Nav->refCRN,qrn);
   QxQ(Nav->qbr,qrn,qbn);

   QxQ(N->qb,qbn,NodeQBN);
   if (Nav->Init == FALSE) {
      for (i=0;i<3;i++) prevVelB[i] = velB[i];
      for (i=0;i<4;i++) prevQBN[i] = NodeQBN[i];
   }

   for(i=0;i<3;i++) AccGGB[i] = 0.0;
   // get back to this when I can figure out a way to get B->pn without truth
data
   // if (GGActive) {
   //    r = CopyUnitV(posN,rhatn);
   //    Coef = -3.0*Orb[S->RefOrb].mu/(r*r*r);
   //    MxV(CBN,rhatn,rhat);
   //    MxV(CBN,B->pn,p);
   //    for(i=0;i<3;i++) p[i] += N->PosB[i];
   //    rhatop = VoV(rhat,p);
   //    for(i=0;i<3;i++) {
   //       AccGGB[i] = Coef*(p[i]-3.0*rhat[i]*rhatop);
   //    }
   // }

   QTxV(N->qb,A->Axis,Axis);
   AccGG = VoV(AccGGB,Axis);

   for(i=0;i<3;i++) {
      dvb[i] = N->VelN[i] - prevVelB[i];
      prevVelB[i] = N->VelN[i];
   }
   for(i=0;i<4;i++) AvgQN[i] = prevQBN[i] + NodeQBN[i];
   UNITQ(AvgQN);
   for(i=0;i<4;i++) prevQBN[i] = NodeQBN[i];
   QxV(AvgQN,dvn,dvb);
   A->DV = VoV(dvb,Axis);
   AvgAcc = A->DV/A->SampleTime;
   accelEst[0] = AvgAcc + AccGG;


   return (accelEst);
}*/

/*--------------------------------------------------------------------*/
/*                   Auxillary helper functions                       */
/*--------------------------------------------------------------------*/
void getEarthAtmoParams(const JDType jd, double *NavFlux10p7,
                        double *NavGeomagIndex)
{
   JDType jd_tt_mjd      = JDChangeSystemEpoch(TT_TIME, GMAT_MJD_EPOCH, jd);
   double jd_tt_mjd_days = JDToDays(jd_tt_mjd);
   if (AtmoOption == TWOSIGMA_ATMO) {
      *NavFlux10p7    = LinInterpTbl(SchattenTable[0], SchattenTable[1],
                                     jd_tt_mjd_days, 1009);
      *NavGeomagIndex = LinInterpTbl(SchattenTable[0], SchattenTable[3],
                                     jd_tt_mjd_days, 1009);
   }
   else if (AtmoOption == NOMINAL_ATMO) {
      *NavFlux10p7    = LinInterpTbl(SchattenTable[0], SchattenTable[2],
                                     jd_tt_mjd_days, 1009);
      *NavGeomagIndex = LinInterpTbl(SchattenTable[0], SchattenTable[4],
                                     jd_tt_mjd_days, 1009);
   }
   else {
      // Pull from user-defined values in Inp_Sim.txt
      *NavFlux10p7    = Flux10p7;
      *NavGeomagIndex = GeomagIndex;
   }
}

/*--------------------------------------------------------------------*/
/*                          RIEKF functions                           */
/*--------------------------------------------------------------------*/

void eomRIEKFJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                         const DateType *date, const mat3x3_t CRB,
                         const quat_t qbr __attribute__((unused)),
                         const vec3_t PosR, const vec3_t VelR, const vec3_t wbr,
                         const double whlH[AC->Nwhl], const double AtmoDensity,
                         double **jacobian)
{
   mat3x3_t tmpM, tmpM2, tmpM3;
   vec3_t tmpV, tmpV2, tmpV3;
   static double **tmpAssign = NULL;
   vec3_t wrnd;
   struct DSMNavType *Nav = &DSM->DsmNav;
   long i, j, rowInd;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   switch (Nav->refFrame) {
      case FRAME_N:
         wrnd = VEC3_ZERO;
         break;
      case FRAME_L:
         break;
      case FRAME_F:
         break;
   }

   memset(jacobian[0], 0, sizeof(double) * Nav->navDim * Nav->navDim);

   if (Nav->stateActive[ROTMAT_STATE] && Nav->stateActive[POS_STATE] &&
       Nav->stateActive[VEL_STATE] && Nav->stateActive[OMEGA_STATE]) {
      FOR_STATES(state)
      {
         memset(tmpAssign[0], 0, sizeof(double) * 9);

         if (Nav->stateActive[state] == TRUE) {
            rowInd = Nav->navInd[state];
            switch (state) {
               case TIME_STATE:
                  tmpAssign[0][0] = 1.0;
                  subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 1, 1);
                  break;
               case ROTMAT_STATE:
                  if (Nav->stateActive[OMEGA_STATE]) {
                     memset(tmpAssign[0], 0, sizeof(double) * 9);
                     for (i = 0; i < 3; i++)
                        tmpAssign[i][i] = 1.0;

                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[OMEGA_STATE], 3, 3);
                  }
                  break;
               case QUAT_STATE:
                  break;
               case OMEGA_STATE:
                  tmpV = MxV(CRB, wbr);
                  tmpM = V2CrossM(tmpV);
                  CopyVG(tmpAssign[0], tmpM.flat, 9);
                  subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 3, 3);

                  // calculate dwbn_dot/dwbn
                  tmpV3 = MTxV(CRB, Nav->refOmega);
                  for (i = 0; i < 3; i++)
                     tmpV3.v[i] += wbr.v[i];
                  tmpM2 = V2CrossM(tmpV3);
                  tmpM3 = MxM(tmpM2, DSM->MOI);
                  tmpV2 = MxV(DSM->MOI, tmpV3);
                  for (long Iw = 0; Iw < AC->Nwhl; Iw++)
                     for (i = 0; i < 3; i++)
                        tmpV2.v[i] += whlH[Iw] * AC->Whl[Iw].Axis.v[i];

                  tmpM = V2CrossM(tmpV2);
                  for (i = 0; i < 9; i++)
                     tmpM.flat[i] -= tmpM3.flat[i];

                  MINVxM3(DSM->MOI, 3, MT(tmpM).rows, tmpM2.rows);
                  tmpM = Adjoint(CRB, MT(tmpM2));
                  // use tmpM = dwbn_dot/dwbn to calc a few derivs
                  if (Nav->refFrame != FRAME_N) {
                     tmpM2 = V2CrossM(Nav->refOmega);
                     tmpM3 = MxM(tmpM, tmpM2);
                     CopyVG(tmpAssign[0], tmpM3.flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[ROTMAT_STATE], 3, 3);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpM.mat[i][j] -= tmpM2.mat[i][j];
                  }
                  CopyVG(tmpAssign[0], tmpM.flat, 9);
                  subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 3, 3);
                  break;
               case POS_STATE:
                  memset(tmpAssign[0], 0, sizeof(double) * 9);
                  for (i = 0; i < 3; i++)
                     tmpAssign[i][i] = 1.0;

                  subMatAdd(jacobian, tmpAssign, rowInd, Nav->navInd[VEL_STATE],
                            3, 3);
                  if (Nav->stateActive[VEL_STATE] == FALSE) {
                     tmpM = V2CrossM(VelR);
                     CopyVG(tmpAssign[0], tmpM.flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[ROTMAT_STATE], 3, 3);
                  }
                  if (Nav->stateActive[OMEGA_STATE] == TRUE) {
                     tmpM = V2CrossM(PosR);
                     CopyVG(tmpAssign[0], tmpM.flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[OMEGA_STATE], 3, 3);
                  }
                  break;
               case VEL_STATE:
                  if (Nav->refFrame != FRAME_N) {
                     tmpV2 = SxV(2.0, Nav->refOmega);
                     tmpM  = V2CrossM(tmpV2);
                     CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 3, 3);

                     tmpM2 = V2CrossM(VelR);
                     tmpM3 = MxM(tmpM, tmpM2);
                     CopyVG(tmpAssign[0], tmpM3.flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[ROTMAT_STATE], 3, 3);

                     tmpM = V2DoubleCrossM(Nav->refOmega);
                     CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);

                     tmpM2 = V2CrossM(PosR);
                     tmpM3 = MxM(tmpM, tmpM2);
                     CopyVG(tmpAssign[0], tmpM3.flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[ROTMAT_STATE], 3, 3);

                     tmpM = V2CrossM(wrnd);
                     CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);
                     tmpM3 = MxM(tmpM, tmpM2);
                     CopyVG(tmpAssign[0], tmpM3.flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[ROTMAT_STATE], 3, 3);
                  }
                  if (Nav->stateActive[OMEGA_STATE] == TRUE) {
                     tmpM = V2CrossM(VelR);
                     CopyVG(tmpAssign[0], tmpM.flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[OMEGA_STATE], 3, 3);
                  }
                  break;
               default:
                  break;
            }
         }
      }
      if (DSM->refOrb->Regime == ORB_CENTRAL) {
         long orbCenter         = DSM->refOrb->World;
         mat3x3_t dAeroFrcdVRel = MAT3X3_ZERO, dAeroTrqdVRel = MAT3X3_ZERO;
         vec3_t worldWR = VEC3_ZERO;
         if (AeroActive) {
            double worldW = GetWorldW(Nav->jd_tt_mjd, &World[orbCenter]);
            for (i = 0; i < 3; i++)
               worldWR.v[i] = -Nav->refCRN.mat[i][2] * worldW;
            getDAeroFrcAndTrqDVRel(DSM, CRB, PosR, VelR, worldW, AtmoDensity,
                                   &dAeroFrcdVRel, &dAeroTrqdVRel);
            tmpM          = MxM(Nav->CRB, dAeroTrqdVRel);
            dAeroTrqdVRel = tmpM;
            dAeroFrcdVRel = SxM(1.0 / DSM->mass, dAeroFrcdVRel);
         }
         FOR_STATES(state)
         {
            if (Nav->stateActive[state] == TRUE) {
               rowInd = Nav->navInd[state];
               switch (state) {
                  case VEL_STATE:
                     if (Nav->stateActive[ROTMAT_STATE] == TRUE) {
                        vec3_t VelRdot = VEC3_ZERO;
                        if (GravPert.Enabled) {
                           vec3_t accelR = VEC3_ZERO;
                           for (i = 0; i < 3; i++)
                              tmpV.v[i] = PosR.v[i] + Nav->refPos.v[i];
                           accelR = NavGravPertAccel(Nav, date, tmpV, 1.0,
                                                     DSM->refOrb);
                           for (i = 0; i < 3; i++)
                              VelRdot.v[i] += accelR.v[i];
                        }
                        // TODO: transition to Encke's method, but refAccel for
                        // SC reference would need to be gravity free
                        for (i = 0; i < 3; i++)
                           tmpV.v[i] = PosR.v[i] + Nav->refPos.v[i];
                        tmpV2 = getGravAccel(DSM->refOrb->mu, tmpV);
                        for (i = 0; i < 3; i++)
                           VelRdot.v[i] += tmpV2.v[i];
                        if (Nav->refOriType != ORI_WORLD) {
                           for (i = 0; i < 3; i++)
                              VelRdot.v[i] -= Nav->refAccel.v[i];
                        }
                        tmpM = V2CrossM(VelRdot);
                        CopyVG(tmpAssign[0], tmpM.flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[ROTMAT_STATE], 3, 3);
                     }

                     for (i = 0; i < 3; i++)
                        tmpV2.v[i] = PosR.v[i] + Nav->refPos.v[i];
                     tmpM2 = getDGravFrcDPos(World[orbCenter].mu, tmpV2);
                     if (GravPert.Enabled) {
                        tmpM = NavDGravPertAccelDPos(Nav, date, tmpV2,
                                                     DSM->refOrb);
                        for (i = 0; i < 9; i++)
                           tmpM2.flat[i] += tmpM.flat[i];
                     }
                     CopyVG(tmpAssign[0], tmpM2.flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);
                     tmpM  = V2CrossM(PosR);
                     tmpM3 = MxM(tmpM2, tmpM);
                     CopyVG(tmpAssign[0], SxM(-1.0, tmpM3).flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[ROTMAT_STATE], 3, 3);

                     if (AeroActive) {
                        CopyVG(tmpAssign[0], dAeroFrcdVRel.flat, 9);

                        subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 3, 3);
                        tmpM2 = V2CrossM(VelR);
                        tmpM3 = MxM(dAeroFrcdVRel, tmpM2);
                        CopyVG(tmpAssign[0], SxM(-1.0, tmpM3).flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[ROTMAT_STATE], 3, 3);

                        tmpM2 = V2CrossM(worldWR);
                        tmpM3 = MxM(dAeroFrcdVRel, tmpM2);
                        CopyVG(tmpAssign[0], tmpM3.flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[POS_STATE], 3, 3);
                        tmpM2 = V2CrossM(PosR);
                        tmpM  = MxM(tmpM3, tmpM2);
                        CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[ROTMAT_STATE], 3, 3);
                     }
                     break;
                  case OMEGA_STATE:
                     if (AeroActive) {
                        CopyVG(tmpAssign[0], dAeroTrqdVRel.flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[VEL_STATE], 3, 3);
                        tmpM2 = V2CrossM(VelR);
                        tmpM3 = MxM(dAeroTrqdVRel, tmpM2);
                        CopyVG(tmpAssign[0], SxM(-1.0, tmpM3).flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[ROTMAT_STATE], 3, 3);

                        tmpM2 = V2CrossM(worldWR);
                        tmpM3 = MxM(dAeroTrqdVRel, tmpM2);
                        CopyVG(tmpAssign[0], tmpM3.flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[POS_STATE], 3, 3);
                        tmpM2 = V2CrossM(PosR);
                        tmpM  = MxM(tmpM3, tmpM2);
                        CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[ROTMAT_STATE], 3, 3);
                     }
                     break;
                  default:
                     break;
               }
            }
         }
      }
      else {
         fprintf(stderr, "Orbit types other than CENTRAL are still in "
                         "development for filtering. Exiting...\n");
         exit(EXIT_FAILURE);
      }
   }
   else {
      fprintf(stderr, "For the moment, can only filter rotation matrix, "
                      "position, velocity, & angular velocity *simultaneously* "
                      "with the RIEKF. Exiting...\n");
      exit(EXIT_FAILURE);
   }
}

void RIEKFUpdateLaw(struct DSMNavType *const Nav)
{
   vec3_t theta, tmpV, dr, dv, dw;
   mat3x3_t dR, tmpM;
   long i;

   const long nRVec = Nav->stateActive[POS_STATE] + Nav->stateActive[VEL_STATE];
   const long nBVec = Nav->stateActive[OMEGA_STATE];

   vec3_t x[nRVec];
   vec3_t xbar[nBVec];

   long curRInd = 0, curBInd = 0;
   FOR_STATES(state)
   {
      if (state == POS_STATE || state == VEL_STATE) {
         for (i = 0; i < 3; i++)
            x[curRInd].v[i] = -Nav->delta[i + Nav->navInd[state]];
         curRInd++;
      }
      else if (state == OMEGA_STATE) {
         for (i = 0; i < 3; i++)
            xbar[curBInd].v[i] = -Nav->delta[i + Nav->navInd[state]];
         curBInd++;
      }
   }

   for (i = 0; i < 3; i++)
      theta.v[i] = -Nav->delta[i + Nav->navInd[ROTMAT_STATE]];

   expmTFG(theta, nRVec, nBVec, x, xbar, &dR);
   curRInd = 0, curBInd = 0;
   FOR_STATES(state)
   {
      switch (state) {
         case POS_STATE:
            dr = x[curRInd];
            curRInd++;
            break;
         case VEL_STATE:
            dv = x[curRInd];
            curRInd++;
            break;
         case OMEGA_STATE:
            dw = xbar[curBInd];
            curBInd++;
            break;
         default:
            break;
      }
   }

   tmpV      = MxV(dR, Nav->PosR);
   Nav->PosR = VAddV_Elem(dr, tmpV);
   tmpV      = MxV(dR, Nav->VelR);
   Nav->VelR = VAddV_Elem(dv, tmpV);

   tmpV = MTxV(Nav->CRB, dw);
   for (i = 0; i < 3; i++)
      Nav->wbr.v[i] += tmpV.v[i];

   Nav->CRB = MxM(dR, Nav->CRB);
   tmpM     = MT(Nav->CRB);
   Nav->qbr = C2Q(tmpM);
}

/*--------------------------------------------------------------------*/
/*                          LIEKF functions                           */
/*--------------------------------------------------------------------*/

void eomLIEKFJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                         const DateType *date, const mat3x3_t CRB,
                         const quat_t qbr __attribute__((unused)),
                         const vec3_t PosR, const vec3_t VelR, const vec3_t wbr,
                         const double whlH[AC->Nwhl], const double AtmoDensity,
                         double **jacobian)
{
   mat3x3_t tmpM, tmpM2, tmpM3;
   vec3_t tmpV, tmpV2, tmpV3, wrnd = VEC3_ZERO;
   static double **tmpAssign = NULL;
   struct DSMNavType *Nav    = &DSM->DsmNav;
   long i, rowInd;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   switch (Nav->refFrame) {
      case FRAME_N:
         wrnd = VEC3_ZERO;
         break;
      case FRAME_L:
         break;
      case FRAME_F:
         break;
   }
   memset(jacobian[0], 0, sizeof(double) * Nav->navDim * Nav->navDim);

   vec3_t aeroTrq, aeroFrc;
   if (AeroActive) {
      const long orbCenter = DSM->refOrb->World;
      getAeroForceAndTorque(DSM, CRB, PosR, VelR,
                            GetWorldW(Nav->jd_tt_mjd, &World[orbCenter]),
                            AtmoDensity, &aeroFrc, &aeroTrq);
      tmpV2 = VAddV_Elem(tmpV2, aeroTrq);
   }

   if (Nav->stateActive[ROTMAT_STATE] && Nav->stateActive[POS_STATE] &&
       Nav->stateActive[VEL_STATE] && Nav->stateActive[OMEGA_STATE] &&
       !Nav->stateActive[QUAT_STATE]) {
      FOR_STATES(state)
      {
         memset(tmpAssign[0], 0, sizeof(double) * 9);

         if (Nav->stateActive[state] == TRUE) {
            rowInd = Nav->navInd[state];
            switch (state) {
               case TIME_STATE:
                  tmpAssign[0][0] = 1.0;
                  subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 1, 1);
                  break;
               case ROTMAT_STATE:
                  memset(tmpAssign[0], 0, sizeof(double) * 9);
                  for (i = 0; i < 3; i++)
                     tmpAssign[i][i] = 1.0;

                  subMatAdd(jacobian, tmpAssign, rowInd,
                            Nav->navInd[OMEGA_STATE], 3, 3);
                  // this is if wbr is not being filtered
                  // V2CrossM(Nav->wbr,tmpM);
                  // for (i=0;i<3;i++) for (j=0;j<3;j++) tmpAssign[i][j] =
                  // -tmpM[i][j];
                  // subMatAdd(jacobian,tmpAssign,rowInd,rowInd,3,3);
                  break;
               case QUAT_STATE:
                  break;
               case OMEGA_STATE:
                  tmpM = V2CrossM(wbr);
                  CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);
                  subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 3, 3);

                  tmpV3 = MTxV(CRB, Nav->refOmega);
                  tmpV3 = VAddV_Elem(tmpV3, wbr);
                  tmpV  = MxV(DSM->MOI, tmpV3);
                  for (long Iw = 0; Iw < AC->Nwhl; Iw++)
                     for (i = 0; i < 3; i++)
                        tmpV.v[i] += whlH[Iw] * AC->Whl[Iw].Axis.v[i];

                  tmpV2 = VxV(tmpV, tmpV3);
                  for (i = 0; i < 3; i++)
                     tmpV2.v[i] += Nav->torqueB.v[i];
                  CopyVG(tmpAssign[0], DSM->MOI.flat, 9);

                  // tmpV = wbn_dot (expressed in B, wrt N)
                  LINSOLVE(tmpAssign, tmpV.v, tmpV2.v, 3);
                  // find wbr_dot (expressed in B, wrt R)
                  if (Nav->refFrame != FRAME_N) {
                     tmpV2 = MTxV(CRB, Nav->refOmega);
                     tmpV3 = VxV(tmpV2, wbr);
                     tmpV2 = MTxV(CRB, wrnd);
                     for (i = 0; i < 3; i++)
                        tmpV.v[i] -= tmpV2.v[i] + tmpV3.v[i];
                  }
                  tmpM = V2CrossM(tmpV);
                  CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);
                  subMatAdd(jacobian, tmpAssign, rowInd,
                            Nav->navInd[ROTMAT_STATE], 3, 3);

                  // calculate dwbn_dot/dwbn
                  tmpV3 = MTxV(CRB, Nav->refOmega);
                  tmpV3 = VAddV_Elem(tmpV3, wbr);
                  tmpM2 = V2CrossM(tmpV3);
                  tmpM3 = MxM(tmpM2, DSM->MOI);
                  tmpV2 = MxV(DSM->MOI, tmpV3);
                  for (long Iw = 0; Iw < AC->Nwhl; Iw++)
                     for (i = 0; i < 3; i++)
                        tmpV2.v[i] += whlH[Iw] * AC->Whl[Iw].Axis.v[i];

                  tmpM = V2CrossM(tmpV2);
                  for (i = 0; i < 9; i++)
                     tmpM.flat[i] -= tmpM3.flat[i];
                  MINVxM3(DSM->MOI, 3, MT(tmpM).rows, tmpM2.rows);
                  // use tmpM2=dwbn_dot/dwbn to calc a few derivs
                  tmpV  = MTxV(CRB, Nav->refOmega);
                  tmpM  = V2CrossM(tmpV);
                  tmpM3 = MTxM(tmpM2, tmpM);
                  CopyVG(tmpAssign[0], tmpM3.flat, 9);
                  subMatAdd(jacobian, tmpAssign, rowInd,
                            Nav->navInd[ROTMAT_STATE], 3, 3);

                  tmpM3 = MSubM_Elem(MT(tmpM2), tmpM);
                  CopyVG(tmpAssign[0], tmpM3.flat, 9);
                  subMatAdd(jacobian, tmpAssign, rowInd,
                            Nav->navInd[OMEGA_STATE], 3, 3);
                  tmpM  = V2CrossM(wbr);
                  tmpM2 = MxM(tmpM3, tmpM);
                  CopyVG(tmpAssign[0], tmpM2.flat, 9);
                  subMatAdd(jacobian, tmpAssign, rowInd,
                            Nav->navInd[ROTMAT_STATE], 3, 3);
                  break;
               case POS_STATE:
                  tmpM = V2CrossM(wbr);
                  CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);

                  subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 3, 3);
                  memset(tmpAssign[0], 0, sizeof(double) * 9);
                  for (i = 0; i < 3; i++)
                     tmpAssign[i][i] = 1.0;

                  subMatAdd(jacobian, tmpAssign, rowInd, Nav->navInd[VEL_STATE],
                            3, 3);
                  break;
               case VEL_STATE:
                  tmpM = V2CrossM(wbr);
                  CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);

                  subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 3, 3);
                  for (i = 0; i < 3; i++)
                     tmpV.v[i] = Nav->forceB.v[i] / DSM->mass;
                  tmpM  = V2CrossM(tmpV);
                  tmpM2 = MxM(CRB, tmpM);
                  tmpM3 = V2CrossM(aeroFrc);

                  const double mass2 = DSM->mass * DSM->mass;
                  for (i = 0; i < 9; i++)
                     tmpAssign[0][i] = tmpM2.flat[i] + tmpM3.flat[i] / mass2;
                  subMatAdd(jacobian, tmpAssign, rowInd,
                            Nav->navInd[ROTMAT_STATE], 3, 3);

                  if (Nav->refFrame != FRAME_N) {
                     tmpV  = MTxV(CRB, Nav->refOmega);
                     tmpV2 = SxV(2.0, tmpV);
                     tmpM  = V2CrossM(tmpV2);
                     CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);

                     subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 3, 3);

                     tmpM = V2DoubleCrossM(tmpV);
                     CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);

                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);

                     tmpV = MTxV(CRB, wrnd);
                     tmpM = V2CrossM(tmpV);
                     CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);

                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);
                  }
                  break;
               default:
                  break;
            }
         }
      }
      if (DSM->refOrb->Regime == ORB_CENTRAL) {
         long orbCenter         = DSM->refOrb->World;
         mat3x3_t dAeroFrcdVRel = MAT3X3_ZERO, dAeroTrqdVRel = MAT3X3_ZERO;
         vec3_t worldWR = VEC3_ZERO;
         if (AeroActive) {
            double worldW = GetWorldW(Nav->jd_tt_mjd, &World[orbCenter]);
            for (i = 0; i < 3; i++)
               worldWR.v[i] = -Nav->refCRN.mat[i][2] * worldW;
            getDAeroFrcAndTrqDVRel(DSM, CRB, PosR, VelR, worldW, AtmoDensity,
                                   &dAeroFrcdVRel, &dAeroTrqdVRel);
            dAeroFrcdVRel = SxM(1.0 / DSM->mass, dAeroFrcdVRel);
         }
         FOR_STATES(state)
         {
            if (Nav->stateActive[state] == TRUE) {
               rowInd = Nav->navInd[state];
               switch (state) {
                  case VEL_STATE: {
                     tmpV2 = VAddV_Elem(PosR, Nav->refPos);
                     tmpM2 = getDGravFrcDPos(World[orbCenter].mu, tmpV2);
                     if (GravPert.Enabled) {
                        tmpM = NavDGravPertAccelDPos(Nav, date, tmpV2,
                                                     DSM->refOrb);
                        for (i = 0; i < 9; i++)
                           tmpM2.flat[i] += tmpM.flat[i];
                     }
                     tmpM = AdjointT(CRB, tmpM2);
                     CopyVG(tmpAssign[0], tmpM.flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);
                     if (AeroActive) {
                        tmpM3 = AdjointT(CRB, dAeroFrcdVRel);
                        CopyVG(tmpAssign[0], tmpM3.flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 3, 3);

                        tmpM2 = V2CrossM(worldWR);
                        // TODO: double check these two lines
                        tmpM  = MxM(tmpM3, tmpM2);
                        tmpM3 = MxM(tmpM, CRB);
                        CopyVG(tmpAssign[0], tmpM.flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[POS_STATE], 3, 3);
                     }
                  } break;
                  case OMEGA_STATE:
                     if (AeroActive) {
                        tmpM3 = MxM(dAeroTrqdVRel, CRB);
                        CopyVG(tmpAssign[0], tmpM3.flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[VEL_STATE], 3, 3);
                        tmpM  = V2CrossM(worldWR);
                        tmpM3 = MxM(dAeroTrqdVRel, tmpM);
                        tmpM  = MxM(tmpM3, CRB);
                        CopyVG(tmpAssign[0], tmpM.flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[POS_STATE], 3, 3);
                     }
                     break;
                  default:
                     break;
               }
            }
         }
      }
      else {
         fprintf(stderr, "Orbit types other than CENTRAL are still in "
                         "development for filtering. Exiting...\n");
         exit(EXIT_FAILURE);
      }
   }
   else {
      fprintf(stderr, "For the moment, can only filter rotation matrix, "
                      "position, velocity, & angular velocity *simultaneously* "
                      "with the LIEKF. Exiting...\n");
      exit(EXIT_FAILURE);
   }
}

void LIEKFUpdateLaw(struct DSMNavType *const Nav)
{
   mat3x3_t dR, tmpM;
   vec3_t theta, dr, dv, dw, tmpV;
   long i;

   long nRVec = Nav->stateActive[POS_STATE] + Nav->stateActive[VEL_STATE];
   long nBVec = Nav->stateActive[OMEGA_STATE];

   vec3_t x[nRVec];
   vec3_t xbar[nBVec];

   long curRInd = 0, curBInd = 0;
   FOR_STATES(state)
   {
      if (state == POS_STATE || state == VEL_STATE) {
         for (i = 0; i < 3; i++)
            x[curRInd].v[i] = -Nav->delta[i + Nav->navInd[state]];
         curRInd++;
      }
      else if (state == OMEGA_STATE) {
         for (i = 0; i < 3; i++)
            xbar[curBInd].v[i] = -Nav->delta[i + Nav->navInd[state]];
         curBInd++;
      }
   }

   for (i = 0; i < 3; i++)
      theta.v[i] = -Nav->delta[i + Nav->navInd[ROTMAT_STATE]];

   expmTFG(theta, nRVec, nBVec, x, xbar, &dR);
   curRInd = 0, curBInd = 0;
   FOR_STATES(state)
   {
      switch (state) {
         case POS_STATE:
            dr = x[curRInd];
            curRInd++;
            break;
         case VEL_STATE:
            dv = x[curRInd];
            curRInd++;
            break;
         case OMEGA_STATE:
            dw = xbar[curBInd];
            curBInd++;
            break;
         default:
            break;
      }
   }

   tmpV = MxV(Nav->CRB, dr);
   for (i = 0; i < 3; i++)
      Nav->PosR.v[i] += tmpV.v[i];
   tmpV = MxV(Nav->CRB, dv);
   for (i = 0; i < 3; i++)
      Nav->VelR.v[i] += tmpV.v[i];

   Nav->CRB = MxM(Nav->CRB, dR);

   tmpV = MTxV(dR, Nav->wbr);
   for (i = 0; i < 3; i++)
      Nav->wbr.v[i] = tmpV.v[i] + dw.v[i];
   tmpM     = MT(Nav->CRB);
   Nav->qbr = C2Q(tmpM);
}

/*--------------------------------------------------------------------*/
/*                          MEKF functions                           */
/*--------------------------------------------------------------------*/

void eomMEKFJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                        const DateType *date, const mat3x3_t CRB,
                        const quat_t qbr, const vec3_t PosR, const vec3_t VelR,
                        const vec3_t wbr, const double whlH[AC->Nwhl],
                        const double AtmoDensity, double **jacobian)
{
   mat3x3_t tmpM, tmpM2, tmpM3;
   vec3_t tmpV, tmpV2, tmpV3;
   vec3_t wrnd               = VEC3_ZERO;
   static double **tmpAssign = NULL;
   struct DSMNavType *Nav    = &DSM->DsmNav;
   long i, rowInd;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   switch (Nav->refFrame) {
      case FRAME_N:
         for (i = 0; i < 3; i++)
            wrnd = VEC3_ZERO;
         break;
      case FRAME_L:
         break;
      case FRAME_F:
         break;
   }

   memset(jacobian[0], 0, sizeof(double) * Nav->navDim * Nav->navDim);

   if (Nav->stateActive[QUAT_STATE] && Nav->stateActive[POS_STATE] &&
       Nav->stateActive[VEL_STATE] && Nav->stateActive[OMEGA_STATE]) {
      FOR_STATES(state)
      {
         memset(tmpAssign[0], 0, sizeof(double) * 9);

         if (Nav->stateActive[state] == TRUE) {
            rowInd = Nav->navInd[state];
            switch (state) {
               case TIME_STATE:
                  tmpAssign[0][0] = 1.0;
                  subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 1, 1);
                  break;
               case ROTMAT_STATE:
                  break;
               case QUAT_STATE:
                  tmpM = V2CrossM(wbr);
                  CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);

                  subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 3, 3);
                  memset(tmpAssign[0], 0, sizeof(double) * 9);
                  for (i = 0; i < 3; i++)
                     tmpAssign[i][i] = 1.0;

                  subMatAdd(jacobian, tmpAssign, rowInd,
                            Nav->navInd[OMEGA_STATE], 3, 3);
                  break;
               case OMEGA_STATE:
                  // calculate dwbn_dot/dwbn
                  tmpV3 = QxV(qbr, Nav->refOmega);
                  tmpV3 = VAddV_Elem(tmpV3, wbr);
                  tmpM2 = V2CrossM(tmpV3);
                  tmpM3 = MxM(tmpM2, DSM->MOI);
                  tmpV2 = MxV(DSM->MOI, tmpV3);
                  for (long Iw = 0; Iw < AC->Nwhl; Iw++)
                     for (i = 0; i < 3; i++)
                        tmpV2.v[i] += whlH[Iw] * AC->Whl[Iw].Axis.v[i];

                  tmpM = V2CrossM(tmpV2);
                  for (i = 0; i < 9; i++)
                     tmpM.flat[i] -= tmpM3.flat[i];

                  MINVxM3(DSM->MOI, 3, MT(tmpM).rows, tmpM2.rows);
                  tmpM2 = MT(tmpM2);
                  // MINV3(DSM->MOI, tmpM3);
                  // MxM(tmpM3, tmpM, tmpM2);
                  CopyVG(tmpAssign[0], tmpM2.flat, 9);
                  subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 3, 3);
                  break;
               case POS_STATE:
                  memset(tmpAssign[0], 0, sizeof(double) * 9);
                  for (i = 0; i < 3; i++)
                     tmpAssign[i][i] = 1.0;

                  subMatAdd(jacobian, tmpAssign, rowInd, Nav->navInd[VEL_STATE],
                            3, 3);
                  break;
               case VEL_STATE:
                  tmpV  = SxV(1.0 / DSM->mass, Nav->forceB);
                  tmpM  = V2CrossM(tmpV);
                  tmpM2 = MxM(CRB, tmpM);
                  CopyVG(tmpAssign[0], tmpM2.flat, 9);
                  subMatAdd(jacobian, tmpAssign, rowInd,
                            Nav->navInd[QUAT_STATE], 3, 3);

                  if (Nav->refFrame != FRAME_N) {
                     tmpV  = MTxV(CRB, Nav->refOmega);
                     tmpV2 = SxV(2.0, tmpV);
                     tmpM  = V2CrossM(tmpV2);
                     CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);

                     subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 3, 3);

                     tmpM = V2DoubleCrossM(tmpV);
                     CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);

                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);

                     tmpV = MTxV(CRB, wrnd);
                     tmpM = V2CrossM(tmpV);
                     CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);
                  }
                  break;
               default:
                  break;
            }
         }
      }
      if (DSM->refOrb->Regime == ORB_CENTRAL) {
         long orbCenter = DSM->refOrb->World;
         mat3x3_t dAeroFrcdVRel, dAeroTrqdVRel;
         vec3_t worldWR = VEC3_ZERO;
         if (AeroActive) {
            double worldW = GetWorldW(Nav->jd_tt_mjd, &World[orbCenter]);
            for (i = 0; i < 3; i++)
               worldWR.v[i] = -Nav->refCRN.mat[i][2] * worldW;
            getDAeroFrcAndTrqDVRel(DSM, CRB, PosR, VelR, worldW, AtmoDensity,
                                   &dAeroFrcdVRel, &dAeroTrqdVRel);
            dAeroFrcdVRel = SxM(1.0 / DSM->mass, dAeroFrcdVRel);
         }
         FOR_STATES(state)
         {
            if (Nav->stateActive[state] == TRUE) {
               rowInd = Nav->navInd[state];
               switch (state) {
                  case VEL_STATE: {
                     tmpV2 = VAddV_Elem(PosR, Nav->refPos);
                     tmpM2 = getDGravFrcDPos(World[orbCenter].mu, tmpV2);
                     if (GravPert.Enabled) {
                        tmpM = NavDGravPertAccelDPos(Nav, date, tmpV2,
                                                     DSM->refOrb);
                        for (i = 0; i < 9; i++)
                           tmpM2.flat[i] += tmpM.flat[i];
                     }
                     CopyVG(tmpAssign[0], tmpM2.flat, 9);
                     subMatAdd(jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);
                     if (AeroActive) {
                        CopyVG(tmpAssign[0], dAeroFrcdVRel.flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd, rowInd, 3, 3);

                        tmpM2 = V2CrossM(worldWR);
                        tmpM3 = MxM(dAeroFrcdVRel, tmpM2);
                        CopyVG(tmpAssign[0], tmpM3.flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[POS_STATE], 3, 3);
                     }
                  } break;
                  case OMEGA_STATE:
                     if (AeroActive) {
                        tmpM3 = MxM(dAeroTrqdVRel, CRB);
                        CopyVG(tmpAssign[0], tmpM3.flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[VEL_STATE], 3, 3);

                        tmpM  = V2CrossM(worldWR);
                        tmpM3 = MxM(dAeroTrqdVRel, tmpM);
                        tmpM  = MxM(tmpM3, CRB);
                        CopyVG(tmpAssign[0], tmpM.flat, 9);
                        subMatAdd(jacobian, tmpAssign, rowInd,
                                  Nav->navInd[POS_STATE], 3, 3);
                     }
                     break;
                  default:
                     break;
               }
            }
         }
      }
      else {
         fprintf(stderr, "Orbit types other than CENTRAL are still in "
                         "development for filtering. Exiting...\n");
         exit(EXIT_FAILURE);
      }
   }
   else {
      fprintf(
          stderr,
          "For the moment, can only filter quaternion, position, velocity, & "
          "angular velocity *simulataneously* with the MEKF. Exiting...\n");
      exit(EXIT_FAILURE);
   }
}

void MEKFUpdateLaw(struct DSMNavType *const Nav)
{
   quat_t q, dq = QUAT_EYE;
   mat3x3_t tmpM;

   for (int i = 0; i < 3; i++) {
      dq.qv.v[i]      = -Nav->delta[i + Nav->navInd[QUAT_STATE]] / 2.0;
      Nav->PosR.v[i] += -Nav->delta[i + Nav->navInd[POS_STATE]];
      Nav->VelR.v[i] += -Nav->delta[i + Nav->navInd[VEL_STATE]];
      Nav->wbr.v[i]  += -Nav->delta[i + Nav->navInd[OMEGA_STATE]];
   }
   dq.qs    = 1.0;
   q        = QxQ(dq, Nav->qbr);
   Nav->qbr = UNITQ(q);
   tmpM     = Q2C(Nav->qbr);
   Nav->CRB = MT(tmpM);
}

/******************************************************************************/
//                            Navigation Functions
/******************************************************************************/
double **GetStateLinTForm(struct DSMNavType *const Nav)
{
   mat3x3_t tmpM;
   double **tForm;
   static double **tmpAssign = NULL;
   long i;

   tForm = CreateMatrix(Nav->navDim, Nav->navDim);
   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   memset(tmpAssign[0], 0, sizeof(double) * 9);

   switch (Nav->type) {
      case LIEKF_NAV:
         CopyVG(tmpAssign[0], SxM(-1.0, Nav->CRB).flat, 9);
         subMatAdd(tForm, tmpAssign, Nav->navInd[ROTMAT_STATE],
                   Nav->navInd[ROTMAT_STATE], 3, 3);
         subMatAdd(tForm, tmpAssign, Nav->navInd[POS_STATE],
                   Nav->navInd[POS_STATE], 3, 3);
         subMatAdd(tForm, tmpAssign, Nav->navInd[VEL_STATE],
                   Nav->navInd[VEL_STATE], 3, 3);
         memset(tmpAssign[0], 0, sizeof(double) * 9);
         for (i = 0; i < 3; i++)
            tmpAssign[i][i] = -1.0;

         subMatAdd(tForm, tmpAssign, Nav->navInd[OMEGA_STATE],
                   Nav->navInd[OMEGA_STATE], 3, 3);
         tmpM = V2CrossM(Nav->wbr);
         CopyVG(tmpAssign[0], SxM(-1.0, tmpM).flat, 9);
         subMatAdd(tForm, tmpAssign, Nav->navInd[OMEGA_STATE],
                   Nav->navInd[ROTMAT_STATE], 3, 3);
         break;
      case RIEKF_NAV:
         memset(tmpAssign[0], 0, sizeof(double) * 9);
         for (i = 0; i < 3; i++)
            tmpAssign[i][i] = -1.0;

         subMatAdd(tForm, tmpAssign, Nav->navInd[ROTMAT_STATE],
                   Nav->navInd[ROTMAT_STATE], 3, 3);
         subMatAdd(tForm, tmpAssign, Nav->navInd[POS_STATE],
                   Nav->navInd[POS_STATE], 3, 3);
         subMatAdd(tForm, tmpAssign, Nav->navInd[VEL_STATE],
                   Nav->navInd[VEL_STATE], 3, 3);
         CopyVG(tmpAssign[0], SxM(-1.0, Nav->CRB).flat, 9);
         subMatAdd(tForm, tmpAssign, Nav->navInd[OMEGA_STATE],
                   Nav->navInd[OMEGA_STATE], 3, 3);

         tmpM = V2CrossM(Nav->PosR);
         CopyVG(tmpAssign[0], tmpM.flat, 9);
         subMatAdd(tForm, tmpAssign, Nav->navInd[POS_STATE],
                   Nav->navInd[ROTMAT_STATE], 3, 3);

         tmpM = V2CrossM(Nav->VelR);
         CopyVG(tmpAssign[0], tmpM.flat, 9);
         subMatAdd(tForm, tmpAssign, Nav->navInd[VEL_STATE],
                   Nav->navInd[ROTMAT_STATE], 3, 3);
         break;
      case MEKF_NAV:
         memset(tmpAssign[0], 0, sizeof(double) * 9);
         for (i = 0; i < 3; i++)
            tmpAssign[i][i] = -1.0;
         subMatAdd(tForm, tmpAssign, Nav->navInd[POS_STATE],
                   Nav->navInd[POS_STATE], 3, 3);
         subMatAdd(tForm, tmpAssign, Nav->navInd[VEL_STATE],
                   Nav->navInd[VEL_STATE], 3, 3);
         subMatAdd(tForm, tmpAssign, Nav->navInd[OMEGA_STATE],
                   Nav->navInd[OMEGA_STATE], 3, 3);
         CopyVG(tmpAssign[0], Nav->CRB.flat, 9);
         subMatAdd(tForm, tmpAssign, Nav->navInd[QUAT_STATE],
                   Nav->navInd[QUAT_STATE], 3, 3);
         break;
      default:
         fprintf(stderr, "Navigation active with undefined or ideal navigation "
                         "type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (tForm);
}

/******************************************************************************/
/* Use unscented transform to calculate statistics in typical error definition*/
/* WIP WIP WIP WIP WIP WIP WIP WIP WIP WIP WIP WIP WIP WIP WIP WIP WIP WIP WIP*/
/* Right now only does linear transformation                                  */
void UnscentedStateTForm(struct DSMNavType *const Nav, double *mean, double **P)
{
   const long navDim = Nav->navDim;
   switch (Nav->type) {
      default: {
         for (long i = 0; i < navDim; i++)
            mean[i] = 0;
         double **linTForm = GetStateLinTForm(Nav);

         MxMG(linTForm, Nav->S, Nav->NxN, navDim, navDim, navDim);
         MxMTG(Nav->NxN, Nav->NxN, P, navDim, navDim, navDim);
         DestroyMatrix(linTForm);
      } break;
   }
}

void configureRefFrame(struct DSMNavType *const Nav, double *const lerp_alpha,
                       const struct OrbitType *refOrb, const double dLerpAlpha,
                       const long reset)
{
   // set up reference frame. Not a fan of effectively using truth data for it
   vec3_t targetPosN, targetVelN;

   if (reset == TRUE)
      *lerp_alpha = 1.0;
   else
      *lerp_alpha += dLerpAlpha;

   if (Nav->Init == FALSE)
      Nav->refAccel = VEC3_ZERO;

   // Set the position and velocity of reference frame in N frame
   switch (Nav->refOriType) {
      case ORI_WORLD: {
         struct WorldType const *W = Nav->refOriPtr;
         if (&World[refOrb->World] != Nav->refOriPtr) {
            // TODO: No reason can't do this, just WIP
            fprintf(stderr,
                    "Navigation reference world %19s is not equal to the "
                    "central body of the SC's orbit, %19s. Exiting...\n",
                    W->Name, World[refOrb->World].Name);
            exit(EXIT_FAILURE);
         }
      } break;
      case ORI_OP: {
         const struct OrbitType *O = Nav->refOriPtr;
         targetPosN                = O->PosN;
         targetVelN                = O->VelN;
      } break;
      default: {
         // make sure if you do sc relative nav, you initialize that sc's nav
         // before you start this sc's
         // TODO: due to comm state, this is a time step behind...
         const struct DSMStateType *TrgState = Nav->refOriPtr;
         const struct BodyType *TrgSB        = Nav->refBodyPtr;
         // pn is position of body origin relative to sc origin
         targetPosN = VAddV_Elem(TrgState->PosN, TrgSB->pn);
         targetVelN = VAddV_Elem(TrgState->VelN, TrgSB->vn);
      } break;
   }

   const double one_m_alpha = 1.0 - *lerp_alpha;

   Nav->refPos = VEC3_ZERO;
   Nav->refVel = VEC3_ZERO;
   for (int i = 0; i < 3; i++) {
      Nav->refPos.v[i] =
          one_m_alpha * Nav->oldRefPos.v[i] + *lerp_alpha * targetPosN.v[i];
      Nav->refVel.v[i] =
          one_m_alpha * Nav->oldRefVel.v[i] + *lerp_alpha * targetVelN.v[i];
   }

   switch (Nav->refFrame) {
      case FRAME_N:
         Nav->refCRN      = MAT3X3_EYE;
         Nav->refOmega    = VEC3_ZERO;
         Nav->refOmegaDot = VEC3_ZERO;
         break;
      // case FRAME_B:
      //    break;
      // case FRAME_F:
      //    break;
      // case FRAME_L:
      //    break;
      default:
         fprintf(stderr, "Unknown Navigation Reference Frame. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }
   if (Nav->refFrame != FRAME_N) {
      vec3_t refPos, refVel, wxr;
      refPos      = MxV(Nav->refCRN, Nav->refPos);
      Nav->refPos = refPos;
      refVel      = MxV(Nav->refCRN, Nav->refVel);
      wxr         = VxV(Nav->refOmega, Nav->refPos);
      Nav->refVel = VSubV_Elem(refVel, wxr);
   }

   if (Nav->Init == TRUE && fabs(one_m_alpha) > __DBL_EPSILON__) {
      const double dt = one_m_alpha * Nav->DT;
      for (int i = 0; i < 3; i++)
         Nav->refAccel.v[i] = (targetVelN.v[i] - Nav->refVel.v[i]) / dt;
   }
   if (reset == TRUE) {
      Nav->oldRefCRN      = Nav->refCRN;
      Nav->oldRefPos      = Nav->refPos;
      Nav->oldRefVel      = Nav->refVel;
      Nav->oldRefOmega    = Nav->refOmega;
      Nav->oldRefOmegaDot = Nav->refOmegaDot;
      *lerp_alpha         = 0.0;
   }
}

void GetM(struct AcType *const AC, struct DSMNavType *const Nav,
          const mat3x3_t CRB, const quat_t qbr __attribute__((unused)),
          const vec3_t PosR, const vec3_t VelR, const vec3_t wbr, double **M)
{
   mat3x3_t tmp3x3, MOIInv;
   long i, j;

   memset(M[0], 0, sizeof(double) * Nav->navDim * Nav->navDim);

   switch (Nav->type) {
      case LIEKF_NAV: {
         for (i = 0; i < Nav->navDim; i++)
            M[i][i] = -1.0;
         for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
               M[Nav->navInd[POS_STATE] + i][Nav->navInd[POS_STATE] + j] =
                   -CRB.mat[j][i];
               M[Nav->navInd[VEL_STATE] + i][Nav->navInd[VEL_STATE] + j] =
                   -CRB.mat[j][i];
            }
         }
         MOIInv = MINV3(AC->MOI);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               M[Nav->navInd[OMEGA_STATE] + i][Nav->navInd[OMEGA_STATE] + j] =
                   -MOIInv.mat[i][j];

         tmp3x3 = V2CrossM(wbr);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               M[Nav->navInd[OMEGA_STATE] + i][Nav->navInd[ROTMAT_STATE] + j] =
                   tmp3x3.mat[i][j];
      } break;
      case RIEKF_NAV: {
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               M[Nav->navInd[ROTMAT_STATE] + i][Nav->navInd[ROTMAT_STATE] + j] =
                   -CRB.mat[i][j];
         for (i = 0; i < 3; i++) {
            M[Nav->navInd[POS_STATE] + i][Nav->navInd[POS_STATE] + i] = -1.0;
            M[Nav->navInd[VEL_STATE] + i][Nav->navInd[VEL_STATE] + i] = -1.0;
         }
         MOIInv = V2CrossM(PosR);
         tmp3x3 = MxM(MOIInv, CRB);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               M[Nav->navInd[POS_STATE] + i][Nav->navInd[ROTMAT_STATE] + j] =
                   -tmp3x3.mat[i][j];
         MOIInv = V2CrossM(VelR);
         tmp3x3 = MxM(MOIInv, CRB);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               M[Nav->navInd[VEL_STATE] + i][Nav->navInd[ROTMAT_STATE] + j] =
                   -tmp3x3.mat[i][j];
         MOIInv = MINV3(AC->MOI);
         tmp3x3 = MxM(CRB, MOIInv);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               M[Nav->navInd[OMEGA_STATE] + i][Nav->navInd[OMEGA_STATE] + j] =
                   -tmp3x3.mat[i][j];
      } break;
      case MEKF_NAV: {
         for (i = 0; i < Nav->navDim; i++)
            M[i][i] = -1.0;
         MOIInv = MINV3(AC->MOI);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               M[Nav->navInd[OMEGA_STATE] + i][Nav->navInd[OMEGA_STATE] + j] =
                   -MOIInv.mat[i][j];
      } break;
      default:
         fprintf(stderr, "Navigation active with undefined or ideal navigation "
                         "type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }
}

void getForceAndTorque(struct AcType *const AC, struct DSMNavType *const Nav,
                       const mat3x3_t CRB, const double *whlH)
{
   long j;

   Nav->forceB  = AC->IdealFrc;
   Nav->torqueB = AC->IdealTrq;

   for (j = 0; j < AC->Nthr; j++) {
      const struct AcThrType *thr = &AC->Thr[j];
      if (thr->ThrustLevelCmd > 0.0) {
         const double appliedForce = thr->ThrustLevelCmd * thr->Fmax;
         for (int i = 0; i < 3; i++) {
            Nav->forceB.v[i]  += appliedForce * thr->Axis.v[i];
            Nav->torqueB.v[i] += appliedForce * thr->rxA.v[i];
         }
      }
   }

   for (j = 0; j < AC->Nwhl; j++) {
      const struct AcWhlType *whl = &AC->Whl[j];
      if ((whlH[j] * signum(whl->Tcmd)) < whl->Hmax)
         for (int i = 0; i < 3; i++)
            Nav->torqueB.v[i] -= whl->Tcmd * whl->Axis.v[i];
   }
   vec3_t bvb;
   mat3x3_t CBN;
   CBN = MTxM(CRB, Nav->refCRN);
   bvb = MxV(CBN, AC->bvn);
   for (j = 0; j < AC->Nmtb; j++) {
      const struct AcMtbType *mtb = &AC->MTB[j];

      vec3_t AxBvb = VxV(mtb->Axis, bvb);
      for (int i = 0; i < 3; i++)
         Nav->torqueB.v[i] -= mtb->Mcmd * AxBvb.v[i];
   }
}

void NavSkDot(const long nav_dim, double **sk, double **F, double **M_sqrtQ,
              double **sk_dot)
{
   // F and M_sqrtQ are used as a scratch space and will be overwritten

   // sk_dot = F * Sk
   MxMG(F, sk, sk_dot, nav_dim, nav_dim, nav_dim);

   // F = Sk \ (F * Sk)
   MINVxMG(sk, sk_dot, F, nav_dim, nav_dim);

   // sk_dot = Sk \ (M * sqrtQ)
   MINVxMG(sk, M_sqrtQ, sk_dot, nav_dim, nav_dim);

   // M_sqrtQ = (Sk \ (M * sqrtQ)) * (Sk \ (M * sqrtQ))^T
   MxMTG(sk_dot, sk_dot, M_sqrtQ, nav_dim, nav_dim, nav_dim);

   // sk_dot + (sk_dot)^T = F + (F)^T + M_sqrtQ
   // sk_dot is lower triangular
   for (long i = 0; i < nav_dim; i++) {
      for (long j = 0; j < i; j++) {
         sk_dot[i][j] = F[i][j] + F[j][i] + M_sqrtQ[i][j];
         sk_dot[j][i] = 0.0;
      }
      sk_dot[i][i] = F[i][i] + M_sqrtQ[i][i] / 2.0;
   }

   // F = sk * sk_dot
   MxMG(sk, sk_dot, F, nav_dim, nav_dim, nav_dim);

   // transfer F to sk_dot, ensuring lower triangular
   memset(sk_dot[0], 0, sizeof(double) * nav_dim * nav_dim);
   for (long i = 0; i < nav_dim; i++) {
      for (long j = i + 1; j < nav_dim; j++)
         sk_dot[j][i] = F[j][i];
      sk_dot[i][i] = F[i][i];
   }
}

void NavEOMs(struct AcType *const AC, struct DSMType *const DSM,
             const DateType *date, const mat3x3_t CRB, const quat_t qbr,
             const vec3_t PosR, const vec3_t VelR, const vec3_t wbr,
             const double *whlH, double **Sk, mat3x3_t *CRBdot, quat_t *qbrdot,
             vec3_t *PosRdot, vec3_t *VelRdot, vec3_t *wbrdot, double *whlHdot,
             double **Skdot, const double AtmoDensity)
{
   long i, j;

   struct DSMNavType *Nav = &DSM->DsmNav;
   const long navDim      = Nav->navDim;

   (*Nav->EOMJacobianFun)(AC, DSM, date, CRB, qbr, PosR, VelR, wbr, whlH,
                          AtmoDensity, Nav->NxN);
   GetM(AC, Nav, CRB, qbr, PosR, VelR, wbr, Nav->M);
   for (i = 0; i < Nav->navDim; i++)
      for (j = 0; j < Nav->navDim; j++)
         Nav->NxN2[i][j] = Nav->M[i][j] * Nav->sqrQ[j];

   NavSkDot(navDim, Sk, Nav->NxN, Nav->NxN2, Skdot);

   vec3_t aeroFrc, aeroTrq;
   const long orbCenter          = DSM->refOrb->World;
   enum orbitRegime const regime = DSM->refOrb->Regime;
   if (AeroActive && regime == ORB_CENTRAL) {
      getAeroForceAndTorque(DSM, CRB, PosR, VelR,
                            GetWorldW(Nav->jd_tt_mjd, &World[orbCenter]),
                            AtmoDensity, &aeroFrc, &aeroTrq);
   }
   getForceAndTorque(AC, Nav, CRB, whlH);

   FOR_STATES(iState)
   {
      if (Nav->stateActive[iState] == TRUE) {
         double **pM3x3;
         mat3x3_t wbrX;
         vec3_t tmpV, tmpV2, wbn;
         switch (iState) {
            case TIME_STATE:
               break;
            case ROTMAT_STATE:
               wbrX    = V2CrossM(wbr);
               *CRBdot = MxM(CRB, wbrX);
               break;
            case QUAT_STATE:
               *qbrdot = QW2QDOT(qbr, wbr);
               break;
            case OMEGA_STATE: {
               wbn = wbr;
               if (Nav->refFrame != FRAME_N) {
                  tmpV = MTxV(CRB, Nav->refOmega);
                  wbn  = VAddV_Elem(wbn, tmpV);
               }
               vec3_t Hb = MxV(DSM->MOI, wbn);

               for (long Iw = 0; Iw < AC->Nwhl; Iw++)
                  for (i = 0; i < 3; i++)
                     Hb.v[i] += whlH[Iw] * AC->Whl[Iw].Axis.v[i];
               tmpV = VxV(Hb, wbn);
               for (i = 0; i < 3; i++)
                  tmpV.v[i] += Nav->torqueB.v[i];

               if (AeroActive && regime == ORB_CENTRAL)
                  for (i = 0; i < 3; i++)
                     tmpV.v[i] += aeroTrq.v[i];

               pM3x3 = CreateMatrix(3, 3);
               CopyVG(pM3x3[0], DSM->MOI.flat, 9);
               LINSOLVE(pM3x3, wbrdot->v, tmpV.v, 3);
               DestroyMatrix(pM3x3);
               if (Nav->refFrame != FRAME_N) {
                  tmpV = MTxV(CRB, Nav->refOmegaDot);
                  for (i = 0; i < 3; i++)
                     wbrdot->v[i] -= tmpV.v[i];
               }
            } break;
            case POS_STATE:
               *PosRdot = VelR;
               break;
            case VEL_STATE:
               *VelRdot = MxV(CRB, Nav->forceB);
               for (i = 0; i < 3; i++)
                  VelRdot->v[i] /= DSM->mass;
               if (Nav->refFrame != FRAME_N) {
                  fprintf(stderr, "Frame types other than Inertial are still "
                                  "in development for filtering. Exiting...\n");
                  exit(EXIT_FAILURE);
               }

               if (GravPert.Enabled) {
                  vec3_t accelR;
                  tmpV   = VAddV_Elem(PosR, Nav->refPos);
                  accelR = NavGravPertAccel(Nav, date, tmpV, 1.0, DSM->refOrb);
                  *VelRdot = VAddV_Elem(*VelRdot, accelR);
               }

               switch (regime) {
                  case ORB_CENTRAL: {
                     tmpV     = VAddV_Elem(PosR, Nav->refPos);
                     tmpV2    = getGravAccel(DSM->refOrb->mu, tmpV);
                     *VelRdot = VAddV_Elem(*VelRdot, tmpV2);
                     if (Nav->refOriType != ORI_WORLD)
                        *VelRdot = VAddV_Elem(*VelRdot, Nav->refAccel);

                     if (AeroActive)
                        for (i = 0; i < 3; i++)
                           VelRdot->v[i] += aeroFrc.v[i] / DSM->mass;
                     break;
                  }
                  default:
                     fprintf(stderr,
                             "Orbit types other than CENTRAL are still in "
                             "development for filtering. Exiting...\n");
                     exit(EXIT_FAILURE);
                     break;
               }
               break;
            default:
               fprintf(stderr, "Invalid State in NavEOMs(). Exiting...\n");
               exit(EXIT_FAILURE);
         }
      }
   }
   for (long Iw = 0; Iw < AC->Nwhl; Iw++)
      whlHdot[Iw] = AC->Whl[Iw].Tcmd;
}

void PropagateNav(struct AcType *const AC, struct DSMType *const DSM,
                  CCSDSTime *const cur_ccsds, const CCSDSTime next_ccsds,
                  const long init)
{
   if (isequal_ccsds(next_ccsds, *cur_ccsds))
      return;

   const CCSDSTime dccsds = CCSDSSub(next_ccsds, *cur_ccsds);

   double AtmoDensity = 0.0;

   long i, j, k;

   struct DSMNavType *Nav = &DSM->DsmNav;
   const double DT        = ccsds2seconds(dccsds);

   CCSDSTime date_ccsds            = date2ccsds(Nav->Date);
   const CCSDSTime date_off_ccdsds = CCSDSSub(*cur_ccsds, date_ccsds);
   const double dateOffset         = ccsds2seconds(date_off_ccdsds);

   if (init == TRUE) {
      if (AeroActive) {
         const long orbCenter = DSM->refOrb->World;
         vec3_t worldWR, VrelR, PosN, PosRWorld;
         PosRWorld           = VAddV_Elem(Nav->PosR, Nav->refPos);
         const double worldw = GetWorldW(Nav->jd_tt_mjd, &World[orbCenter]);
         for (i = 0; i < 3; i++)
            worldWR.v[i] = -Nav->refCRN.mat[i][2] * worldw;
         VrelR = VxV(worldWR, PosRWorld);
         for (i = 0; i < 3; i++)
            VrelR.v[i] += Nav->VelR.v[i] + Nav->refVel.v[i];
         PosN = MTxV(Nav->refCRN, PosRWorld);
         if (orbCenter == EARTH) {
            double NavFlux10p7, NavGeomagIndex;
            double Alt;
            vec3_t PosW;
            mat3x3_t CWN;

            if (EphemOption == 3) {
               JDType jd = ccsds2jd(*cur_ccsds);
               CWN       = SpiceGetCWJ(jd, EARTH);
            }
            else
               CWN = NavGetWorldCWN(orbCenter, Nav->Date);

            PosW = MxV(CWN, PosN);
            Alt  = MAGV(PosW) - World[orbCenter].rad;
            if (Alt < 1000.0E3) { /* What is max alt of MSISE00 validity? */
               JDType jd = Date2JD(Nav->Date, MJD_EPOCH);
               jd        = JDChangeSystem(TT_TIME, jd);
               getEarthAtmoParams(jd, &NavFlux10p7, &NavGeomagIndex);
               Nav->Date.doy =
                   MD2DOY(Nav->Date.Year, Nav->Date.Month, Nav->Date.Day);
               AtmoDensity =
                   NRLMSISE00(Nav->Date, PosW, NavFlux10p7, NavGeomagIndex);
            }
            else
               AtmoDensity = 0.0;
         }
         else if (orbCenter == MARS) {
            AtmoDensity = MarsAtmosphereModel(PosN);
         }
         else
            AtmoDensity = 0.0;
      }
   }
   double dLerpAlpha = 0.0;
   double lerpAlphak = Nav->refLerpAlpha;
   // RK4
   mat3x3_t CRBk[ORDRK];
   quat_t qbrk[ORDRK];
   vec3_t PosRk[ORDRK], VelRk[ORDRK], wbrk[ORDRK];
   double **Skk[ORDRK] = {NULL};
   for (k = 0; k < ORDRK; k++)
      Skk[k] = CreateMatrix(Nav->navDim, Nav->navDim);
   double whlHk[ORDRK][AC->Nwhl];
#if ORDRK == 4
   const double DTk[ORDRK]     = {0.0, DT * 0.5, DT * 0.5, DT};
   const double rkScale[ORDRK] = {DT / 6.0, DT / 3.0, DT / 3.0, DT / 6.0};
#elif ORDRK == 1
   const double DTk[ORDRK]     = {0.0};
   const double rkScale[ORDRK] = {DT};
#endif
   double **Sk = CreateMatrix(Nav->navDim, Nav->navDim);
   for (k = 0; k < ORDRK; k++) {
      DateType date = Nav->Date;
      date          = updateTime(date, dateOffset + DTk[k]);
      lerpAlphak    = Nav->refLerpAlpha;
      dLerpAlpha    = DTk[k] / Nav->DT;
      mat3x3_t CRB;
      quat_t qbr;
      vec3_t PosR, VelR, wbr;
      double whlH[AC->Nwhl];

      for (i = 0; i < Nav->navDim; i++)
         for (j = 0; j <= i; j++)
            Sk[i][j] = Nav->S[i][j];
      CRB  = Nav->CRB;
      qbr  = Nav->qbr;
      PosR = Nav->PosR;
      VelR = Nav->VelR;
      wbr  = Nav->wbr;
      for (i = 0; i < AC->Nwhl; i++)
         whlH[i] = Nav->whlH[i];

      if (k > 0) {
         axpy(DTk[k], Skk[k - 1][0], Sk[0], Nav->navDim * Nav->navDim);

         FOR_STATES(Istate)
         {
            if (Nav->stateActive[Istate] == TRUE) {
               mat3x3_t CBR;
               switch (Istate) {
                  case TIME_STATE:
                     // incremented by DT after this function is called
                     break;
                  case ROTMAT_STATE:
                     for (i = 0; i < 9; i++)
                        CRB.flat[i] += DTk[k] * CRBk[k - 1].flat[i];
                     CBR = MT(CRB);
                     qbr = UNITQ(C2Q(CBR));
                     CBR = Q2C(qbr);
                     CRB = MT(CBR);
                     break;
                  case QUAT_STATE:
                     for (i = 0; i < 4; i++)
                        qbr.q[i] += DTk[k] * qbrk[k - 1].q[i];
                     qbr = UNITQ(qbr);
                     CBR = Q2C(qbr);
                     CRB = MT(CBR);
                     break;
                  case OMEGA_STATE:
                     for (i = 0; i < 3; i++)
                        wbr.v[i] += DTk[k] * wbrk[k - 1].v[i];
                     break;
                  case POS_STATE:
                     for (i = 0; i < 3; i++)
                        PosR.v[i] += DTk[k] * PosRk[k - 1].v[i];
                     break;
                  case VEL_STATE:
                     for (i = 0; i < 3; i++)
                        VelR.v[i] += DTk[k] * VelRk[k - 1].v[i];
                     break;
                  default:
                     break;
               }
            }
         }
         for (i = 0; i < AC->Nwhl; i++)
            whlH[i] = Limit(Nav->whlH[i] + whlHk[k - 1][i] * DTk[k],
                            -AC->Whl[i].Hmax, AC->Whl[i].Hmax);
      }
      configureRefFrame(Nav, &lerpAlphak, DSM->refOrb, dLerpAlpha, FALSE);
      getForceAndTorque(AC, Nav, CRB, whlH);
      NavEOMs(AC, DSM, &date, CRB, qbr, PosR, VelR, wbr, whlH, Sk, &CRBk[k],
              &qbrk[k], &PosRk[k], &VelRk[k], &wbrk[k], whlHk[k], Skk[k],
              AtmoDensity);
   }

   for (k = 0; k < ORDRK; k++) {
      for (i = 0; i < Nav->navDim; i++)
         for (j = 0; j <= i; j++)
            Nav->S[i][j] += Skk[k][i][j] * rkScale[k];
      DestroyMatrix(Skk[k]);
   }
   DestroyMatrix(Sk);

   FOR_STATES(Istate)
   {
      if (Nav->stateActive[Istate] == TRUE) {
         mat3x3_t CBR;
         switch (Istate) {
            case TIME_STATE:
               // incremented by DT at end of this function
               break;
            case ROTMAT_STATE: {
#if ORDRK == 4
               for (k = 0; k < ORDRK; k++)
                  for (i = 0; i < 9; i++)
                     Nav->CRB.flat[i] += rkScale[k] * CRBk->flat[i];
#elif ORDRK == 1
               CRB = MTxM(Nav->CRB, CRBk[0]);
               vec3_t tmpV;
               tmpV.x   = CRB.mat[2][1] * rkScale[0];
               tmpV.y   = CRB.mat[0][2] * rkScale[0];
               tmpV.z   = CRB.mat[1][0] * rkScale[0];
               CRB      = expmso3(tmpV);
               CBR      = Nav.CRB;
               Nav->CRB = MxM(CBR, CRB);
#endif
               CBR      = MT(Nav->CRB);
               Nav->qbr = C2Q(CBR);
               Nav->qbr = UNITQ(Nav->qbr);
               CBR      = Q2C(Nav->qbr);
               Nav->CRB = MT(CBR);
            } break;
            case QUAT_STATE: {
#if ORDRK == 4
               for (k = 0; k < ORDRK; k++)
                  for (i = 0; i < 4; i++)
                     Nav->qbr.q[i] += rkScale[k] * qbrk[k].q[i];
#elif ORDRK == 1
               qbr               = QxQT(qbrk[0], Nav->qbr);
               magvec3_t uq      = UNITV(qbr.qv) * rkScale[0];
               const double tmag = uq.m * rkScale[0];
               if (tmag > __DBL_EPSILON__) {
                  double stmag = sin(tmag);
                  qbrk[0].qv   = SxV(stmag, qbr.qv);
                  qbrk[0].qs   = cos(tmag);
                  qbr          = Nav->qbr;
                  Nav->qbr     = QxQ(qbrk[0], qbr);
               }
#endif
               Nav->qbr = UNITQ(Nav->qbr);
               CBR      = Q2C(Nav->qbr);
               Nav->CRB = MT(CBR);
            } break;
            case OMEGA_STATE:
               for (k = 0; k < ORDRK; k++)
                  for (i = 0; i < 3; i++)
                     Nav->wbr.v[i] += rkScale[k] * wbrk[k].v[i];
               break;
            case POS_STATE:
               for (k = 0; k < ORDRK; k++)
                  for (i = 0; i < 3; i++)
                     Nav->PosR.v[i] += rkScale[k] * PosRk[k].v[i];
               break;
            case VEL_STATE:
               for (k = 0; k < ORDRK; k++)
                  for (i = 0; i < 3; i++)
                     Nav->VelR.v[i] += rkScale[k] * VelRk[k].v[i];
               break;
            default:
               break;
         }
      }
   }

   for (i = 0; i < AC->Nwhl; i++)
      Nav->whlH[i] = Limit(Nav->whlH[i] + AC->Whl[i].Tcmd * DT,
                           -AC->Whl[i].Hmax, AC->Whl[i].Hmax);

   *cur_ccsds      = next_ccsds;
   Nav->ccsds_time = *cur_ccsds;
   Nav->Date       = ccsds2date(*cur_ccsds, TT_TIME);
   configureRefFrame(Nav, &Nav->refLerpAlpha, DSM->refOrb, DT / Nav->DT, FALSE);
}

void CalcInnovation(const enum SensorType type,
                    const struct DSMMeasType *const meas,
                    const double *const meas_est, double *innovation)
{
   switch (type) {
      case STARTRACK_SENSOR: {
         const quat_t q_data = DBL_TO_QUAT(meas->data);
         const quat_t q_est  = DBL_TO_QUAT(meas_est);
         vec3_t inn_v        = Q2AngleVec(QxQT(q_data, q_est));
         VEC3_TO_DBL(innovation, inn_v);
      } break;
      default:
         for (int i = 0; i < meas->errDim; i++)
            innovation[i] = meas->data[i] - meas_est[i];
         break;
   }
}

void Underweighting(const long nav_dim, const long meas_err_dim, double **HS,
                    double **NsqrtR, double **out)
{
   long i, j;
   // Compare square of matrix 2-norms for underweighting
   if (M2Norm2G(HS, meas_err_dim, nav_dim) >=
       (5.0 * M2Norm2G(NsqrtR, meas_err_dim, meas_err_dim))) {
      const double rtp = sqrt(1.2);
      for (i = 0; i < nav_dim; i++)
         for (j = 0; j < meas_err_dim; j++)
            out[i][j] = HS[j][i] * rtp;
   }
   else {
      for (i = 0; i < nav_dim; i++)
         for (j = 0; j < meas_err_dim; j++)
            out[i][j] = HS[j][i];
   }

   for (i = 0; i < meas_err_dim; i++)
      for (j = 0; j < meas_err_dim; j++)
         out[i + nav_dim][j] = NsqrtR[j][i];
}

void GetMeasBatchParams(const struct DSMMeasListType *const meas_list,
                        const enum batchType batching,
                        enum SensorType *const sense_type,
                        CCSDSTime *const meas_ccsds, long *const meas_err_dim,
                        long *const meas_noise_dim)
{
   *meas_err_dim   = 0;
   *meas_noise_dim = 0;
   *sense_type     = meas_list->head->type;
   *meas_ccsds     = meas_list->head->ccsds_time;

   // TODO: avoid this preallocation mess and go with
   // realloc (maybe?)
   switch (batching) {
      case NONE_BATCH:
         *meas_err_dim   = meas_list->head->errDim;
         *meas_noise_dim = meas_list->head->noiseDim;
         break;
      case SENSOR_BATCH: {
         struct DSMMeasType *meas = meas_list->head;
         while (meas != NULL && meas->type == *sense_type &&
                isequal_ccsds(*meas_ccsds, meas->ccsds_time)) {
            *meas_err_dim   += meas->errDim;
            *meas_noise_dim += meas->noiseDim;
            meas             = meas->nextMeas;
         }
      } break;
      case TIME_BATCH: {
         struct DSMMeasType *meas = meas_list->head;
         while (meas != NULL && isequal_ccsds(*meas_ccsds, meas->ccsds_time)) {
            *meas_err_dim   += meas->errDim;
            *meas_noise_dim += meas->noiseDim;
            meas             = meas->nextMeas;
         }
      } break;
      default:
         fprintf(stderr,
                 "Invalid Batching method. If you are reading this, the "
                 "developer probably has a messed up pointer. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }
}

void ParseMeasList(struct AcType *const AC, struct DSMType *const DSM,
                   struct DSMMeasListType *const meas_list, const long nav_dim,
                   const enum batchType batching, const CCSDSTime meas_ccsds,
                   const enum SensorType sense_type,
                   double *const innov_time __attribute__((unused)),
                   long *const __attribute__((unused)),
                   double **innovs[FIN_SENSOR + 1] __attribute__((unused)),
                   double *big_innov, double **big_H, double **big_N,
                   double *big_sqrtR)
{
   long cur_err_dim = 0;
   long cur_r_dim   = 0;
   double *measEstData, **measJacobian;
   while (meas_list->head != NULL) {
      struct DSMMeasType *meas = pop_DSMMeas(meas_list);
      const long err_dim       = meas->errDim;
      const long r_dim         = meas->noiseDim;

      double innov[err_dim];
      measEstData = (*meas->measFun)(AC, DSM, meas->sensorNum);
      measJacobian =
          (*meas->measJacobianFun)(AC, DSM, meas->sensorNum, meas->N);

      CalcInnovation(meas->type, meas, measEstData, innov);

#ifdef REPORT_RESIDUALS
      *innov_time  = (double)ccsds2seconds(meas->ccsds_time);
      *innov_exist = TRUE;
      innovs[meas->type][meas->sensorNum] = calloc(err_dim, sizeof(double));
      for (long i = 0; i < err_dim; i++)
         innovs[meas->type][meas->sensorNum][i] = innov[i];
#endif
      for (long i = 0; i < r_dim; i++) {
         big_sqrtR[cur_r_dim + i] = meas->R[i];
         for (long j = 0; j < err_dim; j++)
            big_N[cur_err_dim + j][cur_r_dim + i] = meas->N[j][i];
      }

      for (long i = 0; i < err_dim; i++) {
         big_innov[cur_err_dim + i] = innov[i];
         for (long j = 0; j < nav_dim; j++)
            big_H[cur_err_dim + i][j] = measJacobian[i][j];
      }

      cur_err_dim += err_dim;
      cur_r_dim   += r_dim;
      free(measEstData);
      DestroyMatrix(measJacobian);
      DestroyMeas(meas);

      if ((batching == NONE_BATCH) || (meas_list->head == NULL) ||
          (!isequal_ccsds(meas_list->head->ccsds_time,
                          meas_ccsds)) || // TIME_BATCH
          (batching == SENSOR_BATCH && meas_list->head->type != sense_type)) {
         break;
      }
   }
}

void SkUpdate(const long nav_dim, const long meas_err_dim, double **Uk,
              const long id, double **Sk)
{
   long u_inds[meas_err_dim];
   double u_list[meas_err_dim][nav_dim];

   for (long i = 0; i < meas_err_dim; i++) {
      u_inds[i] = i;
      for (long j = 0; j < nav_dim; j++)
         u_list[i][j] = Uk[j][i];
   }

   for (long i = 0; i < meas_err_dim; i++) {
      if (!cholDowndate(Sk, u_list[u_inds[i]], nav_dim)) {
         if (u_inds[i] < i) {
            // if the currently offending u caused problems earlier, we're
            // out of luck
            fprintf(stderr,
                    "Cholesky Downdate failed for SC[%li]! Exiting...\n", id);
            exit(EXIT_FAILURE);
         }

         // move the offending u to the end of the list, do it later
         const double tmp_i = u_inds[i];
         for (long j = i; j < meas_err_dim - 1; j++)
            u_inds[j] = u_inds[j + 1];
         u_inds[meas_err_dim - 1] = tmp_i;
      }
   }
   for (long i = 0; i < nav_dim; i++)
      for (long j = 0; j < i; j++)
         Sk[j][i] = 0.0;
}

void GetKUk(const long nav_dim, const long meas_err_dim,
            const long meas_noise_dim, double **S, double **N, double *sqrtR,
            double **H, double **K, double **Uk)
{
   // passing around the pointer to keep notation
   // consistent and save memory ops
   double **Sz, **C;

   // UNDERWEIGHTING
   double **HS = CreateMatrix(meas_err_dim, nav_dim);
   MxMG(H, S, HS, meas_err_dim, nav_dim, nav_dim);
   double **NsqrtR = CreateMatrix(meas_err_dim, meas_noise_dim);
   for (long i = 0; i < meas_err_dim; i++)
      for (long j = 0; j < meas_noise_dim; j++)
         NsqrtR[i][j] = N[i][j] * sqrtR[j];

   double **tmp = CreateMatrix(nav_dim + meas_noise_dim, meas_err_dim);
   Underweighting(nav_dim, meas_err_dim, HS, NsqrtR, tmp);
   DestroyMatrix(NsqrtR);

   Sz         = CreateMatrix(meas_err_dim, meas_err_dim);
   double **U = CreateMatrix(nav_dim + meas_err_dim, meas_err_dim);
   hqrd(tmp, U, Sz, nav_dim + meas_err_dim, meas_err_dim);

   DestroyMatrix(U);
   DestroyMatrix(tmp);

   C = CreateMatrix(nav_dim, meas_err_dim);
   MxMTG(S, HS, C, nav_dim, nav_dim, meas_err_dim);
   DestroyMatrix(HS);

   MxMINVG(C, Sz, Uk, nav_dim, meas_err_dim);
   for (long i = 0; i < meas_err_dim; i++) {
      for (long j = i + 1; j < meas_err_dim; j++) {
         Sz[j][i] = Sz[i][j];
         Sz[i][j] = 0.0;
      }
   }
   MxMINVG(Uk, Sz, K, nav_dim, meas_err_dim);
   DestroyMatrix(Sz);
   DestroyMatrix(C);
}

void KalmanFilt(struct AcType *const AC, struct DSMType *const DSM)
{
   long i;
   struct DSMNavType *Nav = &DSM->DsmNav;

   // TODO: will maybe need to do something to preserve information if a new
   // Nav filter is called
   if (Nav->Init == FALSE)
      configureRefFrame(Nav, &Nav->refLerpAlpha, DSM->refOrb, 0.0, TRUE);

   // Accumulate information from measurements based upon batching method.
   struct DSMMeasListType *const measList = &Nav->measList;

   Nav->ccsds_time           = date2ccsds(Nav->Date);
   CCSDSTime cur_ccsds       = Nav->ccsds_time;
   const CCSDSTime fin_ccsds = CCSDSAddSeconds(cur_ccsds, Nav->DT);
   if (measList->head == NULL)
      PropagateNav(AC, DSM, &cur_ccsds, fin_ccsds, TRUE);
   else {
      long init = TRUE;

      while (measList->head != NULL) {
         long meas_err_dim          = 0;
         long meas_noise_dim        = 0;
         CCSDSTime meas_ccsds       = {0};
         enum SensorType sense_type = NULL_SENSOR;
         GetMeasBatchParams(measList, Nav->batching, &sense_type, &meas_ccsds,
                            &meas_err_dim, &meas_noise_dim);

         if (isless_ccsds(meas_ccsds, cur_ccsds)) {
            fprintf(stderr,
                    "Attempted to propagate Navigation state backwards in "
                    "time. How did that happen? Exiting...\n");
            exit(EXIT_FAILURE);
         }
         else {
#ifdef REPORT_RESIDUALS
            // TODO: set a report residuals "bool" in nav and
            // use that instead of these compile-time
            // directives
            if (Nav->innovationTime > 0.0 && Nav->innovationsExist) {
               DSM_NAV_ResidualsReport(Nav->innovationTime, AC->ID,
                                       &Nav->innovationsReportFirst,
                                       Nav->innovations);
               FOR_SENSORS(sensor)
               {
                  for (i = 0; i < Nav->nSensor[sensor]; i++) {
                     if (Nav->sensorActive[sensor][i] == TRUE) {
                        free(Nav->innovations[sensor][i]);
                        Nav->innovations[sensor][i] = NULL;
                     }
                  }
               }
               Nav->innovationsExist = FALSE;
            }
#endif
            // TODO: investigate only prop once per Kalman filt call and
            // use STM and linearization to prop measurements through time
            PropagateNav(AC, DSM, &cur_ccsds, meas_ccsds, init);
            if (init == TRUE)
               init = FALSE;
         }

         double **bigH, *bigInnov, *big_sqrtR, **bigN;
         bigH      = CreateMatrix(meas_err_dim, Nav->navDim);
         bigN      = CreateMatrix(meas_err_dim, meas_noise_dim);
         bigInnov  = calloc(meas_err_dim, sizeof(double));
         big_sqrtR = calloc(meas_noise_dim, sizeof(double));

         ParseMeasList(AC, DSM, measList, Nav->navDim, Nav->batching,
                       meas_ccsds, sense_type, &Nav->innovationTime,
                       &Nav->innovationsExist, Nav->innovations, bigInnov, bigH,
                       bigN, big_sqrtR);

         double **K  = CreateMatrix(Nav->navDim, meas_err_dim);
         double **Uk = CreateMatrix(Nav->navDim, meas_err_dim);
         GetKUk(Nav->navDim, meas_err_dim, meas_noise_dim, Nav->S, bigN,
                big_sqrtR, bigH, K, Uk);

         MxVG(K, bigInnov, Nav->delta, Nav->navDim, meas_err_dim);
         (*Nav->updateLaw)(Nav);
         DestroyMatrix(K);

         SkUpdate(Nav->navDim, meas_err_dim, Uk, DSM->ID, Nav->S);

         DestroyMatrix(Uk);
         DestroyMatrix(bigH);
         DestroyMatrix(bigN);
         free(bigInnov);
         free(big_sqrtR);
      }
      if (isless_ccsds(cur_ccsds, fin_ccsds))
         PropagateNav(AC, DSM, &cur_ccsds, fin_ccsds, FALSE);
   }

   Nav->steps++;
   Nav->jd_tt_mjd =
       JDAddIntegerMultRatSecs(Nav->jd_tt_mjd_0, Nav->steps, Nav->DT_RAT);
   Nav->Date = JDToDate(Nav->jd_tt_mjd, TT_TIME);

   Nav->ccsds_time = date2ccsds(Nav->Date);
   configureRefFrame(Nav, &Nav->refLerpAlpha, DSM->refOrb,
                     1.0 - Nav->refLerpAlpha, TRUE);
   for (i = 0; i < AC->Nwhl; i++)
      Nav->whlH[i] = AC->Whl[i].H;

   if (Nav->Init == FALSE)
      Nav->Init = TRUE;
}
/******************************************************************************/
//                             Auxillary Functions
/******************************************************************************/
// Add (constant) B to a submatrix of A.
//    iN - starting row
//    iM - starting column
//    n  - number of rows of B
//    m  - number of columns of B
void subMatAdd(double **A, double **B, long const iN, long const iM,
               long const n, long const m)
{
   long i, j, curRow;
   for (i = 0; i < n; i++) {
      curRow = i + iN;
      for (j = 0; j < m; j++)
         A[curRow][j + iM] += B[i][j];
   }
}

// Squared Mahalonobis distance between n-D points x and y according to the
// covariance matrix A
double mahalonobis2(double **A, double *x, double *y, long const n)
{
   double Ainvxy[n];
   double xy[n];
   for (long i = 0; i < n; i++) {
      Ainvxy[i] = 0.0;
      xy[i]     = x[i] - y[i];
   }

   LINSOLVE(A, Ainvxy, xy, n);
   double d = 0.0;
   for (long i = 0; i < n; i++)
      d += xy[i] * Ainvxy[i];

   return d;
}

// yes, this function is here just for pGate = 0.9999...
double chi2InvLookup(double const pGate, long const dim)
{
   long const nDeg = 24, nPGate = 9;
   // tbl generated from scipy.stats.distributions's chi2.ppf
   static const double tbl[24][9] = {
       {1.3233036969314664, 2.7055434540954182, 3.8414588206941205,
        6.6348966010212154, 7.8794385766224151, 9.1405934612440198,
        10.8275661706627329, 12.1156651463973812, 15.1367052266236044},
       {2.7725887222397811, 4.6051701859880918, 5.9914645471079799,
        9.2103403719761801, 10.5966347330960726, 11.9829290942160060,
        13.8155105579642736, 15.2018049190843847, 18.4206807439525839},
       {4.1083449356323172, 6.2513886311703217, 7.8147279032511765,
        11.3448667301443695, 12.8381564665986492, 14.3203470978735261,
        16.2662361962381290, 17.7299962289461561, 21.1075134661604409},
       {5.3852690577793902, 7.7794403397348582, 9.4877290367811540,
        13.2767041359876217, 14.8602590005602426, 16.4239361241365565,
        18.4668269529031690, 19.9973549952478500, 23.5127424449910762},
       {6.6256797638292486, 9.2363568997811214, 11.0704976935163515,
        15.0862724693889874, 16.7496023436390438, 18.3856125556843431,
        20.5150056524328761, 22.1053267782076155, 25.7448319590561212},
       {7.8408041205851182, 10.6446406756684215, 12.5915872437439766,
        16.8118938297709271, 18.5475841785110873, 20.2494020514901258,
        22.4577444848253265, 24.1027989949837469, 27.8563412360141704},
       {9.0371475479081393, 12.0170366237805286, 14.0671404493401671,
        18.4753069065823610, 20.2777398749626201, 22.0403905892453729,
        24.3218863478568572, 26.0177677090150290, 29.8775039092251689},
       {10.2188549702467615, 13.3615661365117280, 15.5073130558654526,
        20.0902350296632335, 21.9549549906595303, 23.7744743182941960,
        26.1244815583761394, 27.8680464033826212, 31.8276280012625854},
       {11.3887514404703687, 14.6836565732598388, 16.9189776046204443,
        21.6659943334619207, 23.5893507812573837, 25.4624786978544044,
        27.8771648712565714, 29.6658081035964258, 33.7199484389649058},
       {12.5488613968893770, 15.9871791721052610, 18.3070380532751464,
        23.2092511589543555, 25.1881795719711725, 27.1121710335106805,
        29.5882984450744146, 31.4198125074004864, 35.5640139419523891},
       {13.7006927460115087, 17.2750085175000763, 19.6751375726824946,
        24.7249703113182804, 26.7568489164696324, 28.7293495199512563,
        31.2641336202399884, 33.1366150041685543, 37.3669864379972765},
       {14.8454036710401809, 18.5493477867032439, 21.0260698174830694,
        26.2169673055358494, 28.2995188220460250, 30.3184791305753976,
        32.9094904073602095, 34.8212746364746621, 39.1344038819498010},
       {15.9839062163120538, 19.8119293071275635, 22.3620324948269378,
        27.6882496104570492, 29.8194712236532204, 31.8830854731341482,
        34.5281789748708903, 36.4777937188961516, 40.8706550138362985},
       {17.1169335960000630, 21.0641442129970571, 23.6847913048405800,
        29.1412377406727892, 31.3193496225952899, 33.4260105126694640,
        36.1232736803981354, 38.1094039322700766, 42.5792889531132985},
       {18.2450856024151342, 22.3071295815786890, 24.9957901397286193,
        30.5779141668924836, 32.8013206457918400, 34.9495851396406252,
        37.6972982183538292, 39.7187597896322799, 44.2632249441752776},
       {19.3688602205845122, 23.5418289230961051, 26.2962276048642423,
        31.9999269088151728, 34.2671865378266887, 36.4557494319381092,
        39.2523547907684787, 41.3080737171376313, 45.9248990511138544},
       {20.4886762383915020, 24.7690353439014572, 27.5871116382753279,
        33.4086636050046195, 35.7184656590046146, 37.9461387813766535,
        40.7902167069025268, 42.8792129603366803, 47.5663695581447428},
       {21.6048897957281625, 25.9894230826372024, 28.8692994303926334,
        34.8053057347050725, 37.1564514566067388, 39.4221470341033537,
        42.3123963316799632, 44.4337707398063202, 49.1893944719668283},
       {22.7178067441998550, 27.2035710293568265, 30.1435272056461443,
        36.1908691292700411, 38.5822565549342329, 40.8849737292633009,
        43.8201959645175307, 45.9731195639012498, 50.7954896656222132},
       {23.8276920430308543, 28.4119805843056348, 31.4104328442309253,
        37.5662347866250599, 39.9968463129386365, 42.3356600752502459,
        45.3147466181258594, 47.4984518854720079, 52.3859732730524996},
       {24.9347770149023127, 29.6150894361827355, 32.6705733409173078,
        38.9321726835160646, 41.4010647714175946, 43.7751167828690555,
        46.7970380415613079, 49.0108115952116208, 53.9620001164119500},
       {26.0392650281650155, 30.8132823439530235, 33.9244384714438070,
        40.2893604375938565, 42.7956549993085389, 45.2041459020426046,
        48.2679422908351654, 50.5111187585322909, 55.5245887757052046},
       {27.1413360029765123, 32.0068996817042972, 35.1724616269080386,
        41.6383981188584755, 44.1812752499710939, 46.6234581701167485,
        49.7282324664314928, 52.0001892890786337, 57.0746431385556363},
       {28.2411500255287642, 33.1962442886281792, 36.4150285018072992,
        42.9798201393516237, 45.5585119365305786, 48.0336869509358309,
        51.1785977773773766, 53.4787507719591062, 58.6129697483020493}};
   const double probGate[9] = {0.75,   0.90,  0.95,   0.99,  0.995,
                               0.9975, 0.999, 0.9995, 0.9999};

   if (dim < 1 || dim > nDeg) {
      fprintf(stderr,
              "Dimension for chi2InvLookup() must be between 1 and %li, "
              "inclusive. Exiting...\n",
              nDeg);
      exit(EXIT_FAILURE);
   }
   if (pGate < probGate[0] || pGate > probGate[nPGate - 1]) {
      fprintf(stderr,
              "Probability Gate for chi2InvLookup() must be between %lf and "
              "%lf, inclusive. Exiting...\n",
              probGate[0], probGate[nPGate - 1]);
      exit(EXIT_FAILURE);
   }
   double out = LinInterpTbl(probGate, tbl[dim - 1], pGate, nPGate);
   return (out);
}
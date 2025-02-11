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

/* REQUIRED GLOBALS                                                   */
/*    WorldType World                                                 */
/*    long AtmoOption                                                 */
/*    double SchattenTable[5][410]                                    */

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

extern double EnckeFQ(double const r[3], double const delta[3]);
extern void Legendre(long N, long M, double x, double P[N + 1][M + 1],
                     double sdP[N + 1][M + 1]);
#if REPORT_RESIDUALS == TRUE
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

   meas->data = calloc(meas->dim, sizeof(double));
   meas->R    = calloc(meas->errDim, sizeof(double));
   meas->N    = CreateMatrix(meas->errDim, meas->errDim);
   for (int i = 0; i < meas->errDim; i++) {
      meas->R[i] = sourceMeas->R[i];
      for (int j = 0; j < meas->errDim; j++)
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
   if (m1->ccsdsSeconds < m2->ccsdsSeconds)
      return -1;
   else if (m1->ccsdsSeconds > m2->ccsdsSeconds)
      return +1;
   else if (m1->ccsdsSubseconds < m2->ccsdsSubseconds)
      return -1;
   else if (m1->ccsdsSubseconds > m2->ccsdsSubseconds)
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
   const double gpst0J2000 =
       -7300.5 + (32.184 + 19) / secPerDay; // -7300.499407592695

   const double DaysSinceWeek     = gpsSec / secPerDay;
   const double DaysSinceRollover = DaysSinceWeek + dayperWk * gpsWk;
   const double DaysSinceEpoch =
       DaysSinceRollover + daysperRollover * gpsRollover;
   return ((DaysSinceEpoch + gpst0J2000) * secPerDay);
}

/**********************************************************************/
/* Given a time in seconds since J2000 TT, find the Prime Meridian    */
/* offset angle of a given world.                                     */
double GetPriMerAng(const long orbCenter, const struct DateType *date)
{
   // TODO: change for spice
   struct WorldType *W = &World[orbCenter];
   double PriMerAng    = 0.0;
   const double time   = DateToTime(date->Year, date->Month, date->Day,
                                    date->Hour, date->Minute, date->Second);

   /* This is based on the behavior in Ephemerides() in 42ephem.c */
   switch (orbCenter) {
      case EARTH: {
         if (EphemOption == EPH_MEAN) {
            PriMerAng = W->PriMerAngJ2000 + W->w * time;
         }
         else {
            struct DateType dateUTC = *date;
            updateTime(&dateUTC, -(32.184 + LeapSec));
            PriMerAng = TwoPi * JD2GMST(dateUTC.JulDay);
         }
      } break;
      case LUNA: {
         PriMerAng = LunaPriMerAng(date->JulDay);
      } break;
      case SOL: // TODO: SOL does not rotate
         break;
      case MERCURY:
      case VENUS:
      case JUPITER:
      case SATURN:
      case URANUS:
      case NEPTUNE:
      case PLUTO:
         PriMerAng = W->w * time;

         // TODO: This is from Ephemerides() in 42ephem.c
         if (EphemOption == EPH_MEAN)
            PriMerAng += W->PriMerAngJ2000;
         break;
      case MINORBODY_0:
      case MINORBODY_1:
      case MINORBODY_2:
      case MINORBODY_3:
      case MINORBODY_4:
      case MINORBODY_5:
      case MINORBODY_6:
      case MINORBODY_7:
      case MINORBODY_8:
      case MINORBODY_9:
      default:
         PriMerAng = W->w * time;
         break;
   }
   PriMerAng = fmod(PriMerAng, TwoPi);
   return PriMerAng;
}

//------------------------------------------------------------------------------
// Acceleration perturbation functions
//------------------------------------------------------------------------------
void SphericalHarmonicsJacobian(const long N, const long M, const double r,
                                const double trigs[4], const double Re,
                                const double K, double **C, double **S,
                                double **Norm, double HV[3][3])
{
   double P[N + 1][M + 1], sdP[N + 1][M + 1];
   long n, m;
   double cphi[M + 1], sphi[M + 1];
   double Rern1[N + 1], sth, cth;

   /* .. Order can't be greater than Degree */
   if (M > N) {
      fprintf(stderr, "Order %ld can't be greater than Degree %ld\n", M, N);
      exit(EXIT_FAILURE);
   }

   /* .. Find Legendre functions */
   cth                = trigs[0];
   sth                = trigs[1]; // sin(theta);
   const double sth2  = sth * sth;
   const double cotth = cth / sth;
   const double r2 = r * r, rsth = r * sth;
   const double rsth2 = rsth * rsth;
   Legendre(N, M, cth, P, sdP);

   /* .. Build cos(m*phi) and sin(m*phi) */
   cphi[0] = 1.0;
   sphi[0] = 0.0;
   cphi[1] = trigs[2]; // cos(phi);
   sphi[1] = trigs[3]; // sin(phi);
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

   HV[0][0] = d2Vdr2;
   HV[1][1] = d2Vdtheta2 / r2;
   HV[2][2] = d2Vdphi2 / rsth2;
   HV[0][1] = d2Vdrdtheta / r;
   HV[0][2] = d2Vdrdphi / rsth;
   HV[1][2] = d2Vdphidtheta / (r * rsth);
   HV[1][0] = HV[0][1];
   HV[2][0] = HV[0][2];
   HV[2][1] = HV[1][2];
}

void SphericalHarmonicsHessian(long N, long M, struct WorldType *W,
                               double PriMerAng, double pbn[3],
                               double HgeoN[3][3])
{
   double CEN[3][3] = {{0.0}}, cth, sth, cph, sph, pbe[3], HV[3][3] = {{0.0}};
   double r;
   long i, j, k;
   const struct SphereHarmType *GravModel = &W->GravModel;

   double gradV[3] = {0.0};

   /*    Transform p to ECEF */
   CEN[0][0] = cos(PriMerAng);
   CEN[1][1] = CEN[0][0];
   CEN[2][2] = 1.0;
   CEN[0][1] = sin(PriMerAng);
   CEN[1][0] = -CEN[0][1];
   MxV(CEN, pbn, pbe);

   const double denom = sqrt(pbe[1] * pbe[1] + pbe[0] * pbe[0]);
   getTrigSphericalCoords(pbe, &cth, &sth, &cph, &sph, &r);
   const double trigs[4] = {cth, sth, cph, sph};

   const double MSE[3][3] = {{pbe[0] / r, pbe[1] / r, cth},
                             {cth * cph, cth * sph, -sth},
                             {-sph, cph, 0.0}};

   /*    Find Jacobian */
   SphericalHarmonicsJacobian(N, M, r, trigs, W->rad, W->mu / W->rad,
                              GravModel->C, GravModel->S, GravModel->Norm, HV);

   /*   Calculate scaled Christoffel Symbols */
   /*     sCS^k_{ij} = CS^k_{ij} * sqrt(g_{kk}) / (sqrt(g_{ii})*sqrt(g_{jj})) */
   /*     due to scaling of gradV and scaling in polar transform */
   double sCS[3][3][3] = {{{0.0}}};

   sCS[0][1][1] = -1.0 / r;      // -r * 1 / (r*r) = -1 / r
   sCS[0][2][2] = sCS[0][1][1];  // -rsth*sth * 1 / (rsth*rsth) = -1/r
   sCS[1][0][1] = -sCS[0][1][1]; // 1/r * r / (1*r) = 1/r
   sCS[1][1][0] = sCS[1][0][1];
   sCS[1][2][2] =
       -pbe[2] / (r * denom);    // -sth*cth * r / (rsth*rsth) = -cth / rsth
   sCS[2][0][2] = sCS[1][0][1];  // 1/r * rsth / (1*rsth) = 1 / r
   sCS[2][1][2] = -sCS[1][2][2]; // cth/sth * rsth / (r*rsth) = cth / rsth
   sCS[2][2][0] = sCS[2][0][2];
   sCS[2][2][1] = sCS[2][1][2];

   SphericalHarmonics(N, M, r, trigs, W->rad, W->mu / W->rad, GravModel->C,
                      GravModel->S, GravModel->Norm, gradV);
   for (k = 0; k < 3; k++)
      for (i = 0; i < 3; i++)
         for (j = 0; j < 3; j++)
            HV[i][j] -= gradV[k] * sCS[k][i][j];

   /*    Transform back to cartesian coords in Newtonian frame */
   double CSN[3][3];
   MxM(MSE, CEN, CSN);
   AdjointT(CSN, HV, HgeoN);
}

void getGravAccel(double const mu, double const pos[3], double gravFrc[3])
{
   int i;
   double posHat[3];
   const double posMag    = CopyUnitV(pos, posHat);
   const double gravScale = -mu / (posMag * posMag);
   for (i = 0; i < 3; i++)
      gravFrc[i] = posHat[i] * gravScale;
}

void getDGravFrcDPos(double const mu, double const pos[3],
                     double dGravFrcdPos[3][3])
{
   int i, j;
   double posHat[3];
   for (i = 0; i < 3; i++)
      posHat[i] = pos[i];
   const double posMag    = UNITV(posHat);
   const double gravScale = -mu / (posMag * posMag * posMag);

   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++)
         dGravFrcdPos[i][j] = -posHat[i] * posHat[j] * 3.0;
      dGravFrcdPos[i][i] += 1.0;
      for (j = 0; j < 3; j++)
         dGravFrcdPos[i][j] *= gravScale;
   }
}

void ThirdBodyGravAccel(double p[3], double s[3], double mu, double accel[3])
{
   const double magp = MAGV(p);
   const double mags = MAGV(s);
   const double p3   = magp * magp * magp;
   const double s3   = mags * mags * mags;
   for (long j = 0; j < 3; j++)
      accel[j] = mu * (s[j] / s3 - p[j] / p3);
}

void NavGravPertAccel(struct DSMNavType *Nav, const struct DateType *date,
                      const double PosR[3], const double mass,
                      const struct OrbitType *O, double VelRdot[3])
{
   double ph[3], pn[3], pr[3], s[3], accelR[3];
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
   for (j = 0; j < 3; j++)
      VelRdot[j] = 0.0;

   for (Iw = SOL; Iw <= PLUTO; Iw++) {
      if (World[Iw].Exists && !(Iw == OrbCenter || Iw == SecCenter)) {
         for (j = 0; j < 3; j++)
            ph[j] = World[Iw].PosH[j] - WCenter->PosH[j];
         MxV(WCenter->CNH, ph, pn);
         MxV(Nav->refCRN, pn, pr);
         for (j = 0; j < 3; j++)
            s[j] = pr[j] - PosR[j];
         ThirdBodyGravAccel(pr, s, World[Iw].mu, accelR);
         for (j = 0; j < 3; j++)
            VelRdot[j] += accelR[j];
      }
   }
   /* Moons of OrbCenter (but not SecCenter) */
   if (OrbCenter != SOL) {
      for (Im = 0; Im < WCenter->Nsat; Im++) {
         Iw = WCenter->Sat[Im];
         if (Iw != SecCenter) {
            for (j = 0; j < 3; j++)
               pn[j] = World[Iw].eph.PosN[j];
            MxV(Nav->refCRN, pn, pr);
            for (j = 0; j < 3; j++)
               s[j] = pr[j] - PosR[j];
            ThirdBodyGravAccel(pr, s, World[Iw].mu, accelR);
            for (j = 0; j < 3; j++)
               VelRdot[j] += accelR[j];
         }
      }
   }
   /* Moons of SecCenter */
   if (O->Regime == ORB_THREE_BODY) {
      for (Im = 0; Im < World[SecCenter].Nsat; Im++) {
         Iw = World[SecCenter].Sat[Im];
         for (j = 0; j < 3; j++)
            pn[j] = World[Iw].eph.PosN[j];
         MTxV(World[SecCenter].CNH, pn, ph);
         MxV(WCenter->CNH, ph, pn);
         for (j = 0; j < 3; j++)
            pn[j] += World[SecCenter].eph.PosN[j];
         MxV(Nav->refCRN, pn, pr);
         for (j = 0; j < 3; j++)
            s[j] = pr[j] - PosR[j];

         ThirdBodyGravAccel(pr, s, World[Iw].mu, accelR);
         for (j = 0; j < 3; j++)
            VelRdot[j] += accelR[j];
      }
   }

   // TODO: maybe make this just 2/0 if it exists
   /* Perturbations due to non-spherical gravity potential */
   const struct SphereHarmType *GravModel = &WCenter->GravModel;
   if (GravModel->N >= 2) {
      double PriMerAng = GetPriMerAng(OrbCenter, date);
      double fGeoN[3], fGeoR[3], PosN[3];
      MTxV(Nav->refCRN, PosR, PosN);
      SphericalHarmGravForce(GravModel->N, GravModel->M, WCenter, PriMerAng,
                             mass, PosN, fGeoN);
      MxV(Nav->refCRN, fGeoN, fGeoR);
      for (j = 0; j < 3; j++)
         VelRdot[j] += fGeoR[j] / mass;
   }
}

void NavDGravPertAccelDPos(struct DSMNavType *Nav, const struct DateType *date,
                           double PosR[3], struct OrbitType const *O,
                           double dGravDPos[3][3])
{
   double ph[3], pn[3], pr[3], s[3], dGdR[3][3];
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
   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         dGravDPos[i][j] = 0.0;

   for (Iw = SOL; Iw <= PLUTO; Iw++) {
      if (World[Iw].Exists && !(Iw == OrbCenter || Iw == SecCenter)) {
         for (j = 0; j < 3; j++)
            ph[j] = World[Iw].PosH[j] - WCenter->PosH[j];
         MxV(WCenter->CNH, ph, pn);
         MxV(Nav->refCRN, pn, pr);
         for (j = 0; j < 3; j++)
            s[j] = pr[j] - PosR[j];
         getDGravFrcDPos(World[Iw].mu, s, dGdR);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               dGravDPos[i][j] += dGdR[i][j];
      }
   }
   /* Moons of OrbCenter (but not SecCenter) */
   if (OrbCenter != SOL) {
      for (Im = 0; Im < WCenter->Nsat; Im++) {
         Iw = WCenter->Sat[Im];
         if (Iw != SecCenter) {
            for (j = 0; j < 3; j++)
               pn[j] = World[Iw].eph.PosN[j];
            MxV(Nav->refCRN, pn, pr);
            for (j = 0; j < 3; j++)
               s[j] = pr[j] - PosR[j];
            getDGravFrcDPos(World[Iw].mu, s, dGdR);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  dGravDPos[i][j] += dGdR[i][j];
         }
      }
   }
   /* Moons of SecCenter */
   if (O->Regime == ORB_THREE_BODY) {
      for (Im = 0; Im < World[SecCenter].Nsat; Im++) {
         Iw = World[SecCenter].Sat[Im];
         for (j = 0; j < 3; j++)
            pn[j] = World[Iw].eph.PosN[j];
         MTxV(World[SecCenter].CNH, pn, ph);
         MxV(WCenter->CNH, ph, pn);
         for (j = 0; j < 3; j++)
            pn[j] += World[SecCenter].eph.PosN[j];
         MxV(Nav->refCRN, pn, pr);
         for (j = 0; j < 3; j++)
            s[j] = pr[j] - PosR[j];

         getDGravFrcDPos(World[Iw].mu, s, dGdR);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               dGravDPos[i][j] += dGdR[i][j];
      }
   }

   // TODO: maybe make this just 2/0 if it exists
   /* Perturbations due to non-spherical gravity potential */
   const struct SphereHarmType *GravModel = &WCenter->GravModel;
   if (GravModel->N >= 2) {
      const double PriMerAng = GetPriMerAng(OrbCenter, date);
      double HgeoN[3][3] = {{0.0}}, HgeoR[3][3] = {{0.0}}, PosN[3] = {0.0};
      MTxV(Nav->refCRN, PosR, PosN);
      SphericalHarmonicsHessian(GravModel->N, GravModel->M, WCenter, PriMerAng,
                                PosN, HgeoN);
      if (Nav->refFrame != FRAME_N) {
         Adjoint(Nav->refCRN, HgeoN, HgeoR);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               dGravDPos[i][j] += HgeoR[i][j];
      }
      else {
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               dGravDPos[i][j] += HgeoN[i][j];
      }
   }
}

void getAeroForceAndTorque(struct DSMType *const DSM, double const CRB[3][3],
                           double const PosR[3], double const VelR[3],
                           double const worldW, double const AtmoDensity,
                           double frcR[3], double trq[3])
{
   // TODO: be able to choose between ballistic coef model and more accurate
   // model basllistic coef is noticeably faster and simplification doesn't
   // change much if torque is trivial
   // Higher fidelity model requires information that exists only in SCType

   const struct DSMNavType *Nav = &DSM->DsmNav;
   double worldWR[3] = {0.0}, VrelR[3] = {0.0};
   double PosRWorld[3] = {0.0};
   for (int i = 0; i < 3; i++)
      PosRWorld[i] = PosR[i] + Nav->refPos[i];
   for (int i = 0; i < 3; i++)
      worldWR[i] = -Nav->refCRN[i][2] * worldW;
   VxV(worldWR, PosRWorld, VrelR);
   for (int i = 0; i < 3; i++)
      VrelR[i] += VelR[i] + Nav->refVel[i];

   double VrelRHat[3]     = {0.0};
   const double WindSpeed = CopyUnitV(VrelR, VrelRHat);
   const double Coef1 = -0.5 * AtmoDensity * WindSpeed * WindSpeed * DSM->mass /
                        Nav->ballisticCoef;
   for (int i = 0; i < 3; i++) {
      frcR[i] = Coef1 * VrelRHat[i];
      trq[i]  = 0.0;
   }
}

void getDAeroFrcAndTrqDVRel(struct DSMType *const DSM, double const CRB[3][3],
                            double const PosR[3], double const VelR[3],
                            double const worldW, double const AtmoDensity,
                            double dAeroFrcdVRel[3][3],
                            double dAeroTrqdVRel[3][3])
{
   // TODO: be able to choose between ballistic coef model and more accurate
   // model basllistic coef is noticeably faster and simplification doesn't
   // change much if torque is trivial
   // Higher fidelity model requires information that exists only in SCType

   const struct DSMNavType *Nav = &DSM->DsmNav;
   double worldWR[3] = {0.0}, VrelR[3] = {0.0};
   double PosRWorld[3] = {0.0};
   for (int i = 0; i < 3; i++)
      PosRWorld[i] = PosR[i] + Nav->refPos[i];
   for (int i = 0; i < 3; i++)
      worldWR[i] = -Nav->refCRN[i][2] * worldW;
   VxV(worldWR, PosRWorld, VrelR);
   for (int i = 0; i < 3; i++)
      VrelR[i] += VelR[i] + Nav->refVel[i];

   double VrelRHat[3]     = {0.0};
   const double WindSpeed = CopyUnitV(VrelR, VrelRHat);
   const double Coef1 =
       -0.5 * AtmoDensity * WindSpeed * DSM->mass / Nav->ballisticCoef;
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
         dAeroFrcdVRel[i][j] = VrelRHat[i] * VrelRHat[j] * Coef1;
         dAeroTrqdVRel[i][j] = 0.0;
      }
      dAeroFrcdVRel[i][i] += Coef1;
   }
}

//------------------------------------------------------------------------------
//                               NAV FUNCTIONS
//------------------------------------------------------------------------------

double **gyroJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                         const long Igyro)
{
   double tmp[3] = {0.0}, tmp2[3] = {0.0};
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
         MTxV(Nav->CRB, Nav->refOmega, tmp);
         for (i = 0; i < 3; i++)
            tmp[i] += Nav->wbr[i];
         VxV(tmp, gyro->Axis, tmp2);

         for (i = 0; i < 3; i++)
            B[0][i] = tmp2[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
         for (i = 0; i < 3; i++)
            B[0][i] = -gyro->Axis[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[OMEGA_STATE], 1, 3);
         break;
      case RIEKF_NAV: {
         double axisR[3] = {0.0};
         MxV(Nav->CRB, gyro->Axis, axisR);
         for (i = 0; i < 3; i++)
            B[0][i] = -axisR[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[OMEGA_STATE], 1, 3);

         VxV(Nav->refOmega, axisR, tmp2);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp2[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
      } break;
      case MEKF_NAV:
         for (i = 0; i < 3; i++)
            B[0][i] = -gyro->Axis[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[OMEGA_STATE], 1, 3);
         if (Nav->refFrame != FRAME_N) {
            QxV(Nav->qbr, Nav->refOmega, tmp);
            VxV(tmp, gyro->Axis, tmp2);

            for (i = 0; i < 3; i++)
               B[0][i] = tmp2[i] * R2D;
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
                        const long Imag)
{
   double tmp[3] = {0.0}, tmp2[3] = {0.0};
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
         MxV(Nav->refCRN, AC->bvn, tmp);
         MTxV(Nav->CRB, tmp, tmp2);
         VxV(tmp2, mag->Axis, tmp);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp[i] * T2mG;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
         break;
      case RIEKF_NAV: {
         double axisR[3] = {0.0};
         MxV(Nav->refCRN, AC->bvn, tmp2);
         MxV(Nav->CRB, mag->Axis, axisR);
         VxV(tmp2, axisR, tmp);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp[i] * T2mG;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
      } break;
      case MEKF_NAV:
         MxV(Nav->refCRN, AC->bvn, tmp);
         QxV(Nav->qbr, tmp, tmp2);
         VxV(tmp2, mag->Axis, tmp);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp[i] * T2mG;
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
                        const long Icss)
{
   double tmp[3] = {0.0}, svb[3] = {0.0}, svr[3] = {0.0};
   static double **B = NULL; // if its static, just need to allocate once,
                             // instead of allocate/deallocate
   const struct DSMNavType *Nav = &DSM->DsmNav;
   const struct AcCssType *css  = &AC->CSS[Icss];
   long i;

   if (B == NULL)
      B = CreateMatrix(1, 3);

   for (i = 0; i < 3; i++)
      B[0][i] = 0.0;

   MxV(Nav->refCRN, AC->svn, svr);

   double **jacobian =
       CreateMatrix(Nav->measTypes[CSS_SENSOR][Icss].dim, Nav->navDim);

   switch (Nav->type) { // will need to figure something out with albedo if that
                        // is active
      case LIEKF_NAV:
         MTxV(Nav->CRB, svr, svb);
         VxV(svb, css->Axis, tmp);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp[i] * css->Scale;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
         break;
      case RIEKF_NAV:
         MxV(Nav->CRB, css->Axis, tmp);
         VxV(svr, tmp, svb);
         for (i = 0; i < 3; i++)
            B[0][i] = svb[i] * css->Scale;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
         break;
      case MEKF_NAV:
         QxV(Nav->qbr, svr, svb);
         VxV(svb, css->Axis, tmp);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp[i] * css->Scale;
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
                        const long Ifss)
{
   double B[3][3] = {{0.0}}, tmp3x3[3][3] = {{0.0}};
   const struct AcFssType *fss  = &AC->FSS[Ifss];
   const struct DSMNavType *Nav = &DSM->DsmNav;
   static double **tmpAssign    = NULL;
   double svb[3], svs[3], CBN[3][3];
   double bhat[3] = {0.0}, hhat[3] = {0.0}, vhat[3] = {0.0};
   double bxsvs[3], hxsvs[3], vxsvs[3];
   long i, j;

   const long BoreAxis = fss->BoreAxis;
   const long H_Axis   = fss->H_Axis;
   const long V_Axis   = fss->V_Axis;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(2, 3);

   for (i = 0; i < 2; i++)
      for (j = 0; j < 3; j++)
         tmpAssign[i][j] = 0.0;

   MTxM(Nav->CRB, Nav->refCRN, CBN);
   MxV(CBN, AC->svn, svb);
   MxV(fss->CB, svb, svs);

   bhat[BoreAxis] = 1.0;
   hhat[H_Axis]   = 1.0;
   vhat[V_Axis]   = 1.0;

   VxV(bhat, svs, bxsvs);
   VxV(hhat, svs, hxsvs);
   VxV(vhat, svs, vxsvs);

   const double svsb = svs[BoreAxis];
   const double svsh = svs[H_Axis];
   const double svsv = svs[V_Axis];

   switch (fss->type) {
      case CONVENTIONAL_FSS: {
         const double denomA = 1.0 / (svsb * svsb + svsh * svsh);
         const double denomB = 1.0 / (svsb * svsb + svsv * svsv);
         for (i = 0; i < 3; i++) {
            B[0][i] = (svsh * bxsvs[i] - svsb * hxsvs[i]) * denomA;
            B[1][i] = (svsv * bxsvs[i] - svsb * vxsvs[i]) * denomB;
         }
      } break;
      case GS_FSS: {
         const double denomA = -1.0 / sqrt(1.0 - svsb * svsb);
         const double denomB = 1.0 / (svsh * svsh + svsv * svsv);
         for (i = 0; i < 3; i++) {
            B[0][i] = bxsvs[i] * denomA;
            B[1][i] = (svsh * vxsvs[i] - svsv * hxsvs[i]) * denomB;
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
         MxM(B, fss->CB, tmp3x3);
         for (i = 0; i < 2; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = tmp3x3[i][j];
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[ROTMAT_STATE], 2, 3);
         break;
      case RIEKF_NAV:
         MxM(B, fss->CB, tmp3x3);
         MxMT(tmp3x3, Nav->CRB, B);
         for (i = 0; i < 2; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = B[i][j];
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[ROTMAT_STATE], 2, 3);
         break;
      case MEKF_NAV:
         MxM(B, fss->CB, tmp3x3);
         for (i = 0; i < 2; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = tmp3x3[i][j];
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
                              struct DSMType *const DSM, const long Ist)
{
   double tmpM[3][3]                  = {{0.0}}, CSB[3][3];
   static double **tmpAssign          = NULL;
   const struct DSMNavType *Nav       = &DSM->DsmNav;
   const struct AcStarTrackerType *st = &AC->ST[Ist];
   long i, j;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         tmpAssign[i][j] = 0.0;

   double **jacobian =
       CreateMatrix(Nav->measTypes[STARTRACK_SENSOR][Ist].errDim, Nav->navDim);
   Q2C(st->qb, CSB);

   switch (Nav->type) {
      case LIEKF_NAV:
      case MEKF_NAV:
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = -CSB[i][j];
         if (Nav->type == LIEKF_NAV)
            subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[ROTMAT_STATE], 3, 3);
         else
            subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[QUAT_STATE], 3, 3);
         break;
      case RIEKF_NAV:
         MxMT(CSB, Nav->CRB, tmpM);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = -tmpM[i][j];
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[ROTMAT_STATE], 3, 3);
         break;
      default:
         fprintf(stderr, "Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double **gpsJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                        const long Igps)
{
   double tmp1[3][3] = {{0.0}}, tmp2[3][3] = {{0.0}}, tmp3[3][3] = {{0.0}},
          tmpX[3][3] = {{0.0}}, tmpV[3] = {0.0};
   static double **tmpAssign    = NULL;
   const struct DSMNavType *Nav = &DSM->DsmNav;
   long i, j;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         tmpAssign[i][j] = 0.0;

   double **jacobian =
       CreateMatrix(Nav->measTypes[GPS_SENSOR][Igps].dim, Nav->navDim);

   switch (Nav->type) {
      case LIEKF_NAV:
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmp1[i][j] = Nav->CRB[i][j];
         if (Nav->refFrame != FRAME_N) {
            MTxM(Nav->refCRN, tmp1, tmp2);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  tmp1[i][j] = tmp2[i][j];

            MTxV(Nav->refCRN, Nav->refOmega, tmpV);
            V2CrossM(tmpV, tmpX);
            MTxM(Nav->refCRN, tmpX, tmp2);
            MxM(tmp2, tmp1, tmpX);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  tmpAssign[i][j] = -tmpX[i][j];
            subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[POS_STATE], 3, 3);
         }
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = -tmp1[i][j];
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[POS_STATE], 3, 3);
         subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[VEL_STATE], 3, 3);
         break;
      case RIEKF_NAV:
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmp1[i][j] = Nav->refCRN[j][i];
         if (Nav->refFrame != FRAME_N) {
            V2CrossM(Nav->refOmega, tmpX);
            MTxM(Nav->refCRN, tmpX, tmp2);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  tmpAssign[i][j] = -tmp2[i][j];
            subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[POS_STATE], 3, 3);
         }
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = -tmp1[i][j];
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[POS_STATE], 3, 3);
         subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[VEL_STATE], 3, 3);

         V2CrossM(Nav->PosR, tmpX);
         MxM(tmp1, tmpX, tmp3);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = tmp3[i][j];
         subMatAdd(jacobian, tmpAssign, 0, Nav->navInd[ROTMAT_STATE], 3, 3);
         if (Nav->refFrame != FRAME_N) {
            MxM(tmp2, tmpX, tmp3);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  tmpAssign[i][j] = tmp3[i][j];
            subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[ROTMAT_STATE], 3, 3);
         }

         V2CrossM(Nav->VelR, tmpX);
         MxM(tmp1, tmpX, tmp3);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = tmp3[i][j];
         subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[ROTMAT_STATE], 3, 3);
         break;
      case MEKF_NAV:
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmp1[i][j] = Nav->refCRN[j][i];
         if (Nav->refFrame != FRAME_N) {
            V2CrossM(Nav->refOmega, tmpX);
            MTxM(Nav->refCRN, tmpX, tmp2);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  tmpAssign[i][j] = -tmp2[i][j];
            subMatAdd(jacobian, tmpAssign, 3, Nav->navInd[POS_STATE], 3, 3);
         }
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = -tmp1[i][j];
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

double **accelJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                          const long Iaccel)
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
   double wbn[3], wrn[3];
   long i;

   double *gyroEst = calloc(1, sizeof(double));

   MTxV(Nav->CRB, Nav->refOmega, wrn);
   for (i = 0; i < 3; i++)
      wbn[i] = Nav->wbr[i] + wrn[i];
   gyroEst[0] = VoV(G->Axis, wbn) * R2D;
   return (gyroEst);
}

double *magFun(struct AcType *const AC, struct DSMType *const DSM,
               const long Imag)
{
   const struct AcMagnetometerType *MAG = &AC->MAG[Imag];
   const struct DSMNavType *Nav         = &DSM->DsmNav;
   double bvb[3], bvn[3], CBN[3][3];
   const double T2mG = 1.0e7; // tesla to milligauss
   long i;

   double *magEst = calloc(1, sizeof(double));

   // Not to really be used.
   for (i = 0; i < 3; i++)
      bvn[i] = AC->bvn[i];

   MTxM(Nav->CRB, Nav->refCRN, CBN);
   MxV(CBN, bvn, bvb);
   magEst[0] = VoV(MAG->Axis, bvb) * T2mG;

   return (magEst);
}

double *cssFun(struct AcType *const AC, struct DSMType *const DSM,
               const long Icss)
{
   const struct AcCssType *css  = &AC->CSS[Icss];
   const struct DSMNavType *Nav = &DSM->DsmNav;
   double svb[3], CBN[3][3];

   double *IllumEst = calloc(1, sizeof(double));

   MTxM(Nav->CRB, Nav->refCRN, CBN);
   MxV(CBN, AC->svn, svb);
   IllumEst[0] = VoV(svb, css->Axis) * css->Scale;

   return (IllumEst);
}

double *fssFun(struct AcType *const AC, struct DSMType *const DSM,
               const long Ifss)
{
   const struct AcFssType *fss  = &AC->FSS[Ifss];
   const struct DSMNavType *Nav = &DSM->DsmNav;
   double svb[3], svs[3], CBN[3][3];

   double *SunAngEst = calloc(2, sizeof(double));

   const long BoreAxis = fss->BoreAxis;
   const long H_Axis   = fss->H_Axis;
   const long V_Axis   = fss->V_Axis;

   MTxM(Nav->CRB, Nav->refCRN, CBN);
   MxV(CBN, AC->svn, svb);
   MxV(fss->CB, svb, svs);

   switch (fss->type) {
      case CONVENTIONAL_FSS: {
         SunAngEst[0] = atan2(svs[H_Axis], svs[BoreAxis]);
         SunAngEst[1] = atan2(svs[V_Axis], svs[BoreAxis]);
      } break;
      case GS_FSS: {
         SunAngEst[0] = atan2(svs[V_Axis], svs[H_Axis]);
         SunAngEst[1] =
             atan2(sqrt(svs[V_Axis] * svs[V_Axis] + svs[H_Axis] * svs[H_Axis]),
                   svs[BoreAxis]);
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
   double qbn[4], qrn[4];

   double *qsnEst = calloc(4, sizeof(double));

   C2Q(Nav->refCRN, qrn);
   QxQ(Nav->qbr, qrn, qbn);
   QxQ(st->qb, qbn, qsnEst);

   return (qsnEst);
}

double *gpsFun(struct AcType *const AC, struct DSMType *const DSM,
               const long Igps)
{
   const struct DSMNavType *Nav = &DSM->DsmNav;
   double tmp3V[3], tmpPosN[3], tmpVelN[3];
   long i;

   double *posNVelNEst = calloc(6, sizeof(double));

   for (i = 0; i < 3; i++)
      tmp3V[i] = Nav->PosR[i] + Nav->refPos[i];
   MTxV(Nav->refCRN, tmp3V, tmpPosN);
   for (i = 0; i < 3; i++)
      tmp3V[i] = Nav->VelR[i] + Nav->refVel[i];
   MTxV(Nav->refCRN, tmp3V, tmpVelN);
   if (Nav->refFrame != FRAME_N) {
      double wrn[3], wxr[3];
      MTxV(Nav->refCRN, Nav->refOmega, wrn);
      VxV(wrn, tmpPosN, wxr);
      for (i = 0; i < 3; i++)
         tmpVelN[i] += wxr[i];
   }

   for (i = 0; i < 3; i++) {
      posNVelNEst[i]     = tmpPosN[i];
      posNVelNEst[3 + i] = tmpVelN[i];
   }

   return (posNVelNEst);
}

// don't need this at the moment, WIP
double *accelFun(struct AcType *const AC, struct DSMType *const DSM,
                 const long Ia)
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
void getEarthAtmoParams(const double JD, double *NavFlux10p7,
                        double *NavGeomagIndex)
{
   if (AtmoOption == TWOSIGMA_ATMO) {
      *NavFlux10p7    = LinInterp(SchattenTable[0], SchattenTable[1], JD, 410);
      *NavGeomagIndex = LinInterp(SchattenTable[0], SchattenTable[3], JD, 410);
   }
   else if (AtmoOption == NOMINAL_ATMO) {
      *NavFlux10p7    = LinInterp(SchattenTable[0], SchattenTable[2], JD, 410);
      *NavGeomagIndex = LinInterp(SchattenTable[0], SchattenTable[4], JD, 410);
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
                         const struct DateType *date, const double CRB[3][3],
                         const double qbr[4], const double PosR[3],
                         const double VelR[3], const double wbr[3],
                         const double whlH[AC->Nwhl], const double AtmoDensity)
{
   double tmpM[3][3] = {{0.0}}, tmpM2[3][3] = {{0.0}}, tmpM3[3][3] = {{0.0}},
          tmpV[3] = {0.0}, tmpV2[3] = {0.0}, tmpV3[3] = {0.0};
   static double **tmpAssign = NULL;
   double wrnd[3]            = {0.0};
   struct DSMNavType *Nav    = &DSM->DsmNav;
   long i, j, rowInd;
   enum States state;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   switch (Nav->refFrame) {
      case FRAME_N:
         for (i = 0; i < 3; i++)
            wrnd[i] = 0.0;
         break;
      case FRAME_L:
         break;
      case FRAME_F:
         break;
   }

   for (i = 0; i < Nav->navDim; i++)
      for (j = 0; j < Nav->navDim; j++)
         Nav->jacobian[i][j] = 0.0;

   if (Nav->stateActive[ROTMAT_STATE] && Nav->stateActive[POS_STATE] &&
       Nav->stateActive[VEL_STATE] && Nav->stateActive[OMEGA_STATE]) {
      for (state = INIT_STATE; state <= FIN_STATE; state++) {
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = 0.0;
         if (Nav->stateActive[state] == TRUE) {
            rowInd = Nav->navInd[state];
            switch (state) {
               case TIME_STATE:
                  tmpAssign[0][0] = 1.0;
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 1, 1);
                  break;
               case ROTMAT_STATE:
                  if (Nav->stateActive[OMEGA_STATE]) {
                     for (i = 0; i < 3; i++) {
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = 0.0;
                        tmpAssign[i][i] = 1.0;
                     }
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[OMEGA_STATE], 3, 3);
                  }
                  break;
               case QUAT_STATE:
                  break;
               case OMEGA_STATE:
                  MxV(CRB, wbr, tmpV);
                  V2CrossM(tmpV, tmpM);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = tmpM[i][j];
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3, 3);

                  // calculate dwbn_dot/dwbn
                  MTxV(CRB, Nav->refOmega, tmpV3);
                  for (i = 0; i < 3; i++)
                     tmpV3[i] += wbr[i];
                  V2CrossM(tmpV3, tmpM2);
                  MxM(tmpM2, DSM->MOI, tmpM3);
                  MxV(DSM->MOI, tmpV3, tmpV2);
                  for (long Iw = 0; Iw < AC->Nwhl; Iw++)
                     for (i = 0; i < 3; i++)
                        tmpV2[i] += AC->Whl[Iw].Axis[i] * whlH[Iw];

                  V2CrossM(tmpV2, tmpM);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpM[i][j] -= tmpM3[i][j];

                  MINVxM3(DSM->MOI, 3, tmpM, tmpM2);
                  Adjoint(CRB, tmpM2, tmpM);
                  // use tmpM = dwbn_dot/dwbn to calc a few derivs
                  if (Nav->refFrame != FRAME_N) {
                     V2CrossM(Nav->refOmega, tmpM2);
                     MxM(tmpM, tmpM2, tmpM3);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = tmpM3[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[ROTMAT_STATE], 3, 3);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpM[i][j] -= tmpM2[i][j];
                  }
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = tmpM[i][j];
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3, 3);
                  break;
               case POS_STATE:
                  for (i = 0; i < 3; i++) {
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = 0.0;
                     tmpAssign[i][i] = 1.0;
                  }
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                            Nav->navInd[VEL_STATE], 3, 3);
                  if (Nav->stateActive[VEL_STATE] == FALSE) {
                     V2CrossM(VelR, tmpM);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = tmpM[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[ROTMAT_STATE], 3, 3);
                  }
                  if (Nav->stateActive[OMEGA_STATE] == TRUE) {
                     V2CrossM(PosR, tmpM);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = tmpM[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[OMEGA_STATE], 3, 3);
                  }
                  break;
               case VEL_STATE:
                  if (Nav->refFrame != FRAME_N) {
                     SxV(2.0, Nav->refOmega, tmpV2);
                     V2CrossM(tmpV2, tmpM);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = -tmpM[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3, 3);
                     V2CrossM(VelR, tmpM2);
                     MxM(tmpM, tmpM2, tmpM3);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = tmpM3[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[ROTMAT_STATE], 3, 3);

                     V2DoubleCrossM(Nav->refOmega, tmpM);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = -tmpM[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);
                     V2CrossM(PosR, tmpM2);
                     MxM(tmpM, tmpM2, tmpM3);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = tmpM3[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[ROTMAT_STATE], 3, 3);

                     V2CrossM(wrnd, tmpM);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = -tmpM[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);
                     MxM(tmpM, tmpM2, tmpM3);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = tmpM3[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[ROTMAT_STATE], 3, 3);
                  }
                  if (Nav->stateActive[OMEGA_STATE] == TRUE) {
                     V2CrossM(VelR, tmpM);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = tmpM[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[OMEGA_STATE], 3, 3);
                  }
                  break;
               default:
                  break;
            }
         }
      }
      if (DSM->refOrb->Regime == ORB_CENTRAL) {
         long orbCenter             = DSM->refOrb->World;
         double dAeroFrcdVRel[3][3] = {{0.0}}, dAeroTrqdVRel[3][3] = {{0.0}};
         double worldWR[3] = {0.0};
         if (AeroActive) {
            double worldW = World[orbCenter].w;
            for (i = 0; i < 3; i++)
               worldWR[i] = -Nav->refCRN[i][2] * worldW;
            getDAeroFrcAndTrqDVRel(DSM, CRB, PosR, VelR, worldW, AtmoDensity,
                                   dAeroFrcdVRel, dAeroTrqdVRel);
            MxM(Nav->CRB, dAeroTrqdVRel, tmpM);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++) {
                  dAeroTrqdVRel[i][j]  = tmpM[i][j];
                  dAeroFrcdVRel[i][j] /= DSM->mass;
               }
         }
         for (state = INIT_STATE; state <= FIN_STATE; state++) {
            if (Nav->stateActive[state] == TRUE) {
               rowInd = Nav->navInd[state];
               switch (state) {
                  case VEL_STATE:
                     if (Nav->stateActive[ROTMAT_STATE] == TRUE) {
                        double VelRdot[3] = {0.0};
                        if (GravPertActive) {
                           double accelR[3] = {0.0};
                           for (i = 0; i < 3; i++)
                              tmpV[i] = PosR[i] + Nav->refPos[i];
                           NavGravPertAccel(Nav, date, tmpV, 1.0, DSM->refOrb,
                                            accelR);
                           for (i = 0; i < 3; i++)
                              VelRdot[i] += accelR[i];
                        }
                        // TODO: transition to Encke's method, but refAccel for
                        // SC reference would need to be gravity free
                        for (i = 0; i < 3; i++)
                           tmpV[i] = PosR[i] + Nav->refPos[i];
                        getGravAccel(DSM->refOrb->mu, tmpV, tmpV2);
                        for (i = 0; i < 3; i++)
                           VelRdot[i] += tmpV2[i];
                        if (Nav->refOriType != ORI_WORLD) {
                           for (i = 0; i < 3; i++)
                              VelRdot[i] -= Nav->refAccel[i];
                        }
                        V2CrossM(VelRdot, tmpM);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = tmpM[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                                  Nav->navInd[ROTMAT_STATE], 3, 3);
                     }

                     for (i = 0; i < 3; i++)
                        tmpV2[i] = PosR[i] + Nav->refPos[i];
                     getDGravFrcDPos(World[orbCenter].mu, tmpV2, tmpM2);
                     if (GravPertActive) {
                        NavDGravPertAccelDPos(Nav, date, tmpV2, DSM->refOrb,
                                              tmpM);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpM2[i][j] += tmpM[i][j];
                     }
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = tmpM2[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);
                     V2CrossM(PosR, tmpM);
                     MxM(tmpM2, tmpM, tmpM3);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = -tmpM3[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[ROTMAT_STATE], 3, 3);

                     if (AeroActive) {
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = dAeroFrcdVRel[i][j];

                        subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3,
                                  3);
                        V2CrossM(VelR, tmpM2);
                        MxM(dAeroFrcdVRel, tmpM2, tmpM3);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = -tmpM3[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                                  Nav->navInd[ROTMAT_STATE], 3, 3);
                        V2CrossM(worldWR, tmpM2);
                        MxM(dAeroFrcdVRel, tmpM2, tmpM3);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = tmpM3[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                                  Nav->navInd[POS_STATE], 3, 3);
                        V2CrossM(PosR, tmpM2);
                        MxM(tmpM3, tmpM2, tmpM);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = -tmpM[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                                  Nav->navInd[ROTMAT_STATE], 3, 3);
                     }
                     break;
                  case OMEGA_STATE:
                     if (AeroActive) {
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = dAeroTrqdVRel[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                                  Nav->navInd[VEL_STATE], 3, 3);
                        V2CrossM(VelR, tmpM2);
                        MxM(dAeroTrqdVRel, tmpM2, tmpM3);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = -tmpM3[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                                  Nav->navInd[ROTMAT_STATE], 3, 3);
                        V2CrossM(worldWR, tmpM2);
                        MxM(dAeroTrqdVRel, tmpM2, tmpM3);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = tmpM3[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                                  Nav->navInd[POS_STATE], 3, 3);
                        V2CrossM(PosR, tmpM2);
                        MxM(tmpM3, tmpM2, tmpM);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = -tmpM[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
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
   double theta[3] = {0.0}, tmpM[3][3] = {{0.0}}, tmpV[3] = {0.0};
   double dR[3][3] = {{0.0}}, dr[3] = {0.0}, dv[3] = {0.0}, dw[3] = {0.0};
   long i, j;

   const long nRVec = Nav->stateActive[POS_STATE] + Nav->stateActive[VEL_STATE];
   const long nBVec = Nav->stateActive[OMEGA_STATE];

   double x[nRVec][3];
   double xbar[nBVec][3];

   long curRInd = 0, curBInd = 0;
   for (j = INIT_STATE; j <= FIN_STATE; j++) {
      if (j == POS_STATE || j == VEL_STATE) {
         for (i = 0; i < 3; i++)
            x[curRInd][i] = -Nav->delta[i + Nav->navInd[j]];
         curRInd++;
      }
      else if (j == OMEGA_STATE) {
         for (i = 0; i < 3; i++)
            xbar[curBInd][i] = -Nav->delta[i + Nav->navInd[j]];
         curBInd++;
      }
   }

   for (i = 0; i < 3; i++)
      theta[i] = -Nav->delta[i + Nav->navInd[ROTMAT_STATE]];

   expmTFG(theta, nRVec, nBVec, x, xbar, dR);
   curRInd = 0, curBInd = 0;
   for (j = INIT_STATE; j <= FIN_STATE; j++) {
      switch (j) {
         case POS_STATE:
            for (i = 0; i < 3; i++)
               dr[i] = x[curRInd][i];
            curRInd++;
            break;
         case VEL_STATE:
            for (i = 0; i < 3; i++)
               dv[i] = x[curRInd][i];
            curRInd++;
            break;
         case OMEGA_STATE:
            for (i = 0; i < 3; i++)
               dw[i] = xbar[curBInd][i];
            curBInd++;
            break;
         default:
            break;
      }
   }

   MxV(dR, Nav->PosR, tmpV);
   for (i = 0; i < 3; i++)
      Nav->PosR[i] = dr[i] + tmpV[i];
   MxV(dR, Nav->VelR, tmpV);
   for (i = 0; i < 3; i++)
      Nav->VelR[i] = dv[i] + tmpV[i];

   MTxV(Nav->CRB, dw, tmpV);
   for (i = 0; i < 3; i++)
      Nav->wbr[i] += tmpV[i];

   MxM(dR, Nav->CRB, tmpM);
   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         Nav->CRB[i][j] = tmpM[i][j];
   MT(Nav->CRB, tmpM);
   C2Q(tmpM, Nav->qbr);
}

/*--------------------------------------------------------------------*/
/*                          LIEKF functions                           */
/*--------------------------------------------------------------------*/

void eomLIEKFJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                         const struct DateType *date, const double CRB[3][3],
                         const double qbr[4], const double PosR[3],
                         const double VelR[3], const double wbr[3],
                         const double whlH[AC->Nwhl], const double AtmoDensity)
{
   double tmpM[3][3] = {{0.0}}, tmpM2[3][3] = {{0.0}}, tmpM3[3][3] = {{0.0}},
          tmpV[3] = {0.0}, tmpV2[3] = {0.0}, tmpV3[3] = {0.0};
   static double **tmpAssign = NULL;
   double wrnd[3]            = {0.0};
   struct DSMNavType *Nav    = &DSM->DsmNav;
   long i, j, rowInd;
   enum States state;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   switch (Nav->refFrame) {
      case FRAME_N:
         for (i = 0; i < 3; i++)
            wrnd[i] = 0.0;
         break;
      case FRAME_L:
         break;
      case FRAME_F:
         break;
   }

   for (i = 0; i < Nav->navDim; i++)
      for (j = 0; j < Nav->navDim; j++)
         Nav->jacobian[i][j] = 0.0;
   double aeroTrq[3] = {0.0}, aeroFrc[3] = {0.0};
   if (AeroActive) {
      const long orbCenter = DSM->refOrb->World;
      getAeroForceAndTorque(DSM, CRB, PosR, VelR, World[orbCenter].w,
                            AtmoDensity, aeroFrc, aeroTrq);
      for (i = 0; i < 3; i++)
         tmpV2[i] += aeroTrq[i];
   }

   if (Nav->stateActive[ROTMAT_STATE] && Nav->stateActive[POS_STATE] &&
       Nav->stateActive[VEL_STATE] && Nav->stateActive[OMEGA_STATE] &&
       !Nav->stateActive[QUAT_STATE]) {
      for (state = INIT_STATE; state <= FIN_STATE; state++) {
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = 0.0;
         if (Nav->stateActive[state] == TRUE) {
            rowInd = Nav->navInd[state];
            switch (state) {
               case TIME_STATE:
                  tmpAssign[0][0] = 1.0;
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 1, 1);
                  break;
               case ROTMAT_STATE:
                  for (i = 0; i < 3; i++) {
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = 0.0;
                     tmpAssign[i][i] = 1.0;
                  }
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd,
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
                  V2CrossM(wbr, tmpM);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = -tmpM[i][j];
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3, 3);

                  MTxV(CRB, Nav->refOmega, tmpV3);
                  for (i = 0; i < 3; i++)
                     tmpV3[i] += wbr[i];
                  MxV(DSM->MOI, tmpV3, tmpV);
                  for (long Iw = 0; Iw < AC->Nwhl; Iw++) {
                     for (i = 0; i < 3; i++)
                        tmpV[i] += AC->Whl[Iw].Axis[i] * whlH[Iw];
                  }
                  VxV(tmpV, tmpV3, tmpV2);
                  for (i = 0; i < 3; i++) {
                     tmpV2[i] += Nav->torqueB[i];
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = DSM->MOI[i][j];
                  }
                  // tmpV = wbn_dot (expressed in B, wrt N)
                  LINSOLVE(tmpAssign, tmpV, tmpV2, 3);
                  // find wbr_dot (expressed in B, wrt R)
                  if (Nav->refFrame != FRAME_N) {
                     MTxV(CRB, Nav->refOmega, tmpV2);
                     VxV(tmpV2, wbr, tmpV3);
                     MTxV(CRB, wrnd, tmpV2);
                     for (i = 0; i < 3; i++)
                        tmpV[i] -= (tmpV3[i] + tmpV2[i]);
                  }
                  V2CrossM(tmpV, tmpM);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = -tmpM[i][j];
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                            Nav->navInd[ROTMAT_STATE], 3, 3);

                  // calculate dwbn_dot/dwbn
                  MTxV(CRB, Nav->refOmega, tmpV3);
                  for (i = 0; i < 3; i++)
                     tmpV3[i] += wbr[i];
                  V2CrossM(tmpV3, tmpM2);
                  MxM(tmpM2, DSM->MOI, tmpM3);
                  MxV(DSM->MOI, tmpV3, tmpV2);
                  for (long Iw = 0; Iw < AC->Nwhl; Iw++) {
                     for (i = 0; i < 3; i++)
                        tmpV2[i] += AC->Whl[Iw].Axis[i] * whlH[Iw];
                  }
                  V2CrossM(tmpV2, tmpM);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpM[i][j] -= tmpM3[i][j];
                  MINVxM3(DSM->MOI, 3, tmpM, tmpM2);
                  // use tmpM2=dwbn_dot/dwbn to calc a few derivs
                  MTxV(CRB, Nav->refOmega, tmpV);
                  V2CrossM(tmpV, tmpM);
                  MxM(tmpM2, tmpM, tmpM3);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = tmpM3[i][j];
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                            Nav->navInd[ROTMAT_STATE], 3, 3);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpM3[i][j] = -tmpM[i][j] + tmpM2[i][j];
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = tmpM3[i][j];
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                            Nav->navInd[OMEGA_STATE], 3, 3);
                  V2CrossM(wbr, tmpM);
                  MxM(tmpM3, tmpM, tmpM2);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = tmpM2[i][j];
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                            Nav->navInd[ROTMAT_STATE], 3, 3);
                  break;
               case POS_STATE:
                  V2CrossM(wbr, tmpM);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = -tmpM[i][j];
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3, 3);
                  for (i = 0; i < 3; i++) {
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = 0.0;
                     tmpAssign[i][i] = 1.0;
                  }
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                            Nav->navInd[VEL_STATE], 3, 3);
                  break;
               case VEL_STATE:
                  V2CrossM(wbr, tmpM);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = -tmpM[i][j];
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3, 3);
                  for (i = 0; i < 3; i++) {
                     tmpV[i] = Nav->forceB[i] / DSM->mass;
                  }
                  V2CrossM(tmpV, tmpM);
                  MxM(CRB, tmpM, tmpM2);
                  V2CrossM(aeroFrc, tmpM3);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] =
                            tmpM2[i][j] + tmpM3[i][j] / (DSM->mass * DSM->mass);
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                            Nav->navInd[ROTMAT_STATE], 3, 3);

                  if (Nav->refFrame != FRAME_N) {
                     MTxV(CRB, Nav->refOmega, tmpV);
                     SxV(2.0, tmpV, tmpV2);
                     V2CrossM(tmpV2, tmpM);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = -tmpM[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3, 3);

                     V2DoubleCrossM(tmpV, tmpM);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = -tmpM[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);

                     MTxV(CRB, wrnd, tmpV);
                     V2CrossM(tmpV, tmpM);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = -tmpM[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);
                  }
                  break;
               default:
                  break;
            }
         }
      }
      if (DSM->refOrb->Regime == ORB_CENTRAL) {
         long orbCenter             = DSM->refOrb->World;
         double dAeroFrcdVRel[3][3] = {{0.0}}, dAeroTrqdVRel[3][3] = {{0.0}};
         double worldWR[3] = {0.0};
         if (AeroActive) {
            double worldW = World[orbCenter].w;
            for (i = 0; i < 3; i++)
               worldWR[i] = -Nav->refCRN[i][2] * worldW;
            getDAeroFrcAndTrqDVRel(DSM, CRB, PosR, VelR, worldW, AtmoDensity,
                                   dAeroFrcdVRel, dAeroTrqdVRel);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  dAeroFrcdVRel[i][j] /= DSM->mass;
         }
         for (state = INIT_STATE; state <= FIN_STATE; state++) {
            if (Nav->stateActive[state] == TRUE) {
               rowInd = Nav->navInd[state];
               switch (state) {
                  case VEL_STATE: {
                     for (i = 0; i < 3; i++)
                        tmpV2[i] = PosR[i] + Nav->refPos[i];
                     getDGravFrcDPos(World[orbCenter].mu, tmpV2, tmpM2);
                     if (GravPertActive) {
                        for (i = 0; i < 3; i++)
                           tmpV2[i] = PosR[i] + Nav->refPos[i];
                        NavDGravPertAccelDPos(Nav, date, tmpV2, DSM->refOrb,
                                              tmpM);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpM2[i][j] += tmpM[i][j];
                     }
                     AdjointT(CRB, tmpM2, tmpM);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = tmpM[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);
                     if (AeroActive) {
                        AdjointT(CRB, dAeroFrcdVRel, tmpM3);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = tmpM3[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3,
                                  3);
                        V2CrossM(worldWR, tmpM2);
                        // TODO: double check these two lines
                        MxM(tmpM3, tmpM2, tmpM);
                        MxM(tmpM, CRB, tmpM3);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = tmpM[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                                  Nav->navInd[POS_STATE], 3, 3);
                     }
                  } break;
                  case OMEGA_STATE:
                     if (AeroActive) {
                        MxM(dAeroTrqdVRel, CRB, tmpM3);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = tmpM3[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                                  Nav->navInd[VEL_STATE], 3, 3);
                        V2CrossM(worldWR, tmpM);
                        MxM(dAeroTrqdVRel, tmpM, tmpM3);
                        MxM(tmpM3, CRB, tmpM);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = tmpM[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
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
   double theta[3] = {0.0}, tmpM[3][3] = {{0.0}}, tmpV[3] = {0.0};
   double dR[3][3] = {{0.0}}, dr[3] = {0.0}, dv[3] = {0.0}, dw[3] = {0.0};
   long i, j;

   long nRVec = Nav->stateActive[POS_STATE] + Nav->stateActive[VEL_STATE];
   long nBVec = Nav->stateActive[OMEGA_STATE];

   double x[nRVec][3];
   double xbar[nBVec][3];

   long curRInd = 0, curBInd = 0;
   for (j = INIT_STATE; j <= FIN_STATE; j++) {
      if (j == POS_STATE || j == VEL_STATE) {
         for (i = 0; i < 3; i++)
            x[curRInd][i] = -Nav->delta[i + Nav->navInd[j]];
         curRInd++;
      }
      else if (j == OMEGA_STATE) {
         for (i = 0; i < 3; i++)
            xbar[curBInd][i] = -Nav->delta[i + Nav->navInd[j]];
         curBInd++;
      }
   }

   for (i = 0; i < 3; i++)
      theta[i] = -Nav->delta[i + Nav->navInd[ROTMAT_STATE]];

   expmTFG(theta, nRVec, nBVec, x, xbar, dR);
   curRInd = 0, curBInd = 0;
   for (j = INIT_STATE; j <= FIN_STATE; j++) {
      switch (j) {
         case POS_STATE:
            for (i = 0; i < 3; i++)
               dr[i] = x[curRInd][i];
            curRInd++;
            break;
         case VEL_STATE:
            for (i = 0; i < 3; i++)
               dv[i] = x[curRInd][i];
            curRInd++;
            break;
         case OMEGA_STATE:
            for (i = 0; i < 3; i++)
               dw[i] = xbar[curBInd][i];
            curBInd++;
            break;
         default:
            break;
      }
   }

   MxV(Nav->CRB, dr, tmpV);
   for (i = 0; i < 3; i++)
      Nav->PosR[i] += tmpV[i];
   MxV(Nav->CRB, dv, tmpV);
   for (i = 0; i < 3; i++)
      Nav->VelR[i] += tmpV[i];

   MxM(Nav->CRB, dR, tmpM);
   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         Nav->CRB[i][j] = tmpM[i][j];

   MTxV(dR, Nav->wbr, tmpV);
   for (i = 0; i < 3; i++)
      Nav->wbr[i] = tmpV[i] + dw[i];
   MT(Nav->CRB, tmpM);
   C2Q(tmpM, Nav->qbr);
}

/*--------------------------------------------------------------------*/
/*                          MEKF functions                           */
/*--------------------------------------------------------------------*/

void eomMEKFJacobianFun(struct AcType *const AC, struct DSMType *const DSM,
                        const struct DateType *date, const double CRB[3][3],
                        const double qbr[4], const double PosR[3],
                        const double VelR[3], const double wbr[3],
                        const double whlH[AC->Nwhl], const double AtmoDensity)
{
   double tmpM[3][3] = {{0.0}}, tmpM2[3][3] = {{0.0}}, tmpM3[3][3] = {{0.0}},
          tmpV[3] = {0.0}, tmpV2[3] = {0.0}, tmpV3[3] = {0.0};
   static double **tmpAssign = NULL;
   double wrnd[3]            = {0.0};
   struct DSMNavType *Nav    = &DSM->DsmNav;
   long i, j, rowInd;
   enum States state;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   switch (Nav->refFrame) {
      case FRAME_N:
         for (i = 0; i < 3; i++)
            wrnd[i] = 0.0;
         break;
      case FRAME_L:
         break;
      case FRAME_F:
         break;
   }

   for (i = 0; i < Nav->navDim; i++)
      for (j = 0; j < Nav->navDim; j++)
         Nav->jacobian[i][j] = 0.0;

   if (Nav->stateActive[QUAT_STATE] && Nav->stateActive[POS_STATE] &&
       Nav->stateActive[VEL_STATE] && Nav->stateActive[OMEGA_STATE]) {
      for (state = INIT_STATE; state <= FIN_STATE; state++) {
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = 0.0;
         if (Nav->stateActive[state] == TRUE) {
            rowInd = Nav->navInd[state];
            switch (state) {
               case TIME_STATE:
                  tmpAssign[0][0] = 1.0;
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 1, 1);
                  break;
               case ROTMAT_STATE:
                  break;
               case QUAT_STATE:
                  V2CrossM(wbr, tmpM);
                  for (i = 0; i < 3; i++) {
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = -tmpM[i][j];
                  }
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3, 3);

                  for (i = 0; i < 3; i++) {
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = 0.0;
                     tmpAssign[i][i] = 1.0;
                  }
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                            Nav->navInd[OMEGA_STATE], 3, 3);
                  break;
               case OMEGA_STATE:
                  // calculate dwbn_dot/dwbn
                  QxV(qbr, Nav->refOmega, tmpV3);
                  for (i = 0; i < 3; i++)
                     tmpV3[i] += wbr[i];
                  V2CrossM(tmpV3, tmpM2);
                  MxM(tmpM2, DSM->MOI, tmpM3);
                  MxV(DSM->MOI, tmpV3, tmpV2);
                  for (long Iw = 0; Iw < AC->Nwhl; Iw++) {
                     for (i = 0; i < 3; i++)
                        tmpV2[i] += AC->Whl[Iw].Axis[i] * whlH[Iw];
                  }
                  V2CrossM(tmpV2, tmpM);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpM[i][j] -= tmpM3[i][j];
                  MINVxM3(DSM->MOI, 3, tmpM, tmpM2);
                  // MINV3(DSM->MOI, tmpM3);
                  // MxM(tmpM3, tmpM, tmpM2);
                  for (i = 0; i < 3; i++) {
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = tmpM2[i][j];
                  }
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3, 3);
                  break;
               case POS_STATE:
                  for (i = 0; i < 3; i++) {
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = 0.0;
                     tmpAssign[i][i] = 1.0;
                  }
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                            Nav->navInd[VEL_STATE], 3, 3);
                  break;
               case VEL_STATE:
                  for (i = 0; i < 3; i++) {
                     tmpV[i] = Nav->forceB[i] / DSM->mass;
                  }
                  V2CrossM(tmpV, tmpM);
                  MxM(CRB, tmpM, tmpM2);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = tmpM2[i][j];
                  subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                            Nav->navInd[QUAT_STATE], 3, 3);

                  if (Nav->refFrame != FRAME_N) {
                     MTxV(CRB, Nav->refOmega, tmpV);
                     SxV(2.0, tmpV, tmpV2);
                     V2CrossM(tmpV2, tmpM);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = -tmpM[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3, 3);

                     V2DoubleCrossM(tmpV, tmpM);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = -tmpM[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);

                     MTxV(CRB, wrnd, tmpV);
                     V2CrossM(tmpV, tmpM);
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = -tmpM[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);
                  }
                  break;
               default:
                  break;
            }
         }
      }
      if (DSM->refOrb->Regime == ORB_CENTRAL) {
         long orbCenter             = DSM->refOrb->World;
         double dAeroFrcdVRel[3][3] = {{0.0}}, dAeroTrqdVRel[3][3] = {{0.0}};
         double worldWR[3] = {0.0};
         if (AeroActive) {
            double worldW = World[orbCenter].w;
            for (i = 0; i < 3; i++)
               worldWR[i] = -Nav->refCRN[i][2] * worldW;
            getDAeroFrcAndTrqDVRel(DSM, CRB, PosR, VelR, worldW, AtmoDensity,
                                   dAeroFrcdVRel, dAeroTrqdVRel);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  dAeroFrcdVRel[i][j] /= DSM->mass;
         }
         for (state = INIT_STATE; state <= FIN_STATE; state++) {
            if (Nav->stateActive[state] == TRUE) {
               rowInd = Nav->navInd[state];
               switch (state) {
                  case VEL_STATE: {
                     for (i = 0; i < 3; i++)
                        tmpV2[i] = PosR[i] + Nav->refPos[i];
                     getDGravFrcDPos(World[orbCenter].mu, tmpV2, tmpM2);
                     if (GravPertActive) {
                        for (i = 0; i < 3; i++)
                           tmpV2[i] = PosR[i] + Nav->refPos[i];
                        NavDGravPertAccelDPos(Nav, date, tmpV2, DSM->refOrb,
                                              tmpM);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpM2[i][j] += tmpM[i][j];
                     }
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           tmpAssign[i][j] = tmpM2[i][j];
                     subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                               Nav->navInd[POS_STATE], 3, 3);
                     if (AeroActive) {
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = dAeroFrcdVRel[i][j];

                        subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3,
                                  3);
                        V2CrossM(worldWR, tmpM2);
                        MxM(dAeroFrcdVRel, tmpM2, tmpM3);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = tmpM3[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                                  Nav->navInd[POS_STATE], 3, 3);
                     }
                  } break;
                  case OMEGA_STATE:
                     if (AeroActive) {
                        MxM(dAeroTrqdVRel, CRB, tmpM3);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = tmpM3[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
                                  Nav->navInd[VEL_STATE], 3, 3);
                        V2CrossM(worldWR, tmpM);
                        MxM(dAeroTrqdVRel, tmpM, tmpM3);
                        MxM(tmpM3, CRB, tmpM);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = tmpM[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd,
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
   double q[4] = {0.0}, dq[4] = {0.0}, tmpM[3][3] = {{0.0}};
   long i;

   for (i = 0; i < 3; i++) {
      dq[i]         = -Nav->delta[i + Nav->navInd[QUAT_STATE]] / 2.0;
      Nav->PosR[i] += -Nav->delta[i + Nav->navInd[POS_STATE]];
      Nav->VelR[i] += -Nav->delta[i + Nav->navInd[VEL_STATE]];
      Nav->wbr[i]  += -Nav->delta[i + Nav->navInd[OMEGA_STATE]];
   }
   dq[3] = 1.0;
   QxQ(dq, Nav->qbr, q);
   UNITQ(q);
   for (i = 0; i < 4; i++)
      Nav->qbr[i] = q[i];
   Q2C(Nav->qbr, tmpM);
   MT(tmpM, Nav->CRB);
}

/******************************************************************************/
//                            Navigation Functions
/******************************************************************************/
double **GetStateLinTForm(struct DSMNavType *const Nav)
{
   double **tForm, tmpM[3][3] = {{0.0}};
   static double **tmpAssign = NULL;
   long i, j;

   tForm = CreateMatrix(Nav->navDim, Nav->navDim);
   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         tmpAssign[i][j] = 0.0;

   switch (Nav->type) {
      case LIEKF_NAV:
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = -Nav->CRB[i][j];
         subMatAdd(tForm, tmpAssign, Nav->navInd[ROTMAT_STATE],
                   Nav->navInd[ROTMAT_STATE], 3, 3);
         subMatAdd(tForm, tmpAssign, Nav->navInd[POS_STATE],
                   Nav->navInd[POS_STATE], 3, 3);
         subMatAdd(tForm, tmpAssign, Nav->navInd[VEL_STATE],
                   Nav->navInd[VEL_STATE], 3, 3);
         for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = 0.0;
            tmpAssign[i][i] = -1.0;
         }
         subMatAdd(tForm, tmpAssign, Nav->navInd[OMEGA_STATE],
                   Nav->navInd[OMEGA_STATE], 3, 3);
         V2CrossM(Nav->wbr, tmpM);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = -tmpM[i][j];
         subMatAdd(tForm, tmpAssign, Nav->navInd[OMEGA_STATE],
                   Nav->navInd[ROTMAT_STATE], 3, 3);
         break;
      case RIEKF_NAV:
         for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = 0.0;
            tmpAssign[i][i] = -1.0;
         }
         subMatAdd(tForm, tmpAssign, Nav->navInd[ROTMAT_STATE],
                   Nav->navInd[ROTMAT_STATE], 3, 3);
         subMatAdd(tForm, tmpAssign, Nav->navInd[POS_STATE],
                   Nav->navInd[POS_STATE], 3, 3);
         subMatAdd(tForm, tmpAssign, Nav->navInd[VEL_STATE],
                   Nav->navInd[VEL_STATE], 3, 3);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = -Nav->CRB[j][i];
         subMatAdd(tForm, tmpAssign, Nav->navInd[OMEGA_STATE],
                   Nav->navInd[OMEGA_STATE], 3, 3);
         V2CrossM(Nav->PosR, tmpM);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = tmpM[i][j];
         subMatAdd(tForm, tmpAssign, Nav->navInd[POS_STATE],
                   Nav->navInd[ROTMAT_STATE], 3, 3);
         V2CrossM(Nav->VelR, tmpM);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = tmpM[i][j];
         subMatAdd(tForm, tmpAssign, Nav->navInd[VEL_STATE],
                   Nav->navInd[ROTMAT_STATE], 3, 3);
         break;
      case MEKF_NAV:
         for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = 0.0;
            tmpAssign[i][i] = -1.0;
         }
         subMatAdd(tForm, tmpAssign, Nav->navInd[POS_STATE],
                   Nav->navInd[POS_STATE], 3, 3);
         subMatAdd(tForm, tmpAssign, Nav->navInd[VEL_STATE],
                   Nav->navInd[VEL_STATE], 3, 3);
         subMatAdd(tForm, tmpAssign, Nav->navInd[OMEGA_STATE],
                   Nav->navInd[OMEGA_STATE], 3, 3);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               tmpAssign[i][j] = Nav->CRB[i][j];
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

void configureRefFrame(struct DSMNavType *const Nav,
                       const struct OrbitType *refOrb, const double dLerpAlpha,
                       const long reset)
{
   // set up reference frame. Not a fan of effectively using truth data for it
   long i, j;
   double targetPosN[3] = {0.0};
   double targetVelN[3] = {0.0};

   if (reset == TRUE)
      Nav->refLerpAlpha = 1.0;
   else
      Nav->refLerpAlpha += dLerpAlpha;

   if (Nav->Init == FALSE) {
      for (i = 0; i < 3; i++)
         Nav->refAccel[i] = 0.0;
   }

   double prevRefVel[3] = {0.0};
   for (i = 0; i < 3; i++)
      prevRefVel[i] = Nav->refVel[i];

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
         for (i = 0; i < 3; i++) {
            targetPosN[i] = O->PosN[i];
            targetVelN[i] = O->VelN[i];
         }
      } break;
      default: {
         // make sure if you do sc relative nav, you initialize that sc's nav
         // before you start this sc's
         // TODO: due to comm state, this is a time step behind...
         const struct DSMStateType *TrgState = Nav->refOriPtr;
         const struct BodyType *TrgSB        = Nav->refBodyPtr;
         for (i = 0; i < 3; i++) {
            // pn is position of body origin relative to sc origin
            targetPosN[i] = TrgState->PosN[i] + TrgSB->pn[i];
            targetVelN[i] = TrgState->VelN[i] + TrgSB->vn[i];
         }
      } break;
   }

   for (i = 0; i < 3; i++) {
      Nav->refPos[i] = (1.0 - Nav->refLerpAlpha) * Nav->oldRefPos[i] +
                       Nav->refLerpAlpha * targetPosN[i];
      Nav->refVel[i] = (1.0 - Nav->refLerpAlpha) * Nav->oldRefVel[i] +
                       Nav->refLerpAlpha * targetVelN[i];
   }

   switch (Nav->refFrame) {
      case FRAME_N:
         for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++)
               Nav->refCRN[i][j] = 0.0;
            Nav->refCRN[i][i]   = 1.0;
            Nav->refOmega[i]    = 0.0;
            Nav->refOmegaDot[i] = 0.0;
         }
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
      double refPos[3] = {0.0}, refVel[3] = {0.0}, wxr[3] = {0.0};
      MxV(Nav->refCRN, Nav->refPos, refPos);
      for (i = 0; i < 3; i++)
         Nav->refPos[i] = refPos[i];
      MxV(Nav->refCRN, Nav->refVel, refVel);
      VxV(Nav->refOmega, Nav->refPos, wxr);
      for (i = 0; i < 3; i++)
         Nav->refVel[i] = refVel[i] - wxr[i];
   }

   if (Nav->Init == TRUE && fabs(1.0 - Nav->refLerpAlpha) > __DBL_EPSILON__) {
      const double dt = (1.0 - Nav->refLerpAlpha) * Nav->DT;
      for (i = 0; i < 3; i++)
         Nav->refAccel[i] = (targetVelN[i] - Nav->refVel[i]) / dt;
   }
   if (reset == TRUE) {
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++)
            Nav->oldRefCRN[i][j] = Nav->refCRN[i][j];
         Nav->oldRefPos[i]      = Nav->refPos[i];
         Nav->oldRefVel[i]      = Nav->refVel[i];
         Nav->oldRefOmega[i]    = Nav->refOmega[i];
         Nav->oldRefOmegaDot[i] = Nav->refOmegaDot[i];
      }
      Nav->refLerpAlpha = 0.0;
   }
}

void GetM(struct AcType *const AC, struct DSMNavType *const Nav,
          const double CRB[3][3], const double qbr[4], const double PosR[3],
          const double VelR[3], const double wbr[3])
{
   double tmp3x3[3][3] = {{0.0}}, MOIInv[3][3] = {{0.0}};
   long i, j;

   for (i = 0; i < Nav->navDim; i++)
      for (j = 0; j < Nav->navDim; j++)
         Nav->M[i][j] = 0.0;

   switch (Nav->type) {
      case LIEKF_NAV: {
         for (i = 0; i < Nav->navDim; i++)
            Nav->M[i][i] = -1.0;
         for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
               Nav->M[Nav->navInd[POS_STATE] + i][Nav->navInd[POS_STATE] + j] =
                   -CRB[j][i];
               Nav->M[Nav->navInd[VEL_STATE] + i][Nav->navInd[VEL_STATE] + j] =
                   -CRB[j][i];
            }
         }
         MINV3(AC->MOI, MOIInv);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               Nav->M[Nav->navInd[OMEGA_STATE] + i]
                     [Nav->navInd[OMEGA_STATE] + j] = -MOIInv[i][j];

         V2CrossM(wbr, tmp3x3);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               Nav->M[Nav->navInd[OMEGA_STATE] + i]
                     [Nav->navInd[ROTMAT_STATE] + j] = tmp3x3[i][j];
      } break;
      case RIEKF_NAV: {
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               Nav->M[Nav->navInd[ROTMAT_STATE] + i]
                     [Nav->navInd[ROTMAT_STATE] + j] = -CRB[i][j];
         for (i = 0; i < 3; i++) {
            Nav->M[Nav->navInd[POS_STATE] + i][Nav->navInd[POS_STATE] + i] =
                -1.0;
            Nav->M[Nav->navInd[VEL_STATE] + i][Nav->navInd[VEL_STATE] + i] =
                -1.0;
         }
         V2CrossM(PosR, MOIInv);
         MxM(MOIInv, CRB, tmp3x3);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               Nav->M[Nav->navInd[POS_STATE] + i]
                     [Nav->navInd[ROTMAT_STATE] + j] = -tmp3x3[i][j];
         V2CrossM(VelR, MOIInv);
         MxM(MOIInv, CRB, tmp3x3);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               Nav->M[Nav->navInd[VEL_STATE] + i]
                     [Nav->navInd[ROTMAT_STATE] + j] = -tmp3x3[i][j];
         MINV3(AC->MOI, MOIInv);
         MxM(CRB, MOIInv, tmp3x3);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               Nav->M[Nav->navInd[OMEGA_STATE] + i]
                     [Nav->navInd[OMEGA_STATE] + j] = -tmp3x3[i][j];
      } break;
      case MEKF_NAV: {
         for (i = 0; i < Nav->navDim; i++)
            Nav->M[i][i] = -1.0;
         MINV3(AC->MOI, MOIInv);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               Nav->M[Nav->navInd[OMEGA_STATE] + i]
                     [Nav->navInd[OMEGA_STATE] + j] = -MOIInv[i][j];
      } break;
      default:
         fprintf(stderr, "Navigation active with undefined or ideal navigation "
                         "type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }
}

void getForceAndTorque(struct AcType *const AC, struct DSMNavType *const Nav,
                       const double CRB[3][3], const double *whlH)
{
   long i, j;

   for (i = 0; i < 3; i++) {
      Nav->forceB[i]  = AC->IdealFrc[i];
      Nav->torqueB[i] = AC->IdealTrq[i];
   }
   for (j = 0; j < AC->Nthr; j++) {
      const struct AcThrType *thr = &AC->Thr[j];
      if (thr->ThrustLevelCmd > 0.0) {
         const double appliedForce = thr->ThrustLevelCmd * thr->Fmax;
         for (i = 0; i < 3; i++) {
            Nav->forceB[i]  += thr->Axis[i] * appliedForce;
            Nav->torqueB[i] += thr->rxA[i] * appliedForce;
         }
      }
   }

   for (j = 0; j < AC->Nwhl; j++) {
      const struct AcWhlType *whl = &AC->Whl[j];
      if ((whlH[j] * signum(whl->Tcmd)) < whl->Hmax)
         for (i = 0; i < 3; i++)
            Nav->torqueB[i] -= whl->Axis[i] * whl->Tcmd;
   }
   double bvb[3] = {0.0}, CBN[3][3] = {{0.0}};
   MTxM(CRB, Nav->refCRN, CBN);
   MxV(CBN, AC->bvn, bvb);
   for (j = 0; j < AC->Nmtb; j++) {
      const struct AcMtbType *mtb = &AC->MTB[j];
      double AxBvb[3]             = {0.0};
      VxV(mtb->Axis, bvb, AxBvb);
      for (i = 0; i < 3; i++)
         Nav->torqueB[i] += AxBvb[i] * mtb->Mcmd;
   }
}

void NavEOMs(struct AcType *const AC, struct DSMType *const DSM,
             const struct DateType *date, const double CRB[3][3],
             const double qbr[4], const double PosR[3], const double VelR[3],
             const double wbr[3], const double *whlH, double CRBdot[3][3],
             double qbrdot[4], double PosRdot[3], double VelRdot[3],
             double wbrdot[3], double *whlHdot, const double AtmoDensity)
{
   long i, j, iState;

   struct DSMNavType *Nav = &DSM->DsmNav;

   double aeroFrc[3] = {0.0}, aeroTrq[3] = {0.0};
   const long orbCenter          = DSM->refOrb->World;
   enum orbitRegime const regime = DSM->refOrb->Regime;
   if (AeroActive && regime == ORB_CENTRAL) {
      getAeroForceAndTorque(DSM, CRB, PosR, VelR, World[orbCenter].w,
                            AtmoDensity, aeroFrc, aeroTrq);
   }
   getForceAndTorque(AC, Nav, CRB, whlH);

   for (iState = INIT_STATE; iState <= FIN_STATE; iState++) {
      if (Nav->stateActive[iState] == TRUE) {
         double **pM3x3, wbrX[3][3] = {{0.0}}, tmpV[3] = {0.0},
                         tmpV2[3] = {0.0}, wbn[3] = {0.0};
         switch (iState) {
            case TIME_STATE:
               break;
            case ROTMAT_STATE:
               V2CrossM(wbr, wbrX);
               MxM(CRB, wbrX, CRBdot);
               break;
            case QUAT_STATE:
               QW2QDOT(qbr, wbr, qbrdot);
               break;
            case OMEGA_STATE: {
               double Hb[3] = {0.0};
               for (i = 0; i < 3; i++)
                  wbn[i] = wbr[i];
               if (Nav->refFrame != FRAME_N) {
                  MTxV(CRB, Nav->refOmega, tmpV);
                  for (i = 0; i < 3; i++)
                     wbn[i] += tmpV[i];
               }
               MxV(DSM->MOI, wbn, Hb);

               for (long Iw = 0; Iw < AC->Nwhl; Iw++)
                  for (i = 0; i < 3; i++)
                     Hb[i] += AC->Whl[Iw].Axis[i] * whlH[Iw];
               VxV(Hb, wbn, tmpV);

               for (i = 0; i < 3; i++) {
                  tmpV[i] += Nav->torqueB[i];
                  if (AeroActive && regime == ORB_CENTRAL)
                     tmpV[i] += aeroTrq[i];
               }

               pM3x3 = CreateMatrix(3, 3);
               for (i = 0; i < 3; i++)
                  for (j = 0; j < 3; j++)
                     pM3x3[i][j] = DSM->MOI[i][j];
               LINSOLVE(pM3x3, wbrdot, tmpV, 3);
               DestroyMatrix(pM3x3);
               if (Nav->refFrame != FRAME_N) {
                  MTxV(CRB, Nav->refOmegaDot, tmpV);
                  for (i = 0; i < 3; i++)
                     wbrdot[i] -= tmpV[i];
               }
            } break;
            case POS_STATE:
               for (i = 0; i < 3; i++)
                  PosRdot[i] = VelR[i];
               break;
            case VEL_STATE:
               MxV(CRB, Nav->forceB, VelRdot);
               for (i = 0; i < 3; i++)
                  VelRdot[i] /= DSM->mass;
               if (Nav->refFrame != FRAME_N) {
                  fprintf(stderr, "Frame types other than Inertial are still "
                                  "in development for filtering. Exiting...\n");
                  exit(EXIT_FAILURE);
               }

               if (GravPertActive) {
                  double accelR[3] = {0.0};
                  for (i = 0; i < 3; i++)
                     tmpV[i] = PosR[i] + Nav->refPos[i];
                  NavGravPertAccel(Nav, date, tmpV, 1.0, DSM->refOrb, accelR);
                  for (i = 0; i < 3; i++)
                     VelRdot[i] += accelR[i];
               }

               switch (regime) {
                  case ORB_CENTRAL: {
                     for (i = 0; i < 3; i++)
                        tmpV[i] = PosR[i] + Nav->refPos[i];
                     getGravAccel(DSM->refOrb->mu, tmpV, tmpV2);
                     for (i = 0; i < 3; i++)
                        VelRdot[i] += tmpV2[i];
                     if (Nav->refOriType != ORI_WORLD) {
                        for (i = 0; i < 3; i++)
                           VelRdot[i] -= Nav->refAccel[i];
                     }

                     if (AeroActive)
                        for (i = 0; i < 3; i++)
                           VelRdot[i] += aeroFrc[i] / DSM->mass;
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
         }
      }
   }
   for (long Iw = 0; Iw < AC->Nwhl; Iw++)
      whlHdot[Iw] = AC->Whl[Iw].Tcmd;
}

void updateNavCCSDS(ccsdsCoarse *sec, ccsdsFine *subsec, const double dSeconds)
{
   const double sizeCheck = 1.0 / (((unsigned long)(CCSDS_FINE_MAX)) << 1);
   double integral;
   const double fractional = modf(dSeconds, &integral);

   *sec += integral;
   if (fractional > sizeCheck) {
      const ccsdsFine dSubSec = fractional * CCSDS_FINE_MAX + .5;
      ccsdsFine test          = 0;
      if (__builtin_add_overflow(*subsec, dSubSec, &test))
         (*sec)++;
      *subsec += dSubSec;
   }
   else if (fractional < -sizeCheck) {
      const ccsdsFine dSubSec = -fractional * CCSDS_FINE_MAX + .5;
      ccsdsFine test          = 0;
      if (__builtin_sub_overflow(*subsec, dSubSec, &test))
         (*sec)--;
      *subsec -= dSubSec;
   }
}

void PropagateNav(struct AcType *const AC, struct DSMType *const DSM,
                  const long dCCSDSSec, const long dCCSDSSubSec,
                  const long init)
{
   double AtmoDensity = 0.0;

   long i, j, k;
   enum States Istate;

   struct DSMNavType *Nav     = &DSM->DsmNav;
   double lerpAlphaState      = Nav->refLerpAlpha;
   const double ccsdsStepSize = CCSDS_STEP_SIZE;
   const double DT =
       (double)(dCCSDSSec) + (double)(dCCSDSSubSec * ccsdsStepSize);

   ccsdsCoarse dateSec;
   ccsdsFine dateSubsec;
   DateToCCSDS(Nav->Date, &dateSec, &dateSubsec);
   updateNavCCSDS(&dateSec, &dateSubsec, -(32.184 + LeapSec));
   const double dateOffset =
       CCSDSSub(Nav->ccsdsSeconds, Nav->ccsdsSubseconds, dateSec, dateSubsec);

   GetM(AC, Nav, Nav->CRB, Nav->qbr, Nav->PosR, Nav->VelR, Nav->wbr);
   if (init == TRUE) {
      if (AeroActive) {
         const long orbCenter = DSM->refOrb->World;
         double worldWR[3]    = {0.0};
         double VrelR[3] = {0.0}, PosN[3] = {0.0}, PosRWorld[3] = {0.0};
         for (i = 0; i < 3; i++)
            PosRWorld[i] = Nav->PosR[i] + Nav->refPos[i];
         for (i = 0; i < 3; i++)
            worldWR[i] = -Nav->refCRN[i][2] * World[orbCenter].w;
         VxV(worldWR, PosRWorld, VrelR);
         for (i = 0; i < 3; i++)
            VrelR[i] += Nav->VelR[i] + Nav->refVel[i];
         MTxV(Nav->refCRN, PosRWorld, PosN);
         if (orbCenter == EARTH) {
            const double ZAxis[3] = {0.0, 0.0, 1.0};
            double NavFlux10p7, NavGeomagIndex;
            double Alt, PosW[3] = {0.0}, CWN[3][3] = {{0.0}};
            const double PriMerAng = GetPriMerAng(orbCenter, &Nav->Date);
            SimpRot(ZAxis, PriMerAng, CWN);
            MxV(CWN, PosN, PosW);
            Alt = MAGV(PosW) - World[orbCenter].rad;
            if (Alt < 1000.0E3) { /* What is max alt of MSISE00 validity? */
               getEarthAtmoParams(Nav->Date.JulDay, &NavFlux10p7,
                                  &NavGeomagIndex);
               AtmoDensity =
                   NRLMSISE00(Nav->Date.Year, Nav->Date.doy, Nav->Date.Hour,
                              Nav->Date.Minute, Nav->Date.Second, PosW,
                              NavFlux10p7, NavGeomagIndex);
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
      (*Nav->EOMJacobianFun)(AC, DSM, &Nav->Date, Nav->CRB, Nav->qbr, Nav->PosR,
                             Nav->VelR, Nav->wbr, Nav->whlH, AtmoDensity);
      // TODO: find a better way to simplify the STM calculation. This is bound
      // to be quite slow due to the number of multiplications.
      for (i = 0; i < Nav->navDim; i++)
         for (j = 0; j < Nav->navDim; j++)
            Nav->NxN2[i][j] = Nav->jacobian[i][j] * ccsdsStepSize;
      expm(Nav->NxN2, Nav->STMStep, Nav->navDim);

      for (i = 0; i < Nav->navDim; i++)
         for (j = 0; j < Nav->navDim; j++)
            Nav->NxN2[i][j] = Nav->jacobian[i][j] * Nav->subStepSize;
      expm(Nav->NxN2, Nav->STM, Nav->navDim);
   }
   double dLerpAlpha = 0.0;
   // RK4
   double CRBk[ORDRK][3][3], qbrk[ORDRK][4], PosRk[ORDRK][3], VelRk[ORDRK][3],
       wbrk[ORDRK][3];
   double whlHk[ORDRK][AC->Nwhl];
#if ORDRK == 4
   const double DTk[ORDRK]     = {0.0, DT * 0.5, DT * 0.5, DT};
   const double rkScale[ORDRK] = {DT / 6.0, DT / 3.0, DT / 3.0, DT / 6.0};
#elif ORDRK == 1
   const double DTk[ORDRK]     = {0.0};
   const double rkScale[ORDRK] = {DT};
#endif
   for (k = 0; k < ORDRK; k++) {
      struct DateType date = Nav->Date;
      updateTime(&date, dateOffset + DTk[k]);
      Nav->refLerpAlpha = lerpAlphaState;
      dLerpAlpha        = DTk[k] / Nav->DT;
      double CRB[3][3], qbr[4], PosR[3], VelR[3], wbr[3], whlH[AC->Nwhl];
      if (k == 0) {
         for (i = 0; i < 3; i++) {
            qbr[i]  = Nav->qbr[i];
            PosR[i] = Nav->PosR[i];
            VelR[i] = Nav->VelR[i];
            wbr[i]  = Nav->wbr[i];
            for (j = 0; j < 3; j++)
               CRB[i][j] = Nav->CRB[i][j];
         }
         qbr[3] = Nav->qbr[3];

         for (i = 0; i < AC->Nwhl; i++)
            whlH[i] = Nav->whlH[i];
      }
      else {
         for (Istate = INIT_STATE; Istate <= FIN_STATE; Istate++) {
            if (Nav->stateActive[Istate] == TRUE) {
               double CBR[3][3] = {{0.0}};
               switch (Istate) {
                  case TIME_STATE:
                     // incremented by DT after this function is called
                     break;
                  case ROTMAT_STATE:
                     for (i = 0; i < 3; i++)
                        for (j = 0; j < 3; j++)
                           CRB[i][j] =
                               Nav->CRB[i][j] + CRBk[k - 1][i][j] * DTk[k];
                     MT(CRB, CBR);
                     C2Q(CBR, qbr);
                     UNITQ(qbr);
                     Q2C(qbr, CBR);
                     MT(CBR, CRB);
                     break;
                  case QUAT_STATE:
                     for (i = 0; i < 4; i++)
                        qbr[i] = Nav->qbr[i] + qbrk[k - 1][i] * DTk[k];
                     UNITQ(qbr);
                     Q2C(qbr, CBR);
                     MT(CBR, CRB);
                     break;
                  case OMEGA_STATE:
                     for (i = 0; i < 3; i++)
                        wbr[i] = Nav->wbr[i] + wbrk[k - 1][i] * DTk[k];
                     break;
                  case POS_STATE:
                     for (i = 0; i < 3; i++)
                        PosR[i] = Nav->PosR[i] + PosRk[k - 1][i] * DTk[k];
                     break;
                  case VEL_STATE:
                     for (i = 0; i < 3; i++)
                        VelR[i] = Nav->VelR[i] + VelRk[k - 1][i] * DTk[k];
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
      configureRefFrame(Nav, DSM->refOrb, dLerpAlpha, FALSE);
      getForceAndTorque(AC, Nav, CRB, whlH);
      NavEOMs(AC, DSM, &date, CRB, qbr, PosR, VelR, wbr, whlH, CRBk[k], qbrk[k],
              PosRk[k], VelRk[k], wbrk[k], whlHk[k], AtmoDensity);
   }

   for (Istate = INIT_STATE; Istate <= FIN_STATE; Istate++) {
      if (Nav->stateActive[Istate] == TRUE) {
         double CBR[3][3] = {{0.0}};
         switch (Istate) {
            case TIME_STATE:
               // incremented by DT at end of this function
               break;
            case ROTMAT_STATE: {
#if ORDRK == 4
               for (k = 0; k < ORDRK; k++)
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        Nav->CRB[i][j] += CRBk[k][i][j] * rkScale[k];
#elif ORDRK == 1
               MTxM(Nav->CRB, CRBk[0], CRB);
               double tmpV[3];
               tmpV[0] = CRB[2][1] * rkScale[0];
               tmpV[1] = CRB[0][2] * rkScale[0];
               tmpV[2] = CRB[1][0] * rkScale[0];
               expmso3(tmpV, CRB);
               for (i = 0; i < 3; i++)
                  for (j = 0; j < 3; j++)
                     CBR[i][j] = Nav->CRB[i][j];
               MxM(CBR, CRB, Nav->CRB);
#endif
               MT(Nav->CRB, CBR);
               C2Q(CBR, Nav->qbr);
               UNITQ(Nav->qbr);
               Q2C(Nav->qbr, CBR);
               MT(CBR, Nav->CRB);
            } break;
            case QUAT_STATE: {
#if ORDRK == 4
               for (i = 0; i < 4; i++)
                  for (k = 0; k < ORDRK; k++)
                     Nav->qbr[i] += qbrk[k][i] * rkScale[k];
#elif ORDRK == 1
               QxQT(qbrk[0], Nav->qbr, qbr);
               double tmag = UNITV(qbr) * rkScale[0];
               if (tmag > __DBL_EPSILON__) {
                  double stmag = sin(tmag);
                  for (i = 0; i < 3; i++)
                     qbrk[0][i] = qbr[i] * stmag;
                  qbrk[0][3] = cos(tmag);
                  for (i = 0; i < 4; i++)
                     qbr[i] = Nav->qbr[i];
                  QxQ(qbrk[0], qbr, Nav->qbr);
               }
#endif
               UNITQ(Nav->qbr);
               Q2C(Nav->qbr, CBR);
               MT(CBR, Nav->CRB);
            } break;
            case OMEGA_STATE:
               for (i = 0; i < 3; i++)
                  for (k = 0; k < ORDRK; k++)
                     Nav->wbr[i] += wbrk[k][i] * rkScale[k];
               break;
            case POS_STATE:
               for (i = 0; i < 3; i++)
                  for (k = 0; k < ORDRK; k++)
                     Nav->PosR[i] += PosRk[k][i] * rkScale[k];
               break;
            case VEL_STATE:
               for (i = 0; i < 3; i++)
                  for (k = 0; k < ORDRK; k++)
                     Nav->VelR[i] += VelRk[k][i] * rkScale[k];
               break;
            default:
               break;
         }
      }
   }

   for (i = 0; i < Nav->navDim; i++)
      for (j = 0; j < Nav->navDim; j++)
         Nav->NxN[i][j] = Nav->M[i][j] * Nav->sqrQ[j];

   const long subSteps = dCCSDSSec * CCSDS_FINE_MAX + dCCSDSSubSec;
   double **STM        = CreateMatrix(Nav->navDim, Nav->navDim);

   QuickMatPow(Nav->navDim, Nav->STMStep, Nav->STM, Nav->subStepSteps, STM,
               subSteps);

   MxMG(STM, Nav->NxN, Nav->NxN2, Nav->navDim, Nav->navDim, Nav->navDim);
   MxMG(STM, Nav->S, Nav->NxN, Nav->navDim, Nav->navDim, Nav->navDim);
   DestroyMatrix(STM);
   double **tmp       = CreateMatrix(Nav->navDim + Nav->navDim, Nav->navDim);
   double **U         = CreateMatrix(Nav->navDim + Nav->navDim, Nav->navDim);
   const double sqrDT = sqrt(DT);
   for (i = 0; i < Nav->navDim; i++) {
      for (j = 0; j < Nav->navDim; j++) {
         tmp[i][j]               = Nav->NxN[j][i];
         tmp[i + Nav->navDim][j] = Nav->NxN2[j][i] * sqrDT;
      }
   }
   hqrd(tmp, U, Nav->NxN, Nav->navDim + Nav->navDim, Nav->navDim); //, 3);
   DestroyMatrix(tmp);
   DestroyMatrix(U);
   for (i = 0; i < Nav->navDim; i++)
      for (j = 0; j <= i; j++)
         Nav->S[i][j] = Nav->NxN[j][i];

   for (i = 0; i < AC->Nwhl; i++)
      Nav->whlH[i] = Limit(Nav->whlH[i] + AC->Whl[i].Tcmd * DT,
                           -AC->Whl[i].Hmax, AC->Whl[i].Hmax);

   Nav->refLerpAlpha = lerpAlphaState;
   configureRefFrame(Nav, DSM->refOrb, DT / Nav->DT, FALSE);
}

void KalmanFilt(struct AcType *const AC, struct DSMType *const DSM)
{
   long i, j;
   struct DSMNavType *Nav = &DSM->DsmNav;

   // TODO: will maybe need to do something to preserve information if a new Nav
   // filter is called
   if (Nav->Init == FALSE)
      configureRefFrame(Nav, DSM->refOrb, 0.0, TRUE);

   // Accumulate information from measurements based upon batching method.
   struct DSMMeasListType *measList = &Nav->measList;
   DateToCCSDS(Nav->Date, &Nav->ccsdsSeconds, &Nav->ccsdsSubseconds);
   updateNavCCSDS(&Nav->ccsdsSeconds, &Nav->ccsdsSubseconds,
                  -(32.184 + LeapSec));
   ccsdsCoarse finSeconds  = Nav->ccsdsSeconds;
   ccsdsFine finSubseconds = Nav->ccsdsSubseconds;
   updateNavCCSDS(&finSeconds, &finSubseconds, Nav->DT);
   if (measList->head == NULL) {
      PropagateNav(
          AC, DSM, finSeconds - Nav->ccsdsSeconds,
          (signed long)finSubseconds - (signed long)Nav->ccsdsSubseconds, TRUE);
      Nav->steps++;
   }
   else {
      long init = TRUE;
      while (measList->head != NULL) {
         long measDim                    = 0;
         const enum SensorType senseType = measList->head->type;
         const long measSec              = measList->head->ccsdsSeconds;
         const long measSubsec           = measList->head->ccsdsSubseconds;

         // TODO: avoid this preallocation mess and go with realloc (maybe?)
         switch (Nav->batching) {
            case NONE_BATCH:
               measDim = measList->head->errDim;
               break;
            case SENSOR_BATCH: {
               struct DSMMeasType *meas = measList->head;
               while (meas != NULL && meas->type == senseType &&
                      meas->ccsdsSeconds == measSec &&
                      meas->ccsdsSubseconds == measSubsec) {
                  measDim += meas->errDim;
                  meas     = meas->nextMeas;
               }
            } break;
            case TIME_BATCH: {
               struct DSMMeasType *meas = measList->head;
               while (meas != NULL && meas->ccsdsSeconds == measSec &&
                      meas->ccsdsSubseconds == measSubsec) {
                  measDim += meas->errDim;
                  meas     = meas->nextMeas;
               }
            } break;
            default:
               fprintf(
                   stderr,
                   "Invalid Batching method. If you are reading this, the "
                   "developer probably has a messed up pointer. Exiting...\n");
               exit(EXIT_FAILURE);
               break;
         }

         const long dSec =
             (signed long)measSec - (signed long)Nav->ccsdsSeconds;
         const long dSubsec =
             ((signed long)measSubsec) - ((signed long)Nav->ccsdsSubseconds);

         const double ccsdsStepSize = CCSDS_STEP_SIZE;
         const double dt            = dSec + dSubsec * ccsdsStepSize;
         if (dt >= 0) {
#if REPORT_RESIDUALS ==                                                        \
    TRUE // TODO: set a report residuals "bool" in nav and use that instead of
         // these compile-time directives
            DSM_NAV_ResidualsReport(Nav->Time + (double)(Nav->subStep * DTSIM),
                                    Nav->residuals);
#endif
            // TODO: investigate only prop once per Kalman filt call and use STM
            // and linearization to prop measurements through time
            PropagateNav(AC, DSM, dSec, dSubsec, init);
            if (init == TRUE)
               init = FALSE;
            Nav->ccsdsSeconds    = measSec;
            Nav->ccsdsSubseconds = measSubsec;
#if REPORT_RESIDUALS == TRUE
            for (enum SensorType sensor = INIT_SENSOR; sensor < FIN_SENSOR;
                 sensor++) {
               if (Nav->sensorActive[sensor] == TRUE) {
                  for (i = 0; i < Nav->nSensor[sensor]; i++) {
                     free(Nav->residuals[sensor][i]);
                     Nav->residuals[sensor][i] = NULL;
                  }
               }
            }
#endif
         }
         else {
            fprintf(stderr, "Attempted to propagate Navigation state backwards "
                            "in time. How did that happen? Exiting...\n");
            exit(EXIT_FAILURE);
         }

         double **bigH, *bigResid, *bigR, **bigN;
         bigH     = CreateMatrix(measDim, Nav->navDim);
         bigN     = CreateMatrix(measDim, measDim);
         bigResid = calloc(measDim, sizeof(double));
         bigR     = calloc(measDim, sizeof(double));

         long curDim = 0;
         long dim    = -2;
         double *measEstData, **measJacobian;

         while (measList->head != NULL) {
            struct DSMMeasType *meas = pop_DSMMeas(measList);
            if (dim != meas->errDim)
               dim = meas->errDim;

            double resid[dim];
            measEstData  = (*meas->measFun)(AC, DSM, meas->sensorNum);
            measJacobian = (*meas->measJacobianFun)(AC, DSM, meas->sensorNum);

            if (meas->type == STARTRACK_SENSOR) {
               double tmpq[4];
               QxQT(meas->data, measEstData, tmpq);
               Q2AngleVec(tmpq, resid);
            }
            else {
               for (i = 0; i < dim; i++)
                  resid[i] = meas->data[i] - measEstData[i];
            }
#if REPORT_RESIDUALS == TRUE
            Nav->residuals[meas->type][meas->sensorNum] =
                calloc(dim, sizeof(double));
            for (i = 0; i < dim; i++)
               Nav->residuals[meas->type][meas->sensorNum][i] = resid[i];
#endif

            for (i = 0; i < dim; i++) {
               bigResid[curDim + i] = resid[i];
               bigR[curDim + i]     = meas->R[i];
               for (j = 0; j < Nav->navDim; j++)
                  bigH[curDim + i][j] = measJacobian[i][j];
               for (j = 0; j < dim; j++)
                  bigN[curDim + i][curDim + j] = meas->N[i][j];
            }

            curDim += dim;
            free(measEstData);
            DestroyMatrix(measJacobian);
            DestroyMeas(meas);

            if ((Nav->batching == NONE_BATCH) || (measList->head == NULL) ||
                (measList->head->ccsdsSubseconds != measSubsec ||
                 measList->head->ccsdsSeconds != measSec) || // TIME_BATCH
                (Nav->batching == SENSOR_BATCH &&
                 measList->head->type != senseType)) {
               break;
            }
         }

         double **Sz, **K, **C, **tmp;
         C = CreateMatrix(Nav->navDim, measDim);

         // UNDERWEIGHTING
         double **HS = CreateMatrix(measDim, Nav->navDim);
         MxMG(bigH, Nav->S, HS, measDim, Nav->navDim, Nav->navDim);
         double **NR = CreateMatrix(measDim, measDim);
         for (i = 0; i < measDim; i++)
            for (j = 0; j < measDim; j++)
               NR[i][j] = bigN[i][j] * bigR[j];

         tmp = CreateMatrix(Nav->navDim + measDim, measDim);
         // Compare square of matrix 2-norms for underweighting
         if (M2Norm2G(HS, measDim, Nav->navDim) >=
             (5.0 * M2Norm2G(NR, measDim, measDim))) {
            const double rtp = sqrt(1.2);
            for (i = 0; i < Nav->navDim; i++)
               for (j = 0; j < measDim; j++)
                  tmp[i][j] = HS[j][i] * rtp;
         }
         else {
            for (i = 0; i < Nav->navDim; i++)
               for (j = 0; j < measDim; j++)
                  tmp[i][j] = HS[j][i];
         }

         for (i = 0; i < measDim; i++)
            for (j = 0; j < measDim; j++)
               tmp[i + Nav->navDim][j] = NR[j][i];

         DestroyMatrix(NR);
         Sz         = CreateMatrix(measDim, measDim);
         double **U = CreateMatrix(Nav->navDim + measDim, measDim);
         hqrd(tmp, U, Sz, Nav->navDim + measDim, measDim);

         DestroyMatrix(U);
         DestroyMatrix(tmp);

         double **Uk = CreateMatrix(Nav->navDim, measDim);

         MxMTG(Nav->S, HS, C, Nav->navDim, Nav->navDim, measDim);
         DestroyMatrix(HS);

         MxMINVG(C, Sz, Uk, Nav->navDim, measDim);
         for (i = 0; i < measDim; i++) {
            for (j = i + 1; j < measDim; j++) {
               Sz[j][i] = Sz[i][j];
               Sz[i][j] = 0.0;
            }
         }
         K = CreateMatrix(Nav->navDim, measDim);
         MxMINVG(Uk, Sz, K, Nav->navDim, measDim);
         MxVG(K, bigResid, Nav->delta, Nav->navDim, measDim);
         (*Nav->updateLaw)(Nav);

         // long downdateFail = FALSE;
         for (i = 0; i < Nav->navDim; i++)
            for (j = 0; j <= i; j++) {
               Nav->NxN[j][i] = 0.0;
               Nav->NxN[i][j] = Nav->S[i][j];
            }
         for (i = 0; i < measDim; i++) {
            double u[Nav->navDim];
            for (j = 0; j < Nav->navDim; j++)
               u[j] = Uk[j][i];
            if (cholDowndate(Nav->NxN, u, Nav->navDim) == FALSE) {
               // downdateFail = TRUE;
               fprintf(stderr, "Cholesky Downdate failed! Exiting...\n");
               exit(EXIT_FAILURE);
               // TODO: data dump to help diagnose downdate failure??
               // TODO: Defer downdate if failure?
            }
         }
         for (i = 0; i < Nav->navDim; i++)
            for (j = 0; j <= i; j++)
               Nav->S[i][j] = Nav->NxN[i][j];

         DestroyMatrix(Sz);
         DestroyMatrix(Uk);
         DestroyMatrix(K);

         DestroyMatrix(bigH);
         DestroyMatrix(bigN);
         DestroyMatrix(C);
         free(bigResid);
         free(bigR);
      }
      if (Nav->ccsdsSeconds < finSeconds ||
          Nav->ccsdsSubseconds < finSubseconds)
         PropagateNav(AC, DSM, finSeconds - Nav->ccsdsSeconds,
                      (signed long)finSubseconds -
                          (signed long)Nav->ccsdsSubseconds,
                      FALSE);

      Nav->steps++;
   }
   Nav->Date = Nav->Date0;

   updateTime(&Nav->Date, Nav->DT * Nav->steps);
   DateToCCSDS(Nav->Date, &Nav->ccsdsSeconds, &Nav->ccsdsSubseconds);
   updateNavCCSDS(&Nav->ccsdsSeconds, &Nav->ccsdsSubseconds,
                  -(32.184 + LeapSec));
   configureRefFrame(Nav, DSM->refOrb, 1.0 - Nav->refLerpAlpha, TRUE);
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
   double out = LinInterp(probGate, tbl[dim - 1], pGate, nPGate);
   return (out);
}
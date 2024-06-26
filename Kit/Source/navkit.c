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

void InitMeasList(struct DSMMeasListType *list) {
   list->head    = NULL;
   list->length  = 0;
   list->measDim = 0;
}

void appendMeas(struct DSMMeasListType *list, struct DSMMeasType *newMeas) {
   struct DSMMeasType *last = list->head;
   if (newMeas == NULL) {
      return;
   } else if (last == NULL) {
      list->head = newMeas;
   } else {
      while (last->nextMeas != NULL) {
         last = last->nextMeas;
      }
      last->nextMeas = newMeas;
   }

   list->length  += 1;
   list->measDim += newMeas->errDim;
}

void appendList(struct DSMMeasListType *list1, struct DSMMeasListType *list2) {
   struct DSMMeasType *last = list1->head;
   if (list2->head == NULL) {
      return;
   } else if (last == NULL) {
      list1->head = list2->head;
   } else {
      while (last->nextMeas != NULL) {
         last = last->nextMeas;
      }
      last->nextMeas = list2->head;
   }

   list1->length  += list2->length;
   list1->measDim += list2->measDim;
}

void DestroyMeas(struct DSMMeasType *meas) {
   free(meas->data);
   free(meas->R);
   DestroyMatrix(meas->N);
   free(meas);
   meas = NULL;
}

void push(struct DSMMeasListType *list, struct DSMMeasType *meas) {
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

struct DSMMeasType *pop_DSMMeas(struct DSMMeasListType *list) {
   if (list->head == NULL)
      return NULL;
   struct DSMMeasType *meas  = list->head;
   list->head                = meas->nextMeas;
   meas->nextMeas            = NULL;
   list->length             -= 1;
   list->measDim            -= meas->errDim;
   return meas;
}

void DestroyMeasList(struct DSMMeasListType *list) {
   struct DSMMeasType *meas = pop_DSMMeas(list);
   while (meas != NULL) {
      DestroyMeas(meas);
      meas = pop_DSMMeas(list);
   }
}

struct DSMMeasType *swap_DSMMeas(struct DSMMeasType *ptr1,
                                 struct DSMMeasType *ptr2) {
   struct DSMMeasType *tmp = ptr2->nextMeas;
   ptr2->nextMeas          = ptr1;
   ptr1->nextMeas          = tmp;
   return ptr2;
}

// Measurements lists shouldn't be that long, maybe 20 or so per call, so bubble
// sort should be fine Likely won't need to sort anything anyway do to how lists
// are populated
void bubbleSort(struct DSMMeasListType *list) {
   long count = list->length;
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
                               enum sensorType const type,
                               long const sensorNum) {
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
int comparator_DSMMeas(const void *v1, const void *v2) {
   const struct DSMMeasType *m1 = *(struct DSMMeasType **)v1;
   const struct DSMMeasType *m2 = *(struct DSMMeasType **)v2;
   if (m1->step < m2->step)
      return -1;
   else if (m1->step > m2->step)
      return +1;
   else if (m1->subStep < m2->subStep)
      return -1;
   else if (m1->subStep > m2->subStep)
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
                        double const gpsSec) {
   const double secPerDay       = 86400.0;
   const double dayperWk        = 7.0;
   const double daysperRollover = 7168.0;
   const double jdGPSt0    = 2444244.5 + (32.184 + 19.0) / secPerDay; // In TT
   const double jdJ2000    = 2451545.0;
   const double gpst0J2000 = jdGPSt0 - jdJ2000; // -7300.50055768...

   double DaysSinceWeek, DaysSinceRollover, DaysSinceEpoch;

   DaysSinceWeek     = gpsSec / secPerDay;
   DaysSinceRollover = DaysSinceWeek + dayperWk * gpsWk;
   DaysSinceEpoch    = DaysSinceRollover + daysperRollover * gpsRollover;
   return ((DaysSinceEpoch + gpst0J2000) * secPerDay);
}

/**********************************************************************/
/* Given a time in seconds since J2000 TT, find the Prime Meridian    */
/* offset angle of a given world.                                     */
double GetPriMerAng(const long orbCenter, const struct DateType *date) {
   // Subtract the memory location of the starting index of the World array from
   // the memory location of W to get the index of W
   struct WorldType *W = &World[orbCenter];
   double PriMerAng    = 0.0;

   /* This is based on the behavior in Ephemerides() in 42ephem.c */
   switch (orbCenter) {
      case EARTH: {
         struct DateType dateUTC = *date;
         updateNavTime(&dateUTC, -(32.184 + (double)LeapSec));
         PriMerAng = TwoPi * Date2GMST(&dateUTC);
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
         PriMerAng = W->w * DateToTime(date->Year, date->Month, date->Day,
                                       date->Hour, date->Minute, date->Second);

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
         PriMerAng = W->w * DateToTime(date->Year, date->Month, date->Day,
                                       date->Hour, date->Minute, date->Second);
         break;
   }
   PriMerAng = fmod(PriMerAng, TwoPi);
   return PriMerAng;
}

//------------------------------------------------------------------------------
// Acceleration perturbation functions
//------------------------------------------------------------------------------
void SphericalHarmonicsJacobian(long N, long M, double r, double pbe[3],
                                double phi, double theta, double Re, double K,
                                double C[19][19], double S[19][19],
                                double HV[3][3]) {
   double P[19][19], sdP[19][19];
   long n, m;
   double cphi[M + 1], sphi[M + 1];
   double Rern1[N + 1], sth, cth;

   /* .. Order can't be greater than Degree */
   if (M > N) {
      printf("Order %ld can't be greater than Degree %ld\n", M, N);
      exit(EXIT_FAILURE);
   }

   /* .. Find Legendre functions */
   cth          = pbe[2] / r;
   double sth2  = 1.0 - cth * cth;
   sth          = sqrt(sth2); // sin(theta);
   double cotth = cth / sth;
   double rsth2 = r * r - pbe[2] * pbe[2];
   double r2 = r * r, rsth = r * sth;
   Legendre(N, M, cth, P, sdP);

   /* .. Build cos(m*phi) and sin(m*phi) */
   double rxy2  = pbe[1] * pbe[1] + pbe[0] * pbe[0];
   double denom = sqrt(rxy2);
   cphi[0]      = 1.0;
   sphi[0]      = 0.0;
   cphi[1]      = pbe[0] / denom; // cos(phi);
   sphi[1]      = pbe[1] / denom; // sin(phi);
   if (M >= 2) {
      cphi[2] = (pbe[0] * pbe[0] - pbe[1] * pbe[1]) / rxy2;
      sphi[2] = (2.0 * pbe[0] * pbe[1]) / rxy2;
      for (m = 3; m <= M; m++) {
         cphi[m] = cphi[m - 1] * cphi[1] - sphi[m - 1] * sphi[1];
         sphi[m] = sphi[m - 1] * cphi[1] + cphi[m - 1] * sphi[1];
      }
   }

   double d2Vdr2 = 0.0, d2Vdphi2 = 0.0, d2Vdtheta2 = 0.0, d2Vdrdphi = 0.0,
          d2Vdrdtheta = 0.0, d2Vdphidtheta = 0.0;
   /* .. Find Jacobian of V */
   /* .. Rern1 = (Re/r)^(n+1) */
   Rern1[0] = Re / r;
   for (n = 1; n <= N; n++)
      Rern1[n] = Rern1[n - 1] * Rern1[0];
   for (n = N; n >= 2; n--) {
      for (m = MIN(n, M); m >= 0; m--) {
         double Pbar    = P[n][m];
         double sdPbar  = sdP[n][m];
         double CcSsbar = (C[n][m] * cphi[m] + S[n][m] * sphi[m]) * Rern1[n];
         double ScCsbar = (S[n][m] * cphi[m] - C[n][m] * sphi[m]) * Rern1[n];

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
   double Kr      = K / r;
   d2Vdr2        *= Kr / r;
   d2Vdphi2      *= K;
   d2Vdtheta2    *= K;
   d2Vdrdphi     *= Kr;
   d2Vdrdtheta   *= Kr;
   d2Vdphidtheta *= K;

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

struct SphereHarmType *getGravModel(long Iworld) {
   struct SphereHarmType *gravModel = NULL;
   switch (Iworld) {
      case EARTH:
         gravModel = &EarthGravModel;
         break;
      case MARS:
         gravModel = &MarsGravModel;
         break;
      case LUNA:
         gravModel = &LunaGravModel;
         break;
      default:
         break;
   }
   return gravModel;
}

void SphericalHarmonicsHessian(long N, long M, struct WorldType *W,
                               double PriMerAng, double pbn[3],
                               double HgeoN[3][3]) {
   double CEN[3][3] = {{0.0}}, cth, sth, cph, sph, pbe[3], HV[3][3] = {{0.0}};
   double r, rr, theta, phi;
   long i, j, k;
   struct SphereHarmType *GravModel = getGravModel(W - World);

   double gradV[3] = {0.0};

   /*    Transform p to ECEF */
   CEN[0][0] = cosl(PriMerAng);
   CEN[1][1] = CEN[0][0];
   CEN[2][2] = 1.0;
   CEN[0][1] = sinl(PriMerAng);
   CEN[1][0] = -CEN[0][1];
   MxV(CEN, pbn, pbe);

   rr           = pbe[0] * pbe[0] + pbe[1] * pbe[1] + pbe[2] * pbe[2];
   r            = sqrt(rr);
   cth          = pbe[2] / r;          // cos(theta);
   sth          = sqrt(1 - cth * cth); // sin(theta);
   theta        = acos(cth);
   double rxy2  = pbe[1] * pbe[1] + pbe[0] * pbe[0];
   double denom = sqrt(rxy2);
   sph          = pbe[1] / denom; // sin(phi);
   cph          = pbe[0] / denom; // cos(phi);
   phi          = atan2(pbe[1], pbe[0]);

   double MSE[3][3] = {{pbe[0] / r, pbe[1] / r, cth},
                       {cth * cph, cth * sph, -sth},
                       {-sph, cph, 0.0}};

   /*    Find Jacobian */
   SphericalHarmonicsJacobian(N, M, r, pbe, phi, theta, W->rad, W->mu / W->rad,
                              GravModel->C, GravModel->S, HV);

   /*    Calculate scaled Christoffel Symbols */
   /*      sCS^k_{ij} = CS^k_{ij} * sqrt(g_{kk}) / (sqrt(g_{ii})*sqrt(g_{jj}))
    */
   /*      due to scaling of gradV and scaling in polar transform */
   double sCS[3][3][3] = {{{0.0}}};
   sCS[0][1][1]        = -1.0 / r;      // -r * 1 / (r*r) = -1 / r
   sCS[0][2][2]        = sCS[0][1][1];  // -rsth*sth * 1 / (rsth*rsth) = -1/r
   sCS[1][0][1]        = -sCS[0][1][1]; // 1/r * r / (1*r) = 1/r
   sCS[1][1][0]        = sCS[1][0][1];
   sCS[1][2][2] =
       -pbe[2] / (r * sqrt(rxy2)); // -sth*cth * r / (rsth*rsth) = -cth / rsth
   sCS[2][0][2] = sCS[1][0][1];    // 1/r * rsth / (1*rsth) = 1 / r
   sCS[2][1][2] = -sCS[1][2][2];   // cth/sth * rsth / (r*rsth) = cth / rsth
   sCS[2][2][0] = sCS[2][0][2];
   sCS[2][2][1] = sCS[2][1][2];

   SphericalHarmonics(N, M, r, pbe, W->rad, W->mu / W->rad, GravModel->C,
                      GravModel->S, gradV);
   for (k = 0; k < 3; k++)
      for (i = 0; i < 3; i++)
         for (j = 0; j < 3; j++)
            HV[i][j] -= gradV[k] * sCS[k][i][j];

   /*    Transform back to cartesian coords in Newtonian frame */
   double CSN[3][3];
   MxM(MSE, CEN, CSN);
   AdjointT(CSN, HV, HgeoN);
}

void getGravAccel(double const mu, double const pos[3], double gravFrc[3]) {
   int i;
   double posHat[3];
   double posMag = CopyUnitV(pos, posHat);

   double gravScale = -mu / (posMag * posMag);
   for (i = 0; i < 3; i++)
      gravFrc[i] = posHat[i] * gravScale;
}

void getDGravFrcDPos(double const mu, double const pos[3],
                     double dGravFrcdPos[3][3]) {
   int i, j;
   double posHat[3];
   for (i = 0; i < 3; i++)
      posHat[i] = pos[i];
   double posMag    = UNITV(posHat);
   double gravScale = -mu / (posMag * posMag * posMag);

   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++)
         dGravFrcdPos[i][j] = -posHat[i] * posHat[j] * 3.0;
      dGravFrcdPos[i][i] += 1.0;
      for (j = 0; j < 3; j++)
         dGravFrcdPos[i][j] *= gravScale;
   }
}

void ThirdBodyGravAccel(double p[3], double s[3], double mu, double accel[3]) {
   double magp, mags, p3, s3;
   long j;

   magp = MAGV(p);
   mags = MAGV(s);
   p3   = magp * magp * magp;
   s3   = mags * mags * mags;
   for (j = 0; j < 3; j++)
      accel[j] = mu * (s[j] / s3 - p[j] / p3);
}

void NavGravPertAccel(struct DSMNavType *Nav, const struct DateType *date,
                      double PosR[3], double mass, long RefOrb,
                      double VelRdot[3]) {
   struct OrbitType *O;
   double ph[3], pn[3], pr[3], s[3], accelR[3];
   long Iw, Im, j;
   long OrbCenter, SecCenter;

   O = &Orb[RefOrb];
   if (O->Regime == ORB_CENTRAL) {
      OrbCenter = O->World;
      SecCenter = -1; /* Nonsense value */
   } else {
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
   struct SphereHarmType *GravModel = getGravModel(OrbCenter);
   if (GravModel->N >= 2) {
      double PriMerAng = GetPriMerAng(OrbCenter, date);
      double fGeoN[3], fGeoR[3], PosN[3];
      MTxV(Nav->refCRN, PosR, PosN);
      void (*SphericalHarmGravForce)(const char *ModelPath, long N, long M,
                                     double C[19][19], double S[19][19],
                                     double mass, double pbn[3],
                                     double PriMerAng, double FgeoN[3]) = NULL;
      switch (OrbCenter) {
         case EARTH:
            SphericalHarmGravForce = &EGM96;
            break;
         case MARS:
            SphericalHarmGravForce = &GMM2B;
            break;
         case LUNA:
            SphericalHarmGravForce = &GLGM2;
            break;
         default:
            break;
      }
      SphericalHarmGravForce(ModelPath, GravModel->N, GravModel->M,
                             GravModel->C, GravModel->S, mass, PosN, PriMerAng,
                             fGeoN);
      MxV(Nav->refCRN, fGeoN, fGeoR);
      for (j = 0; j < 3; j++)
         VelRdot[j] += fGeoR[j] / mass;
   }
}

void NavDGravPertAccelDPos(struct DSMNavType *Nav, const struct DateType *date,
                           double PosR[3], long RefOrb,
                           double dGravDPos[3][3]) {
   struct OrbitType *O;
   double ph[3], pn[3], pr[3], s[3], dGdR[3][3];
   long Iw, Im, i, j;
   long OrbCenter, SecCenter;

   O = &Orb[RefOrb];
   if (O->Regime == ORB_CENTRAL) {
      OrbCenter = O->World;
      SecCenter = -1; /* Nonsense value */
   } else {
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
   struct SphereHarmType *GravModel = getGravModel(OrbCenter);
   if (GravModel->N >= 2) {
      double PriMerAng   = GetPriMerAng(OrbCenter, date);
      double HgeoN[3][3] = {{0.0}}, HgeoR[3][3] = {{0.0}}, PosN[3] = {0.0};
      MTxV(Nav->refCRN, PosR, PosN);
      SphericalHarmonicsHessian(GravModel->N, GravModel->M, WCenter, PriMerAng,
                                PosN, HgeoN);
      if (Nav->refFrame != FRAME_N) {
         Adjoint(Nav->refCRN, HgeoN, HgeoR);
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               dGravDPos[i][j] += HgeoR[i][j];
      } else {
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               dGravDPos[i][j] += HgeoN[i][j];
      }
   }
}

void getAeroForceAndTorque(struct SCType *const S, double const CRB[3][3],
                           double const VrelR[3], double frcR[3], double trq[3],
                           double AtmoDensity) {
   struct BodyType *B;
   struct GeomType *G;
   struct PolyType *P;
   long Ib, Ipoly, i;
   double WoN, PolyArea, Area, WindSpeed, Coef1, Coef2;
   double cp[3], Vrelb[3], fb[3], fR[3], trqB[3], VrelRHat[3];

   WindSpeed = CopyUnitV(VrelR, VrelRHat);
   // TODO: don't use S->AtmoDensity (and maybe S->DragCoef?)
   // TODO: be able to choose between ballistic coef model and more accurate
   // model basllistic coef is noticeably faster and simplification doesn't
   // change much if torque is trivial

   // Coef1 = -0.5 * AtmoDensity * WindSpeed * WindSpeed * AC->mass /
   // Nav->ballisticCoef; for (i = 0; i < 3; i++)
   //    frcR[i] = Coef1 * VrelRHat[i];

   Coef1 = -0.5 * AtmoDensity * S->DragCoef * WindSpeed * WindSpeed;

   /* Find Force and Torque on each Body, in that body's frame */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];

      double CBb[3][3] = {{1.0, 0.0, 0.0},
                          {0.0, 1.0, 0.0},
                          {0.0, 0.0, 1.0}}; // rotation from Ib to zeroth body
      double CRb[3][3];                     // rotation from Ib to Nav ref frame
      if (Ib != 0) {
         MxMT(S->B[0].CN, B->CN, CBb);
      }
      MxM(CRB, CBb, CRb);

      /* Transform Rel Wind to B */
      MTxV(CRb, VrelRHat, Vrelb);

      /* Find total projected area and cp for Body */
      Area = 0.0;
      for (i = 0; i < 3; i++)
         cp[i] = 0.0;
      G = &Geom[B->GeomTag];
      for (Ipoly = 0; Ipoly < G->Npoly; Ipoly++) {
         P   = &G->Poly[Ipoly];
         WoN = VoV(Vrelb, P->Norm);
         if (WoN > 0.0) {
            PolyArea  = WoN * P->Area;
            Area     += PolyArea;
            for (i = 0; i < 3; i++)
               cp[i] += PolyArea * (P->Centroid[i] - B->cm[i]);
         }
      }
      if (Area > 0.0) {
         for (i = 0; i < 3; i++)
            cp[i] /= Area;
      }

      /* Compute force and torque exerted on B */
      Coef2 = Coef1 * Area;
      for (i = 0; i < 3; i++)
         fb[i] = Coef2 * Vrelb[i];
      MxV(CRb, fb, fR);
      for (i = 0; i < 3; i++) {
         frcR[i] += fR[i];
      }
      VxV(cp, fb, trqB);
      for (i = 0; i < 3; i++)
         trq[i] += trqB[i];
   }
}

void getDAeroFrcAndTrqDVRel(struct SCType *const S, double const CRB[3][3],
                            double const VrelR[3], double dAeroFrcdVRel[3][3],
                            double dAeroTrqdVRel[3][3], double AtmoDensity) {
   struct BodyType *B;
   struct GeomType *G;
   struct PolyType *P;
   long Ib, Ipoly, i, j;
   double WoN, PolyArea, Area, WindSpeed, Coef;
   double Vrelb[3], cp[3], VrelRHat[3];

   double AiniT[3] = {0.0};
   WindSpeed       = CopyUnitV(VrelR, VrelRHat);
   // TODO: be able to choose between ballistic coef model and more accurate
   // model basllistic coef is noticeably faster and simplification doesn't
   // change much if torque is trivial

   Coef = -0.5 * AtmoDensity * S->DragCoef * WindSpeed;
   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         dAeroFrcdVRel[i][j] = 0.0;
         dAeroTrqdVRel[i][j] = 0.0;
      }
   }

   /* .. Find Force and Torque on each Body, in that body's frame */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];

      double CBb[3][3] = {{1.0, 0.0, 0.0},
                          {0.0, 1.0, 0.0},
                          {0.0, 0.0, 1.0}}; // rotation from Ib to zeroth body
      double CRb[3][3];                     // rotation from Ib to Nav ref frame
      if (Ib != 0) {
         MxMT(S->B[0].CN, B->CN, CBb);
      }
      MxM(CRB, CBb, CRb);

      /* Transform Rel Wind to B */
      MTxV(CRb, VrelRHat, Vrelb);

      double AiciniT[3][3] = {{0.0}}, Ajvjnj[3][3] = {{0.0}};
      /* Find total projected area and cp for Body */
      Area = 0.0;
      for (i = 0; i < 3; i++)
         cp[i] = 0.0;
      G = &Geom[B->GeomTag];
      for (Ipoly = 0; Ipoly < G->Npoly; Ipoly++) {
         P   = &G->Poly[Ipoly];
         WoN = VoV(Vrelb, P->Norm);
         if (WoN > 0.0) {
            PolyArea         = WoN * P->Area;
            Area            += PolyArea;
            double CRbni[3]  = {0.0};
            MxV(CRb, P->Norm, CRbni);
            double tmpM[3][3] = {{0.0}}, tmpM2[3][3] = {{0.0}};
            for (i = 0; i < 3; i++) {
               double ci  = P->Centroid[i] - B->cm[i];
               AiniT[i]  += P->Area * CRbni[i];
               cp[i]     += PolyArea * ci;

               tmpM[i][i] += P->Area * WoN;
               for (j = 0; j < 3; j++) {
                  AiciniT[i][j] += P->Area * ci * P->Norm[j];
                  tmpM[i][j]    -= P->Area * Vrelb[i] * P->Norm[j];
               }
            }
            MxMT(tmpM, CRb, tmpM2);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  Ajvjnj[i][j] += tmpM2[i][j];
         }
      }
      if (Area > 0.0) {
         for (i = 0; i < 3; i++) {
            cp[i] /= Area;
            for (j = 0; j < 3; j++) {
               AiciniT[i][j] /= Area;
               Ajvjnj[i][j]  /= Area;
            }
         }

         double dAeroFrcdVRelIb[3][3] = {{0.0}};
         for (i = 0; i < 3; i++) {
            dAeroFrcdVRelIb[i][i] += Area;
            for (j = 0; j < 3; j++) {
               dAeroFrcdVRelIb[i][j] += VrelRHat[i] * AiniT[j];
               dAeroFrcdVRelIb[i][j] *= Coef;
               dAeroFrcdVRel[i][j]   += dAeroFrcdVRelIb[i][j];
            }
         }
         /* Compute torque exerted on B */
         double tmpM[3][3], tmpM2[3][3], cpX[3][3];
         for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++)
               tmpM[i][j] = Vrelb[i] * Vrelb[j];
            tmpM[i][i] += 1.0;
         }

         MxM(Ajvjnj, tmpM, tmpM2);
         MxM(AiciniT, tmpM2, tmpM);
         V2CrossM(Vrelb, AiciniT);
         MxM(AiciniT, tmpM, tmpM2);

         V2CrossM(cp, cpX);
         AdjointT(CRb, dAeroFrcdVRelIb, tmpM);
         MxM(cpX, tmpM, dAeroFrcdVRelIb);
         double ACoef = Area * Coef;
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               dAeroTrqdVRel[i][j] += tmpM[i][j] - tmpM2[i][j] * ACoef;
      }
   }
}

//------------------------------------------------------------------------------
//                               NAV FUNCTIONS
//------------------------------------------------------------------------------

double **gyroJacobianFun(struct SCType *const S, const long Igyro) {
   double Axis[3] = {0.0}, tmp[3] = {0.0}, tmp2[3] = {0.0};
   static double **B = NULL; // if its static, just need to allocate once,
                             // instead of allocate/deallocate
   double **jacobian;
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   struct AcGyroType *gyro;
   struct NodeType *N;
   long i;

   AC   = &S->AC;
   DSM  = &S->DSM;
   Nav  = &DSM->DsmNav;
   gyro = &AC->Gyro[Igyro];
   N    = &S->B[0].Node[S->Gyro[Igyro].Node];

   if (B == NULL)
      B = CreateMatrix(1, 3);

   jacobian = CreateMatrix(Nav->measTypes[GYRO_SENSOR][Igyro].dim, Nav->navDim);

   switch (Nav->type) {
      case LIEKF_NAV:
         MTxV(Nav->CRB, Nav->refOmega, tmp);
         for (i = 0; i < 3; i++)
            tmp[i] += Nav->wbr[i];
         QTxV(N->qb, gyro->Axis, Axis);
         VxV(tmp, Axis, tmp2);

         for (i = 0; i < 3; i++)
            B[0][i] = tmp2[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
         for (i = 0; i < 3; i++)
            B[0][i] = -Axis[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[OMEGA_STATE], 1, 3);
         break;
      case RIEKF_NAV:
         QTxV(N->qb, gyro->Axis, tmp);
         MxV(Nav->CRB, tmp, Axis);
         for (i = 0; i < 3; i++)
            B[0][i] = -Axis[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[OMEGA_STATE], 1, 3);

         VxV(Nav->refOmega, Axis, tmp2);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp2[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
         break;
      case MEKF_NAV:
         QTxV(N->qb, gyro->Axis, Axis);
         for (i = 0; i < 3; i++)
            B[0][i] = -Axis[i] * R2D;
         subMatAdd(jacobian, B, 0, Nav->navInd[OMEGA_STATE], 1, 3);
         if (Nav->refFrame != FRAME_N) {
            QxV(Nav->qbr, Nav->refOmega, tmp);
            VxV(tmp, Axis, tmp2);

            for (i = 0; i < 3; i++)
               B[0][i] = tmp2[i] * R2D;
            subMatAdd(jacobian, B, 0, Nav->navInd[QUAT_STATE], 1, 3);
         }
         break;
      default:
         printf("Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double **magJacobianFun(struct SCType *const S, const long Imag) {
   double **jacobian, tmp[3] = {0.0}, tmp2[3] = {0.0}, Axis[3] = {0.0};
   static double **B = NULL; // if its static, just need to allocate once,
                             // instead of allocate/deallocate
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   struct AcMagnetometerType *mag;
   struct NodeType *N;
   const double T2mG = 1.0e7; // tesla to milligauss
   long i;

   AC  = &S->AC;
   DSM = &S->DSM;
   Nav = &DSM->DsmNav;
   mag = &AC->MAG[Imag];
   N   = &S->B[0].Node[S->MAG[Imag].Node];

   if (B == NULL)
      B = CreateMatrix(1, 3);

   jacobian = CreateMatrix(Nav->measTypes[MAG_SENSOR][Imag].dim, Nav->navDim);

   switch (Nav->type) {
      case LIEKF_NAV:
         MxV(Nav->refCRN, AC->bvn, tmp);
         MTxV(Nav->CRB, tmp, tmp2);
         QTxV(N->qb, mag->Axis, Axis);
         VxV(tmp2, Axis, tmp);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp[i] * T2mG;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
         break;
      case RIEKF_NAV:
         MxV(Nav->refCRN, AC->bvn, tmp2);
         QTxV(N->qb, mag->Axis, tmp);
         MxV(Nav->CRB, tmp, Axis);
         VxV(tmp2, Axis, tmp);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp[i] * T2mG;
         subMatAdd(jacobian, B, 0, Nav->navInd[ROTMAT_STATE], 1, 3);
         break;
      case MEKF_NAV:
         MxV(Nav->refCRN, AC->bvn, tmp);
         QxV(Nav->qbr, tmp, tmp2);
         QTxV(N->qb, mag->Axis, Axis);
         VxV(tmp2, Axis, tmp);
         for (i = 0; i < 3; i++)
            B[0][i] = tmp[i] * T2mG;
         subMatAdd(jacobian, B, 0, Nav->navInd[QUAT_STATE], 1, 3);
         break;
      default:
         printf("Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double **cssJacobianFun(struct SCType *const S, const long Icss) {
   double **jacobian, tmp[3] = {0.0}, svb[3] = {0.0}, svr[3] = {0.0};
   static double **B = NULL; // if its static, just need to allocate once,
                             // instead of allocate/deallocate
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   struct AcCssType *css;
   long i;

   AC  = &S->AC;
   DSM = &S->DSM;
   Nav = &DSM->DsmNav;
   css = &AC->CSS[Icss];

   if (B == NULL)
      B = CreateMatrix(1, 3);

   MxV(Nav->refCRN, AC->svn, svr);

   jacobian = CreateMatrix(Nav->measTypes[CSS_SENSOR][Icss].dim, Nav->navDim);

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
         printf("Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double **fssJacobianFun(struct SCType *const S, const long Ifss) {
   double **jacobian, B[3][3] = {{0.0}}, tmp3x3[3][3] = {{0.0}};
   struct AcFssType *fss;
   static double **tmpAssign = NULL;
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   double svb[3], svs[3], CBN[3][3];
   double bhat[3] = {0.0}, hhat[3] = {0.0}, vhat[3] = {0.0};
   double svsb, svsh, svsv, bxsvs[3], hxsvs[3], vxsvs[3];
   double denomA, denomB;
   long H_Axis, V_Axis, BoreAxis;
   long i, j;

   AC       = &S->AC;
   DSM      = &S->DSM;
   Nav      = &DSM->DsmNav;
   fss      = &AC->FSS[Ifss];
   H_Axis   = S->FSS[Ifss].H_Axis;
   V_Axis   = S->FSS[Ifss].V_Axis;
   BoreAxis = S->FSS[Ifss].BoreAxis;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(2, 3);

   MTxM(Nav->CRB, Nav->refCRN, CBN);
   MxV(CBN, AC->svn, svb);
   MxV(fss->CB, svb, svs);

   svsb   = svs[BoreAxis];
   svsh   = svs[H_Axis];
   svsv   = svs[V_Axis];
   denomA = 1.0 / (svsb * svsb + svsh * svsh);
   denomB = 1.0 / (svsb * svsb + svsv * svsv);

   bhat[BoreAxis] = 1.0;
   hhat[H_Axis]   = 1.0;
   vhat[V_Axis]   = 1.0;

   VxV(bhat, svs, bxsvs);
   VxV(hhat, svs, hxsvs);
   VxV(vhat, svs, vxsvs);

   jacobian = CreateMatrix(Nav->measTypes[FSS_SENSOR][Ifss].dim, Nav->navDim);

   for (i = 0; i < 3; i++) {
      B[0][i] = (svsh * bxsvs[i] - svsb * hxsvs[i]) * denomA;
      B[1][i] = (svsv * bxsvs[i] - svsb * vxsvs[i]) * denomB;
   }
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
         printf("Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double **startrackJacobianFun(struct SCType *const S, const long Ist) {
   double **jacobian, tmpM[3][3] = {{0.0}}, qsb[4], CSB[3][3];
   static double **tmpAssign = NULL;
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   struct AcStarTrackerType *st;
   long i, j;

   struct NodeType *N;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   AC  = &S->AC;
   DSM = &S->DSM;
   Nav = &DSM->DsmNav;
   st  = &AC->ST[Ist];
   N   = &S->B[0].Node[S->ST[Ist].Node];

   jacobian =
       CreateMatrix(Nav->measTypes[STARTRACK_SENSOR][Ist].errDim, Nav->navDim);
   QxQ(st->qb, N->qb, qsb);
   Q2C(qsb, CSB);

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
         printf("Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double **gpsJacobianFun(struct SCType *const S, const long Igps) {
   double **jacobian, tmp1[3][3] = {{0.0}}, tmp2[3][3] = {{0.0}},
                      tmp3[3][3] = {{0.0}}, tmpX[3][3] = {{0.0}},
                      tmpV[3] = {0.0};
   static double **tmpAssign  = NULL;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   long i, j;

   DSM = &S->DSM;
   Nav = &DSM->DsmNav;

   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

   jacobian = CreateMatrix(Nav->measTypes[GPS_SENSOR][Igps].dim, Nav->navDim);

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
         printf("Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double **accelJacobianFun(struct SCType *const S, const long Iaccel) {
   double **jacobian;
   struct DSMType *DSM;
   struct DSMNavType *Nav;

   DSM = &S->DSM;
   Nav = &DSM->DsmNav;

   jacobian =
       CreateMatrix(Nav->measTypes[ACCEL_SENSOR][Iaccel].dim, Nav->navDim);

   switch (Nav->type) {
      case LIEKF_NAV:
         break;
      case RIEKF_NAV:
         break;
      case MEKF_NAV:
         break;
      default:
         printf("Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (jacobian);
}

double *gyroFun(struct SCType *const S, const long Ig) {
   struct AcGyroType *G;
   struct NodeType *N;
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   double axis[3], wbn[3], wrn[3], *gyroEst;
   long i;

   gyroEst = calloc(1, sizeof(double));

   AC  = &S->AC;
   DSM = &S->DSM;
   Nav = &DSM->DsmNav;
   G   = &AC->Gyro[Ig];
   N   = &S->B[0].Node[S->Gyro[Ig].Node];

   QTxV(N->qb, G->Axis, axis);
   MTxV(Nav->CRB, Nav->refOmega, wrn);
   for (i = 0; i < 3; i++)
      wbn[i] = Nav->wbr[i] + wrn[i];
   gyroEst[0] = VoV(axis, wbn) * R2D;
   return (gyroEst);
}

double *magFun(struct SCType *const S, const long Imag) {
   struct AcMagnetometerType *MAG;
   struct NodeType *N;
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   double axis[3], bvb[3], bvn[3], CBN[3][3], *magEst;
   const double T2mG = 1.0e7; // tesla to milligauss
   long i;

   magEst = calloc(1, sizeof(double));

   AC  = &S->AC;
   DSM = &S->DSM;
   Nav = &DSM->DsmNav;
   MAG = &AC->MAG[Imag];
   N   = &S->B[0].Node[S->MAG[Imag].Node];

   // Not to really be used.
   for (i = 0; i < 3; i++)
      bvn[i] = AC->bvn[i];

   MTxM(Nav->CRB, Nav->refCRN, CBN);
   MxV(CBN, bvn, bvb);
   QTxV(N->qb, MAG->Axis, axis);
   magEst[0] = VoV(axis, bvb) * T2mG;

   return (magEst);
}

double *cssFun(struct SCType *const S, const long Icss) {
   struct AcCssType *css;
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   double *IllumEst, svb[3], CBN[3][3];

   IllumEst = calloc(1, sizeof(double));

   AC  = &S->AC;
   DSM = &S->DSM;
   Nav = &DSM->DsmNav;
   css = &AC->CSS[Icss];

   MTxM(Nav->CRB, Nav->refCRN, CBN);
   MxV(CBN, AC->svn, svb);
   IllumEst[0] = VoV(svb, css->Axis) * css->Scale;

   return (IllumEst);
}

double *fssFun(struct SCType *const S, const long Ifss) {
   struct AcFssType *fss;
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   double *SunAngEst, svb[3], svs[3], CBN[3][3];
   long H_Axis, V_Axis, BoreAxis;

   SunAngEst = calloc(2, sizeof(double));

   AC       = &S->AC;
   DSM      = &S->DSM;
   Nav      = &DSM->DsmNav;
   fss      = &AC->FSS[Ifss];
   H_Axis   = S->FSS[Ifss].H_Axis;
   V_Axis   = S->FSS[Ifss].V_Axis;
   BoreAxis = S->FSS[Ifss].BoreAxis;

   MTxM(Nav->CRB, Nav->refCRN, CBN);
   MxV(CBN, AC->svn, svb);
   MxV(fss->CB, svb, svs);

   SunAngEst[0] = atan2(svs[H_Axis], svs[BoreAxis]);
   SunAngEst[1] = atan2(svs[V_Axis], svs[BoreAxis]);

   return (SunAngEst);
}

double *startrackFun(struct SCType *const S, const long Ist) {
   struct AcStarTrackerType *st;
   struct NodeType *N;
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   double qsb[4], qbn[4], qrn[4], *qsnEst;

   qsnEst = calloc(4, sizeof(double));
   AC     = &S->AC;
   DSM    = &S->DSM;
   Nav    = &DSM->DsmNav;
   st     = &AC->ST[Ist];
   N      = &S->B[0].Node[S->ST[Ist].Node];

   QxQ(st->qb, N->qb, qsb);
   C2Q(Nav->refCRN, qrn);
   QxQ(Nav->qbr, qrn, qbn);
   QxQ(qsb, qbn, qsnEst);

   return (qsnEst);
}

double *gpsFun(struct SCType *const S, const long Igps) {
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   double *posNVelNEst, tmp3V[3], tmpPosN[3], tmpVelN[3];
   long i;

   posNVelNEst = calloc(6, sizeof(double));
   DSM         = &S->DSM;
   Nav         = &DSM->DsmNav;

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
double *accelFun(struct SCType *const S, const long Ia) {
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
                        double *NavGeomagIndex) {
   if (AtmoOption == TWOSIGMA_ATMO) {
      *NavFlux10p7    = LinInterp(SchattenTable[0], SchattenTable[1], JD, 410);
      *NavGeomagIndex = LinInterp(SchattenTable[0], SchattenTable[3], JD, 410);
   } else if (AtmoOption == NOMINAL_ATMO) {
      *NavFlux10p7    = LinInterp(SchattenTable[0], SchattenTable[2], JD, 410);
      *NavGeomagIndex = LinInterp(SchattenTable[0], SchattenTable[4], JD, 410);
   } else {
      // Pull from user-defined values in Inp_Sim.txt
      *NavFlux10p7    = Flux10p7;
      *NavGeomagIndex = GeomagIndex;
   }
}

/*--------------------------------------------------------------------*/
/*                          RIEKF functions                           */
/*--------------------------------------------------------------------*/

void eomRIEKFJacobianFun(struct SCType *const S, const struct DateType *date,
                         double const CRB[3][3], double const qbr[4],
                         double const PosR[3], double const VelR[3],
                         double const wbr[3], double const whlH[S->AC.Nwhl],
                         const double AtmoDensity) {
   double tmpM[3][3] = {{0.0}}, tmpM2[3][3] = {{0.0}}, tmpM3[3][3] = {{0.0}},
          tmpV[3] = {0.0}, tmpV2[3] = {0.0}, tmpV3[3] = {0.0};
   static double **tmpAssign = NULL;
   double wrnd[3]            = {0.0};
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   long i, j, rowInd;
   enum navState state;

   AC  = &S->AC;
   DSM = &S->DSM;
   Nav = &DSM->DsmNav;

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
                  MxM(tmpM2, AC->MOI, tmpM3);
                  MxV(AC->MOI, tmpV3, tmpV2);
                  for (long Iw = 0; Iw < AC->Nwhl; Iw++)
                     for (i = 0; i < 3; i++)
                        tmpV2[i] += AC->Whl[Iw].Axis[i] * whlH[Iw];

                  V2CrossM(tmpV2, tmpM);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpM[i][j] -= tmpM3[i][j];

                  MINVxM3(AC->MOI, 3, tmpM, tmpM2);
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
      if (Orb[S->RefOrb].Regime == ORB_CENTRAL) {
         long orbCenter             = Orb[S->RefOrb].World;
         double dAeroFrcdVRel[3][3] = {{0.0}}, dAeroTrqdVRel[3][3] = {{0.0}};
         double worldWR[3] = {0.0};
         if (AeroActive) {
            double VrelR[3] = {0.0}, PosRWorld[3] = {0.0};
            for (i = 0; i < 3; i++)
               PosRWorld[i] = PosR[i] + Nav->refPos[i];
            for (i = 0; i < 3; i++)
               worldWR[i] = -Nav->refCRN[i][2] * World[orbCenter].w;
            VxV(worldWR, PosRWorld, VrelR);
            for (i = 0; i < 3; i++)
               VrelR[i] += VelR[i] + Nav->refVel[i];
            getDAeroFrcAndTrqDVRel(S, CRB, VrelR, dAeroFrcdVRel, dAeroTrqdVRel,
                                   AtmoDensity);
            MxM(Nav->CRB, dAeroTrqdVRel, tmpM);
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  dAeroTrqdVRel[i][j] = tmpM[i][j];
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
                           NavGravPertAccel(Nav, date, tmpV, 1.0, S->RefOrb,
                                            accelR);
                           for (i = 0; i < 3; i++)
                              VelRdot[i] += accelR[i];
                        }
                        for (i = 0; i < 3; i++)
                           tmpV[i] = PosR[i] + Nav->refPos[i];
                        getGravAccel(Orb[S->RefOrb].mu, tmpV, tmpV2);
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
                        for (i = 0; i < 3; i++)
                           tmpV2[i] = PosR[i] + Nav->refPos[i];
                        NavDGravPertAccelDPos(Nav, date, tmpV2, S->RefOrb,
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
                           for (j = 0; j < 3; j++) {
                              dAeroFrcdVRel[i][j] /= AC->mass;
                              tmpAssign[i][j]      = dAeroFrcdVRel[i][j];
                           }
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
      } else {
         printf("Orbit types other than CENTRAL are still in development for "
                "filtering. Exiting...\n");
         exit(EXIT_FAILURE);
      }
   } else {
      printf("For the moment, can only filter rotation matrix, position, "
             "velocity, & angular "
             "velocity *simultaneously* with the RIEKF. Exiting...\n");
      exit(EXIT_FAILURE);
   }
}

void RIEKFUpdateLaw(struct DSMNavType *const Nav) {
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
      } else if (j == OMEGA_STATE) {
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

void eomLIEKFJacobianFun(struct SCType *const S, const struct DateType *date,
                         double const CRB[3][3], double const qbr[4],
                         double const PosR[3], double const VelR[3],
                         double const wbr[3], double const whlH[S->AC.Nwhl],
                         const double AtmoDensity) {
   double tmpM[3][3] = {{0.0}}, tmpM2[3][3] = {{0.0}}, tmpM3[3][3] = {{0.0}},
          tmpV[3] = {0.0}, tmpV2[3] = {0.0}, tmpV3[3] = {0.0};
   static double **tmpAssign = NULL;
   double wrnd[3]            = {0.0};
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   long i, j, rowInd;
   enum navState state;

   AC  = &S->AC;
   DSM = &S->DSM;
   Nav = &DSM->DsmNav;

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
       Nav->stateActive[VEL_STATE] && Nav->stateActive[OMEGA_STATE] &&
       !Nav->stateActive[QUAT_STATE]) {
      for (state = INIT_STATE; state <= FIN_STATE; state++) {
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
                  MxV(AC->MOI, tmpV3, tmpV);
                  for (long Iw = 0; Iw < AC->Nwhl; Iw++) {
                     for (i = 0; i < 3; i++)
                        tmpV[i] += AC->Whl[Iw].Axis[i] * whlH[Iw];
                  }
                  VxV(tmpV, tmpV3, tmpV2);
                  if (AeroActive) {
                     long orbCenter    = Orb[S->RefOrb].World;
                     double aeroTrq[3] = {0.0}, worldWR[3] = {0.0},
                            VrelR[3]     = {0.0};
                     double PosRWorld[3] = {0.0};
                     for (i = 0; i < 3; i++)
                        PosRWorld[i] = PosR[i] + Nav->refPos[i];
                     for (i = 0; i < 3; i++)
                        worldWR[i] = -Nav->refCRN[i][2] * World[orbCenter].w;
                     VxV(worldWR, PosRWorld, VrelR);
                     for (i = 0; i < 3; i++)
                        VrelR[i] += VelR[i] + Nav->refVel[i];
                     getAeroForceAndTorque(S, CRB, VrelR, tmpV, aeroTrq,
                                           AtmoDensity);
                     for (i = 0; i < 3; i++)
                        tmpV2[i] += aeroTrq[i];
                  }
                  for (i = 0; i < 3; i++) {
                     tmpV2[i] += Nav->torqueB[i];
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = AC->MOI[i][j];
                  }
                  LINSOLVE(tmpAssign, tmpV, tmpV2,
                           3); // tmpV = wbn_dot (expressed in B, wrt N)
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
                  MxM(tmpM2, AC->MOI, tmpM3);
                  MxV(AC->MOI, tmpV3, tmpV2);
                  for (long Iw = 0; Iw < AC->Nwhl; Iw++) {
                     for (i = 0; i < 3; i++)
                        tmpV2[i] += AC->Whl[Iw].Axis[i] * whlH[Iw];
                  }
                  V2CrossM(tmpV2, tmpM);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpM[i][j] -= tmpM3[i][j];
                  MINVxM3(AC->MOI, 3, tmpM, tmpM2);
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
                     tmpV[i] = Nav->forceB[i] / AC->mass;
                  }
                  V2CrossM(tmpV, tmpM);
                  MxM(CRB, tmpM, tmpM2);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpAssign[i][j] = tmpM2[i][j];
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
      if (Orb[S->RefOrb].Regime == ORB_CENTRAL) {
         long orbCenter             = Orb[S->RefOrb].World;
         double dAeroFrcdVRel[3][3] = {{0.0}}, dAeroTrqdVRel[3][3] = {{0.0}};
         double worldWR[3] = {0.0};
         if (AeroActive) {
            double VrelR[3]     = {0.0};
            double PosRWorld[3] = {0.0};
            for (i = 0; i < 3; i++)
               PosRWorld[i] = PosR[i] + Nav->refPos[i];
            for (i = 0; i < 3; i++)
               worldWR[i] = -Nav->refCRN[i][2] * World[orbCenter].w;
            VxV(worldWR, PosRWorld, VrelR);
            for (i = 0; i < 3; i++)
               VrelR[i] += VelR[i] + Nav->refVel[i];
            getDAeroFrcAndTrqDVRel(S, CRB, VrelR, dAeroFrcdVRel, dAeroTrqdVRel,
                                   AtmoDensity);
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
                        NavDGravPertAccelDPos(Nav, date, tmpV2, S->RefOrb,
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
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              dAeroFrcdVRel[i][j] /= AC->mass;
                        AdjointT(CRB, dAeroFrcdVRel, tmpM3);
                        for (i = 0; i < 3; i++)
                           for (j = 0; j < 3; j++)
                              tmpAssign[i][j] = tmpM3[i][j];
                        subMatAdd(Nav->jacobian, tmpAssign, rowInd, rowInd, 3,
                                  3);
                        V2CrossM(worldWR, tmpM2);
                        MxM(tmpM, tmpM2, tmpM3);
                        MxM(tmpM3, CRB, tmpM);
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
      } else {
         printf("Orbit types other than CENTRAL are still in development for "
                "filtering. Exiting...\n");
         exit(EXIT_FAILURE);
      }
   } else {
      printf("For the moment, can only filter rotation matrix, position, "
             "velocity, & angular "
             "velocity *simultaneously* with the LIEKF. Exiting...\n");
      exit(EXIT_FAILURE);
   }
}

void LIEKFUpdateLaw(struct DSMNavType *const Nav) {
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
      } else if (j == OMEGA_STATE) {
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

void eomMEKFJacobianFun(struct SCType *const S, const struct DateType *date,
                        double const CRB[3][3], double const qbr[4],
                        double const PosR[3], double const VelR[3],
                        double const wbr[3], double const whlH[S->AC.Nwhl],
                        const double AtmoDensity) {
   double tmpM[3][3] = {{0.0}}, tmpM2[3][3] = {{0.0}}, tmpM3[3][3] = {{0.0}},
          tmpV[3] = {0.0}, tmpV2[3] = {0.0}, tmpV3[3] = {0.0};
   static double **tmpAssign = NULL;
   double wrnd[3]            = {0.0};
   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   long i, j, rowInd;
   enum navState state;

   AC  = &S->AC;
   DSM = &S->DSM;
   Nav = &DSM->DsmNav;

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
                  MxM(tmpM2, AC->MOI, tmpM3);
                  MxV(AC->MOI, tmpV3, tmpV2);
                  for (long Iw = 0; Iw < AC->Nwhl; Iw++) {
                     for (i = 0; i < 3; i++)
                        tmpV2[i] += AC->Whl[Iw].Axis[i] * whlH[Iw];
                  }
                  V2CrossM(tmpV2, tmpM);
                  for (i = 0; i < 3; i++)
                     for (j = 0; j < 3; j++)
                        tmpM[i][j] -= tmpM3[i][j];
                  MINVxM3(AC->MOI, 3, tmpM, tmpM2);
                  // MINV3(AC->MOI, tmpM3);
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
                     tmpV[i] = Nav->forceB[i] / AC->mass;
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
      if (Orb[S->RefOrb].Regime == ORB_CENTRAL) {
         long orbCenter             = Orb[S->RefOrb].World;
         double dAeroFrcdVRel[3][3] = {{0.0}}, dAeroTrqdVRel[3][3] = {{0.0}};
         double worldWR[3] = {0.0};
         if (AeroActive) {
            double VrelR[3]     = {0.0};
            double PosRWorld[3] = {0.0};
            for (i = 0; i < 3; i++)
               PosRWorld[i] = PosR[i] + Nav->refPos[i];
            for (i = 0; i < 3; i++)
               worldWR[i] = -Nav->refCRN[i][2] * World[orbCenter].w;
            VxV(worldWR, PosRWorld, VrelR);
            for (i = 0; i < 3; i++)
               VrelR[i] += VelR[i] + Nav->refVel[i];
            getDAeroFrcAndTrqDVRel(S, CRB, VrelR, dAeroFrcdVRel, dAeroTrqdVRel,
                                   AtmoDensity);
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
                        NavDGravPertAccelDPos(Nav, date, tmpV2, S->RefOrb,
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
                           for (j = 0; j < 3; j++) {
                              dAeroFrcdVRel[i][j] /= AC->mass;
                              tmpAssign[i][j]      = dAeroFrcdVRel[i][j];
                           }
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
      } else {
         printf("Orbit types other than CENTRAL are still in development for "
                "filtering. Exiting...\n");
         exit(EXIT_FAILURE);
      }
   } else {
      printf("For the moment, can only filter quaternion, position, velocity, "
             "& angular velocity "
             "*simulataneously* with the MEKF. Exiting...\n");
      exit(EXIT_FAILURE);
   }
}

void MEKFUpdateLaw(struct DSMNavType *const Nav) {
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
double **GetStateLinTForm(struct SCType *const S) {
   double **tForm, tmpM[3][3] = {{0.0}};
   static double **tmpAssign = NULL;
   struct DSMType *DSM;
   struct DSMNavType *Nav;
   long i, j;

   DSM = &S->DSM;
   Nav = &DSM->DsmNav;

   tForm = CreateMatrix(Nav->navDim, Nav->navDim);
   if (tmpAssign == NULL)
      tmpAssign = CreateMatrix(3, 3);

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
         printf("Navigation active with undefined or ideal navigation type. "
                "Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   return (tForm);
}

void configureRefFrame(struct SCType *const S, const double dLerpAlpha,
                       const long reset) {
   // set up reference frame. Not a fan of effectively using truth data for it
   long i, j;
   double targetPosN[3] = {0.0};
   double targetVelN[3] = {0.0};

   struct DSMType *DSM;
   struct DSMNavType *Nav;

   DSM = &S->DSM;
   Nav = &DSM->DsmNav;
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
      case ORI_WORLD:
         if (Nav->refOri != Orb[S->RefOrb].World) {
            printf("Navigation reference world %ld is not equal to the central "
                   "body of the SC's "
                   "orbit, %ld. Exiting...\n",
                   Nav->refOri, Orb[S->RefOrb].World);
            exit(EXIT_FAILURE);
         }
         for (i = 0; i < 3; i++)
            Nav->refPos[i] = 0.0;
         break;
      case ORI_OP: {
         struct OrbitType *O = &Orb[S->RefOrb];
         for (i = 0; i < 3; i++) {
            targetVelN[i] = O->VelN[i];
            targetPosN[i] = O->PosN[i];
         }
      } break;
      default: {
         // make sure if you do sc relative nav, you initialize that sc's nav
         // before you start this sc's nav
         struct AcType *acTarget = &SC[Nav->refOriType].AC;
         for (i = 0; i < 3; i++) {
            targetVelN[i] =
                acTarget->VelN[i] + SC[Nav->refOriType].B[Nav->refOri].vn[i];
            targetPosN[i] =
                acTarget->PosN[i] + SC[Nav->refOriType].B[Nav->refOri].pn[i];
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
         printf("Unknown Navigation Reference Frame. Exiting...\n");
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
      double dt = (1.0 - Nav->refLerpAlpha) * Nav->DT;
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
          double CRB[3][3], double qbr[4], double PosR[3], double VelR[3],
          double wbr[3]) {
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
         printf("Navigation active with undefined or ideal navigation type. "
                "Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }
}

void getForceAndTorque(struct AcType *const AC, struct DSMNavType *const Nav,
                       double const CRB[3][3], double const *whlH) {
   long i, j;

   for (i = 0; i < 3; i++) {
      Nav->forceB[i]  = AC->IdealFrc[i];
      Nav->torqueB[i] = AC->IdealTrq[i];
   }
   for (j = 0; j < AC->Nthr; j++) {
      struct AcThrType *thr = &AC->Thr[j];
      if (thr->ThrustLevelCmd > 0.0) {
         double appliedForce = thr->ThrustLevelCmd * thr->Fmax;
         for (i = 0; i < 3; i++) {
            Nav->forceB[i]  += thr->Axis[i] * appliedForce;
            Nav->torqueB[i] += thr->rxA[i] * appliedForce;
         }
      }
   }

   for (j = 0; j < AC->Nwhl; j++) {
      struct AcWhlType *whl = &AC->Whl[j];
      if ((whlH[j] * signum(whl->Tcmd)) < whl->Hmax)
         for (i = 0; i < 3; i++)
            Nav->torqueB[i] -= whl->Axis[i] * whl->Tcmd;
   }
   double bvb[3] = {0.0}, CBN[3][3] = {{0.0}};
   MTxM(CRB, Nav->refCRN, CBN);
   MxV(CBN, AC->bvn, bvb);
   for (j = 0; j < AC->Nmtb; j++) {
      struct AcMtbType *mtb = &AC->MTB[j];
      double AxBvb[3]       = {0.0};
      VxV(mtb->Axis, bvb, AxBvb);
      for (i = 0; i < 3; i++)
         Nav->torqueB[i] += AxBvb[i] * mtb->Mcmd;
   }
}

void NavEOMs(struct SCType *const S, const struct DateType *date,
             double const CRB[3][3], double const qbr[4], double const PosR[3],
             double const VelR[3], double const wbr[3], double const *whlH,
             double CRBdot[3][3], double qbrdot[4], double PosRdot[3],
             double VelRdot[3], double wbrdot[3], double *whlHdot,
             const long refOrb, const double AtmoDensity) {
   long i, j, iState;

   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;

   AC  = &S->AC;
   DSM = &S->DSM;
   Nav = &DSM->DsmNav;

   double aeroFrc[3] = {0.0}, aeroTrq[3] = {0.0};
   long orbCenter = Orb[refOrb].World;
   if (AeroActive && Orb[refOrb].Regime == ORB_CENTRAL) {
      double worldWR[3] = {0.0}, VrelR[3] = {0.0};
      double PosRWorld[3] = {0.0};
      for (i = 0; i < 3; i++)
         PosRWorld[i] = PosR[i] + Nav->refPos[i];
      for (i = 0; i < 3; i++)
         worldWR[i] = -Nav->refCRN[i][2] * World[orbCenter].w;
      VxV(worldWR, PosRWorld, VrelR);
      for (i = 0; i < 3; i++)
         VrelR[i] += VelR[i] + Nav->refVel[i];
      getAeroForceAndTorque(S, CRB, VrelR, aeroFrc, aeroTrq, AtmoDensity);
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
               MxV(AC->MOI, wbn, Hb);

               for (long Iw = 0; Iw < AC->Nwhl; Iw++)
                  for (i = 0; i < 3; i++)
                     Hb[i] += AC->Whl[Iw].Axis[i] * whlH[Iw];
               VxV(Hb, wbn, tmpV);

               for (i = 0; i < 3; i++) {
                  tmpV[i] += Nav->torqueB[i];
                  if (AeroActive && Orb[refOrb].Regime == ORB_CENTRAL)
                     tmpV[i] += aeroTrq[i];
               }

               pM3x3 = CreateMatrix(3, 3);
               for (i = 0; i < 3; i++)
                  for (j = 0; j < 3; j++)
                     pM3x3[i][j] = AC->MOI[i][j];
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
                  VelRdot[i] /= AC->mass;
               if (Nav->refFrame != FRAME_N) {
                  printf("Frame types other than Inertial are still in "
                         "development for filtering. "
                         "Exiting...\n");
                  exit(EXIT_FAILURE);
               }

               if (GravPertActive) {
                  double accelR[3] = {0.0};
                  for (i = 0; i < 3; i++)
                     tmpV[i] = PosR[i] + Nav->refPos[i];
                  NavGravPertAccel(Nav, date, tmpV, 1.0, refOrb, accelR);
                  for (i = 0; i < 3; i++)
                     VelRdot[i] += accelR[i];
               }

               switch (Orb[refOrb].Regime) {
                  case ORB_CENTRAL: {
                     for (i = 0; i < 3; i++)
                        tmpV[i] = PosR[i] + Nav->refPos[i];
                     getGravAccel(Orb[S->RefOrb].mu, tmpV, tmpV2);
                     for (i = 0; i < 3; i++)
                        VelRdot[i] += tmpV2[i];
                     if (Nav->refOriType != ORI_WORLD) {
                        for (i = 0; i < 3; i++)
                           VelRdot[i] -= Nav->refAccel[i];
                     }

                     if (AeroActive)
                        for (i = 0; i < 3; i++)
                           VelRdot[i] += aeroFrc[i] / AC->mass;
                     break;
                  }
                  default:
                     printf("Orbit types other than CENTRAL are still in "
                            "development for "
                            "filtering. Exiting...\n");
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

void updateNavTime(struct DateType *Time, const double dSeconds) {
   if (fabs(dSeconds) > 0.0) {
      Time->Second  += dSeconds;
      long quotient  = Time->Second / 60.0;
      Time->Second   = fmod(Time->Second, 60.0);
      if (Time->Second < 0) {
         Time->Second += 60;
         quotient--;
      }

      if (quotient != 0.0) {
         Time->Minute += quotient;
         quotient      = Time->Minute / 60;
         Time->Minute %= 60;
         if (Time->Minute < 0) {
            Time->Minute += 60;
            quotient--;
         }

         if (quotient != 0) {
            Time->Hour += quotient;
            quotient    = Time->Hour / 24;
            Time->Hour %= 24;
            if (Time->Hour < 0) {
               Time->Hour += 24;
               quotient--;
            }

            if (quotient != 0) {
               Time->JulDay =
                   floor(Time->JulDay + 0.5 + (double)quotient) - 0.5;
               long tmpHr = 0, tmpMin = 0;
               double tmpSec = 0.0;
               JDToDate(Time->JulDay, &Time->Year, &Time->Month, &Time->Day,
                        &tmpHr, &tmpMin, &tmpSec);
               Time->doy = MD2DOY(Time->Year, Time->Month, Time->Day);
            }
         }
      }
      Time->JulDay = DateToJD(Time->Year, Time->Month, Time->Day, Time->Hour,
                              Time->Minute, Time->Second);
   }
}

void PropagateNav(struct SCType *const S, const long dSubStep,
                  const long dStep) {
   double lerpAlphaState, AtmoDensity = 0.0;

   long i, j, k;
   enum navState Istate;

   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;

   AC              = &S->AC;
   DSM             = &S->DSM;
   Nav             = &DSM->DsmNav;
   lerpAlphaState  = Nav->refLerpAlpha;
   const double DT = (dSubStep * Nav->subStepSize) + (dStep * Nav->DT);

   GetM(AC, Nav, Nav->CRB, Nav->qbr, Nav->PosR, Nav->VelR, Nav->wbr);
   if (Nav->subStep == 0) {
      if (AeroActive) {
         long orbCenter    = Orb[S->RefOrb].World;
         double worldWR[3] = {0.0};
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
            double PriMerAng = GetPriMerAng(orbCenter, &Nav->Date);
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
            } else
               AtmoDensity = 0.0;
         } else if (orbCenter == MARS) {
            AtmoDensity = MarsAtmosphereModel(PosN);
         } else
            AtmoDensity = 0.0;
      }
      (*Nav->EOMJacobianFun)(S, &Nav->Date, Nav->CRB, Nav->qbr, Nav->PosR,
                             Nav->VelR, Nav->wbr, Nav->whlH, AtmoDensity);
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
      updateNavTime(&date, Nav->subStep * Nav->subStepSize + DTk[k]);
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
      } else {
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
      configureRefFrame(S, dLerpAlpha, FALSE);
      getForceAndTorque(AC, Nav, CRB, whlH);
      NavEOMs(S, &date, CRB, qbr, PosR, VelR, wbr, whlH, CRBk[k], qbrk[k],
              PosRk[k], VelRk[k], wbrk[k], whlHk[k], S->RefOrb, AtmoDensity);
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

   double **STM = CreateMatrix(Nav->navDim, Nav->navDim);
   for (i = 0; i < Nav->navDim; i++) {
      for (j = 0; j < Nav->navDim; j++) {
         STM[i][j]      = Nav->STM[i][j];
         Nav->NxN[i][j] = Nav->M[i][j] * Nav->sqrQ[j];
      }
   }
   long subSteps = dSubStep + dStep * Nav->subStepMax;
   double **A    = CreateMatrix(Nav->navDim, Nav->navDim);
   for (k = 1; k < subSteps; k++) {
      // Swap the pointers; should be faster than assigning elements of A
      double **t = A;
      A          = STM;
      STM        = t;
      MxMG(Nav->STM, A, STM, Nav->navDim, Nav->navDim, Nav->navDim);
   }
   DestroyMatrix(A);

   MxMG(STM, Nav->NxN, Nav->NxN2, Nav->navDim, Nav->navDim, Nav->navDim);
   MxMG(STM, Nav->S, Nav->NxN, Nav->navDim, Nav->navDim, Nav->navDim);
   DestroyMatrix(STM);
   double **tmp = CreateMatrix(Nav->navDim + Nav->navDim, Nav->navDim);
   double **U   = CreateMatrix(Nav->navDim + Nav->navDim, Nav->navDim);
   double sqrDT = sqrt(DT);
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
   configureRefFrame(S, DT / Nav->DT, FALSE);
}

void KalmanFilt(struct SCType *const S) {
   long i, j;

   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMNavType *Nav;

   AC  = &S->AC;
   DSM = &S->DSM;
   Nav = &DSM->DsmNav;

   // TODO: will maybe need to do something to preserve information if a new Nav
   // filter is called
   if (Nav->Init == FALSE)
      configureRefFrame(S, 0.0, TRUE);

   // Accumulate information from measurements based upon batching method.
   Nav->subStep                     = 0;
   struct DSMMeasListType *measList = &Nav->measList;
   if (measList->head == NULL) {
      // TODO: problems might occur if Nav->DT is not a whole number multiple of
      // Nav->subStepSize
      PropagateNav(S, Nav->subStepMax, 0.0);
      Nav->step++;
      Nav->subStep = 0;
   } else {
      while (measList->head != NULL) {
         long measDim              = 0;
         enum sensorType senseType = measList->head->type;
         long measStep             = measList->head->step;
         long measSubStep          = measList->head->subStep;

         // TODO: avoid this preallocation mess and go with realloc (maybe?)
         switch (Nav->batching) {
            case NONE_BATCH:
               measDim = measList->head->errDim;
               break;
            case SENSOR_BATCH: {
               struct DSMMeasType *meas = measList->head;
               while (meas != NULL && meas->type == senseType &&
                      meas->subStep == measSubStep) {
                  measDim += meas->errDim;
                  meas     = meas->nextMeas;
               }
            } break;
            case TIME_BATCH: {
               struct DSMMeasType *meas = measList->head;
               while (meas != NULL && meas->subStep == measSubStep) {
                  measDim += meas->errDim;
                  meas     = meas->nextMeas;
               }
            } break;
            default:
               printf("Invalid Batching method. If you are reading this, the "
                      "developer probably "
                      "has a messed up pointer. Exiting...\n");
               exit(EXIT_FAILURE);
               break;
         }

         long dSubStep = measSubStep - Nav->subStep;
         long dStep    = measStep - Nav->step;
         if ((dSubStep > 0 && dStep >= 0) || (dSubStep >= 0 && dStep > 0)) {
#if REPORT_RESIDUALS ==                                                        \
    TRUE // TODO: set a report residuals "bool" in nav and use that instead of
         // these compile-time directives
            DSM_NAV_ResidualsReport(Nav->Time + (double)(Nav->subStep * DTSIM),
                                    Nav->residuals);
#endif
            // TODO: investigate only prop once per Kalman filt call and use STM
            // and linearization to prop measurements through time
            PropagateNav(S, dSubStep, dStep);
            Nav->step    = measStep;
            Nav->subStep = measSubStep;
#if REPORT_RESIDUALS == TRUE
            for (enum sensorType sensor = INIT_SENSOR; sensor < FIN_SENSOR;
                 sensor++) {
               if (Nav->sensorActive[sensor] == TRUE) {
                  for (i = 0; i < Nav->nSensor[sensor]; i++) {
                     free(Nav->residuals[sensor][i]);
                     Nav->residuals[sensor][i] = NULL;
                  }
               }
            }
#endif
         } else if (dSubStep < 0 || dStep < 0.0) {
            printf("Attempted to propagate Navigation state backwards in time. "
                   "How did that "
                   "happen? Exiting...\n");
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
            measEstData  = (*meas->measFun)(S, meas->sensorNum);
            measJacobian = (*meas->measJacobianFun)(S, meas->sensorNum);

            if (meas->type == STARTRACK_SENSOR) {
               double tmpq[4];
               QxQT(meas->data, measEstData, tmpq);
               Q2AngleVec(tmpq, resid);
            } else {
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
                (measList->head->subStep != measSubStep) || // TIME_BATCH
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
         } else {
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
               printf("Cholesky Downdate failed! Exiting...\n");
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
      if (Nav->subStep < Nav->subStepMax)
         PropagateNav(S, Nav->subStepMax - Nav->subStep, 0.0);

      Nav->step++;
      Nav->subStep = 0;
   }
   Nav->Date = Nav->Date0;
   updateNavTime(&Nav->Date, Nav->step * Nav->DT);
   configureRefFrame(S, 1.0 - Nav->refLerpAlpha, TRUE);
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
               long const n, long const m) {
   long i, j, curRow;
   for (i = 0; i < n; i++) {
      curRow = i + iN;
      for (j = 0; j < m; j++)
         A[curRow][j + iM] += B[i][j];
   }
}

double mahalonobis2(double **A, double *x, long const n) {
   long i, j;

   double d = 0.0, xTA[n];
   for (j = 0; j < n; j++) {
      xTA[j] = 0.0;
      for (i = 0; i < n; i++)
         xTA[j] += x[i] * A[i][j];
      d += x[j] * xTA[j];
   }
   return d;
}

// yes, this function is here just for pGate = 0.9999...
double chi2InvLookup(double const pGate, long const dim) {
   long const nDeg = 24, nPGate = 9;
   // tbl generated from MATLAB's chi2inv
   static const double tbl[24][9] = {{
                                         1.3233036969314664,
                                         2.705543454095404,
                                         3.841458820694124,
                                         6.6348966010212145,
                                         7.879438576622417,
                                         9.14059346124402,
                                         10.827566170662733,
                                         12.11566514639738,
                                         15.136705226623606,
                                     },
                                     {
                                         2.772588722239781,
                                         4.605170185988092,
                                         5.991464547107979,
                                         9.21034037197618,
                                         10.596634733096073,
                                         11.982929094216006,
                                         13.815510557964274,
                                         15.201804919084385,
                                         18.420680743952584,
                                     },
                                     {
                                         4.108344935632312,
                                         6.251388631170325,
                                         7.814727903251179,
                                         11.344866730144373,
                                         12.838156466598647,
                                         14.320347097873526,
                                         16.26623619623813,
                                         17.72999622894616,
                                         21.107513466160444,
                                     },
                                     {
                                         5.38526905777939,
                                         7.779440339734858,
                                         9.487729036781154,
                                         13.276704135987622,
                                         14.860259000560243,
                                         16.423936124136556,
                                         18.46682695290317,
                                         19.99735499524785,
                                         23.512742444991076,
                                     },
                                     {
                                         6.625679763829247,
                                         9.236356899781123,
                                         11.070497693516351,
                                         15.08627246938899,
                                         16.74960234363904,
                                         18.385612555684343,
                                         20.515005652432873,
                                         22.105326778207612,
                                         25.74483195905612,
                                     },
                                     {
                                         7.840804120585122,
                                         10.644640675668422,
                                         12.591587243743977,
                                         16.811893829770927,
                                         18.547584178511087,
                                         20.249402051490126,
                                         22.457744484825323,
                                         24.102798994983747,
                                         27.85634123601417,
                                     },
                                     {
                                         9.037147547908143,
                                         12.017036623780532,
                                         14.067140449340169,
                                         18.475306906582357,
                                         20.27773987496262,
                                         22.04039058924537,
                                         24.321886347856854,
                                         26.01776770901503,
                                         29.87750390922517,
                                     },
                                     {
                                         10.218854970246761,
                                         13.36156613651173,
                                         15.50731305586545,
                                         20.090235029663233,
                                         21.95495499065953,
                                         23.774474318294196,
                                         26.12448155837614,
                                         27.86804640338262,
                                         31.827628001262585,
                                     },
                                     {
                                         11.388751440470372,
                                         14.683656573259837,
                                         16.918977604620448,
                                         21.665994333461924,
                                         23.589350781257387,
                                         25.462478697854408,
                                         27.877164871256568,
                                         29.66580810359642,
                                         33.719948438964906,
                                     },
                                     {
                                         12.548861396889377,
                                         15.987179172105265,
                                         18.307038053275146,
                                         23.209251158954356,
                                         25.18817957197117,
                                         27.112171033510684,
                                         29.58829844507442,
                                         31.41981250740049,
                                         35.564013941952396,
                                     },
                                     {
                                         13.700692746011514,
                                         17.275008517500073,
                                         19.67513757268249,
                                         24.724970311318277,
                                         26.756848916469636,
                                         28.729349519951263,
                                         31.264133620239985,
                                         33.136615004168554,
                                         37.366986437997284,
                                     },
                                     {
                                         14.845403671040176,
                                         18.54934778670325,
                                         21.02606981748307,
                                         26.216967305535853,
                                         28.299518822046025,
                                         30.31847913057539,
                                         32.90949040736021,
                                         34.821274636474655,
                                         39.13440388194981,
                                     },
                                     {
                                         15.983906216312052,
                                         19.81192930712756,
                                         22.362032494826934,
                                         27.68824961045705,
                                         29.819471223653217,
                                         31.883085473134148,
                                         34.52817897487089,
                                         36.47779371889615,
                                         40.8706550138363,
                                     },
                                     {
                                         17.116933596000067,
                                         21.064144212997064,
                                         23.684791304840576,
                                         29.141237740672796,
                                         31.31934962259528,
                                         33.426010512669464,
                                         36.12327368039813,
                                         38.10940393227008,
                                         42.5792889531133,
                                     },
                                     {
                                         18.245085602415134,
                                         22.307129581578693,
                                         24.995790139728616,
                                         30.57791416689249,
                                         32.80132064579183,
                                         34.94958513964062,
                                         37.69729821835383,
                                         39.71875978963228,
                                         44.26322494417528,
                                     },
                                     {
                                         19.368860220584512,
                                         23.541828923096105,
                                         26.29622760486423,
                                         31.999926908815176,
                                         34.26718653782669,
                                         36.45574943193811,
                                         39.252354790768464,
                                         41.30807371713764,
                                         45.92489905111385,
                                     },
                                     {
                                         20.488676238391502,
                                         24.76903534390146,
                                         27.58711163827534,
                                         33.40866360500461,
                                         35.71846565900461,
                                         37.94613878137666,
                                         40.79021670690253,
                                         42.87921296033667,
                                         47.566369558144736,
                                     },
                                     {
                                         21.604889795728162,
                                         25.98942308263721,
                                         28.869299430392623,
                                         34.805305734705065,
                                         37.15645145660674,
                                         39.422147034103354,
                                         42.31239633167996,
                                         44.433770739806334,
                                         49.18939447196683,
                                     },
                                     {
                                         22.717806744199855,
                                         27.203571029356844,
                                         30.14352720564616,
                                         36.19086912927004,
                                         38.58225655493424,
                                         40.8849737292633,
                                         43.82019596451753,
                                         45.97311956390125,
                                         50.79548966562221,
                                     },
                                     {
                                         23.827692043030858,
                                         28.41198058430563,
                                         31.410432844230918,
                                         37.56623478662507,
                                         39.99684631293865,
                                         42.33566007525024,
                                         45.31474661812586,
                                         47.49845188547201,
                                         52.3859732730525,
                                     },
                                     {
                                         24.934777014902316,
                                         29.61508943618274,
                                         32.670573340917315,
                                         38.93217268351607,
                                         41.40106477141761,
                                         43.77511678286907,
                                         46.797038041561315,
                                         49.010811595211635,
                                         53.96200011641196,
                                     },
                                     {
                                         26.03926502816501,
                                         30.813282343953027,
                                         33.92443847144381,
                                         40.289360437593864,
                                         42.795654999308546,
                                         45.2041459020426,
                                         48.26794229083518,
                                         50.51111875853229,
                                         55.5245887757052,
                                     },
                                     {
                                         27.141336002976505,
                                         32.006899681704304,
                                         35.17246162690806,
                                         41.638398118858476,
                                         44.18127524997109,
                                         46.623458170116734,
                                         49.7282324664315,
                                         52.00018928907863,
                                         57.07464313855563,
                                     },
                                     {
                                         28.241150025528764,
                                         33.19624428862818,
                                         36.41502850180731,
                                         42.97982013935165,
                                         45.558511936530586,
                                         48.03368695093583,
                                         51.17859777737739,
                                         53.47875077195909,
                                         58.61296974830205,
                                     }};
   const double probGate[9]       = {0.75,   0.90,  0.95,   0.99,  0.995,
                                     0.9975, 0.999, 0.9995, 0.9999};

   if (dim < 1 || dim > nDeg) {
      printf("Dimension for chi2InvLookup() must be between 1 and %li, "
             "inclusive. Exiting...\n",
             nDeg);
      exit(EXIT_FAILURE);
   }
   if (pGate < probGate[0] || pGate > probGate[nPGate - 1]) {
      printf("Probability Gate for chi2InvLookup() must be between %lf and "
             "%lf, inclusive. "
             "Exiting...\n",
             probGate[0], probGate[nPGate - 1]);
      exit(EXIT_FAILURE);
   }

   double out = 0.0;
   if (pGate == probGate[nPGate - 1]) {
      out = tbl[dim - 1][nPGate - 1];
   } else if (pGate == probGate[0]) {
      out = tbl[dim - 1][0];
   } else {
      long lowPGateInd;
      for (lowPGateInd = nPGate - 1; lowPGateInd >= 0; lowPGateInd--) {
         if (probGate[lowPGateInd] <= pGate) {
            break;
         }
      }
      out = (tbl[dim - 1][lowPGateInd + 1] - tbl[dim - 1][lowPGateInd + 0]) /
                (probGate[lowPGateInd + 1] - probGate[lowPGateInd + 0]) *
                (pGate - probGate[lowPGateInd + 0]) +
            tbl[dim - 1][lowPGateInd + 0];
   }
   return (out);
}
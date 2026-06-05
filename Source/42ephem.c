/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight World   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "42.h"

/* #ifdef __cplusplus
** namespace _42 {
** using namespace Kit;
** #endif
*/

/**********************************************************************/
void AssignScToOrbit(struct SCType *S, long Iorb)
{
   struct OrbitType *OldOrb, *NewOrb;
   long i;

   if (Iorb < 0 || Iorb >= Norb) {
      fprintf(stderr, "Desired orbit is out of bounds.\n");
      exit(EXIT_FAILURE);
   }

   OldOrb         = &Orb[S->RefOrb];
   NewOrb         = &Orb[Iorb];
   NewOrb->Exists = TRUE;

   if (NewOrb->World != OldOrb->World) {
      fprintf(stderr,
              "New Orbit and Old Orbit must (for the present) be in same "
              "gravitational system.\n");
      exit(EXIT_FAILURE);
   }

   /* .. Update PosR, VelR, PosEH, VelEH */
   for (i = 0; i < 3; i++) {
      S->PosR[i] = S->PosN[i] - NewOrb->PosN[i];
      S->VelR[i] = S->PosN[i] - NewOrb->PosN[i];
   }
   RelRV2EHRV(MAGV(NewOrb->PosN), MAGV(NewOrb->wln), NewOrb->CLN, S->PosR,
              S->VelR, S->PosEH, S->VelEH);

   /* .. Update RefOrb tag */
   S->RefOrb = Iorb;
}
/**********************************************************************/
void FindSCinFormation(struct SCType *S)
{

   double psn[3], pcmn[3];
   double wxr[3], wxrn[3], vsn[3];
   long j;
   struct FormationType *F;

   F = &Frm[S->RefOrb];

   /* .. Find CSF */
   MxMT(S->B[0].CN, F->CN, S->CF);

   if (S->OrbDOF) {
      /* Find PosF */
      MTxV(S->B[0].CN, S->cm, pcmn);
      for (j = 0; j < 3; j++) {
         psn[j] = S->PosR[j] - F->PosR[j] - pcmn[j];
      }
      MxV(F->CN, psn, S->PosF);

      /* Find VelF */
      VxV(S->B[0].wn, S->cm, wxr);
      MTxV(S->B[0].CN, wxr, wxrn);
      for (j = 0; j < 3; j++) {
         vsn[j] = S->VelR[j] - wxrn[j];
      }
      MxV(F->CN, vsn, S->VelF);
   }
#if 0
      else {
         /* Find PosR */
         MTxV(F->CN,S->PosF,psn);
         MTxV(S->B[0].CN,S->cm,pcmn);
         for(j=0;j<3;j++)
            S->PosR[j] = psn[j] + F->PosR[j] + pcmn[j];
         /* Find VelR */
         MTxV(F->CN,S->VelF,vsn);
         VxV(S->B[0].wn,S->cm,wxr);
         MTxV(S->B[0].CN,wxr,wxrn);
         for(j=0;j<3;j++) S->VelR[j] = vsn[j] + wxrn[j];
      }
#endif
}
/**********************************************************************/
void CheckOrbitRectification(struct SCType *scs, struct OrbitType *O)
{
   long Isc, i;
   double m     = 0.0;
   double mr[3] = {0.0, 0.0, 0.0};
   double mv[3] = {0.0, 0.0, 0.0};
   double PosR[3], VelR[3];
   double a, n;
   struct SCType *S;

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (scs[Isc].Exists && scs[Isc].RefOrb == O->Tag) {
         m += scs[Isc].mass;
         for (i = 0; i < 3; i++) {
            mr[i] += scs[Isc].mass * scs[Isc].PosR[i];
            mv[i] += scs[Isc].mass * scs[Isc].VelR[i];
         }
      }
   }
   for (i = 0; i < 3; i++) {
      PosR[i] = mr[i] / m;
      VelR[i] = mv[i] / m;
   }
   if (MAGV(PosR) > 50.0E3) {   /* Visualization gets jittery at about 50 km */
      for (i = 0; i < 3; i++) { /* due to SC.PosR-POV.rr being difference of */
         O->PosN[i] += PosR[i]; /* large quantities */
         O->VelN[i] += VelR[i];
      }
      if (O->Regime == ORB_CENTRAL)
         RV2Eph(DynTime, O->mu, O->PosN, O->VelN, &O->SMA, &O->ecc, &O->inc,
                &O->RAAN, &O->ArgP, &O->anom, &O->tp, &O->SLR, &O->alpha,
                &O->rmin, &O->MeanMotion, &O->Period);
      FindCLN(O->PosN, O->VelN, O->CLN, O->wln);
      a = MAGV(O->PosN);
      n = sqrt(O->mu / (a * a * a));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (scs[Isc].Exists && scs[Isc].RefOrb == O->Tag) {
            S = &scs[Isc];
            for (i = 0; i < 3; i++) {
               S->PosR[i] -= PosR[i];
               S->VelR[i] -= VelR[i];
            }
            RelRV2EHRV(a, n, O->CLN, S->PosR, S->VelR, S->PosEH, S->VelEH);
         }
      }
      printf("Orb[%ld] rectified at Time = %12.3f\n", O->Tag, SimTime);
   }
}
/**********************************************************************/
/*  The N frame is the reference frame to which most dynamical        */
/*  variables are referenced.  Changing orbit Worlds changes the N   */
/*  frame being used.  To avoid discontinuities in actual positions   */
/*  or attitudes, several dynamical variables must be adjusted.       */
void ChangeNFrame(struct SCType *const scs, struct WorldType *const worlds,
                  struct OrbitType *O, long OldWorld, long NewWorld)
{
   double CN1H[3][3], CN2H[3][3], CL1H[3][3], CL2H[3][3], CH[3][3], VH[3];
   double CBN1[3][3], CBN2[3][3];
   struct FormationType *F;
   struct SCType *S;
   struct BodyType *B;
   struct DynType *D;
   long Isc, Ib, i, j;

   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         CN1H[i][j] = worlds[OldWorld].CNH[i][j];
         CN2H[i][j] = worlds[NewWorld].CNH[i][j];
      }
   }

   /* .. Orb */
   MxM(O->CLN, CN1H, CL1H);
   FindCLN(O->PosN, O->VelN, O->CLN, O->wln);
   MxM(O->CLN, CN2H, CL2H);
   /* Update Formation Frame */
   F = &Frm[O->Tag];
   if (F->FixedInFrame == 'L') {
      MxM(F->CL, CL1H, CH);
      MxMT(CH, CL2H, F->CL);
      MxM(F->CL, O->CLN, F->CN);
   }
   else {
      MxM(F->CN, CN1H, CH);
      MxMT(CH, CN2H, F->CN);
      MxMT(F->CN, O->CLN, F->CL);
   }

   /* .. SC */
   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &scs[Isc];
      if (S->Exists && S->RefOrb == O->Tag) {
         MxM(S->CLN, CN1H, CL1H);

         MTxV(CN1H, S->PosR, VH);
         MxV(CN2H, VH, S->PosR);
         MTxV(CN1H, S->VelR, VH);
         MxV(CN2H, VH, S->VelR);
         for (i = 0; i < 3; i++) {
            S->PosN[i] = O->PosN[i] + S->PosR[i];
            S->VelN[i] = O->VelN[i] + S->VelR[i];
         }

         FindCLN(S->PosN, S->VelN, S->CLN, S->wln);
         MxM(S->CLN, CN2H, CL2H);
         MTxV(CL1H, S->PosEH, VH);
         MxV(CL2H, VH, S->PosEH);
         MTxV(CL1H, S->VelEH, VH);
         MxV(CL2H, VH, S->VelEH);

         /* Bodies */
         for (Ib = 0; Ib < S->Nb; Ib++) {
            B = &S->B[Ib];
            MxM(B->CN, CN1H, CH);
            MxMT(CH, CN2H, B->CN);
            C2Q(B->CN, B->qn);
            MTxV(CN1H, B->vn, VH);
            MxV(CN2H, VH, B->vn);
            MTxV(CN1H, B->pn, VH);
            MxV(CN2H, VH, B->pn);
         }
         /* Dyn */
         D = &S->Dyn;
         Q2C(&D->x[0], CBN1);
         MxM(CBN1, CN1H, CH);
         MxMT(CH, CN2H, CBN2);
         C2Q(CBN2, &D->x[0]);
         MTxV(CN1H, &D->u[D->Nu - 3], VH);
         MxV(CN2H, VH, &D->u[D->Nu - 3]);
      }
   }

   /* .. POV */
   if (POV.Host.RefOrb == O->Tag) {
      POV.Host.World = NewWorld;
      MxM(POV.CN, CN1H, CH);
      MxMT(CH, CN2H, POV.CN);
      MxMT(POV.CN, scs[POV.Host.SC].CLN, POV.CL);

      if (POV.Frame == FRAME_N) {
         for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++)
               POV.C[i][j] = POV.CN[i][j];
         }
      }
      else if (POV.Frame == FRAME_L) {
         for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++)
               POV.C[i][j] = POV.CL[i][j];
         }
      }
      else if (POV.Frame == FRAME_F) {
         /* Still needs work */
         for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++)
               POV.C[i][j] = POV.CF[i][j];
         }
      }
      else if (POV.Frame == FRAME_S || POV.Frame == FRAME_B) {
         for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++)
               POV.C[i][j] = POV.CB[i][j];
         }
      }
      C2Q(POV.C, POV.q);
   }
}
/**********************************************************************/
void CheckChangeOfOrbitWorld(struct SCType *const scs,
                             struct WorldType *const worlds,
                             struct OrbitType *O)
{
#define NO_TRANSITION         0
#define CENTRAL1_TO_THREEBODY 1
#define CENTRAL2_TO_THREEBODY 2
#define THREEBODY_TO_CENTRAL1 3
#define THREEBODY_TO_CENTRAL2 4

   long i, Im, Iw;
   double dr[3], rh[3], vh[3];
   struct WorldType *P;
   long Transition = NO_TRANSITION;
   long Body1 = 0, Body2 = 1;

   if (O->Regime == ORB_CENTRAL) {
      /* Falling "in" from Body 1-centered to 3-body */
      P = &worlds[O->World];
      for (Im = 0; Im < P->Nsat; Im++) {
         Iw = P->Sat[Im];
         for (i = 0; i < 3; i++)
            dr[i] = O->PosN[i] - worlds[Iw].eph.PosN[i];
         if (MAGV(dr) < 1.99 * worlds[Iw].RadOfInfluence) {
            Transition = CENTRAL1_TO_THREEBODY;
            Body1      = O->World;
            Body2      = Iw;
         }
      }
      if (MAGV(O->PosN) > 0.51 * worlds[O->World].RadOfInfluence) {
         /* Falling "out" from Body 2-centered to 3-body */
         Transition = CENTRAL2_TO_THREEBODY;
         Body1      = worlds[O->World].Parent;
         Body2      = O->World;
      }
   }
   else { /* ORB_THREE_BODY */
      /* Falling "in" from 3-body to Body 2-centered */
      Iw = O->Body2;
      for (i = 0; i < 3; i++)
         dr[i] = O->PosN[i] - worlds[Iw].eph.PosN[i];
      if (MAGV(dr) < 0.49 * worlds[Iw].RadOfInfluence) {
         Transition = THREEBODY_TO_CENTRAL2;
         Body1      = O->Body1;
         Body2      = O->Body2;
      }
      else {
         /* Falling "out" from 3-body to Body 1-centered */
         for (i = 0; i < 3; i++)
            dr[i] = O->PosN[i] - worlds[Iw].eph.PosN[i];
         if (MAGV(dr) > 2.01 * worlds[Iw].RadOfInfluence) {
            Transition = THREEBODY_TO_CENTRAL1;
            Body1      = O->Body1;
            Body2      = O->Body2;
         }
      }
   }

   switch (Transition) {
      case NO_TRANSITION:
         break;
      case CENTRAL1_TO_THREEBODY:
         O->Regime = ORB_THREE_BODY;
         O->Body1  = Body1;
         O->Body2  = Body2;
         O->mu1    = worlds[Body1].mu;
         O->mu2    = worlds[Body2].mu;
         printf("Orbit %ld transitioned from Central-1 to 3-body orbit at Time "
                "= %lf\n",
                O->Tag, SimTime);
         break;
      case CENTRAL2_TO_THREEBODY:
         O->Regime = ORB_THREE_BODY;
         O->Body1  = Body1;
         O->Body2  = Body2;
         O->World  = Body1;
         O->mu1    = worlds[Body1].mu;
         O->mu2    = worlds[Body2].mu;
         O->mu     = O->mu1;
         MTxV(worlds[Body2].CNH, O->PosN, rh);
         MTxV(worlds[Body2].CNH, O->VelN, vh);
         MxV(worlds[Body1].CNH, rh, O->PosN);
         MxV(worlds[Body1].CNH, vh, O->VelN);
         for (i = 0; i < 3; i++) {
            O->PosN[i] += worlds[Body2].eph.PosN[i];
            O->VelN[i] += worlds[Body2].eph.VelN[i];
         }
         RV2Eph(DynTime, O->mu, O->PosN, O->VelN, &O->SMA, &O->ecc, &O->inc,
                &O->RAAN, &O->ArgP, &O->anom, &O->tp, &O->SLR, &O->alpha,
                &O->rmin, &O->MeanMotion, &O->Period);
         printf("Orbit %ld transitioned to 3-body orbit at Time = %lf\n",
                O->Tag, SimTime);
         /* Change of N frame has far-ranging effects */
         ChangeNFrame(scs, worlds, O, Body2, Body1);
         break;
      case THREEBODY_TO_CENTRAL1:
         O->Regime = ORB_CENTRAL;
         O->World  = Body1;
         O->mu     = worlds[O->World].mu;
         printf("Orbit %ld transitioned to central orbit at Time = %lf\n",
                O->Tag, SimTime);
         break;
      case THREEBODY_TO_CENTRAL2:
         O->Regime = ORB_CENTRAL;
         O->World  = Body2;
         O->mu     = worlds[O->World].mu;
         for (i = 0; i < 3; i++) {
            O->PosN[i] -= worlds[Body2].eph.PosN[i];
            O->VelN[i] -= worlds[Body2].eph.VelN[i];
         }
         MTxV(worlds[Body1].CNH, O->PosN, rh);
         MTxV(worlds[Body1].CNH, O->VelN, vh);
         MxV(worlds[Body2].CNH, rh, O->PosN);
         MxV(worlds[Body2].CNH, vh, O->VelN);
         RV2Eph(DynTime, O->mu, O->PosN, O->VelN, &O->SMA, &O->ecc, &O->inc,
                &O->RAAN, &O->ArgP, &O->anom, &O->tp, &O->SLR, &O->alpha,
                &O->rmin, &O->MeanMotion, &O->Period);
         printf("Orbit %ld transitioned to central orbit at Time = %lf\n",
                O->Tag, SimTime);
         /* Change of N frame has far-ranging effects */
         ChangeNFrame(scs, worlds, O, Body1, Body2);
         break;
   }

#undef NO_TRANSITION
#undef CENTRAL1_TO_THREEBODY
#undef CENTRAL2_TO_THREEBODY
#undef THREEBODY_TO_CENTRAL1
#undef THREEBODY_TO_CENTRAL2
}
/**********************************************************************/
void SplineToPosVel(struct LagrangeSystemType *lagsys, struct OrbitType *O,
                    const double dyntime)
{
   DateType NodeDate;
   char newline;
   long i, j, k;
   double X[4], Y[4];
   double x[3], v[3], xn[3], vn[3];

   NodeDate.system = O->EphemSystem;

   /* .. Get nodes from O->SplineFile */
   while (dyntime > O->NodeDynTime[2]) {
      for (i = 0; i < 3; i++) {
         O->NodeDynTime[i] = O->NodeDynTime[i + 1];
         for (j = 0; j < 3; j++) {
            O->NodePos[i][j] = O->NodePos[i + 1][j];
            O->NodeVel[i][j] = O->NodeVel[i + 1][j];
         }
      }
      double sec = 0;
      fscanf(O->SplineFile,
             "%ld-%ld-%ldT%ld:%ld:%lf %lf %lf %lf %lf %lf %lf %[\n]",
             &NodeDate.Year, &NodeDate.Month, &NodeDate.Day, &NodeDate.Hour,
             &NodeDate.Minute, &sec, &O->NodePos[3][0], &O->NodePos[3][1],
             &O->NodePos[3][2], &O->NodeVel[3][0], &O->NodeVel[3][1],
             &O->NodeVel[3][2], &newline);
      NodeDate.Second   = double2rational(sec);
      O->NodeDynTime[3] = Date2TimeSystem(NodeDate, TT_TIME);
      for (j = 0; j < 3; j++) {
         O->NodePos[3][j] *= 1000.0;
         O->NodeVel[3][j] *= 1000.0;
      }
      if (feof(O->SplineFile)) {
         fprintf(stderr, "Oops.  Reached end of Spline file.\n");
         exit(EXIT_FAILURE);
      }
   }

   /* .. Interpolate Spline */
   for (k = 0; k < 4; k++)
      X[k] = O->NodeDynTime[k];
   for (j = 0; j < 3; j++) {
      for (k = 0; k < 4; k++)
         Y[k] = O->NodePos[k][j];
      x[j] = CubicSpline(dyntime, X, Y);
      for (k = 0; k < 4; k++)
         Y[k] = O->NodeVel[k][j];
      v[j] = CubicSpline(dyntime, X, Y);
   }

   if (O->Regime == ORB_CENTRAL) {
      for (j = 0; j < 3; j++) {
         O->PosN[j] = x[j];
         O->VelN[j] = v[j];
      }
      RV2Eph(O->Epoch, O->mu, O->PosN, O->VelN, &O->SMA, &O->ecc, &O->inc,
             &O->RAAN, &O->ArgP, &O->anom, &O->tp, &O->SLR, &O->alpha, &O->rmin,
             &O->MeanMotion, &O->Period);
      O->tp += SimTime;
   }
   else if (O->Regime == ORB_THREE_BODY) {
      MTxV(lagsys[O->Sys].CLN, x, xn);
      MTxV(lagsys[O->Sys].CLN, v, vn);
      for (j = 0; j < 3; j++) {
         O->PosN[j] = xn[j] + lagsys[O->Sys].LP[O->LP].PosN[j];
         O->VelN[j] = vn[j] + lagsys[O->Sys].LP[O->LP].VelN[j];
      }
   }
   else {
      fprintf(stderr, "Invalid Orbit Regime in SplineToPosVel.\n");
      exit(EXIT_FAILURE);
   }
}
/**********************************************************************/
void OrbitMotion(struct WorldType *const worlds, struct RegionType *rgn,
                 struct LagrangeSystemType *lagsys, struct OrbitType *const orb,
                 struct FormationType *const frm, JDType jd)
{
   long i, j;
   struct RegionType *R;

#if 0
      static long RectCtr = 0;
      RectCtr++;
      if (RectCtr > 100) {
         RectCtr = 0;
         for(Iorb=0;Iorb<Norb;Iorb++) {
            if (Orb[Iorb].Exists) {
               CheckOrbitRectification(&Orb[Iorb]);
               CheckChangeOfOrbitWorld(&Orb[Iorb]);
            }
         }
      }
#endif
   const double dyntime = JDToDynTime(jd);

   if (orb->Exists) {
      if (orb->Regime == ORB_THREE_BODY) {
         if (orb->LagDOF == LAGDOF_MODES) {
            LagModes2RV(dyntime, &lagsys[orb->Sys], orb, orb->PosN, orb->VelN);
         }
         else if (orb->LagDOF == LAGDOF_COWELL) {
            ThreeBodyOrbitRK4(worlds, orb);
            RV2LagModes(dyntime, &lagsys[orb->Sys], orb);
            orb->Epoch = dyntime;
         }
         else if (orb->LagDOF == LAGDOF_SPLINE) {
            SplineToPosVel(lagsys, orb, dyntime);
         }
      }
      else if (orb->Regime == ORB_CENTRAL || orb->Regime == ORB_N_BODY) {
         if (orb->SplineActive)
            SplineToPosVel(lagsys, orb, dyntime);
         else if (orb->J2DriftEnabled)
            MeanEph2RV(orb, dyntime);
         else {
            Eph2RV(orb->mu, orb->SLR, orb->ecc, orb->inc, orb->RAAN, orb->ArgP,
                   dyntime - orb->tp, orb->PosN, orb->VelN, &orb->anom);
         }
      }
      /* Else is ORB_ZERO or ORB_FLIGHT, and no action required */

      /* Update CLN */
      switch (orb->Regime) {
         case ORB_ZERO:
            /* L is aligned with N, wln is zero */
            for (i = 0; i < 3; i++) {
               for (j = 0; j < 3; j++)
                  orb->CLN[i][j] = 0.0;
               orb->CLN[i][i] = 1.0;
               orb->wln[i]    = 0.0;
            }
            break;
         case ORB_FLIGHT:
            /* L is East-North-Up */
            R = &rgn[orb->Region];
            for (i = 0; i < 3; i++) {
               orb->PosN[i] = R->PosN[i];
               orb->VelN[i] = R->VelN[i];
            }
            FindENU(orb->PosN, GetWorldW(jd, &worlds[orb->World]), orb->CLN,
                    orb->wln);
            break;
         case ORB_N_BODY:
         case ORB_CENTRAL:
            /* L is LVLH */
            FindCLN(orb->PosN, orb->VelN, orb->CLN, orb->wln);
            break;
         case ORB_THREE_BODY:
            /* L is Rotating Frame XYZ? */
            FindCLN(orb->PosN, orb->VelN, orb->CLN, orb->wln);
            break;
         default:
            fprintf(stderr,
                    "Unknown Orbit Regime in Ephemerides.  Bailing out.\n");
            exit(EXIT_FAILURE);
      }

      /* Update Formation Frame */
      if (frm->FixedInFrame == 'L') {
         MxM(frm->CL, orb->CLN, frm->CN);
      }
      else {
         MxMT(frm->CN, orb->CLN, frm->CL);
      }
   }
}
/**********************************************************************/
ephemType GetEphemType(const char *s)
{
   if (!strcmp(s, "MEAN"))
      return EPH_MEAN;
   else if (!strcmp(s, "DE421"))
      return EPH_DE421;
   else if (!strcmp(s, "DE424"))
      return EPH_DE424;
   else if (!strcmp(s, "DE430"))
      return EPH_DE430;
   else if (!strcmp(s, "DE440"))
      return EPH_DE440;
   else if (!strcmp(s, "GMAT421"))
      return EPH_GMAT421;
   else if (!strcmp(s, "GMAT424"))
      return EPH_GMAT424;
   else if (!strcmp(s, "SPICE"))
      return EPH_SPICE;
   fprintf(stderr, "Bogus input %s in GetEphemType (42init.c:%d)\n", s,
           __LINE__);
   exit(EXIT_FAILURE);
}
/**********************************************************************/
long LoadEphems(const ephemType ephem, const JDType jd,
                JPLHeaderType *const jpl_hdr, struct WorldType *const worlds)
{
   /* Preload Ephemeris Kernels/Definitions */
   switch (ephem) {
      case EPH_MEAN: // No Ephem to Load
         break;
      case EPH_DE430:
      case EPH_DE440:
      case EPH_DE421:
      case EPH_DE424:
      case EPH_GMAT421:
      case EPH_GMAT424:
         return LoadJplEphems(ephem, ModelPath, jpl_hdr, jd, worlds);
         break;
      case EPH_SPICE:
         // Load up Spice kernels
         return SpiceLoadKernels(ModelPath);
         break;
      default:
         fprintf(stderr, "Unknown Ephem Type. Exiting...\n");
         exit(EXIT_FAILURE);
   }
   return (0);
}
/**********************************************************************/
long InitJplHeader(const ephemType ephem, const char eph_path[128],
                   JPLHeaderType *hdr_data)
{
   // read the header file
#define buf_size 512
   // holds flag if NCOEFF and each group of 1030, 1040, 1041, and 1050 are
   // found
   int grp_found[5] = {0};

   hdr_data->eph = ephem;
   strcpy(hdr_data->eph_path, eph_path);

   const TimeSystem cheb_system = TDB_TIME;
   const TimeSystem cheb_epoch  = GMAT_MJD_EPOCH;

   switch (ephem) {
      case EPH_DE421:
      case EPH_GMAT421: {
         strcpy(hdr_data->eph_str, "421");
      } break;
      case EPH_DE424:
      case EPH_GMAT424: {
         strcpy(hdr_data->eph_str, "424");
      } break;
      case EPH_DE430: {
         strcpy(hdr_data->eph_str, "430");
      } break;
      case EPH_DE440: {
         strcpy(hdr_data->eph_str, "440");
      } break;
      default:
         fprintf(stderr, "Unknown ephem type in InitJplHeader(). Exiting...\n");
         exit(EXIT_FAILURE);
   }
   strcpy(hdr_data->hdr_name, "header.");
   strcat(hdr_data->hdr_name, hdr_data->eph_str);

   FILE *const hdr_file =
       FileOpen(hdr_data->eph_path, hdr_data->hdr_name, "rt");

   long grp_num        = 0;
   char line[buf_size] = {"\0"};
   while (fgets(line, buf_size, hdr_file)) {
      if (sscanf(line, "KSIZE=%ld NCOEFF=%ld", &grp_num, &hdr_data->n_coeff) ==
          2) {
         grp_found[0] = 1;
         break;
      }
   }
   hdr_data->blk_len   = hdr_data->n_coeff + 2;
   hdr_data->blk_lines = ((double)hdr_data->blk_len) / 3.0 + 0.5;

   const char *tok_check = " \n\0";

   while (!all_int(4, &grp_found[1]) && fgets(line, buf_size, hdr_file)) {
      const int sscanf_check = sscanf(line, "GROUP %ld", &grp_num) == 1;
      switch ((sscanf_check) ? grp_num : -1) {
         case 1030: {
            while (fgets(line, buf_size, hdr_file)) {
               double jd_days[2] = {0};
               if (sscanf(line, "%lf %lf %lf", &jd_days[0], &jd_days[1],
                          &hdr_data->n_days) == 3) {
                  grp_found[1] = 1;
                  for (int i = 0; i < 2; i++) {
                     hdr_data->jd_range[i] =
                         JDFromDays(jd_days[i], cheb_system, ZERO_EPOCH);
                     JDChangeEpoch(cheb_epoch, &hdr_data->jd_range[i]);
                  }
                  break;
               }
            }
         } break;
         case 1040: {
            grp_found[2] = 1;
            while (fgets(line, buf_size, hdr_file)) {
               if (sscanf(line, "%ld", &hdr_data->n_data) == 1) {
                  break;
               }
            }
            hdr_data->group_1040 = malloc(hdr_data->n_data * sizeof(char[10]));
            char (*const group_1040_start)[10] = hdr_data->group_1040;

            // Assuming data names in group 1040 start immediately after
            // n_data
            while (fgets(line, buf_size, hdr_file)) {
               const char *tok = strtok(line, tok_check);
               if (!tok)
                  break;
               while (tok != NULL) {
                  strcpy(*hdr_data->group_1040, tok);
                  hdr_data->group_1040++;
                  tok = strtok(NULL, tok_check);
               }
            }
            hdr_data->group_1040 = group_1040_start;
         } break;
         case 1041: {
            grp_found[3] = 1;
            while (fgets(line, buf_size, hdr_file)) {
               long n_group_1041 = 0;
               if (sscanf(line, "%ld", &n_group_1041) == 1) {
                  // assuming group 1041 is AFTER group 1040
                  if (n_group_1041 != hdr_data->n_data) {
                     fprintf(stderr,
                             "The data length for groups 1040 and 1041 in DE "
                             "header file '%s' do not match.  "
                             "Exiting...\n\tGroup "
                             "1040 dimension: %ld\n\tGroup 1040 dimension: %ld",
                             hdr_data->hdr_name, hdr_data->n_data,
                             n_group_1041);
                     exit(EXIT_FAILURE);
                  }
                  break;
               }
            }
            hdr_data->group_1041 = calloc(hdr_data->n_data, sizeof(double));
            double *const group_1041_start = hdr_data->group_1041;

            // assuming  group 1041 data is immediately after
            while (fgets(line, buf_size, hdr_file)) {
               replace_char(line, 'D', 'E');
               const char *tok = strtok(line, tok_check);
               if (!tok)
                  break;
               while (tok != NULL) {
                  *hdr_data->group_1041 = atof(tok);
                  hdr_data->group_1041++;
                  tok = strtok(NULL, tok_check);
               }
            }
            hdr_data->group_1041 = group_1041_start;
         } break;
         case 1050: {
            grp_found[4] = 1;
            while (fgets(line, buf_size, hdr_file)) {
               if (is_line_empty(line))
                  continue;

               for (int i = 0; i < 3; i++) {
                  const char *tok = strtok(line, tok_check);
                  for (int j = 0; j < 11; j++) {
                     hdr_data->group_1050[j][i] = atoi(tok);
                     tok                        = strtok(NULL, tok_check);
                  }
                  fgets(line, buf_size, hdr_file);
               }
               break;
            }
         } break;
         default:
            break;
      }
   }
#undef buf_size
   fclose(hdr_file);
   return (all_int(5, grp_found));
}
/******************************************************************************/
double getDEHeader1041Data(const JPLHeaderType *const hdr_data,
                           const char *grp_1040_name)
{
   // Get data from group 1040/1041 in JPL DE header
   for (int i = 0; i < hdr_data->n_data; i++) {
      if (!strncmp(hdr_data->group_1040[i], grp_1040_name, 9))
         return hdr_data->group_1041[i];
   }
   fprintf(stderr, "Could not find `%s` in group 1040 of file %s. Exiting...\n",
           grp_1040_name, hdr_data->hdr_name);
   exit(EXIT_FAILURE);
}
/**********************************************************************/
long LoadJplEphems(ephemType ephem, char EphemPath[128],
                   JPLHeaderType *const jpl_hdr, const JDType jd,
                   struct WorldType *const worlds)
{
   FILE *infile = NULL;
   long BlockNum, NumEntries;
   long FoundBlock;
   char line[512];
   JDType jd_block[2];
   long i, n, Ic, Iw;
   long Nseg, Start, N;
   struct Cheb3DType *Cheb;

   const TimeSystem cheb_system = TDB_TIME;
   const TimeSystem cheb_epoch  = GMAT_MJD_EPOCH;

   JDType jd_cheb = jd, jd_cheb_z = jd;
   JDChangeSystemEpoch(cheb_system, cheb_epoch, &jd_cheb);
   JDChangeSystemEpoch(cheb_system, ZERO_EPOCH, &jd_cheb_z);

   if (jpl_hdr->n_data == 0)
      InitJplHeader(ephem, EphemPath, jpl_hdr);

   // search for the list of file to use with this ephemType
   // only need to do this once and keep it around
   static char (*f_names)[256]  = NULL;
   static JDType(*jd_ranges)[2] = NULL;
   static long n_match          = 0;
   if (f_names == NULL) {
      char search_fmt[20] = "ascp*.";
      strcat(search_fmt, jpl_hdr->eph_str);
      FilesMatchingFmt(EphemPath, search_fmt, &f_names, &n_match);
      if (!n_match) {
         fprintf(stderr,
                 "Could not find any files in directory '%s' for DE type '%s' "
                 "matching glob format '%s'. Exiting...\n",
                 jpl_hdr->eph_path, jpl_hdr->eph_str, search_fmt);
         exit(EXIT_FAILURE);
      }
      jd_ranges             = malloc(n_match * sizeof(JDType[2]));
      double jd_rng_days[2] = {0.0};

      // preload the jd ranges for each file for the chosen DE
      for (i = 0; i < n_match; i++) {
         int first_block = 0;
         double dummy[2] = {0.0};

         infile = FileOpen("", f_names[i], "rt");
         while (fgets(line, 512, infile)) {
            if (sscanf(line, "%ld %ld", &BlockNum, &NumEntries) == 2) {
               fgets(line, 512, infile);
               if (sscanf(line, "%lf %lf %lf", &dummy[0], &jd_rng_days[1],
                          &dummy[1]) == 3)
                  if (!first_block) {
                     first_block    = 1;
                     jd_rng_days[0] = dummy[0];
                  }
            }
         }
         // convert to desired Epoch
         for (int j = 0; j < 2; j++) {
            jd_ranges[i][j] =
                JDFromDays(jd_rng_days[j], cheb_system, ZERO_EPOCH);
            JDChangeEpoch(cheb_epoch, &jd_ranges[i][j]);
         }
         fclose(infile);
      }
   }

   // Make sure the chosen JD is covered by desired DE
   if (isless_jd(jd_cheb, jpl_hdr->jd_range[0]) ||
       isgreater_jd(jd_cheb, jpl_hdr->jd_range[1])) {
      fprintf(stderr,
              "JD is not contained in DE%s ephem files.  Falling back to "
              "lower-precision planetary ephemerides.\n",
              jpl_hdr->eph_str);
      return (1); // TODO: what do we actually do in this case?
   }

   // Figure out which jd range desired JD is in
   int cur_file = -1;
   for (i = 0; i < n_match; i++) {
      if (isgreaterequal_jd(jd_cheb, jd_ranges[i][0]) &&
          isless_jd(jd_cheb, jd_ranges[i][1])) {
         cur_file = i;
         break;
      }
   }
   if (cur_file == -1) {
      fprintf(stderr,
              "Could not find any files in directory '%s' for DE type '%s' "
              "that Julian Date %lf is contained within. Exiting...\n",
              jpl_hdr->eph_path, jpl_hdr->eph_str, JDToDays(jd_cheb_z));
      exit(EXIT_FAILURE);
   }

   // Search found file for block containing chosen JD
   const long blk_len = jpl_hdr->n_coeff + 2;
   double Block[blk_len];

   FoundBlock = 0;
   infile     = FileOpen("", f_names[cur_file], "rt");
   while (!FoundBlock) {
      fgets(line, 512, infile);
      if (sscanf(line, "%ld %ld", &BlockNum, &NumEntries) == 2) {
         fgets(line, 512, infile);
         if (sscanf(line, "%lf %lf %lf", &Block[0], &Block[1], &Block[2]) ==
             3) {
            jd_block[0] = JDFromDays(Block[0], cheb_system, ZERO_EPOCH);
            jd_block[1] = JDFromDays(Block[1], cheb_system, ZERO_EPOCH);
            if (isgreaterequal_jd(jd_cheb_z, jd_block[0]) &&
                isless_jd(jd_cheb_z, jd_block[1])) {
               FoundBlock = 1;

               for (i = 0; i < 2; i++)
                  JDChangeEpoch(GMAT_MJD_EPOCH, &jd_block[i]);
            }
         }
      }
   }

   /* .. Load block */
   for (i = 1; i < jpl_hdr->blk_lines; i++) {
      fgets(line, 512, infile);
      sscanf(line, "%lf %lf %lf", &Block[3 * i], &Block[3 * i + 1],
             &Block[3 * i + 2]);
   }
   fclose(infile);

   /* .. Distribute to Worlds [Starting Entry (1-based), Order, Number of
    * Segments] */
   // Note that the data for 'EARTH' is Earth-Moon barycenter and 'MOON' is the
   // geocentric position of the Moon
   // the order of bodies is the order of columns in block 1050
   static int bodies[11] = {MERCURY, VENUS,   EARTH, MARS, JUPITER, SATURN,
                            URANUS,  NEPTUNE, PLUTO, LUNA, SOL};
   for (int j = 0; j < 11; j++) {
      Iw    = bodies[j];
      Nseg  = jpl_hdr->group_1050[j][2];
      Start = jpl_hdr->group_1050[j][0] - 1;
      N     = jpl_hdr->group_1050[j][1];

      worlds[Iw].eph.Ncheb = Nseg;
      worlds[Iw].eph.Cheb  = (struct Cheb3DType *)realloc(
          worlds[Iw].eph.Cheb, Nseg * sizeof(struct Cheb3DType));
      JDType jd_blk_diff_days = JDSub(jd_block[1], jd_block[0]);
      for (Ic = 0; Ic < Nseg; Ic++) {
         Cheb           = &worlds[Iw].eph.Cheb[Ic];
         Rational mul_1 = InitRational(0, Ic, Nseg);
         Rational mul_2 = InitRational(0, Nseg - 1 - Ic, Nseg);
         Cheb->JD1 = JDAddRationalMult(jd_block[0], mul_1, jd_blk_diff_days);
         Cheb->JD2 = JDSubRationalMult(jd_block[1], mul_2, jd_blk_diff_days);
         Cheb->N   = N;
         for (n = 0; n < N; n++)
            for (i = 0; i < 3; i++)
               Cheb->Coef[i][n] = Block[Start + N * 3 * Ic + N * i + n];
      }
   }

   /* Specific Earth-Moon Mass Ratio and AU  Definitions */
   EMRAT = getDEHeader1041Data(jpl_hdr, "EMRAT"); // Earth/Moon Mass Ratio
   AU    = getDEHeader1041Data(jpl_hdr, "AU");    // Kilometers per 1 AU

   // Conversion of GM from AU^3/day^2 to m^3/s^2 using DE appropriate values
   AUd2ms = (ipow(AU, 3) / ipow(SEC_PER_DAY, 2)) * 1.0e9;

   return (0);
}
//**********************************************************************/
long UpdateJplEphems(JDType jd_tdb_j2000, JDType jd_tt_j2000,
                     const JPLHeaderType *const jpl_hdr,
                     struct WorldType *const worlds)
{
   long i, Iw;
   struct Cheb3DType *Cheb;
   struct OrbitType *Eph;
   struct WorldType *W;
   double u, dudJD, T[20], U[20], P, dPdu;
   double rh[3], vh[3];
   double EarthMoonBaryPosH[3], EarthMoonBaryVelH[3];
   double ZAxis[3] = {0.0, 0.0, 1.0};
   double PosJ[3], VelJ[3];
   double C_W_TETE[3][3] = {{0.0}}, C_TEME_TETE[3][3] = {{0.0}},
          C_TETE_J2000[3][3] = {{0.0}};

   JDChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, &jd_tdb_j2000);
   JDType jd_tdb_mjd = jd_tdb_j2000;
   JDChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, &jd_tdb_mjd);
   JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, &jd_tt_j2000);

   const double j2000_sec = JDToDynTime(jd_tt_j2000);
   const double GMST      = JD2GMST(jd_tt_j2000);

   struct WorldType *const sol = &worlds[SOL];
   JDType jd_sol_cheb          = jd_tdb_mjd;
   JDChangeSystemEpoch(sol->eph.Cheb->JD1.system, sol->eph.Cheb->JD1.epoch,
                       &jd_sol_cheb);

   /* .. Initialize Planetary Pos/Vel */
   for (Iw = SOL; Iw <= LUNA; Iw++) {
      W = &worlds[Iw];
      if (!W->Exists)
         continue;
      Eph = &W->eph;
      /* Determine segment */
      Cheb = &Eph->Cheb[0];

      // Cheb jd will be TDB_TIME and GMAT_MJD_EPOCH, lets just make sure,
      // in case we do something different later
      JDType jd_cheb = jd_sol_cheb;
      JDChangeSystemEpoch(Cheb->JD1.system, Cheb->JD1.epoch, &jd_cheb);
      while (isgreater_jd(jd_cheb, Cheb->JD2))
         Cheb++;
      /* Apply Chebyshev polynomials */
      dudJD = 2.0 / JDSubToDays(Cheb->JD2, Cheb->JD1);
      u     = JDSubToDays(jd_cheb, Cheb->JD1) * dudJD - 1.0;
      ChebyPolys(u, Cheb->N, T, U);
      for (i = 0; i < 3; i++) {
         ChebyInterp(T, U, Cheb->Coef[i], Cheb->N, &P, &dPdu);
         PosJ[i] = 1000.0 * P;
         VelJ[i] = 1000.0 * dPdu * dudJD / SEC_PER_DAY;
      }
      QTxV(worlds[EARTH].qnh, PosJ, Eph->PosN);
      QTxV(worlds[EARTH].qnh, VelJ, Eph->VelN);
   }

   /* Adjust for barycenters */
   /* Move planets from barycentric to Sun-centered */
   for (Iw = PLUTO; Iw >= SOL && Iw <= PLUTO; Iw--) {
      W = &worlds[Iw];
      if (!W->Exists)
         continue;
      for (i = 0; i < 3; i++) {
         W->eph.PosN[i] -= sol->eph.PosN[i];
         W->eph.VelN[i] -= sol->eph.VelN[i];
         W->PosH[i]      = W->eph.PosN[i];
         W->VelH[i]      = W->eph.VelN[i];
      }
      /* Calculate PriMerAng for Planets */
      W->PriMerAng = GetWorldCWN(jd_tdb_j2000, W->ang_data, W->CWN);
      C2Q(W->CWN, W->qwn);
   }

   /* Adjust Earth from Earth-Moon barycenter */
   /* (Moon PosVel is geocentric, not from barycenter) */
   for (i = 0; i < 3; i++) {
      EarthMoonBaryPosH[i]       = worlds[LUNA].eph.PosN[i] / (1.0 + EMRAT);
      EarthMoonBaryVelH[i]       = worlds[LUNA].eph.VelN[i] / (1.0 + EMRAT);
      worlds[EARTH].eph.PosN[i] -= EarthMoonBaryPosH[i];
      worlds[EARTH].eph.VelN[i] -= EarthMoonBaryVelH[i];
      worlds[EARTH].PosH[i]      = worlds[EARTH].eph.PosN[i];
      worlds[EARTH].VelH[i]      = worlds[EARTH].eph.VelN[i];
   }
   for (i = 0; i < 3; i++) {
      rh[i]                = worlds[LUNA].eph.PosN[i];
      vh[i]                = worlds[LUNA].eph.VelN[i];
      worlds[LUNA].PosH[i] = worlds[EARTH].PosH[i] + worlds[LUNA].eph.PosN[i];
      worlds[LUNA].VelH[i] = worlds[EARTH].VelH[i] + worlds[LUNA].eph.VelN[i];
   }
   /* Rotate Moon into ECI */
   QxV(worlds[EARTH].qnh, rh, worlds[LUNA].eph.PosN);
   QxV(worlds[EARTH].qnh, vh, worlds[LUNA].eph.VelN);

   for (Iw = SOL; Iw <= LUNA; Iw++) {
      W = &worlds[Iw];
      if (!W->Exists)
         continue;
      if (Iw == EARTH) {
         /* .. Earth rotation is a special case */
         W->PriMerAng = TwoPi * GMST;
         HiFiEarthPrecNute(jd_tt_j2000, C_TEME_TETE, C_TETE_J2000);
         SimpRot(ZAxis, W->PriMerAng, C_W_TETE);
         MxM(C_W_TETE, C_TETE_J2000, W->CWN);
      }
      else {
         W->PriMerAng = GetWorldCWN(jd_tdb_j2000, W->ang_data, W->CWN);
         GetWorldCNJ(jd_tdb_j2000, W->ang_data, W->CNJ);
         MxM(W->CNJ, worlds[EARTH].CNH, W->CNH);
         C2Q(W->CNJ, W->qnj);
      }
      C2Q(W->CWN, W->qwn);
      C2Q(W->CNH, W->qnh);
   }

   for (Iw = MERCURY; Iw <= LUNA; Iw++) {
      Eph = &worlds[Iw].eph;
      if (!worlds[Iw].Exists)
         continue;
      RV2Eph(j2000_sec, Eph->mu, Eph->PosN, Eph->VelN, &Eph->SMA, &Eph->ecc,
             &Eph->inc, &Eph->RAAN, &Eph->ArgP, &Eph->anom, &Eph->tp, &Eph->SLR,
             &Eph->alpha, &Eph->rmin, &Eph->MeanMotion, &Eph->Period);
   }
   return (0);
}
/**********************************************************************/
long UpdateMeanEphems(JDType jd_tdb_j2000, JDType jd_tt_j2000,
                      struct WorldType *const worlds)
{
   struct OrbitType *Eph;
   struct WorldType *W;

   JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, &jd_tt_j2000);
   JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, &jd_tdb_j2000);
   const double j2000sec = JDToDynTime(jd_tt_j2000);
   const double GMST     = JD2GMST(jd_tt_j2000);

   double r1[3], rh[3], vh[3];
   const double ZAxis[3] = {0.0, 0.0, 1.0};
   long j, Ip;
   double C_W_TETE[3][3], C_TEME_TETE[3][3], C_TETE_J2000[3][3];

   for (Ip = MERCURY; Ip <= PLUTO; Ip++) {
      W = &worlds[Ip];
      if (W->Exists) {
         Eph = &W->eph;
         Eph2RV(Eph->mu, Eph->SLR, Eph->ecc, Eph->inc, Eph->RAAN, Eph->ArgP,
                j2000sec - Eph->tp, Eph->PosN, Eph->VelN, &Eph->anom);
         for (j = 0; j < 3; j++) {
            W->PosH[j] = Eph->PosN[j];
            W->VelH[j] = Eph->VelN[j];
         }
      }
   }
   if (worlds[LUNA].Exists) {
      Eph = &worlds[LUNA].eph;
      /* Meeus computes Luna Position in geocentric ecliptic */

      LunaPosition(jd_tt_j2000, rh);
      JDType jd_tt_j2000_2 = JDAddDays(jd_tt_j2000, 0.01);
      LunaPosition(jd_tt_j2000_2, r1);
      for (j = 0; j < 3; j++)
         vh[j] = (r1[j] - rh[j]) / (864.0);
      /* Convert to Earth's N frame */
      MxV(worlds[EARTH].CNH, rh, Eph->PosN);
      MxV(worlds[EARTH].CNH, vh, Eph->VelN);
      /* Find Luna's osculating elements */
      RV2Eph(j2000sec, Eph->mu, Eph->PosN, Eph->VelN, &Eph->SMA, &Eph->ecc,
             &Eph->inc, &Eph->RAAN, &Eph->ArgP, &Eph->anom, &Eph->tp, &Eph->SLR,
             &Eph->alpha, &Eph->rmin, &Eph->MeanMotion, &Eph->Period);
      for (j = 0; j < 3; j++) {
         worlds[LUNA].PosH[j] = rh[j] + worlds[EARTH].PosH[j];
         worlds[LUNA].VelH[j] = vh[j] + worlds[EARTH].VelH[j];
      }
   }

   for (Ip = SOL; Ip <= PLUTO; Ip++) {
      W = &worlds[Ip];
      if (W->Exists) {
         if (Ip == EARTH) {
            /* .. Earth rotation is a special case */
            W->PriMerAng = TwoPi * GMST;
            HiFiEarthPrecNute(jd_tt_j2000, C_TEME_TETE, C_TETE_J2000);
            SimpRot(ZAxis, W->PriMerAng, C_W_TETE);
            MxM(C_W_TETE, C_TETE_J2000, W->CWN);
         }
         else {
            W->PriMerAng = GetWorldCWN(jd_tdb_j2000, W->ang_data, W->CWN);
         }
         C2Q(W->CWN, W->qwn);
      }
   }

   return (0);
}
/**********************************************************************/
long UpdateMinorBodies(const JDType jd, struct WorldType *const minor_worlds,
                       const double earth_CNH[3][3])
{
   struct OrbitType *Eph;
   struct WorldType *W;
   long j, Imb;

   const double j2000_sec = JDToDynTime(jd);
   JDType jd_tdb_j2000    = jd;
   JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, &jd_tdb_j2000);

   /* .. Locate Asteroids and Comets */
   for (Imb = 0; Imb < Nmb; Imb++) {
      W = &minor_worlds[Imb];
      if (W->Exists) {
         Eph = &W->eph;
         Eph2RV(Eph->mu, Eph->SLR, Eph->ecc, Eph->inc, Eph->RAAN, Eph->ArgP,
                j2000_sec - Eph->tp, Eph->PosN, Eph->VelN, &Eph->anom);
         for (j = 0; j < 3; j++) {
            W->PosH[j] = Eph->PosN[j];
            W->VelH[j] = Eph->VelN[j];
         }

         W->PriMerAng = GetWorldCWN(jd_tdb_j2000, W->ang_data, W->CWN);
         GetWorldCNJ(jd_tdb_j2000, W->ang_data, W->CNJ);
         MxM(W->CNJ, earth_CNH, W->CNH);
         C2Q(W->CNJ, W->qnj);
         C2Q(W->CWN, W->qwn);
         C2Q(W->CNH, W->qnh);
      }
   }
   return (0);
}
/**********************************************************************/
long UpdateNonEphemMoons(JDType jd_tdb_j2000, JDType jd_tt_j2000,
                         struct WorldType *const worlds,
                         const double earth_CNH[3][3])
{
   struct OrbitType *Eph;
   struct WorldType *W, *M;
   double rh[3], vh[3];
   long i;
   WorldID Ip, Iw;

   JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, &jd_tdb_j2000);
   JDType jd_tdb_mjd = jd_tdb_j2000;
   JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, &jd_tdb_mjd);
   const double j2000_sec = JDToDynTime(jd_tt_j2000);

   /* .. Other planets' moons */
   for (Ip = MERCURY; Ip <= PLUTO; Ip++) {
      W = &worlds[Ip];
      if (Ip != EARTH && W->Exists) {
         for (long Im = 0; Im < W->Nsat; Im++) {
            Iw = W->Sat[Im];
            M  = &worlds[Iw];
            if (!M->Exists)
               continue;
            Eph = &M->eph;
            Eph2RV(Eph->mu, Eph->SLR, Eph->ecc, Eph->inc, Eph->RAAN, Eph->ArgP,
                   j2000_sec - Eph->tp, Eph->PosN, Eph->VelN, &Eph->anom);
            GetWorldCNJ(jd_tdb_mjd, M->ang_data, M->CNJ);
            MxM(M->CNJ, earth_CNH, M->CNH);
            MTxV(W->CNH, Eph->PosN, rh);
            MTxV(W->CNH, Eph->VelN, vh);
            for (i = 0; i < 3; i++) {
               M->PosH[i] = rh[i] + W->PosH[i];
               M->VelH[i] = vh[i] + W->VelH[i];
            }

            M->PriMerAng = GetWorldCWN(jd_tdb_mjd, M->ang_data, M->CWN);
            C2Q(M->CNJ, M->qnj);
            C2Q(M->CWN, M->qwn);
            C2Q(M->CNH, M->qnh);
         }
      }
   }
   return (0);
}
/**********************************************************************/
long UpdateEphems(const ephemType ephem, const JDType jd_tdb_j2000,
                  JDType jd_tt_j2000, const JPLHeaderType *const jpl_hdr,
                  struct WorldType *const worlds)
{
   JDType jd_tdb_mjd = jd_tdb_j2000;
   JDChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, &jd_tdb_mjd);
   JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, &jd_tt_j2000);

   long main_ephem_check = 0;
   switch (ephem) {
      case EPH_MEAN: {
         main_ephem_check = UpdateMeanEphems(jd_tdb_j2000, jd_tt_j2000, worlds);
      } break;
      case EPH_DE430:
      case EPH_DE440:
      case EPH_DE421:
      case EPH_DE424:
      case EPH_GMAT421:
      case EPH_GMAT424: {
         // variable time step integrator can go back and forth
         // -> check both directions
         JDType jd_cheb = jd_tdb_mjd;
         JDChangeSystemEpoch(worlds[SOL].eph.Cheb[0].JD1.system,
                             worlds[SOL].eph.Cheb[0].JD1.epoch, &jd_cheb);
         if (isgreaterequal_jd(jd_cheb, worlds[SOL].eph.Cheb[1].JD2) ||
             isless_jd(jd_cheb, worlds[SOL].eph.Cheb[0].JD1))
            LoadJplEphems(ephem, ModelPath, &JplHeader, jd_cheb, worlds);
         /* Load Planetary/Luna ephems */
         main_ephem_check =
             UpdateJplEphems(jd_tdb_j2000, jd_tt_j2000, jpl_hdr, worlds);
      } break;
      case EPH_SPICE: {
         main_ephem_check = SpiceUpdateEphems(jd_tdb_mjd, worlds);
         MxM(CGJ, World[EARTH].CNH, CGH);
         C2Q(World[EARTH].CNH, qjh); // TODO: burn qjh
      } break;
      default:
         fprintf(stderr, "Uknown Ephem Type. Exiting...\n");
         exit(EXIT_FAILURE);
   }

   /* .. Minor Bodies */
   main_ephem_check |=
       UpdateMinorBodies(jd_tdb_mjd, &worlds[NMAJORWORLD], worlds[EARTH].CNH);
   /* .. Other planets' moons */
   if (ephem != EPH_SPICE)
      main_ephem_check |= UpdateNonEphemMoons(jd_tdb_mjd, jd_tt_j2000, worlds,
                                              worlds[EARTH].CNH);

   return main_ephem_check;
}
/**********************************************************************/
void WorldEphemerides(JDType jd_tdb_j2000, JDType jd_tt_j2000, ephemType ephem,
                      struct WorldType *const worlds, struct RegionType *rgn,
                      struct LagrangeSystemType *lagsys)
{
   // BE VERY CAREFUL!! BOTH JDTYPES MUST BE ASSOCIATED WITH THE SAME TIME
   struct WorldType *W;
   struct RegionType *R;
   double ptn[10][3], vtn[10][3], ptw[3];
   struct LagrangeSystemType *LS;
   long i, j, Ir;

   JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, &jd_tdb_j2000);
   JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, &jd_tt_j2000);
   UpdateEphems(ephem, jd_tdb_j2000, jd_tt_j2000, &JplHeader, worlds);

   const double jd2000_tt_sec = JDToDynTime(jd_tt_j2000);

   /* .. Locate Lagrange Points in N of LagSys Body 1 */
   /* Updates some Lagrange point parameters, can help get a more accurate CLN
      but can sometimes cause UnitV errors that make plotting more difficult*/

   // // Updating Lagrange Points may help with plotting, needs further testing
   // UpdateLagrangePoints();
   for (i = 0; i < 3; i++) {
      LS = &lagsys[i];
      if (LS->Exists)
         for (j = 0; j < 5; j++)
            FindLagPtPosVel(jd2000_tt_sec, LS, j, LS->LP[j].PosN,
                            LS->LP[j].VelN, LS->CLN);
   }

   /* .. Regions */
   for (Ir = 0; Ir < Nrgn; Ir++) {
      R = &rgn[Ir];
      W = &worlds[R->World];
      MTxV(W->CWN, R->PosW, R->PosN);
      const double W_w = GetWorldW(jd_tdb_j2000, W);
      R->VelN[0]       = -W_w * R->PosN[1];
      R->VelN[1]       = W_w * R->PosN[0];
      R->VelN[2]       = 0.0;
      MxM(R->CW, W->CWN, R->CN);
   }

   // TODO: Tdrs global
   /* .. TDRS Spacecraft */
   if (worlds[EARTH].Exists) {
      TDRSPosVel(worlds[EARTH].PriMerAng, jd2000_tt_sec, ptn, vtn);
      for (i = 0; i < 10; i++) {
         MxV(worlds[EARTH].CWN, ptn[i], Tdrs[i].rw);
         for (j = 0; j < 3; j++) {
            Tdrs[i].PosN[j] = ptn[i][j];
            Tdrs[i].VelN[j] = vtn[i][j];
         }
         CopyUnitV(Tdrs[i].rw, ptw);
         Tdrs[i].lat = asin(ptw[2]);
         Tdrs[i].lng = atan2(ptw[1], ptw[0]);
      }
   }
}
/**********************************************************************/
void SCEphemerides(const JDType jd, struct SCType *sc,
                   struct WorldType *const world, struct OrbitType *const orb)
{
   double svh[3], p, pvn[3], SoP, Rp;
   long i, j;
   double MagR1, MeanMotion;

   if (sc->Exists) {
      /* Local-vertical frame tied to SC */
      if (orb->Regime == ORB_ZERO) {
         for (i = 0; i < 3; i++) {
            sc->PosR[i] = sc->PosN[i] - orb->PosN[i];
            sc->VelR[i] = sc->VelN[i] - orb->VelN[i];
            for (j = 0; j < 3; j++)
               sc->CLN[i][j] = 0.0;
            sc->CLN[i][i] = 1.0;
            sc->wln[i]    = 0.0;
         }
      }
      else if (orb->Regime == ORB_FLIGHT) {
         for (j = 0; j < 3; j++) {
            sc->PosR[j] = sc->PosN[j] - orb->PosN[j];
            sc->VelR[j] = sc->VelN[j] - orb->VelN[j];
         }
         FindENU(sc->PosN, GetWorldW(jd, world), sc->CLN, sc->wln);
      }
      else if (orb->Regime == ORB_CENTRAL || orb->Regime == ORB_N_BODY) {
         if (sc->OrbDOF == ORBDOF_COWELL) {
            for (j = 0; j < 3; j++) {
               sc->PosR[j] = sc->PosN[j] - orb->PosN[j];
               sc->VelR[j] = sc->VelN[j] - orb->VelN[j];
            }
         }
         else {
            for (j = 0; j < 3; j++) {
               sc->PosN[j] = orb->PosN[j] + sc->PosR[j];
               sc->VelN[j] = orb->VelN[j] + sc->VelR[j];
            }
         }
         FindCLN(sc->PosN, sc->VelN, sc->CLN, sc->wln);
         RelRV2EHRV(orb->SMA, orb->MeanMotion, orb->CLN, sc->PosR, sc->VelR,
                    sc->PosEH, sc->VelEH);
      }
      else { /* ORB_THREE_BODY */
         for (j = 0; j < 3; j++) {
            sc->PosN[j] = orb->PosN[j] + sc->PosR[j];
            sc->VelN[j] = orb->VelN[j] + sc->VelR[j];
         }
         MagR1      = MAGV(orb->PosN);
         MeanMotion = sqrt(orb->mu1 / (MagR1 * MagR1 * MagR1));
         RelRV2EHRV(MagR1, MeanMotion, orb->CLN, sc->PosR, sc->VelR, sc->PosEH,
                    sc->VelEH);
         FindCLN(sc->PosN, sc->VelN, sc->CLN, sc->wln);
      }
      /* Equatorial Frame: e1 = n3, e2 = East, e3 points to World axis */
      FindCEN(sc->PosN, sc->CEN);

      /* Locate Spacecraft in H frame */
      MTxV(world->CNH, sc->PosN, sc->PosH);
      MTxV(world->CNH, sc->VelN, sc->VelH);
      for (j = 0; j < 3; j++) {
         sc->PosH[j] += world->PosH[j];
         sc->VelH[j] += world->VelH[j];
      }

      /* Sun unit vector */
      for (j = 0; j < 3; j++)
         svh[j] = -world->PosH[j];
      MxV(world->CNH, svh, sc->svn);
      for (j = 0; j < 3; j++)
         sc->svn[j] -= sc->PosN[j];
      UNITV(sc->svn);
      MxV(sc->B[0].CN, sc->svn, sc->svb);

      /* Eclipse Flag */
      if (world->Type == SUN)
         sc->Eclipse = FALSE;
      else {
         p = MAGV(sc->PosN);
         for (j = 0; j < 3; j++)
            pvn[j] = -sc->PosN[j];
         UNITV(pvn);
         SoP         = VoV(sc->svn, pvn);
         sc->Eclipse = FALSE;
         if (SoP > 0.0) {
            Rp = world->rad / p;
            if (Rp * Rp > 1.0 - SoP * SoP) {
               sc->Eclipse = TRUE;
            }
         }
      }

      /* S/C relationship to its Formation */
      FindSCinFormation(sc);
   }
}
/**********************************************************************/
void Ephemerides(const JDType jd, ephemType ephem, struct SCType *scs,
                 struct WorldType *const worlds, struct RegionType *rgn,
                 struct LagrangeSystemType *lagsys,
                 struct OrbitType *const orbs)
{
   JDType jd_tdb_j2000 = jd;
   JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, &jd_tdb_j2000);
   JDType jd_tt_j2000 = jd;
   JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, &jd_tt_j2000);
   WorldEphemerides(jd_tdb_j2000, jd_tt_j2000, ephem, worlds, rgn, lagsys);
   for (int i = 0; i < Nsc; i++)
      SCEphemerides(jd_tdb_j2000, &scs[i], worlds, &orbs[scs[i].RefOrb]);
}

/* #ifdef __cplusplus
** }
** #endif
*/

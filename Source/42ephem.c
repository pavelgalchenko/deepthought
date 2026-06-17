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
   S->PosR = VSubV_Elem(S->PosN, NewOrb->PosN);
   S->VelR = VSubV_Elem(S->PosN, NewOrb->PosN);

   pair_vec3_t pair = RelRV2EHRV(MAGV(NewOrb->PosN), MAGV(NewOrb->wln),
                                 NewOrb->CLN, S->PosR, S->VelR);
   S->PosEH         = pair.first;
   S->VelEH         = pair.second;

   /* .. Update RefOrb tag */
   S->RefOrb = Iorb;
}
/**********************************************************************/
void FindSCinFormation(struct SCType *S)
{

   vec3_t psn, pcmn;
   vec3_t wxr, wxrn, vsn;
   struct FormationType *F;

   F = &Frm[S->RefOrb];

   /* .. Find CSF */
   S->CF = MxMT(S->B[0].CN, F->CN);

   if (S->OrbDOF) {
      /* Find PosF */
      pcmn = MTxV(S->B[0].CN, S->cm);
      for (int j = 0; j < 3; j++)
         psn.v[j] = S->PosR.v[j] - F->PosR.v[j] - pcmn.v[j];
      S->PosF = MxV(F->CN, psn);

      /* Find VelF */
      wxr     = VxV(S->B[0].wn, S->cm);
      wxrn    = MTxV(S->B[0].CN, wxr);
      vsn     = VSubV_Elem(S->VelR, wxrn);
      S->VelF = MxV(F->CN, vsn);
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
   long Isc;
   double m  = 0.0;
   vec3_t mr = VEC3_ZERO;
   vec3_t mv = VEC3_ZERO;
   vec3_t PosR, VelR;
   double a, n;
   struct SCType *S;

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (scs[Isc].Exists && scs[Isc].RefOrb == O->Tag) {
         m += scs[Isc].mass;
         for (int i = 0; i < 3; i++) {
            mr.v[i] += scs[Isc].mass * scs[Isc].PosR.v[i];
            mv.v[i] += scs[Isc].mass * scs[Isc].VelR.v[i];
         }
      }
   }
   for (int i = 0; i < 3; i++) {
      PosR.v[i] = mr.v[i] / m;
      VelR.v[i] = mv.v[i] / m;
   }
   /* Visualization gets jittery at about 50 km due to SC.PosR-POV.rr being */
   /* difference of large quantities */
   if (MAGV(PosR) > 50.0E3) {
      O->PosN = VAddV_Elem(O->PosN, PosR);
      O->VelN = VAddV_Elem(O->VelN, VelR);
      if (O->Regime == ORB_CENTRAL)
         RV2Eph(DynTime, O->mu, O->PosN, O->VelN, &O->SMA, &O->ecc, &O->inc,
                &O->RAAN, &O->ArgP, &O->anom, &O->tp, &O->SLR, &O->alpha,
                &O->rmin, &O->MeanMotion, &O->Period);
      FindCLN(O->PosN, O->VelN, &O->CLN, &O->wln);
      a = MAGV(O->PosN);
      n = sqrt(O->mu / (a * a * a));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (scs[Isc].Exists && scs[Isc].RefOrb == O->Tag) {
            S                = &scs[Isc];
            S->PosR          = VSubV_Elem(S->PosR, PosR);
            S->VelR          = VSubV_Elem(S->VelR, VelR);
            pair_vec3_t pair = RelRV2EHRV(a, n, O->CLN, S->PosR, S->VelR);
            S->PosEH         = pair.first;
            S->VelEH         = pair.second;
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
   mat3x3_t CN1H, CN2H, CL1H, CL2H, CH, CBN1, CBN2;
   vec3_t VH;
   struct FormationType *F;
   struct SCType *S;
   struct BodyType *B;
   struct DynType *D;
   long Isc, Ib;

   CN1H = worlds[OldWorld].CNH;
   CN2H = worlds[NewWorld].CNH;

   /* .. Orb */
   CL1H = MxM(O->CLN, CN1H);
   FindCLN(O->PosN, O->VelN, &O->CLN, &O->wln);
   CL2H = MxM(O->CLN, CN2H);
   /* Update Formation Frame */
   F = &Frm[O->Tag];
   if (F->FixedInFrame == 'L') {
      CH    = MxM(F->CL, CL1H);
      F->CL = MxMT(CH, CL2H);
      F->CN = MxM(F->CL, O->CLN);
   }
   else {
      CH    = MxM(F->CN, CN1H);
      F->CN = MxMT(CH, CN2H);
      F->CL = MxMT(F->CN, O->CLN);
   }

   /* .. SC */
   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &scs[Isc];
      if (S->Exists && S->RefOrb == O->Tag) {
         CL1H = MxM(S->CLN, CN1H);

         VH      = MTxV(CN1H, S->PosR);
         S->PosR = MxV(CN2H, VH);
         VH      = MTxV(CN1H, S->VelR);
         S->VelR = MxV(CN2H, VH);
         S->PosN = VAddV_Elem(O->PosN, S->PosR);
         S->VelN = VAddV_Elem(O->VelN, S->VelR);

         FindCLN(S->PosN, S->VelN, &S->CLN, &S->wln);
         CL2H     = MxM(S->CLN, CN2H);
         VH       = MTxV(CL1H, S->PosEH);
         S->PosEH = MxV(CL2H, VH);
         VH       = MTxV(CL1H, S->VelEH);
         S->VelEH = MxV(CL2H, VH);

         /* Bodies */
         for (Ib = 0; Ib < S->Nb; Ib++) {
            B     = &S->B[Ib];
            CH    = MxM(B->CN, CN1H);
            B->CN = MxMT(CH, CN2H);
            B->qn = C2Q(B->CN);
            VH    = MTxV(CN1H, B->vn);
            B->vn = MxV(CN2H, VH);
            VH    = MTxV(CN1H, B->pn);
            B->pn = MxV(CN2H, VH);
         }
         /* Dyn */
         D         = &S->Dyn;
         quat_t qv = DBL_TO_QUAT(&D->x[0]);
         CBN1      = Q2C(qv);
         CH        = MxM(CBN1, CN1H);
         CBN2      = MxMT(CH, CN2H);
         qv        = C2Q(CBN2);
         QUAT_TO_DBL(&D->x[0], qv);
         vec3_t xv = DBL_TO_VEC3(&D->u[D->Nu - 3]);
         VH        = MTxV(CN1H, xv);
         xv        = MxV(CN2H, VH);
         VEC3_TO_DBL(&D->u[D->Nu - 3], xv);
      }
   }

   /* .. POV */
   if (POV.Host.RefOrb == O->Tag) {
      POV.Host.World = NewWorld;
      CH             = MxM(POV.CN, CN1H);
      POV.CN         = MxMT(CH, CN2H);
      POV.CL         = MxMT(POV.CN, scs[POV.Host.SC].CLN);

      switch (POV.Frame) {
         case FRAME_N:
            POV.C = POV.CN;
            break;
         case FRAME_L:
            POV.C = POV.CL;
            break;
         case FRAME_F:
            /* Still needs work */
            POV.C = POV.CF;
            break;
         case FRAME_S:
         case FRAME_B:
            POV.C = POV.CB;
            break;
      }
      POV.q = C2Q(POV.C);
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

   long Im, Iw;
   vec3_t dr, rh, vh;
   struct WorldType *P;
   long Transition = NO_TRANSITION;
   long Body1 = 0, Body2 = 1;

   if (O->Regime == ORB_CENTRAL) {
      /* Falling "in" from Body 1-centered to 3-body */
      P = &worlds[O->World];
      for (Im = 0; Im < P->Nsat; Im++) {
         Iw = P->Sat[Im];
         dr = VSubV_Elem(O->PosN, worlds[Iw].eph.PosN);
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
      dr = VSubV_Elem(O->PosN, worlds[Iw].eph.PosN);
      if (MAGV(dr) < 0.49 * worlds[Iw].RadOfInfluence) {
         Transition = THREEBODY_TO_CENTRAL2;
         Body1      = O->Body1;
         Body2      = O->Body2;
      }
      else {
         /* Falling "out" from 3-body to Body 1-centered */
         dr = VSubV_Elem(O->PosN, worlds[Iw].eph.PosN);
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
         rh        = MTxV(worlds[Body2].CNH, O->PosN);
         vh        = MTxV(worlds[Body2].CNH, O->VelN);
         O->PosN   = MxV(worlds[Body1].CNH, rh);
         O->VelN   = MxV(worlds[Body1].CNH, vh);
         O->PosN   = VAddV_Elem(O->PosN, worlds[Body2].eph.PosN);
         O->VelN   = VAddV_Elem(O->VelN, worlds[Body2].eph.VelN);

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
         O->PosN   = VSubV_Elem(O->PosN, worlds[Body2].eph.PosN);
         O->VelN   = VSubV_Elem(O->VelN, worlds[Body2].eph.VelN);
         rh        = MTxV(worlds[Body1].CNH, O->PosN);
         vh        = MTxV(worlds[Body1].CNH, O->VelN);
         O->PosN   = MxV(worlds[Body2].CNH, rh);
         O->VelN   = MxV(worlds[Body2].CNH, vh);
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
   vec4_t X, Y;
   vec3_t x, v, xn, vn;

   NodeDate.system = O->EphemSystem;

   /* .. Get nodes from O->SplineFile */
   while (dyntime > O->NodeDynTime[2]) {
      for (i = 0; i < 3; i++) {
         O->NodeDynTime[i] = O->NodeDynTime[i + 1];
         O->NodePos[i]     = O->NodePos[i + 1];
         O->NodeVel[i]     = O->NodeVel[i + 1];
      }
      double sec = 0;
      fscanf(O->SplineFile,
             "%ld-%ld-%ldT%ld:%ld:%lf %lf %lf %lf %lf %lf %lf %[\n]",
             &NodeDate.Year, &NodeDate.Month, &NodeDate.Day, &NodeDate.Hour,
             &NodeDate.Minute, &sec, &O->NodePos[3].v[0], &O->NodePos[3].v[1],
             &O->NodePos[3].v[2], &O->NodeVel[3].v[0], &O->NodeVel[3].v[1],
             &O->NodeVel[3].v[2], &newline);
      NodeDate.Second   = double2rational(sec);
      O->NodeDynTime[3] = Date2TimeSystem(NodeDate, TT_TIME);
      O->NodePos[3]     = SxV(1000.0, O->NodePos[3]);
      O->NodeVel[3]     = SxV(1000.0, O->NodeVel[3]);

      if (feof(O->SplineFile)) {
         fprintf(stderr, "Oops.  Reached end of Spline file.\n");
         exit(EXIT_FAILURE);
      }
   }

   /* .. Interpolate Spline */
   for (k = 0; k < 4; k++)
      X.q[k] = O->NodeDynTime[k];
   for (j = 0; j < 3; j++) {
      for (k = 0; k < 4; k++)
         Y.q[k] = O->NodePos[k].v[j];
      x.v[j] = CubicSpline(dyntime, X.q, Y.q);
      for (k = 0; k < 4; k++)
         Y.q[k] = O->NodeVel[k].v[j];
      v.v[j] = CubicSpline(dyntime, X.q, Y.q);
   }

   if (O->Regime == ORB_CENTRAL) {
      O->PosN = x;
      O->VelN = v;
      RV2Eph(O->Epoch, O->mu, O->PosN, O->VelN, &O->SMA, &O->ecc, &O->inc,
             &O->RAAN, &O->ArgP, &O->anom, &O->tp, &O->SLR, &O->alpha, &O->rmin,
             &O->MeanMotion, &O->Period);
      O->tp += SimTime;
   }
   else if (O->Regime == ORB_THREE_BODY) {
      xn = MTxV(lagsys[O->Sys].CLN, x);
      vn = MTxV(lagsys[O->Sys].CLN, v);
      for (j = 0; j < 3; j++) {
         O->PosN = VAddV_Elem(xn, lagsys[O->Sys].LP[O->LP].PosN);
         O->VelN = VAddV_Elem(vn, lagsys[O->Sys].LP[O->LP].VelN);
      }
   }
   else {
      fprintf(stderr, "Invalid Orbit Regime in SplineToPosVel.\n");
      exit(EXIT_FAILURE);
   }
}
/**********************************************************************/
void OrbitOrientation(const JDType jd, const struct WorldType *const world,
                      const struct OrbitType *orb,
                      struct FormationType *const frm, mat3x3_t *const CLN,
                      vec3_t *const wln)
{
   if (!orb->Exists)
      return;

   /* Update CLN and wln */
   switch (orb->Regime) {
      case ORB_ZERO:
         /* L is aligned with N, wln is zero */
         *CLN = MAT3X3_EYE;
         *wln = VEC3_ZERO;
         break;
      case ORB_FLIGHT: {
         /* L is East-North-Up */
         FindENU(orb->PosN, GetWorldW(jd, world), CLN, wln);
      } break;
      case ORB_N_BODY:
      case ORB_CENTRAL:
         /* L is LVLH */
         FindCLN(orb->PosN, orb->VelN, CLN, wln);
         break;
      case ORB_THREE_BODY:
         /* L is Rotating Frame XYZ? */
         FindCLN(orb->PosN, orb->VelN, CLN, wln);
         break;
      default:
         fprintf(stderr,
                 "Unknown Orbit Regime in OrbitOrientation.  Bailing out.\n");
         exit(EXIT_FAILURE);
   }

   /* Update Formation Frame */
   if (frm->FixedInFrame == 'L')
      frm->CN = MxM(frm->CL, *CLN);
   else
      frm->CL = MxMT(frm->CN, *CLN);
}
/**********************************************************************/
void OrbitMotion(JDType jd, struct WorldType *const worlds,
                 struct OrbitType *const orb, struct RegionType *rgn,
                 struct LagrangeSystemType *lagsys,
                 struct FormationType *const frm)
{
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

   if (!orb->Exists)
      return;
   switch (orb->Regime) {
      case ORB_FLIGHT:
         orb->PosN = rgn->PosN;
         orb->VelN = rgn->VelN;
         break;
      case ORB_ZERO:
         break;
      case ORB_N_BODY:
      case ORB_CENTRAL:
         if (orb->SplineActive)
            SplineToPosVel(lagsys, orb, dyntime);
         else if (orb->J2DriftEnabled)
            MeanEph2RV(orb, dyntime);
         else
            Eph2RV(orb->mu, orb->SLR, orb->ecc, orb->inc, orb->RAAN, orb->ArgP,
                   dyntime - orb->tp, &orb->PosN, &orb->VelN, &orb->anom);
         break;
      case ORB_THREE_BODY:
         if (orb->LagDOF == LAGDOF_MODES)
            LagModes2RV(dyntime, &lagsys[orb->Sys], orb, &orb->PosN,
                        &orb->VelN);
         else if (orb->LagDOF == LAGDOF_COWELL) {
            ThreeBodyOrbitRK4(worlds, orb);
            RV2LagModes(dyntime, &lagsys[orb->Sys], orb);
            orb->Epoch = dyntime;
         }
         else if (orb->LagDOF == LAGDOF_SPLINE)
            SplineToPosVel(lagsys, orb, dyntime);
         break;
      default:
         fprintf(stderr,
                 "Unknown Orbit Regime in OrbitMotion.  Bailing out.\n");
         exit(EXIT_FAILURE);
   }

   OrbitOrientation(jd, &worlds[orb->World], orb, frm, &orb->CLN, &orb->wln);
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
   const EpochTT cheb_epoch     = GMAT_MJD_EPOCH;

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
                     hdr_data->jd_range[i] =
                         JDChangeEpoch(cheb_epoch, hdr_data->jd_range[i]);
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
   const EpochTT cheb_epoch     = GMAT_MJD_EPOCH;

   JDType jd_cheb   = JDChangeSystemEpoch(cheb_system, cheb_epoch, jd);
   JDType jd_cheb_z = JDChangeSystemEpoch(cheb_system, ZERO_EPOCH, jd);

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
            jd_ranges[i][j] = JDChangeEpoch(cheb_epoch, jd_ranges[i][j]);
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
                  jd_block[i] = JDChangeEpoch(GMAT_MJD_EPOCH, jd_block[i]);
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
                     struct WorldType *const worlds)
{
   long i;
   WorldID Iw;
   struct Cheb3DType *Cheb;
   struct OrbitType *Eph;
   struct WorldType *W;
   double u, dudJD, T[20], U[20], P, dPdu;
   vec3_t rh, vh;
   vec3_t EarthMoonBaryPosH, EarthMoonBaryVelH;
   vec3_t ZAxis = VEC3_PZAXIS;
   vec3_t PosJ, VelJ;
   mat3x3_t C_W_TETE, C_TETE_J2000;

   jd_tdb_j2000 = JDChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, jd_tdb_j2000);
   JDType jd_tdb_mjd =
       JDChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, jd_tdb_j2000);
   jd_tt_j2000 = JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, jd_tt_j2000);

   const double j2000_sec = JDToDynTime(jd_tt_j2000);
   const double GMST      = JD2GMST(jd_tt_j2000);

   struct WorldType *const sol = &worlds[SOL];
   JDType jd_sol_cheb          = JDChangeSystemEpoch(
       sol->eph.Cheb->JD1.system, sol->eph.Cheb->JD1.epoch, jd_tdb_mjd);

   /* .. Initialize Planetary Pos/Vel */
   for (Iw = SOL; Iw <= LUNA; Iw++) {
      W = &worlds[Iw];
      // need Luna position to figure out Earth Position
      if (!W->Exists && (Iw == LUNA && worlds[EARTH].Exists))
         continue;
      Eph = &W->eph;
      /* Determine segment */
      Cheb = &Eph->Cheb[0];

      // Cheb jd will be TDB_TIME and GMAT_MJD_EPOCH, lets just make sure,
      // in case we do something different later
      JDType jd_cheb =
          JDChangeSystemEpoch(Cheb->JD1.system, Cheb->JD1.epoch, jd_sol_cheb);
      while (isgreater_jd(jd_cheb, Cheb->JD2))
         Cheb++;
      /* Apply Chebyshev polynomials */
      dudJD = 2.0 / JDSubToDays(Cheb->JD2, Cheb->JD1);
      u     = JDSubToDays(jd_cheb, Cheb->JD1) * dudJD - 1.0;
      ChebyPolys(u, Cheb->N, T, U);
      for (i = 0; i < 3; i++) {
         ChebyInterp(T, U, Cheb->Coef[i], Cheb->N, &P, &dPdu);
         PosJ.v[i] = 1000.0 * P;
         VelJ.v[i] = 1000.0 * dPdu * dudJD / SEC_PER_DAY;
      }
      Eph->PosN = QTxV(worlds[EARTH].qnh, PosJ);
      Eph->VelN = QTxV(worlds[EARTH].qnh, VelJ);
   }

   /* Adjust for barycenters */
   /* Move planets from barycentric to Sun-centered */
   for (Iw = PLUTO; Iw >= SOL && Iw <= PLUTO; Iw--) {
      W = &worlds[Iw];
      if (!W->Exists && !(Iw == LUNA && worlds[EARTH].Exists))
         continue;
      W->eph.PosN = VSubV_Elem(W->eph.PosN, sol->eph.PosN);
      W->eph.VelN = VSubV_Elem(W->eph.VelN, sol->eph.VelN);
      W->PosH     = W->eph.PosN;
      W->VelH     = W->eph.VelN;
      /* Calculate PriMerAng for Planets */
      pair_dbl_mat3x3_t dbl_mat = GetWorldCWN(jd_tdb_j2000, W->ang_data);
      W->PriMerAng              = dbl_mat.dbl;
      W->CWN                    = dbl_mat.mat;
      W->qwn                    = C2Q(W->CWN);
   }

   /* Adjust Earth from Earth-Moon barycenter */
   /* (Moon PosVel is geocentric, not from barycenter) */
   for (i = 0; i < 3; i++) {
      EarthMoonBaryPosH.v[i]       = worlds[LUNA].eph.PosN.v[i] / (1.0 + EMRAT);
      EarthMoonBaryVelH.v[i]       = worlds[LUNA].eph.VelN.v[i] / (1.0 + EMRAT);
      worlds[EARTH].eph.PosN.v[i] -= EarthMoonBaryPosH.v[i];
      worlds[EARTH].eph.VelN.v[i] -= EarthMoonBaryVelH.v[i];
   }
   worlds[EARTH].PosH = worlds[EARTH].eph.PosN;
   worlds[EARTH].VelH = worlds[EARTH].eph.VelN;

   rh                = worlds[LUNA].eph.PosN;
   vh                = worlds[LUNA].eph.VelN;
   worlds[LUNA].PosH = VAddV_Elem(worlds[EARTH].PosH, worlds[LUNA].eph.PosN);
   worlds[LUNA].VelH = VAddV_Elem(worlds[EARTH].VelH, worlds[LUNA].eph.VelN);

   /* Rotate Moon into ECI */
   worlds[LUNA].eph.PosN = QxV(worlds[EARTH].qnh, rh);
   worlds[LUNA].eph.VelN = QxV(worlds[EARTH].qnh, vh);

   for (Iw = SOL; Iw <= LUNA; Iw++) {
      W = &worlds[Iw];
      if (!W->Exists)
         continue;
      if (Iw == EARTH) {
         /* .. Earth rotation is a special case */
         W->PriMerAng             = TwoPi * GMST;
         const pair_mat3x3_t pair = HiFiEarthPrecNute(jd_tt_j2000);
         C_TETE_J2000             = pair.second;
         C_W_TETE                 = SimpRot(ZAxis, W->PriMerAng);
         W->CWN                   = MxM(C_W_TETE, C_TETE_J2000);
      }
      else {
         pair_dbl_mat3x3_t dbl_mat = GetWorldCWN(jd_tdb_j2000, W->ang_data);
         W->PriMerAng              = dbl_mat.dbl;
         W->CWN                    = dbl_mat.mat;
         W->CNJ                    = GetWorldCNJ(jd_tdb_j2000, W->ang_data);
         W->CNH                    = MxM(W->CNJ, worlds[EARTH].CNH);
         W->qnj                    = C2Q(W->CNJ);
      }
      W->qwn = C2Q(W->CWN);
      W->qnh = C2Q(W->CNH);
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

   jd_tt_j2000  = JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, jd_tt_j2000);
   jd_tdb_j2000 = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd_tdb_j2000);
   const double j2000sec = JDToDynTime(jd_tt_j2000);
   const double GMST     = JD2GMST(jd_tt_j2000);

   vec3_t r1, rh, vh;
   const vec3_t ZAxis = VEC3_PZAXIS;
   long Ip;
   mat3x3_t C_W_TETE, C_TETE_J2000;

   for (Ip = MERCURY; Ip <= PLUTO; Ip++) {
      W = &worlds[Ip];
      if (W->Exists) {
         Eph = &W->eph;
         Eph2RV(Eph->mu, Eph->SLR, Eph->ecc, Eph->inc, Eph->RAAN, Eph->ArgP,
                j2000sec - Eph->tp, &Eph->PosN, &Eph->VelN, &Eph->anom);
         W->PosH = Eph->PosN;
         W->VelH = Eph->VelN;
      }
   }
   if (worlds[LUNA].Exists) {
      Eph = &worlds[LUNA].eph;
      /* Meeus computes Luna Position in geocentric ecliptic */

      rh                   = LunaPosition(jd_tt_j2000);
      JDType jd_tt_j2000_2 = JDAddDays(jd_tt_j2000, 0.01);
      r1                   = LunaPosition(jd_tt_j2000_2);
      for (int j = 0; j < 3; j++)
         vh.v[j] = (r1.v[j] - rh.v[j]) / (864.0);
      /* Convert to Earth's N frame */
      Eph->PosN = MxV(worlds[EARTH].CNH, rh);
      Eph->VelN = MxV(worlds[EARTH].CNH, vh);
      /* Find Luna's osculating elements */
      RV2Eph(j2000sec, Eph->mu, Eph->PosN, Eph->VelN, &Eph->SMA, &Eph->ecc,
             &Eph->inc, &Eph->RAAN, &Eph->ArgP, &Eph->anom, &Eph->tp, &Eph->SLR,
             &Eph->alpha, &Eph->rmin, &Eph->MeanMotion, &Eph->Period);

      worlds[LUNA].PosH = VAddV_Elem(rh, worlds[EARTH].PosH);
      worlds[LUNA].VelH = VAddV_Elem(vh, worlds[EARTH].VelH);
   }

   for (Ip = SOL; Ip <= PLUTO; Ip++) {
      W = &worlds[Ip];
      if (W->Exists) {
         if (Ip == EARTH) {
            /* .. Earth rotation is a special case */
            W->PriMerAng             = TwoPi * GMST;
            const pair_mat3x3_t pair = HiFiEarthPrecNute(jd_tt_j2000);
            C_TETE_J2000             = pair.second;
            C_W_TETE                 = SimpRot(ZAxis, W->PriMerAng);
            W->CWN                   = MxM(C_W_TETE, C_TETE_J2000);
         }
         else {
            pair_dbl_mat3x3_t dbl_mat = GetWorldCWN(jd_tdb_j2000, W->ang_data);
            W->PriMerAng              = dbl_mat.dbl;
            W->CWN                    = dbl_mat.mat;
         }
         W->qwn = C2Q(W->CWN);
      }
   }

   return (0);
}
/**********************************************************************/
long UpdateMinorBodies(JDType jd_tdb_j2000, const JDType jd_tt_j2000,
                       struct WorldType *const minor_worlds,
                       const mat3x3_t earth_CNH)
{
   struct OrbitType *Eph;
   struct WorldType *W;
   long Imb;

   const double j2000_sec = JDToDynTime(jd_tt_j2000);
   jd_tdb_j2000 = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd_tdb_j2000);

   /* .. Locate Asteroids and Comets */
   for (Imb = 0; Imb < Nmb; Imb++) {
      W = &minor_worlds[Imb];
      if (W->Exists) {
         Eph = &W->eph;
         Eph2RV(Eph->mu, Eph->SLR, Eph->ecc, Eph->inc, Eph->RAAN, Eph->ArgP,
                j2000_sec - Eph->tp, &Eph->PosN, &Eph->VelN, &Eph->anom);

         W->PosH = Eph->PosN;
         W->VelH = Eph->VelN;

         pair_dbl_mat3x3_t dbl_mat = GetWorldCWN(jd_tdb_j2000, W->ang_data);
         W->PriMerAng              = dbl_mat.dbl;
         W->CWN                    = dbl_mat.mat;
         W->CNJ                    = GetWorldCNJ(jd_tdb_j2000, W->ang_data);
         W->CNH                    = MxM(W->CNJ, earth_CNH);
         W->qnj                    = C2Q(W->CNJ);
         W->qwn                    = C2Q(W->CWN);
         W->qnh                    = C2Q(W->CNH);
      }
   }
   return (0);
}
/**********************************************************************/
long UpdateNonEphemMoons(JDType jd_tdb_j2000, JDType jd_tt_j2000,
                         struct WorldType *const worlds,
                         const mat3x3_t earth_CNH)
{
   struct OrbitType *Eph;
   struct WorldType *W, *M;
   vec3_t rh, vh;
   WorldID Ip, Iw;

   jd_tdb_j2000      = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd_tdb_j2000);
   JDType jd_tdb_mjd = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd_tdb_j2000);

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
                   j2000_sec - Eph->tp, &Eph->PosN, &Eph->VelN, &Eph->anom);
            M->CNJ  = GetWorldCNJ(jd_tdb_mjd, M->ang_data);
            M->CNH  = MxM(M->CNJ, earth_CNH);
            rh      = MTxV(W->CNH, Eph->PosN);
            vh      = MTxV(W->CNH, Eph->VelN);
            M->PosH = VAddV_Elem(rh, W->PosH);
            M->VelH = VAddV_Elem(vh, W->VelH);

            pair_dbl_mat3x3_t dbl_mat = GetWorldCWN(jd_tdb_j2000, M->ang_data);
            M->PriMerAng              = dbl_mat.dbl;
            M->CWN                    = dbl_mat.mat;
            M->qnj                    = C2Q(M->CNJ);
            M->qwn                    = C2Q(M->CWN);
            M->qnh                    = C2Q(M->CNH);
         }
      }
   }
   return (0);
}
/**********************************************************************/
long UpdateEphems(const ephemType ephem, const JDType jd_tdb_j2000,
                  JDType jd_tt_j2000, struct WorldType *const worlds)
{
   JDType jd_tdb_mjd =
       JDChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, jd_tdb_j2000);
   jd_tt_j2000 = JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, jd_tt_j2000);

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
         JDType jd_cheb =
             JDChangeSystemEpoch(worlds[SOL].eph.Cheb[0].JD1.system,
                                 worlds[SOL].eph.Cheb[0].JD1.epoch, jd_tdb_mjd);
         if (isgreaterequal_jd(jd_cheb, worlds[SOL].eph.Cheb[1].JD2) ||
             isless_jd(jd_cheb, worlds[SOL].eph.Cheb[0].JD1))
            LoadJplEphems(ephem, ModelPath, &JplHeader, jd_cheb, worlds);
         /* Load Planetary/Luna ephems */
         main_ephem_check = UpdateJplEphems(jd_tdb_j2000, jd_tt_j2000, worlds);
      } break;
      case EPH_SPICE: {
         main_ephem_check = SpiceUpdateEphems(jd_tdb_mjd, worlds);
         CGH              = MxM(CGJ, World[EARTH].CNH);
         qjh              = C2Q(World[EARTH].CNH); // TODO: burn qjh
      } break;
      default:
         fprintf(stderr, "Uknown Ephem Type. Exiting...\n");
         exit(EXIT_FAILURE);
   }

   /* .. Minor Bodies */
   main_ephem_check |= UpdateMinorBodies(
       jd_tdb_j2000, jd_tt_j2000, &worlds[NMAJORWORLD], worlds[EARTH].CNH);
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
   vec3_t ptn[10], vtn[10], ptw;
   struct LagrangeSystemType *LS;
   long i, j, Ir;

   jd_tdb_j2000 = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd_tdb_j2000);
   jd_tt_j2000  = JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, jd_tt_j2000);
   UpdateEphems(ephem, jd_tdb_j2000, jd_tt_j2000, worlds);

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
            FindLagPtPosVel(jd2000_tt_sec, LS, j, &LS->LP[j].PosN,
                            &LS->LP[j].VelN, &LS->CLN);
   }

   /* .. Regions */
   for (Ir = 0; Ir < Nrgn; Ir++) {
      R                = &rgn[Ir];
      W                = &worlds[R->World];
      R->PosN          = MTxV(W->CWN, R->PosW);
      const double W_w = GetWorldW(jd_tdb_j2000, W);
      R->VelN.v[0]     = -W_w * R->PosN.v[1];
      R->VelN.v[1]     = W_w * R->PosN.v[0];
      R->VelN.v[2]     = 0.0;
      R->CN            = MxM(R->CW, W->CWN);
   }

   // TODO: Tdrs global
   /* .. TDRS Spacecraft */
   if (worlds[EARTH].Exists) {
      TDRSPosVel(worlds[EARTH].PriMerAng, jd2000_tt_sec, ptn, vtn);
      for (i = 0; i < 10; i++) {
         Tdrs[i].rw   = MxV(worlds[EARTH].CWN, ptn[i]);
         Tdrs[i].PosN = ptn[i];
         Tdrs[i].VelN = vtn[i];

         ptw         = UNITV(Tdrs[i].rw).v;
         Tdrs[i].lat = asin(ptw.z);
         Tdrs[i].lng = atan2(ptw.y, ptw.x);
      }
   }
}
/**********************************************************************/
void SCEphemerides(const JDType jd, struct SCType *sc,
                   struct WorldType *const world, struct OrbitType *const orb)
{
   vec3_t svh, pvn;
   double MagR1, MeanMotion, SoP, Rp, p;

   if (sc->Exists) {
      /* Local-vertical frame tied to SC */
      if (orb->Regime == ORB_ZERO) {
         sc->CLN  = MAT3X3_EYE;
         sc->wln  = VEC3_ZERO;
         sc->PosR = VSubV_Elem(sc->PosN, orb->PosN);
         sc->VelR = VSubV_Elem(sc->VelN, orb->VelN);
      }
      else if (orb->Regime == ORB_FLIGHT) {
         sc->PosR = VSubV_Elem(sc->PosN, orb->PosN);
         sc->VelR = VSubV_Elem(sc->VelN, orb->VelN);
         FindENU(sc->PosN, GetWorldW(jd, world), &sc->CLN, &sc->wln);
      }
      else if (orb->Regime == ORB_CENTRAL || orb->Regime == ORB_N_BODY) {
         if (sc->OrbDOF == ORBDOF_COWELL) {
            sc->PosR = VSubV_Elem(sc->PosN, orb->PosN);
            sc->VelR = VSubV_Elem(sc->VelN, orb->VelN);
         }
         else {
            sc->PosN = VAddV_Elem(orb->PosN, sc->PosR);
            sc->VelN = VAddV_Elem(orb->VelN, sc->VelR);
         }
         FindCLN(sc->PosN, sc->VelN, &sc->CLN, &sc->wln);
         pair_vec3_t pair = RelRV2EHRV(orb->SMA, orb->MeanMotion, orb->CLN,
                                       sc->PosR, sc->VelR);
         sc->PosEH        = pair.first;
         sc->VelEH        = pair.second;
      }
      else { /* ORB_THREE_BODY */
         sc->PosN = VAddV_Elem(orb->PosN, sc->PosR);
         sc->VelN = VAddV_Elem(orb->VelN, sc->VelR);

         MagR1      = MAGV(orb->PosN);
         MeanMotion = sqrt(orb->mu1 / (MagR1 * MagR1 * MagR1));
         pair_vec3_t pair =
             RelRV2EHRV(MagR1, MeanMotion, orb->CLN, sc->PosR, sc->VelR);
         sc->PosEH = pair.first;
         sc->VelEH = pair.second;
         FindCLN(sc->PosN, sc->VelN, &sc->CLN, &sc->wln);
      }
      /* Equatorial Frame: e1 = n3, e2 = East, e3 points to World axis */
      sc->CEN = FindCEN(sc->PosN);

      /* Locate Spacecraft in H frame */
      sc->PosH = MTxV(world->CNH, sc->PosN);
      sc->VelH = MTxV(world->CNH, sc->VelN);
      sc->PosH = VAddV_Elem(sc->PosH, world->PosH);
      sc->VelH = VAddV_Elem(sc->VelH, world->VelH);

      /* Sun unit vector */
      svh     = NegV_Elem(world->PosH);
      sc->svn = MxV(world->CNH, svh);
      sc->svn = VSubV_Elem(sc->svn, sc->PosN);
      sc->svn = UNITV(sc->svn).v;
      sc->svb = MxV(sc->B[0].CN, sc->svn);

      /* Eclipse Flag */
      if (world->Type == SUN)
         sc->Eclipse = FALSE;
      else {
         p           = MAGV(sc->PosN);
         pvn         = NegV_Elem(sc->PosN);
         pvn         = UNITV(pvn).v;
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
   JDType jd_tdb_j2000 = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);
   JDType jd_tt_j2000  = JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, jd);
   WorldEphemerides(jd_tdb_j2000, jd_tt_j2000, ephem, worlds, rgn, lagsys);
   for (int i = 0; i < Nsc; i++)
      SCEphemerides(jd_tdb_j2000, &scs[i], worlds, &orbs[scs[i].RefOrb]);
}

/* #ifdef __cplusplus
** }
** #endif
*/

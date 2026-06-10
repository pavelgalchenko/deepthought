/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

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
double ViscousFriction(struct WhlType *W) __attribute__((pure));
double ViscousFriction(struct WhlType *W)
{
   return (-W->ViscCoef * W->w);
}
/**********************************************************************/
/* Ref: Lugre Friction Model.pdf                                      */
void WhlDrag(struct WhlType *W)
{
   // TODO: this will have problems with the rk integrators
   double v0, g, m, vd, zdot;

   v0 = W->w / W->StribeckZone;
   g  = W->CoulCoef + W->StribeckCoef * exp(-(v0 * v0));
   m  = W->LugreSpringCoef * fabs(W->w) / g;
   if (m * DTSIM > 1.0) {
      zdot       = (g * signum(W->w) / W->LugreSpringCoef - W->z) / DTSIM;
      W->FricTrq = -g * signum(W->w) + ViscousFriction(W);
   }
   else {
      zdot       = W->w - m * W->z;
      vd         = W->w / W->LugreDampZone;
      W->FricTrq = -W->LugreSpringCoef * W->z -
                   W->LugreDampCoef * exp(-vd * vd) * zdot + ViscousFriction(W);
   }

   W->z += zdot * DTSIM;
}
/**********************************************************************/
#define SMOOTH_INTERVAL (0.05) // ratio of WhlType::H
void WhlModel(const int smoothing, struct WhlType *W, struct SCType *S)
{
   struct BodyType *B;
   struct NodeType *N;

   W->Trq = W->Tcmd;
   if (S->WhlDragActive) {
      WhlDrag(W);
      W->Trq += W->FricTrq;
   }
   if (W->Trq < -W->Tmax)
      W->Trq = -W->Tmax;
   if (W->Trq > W->Tmax)
      W->Trq = W->Tmax;

   if (W->Trq * W->H > 0) {
      // smooth W->Trq such that it is at it's current value at
      // W->H = (1 - SMOOTH_INTERVAL) * W->Hmax and zero
      // once the wheel reaches saturation
      static double oneMInterval = 1.0 - SMOOTH_INTERVAL;
      if (fabs(W->H) >= W->Hmax)
         W->Trq = 0.0;
      else if (smoothing && fabs(W->H) > oneMInterval * W->Hmax) {
         // convert to interval (0.0, 1.0)
         const double t = (W->Hmax - fabs(W->H)) / (SMOOTH_INTERVAL * W->Hmax);
         W->Trq         = smootherstep(t) * W->Trq;
      }
   }

   if (S->FlexActive) {
      B = &S->B[W->Body];
      N = &B->Node[W->Node];
      for (int i = 0; i < 3; i++)
         N->Trq.v[i] += W->Trq * W->A.v[i];
   }
}
#undef SMOOTH_INTERVAL
/**********************************************************************/
void MTBModel(struct MTBType *MTB, vec3_t bvb)
{
   MTB->M   = Limit(MTB->Mcmd, -MTB->Mmax, MTB->Mmax);
   MTB->Trq = SxV(MTB->M, VxV(MTB->A, bvb));
}
/**********************************************************************/
#define SMOOTH_INTERVAL (0.05) // seconds
void ThrModel(const int smoothing, struct ThrType *Thr, struct SCType *S,
              JDType jd)
{
   struct BodyType *B;
   struct NodeType *N;

   if (Thr->Mode == THR_PULSED) { /* THR_PULSED */
      // TODO: make this less ad-hoc, or at least make it user configurable
      JDType jd_thr = Thr->PulseWidthFinTimeStamp;
      if (isequal_jd_systemepoch(jd_thr, jd) && smoothing) {
         static double halfInterval = SMOOTH_INTERVAL / 2.0;
         const double timeToEnd     = JDSubToSeconds(jd_thr, jd);
         if (timeToEnd >= halfInterval)
            Thr->F = Thr->Fmax;
         else if (fabs(timeToEnd) < halfInterval) {
            // convert to interval (0.0, 1.0)
            const double t = timeToEnd / SMOOTH_INTERVAL + 0.5;
            Thr->F         = smootherstep(t) * Thr->Fmax;
         }
         else
            Thr->F = 0.0;
      }
      else if (isequal_jd_systemepoch(jd_thr, jd) && isgreater_jd(jd_thr, jd)) {
         double thrust_steps = JDSubToSeconds(jd_thr, jd) / DTSIM;
         if (thrust_steps >= 1.0)
            Thr->F = Thr->Fmax;
         else
            Thr->F = thrust_steps * Thr->Fmax;
      }
      else
         Thr->F = 0.0;
   }
   else /* THR_PROPORTIONAL */
      Thr->F = Thr->ThrustLevelCmd * Thr->Fmax;

   if (Thr->F < 0.0)
      Thr->F = 0.0;
   if (Thr->F > Thr->Fmax)
      Thr->F = Thr->Fmax;

   Thr->Frc = SxV(Thr->F, Thr->A);

   B = &S->B[Thr->Body];
   N = &B->Node[Thr->Node];

   Thr->Trq = VxV(N->PosCm, Thr->Frc);

   if (S->FlexActive) {
      N->Trq = VpVElem(N->Trq, Thr->Trq);
      N->Frc = VpVElem(N->Frc, Thr->Frc);
   }
}
#undef SMOOTH_INTERVAL
/**********************************************************************/
void ThrusterPlumeFrcTrq(struct SCType *S)
{
   /* Plume Parameters */
   double Temp = 100.0; /* WAG */
   double R    = 8.134; /* J/(K*mol) */
   double Beta = 1.0 / sqrt(2.0 * R * Temp);
   double Ve   = 100.0; /* WAG */
   double s    = Beta * Ve;
   double y    = s;
   double A1   = exp(-y * y) + sqrt(Pi) * y * (1.0 + erf(y));
   double mdot = 0.1; /* WAG */
   double Coef = mdot / (Beta * A1 * Pi);
   /* Other variables */
   struct ThrType *T;
   struct BodyType *B, *Bt;
   struct NodeType *Nt;
   struct GeomType *G;
   struct PolyType *P;
   mat3x3_t CPB;
   vec3_t PosThrN, PosThrB, AxisN, PosB, PosP, Phat, FrcP, FrcB, r, TrqB, FrcN;
   double AoN, MagPos, cosphi, w, w2, Wpoly, TotalCoef;
   long Ithr, Ipoly, Ib;

   for (Ithr = 0; Ithr < S->Nthr; Ithr++) {
      T  = &S->Thr[Ithr];
      Bt = &S->B[T->Body];
      Nt = &Bt->Node[T->Node];

      if (T->F > 0.0) { /* Check that this is legit */

         /* Find Force and Torque on each Body */
         for (Ib = 0; Ib < S->Nb; Ib++) {
            B = &S->B[Ib];
            G = &Geom[B->GeomTag];

            /* Find thruster location, axis in B */
            PosThrN = MTxV(Bt->CN, Nt->PosB);
            for (int i = 0; i < 3; i++)
               PosThrN.v[i] += Bt->pn.v[i] - B->pn.v[i];
            PosThrB = MxV(B->CN, PosThrN);
            /* Note that plume axis is opposite T->A */
            /* CPB is DCM from B to Plume (P) frame */
            AxisN            = MTxV(Bt->CN, T->A);
            CPB.rows[0]      = MxV(B->CN, AxisN);
            CPB.rows[0]      = SxV(-1.0, CPB.rows[0]);
            pair_vec3_t pair = PerpBasis(CPB.rows[0]);
            CPB.rows[1]      = pair.first;
            CPB.rows[2]      = pair.second;

            /* Find force and torque on each illuminated polygon */
            for (Ipoly = 0; Ipoly < G->Npoly; Ipoly++) {
               P = &G->Poly[Ipoly];
               if (strcmp(Matl[P->Matl].Label,
                          "INTERIOR")) { /* Plume doesn't see interior polys */
                  AoN = VoV(CPB.rows[0], P->Norm);
                  if (AoN < 0.0) { /* Plume doesn't see polys facing away */
                     /* Find plume pressure (momentum flux) at poly centroid */
                     PosB = VmVElem(P->Centroid, PosThrB);
                     PosP = MxV(CPB, PosB);
                     if (PosP.x > 0.0) { /* Ignore backflow */
                        magvec3_t uv  = UNITV(PosP);
                        MagPos        = uv.m;
                        Phat          = uv.v;
                        cosphi        = Phat.x;
                        w             = s * cosphi;
                        w2            = w * w;
                        Wpoly         = (w2 + 2.5) * w * exp(-w2);
                        Wpoly        += (0.75 + 3.0 * w2 + w2 * w2) * sqrt(Pi) *
                                        (1.0 + erf(w));
                        Wpoly        *= exp(w2 - s * s);
                        TotalCoef =
                            P->Area * Coef * cosphi * Wpoly / (MagPos * MagPos);

                        FrcP = SxV(TotalCoef, Phat);
                        FrcB = MTxV(CPB, FrcP);

                        /* Find plume force in B frame */
                        r       = VmVElem(P->Centroid, B->cm);
                        TrqB    = VxV(r, FrcB);
                        FrcN    = MTxV(B->CN, FrcB);
                        B->FrcN = VpVElem(B->FrcN, FrcN);
                        B->FrcB = VpVElem(B->FrcB, FrcB);
                        B->Trq  = VpVElem(B->Trq, TrqB);
                     }
                  }
               }
            }
         }
      }
   }
}
/**********************************************************************/
/*  This function is called at the simulation rate.  Sub-sampling of  */
/*  actuators should be done on a case-by-case basis.                 */

void Actuators(const int smoothing, struct SCType *S, JDType jd)
{

   struct NodeType *N;
   long i, j;
   vec3_t FrcN, FrcB;
   struct AcType *AC;
   struct JointType *G;
   struct AcJointType *AG;
   struct ThrType *Thr;
   struct ShakerType *Sh;
   struct WhlType *W;

   AC = &S->AC;

   /* Ideal Actuators */
   for (i = 0; i < 3; i++)
      FrcB.v[i] = S->IdealAct[i].Fcmd;
   FrcN         = MTxV(S->B[0].CN, FrcB);
   S->B[0].FrcB = VpVElem(S->B[0].FrcB, FrcB);
   S->B[0].FrcN = VpVElem(S->B[0].FrcN, FrcN);
   for (i = 0; i < 3; i++)
      S->B[0].Trq.v[i] += S->IdealAct[i].Tcmd;

   if (S->FlexActive) {
      N = &S->B[0].Node[0]; /* Arbitrarily put ideal actuators at Node 0 */
      for (i = 0; i < 3; i++) {
         N->Trq.v[i] += S->IdealAct[i].Tcmd;
         N->Frc.v[i] += S->IdealAct[i].Fcmd;
      }
   }

   /* Wheels */
   for (i = 0; i < S->Nw; i++) {
      WhlModel(smoothing, &S->Whl[i], S);
   }
   /* MTBs */
   for (i = 0; i < S->Nmtb; i++) {
      MTBModel(&S->MTB[i], S->bvb);
      S->B[0].Trq = VpVElem(S->B[0].Trq, S->MTB[i].Trq);
   }

   /* Gimbal Drives */
   for (i = 0; i < AC->Ng; i++) {
      G  = &S->G[i];
      AG = &AC->G[i];
      if (G->Type == ACTUATED_JOINT) {
         for (j = 0; j < G->RotDOF; j++) {
            G->AngRateCmd.v[j] = AG->Cmd.AngRate.v[j];
         }
         for (j = 0; j < G->TrnDOF; j++) {
            G->PosRateCmd.v[j] = AG->Cmd.PosRate.v[j];
         }
      }
   }

   /* Thrusters */
   for (i = 0; i < S->Nthr; i++) {
      Thr = &S->Thr[i];
      ThrModel(smoothing, Thr, S, jd);
      FrcN                 = MTxV(S->B[Thr->Body].CN, Thr->Frc);
      S->B[Thr->Body].Trq  = VpVElem(S->B[Thr->Body].Trq, Thr->Trq);
      S->B[Thr->Body].FrcN = VpVElem(S->B[Thr->Body].FrcN, FrcN);
      S->B[Thr->Body].FrcB = VpVElem(S->B[Thr->Body].FrcB, Thr->Frc);
   }
   if (ThrusterPlumesActive) {
      ThrusterPlumeFrcTrq(S);
   }

   /* Wheel Jitter and Shakers only affect Flex */
   if (S->FlexActive) {
      for (i = 0; i < S->Nsh; i++) {
         Sh = &S->Shaker[i];
         N  = &S->B[Sh->Body].Node[Sh->Node];
         ShakerJitter(Sh, S);
         if (Sh->FrcTrq == FORCE)
            for (i = 0; i < 3; i++)
               N->Frc.v[i] += Sh->Output * Sh->Axis.v[i];
         else
            for (i = 0; i < 3; i++)
               N->Trq.v[i] += Sh->Output * Sh->Axis.v[i];
      }

      if (S->WhlJitterActive) {
         for (i = 0; i < S->Nw; i++) {
            W = &S->Whl[i];
            N = &S->B[W->Body].Node[W->Node];
            WheelJitter(W, S);
            N->Frc = VpVElem(N->Frc, W->JitFrc);
            N->Trq = VpVElem(N->Trq, W->JitTrq);
         }
      }
   }
}

/* #ifdef __cplusplus
** }
** #endif
*/

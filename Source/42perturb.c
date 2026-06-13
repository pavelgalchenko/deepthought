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
/*********************************************************************/
/* Ref: Sutherland-Hodgman                                           */
long ClipEdgeAgainstPlane(vec3_t V1, vec3_t V2, vec3_t A, vec3_t B, vec3_t C,
                          vec3_t DirVec, vec3_t OutVtx[2])
{
   double c1, c2;
   vec3_t P1, P2;
   vec4_t Bary1, Bary2;
   long Nout, i;

   ProjectPointOntoTriangle(A, B, C, DirVec, V1, &P1, &Bary1);
   ProjectPointOntoTriangle(A, B, C, DirVec, V2, &P2, &Bary2);

   for (i = 0; i < 4; i++) {
      if (fabs(Bary1.q[i]) < 1.0E-6)
         Bary1.q[i] = 0.0;
      if (fabs(Bary2.q[i]) < 1.0E-6)
         Bary2.q[i] = 0.0;
   }

   /* .. Clip against plane formed by AB edge and DirVec */
   c1 = Bary1.z;
   c2 = Bary2.z;
   if (Bary1.s > 0.0 && Bary2.s > 0.0) {
      /* Edge is above plane of ABC */
      if (c1 >= 0.0 && c2 >= 0.0) {
         /* Case 1: Both Inside */
         OutVtx[0] = V2;
         Nout      = 1;
      }
      else if (c1 >= 0.0 && c2 < 0.0) {
         /* Case 2: P1 inside, P2 outside */
         Nout = 1;
         for (i = 0; i < 3; i++)
            OutVtx[0].v[i] = V1.v[i] + c1 / (c1 - c2) * (V2.v[i] - V1.v[i]);
      }
      else if (c1 < 0.0 && c2 >= 0.0) {
         /* Case 4: P1 outside, P2 inside */
         Nout = 2;
         for (i = 0; i < 3; i++)
            OutVtx[0].v[i] = V2.v[i] + c2 / (c2 - c1) * (V1.v[i] - V2.v[i]);
         OutVtx[1] = V2;
      }
      else {
         Nout = 0;
      }
   }
   else {
      Nout = 0;
   }

   return (Nout);
}
/*********************************************************************/
void FindUnshadedAreas(struct SCType *S, vec3_t DirVecN)
{
   struct SilEdgeType *SilEdge = NULL, SwapEdge, *SE = NULL;
   struct SilVtxType *SilVtx  = NULL;
   struct SilVtxType *InVtx   = NULL;
   struct SilVtxType *ClipVtx = NULL;
   struct BodyType *B         = NULL;
   struct GeomType *G         = NULL;
   struct EdgeType *E         = NULL;
   struct PolyType *P         = NULL;
   double DoN1, DoN2;
   double ClipArea;
   double dA;
   long SilNe, SilNv, SilNc = 0, SilNin, Nout;
   long Ib, Ie, Je, Ipoly, i, Ic, Iout, Iv;
   vec3_t pn, ClipCtr, rA, ProjPtN, V1, V2;
   vec3_t PtA, PtB, PtC, dV1, dV2, V1xV2, OutVtx[2], DirVecB;
   vec4_t Bary;
   vec3_t Vtx[3];
   long B1, B2;

   ClipVtx = (struct SilVtxType *)calloc(1, sizeof(struct SilVtxType));

   /* .. Form Silhouette */
   /* TODO: This handles self-shadowing.  Extend to shadowing by other S/C in
    * same Orb */
   /* Form list of edges */
   SilNe = 0;
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B       = &S->B[Ib];
      G       = &Geom[B->GeomTag];
      DirVecB = MxV(B->CN, DirVecN);
      for (Ie = 0; Ie < G->Nedge; Ie++) {
         E    = &G->Edge[Ie];
         DoN1 = VoV(DirVecB, G->Poly[E->Poly1].Norm);
         DoN2 = VoV(DirVecB, G->Poly[E->Poly2].Norm);
         if ((DoN1 > 0.0 && DoN2 <= 0.0) || (DoN1 <= 0.0 && DoN2 > 0.0)) {
            if (SilNe == 0) {
               SilEdge =
                   (struct SilEdgeType *)calloc(1, sizeof(struct SilEdgeType));
            }
            else {
               SilEdge = (struct SilEdgeType *)realloc(
                   SilEdge, (SilNe + 1) * sizeof(struct SilEdgeType));
            }
            SE       = &SilEdge[SilNe];
            SE->Body = Ib;
            if (DoN1 > 0.0) {
               SE->Iv1    = E->Vtx1;
               SE->Iv2    = E->Vtx2;
               SE->PosV1B = G->V[E->Vtx1];
               SE->PosV2B = G->V[E->Vtx2];
            }
            else {
               SE->Iv1    = E->Vtx2;
               SE->Iv2    = E->Vtx1;
               SE->PosV1B = G->V[E->Vtx2];
               SE->PosV2B = G->V[E->Vtx1];
            }
            SE->PosV1N = MTxV(B->CN, SE->PosV1B);
            SE->PosV2N = MTxV(B->CN, SE->PosV2B);
            SE->PosV1N = VAddV_Elem(SE->PosV1N, B->pn);
            SE->PosV2N = VAddV_Elem(SE->PosV2N, B->pn);
            SilNe++;
         }
      }
   }
   /* Put list of edges in sequence */
   for (Ie = 0; Ie < SilNe - 1; Ie++) {
      for (Je = Ie + 1; Je < SilNe; Je++) {
         if (SilEdge[Je].Body == SilEdge[Ie].Body &&
             SilEdge[Je].Iv1 == SilEdge[Ie].Iv2) {
            memcpy(&SwapEdge, &SilEdge[Je], sizeof(struct SilEdgeType));
            memcpy(&SilEdge[Je], &SilEdge[Ie + 1], sizeof(struct SilEdgeType));
            memcpy(&SilEdge[Ie + 1], &SwapEdge, sizeof(struct SilEdgeType));
         }
      }
   }
   /* Form list of vertices, closing loops as needed */
   SilVtx         = (struct SilVtxType *)calloc(1, sizeof(struct SilVtxType));
   SilNv          = 1;
   SilVtx[0].Body = SilEdge[0].Body;
   SilVtx[0].PosB = SilEdge[0].PosV1B;
   SilVtx[0].PosN = SilEdge[0].PosV1N;
   for (Ie = 0; Ie < SilNe - 1; Ie++) {
      if (SilEdge[Ie + 1].Body == SilEdge[Ie].Body &&
          SilEdge[Ie + 1].Iv1 == SilEdge[Ie].Iv2) {
         SilVtx = (struct SilVtxType *)realloc(
             SilVtx, (SilNv + 1) * sizeof(struct SilVtxType));
         SilVtx[SilNv].Body = SilEdge[Ie].Body;
         SilVtx[SilNv].PosB = SilEdge[Ie].PosV2B;
         SilVtx[SilNv].PosN = SilEdge[Ie].PosV2N;
         SilNv++;
      }
      else {
         SilVtx = (struct SilVtxType *)realloc(
             SilVtx, (SilNv + 2) * sizeof(struct SilVtxType));
         SilVtx[SilNv].Body = SilEdge[Ie].Body;
         SilVtx[SilNv].PosB = SilEdge[Ie].PosV2B;
         SilVtx[SilNv].PosN = SilEdge[Ie].PosV2N;
         SilNv++;
         SilVtx[SilNv].Body = SilEdge[Ie + 1].Body;
         SilVtx[SilNv].PosB = SilEdge[Ie + 1].PosV1B;
         SilVtx[SilNv].PosN = SilEdge[Ie + 1].PosV1N;
         SilNv++;
      }
   }
   SilVtx = (struct SilVtxType *)realloc(SilVtx, (SilNv + 1) *
                                                     sizeof(struct SilVtxType));
   SilVtx[SilNv].Body = SilEdge[SilNe - 1].Body;
   SilVtx[SilNv].PosB = SilEdge[SilNe - 1].PosV2B;
   SilVtx[SilNv].PosN = SilEdge[SilNe - 1].PosV2N;
   SilNv++;

   /* .. Find unshaded areas, centroids */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B       = &S->B[Ib];
      G       = &Geom[B->GeomTag];
      DirVecB = MxV(B->CN, DirVecN);
      for (Ipoly = 0; Ipoly < G->Npoly; Ipoly++) {
         P = &G->Poly[Ipoly];
         if (VoV(P->Norm, DirVecB) > 0.0) {
            /* Transform Poly to N */
            for (i = 0; i < 3; i++) {
               Vtx[i] = MTxV(B->CN, G->V[P->V[i]]);
               Vtx[i] = VAddV_Elem(Vtx[i], B->pn);
            }

            /* Clip Silhouette against Poly */
            if (SilNv > 0) {
               free(InVtx);
               InVtx = (struct SilVtxType *)calloc(SilNv,
                                                   sizeof(struct SilVtxType));
               memcpy(InVtx, SilVtx, SilNv * sizeof(struct SilVtxType));
            }
            SilNin = SilNv;
            for (Iv = 0; Iv < 3; Iv++) {
               if (SilNin > 2) {
                  SilNc = 0;
                  for (Ie = 0; Ie < SilNin; Ie++) {
                     B1 = InVtx[Ie].Body;
                     B2 = InVtx[(Ie + 1) % SilNin].Body;
                     if (B1 == B2) {
                        /* Skip edges that jump between bodies */
                        V1   = InVtx[Ie].PosN;
                        V2   = InVtx[(Ie + 1) % SilNin].PosN;
                        Nout = ClipEdgeAgainstPlane(
                            V1, V2, Vtx[Iv], Vtx[(Iv + 1) % 3],
                            Vtx[(Iv + 2) % 3], DirVecN, OutVtx);
                        if (Nout > 0) {
                           ClipVtx = (struct SilVtxType *)realloc(
                               ClipVtx,
                               (SilNc + Nout) * sizeof(struct SilVtxType));
                           for (Iout = 0; Iout < Nout; Iout++) {
                              ClipVtx[SilNc + Iout].Body = B1;
                              ClipVtx[SilNc + Iout].PosN = OutVtx[Iout];
                              pn = VSubV_Elem(OutVtx[Iout], S->B[B1].pn);

                              ClipVtx[SilNc + Iout].PosB = MxV(S->B[B1].CN, pn);
                           }
                           SilNc += Nout;
                        }
                     }
                  }
                  if (SilNc > 0) {
                     free(InVtx);
                     InVtx = (struct SilVtxType *)calloc(
                         SilNc, sizeof(struct SilVtxType));
                     memcpy(InVtx, ClipVtx, SilNc * sizeof(struct SilVtxType));
                  }
                  SilNin = SilNc;
               }
            }

            /* Compute unshaded area, centroid in B */
            ClipArea = 0.0;
            ClipCtr  = VEC3_ZERO;
            rA       = VEC3_ZERO;
            if (SilNc > 2) {
               ProjectPointOntoTriangle(Vtx[0], Vtx[1], Vtx[2], DirVecN,
                                        ClipVtx[0].PosN, &ProjPtN, &Bary);
               ProjPtN = VSubV_Elem(ProjPtN, B->pn);
               PtA     = MxV(B->CN, ProjPtN);
               ProjectPointOntoTriangle(Vtx[0], Vtx[1], Vtx[2], DirVecN,
                                        ClipVtx[1].PosN, &ProjPtN, &Bary);
               ProjPtN = VSubV_Elem(ProjPtN, B->pn);
               PtB     = MxV(B->CN, ProjPtN);
               for (Ic = 2; Ic < SilNc; Ic++) {
                  ProjectPointOntoTriangle(Vtx[0], Vtx[1], Vtx[2], DirVecN,
                                           ClipVtx[Ic].PosN, &ProjPtN, &Bary);
                  ProjPtN = VSubV_Elem(ProjPtN, B->pn);
                  PtC     = MxV(B->CN, ProjPtN);
                  dV1     = VSubV_Elem(PtB, PtA);
                  dV2     = VSubV_Elem(PtC, PtA);

                  V1xV2     = VxV(dV1, dV2);
                  dA        = 0.5 * VoV(V1xV2, P->Norm); /* Signed Area */
                  ClipArea += dA;
                  for (i = 0; i < 3; i++)
                     rA.v[i] += (PtA.v[i] + PtB.v[i] + PtC.v[i]) / 3.0 * dA;
                  PtB = PtC;
               }
            }
            if (ClipArea > 0.0) {
               ClipCtr = SxV(1.0 / ClipArea, rA);
            }
            if (ClipArea < P->Area) {
               P->UnshadedArea = P->Area - ClipArea;
               for (i = 0; i < 3; i++)
                  P->UnshadedCtr.v[i] =
                      (P->Area * P->Centroid.v[i] - ClipArea * ClipCtr.v[i]) /
                      P->UnshadedArea;
            }
            else {
               P->UnshadedArea = 0.0;
            }
         }
      }
   }

   free(SilEdge);
   free(SilVtx);
   free(InVtx);
   free(ClipVtx);
}

/**********************************************************************/
void GravGradFrcTrq(struct WorldType *const worlds, struct OrbitType *const orb,
                    struct SCType *S)
{
   double r, Coef, rhatoc;
   long Ib;
   struct BodyType *B;
   struct WorldType *W;
   mat3x3_t GravGradN, CGG, GravGradB;
   vec3_t FrcN, FrcB;
   vec3_t Tb = VEC3_ZERO, Tn;
   vec3_t rhat, c, axIoa, rb, GGxI, GGxpn;

   S->gravTrqN = VEC3_ZERO;
   S->gravTrqB = VEC3_ZERO;

   if ((orb->Regime == ORB_ZERO || orb->Regime == ORB_FLIGHT) &&
       orb->PolyhedronGravityEnabled) {
      W = &worlds[orb->World];
      PolyhedronGravGrad(&Geom[W->GeomTag], W->Density, S->PosN, W->CWN,
                         &GravGradN);

      if (S->Nb == 1) {
         B = &S->B[0];
         /* GG torque */
         CGG       = MxM(B->CN, GravGradN);
         GravGradB = MxMT(CGG, B->CN);
         GGxI      = GravGradTimesInertia(GravGradB, B->I);
         B->Trq    = VAddV_Elem(B->Trq, GGxI);
      }
      else {
         for (Ib = 0; Ib < S->Nb; Ib++) {
            B = &S->B[Ib];
            /* GG torque */
            CGG       = MxM(B->CN, GravGradN);
            GravGradB = MxMT(CGG, B->CN);
            GGxI      = GravGradTimesInertia(GravGradB, B->I);
            B->Trq    = VAddV_Elem(B->Trq, GGxI);

            /* GG force */
            GGxpn   = MxV(GravGradN, B->pn);
            FrcN    = SxV(B->mass, GGxpn);
            FrcB    = MxV(B->CN, FrcN);
            B->FrcN = VAddV_Elem(B->FrcN, FrcN);
            B->gravPertAccN =
                VAddV_Elem(B->gravPertAccN, SxV(1.0 / B->mass, FrcN));
            B->FrcB = VAddV_Elem(B->FrcB, FrcB);
         }
      }
   }
   else {
      magvec3_t uv = UNITV(S->PosN);
      r            = uv.m;
      rhat         = uv.v;
      Coef         = orb->mu / (r * r * r);

      if (S->Nb == 1) {
         B = &S->B[0];
         /* GG torque */
         rb    = MxV(B->CN, rhat);
         axIoa = vxMov(rb, B->I);
         for (int i = 0; i < 3; i++) {
            Tb.v[i]     += 3.0 * Coef * axIoa.v[i];
            B->Trq.v[i] += 3.0 * Coef * axIoa.v[i];
         }
         Tn          = MTxV(B->CN, Tb);
         S->gravTrqN = VAddV_Elem(S->gravTrqN, Tn);
         S->gravTrqB = VAddV_Elem(S->gravTrqB, Tb);
      }
      else {
         rhat = UNITV(S->PosN).v;
         for (Ib = 0; Ib < S->Nb; Ib++) {
            B = &S->B[Ib];
            /* GG torque */
            rb    = MxV(B->CN, rhat);
            axIoa = vxMov(rb, B->I);
            for (int i = 0; i < 3; i++)
               B->Trq.v[i] += 3.0 * Coef * axIoa.v[i];

            /* GG force from Hughes, p. 246, eq. (56) */
            c      = SxV(B->mass, B->pn);
            rhatoc = VoV(rhat, c);
            for (int i = 0; i < 3; i++)
               FrcN.v[i] = -Coef * (c.v[i] - 3.0 * rhat.v[i] * rhatoc);
            FrcB    = MxV(B->CN, FrcN);
            B->FrcN = VAddV_Elem(B->FrcN, FrcN);
            B->gravPertAccN =
                VAddV_Elem(B->gravPertAccN, SxV(1.0 / B->mass, FrcN));
            B->FrcB = VAddV_Elem(B->FrcB, FrcB);
         }
      }
   }
}
/**********************************************************************/
vec3_t ThirdBodyGravForce(vec3_t p, vec3_t s, double mu, double mass)
{
   vec3_t Frc;
   double magp, mags, p3, s3;
   long j;

   magp = MAGV(p);
   mags = MAGV(s);
   p3   = magp * magp * magp;
   s3   = mags * mags * mags;
   for (j = 0; j < 3; j++)
      Frc.v[j] = mu * mass * (s.v[j] / s3 - p.v[j] / p3);
   return Frc;
}
/**********************************************************************/
void GravPertForce(struct WorldType *const worlds, struct OrbitType *const orbs,
                   struct SCType *S)
{
   struct OrbitType *O;
   vec3_t ph, p, s, FrcN;
   long Iw, Im;
   long OrbCenter, SecCenter;

   O = &orbs[S->RefOrb];
   if (O->Regime == ORB_CENTRAL) {
      OrbCenter = O->World;
      SecCenter = -1; /* Nonsense value */
   }
   else {
      OrbCenter = O->Body1;
      SecCenter = O->Body2;
   }
   struct WorldType *WCenter = &worlds[OrbCenter];
   /* Sun and all existing planets */
   for (Iw = SOL; Iw <= PLUTO; Iw++) {
      if (worlds[Iw].Exists && !(Iw == OrbCenter || Iw == SecCenter)) {
         ph      = VSubV_Elem(worlds[Iw].PosH, WCenter->PosH);
         p       = MxV(WCenter->CNH, ph);
         s       = VSubV_Elem(p, S->PosN);
         FrcN    = ThirdBodyGravForce(p, s, worlds[Iw].mu, S->mass);
         S->FrcN = VAddV_Elem(S->FrcN, FrcN);
         S->gravPertAccN =
             VAddV_Elem(S->gravPertAccN, SxV(1.0 / S->mass, FrcN));
      }
   }
   /* Moons of OrbCenter (but not SecCenter) */
   if (OrbCenter != SOL) {
      for (Im = 0; Im < WCenter->Nsat; Im++) {
         Iw = WCenter->Sat[Im];
         if (Iw != SecCenter) {
            p       = worlds[Iw].eph.PosN;
            s       = VSubV_Elem(p, S->PosN);
            FrcN    = ThirdBodyGravForce(p, s, worlds[Iw].mu, S->mass);
            S->FrcN = VAddV_Elem(S->FrcN, FrcN);
            S->gravPertAccN =
                VAddV_Elem(S->gravPertAccN, SxV(1.0 / S->mass, FrcN));
         }
      }
   }
   /* Moons of SecCenter */
   if (O->Regime == ORB_THREE_BODY) {
      for (Im = 0; Im < worlds[SecCenter].Nsat; Im++) {
         Iw      = worlds[SecCenter].Sat[Im];
         p       = worlds[Iw].eph.PosN;
         ph      = MTxV(worlds[SecCenter].CNH, p);
         p       = MxV(WCenter->CNH, ph);
         p       = VAddV_Elem(p, worlds[SecCenter].eph.PosN);
         s       = VSubV_Elem(p, S->PosN);
         FrcN    = ThirdBodyGravForce(p, s, worlds[Iw].mu, S->mass);
         S->FrcN = VAddV_Elem(S->FrcN, FrcN);
         S->gravPertAccN =
             VAddV_Elem(S->gravPertAccN, SxV(1.0 / S->mass, FrcN));
      }
   }

   struct SphereHarmType *gravModel = &WCenter->GravModel;
   FrcN            = SphericalHarmGravForce(gravModel->N, gravModel->M, WCenter,
                                            WCenter->CWN, S->mass, S->PosN);
   S->gravPertAccN = VAddV_Elem(S->gravPertAccN, SxV(1.0 / S->mass, FrcN));
   S->FrcN         = VAddV_Elem(S->FrcN, FrcN);
   /* else if O->CenterType == MINORBODY, use provided gravity model */
}
/**********************************************************************/
void AeroFrcTrq(JDType jd, struct WorldType *const worlds,
                struct OrbitType *const orb, struct SCType *S)
{

   vec3_t VrelN, VrelB, cp, Fb, Fn, Trq, Tn;
   double WoN, Coef, Area, PolyArea, WindSpeed;
   long Ib;
   long Ipoly;
   long OrbCenter;
   struct BodyType *B;
   struct GeomType *G;
   struct PolyType *P;

   S->aeroFrcN = VEC3_ZERO;
   S->aeroFrcB = VEC3_ZERO;
   S->aeroTrqN = VEC3_ZERO;
   S->aeroTrqB = VEC3_ZERO;

   OrbCenter = orb->World;

   /* .. Find Velocity Relative to Atmosphere, expressed in N */
   const double W_w = GetWorldW(jd, &worlds[OrbCenter]);

   VrelN.v[0]   = S->VelN.v[0] + W_w * S->PosN.v[1];
   VrelN.v[1]   = S->VelN.v[1] - W_w * S->PosN.v[0];
   VrelN.v[2]   = S->VelN.v[2];
   magvec3_t uv = UNITV(VrelN);
   WindSpeed    = uv.m;
   VrelN        = uv.v;

   if (AeroShadowsActive) {
      FindUnshadedAreas(S, VrelN);
   }

   /* .. Find Force and Torque on each Body, in that body's frame */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];

      /* Transform Rel Wind to B */
      VrelB = MxV(B->CN, VrelN);

      /* Find total projected area and cp for Body */
      Area = 0.0;
      cp   = VEC3_ZERO;
      G    = &Geom[B->GeomTag];
      for (Ipoly = 0; Ipoly < G->Npoly; Ipoly++) {
         P = &G->Poly[Ipoly];
         if (strncmp(Matl[P->Matl].Label, "SHADED",
                     6)) { /* Aero doesn't see shaded polys */
            WoN = VoV(VrelB, P->Norm);
            if (WoN > 0.0) {
               PolyArea  = WoN * P->UnshadedArea;
               Area     += PolyArea;
               for (int i = 0; i < 3; i++)
                  cp.v[i] += PolyArea * (P->UnshadedCtr.v[i] - B->cm.v[i]);
            }
         }
      }
      if (Area > 0.0)
         cp = SxV(1.0 / Area, cp);

      S->aeroProjectedArea = Area;

      /* Compute force and torque exerted on B */
      Coef = -0.5 * S->AtmoDensity * S->DragCoef * WindSpeed * WindSpeed * Area;
      Fb   = SxV(Coef, VrelB);
      Fn   = MTxV(B->CN, Fb);
      B->FrcN     = VAddV_Elem(B->FrcN, Fn);
      B->FrcB     = VAddV_Elem(B->FrcB, Fb);
      S->aeroFrcN = VAddV_Elem(S->aeroFrcN, Fn);
      S->aeroFrcB = VAddV_Elem(S->aeroFrcB, Fb);

      Trq         = VxV(cp, Fb);
      Tn          = MTxV(B->CN, Trq);
      B->Trq      = VAddV_Elem(B->Trq, Trq);
      S->aeroTrqN = VAddV_Elem(S->aeroTrqN, Tn);
      S->aeroTrqB = VAddV_Elem(S->aeroTrqB, Trq);
   }
}
/**********************************************************************/
void SolPressFrcTrq(struct SCType *S)
{
   long Ib, i;
   long Ipoly;
   double SoN, Coef, SolarPressure;
   vec3_t svb, r, Fb, Fn, Tb, Tn;
   struct BodyType *B;
   struct GeomType *G;
   struct PolyType *P;
   struct MatlType *M;
   double srpAreaSum;

   S->srpFrcN = VEC3_ZERO;
   S->srpFrcB = VEC3_ZERO;
   S->srpTrqN = VEC3_ZERO;
   S->srpTrqB = VEC3_ZERO;
   srpAreaSum = 0;

   if (!S->Eclipse) {
      /* Solar pressure is 4.5E-6 N/m^2 at Earth orbit radius, */
      /* and falls off as R^2                                  */
      SolarPressure = 4.5E-6 * 2.238E22 / VoV(S->PosH, S->PosH);

      if (SolPressShadowsActive)
         FindUnshadedAreas(S, S->svn);

      /* .. Find Force and Torque on each Body */
      for (Ib = 0; Ib < S->Nb; Ib++) {
         B = &S->B[Ib];
         G = &Geom[B->GeomTag];

         /* Find force and torque on each illuminated polygon */
         for (Ipoly = 0; Ipoly < G->Npoly; Ipoly++) {
            P = &G->Poly[Ipoly];
            if (strncmp(Matl[P->Matl].Label, "SHADED",
                        6)) { /* SRP doesn't see shaded polys */
               svb = MxV(B->CN, S->svn);
               SoN = VoV(svb, P->Norm);
               if (SoN > 0.0) {
                  M           = &Matl[P->Matl];
                  Coef        = -SolarPressure * P->UnshadedArea * SoN;
                  srpAreaSum += P->UnshadedArea * SoN;
                  for (i = 0; i < 3; i++) {
                     Fb.v[i] =
                         Coef * ((1.0 - M->SpecFrac) * svb.v[i] +
                                 2.0 * (M->SpecFrac * SoN + M->DiffFrac / 3.0) *
                                     P->Norm.v[i]);
                  }
                  r          = VSubV_Elem(P->UnshadedCtr, B->cm);
                  Tb         = VxV(r, Fb);
                  Tn         = MTxV(B->CN, Tb);
                  Fn         = MTxV(B->CN, Fb);
                  B->FrcN    = VAddV_Elem(B->FrcN, Fn);
                  B->FrcB    = VAddV_Elem(B->FrcB, Fb);
                  B->Trq     = VAddV_Elem(B->Trq, Tb);
                  S->srpFrcN = VAddV_Elem(S->srpFrcN, Fn);
                  S->srpFrcB = VAddV_Elem(S->srpFrcB, Fb);
                  S->srpTrqN = VAddV_Elem(S->srpTrqN, Tn);
                  S->srpTrqB = VAddV_Elem(S->srpTrqB, Tb);
               }
            }
         }
      }
   }

   S->srpProjectedArea = srpAreaSum;
}
/**********************************************************************/
void ResidualDipoleTrq(struct SCType *S)
{
   struct BodyType *B;
   vec3_t bvb, Trq;
   long Ib;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B      = &S->B[Ib];
      bvb    = MxV(B->CN, S->bvn);
      Trq    = VxV(B->EmbeddedDipole, bvb);
      B->Trq = VAddV_Elem(B->Trq, Trq);
   }
}
/**********************************************************************/
/* A point is fixed in Body B of Spacecraft S.                        */
/* Given its components in B, PosB, find its position and velocity    */
/* wrt R, expressed in N.                                             */
void FindPosVelR(struct SCType *S, struct BodyType *B, vec3_t PosB,
                 vec3_t *PosR, vec3_t *VelR)
{
   vec3_t PosCMB, PosCMN;
   vec3_t VelCMB, VelCMN;

   /* From cm of B */
   PosCMB = VSubV_Elem(PosB, B->cm);
   VelCMB = VxV(B->wn, PosCMB);

   /* Transform to N */
   PosCMN = MTxV(B->CN, PosCMB);
   VelCMN = MTxV(B->CN, VelCMB);

   /* From cm of SC, then from origin of R */
   for (int i = 0; i < 3; i++) {
      PosR->v[i] = PosCMN.v[i] + B->pn.v[i] + S->PosR.v[i];
      VelR->v[i] = VelCMN.v[i] + B->vn.v[i] + S->VelR.v[i];
   }
}
/**********************************************************************/
/* For each Poly in Body B, find force and torque due to contact with */
/* each poly in Region R.                                             */
void BodyRgnContactFrcTrq(struct SCType *S, long Ibody, struct RegionType *R)
{
   struct GeomType *Gb, *Gr;
   struct BodyType *B;
   struct PolyType *Pb, *Pr;
   struct EdgeType *E;
   vec3_t FrcN, FrcB, TrqB, rb, wxrb;
   vec3_t prn, vrn, pbrn, vbrn;
   vec3_t PosP, VelP, FrcP;
   mat3x3_t CPR, CPN;
   double ContactArea;
   double Dist, MinDist;
   vec3_t PosR, VelR, RelPosR, PosRR;
   static long HitPoly = 0;
   long OtherPoly;
   long Ib, Ie, Done;
   vec3_t Fn, Fb, Tb;

   B  = &S->B[Ibody];
   Gb = &Geom[B->GeomTag];
   Gr = &Geom[R->GeomTag];

   FrcN = VEC3_ZERO;
   FrcB = VEC3_ZERO;
   TrqB = VEC3_ZERO;

   /* Loop through all Polys (Pb) in Gb */
   for (Ib = 0; Ib < Gb->Npoly; Ib++) {
      Pb = &Gb->Poly[Ib];
      /* Use Centroid for proximity */
      /* Find position and velocity of Centroid wrt origin of R */
      FindPosVelR(S, B, Pb->Centroid, &PosR, &VelR);
      PosRR = MxV(R->CN, PosR);

      /* Find poly (Pr) in Gr closest to Pb */
      Done = 0;
      while (!Done) {
         Done    = 1;
         RelPosR = VSubV_Elem(PosRR, Gr->Poly[HitPoly].Centroid);
         MinDist = MAGV(RelPosR);
         /* Check neighboring polys */
         for (Ie = 0; Ie < 3; Ie++) {
            E = &Gr->Edge[Gr->Poly[HitPoly].E[Ie]];
            if (E->Poly1 >= 0 && E->Poly2 >= 0) { /* Screen edges of region */
               OtherPoly = (E->Poly1 == HitPoly ? E->Poly2 : E->Poly1);
               RelPosR   = VSubV_Elem(PosRR, Gr->Poly[OtherPoly].Centroid);
               Dist      = MAGV(RelPosR);
               if (Dist < MinDist) {
                  MinDist = Dist;
                  HitPoly = OtherPoly;
                  Done    = 0;
                  break;
               }
            }
         }
      }

      /* Interact with selected poly */
      Pr   = &Gr->Poly[HitPoly];
      prn  = MTxV(R->CN, Pr->Centroid);
      wxrb = VxV(R->wn, Pr->Centroid);
      vrn  = MTxV(R->CN, wxrb);

      pbrn        = VSubV_Elem(PosR, prn);
      vbrn        = VSubV_Elem(VelR, vrn);
      CPR.rows[0] = Pr->Uhat;
      CPR.rows[1] = Pr->Vhat;
      CPR.rows[2] = Pr->Norm;
      CPN         = MxM(CPR, R->CN);
      PosP        = MxV(CPN, pbrn);
      VelP        = MxV(CPN, vbrn);

      /* Find contact force */
      FrcP = VEC3_ZERO;
      if (PosP.z < Pb->radius) {
         if (PosP.z > 0.0) {
            ContactArea = (1.0 - PosP.v[2] / Pb->radius) * Pb->Area;
            FrcP.v[2]   = -R->DampCoef * VelP.v[2] * ContactArea;
            FrcP.v[0]   = 0.0;
            FrcP.v[1]   = 0.0;
         }
         else {
            ContactArea = (1.0 - PosP.v[2] / Pb->radius) * Pb->Area;
            FrcP.v[2] = -(R->ElastCoef * PosP.v[2] + R->DampCoef * VelP.v[2]) *
                        ContactArea;
            FrcP.v[0] = -R->FricCoef * FrcP.v[2] * VelP.v[0];
            FrcP.v[1] = -R->FricCoef * FrcP.v[2] * VelP.v[1];
         }
      }

      /* Transform into N, B frames */
      rb   = VSubV_Elem(Pb->Centroid, B->cm);
      Fn   = MTxV(CPN, FrcP);
      Fb   = MxV(B->CN, Fn);
      Tb   = VxV(rb, Fb);
      FrcN = VAddV_Elem(FrcN, Fn);
      FrcB = VAddV_Elem(FrcB, Fb);
      TrqB = VAddV_Elem(TrqB, Tb);
   }

   B->FrcN = VAddV_Elem(B->FrcN, FrcN);
   B->FrcB = VAddV_Elem(B->FrcB, FrcB);
   B->Trq  = VAddV_Elem(B->Trq, TrqB);
}
/**********************************************************************/
/* For each Poly in Body Ba, find force and torque due to contact     */
/* with each poly in Body Bb.                                         */
void BodyBodyContactFrcTrq(struct SCType *Sa, long Ibody, struct SCType *Sb,
                           long Jbody)
{
   struct GeomType *Ga, *Gb;
   struct BodyType *Ba, *Bb;
   struct PolyType *Pa, *Pb;
   struct OctreeType *Oa, *Ob;
   struct OctreeCellType *OCa, *OCb;
   vec3_t PosAN = VEC3_ZERO, PosBN = VEC3_ZERO;
   vec3_t VelAN = VEC3_ZERO, VelBN = VEC3_ZERO;
   vec3_t pan, ra, van;
   vec3_t pbn, rb, vbn;
   vec3_t FrcN = VEC3_ZERO, TrqA = VEC3_ZERO, TrqB = VEC3_ZERO;
   vec3_t FrcA, FrcB, NormAxis, NormAN, NormBN;
   vec3_t Fn, Fa, Ta, Fb, Tb, dx, dv;
   vec3_t TanAxis;
   long Ia, Ib;
   double hbar, r2, v2, r, v;
   double ContactArea;
   double NormDist, NormRate, NormFrc;
   long ExhaustedA, ExhaustedB, FoundOneInB;

   double PressCoef = 1.0E6; /* Point Solution, ad hoc */
   double ViscCoef  = 1.0E2;
   double FricCoef  = 0.5;

   Ba = &Sa->B[Ibody];
   Bb = &Sb->B[Jbody];
   Ga = &Geom[Ba->GeomTag];
   Gb = &Geom[Bb->GeomTag];
   Oa = Ga->Octree;
   Ob = Gb->Octree;

   /* Search Ga's Octree */
   ExhaustedA = 0;
   OCa        = &Oa->OctCell[0];
   while (!ExhaustedA) {
      FindPosVelR(Sa, Ba, OCa->center, &PosAN, &VelAN);

      /* Search Gb's Octree for interactions */
      OCb         = &Ob->OctCell[0];
      ExhaustedB  = 0;
      FoundOneInB = 0;

      while (!ExhaustedB) {
         FindPosVelR(Sb, Bb, OCb->center, &PosBN, &VelBN);
         dx = VSubV_Elem(PosAN, PosBN);
         if (MAGV(dx) <
             OCa->radius + OCb->radius) { /* OctCells are close enough */
            FoundOneInB = 1;
            for (Ia = 0; Ia < OCa->Npoly; Ia++) {
               Pa = &Ga->Poly[OCa->Poly[Ia]];
               FindPosVelR(Sa, Ba, Pa->Centroid, &pan, &van);
               ra = VSubV_Elem(Pa->Centroid, Ba->cm);
               for (Ib = 0; Ib < OCb->Npoly; Ib++) {
                  Pb = &Gb->Poly[OCb->Poly[Ib]];
                  FindPosVelR(Sb, Bb, Pb->Centroid, &pbn, &vbn);
                  rb = VSubV_Elem(Pb->Centroid, Bb->cm);

                  /* Use SPH concepts */
                  hbar        = 0.5 * (Pa->radius + Pb->radius);
                  ContactArea = 0.5 * (Pa->Area + Pb->Area);
                  dx          = VSubV_Elem(pan, pbn);
                  dv          = VSubV_Elem(van, vbn);
                  r2          = VoV(dx, dx);
                  v2          = VoV(dv, dv);
                  r           = sqrt(r2);
                  v           = sqrt(v2);

                  if (r < 2.0 * hbar) {
                     if (v * DTSIM > 0.1 * hbar) {
                        printf("Warning: CFL Violation in "
                               "BodyBodyContactFrcTrq.  "
                               "Suggest DTSIM < %lf sec.\n",
                               0.1 * hbar / v);
                     }

                     /* Find contact force exerted by Pb on Pa */
                     NormAN   = MTxV(Ba->CN, Pa->Norm);
                     NormBN   = MTxV(Bb->CN, Pb->Norm);
                     NormAxis = VSubV_Elem(NormBN, NormAN);
                     NormAxis = UNITV(NormAxis).v;
                     NormDist = VoV(dx, NormAxis);
                     NormRate = VoV(dv, NormAxis);
                     for (int i = 0; i < 3; i++)
                        TanAxis.v[i] = dv.v[i] - NormRate * NormAxis.v[i];
                     TanAxis = UNITV(TanAxis).v;
                     if (NormDist < 0.0) {
                        NormFrc = ContactArea *
                                  (-PressCoef * NormDist - ViscCoef * NormRate);
                        for (int i = 0; i < 3; i++)
                           Fn.v[i] = NormFrc * NormAxis.v[i] -
                                     FricCoef * NormFrc * TanAxis.v[i];

                        /* Transform into N, A, B frames */
                        Fa   = MxV(Ba->CN, Fn);
                        Ta   = VxV(ra, Fa);
                        Fb   = MxV(Bb->CN, Fn);
                        Tb   = VxV(rb, Fb);
                        FrcN = VAddV_Elem(FrcN, Fn);
                        TrqA = VAddV_Elem(TrqA, Ta);
                        TrqB = VAddV_Elem(TrqB, Tb);
                     }
                  }
               }
            }
            if (OCb->NextOnHit == 0)
               ExhaustedB = 1;
            else
               OCb = &Ob->OctCell[OCb->NextOnHit];
         }
         else if (OCb->NextOnMiss == 0)
            ExhaustedB = 1;
         else
            OCb = &Ob->OctCell[OCb->NextOnMiss];
      }
      if (FoundOneInB) {
         if (OCa->NextOnHit == 0)
            ExhaustedA = 1;
         else
            OCa = &Oa->OctCell[OCa->NextOnHit];
      }
      else if (OCa->NextOnMiss == 0)
         ExhaustedA = 1;
      else
         OCa = &Oa->OctCell[OCa->NextOnMiss];
   }

   FrcA              = MxV(Ba->CN, FrcN);
   FrcB              = MxV(Bb->CN, FrcN);
   Ba->SCContactFrcN = VAddV_Elem(Ba->SCContactFrcN, FrcN);
   Ba->SCContactFrcB = VAddV_Elem(Ba->SCContactFrcB, FrcA);
   Ba->SCContactTrq  = VAddV_Elem(Ba->SCContactTrq, TrqA);
   Bb->SCContactFrcN = VSubV_Elem(Bb->SCContactFrcN, FrcN);
   Bb->SCContactFrcB = VAddV_Elem(Bb->SCContactFrcB, FrcB);
   Bb->SCContactTrq  = VSubV_Elem(Bb->SCContactTrq, TrqB);
}
/**********************************************************************/
void SCContactFrcTrq(struct OrbitType *const orbs, struct SCType *scs,
                     const long sc_id)
{
   // TODO: split this between sc and not sc contacts. sc contact forces will
   // need to be outside the integrator.
   struct SCType *Sc;
   struct BodyType *Bi, *Bj;
   struct GeomType *Gi, *Gj;
   vec3_t dx, cmb, cmni, cmnj;
   long Isc, Ib, Jb;

   struct SCType *S    = &scs[sc_id];
   struct OrbitType *O = &orbs[S->RefOrb];

   /* .. Contact with other S/C */
   for (Isc = S->ID + 1; Isc < Nsc; Isc++) {
      // start from S->ID + 1 to avoid double counting forces
      Sc = &scs[Isc];
      /* Cheap S/Sc proximity checks */
      if (!Sc->Exists)
         continue;
      if (Sc->ID == S->ID)
         continue;
      if (orbs[Sc->RefOrb].World != O->World)
         continue;
      dx = VSubV_Elem(S->PosN, Sc->PosN);
      if (MAGV(dx) > 1.2 * (S->BBox.radius + Sc->BBox.radius))
         continue;

      /* Check each body of S vs each body of Sc */
      for (Ib = 0; Ib < S->Nb; Ib++) {
         Bi   = &S->B[Ib];
         Gi   = &Geom[Bi->GeomTag];
         cmb  = VSubV_Elem(Bi->cm, Gi->BBox.center);
         cmni = MTxV(Bi->CN, cmb);
         for (Jb = 0; Jb < Sc->Nb; Jb++) {
            /* Cheap Bi/Bj proximity checks */
            Bj   = &Sc->B[Jb];
            Gj   = &Geom[Bj->GeomTag];
            cmb  = VSubV_Elem(Bj->cm, Gj->BBox.center);
            cmnj = MTxV(Bj->CN, cmb);

            for (int i = 0; i < 3; i++)
               dx.v[i] = (S->PosN.v[i] + Bi->pn.v[i] - cmni.v[i]) -
                         (Sc->PosN.v[i] + Bj->pn.v[i] - cmnj.v[i]);
            if (MAGV(dx) > (Gi->BBox.radius + Gj->BBox.radius))
               continue;
            BodyBodyContactFrcTrq(S, Ib, Sc, Jb);
         }
      }
   }
}
/**********************************************************************/
void NonSCContactFrcTrq(struct OrbitType *const O, struct SCType *S)
{
   // TODO: split this between sc and not sc contacts. sc contact forces will
   // need to be outside the integrator.
   struct RegionType *R;
   vec3_t dx;
   long Ir, Ib;

   /* .. Contact with Regions */
   for (Ir = 0; Ir < Nrgn; Ir++) {
      R = &Rgn[Ir];
      /* Cheap proximity checks */
      if (!R->Exists)
         continue;
      if (R->World != O->World)
         continue;
      dx = VSubV_Elem(S->PosN, R->PosN);
      if (MAGV(dx) > S->BBox.radius + Geom[R->GeomTag].BBox.radius)
         continue;

      /* Check each body vs Region */
      for (Ib = 0; Ib < S->Nb; Ib++) {
         BodyRgnContactFrcTrq(S, Ib, R);
      }
   }
}
/**********************************************************************/
/* .. Resolve perturbation torque and force system to torques about   */
/* .. the S/C cm.  Express these in a special Sun-orbit frame to      */
/* .. determine actuator capacity requirements.                       */
void EnvTrq(struct SCType *S)
{
   long Ib;
   mat3x3_t CSN;
   vec3_t S1, S2, S3, rxF, TrqN, SumTrqN, TrqS;
   vec3_t TrqB, Hn, Hb;
   struct EnvTrqType *E;
   char envfilename[40];

   E = &S->EnvTrq;

   if (E->First) {
      E->First = 0;
      sprintf(envfilename, "EnvTrq%02ld.42", S->ID);
      E->envfile = FileOpen(OutPath, envfilename, "w");
   }

   /* Define S frame: s3 is orbit normal, s1 is orbit noon */
   S3          = VxV(S->PosN, S->VelN);
   S3          = UNITV(S3).v;
   S2          = VxV(S3, S->svn);
   S2          = UNITV(S2).v;
   S1          = VxV(S2, S3);
   CSN.rows[0] = S1;
   CSN.rows[1] = S2;
   CSN.rows[2] = S3;
   SumTrqN     = VEC3_ZERO;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      TrqN = MTxV(S->B[Ib].CN, S->B[Ib].Trq);
      rxF  = VxV(S->B[Ib].pn, S->B[Ib].FrcN);
      for (int i = 0; i < 3; i++)
         SumTrqN.v[i] += TrqN.v[i] + rxF.v[i];
   }
   TrqS = MxV(CSN, SumTrqN);

   for (int i = 0; i < 3; i++)
      E->Hs.v[i] += TrqS.v[i] * DTSIM;

   if (OutFlag) {
      /* Express Trq, H in B0 frame */
      TrqB = MxV(S->B[0].CN, SumTrqN);
      Hn   = MTxV(CSN, E->Hs);
      Hb   = MxV(S->B[0].CN, Hn);
      fprintf(E->envfile,
              "%18.12le %18.12le %18.12le %18.12le %18.12le %18.12le %18.12le "
              "%18.12le %18.12le %18.12le %18.12le %18.12le\n",
              TrqS.v[0], TrqS.v[1], TrqS.v[2], E->Hs.v[0], E->Hs.v[1],
              E->Hs.v[2], TrqB.v[0], TrqB.v[1], TrqB.v[2], Hb.v[0], Hb.v[1],
              Hb.v[2]);
   }
}
/**********************************************************************/
/*  This file contains perturbation torque and force models to apply  */
/*  as desired to each spacecraft.                                    */
/*  Remember that torques are expressed in the Body frame, but forces */
/*  are expressed in the N frame.                                     */
void Perturbations(JDType jd, struct WorldType *const worlds,
                   struct OrbitType *const O, struct SCType *S)
{
   // Only need up to 2 of the worlds, and that is only in the case of a 3
   // body orbit
   /* .. Gravity-Gradient Torques */
   if (GGActive)
      GravGradFrcTrq(worlds, O, S);

   /* .. Gravity Perturbation Forces */
   if (GravPertActive && O->Regime != ORB_N_BODY)
      GravPertForce(worlds, O, S);

   /* .. Aerodynamic Forces and Torques */
   if (AeroActive)
      AeroFrcTrq(jd, worlds, O, S);

   /* .. Solar Radiation Pressure Forces and Torques */
   if (SolPressActive)
      SolPressFrcTrq(S);

   /* .. Embedded Magnetic Dipole Torque */
   if (ResidualDipoleActive)
      ResidualDipoleTrq(S);

   /* .. Contact Forces and Torques */
   if (ContactActive) {
      NonSCContactFrcTrq(O, S);
      // Since Perturbations() is called inside the integrator, we don't want
      // to actually calculate the spacecraft/spacecraft contact forces
      // inside the integrator
      AddSCContactFrcTrq(S);
   }

   /* .. CFD Slosh Forces and Torques */
#ifdef _ENABLE_CFD_SLOSH_
   if (SloshActive)
      CfdSlosh(S);
   /* FakeCfdSlosh(S); */
#endif

   /* .. Find Momentum Accumulation for Actuator Sizing */
   if (ComputeEnvTrq)
      EnvTrq(S);
}

/* #ifdef __cplusplus
** }
** #endif
*/

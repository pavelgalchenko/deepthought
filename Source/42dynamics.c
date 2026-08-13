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
/*  Adjust body positions and velocities so that they are related to  */
/*  those of the spacecraft center of mass.                           */
void MotionConstraints(struct SCType *S)
{
   struct BodyType *B;
   vec3_t pcm = VEC3_ZERO, vcm = VEC3_ZERO;
   long Ib, i;

   /* Constrain Sum(mass*pn = 0.0), Sum(mass*vn = 0.0) */

   S->mass = 0.0;
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      for (i = 0; i < 3; i++) {
         pcm.v[i] += B->mass * B->pn.v[i];
         vcm.v[i] += B->mass * B->vn.v[i];
      }
      S->mass += B->mass;
   }
   pcm = SxV(1.0 / S->mass, pcm);
   vcm = SxV(1.0 / S->mass, vcm);

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B     = &S->B[Ib];
      B->pn = VSubV_Elem(B->pn, pcm);
      B->vn = VSubV_Elem(B->pn, vcm);
   }
   /* Adjust Dyn States corresponding to B[0].vn, B[0].pn */
   for (i = 0; i < 3; i++) {
      S->Dyn.u[S->Dyn.Nu - 3 + i] = S->B[0].vn.v[i];
      S->Dyn.x[S->Dyn.Nx - 3 + i] = S->B[0].pn.v[i];
   }
}
/**********************************************************************/
/*  Given body locations, attitudes, and mass properties,             */
/*  find SC mass center and inertia matrix.                           */
void SCMassProps(struct SCType *S)
{
   struct BodyType *B0, *B;
   vec3_t pnb, p;
   mat3x3_t pp, CI0, MOI;
   double p2;
   long i, j, Ib;

   B0 = &S->B[0];

   /* Locate SC.cm wrt B0 origin */
   pnb   = MxV(B0->CN, B0->pn);
   S->cm = VSubV_Elem(B0->cm, pnb);

   /* Compute composite inertia matrix, SC.I */
   S->I = B0->I;

   p  = MxV(B0->CN, B0->pn);
   p2 = VoV(p, p);
   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         pp.mat[i][j] = -p.v[i] * p.v[j];
      }
      pp.mat[i][i] += p2;
   }
   for (i = 0; i < 9; i++)
      S->I.flat[i] += B0->mass * pp.flat[i];

   for (Ib = 1; Ib < S->Nb; Ib++) {
      B   = &S->B[Ib];
      CI0 = MxMT(B->CN, B0->CN);
      p   = MxV(B0->CN, B->pn);
      MOI = PARAXIS(B->I, CI0, B->mass, p);
      for (i = 0; i < 3; i++)
         S->I.rows[i] = VAddV_Elem(S->I.rows[i], MOI.rows[i]);
   }
}
/**********************************************************************/
void MapJointStatesToStateVector(struct SCType *S)
{
   mat3x3_t CGoGi;
   quat_t qgogi;
   long i, Ig;
   struct JointType *G;
   struct DynType *D;

   D = &S->Dyn;

   /* Map in state variables */
   for (i = 0; i < 3; i++) {
      D->u[i]             = S->B[0].wn.v[i];
      D->u[D->Nu - 3 + i] = S->B[0].vn.v[i];
      D->x[D->Nx - 3 + i] = S->B[0].pn.v[i];
   }
   S->B[0].qn = UNITQ(S->B[0].qn);
   for (i = 0; i < 4; i++)
      D->x[i] = S->B[0].qn.q[i];

   for (Ig = 0; Ig < S->Ng; Ig++) {
      G = &S->G[Ig];
      if (G->IsSpherical) {
         CGoGi = A2C(G->RotSeq, G->Ang.v[0], G->Ang.v[1], G->Ang.v[2]);
         qgogi = C2Q(CGoGi);
         for (i = 0; i < 3; i++)
            D->u[G->Rotu0 + i] = G->AngRate.v[i];
         qgogi = UNITQ(qgogi);
         for (i = 0; i < 4; i++)
            D->x[G->Rotx0 + i] = qgogi.q[i];
      }
      else {
         for (i = 0; i < G->RotDOF; i++) {
            D->u[G->Rotu0 + i] = G->AngRate.v[i];
            D->x[G->Rotx0 + i] = G->Ang.v[i];
         }
      }
      for (i = 0; i < G->TrnDOF; i++) {
         D->u[G->Trnu0 + i] = G->PosRate.v[i];
         D->x[G->Trnx0 + i] = G->Pos.v[i];
      }
      G->CGoGi = A2C(G->RotSeq, G->Ang.v[0], G->Ang.v[1], G->Ang.v[2]);
      JointPartials(TRUE, G->IsSpherical, G->RotSeq, G->TrnSeq, G->Ang,
                    G->AngRate, &G->Gamma, &G->Gs, &G->Gds, G->PosRate,
                    &G->Delta, &G->Ds, &G->Dds);
      /* CTrqBo is constant for rigid body dynamics */
      /* It gets overwritten for flex */
      G->CTrqBo = G->CBoGo;
   }

   for (i = 0; i < S->Nw; i++) {
      D->h[i] = S->Whl[i].H;
      D->a[i] = S->Whl[i].Ang;
   }
}
/**********************************************************************/
void MapStateVectorToBodyStates(double *u, double *x, double *h, double *a,
                                double *uf, double *xf, struct SCType *S)
{
   vec3_t wi, ri, ro, wxr, wxri, wxro, xg;
   mat3x3_t CBfiBi, CBfoBo, CGoBfi;
   vec3_t wgon, wo, fvi, fvo;
   quat_t qfi, qfo;
   vec3_t vg, vgb, vgn;
   struct BodyType *Bi, *Bo, *B;
   struct JointType *G;
   struct WhlType *W;
   long Ig, i, j, Nu, Nx, Ng, If, Ib, Iw;

   Nu = S->Dyn.Nu;
   Nx = S->Dyn.Nx;
   Ng = S->Ng;
   for (i = 0; i < 3; i++) {
      S->B[0].wn.v[i]    = u[i];
      S->B[0].qn.qv.v[i] = x[i];
      S->B[0].vn.v[i]    = u[Nu - 3 + i];
      S->B[0].pn.v[i]    = x[Nx - 3 + i];
   }
   S->B[0].qn.qs = x[3];
   S->B[0].CN    = Q2C(S->B[0].qn);
   S->B[0].qn    = UNITQ(S->B[0].qn);

   if (S->FlexActive) {
      for (Ib = 0; Ib < S->Nb; Ib++) {
         B = &S->B[Ib];
         for (If = 0; If < B->Nf; If++) {
            B->xi[If]  = uf[B->f0 + If];
            B->eta[If] = xf[B->f0 + If];
         }
      }
   }

   for (Ig = 0; Ig < Ng; Ig++) {
      G  = &S->G[Ig];
      Bi = &S->B[G->Bin];
      Bo = &S->B[G->Bout];
      if (G->IsSpherical) {
         for (i = 0; i < 4; i++)
            G->q.q[i] = x[G->Rotx0 + i];
         G->q = UNITQ(G->q);
         for (i = 0; i < 3; i++) {
            G->AngRate.v[i] = u[G->Rotu0 + i];
         }
         G->CGoGi = Q2C(G->q);
         G->Ang   = C2A(G->RotSeq, G->CGoGi);
      }
      else {
         for (i = 0; i < G->RotDOF; i++) {
            G->AngRate.v[i] = u[G->Rotu0 + i];
            x[G->Rotx0 + i] = WrapTo2Pi(x[G->Rotx0 + i] + Pi) - Pi;
            G->Ang.v[i]     = x[G->Rotx0 + i];
         }
         if (G->RotDOF == 3) {
            if (fabs(G->Ang.y) > 1.5533) {
               printf("Warning:  Joint %ld is near gimbal lock.\n", Ig);
            }
         }
         G->CGoGi = A2C(G->RotSeq, G->Ang.v[0], G->Ang.v[1], G->Ang.v[2]);
      }
      for (i = 0; i < G->TrnDOF; i++) {
         G->PosRate.v[i] = u[G->Trnu0 + i];
         G->Pos.v[i]     = x[G->Trnx0 + i];
      }
      if (S->FlexActive) {
         /* Flex */
         G->FlexPosi    = VEC3_ZERO;
         G->FlexVeli    = VEC3_ZERO;
         G->FlexAngi    = VEC3_ZERO;
         G->FlexAngVeli = VEC3_ZERO;
         G->FlexPoso    = VEC3_ZERO;
         G->FlexVelo    = VEC3_ZERO;
         G->FlexAngo    = VEC3_ZERO;
         G->FlexAngVelo = VEC3_ZERO;
         for (i = 0; i < 3; i++) {
            for (If = 0; If < Bi->Nf; If++) {
               G->FlexPosi.v[i]    += G->PSIi[i][If] * Bi->eta[If];
               G->FlexVeli.v[i]    += G->PSIi[i][If] * Bi->xi[If];
               G->FlexAngi.v[i]    += G->THETAi[i][If] * Bi->eta[If];
               G->FlexAngVeli.v[i] += G->THETAi[i][If] * Bi->xi[If];
            }
            for (If = 0; If < Bo->Nf; If++) {
               G->FlexPoso.v[i]    += G->PSIo[i][If] * Bo->eta[If];
               G->FlexVelo.v[i]    += G->PSIo[i][If] * Bo->xi[If];
               G->FlexAngo.v[i]    += G->THETAo[i][If] * Bo->eta[If];
               G->FlexAngVelo.v[i] += G->THETAo[i][If] * Bo->xi[If];
            }
         }
         /* CN, qn */
         qfi.qs = 1.0;
         qfo.qs = 1.0;
         for (i = 0; i < 3; i++) {
            qfi.qv.v[i]  = sin(0.5 * G->FlexAngi.v[i]);
            qfo.qv.v[i]  = sin(0.5 * G->FlexAngo.v[i]);
            qfi.qs      -= qfi.qv.v[i] * qfi.qv.v[i];
            qfo.qs      -= qfo.qv.v[i] * qfo.qv.v[i];
         }
         qfi.qs    = sqrt(qfi.qs);
         qfo.qs    = sqrt(qfo.qs);
         CBfiBi    = Q2C(qfi);
         CBfoBo    = Q2C(qfo);
         G->CTrqBo = MTxM(G->CBoGo, CBfoBo);
         CGoBfi    = MxM(G->CGoGi, G->CGiBi);
         G->CTrqBi = MxM(CGoBfi, CBfiBi);
      }
      else
         G->CTrqBi = MxM(G->CGoGi, G->CGiBi);

      G->COI = MTxM(G->CTrqBo, G->CTrqBi);
      Bo->CN = MxM(G->COI, Bi->CN);
      Bo->qn = C2Q(Bo->CN);

      /* wn */
      wgon   = ADOT2W(G->IsSpherical, G->RotSeq, G->Ang, G->AngRate);
      Bo->wn = MxV(G->CBoGo, wgon);
      if (S->FlexActive) {
         Bo->wn = VSubV_Elem(Bo->wn, G->FlexAngVelo);
         wi     = VAddV_Elem(Bi->wn, G->FlexAngVeli);
         wo     = MxV(G->COI, wi);
      }
      else
         wo = MxV(G->COI, Bi->wn);

      Bo->wn = VAddV_Elem(Bo->wn, wo);

      /* pn, vn */
      xg = VEC3_ZERO;
      vg = VEC3_ZERO;
      for (i = 0; i < 3; i++) {
         for (j = 0; j < G->TrnDOF; j++) {
            xg.v[i] += G->Delta.mat[i][j] * G->Pos.v[j];
            vg.v[i] += G->Delta.mat[i][j] * G->PosRate.v[j];
         }
      }
      G->xb = MTxV(G->CGiBi, xg);
      G->ro = G->RigidRout;
      G->ri = VAddV_Elem(G->RigidRin, G->xb);

      G->xn = MTxV(Bi->CN, G->xb);
      ri    = MTxV(Bi->CN, G->ri);
      ro    = MTxV(Bo->CN, G->ro);
      for (i = 0; i < 3; i++)
         Bo->pn.v[i] = Bi->pn.v[i] + ri.v[i] - ro.v[i];
      /* vn */
      wxr  = VxV(Bi->wn, G->ri);
      wxri = MTxV(Bi->CN, wxr);
      wxr  = VxV(Bo->wn, G->ro);
      wxro = MTxV(Bo->CN, wxr);
      vgb  = MTxV(G->CGiBi, vg);
      vgn  = MTxV(Bi->CN, vgb);
      for (i = 0; i < 3; i++)
         Bo->vn.v[i] = Bi->vn.v[i] + wxri.v[i] + vgn.v[i] - wxro.v[i];
      if (S->FlexActive) {
         G->ri = VAddV_Elem(G->ri, G->FlexPosi);
         G->ro = VAddV_Elem(G->ro, G->FlexPoso);
         fvi   = MTxV(Bi->CN, G->FlexVeli);
         fvo   = MTxV(Bo->CN, G->FlexVelo);
         for (i = 0; i < 3; i++)
            Bo->vn.v[i] += fvi.v[i] - fvo.v[i];
      }
   }

   /* Wheels */
   for (Iw = 0; Iw < S->Nw; Iw++) {
      W      = &S->Whl[Iw];
      W->H   = h[Iw];
      W->w   = W->H / W->J;
      W->Ang = a[Iw];
   }
}
/**********************************************************************/
void BodyStatesToNodeStates(struct SCType *S)
{
   struct BodyType *B;
   struct NodeType *N;
   vec3_t vb, wxr;
   long Ib, In, If, i;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      for (In = 0; In < B->NumNodes; In++) {
         N          = &B->Node[In];
         N->PosB    = N->NomPosB;
         N->VelB    = VEC3_ZERO;
         N->qb      = QUAT_EYE;
         N->AngVelB = B->wn;

         if (S->FlexActive) {
            N->FlexPos     = VEC3_ZERO;
            N->FlexVel     = VEC3_ZERO;
            N->FlexAng     = VEC3_ZERO;
            N->FlexAngRate = VEC3_ZERO;
            for (i = 0; i < 3; i++) {
               for (If = 0; If < B->Nf; If++) {
                  N->FlexPos.v[i]     += N->PSI[i][If] * B->eta[If];
                  N->FlexVel.v[i]     += N->PSI[i][If] * B->xi[If];
                  N->FlexAng.v[i]     += N->THETA[i][If] * B->eta[If];
                  N->FlexAngRate.v[i] += N->THETA[i][If] * B->xi[If];
               }
            }
            N->PosB    = VAddV_Elem(N->PosB, N->FlexPos);
            N->VelB    = VAddV_Elem(N->VelB, N->FlexVel);
            N->AngVelB = VAddV_Elem(N->AngVelB, N->FlexAngRate);
            N->qb.qv   = SxV(0.5, N->FlexAng);
            N->qb.qs   = sqrt(1.0 - VoV(N->qb.qv, N->qb.qv));
         }
         vb       = MxV(B->CN, B->vn);
         N->PosCm = VSubV_Elem(N->PosB, B->cm);
         wxr      = VxV(N->AngVelB, N->PosCm);
         for (i = 0; i < 3; i++)
            N->VelB.v[i] += vb.v[i] + wxr.v[i];
         N->VelN = MTxV(B->CN, N->VelB);
      }
   }
}
/**********************************************************************/
void FindTotalAngMom(struct SCType *S)
{

   struct BodyType *B;
   struct WhlType *W;
   vec3_t Hb, Hn, mv, rxmv, Hwn;
   vec3_t Hwb = VEC3_ZERO;
   long Ib, Iwhl;

   /* Zero */
   S->Hvn = VEC3_ZERO;
   S->Hvb = VEC3_ZERO;

   /* Bodies */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B    = &S->B[Ib];
      Hb   = MxV(B->I, B->wn);
      Hb   = VAddV_Elem(Hb, B->EmbeddedMom);
      Hn   = MTxV(B->CN, Hb);
      mv   = SxV(B->mass, B->vn);
      rxmv = VxV(B->pn, mv);
      for (int i = 0; i < 3; i++)
         S->Hvn.v[i] += Hn.v[i] + rxmv.v[i];
   }

   /* Wheels */
   for (Iwhl = 0; Iwhl < S->Nw; Iwhl++) {
      W      = &S->Whl[Iwhl];
      Hwb    = SxV(W->H, W->A);
      Hwn    = MTxV(S->B[W->Body].CN, Hwb);
      S->Hvn = VAddV_Elem(S->Hvn, Hwn);
   }

   /* Express in B[0] frame */
   S->Hvb = MxV(S->B[0].CN, S->Hvn);
}
/**********************************************************************/
double FindTotalKineticEnergy(struct OrbitType *orbs, struct SCType *S)
{
   struct BodyType *B;
   struct WhlType *W;
   vec3_t Iw, mv;
   double KE = 0.0;
   long Ib, Iwhl;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B   = &S->B[Ib];
      Iw  = MxV(B->I, B->wn);
      mv  = SxV(B->mass, B->vn);
      KE += 0.5 * (VoV(B->wn, Iw) + VoV(B->vn, mv));
   }

   for (Iwhl = 0; Iwhl < S->Nw; Iwhl++) {
      W   = &S->Whl[Iwhl];
      KE += 0.5 * W->w * W->J * W->w;
   }

   if (orbs[S->RefOrb].Regime == ORB_ZERO) {
      KE += 0.5 * S->mass * VoV(S->VelN, S->VelN);
   }
   else if (orbs[S->RefOrb].Regime == ORB_FLIGHT) {
      KE += 0.5 * S->mass * VoV(S->VelR, S->VelR);
   }

   return (KE);
}
/**********************************************************************/
void FindBodyPathDCMs(struct SCType *S)
{
   struct DynType *D;
   struct JointType *G;
   long Ig, Bo, Bi, Gi;

   D = &S->Dyn;

   for (Ig = 0; Ig < S->Ng; Ig++) {
      G  = &S->G[Ig];
      Bo = G->Bout;
      Bi = G->Bin;

      D->BodyPathTable[Bo][Bi].Coi = G->COI;

      while (Bi != 0) {
         Gi = S->B[Bi].Gin;
         Bi = S->G[Gi].Bin;

         D->BodyPathTable[Bo][Bi].Coi = MxMT(S->B[Bo].CN, S->B[Bi].CN);
      }
   }
}
/**********************************************************************/
void FindPathVectors(struct SCType *S)
{
   struct DynType *D;
   struct JointType *G;
   vec3_t ri, ro;
   long Ig, Jg, Ia, Bi, Bo;

   D = &S->Dyn;

   for (Ig = 0; Ig < S->Ng; Ig++) {
      G  = &S->G[Ig];
      Bi = G->Bin;
      Bo = G->Bout;
      ri = MTxV(S->B[Bi].CN, G->ri);
      ro = MTxV(S->B[Bo].CN, G->ro);

      for (int i = 0; i < 3; i++)
         S->B[Bo].beta.v[i] = S->B[Bi].beta.v[i] + (ro.v[i] - ri.v[i]);

      for (Ia = 0; Ia < G->Nanc; Ia++) {
         Jg = G->Anc[Ia];
         for (int i = 0; i < 3; i++)
            D->JointPathTable[Bo][Jg].rho.v[i] =
                D->JointPathTable[Bi][Jg].rho.v[i] + (ro.v[i] - ri.v[i]);
      }
      D->JointPathTable[Bo][Ig].rho = ro;
   }
}
/**********************************************************************/
/*  PAngVel and IPAngVel                                              */
void FindPAngVel(struct SCType *S)
{
   struct DynType *D;
   struct BodyType *Bib;
   struct JointType *G;
   mat3x3_t CGo, CG, IC;
   long Ib, i, j, k, i0, j0;
   long Jb, Ig;

   D = &S->Dyn;

   /* PAngVel and IPAngVel */
   for (Ib = 1; Ib < S->Nb; Ib++) {
      Ig  = S->B[Ib].Gin;
      Bib = &S->B[Ib];
      G   = &S->G[Ig];
      i0  = 3 * Ib;
      j0  = G->Rotu0;
      CGo = MAT3X3_ZERO;
      for (i = 0; i < 3; i++) {
         for (j = 0; j < G->RotDOF; j++) {
            for (k = 0; k < 3; k++)
               CGo.mat[i][j] += G->CTrqBo.mat[k][i] * G->Gamma.mat[k][j];
         }
      }
      for (i = 0; i < 3; i++) {
         for (j = 0; j < G->RotDOF; j++) {
            D->PAngVel[i0 + i][j0 + j]  = CGo.mat[i][j];
            D->IPAngVel[i0 + i][j0 + j] = 0.0;
            for (k = 0; k < 3; k++)
               D->IPAngVel[i0 + i][j0 + j] += Bib->I.mat[i][k] * CGo.mat[k][j];
         }
      }
      Jb = G->Bin;
      while (Jb > 0) {
         Ig  = S->B[Jb].Gin;
         G   = &S->G[Ig];
         j0  = G->Rotu0;
         CGo = MAT3X3_ZERO;
         for (i = 0; i < 3; i++) {
            for (j = 0; j < G->RotDOF; j++) {
               for (k = 0; k < 3; k++) {
                  CGo.mat[i][j] += G->CTrqBo.mat[k][i] * G->Gamma.mat[k][j];
               }
            }
         }
         CG = MAT3X3_ZERO;
         for (i = 0; i < 3; i++) {
            for (j = 0; j < G->RotDOF; j++) {
               for (k = 0; k < 3; k++)
                  CG.mat[i][j] +=
                      D->BodyPathTable[Ib][Jb].Coi.mat[i][k] * CGo.mat[k][j];
            }
         }
         for (i = 0; i < 3; i++) {
            for (j = 0; j < G->RotDOF; j++) {
               D->PAngVel[i0 + i][j0 + j]  = CG.mat[i][j];
               D->IPAngVel[i0 + i][j0 + j] = 0.0;
               for (k = 0; k < 3; k++) {
                  D->IPAngVel[i0 + i][j0 + j] +=
                      Bib->I.mat[i][k] * CG.mat[k][j];
               }
            }
         }
         Jb = G->Bin;
      }
      IC = MxM(Bib->I, D->BodyPathTable[Ib][0].Coi);
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++) {
            D->PAngVel[i0 + i][j]  = D->BodyPathTable[Ib][0].Coi.mat[i][j];
            D->IPAngVel[i0 + i][j] = IC.mat[i][j];
         }
      }
   }
}
/**********************************************************************/
/* PVel and mPVel                                                     */
void FindPVel(struct SCType *S)
{
   struct DynType *D;
   struct BodyType *Bib, *Bjb;
   struct JointType *G;
   mat3x3_t RC, BC;
   mat3x3_t RCB, RCG;
   mat3x3_t CNG, CD;
   double m;
   long Ib, Jb, Ig, i, j, k, i0, j0;

   D = &S->Dyn;

   /* PVel and mPVel */
   for (Ib = 1; Ib < S->Nb; Ib++) {
      Bib = &S->B[Ib];
      i0  = 3 * Ib;
      m   = Bib->mass;
      Jb  = Ib;
      while (Jb > 0) {
         Bjb = &S->B[Jb];
         Ig  = Bjb->Gin;
         G   = &S->G[Ig];
         /* Rotation */
         j0  = G->Rotu0;
         RCB = VcrossMT(D->JointPathTable[Ib][Ig].rho, Bjb->CN);
         RC  = MxMT(RCB, G->CTrqBo);
         RCG = MAT3X3_ZERO;
         for (i = 0; i < 3; i++) {
            for (j = 0; j < G->RotDOF; j++) {
               for (k = 0; k < 3; k++) {
                  RCG.mat[i][j] += RC.mat[i][k] * G->Gamma.mat[k][j];
               }
            }
         }
         for (i = 0; i < 3; i++) {
            for (j = 0; j < G->RotDOF; j++) {
               D->PVel[i0 + i][j0 + j]  = RCG.mat[i][j];
               D->mPVel[i0 + i][j0 + j] = m * RCG.mat[i][j];
            }
         }
         /* Translation */
         j0  = G->Trnu0;
         CNG = MTxMT(S->B[G->Bin].CN, G->CGiBi);
         CD  = MAT3X3_ZERO;
         for (i = 0; i < 3; i++) {
            for (j = 0; j < G->TrnDOF; j++) {
               for (k = 0; k < 3; k++) {
                  CD.mat[i][j] += CNG.mat[i][k] * G->Delta.mat[k][j];
               }
            }
         }
         for (i = 0; i < 3; i++) {
            for (j = 0; j < G->TrnDOF; j++) {
               D->PVel[i0 + i][j0 + j]  = CD.mat[i][j];
               D->mPVel[i0 + i][j0 + j] = m * CD.mat[i][j];
            }
         }
         Jb = G->Bin;
      }
      /* First Column */
      BC = VcrossMT(Bib->beta, S->B[0].CN);
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++) {
            D->PVel[i0 + i][j]  = BC.mat[i][j];
            D->mPVel[i0 + i][j] = m * BC.mat[i][j];
         }
      }
   }
   /* Last Column populated in InitRigidDyn */
}
/**********************************************************************/
/*  PAngVelf and IPAngVelf                                            */
void FindPAngVelf(struct SCType *S)
{
   struct DynType *D;
   struct BodyType *Bib, *Bjb;
   struct JointType *Gi, *Go;
   long Ib, i, j, k, i0, j0;
   long Jb, Gin;

   D = &S->Dyn;

   /* PAngVelf and IPAngVelf */
   for (Ib = 1; Ib < S->Nb; Ib++) {
      i0  = 3 * Ib;
      Bib = &S->B[Ib];
      Gin = Bib->Gin;
      Gi  = &S->G[Gin];
      j0  = Bib->f0;
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Bib->Nf; j++) {
            D->PAngVelf[i0 + i][j0 + j] = -Gi->THETAo[i][j];
         }
      }
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Bib->Nf; j++) {
            D->IPAngVelf[i0 + i][j0 + j] = 0.0;
            for (k = 0; k < 3; k++)
               D->IPAngVelf[i0 + i][j0 + j] +=
                   Bib->I.mat[i][k] * D->PAngVelf[i0 + k][j0 + j];
         }
      }
      Jb = Gi->Bin;
      while (Jb > 0) {
         Go  = Gi;
         Bjb = &S->B[Jb];
         Gin = Bjb->Gin;
         Gi  = &S->G[Gin];
         j0  = Bjb->f0;
         for (i = 0; i < 3; i++) {
            for (j = 0; j < Bjb->Nf; j++) {
               D->PAngVelf[i0 + i][j0 + j] = 0.0;
               for (k = 0; k < 3; k++) {
                  D->PAngVelf[i0 + i][j0 + j] +=
                      D->BodyPathTable[Ib][Jb].Coi.mat[i][k] *
                      (Go->THETAi[k][j] - Gi->THETAo[k][j]);
               }
            }
         }
         for (i = 0; i < 3; i++) {
            for (j = 0; j < Bjb->Nf; j++) {
               D->IPAngVelf[i0 + i][j0 + j] = 0.0;
               for (k = 0; k < 3; k++) {
                  D->IPAngVelf[i0 + i][j0 + j] +=
                      Bib->I.mat[i][k] * D->PAngVelf[i0 + k][j0 + j];
               }
            }
         }
         Jb = Gi->Bin;
      }
      Bjb = &S->B[0];
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Bjb->Nf; j++) {
            D->PAngVelf[i0 + i][j] = 0.0;
            for (k = 0; k < 3; k++) {
               D->PAngVelf[i0 + i][j] +=
                   D->BodyPathTable[Ib][0].Coi.mat[i][k] * Gi->THETAi[k][j];
            }
         }
      }
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Bjb->Nf; j++) {
            D->IPAngVelf[i0 + i][j] = 0.0;
            for (k = 0; k < 3; k++) {
               D->IPAngVelf[i0 + i][j] +=
                   Bib->I.mat[i][k] * D->PAngVelf[i0 + k][j];
            }
         }
      }
   }
}
/**********************************************************************/
/* PVelf and mPVelf                                                   */
void FindPVelf(struct SCType *S)
{
   struct DynType *D;
   struct BodyType *Bib, *Bjb;
   struct JointType *Gi, *Go;
   mat3x3_t RCi, RCo;
   double m;
   long Ib, Jb, Gin, Gout, i, j, k, i0, j0;

   D = &S->Dyn;

   /* PVelf */
   for (Ib = 1; Ib < S->Nb; Ib++) {
      i0  = 3 * Ib;
      Bib = &S->B[Ib];
      j0  = Bib->f0;
      m   = Bib->mass;
      Gin = Bib->Gin;
      Gi  = &S->G[Gin];
      RCi = VcrossMT(D->JointPathTable[Ib][Gin].rho, Bib->CN);
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Bib->Nf; j++) {
            D->PVelf[i0 + i][j0 + j] = 0.0;
            for (k = 0; k < 3; k++) {
               D->PVelf[i0 + i][j0 + j] -= Bib->CN.mat[k][i] * Gi->PSIo[k][j] +
                                           RCi.mat[i][k] * Gi->THETAo[k][j];
            }
            D->mPVelf[i0 + i][j0 + j] = m * D->PVelf[i0 + i][j0 + j];
         }
      }
      Jb = Gi->Bin;

      while (Jb > 0) {
         Bjb  = &S->B[Jb];
         j0   = Bjb->f0;
         Gout = Gin;
         Go   = Gi;
         Gin  = Bjb->Gin;
         Gi   = &S->G[Gin];
         RCi  = VcrossMT(D->JointPathTable[Ib][Gin].rho, Bjb->CN);
         RCo  = VcrossMT(D->JointPathTable[Ib][Gout].rho, Bjb->CN);
         for (i = 0; i < 3; i++) {
            for (j = 0; j < Bjb->Nf; j++) {
               D->PVelf[i0 + i][j0 + j] = 0.0;
               for (k = 0; k < 3; k++) {
                  D->PVelf[i0 + i][j0 + j] +=
                      Bjb->CN.mat[k][i] * (Go->PSIi[k][j] - Gi->PSIo[k][j]) +
                      RCo.mat[i][k] * Go->THETAi[k][j] -
                      RCi.mat[i][k] * Gi->THETAo[k][j];
               }
               D->mPVelf[i0 + i][j0 + j] = m * D->PVelf[i0 + i][j0 + j];
            }
         }
         Jb = Gi->Bin;
      }

      Bjb = &S->B[0];
      RCi = VcrossMT(D->JointPathTable[Ib][Gin].rho, Bjb->CN);
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Bjb->Nf; j++) {
            D->PVelf[i0 + i][j] = 0.0;
            for (k = 0; k < 3; k++) {
               D->PVelf[i0 + i][j] += Bjb->CN.mat[k][i] * Gi->PSIi[k][j] +
                                      RCi.mat[i][k] * Gi->THETAi[k][j];
            }
            D->mPVelf[i0 + i][j] = m * D->PVelf[i0 + i][j];
         }
      }
   }
}
/**********************************************************************/
/* Add (c+Pf*eta)xPVel to IPAngVel                                    */
void AugmentIPAngVel(struct SCType *S)
{
   struct DynType *D;
   struct BodyType *Bib, *Bjb;
   struct JointType *G;
   mat3x3_t cplusPetaN;
   long Ib, i, j, k, i0, j0;
   long Jb, Ig;

   D = &S->Dyn;

   /* IPAngVel */
   for (Ib = 1; Ib < S->Nb; Ib++) {
      Ig         = S->B[Ib].Gin;
      Bib        = &S->B[Ib];
      G          = &S->G[Ig];
      i0         = 3 * Ib;
      j0         = G->Rotu0;
      cplusPetaN = MxM(Bib->cplusPeta, Bib->CN);
      for (i = 0; i < 3; i++) {
         for (j = 0; j < G->RotDOF; j++) {
            for (k = 0; k < 3; k++) {
               D->IPAngVel[i0 + i][j0 + j] +=
                   cplusPetaN.mat[i][k] * D->PVel[i0 + k][j0 + j];
            }
         }
      }
      Jb = G->Bin;
      while (Jb > 0) {
         Bjb = &S->B[Jb];
         Ig  = Bjb->Gin;
         G   = &S->G[Ig];
         j0  = G->Rotu0;
         for (i = 0; i < 3; i++) {
            for (j = 0; j < G->RotDOF; j++) {
               for (k = 0; k < 3; k++) {
                  D->IPAngVel[i0 + i][j0 + j] +=
                      cplusPetaN.mat[i][k] * D->PVel[i0 + k][j0 + j];
               }
            }
         }
         j0 = G->Trnu0;
         for (i = 0; i < 3; i++) {
            for (j = 0; j < G->TrnDOF; j++) {
               for (k = 0; k < 3; k++) {
                  D->IPAngVel[i0 + i][j0 + j] +=
                      cplusPetaN.mat[i][k] * D->PVel[i0 + k][j0 + j];
               }
            }
         }
         Jb = G->Bin;
      }
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++) {
            for (k = 0; k < 3; k++) {
               D->IPAngVel[i0 + i][j] +=
                   cplusPetaN.mat[i][k] * D->PVel[i0 + k][j];
            }
         }
      }
      /* Last column */
      j0 = D->Nu - 3;
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++) {
            D->IPAngVel[i0 + i][j0 + j] = cplusPetaN.mat[i][j];
         }
      }
   }
}
/**********************************************************************/
/* Add -(c+Pf*eta)xPAngVel to mPVel                                   */
void AugmentMPVel(struct SCType *S)
{
   struct DynType *D;
   struct BodyType *Bib, *Bjb;
   struct JointType *G;
   mat3x3_t CcplusPeta;
   long Ib, Jb, Ig, i, j, k, i0, j0;

   D = &S->Dyn;

   /* mPVel */
   for (Ib = 1; Ib < S->Nb; Ib++) {
      Bib        = &S->B[Ib];
      i0         = 3 * Ib;
      Jb         = Ib;
      CcplusPeta = MTxM(Bib->CN, Bib->cplusPeta);
      while (Jb > 0) {
         Bjb = &S->B[Jb];
         Ig  = Bjb->Gin;
         G   = &S->G[Ig];
         j0  = G->Rotu0;
         for (i = 0; i < 3; i++) {
            for (j = 0; j < G->RotDOF; j++) {
               for (k = 0; k < 3; k++)
                  D->mPVel[i0 + i][j0 + j] -=
                      CcplusPeta.mat[i][k] * D->PAngVel[i0 + k][j0 + j];
            }
         }
         Jb = G->Bin;
      }
      /* First Column */
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++) {
            for (k = 0; k < 3; k++)
               D->mPVel[i0 + i][j] -=
                   CcplusPeta.mat[i][k] * D->PAngVel[i0 + k][j];
         }
      }
   }
}
/**********************************************************************/
/* Add (c+Pf*eta)xPVelf + (Hf+Qf*eta) to IPAngVelf                    */
void AugmentIPAngVelf(struct SCType *S)
{
   struct DynType *D;
   struct BodyType *Bib, *Bjb;
   struct JointType *Gi;
   mat3x3_t cplusPetaN;
   long Ib, i, j, k, i0, j0;
   long Jb, Gin;

   D = &S->Dyn;

   /* Add (Hf+Qf*eta) */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      i0  = 3 * Ib;
      Bib = &S->B[Ib];
      j0  = Bib->f0;
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Bib->Nf; j++) {
            D->IPAngVelf[i0 + i][j0 + j] += Bib->HplusQeta[i][j];
         }
      }
   }

   if (S->RefPt == REFPT_JOINT) {
      /* Add (c+Pf*eta)xPVelf */
      for (Ib = 1; Ib < S->Nb; Ib++) {
         i0         = 3 * Ib;
         Bib        = &S->B[Ib];
         Gin        = Bib->Gin;
         Gi         = &S->G[Gin];
         j0         = Bib->f0;
         cplusPetaN = MxM(Bib->cplusPeta, Bib->CN);
         for (i = 0; i < 3; i++) {
            for (j = 0; j < Bib->Nf; j++) {
               D->IPAngVelf[i0 + i][j0 + j] += Bib->HplusQeta[i][j];
               for (k = 0; k < 3; k++) {
                  D->IPAngVelf[i0 + i][j0 + j] +=
                      cplusPetaN.mat[i][k] * D->PVelf[i0 + k][j0 + j];
               }
            }
         }

         Jb = Gi->Bin;
         while (Jb > 0) {
            Bjb = &S->B[Jb];
            Gin = Bjb->Gin;
            Gi  = &S->G[Gin];
            j0  = Bjb->f0;
            for (i = 0; i < 3; i++) {
               for (j = 0; j < Bjb->Nf; j++) {
                  for (k = 0; k < 3; k++) {
                     D->IPAngVelf[i0 + i][j0 + j] +=
                         cplusPetaN.mat[i][k] * D->PVelf[i0 + k][j0 + j];
                  }
               }
            }
            Jb = Gi->Bin;
         }

         Bjb = &S->B[0];
         for (i = 0; i < 3; i++) {
            for (j = 0; j < Bjb->Nf; j++) {
               for (k = 0; k < 3; k++) {
                  D->IPAngVelf[i0 + i][j] +=
                      cplusPetaN.mat[i][k] * D->PVelf[i0 + k][j];
               }
            }
         }
      }
   }
}
/**********************************************************************/
/* Add -[(c+Pf*eta)xPAngVelf - Pf] to mPVelf                          */
void AugmentMPVelf(struct SCType *S)
{
   struct DynType *D;
   struct BodyType *Bib, *Bjb;
   struct JointType *Gi;
   mat3x3_t CcplusPeta;
   long Ib, Jb, Gin, i, j, k, i0, j0;
   long Nfi, Nfj;

   D = &S->Dyn;

   /* B[0] sees only Pf term */
   Bib = &S->B[0];
   Nfi = Bib->Nf;
   for (i = 0; i < 3; i++) {
      for (j = 0; j < Nfi; j++) {
         D->mPVelf[i][j] += Bib->CnbP[i][j];
      }
   }

   /* Other bodies see both terms */
   for (Ib = 1; Ib < S->Nb; Ib++) {
      i0         = 3 * Ib;
      Bib        = &S->B[Ib];
      j0         = Bib->f0;
      Gin        = Bib->Gin;
      Gi         = &S->G[Gin];
      Nfi        = Bib->Nf;
      CcplusPeta = MTxM(Bib->CN, Bib->cplusPeta);
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Nfi; j++) {
            D->mPVelf[i0 + i][j0 + j] += Bib->CnbP[i][j];
            for (k = 0; k < 3; k++) {
               D->mPVelf[i0 + i][j0 + j] -=
                   CcplusPeta.mat[i][k] * D->PAngVelf[i0 + k][j0 + j];
            }
         }
      }
      Jb = Gi->Bin;

      while (Jb > 0) {
         Bjb = &S->B[Jb];
         j0  = Bjb->f0;
         Gin = Bjb->Gin;
         Gi  = &S->G[Gin];
         Nfj = Bjb->Nf;
         for (i = 0; i < 3; i++) {
            for (j = 0; j < Nfj; j++) {
               for (k = 0; k < 3; k++) {
                  D->mPVelf[i0 + i][j0 + j] -=
                      CcplusPeta.mat[i][k] * D->PAngVelf[i0 + k][j0 + j];
               }
            }
         }
         Jb = Gi->Bin;
      }

      Bjb = &S->B[0];
      Nfj = Bjb->Nf;
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Nfj; j++) {
            for (k = 0; k < 3; k++) {
               D->mPVelf[i0 + i][j] -=
                   CcplusPeta.mat[i][k] * D->PAngVelf[i0 + k][j];
            }
         }
      }
   }
}
/**********************************************************************/
void FindHplusQetaPAngVelf(struct SCType *S)
{
   struct DynType *D;
   struct BodyType *Bib, *Bjb;
   struct JointType *Gi;
   double **HpQe;
   long Ib, i, j, i0, j0;
   long Jb, Gin;
   long Nfi, Nfj;

   D = &S->Dyn;

   /* (HplusQeta)^T*PAngVelf */
   for (Ib = 1; Ib < S->Nb; Ib++) {
      Bib  = &S->B[Ib];
      Nfi  = Bib->Nf;
      Gin  = Bib->Gin;
      Gi   = &S->G[Gin];
      i0   = Bib->f0;
      HpQe = Bib->HplusQeta;
      for (i = 0; i < Nfi; i++) {
         for (j = 0; j < Nfi; j++) {
            D->HplusQetaPAngVelf[i0 + i][i0 + j] =
                HpQe[0][i] * D->PAngVelf[3 * Ib][i0 + j] +
                HpQe[1][i] * D->PAngVelf[3 * Ib + 1][i0 + j] +
                HpQe[2][i] * D->PAngVelf[3 * Ib + 2][i0 + j];
         }
      }
      Jb = Gi->Bin;
      while (Jb > 0) {
         Bjb = &S->B[Jb];
         Gin = Bjb->Gin;
         Gi  = &S->G[Gin];
         j0  = Bjb->f0;
         Nfj = Bjb->Nf;
         for (i = 0; i < Nfi; i++) {
            for (j = 0; j < Nfj; j++) {
               D->HplusQetaPAngVelf[i0 + i][j0 + j] =
                   HpQe[0][i] * D->PAngVelf[3 * Ib][j0 + j] +
                   HpQe[1][i] * D->PAngVelf[3 * Ib + 1][j0 + j] +
                   HpQe[2][i] * D->PAngVelf[3 * Ib + 2][j0 + j];
            }
         }
         Jb = Gi->Bin;
      }

      Bjb = &S->B[0];
      Nfj = Bjb->Nf;
      for (i = 0; i < Nfi; i++) {
         for (j = 0; j < Nfj; j++) {
            D->HplusQetaPAngVelf[i0 + i][j] =
                HpQe[0][i] * D->PAngVelf[3 * Ib][j] +
                HpQe[1][i] * D->PAngVelf[3 * Ib + 1][j] +
                HpQe[2][i] * D->PAngVelf[3 * Ib + 2][j];
         }
      }
   }
}
/**********************************************************************/
/* Compute Pf^T*CBN*PVelf                                             */
void FindPCPVelf(struct SCType *S)
{
   struct DynType *D;
   struct BodyType *Bib, *Bjb;
   struct JointType *Gi;
   double **CP;
   long Ib, Jb, Gin, i, j, i0, j0;
   long Nfi, Nfj;

   D = &S->Dyn;

   /* PCPVelf */
   for (Ib = 1; Ib < S->Nb; Ib++) {
      Bib = &S->B[Ib];
      CP  = Bib->CnbP;
      i0  = Bib->f0;
      Nfi = Bib->Nf;
      for (i = 0; i < Nfi; i++) {
         for (j = 0; j < Nfi; j++) {
            D->PCPVelf[i0 + i][i0 + j] =
                CP[0][i] * D->PVelf[3 * Ib][i0 + j] +
                CP[1][i] * D->PVelf[3 * Ib + 1][i0 + j] +
                CP[2][i] * D->PVelf[3 * Ib + 2][i0 + j];
         }
      }
      Gin = Bib->Gin;
      Gi  = &S->G[Gin];
      Jb  = Gi->Bin;

      while (Jb > 0) {
         Bjb = &S->B[Jb];
         Gin = Bjb->Gin;
         Gi  = &S->G[Gin];
         j0  = Bjb->f0;
         Nfj = Bjb->Nf;
         for (i = 0; i < Nfi; i++) {
            for (j = 0; j < Nfj; j++) {
               D->PCPVelf[i0 + i][j0 + j] =
                   CP[0][i] * D->PVelf[3 * Ib][j0 + j] +
                   CP[1][i] * D->PVelf[3 * Ib + 1][j0 + j] +
                   CP[2][i] * D->PVelf[3 * Ib + 2][j0 + j];
            }
         }
         Jb = Gi->Bin;
      }
      Bjb = &S->B[0];
      Nfj = Bjb->Nf;
      for (i = 0; i < Nfi; i++) {
         for (j = 0; j < Nfj; j++) {
            D->PCPVelf[i0 + i][j] = CP[0][i] * D->PVelf[3 * Ib][j] +
                                    CP[1][i] * D->PVelf[3 * Ib + 1][j] +
                                    CP[2][i] * D->PVelf[3 * Ib + 2][j];
         }
      }
   }
}
/**********************************************************************/
void FindAlphaR(struct SCType *S)
{
   struct JointType *G;
   struct BodyType *Bi, *Bo;
   vec3_t CGs, CGds, wxGs, wxFo, wxFi, CwxFi, CAlphaR;
   long Ig;

   for (Ig = 0; Ig < S->Ng; Ig++) {
      G       = &S->G[Ig];
      Bi      = &S->B[G->Bin];
      Bo      = &S->B[G->Bout];
      CGs     = MTxV(G->CTrqBo, G->Gs);
      CGds    = MTxV(G->CTrqBo, G->Gds);
      wxGs    = VxV(Bo->wn, CGs);
      CAlphaR = MxV(G->COI, Bi->AlphaR);
      for (int i = 0; i < 3; i++)
         Bo->AlphaR.v[i] = CAlphaR.v[i] + CGds.v[i] + wxGs.v[i];
   }

   if (S->FlexActive) {
      for (Ig = 0; Ig < S->Ng; Ig++) {
         G     = &S->G[Ig];
         Bi    = &S->B[G->Bin];
         Bo    = &S->B[G->Bout];
         wxFo  = VxV(Bo->wn, G->FlexAngVelo);
         wxFi  = VxV(Bi->wn, G->FlexAngVeli);
         CwxFi = MxV(G->COI, wxFi);
         for (int i = 0; i < 3; i++)
            Bo->AlphaR.v[i] += CwxFi.v[i] - wxFo.v[i];
      }
   }
}
/**********************************************************************/
void FindAccR(struct SCType *S)
{
   struct JointType *G;
   struct BodyType *Bi, *Bo;
   vec3_t wxr, wxwxr, Cwri, Cwro;
   vec3_t Dsb, wxDsb, wxDsn;
   vec3_t axr, Caxri, Caxro;
   vec3_t wxv, Cwxvi, Cwxvo;
   long Ig, i;

   for (Ig = 0; Ig < S->Ng; Ig++) {
      G  = &S->G[Ig];
      Bi = &S->B[G->Bin];
      Bo = &S->B[G->Bout];

      wxr   = VxV(Bi->wn, G->ri);
      wxwxr = VxV(Bi->wn, wxr);
      Cwri  = MTxV(Bi->CN, wxwxr);

      wxr   = VxV(Bo->wn, G->ro);
      wxwxr = VxV(Bo->wn, wxr);
      Cwro  = MTxV(Bo->CN, wxwxr);

      axr   = VxV(Bo->AlphaR, G->ro);
      Caxro = MTxV(Bo->CN, axr);

      axr   = VxV(Bi->AlphaR, G->ri);
      Caxri = MTxV(Bi->CN, axr);

      Dsb   = MTxV(G->CGiBi, G->Ds);
      wxDsb = VxV(Bi->wn, Dsb);
      wxDsn = MTxV(Bi->CN, wxDsb);

      for (i = 0; i < 3; i++)
         Bo->AccR.v[i] = Bi->AccR.v[i] + Cwri.v[i] - Cwro.v[i] + Caxri.v[i] -
                         Caxro.v[i] + 2.0 * wxDsn.v[i];
   }

   if (S->FlexActive) {
      for (Ig = 0; Ig < S->Ng; Ig++) {
         G  = &S->G[Ig];
         Bi = &S->B[G->Bin];
         Bo = &S->B[G->Bout];

         wxv   = VxV(Bo->wn, G->FlexVelo);
         Cwxvo = MTxV(Bo->CN, wxv);

         wxv   = VxV(Bi->wn, G->FlexVeli);
         Cwxvi = MTxV(Bi->CN, wxv);

         for (i = 0; i < 3; i++)
            Bo->AccR.v[i] += 2.0 * (Cwxvi.v[i] - Cwxvo.v[i]);
      }
   }
}
/**********************************************************************/
/*  Find Peta, cplusPeta, HplusQeta, CnbP for each body          */
void FindFlexTerms(struct SCType *S)
{
   struct BodyType *B;
   long Nf, Ib, i, j, k;
   vec3_t cPe;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B  = &S->B[Ib];
      Nf = B->Nf;
      if (Nf < 1)
         continue;
      B->Peta = VEC3_ZERO;
      memset(B->CnbP[0], 0, 3 * Nf * sizeof(double));

      for (i = 0; i < 3; i++) {
         for (k = 0; k < Nf; k++)
            B->Peta.v[i] += B->Pf[i][k] * B->eta[k];

         cPe.v[i] = B->c.v[i] + B->Peta.v[i];
         for (j = 0; j < Nf; j++)
            for (k = 0; k < 3; k++)
               B->CnbP[i][j] += B->CN.mat[k][i] * B->Pf[k][j];
      }
      B->cplusPeta = V2CrossM(cPe);
   }

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B  = &S->B[Ib];
      Nf = B->Nf;
      if (Nf < 1)
         continue;
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Nf; j++) {
            B->HplusQeta[i][j] = B->Hf[i][j];
         }
      }
   }
   if (S->IncludeSecondOrderFlexTerms) {
      for (Ib = 0; Ib < S->Nb; Ib++) {
         B  = &S->B[Ib];
         Nf = B->Nf;
         if (Nf < 1)
            continue;
         for (i = 0; i < 3; i++) {
            for (j = 0; j < Nf; j++) {
               for (k = 0; k < Nf; k++) {
                  B->HplusQeta[i][j] +=
                      B->Qf[IDX3(i, j, k, Nf, Nf)] * B->eta[k];
               }
            }
         }
      }
   }
}
/**********************************************************************/
void FindInertiaTrq(struct SCType *S)
{
   struct BodyType *B;
   struct WhlType *W;
   vec3_t H, wxH, Ia;
   vec3_t cPexa;
   vec3_t CAccR;
   long Ib, Iw;

   /* -I*AlphaR - wxH for all bodies */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      H = MxV(B->I, B->wn);
      for (int i = 0; i < 3; i++)
         H.v[i] += B->EmbeddedMom.v[i];
      wxH = VxV(B->wn, H);
      Ia  = MxV(B->I, B->AlphaR);
      for (int i = 0; i < 3; i++)
         B->InertiaTrq.v[i] = -Ia.v[i] - wxH.v[i];
   }

   for (Iw = 0; Iw < S->Nw; Iw++) {
      W = &S->Whl[Iw];
      B = &S->B[W->Body];
      H = SxV(W->H, W->A);

      wxH           = VxV(B->wn, H);
      B->InertiaTrq = VSubV_Elem(B->InertiaTrq, wxH);
   }

   if (S->FlexActive && S->RefPt == REFPT_JOINT) {
      /* -(c + Pf*eta) x AccR */
      for (Ib = 0; Ib < S->Nb; Ib++) {
         B             = &S->B[Ib];
         CAccR         = MxV(B->CN, B->AccR);
         cPexa         = MxV(B->cplusPeta, CAccR);
         B->InertiaTrq = VSubV_Elem(B->InertiaTrq, cPexa);
      }
   }
}
/**********************************************************************/
void FindInertiaFrc(struct SCType *S)
{
   struct BodyType *B;
   vec3_t cPexa, cPexw, cPexwxw, Pxi, wxPxi;
   vec3_t FlexInertiaFrc, FlexInertiaFrcN;
   long Ib, i, j, Nf;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B             = &S->B[Ib];
      B->InertiaFrc = SxV(-B->mass, B->AccR);
   }

   if (S->FlexActive && S->RefPt == REFPT_JOINT) {
      /* -AlphaR x (c+Pf*eta) - wx(wx(c+Pf*eta)) - 2wxPf*xi */
      for (Ib = 0; Ib < S->Nb; Ib++) {
         B       = &S->B[Ib];
         Nf      = B->Nf;
         cPexa   = MxV(B->cplusPeta, B->AlphaR);
         cPexw   = MxV(B->cplusPeta, B->wn);
         cPexwxw = VxV(cPexw, B->wn);
         Pxi     = VEC3_ZERO;
         for (i = 0; i < 3; i++)
            for (j = 0; j < Nf; j++)
               Pxi.v[i] += B->Pf[i][j] * B->xi[j];

         wxPxi = VxV(B->wn, Pxi);
         for (i = 0; i < 3; i++)
            FlexInertiaFrc.v[i] = cPexa.v[i] - cPexwxw.v[i] - 2.0 * wxPxi.v[i];
         FlexInertiaFrcN = MTxV(B->CN, FlexInertiaFrc);
         B->InertiaFrc   = VAddV_Elem(B->InertiaFrc, FlexInertiaFrcN);
      }
   }
}
/**********************************************************************/
void FindFlexInertiaFrc(struct SCType *S)
{
   long Ib, Nf, f0;
   struct BodyType *B;
   struct DynType *D;
   long i, j, k;

   D = &S->Dyn;

   if (S->RefPt == REFPT_JOINT) {
      /* -Pf*AccR - Hf*AlphaR */
      for (Ib = 0; Ib < S->Nb; Ib++) {
         B  = &S->B[Ib];
         Nf = B->Nf;
         f0 = B->f0;
         for (i = 0; i < Nf; i++) {
            for (k = 0; k < 3; k++)
               D->FlexFrc[f0 + i] -= B->CnbP[k][i] * B->AccR.v[k] +
                                     B->HplusQeta[k][i] * B->AlphaR.v[k];
         }
      }
   }

   if (S->IncludeSecondOrderFlexTerms) {
      /*  -w*R*w - w*S*w*eta - 2*xi*Q*w */
      for (Ib = 0; Ib < S->Nb; Ib++) {
         B  = &S->B[Ib];
         Nf = B->Nf;
         f0 = B->f0;

         /* Qxi */
         for (i = 0; i < 3; i++) {
            for (j = 0; j < Nf; j++) {
               B->Qxi[i][j] = 0.0;
               for (k = 0; k < Nf; k++) {
                  B->Qxi[i][j] += B->Qf[IDX3(i, j, k, Nf, Nf)] * B->xi[k];
               }
            }
         }

         /* Rw */
         for (i = 0; i < 3; i++) {
            for (j = 0; j < Nf; j++) {
               B->Rw[i][j] = B->Rf[IDX3(i, j, 0, Nf, 3)] * B->wn.v[0] +
                             B->Rf[IDX3(i, j, 1, Nf, 3)] * B->wn.v[1] +
                             B->Rf[IDX3(i, j, 2, Nf, 3)] * B->wn.v[2];
            }
         }

         /* Swe */
         for (i = 0; i < 3; i++) {
            for (j = 0; j < Nf; j++) {
               for (k = 0; k < Nf; k++) {
                  B->Sw[IDX3(i, j, k, Nf, Nf)] =
                      B->Sf[IDX4(i, j, k, 0, Nf, Nf, 3)] * B->wn.v[0] +
                      B->Sf[IDX4(i, j, k, 1, Nf, Nf, 3)] * B->wn.v[1] +
                      B->Sf[IDX4(i, j, k, 2, Nf, Nf, 3)] * B->wn.v[2];
               }
            }
         }
         for (i = 0; i < 3; i++) {
            for (j = 0; j < Nf; j++) {
               B->Swe[i][j] = 0.0;
               for (k = 0; k < Nf; k++)
                  B->Swe[i][j] += B->Sw[IDX3(i, j, k, Nf, Nf)] * B->eta[k];
            }
         }

         /* Add them up */
         for (i = 0; i < Nf; i++) {
            for (j = 0; j < 3; j++)
               D->FlexFrc[f0 + i] -=
                   (B->Rw[j][i] + B->Swe[j][i] + 2.0 * B->Qxi[j][i]) *
                   B->wn.v[j];
         }
      }
   }
}
/**********************************************************************/
void FindFlexFrc(struct SCType *S)
{
   long Ib, In, i, k, Nf, f0;
   struct BodyType *B;
   struct DynType *D;
   struct NodeType *N;

   D = &S->Dyn;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B  = &S->B[Ib];
      Nf = B->Nf;
      f0 = B->f0;
      for (i = 0; i < Nf; i++) {
         D->FlexFrc[f0 + i] = 0.0;
         for (k = 0; k < Nf; k++)
            D->FlexFrc[f0 + i] -=
                B->Cf[i][k] * B->xi[k] + B->Kf[i][k] * B->eta[k];
      }
      for (In = 0; In < B->NumNodes; In++) {
         N = &B->Node[In];
         for (i = 0; i < Nf; i++) {
            D->FlexFrc[f0 + i] +=
                N->PSI[0][i] * N->Frc.v[0] + N->PSI[1][i] * N->Frc.v[1] +
                N->PSI[2][i] * N->Frc.v[2] + N->THETA[0][i] * N->Trq.v[0] +
                N->THETA[1][i] * N->Trq.v[1] + N->THETA[2][i] * N->Trq.v[2];
         }
      }
   }
}
/**********************************************************************/
void EchoPVel(struct SCType *S)
{
   FILE *outfile;
   long i, j, Nb, Nu, Nf;
   struct DynType *D;

   D = &S->Dyn;

   Nb = S->Nb;
   Nu = D->Nu;
   Nf = D->Nf;

   outfile = FileOpen(InOutPath, "PVel.42", "w");
   for (i = 0; i < 3 * Nb; i++) {
      for (j = 0; j < Nu; j++)
         fprintf(outfile, " %24.16le", D->PVel[i][j]);
      fprintf(outfile, "\n");
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "PAngVel.42", "w");
   for (i = 0; i < 3 * Nb; i++) {
      for (j = 0; j < Nu; j++)
         fprintf(outfile, " %24.16le", D->PAngVel[i][j]);
      fprintf(outfile, "\n");
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "PVelf.42", "w");
   for (i = 0; i < 3 * Nb; i++) {
      for (j = 0; j < Nf; j++)
         fprintf(outfile, " %24.16le", D->PVelf[i][j]);
      fprintf(outfile, "\n");
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "PAngVelf.42", "w");
   for (i = 0; i < 3 * Nb; i++) {
      for (j = 0; j < Nf; j++)
         fprintf(outfile, " %24.16le", D->PAngVelf[i][j]);
      fprintf(outfile, "\n");
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "mPVel.42", "w");
   for (i = 0; i < 3 * Nb; i++) {
      for (j = 0; j < Nu; j++)
         fprintf(outfile, " %24.16le", D->mPVel[i][j]);
      fprintf(outfile, "\n");
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "IPAngVel.42", "w");
   for (i = 0; i < 3 * Nb; i++) {
      for (j = 0; j < Nu; j++)
         fprintf(outfile, " %24.16le", D->IPAngVel[i][j]);
      fprintf(outfile, "\n");
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "mPVelf.42", "w");
   for (i = 0; i < 3 * Nb; i++) {
      for (j = 0; j < Nf; j++)
         fprintf(outfile, " %24.16le", D->mPVelf[i][j]);
      fprintf(outfile, "\n");
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "IPAngVelf.42", "w");
   for (i = 0; i < 3 * Nb; i++) {
      for (j = 0; j < Nf; j++)
         fprintf(outfile, " %24.16le", D->IPAngVelf[i][j]);
      fprintf(outfile, "\n");
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "Mf.42", "w");
   for (i = 0; i < Nf; i++) {
      for (j = 0; j < Nf; j++)
         fprintf(outfile, " %24.16le", D->Mf[i][j]);
      fprintf(outfile, "\n");
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "HQeOMf.42", "w");
   for (i = 0; i < Nf; i++) {
      for (j = 0; j < Nf; j++)
         fprintf(outfile, " %24.16le", D->HplusQetaPAngVelf[i][j]);
      fprintf(outfile, "\n");
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "BodyTrq.42", "w");
   for (i = 0; i < 3 * Nb; i++) {
      fprintf(outfile, " %24.16le\n", D->BodyTrq[i]);
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "BodyFrc.42", "w");
   for (i = 0; i < 3 * Nb; i++) {
      fprintf(outfile, " %24.16le\n", D->BodyFrc[i]);
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "FlexFrc.42", "w");
   for (i = 0; i < Nf; i++) {
      fprintf(outfile, " %24.16le\n", D->FlexFrc[i]);
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "FlexInertiaFrc.42", "w");
   for (i = 0; i < Nf; i++) {
      fprintf(outfile, " %24.16le\n", D->FlexInertiaFrc[i]);
   }
   fclose(outfile);
}
/**********************************************************************/
void EchoEOM(double **COEF, double *State, double *RHS, long Ns)
{
   FILE *outfile;
   long i, j;

   outfile = FileOpen(InOutPath, "COEF.42", "w");

   for (i = 0; i < Ns; i++) {
      for (j = 0; j < Ns; j++)
         fprintf(outfile, " %24.16le", COEF[i][j]);
      fprintf(outfile, "\n");
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "x.42", "w");
   for (i = 0; i < Ns; i++)
      fprintf(outfile, "%24.16le\n", State[i]);
   fclose(outfile);

   outfile = FileOpen(InOutPath, "RHS.42", "w");
   for (i = 0; i < Ns; i++)
      fprintf(outfile, "%24.16le\n", RHS[i]);
   fclose(outfile);
}
/**********************************************************************/
void EchoUdot(double *State, long Ns)
{
   static FILE *outfile;
   long i;
   static long First = 1;

   if (First) {
      First   = 0;
      outfile = FileOpen(InOutPath, "udot.42", "w");
   }

   for (i = 0; i < Ns; i++)
      fprintf(outfile, "%24.16le ", State[i]);
   fprintf(outfile, "\n");
}
/********************************************************************/
void EchoRemAcc(struct SCType *S)
{
   FILE *outfile;
   long i, Ib;
   struct BodyType *B;

   outfile = FileOpen(InOutPath, "AccR.42", "w");
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      for (i = 0; i < 3; i++)
         fprintf(outfile, " %24.16le\n", B->AccR.v[i]);
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "AlphaR.42", "w");
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      for (i = 0; i < 3; i++)
         fprintf(outfile, " %24.16le\n", B->AlphaR.v[i]);
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "wn.42", "w");
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      for (i = 0; i < 3; i++)
         fprintf(outfile, " %24.16le\n", B->wn.v[i]);
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "InertiaTrq.42", "w");
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      for (i = 0; i < 3; i++)
         fprintf(outfile, " %24.16le\n", B->InertiaTrq.v[i]);
   }
   fclose(outfile);

   outfile = FileOpen(InOutPath, "InertiaFrc.42", "w");
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      for (i = 0; i < 3; i++)
         fprintf(outfile, " %24.16le\n", B->InertiaFrc.v[i]);
   }
   fclose(outfile);
}
/**********************************************************************/
void KaneNBodyEOM(double *u, double *x, double *h, double *a, double *uf,
                  double *xf, double *udot, double *xdot, double *hdot,
                  double *adot, double *ufdot, double *xfdot, struct SCType *S)
{
   long i, j, k, Nk, Ig, Ib, Iw, N, ii, jj;
   struct DynType *D;
   struct JointType *G;
   struct BodyType *B;
   struct WhlType *W;
   vec3_t TrqBo, TrqGo, TrqBi;
   vec3_t FrcBo, FrcGo, FrcBi, FrcGi;
   vec3_t FrcBiN, FrcBoN;
   vec3_t rxFi, rxFo;
   D = &S->Dyn;

   /* .. Dynamics */

   MapStateVectorToBodyStates(u, x, h, a, uf, xf, S);

   /* Joint Partials */
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G = &S->G[Ig];
      JointPartials(FALSE, G->IsSpherical, G->RotSeq, G->TrnSeq, G->Ang,
                    G->AngRate, &G->Gamma, &G->Gs, &G->Gds, G->PosRate,
                    &G->Delta, &G->Ds, &G->Dds);
   }

   /* Path vectors, beta and rho */
   FindBodyPathDCMs(S);
   FindPathVectors(S);

   /* Find Peta, cplusPeta, HplusQeta, CnbP for each body */
   FindFlexTerms(S);

   /* Partial Angular Velocity Matrix (PAngVel) and I*PAngVel */
   FindPAngVel(S);
   /* Partial Velocity Matrix (PVel) and m*PVel */
   FindPVel(S);
   if (S->FlexActive) {
      /* Flex Partial Angular Velocity Matrix (PAngVelf) and I*PAngVelf */
      FindPAngVelf(S);
      /* Flex Partial Velocity Matrix (PVelf) and m*PVelf */
      FindPVelf(S);
      /* Add (c+Pf*eta)xPVel, etc to IPAngVelf and mPVelf */
      AugmentIPAngVelf(S);
      FindHplusQetaPAngVelf(S);
      if (S->RefPt == REFPT_JOINT) {
         AugmentIPAngVel(S);
         AugmentMPVel(S);
         AugmentMPVelf(S);
         FindPCPVelf(S);
      }
   }

   /* Remainder Accelerations, AlphaR and AccR */
   FindAlphaR(S);
   FindAccR(S);

   FindInertiaTrq(S);
   FindInertiaFrc(S);

   /* Joint forces and torques */
   for (Ig = 0; Ig < S->Ng; Ig++) {
      JointFrcTrq(&S->G[Ig], S);
   }

   /* "F-bendy" and "T-bendy", Spring/Damping, and nonlinear terms */
   if (S->FlexActive) {
      FindFlexFrc(S);
      FindFlexInertiaFrc(S);
   }

   /* Assemble BodyTrq, BodyFrc, and FlexFrc Terms */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      for (i = 0; i < 3; i++) {
         D->BodyTrq[3 * Ib + i] = B->Trq.v[i] + B->InertiaTrq.v[i];
         D->BodyFrc[3 * Ib + i] = B->FrcN.v[i] + B->InertiaFrc.v[i];
      }
   }
   /* Add Wheel Torques */
   for (Iw = 0; Iw < S->Nw; Iw++) {
      W  = &S->Whl[Iw];
      Ib = W->Body;
      for (i = 0; i < 3; i++) {
         D->BodyTrq[3 * Ib + i] -= W->Trq * W->A.v[i];
      }
   }

   /* Applied Joint Torques and Forces */
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G     = &S->G[Ig];
      FrcGi = VEC3_ZERO;
      TrqGo = VEC3_ZERO;
      for (i = 0; i < 3; i++) {
         for (j = 0; j < G->RotDOF; j++) {
            TrqGo.v[i] += G->Gamma.mat[i][j] * (G->Trq.v[j]);
         }
         for (j = 0; j < G->TrnDOF; j++) {
            FrcGi.v[i] += G->Delta.mat[i][j] * (G->Frc.v[j]);
         }
      }

      /* Force Transformations*/
      FrcGo  = MxV(G->CGoGi, FrcGi);
      FrcBi  = MTxV(G->CTrqBi, FrcGo);
      FrcBo  = MTxV(G->CTrqBo, FrcGo);
      FrcBiN = MTxV(S->B[G->Bin].CN, FrcBi);
      FrcBoN = MTxV(S->B[G->Bout].CN, FrcBo);
      rxFi   = VxV(G->ri, FrcBi);
      rxFo   = VxV(G->ro, FrcBo);

      /* Torque Transformations */
      TrqBi = MTxV(G->CTrqBi, TrqGo);
      TrqBo = MTxV(G->CTrqBo, TrqGo);

      for (i = 0; i < 3; i++) {
         D->BodyTrq[3 * G->Bin + i]  -= TrqBi.v[i] + rxFi.v[i];
         D->BodyTrq[3 * G->Bout + i] += TrqBo.v[i] + rxFo.v[i];
         D->BodyFrc[3 * G->Bin + i]  -= FrcBiN.v[i];
         D->BodyFrc[3 * G->Bout + i] += FrcBoN.v[i];
      }
   }

   /* Assemble COEF and RHS */
   Nk = 3 * S->Nb;
   for (i = 0; i < D->Nu; i++) {
      /* Upper Left of COEF */
      for (j = i; j < D->Nu; j++) {
         D->COEF[i][j] = 0.0;
         for (k = 0; k < Nk; k++) {
            D->COEF[i][j] += D->PAngVel[k][i] * D->IPAngVel[k][j] +
                             D->PVel[k][i] * D->mPVel[k][j];
         }
         D->COEF[j][i] = D->COEF[i][j];
      }
      /* Upper Right of COEF */
      for (j = 0; j < D->Nf; j++) {
         D->COEF[i][D->Nu + j] = 0.0;
         for (k = 0; k < Nk; k++) {
            D->COEF[i][D->Nu + j] += D->PAngVel[k][i] * D->IPAngVelf[k][j] +
                                     D->PVel[k][i] * D->mPVelf[k][j];
         }
      }
      /* Upper RHS */
      D->RHS[i] = 0.0;
      for (k = 0; k < Nk; k++) {
         D->RHS[i] +=
             D->PAngVel[k][i] * D->BodyTrq[k] + D->PVel[k][i] * D->BodyFrc[k];
      }
   }
   for (i = 0; i < D->Nf; i++) {
      /* Lower Left of COEF is Transpose of Upper Right */
      for (j = 0; j < D->Nu; j++)
         D->COEF[D->Nu + i][j] = D->COEF[j][D->Nu + i];
      /* Lower Right of COEF */
      for (j = 0; j < D->Nf; j++) {
         D->COEF[D->Nu + i][D->Nu + j] = D->Mf[i][j];
         for (k = 0; k < Nk; k++) {
            D->COEF[D->Nu + i][D->Nu + j] +=
                D->PAngVelf[k][i] * D->IPAngVelf[k][j] +
                D->PVelf[k][i] * D->mPVelf[k][j];
         }
      }
      /* Lower RHS */
      D->RHS[D->Nu + i] = D->FlexFrc[i] + D->FlexInertiaFrc[i];
      for (k = 0; k < Nk; k++)
         D->RHS[D->Nu + i] +=
             D->PAngVelf[k][i] * D->BodyTrq[k] + D->PVelf[k][i] * D->BodyFrc[k];
   }
   if (S->RefPt == REFPT_JOINT) {
      for (i = 0; i < D->Nf; i++) {
         for (j = 0; j < D->Nf; j++) {
            D->COEF[D->Nu + i][D->Nu + j] +=
                D->PCPVelf[i][j] + D->HplusQetaPAngVelf[i][j];
         }
      }
   }

   /* .. Eliminate locked DOF */
   if (D->SomeJointsLocked) {
      N = D->Nu + D->Nf;
      /* Eliminate Rows */
      for (i = 0; i < D->Ns; i++) {
         ii = D->ActiveStateIdx[i];
         for (j = 0; j < N; j++) {
            D->COEF[i][j] = D->COEF[ii][j];
         }
         D->RHS[i] = D->RHS[ii];
      }
      /* Eliminate Columns */
      for (j = 0; j < D->Ns; j++) {
         jj = D->ActiveStateIdx[j];
         for (i = 0; i < D->Ns; i++) {
            D->COEF[i][j] = D->COEF[i][jj];
         }
      }
   }

   /* .. Solve EOM */
   // EchoPVel(S);
   // EchoRemAcc(S);
   // if (First) {
   //    First = 0;
   //    EchoEOM(D->COEF,D->ActiveState,D->RHS,D->Ns);
   // }
   LINSOLVE(D->COEF, D->ActiveState, D->RHS, D->Ns);
   // EchoUdot(D->ActiveState,D->Ns);

   /* .. Map out result */
   if (D->SomeJointsLocked) {
      N = D->Ns - D->Nf;
      for (i = 0; i < N; i++)
         udot[D->ActiveStateIdx[i]] = D->ActiveState[i];
      for (i = 0; i < D->Nf; i++)
         ufdot[i] = D->ActiveState[N + i];
   }
   else {
      for (i = 0; i < D->Nu; i++)
         udot[i] = D->ActiveState[i];
      for (i = 0; i < D->Nf; i++)
         ufdot[i] = D->ActiveState[D->Nu + i];
   }

   /* .. Kinematics */
   /* B[0].qn */
   quat_t q = DBL_TO_QUAT(&x[0]);
   quat_t qdot;
   vec3_t w = DBL_TO_VEC3(&u[0]);
   qdot     = QW2QDOT(q, w);
   QUAT_TO_DBL(&xdot[0], qdot);

   /* Joints, rotation and translation */
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G = &S->G[Ig];
      if (G->IsSpherical) {
         q    = DBL_TO_QUAT(&x[G->Rotx0]);
         w    = DBL_TO_VEC3(&u[G->Rotu0]);
         qdot = QW2QDOT(q, w);
         QUAT_TO_DBL(&xdot[G->Rotx0], qdot);
      }
      else {
         for (i = 0; i < G->RotDOF; i++)
            xdot[G->Rotx0 + i] = u[G->Rotu0 + i];
      }
      for (i = 0; i < G->TrnDOF; i++)
         xdot[G->Trnx0 + i] = u[G->Trnu0 + i];
   }
   /* B[0].pn */
   for (i = 0; i < 3; i++)
      xdot[D->Nx - 3 + i] = u[D->Nu - 3 + i];
   /* Flex Modes */
   for (i = 0; i < D->Nf; i++)
      xfdot[i] = uf[i];

   /* .. Wheel-body interaction  */
   for (i = 0; i < S->Nw; i++) {
      hdot[i] = S->Whl[i].Trq;
      adot[i] = h[i] / S->Whl[i].J;
   }
}
/**********************************************************************/
/*  PAngVelc (for Constraints)                                        */
void FindPAngVelc(struct SCType *S)
{
   struct DynType *D;
   struct JointType *G;
   mat3x3_t CGo, CG;
   long Ib, i, j, k, i0, j0;
   long Jb, Ig, Nc;

   D = &S->Dyn;

   for (Ib = 1; Ib < S->Nb; Ib++) {
      Ig  = S->B[Ib].Gin;
      G   = &S->G[Ig];
      i0  = 3 * Ib;
      j0  = G->Rotc0;
      Nc  = 3 - G->RotDOF;
      CGo = MAT3X3_ZERO;
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Nc; j++) {
            for (k = 0; k < 3; k++) {
               CGo.mat[i][j] +=
                   G->CTrqBo.mat[k][i] * G->Gamma.mat[k][G->RotDOF + j];
            }
            D->PAngVelc[i0 + i][j0 + j] = CGo.mat[i][j];
         }
      }
      Jb = G->Bin;
      while (Jb > 0) {
         Ig = S->B[Jb].Gin;
         G  = &S->G[Ig];
         j0 = G->Rotc0;
         Nc = 3 - G->RotDOF;
         CG = MAT3X3_ZERO;
         for (i = 0; i < 3; i++)
            for (j = 0; j < Nc; j++)
               for (k = 0; k < 3; k++)
                  CG.mat[i][j] +=
                      D->BodyPathTable[Ib][Jb].Coi.mat[i][k] * CGo.mat[k][j];

         for (i = 0; i < 3; i++)
            for (j = 0; j < Nc; j++)
               D->PAngVelc[i0 + i][j0 + j] = CG.mat[i][j];
         Jb = G->Bin;
      }
   }
}
/**********************************************************************/
/* PVelc (for Constraints)                                            */
void FindPVelc(struct SCType *S)
{
   struct DynType *D;
   struct BodyType *Bjb;
   struct JointType *G;
   mat3x3_t RC;
   mat3x3_t RCB, RCG;
   mat3x3_t CNG, CD;
   long Ib, Jb, Ig, i, j, k, i0, j0, Nc;

   D = &S->Dyn;

   for (Ib = 1; Ib < S->Nb; Ib++) {
      i0 = 3 * Ib;
      Jb = Ib;
      while (Jb > 0) {
         Bjb = &S->B[Jb];
         Ig  = Bjb->Gin;
         G   = &S->G[Ig];
         /* Rotation */
         j0  = G->Rotc0;
         Nc  = 3 - G->RotDOF;
         RCB = VcrossMT(D->JointPathTable[Ib][Ig].rho, Bjb->CN);
         RC  = MxMT(RCB, G->CTrqBo);
         RCG = MAT3X3_ZERO;
         for (i = 0; i < 3; i++)
            for (j = 0; j < Nc; j++)
               for (k = 0; k < 3; k++)
                  RCG.mat[i][j] +=
                      RC.mat[i][k] * G->Gamma.mat[k][G->RotDOF + j];

         for (i = 0; i < 3; i++)
            for (j = 0; j < Nc; j++)
               D->PVelc[i0 + i][j0 + j] = RCG.mat[i][j];

         /* Translation */
         j0  = G->Trnc0;
         Nc  = 3 - G->TrnDOF;
         CNG = MTxMT(S->B[G->Bin].CN, G->CGiBi);
         CD  = MAT3X3_ZERO;
         for (i = 0; i < 3; i++) {
            for (j = 0; j < Nc; j++) {
               for (k = 0; k < 3; k++) {
                  CD.mat[i][j] +=
                      CNG.mat[i][k] * G->Delta.mat[k][G->TrnDOF + j];
               }
               D->PVelc[i0 + i][j0 + j] = CD.mat[i][j];
            }
         }
         Jb = G->Bin;
      }
   }
}
/**********************************************************************/
void KaneNBodyConstraints(struct SCType *S, double *u, double *x, double *h,
                          double *a, double *uf, double *xf)
{
   struct DynType *D;
   struct BodyType *B;
   struct JointType *G;
   struct WhlType *W;
   long Ig, Ib, Iw, i, j;
   vec3_t TrqBo, TrqGo, TrqBi;
   vec3_t FrcBo, FrcGo, FrcBi, FrcGi;
   vec3_t FrcBiN, FrcBoN;
   vec3_t rxFi, rxFo;

   D = &S->Dyn;

   MapStateVectorToBodyStates(u, x, h, a, uf, xf, S);

   for (Ig = 0; Ig < S->Ng; Ig++) {
      G = &S->G[Ig];
      JointPartials(FALSE, G->IsSpherical, G->RotSeq, G->TrnSeq, G->Ang,
                    G->AngRate, &G->Gamma, &G->Gs, &G->Gds, G->PosRate,
                    &G->Delta, &G->Ds, &G->Dds);
   }
   FindBodyPathDCMs(S);
   FindPathVectors(S);

   FindPAngVel(S);
   FindPVel(S);

   FindPAngVelc(S);
   FindPVelc(S);

   FindAlphaR(S);
   FindAccR(S);
   FindInertiaTrq(S);
   FindInertiaFrc(S);

   /* Non-actuator-induced joint torques */
   for (Ig = 0; Ig < S->Ng; Ig++)
      JointFrcTrq(&S->G[Ig], S);

   /* "F-bendy" and "T-bendy", Spring/Damping, and nonlinear terms */
   if (S->FlexActive) {
      FindFlexFrc(S);
      FindFlexInertiaFrc(S);
   }

   /* Assemble BodyTrq, BodyFrc, and FlexFrc Terms */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      for (i = 0; i < 3; i++) {
         D->BodyTrq[3 * Ib + i] = B->Trq.v[i] + B->InertiaTrq.v[i];
         D->BodyFrc[3 * Ib + i] = B->FrcN.v[i] + B->InertiaFrc.v[i];
      }
   }
   /* Add Wheel Torques */
   for (Iw = 0; Iw < S->Nw; Iw++) {
      W  = &S->Whl[Iw];
      Ib = W->Body;
      for (i = 0; i < 3; i++) {
         D->BodyTrq[3 * Ib + i] -= W->Trq * W->A.v[i];
      }
   }
   /* Applied Joint Torques and Forces */
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G     = &S->G[Ig];
      FrcGi = VEC3_ZERO;
      TrqGo = VEC3_ZERO;
      for (i = 0; i < 3; i++) {
         for (j = 0; j < G->RotDOF; j++) {
            TrqGo.v[i] += G->Gamma.mat[i][j] * G->Trq.v[j];
         }
         for (j = 0; j < G->TrnDOF; j++) {
            FrcGi.v[i] += G->Delta.mat[i][j] * G->Frc.v[j];
         }
      }

      /* Force Transformations*/
      FrcGo  = MxV(G->CGoGi, FrcGi);
      FrcBi  = MTxV(G->CTrqBi, FrcGo);
      FrcBo  = MTxV(G->CTrqBo, FrcGo);
      FrcBiN = MTxV(S->B[G->Bin].CN, FrcBi);
      FrcBoN = MTxV(S->B[G->Bout].CN, FrcBo);
      rxFi   = VxV(G->ri, FrcBi);
      rxFo   = VxV(G->ro, FrcBo);

      /* Torque Transformations */
      TrqBi = MTxV(G->CTrqBi, TrqGo);
      TrqBo = MTxV(G->CTrqBo, TrqGo);

      for (i = 0; i < 3; i++) {
         D->BodyTrq[3 * G->Bin + i]  -= TrqBi.v[i] + rxFi.v[i];
         D->BodyTrq[3 * G->Bout + i] += TrqBo.v[i] + rxFo.v[i];
         D->BodyFrc[3 * G->Bin + i]  -= FrcBiN.v[i];
         D->BodyFrc[3 * G->Bout + i] += FrcBoN.v[i];
      }
   }

   /* Assemble Total Trq, Total Frc */
   for (i = 0; i < 3 * S->Nb; i++) {
      D->TotalTrq[i] = D->BodyTrq[i];
      D->TotalFrc[i] = D->BodyFrc[i];
      for (j = 0; j < D->Nu; j++) {
         D->TotalTrq[i] -= D->IPAngVel[i][j] * D->du[j];
         D->TotalFrc[i] -= D->mPVel[i][j] * D->du[j];
      }
   }
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      for (i = 0; i < 3; i++) {
         D->TotalTrq[3 * Ib + i] += B->InertiaTrq.v[i];
         D->TotalFrc[3 * Ib + i] += B->InertiaFrc.v[i];
      }
   }

   /* Find Generalized Constraint Forces */
   for (i = 0; i < D->Nc; i++) {
      D->GenConstraintFrc[i] = 0.0;
      for (j = 0; j < 3 * S->Nb; j++) {
         D->GenConstraintFrc[i] -= D->PAngVelc[j][i] * D->TotalTrq[j] +
                                   D->PVelc[j][i] * D->TotalFrc[j];
      }
   }
}
/**********************************************************************/
void FindBodyAccelerations(struct SCType *S, double *du)
{
   struct DynType *D;
   struct BodyType *B;
   long Ib, i, j;

   D = &S->Dyn;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B        = &S->B[Ib];
      B->alpha = B->AlphaR;
      for (i = 0; i < 3; i++) {
         B->accel.v[i] = S->FrcN.v[i] / S->mass + B->AccR.v[i];
         for (j = 0; j < D->Nu; j++) {
            B->alpha.v[i] += D->PAngVel[3 * Ib + i][j] * du[j];
            B->accel.v[i] += D->PVel[3 * Ib + i][j] * du[j];
         }
      }
   }
}
/**********************************************************************/
void KaneNBodyEOM_RK(struct SCType *S, double *const xdot_out)
{
   struct DynType *D;
   struct JointType *G;
   double *u, *du;
   double *x, *dx;
   double *h, *dh;
   double *a, *da;
   double *uf, *duf;
   double *xf, *dxf;
   long i, iu, Ig;
   long Nu, Nx, Nw, Nf;

   /* Save some typing (and dereferencing) */
   D  = &S->Dyn;
   Nu = D->Nu;
   Nx = D->Nx;
   Nw = S->Nw;
   Nf = D->Nf;

   u  = D->u;
   x  = D->x;
   h  = D->h;
   a  = D->a;
   uf = D->uf;
   xf = D->xf;

   du  = D->du;
   dx  = D->dx;
   dh  = D->dh;
   da  = D->da;
   duf = D->duf;
   dxf = D->dxf;

   /* State vector initialized in InitKaneNBody() */

   /* .. Check for Locked Joint DOFs */
   for (i = 0; i < 3; i++)
      D->ActiveStateIdx[i] = i; /* Body 0 angular DOF never locked */
   D->Ns = 3;
   iu    = 3;
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G               = &S->G[Ig];
      G->ActiveRotu0  = D->Ns;
      G->ActiveRotDOF = 0;
      for (i = 0; i < G->RotDOF; i++) {
         if (!G->RotLocked[i]) {
            G->ActiveRotDOF++;
            D->ActiveStateIdx[D->Ns] = iu;
            D->Ns++;
         }
         else {
            u[iu] = 0.0;
         }
         iu++;
      }
      G->ActiveTrnu0  = D->Ns;
      G->ActiveTrnDOF = 0;
      for (i = 0; i < G->TrnDOF; i++) {
         if (!G->TrnLocked[i]) {
            G->ActiveTrnDOF++;
            D->ActiveStateIdx[D->Ns] = iu;
            D->Ns++;
         }
         else {
            u[iu] = 0.0;
         }
         iu++;
      }
   }
   for (i = 0; i < 3; i++) { /* Body 0 translational DOF never locked */
      D->ActiveStateIdx[D->Ns] = iu;
      D->Ns++;
      iu++;
   }
   D->SomeJointsLocked  = ((D->Ns == D->Nu) ? 0 : 1);
   D->Ns               += D->Nf;

   /*  Calculate the eom */
   KaneNBodyEOM(u, x, h, a, uf, xf, du, dx, dh, da, duf, dxf, S);

   // TODO: in the original version, this is only called once after the first
   // eom call
   /* This call must be made here, so that du is taken at the */
   /* same instant as all the other configuration variables */
   if (S->ConstraintsRequested) {
      KaneNBodyConstraints(S, u, x, h, a, uf, xf);
   }
   // FindBodyAccelerations(S, du); terms calculated here are not used anywhere

   long offset = 0;
   CopyVG(&xdot_out[offset], du, Nu);
   offset += Nu;
   CopyVG(&xdot_out[offset], dx, Nx);
   offset += Nx;
   CopyVG(&xdot_out[offset], dh, Nw);
   offset += Nw;
   CopyVG(&xdot_out[offset], da, Nw);
   offset += Nw;
   CopyVG(&xdot_out[offset], duf, Nf);
   offset += Nf;
   CopyVG(&xdot_out[offset], dxf, Nf);
}
/******************************************************************************/
/*  Finds rotational and translational joint partials                         */
/*  On Init, populate all matrix elements.  Else, only populate               */
/*  variable ones.                                                            */
void OrderNJointPartials(struct JointType *G)
{
   double s2, c2, s3, c3;
   mat3x3_t Pw    = MAT3X3_ZERO;
   mat3x3_t Pwdot = MAT3X3_ZERO;
   mat3x3_t Pv    = MAT3X3_ZERO;
   mat3x3_t CPv;
   long i1, i2, i3, Cyclic, i, j, k;

   if (G->Init) {
      G->Init = 0;

      G->Pw    = MAT3X3_ZERO;
      G->Pv    = MAT3X3_ZERO;
      G->Pwdot = MAT3X3_ZERO;
      for (i = 0; i < 6; i++)
         for (j = 0; j < 6; j++)
            G->P[i][j] = 0.0;

      if (G->IsSpherical)
         G->Pw = G->CBoGo;

      i3 = G->TrnSeq % 10;         /* Pick off third digit */
      i2 = (G->TrnSeq % 100) / 10; /* Extract second digit */
      i1 = G->TrnSeq / 100;        /* Pick off first digit */

      Pv.mat[i1 - 1][0] = 1.0;
      Pv.mat[i2 - 1][1] = 1.0;
      Pv.mat[i3 - 1][2] = 1.0;
      G->Pv             = MTxM(G->CGiBi, Pv);
   }

   if (!G->IsSpherical) {
      i3 = G->RotSeq % 10;         /* Pick off third digit */
      i2 = (G->RotSeq % 100) / 10; /* Extract second digit */
      i1 = G->RotSeq / 100;        /* Pick off first digit */

      s2 = sin(G->Ang.y);
      c2 = cos(G->Ang.y);
      s3 = sin(G->Ang.z);
      c3 = cos(G->Ang.z);

      Cyclic = (i2 - i1) * (i3 - i2) * (i3 - i1);
      /* Convert (123) style to [012] subscripts */
      i1--;
      i2--;
      i3--;
      if (Cyclic > 0) { /* 123, 231, 312 */
         Pw.mat[i1][0] = c2 * c3;
         Pw.mat[i1][1] = s3;
         Pw.mat[i2][0] = -c2 * s3;
         Pw.mat[i2][1] = c3;
         Pw.mat[i3][0] = s2;
         Pw.mat[i3][2] = 1.0;

         Pwdot.mat[i1][0] =
             -s2 * c3 * G->AngRate.v[1] - c2 * s3 * G->AngRate.v[2];
         Pwdot.mat[i1][1] = c3 * G->AngRate.v[2];
         Pwdot.mat[i2][0] =
             s2 * s3 * G->AngRate.v[1] - c2 * c3 * G->AngRate.v[2];
         Pwdot.mat[i2][1] = -s3 * G->AngRate.v[2];
         Pwdot.mat[i3][0] = c2 * G->AngRate.v[1];
      }
      else if (Cyclic < 0) { /* 321, 132, 213 */
         Pw.mat[i1][0] = c2 * c3;
         Pw.mat[i1][1] = -s3;
         Pw.mat[i2][0] = c2 * s3;
         Pw.mat[i2][1] = c3;
         Pw.mat[i3][0] = -s2;
         Pw.mat[i3][2] = 1.0;

         Pwdot.mat[i1][0] =
             -s2 * c3 * G->AngRate.v[1] - c2 * s3 * G->AngRate.v[2];
         Pwdot.mat[i1][1] = -c3 * G->AngRate.v[2];
         Pwdot.mat[i2][0] =
             -s2 * s3 * G->AngRate.v[1] + c2 * c3 * G->AngRate.v[2];
         Pwdot.mat[i2][1] = -s3 * G->AngRate.v[2];
         Pwdot.mat[i3][0] = -c2 * G->AngRate.v[1];
      }
      else {
         fprintf(stderr,
                 "RotSeq %ld is not a Body-3 Sequence, so is not supported.\n",
                 G->RotSeq);
         exit(EXIT_FAILURE);
      }
      G->Pw    = MxM(G->CBoGo, Pw);
      G->Pwdot = MxM(G->CBoGo, Pwdot);
   }

   /* Pw is expressed in Bo, Pv is expressed in Bi */
   /* Express P in Bo */
   for (i = 0; i < 3; i++) {
      for (j = 0; j < G->TrnDOF; j++) {
         CPv.mat[i][j] = 0.0;
         for (k = 0; k < 3; k++)
            CPv.mat[i][j] += G->COI.mat[i][k] * G->Pv.mat[k][j];
      }
   }
   for (i = 0; i < 3; i++) {
      for (j = 0; j < G->RotDOF; j++)
         G->P[i][j] = G->Pw.mat[i][j];
      for (j = 0; j < G->TrnDOF; j++)
         G->P[3 + i][G->RotDOF + j] = CPv.mat[i][j];
   }
}
/******************************************************************************/
void MINV1to6(double A[6][6], double AI[6][6], long N)
{
   long I, J, ROW;
   long IPIVOT = 0;
   double M[6][6];
   double PIVOT, K, TA[6], TB[6];

   for (I = 0; I < N; I++) {
      for (J = 0; J < N; J++) {
         M[I][J]  = A[I][J];
         AI[I][J] = 0.0;
      }
      AI[I][I] = 1.0;
   }

   for (ROW = 0; ROW < N; ROW++) {
      PIVOT  = M[ROW][ROW];
      IPIVOT = ROW;
      for (I = ROW + 1; I < N; I++) {
         if (fabs(M[I][ROW]) > fabs(PIVOT)) {
            PIVOT  = M[I][ROW];
            IPIVOT = I;
         }
      }
      if (PIVOT == 0.0) {
         fprintf(stderr, "Matrix is singular in MINV1to6\n");
         exit(EXIT_FAILURE);
      }

      for (J = 0; J < N; J++) {
         TA[J]         = M[IPIVOT][J];
         TB[J]         = AI[IPIVOT][J];
         M[IPIVOT][J]  = M[ROW][J];
         AI[IPIVOT][J] = AI[ROW][J];
         M[ROW][J]     = TA[J] / PIVOT;
         AI[ROW][J]    = TB[J] / PIVOT;
      }
      for (I = ROW + 1; I < N; I++) {
         K = M[I][ROW];
         for (J = 0; J < N; J++) {
            M[I][J]  = M[I][J] - K * M[ROW][J];
            AI[I][J] = AI[I][J] - K * AI[ROW][J];
         }
      }
   }

   /*    M is now upper diagonal */

   for (ROW = N - 1; ROW > 0; ROW--) {
      for (I = 0; I < ROW; I++) {
         K = M[I][ROW];
         for (J = 0; J < N; J++) {
            M[I][J]  = M[I][J] - K * M[ROW][J];
            AI[I][J] = AI[I][J] - K * AI[ROW][J];
         }
      }
   }
}
/******************************************************************************/
void ShiftArtFrc(double F[6], vec3_t r, double Fbar[6])
{
   vec3_t F2  = DBL_TO_VEC3(&F[3]);
   vec3_t rxF = VxV(r, F2);

   Fbar[0] = F[0] - rxF.v[0];
   Fbar[1] = F[1] - rxF.v[1];
   Fbar[2] = F[2] - rxF.v[2];
   Fbar[3] = F[3];
   Fbar[4] = F[4];
   Fbar[5] = F[5];
}
/******************************************************************************/
void ShiftSpatAcc(double a[6], vec3_t r, double abar[6])
{
   vec3_t a2  = DBL_TO_VEC3(&a[0]);
   vec3_t axr = VxV(a2, r);

   abar[0] = a[0];
   abar[1] = a[1];
   abar[2] = a[2];
   abar[3] = a[3] + axr.v[0];
   abar[4] = a[4] + axr.v[1];
   abar[5] = a[5] + axr.v[2];
}
/******************************************************************************/
void ShiftArtMass(double A[6][6], vec3_t r, double B[6][6])
{
   mat3x3_t rx, rxA21, A12xr, rxA22, A22xr, rxA22xr;
   long i, j, k;

   rx    = V2CrossM(r);
   rxA21 = MAT3X3_ZERO;
   A12xr = MAT3X3_ZERO;
   rxA22 = MAT3X3_ZERO;
   A22xr = MAT3X3_ZERO;

   for (j = 0; j < 3; j++) {
      for (i = 0; i < 3; i++) {
         for (k = 0; k < 3; k++) {
            rxA21.mat[i][j] += rx.mat[i][k] * A[3 + k][j];
            A12xr.mat[i][j] += A[i][3 + k] * rx.mat[k][j];
            rxA22.mat[i][j] += rx.mat[i][k] * A[3 + k][3 + j];
            A22xr.mat[i][j] += A[3 + i][3 + k] * rx.mat[k][j];
         }
      }
   }
   rxA22xr = MxM(rxA22, rx);

   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         for (k = 0; k < 3; k++) {
            B[i][j] =
                A[i][j] - rxA21.mat[i][j] + A12xr.mat[i][j] - rxA22xr.mat[i][j];
            B[i][3 + j] = A[i][3 + j] - rxA22.mat[i][j];
            B[3 + i][j] = A[3 + i][j] + A22xr.mat[i][j];
         }
         B[3 + i][3 + j] = A[3 + i][3 + j];
      }
   }
}
/******************************************************************************/
void RotateSpatVec(mat3x3_t CBA, double Va[6], double Vb[6])
{
   long i, j;

   for (i = 0; i < 3; i++) {
      Vb[i]     = 0.0;
      Vb[3 + i] = 0.0;
      for (j = 0; j < 3; j++) {
         Vb[i]     += CBA.mat[i][j] * Va[j];
         Vb[3 + i] += CBA.mat[i][j] * Va[3 + j];
      }
   }
}
/******************************************************************************/
void RotateSpatMat(mat3x3_t CBA, double Ma[6][6], double Mb[6][6])
{
   mat3x3_t CM11, CM12, CM21, CM22;
   long i, j, k;

   CM11 = MAT3X3_ZERO;
   CM12 = MAT3X3_ZERO;
   CM21 = MAT3X3_ZERO;
   CM22 = MAT3X3_ZERO;
   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         for (k = 0; k < 3; k++) {
            CM11.mat[i][j] += CBA.mat[i][k] * Ma[k][j];
            CM12.mat[i][j] += CBA.mat[i][k] * Ma[k][3 + j];
            CM21.mat[i][j] += CBA.mat[i][k] * Ma[3 + k][j];
            CM22.mat[i][j] += CBA.mat[i][k] * Ma[3 + k][3 + j];
         }
      }
   }

   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         Mb[i][j]         = 0.0;
         Mb[i][3 + j]     = 0.0;
         Mb[3 + i][j]     = 0.0;
         Mb[3 + i][3 + j] = 0.0;
         for (k = 0; k < 3; k++) {
            Mb[i][j]         += CM11.mat[i][k] * CBA.mat[j][k];
            Mb[i][3 + j]     += CM12.mat[i][k] * CBA.mat[j][k];
            Mb[3 + i][j]     += CM21.mat[i][k] * CBA.mat[j][k];
            Mb[3 + i][3 + j] += CM22.mat[i][k] * CBA.mat[j][k];
         }
      }
   }
}
/******************************************************************************/
void OrderNJointCOI(struct JointType *G)
{
   mat3x3_t CBoGi;

   if (G->IsSpherical) {
      G->CGoGi = Q2C(G->q);
      G->Ang   = C2A(G->RotSeq, G->CGoGi);
   }
   else
      G->CGoGi = A2C(G->RotSeq, G->Ang.v[0], G->Ang.v[1], G->Ang.v[2]);

   CBoGi  = MxM(G->CBoGo, G->CGoGi);
   G->COI = MxM(CBoGi, G->CGiBi);
}
/******************************************************************************/
void ScatterStates(struct JointType *G)
{
   struct BodyType *Bi, *Bo;
   vec3_t Pwu, Pvu, Pdwu;
   vec3_t pni, vi, Cvi, wxPvu, ai, Cai;
   vec3_t wxri, wxro, Calfri, wxPwu;
   vec3_t axri, axro, wxwxri;
   vec3_t wxwxro, Cwi;
   vec3_t Iw, Ialfr, wxH;
   long i, j;

   Bi = G->Bi;
   Bo = G->Bo;

   Pwu  = VEC3_ZERO;
   Pvu  = VEC3_ZERO;
   Pdwu = VEC3_ZERO;
   for (i = 0; i < 3; i++) {
      for (j = 0; j < G->RotDOF; j++) {
         Pwu.v[i]  += G->Pw.mat[i][j] * G->AngRate.v[j];
         Pdwu.v[i] += G->Pwdot.mat[i][j] * G->AngRate.v[j];
      }
      for (j = 0; j < G->TrnDOF; j++) {
         Pvu.v[i] += G->Pv.mat[i][j] * G->PosRate.v[j];
      }
   }

   Bo->CN = MxM(G->COI, Bi->CN);

   pni    = VAddV_Elem(Bi->pn, G->riplusPx);
   Bo->pn = MxV(G->COI, pni);
   Bo->pn = VSubV_Elem(Bo->pn, G->RigidRout);

   /* Velocities */
   Cwi    = MxV(G->COI, Bi->wn);
   Bo->wn = VAddV_Elem(Cwi, Pwu);

   wxri = VxV(Bi->wn, G->riplusPx);
   wxro = VxV(Bo->wn, G->RigidRout);
   for (i = 0; i < 3; i++)
      vi.v[i] = Bi->vn.v[i] + Pvu.v[i] + wxri.v[i];
   Cvi    = MxV(G->COI, vi);
   Bo->vn = VSubV_Elem(Cvi, wxro);

   /* Remainder Accelerations */
   Calfri = MxV(G->COI, Bi->RemAlf);
   wxPwu  = VxV(Bo->wn, Pwu);
   for (i = 0; i < 3; i++)
      Bo->RemAlf.v[i] = Calfri.v[i] + Pdwu.v[i] + wxPwu.v[i];

   axri   = VxV(Bi->RemAlf, G->riplusPx);
   wxwxri = VxV(Bi->wn, wxri);
   wxPvu  = VxV(Bi->wn, Pvu);
   for (i = 0; i < 3; i++)
      ai.v[i] = Bi->RemAcc.v[i] + 2.0 * wxPvu.v[i] + axri.v[i] + wxwxri.v[i];
   Cai    = MxV(G->COI, ai);
   axro   = VxV(Bo->RemAlf, G->RigidRout);
   wxwxro = VxV(Bo->wn, wxro);
   for (i = 0; i < 3; i++)
      Bo->RemAcc.v[i] = Cai.v[i] - axro.v[i] - wxwxro.v[i];

   Iw = MxV(Bo->I, Bo->wn);
   for (i = 0; i < 3; i++)
      Bo->H.v[i] = Iw.v[i] + Bo->WhlMom.v[i] + Bo->EmbeddedMom.v[i];

   Ialfr = MxV(Bo->I, Bo->RemAlf);
   wxH   = VxV(Bo->wn, Bo->H);
   for (i = 0; i < 3; i++) {
      Bo->RemInertiaFrc[i]     = -Ialfr.v[i] - wxH.v[i];
      Bo->RemInertiaFrc[3 + i] = -Bo->mass * Bo->RemAcc.v[i];
   }
}
/******************************************************************************/
void GatherMassAndForce(struct JointType *G, struct SCType *S)
{
   struct BodyType *Bo;
   struct JointType *Gd;
   vec3_t rdk;
   double F[6], TF[6], CTF[6], SCTF[6];
   double M[6][6], TM[6][6], CTMC[6][6], SCTMCS[6][6];
   mat3x3_t Coc;
   long i, Id, j, k;

   Bo = G->Bo;

   /* Articulated-Body Force */
   for (i = 0; i < 6; i++) {
      F[i] = Bo->SpatFrc[i] + Bo->RemInertiaFrc[i];
   }
   ShiftArtFrc(F, G->RigidRout, G->ArtFrc);
   for (Id = 0; Id < Bo->Nd; Id++) {
      Gd = &S->G[Bo->Gd[Id]];
      for (i = 0; i < 6; i++) {
         TF[i] = 0.0;
         for (j = 0; j < 6; j++)
            TF[i] += Gd->TransMtx[i][j] * Gd->ArtFrc[j];
      }
      Coc = MT(Gd->COI);
      RotateSpatVec(Coc, TF, CTF);
      rdk = VSubV_Elem(G->RigidRout, Gd->riplusPx);
      ShiftArtFrc(CTF, rdk, SCTF);
      for (i = 0; i < 6; i++)
         G->ArtFrc[i] += SCTF[i];
   }

   /* Articulated-Body Mass */
   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         M[i][j]         = Bo->I.mat[i][j];
         M[i][3 + j]     = 0.0;
         M[3 + i][j]     = 0.0;
         M[3 + i][3 + j] = 0.0;
      }
      M[3 + i][3 + i] = Bo->mass;
   }
   ShiftArtMass(M, G->RigidRout, G->ArtMass);
   for (Id = 0; Id < Bo->Nd; Id++) {
      Gd = &S->G[Bo->Gd[Id]];
      for (i = 0; i < 6; i++) {
         for (j = 0; j < 6; j++) {
            TM[i][j] = 0.0;
            for (k = 0; k < 6; k++) {
               TM[i][j] += Gd->TransMtx[i][k] * Gd->ArtMass[k][j];
            }
         }
      }
      Coc = MT(Gd->COI);
      RotateSpatMat(Coc, TM, CTMC);
      rdk = VSubV_Elem(G->ro, Gd->riplusPx);
      ShiftArtMass(CTMC, rdk, SCTMCS);
      for (i = 0; i < 6; i++) {
         for (j = 0; j < 6; j++)
            G->ArtMass[i][j] += SCTMCS[i][j];
      }
   }
}
/******************************************************************************/
void GatherDynMtx(struct JointType *G, struct SCType *S __attribute__((unused)))
{
   double MP[6][6];
   long i, j, k;

   /* Dynamic Matrix, D */
   for (i = 0; i < 6; i++) {
      for (j = 0; j < G->Nu; j++) {
         MP[i][j] = 0.0;
         for (k = 0; k < 6; k++)
            MP[i][j] += G->ArtMass[i][k] * G->P[k][j];
      }
   }
   for (i = 0; i < G->Nu; i++) {
      for (j = 0; j < G->Nu; j++) {
         G->DynMtx[i][j] = 0.0;
         for (k = 0; k < 6; k++)
            G->DynMtx[i][j] += G->P[k][i] * MP[k][j];
      }
   }
   MINV1to6(G->DynMtx, G->InvDynMtx, G->Nu);

   /* Absorption Matrix, A */
   for (i = 0; i < G->Nu; i++) {
      for (j = 0; j < 6; j++) {
         G->InvDynPT[i][j] = 0.0;
         for (k = 0; k < G->Nu; k++)
            G->InvDynPT[i][j] += G->InvDynMtx[i][k] * G->P[j][k];
      }
   }
   /* TODO: Save a few cycles by not computing A and T for GN (A = U, T = 0) */
   for (i = 0; i < 6; i++) {
      for (j = 0; j < 6; j++) {
         G->AbsorpMtx[i][j] = 0.0;
         for (k = 0; k < G->Nu; k++)
            G->AbsorpMtx[i][j] += MP[i][k] * G->InvDynPT[k][j];
      }
   }

   /* Transmission Matrix, T */
   for (i = 0; i < 6; i++) {
      for (j = 0; j < 6; j++)
         G->TransMtx[i][j] = -G->AbsorpMtx[i][j];
      G->TransMtx[i][i] += 1.0;
   }
}
/******************************************************************************/
void ScatterStateDerivatives(struct JointType *G)
{
   struct BodyType *Bi, *Bo;
   double Ma[6], Saui[6], CSaui[6], F[6], CSauiPudot[6];
   vec3_t rko;
   long i, j;

   Bi = G->Bi;
   Bo = G->Bo;

   ShiftSpatAcc(Bi->AccU, G->riplusPx, Saui);
   RotateSpatVec(G->COI, Saui, CSaui);

   for (i = 0; i < 6; i++) {
      Ma[i] = 0.0;
      for (j = 0; j < 6; j++)
         Ma[i] += G->ArtMass[i][j] * CSaui[j];
      F[i] = G->ArtFrc[i] - Ma[i];
   }
   for (i = 0; i < G->Nu; i++) {
      G->udot[i] = 0.0;
      for (j = 0; j < 6; j++)
         G->udot[i] += G->InvDynPT[i][j] * F[j];
   }
   for (i = 0; i < 6; i++) {
      CSauiPudot[i] = CSaui[i];
      for (j = 0; j < G->Nu; j++)
         CSauiPudot[i] += G->P[i][j] * G->udot[j];
   }
   rko = NegV_Elem(G->RigidRout);
   ShiftSpatAcc(CSauiPudot, rko, Bo->AccU);
}
/******************************************************************************/
void OrderNMultiBodyEOM(struct SCType *S)
{
   struct BodyType *B, *Bi, *Bo;
   struct JointType *G;
   struct WhlType *W;
   vec3_t Iow, wxH;
   vec3_t TrqBo, TrqGo, TrqBi;
   vec3_t FrcBo, FrcBi;
   vec3_t rxFi, rxFo;
   long i, j, Ib, Ig, Iw;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B         = &S->B[Ib];
      B->WhlMom = VEC3_ZERO;
   }
   for (Iw = 0; Iw < S->Nw; Iw++) {
      W = &S->Whl[Iw];
      B = &S->B[W->Body];
      for (i = 0; i < 3; i++) {
         B->SpatFrc[i]  -= W->Trq * W->A.v[i];
         B->WhlMom.v[i] += W->H * W->A.v[i];
      }
   }

   /* First Pass: Root to tips */
   B = &S->B[0];
   G = &S->GN;
   OrderNJointCOI(G);
   OrderNJointPartials(G);
   B->wn = G->AngRate;
   B->CN = G->COI;

   B->pn = MxV(B->CN, G->Pos);
   B->vn = MxV(B->CN, G->PosRate);
   Iow   = MxV(B->I, B->wn);
   for (i = 0; i < 3; i++)
      B->H.v[i] = Iow.v[i] + B->WhlMom.v[i] + B->EmbeddedMom.v[i];
   wxH = VxV(B->wn, B->H);
   for (i = 0; i < 3; i++) {
      B->RemInertiaFrc[i]     = -wxH.v[i];
      B->RemInertiaFrc[3 + i] = 0.0;
   }
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G = &S->G[Ig];
      OrderNJointCOI(G);
      OrderNJointPartials(G);
      G->riplusPx = G->RigidRin;
      for (i = 0; i < 3; i++)
         for (j = 0; j < G->TrnDOF; j++)
            G->riplusPx.v[i] += G->Pv.mat[i][j] * G->Pos.v[j];

      ScatterStates(G);
   }

   /* Apply joint torques/forces to bodies */
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G  = &S->G[Ig];
      Bi = G->Bi;
      Bo = G->Bo;
      JointFrcTrq(G, S);
      FrcBi = VEC3_ZERO;
      TrqGo = VEC3_ZERO;
      for (i = 0; i < 3; i++) {
         for (j = 0; j < G->RotDOF; j++)
            TrqGo.v[i] += G->Pw.mat[i][j] * G->Trq.v[j];

         for (j = 0; j < G->TrnDOF; j++)
            FrcBi.v[i] += G->Pv.mat[i][j] * G->Frc.v[j];
      }
      /* Force Transformations*/
      FrcBo = MxV(G->COI, FrcBi);
      rxFi  = VxV(G->riplusPx, FrcBi);
      rxFo  = VxV(G->RigidRout, FrcBo);

      /* Torque Transformations */
      TrqBi = MTxV(G->CTrqBi, TrqGo);
      TrqBo = MTxV(G->CTrqBo, TrqGo);

      for (i = 0; i < 3; i++) {
         Bi->SpatFrc[i]     -= TrqBi.v[i] + rxFi.v[i];
         Bo->SpatFrc[i]     += TrqBo.v[i] + rxFo.v[i];
         Bi->SpatFrc[3 + i] -= FrcBi.v[i];
         Bo->SpatFrc[3 + i] += FrcBo.v[i];
      }
   }

   /* Second Pass: Tips to Root */
   for (Ig = S->Ng - 1; Ig >= 0; Ig--) {
      G = &S->G[Ig];
      GatherMassAndForce(G, S);
      GatherDynMtx(G, S);
   }
   G = &S->GN;
   GatherMassAndForce(G, S);
   GatherDynMtx(G, S);

   /* Third Pass: Root to tips */
   B = &S->B[0];
   G = &S->GN;
   for (i = 0; i < 6; i++) {
      G->udot[i] = 0.0;
      for (j = 0; j < 6; j++)
         G->udot[i] += G->InvDynPT[i][j] * G->ArtFrc[j];
   }
   for (i = 0; i < 3; i++) {
      B->AccU[i]     = G->udot[i];
      B->AccU[3 + i] = B->CN.mat[i][0] * G->udot[3] +
                       B->CN.mat[i][1] * G->udot[4] +
                       B->CN.mat[i][2] * G->udot[5];
   }

   for (Ig = 0; Ig < S->Ng; Ig++)
      ScatterStateDerivatives(&S->G[Ig]);

   /* Kinematic EOM */
   G       = &S->GN;
   G->qdot = QW2QDOT(G->q, G->AngRate);
   for (i = 0; i < 3; i++)
      G->xdot.v[i] = G->PosRate.v[i];
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G = &S->G[Ig];
      if (G->IsSpherical)
         G->qdot = QW2QDOT(G->q, G->AngRate);
      else
         for (i = 0; i < G->RotDOF; i++)
            G->qdot.q[i] = G->AngRate.v[i];
      for (i = 0; i < G->TrnDOF; i++)
         G->xdot.v[i] = G->PosRate.v[i];
   }

   /* Wheel EOM */
   for (Iw = 0; Iw < S->Nw; Iw++) {
      W       = &S->Whl[Iw];
      W->Hdot = W->Trq;
   }
}
/******************************************************************************/
void StateVectorToJoints(double *u, double *x, const long Nu, const long Nx,
                         struct JointType *GN, struct JointType *GList,
                         const long Ng)
{
   for (int i = 0; i < 3; i++) {
      GN->AngRate.v[i] = u[i];
      GN->PosRate.v[i] = u[Nu - 3 + i];
      GN->Pos.v[i]     = x[Nx - 3 + i];
   }
   for (int i = 0; i < 4; i++)
      GN->q.q[i] = x[i];
   GN->q = UNITQ(GN->q);

   for (int Ig = 0; Ig < Ng; Ig++) {
      struct JointType *G = &GList[Ig];
      for (int i = 0; i < G->RotDOF; i++)
         G->AngRate.v[i] = u[G->Rotu0 + i];

      for (int i = 0; i < G->TrnDOF; i++) {
         G->PosRate.v[i] = u[G->Trnu0 + i];
         G->Pos.v[i]     = x[G->Trnx0 + i];
      }
      for (int i = 0; i < ((G->IsSpherical) ? 4 : G->RotDOF); i++)
         G->q.q[i] = x[G->Rotx0 + i];
   }
}
/******************************************************************************/
void OrderNMultiBodyEOM_RK(struct SCType *S, double *const xdot_out)
{
   struct DynType *D; /* Copy to/from D->u, D->x */
   struct BodyType *B;
   struct JointType *G;
   struct WhlType *W;
   long i, Ig, Iw, Ib;

   /* Copy states from Dyn */
   D = &S->Dyn;
   G = &S->GN;

   /* Set up for EOM Call */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      for (i = 0; i < 3; i++) {
         B->SpatFrc[i]     = B->Trq.v[i];
         B->SpatFrc[3 + i] = B->FrcB.v[i];
      }
   }

   StateVectorToJoints(D->u, D->x, D->Nu, D->Nx, &S->GN, S->G, S->Ng);

   for (Iw = 0; Iw < S->Nw; Iw++) {
      W    = &S->Whl[Iw];
      W->H = D->h[Iw];
      W->w = W->H / W->J;
   }

   /*  Call the EOM */
   OrderNMultiBodyEOM(S);

   /*  Extract the data */
   G = &S->GN;
   for (i = 0; i < 3; i++) {
      xdot_out[i]                     = G->udot[i];
      xdot_out[D->Nu - 3 + i]         = G->udot[3 + i];
      xdot_out[D->Nu + D->Nx - 3 + i] = G->xdot.v[i];
   }
   for (i = 0; i < 4; i++)
      xdot_out[D->Nu + i] = G->qdot.q[i];

   for (Ig = 0; Ig < S->Ng; Ig++) {
      G = &S->G[Ig];
      for (i = 0; i < G->RotDOF; i++)
         xdot_out[G->Rotu0 + i] = G->udot[i];
      for (i = 0; i < G->TrnDOF; i++) {
         xdot_out[G->Trnu0 + i]         = G->udot[G->RotDOF + i];
         xdot_out[D->Nu + G->Trnx0 + i] = G->xdot.v[i];
      }
      for (i = 0; i < ((G->IsSpherical) ? 4 : G->RotDOF); i++)
         xdot_out[D->Nu + G->Rotx0 + i] = G->qdot.q[i];
   }
   for (Iw = 0; Iw < S->Nw; Iw++) {
      W = &S->Whl[Iw];

      xdot_out[D->Nu + D->Nx + Iw] = W->Hdot;
   }
}
/**********************************************************************/
/* Utility function for Encke's method.  Computes f(q).               */
/* See Battin, p. 449                                                 */
double EnckeFQ(vec3_t r, vec3_t delta) __attribute__((const));
double EnckeFQ(vec3_t r, vec3_t delta)
{
   double q, q1;

   q = (delta.v[0] * (delta.v[0] - 2.0 * r.v[0]) +
        delta.v[1] * (delta.v[1] - 2.0 * r.v[1]) +
        delta.v[2] * (delta.v[2] - 2.0 * r.v[2])) /
       VoV(r, r);

   q1 = 1.0 + q;

   return (q * (3.0 + q * (3.0 + q)) / (1.0 + sqrt(q1 * q1 * q1)));
}
/**********************************************************************/
/*  Orbit dynamics using Encke's method                               */
/*  See Battin, p. 449                                                */
/*   u[0-2] is Rrel(1-3)                                              */
/*   u[3-5] is Vrel(1-3)                                              */
vec3_t EnckeEOM(vec3_t xr, vec3_t orb_R, double muR3)
{
   double fq;
   vec3_t r;

   r.x = orb_R.x + xr.x;
   r.y = orb_R.y + xr.y;
   r.z = orb_R.z + xr.z;
   fq  = EnckeFQ(r, xr);

   vec3_t gravAccel;
   gravAccel.x = -muR3 * (xr.x + fq * r.x);
   gravAccel.y = -muR3 * (xr.y + fq * r.y);
   gravAccel.z = -muR3 * (xr.z + fq * r.z);
   return gravAccel;
}
/**********************************************************************/
__attribute__((const)) static vec3_t EnckeEOM_RK(const vec3_t rvec,
                                                 const vec3_t orb_R, double mu);
static vec3_t EnckeEOM_RK(vec3_t rvec, vec3_t orb_R, double mu)
{
   vec3_t gravAccel;
   double magr, muR3;

   magr = MAGV(orb_R);
   muR3 = mu / (magr * magr * magr);

   /* .. EOM  */
   gravAccel = EnckeEOM(rvec, orb_R, muR3);
   return gravAccel;
}
/**********************************************************************/
__attribute__((const)) static vec3_t CowellEOM_RK(const vec3_t rvec,
                                                  const double mu);
static vec3_t CowellEOM_RK(const vec3_t rvec, const double mu)
{
   double r, muR3;

   r    = MAGV(rvec);
   muR3 = mu / (r * r * r);

   vec3_t gravAccel;
   gravAccel.x = -muR3 * rvec.x;
   gravAccel.y = -muR3 * rvec.y;
   gravAccel.z = -muR3 * rvec.z;
   return gravAccel;
}
/**********************************************************************/
__attribute__((pure)) static vec3_t
PolyhedronCowellEOM_RK(const vec3_t rvec, struct WorldType *const world);
static vec3_t PolyhedronCowellEOM_RK(const vec3_t rvec,
                                     struct WorldType *const world)
{
   struct GeomType *G;
   vec3_t gravAccel;

   G = &Geom[world->GeomTag];

   /* .. EOM Call */
   PolyhedronGravAcc(G, world->Density, rvec, world->CWN, &gravAccel);
   return gravAccel;
}
/**********************************************************************/
/*  Orbit dynamics using Encke's method                               */
/*  Perturbation from Three-Body trajectory                           */
/*  See Battin, p. 449                                                */
/*   u[0-2] is Rrel(1-3)                                              */
/*   u[3-5] is Vrel(1-3)                                              */

vec3_t ThreeBodyEnckeEOM(vec3_t r, vec3_t R1, double muR13, vec3_t R2,
                         double muR23)
{
   vec3_t r1, r2;
   double fq1, fq2;

   r1 = VAddV_Elem(R1, r);
   r2 = VAddV_Elem(R2, r);

   fq1 = EnckeFQ(r1, r);
   fq2 = EnckeFQ(r2, r);

   vec3_t gravAccel;
   gravAccel.x = -muR13 * (r.x + fq1 * r1.x) - muR23 * (r.x + fq2 * r2.x);
   gravAccel.y = -muR13 * (r.y + fq1 * r1.y) - muR23 * (r.y + fq2 * r2.y);
   gravAccel.z = -muR13 * (r.z + fq1 * r1.z) - muR23 * (r.z + fq2 * r2.z);
   return gravAccel;
}
/**********************************************************************/
__attribute__((pure)) static vec3_t
ThreeBodyEnckeEOM_RK(const vec3_t rvec, struct OrbitType *const orb,
                     const vec3_t PosN2);
static vec3_t ThreeBodyEnckeEOM_RK(const vec3_t rvec,
                                   struct OrbitType *const orb,
                                   const vec3_t PosN2)
{
   vec3_t R1, R2, gravAccel;
   double MagR1, muR13, MagR2, muR23;

   R1 = orb->PosN;
   R2 = VSubV_Elem(R1, PosN2);

   MagR1 = MAGV(R1);
   muR13 = orb->mu1 / (MagR1 * MagR1 * MagR1);
   MagR2 = MAGV(R2);
   muR23 = orb->mu2 / (MagR2 * MagR2 * MagR2);

   /* .. EOM Call */
   gravAccel = ThreeBodyEnckeEOM(rvec, R1, muR13, R2, muR23);
   return gravAccel;
}
/**********************************************************************/
__attribute__((const)) static vec3_t
EulHillEOM_RK(const vec3_t rvec, const vec3_t vvec, double orb_n);
static vec3_t EulHillEOM_RK(const vec3_t rvec, const vec3_t vvec, double orb_n)
{
   vec3_t gravAccel;
   const double n2 = orb_n * orb_n;

   // assuming x is already in euler hill frame
   gravAccel.x = +2.0 * orb_n * vvec.z;
   gravAccel.y = -n2 * rvec.y;
   gravAccel.z = -2.0 * orb_n * vvec.x + 3.0 * n2 * rvec.z;
   return gravAccel;
}
/**********************************************************************/
void ThreeBodyOrbitEOM(double mu1, double mu2, vec3_t p, double u[6],
                       double udot[6])
{

   vec3_t r2;
   double r13, r23, p3, c1, c2, c3;

   r2.v[0] = u[0] - p.v[0];
   r2.v[1] = u[1] - p.v[1];
   r2.v[2] = u[2] - p.v[2];

   vec3_t uv = DBL_TO_VEC3(u);
   r13       = MAGV(uv);
   r13       = r13 * r13 * r13;
   r23       = MAGV(r2);
   r23       = r23 * r23 * r23;
   p3        = MAGV(p);
   p3        = p3 * p3 * p3;

   c1 = -mu1 / r13;
   c2 = -mu2 / r23;
   c3 = mu2 / p3;

   udot[0] = u[3];
   udot[1] = u[4];
   udot[2] = u[5];
   udot[3] = c1 * u[0] + c2 * r2.v[0] + c3 * p.v[0];
   udot[4] = c1 * u[1] + c2 * r2.v[1] + c3 * p.v[1];
   udot[5] = c1 * u[2] + c2 * r2.v[2] + c3 * p.v[2];
}
/************************************************************/
/*  Propagates motion of Reference Orbit under              */
/*  gravitational attraction of two large bodies.           */
void ThreeBodyOrbitRK4(struct WorldType *worlds, struct OrbitType *orb)
{
   double u[6], uu[6], m1[6], m2[6], m3[6], m4[6];
   long j;

   u[0] = orb->PosN.v[0];
   u[1] = orb->PosN.v[1];
   u[2] = orb->PosN.v[2];
   u[3] = orb->VelN.v[0];
   u[4] = orb->VelN.v[1];
   u[5] = orb->VelN.v[2];

   /* .. 4th Order Runga-Kutta Integration */
   ThreeBodyOrbitEOM(orb->mu1, orb->mu2, worlds[orb->Body2].eph.PosN, u, m1);
   for (j = 0; j < 6; j++)
      uu[j] = u[j] + 0.5 * DTSIM * m1[j];
   ThreeBodyOrbitEOM(orb->mu1, orb->mu2, worlds[orb->Body2].eph.PosN, uu, m2);
   for (j = 0; j < 6; j++)
      uu[j] = u[j] + 0.5 * DTSIM * m2[j];
   ThreeBodyOrbitEOM(orb->mu1, orb->mu2, worlds[orb->Body2].eph.PosN, uu, m3);
   for (j = 0; j < 6; j++)
      uu[j] = u[j] + DTSIM * m3[j];
   ThreeBodyOrbitEOM(orb->mu1, orb->mu2, worlds[orb->Body2].eph.PosN, uu, m4);
   for (j = 0; j < 6; j++)
      u[j] += DTSIM / 6.0 * (m1[j] + 2.0 * (m2[j] + m3[j]) + m4[j]);

   orb->PosN.v[0] = u[0];
   orb->PosN.v[1] = u[1];
   orb->PosN.v[2] = u[2];
   orb->VelN.v[0] = u[3];
   orb->VelN.v[1] = u[4];
   orb->VelN.v[2] = u[5];
}
/**********************************************************************/
void FixedOrbitPosition(struct OrbitType *orb, struct FormationType *const frm,
                        struct SCType *S)
{
   if (frm->FixedInFrame == 'L')
      /* TODO: This misbehaves for hyperbolic orbit.  Investigate */
      S->PosR = MxV(orb->CLN, S->PosEH);
   else
      S->PosEH = MTxV(orb->CLN, S->PosR);
}
/**********************************************************************/
void AddSCContactFrcTrq(struct SCType *S)
{
   for (long Ib = 0; Ib < S->Nb; Ib++) {
      for (long i = 0; i < 3; i++) {
         S->B[Ib].FrcN.v[i] += S->B[Ib].SCContactFrcN.v[i];
         S->B[Ib].FrcB.v[i] += S->B[Ib].SCContactFrcB.v[i];
         S->B[Ib].Trq.v[i]  += S->B[Ib].SCContactTrq.v[i];
      }
   }
}
/**********************************************************************/
/*   Divide acting forces into two components:                        */
/*   The external component perturbs the orbit and the internal       */
/*   (differential) component affects only attitude motion (for       */
/*   multi-body S/C).                                                 */
/*   Thus, S->Frc = External component                                */
/*   and   S->B[j].Frc = Internal component                           */
void PartitionForces(struct SCType *S)
{
   long Ib;
   vec3_t gravPertAccN = VEC3_ZERO;
   vec3_t FextN        = VEC3_ZERO;
   vec3_t FextB;
   long Nb;

   Nb = S->Nb;
   for (Ib = 0; Ib < Nb; Ib++) {
      FextN        = VAddV_Elem(FextN, S->B[Ib].FrcN);
      gravPertAccN = VAddV_Elem(gravPertAccN, S->B[Ib].gravPertAccN);
   }

   for (int i = 0; i < 3; i++) {
      S->FrcN.v[i]         += FextN.v[i];
      S->gravPertAccN.v[i] += gravPertAccN.v[i];
      S->AccN.v[i] = FextN.v[i] / S->mass; /* For accelerometer model */
   }

   for (Ib = 0; Ib < Nb; Ib++) {
      FextB = MxV(S->B[Ib].CN, FextN);
      for (int i = 0; i < 3; i++) {
         S->B[Ib].FrcN.v[i] -= FextN.v[i] * S->B[Ib].mass / S->mass;
         S->B[Ib].FrcB.v[i] -= FextB.v[i] * S->B[Ib].mass / S->mass;
      }
   }
}
/**********************************************************************/
vec3_t GetPrimaryGravAccel(const long OrbDOF, const vec3_t rvec,
                           const vec3_t vvec, struct WorldType *world,
                           struct OrbitType *orb)
{
   vec3_t gravAccN     = VEC3_ZERO;
   struct WorldType *W = &world[orb->World];
   switch (orb->Regime) {
      case ORB_ZERO:
      case ORB_FLIGHT:
         if (orb->PolyhedronGravityEnabled)
            gravAccN = PolyhedronCowellEOM_RK(rvec, W);
         else
            gravAccN = CowellEOM_RK(rvec, orb->mu);
         break;
      case ORB_CENTRAL:
         switch (OrbDOF) {
            case ORBDOF_FIXED:
               break;
            case ORBDOF_EULER_HILL:
               // assumes incoming rvec and vvec are already in Euler-Hill frame
               gravAccN = EulHillEOM_RK(rvec, vvec, orb->MeanMotion);
               break;
            case ORBDOF_COWELL:
               gravAccN = CowellEOM_RK(rvec, orb->mu);
               break;
            default:
               gravAccN = EnckeEOM_RK(rvec, orb->PosN, orb->mu);
               break;
         }
         break;
      case ORB_N_BODY:
         switch (OrbDOF) {
            case ORBDOF_COWELL:
               gravAccN = CowellEOM_RK(rvec, orb->mu);
               break;
            default:
               printf("ERROR: MUST USE COWELLS METHOD!!! \n");
               exit(EXIT_FAILURE);
         }
         break;
      case ORB_THREE_BODY:
         switch (OrbDOF) {
            case ORBDOF_FIXED:
               break;
            case ORBDOF_EULER_HILL:
               gravAccN = EulHillEOM_RK(rvec, vvec, orb->MeanMotion);
               break;
            case ORBDOF_COWELL:
               gravAccN = CowellEOM_RK(rvec, orb->mu);
               break;
            default:
               gravAccN =
                   ThreeBodyEnckeEOM_RK(rvec, orb, world[orb->Body2].eph.PosN);
               break;
         }
         break;
      default:
         fprintf(stderr, "Unknown Orbit Regime in Dynamics.  Bailing out.\n");
         exit(EXIT_FAILURE);
   }
   return gravAccN;
}
/**********************************************************************/
void SCOde(RKIndType jd_tt_mjd, double *x, RKParams *const params, double *xdot)
{
   if (params == NULL) {
      fprintf(
          stderr,
          "In SCOde, the params parameter is required to be set. Exiting...\n");
      exit(EXIT_FAILURE);
   }

   // x ENDS with posn & veln
   SCRKParams *scparams = (SCRKParams *)params;

   const long dim = params->dim;

#ifdef DEBUG_MODE
   if (any_isnan(dim, x)) {
      fprintf(stderr, "In SCOde, have nan input state. Exiting...\n");
      exit(EXIT_FAILURE);
   }
#endif

   struct SCType *sc                 = scparams->sc;
   struct OrbitType *orb             = scparams->orb;
   struct WorldType *world           = scparams->worlds;
   struct RegionType *rgn            = scparams->rgn;
   struct LagrangeSystemType *lagsys = scparams->lagsys;
   struct FormationType *frm         = scparams->frm;
   ephemType ephem                   = scparams->ephem;

   jd_tt_mjd          = JDChangeSystemEpoch(TT_TIME, GMAT_MJD_EPOCH, jd_tt_mjd);
   JDType jd_tt_j2000 = JDChangeSystemEpoch(TT_TIME, J2000_EPOCH, jd_tt_mjd);
   JDType jd_tdb_j2000 =
       JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd_tt_j2000);

   double *x_trn    = NULL;
   double *xdot_trn = NULL;

   // TODO: three body orbit is integrated sometimes, so add its states to the
   // integration
   WorldEphemerides(jd_tdb_j2000, jd_tt_j2000, ephem, world, rgn, lagsys);
   OrbitMotion(jd_tt_mjd, world, orb, rgn, lagsys, frm);
   RKStateToS(orb, x, sc);
   if (sc->OrbDOF == ORBDOF_EULER_HILL) {
      vec3_t pv = DBL_TO_VEC3(x_trn);
      vec3_t vv = DBL_TO_VEC3(&x_trn[3]);
      pair_vec3_t pair =
          EHRV2RelRV(orb->SMA, orb->MeanMotion, Orb->CLN, pv, vv);
      sc->PosR = pair.first;
      sc->VelR = pair.second;
   }
   else if (sc->OrbDOF == ORBDOF_FIXED)
      FixedOrbitPosition(orb, frm, sc);
   SCEphemerides(jd_tdb_j2000, sc, &world[orb->World], orb);

   ZeroNonSCContactFrcTrq(sc);

   /* Magnetic Field, Atmospheric Density */
   Environment(jd_tt_mjd, world, orb, sc);
   Perturbations(jd_tdb_j2000, world, orb, sc);
   Actuators(TRUE, sc, jd_tt_mjd);
   PartitionForces(sc); /* Orbit-affecting and "internal" */

   switch (sc->DynMethod) {
      case DYN_GAUSS_ELIM:
         KaneNBodyEOM_RK(sc, xdot);
         break;
      case DYN_ORDER_N:
         OrderNMultiBodyEOM_RK(sc, xdot);
         break;
      default:
         fprintf(stderr, "Unknown Dynamics Solution option.  Bailing out.\n");
         exit(EXIT_FAILURE);
   }

   if (sc->OrbDOF != ORBDOF_FIXED) {
      vec3_t rvec, vvec, accel = VEC3_ZERO;
      x_trn    = &x[dim - 6];
      xdot_trn = &xdot[dim - 6];
      rvec     = DBL_TO_VEC3(x_trn);
      vvec     = DBL_TO_VEC3(&x_trn[3]);

      xdot_trn[0] = x_trn[3];
      xdot_trn[1] = x_trn[4];
      xdot_trn[2] = x_trn[5];
      accel       = SxV(1.0 / sc->mass, sc->FrcN);
      if (sc->OrbDOF == ORBDOF_EULER_HILL)
         accel = MxV(orb->CLN, accel);

      sc->gravPriAccN = GetPrimaryGravAccel(sc->OrbDOF, rvec, vvec, world, orb);
      VEC3_TO_DBL(&xdot_trn[3], VAddV_Elem(sc->gravPriAccN, accel));
   }

#ifdef DEBUG_MODE
   if (any_isnan(dim, xdot)) {
      fprintf(stderr, "In SCOde, have nan state derivative. Exiting...\n");
      exit(EXIT_FAILURE);
   }
#endif
}

/* #ifdef __cplusplus
** }
** #endif
*/

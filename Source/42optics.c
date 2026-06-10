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
long Aperture(vec3_t FldPnt, vec3_t FldDir, vec3_t ctr, vec3_t axis,
              double ApRad, vec3_t *IntPnt, vec3_t *OutDir)
{
   double eps = 1.0E-12;
   double dl  = 1.0E6;
   double K   = 1.0;

   double PoA, r, l, a;
   vec3_t cq, rvec, cp;
   long k = 0;

   long InAperture = TRUE;

   cq = VmVElem(FldPnt, ctr);
   l  = VoV(cq, axis);
   while (fabs(dl) > eps && k < 10) {
      for (int i = 0; i < 3; i++)
         IntPnt->v[i] = FldPnt.v[i] + l * FldDir.v[i];
      cp  = VpVElem(*IntPnt, ctr);
      PoA = VoV(cp, axis);
      for (int i = 0; i < 3; i++)
         rvec.v[i] = cp.v[i] - PoA * axis.v[i];
      r = MAGV(rvec);

      /* Surface Equation */
      a = 0.0;

      dl  = -K * (a - PoA);
      l  += dl;
      k++;
   }
   for (int i = 0; i < 3; i++)
      IntPnt->v[i] = FldPnt.v[i] + l * FldDir.v[i];
   if (r > ApRad)
      InAperture = FALSE;

   *OutDir = FldDir;

   return (InAperture);
}
/**********************************************************************/
long PlanarMirror(vec3_t FldPnt, vec3_t FldDir, vec3_t ctr, vec3_t axis,
                  double ApRad, vec3_t *IntPnt, vec3_t *ReflDir)
{
   double eps = 1.0E-12;
   double dl  = 1.0E6;
   double K   = 1.0;

   double PoA, r, a, l;
   vec3_t rvec, nhat, cq, cp;
   double LoN;
   long k = 0;

   long InAperture = TRUE;

   cq = VmVElem(FldPnt, ctr);
   l  = VoV(cq, axis);
   while (fabs(dl) > eps && k < 10) {
      for (int i = 0; i < 3; i++)
         IntPnt->v[i] = FldPnt.v[i] + l * FldDir.v[i];
      cp  = VmVElem(*IntPnt, ctr);
      PoA = VoV(cp, axis);
      for (int i = 0; i < 3; i++)
         rvec.v[i] = cp.v[i] - PoA * axis.v[i];
      r = MAGV(rvec);

      /* Mirror Equation */
      a = 0.0;

      dl  = -K * (a - PoA);
      l  += dl;
      k++;
   }
   for (int i = 0; i < 3; i++)
      IntPnt->v[i] = FldPnt.v[i] + l * FldDir.v[i];
   if (r > ApRad)
      InAperture = FALSE;

   /* Mirror Normal */
   nhat = axis;
   LoN  = VoV(FldDir, nhat);
   for (int i = 0; i < 3; i++)
      ReflDir->v[i] = FldDir.v[i] - 2.0 * LoN * nhat.v[i];

   return (InAperture);
}
/**********************************************************************/
long ConicMirror(vec3_t FldPnt, vec3_t FldDir, vec3_t ctr, vec3_t axis,
                 double foclen, double ConicConst, double ConicSign,
                 double ApRad, vec3_t *IntPnt, vec3_t *ReflDir)
{
   double eps = 1.0E-12;
   double dl  = 1.0E6;
   double G   = 1.0;

   double R, D;

   double PoA, r, l, a, LoN;
   vec3_t nhat, cq, rvec, cp;
   double Den;
   long k = 0;
   long i;

   long InAperture = TRUE;

   R = 2.0 * foclen;

   cq = VmVElem(FldPnt, ctr);
   l  = VoV(cq, axis);
   for (i = 0; i < 3; i++)
      IntPnt->v[i] = FldPnt.v[i] + l * FldDir.v[i];
   while (fabs(dl) > eps && k < 10) {
      cp  = VpVElem(*IntPnt, ctr);
      PoA = VoV(cp, axis);
      for (i = 0; i < 3; i++)
         rvec.v[i] = cp.v[i] - PoA * axis.v[i];
      r = MAGV(rvec);

      /* Mirror Equation */
      D = sqrt(R * R - (1.0 + ConicConst) * r * r);
      a = ConicSign * r * r / (R + D);

      dl  = -G * (a - PoA);
      l  += dl;
      for (i = 0; i < 3; i++)
         IntPnt->v[i] = FldPnt.v[i] + l * FldDir.v[i];
      k++;
   }
   if (r > ApRad)
      InAperture = FALSE;

   /* Mirror Normal */
   Den = sqrt(R * R - ConicConst * r * r);
   for (i = 0; i < 3; i++)
      nhat.v[i] = (-ConicSign * rvec.v[i] + D * axis.v[i]) / Den;
   LoN = VoV(FldDir, nhat);
   for (i = 0; i < 3; i++)
      ReflDir->v[i] = FldDir.v[i] - 2.0 * LoN * nhat.v[i];

   return (InAperture);
}
/**********************************************************************/
long ThinLens(vec3_t FldPnt, vec3_t FldDir, vec3_t ctr, vec3_t axis,
              double foclen, double ApRad, vec3_t *IntPnt, vec3_t *RefrDir)
{
   long RayOnAxis  = FALSE;
   long InAperture = TRUE;
   vec3_t dp, binorm, rhat;
   double r0, a0, r1;
   double TanTheta0, TanTheta1;
   double theta1, CosTheta1, SinTheta1;
   double eps = 1.0E-6;

   dp     = VmVElem(FldPnt, ctr);
   binorm = VxV(dp, axis);
   if (MAGV(binorm) < eps) {
      binorm = VxV(FldDir, axis);
      if (MAGV(binorm) < eps)
         RayOnAxis = TRUE;
   }

   if (RayOnAxis) {
      *IntPnt  = ctr;
      *RefrDir = VNegElem(axis);
   }
   else {
      binorm = UNITV(binorm).v;
      rhat   = VxV(axis, binorm);

      a0 = VoV(dp, axis);
      r0 = VoV(dp, rhat);

      TanTheta0 = VoV(FldDir, rhat) / VoV(FldDir, axis);

      r1 = r0 - a0 * TanTheta0;
      if (fabs(r1) > ApRad)
         InAperture = FALSE;
      TanTheta1 = (r0 + (a0 - foclen) / foclen * r1);
      theta1    = atan(TanTheta1);
      CosTheta1 = cos(theta1);
      SinTheta1 = sin(theta1);

      for (int i = 0; i < 3; i++) {
         IntPnt->v[i]  = ctr.v[i] + r1 * rhat.v[i];
         RefrDir->v[i] = -CosTheta1 * axis.v[i] - SinTheta1 * rhat.v[i];
      }
   }

   return (InAperture);
}
/**********************************************************************/
long Detector(vec3_t FldPnt, vec3_t FldDir, vec3_t ctr, vec3_t axis,
              double ApRad, vec3_t *IntPnt)
{
   double eps = 1.0E-12;
   double dl  = 1.0E6;
   double K   = 1.0;

   double PoA, r, l, a;
   vec3_t cq, rvec, cp;
   long k = 0;

   long InAperture = TRUE;

   cq = VmVElem(FldPnt, ctr);
   l  = VoV(cq, axis);
   while (fabs(dl) > eps && k < 10) {
      for (int i = 0; i < 3; i++)
         IntPnt->v[i] = FldPnt.v[i] + l * FldDir.v[i];
      cp  = VmVElem(*IntPnt, ctr);
      PoA = VoV(cp, axis);
      for (int i = 0; i < 3; i++)
         rvec.v[i] = cp.v[i] - PoA * axis.v[i];
      r = MAGV(rvec);

      /* Surface Equation */
      a = 0.0;

      dl  = -K * (a - PoA);
      l  += dl;
      k++;
   }
   for (int i = 0; i < 3; i++)
      IntPnt->v[i] = FldPnt.v[i] + l * FldDir.v[i];
   if (r > ApRad)
      InAperture = FALSE;

   return (InAperture);
}
/**********************************************************************/
long OpticalFieldPoint(vec3_t StarVecB, struct OpticsType *O, vec3_t *FldPntB,
                       vec3_t *FldDirB)
{
   struct NodeType *N;
   vec3_t InPntB, InDirB, OutPntB, OutDirB;
   long InAp;

   N = &SC[O->SC].B[O->Body].Node[O->Node];

   /* Send ray upstream from focal point of ApFocus */
   for (int i = 0; i < 3; i++)
      InPntB.v[i] = N->NomPosB.v[i] - O->FocLen * O->Axis.v[i];
   InDirB   = StarVecB;
   InAp     = Aperture(InPntB, InDirB, N->NomPosB, O->Axis, O->ApRad, &OutPntB,
                       &OutDirB);
   *FldPntB = OutPntB;
   *FldDirB = VNegElem(O->Axis);

   return (InAp);
}
/**********************************************************************/
/* Returns number of elements successfully passed [0:Nopt]            */
long OpticalTrain(long FldSC, long FldBody, vec3_t FldPntB, vec3_t FldDirB,
                  long Nopt, struct OpticsType *Opt, long *OutSC, long *OutBody,
                  vec3_t *OutPntB, vec3_t *OutDirB)
{
   struct SCType *S;
   struct BodyType *B;
   struct NodeType *N;
   struct OpticsType *O;
   vec3_t InPntN, InDirN, InPntB, InDirB;
   vec3_t axis;
   long Io;
   long InAp;

   S      = &SC[FldSC];
   B      = &S->B[FldBody];
   InPntN = MTxV(B->CN, FldPntB);
   InDirN = MTxV(B->CN, FldDirB);
   for (int i = 0; i < 3; i++)
      InPntN.v[i] += B->pn.v[i] + S->PosR.v[i];

   /* .. For each element in optical train */
   for (Io = 0; Io < Nopt; Io++) {
      O    = &Opt[Io];
      S    = &SC[O->SC];
      B    = &S->B[O->Body];
      N    = &B->Node[O->Node];
      axis = QxV(N->qb, O->Axis);
      /* Transform incoming ray */
      for (int i = 0; i < 3; i++)
         InPntN.v[i] -= B->pn.v[i] + S->PosR.v[i];
      InPntB = MxV(B->CN, InPntN);
      InDirB = MxV(B->CN, InDirN);
      printf("Opt[%ld]:  InPnt: %lf %lf %lf   InDirB: %lf %lf %lf\n", Io,
             InPntB.v[0], InPntB.v[1], InPntB.v[2], InDirB.v[0], InDirB.v[1],
             InDirB.v[2]);
      /* Find Reflection */
      switch (O->Type) {
         case OPT_APERTURE:
            InAp = Aperture(InPntB, InDirB, N->PosB, axis, O->ApRad, OutPntB,
                            OutDirB);
            break;
         case OPT_CONIC:
            InAp = ConicMirror(InPntB, InDirB, N->PosB, axis, O->FocLen,
                               O->ConicConst, O->ConicSign, O->ApRad, OutPntB,
                               OutDirB);
            break;
         case OPT_THINLENS:
            InAp = ThinLens(InPntB, InDirB, N->PosB, axis, O->FocLen, O->ApRad,
                            OutPntB, OutDirB);
            break;
         case OPT_DETECTOR:
            InAp = Detector(InPntB, InDirB, N->PosB, axis, O->ApRad, OutPntB);
            *OutDirB = VEC3_ZERO;
            break;
      }
      if (!InAp)
         return (Io);
      else {
         *OutSC   = O->SC;
         *OutBody = O->Body;
         /* Copy and transform outgoing ray for next element */
         InPntN = MTxV(B->CN, *OutPntB);
         InDirN = MTxV(B->CN, *OutDirB);
         for (int i = 0; i < 3; i++)
            InPntN.v[i] += B->pn.v[i] + S->PosR.v[i];
      }
   }

   return (Nopt);
}

/* #ifdef __cplusplus
** }
** #endif
*/

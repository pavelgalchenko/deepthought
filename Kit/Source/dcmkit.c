/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "dcmkit.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

/**********************************************************************/
/*    Convert direction cosine matrix to quaternion.  Bulletproof.    */
quat_t C2Q(const mat3x3_t C)
{
   quat_t Q = QUAT_EYE;
   double K1, K2, K3, K4, K;

   K1 = 1.0 + C.mat[0][0] - C.mat[1][1] - C.mat[2][2];
   K2 = 1.0 - C.mat[0][0] + C.mat[1][1] - C.mat[2][2];
   K3 = 1.0 - C.mat[0][0] - C.mat[1][1] + C.mat[2][2];
   K4 = 1.0 + C.mat[0][0] + C.mat[1][1] + C.mat[2][2];

   K = K1;
   if (K2 > K)
      K = K2;
   if (K3 > K)
      K = K3;
   if (K4 > K)
      K = K4;

   if (K == K1) {
      Q.q[0] = 0.5 * sqrt(K1);
      Q.q[1] = 0.25 * (C.mat[0][1] + C.mat[1][0]) / Q.q[0];
      Q.q[2] = 0.25 * (C.mat[2][0] + C.mat[0][2]) / Q.q[0];
      Q.q[3] = 0.25 * (C.mat[1][2] - C.mat[2][1]) / Q.q[0];
   }
   else if (K == K2) {
      Q.q[1] = 0.5 * sqrt(K2);
      Q.q[0] = 0.25 * (C.mat[1][0] + C.mat[0][1]) / Q.q[1];
      Q.q[2] = 0.25 * (C.mat[2][1] + C.mat[1][2]) / Q.q[1];
      Q.q[3] = 0.25 * (C.mat[2][0] - C.mat[0][2]) / Q.q[1];
   }
   else if (K == K3) {
      Q.q[2] = 0.5 * sqrt(K3);
      Q.q[0] = 0.25 * (C.mat[2][0] + C.mat[0][2]) / Q.q[2];
      Q.q[1] = 0.25 * (C.mat[1][2] + C.mat[2][1]) / Q.q[2];
      Q.q[3] = 0.25 * (C.mat[0][1] - C.mat[1][0]) / Q.q[2];
   }
   else {
      Q.q[3] = 0.5 * sqrt(K4);
      Q.q[0] = 0.25 * (C.mat[1][2] - C.mat[2][1]) / Q.q[3];
      Q.q[1] = 0.25 * (C.mat[2][0] - C.mat[0][2]) / Q.q[3];
      Q.q[2] = 0.25 * (C.mat[0][1] - C.mat[1][0]) / Q.q[3];
   }
   return Q;
}
/**********************************************************************/
/*  Convert quaternion to direction cosine matrix                     */
mat3x3_t Q2C(const quat_t Q)
{
   double TwoQ00, TwoQ11, TwoQ22;
   double TwoQ01, TwoQ02, TwoQ03;
   double TwoQ12, TwoQ13, TwoQ23;

   TwoQ00 = 2.0 * Q.q[0] * Q.q[0];
   TwoQ11 = 2.0 * Q.q[1] * Q.q[1];
   TwoQ22 = 2.0 * Q.q[2] * Q.q[2];
   TwoQ01 = 2.0 * Q.q[0] * Q.q[1];
   TwoQ02 = 2.0 * Q.q[0] * Q.q[2];
   TwoQ03 = 2.0 * Q.q[0] * Q.q[3];
   TwoQ12 = 2.0 * Q.q[1] * Q.q[2];
   TwoQ13 = 2.0 * Q.q[1] * Q.q[3];
   TwoQ23 = 2.0 * Q.q[2] * Q.q[3];

   mat3x3_t C;

   C.mat[0][0] = 1.0 - (TwoQ11 + TwoQ22);
   C.mat[0][1] = TwoQ01 + TwoQ23;
   C.mat[0][2] = TwoQ02 - TwoQ13;
   C.mat[1][0] = TwoQ01 - TwoQ23;
   C.mat[1][1] = 1.0 - (TwoQ22 + TwoQ00);
   C.mat[1][2] = TwoQ12 + TwoQ03;
   C.mat[2][0] = TwoQ02 + TwoQ13;
   C.mat[2][1] = TwoQ12 - TwoQ03;
   C.mat[2][2] = 1.0 - (TwoQ00 + TwoQ11);
   return C;
}
/**********************************************************************/
/*   Convert Euler angle sequence to direction cosine matrix          */

mat3x3_t A2C(long SEQ, double TH1, double TH2, double TH3)
{
   double S1, C1;
   double S2 = 0.0;
   double C2 = 1.0;
   double S3 = 0.0;
   double C3 = 1.0;

   S1 = sin(TH1);
   C1 = cos(TH1);
   if (SEQ > 10) { /* Two digits or more */
      S2 = sin(TH2);
      C2 = cos(TH2);
   }
   if (SEQ > 100) { /* Three digits */
      S3 = sin(TH3);
      C3 = cos(TH3);
   }

   mat3x3_t C;
   switch (SEQ) {
      case 1: {
         C.mat[0][0] = 1.0;
         C.mat[1][0] = 0.0;
         C.mat[2][0] = 0.0;
         C.mat[0][1] = 0.0;
         C.mat[1][1] = C1;
         C.mat[2][1] = -S1;
         C.mat[0][2] = 0.0;
         C.mat[1][2] = S1;
         C.mat[2][2] = C1;
      } break;
      case 2: {
         C.mat[0][0] = C1;
         C.mat[1][0] = 0.0;
         C.mat[2][0] = S1;
         C.mat[0][1] = 0.0;
         C.mat[1][1] = 1.0;
         C.mat[2][1] = 0.0;
         C.mat[0][2] = -S1;
         C.mat[1][2] = 0.0;
         C.mat[2][2] = C1;
      } break;
      case 3: {
         C.mat[0][0] = C1;
         C.mat[1][0] = -S1;
         C.mat[2][0] = 0.0;
         C.mat[0][1] = S1;
         C.mat[1][1] = C1;
         C.mat[2][1] = 0.0;
         C.mat[0][2] = 0.0;
         C.mat[1][2] = 0.0;
         C.mat[2][2] = 1.0;
      } break;
      case 12: {
         C.mat[0][0] = C2;
         C.mat[1][0] = 0.0;
         C.mat[2][0] = S2;
         C.mat[0][1] = S1 * S2;
         C.mat[1][1] = C1;
         C.mat[2][1] = -S1 * C2;
         C.mat[0][2] = -C1 * S2;
         C.mat[1][2] = S1;
         C.mat[2][2] = C1 * C2;
      } break;
      case 13: {
         C.mat[0][0] = C2;
         C.mat[1][0] = -S2;
         C.mat[2][0] = 0.0;
         C.mat[0][1] = C1 * S2;
         C.mat[1][1] = C1 * C2;
         C.mat[2][1] = -S1;
         C.mat[0][2] = S1 * S2;
         C.mat[1][2] = S1 * C2;
         C.mat[2][2] = C1;
      } break;
      case 21: {
         C.mat[0][0] = C1;
         C.mat[1][0] = S1 * S2;
         C.mat[2][0] = S1 * C2;
         C.mat[0][1] = 0.0;
         C.mat[1][1] = C2;
         C.mat[2][1] = -S2;
         C.mat[0][2] = -S1;
         C.mat[1][2] = C1 * S2;
         C.mat[2][2] = C1 * C2;
      } break;
      case 23: {
         C.mat[0][0] = C1 * C2;
         C.mat[1][0] = -C1 * S2;
         C.mat[2][0] = S1;
         C.mat[0][1] = S2;
         C.mat[1][1] = C2;
         C.mat[2][1] = 0.0;
         C.mat[0][2] = -S1 * C2;
         C.mat[1][2] = S1 * S2;
         C.mat[2][2] = C1;
      } break;
      case 31: {
         C.mat[0][0] = C1;
         C.mat[1][0] = -S1 * C2;
         C.mat[2][0] = S1 * S2;
         C.mat[0][1] = S1;
         C.mat[1][1] = C1 * C2;
         C.mat[2][1] = -C1 * S2;
         C.mat[0][2] = 0.0;
         C.mat[1][2] = S2;
         C.mat[2][2] = C2;
      } break;
      case 32: {
         C.mat[0][0] = C1 * C2;
         C.mat[1][0] = -S1;
         C.mat[2][0] = C1 * S2;
         C.mat[0][1] = S1 * C2;
         C.mat[1][1] = C1;
         C.mat[2][1] = S1 * S2;
         C.mat[0][2] = -S2;
         C.mat[1][2] = 0.0;
         C.mat[2][2] = C2;
      } break;
      case 123: {
         C.mat[0][0] = C2 * C3;
         C.mat[1][0] = -C2 * S3;
         C.mat[2][0] = S2;
         C.mat[0][1] = S1 * S2 * C3 + S3 * C1;
         C.mat[1][1] = -S1 * S2 * S3 + C3 * C1;
         C.mat[2][1] = -S1 * C2;
         C.mat[0][2] = -C1 * S2 * C3 + S3 * S1;
         C.mat[1][2] = C1 * S2 * S3 + C3 * S1;
         C.mat[2][2] = C1 * C2;
      } break;
      case 231: {
         C.mat[0][0] = C1 * C2;
         C.mat[1][0] = -C1 * S2 * C3 + S3 * S1;
         C.mat[2][0] = C1 * S2 * S3 + C3 * S1;
         C.mat[0][1] = S2;
         C.mat[1][1] = C2 * C3;
         C.mat[2][1] = -C2 * S3;
         C.mat[0][2] = -S1 * C2;
         C.mat[1][2] = S1 * S2 * C3 + S3 * C1;
         C.mat[2][2] = -S1 * S2 * S3 + C3 * C1;
      } break;
      case 312: {
         C.mat[0][0] = -S1 * S2 * S3 + C3 * C1;
         C.mat[1][0] = -S1 * C2;
         C.mat[2][0] = S1 * S2 * C3 + S3 * C1;
         C.mat[0][1] = C1 * S2 * S3 + C3 * S1;
         C.mat[1][1] = C1 * C2;
         C.mat[2][1] = -C1 * S2 * C3 + S3 * S1;
         C.mat[0][2] = -C2 * S3;
         C.mat[1][2] = S2;
         C.mat[2][2] = C2 * C3;
      } break;
      case 132: {
         C.mat[0][0] = C2 * C3;
         C.mat[1][0] = -S2;
         C.mat[2][0] = C2 * S3;
         C.mat[0][1] = C1 * S2 * C3 + S3 * S1;
         C.mat[1][1] = C1 * C2;
         C.mat[2][1] = C1 * S2 * S3 - C3 * S1;
         C.mat[0][2] = S1 * S2 * C3 - S3 * C1;
         C.mat[1][2] = S1 * C2;
         C.mat[2][2] = S1 * S2 * S3 + C3 * C1;
      } break;
      case 213: {
         C.mat[0][0] = S1 * S2 * S3 + C3 * C1;
         C.mat[1][0] = S1 * S2 * C3 - S3 * C1;
         C.mat[2][0] = S1 * C2;
         C.mat[0][1] = C2 * S3;
         C.mat[1][1] = C2 * C3;
         C.mat[2][1] = -S2;
         C.mat[0][2] = C1 * S2 * S3 - C3 * S1;
         C.mat[1][2] = C1 * S2 * C3 + S3 * S1;
         C.mat[2][2] = C1 * C2;
      } break;
      case 321: {
         C.mat[0][0] = C1 * C2;
         C.mat[1][0] = C1 * S2 * S3 - C3 * S1;
         C.mat[2][0] = C1 * S2 * C3 + S3 * S1;
         C.mat[0][1] = S1 * C2;
         C.mat[1][1] = S1 * S2 * S3 + C3 * C1;
         C.mat[2][1] = S1 * S2 * C3 - S3 * C1;
         C.mat[0][2] = -S2;
         C.mat[1][2] = C2 * S3;
         C.mat[2][2] = C2 * C3;
      } break;
      case 121: {
         C.mat[0][0] = C2;
         C.mat[1][0] = S2 * S3;
         C.mat[2][0] = S2 * C3;
         C.mat[0][1] = S1 * S2;
         C.mat[1][1] = -S1 * C2 * S3 + C3 * C1;
         C.mat[2][1] = -S1 * C2 * C3 - S3 * C1;
         C.mat[0][2] = -C1 * S2;
         C.mat[1][2] = C1 * C2 * S3 + C3 * S1;
         C.mat[2][2] = C1 * C2 * C3 - S3 * S1;
      } break;
      case 131: {
         C.mat[0][0] = C2;
         C.mat[1][0] = -S2 * C3;
         C.mat[2][0] = S2 * S3;
         C.mat[0][1] = C1 * S2;
         C.mat[1][1] = C1 * C2 * C3 - S3 * S1;
         C.mat[2][1] = -C1 * C2 * S3 - C3 * S1;
         C.mat[0][2] = S1 * S2;
         C.mat[1][2] = S1 * C2 * C3 + S3 * C1;
         C.mat[2][2] = -S1 * C2 * S3 + C3 * C1;
      } break;
      case 212: {
         C.mat[0][0] = -S1 * C2 * S3 + C3 * C1;
         C.mat[1][0] = S1 * S2;
         C.mat[2][0] = S1 * C2 * C3 + S3 * C1;
         C.mat[0][1] = S2 * S3;
         C.mat[1][1] = C2;
         C.mat[2][1] = -S2 * C3;
         C.mat[0][2] = -C1 * C2 * S3 - C3 * S1;
         C.mat[1][2] = C1 * S2;
         C.mat[2][2] = C1 * C2 * C3 - S3 * S1;
      } break;
      case 232: {
         C.mat[0][0] = C1 * C2 * C3 - S1 * S3;
         C.mat[1][0] = -C1 * S2;
         C.mat[2][0] = C1 * C2 * S3 + S1 * C3;
         C.mat[0][1] = S2 * C3;
         C.mat[1][1] = C2;
         C.mat[2][1] = S2 * S3;
         C.mat[0][2] = -S1 * C2 * C3 - C1 * S3;
         C.mat[1][2] = S1 * S2;
         C.mat[2][2] = -S1 * C2 * S3 + C1 * C3;
      } break;
      case 313: {
         C.mat[0][0] = -S1 * C2 * S3 + C3 * C1;
         C.mat[1][0] = -S1 * C2 * C3 - S3 * C1;
         C.mat[2][0] = S1 * S2;
         C.mat[0][1] = C1 * C2 * S3 + C3 * S1;
         C.mat[1][1] = C1 * C2 * C3 - S3 * S1;
         C.mat[2][1] = -C1 * S2;
         C.mat[0][2] = S2 * S3;
         C.mat[1][2] = S2 * C3;
         C.mat[2][2] = C2;
      } break;
      case 323: {
         C.mat[0][0] = C1 * C2 * C3 - S3 * S1;
         C.mat[1][0] = -C1 * C2 * S3 - C3 * S1;
         C.mat[2][0] = C1 * S2;
         C.mat[0][1] = S1 * C2 * C3 + S3 * C1;
         C.mat[1][1] = -S1 * C2 * S3 + C3 * C1;
         C.mat[2][1] = S1 * S2;
         C.mat[0][2] = -S2 * C3;
         C.mat[1][2] = S2 * S3;
         C.mat[2][2] = C2;
      } break;
      default:
         fprintf(stderr, "Bogus Euler Sequence %ld in A2C\n", SEQ);
         exit(EXIT_FAILURE);
   }
   return C;
}
/**********************************************************************/
/*  Convert direction cosine matrix to Euler angles                   */

vec3_t C2A(long SEQ, mat3x3_t C)
{
   vec3_t TH;
   double *TH1 = &TH.x;
   double *TH2 = &TH.y;
   double *TH3 = &TH.z;
   switch (SEQ) {
      case 123: {
         *TH1 = atan2(-C.mat[2][1], C.mat[2][2]);
         *TH2 = asin(C.mat[2][0]);
         *TH3 = atan2(-C.mat[1][0], C.mat[0][0]);
      } break;
      case 231: {
         *TH1 = atan2(-C.mat[0][2], C.mat[0][0]);
         *TH2 = asin(C.mat[0][1]);
         *TH3 = atan2(-C.mat[2][1], C.mat[1][1]);
      } break;
      case 312: {
         *TH1 = atan2(-C.mat[1][0], C.mat[1][1]);
         *TH2 = asin(C.mat[1][2]);
         *TH3 = atan2(-C.mat[0][2], C.mat[2][2]);
      } break;
      case 132: {
         *TH1 = atan2(C.mat[1][2], C.mat[1][1]);
         *TH2 = asin(-C.mat[1][0]);
         *TH3 = atan2(C.mat[2][0], C.mat[0][0]);
      } break;
      case 213: {
         *TH1 = atan2(C.mat[2][0], C.mat[2][2]);
         *TH2 = asin(-C.mat[2][1]);
         *TH3 = atan2(C.mat[0][1], C.mat[1][1]);
      } break;
      case 321: {
         *TH1 = atan2(C.mat[0][1], C.mat[0][0]);
         *TH2 = asin(-C.mat[0][2]);
         *TH3 = atan2(C.mat[1][2], C.mat[2][2]);
      } break;
      case 121: {
         *TH1 = atan2(C.mat[0][1], -C.mat[0][2]);
         *TH2 = acos(C.mat[0][0]);
         *TH3 = atan2(C.mat[1][0], C.mat[2][0]);
      } break;
      case 131: {
         *TH1 = atan2(C.mat[0][2], C.mat[0][1]);
         *TH2 = acos(C.mat[0][0]);
         *TH3 = atan2(C.mat[2][0], -C.mat[1][0]);
      } break;
      case 212: {
         *TH1 = atan2(C.mat[1][0], C.mat[1][2]);
         *TH2 = acos(C.mat[1][1]);
         *TH3 = atan2(C.mat[0][1], -C.mat[2][1]);
      } break;
      case 232: {
         *TH1 = atan2(C.mat[1][2], -C.mat[1][0]);
         *TH2 = acos(C.mat[1][1]);
         *TH3 = atan2(C.mat[2][1], C.mat[0][1]);
      } break;
      case 313: {
         *TH1 = atan2(C.mat[2][0], -C.mat[2][1]);
         *TH2 = acos(C.mat[2][2]);
         *TH3 = atan2(C.mat[0][2], C.mat[1][2]);
      } break;
      case 323: {
         *TH1 = atan2(C.mat[2][1], C.mat[2][0]);
         *TH2 = acos(C.mat[2][2]);
         *TH3 = atan2(C.mat[1][2], -C.mat[0][2]);
      } break;
      default:
         fprintf(stderr, "Bogus Euler Sequence %ld in C2A\n", SEQ);
         exit(EXIT_FAILURE);
   }
   return TH;
}
/**********************************************************************/
/* Compute direction cosine matrix corresponding to a                 */
/* simple rotation of THETA radians about a unit vector               */
/* parallel to AXIS                                                   */

mat3x3_t SimpRot(const vec3_t AXIS, const double THETA)
{
   double CTH, STH, CTH1;
   vec3_t AX;

   CTH  = cos(THETA);
   STH  = sin(THETA);
   CTH1 = 1.0 - CTH;
   AX   = UNITV(AXIS).v;

   mat3x3_t C;
   C.mat[0][0] = CTH + AX.x * AX.x * CTH1;
   C.mat[1][0] = -AX.z * STH + AX.x * AX.y * CTH1;
   C.mat[2][0] = AX.y * STH + AX.z * AX.x * CTH1;
   C.mat[0][1] = AX.z * STH + AX.x * AX.y * CTH1;
   C.mat[1][1] = CTH + AX.y * AX.y * CTH1;
   C.mat[2][1] = -AX.x * STH + AX.y * AX.z * CTH1;
   C.mat[0][2] = -AX.y * STH + AX.z * AX.x * CTH1;
   C.mat[1][2] = AX.x * STH + AX.y * AX.z * CTH1;
   C.mat[2][2] = CTH + AX.z * AX.z * CTH1;
   return C;
}
/**********************************************************************/
vec3_t Q2AngleVec(quat_t Q)
{

   double s;

   Q = UNITQ(Q);

   if (Q.qs > 0.0)
      s = 2.0 / sinc(acos(Q.qs));
   else
      s = -2.0 / sinc(acos(-Q.qs));

   return SxV(s, Q.qv);
}
/**********************************************************************/
/*  Given body rates and quaternion, find qdot.  Ref Kane, 1.13       */
quat_t QW2QDOT(const quat_t Q, const vec3_t W)
{
   quat_t QDOT;
   QDOT.x  = 0.5 * (W.v[0] * Q.q[3] - W.v[1] * Q.q[2] + W.v[2] * Q.q[1]);
   QDOT.y  = 0.5 * (W.v[0] * Q.q[2] + W.v[1] * Q.q[3] - W.v[2] * Q.q[0]);
   QDOT.z  = 0.5 * (-W.v[0] * Q.q[1] + W.v[1] * Q.q[0] + W.v[2] * Q.q[3]);
   QDOT.qs = 0.5 * (-W.v[0] * Q.q[0] - W.v[1] * Q.q[1] - W.v[2] * Q.q[2]);
   return QDOT;
}

/****************************************************************************/
/*  Given body rates and quaternion,find qdot. ref Zimmerman, GSFC, 1969    */
/*void MQW2QDOT(double Q[4], double W[3], double DT, double QDOT[4])        */
/*{                                                                         */
/*    QDOT[0] = 0.5*(
 * W[0]*Q[3]-W[1]*Q[2]+W[2]*Q[1])+DT*(1-(Q[0]*Q[0]+Q[1]*Q[1]+Q[2]*Q[2]+Q[3]*Q[3]))*Q[0];*/
/*    QDOT[1] = 0.5*(
 * W[0]*Q[2]+W[1]*Q[3]-W[2]*Q[0])+DT*(1-(Q[0]*Q[0]+Q[1]*Q[1]+Q[2]*Q[2]+Q[3]*Q[3]))*Q[1];*/
/*    QDOT[2] =
 * 0.5*(-W[0]*Q[1]+W[1]*Q[0]+W[2]*Q[3])+DT*(1-(Q[0]*Q[0]+Q[1]*Q[1]+Q[2]*Q[2]+Q[3]*Q[3]))*Q[2];*/
/*    QDOT[3] =
 * 0.5*(-W[0]*Q[0]-W[1]*Q[1]-W[2]*Q[2])+DT*(1-(Q[0]*Q[0]+Q[1]*Q[1]+Q[2]*Q[2]+Q[3]*Q[3]))*Q[3];*/

/*}*/

/**********************************************************************/
/*  Parallel axis theorem.                                            */
/*     IB:  Central inertia matrix of a body B                        */
/*     CBA: Dircos from frame A to frame B                            */
/*     m:   Mass of B                                                 */
/*     pba: Location of mass center of B, wrt origin of A             */
/*     IBA: Inertia of B about the origin of A, expressed in A        */
mat3x3_t PARAXIS(mat3x3_t IB, mat3x3_t CBA, double m, vec3_t pba)
{
   mat3x3_t CI = MAT3X3_ZERO, CIC = MAT3X3_ZERO, pp = MAT3X3_ZERO;
   double p2;
   long i, j, k;

   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         for (k = 0; k < 3; k++) {
            CI.mat[i][j] += CBA.mat[k][i] * IB.mat[k][j];
         }
      }
   }
   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         for (k = 0; k < 3; k++) {
            CIC.mat[i][j] += CI.mat[i][k] * CBA.mat[k][j];
         }
      }
   }
   p2 = VoV(pba, pba);

   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         pp.mat[i][j] = -pba.v[i] * pba.v[j];
      }
      pp.mat[i][i] += p2;
   }

   mat3x3_t IBA;
   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         IBA.mat[i][j] = CIC.mat[i][j] + m * pp.mat[i][j];
   return IBA;
}
/******************************************************************************/
void PrincipalMOI(mat3x3_t Ib, vec3_t *Ip, mat3x3_t *CPB)
{
   double Tol = 1.0E-12;
   long MaxK  = 100;
   mat3x3_t I;
   mat3x3_t C = MAT3X3_EYE;
   mat3x3_t CI, CICT, CCPB;
   double MaxOffDiag, th, MaxEl, Swap;
   long i, j, k;
   long id, jd;
   long Jmax;
   long Done;

   I    = Ib;
   *CPB = MAT3X3_EYE;
   k    = 1;
   Done = 0;

   while (!Done) {
      MaxOffDiag = fabs(I.mat[0][1]);
      id         = 0;
      jd         = 1;
      if (fabs(I.mat[0][2]) > MaxOffDiag) {
         MaxOffDiag = fabs(I.mat[0][2]);
         id         = 0;
         jd         = 2;
      }
      if (fabs(I.mat[1][2]) > MaxOffDiag) {
         MaxOffDiag = fabs(I.mat[1][2]);
         id         = 1;
         jd         = 2;
      }

      if (I.mat[id][id] == I.mat[jd][jd]) {
         th = M_PI_4; /* pi/4 */
      }
      else {
         th = 0.5 * atan2(2.0 * I.mat[id][jd], I.mat[id][id] - I.mat[jd][jd]);
      }

      C             = MAT3X3_EYE;
      C.mat[id][id] = cos(th);
      C.mat[jd][jd] = cos(th);
      C.mat[id][jd] = sin(th);
      C.mat[jd][id] = -sin(th);

      CI   = MxM(C, I);
      CICT = MxMT(CI, C);
      CCPB = MxM(C, *CPB);
      I    = CICT;
      *CPB = CCPB;

      k++;
      if (MaxOffDiag < Tol || k > MaxK)
         Done = 1;
   }

   for (i = 0; i < 3; i++)
      Ip->v[i] = I.mat[i][i];

   /* Flip Signs to make max elements in each row positive */
   for (i = 0; i < 3; i++) {
      MaxEl = fabs(CPB->mat[i][0]);
      Jmax  = 0;
      if (fabs(CPB->mat[i][1]) > MaxEl) {
         MaxEl = fabs(CPB->mat[i][1]);
         Jmax  = 1;
      }
      if (fabs(CPB->mat[i][2]) > MaxEl) {
         MaxEl = fabs(CPB->mat[i][2]);
         Jmax  = 2;
      }
      if (CPB->mat[i][Jmax] < 0.0) {
         for (j = 0; j < 3; j++)
            CPB->mat[i][j] *= -1.0;
      }
   }
   /* Permute rows to make CPB as diagonal as possible */
   Done = 0;
   while (!Done) {
      Done = 1;
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++) {
            if (fabs(CPB->mat[j][i]) > fabs(CPB->mat[i][i])) {
               Done = 0;
               for (k = 0; k < 3; k++) {
                  Swap           = CPB->mat[j][k];
                  CPB->mat[j][k] = CPB->mat[i][k];
                  CPB->mat[i][k] = Swap;
               }
               Swap     = Ip->v[j];
               Ip->v[j] = Ip->v[i];
               Ip->v[i] = Swap;
            }
         }
      }
   }
}
/**********************************************************************/
/*  Given quaternion measurements, find body rates.  Ref Kane, 1.13   */
vec3_t Q2W(quat_t q, quat_t qdot)
{
   vec3_t w;
   w.x = 2.0 * (qdot.q[0] * q.q[3] + qdot.q[1] * q.q[2] - qdot.q[2] * q.q[1] -
                qdot.q[3] * q.q[0]);

   w.y = 2.0 * (-qdot.q[0] * q.q[2] + qdot.q[1] * q.q[3] + qdot.q[2] * q.q[0] -
                qdot.q[3] * q.q[1]);

   w.z = 2.0 * (qdot.q[0] * q.q[1] - qdot.q[1] * q.q[0] + qdot.q[2] * q.q[3] -
                qdot.q[3] * q.q[2]);
   return w;
}
/**********************************************************************/
/*  Finds rotational and translational joint partials                 */
/*  On Init, populate all matrix elements.  Else, only populate       */
/*  variable ones.                                                    */
void JointPartials(long Init, long IsSpherical, long RotSeq, long TrnSeq,
                   vec3_t ang, vec3_t sig, mat3x3_t *Gamma, vec3_t *Gs,
                   vec3_t *Gds, vec3_t s, mat3x3_t *Delta, vec3_t *Ds,
                   vec3_t *Dds)
{
   double s2, c2, s3, c3;
   long i1, i2, i3, Cyclic, i;

   if (Init) {
      if (IsSpherical) {
         *Gamma = MAT3X3_EYE;
      }
      else {
         *Gamma = MAT3X3_ZERO;
      }
      *Delta = MAT3X3_ZERO;
      *Gs    = VEC3_ZERO;
      *Gds   = VEC3_ZERO;
      *Ds    = VEC3_ZERO;
      *Dds   = VEC3_ZERO;

      i3                    = RotSeq % 10;         /* Pick off third digit */
      i2                    = (RotSeq % 100) / 10; /* Extract second digit */
      i1                    = RotSeq / 100;        /* Pick off first digit */
      Gamma->mat[i3 - 1][2] = 1.0;

      i3 = TrnSeq % 10;         /* Pick off third digit */
      i2 = (TrnSeq % 100) / 10; /* Extract second digit */
      i1 = TrnSeq / 100;        /* Pick off first digit */

      Delta->mat[i1 - 1][0] = 1.0;
      Delta->mat[i2 - 1][1] = 1.0;
      Delta->mat[i3 - 1][2] = 1.0;
   }

   if (IsSpherical) {
      *Gs = sig;
   }
   else {
      i3 = RotSeq % 10;         /* Pick off third digit */
      i2 = (RotSeq % 100) / 10; /* Extract second digit */
      i1 = RotSeq / 100;        /* Pick off first digit */

      s2 = sin(ang.y);
      c2 = cos(ang.y);
      s3 = sin(ang.z);
      c3 = cos(ang.z);

      Cyclic = (i2 - i1) * (i3 - i2) * (i3 - i1);
      /* Convert (123) style to [012] subscripts */
      i1--;
      i2--;
      i3--;
      if (Cyclic > 0) { /* 123, 231, 312 */
         Gamma->mat[i1][0] = c2 * c3;
         Gamma->mat[i1][1] = s3;
         Gamma->mat[i2][0] = -c2 * s3;
         Gamma->mat[i2][1] = c3;
         Gamma->mat[i3][0] = s2;
         Gds->v[i1] = -sig.v[0] * (sig.v[1] * s2 * c3 + sig.v[2] * c2 * s3) +
                      sig.v[1] * sig.v[2] * c3;
         Gds->v[i2] = sig.v[0] * (sig.v[1] * s2 * s3 - sig.v[2] * c2 * c3) -
                      sig.v[1] * sig.v[2] * s3;
         Gds->v[i3] = sig.v[0] * sig.v[1] * c2;
      }
      else if (Cyclic < 0) { /* 321, 132, 213 */
         Gamma->mat[i1][0] = c2 * c3;
         Gamma->mat[i1][1] = -s3;
         Gamma->mat[i2][0] = c2 * s3;
         Gamma->mat[i2][1] = c3;
         Gamma->mat[i3][0] = -s2;
         Gds->v[i1] = -sig.v[0] * (sig.v[1] * s2 * c3 + sig.v[2] * c2 * s3) -
                      sig.v[1] * sig.v[2] * c3;
         Gds->v[i2] = sig.v[0] * (-sig.v[1] * s2 * s3 + sig.v[2] * c2 * c3) -
                      sig.v[1] * sig.v[2] * s3;
         Gds->v[i3] = -sig.v[0] * sig.v[1] * c2;
      }
      else {
         fprintf(stderr, "Bogus RotSeq %ld in JointPartials\n", RotSeq);
         exit(EXIT_FAILURE);
      }
      for (i = 0; i < 3; i++)
         Gs->v[i] = Gamma->mat[i][0] * sig.v[0] + Gamma->mat[i][1] * sig.v[1] +
                    Gamma->mat[i][2] * sig.v[2];
   }

   i3 = TrnSeq % 10;         /* Pick off third digit */
   i2 = (TrnSeq % 100) / 10; /* Extract second digit */
   i1 = TrnSeq / 100;        /* Pick off first digit */
   i1--;
   i2--;
   i3--;

   Ds->v[i1] = s.x;
   Ds->v[i2] = s.y;
   Ds->v[i3] = s.z;
}
/**********************************************************************/
vec3_t ADOT2W(long IsSpherical, long Seq, vec3_t ang, vec3_t u)
{
   vec3_t w;
   double s2, c2, s3, c3;
   long i1, i2, i3, Cyclic;

   if (IsSpherical) {
      w = u;
   }
   else {
      i3 = Seq % 10;         /* Pick off third digit */
      i2 = (Seq % 100) / 10; /* Extract second digit */
      i1 = Seq / 100;        /* Pick off first digit */

      s2 = sin(ang.y);
      c2 = cos(ang.y);
      s3 = sin(ang.z);
      c3 = cos(ang.z);

      Cyclic = (i2 - i1) * (i3 - i2) * (i3 - i1);
      /* Convert (123) style to [012] subscripts */
      i1--;
      i2--;
      i3--;
      if (Cyclic > 0) { /* 123, 231, 312 */
         w.v[i1] = c2 * c3 * u.x + s3 * u.y;
         w.v[i2] = -c2 * s3 * u.x + c3 * u.y;
         w.v[i3] = s2 * u.x + u.z;
      }
      else if (Cyclic < 0) { /* 321, 132, 213 */
         w.v[i1] = c2 * c3 * u.x - s3 * u.y;
         w.v[i2] = c2 * s3 * u.x + c3 * u.y;
         w.v[i3] = -s2 * u.x + u.z;
      }
      else if ((i2 - i1 + 3) % 3 == 1) { /* 121, 232, 313 */
         i3      = (i2 + 4) % 3;
         w.v[i1] = c2 * u.x + u.z;
         w.v[i2] = s2 * s3 * u.x + c3 * u.y;
         w.v[i3] = s2 * c3 * u.x - s3 * u.y;
      }
      else if ((i2 - i1 + 3) % 3 == 2) { /* 212, 323, 131 */
         i3      = (i2 + 2) % 3;
         w.v[i1] = c2 * u.x + u.z;
         w.v[i2] = s2 * s3 * u.x + c3 * u.y;
         w.v[i3] = -s2 * c3 * u.x + s3 * u.y;
      }
      else {
         fprintf(stderr, "Bogus Seq %ld in ADOT2W\n", Seq);
         exit(EXIT_FAILURE);
      }
   }
   return w;
}
/**********************************************************************/
vec3_t W2ADOT(long Seq, vec3_t ang, vec3_t w)
{
   vec3_t adot;
   double s2, c2, s3, c3;
   long i1, i2, i3, Cyclic;

   i3 = Seq % 10;         /* Pick off third digit */
   i2 = (Seq % 100) / 10; /* Extract second digit */
   i1 = Seq / 100;        /* Pick off first digit */

   s2 = sin(ang.y);
   c2 = cos(ang.y);
   s3 = sin(ang.z);
   c3 = cos(ang.z);

   Cyclic = (i2 - i1) * (i3 - i2) * (i3 - i1);
   /* Convert (123) style to [012] subscripts */
   i1--;
   i2--;
   i3--;
   if (Cyclic > 0) { /* 123, 231, 312 */
      if (fabs(c2) < 1.0E-6)
         printf("Joint near gimbal lock in W2ADOT\n");
      adot.x = (w.v[i1] * c3 - w.v[i2] * s3) / c2;
      adot.y = w.v[i1] * s3 + w.v[i2] * c3;
      adot.z = (-w.v[i1] * c3 + w.v[i2] * s3) * s2 / c2 + w.v[i3];
   }
   else if (Cyclic < 0) { /* 321, 132, 213 */
      if (fabs(c2) < 1.0E-6)
         printf("Joint near gimbal lock in W2ADOT\n");
      adot.x = (w.v[i2] * s3 + w.v[i1] * c3) / c2;
      adot.y = w.v[i2] * c3 - w.v[i1] * s3;
      adot.z = w.v[i3] + (w.v[i2] * s3 + w.v[i1] * c3) * s2 / c2;
   }
   else if ((i2 - i1 + 3) % 3 == 1) { /* 121, 232, 313 */
      i3 = (i2 + 4) % 3;
      if (fabs(s2) < 1.0E-6)
         printf("Joint near indeterminate in W2ADOT\n");
      adot.x = (w.v[i2] * s3 + w.v[i3] * c3) / s2;
      adot.y = w.v[i2] * c3 - w.v[i3] * s3;
      adot.z = w.v[i1] - (w.v[i2] * s3 + w.v[i3] * c3) * c2 / s2;
   }
   else if ((i2 - i1 + 3) % 3 == 2) { /* 212, 323, 131 */
      i3 = (i2 + 2) % 3;
      if (fabs(s2) < 1.0E-6)
         printf("Joint near indeterminate in W2ADOT\n");
      adot.x = (w.v[i2] * s3 - w.v[i3] * c3) / s2;
      adot.y = w.v[i2] * c3 + w.v[i3] * s3;
      adot.z = w.v[i1] + (w.v[i3] * c3 - w.v[i2] * s3) * c2 / s2;
   }
   else {
      fprintf(stderr, "Bogus Seq %ld in W2ADOT\n", Seq);
      exit(EXIT_FAILURE);
   }
   return adot;
}
/**********************************************************************/
mat3x3_t W2CDOT(vec3_t w, mat3x3_t C)
{
   mat3x3_t Cdot;
   Cdot.mat[0][0] = C.mat[1][0] * w.z - C.mat[2][0] * w.y;
   Cdot.mat[1][0] = C.mat[2][0] * w.x - C.mat[0][0] * w.z;
   Cdot.mat[2][0] = C.mat[0][0] * w.y - C.mat[1][0] * w.x;
   Cdot.mat[0][1] = C.mat[1][1] * w.z - C.mat[2][1] * w.y;
   Cdot.mat[1][1] = C.mat[2][1] * w.x - C.mat[0][1] * w.z;
   Cdot.mat[2][1] = C.mat[0][1] * w.y - C.mat[1][1] * w.x;
   Cdot.mat[0][2] = C.mat[1][2] * w.z - C.mat[2][2] * w.y;
   Cdot.mat[1][2] = C.mat[2][2] * w.x - C.mat[0][2] * w.z;
   Cdot.mat[2][2] = C.mat[0][2] * w.y - C.mat[1][2] * w.x;
   return Cdot;
}
/**********************************************************************/
vec3_t CDOT2W(mat3x3_t C, mat3x3_t Cdot)
{
   vec3_t w;
   w.x = C.mat[2][0] * Cdot.mat[1][0] + C.mat[2][1] * Cdot.mat[1][1] +
         C.mat[2][2] * Cdot.mat[1][2];
   w.y = C.mat[0][1] * Cdot.mat[2][1] + C.mat[0][2] * Cdot.mat[2][2] +
         C.mat[0][0] * Cdot.mat[2][0];
   w.z = C.mat[1][2] * Cdot.mat[0][2] + C.mat[1][0] * Cdot.mat[0][0] +
         C.mat[1][1] * Cdot.mat[0][1];
   return w;
}
/**********************************************************************/

/* #ifdef __cplusplus
** }
** #endif
*/

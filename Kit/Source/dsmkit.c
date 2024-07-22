/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "dsmkit.h"

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

/**********************************************************************/
/* Given a relative position and velocity vector, find the angular    */
/* velocity at which the relative position vector is rotating.        */
void DSM_RelMotionToAngRate(double RelPosN[3], double RelVelN[3], double wn[3])
{
   double magp, phat[3], Axis[3], Vpar, Vperp[3], magvp;
   long i;

   magp = CopyUnitV(RelPosN, phat);

   VxV(RelPosN, RelVelN, Axis);
   UNITV(Axis);

   Vpar = VoV(RelVelN, phat);
   for (i = 0; i < 3; i++)
      Vperp[i] = RelVelN[i] - Vpar * phat[i];
   magvp = MAGV(Vperp);
   for (i = 0; i < 3; i++)
      wn[i] = magvp / magp * Axis[i];
}

/**********************************************************************/
/*  Some Simple Sensor Processing Functions                           */
/*  corresponding to the Sensor Models in 42sensors.c                 */
/*  Note!  These are simple, sometimes naive.  Use with care.         */
/**********************************************************************/
void DSM_GyroProcessing(struct AcType *AC)
{
   struct AcGyroType *G;
   double A0xA1[3];
   double A[3][3], b[3], Ai[3][3];
   double AtA[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
   double Atb[3]    = {0.0, 0.0, 0.0};
   double AtAi[3][3];
   long Ig, i, j;

   if (AC->Ngyro == 0) {
      /* AC->wbn populated by true S->B[0].wn in 42sensors.c */
   }
   else if (AC->Ngyro == 1) {
      G = &AC->Gyro[0];
      for (i = 0; i < 3; i++)
         AC->wbn[i] = G->Rate * G->Axis[i];
   }
   else if (AC->Ngyro == 2) {
      VxV(AC->Gyro[0].Axis, AC->Gyro[1].Axis, A0xA1);
      for (i = 0; i < 3; i++) {
         A[0][i] = AC->Gyro[0].Axis[i];
         A[1][i] = AC->Gyro[1].Axis[i];
         A[2][i] = A0xA1[i];
      }
      b[0] = AC->Gyro[0].Rate;
      b[1] = AC->Gyro[1].Rate;
      b[2] = 0.0;
      MINV3(A, Ai);
      MxV(Ai, b, AC->wbn);
   }
   else if (AC->Ngyro > 2) {
      /* Normal Equations */
      for (Ig = 0; Ig < AC->Ngyro; Ig++) {
         G = &AC->Gyro[Ig];
         for (i = 0; i < 3; i++) {
            Atb[i] += G->Rate * G->Axis[i];
            for (j = 0; j < 3; j++) {
               AtA[i][j] += G->Axis[i] * G->Axis[j];
            }
         }
      }
      MINV3(AtA, AtAi);
      MxV(AtAi, Atb, AC->wbn);
   }
}
/**********************************************************************/
void DSM_MagnetometerProcessing(struct AcType *AC)
{
   struct AcMagnetometerType *M;
   double A0xA1[3];
   double A[3][3], b[3], Ai[3][3];
   double AtA[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
   double Atb[3]    = {0.0, 0.0, 0.0};
   double AtAi[3][3];
   long Im, i, j;

   if (AC->Nmag == 0) {
      /* AC->bvb populated by true S->bvb in 42sensors.c */
   }
   else if (AC->Nmag == 1) {
      M = &AC->MAG[0];
      for (i = 0; i < 3; i++)
         AC->bvb[i] = M->Field * M->Axis[i];
   }
   else if (AC->Nmag == 2) {
      VxV(AC->MAG[0].Axis, AC->MAG[1].Axis, A0xA1);
      for (i = 0; i < 3; i++) {
         A[0][i] = AC->MAG[0].Axis[i];
         A[1][i] = AC->MAG[1].Axis[i];
         A[2][i] = A0xA1[i];
      }
      b[0] = AC->MAG[0].Field;
      b[1] = AC->MAG[1].Field;
      b[2] = 0.0;
      MINV3(A, Ai);
      MxV(Ai, b, AC->bvb);
   }
   else if (AC->Nmag > 2) {
      /* Normal Equations */
      for (Im = 0; Im < AC->Nmag; Im++) {
         M = &AC->MAG[Im];
         for (i = 0; i < 3; i++) {
            Atb[i] += M->Field * M->Axis[i];
            for (j = 0; j < 3; j++) {
               AtA[i][j] += M->Axis[i] * M->Axis[j];
            }
         }
      }
      MINV3(AtA, AtAi);
      MxV(AtAi, Atb, AC->bvb);
   }
}
/**********************************************************************/
void DSM_CssProcessing(struct AcType *AC)
{
   struct AcCssType *Css;
   double AtA[3][3]  = {{0.0}};
   double Atb[3]     = {0.0};
   double AtAi[3][3] = {{0.0}};
   double A[2][3] = {{0.0}}, b[2] = {0.0};
   long Ic, i, j;
   long Nvalid          = 0;
   double InvalidSVB[3] = {1.0, 0.0,
                           0.0}; /* Safe vector if SunValid == FALSE */

   if (AC->Ncss == 0) {
      /* AC->svb populated by true S->svb in 42sensors.c */
   }
   else {
      for (Ic = 0; Ic < AC->Ncss; Ic++) {
         Css = &AC->CSS[Ic];
         if (Css->Valid) {
            Nvalid++;
            /* Normal equations, assuming Nvalid will end up > 2 */
            for (i = 0; i < 3; i++) {
               Atb[i] += Css->Axis[i] * Css->Illum / Css->Scale;

               for (j = 0; j < 3; j++) {
                  AtA[i][j] += Css->Axis[i] * Css->Axis[j];
               }
            }
            /* In case Nvalid ends up == 2 */
            for (i = 0; i < 3; i++) {
               A[0][i] = A[1][i];
               A[1][i] = Css->Axis[i];
            }
            b[0] = b[1];
            b[1] = Css->Illum / Css->Scale;
         }
      }
      if (Nvalid > 2) {
         AC->SunValid = TRUE;
         MINV3(AtA, AtAi);
         MxV(AtAi, Atb, AC->svb);
         UNITV(AC->svb);
      }
      else if (Nvalid == 2) {
         AC->SunValid = TRUE;
         for (i = 0; i < 3; i++)
            AC->svb[i] = b[0] * A[0][i] + b[1] * A[1][i];
         UNITV(AC->svb);
      }
      else if (Nvalid == 1) {
         AC->SunValid = TRUE;
         for (i = 0; i < 3; i++)
            AC->svb[i] = Atb[i];
         UNITV(AC->svb);
      }
      else {
         AC->SunValid = FALSE;
         for (i = 0; i < 3; i++)
            AC->svb[i] = InvalidSVB[i];
      }
   }
}
/******************************************************************************/
/* This function assumes FSS FOVs don't overlap, and FSS overwrites CSS */
void DSM_FssProcessing(struct AcType *AC)
{
   struct AcFssType *FSS;
   double tanx, tany, z;
   long Ifss, i;

   for (Ifss = 0; Ifss < AC->Nfss; Ifss++) {
      FSS = &AC->FSS[Ifss];
      if (FSS->Valid) {
         AC->SunValid    = 1;
         tanx            = tan(FSS->SunAng[0]);
         tany            = tan(FSS->SunAng[1]);
         z               = 1.0 / sqrt(1.0 + tanx * tanx + tany * tany);
         FSS->SunVecS[0] = z * tanx;
         FSS->SunVecS[1] = z * tany;
         FSS->SunVecS[2] = z;
         MTxV(FSS->CB, FSS->SunVecS, FSS->SunVecB);
         for (i = 0; i < 3; i++)
            AC->svb[i] = FSS->SunVecB[i];
      }
   }
}
/**********************************************************************/
/* TODO: Weight measurements to reduce impact of "weak" axis */
void DSM_StarTrackerProcessing(struct AcType *AC)
{
   long Ist, i;
   struct AcStarTrackerType *ST;
   long Nvalid = 0;
   double qbn[4];

   if (AC->Nst == 0) {
      /* AC->qbn populated by true S->B[0].qn in 42sensors.c */
      AC->StValid = TRUE;
   }
   else {
      /* Naive averaging */
      for (i = 0; i < 4; i++)
         AC->qbn[i] = 0.0;
      for (Ist = 0; Ist < AC->Nst; Ist++) {
         ST = &AC->ST[Ist];
         if (ST->Valid) {
            Nvalid++;
            QTxQ(ST->qb, ST->qn, qbn);
            RECTIFYQ(qbn);
            for (i = 0; i < 4; i++)
               AC->qbn[i] += qbn[i];
         }
      }
      if (Nvalid > 0) {
         AC->StValid = TRUE;
         UNITQ(AC->qbn);
      }
      else {
         AC->StValid = FALSE;
         AC->qbn[3]  = 1.0;
      }
   }
}
/**********************************************************************/
void DSM_GpsProcessing(struct AcType *AC)
{
   struct AcGpsType *G;
   double DaysSinceWeek, DaysSinceRollover, DaysSinceEpoch, JD;
   long i;

   if (AC->Ngps == 0) {
      /* AC->Time, AC->PosN, AC->VelN */
      /* populated in 42sensors.c */
   }
   else {
      G = &AC->GPS[0];
      /* GPS Time is seconds since 6 Jan 1980 00:00:00.0, which is JD =
       * 2444244.5 */
      DaysSinceWeek     = G->Sec / 86400.0;
      DaysSinceRollover = DaysSinceWeek + 7.0 * G->Week;
      DaysSinceEpoch    = DaysSinceRollover + 7168.0 * G->Rollover;
      JD                = DaysSinceEpoch + 2444244.5;
      /* AC->Time is seconds since J2000, which is JD = 2451545.0 */
      AC->Time = (JD - 2451545.0) * 86400.0;

      /* Position, Velocity */
      for (i = 0; i < 3; i++) {
         AC->PosN[i] = AC->GPS[0].PosN[i];
         AC->VelN[i] = AC->GPS[0].VelN[i];
      }
   }
}
/**********************************************************************/
void DSM_AccelProcessing(struct AcType *AC) {}
/**********************************************************************/
/*  End Sensor Processing Functions                                   */
/**********************************************************************/
/*  Some Actuator Processing Functions                                */
/**********************************************************************/
void DSM_WheelProcessing(struct AcType *AC)
{
   struct AcWhlType *W;
   long Iw;

   for (Iw = 0; Iw < AC->Nwhl; Iw++) {
      W       = &AC->Whl[Iw];
      W->Tcmd = Limit(-VoV(AC->Tcmd, W->DistVec), -W->Tmax, W->Tmax);
   }
}
/**********************************************************************/
void DSM_MtbProcessing(struct AcType *AC)
{
   struct AcMtbType *M;
   long Im;

   for (Im = 0; Im < AC->Nmtb; Im++) {
      M       = &AC->MTB[Im];
      M->Mcmd = Limit(VoV(AC->Mcmd, M->DistVec), -M->Mmax, M->Mmax);
   }
}
/**********************************************************************/
/*  End Actuator Processing Functions                                   */
/**********************************************************************/
/*  Some "Comm" Processing Functions                                */
/**********************************************************************/
void DSM_CommStateProcessing(struct DSMStateType *state,
                             struct DSMStateType *commState)
{
   commState->Time = state->Time;
   commState->ID   = state->ID;
   for (int i = 0; i < 3; i++) {
      commState->VelR[i] = state->VelR[i];
      commState->PosR[i] = state->PosR[i];
      commState->VelN[i] = state->VelN[i];
      commState->PosN[i] = state->PosN[i];
      commState->wbn[i]  = state->wbn[i];
      commState->qbn[i]  = state->qbn[i];
      for (int j = 0; j < 3; j++)
         commState->CBN[i][j] = state->CBN[i][j];
   }
   commState->qbn[3] = state->qbn[3];
}

/* #ifdef __cplusplus
** }
** #endif
*/

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
#include "DSMTypes.h"
#include "navkit.h"

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
struct DSMMeasListType *DSM_GyroProcessing(struct AcType *const AC,
                                           struct DSMType *const DSM)
{
   struct AcGyroType *G;
   struct DSMNavType *Nav;
   struct DSMMeasType *meas         = NULL;
   struct DSMMeasListType *measList = NULL;
   long Ig, i, j;

   Nav = &DSM->DsmNav;

   if (Nav->NavigationActive == TRUE &&
       Nav->sensorActive[GYRO_SENSOR] == TRUE) {
      for (Ig = 0; Ig < AC->Ngyro; Ig++) {
         G = &AC->Gyro[Ig];
         if (G->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            meas                  = CreateMeas(Nav, GYRO_SENSOR, Ig);
            meas->ccsdsSeconds    = Nav->ccsdsSeconds;
            meas->ccsdsSubseconds = Nav->ccsdsSubseconds;
            meas->data[0]         = G->Rate * R2D;
            appendMeas(measList, meas);
         }
      }
   }
   else if (Nav->NavigationActive == FALSE && AC->Ngyro != 0) {
      double A0xA1[3];
      double A[3][3], b[3], Ai[3][3];
      double AtA[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
      double Atb[3]    = {0.0, 0.0, 0.0};
      double AtAi[3][3];
      if (AC->Ngyro == 1) {
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
   return (measList);
}
/**********************************************************************/
struct DSMMeasListType *DSM_MagnetometerProcessing(struct AcType *const AC,
                                                   struct DSMType *const DSM)
{
   struct AcMagnetometerType *M;
   struct DSMNavType *Nav;
   struct DSMMeasType *meas         = NULL;
   struct DSMMeasListType *measList = NULL;
   const double T2mG                = 1.0e7; // tesla to milligauss
   long Im, i, j;

   Nav = &DSM->DsmNav;

   if (Nav->NavigationActive == TRUE && Nav->sensorActive[MAG_SENSOR] == TRUE) {
      for (Im = 0; Im < AC->Nmag; Im++) {
         M = &AC->MAG[Im];
         if (M->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            meas                  = CreateMeas(Nav, MAG_SENSOR, Im);
            meas->ccsdsSeconds    = Nav->ccsdsSeconds;
            meas->ccsdsSubseconds = Nav->ccsdsSubseconds;
            meas->data[0]         = M->Field * T2mG;
            appendMeas(measList, meas);
         }
      }
   }
   else if (Nav->NavigationActive == FALSE && AC->Nmag != 0) {
      double A0xA1[3];
      double A[3][3], b[3], Ai[3][3];
      double AtA[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
      double Atb[3]    = {0.0, 0.0, 0.0};
      double AtAi[3][3];
      if (AC->Nmag == 1) {
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
   return (measList);
}
/**********************************************************************/
struct DSMMeasListType *DSM_CssProcessing(struct AcType *const AC,
                                          struct DSMType *const DSM)
{
   struct AcCssType *Css;
   struct DSMNavType *Nav;
   struct DSMMeasType *meas         = NULL;
   struct DSMMeasListType *measList = NULL;
   long Ic, i, j;

   Nav = &DSM->DsmNav;

   if (Nav->NavigationActive == TRUE && Nav->sensorActive[CSS_SENSOR] == TRUE) {
      for (Ic = 0; Ic < AC->Ncss; Ic++) {
         Css = &AC->CSS[Ic];
         if (Css->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            meas                  = CreateMeas(Nav, CSS_SENSOR, Ic);
            meas->ccsdsSeconds    = Nav->ccsdsSeconds;
            meas->ccsdsSubseconds = Nav->ccsdsSubseconds;
            meas->data[0]         = Css->Illum;
            appendMeas(measList, meas);
         }
      }
   }
   else if (Nav->NavigationActive == FALSE && AC->Ncss != 0) {
      double AtA[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
      double Atb[3]    = {0.0, 0.0, 0.0};
      double AtAi[3][3];
      double A[2][3], b[2] = {0.0};
      long Nvalid          = 0;
      double InvalidSVB[3] = {1.0, 0.0,
                              0.0}; /* Safe vector if SunValid == FALSE */
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
   return (measList);
}
/******************************************************************************/
/* This function assumes FSS FOVs don't overlap, and FSS overwrites CSS */
struct DSMMeasListType *DSM_FssProcessing(struct AcType *const AC,
                                          struct DSMType *const DSM)
{
   struct AcFssType *FSS;
   struct DSMNavType *Nav;
   struct DSMMeasType *meas         = NULL;
   struct DSMMeasListType *measList = NULL;
   long Ifss, i;

   Nav = &DSM->DsmNav;

   if (Nav->NavigationActive == TRUE && Nav->sensorActive[FSS_SENSOR] == TRUE) {
      for (Ifss = 0; Ifss < AC->Nfss; Ifss++) {
         FSS = &AC->FSS[Ifss];
         if (FSS->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            meas                  = CreateMeas(Nav, FSS_SENSOR, Ifss);
            meas->ccsdsSeconds    = Nav->ccsdsSeconds;
            meas->ccsdsSubseconds = Nav->ccsdsSubseconds;
            meas->data[0]         = FSS->SunAng[0];
            meas->data[1]         = FSS->SunAng[1];
            appendMeas(measList, meas);
         }
      }
   }
   else if (Nav->NavigationActive == FALSE && AC->Nfss != 0) {
      double tanx, tany, z;

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
   return (measList);
}
/**********************************************************************/
/* TODO: Weight measurements to reduce impact of "weak" axis */
struct DSMMeasListType *DSM_StarTrackerProcessing(struct AcType *const AC,
                                                  struct DSMType *const DSM)
{
   struct AcStarTrackerType *ST;
   struct DSMNavType *Nav;
   struct DSMMeasType *meas         = NULL;
   struct DSMMeasListType *measList = NULL;
   long Ist, i;

   Nav = &DSM->DsmNav;

   if (Nav->NavigationActive == TRUE &&
       Nav->sensorActive[STARTRACK_SENSOR] == TRUE) {
      for (Ist = 0; Ist < AC->Nst; Ist++) {
         ST = &AC->ST[Ist];
         if (ST->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            meas                  = CreateMeas(Nav, STARTRACK_SENSOR, Ist);
            meas->ccsdsSeconds    = Nav->ccsdsSeconds;
            meas->ccsdsSubseconds = Nav->ccsdsSubseconds;
            for (i = 0; i < 4; i++)
               meas->data[i] = ST->qn[i];
            appendMeas(measList, meas);
         }
      }
   }
   else if (Nav->NavigationActive == FALSE && AC->Nst != 0) {
      long Nvalid = 0;
      double qbn[4];
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
   return (measList);
}
/**********************************************************************/
struct DSMMeasListType *DSM_GpsProcessing(struct AcType *const AC,
                                          struct DSMType *const DSM)
{
   struct AcGpsType *G;
   struct DSMNavType *Nav;
   struct DSMMeasType *meas         = NULL;
   struct DSMMeasListType *measList = NULL;
   long Igps, i;

   Nav = &DSM->DsmNav;

   if (Nav->NavigationActive == TRUE && Nav->sensorActive[GPS_SENSOR] == TRUE) {
      for (Igps = 0; Igps < AC->Ngps; Igps++) {
         // TODO: handle time better
         G = &AC->GPS[Igps];
         // AC->Time = gpsTime2J2000Sec(G->Sec, G->Week, G->Rollover);
         if (G->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            meas                  = CreateMeas(Nav, GPS_SENSOR, Igps);
            meas->ccsdsSeconds    = Nav->ccsdsSeconds;
            meas->ccsdsSubseconds = Nav->ccsdsSubseconds;
            for (i = 0; i < 3; i++) {
               meas->data[i]     = G->PosN[i];
               meas->data[3 + i] = G->VelN[i];
            }
            appendMeas(measList, meas);
         }
      }
   }
   // Do this to populate AC->Time, AC->PosN, & AC->VelN
   else if (Nav->NavigationActive == FALSE && AC->Ngps != 0) {
      // double DaysSinceWeek,DaysSinceRollover,DaysSinceEpoch,JD;
      G = &AC->GPS[0];
      /* GPS Time is seconds since 6 Jan 1980 00:00:00.0, which is JD =
       * 2444244.5 */
      AC->Time = gpsTime2J2000Sec(G->Rollover, G->Week, G->Sec);

      /* Position, Velocity */
      for (i = 0; i < 3; i++) {
         AC->PosN[i] = AC->GPS[0].PosN[i];
         AC->VelN[i] = AC->GPS[0].VelN[i];
      }
   }
   return (measList);
}
/**********************************************************************/
struct DSMMeasListType *DSM_AccelProcessing(struct AcType *const AC,
                                            struct DSMType *const DSM)
{
   struct AcAccelType *Acc;
   struct DSMNavType *Nav;
   struct DSMMeasType *meas         = NULL;
   struct DSMMeasListType *measList = NULL;
   long Iacc;

   Nav = &DSM->DsmNav;

   if (Nav->NavigationActive == TRUE &&
       Nav->sensorActive[ACCEL_SENSOR] == TRUE) {
      for (Iacc = 0; Iacc < AC->Nst; Iacc++) {
         Acc = &AC->Accel[Iacc];
         if (Acc->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            meas                  = CreateMeas(Nav, ACCEL_SENSOR, Iacc);
            meas->ccsdsSeconds    = Nav->ccsdsSeconds;
            meas->ccsdsSubseconds = Nav->ccsdsSubseconds;
            meas->data[0]         = Acc->Acc;
            appendMeas(measList, meas);
         }
      }
   }
   return (measList);
}
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
/*  End Actuator Processing Functions                                 */
/**********************************************************************/
/*  Some "Comm" Processing Functions                                  */
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

      commState->svn[i] = state->svn[i];
      commState->svb[i] = state->svb[i];
      commState->bvn[i] = state->bvn[i];
      commState->bvb[i] = state->bvb[i];
   }
   commState->qbn[3] = state->qbn[3];
}
/**********************************************************************/
/*  End "Comm" Processing Functions                                   */
/**********************************************************************/

/* #ifdef __cplusplus
** }
** #endif
*/

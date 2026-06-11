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
#include "42constants.h"
#include "dcmkit.h"
#include "defineskit.h"
#include "fswkit.h"
#include "navkit.h"

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

/**********************************************************************/
/* Given a relative position and velocity vector, find the angular    */
/* velocity at which the relative position vector is rotating.        */
vec3_t DSM_RelMotionToAngRate(vec3_t RelPosN, vec3_t RelVelN)
{
   vec3_t wn;
   double magp, Vpar, magvp;
   vec3_t phat, Vperp;
   magvec3_t uAxis;
   vec3_t *const Axis = &uAxis.v;
   long i;

   uAxis = UNITV(RelPosN);
   magp  = uAxis.m;
   phat  = uAxis.v;

   *Axis = VxV(RelPosN, RelVelN);
   uAxis = UNITV(*Axis);

   Vpar = VoV(RelVelN, phat);
   for (i = 0; i < 3; i++)
      Vperp.v[i] = RelVelN.v[i] - Vpar * phat.v[i];
   magvp = MAGV(Vperp);
   for (i = 0; i < 3; i++)
      wn.v[i] = magvp / magp * Axis->v[i];
   return wn;
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

   if (Nav->NavigationActive == TRUE) {
      for (Ig = 0; Ig < AC->Ngyro; Ig++) {
         G = &AC->Gyro[Ig];
         if (Nav->sensorActive[GYRO_SENSOR][Ig] == TRUE && G->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            meas             = CreateMeas(Nav, GYRO_SENSOR, Ig);
            meas->ccsds_time = Nav->ccsds_time;
            meas->data[0]    = G->Rate * R2D;
            appendMeas(measList, meas);
         }
      }
   }
   else if (Nav->NavigationActive == FALSE && AC->Ngyro != 0) {
      vec3_t A0xA1, b, Atb = VEC3_ZERO;
      mat3x3_t A, Ai, AtAi;
      mat3x3_t AtA = MAT3X3_ZERO;
      if (AC->Ngyro == 1) {
         G = &AC->Gyro[0];
         for (i = 0; i < 3; i++)
            AC->wbn.v[i] = G->Rate * G->Axis.v[i];
      }
      else if (AC->Ngyro == 2) {
         A0xA1 = VxV(AC->Gyro[0].Axis, AC->Gyro[1].Axis);
         for (i = 0; i < 3; i++) {
            A.mat[0][i] = AC->Gyro[0].Axis.v[i];
            A.mat[1][i] = AC->Gyro[1].Axis.v[i];
            A.mat[2][i] = A0xA1.v[i];
         }
         b.x     = AC->Gyro[0].Rate;
         b.y     = AC->Gyro[1].Rate;
         b.z     = 0.0;
         Ai      = MINV3(A);
         AC->wbn = MxV(Ai, b);
      }
      else if (AC->Ngyro > 2) {
         /* Normal Equations */
         for (Ig = 0; Ig < AC->Ngyro; Ig++) {
            G = &AC->Gyro[Ig];
            for (i = 0; i < 3; i++) {
               Atb.v[i] += G->Rate * G->Axis.v[i];
               for (j = 0; j < 3; j++) {
                  AtA.mat[i][j] += G->Axis.v[i] * G->Axis.v[j];
               }
            }
         }
         AtAi    = MINV3(AtA);
         AC->wbn = MxV(AtAi, Atb);
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

   if (Nav->NavigationActive == TRUE) {
      for (Im = 0; Im < AC->Nmag; Im++) {
         M = &AC->MAG[Im];
         if (Nav->sensorActive[MAG_SENSOR][Im] == TRUE && M->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            meas             = CreateMeas(Nav, MAG_SENSOR, Im);
            meas->ccsds_time = Nav->ccsds_time;
            meas->data[0]    = M->Field * T2mG;
            appendMeas(measList, meas);
         }
      }
   }
   else if (Nav->NavigationActive == FALSE && AC->Nmag != 0) {
      vec3_t A0xA1, b, Atb;
      mat3x3_t AtA = MAT3X3_ZERO;
      mat3x3_t A, Ai, AtAi;
      if (AC->Nmag == 1) {
         M = &AC->MAG[0];
         for (i = 0; i < 3; i++)
            AC->bvb.v[i] = M->Field * M->Axis.v[i];
      }
      else if (AC->Nmag == 2) {
         A0xA1     = VxV(AC->MAG[0].Axis, AC->MAG[1].Axis);
         A.rows[0] = AC->MAG[0].Axis;
         A.rows[1] = AC->MAG[1].Axis;
         A.rows[2] = A0xA1;

         b.x     = AC->MAG[0].Field;
         b.y     = AC->MAG[1].Field;
         b.z     = 0.0;
         Ai      = MINV3(A);
         AC->bvb = MxV(Ai, b);
      }
      else if (AC->Nmag > 2) {
         /* Normal Equations */
         for (Im = 0; Im < AC->Nmag; Im++) {
            M = &AC->MAG[Im];
            for (i = 0; i < 3; i++)
               Atb.v[i] = M->Field * M->Axis.v[i];
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  AtA.mat[i][j] += M->Axis.v[i] * M->Axis.v[j];
         }
         AtAi    = MINV3(AtA);
         AC->bvb = MxV(AtAi, Atb);
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

   if (Nav->NavigationActive == TRUE) {
      for (Ic = 0; Ic < AC->Ncss; Ic++) {
         Css = &AC->CSS[Ic];
         if (Nav->sensorActive[CSS_SENSOR][Ic] == TRUE && Css->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            meas             = CreateMeas(Nav, CSS_SENSOR, Ic);
            meas->ccsds_time = Nav->ccsds_time;
            meas->data[0]    = Css->Illum;
            appendMeas(measList, meas);
         }
      }
   }
   else if (Nav->NavigationActive == FALSE && AC->Ncss != 0) {

      magvec3_t usvb;

      mat3x3_t AtA = MAT3X3_ZERO;
      vec3_t Atb;
      mat3x3_t AtAi;
      double A[2][3], b[2] = {0.0};
      long Nvalid = 0;
      /* Safe vector if SunValid == FALSE */
      const vec3_t InvalidSVB = VEC3_PXAXIS;
      for (Ic = 0; Ic < AC->Ncss; Ic++) {
         Css = &AC->CSS[Ic];
         if (Css->Valid) {
            Nvalid++;
            /* Normal equations, assuming Nvalid will end up > 2 */
            for (i = 0; i < 3; i++)
               Atb.v[i] = Css->Illum / Css->Scale * Css->Axis.v[i];
            for (i = 0; i < 3; i++)
               for (j = 0; j < 3; j++)
                  AtA.mat[i][j] += Css->Axis.v[i] * Css->Axis.v[j];

            /* In case Nvalid ends up == 2 */
            for (i = 0; i < 3; i++) {
               A[0][i] = A[1][i];
               A[1][i] = Css->Axis.v[i];
            }
            b[0] = b[1];
            b[1] = Css->Illum / Css->Scale;
         }
      }
      if (Nvalid > 2) {
         AC->SunValid = TRUE;
         AtAi         = MINV3(AtA);
         AC->svb      = MxV(AtAi, Atb);
         usvb         = UNITV(AC->svb);
         AC->svb      = usvb.v;
      }
      else if (Nvalid == 2) {
         AC->SunValid = TRUE;
         for (i = 0; i < 3; i++)
            AC->svb.v[i] = b[0] * A[0][i] + b[1] * A[1][i];
         usvb    = UNITV(AC->svb);
         AC->svb = usvb.v;
      }
      else if (Nvalid == 1) {
         AC->SunValid = TRUE;
         AC->svb      = Atb;
         usvb         = UNITV(AC->svb);
         AC->svb      = usvb.v;
      }
      else {
         AC->SunValid = FALSE;
         AC->svb      = InvalidSVB;
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
   long Ifss;

   Nav = &DSM->DsmNav;

   if (Nav->NavigationActive == TRUE) {
      for (Ifss = 0; Ifss < AC->Nfss; Ifss++) {
         FSS = &AC->FSS[Ifss];
         if (Nav->sensorActive[FSS_SENSOR][Ifss] == TRUE &&
             FSS->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            AC->SunValid     = 1;
            meas             = CreateMeas(Nav, FSS_SENSOR, Ifss);
            meas->ccsds_time = Nav->ccsds_time;
            meas->data[0]    = FSS->SunAng[0];
            meas->data[1]    = FSS->SunAng[1];
            appendMeas(measList, meas);
         }
      }
   }
   else if (Nav->NavigationActive == FALSE && AC->Nfss != 0) {

      for (Ifss = 0; Ifss < AC->Nfss; Ifss++) {
         FSS = &AC->FSS[Ifss];
         if (FSS->Valid) {
            AC->SunValid = 1;
            switch (FSS->type) {
               case CONVENTIONAL_FSS: {
                  double tanx    = tan(FSS->SunAng[0]);
                  double tany    = tan(FSS->SunAng[1]);
                  double z       = 1.0 / sqrt(1.0 + tanx * tanx + tany * tany);
                  FSS->SunVecS.x = z * tanx;
                  FSS->SunVecS.y = z * tany;
                  FSS->SunVecS.x = z;
               } break;
               case GS_FSS: {
                  double ct      = cos(FSS->SunAng[0]);
                  double st      = sin(FSS->SunAng[0]);
                  double cp      = cos(FSS->SunAng[1]);
                  double sp      = sin(FSS->SunAng[1]);
                  FSS->SunVecS.x = ct;
                  FSS->SunVecS.y = st * cp;
                  FSS->SunVecS.z = st * sp;
               } break;
               default:
                  fprintf(stderr, "Invalid FSS Type. How did it get this far? "
                                  "Exiting...\n");
                  exit(EXIT_FAILURE);
            }
            FSS->SunVecB = MTxV(FSS->CB, FSS->SunVecS);
            AC->svb      = FSS->SunVecB;
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

   if (Nav->NavigationActive == TRUE) {
      for (Ist = 0; Ist < AC->Nst; Ist++) {
         ST = &AC->ST[Ist];
         if (Nav->sensorActive[STARTRACK_SENSOR][Ist] == TRUE &&
             ST->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            meas             = CreateMeas(Nav, STARTRACK_SENSOR, Ist);
            meas->ccsds_time = Nav->ccsds_time;
            for (i = 0; i < 4; i++)
               meas->data[i] = ST->qn.q[i];
            appendMeas(measList, meas);
         }
      }
   }
   else if (Nav->NavigationActive == FALSE && AC->Nst != 0) {
      long Nvalid = 0;
      quat_t qbn;
      /* Naive averaging */
      AC->qbn = QUAT_ZERO;
      for (Ist = 0; Ist < AC->Nst; Ist++) {
         ST = &AC->ST[Ist];
         if (ST->Valid) {
            Nvalid++;
            qbn = QTxQ(ST->qb, ST->qn);
            qbn = RECTIFYQ(qbn);
            for (i = 0; i < 4; i++)
               AC->qbn.q[i] += qbn.q[i];
         }
      }
      if (Nvalid > 0) {
         AC->StValid = TRUE;
         AC->qbn     = UNITQ(AC->qbn);
      }
      else {
         AC->StValid = FALSE;
         AC->qbn.qs  = 1.0;
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

   if (Nav->NavigationActive == TRUE) {
      for (Igps = 0; Igps < AC->Ngps; Igps++) {
         // TODO: handle time better
         G = &AC->GPS[Igps];
         // AC->Time = gpsTime2J2000Sec(G->Sec, G->Week, G->Rollover);
         if (Nav->sensorActive[GPS_SENSOR][Igps] == TRUE && G->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            meas             = CreateMeas(Nav, GPS_SENSOR, Igps);
            meas->ccsds_time = Nav->ccsds_time;
            for (i = 0; i < 3; i++) {
               meas->data[i]     = G->PosN.v[i];
               meas->data[3 + i] = G->VelN.v[i];
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
      AC->PosN = AC->GPS[0].PosN;
      AC->VelN = AC->GPS[0].VelN;
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

   if (Nav->NavigationActive == TRUE) {
      for (Iacc = 0; Iacc < AC->Nacc; Iacc++) {
         Acc = &AC->Accel[Iacc];
         if (Nav->sensorActive[ACCEL_SENSOR][Iacc] == TRUE &&
             Acc->Valid == TRUE) {
            if (measList == NULL) {
               measList = malloc(sizeof(struct DSMMeasListType));
               InitMeasList(measList);
            }
            meas             = CreateMeas(Nav, ACCEL_SENSOR, Iacc);
            meas->ccsds_time = Nav->ccsds_time;
            meas->data[0]    = Acc->Acc;
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
struct DSMStateType DSM_CommStateProcessing(struct DSMStateType state)
{
   return state;
}
/**********************************************************************/
/*  End "Comm" Processing Functions                                   */
/**********************************************************************/

/* #ifdef __cplusplus
** }
** #endif
*/

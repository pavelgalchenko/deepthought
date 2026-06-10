/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "Ac.h"

/* #ifdef __cplusplus
** namespace _42 {
** using namespace Kit;
** #endif
*/

extern void WriteToFile(FILE *StateFile, struct AcType *AC);
extern void WriteToGmsec(struct AcType *AC);
extern void WriteToSocket(SOCKET Socket, struct AcType *AC);
extern void ReadFromFile(FILE *StateFile, struct AcType *AC);
extern void ReadFromGmsec(struct AcType *AC);
extern void ReadFromSocket(SOCKET Socket, struct AcType *AC);

#ifdef _AC_STANDALONE_
/**********************************************************************/
/* This function copies needed parameters from the SC structure to    */
/* the AC structure.  This is a crude first pass.  It only allocates  */
/* memory for the structures, and counts on the data to be filled in  */
/* via messages.                                                      */
void AllocateAC(struct AcType *AC)
{

   /* Bodies */
   AC->Nb = 2;
   if (AC->Nb > 0) {
      AC->B = (struct AcBodyType *)calloc(AC->Nb, sizeof(struct AcBodyType));
   }

   /* Joints */
   AC->Ng = 1;
   if (AC->Ng > 0) {
      AC->G = (struct AcJointType *)calloc(AC->Ng, sizeof(struct AcJointType));
   }

   /* Wheels */
   AC->Nwhl = 4;
   if (AC->Nwhl > 0) {
      AC->Whl = (struct AcWhlType *)calloc(AC->Nwhl, sizeof(struct AcWhlType));
   }

   /* Magnetic Torquer Bars */
   AC->Nmtb = 3;
   if (AC->Nmtb > 0) {
      AC->MTB = (struct AcMtbType *)calloc(AC->Nmtb, sizeof(struct AcMtbType));
   }

   /* Thrusters */
   AC->Nthr = 0;
   if (AC->Nthr > 0) {
      AC->Thr = (struct AcThrType *)calloc(AC->Nthr, sizeof(struct AcThrType));
   }

   /* Control Moment Gyros */

   /* Gyro Axes */
   AC->Ngyro = 3;
   if (AC->Ngyro > 0) {
      AC->Gyro =
          (struct AcGyroType *)calloc(AC->Ngyro, sizeof(struct AcGyroType));
   }

   /* Magnetometer Axes */
   AC->Nmag = 3;
   if (AC->Nmag > 0) {
      AC->MAG = (struct AcMagnetometerType *)calloc(
          AC->Nmag, sizeof(struct AcMagnetometerType));
   }

   /* Coarse Sun Sensors */
   AC->Ncss = 8;
   if (AC->Ncss > 0) {
      AC->CSS = (struct AcCssType *)calloc(AC->Ncss, sizeof(struct AcCssType));
   }

   /* Fine Sun Sensors */
   AC->Nfss = 1;
   if (AC->Nfss > 0) {
      AC->FSS = (struct AcFssType *)calloc(AC->Nfss, sizeof(struct AcFssType));
   }

   /* Star Trackers */
   AC->Nst = 1;
   if (AC->Nst > 0) {
      AC->ST = (struct AcStarTrackerType *)calloc(
          AC->Nst, sizeof(struct AcStarTrackerType));
   }

   /* GPS */
   AC->Ngps = 1;
   if (AC->Ngps > 0) {
      AC->GPS = (struct AcGpsType *)calloc(AC->Ngps, sizeof(struct AcGpsType));
   }

   /* Accelerometer Axes */
}
/**********************************************************************/
void InitAC(struct AcType *AC)
{
   AC->Init = 1;

   AC->EchoEnabled = 1;

   /* Controllers */
   AC->CfsCtrl.Init = 1;
}
#endif
/**********************************************************************/
/*  Some Simple Sensor Processing Functions                           */
/*  corresponding to the Sensor Models in 42sensors.c                 */
/*  Note!  These are simple, sometimes naive.  Use with care.         */
/**********************************************************************/
void GyroProcessing(struct AcType *AC)
{
   struct AcGyroType *G;
   mat3x3_t A, Ai, AtAi, AtA = MAT3X3_ZERO;
   vec3_t A0xA1, b, Atb = VEC3_ZERO;
   long Ig, i, j;

   if (AC->Ngyro == 0) {
      /* AC->wbn populated by true S->B[0].wn in 42sensors.c */
   }
   else if (AC->Ngyro == 1) {
      G       = &AC->Gyro[0];
      AC->wbn = SxV(G->Rate, G->Axis);
   }
   else if (AC->Ngyro == 2) {
      A0xA1     = VxV(AC->Gyro[0].Axis, AC->Gyro[1].Axis);
      A.rows[0] = AC->Gyro[0].Axis;
      A.rows[1] = AC->Gyro[1].Axis;
      A.rows[2] = A0xA1;

      b.v[0]  = AC->Gyro[0].Rate;
      b.v[1]  = AC->Gyro[1].Rate;
      b.v[2]  = 0.0;
      Ai      = MINV3(A);
      AC->wbn = MxV(Ai, b);
   }
   else if (AC->Ngyro > 2) {
      /* Normal Equations */
      for (Ig = 0; Ig < AC->Ngyro; Ig++) {
         G = &AC->Gyro[Ig];
         for (i = 0; i < 3; i++) {
            Atb.v[i] += G->Rate * G->Axis.v[i];
            for (j = 0; j < 3; j++)
               AtA.mat[i][j] += G->Axis.v[i] * G->Axis.v[j];
         }
      }
      AtAi    = MINV3(AtA);
      AC->wbn = MxV(AtAi, Atb);
   }
}
/**********************************************************************/
void MagnetometerProcessing(struct AcType *AC)
{
   struct AcMagnetometerType *M;
   mat3x3_t A, Ai, AtAi, AtA = MAT3X3_ZERO;
   vec3_t A0xA1, b, Atb = VEC3_ZERO;
   long Im, i, j;

   if (AC->Nmag == 0) {
      /* AC->bvb populated by true S->bvb in 42sensors.c */
   }
   else if (AC->Nmag == 1) {
      M       = &AC->MAG[0];
      AC->bvb = SxV(M->Field, M->Axis);
   }
   else if (AC->Nmag == 2) {
      A0xA1     = VxV(AC->MAG[0].Axis, AC->MAG[1].Axis);
      A.rows[0] = AC->MAG[0].Axis;
      A.rows[1] = AC->MAG[1].Axis;
      A.rows[2] = A0xA1;

      b.v[0]  = AC->MAG[0].Field;
      b.v[1]  = AC->MAG[1].Field;
      b.v[2]  = 0.0;
      Ai      = MINV3(A);
      AC->bvb = MxV(Ai, b);
   }
   else if (AC->Nmag > 2) {
      /* Normal Equations */
      for (Im = 0; Im < AC->Nmag; Im++) {
         M = &AC->MAG[Im];
         for (i = 0; i < 3; i++) {
            Atb.v[i] += M->Field * M->Axis.v[i];
            for (j = 0; j < 3; j++)
               AtA.mat[i][j] += M->Axis.v[i] * M->Axis.v[j];
         }
      }
      AtAi    = MINV3(AtA);
      AC->bvb = MxV(AtAi, Atb);
   }
}
/**********************************************************************/
void CssProcessing(struct AcType *AC)
{
   struct AcCssType *Css;
   vec3_t Atb = VEC3_ZERO;
   mat3x3_t AtA, AtAi;
   vec3_t A[2];
   double b[2] = {0.0};
   long Ic, i, j;
   long Nvalid       = 0;
   vec3_t InvalidSVB = VEC3_PXAXIS; /* Safe vector if SunValid == FALSE */

   if (AC->Ncss == 0) {
      /* AC->svb populated by true S->svb in 42sensors.c */
   }
   else {
      for (Ic = 0; Ic < AC->Ncss; Ic++) {
         Css = &AC->CSS[Ic];
         if (Css->Valid) {
            Nvalid++;

            b[0] = b[1];
            b[1] = Css->Illum / Css->Scale;
            /* Normal equations, assuming Nvalid will end up > 2 */
            for (i = 0; i < 3; i++) {
               Atb.v[i] += Css->Axis.v[i] * Css->Illum / Css->Scale;
               for (j = 0; j < 3; j++)
                  AtA.mat[i][j] += Css->Axis.v[i] * Css->Axis.v[j];
            }

            /* In case Nvalid ends up == 2 */
            A[0] = A[1];
            A[1] = Css->Axis;
         }
      }
      if (Nvalid > 2) {
         AC->SunValid = TRUE;
         AtAi         = MINV3(AtA);
         AC->svb      = MxV(AtAi, Atb);
         AC->svb      = UNITV(AC->svb).v;
      }
      else if (Nvalid == 2) {
         AC->SunValid = TRUE;
         for (i = 0; i < 3; i++)
            AC->svb.v[i] = b[0] * A[0].v[i] + b[1] * A[1].v[i];
         AC->svb = UNITV(AC->svb).v;
      }
      else if (Nvalid == 1) {
         AC->SunValid = TRUE;
         AC->svb      = Atb;
         AC->svb      = UNITV(AC->svb).v;
      }
      else {
         AC->SunValid = FALSE;
         AC->svb      = InvalidSVB;
      }
   }
}
/******************************************************************************/
/* This function assumes FSS FOVs don't overlap, and FSS overwrites CSS */
void FssProcessing(struct AcType *AC)
{
   struct AcFssType *FSS;
   double tanx, tany, z;
   long Ifss;

   for (Ifss = 0; Ifss < AC->Nfss; Ifss++) {
      FSS = &AC->FSS[Ifss];
      if (FSS->Valid) {
         AC->SunValid   = 1;
         tanx           = tan(FSS->SunAng[0]);
         tany           = tan(FSS->SunAng[1]);
         z              = 1.0 / sqrt(1.0 + tanx * tanx + tany * tany);
         FSS->SunVecS.x = z * tanx;
         FSS->SunVecS.y = z * tany;
         FSS->SunVecS.z = z;
         FSS->SunVecB   = MTxV(FSS->CB, FSS->SunVecS);
         AC->svb        = FSS->SunVecB;
      }
   }
}
/**********************************************************************/
/* TODO: Weight measurements to reduce impact of "weak" axis */
void StarTrackerProcessing(struct AcType *AC)
{
   long Ist, i;
   struct AcStarTrackerType *ST;
   long Nvalid = 0;
   quat_t qbn;

   if (AC->Nst == 0) {
      /* AC->qbn populated by true S->B[0].qn in 42sensors.c */
      AC->StValid = TRUE;
   }
   else {
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
}
/**********************************************************************/
void GpsProcessing(struct AcType *AC)
{
   struct AcGpsType *G;
   double DaysSinceWeek, DaysSinceRollover, DaysSinceEpoch, JD;

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
      AC->PosN = AC->GPS[0].PosN;
      AC->VelN = AC->GPS[0].VelN;
   }
}
/**********************************************************************/
void AccelProcessing(struct AcType *AC __attribute__((unused)))
    __attribute__((unused));
void AccelProcessing(struct AcType *AC __attribute__((unused))) {}
/**********************************************************************/
/*  End Sensor Processing Functions                                   */
/**********************************************************************/
/*  Some Actuator Processing Functions                                */
/**********************************************************************/
void WheelProcessing(struct AcType *AC)
{
   struct AcWhlType *W;
   long Iw;

   for (Iw = 0; Iw < AC->Nwhl; Iw++) {
      W       = &AC->Whl[Iw];
      W->Tcmd = Limit(-VoV(AC->Tcmd, W->DistVec), -W->Tmax, W->Tmax);
   }
}
/**********************************************************************/
void MtbProcessing(struct AcType *AC)
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
void AcFsw(struct AcType *AC)
{
   struct AcCfsCtrlType *C;
   struct AcJointType *G;
   vec3_t L1, L2, L3;
   vec3_t HxB;
   double AngErr;
   long i, j;

   C = &AC->CfsCtrl;
   G = &AC->G[0];

   if (C->Init) {
      C->Init = 0;
      for (i = 0; i < 3; i++)
         FindPDGains(AC->MOI.mat[i][i], 0.1, 0.7, &C->Kr.v[i], &C->Kp.v[i]);
      C->Kunl = 1.0E6;
      FindPDGains(100.0, 0.2, 1.0, &G->AngRateGain.v[0], &G->AngGain.v[0]);
      G->MaxAngRate.x = 1.0 * D2R;
      G->MaxTrq.x     = 10.0;
   }

   /* .. Sensor Processing */
   GyroProcessing(AC);
   MagnetometerProcessing(AC);
   CssProcessing(AC);
   FssProcessing(AC);
   StarTrackerProcessing(AC);
   GpsProcessing(AC);

   /* .. Commanded Attitude */
   if (AC->GPS[0].Valid) {
      L3 = UNITV(AC->PosN).v;
      L2 = VxV(AC->PosN, AC->VelN);
      L2 = UNITV(L2).v;
      L3 = UNITV(L3).v;
      L2 = VNegElem(L2);
      L3 = VNegElem(L3);

      L1              = VxV(L2, L3);
      L1              = UNITV(L1).v;
      AC->CLN.rows[0] = L1;
      AC->CLN.rows[1] = L2;
      AC->CLN.rows[2] = L3;
      AC->qln         = C2Q(AC->CLN);
      AC->wln.v[0]    = 0.0;
      AC->wln.v[1]    = -MAGV(AC->VelN) / MAGV(AC->PosN);
      AC->wln.v[2]    = 0.0;
   }
   else {
      AC->CLN = MAT3X3_EYE;
      AC->qln = QUAT_EYE;
      AC->wln = VEC3_ZERO;
   }

   /* .. Attitude Control */
   if (AC->StValid) {
      AC->qbr = QxQT(AC->qbn, AC->qln);
      AC->qbr = RECTIFYQ(AC->qbr);
   }
   else
      AC->qbr = QUAT_EYE;

   C->werr = VmVElem(AC->wbn, AC->wln);
   for (i = 0; i < 3; i++) {
      C->therr.v[i] = Limit(2.0 * AC->qbr.qv.v[i], -0.1, 0.1);
      AC->Tcmd.v[i] = Limit(
          -C->Kr.v[i] * C->werr.v[i] - C->Kp.v[i] * C->therr.v[i], -0.1, 0.1);
   }
   /* .. Momentum Management */
   for (i = 0; i < 3; i++) {
      AC->Hvb.v[i] = AC->MOI.mat[i][i] * AC->wbn.v[i];
      for (j = 0; j < AC->Nwhl; j++)
         AC->Hvb.v[i] += AC->Whl[j].Axis.v[i] * AC->Whl[j].H;
   }
   HxB      = VxV(AC->Hvb, AC->bvb);
   AC->Mcmd = SxV(C->Kunl, HxB);

   /* .. Solar Array Steering */
   G->Cmd.Ang.x = atan2(AC->svb.x, AC->svb.z);
   AngErr       = fmod(G->Ang.x - G->Cmd.Ang.x, AC->TwoPi);
   if (AngErr > AC->Pi)
      AngErr -= AC->TwoPi;
   if (AngErr < -AC->Pi)
      AngErr += AC->TwoPi;
   G->Cmd.AngRate.x = -G->AngGain.x / G->AngRateGain.x * AngErr;
   G->Cmd.AngRate.x =
       Limit(G->Cmd.AngRate.x, -G->MaxAngRate.x, G->MaxAngRate.x);

   /* .. Actuator Processing */
   WheelProcessing(AC);
   MtbProcessing(AC);
}
#ifdef _AC_STANDALONE_
/**********************************************************************/
int main(int argc, char **argv)
{
   FILE *ParmDumpFile;
   char FileName[120];
   struct AcType AC;
   SOCKET Socket;
   char hostname[20] = "localhost";
   int Port          = 10001;

   if (argc > 1) {
      AC.ID = atoi(argv[1]);
      Port  = 10001 + AC.ID;
   }

   AllocateAC(&AC);

   Socket = InitSocketClient(hostname, Port, 1);

   /* Load parms */
   AC.EchoEnabled = 1;
   ReadFromSocket(Socket, &AC);

   InitAC(&AC);
   AcFsw(&AC);

   sprintf(FileName, "./Database/AcParmDump%02ld.txt", AC.ID);
   ParmDumpFile = fopen(FileName, "wt");
   WriteToFile(ParmDumpFile, &AC);
   fclose(ParmDumpFile);
   WriteToSocket(Socket, &AC);

   while (1) {
      ReadFromSocket(Socket, &AC);
      AcFsw(&AC);
      WriteToSocket(Socket, &AC);
   }

   return (0);
}
#endif
/* #ifdef __cplusplus
** }
** #endif
*/

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
/*  Substantial contributions to this model provided                  */
/*  by Jeffrey Calixto, 2019 summer intern.                           */
/*                                                                    */
/*  Acceleration of a point A fixed in SC[Isc].B[0], expressed in     */
/*  B[0].  Due to 42's accounting of forces (esp. gravity), the       */
/*  gravity-gradient force accounting for the offset from             */
/*  SC[Isc].B[0].cm to A must be explicitly accounted for.  All other */
/*  gravity terms apply equally to A and B[0].  (Assuming gravity-    */
/*  gradient from non-spherical primary and 3rd-body forces is        */
/*  negligible.)  Surface forces are included in S->         */
void AccelerometerModel(struct SCType *S)
{
   struct AccelType *A;
   struct BodyType *B;
   struct NodeType *N;
   double p[3];
   double r, Coef, rhatn[3], rhat[3], rhatop;
   double AccGGB[3], AccGG, Axis[3];
   double NodeQN[4], AvgQN[4];
   double dvn[3], dvb[3], AvgAcc;
   long i;
   long Ia;
   double PrevBias;

   for (Ia = 0; Ia < S->Nacc; Ia++) {
      A = &S->Accel[Ia];

      for (i = 0; i < 3; i++)
         A->AccumAccN[i] += S->AccN[i] * DTSIM;
      A->SampleCounter++;
      if (A->SampleCounter >= A->MaxCounter) {
         A->SampleCounter = 0;
         B                = &S->B[0];
         N                = &B->Node[A->Node];

         /* Grav-grad force (see Hughes, p.246, eq (56)) */
         for (i = 0; i < 3; i++)
            AccGGB[i] = 0.0;
         if (GGActive) {
            r    = MAGV(S->PosN);
            Coef = -3.0 * Orb[S->RefOrb].mu / (r * r * r);
            CopyUnitV(S->PosN, rhatn);
            MxV(B->CN, rhatn, rhat);
            MxV(B->CN, B->pn, p);
            for (i = 0; i < 3; i++)
               p[i] += N->PosB[i];
            rhatop = VoV(rhat, p);
            for (i = 0; i < 3; i++) {
               AccGGB[i] = Coef * (p[i] - 3.0 * rhat[i] * rhatop);
            }
         }

         QTxV(N->qb, A->Axis, Axis);
         AccGG = VoV(AccGGB, Axis);

         for (i = 0; i < 3; i++) {
            dvn[i]          = A->AccumAccN[i] + N->VelN[i] - A->PrevVelN[i];
            A->PrevVelN[i]  = N->VelN[i];
            A->AccumAccN[i] = 0.0;
         }
         QxQ(N->qb, B->qn, NodeQN);
         for (i = 0; i < 4; i++)
            AvgQN[i] = A->PrevQN[i] + NodeQN[i];
         UNITQ(AvgQN);
         for (i = 0; i < 4; i++)
            A->PrevQN[i] = NodeQN[i];
         QxV(AvgQN, dvn, dvb);
         A->DV      = VoV(dvb, Axis);
         AvgAcc     = A->DV / A->SampleTime;
         A->TrueAcc = AvgAcc + AccGG;

         PrevBias = A->CorrCoef * A->Bias;
         A->Bias  = PrevBias + A->BiasStabCoef * GaussianRandom(RNG);
         A->AccError =
             0.5 * (A->Bias + PrevBias) + A->DVRWCoef * GaussianRandom(RNG);

         A->MeasAcc =
             Limit(A->Scale * A->TrueAcc + A->AccError, -A->MaxAcc, A->MaxAcc);

         A->DV =
             A->MeasAcc * A->SampleTime + A->DVNoiseCoef * GaussianRandom(RNG);

         A->Counts = (long)(A->DV / A->SampleTime / A->Quant + 0.5);

         A->MeasAcc = ((double)A->Counts) * A->Quant;

         S->AC.Accel[Ia].Acc   = A->MeasAcc;
         S->AC.Accel[Ia].Valid = TRUE;
      }
      else
         S->AC.Accel[Ia].Valid = FALSE;
   }
}
/**********************************************************************/
void GyroModel(struct SCType *S)
{
   struct GyroType *G;
   struct BodyType *B;
   struct NodeType *N;
   long Ig;
   double Axis[3];
   double PrevBias, RateError, PrevAngle;
   long Counts, PrevCounts;

   for (Ig = 0; Ig < S->Ngyro; Ig++) {
      G = &S->Gyro[Ig];

      G->SampleCounter++;
      if (G->SampleCounter >= G->MaxCounter) {
         G->SampleCounter = 0;
         B                = &S->B[0];
         N                = &B->Node[G->Node];
         QTxV(N->qb, G->Axis, Axis);
         G->TrueRate = VoV(N->AngVelB, Axis);

         PrevBias = G->CorrCoef * G->Bias;
         G->Bias  = PrevBias + G->BiasStabCoef * GaussianRandom(RNG);
         RateError =
             0.5 * (G->Bias + PrevBias) + G->ARWCoef * GaussianRandom(RNG);

         G->MeasRate =
             Limit(G->Scale * G->TrueRate + RateError, -G->MaxRate, G->MaxRate);

         PrevAngle = G->Angle;
         G->Angle  = PrevAngle + G->MeasRate * G->SampleTime +
                    G->AngNoiseCoef * GaussianRandom(RNG);

         PrevCounts = (long)(PrevAngle / G->Quant + 0.5);
         Counts     = (long)(G->Angle / G->Quant + 0.5);

         G->MeasRate =
             ((double)(Counts - PrevCounts)) * G->Quant / G->SampleTime;

         S->AC.Gyro[Ig].Rate  = G->MeasRate;
         S->AC.Gyro[Ig].Valid = TRUE;
      }
      else
         S->AC.Gyro[Ig].Valid = FALSE;
   }
}
/**********************************************************************/
void MagnetometerModel(struct SCType *S)
{
   struct MagnetometerType *MAG;
   long Counts, Imag;
   double Signal;

   for (Imag = 0; Imag < S->Nmag; Imag++) {
      MAG = &S->MAG[Imag];

      MAG->SampleCounter++;
      if (MAG->SampleCounter >= MAG->MaxCounter) {
         MAG->SampleCounter = 0;

         Signal = MAG->Scale * VoV(S->bvb, MAG->Axis) +
                  MAG->Noise * GaussianRandom(RNG);
         Signal     = Limit(Signal, -MAG->Saturation, MAG->Saturation);
         Counts     = (long)(Signal / MAG->Quant + 0.5);
         MAG->Field = ((double)Counts) * MAG->Quant;

         S->AC.MAG[Imag].Field = MAG->Field;
         S->AC.MAG[Imag].Valid = TRUE;
      }
      else
         S->AC.MAG[Imag].Valid = FALSE;
   }
}
/**********************************************************************/
/* This model credit Paul McKee, summer intern 2018                   */
void CssModel(struct SCType *S)
{
   struct CssType *CSS;
   long Counts, Icss;
   double Signal;
   double SoA;
   double svb[3];

   for (Icss = 0; Icss < S->Ncss; Icss++) {
      CSS = &S->CSS[Icss];

      CSS->SampleCounter++;
      if (CSS->SampleCounter >= CSS->MaxCounter) {
         CSS->SampleCounter = 0;

         if (S->Eclipse) {
            CSS->Valid = FALSE;
            CSS->Illum = 0.0;
         }
         else {
            MxV(S->B[CSS->Body].CN, S->svn, svb);
            SoA = VoV(svb, CSS->Axis);
            if (SoA > CSS->CosFov) {
               /* Sun within FOV */
               CSS->Valid = TRUE;
               Signal     = CSS->Scale * SoA;
               Counts     = (long)(Signal / CSS->Quant + 0.5);
               CSS->Illum = ((double)Counts) * CSS->Quant;
            }
            else {
               /* Sun not in FOV */
               CSS->Valid = FALSE;
               CSS->Illum = 0.0;
            }
         }

#ifdef _ENABLE_GUI_
         CSS->Albedo = 0.0;
         if (AlbedoActive) {
            FindCssAlbedo(S, CSS);
            Signal      = CSS->Scale * CSS->Albedo;
            Counts      = (long)(Signal / CSS->Quant + 0.5);
            CSS->Illum += ((double)Counts) * CSS->Quant;
            CSS->Illum  = Limit(CSS->Illum, 0.0, CSS->Scale);
         }
#endif

         /* Copy into AC structure */
         S->AC.CSS[Icss].Illum = CSS->Illum;
      }
      else
         CSS->Valid = FALSE;

      S->AC.CSS[Icss].Valid = CSS->Valid;
   }
}
/**********************************************************************/
void FssModel(struct SCType *S)
{
   struct FssType *FSS;
   static struct RandomProcessType *FssNoise;
   double svs[3], SunAng[2], Signal;
   long Counts;
   static long First = 1;
   long Ifss, i;

   if (First) {
      First    = 0;
      FssNoise = CreateRandomProcess(10);
   }

   for (Ifss = 0; Ifss < S->Nfss; Ifss++) {
      FSS = &S->FSS[Ifss];

      FSS->SampleCounter++;
      if (FSS->SampleCounter >= FSS->MaxCounter) {
         FSS->SampleCounter = 0;

         if (S->Eclipse) {
            FSS->Valid = FALSE;
         }
         else {
            MxV(FSS->CB, S->svb, svs);
            long fov_condition = TRUE;
            const double svsh  = svs[FSS->H_Axis];
            const double svsv  = svs[FSS->V_Axis];
            const double svsb  = svs[FSS->BoreAxis];

            switch (FSS->type) {
               case CONVENTIONAL_FSS: {
                  SunAng[0]     = atan2(svsh, svsb);
                  SunAng[1]     = atan2(svsv, svsb);
                  fov_condition = fabs(SunAng[0]) < FSS->FovHalfAng[0] &&
                                  fabs(SunAng[1]) < FSS->FovHalfAng[1];
               } break;
               case GS_FSS: {
                  SunAng[0]     = atan2(svsv, svsh);
                  SunAng[1]     = atan2(sqrt(svsv * svsv + svsh * svsh), svsb);
                  fov_condition = SunAng[0] < FSS->FovHalfAng[0];
               } break;
               default:
                  fprintf(stderr, "Invalid FSS Type. How did it get this far? "
                                  "Exiting...\n");
                  exit(EXIT_FAILURE);
            }
            if (fov_condition && svs[FSS->BoreAxis] > 0.0) {
               FSS->Valid = TRUE;
            }
            else {
               FSS->Valid = FALSE;
            }
         }

         if (FSS->Valid) {
            for (i = 0; i < 2; i++) {
               Signal         = SunAng[i] + FSS->NEA * GaussianRandom(FssNoise);
               Counts         = (long)(Signal / FSS->Quant + 0.5);
               FSS->SunAng[i] = ((double)Counts) * FSS->Quant;
            }
         }
         else {
            FSS->SunAng[0] = 0.0;
            FSS->SunAng[1] = 0.0;
         }

         for (i = 0; i < 2; i++)
            S->AC.FSS[Ifss].SunAng[i] = FSS->SunAng[i];
      }
      else
         FSS->Valid = FALSE;

      S->AC.FSS[Ifss].Valid = FSS->Valid;
   }
}
/**********************************************************************/
void StarTrackerModel(struct SCType *S)
{
   struct StarTrackerType *ST;
   struct NodeType *N;
   static struct RandomProcessType *StNoise;
   struct WorldType *W;
   double qsn[4], Qnoise[4];
   double BoS, OrbRad, LimbAng, NadirVecB[3], BoN;
   double mvn[3], MoonDist, mvb[3], BoM;
   double qsb[4];
   static long First = 1;
   long Ist, i;

   if (First) {
      First   = 0;
      StNoise = CreateRandomProcess(1);
   }

   for (Ist = 0; Ist < S->Nst; Ist++) {
      ST = &S->ST[Ist];

      ST->SampleCounter++;
      if (ST->SampleCounter >= ST->MaxCounter) {
         ST->SampleCounter = 0;
         N                 = &S->B[0].Node[ST->Node];

         ST->Valid = TRUE;
         /* Sun Occultation? */
         BoS = VoV(ST->CB[ST->BoreAxis], S->svb);
         if (BoS > ST->CosSunExclAng)
            ST->Valid = FALSE;
         /* Earth Occultation? (Generalized to whatever world we're orbiting) */
         W       = &World[Orb[S->RefOrb].World];
         OrbRad  = MAGV(S->PosN);
         LimbAng = asin(W->rad / OrbRad);
         MxV(S->B[0].CN, S->CLN[2], NadirVecB);
         BoN = VoV(ST->CB[ST->BoreAxis], NadirVecB);
         if (BoN > cos(LimbAng + ST->EarthExclAng))
            ST->Valid = FALSE;
         /* Moon Occultation? (Only worked out if orbiting Earth.  Customize as
          * needed)*/
         if ((ST->Valid == TRUE) && (Orb[S->RefOrb].World == EARTH)) {
            for (i = 0; i < 3; i++)
               mvn[i] = World[LUNA].eph.PosN[i] - S->PosN[i];
            MoonDist = UNITV(mvn);
            LimbAng  = asin(World[LUNA].rad / MoonDist);
            MxV(S->B[0].CN, mvn, mvb);
            BoM = VoV(ST->CB[ST->BoreAxis], mvb);
            if (BoM > cos(LimbAng + ST->MoonExclAng))
               ST->Valid = FALSE;
         }
         if (ST->Valid) {
            QxQ(ST->qb, N->qb, qsb);
            QxQ(qsb, S->B[0].qn, qsn);
            /* Add Noise in ST frame */
            for (i = 0; i < 3; i++)
               Qnoise[i] = 0.5 * ST->NEA[i] * GaussianRandom(StNoise);
            Qnoise[3] = 1.0;
            UNITQ(Qnoise);
            QxQ(Qnoise, qsn, ST->qn);
         }

         for (i = 0; i < 4; i++) {
            S->AC.ST[Ist].qn[i] = ST->qn[i];
         }
      }
      else
         ST->Valid = FALSE;

      S->AC.ST[Ist].Valid = ST->Valid;
   }
}
/**********************************************************************/
void GpsModel(struct SCType *S)
{
   struct GpsType *GPS;
   static struct RandomProcessType *GpsNoise;
   double PosW[3], MagPosW;
   long Ig, i;
   static long First = 1;

   if (First) {
      // TODO: AC->Time needs to be initialized before DsmSensorModule() is
      // called, but not here if possible
      S->AC.Time = DynTime;
      First      = 0;
      GpsNoise   = CreateRandomProcess(2);
   }

   if (Orb[S->RefOrb].World == EARTH) {
      for (Ig = 0; Ig < S->Ngps; Ig++) {
         GPS = &S->GPS[Ig];

         GPS->SampleCounter++;
         if (GPS->SampleCounter >= GPS->MaxCounter) {
            GPS->SampleCounter = 0;

            GPS->Valid = TRUE;

            GPS->Rollover = GpsRollover;
            GPS->Week     = GpsWeek;
            GPS->Sec = GpsSecond + GPS->TimeNoise * GaussianRandom(GpsNoise);

            for (i = 0; i < 3; i++) {
               GPS->PosN[i] =
                   S->PosN[i] + GPS->PosNoise * GaussianRandom(GpsNoise);
               GPS->VelN[i] =
                   S->VelN[i] + GPS->VelNoise * GaussianRandom(GpsNoise);
            }
            MxV(World[EARTH].CWN, S->PosN, PosW);
            MxV(World[EARTH].CWN, GPS->PosN, GPS->PosW);
            MxV(World[EARTH].CWN, GPS->VelN, GPS->VelW);
            /* Subtract Earth rotation velocity */
            GPS->VelW[0] -= -World[EARTH].w * PosW[1];
            GPS->VelW[1] -= World[EARTH].w * PosW[0];

            MagPosW  = MAGV(GPS->PosW);
            GPS->Lng = atan2(GPS->PosW[1], GPS->PosW[0]);
            GPS->Lat = asin(GPS->PosW[2] / MagPosW);
            GPS->Alt = MagPosW - World[EARTH].rad;
            ECEFToWGS84(GPS->PosW, &GPS->WgsLat, &GPS->WgsLng, &GPS->WgsAlt);

            S->AC.GPS[Ig].Rollover = GPS->Rollover;
            S->AC.GPS[Ig].Week     = GPS->Week;
            S->AC.GPS[Ig].Sec      = GPS->Sec;

            for (i = 0; i < 3; i++) {
               S->AC.GPS[Ig].PosN[i] = GPS->PosN[i];
               S->AC.GPS[Ig].VelN[i] = GPS->VelN[i];
               S->AC.GPS[Ig].PosW[i] = GPS->PosW[i];
               S->AC.GPS[Ig].VelW[i] = GPS->VelW[i];
            }
            S->AC.GPS[Ig].Lng    = GPS->Lng;
            S->AC.GPS[Ig].Lat    = GPS->Lat;
            S->AC.GPS[Ig].Alt    = GPS->Alt;
            S->AC.GPS[Ig].WgsLng = GPS->WgsLng;
            S->AC.GPS[Ig].WgsLat = GPS->WgsLat;
            S->AC.GPS[Ig].WgsAlt = GPS->WgsAlt;
         }
         else
            GPS->Valid = FALSE;

         S->AC.GPS[Ig].Valid = GPS->Valid;
      }
   }
   else {
      for (Ig = 0; Ig < S->Ngps; Ig++) {
         S->GPS[Ig].Valid    = FALSE;
         S->AC.GPS[Ig].Valid = S->GPS[Ig].Valid;
      }
   }
}
/**********************************************************************/
void FullFgsModel(struct FgsType *F, struct SCType *S)
{
   struct OpticsType *O;
   struct BodyType *B;
   struct NodeType *N;
   double ar, StarVecFr[3];
   double qbb0[4], qb0r[4], qbr[4];
   double StarVecB[3], StarPosB[3];
   double FldPntB[3], FldDirB[3], OutPntB[3], OutDirB[3];
   double x, y, qfb[4], CFB[3][3];
   long i;
   long OutSC, OutBody;
   long InAp, NumOptPassed;

   /* Create Guide Star in Fr, transform to R */
   ar                     = sqrt(1.0 - F->Hr * F->Hr - F->Vr * F->Vr);
   StarVecFr[F->H_Axis]   = F->Hr;
   StarVecFr[F->V_Axis]   = F->Vr;
   StarVecFr[F->BoreAxis] = ar;

   QTxV(F->qr, StarVecFr, F->StarVecR);

   /* Create FldPntB, FldDirB from StarVecR */
   O = &F->Opt[0];
   B = &S->B[O->Body];
   QxQT(B->qn, S->B[0].qn, qbb0);
   QxQT(S->B[0].qn, S->AC.qrn, qb0r);
   QxQ(qbb0, qb0r, qbr);
   QxV(qbr, F->StarVecR, StarVecB);

   InAp = OpticalFieldPoint(StarVecB, O, FldPntB, FldDirB);
   if (!InAp) {
      fprintf(stderr, "Hmm.  FGS field point is not within aperture.\n");
      exit(EXIT_FAILURE);
   }

   NumOptPassed =
       OpticalTrain(F->Opt[0].SC, F->Opt[0].Body, FldPntB, FldDirB, F->Nopt,
                    F->Opt, &OutSC, &OutBody, OutPntB, OutDirB);

   if (NumOptPassed == F->Nopt)
      F->Valid = TRUE;
   else
      F->Valid = FALSE;

   /* Find H,V from OutPntB */
   O = &F->Opt[F->Nopt - 1];
   B = &S->B[O->Body];
   N = &B->Node[O->Node];
   QxQ(F->qb, N->qb, qfb);
   Q2C(qfb, CFB);
   x = 0.0;
   y = 0.0;
   for (i = 0; i < 3; i++)
      StarPosB[i] = OutPntB[i] - N->PosB[i];
   for (i = 0; i < 3; i++) {
      x += CFB[F->H_Axis][i] * StarPosB[i];
      y += CFB[F->V_Axis][i] * StarPosB[i];
   }
   F->H = x / O->FocLen;
   F->V = y / O->FocLen;

   /* Apply PSF Image */
   /* Accumulate GW */

   F->SampleCounter++;
   if (F->SampleCounter >= F->MaxCounter) {
      F->SampleCounter = 0;
      /* Centroiding */
      /* Output Angles */
      F->Ang[F->BoreAxis] = 0.0;
      F->Ang[F->H_Axis]   = (F->V - F->Vr);
      F->Ang[F->V_Axis]   = -(F->H - F->Hr);
   }
}
/**********************************************************************/
void SimpleFgsModel(struct FgsType *F, struct SCType *S)
{
   struct BodyType *B;
   struct NodeType *N;
   double ar;
   double StarVecFr[3];
   double qbb0[4], qb0r[4], qbr[4], qfb[4];
   double StarVecB[3], StarVecF[3];

   F->SampleCounter++;
   if (F->SampleCounter >= F->MaxCounter) {
      F->SampleCounter = 0;
      B                = &S->B[F->Body];
      N                = &B->Node[F->Node];

      /* Create Guide Star in Fr, transform to R */
      F->Hr                  = 0.0;
      F->Vr                  = 0.0;
      ar                     = sqrt(1.0 - F->Hr * F->Hr - F->Vr * F->Vr);
      StarVecFr[F->H_Axis]   = F->Hr;
      StarVecFr[F->V_Axis]   = F->Vr;
      StarVecFr[F->BoreAxis] = ar;

      /* CFrR = CFB */
      QTxV(F->qb, StarVecFr, F->StarVecR);

      /* Transform Guide Star from Fr to F */
      QxQT(B->qn, S->B[0].qn, qbb0);
      QxQT(S->B[0].qn, S->AC.qrn, qb0r);
      QxQ(qbb0, qb0r, qbr);
      QxV(qbr, F->StarVecR, StarVecB);
      QxQ(F->qb, N->qb, qfb);
      QxV(qfb, StarVecB, StarVecF);
      F->H = StarVecF[F->H_Axis] + F->NEA * GaussianRandom(RNG);
      F->V = StarVecF[F->V_Axis] + F->NEA * GaussianRandom(RNG);

      F->Ang[F->BoreAxis] = 0.0;
      F->Ang[F->H_Axis]   = (F->V - F->Vr);
      F->Ang[F->V_Axis]   = -(F->H - F->Hr);

      if (StarVecF[F->BoreAxis] > 0.0 && fabs(F->H) < F->FovHalfAng[0] &&
          fabs(F->V) < F->FovHalfAng[1]) {
         F->Valid = TRUE;
      }
      else {
         F->Valid = FALSE;
         F->H     = 0.0;
         F->V     = 0.0;
      }
   }
}
/**********************************************************************/
void FgsModel(struct SCType *S)
{
   struct FgsType *F;
   long Ifgs;

   for (Ifgs = 0; Ifgs < S->Nfgs; Ifgs++) {
      F = &S->Fgs[Ifgs];

      if (F->HasOptics)
         FullFgsModel(F, S);
      else
         SimpleFgsModel(F, S);
   }
}
/**********************************************************************/
/*  This function is called at the simulation rate.  Sub-sampling of  */
/*  sensors should be done on a case-by-case basis.                   */
void Sensors(struct SCType *S)
{

   double evn[3], evb[3];
   long i, j, k, DOF;
   struct AcType *AC;
   struct JointType *G;

   AC = &S->AC;

   /* Ephemeris */
   AC->EphValid = 1;
   for (i = 0; i < 3; i++) {
      AC->svn[i] = S->svn[i];
      AC->bvn[i] = S->bvn[i];
   }

   /* Accelerometer */
   if (S->Nacc > 0) {
      AccelerometerModel(S);
   }

   /* Gyro */
   if (S->Ngyro == 0) {
      for (i = 0; i < 3; i++)
         AC->wbn[i] = S->B[0].wn[i];
   }
   else {
      GyroModel(S);
   }

   /* Magnetometer */
   if (Orb[S->RefOrb].World == EARTH) {
      AC->MagValid = TRUE;
      if (S->Nmag == 0) {
         for (i = 0; i < 3; i++)
            AC->bvb[i] = S->bvb[i];
      }
      else {
         MagnetometerModel(S);
      }
   }
   else {
      AC->MagValid = FALSE;
   }

   /* Sun Sensors */
   if (S->Ncss == 0 && S->Nfss == 0) {
      if (S->Eclipse) {
         AC->SunValid = FALSE;
      }
      else {
         AC->SunValid = TRUE;
         MxV(S->B[0].CN, S->svn, AC->svb);
      }
   }
   if (S->Ncss > 0) {
      CssModel(S);
   }
   if (S->Nfss > 0) {
      FssModel(S);
   }

   /* Star Tracker */
   if (S->Nst == 0) {
      for (i = 0; i < 4; i++)
         AC->qbn[i] = S->B[0].qn[i];
      Q2C(AC->qbn, AC->CBN);
   }
   else {
      StarTrackerModel(S);
   }

   /* GPS Receiver (or ephem model) */
   if (S->Ngps == 0) {
      AC->Time = DynTime;
      for (i = 0; i < 3; i++) {
         AC->PosN[i] = S->PosN[i];
         AC->VelN[i] = S->VelN[i];
      }
   }
   else {
      GpsModel(S);
   }

   /* Earth Sensor */
   for (i = 0; i < 3; i++)
      evn[i] = -S->PosN[i];
   UNITV(evn);
   MxV(S->B[0].CN, evn, evb);
   if (evb[2] > 0.866) {
      AC->ES.Valid = TRUE;
      AC->ES.Roll  = evb[1];
      AC->ES.Pitch = -evb[0];
   }
   else {
      AC->ES.Valid = FALSE;
      AC->ES.Roll  = 0.0;
      AC->ES.Pitch = 0.0;
   }

   /* Gimbal Angles */
   for (i = 0; i < AC->Ng; i++) {
      G   = &S->G[i];
      DOF = AC->G[i].RotDOF;
      for (j = 0; j < DOF; j++) {
         AC->G[i].Ang[j]     = G->Ang[j];
         AC->G[i].AngRate[j] = G->AngRate[j];
      }
      for (j = 0; j < 3; j++) {
         for (k = 0; k < 3; k++) {
            AC->G[i].COI[j][k] = G->COI[j][k];
         }
      }
      DOF = AC->G[i].TrnDOF;
      for (j = 0; j < DOF; j++) {
         AC->G[i].Pos[j]     = G->Pos[j];
         AC->G[i].PosRate[j] = G->PosRate[j];
      }
   }

   /* Wheel Tachs */
   for (i = 0; i < S->Nw; i++) {
      AC->Whl[i].H = S->Whl[i].H;
      AC->Whl[i].w = S->Whl[i].w;
   }

   /* Fine Guidance Sensors */
   if (S->Nfgs > 0) {
      FgsModel(S);
   }
}

/* #ifdef __cplusplus
** }
** #endif
*/

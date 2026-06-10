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
void AccelerometerModel(struct OrbitType *orb, struct SCType *S)
{
   struct AccelType *A;
   struct BodyType *B;
   struct NodeType *N;
   double r, Coef, rhatop, AccGG, AvgAcc;
   vec3 AccGGB, Axis, rhatn, rhat, p;
   vec3 dvn, dvb;
   quat NodeQN, AvgQN;
   long Ia;
   double PrevBias;

   for (Ia = 0; Ia < S->Nacc; Ia++) {
      A = &S->Accel[Ia];

      for (int i = 0; i < 3; i++)
         A->AccumAccN.v[i] += S->AccN.v[i] * DTSIM;
      A->SampleCounter++;
      if (A->SampleCounter >= A->MaxCounter) {
         A->SampleCounter = 0;
         B                = &S->B[0];
         N                = &B->Node[A->Node];

         /* Grav-grad force (see Hughes, p.246, eq (56)) */
         AccGGB = VEC3_ZERO;
         if (GGActive) {
            r    = MAGV(S->PosN);
            Coef = -3.0 * orb->mu / (r * r * r);
            CopyUnitV(S->PosN, &rhatn);
            rhat   = MxV(B->CN, rhatn);
            p      = MxV(B->CN, B->pn);
            p      = VpVElem(p, N->PosB);
            rhatop = VoV(rhat, p);
            for (int i = 0; i < 3; i++)
               AccGGB.v[i] = Coef * (p.v[i] - 3.0 * rhat.v[i] * rhatop);
         }

         Axis  = QTxV(N->qb, A->Axis);
         AccGG = VoV(AccGGB, Axis);
         for (int i = 0; i < 3; i++)
            dvn.v[i] = A->AccumAccN.v[i] + N->VelN.v[i] - A->PrevVelN.v[i];

         A->PrevVelN  = N->VelN;
         A->AccumAccN = VEC3_ZERO;
         NodeQN       = QxQ(N->qb, B->qn);
         for (int i = 0; i < 4; i++)
            AvgQN.q[i] = A->PrevQN.q[i] + NodeQN.q[i];
         AvgQN      = UNITQ(AvgQN);
         A->PrevQN  = NodeQN;
         dvb        = QxV(AvgQN, dvn);
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
   vec3 Axis;
   double PrevBias, RateError, PrevAngle;
   long Counts, PrevCounts;

   for (Ig = 0; Ig < S->Ngyro; Ig++) {
      G = &S->Gyro[Ig];

      G->SampleCounter++;
      if (G->SampleCounter >= G->MaxCounter) {
         G->SampleCounter = 0;
         B                = &S->B[0];
         N                = &B->Node[G->Node];
         Axis             = QTxV(N->qb, G->Axis);
         G->TrueRate      = VoV(N->AngVelB, Axis);

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

         Signal     = MAG->Scale * VoV(S->bvb, MAG->Axis) +
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
   vec3 svb;

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
            svb = MxV(S->B[CSS->Body].CN, S->svn);
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
   vec3 svs;
   double SunAng[2], Signal;
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
            svs                = MxV(FSS->CB, S->svb);
            long fov_condition = TRUE;
            const double svsh  = svs.v[FSS->H_Axis];
            const double svsv  = svs.v[FSS->V_Axis];
            const double svsb  = svs.v[FSS->BoreAxis];

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
            if (fov_condition && svs.v[FSS->BoreAxis] > 0.0) {
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
void StarTrackerModel(struct WorldType *const worlds,
                      struct OrbitType *const orb, struct SCType *S)
{
   struct StarTrackerType *ST;
   struct NodeType *N;
   static struct RandomProcessType *StNoise;
   struct WorldType *W;
   quat qsn, Qnoise, qsb;
   double BoS, OrbRad, LimbAng, BoN, MoonDist, BoM;
   vec3 mvn, mvb, NadirVecB;
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
         BoS = VoV(ST->CB.rows[ST->BoreAxis], S->svb);
         if (BoS > ST->CosSunExclAng)
            ST->Valid = FALSE;
         /* Earth Occultation? (Generalized to whatever world we're orbiting)
          */
         W         = &worlds[orb->World];
         OrbRad    = MAGV(S->PosN);
         LimbAng   = asin(W->rad / OrbRad);
         NadirVecB = MxV(S->B[0].CN, S->CLN.rows[2]);
         BoN       = VoV(ST->CB.rows[ST->BoreAxis], NadirVecB);
         if (BoN > cos(LimbAng + ST->EarthExclAng))
            ST->Valid = FALSE;
         /* Moon Occultation? (Only worked out if orbiting Earth.  Customize
          * as needed)*/
         if ((ST->Valid == TRUE) && (orb->World == EARTH)) {
            mvn      = VmVElem(worlds[LUNA].eph.PosN, S->PosN);
            MoonDist = UNITV(&mvn);
            LimbAng  = asin(worlds[LUNA].rad / MoonDist);
            mvb      = MxV(S->B[0].CN, mvn);
            BoM      = VoV(ST->CB.rows[ST->BoreAxis], mvb);
            if (BoM > cos(LimbAng + ST->MoonExclAng))
               ST->Valid = FALSE;
         }
         if (ST->Valid) {
            qsb = QxQ(ST->qb, N->qb);
            qsn = QxQ(qsb, S->B[0].qn);
            /* Add Noise in ST frame */
            for (i = 0; i < 3; i++)
               Qnoise.qv.v[i] = 0.5 * ST->NEA[i] * GaussianRandom(StNoise);
            Qnoise.qs = 1.0;
            Qnoise    = UNITQ(Qnoise);
            ST->qn    = QxQ(Qnoise, qsn);
         }

         S->AC.ST[Ist].qn = ST->qn;
      }
      else
         ST->Valid = FALSE;

      S->AC.ST[Ist].Valid = ST->Valid;
   }
}
/**********************************************************************/
void GpsModel(struct WorldType *const worlds, struct OrbitType *const orb,
              struct SCType *S)
{
   struct GpsType *GPS;
   static struct RandomProcessType *GpsNoise;
   vec3 PosW;
   double MagPosW;
   long Ig, i;
   static long First = 1;

   if (First) {
      // TODO: AC->Time needs to be initialized before DsmSensorModule() is
      // called, but not here if possible
      S->AC.Time = DynTime;
      First      = 0;
      GpsNoise   = CreateRandomProcess(2);
   }

   if (orb->World == EARTH) {
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
               GPS->PosN.v[i] =
                   S->PosN.v[i] + GPS->PosNoise * GaussianRandom(GpsNoise);
               GPS->VelN.v[i] =
                   S->VelN.v[i] + GPS->VelNoise * GaussianRandom(GpsNoise);
            }
            PosW      = MxV(worlds[EARTH].CWN, S->PosN);
            GPS->PosW = MxV(worlds[EARTH].CWN, GPS->PosN);
            GPS->VelW = MxV(worlds[EARTH].CWN, GPS->VelN);
            /* Subtract Earth rotation velocity */
            const double W_w = GetWorldW(JD_TDB_MJD, &worlds[EARTH]);

            GPS->VelW.v[0] -= -W_w * PosW.v[1];
            GPS->VelW.v[1] -= W_w * PosW.v[0];

            MagPosW  = MAGV(GPS->PosW);
            GPS->Lng = atan2(GPS->PosW.y, GPS->PosW.x);
            GPS->Lat = asin(GPS->PosW.z / MagPosW);
            GPS->Alt = MagPosW - worlds[EARTH].rad;
            ECEFToWGS84(GPS->PosW, &GPS->WgsLat, &GPS->WgsLng, &GPS->WgsAlt);

            S->AC.GPS[Ig].Rollover = GPS->Rollover;
            S->AC.GPS[Ig].Week     = GPS->Week;
            S->AC.GPS[Ig].Sec      = GPS->Sec;

            S->AC.GPS[Ig].PosN = GPS->PosN;
            S->AC.GPS[Ig].VelN = GPS->VelN;
            S->AC.GPS[Ig].PosW = GPS->PosW;
            S->AC.GPS[Ig].VelW = GPS->VelW;

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
   double ar;
   quat qbb0, qb0r, qbr, qfb;
   vec3 StarVecB, StarPosB, StarVecFr;
   vec3 FldPntB, FldDirB, OutPntB, OutDirB;
   mat3x3 CFB;
   double x, y;
   long i;
   long OutSC, OutBody;
   long InAp, NumOptPassed;

   /* Create Guide Star in Fr, transform to R */
   ar                       = sqrt(1.0 - F->Hr * F->Hr - F->Vr * F->Vr);
   StarVecFr.v[F->H_Axis]   = F->Hr;
   StarVecFr.v[F->V_Axis]   = F->Vr;
   StarVecFr.v[F->BoreAxis] = ar;

   F->StarVecR = QTxV(F->qr, StarVecFr);

   /* Create FldPntB, FldDirB from StarVecR */
   O        = &F->Opt[0];
   B        = &S->B[O->Body];
   qbb0     = QxQT(B->qn, S->B[0].qn);
   qb0r     = QxQT(S->B[0].qn, S->AC.qrn);
   qbr      = QxQ(qbb0, qb0r);
   StarVecB = QxV(qbr, F->StarVecR);

   InAp = OpticalFieldPoint(StarVecB, O, &FldPntB, &FldDirB);
   if (!InAp) {
      fprintf(stderr, "Hmm.  FGS field point is not within aperture.\n");
      exit(EXIT_FAILURE);
   }

   NumOptPassed =
       OpticalTrain(F->Opt[0].SC, F->Opt[0].Body, FldPntB, FldDirB, F->Nopt,
                    F->Opt, &OutSC, &OutBody, &OutPntB, &OutDirB);

   if (NumOptPassed == F->Nopt)
      F->Valid = TRUE;
   else
      F->Valid = FALSE;

   /* Find H,V from OutPntB */
   O        = &F->Opt[F->Nopt - 1];
   B        = &S->B[O->Body];
   N        = &B->Node[O->Node];
   qfb      = QxQ(F->qb, N->qb);
   CFB      = Q2C(qfb);
   x        = 0.0;
   y        = 0.0;
   StarPosB = VmVElem(OutPntB, N->PosB);
   for (i = 0; i < 3; i++) {
      x += CFB.mat[F->H_Axis][i] * StarPosB.v[i];
      y += CFB.mat[F->V_Axis][i] * StarPosB.v[i];
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
      F->Ang.v[F->BoreAxis] = 0.0;
      F->Ang.v[F->H_Axis]   = (F->V - F->Vr);
      F->Ang.v[F->V_Axis]   = -(F->H - F->Hr);
   }
}
/**********************************************************************/
void SimpleFgsModel(struct FgsType *F, struct SCType *S)
{
   struct BodyType *B;
   struct NodeType *N;
   double ar;
   quat qbb0, qb0r, qbr, qfb;
   vec3 StarVecFr, StarVecB, StarVecF;

   F->SampleCounter++;
   if (F->SampleCounter >= F->MaxCounter) {
      F->SampleCounter = 0;
      B                = &S->B[F->Body];
      N                = &B->Node[F->Node];

      /* Create Guide Star in Fr, transform to R */
      F->Hr                    = 0.0;
      F->Vr                    = 0.0;
      ar                       = sqrt(1.0 - F->Hr * F->Hr - F->Vr * F->Vr);
      StarVecFr.v[F->H_Axis]   = F->Hr;
      StarVecFr.v[F->V_Axis]   = F->Vr;
      StarVecFr.v[F->BoreAxis] = ar;

      /* CFrR = CFB */
      F->StarVecR = QTxV(F->qb, StarVecFr);

      /* Transform Guide Star from Fr to F */
      qbb0     = QxQT(B->qn, S->B[0].qn);
      qb0r     = QxQT(S->B[0].qn, S->AC.qrn);
      qbr      = QxQ(qbb0, qb0r);
      StarVecB = QxV(qbr, F->StarVecR);
      qfb      = QxQ(F->qb, N->qb);
      StarVecF = QxV(qfb, StarVecB);
      F->H     = StarVecF.v[F->H_Axis] + F->NEA * GaussianRandom(RNG);
      F->V     = StarVecF.v[F->V_Axis] + F->NEA * GaussianRandom(RNG);

      F->Ang.v[F->BoreAxis] = 0.0;
      F->Ang.v[F->H_Axis]   = (F->V - F->Vr);
      F->Ang.v[F->V_Axis]   = -(F->H - F->Hr);

      if (StarVecF.v[F->BoreAxis] > 0.0 && fabs(F->H) < F->FovHalfAng[0] &&
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
void Sensors(struct WorldType *const worlds, struct OrbitType *const orb,
             struct SCType *S)
{

   vec3 evn, evb;
   long i, j, DOF;
   struct AcType *AC;
   struct JointType *G;

   AC = &S->AC;

   /* Ephemeris */
   AC->EphValid = 1;
   AC->svn      = S->svn;
   AC->bvn      = S->bvn;

   /* Accelerometer */
   if (S->Nacc > 0) {
      AccelerometerModel(orb, S);
   }

   /* Gyro */
   if (S->Ngyro == 0)
      AC->wbn = S->B[0].wn;
   else
      GyroModel(S);

   /* Magnetometer */
   if (orb->World == EARTH) {
      AC->MagValid = TRUE;
      if (S->Nmag == 0)
         AC->bvb = S->bvb;
      else
         MagnetometerModel(S);
   }
   else {
      AC->MagValid = FALSE;
   }

   /* Sun Sensors */
   if (S->Ncss == 0 && S->Nfss == 0) {
      if (S->Eclipse)
         AC->SunValid = FALSE;
      else {
         AC->SunValid = TRUE;
         AC->svb      = MxV(S->B[0].CN, S->svn);
      }
   }
   if (S->Ncss > 0)
      CssModel(S);
   if (S->Nfss > 0)
      FssModel(S);

   /* Star Tracker */
   if (S->Nst == 0) {
      AC->qbn = S->B[0].qn;
      AC->CBN = Q2C(AC->qbn);
   }
   else
      StarTrackerModel(worlds, orb, S);

   /* GPS Receiver (or ephem model) */
   if (S->Ngps == 0) {
      AC->Time = DynTime;
      AC->PosN = S->PosN;
      AC->VelN = S->VelN;
   }
   else
      GpsModel(worlds, orb, S);

   /* Earth Sensor */
   evn = VNegElem(S->PosN);
   UNITV(&evn);
   evb = MxV(S->B[0].CN, evn);
   if (evb.z > 0.866) {
      AC->ES.Valid = TRUE;
      AC->ES.Roll  = evb.y;
      AC->ES.Pitch = -evb.x;
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
         AC->G[i].Ang.v[j]     = G->Ang.v[j];
         AC->G[i].AngRate.v[j] = G->AngRate.v[j];
      }
      AC->G[i].COI = G->COI;

      DOF = AC->G[i].TrnDOF;
      for (j = 0; j < DOF; j++) {
         AC->G[i].Pos.v[j]     = G->Pos.v[j];
         AC->G[i].PosRate.v[j] = G->PosRate.v[j];
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

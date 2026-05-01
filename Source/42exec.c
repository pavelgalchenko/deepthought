/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#define DECLARE_GLOBALS
#include "42.h"
#undef DECLARE_GLOBALS

/* #ifdef __cplusplus
** namespace _42 {
** using namespace Kit;
** #endif
*/

#ifdef _ENABLE_GUI_
extern int HandoffToGui(int argc, char **argv);
#endif

/**********************************************************************/
void ReportProgress(void)
{
#define PROGRESSPERCENT 10

   static long ProgressPercent = 0;
   static long ProgressCtr     = 0;
   static double ProgressTime  = 0.0;

   if (TimeMode == FAST_TIME) {

      if (SimTime >= ProgressTime) {
         ProgressCtr++;
         ProgressTime =
             (double)(ProgressCtr * PROGRESSPERCENT) / 100.0 * STOPTIME;
         printf("    42 Case %s is %3.1li%% Complete at Time = %12.3f\n",
                InOutPath, ProgressPercent, SimTime);
         ProgressPercent += PROGRESSPERCENT;
      }
   }
}
/**********************************************************************/
void ManageFlags(long *const nout, long *const GLnout, int *set_nout)
{
   static long iout   = 1000000;
   static long GLiout = 1000000;

   if (!*set_nout) {
      *set_nout = TRUE;
      *nout     = RationalRoundUp(RationalDivide(DTOUT_RAT, DTSIM_RAT));
      *GLnout   = RationalRoundUp(RationalDivide(DTOUTGL_RAT, DTSIM_RAT));
   }

   iout++;
   if (iout >= *nout) {
      iout    = 0;
      OutFlag = TRUE;
   }
   else
      OutFlag = FALSE;

   GLiout++;
   if (GLiout >= *GLnout) {
      GLiout    = 0;
      GLOutFlag = TRUE;
   }
   else
      GLOutFlag = FALSE;
}
/**********************************************************************/
long AdvanceTime(void)
{
   static long itime    = 0;
   static long PrevTick = 0;
   static long CurrTick = 1;
   long Done            = 0;

   // TODO: This is where the real fun is

   /* Advance time to next Timestep */
   switch (TimeMode) {
      case REAL_TIME:
         usleep(1.0E6 * DTSIM);
      case FAST_TIME: {
         // TODO: was thinking about changing it around so that the time is
         // stepped with JD_TDB_MJD = JD_TDB_MJD_0 + SimTime, but that means
         // SimTime and other time step info becomes TDB instead of TT
         // Becuase of this, do we want to get rid of JD_TDB_MJD in favor of
         // JD_TT_MJD?

         // TODO: this implementation will eventually get notable floating point
         // errors if SimTime gets sufficiently large
         itime++;
         SimTime    = ((double)itime) * DTSIM;
         JD_TT_MJD  = JDAddMultRatSecs(JD_TT_MJD_0, itime, DTSIM_RAT);
         JD_TDB_MJD = JD_TT_MJD;
         ChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, &JD_TDB_MJD);

         JDType jd_utc_j2000 = JD_TT_MJD;
         ChangeSystemEpoch(UTC_TIME, J2000_EPOCH, &jd_utc_j2000);

         TT  = JDToDate(JD_TT_MJD, TT_TIME);
         UTC = JDToDate(jd_utc_j2000, UTC_TIME);
         TDB = JDToDate(JD_TDB_MJD, TDB_TIME);

         DynTime    = JDToDynTime(JD_TT_MJD);
         CivilTime  = JDToTime(jd_utc_j2000); /* UTC "clock" time */
         AtomicTime = DynTime - 32.184;       /* TAI */
         GpsTime    = AtomicTime - 19.0;

         GpsTimeToGpsDate(GpsTime, &GpsRollover, &GpsWeek, &GpsSecond);
      } break;
      case EXTERNAL_TIME: {
         while (CurrTick == PrevTick) {
            CurrTick = (long)(1.0E-6 * usec() / DTSIM);
         }
         PrevTick = CurrTick;
         itime++;
         SimTime = ((double)itime) * DTSIM;
         RealSystemTime(&UTC, DTSIM);
         JD_TT_MJD = Date2JD(UTC, GMAT_MJD_EPOCH);
         CivilTime = JDToTime(JD_TT_MJD);
         ChangeSystemEpoch(TT_TIME, GMAT_MJD_EPOCH, &JD_TT_MJD);
         JD_TDB_MJD = JD_TT_MJD;
         ChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, &JD_TDB_MJD);

         DynTime    = JDToDynTime(JD_TT_MJD);
         AtomicTime = DynTime - 32.184; /* TAI */
         GpsTime    = AtomicTime - 19.0;

         TT          = JDToDate(JD_TT_MJD, TT_TIME);
         TDB         = JDToDate(JD_TDB_MJD, TDB_TIME);
         JD_TT_MJD_0 = JDSubSeconds(JD_TT_MJD, SimTime);

         GpsTimeToGpsDate(GpsTime, &GpsRollover, &GpsWeek, &GpsSecond);
      } break;
      case NOS3_TIME: {
         UTC       = NOS3Time();
         JD_TT_MJD = Date2JD(UTC, GMAT_MJD_EPOCH);
         CivilTime = JDToTime(JD_TT_MJD);
         ChangeSystemEpoch(TT_TIME, GMAT_MJD_EPOCH, &JD_TT_MJD);
         JD_TDB_MJD = JD_TT_MJD;
         ChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, &JD_TDB_MJD);

         DynTime    = JDToDynTime(JD_TT_MJD);
         AtomicTime = DynTime - 32.184; /* TAI */
         GpsTime    = AtomicTime - 19.0;

         TT      = JDToDate(JD_TT_MJD, TT_TIME);
         TDB     = JDToDate(JD_TDB_MJD, TDB_TIME);
         SimTime = JDToSeconds(JDSub(JD_TT_MJD, JD_TT_MJD_0));

         GpsTimeToGpsDate(GpsTime, &GpsRollover, &GpsWeek, &GpsSecond);
      } break;
   }

   /* Check for end of run */
   if (SimTime > STOPTIME)
      Done = 1;
   else
      Done = 0;

   return (Done);
}
/*********************************************************************/
/* The SC Bounding Box is referred to the origin of B0,              */
/* and expressed in B0                                               */
void UpdateScBoundingBox(struct SCType *S)
{
#define REFPT_CM 0
   struct BodyType *B, *B0;
   struct BoundingBoxType *BBox;
   struct GeomType *G;
   double ctrB[3], ctrN[3], ctrB0[3], maxB0, minB0, r[3];
   long Ib, i;

   B0   = &S->B[0];
   BBox = &S->BBox;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      G = &Geom[B->GeomTag];
      for (i = 0; i < 3; i++) {
         ctrB[i] = G->BBox.center[i];
         if (S->RefPt == REFPT_CM) {
            ctrB[i] -= B->cm[i];
         }
      }
      MTxV(B->CN, ctrB, ctrN);
      for (i = 0; i < 3; i++) {
         ctrN[i] += (B->pn[i] - B0->pn[i]);
      }
      MxV(B0->CN, ctrN, ctrB0);
      for (i = 0; i < 3; i++) {
         if (S->RefPt == REFPT_CM) {
            ctrB0[i] += B0->cm[i];
         }
         maxB0 = ctrB0[i] + G->BBox.radius;
         minB0 = ctrB0[i] - G->BBox.radius;
         if (BBox->max[i] < maxB0)
            BBox->max[i] = maxB0;
         if (BBox->min[i] > minB0)
            BBox->min[i] = minB0;
      }
   }
   for (i = 0; i < 3; i++) {
      BBox->center[i] = 0.5 * (BBox->max[i] + BBox->min[i]);
      r[i]            = BBox->max[i] - BBox->center[i];
   }
   BBox->radius = MAGV(r);
#undef REFPT_CM
}
/**********************************************************************/
void ManageBoundingBoxes(void)
{
   static long BBoxCtr = 100;
   long Isc;
   struct SCType *S;

   BBoxCtr++;
   if (BBoxCtr > 100) {
      BBoxCtr = 0;
      for (Isc = 0; Isc < Nsc; Isc++) {
         S = &SC[Isc];
         if (S->Exists) {
            UpdateScBoundingBox(S);
         }
      }
   }
}
/**********************************************************************/
/* Zero forces and torques                                            */
void ZeroFrcTrq(struct SCType *S)
{
   struct BodyType *B;
   struct JointType *G;
   struct NodeType *FN;
   long Isc, Ib, Ig, In;

   for (Isc = 0; Isc < Nsc; Isc++) {
      S->FrcN[0] = 0.0;
      S->FrcN[1] = 0.0;
      S->FrcN[2] = 0.0;

      for (Ib = 0; Ib < S->Nb; Ib++) {
         B          = &S->B[Ib];
         B->FrcN[0] = 0.0;
         B->FrcN[1] = 0.0;
         B->FrcN[2] = 0.0;
         B->FrcB[0] = 0.0;
         B->FrcB[1] = 0.0;
         B->FrcB[2] = 0.0;
         B->Trq[0]  = 0.0;
         B->Trq[1]  = 0.0;
         B->Trq[2]  = 0.0;
      }
      for (Ig = 0; Ig < S->Ng; Ig++) {
         G         = &S->G[Ig];
         G->Frc[0] = 0.0;
         G->Frc[1] = 0.0;
         G->Frc[2] = 0.0;
         G->Trq[0] = 0.0;
         G->Trq[1] = 0.0;
         G->Trq[2] = 0.0;
      }
      for (Ib = 0; Ib < S->Nb; Ib++) {
         B = &S->B[Ib];
         for (In = 0; In < B->NumNodes; In++) {
            FN         = &B->Node[In];
            FN->Frc[0] = 0.0;
            FN->Frc[1] = 0.0;
            FN->Frc[2] = 0.0;
            FN->Trq[0] = 0.0;
            FN->Trq[1] = 0.0;
            FN->Trq[2] = 0.0;
         }
      }
   }
}
/**********************************************************************/
long SimStep(void)
{
   long Isc;
   static long First = 1;
   struct SCType *S;
   long SimComplete;
   double TotalRunTime;
   static long nout = 0, GLnout = 0;
   static int set_nout = FALSE;

   if (First) {
      First   = 0;
      SimTime = 0.0;
      /* First call just initializes timer */
      RealRunTime(&TotalRunTime, DTSIM);
      ManageFlags(&nout, &GLnout, &set_nout);

      /* Sun, Moon, Planets, Spacecraft, Useful Auxiliary Frames */
      Ephemerides(SC, World, Orb);
      for (Isc = 0; Isc < Nsc; Isc++) {
         S = &SC[Isc];
         if (S->Exists) {
            ZeroFrcTrq(S);
         }
      }
      for (Isc = 0; Isc < Nsc; Isc++) {
         S = &SC[Isc];
         if (S->Exists) {
            /* Magnetic Field, Atmospheric Density */
            Environment(JD_TDB_MJD, World, Orb, S);
            Perturbations(World, Orb, S); /* Environmental Forces and Torques */
            Sensors(World, Orb, S);
            FlightSoftWare(S);
            Actuators(S);
            PartitionForces(S); /* Orbit-affecting and "internal" */
         }
      }
      for (Isc = 0; Isc < Nsc; Isc++) {
         S = &SC[Isc];
         if (S->Exists && S->FswTag == DSM_FSW) {
            struct DSMType *DSM = &S->DSM;
            DSM->CommStateProcessing(&DSM->state, &DSM->commState);
         }
      }
      Report(); /* File Output */
   }

   ReportProgress();
   ManageFlags(&nout, &GLnout, &set_nout);

   /* Read and Interpret Command Script File */
   CmdInterpreter();

   /* Update Dynamics to next Timestep */
   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists)
         Dynamics(World, Orb, &SC[Isc]);
   }
   SimComplete = AdvanceTime();
   OrbitMotion(World, Orb, DynTime);

   /* Update SC Bounding Boxes occasionally */
   ManageBoundingBoxes();

   InterProcessComm(); /* Send and receive from external processes */
   /* Sun, Moon, Planets, Spacecraft, Useful Auxiliary Frames */
   Ephemerides(SC, World, Orb);
   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &SC[Isc];
      if (S->Exists) {
         ZeroFrcTrq(S);
      }
   }
   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &SC[Isc];
      if (S->Exists) {
         /* Magnetic Field, Atmospheric Density */
         Environment(JD_TDB_MJD, World, Orb, S);
         Perturbations(World, Orb, S); /* Environmental Forces and Torques */
         Sensors(World, Orb, S);
         FlightSoftWare(S);
         Actuators(S);
         PartitionForces(S); /* Orbit-affecting and "internal" */
      }
   }
   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &SC[Isc];
      if (S->Exists && S->FswTag == DSM_FSW) {
         struct DSMType *DSM = &S->DSM;
         DSM->CommStateProcessing(&DSM->state, &DSM->commState);
      }
   }
   Report(); /* File Output */

   /* Exit when Stoptime is reached */
   if (SimComplete) {
      if (TimeMode == FAST_TIME) {
         RealRunTime(&TotalRunTime, DTSIM);
         printf("     Total Run Time = %9.2lf sec\n", TotalRunTime);
         printf("     Sim Speed = %8.2lf x Real\n", STOPTIME / TotalRunTime);
      }
   }
   return (SimComplete);
}
/**********************************************************************/
int exec(int argc, char **argv)
{
   long Done = 0;

   MapTime      = 0.0;
   JointTime    = 0.0;
   PathTime     = 0.0;
   PVelTime     = 0.0;
   FrcTrqTime   = 0.0;
   AssembleTime = 0.0;
   LockTime     = 0.0;
   TriangleTime = 0.0;
   SubstTime    = 0.0;
   SolveTime    = 0.0;

   InitSim(argc, argv);
   CmdInterpreter();
   InitInterProcessComm();
#ifdef _ENABLE_GUI_
   if (GLEnable) {
      HandoffToGui(argc, argv);
   }
   else {
      while (!Done) {
         Done = SimStep();
      }
   }
#else
   /* Crunch numbers till done */
   while (!Done) {
      Done = SimStep();
   }
#endif

   // printf("\n\nMap Time = %lf sec\n",MapTime);
   // printf("Joint Partial Time = %lf sec\n",JointTime);
   // printf("Path Time = %lf sec\n",PathTime);
   // printf("PVel Time = %lf sec\n",PVelTime);
   // printf("FrcTrq Time = %lf sec\n",FrcTrqTime);
   // printf("Assemble Time = %lf sec\n",AssembleTime);
   // printf("Lock Time = %lf sec\n",LockTime);
   // printf("Triangularize Time = %lf sec\n",TriangleTime);
   // printf("Fwd Substitution Time = %lf sec\n",SubstTime);
   // printf("Solve Time = %lf sec\n",SolveTime);
   return (0);
}

/* #ifdef __cplusplus
** }
** #endif
*/

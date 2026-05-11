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
static void _ttjd2others(const JDType tt_jd, JDType *const tdb_mjd_jd,
                         DateType *const tt, DateType *const tdb,
                         double *const tt_time, double *const tai_time,
                         double *const gps_time, long *const gps_rollover,
                         long *const gps_wk, double *const gps_sec)
{
   *tdb_mjd_jd = tt_jd;
   JDChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, &*tdb_mjd_jd);
   *tt  = JDToDate(tt_jd, TT_TIME);
   *tdb = JDToDate(*tdb_mjd_jd, TDB_TIME);

   *tt_time  = JDToDynTime(tt_jd);
   *tai_time = *tt_time - 32.184;
   *gps_time = *tai_time - 19.0;
   GpsTimeToGpsDate(*gps_time, gps_rollover, gps_wk, gps_sec);
}
/**********************************************************************/
long AdvanceTime(JDType *jd_tt_mjd, JDType *jd_tdb_mjd, DateType *tt,
                 DateType *tdb, DateType *utc, double *simtime, double *dyntime,
                 double *atomictime, double *gpstime, double *civiltime,
                 long *gpsrollover, long *gpsweek, double *gpssecond)
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
         // Because of this, do we want to get rid of JD_TDB_MJD in favor of
         // JD_TT_MJD?

         // TODO: this implementation will eventually get notable floating point
         // errors if SimTime gets sufficiently large
         itime++;
         *simtime = ((double)itime) * DTSIM;

         *jd_tt_mjd = JDAddMultRatSecs(JD_TT_MJD_0, itime, DTSIM_RAT);
         *utc       = JDToDate(*jd_tt_mjd, UTC_TIME);
      } break;
      case EXTERNAL_TIME: {
         while (CurrTick == PrevTick) {
            CurrTick = (long)(1.0E-6 * usec() / DTSIM);
         }
         PrevTick = CurrTick;
         itime++;
         *simtime = ((double)itime) * DTSIM;
         *utc     = RealSystemTime();

         *jd_tt_mjd = Date2JD(*utc, GMAT_MJD_EPOCH);
         JDChangeSystem(TT_TIME, jd_tt_mjd);
         JD_TT_MJD_0 = JDSubSeconds(*jd_tt_mjd, *simtime);
      } break;
      case NOS3_TIME: {
         const Rational tick_time = NOS3Time(DTSIM_RAT);
         *simtime                 = rational2double(tick_time);

         *jd_tt_mjd = JDAddRationalSeconds(JD_TT_MJD_0, tick_time);
         *utc       = JDToDate(*jd_tt_mjd, UTC_TIME);
      } break;
   }
   *civiltime = Date2Time(*utc); /* UTC "clock" time */
   _ttjd2others(*jd_tt_mjd, jd_tdb_mjd, tt, tdb, dyntime, atomictime, gpstime,
                gpsrollover, gpsweek, gpssecond);

   /* Check for end of run */
   if (*simtime > STOPTIME)
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
void MapSToRKState(const struct OrbitType *const orb, struct SCType *S,
                   double *x_rk)
{
   double *x_trn  = NULL;
   const long dim = S->rkparams.base.dim;

   switch (S->DynMethod) {
      case DYN_GAUSS_ELIM: { // TODO
      } break;
      case DYN_ORDER_N: { // TODO
      } break;
      default:
         fprintf(stderr, "Unknown Dynamics Solution option.  Bailing out.\n");
         exit(EXIT_FAILURE);
   }

   switch (orb->Regime) {
      case ORB_ZERO:
      case ORB_FLIGHT:
         x_trn = &x_rk[dim - 6];
         CopyVG(x_trn, S->PosN, 3);
         CopyVG(&x_trn[3], S->VelN, 3);
      case ORB_CENTRAL:
         switch (S->OrbDOF) {
            case ORBDOF_FIXED:
               break;
            case ORBDOF_EULER_HILL:
               x_trn = &x_rk[dim - 6];
               CopyVG(x_trn, S->PosEH, 3);
               CopyVG(&x_trn[3], S->VelEH, 3);
               break;
            case ORBDOF_COWELL:
               x_trn = &x_rk[dim - 6];
               CopyVG(x_trn, S->PosN, 3);
               CopyVG(&x_trn[3], S->VelN, 3);
               break;
            default:
               x_trn = &x_rk[dim - 6];
               CopyVG(x_trn, S->PosR, 3);
               CopyVG(&x_trn[3], S->VelR, 3);
               break;
         }
         break;
      case ORB_N_BODY:
         switch (S->OrbDOF) {
            case ORBDOF_COWELL:
               x_trn = &x_rk[dim - 6];
               CopyVG(x_trn, S->PosN, 3);
               CopyVG(&x_trn[3], S->VelN, 3);
               break;
            default:
               printf("ERROR: MUST USE COWELLS METHOD!!! \n");
               exit(EXIT_FAILURE);
         }
         break;
      case ORB_THREE_BODY:
         switch (S->OrbDOF) {
            case ORBDOF_FIXED:
               break;
            case ORBDOF_EULER_HILL:
               x_trn = &x_rk[dim - 6];
               CopyVG(x_trn, S->PosEH, 3);
               CopyVG(&x_trn[3], S->VelEH, 3);
               break;
            case ORBDOF_COWELL:
               x_trn = &x_rk[dim - 6];
               CopyVG(x_trn, S->PosN, 3);
               CopyVG(&x_trn[3], S->VelN, 3);
               break;
            default:
               break;
         }
         break;
      default:
         fprintf(stderr, "Unknown Orbit Regime in Dynamics.  Bailing out.\n");
         exit(EXIT_FAILURE);
   }
}
/**********************************************************************/
void MapRKStateToS(const struct OrbitType *const orb, struct SCType *S,
                   double *x_rk)
{
   double *x_trn  = NULL;
   const long dim = S->rkparams.base.dim;

   switch (S->DynMethod) {
      case DYN_GAUSS_ELIM: {
         struct DynType *D = &S->Dyn;
         double *u, *x, *h, *a, *uf, *xf;
         long offset  = 0;
         u            = &x_rk[offset];
         offset      += D->Nu;
         x            = &x_rk[offset];
         offset      += D->Nx;
         h            = &x_rk[offset];
         offset      += S->Nw;
         a            = &x_rk[offset];
         offset      += S->Nw;
         uf           = &x_rk[offset];
         offset      += D->Nf;
         xf           = &x_rk[offset];
         CopyVG(D->u, u, D->Nu);
         CopyVG(D->x, x, D->Nx);
         CopyVG(D->h, h, S->Nw);
         CopyVG(D->a, a, S->Nw);
         CopyVG(D->uf, uf, D->Nf);
         CopyVG(D->xf, xf, D->Nf);
         MapStateVectorToBodyStates(u, x, h, a, uf, xf, S);
         MotionConstraints(S);
         BodyStatesToNodeStates(S);
         SCMassProps(S);
         FindTotalAngMom(S);
      } break;
      case DYN_ORDER_N: {
         struct DynType *D = &S->Dyn;
         double *u, *x, *h, *a, *uf, *xf;
         long offset  = 0;
         u            = &x_rk[offset];
         offset      += D->Nu;
         x            = &x_rk[offset];
         offset      += D->Nx;
         h            = &x_rk[offset];
         offset      += S->Nw;
         a            = &x_rk[offset];
         offset      += S->Nw;
         uf           = &x_rk[offset];
         offset      += D->Nf;
         xf           = &x_rk[offset];
         CopyVG(D->u, u, D->Nu);
         CopyVG(D->x, x, D->Nx);
         CopyVG(D->h, h, S->Nw);
         CopyVG(D->a, a, S->Nw);
         CopyVG(D->uf, uf, D->Nf);
         CopyVG(D->xf, xf, D->Nf);
         MapStateVectorToBodyStates(u, x, h, a, uf, xf, S);
         MotionConstraints(S);
         BodyStatesToNodeStates(S);
         SCMassProps(S);
         FindTotalAngMom(S);
      } break;
      default:
         fprintf(stderr, "Unknown Dynamics Solution option.  Bailing out.\n");
         exit(EXIT_FAILURE);
   }

   switch (orb->Regime) {
      case ORB_ZERO:
      case ORB_FLIGHT:
         x_trn = &x_rk[dim - 6];
         CopyVG(S->PosN, x_trn, 3);
         CopyVG(S->VelN, &x_trn[3], 3);
      case ORB_CENTRAL:
         switch (S->OrbDOF) {
            case ORBDOF_FIXED:
               break;
            case ORBDOF_EULER_HILL:
               x_trn = &x_rk[dim - 6];
               CopyVG(S->PosEH, x_trn, 3);
               CopyVG(S->VelEH, &x_trn[3], 3);
               EHRV2RelRV(orb->SMA, orb->MeanMotion, orb->CLN, S->PosEH,
                          S->VelEH, S->PosR, S->VelR);
               break;
            case ORBDOF_COWELL:
               x_trn = &x_rk[dim - 6];
               CopyVG(S->PosN, x_trn, 3);
               CopyVG(S->VelN, &x_trn[3], 3);
               break;
            default:
               x_trn = &x_rk[dim - 6];
               CopyVG(S->PosR, x_trn, 3);
               CopyVG(S->VelR, &x_trn[3], 3);
               break;
         }
         break;
      case ORB_N_BODY:
         switch (S->OrbDOF) {
            case ORBDOF_COWELL:
               x_trn = &x_rk[dim - 6];
               CopyVG(S->PosN, x_trn, 3);
               CopyVG(S->VelN, &x_trn[3], 3);
               break;
            default:
               printf("ERROR: MUST USE COWELLS METHOD!!! \n");
               exit(EXIT_FAILURE);
         }
         break;
      case ORB_THREE_BODY:
         switch (S->OrbDOF) {
            case ORBDOF_FIXED:
               break;
            case ORBDOF_EULER_HILL:
               x_trn = &x_rk[dim - 6];
               CopyVG(S->PosEH, x_trn, 3);
               CopyVG(S->VelEH, &x_trn[3], 3);
               EHRV2RelRV(orb->SMA, orb->MeanMotion, orb->CLN, S->PosEH,
                          S->VelEH, S->PosR, S->VelR);
               break;
            case ORBDOF_COWELL:
               x_trn = &x_rk[dim - 6];
               CopyVG(S->PosN, x_trn, 3);
               CopyVG(S->VelN, &x_trn[3], 3);
               break;
            default:
               break;
         }
         break;
      default:
         fprintf(stderr, "Unknown Orbit Regime in Dynamics.  Bailing out.\n");
         exit(EXIT_FAILURE);
   }
}
/**********************************************************************/
long SimStep_New(void)
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
      Ephemerides(JD_TDB_MJD, SC, World, Rgn, LagSys, Orb);
      for (Isc = 0; Isc < Nsc; Isc++) {
         S = &SC[Isc];
         if (S->Exists) {
            ZeroFrcTrq(S);
         }
      }
      for (Isc = 0; Isc < Nsc; Isc++) {
         S = &SC[Isc];
         if (S->Exists) {
            struct OrbitType *O = &Orb[S->RefOrb];
            /* Magnetic Field, Atmospheric Density */
            Environment(JD_TDB_MJD, World, O, S);
            Perturbations(World, O, S); /* Environmental Forces and Torques */
            SCContactFrcTrq(Orb, SC, Isc);
            Sensors(World, O, S);
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

   JDType jd_f = JDAddRationalSeconds(JD_TT_MJD, DTSIM_RAT);

   SCContactFrcTrq(Orb, SC, Isc);
   /* Update Dynamics to next Timestep */
   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         // Clean up World_dupe for the next integration
         for (int i = 0; i < NWORLD; i++)
            CopyWorld(&World_dupe[i], World[i]);
         CopyOrbit(&S->rkparams.orb, Orb[S->RefOrb]);

         MapSToRKState(&S->rkparams.orb, S, S->rk_state);
         RungeKuttaStep(&S->RKIntegrator, JD_TT_MJD, DTSIM, S->rk_state);

         // TODO: assuming that the last call in RungeKutta got us to the
         // current time. Otherwise:
         // WorldEphemerides(jd_f, World_dupe, S->rkparams.rgn,
         //                  S->rkparams.lagsys);
         // OrbitMotion(World_dupe, S->rkparams.rgn, S->rkparams.lagsys,
         //             &S->rkparams.orb, &S->rkparams.frm, JDToDynTime(jd_f));
         MapRKStateToS(&S->rkparams.orb, S, S->rk_state);
      }
   }
   SimComplete = AdvanceTime(&JD_TT_MJD, &JD_TDB_MJD, &TT, &TDB, &UTC, &SimTime,
                             &DynTime, &AtomicTime, &GpsTime, &CivilTime,
                             &GpsRollover, &GpsWeek, &GpsSecond);

   WorldEphemerides(JD_TT_MJD, World, Rgn, LagSys);
   for (long Iorb = 0; Iorb < Norb; Iorb++)
      OrbitMotion(World, Rgn, LagSys, &Orb[Iorb], &Frm[Iorb], DynTime);

   /* Update SC Bounding Boxes occasionally */
   ManageBoundingBoxes();

   InterProcessComm(); /* Send and receive from external processes */
   /* Sun, Moon, Planets, Spacecraft, Useful Auxiliary Frames */
   SCEphemerides(JD_TDB_MJD, SC, World, Orb);
   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &SC[Isc];
      if (S->Exists) {
         struct OrbitType *O = &Orb[S->RefOrb];
         Sensors(World, O, S);
         FlightSoftWare(S);
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
long SimStep_Old(void)
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
      Ephemerides(JD_TDB_MJD, SC, World, Rgn, LagSys, Orb);
      for (Isc = 0; Isc < Nsc; Isc++) {
         S = &SC[Isc];
         if (S->Exists) {
            ZeroFrcTrq(S);
         }
      }
      for (Isc = 0; Isc < Nsc; Isc++) {
         S = &SC[Isc];
         if (S->Exists) {
            struct OrbitType *O = &Orb[S->RefOrb];
            /* Magnetic Field, Atmospheric Density */
            Environment(JD_TDB_MJD, World, O, S);
            Perturbations(World, O, S); /* Environmental Forces and Torques */
            SCContactFrcTrq(Orb, SC, Isc);
            Sensors(World, O, S);
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
         Dynamics(World, &Orb[SC[Isc].RefOrb], &Frm[SC[Isc].RefOrb], &SC[Isc]);
   }
   SimComplete = AdvanceTime(&JD_TT_MJD, &JD_TDB_MJD, &TT, &TDB, &UTC, &SimTime,
                             &DynTime, &AtomicTime, &GpsTime, &CivilTime,
                             &GpsRollover, &GpsWeek, &GpsSecond);
   for (long Iorb = 0; Iorb < Norb; Iorb++)
      OrbitMotion(World, Rgn, LagSys, &Orb[Iorb], &Frm[Iorb], DynTime);

   /* Update SC Bounding Boxes occasionally */
   ManageBoundingBoxes();

   InterProcessComm(); /* Send and receive from external processes */
   /* Sun, Moon, Planets, Spacecraft, Useful Auxiliary Frames */
   Ephemerides(JD_TDB_MJD, SC, World, Rgn, LagSys, Orb);
   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &SC[Isc];
      if (S->Exists) {
         ZeroFrcTrq(S);
      }
   }
   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &SC[Isc];
      if (S->Exists) {
         struct OrbitType *O = &Orb[S->RefOrb];
         /* Magnetic Field, Atmospheric Density */
         Environment(JD_TDB_MJD, World, O, S);
         Perturbations(World, O, S); /* Environmental Forces and Torques */
         SCContactFrcTrq(Orb, SC, Isc);
         Sensors(World, O, S);
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
#ifdef OLD_INTEGRATOR
         Done = SimStep_Old();
#else
         Done = SimStep_New();
#endif
      }
   }
#else
   /* Crunch numbers till done */
   while (!Done) {
#ifdef OLD_INTEGRATOR
      Done = SimStep_Old();
#else
      Done = SimStep_New();
#endif
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

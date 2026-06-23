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
      *nout = RationalRoundUp(ToRational(RationalDivide(DTOUT_RAT, DTSIM_RAT)));
      *GLnout =
          RationalRoundUp(ToRational(RationalDivide(DTOUTGL_RAT, DTSIM_RAT)));
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
   *tdb_mjd_jd = JDChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, *tdb_mjd_jd);
   *tt         = JDToDate(tt_jd, TT_TIME);
   *tdb        = JDToDate(*tdb_mjd_jd, TDB_TIME);

   *tt_time  = JDToDynTime(tt_jd);
   *tai_time = *tt_time - 32.184;
   *gps_time = *tai_time - 19.0;
   GpsTimeToGpsDate(*gps_time, gps_rollover, gps_wk, gps_sec);
}
/**********************************************************************/
long AdvanceTime(const Rational dtsim_rat, JDType *jd_tt_mjd,
                 JDType *jd_tdb_mjd, DateType *tt, DateType *tdb, DateType *utc,
                 double *simtime, double *dyntime, double *atomictime,
                 double *gpstime, double *civiltime, long *gpsrollover,
                 long *gpsweek, double *gpssecond)
{
   static long itime    = 0;
   static long PrevTick = 0;
   static long CurrTick = 1;
   const double dtsim   = rational2double(dtsim_rat);

   /* Advance time to next Timestep */
   switch (TimeMode) {
      case REAL_TIME:
         usleep(1.0E6 * dtsim);
         [[fallthrough]];
      case FAST_TIME: {
         // TODO: was thinking about changing it around so that the time is
         // stepped with JD_TDB_MJD = JD_TDB_MJD_0 + SimTime, but that means
         // SimTime and other time step info becomes TDB instead of TT
         // Because of this, do we want to get rid of JD_TDB_MJD in favor of
         // JD_TT_MJD?

         // TODO: this implementation will eventually get notable floating point
         // errors if SimTime gets sufficiently large
         itime++;
         *simtime = ((double)itime) * dtsim;

         *jd_tt_mjd = JDAddIntegerMultRatSecs(JD_TT_MJD_0, itime, dtsim_rat);
         *utc       = JDToDate(*jd_tt_mjd, UTC_TIME);
      } break;
      case EXTERNAL_TIME: {
         while (CurrTick == PrevTick) {
            CurrTick = (long)(1.0E-6 * usec() / dtsim);
         }
         PrevTick = CurrTick;
         itime++;
         *simtime = ((double)itime) * dtsim;
         *utc     = RealSystemTime();

         *jd_tt_mjd  = Date2JD(*utc, GMAT_MJD_EPOCH);
         *jd_tt_mjd  = JDChangeSystem(TT_TIME, *jd_tt_mjd);
         JD_TT_MJD_0 = JDSubSeconds(*jd_tt_mjd, *simtime);
      } break;
      case NOS3_TIME: {
         const Rational tick_time = NOS3Time(dtsim_rat);
         *simtime                 = rational2double(tick_time);

         *jd_tt_mjd = JDAddSeconds(JD_TT_MJD_0, tick_time);
         *utc       = JDToDate(*jd_tt_mjd, UTC_TIME);
      } break;
   }
   *civiltime = Date2Time(*utc); /* UTC "clock" time */
   _ttjd2others(*jd_tt_mjd, jd_tdb_mjd, tt, tdb, dyntime, atomictime, gpstime,
                gpsrollover, gpsweek, gpssecond);

   /* return if at end of run */
   return *simtime > STOPTIME;
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
   vec3_t ctrB, ctrN, ctrB0, r;
   double maxB0, minB0;
   long Ib, i;

   B0   = &S->B[0];
   BBox = &S->BBox;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B    = &S->B[Ib];
      G    = &Geom[B->GeomTag];
      ctrB = G->BBox.center;
      if (S->RefPt == REFPT_CM)
         ctrB = VSubV_Elem(ctrB, B->cm);

      ctrN = MTxV(B->CN, ctrB);
      for (i = 0; i < 3; i++)
         ctrN.v[i] += (B->pn.v[i] - B0->pn.v[i]);

      ctrB0 = MxV(B0->CN, ctrN);
      if (S->RefPt == REFPT_CM)
         ctrB0 = VAddV_Elem(ctrB0, B0->cm);

      for (i = 0; i < 3; i++) {
         maxB0 = ctrB0.v[i] + G->BBox.radius;
         minB0 = ctrB0.v[i] - G->BBox.radius;
         if (BBox->max.v[i] < maxB0)
            BBox->max.v[i] = maxB0;
         if (BBox->min.v[i] > minB0)
            BBox->min.v[i] = minB0;
      }
   }
   for (i = 0; i < 3; i++) {
      BBox->center.v[i] = 0.5 * (BBox->max.v[i] + BBox->min.v[i]);
      r.v[i]            = BBox->max.v[i] - BBox->center.v[i];
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
         if (S->Exists)
            UpdateScBoundingBox(S);
      }
   }
}
/**********************************************************************/
/* Zero forces and torques                                            */
void ZeroNonSCContactFrcTrq(struct SCType *S)
{
   struct BodyType *B;
   struct JointType *G;
   struct NodeType *FN;
   long Ib, Ig, In;

   S->FrcN         = VEC3_ZERO;
   S->gravPertAccN = VEC3_ZERO;
   S->gravPriAccN  = VEC3_ZERO;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B               = &S->B[Ib];
      B->gravPriAccN  = VEC3_ZERO;
      B->gravPertAccN = VEC3_ZERO;
      B->FrcN         = VEC3_ZERO;
      B->FrcB         = VEC3_ZERO;
      B->Trq          = VEC3_ZERO;
   }
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G      = &S->G[Ig];
      G->Frc = VEC3_ZERO;
      G->Trq = VEC3_ZERO;
   }
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      for (In = 0; In < B->NumNodes; In++) {
         FN      = &B->Node[In];
         FN->Frc = VEC3_ZERO;
         FN->Trq = VEC3_ZERO;
      }
   }
}
/**********************************************************************/
void ZeroFrcTrq(struct SCType *S)
{
   struct BodyType *B;
   long Ib;
   ZeroNonSCContactFrcTrq(S);
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];

      B->SCContactFrcN = VEC3_ZERO;
      B->SCContactFrcB = VEC3_ZERO;
      B->SCContactTrq  = VEC3_ZERO;
   }
}
/**********************************************************************/
void SToRKState(const struct OrbitType *const orb, struct SCType *S,
                double *x_rk)
{
   double *x_trn  = NULL;
   const long dim = S->rkparams.base.dim;

   struct DynType *D = &S->Dyn;
   switch (S->DynMethod) {
      case DYN_GAUSS_ELIM: {
         /* .. Check for Locked Joint DOFs */
         for (int i = 0; i < 3; i++)
            D->ActiveStateIdx[i] = i; /* Body 0 angular DOF never locked */
         D->Ns  = 3;
         int iu = 3;
         for (int Ig = 0; Ig < S->Ng; Ig++) {
            struct JointType *G = &S->G[Ig];
            G->ActiveRotu0      = D->Ns;
            G->ActiveRotDOF     = 0;
            for (int i = 0; i < G->RotDOF; i++) {
               if (!G->RotLocked[i]) {
                  G->ActiveRotDOF++;
                  D->ActiveStateIdx[D->Ns] = iu;
                  D->Ns++;
               }
               else {
                  D->u[iu] = 0.0;
               }
               iu++;
            }
            G->ActiveTrnu0  = D->Ns;
            G->ActiveTrnDOF = 0;
            for (int i = 0; i < G->TrnDOF; i++) {
               if (!G->TrnLocked[i]) {
                  G->ActiveTrnDOF++;
                  D->ActiveStateIdx[D->Ns] = iu;
                  D->Ns++;
               }
               else {
                  D->u[iu] = 0.0;
               }
               iu++;
            }
         }
         for (int i = 0; i < 3;
              i++) { /* Body 0 translational DOF never locked */
            D->ActiveStateIdx[D->Ns] = iu;
            D->Ns++;
            iu++;
         }
         D->SomeJointsLocked  = ((D->Ns == D->Nu) ? 0 : 1);
         D->Ns               += D->Nf;
         long offset          = 0;
         CopyVG(&x_rk[offset], D->u, D->Nu);
         offset += D->Nu;
         CopyVG(&x_rk[offset], D->x, D->Nx);
         offset += D->Nx;
         CopyVG(&x_rk[offset], D->h, S->Nw);
         offset += S->Nw;
         CopyVG(&x_rk[offset], D->a, S->Nw);
         offset += S->Nw;
         CopyVG(&x_rk[offset], D->uf, D->Nf);
         offset += D->Nf;
         CopyVG(&x_rk[offset], D->xf, D->Nf);
      } break;
      case DYN_ORDER_N: {
         long offset = 0;
         CopyVG(&x_rk[offset], D->u, D->Nu);
         offset += D->Nu;
         CopyVG(&x_rk[offset], D->x, D->Nx);
         offset += D->Nx;
         CopyVG(&x_rk[offset], D->h, S->Nw);
         for (int Iw = 0; Iw < S->Nw; Iw++) {
            struct WhlType *W = &S->Whl[Iw];

            x_rk[D->Nu + D->Nx + Iw] = W->H;
         }
      } break;
      default:
         fprintf(stderr, "Unknown Dynamics Solution option.  Bailing out.\n");
         exit(EXIT_FAILURE);
   }

   switch (orb->Regime) {
      case ORB_ZERO:
      case ORB_FLIGHT:
         x_trn = &x_rk[dim - 6];
         CopyVG(x_trn, S->PosN.v, 3);
         CopyVG(&x_trn[3], S->VelN.v, 3);
         [[fallthrough]];
      case ORB_CENTRAL:
         switch (S->OrbDOF) {
            case ORBDOF_FIXED:
               break;
            case ORBDOF_EULER_HILL:
               x_trn = &x_rk[dim - 6];
               CopyVG(x_trn, S->PosEH.v, 3);
               CopyVG(&x_trn[3], S->VelEH.v, 3);
               break;
            case ORBDOF_COWELL:
               x_trn = &x_rk[dim - 6];
               CopyVG(x_trn, S->PosN.v, 3);
               CopyVG(&x_trn[3], S->VelN.v, 3);
               break;
            default:
               x_trn = &x_rk[dim - 6];
               CopyVG(x_trn, S->PosR.v, 3);
               CopyVG(&x_trn[3], S->VelR.v, 3);
               break;
         }
         break;
      case ORB_N_BODY:
         switch (S->OrbDOF) {
            case ORBDOF_COWELL:
               x_trn = &x_rk[dim - 6];
               CopyVG(x_trn, S->PosN.v, 3);
               CopyVG(&x_trn[3], S->VelN.v, 3);
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
               CopyVG(x_trn, S->PosEH.v, 3);
               CopyVG(&x_trn[3], S->VelEH.v, 3);
               break;
            case ORBDOF_COWELL:
               x_trn = &x_rk[dim - 6];
               CopyVG(x_trn, S->PosN.v, 3);
               CopyVG(&x_trn[3], S->VelN.v, 3);
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
void RKStateToS(struct OrbitType *const orb, double *x_rk, struct SCType *S)
{
   double *x_trn  = NULL;
   const long dim = S->rkparams.base.dim;

   struct DynType *D = &S->Dyn;
   long offset       = 0;
   switch (S->DynMethod) {
      case DYN_GAUSS_ELIM:
         offset  = 0;
         offset += D->Nu;
         offset += D->Nx;
         offset += S->Nw;
         CopyVG(D->a, &x_rk[offset], S->Nw);
         offset += S->Nw;
         CopyVG(D->uf, &x_rk[offset], D->Nf);
         offset += D->Nf;
         CopyVG(D->xf, &x_rk[offset], D->Nf);
         [[fallthrough]];
      case DYN_ORDER_N:
         offset = 0;
         CopyVG(D->u, &x_rk[offset], D->Nu);
         offset += D->Nu;
         CopyVG(D->x, &x_rk[offset], D->Nx);
         quat_t q = DBL_TO_QUAT(D->x);
         q        = UNITQ(q);
         QUAT_TO_DBL(D->x, q);
         offset += D->Nx;
         CopyVG(D->h, &x_rk[offset], S->Nw);
         break;
      default:
         fprintf(stderr, "Unknown Dynamics Solution option.  Bailing out.\n");
         exit(EXIT_FAILURE);
   }
   MapStateVectorToBodyStates(D->u, D->x, D->h, D->a, D->uf, D->xf, S);
   MotionConstraints(S);
   BodyStatesToNodeStates(S);
   SCMassProps(S);
   FindTotalAngMom(S);

   switch (orb->Regime) {
      case ORB_ZERO:
      case ORB_FLIGHT:
         x_trn = &x_rk[dim - 6];
         CopyVG(S->PosN.v, x_trn, 3);
         CopyVG(S->VelN.v, &x_trn[3], 3);
         [[fallthrough]];
      case ORB_CENTRAL:
         switch (S->OrbDOF) {
            case ORBDOF_FIXED:
               break;
            case ORBDOF_EULER_HILL: {
               x_trn = &x_rk[dim - 6];
               CopyVG(S->PosEH.v, x_trn, 3);
               CopyVG(S->VelEH.v, &x_trn[3], 3);
               pair_vec3_t pair = EHRV2RelRV(orb->SMA, orb->MeanMotion,
                                             orb->CLN, S->PosEH, S->VelEH);
               S->PosR          = pair.first;
               S->VelR          = pair.second;
            } break;
            case ORBDOF_COWELL:
               x_trn = &x_rk[dim - 6];
               CopyVG(S->PosN.v, x_trn, 3);
               CopyVG(S->VelN.v, &x_trn[3], 3);
               break;
            default:
               x_trn = &x_rk[dim - 6];
               CopyVG(S->PosR.v, x_trn, 3);
               CopyVG(S->VelR.v, &x_trn[3], 3);
               break;
         }
         break;
      case ORB_N_BODY:
         switch (S->OrbDOF) {
            case ORBDOF_COWELL:
               x_trn = &x_rk[dim - 6];
               CopyVG(S->PosN.v, x_trn, 3);
               CopyVG(S->VelN.v, &x_trn[3], 3);
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
            case ORBDOF_EULER_HILL: {
               x_trn = &x_rk[dim - 6];
               CopyVG(S->PosEH.v, x_trn, 3);
               CopyVG(S->VelEH.v, &x_trn[3], 3);
               pair_vec3_t pair = EHRV2RelRV(orb->SMA, orb->MeanMotion,
                                             orb->CLN, S->PosEH, S->VelEH);
               S->PosR          = pair.first;
               S->VelR          = pair.second;
            } break;
            case ORBDOF_COWELL:
               x_trn = &x_rk[dim - 6];
               CopyVG(S->PosN.v, x_trn, 3);
               CopyVG(S->VelN.v, &x_trn[3], 3);
               break;
            default:
               x_trn = &x_rk[dim - 6];
               CopyVG(S->PosR.v, x_trn, 3);
               CopyVG(S->VelR.v, &x_trn[3], 3);
               break;
         }
         break;
      default:
         fprintf(stderr, "Unknown Orbit Regime in Dynamics.  Bailing out.\n");
         exit(EXIT_FAILURE);
   }
}
/**********************************************************************/
static long _check_do_world_orientation(
    const struct WorldType *const w, const long Iw,
    const struct SCType *const scs, const long n_scs,
    const struct RegionType *const regions, const long n_rgn,
    const struct GroundStationType *ground_stations, const long n_gndstn,
    const struct OrbitType *const orbs, const ephemType ephem_option,
    const long gui_active, const long grav_pert_active, const long atmo_active)
    __attribute__((pure));
static long _check_do_world_orientation(
    const struct WorldType *const w, const long Iw,
    const struct SCType *const scs, const long n_scs,
    const struct RegionType *const regions __attribute__((unused)),
    const long n_rgn __attribute__((unused)),
    const struct GroundStationType *ground_stations, const long n_gndstn,
    const struct OrbitType *const orbs, const ephemType ephem_option,
    const long gui_active, const long grav_pert_active __attribute__((unused)),
    const long atmo_active __attribute__((unused)))
{
   // This is all to avoid more calls to pxform_c if we don't *need* them
   // we don't need to set the world orientation unless any of:
   //    a) It is either EARTH or SOL
   //    b) GUI is enabled and any of:
   //       1) is a world (orrey can need any of them sometimes, it seems)
   //       2) world is pov host or target
   //       3) world has an orbit
   //       4) world has an active satellite body
   //       5) world is close enough to show as a disk
   //    c) it is orbited by a spacecraft and any of:
   //       1) grav perts are active and it has harmonic grav
   //       2) certain dsm/fsw routines use world orientation
   //       3) atmo drag is active and the spacecraft is in the atmo
   //       4) albedo is active
   //    d) world is the secondary body for a spacecraft's 3body orbit
   //    e) world has a Region
   //    f) world has a GroundStation

   // do the simple ones first
   if (!w->Exists)
      return FALSE;
   if ((ephem_option != EPH_SPICE) || Iw == SOL || Iw == EARTH || gui_active)
      return TRUE;

   // check SC related conditions
   for (long Isc = 0; Isc < n_scs; Isc++) {
      const struct SCType *const sc = &scs[Isc];
      if (!sc->Exists)
         continue;
      const struct OrbitType *const orb = &orbs[sc->RefOrb];
      switch (orb->Regime) {
         case ORB_THREE_BODY: {
            if (Iw == orb->Body2)
               return TRUE;
         } break;
         case ORB_N_BODY:
         case ORB_CENTRAL:
         case ORB_ZERO: {
            if (Iw == orb->World)
               return TRUE;
         } break;
         default:
            // if ORB_FLIGHT, then will be checking regions anyway
            break;
      }
   }

   // check Region related conditions
   for (long Irgn = 0; Irgn < Nrgn; Irgn++) {
      const struct RegionType *rgn = &Rgn[Irgn];
      if (Iw == rgn->World)
         return TRUE;
   }

   // check Ground Station related conditions
   for (long Igndstn = 0; Igndstn < n_gndstn; Igndstn++) {
      const struct GroundStationType *gndstn = &ground_stations[Igndstn];
      if (!gndstn->Exists)
         continue;
      if (gndstn->World == Iw)
         return TRUE;
   }
   return FALSE;
}
/**********************************************************************/
void CheckDoWorldOrientation(
    struct WorldType *const world, const struct SCType *const scs,
    const long n_scs, const struct RegionType *const regions, const long n_rgn,
    const struct GroundStationType *ground_stations, const long n_gndstn,
    const struct OrbitType *const orbs, const ephemType ephem_option,
    const long gui_active, const long grav_pert_active, const long atmo_active)
{
   for (WorldID Iw = SOL; Iw < NWORLD; Iw++) {
      struct WorldType *const w = &world[Iw];

      w->OrientWorld = _check_do_world_orientation(
          w, Iw, scs, n_scs, regions, n_rgn, ground_stations, n_gndstn, orbs,
          ephem_option, gui_active, grav_pert_active, atmo_active);
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
      RealRunTime(&TotalRunTime);
      ManageFlags(&nout, &GLnout, &set_nout);

      // is this necessary???
      // for (long Iorb = 0; Iorb < Norb; Iorb++)
      //    OrbitMotion(JD_TDB_MJD, World, &Orb[Iorb], Rgn, LagSys, &Frm[Iorb]);

      /* Sun, Moon, Planets, Useful Auxiliary Frames */
      WorldEphemerides(JD_TDB_MJD, JD_TT_MJD, EphemOption, World, Rgn, LagSys);

      for (Isc = 0; Isc < Nsc; Isc++) {
         S = &SC[Isc];
         if (S->Exists) {
            struct OrbitType *O = &Orb[S->RefOrb];
            /* Spacecraft */
            SCEphemerides(JD_TDB_MJD, S, &World[O->World], O);
            ZeroFrcTrq(S);
         }
      }
      for (Isc = 0; Isc < Nsc; Isc++) {
         S = &SC[Isc];
         if (S->Exists) {

            struct OrbitType *O = &Orb[S->RefOrb];
            /* Magnetic Field, Atmospheric Density */
            Environment(JD_TDB_MJD, World, O, S);
            if (ContactActive)
               SCContactFrcTrq(Orb, SC, Isc);

            /* Environmental Forces and Torques */
            Perturbations(JD_TDB_MJD, World, O, S);

            /* Orbit-affecting and "internal" */
            Actuators(FALSE, S, JD_TT_MJD);
            PartitionForces(S);

            SToRKState(S->rkparams.orb, S, S->rk_state);
            if (S->OrbDOF != ORBDOF_FIXED) {
               const long dim    = S->rkparams.base.dim;
               const vec3_t rvec = DBL_TO_VEC3(&S->rk_state[dim - 6]);
               const vec3_t vvec = DBL_TO_VEC3(&S->rk_state[dim - 3]);
               S->gravPriAccN    = GetPrimaryGravAccel(S->OrbDOF, rvec, vvec,
                                                       World, &Orb[S->RefOrb]);
            }

            Sensors(World, O, S);
            FlightSoftWare(S);
         }
      }
      for (Isc = 0; Isc < Nsc; Isc++) {
         S = &SC[Isc];
         if (S->Exists && S->FswTag == DSM_FSW) {
            struct DSMType *DSM = &S->DSM;
            DSM->commState      = DSM->CommStateProcessing(DSM->state);
         }
      }
      Report(); /* File Output */
   }

   ReportProgress();
   ManageFlags(&nout, &GLnout, &set_nout);

   /* Read and Interpret Command Script File */
   CmdInterpreter();

   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &SC[Isc];
      if (S->Exists)
         ZeroFrcTrq(S);
   }
   if (ContactActive) {
      for (Isc = 0; Isc < Nsc; Isc++) {
         S = &SC[Isc];
         if (S->Exists)
            SCContactFrcTrq(Orb, SC, Isc);
      }
   }

   CheckDoWorldOrientation(World, SC, Nsc, Rgn, Nrgn, GroundStation, Ngnd, Orb,
                           EphemOption, GLEnable, GravPertActive, AeroActive);

   /* Update Dynamics to next Timestep */
   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &SC[Isc];
      if (S->Exists) {
         // TODO: do I need to do this??
         SToRKState(S->rkparams.orb, S, S->rk_state);
         RungeKuttaStep(&S->RKIntegrator, TRUE, JD_TT_MJD, DTSIM, S->rk_state);
         RKStateToS(S->rkparams.orb, S->rk_state, S);

         if (S->OrbDOF != ORBDOF_FIXED) {
            const long dim    = S->rkparams.base.dim;
            const vec3_t rvec = DBL_TO_VEC3(&S->rk_state[dim - 6]);
            const vec3_t vvec = DBL_TO_VEC3(&S->rk_state[dim - 3]);
            S->gravPriAccN = GetPrimaryGravAccel(S->OrbDOF, rvec, vvec, World,
                                                 &Orb[S->RefOrb]);
         }
      }
   }
   SimComplete = AdvanceTime(DTSIM_RAT, &JD_TT_MJD, &JD_TDB_MJD, &TT, &TDB,
                             &UTC, &SimTime, &DynTime, &AtomicTime, &GpsTime,
                             &CivilTime, &GpsRollover, &GpsWeek, &GpsSecond);

   /* Update SC Bounding Boxes occasionally */
   ManageBoundingBoxes();

   /* Send and receive from external processes */
   InterProcessComm();

   /* Read sensors and run flight software */
   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &SC[Isc];
      if (S->Exists) {
         struct OrbitType *O = &Orb[S->RefOrb];
         Sensors(World, O, S);
         FlightSoftWare(S);
      }
   }

   /* Assign DSM data to comm states */
   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &SC[Isc];
      if (S->Exists && S->FswTag == DSM_FSW) {
         struct DSMType *DSM = &S->DSM;
         DSM->commState      = DSM->CommStateProcessing(DSM->state);
      }
   }
   /* File Output */
   Report();

   /* Exit when Stoptime is reached */
   if (SimComplete) {
      if (TimeMode == FAST_TIME) {
         RealRunTime(&TotalRunTime);
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

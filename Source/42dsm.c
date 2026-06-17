/*    This file is distributed with DeepThought,                      */
/*    a fork of 42,                                                   */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Contributors [aka cool froods]:                                 */
/*    - Daniel Newberry - NASA WFF Intern, Summer 2023 & 2024         */
/*      drnmvd@mst.edu                                                */
/*    - Jerry Varghese - NASA WFF Intern, Summer 2023 & 2024          */
/*      varghes5@purdue.edu                                           */
/*    - Hailey Warner  - NASA WFF Intern, Summer 2024                 */
/*      hlwarner@stanford.edu                                         */
/*    - Matthew Zaffram - NASA WFF Intern, Summer 2022 & 2021         */
/*      mzaffram@gmail.com                                            */
/*    - Rod Regado - NASA WFF Intern, Summer 2022                     */
/*      regadorod@gmail.com                                           */
/*    -.....                                                          */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "42dsm.h"

#define EPS_DSM 1e-12

//------------------------------------------------------------------------------
//                                NAV FUNCTIONS
//------------------------------------------------------------------------------
// Assigns the Jacobian and update functions as needed. Also initializes the
// the default measurement data
void AssignNavFunctions(struct DSMNavType *const Nav,
                        const enum NavType navType)
{
   switch (navType) {
      case RIEKF_NAV:
         Nav->EOMJacobianFun = &eomRIEKFJacobianFun;
         Nav->updateLaw      = &RIEKFUpdateLaw;
         break;
      case LIEKF_NAV:
         Nav->EOMJacobianFun = &eomLIEKFJacobianFun;
         Nav->updateLaw      = &LIEKFUpdateLaw;
         break;
      case MEKF_NAV:
         Nav->EOMJacobianFun = &eomMEKFJacobianFun;
         Nav->updateLaw      = &MEKFUpdateLaw;
         break;
      default:
         printf("Undefined Navigation filter type. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }
}
//------------------------------------------------------------------------------
//                               FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// This solution minimizes 2-norm of thruster commands, resulting in minimum
// power solution. This solution does not know about constraints.  This is able
// to handle the new form of Thruster processing, with Force/Torque ONLY (3DOF),
// OR both Force & Torque (6DOF)
//------------------------------------------------------------------------------
void ThrProcessingMinPower(struct AcType *AC)
{
   long i, j;
   double cmdVec[6], distDotCmd;

   VEC3_TO_DBL(cmdVec, AC->Fcmd);
   VEC3_TO_DBL(&cmdVec[3], AC->Tcmd);

   // Assigning PulseWidth to each Thruster
   for (i = 0; i < AC->Nthr; i++) {
      distDotCmd = 0.0;
      for (j = 0; j < 6; j++)
         distDotCmd += AC->Thr[i].DistVec[j] * cmdVec[j];
      AC->Thr[i].PulseWidthCmd  = Limit(distDotCmd * AC->DT, 0.0, AC->DT);
      AC->Thr[i].ThrustLevelCmd = Limit(distDotCmd, 0.0, 1.0);
      if (AC->Thr[i].PulseWidthCmd > 0)
         AC->Thr[i].PulseWidthFinTimeStamp =
             JDAddSeconds(JD_TT_MJD, AC->Thr[i].PulseWidthCmd);
      else {
         // flag for not set
         AC->Thr[i].PulseWidthFinTimeStamp.system = UTC_TIME;
      }
   }
}
//-------------------------- Initialize Thruster Info --------------------------
// This does the heavy lifting for figuring out how to allocate Thrusters for a
// given Force/Torque Command for use in ThrProcessingMinPower()
//------------------------------------------------------------------------------
void InitThrDistVecs(struct AcType *AC, int DOF, enum CtrlState controllerState)
{
   double **A, **APlus;
   long i, j;

   for (i = 0; i < AC->Nthr; i++) {
      for (j = 0; j < 6; j++)
         AC->Thr[i].DistVec[j] = 0.0;
   }

   A     = CreateMatrix(DOF, AC->Nthr);
   APlus = CreateMatrix(AC->Nthr, DOF);
   for (i = 0; i < AC->Nthr; i++) {
      for (j = 0; j < 3; j++) {
         if (DOF == 3) {
            if (controllerState == TRN_STATE)
               A[j][i] = AC->Thr[i].Axis.v[j];
            else if (controllerState == ATT_STATE ||
                     controllerState == DMP_STATE) {
               A[j][i] = AC->Thr[i].rxA.v[j];
            }
         }
         else if (DOF == 6) {
            A[j][i]     = AC->Thr[i].Axis.v[j];
            A[j + 3][i] = AC->Thr[i].rxA.v[j];
         }
      }
   }
   // Without this, errors would arise if thrusters have different max thrusts
   for (i = 0; i < AC->Nthr; i++)
      for (j = 0; j < DOF; j++)
         A[j][i] *= AC->Thr[i].Fmax;

   PINVG(A, APlus, DOF, AC->Nthr);

   for (i = 0; i < AC->Nthr; i++) {
      if (DOF == 3) {
         // Unused entries of DistVec will be zero, so won't cause issues with
         // eventual dot product
         for (j = 0; j < DOF; j++) {
            if (controllerState == TRN_STATE)
               AC->Thr[i].DistVec[j] = APlus[i][j];
            else if (controllerState == ATT_STATE ||
                     controllerState == DMP_STATE)
               AC->Thr[i].DistVec[j + 3] = APlus[i][j];
         }
      }
      else if (DOF == 6)
         for (j = 0; j < DOF; j++)
            AC->Thr[i].DistVec[j] = APlus[i][j];
   }
   DestroyMatrix(A);
   DestroyMatrix(APlus);
}
//------------------------------------------------------------------------------
//                           Initialize DSM Structure
//------------------------------------------------------------------------------
void InitDSM(struct SCType *S)
{
   struct DSMType *DSM            = &S->DSM;
   struct DSMNavType *Nav         = &DSM->DsmNav;
   struct DSMCmdType *Cmd         = &DSM->Cmd;
   struct DSMStateType *state     = &DSM->state;
   struct DSMStateType *commState = &DSM->commState;

   S->InitDSM               = 0;
   DSM->Init                = 1;
   DSM->ID                  = S->ID;
   state->ID                = DSM->ID;
   commState->ID            = DSM->ID;
   DSM->CommStateProcessing = &DSM_CommStateProcessing;
   DSM->CmdInit             = 1;
   DSM->DT                  = S->AC.DT;
   DSM->mass                = S->AC.mass;
   DSM->refOrb              = &Orb[S->RefOrb];

   DSM->MOI = S->AC.MOI;

   double avgArea = 0.0;
   long nPoly     = 0;
   long Ib, Ipoly;
   for (Ib = 0; Ib < S->Nb; Ib++) {
      struct BodyType *B = &S->B[Ib];
      struct GeomType *G = &Geom[B->GeomTag];
      for (Ipoly = 0; Ipoly < G->Npoly; Ipoly++) {
         struct PolyType *P = &G->Poly[Ipoly];
         nPoly++;
         avgArea += P->Area;
      }
   }
   avgArea            /= nPoly;
   Nav->ballisticCoef  = DSM->mass / (S->DragCoef * avgArea);
   S->InitDSM          = 0;
   DSM->Init           = 1;

   /* Controllers */
   DSM->DsmCtrl.Init         = 1;
   DSM->DsmCtrl.H_DumpActive = FALSE;
   DSM->CmdArray             = NULL;

   Cmd->TranslationCtrlActive = FALSE;
   Cmd->AttitudeCtrlActive    = FALSE;
   Cmd->H_DumpActive          = FALSE;
   strcpy(Cmd->dmp_actuator, "");
   Cmd->ActNumCmds = 0;

   Nav->type              = IDEAL_NAV;
   Nav->batching          = NONE_BATCH;
   Nav->refFrame          = FRAME_N;
   Nav->NavigationActive  = FALSE;
   Nav->DT                = S->AC.DT;
   Nav->ccsds_time.coarse = 0;
   Nav->ccsds_time.fine   = 0;
   Nav->steps             = 0;
   Nav->jd_tt_mjd_0       = JD_ZERO;
   Nav->jd_tt_mjd         = JD_ZERO;
   Nav->Date.Year         = 0;
   Nav->Date.Month        = 0;
   Nav->Date.Day          = 0;
   Nav->Date.doy          = 0;
   Nav->Date.Hour         = 0;
   Nav->Date.Minute       = 0;
   Nav->Date.Second       = RATIONAL_ZERO;

   FOR_STATES(i)
   {
      Nav->stateActive[i] = FALSE;
   }
   FOR_SENSORS(i)
   {
      Nav->sensorActive[i] = NULL;
      Nav->measTypes[i]    = NULL;
      Nav->innovations[i]  = NULL;
   }
   InitMeasList(&Nav->measList);
   Nav->innovationsReportFirst = TRUE;
   Nav->innovationTime         = -1.0;
   Nav->innovationsExist       = FALSE;

   /* Initialize pointers to NULL */
   Nav->sqrQ       = NULL;
   Nav->M          = NULL;
   Nav->P          = NULL;
   Nav->delta      = NULL;
   Nav->jacobian   = NULL;
   Nav->STM        = NULL;
   Nav->STMStep    = NULL;
   Nav->NxN        = NULL;
   Nav->NxN2       = NULL;
   Nav->whlH       = NULL;
   Nav->refOriBody = 0;
   Nav->refOriType = 0;
   Nav->refOriPtr  = NULL;
   Nav->refBodyPtr = NULL;

   Nav->oldRefCRN      = MAT3X3_EYE;
   Nav->oldRefPos      = VEC3_ZERO;
   Nav->oldRefVel      = VEC3_ZERO;
   Nav->oldRefOmega    = VEC3_ZERO;
   Nav->oldRefOmegaDot = VEC3_ZERO;
   Nav->forceB         = VEC3_ZERO;
   Nav->torqueB        = VEC3_ZERO;

   Nav->reportConfigured = FALSE;
}
//------------------------------------------------------------------------------
//                           COMMAND INTERPRETER
//------------------------------------------------------------------------------

#define FIELDWIDTH 63
//------------------------------------ GAINS -----------------------------------
long GetGains(struct DSMType *const DSM, struct fy_node *gainsNode,
              enum CtrlState controllerState)
{
   long GainsProcessed = FALSE;

   enum CtrlType *controller = NULL;
   struct DSMCmdType *Cmd    = &DSM->Cmd;
   double *kp = NULL, *kr = NULL, *ki = NULL, *limit_vec = NULL;

   switch (controllerState) {
      case TRN_STATE:
         controller = &Cmd->trn_controller;
         kp         = Cmd->trn_kp.v;
         kr         = Cmd->trn_kr.v;
         ki         = Cmd->trn_ki.v;
         limit_vec  = Cmd->trn_kilimit.v;
         break;
      case ATT_STATE:
         controller = &Cmd->att_controller;
         kp         = Cmd->att_kp.v;
         kr         = Cmd->att_kr.v;
         ki         = Cmd->att_ki.v;
         limit_vec  = Cmd->att_kilimit.v;
         break;
      case FULL_STATE:
         // PLACEHOLDER
         break;
      case DMP_STATE:
         controller = &Cmd->dmp_controller;
         kp         = Cmd->dmp_kp.v;
         break;
      default:
         break;
   }
   long i;
   char gainMode[31] = {0};
   double omega, zeta, alpha, k_lya, limit;
   fy_node_scanf(gainsNode, "/Type %30s", gainMode);
   struct fy_node *gainsDataNode = fy_node_by_path_def(gainsNode, "/Gains");
   if (!strcmp(gainMode, "PID")) {
      const char gainPaths[4][20] = {"/Kp", "/Kr", "/Ki", "/Ki_Limit"};
      double *const gains[4]      = {kp, kr, ki, limit_vec};
      long gainsGood              = TRUE;
      for (i = 0; i < 4; i++)
         gainsGood &= assignYAMLToDoubleArray(
                          3, fy_node_by_path_def(gainsDataNode, gainPaths[i]),
                          gains[i]) == 3;
      if (gainsGood && *controller == PID_CNTRL)
         GainsProcessed = TRUE;
   }
   else if (!strcmp(gainMode, "PID_WN")) {
      if (fy_node_scanf(gainsDataNode,
                        "/Omega %lf /Zeta %lf /Alpha %lf /Ki_Limit %lf", &omega,
                        &zeta, &alpha, &limit) == 4 &&
          *controller == PID_CNTRL) {
         GainsProcessed = TRUE;
         for (i = 0; i < 3; i++) {
            kp[i]        = (2 * zeta * alpha + 1) * omega * omega;
            kr[i]        = (2 * zeta + alpha) * omega;
            ki[i]        = alpha * omega * omega * omega;
            limit_vec[i] = limit;
         }
         switch (controllerState) {
            case TRN_STATE:
               for (i = 0; i < 3; i++) {
                  kp[i] *= DSM->mass;
                  kr[i] *= DSM->mass;
                  ki[i] *= DSM->mass;
               }
               break;
            case ATT_STATE:
               for (i = 0; i < 3; i++) {
                  kp[i] *= DSM->MOI.mat[i][i];
                  kr[i] *= DSM->MOI.mat[i][i];
                  ki[i] *= DSM->MOI.mat[i][i];
               }
               break;
            case FULL_STATE:
               // PLACEHOLDER
               break;
            case DMP_STATE:
               // shouldn't get here
               break;
            default:
               break;
         }
      }
   }
   else if (!strcmp(gainMode, "FC_LYA")) {
      struct fy_node *kNode = fy_node_by_path_def(gainsDataNode, "/K_lya");
      switch (*controller) {
         case LYA_2BODY_CNTRL:
            if (fy_node_sequence_item_count(kNode) == 2) {
               fy_node_scanf(fy_node_sequence_get_by_index(kNode, 0), "/ %lf",
                             &omega);
               fy_node_scanf(fy_node_sequence_get_by_index(kNode, 1), "/ %lf",
                             &zeta);
               for (i = 0; i < 3; i++) {
                  kp[i] = omega * omega * DSM->mass;
                  kr[i] = 2 * zeta * omega * DSM->mass;
               }
               GainsProcessed = TRUE;
            }
            break;
         case LYA_ATT_CNTRL:
            if (fy_node_sequence_item_count(kNode) == 2) {
               fy_node_scanf(fy_node_sequence_get_by_index(kNode, 0), "/ %lf",
                             &k_lya);
               for (i = 0; i < 3; i++) {
                  kp[i] = k_lya;
                  kr[i] = sqrt(2.0 * k_lya * DSM->MOI.mat[i][i]);
               }
               GainsProcessed = TRUE;
            }
            break;
         default:
            break;
      }
   }
   else if (!strcmp(gainMode, "MomentumDump")) {
      if (controllerState != DMP_STATE) {
         fprintf(
             stderr,
             "Gain alias %s gain sets can only be used for momentum dumping. "
             "Exiting...",
             fy_anchor_get_text(fy_node_get_anchor(gainsNode), NULL));
         exit(EXIT_FAILURE);
      }
      struct fy_node *kpNode = fy_node_by_path_def(gainsDataNode, "/Kp");
      if (assignYAMLToDoubleArray(3, kpNode, kp) == 3)
         GainsProcessed = TRUE;
   }

   // Set GainsProcessed flag for relevant controller
   if (GainsProcessed == TRUE) {
      if (controllerState == TRN_STATE) {
         Cmd->NewTrnGainsProcessed = TRUE;
      }
      else if (controllerState == ATT_STATE) {
         Cmd->NewAttGainsProcessed = TRUE;
      }
   }
   return (GainsProcessed);
}
//----------------------------------- LIMITS -----------------------------------
long GetLimits(struct DSMType *const DSM, struct fy_node *limsNode,
               enum CtrlState controllerState)
{
   long LimitsProcessed = FALSE;

   double *fMax = NULL, *vMax = NULL;
   struct DSMCmdType *Cmd    = &DSM->Cmd;
   enum CtrlType *controller = NULL;
   switch (controllerState) {
      case TRN_STATE:
         controller = &Cmd->trn_controller;
         fMax       = Cmd->FrcB_max.v;
         vMax       = Cmd->vel_max.v;
         break;
      case ATT_STATE:
         controller = &Cmd->att_controller;
         fMax       = Cmd->Trq_max.v;
         vMax       = Cmd->w_max.v;
         break;
      case FULL_STATE:
         // PLACEHOLDER
         break;
      case DMP_STATE:
         controller = &Cmd->dmp_controller;
         fMax       = Cmd->dTrq_max.v;
         break;
      default:
         break;
   }
   if (assignYAMLToDoubleArray(3, fy_node_by_path_def(limsNode, "/Force Max"),
                               fMax) == 3 &&
       (*controller == H_DUMP_CNTRL ||
        assignYAMLToDoubleArray(
            3, fy_node_by_path_def(limsNode, "/Velocity Max"), vMax) == 3))
      LimitsProcessed = TRUE;
   if (controllerState == ATT_STATE) {
      for (long i = 0; i < 3; i++)
         vMax[i] *= D2R;
   }
   return (LimitsProcessed);
}
//--------------------------------- CONTROLLER ---------------------------------
long GetController(struct DSMType *const DSM, struct fy_node *ctrlNode,
                   enum CtrlState controllerState)
{
   struct fy_node *gainNode = NULL, *limNode = NULL;

   long CntrlProcessed = FALSE;

   struct DSMCmdType *Cmd = &DSM->Cmd;

   enum CtrlType controller;
   char ctrlType[40] = {0};
   if (fy_node_scanf(ctrlNode, "/Type %39s", ctrlType) == 1) {
      gainNode = fy_node_by_path_def(ctrlNode, "/Gains");
      limNode  = fy_node_by_path_def(ctrlNode, "/Limits");
      if (!strcmp(ctrlType, "PID_CNTRL"))
         controller = PID_CNTRL;
      else if (!strcmp(ctrlType, "LYA_ATT_CNTRL"))
         controller = LYA_ATT_CNTRL;
      else if (!strcmp(ctrlType, "LYA_2BODY_CNTRL"))
         controller = LYA_2BODY_CNTRL;
      else if (!strcmp(ctrlType, "H_DUMP_CNTRL"))
         controller = H_DUMP_CNTRL;
      else {
         fprintf(stderr, "%s is an invalid control type. Exiting...\n",
                 ctrlType);
         exit(EXIT_FAILURE);
      }
      // There should be a nicer way to handle this that doesn't require
      // hardcoding for things...
      if (controller == LYA_ATT_CNTRL && controllerState != ATT_STATE) {
         fprintf(
             stderr,
             "Can only use LYA_ATT_CNTRL for attitude control. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      if (controller == H_DUMP_CNTRL && controllerState != DMP_STATE) {
         fprintf(stderr,
                 "Can only use H_DUMP_CNTRL for momentum dumping control. "
                 "Exiting...\n");
         exit(EXIT_FAILURE);
      }
      if (controller == LYA_2BODY_CNTRL && controllerState != TRN_STATE) {
         fprintf(stderr,
                 "Can only use LYA_2BODY_CNTRL for translation control. "
                 "Exiting...\n");
         exit(EXIT_FAILURE);
      }
      CntrlProcessed = TRUE;
      switch (controllerState) {
         case TRN_STATE:
            Cmd->trn_controller = controller;
            break;
         case ATT_STATE:
            Cmd->att_controller = controller;
            break;
         case FULL_STATE:
            // PLACEHOLDER
            break;
         case DMP_STATE:
            Cmd->dmp_controller = controller;
            break;
         default:
            break;
      }
      if (GetGains(DSM, gainNode, controllerState) == FALSE) {
         fprintf(stderr,
                 "For Controller alias %s, could not find Gain alias %s or "
                 "invalid format. Exiting...\n",
                 fy_anchor_get_text(fy_node_get_anchor(ctrlNode), NULL),
                 fy_anchor_get_text(fy_node_get_anchor(gainNode), NULL));
         exit(EXIT_FAILURE);
      }
      if (GetLimits(DSM, limNode, controllerState) == FALSE) {
         fprintf(stderr,
                 "For Controller alias %s, could not find Limit alias %s or "
                 "invalid format. Exiting...\n",
                 fy_anchor_get_text(fy_node_get_anchor(ctrlNode), NULL),
                 fy_anchor_get_text(fy_node_get_anchor(limNode), NULL));
         exit(EXIT_FAILURE);
      }
   }

   return (CntrlProcessed);
}
//---------------------------------- ACTUATORS ---------------------------------
long GetActuators(struct AcType *const AC, struct DSMType *const DSM,
                  struct fy_node *actNode, enum CtrlState controllerState)
{
   long ActuatorsProcessed = FALSE;

   struct DSMCmdType *Cmd = &DSM->Cmd;
   char actName[40]       = {0};
   if (fy_node_scanf(actNode, "/Type %39s", actName) == 1) {
      // disable dumping if new attitude command is declared without using
      // wheels
      if (controllerState == ATT_STATE && strcmp(actName, "WHL")) {
         // Null out dump actuator to avoid other errors
         strcpy(Cmd->dmp_actuator, "");
         Cmd->H_DumpActive = FALSE;
      }
      ActuatorsProcessed = TRUE;
      // This handles invalid actuator names
      if (!strcmp(actName, "WHL")) {
         if (controllerState == TRN_STATE || controllerState == DMP_STATE)
            ActuatorsProcessed = FALSE;
      }
      else if (!strcmp(actName, "MTB")) {
         if (controllerState == TRN_STATE)
            ActuatorsProcessed = FALSE;
      }
      else if (!strcmp(actName, "THR_3DOF")) {
         InitThrDistVecs(AC, 3, controllerState);
      }
      else if (!strcmp(actName, "THR_6DOF")) {
         InitThrDistVecs(AC, 6, controllerState);
      }
      else if (!strcmp(actName, "Ideal")) {
         // Ideal do what it wants
      }
      else {
         ActuatorsProcessed = FALSE;
      }

      switch (controllerState) {
         case TRN_STATE:
            strcpy(Cmd->trn_actuator, actName);
            break;
         case ATT_STATE:
            strcpy(Cmd->att_actuator, actName);
            break;
         case FULL_STATE:
            // PLACEHOLDER
            break;
         case DMP_STATE:
            strcpy(Cmd->dmp_actuator, actName);
            break;
         default:
            break;
      }
   }
   return (ActuatorsProcessed);
}
//------------------------- TRANSLATIONAL CMD ----------------------------------
long GetTranslationCmd(struct AcType *const AC, struct DSMType *const DSM,
                       struct fy_node *trnCmdNode, const double DsmCmdTime)
{
   struct fy_node *ctrlNode = NULL, *actNode = NULL, *limNode = NULL;
   long TranslationCmdProcessed = FALSE;

   struct DSMCmdType *const Cmd = &DSM->Cmd;

   char subType[FIELDWIDTH + 1] = {};
   const char *searchStr        = "/Subtype %" STR(FIELDWIDTH) "[^\n]";
   fy_node_scanf(trnCmdNode, searchStr, subType);
   if (!strcmp(subType, "NO_CHANGE")) {
      TranslationCmdProcessed = TRUE;
      return (TranslationCmdProcessed);
   }
   else if (!strcmp(subType, "Passive")) {
      Cmd->TranslationCtrlActive = FALSE;
      TranslationCmdProcessed    = TRUE;
      return (TranslationCmdProcessed);
   }

   struct fy_node *cmdNode = fy_node_by_path_def(trnCmdNode, "/Command Data");
   if (cmdNode == NULL) {
      fprintf(stderr,
              "Could not find Command Data for Translation command of subtype "
              "%s. Exiting...\n",
              subType);
      exit(EXIT_FAILURE);
   }
   const char *cmdName =
       fy_node_get_scalar0(fy_node_by_path_def(cmdNode, "/Description"));

   if (!strcmp(subType, "Position")) {
      Cmd->TranslationCtrlActive = TRUE;
      long isGood = fy_node_scanf(cmdNode,
                                  "/Origin %19s "
                                  "/Frame %19s",
                                  Cmd->RefOrigin, Cmd->RefFrame) == 2;
      if (!strcmp(Cmd->RefFrame, "E")) {
         isGood &= fy_node_scanf(cmdNode,
                                 "/Distance %lf "
                                 "/Phase %lf",
                                 &Cmd->Distance, &Cmd->Phase) == 2;

         Cmd->Phase *= D2R;
         strcpy(Cmd->TranslationType, "Position");
      }
      else {
         isGood &=
             assignYAMLToDoubleArray(
                 3, fy_node_by_path_def(cmdNode, "/Position"), Cmd->Pos.v) == 3;
      }
      ctrlNode  = fy_node_by_path_def(cmdNode, "/Controller");
      actNode   = fy_node_by_path_def(cmdNode, "/Actuator");
      isGood   &= ctrlNode != NULL && actNode != NULL;

      if (isGood) {
         TranslationCmdProcessed = TRUE;
         Cmd->ManeuverMode       = MAN_INACTIVE;
      }

      if (TranslationCmdProcessed == FALSE) {
         fprintf(stderr, "Position Command %s has invalid format. Exiting...\n",
                 cmdName);
         exit(EXIT_FAILURE);
      }
   }
   else if (!strcmp(subType, "Translation")) {
      Cmd->TranslationCtrlActive = TRUE;

      long isGood  = fy_node_scanf(cmdNode,
                                   "/Origin %19s "
                                   "/Frame %19s",
                                   Cmd->RefOrigin, Cmd->RefFrame) == 2;
      isGood      &= fy_node_scanf(cmdNode, "/Translation Type %19s ",
                                   Cmd->TranslationType) == 1;
      if (!strcmp(Cmd->TranslationType, "Circumnavigation")) {
         isGood &= fy_node_scanf(cmdNode,
                                 "/Distance %lf "
                                 "/Phase %lf ",
                                 &Cmd->Distance, &Cmd->Phase) == 2;

         Cmd->Phase      *= D2R;
         Cmd->ResetTimer  = 1;
      }
      else if (!strcmp(Cmd->TranslationType, "Docking")) {
         isGood &=
             fy_node_scanf(cmdNode, "/Time to Dock %lf ", &Cmd->TimeDock) == 1;
         Cmd->ResetTimer = 1;
      }
      ctrlNode  = fy_node_by_path_def(cmdNode, "/Controller");
      actNode   = fy_node_by_path_def(cmdNode, "/Actuator");
      isGood   &= ctrlNode != NULL && actNode != NULL;

      if (isGood) {
         TranslationCmdProcessed = TRUE;
         Cmd->ManeuverMode       = MAN_INACTIVE;
      }
   }
   else if (!strcmp(subType, "Maneuver")) {
      Cmd->TranslationCtrlActive   = TRUE;
      char manType[FIELDWIDTH + 1] = {0};
      const char *searchManStr =
          "/Type %" STR(FIELDWIDTH) "s /Frame %19s /Duration %lf";

      long isGood = fy_node_scanf(cmdNode, searchManStr, manType, Cmd->RefFrame,
                                  &Cmd->BurnTime) == 3;
      limNode     = fy_node_by_path_def(cmdNode, "/Limits");
      actNode     = fy_node_by_path_def(cmdNode, "/Actuator");
      isGood &=
          assignYAMLToDoubleArray(3, fy_node_by_path_def(cmdNode, "/Delta V"),
                                  Cmd->DeltaV.v) == 3;
      if (isGood) {
         TranslationCmdProcessed = TRUE;
         Cmd->BurnStopTime       = DsmCmdTime + Cmd->BurnTime;
         if (!strcmp(manType, "CONSTANT"))
            Cmd->ManeuverMode = MAN_CONSTANT;
         else if (!strcmp(manType, "SMOOTHED"))
            Cmd->ManeuverMode = MAN_SMOOTHED;
         else {
            fprintf(
                stderr,
                "%s is an invalid maneuver mode for Maneuver %s. Exiting...",
                manType, cmdName);
            exit(EXIT_FAILURE);
         }
      }
      if (TranslationCmdProcessed == FALSE) {
         fprintf(stderr,
                 "Translation Command %s has invalid format. Exiting...\n",
                 cmdName);
         exit(EXIT_FAILURE);
      }
   }

   if (TranslationCmdProcessed == TRUE && Cmd->TranslationCtrlActive == TRUE) {
      if (Cmd->ManeuverMode == MAN_INACTIVE) {
         if (GetController(DSM, ctrlNode, TRN_STATE) == FALSE) {
            fprintf(stderr,
                    "For %s command %s, could not find Controller alias %s or "
                    "invalid format. Exiting...\n",
                    subType, cmdName,
                    fy_anchor_get_text(fy_node_get_anchor(ctrlNode), NULL));
            exit(EXIT_FAILURE);
         }
      }
      else {
         if (GetLimits(DSM, limNode, TRN_STATE) == FALSE) {
            fprintf(
                stderr,
                "For %s command %s, could not find Limit alias %s or invalid "
                "format. Exiting...\n",
                subType, cmdName,
                fy_anchor_get_text(fy_node_get_anchor(limNode), NULL));
            exit(EXIT_FAILURE);
         }
      }
      if (GetActuators(AC, DSM, actNode, TRN_STATE) == FALSE) {
         fprintf(
             stderr,
             "For %s command %s, could not find Actuator alias %s or invalid "
             "format. Exiting...\n",
             subType, cmdName,
             fy_anchor_get_text(fy_node_get_anchor(actNode), NULL));
         exit(EXIT_FAILURE);
      }
   }

   return (TranslationCmdProcessed);
}
//-----------------------ATTITUDE CMD ---------------------------------------
long GetAttitudeCmd(struct AcType *const AC, struct DSMType *const DSM,
                    struct fy_node *attCmdNode)
{
   struct fy_node *ctrlNode = NULL, *actNode = NULL;
   long AttitudeCmdProcessed = FALSE, AttPriCmdProcessed = FALSE,
        AttSecCmdProcessed = FALSE;
   char GroundStationCmd[30];

   enum CtrlState state   = ATT_STATE;
   struct DSMCmdType *Cmd = &DSM->Cmd;

   char subType[FIELDWIDTH + 1] = {};
   const char *searchStr        = "/Subtype %" STR(FIELDWIDTH) "[^\n]";
   fy_node_scanf(attCmdNode, searchStr, subType);
   if (!strcmp(subType, "NO_CHANGE")) {
      AttitudeCmdProcessed = TRUE;
      return (AttitudeCmdProcessed);
   }
   else if (!strcmp(subType, "Passive")) {
      Cmd->AttitudeCtrlActive = FALSE;
      Cmd->H_DumpActive       = FALSE;
      AttitudeCmdProcessed    = TRUE;
      return (AttitudeCmdProcessed);
   }

   struct fy_node *cmdNode = fy_node_by_path_def(attCmdNode, "/Command Data");

   if (cmdNode == NULL) {
      fprintf(stderr,
              "Could not find Command Data for Attitude command of subtype %s. "
              "Exiting...\n",
              subType);
      exit(EXIT_FAILURE);
   }
   const char *cmdName =
       fy_node_get_scalar0(fy_node_by_path_def(cmdNode, "/Description"));

   if (!strcmp(subType, "Two Vector") || !strcmp(subType, "One Vector")) {
      long kMax;
      struct DSMCmdVecType *const vecs[] = {&Cmd->PriVec, &Cmd->SecVec};
      struct fy_node *const nodes[]      = {
          fy_node_by_path_def(cmdNode, "/Primary Vector"),
          fy_node_by_path_def(cmdNode, "/Secondary Vector")};
      char *const cmdRefFrm[]  = {Cmd->PriAttRefFrame, Cmd->SecAttRefFrame};
      long *const attcmdProc[] = {&AttPriCmdProcessed, &AttSecCmdProcessed};

      if (nodes[0] == NULL) {
         fprintf(stderr,
                 "For Vector command %s, could not find Primary Vector. "
                 "Exiting...\n",
                 cmdName);
         exit(EXIT_FAILURE);
      }

      if (!strcmp(subType, "Two Vector")) {
         kMax        = 2;
         Cmd->Method = PARM_VECTORS;

         if (nodes[1] == NULL) {
            fprintf(stderr,
                    "For Two Vector command %s, could not find Secondary "
                    "Vector. Exiting...\n",
                    cmdName);
            exit(EXIT_FAILURE);
         }
      }
      else {
         kMax               = 1;
         Cmd->Method        = PARM_UNITVECTOR;
         AttSecCmdProcessed = TRUE;
      }

      for (int k = 0; k < kMax; k++) {
         struct fy_node *tgtNode = fy_node_by_path_def(nodes[k], "/Target");
         assignYAMLToDoubleArray(3, fy_node_by_path_def(nodes[k], "/Axis"),
                                 vecs[k]->cmd_axis.v);
         char tgtType[50] = {0};
         fy_node_scanf(tgtNode, "/Type %49s", tgtType);

         if (!strcmp(tgtType, "BODY") || !strcmp(tgtType, "SC")) {
            vecs[k]->CmdMode = CMD_TARGET;
            char target[50]  = {0};
            fy_node_scanf(tgtNode, "/Target %49s", target);
            if (!strcmp(tgtType, "BODY")) {
               vecs[k]->TrgType = TARGET_WORLD;
               long gsNum;
               strcpy(GroundStationCmd, "GroundStation_[%ld]");
               if (sscanf(target, GroundStationCmd, &gsNum) == 1) {
                  vecs[k]->TrgWorld = GroundStation[gsNum].World;
                  vecs[k]->W        = GroundStation[gsNum].PosW;
               }
               else {
                  vecs[k]->TrgWorld = GetWorldID(target);
                  vecs[k]->W        = VEC3_ZERO;
               }
            }
            else if (!strcmp(tgtType, "SC")) {
               vecs[k]->TrgType = TARGET_SC;
               if (sscanf(target, "SC[%ld].B[%ld]", &vecs[k]->TrgSC,
                          &vecs[k]->TrgBody) == 2) {
                  // Decode Current SC ID Number
                  if (vecs[k]->TrgSC >= Nsc) {
                     fprintf(stderr,
                             "This mission only has %ld spacecraft, but "
                             "spacecraft %ld was attempted to be set as the "
                             "primary target vector. Exiting...\n",
                             Nsc, vecs[k]->TrgSC);
                     exit(EXIT_FAILURE);
                  }

                  if (vecs[k]->TrgBody >= SC[vecs[k]->TrgSC].Nb) {
                     fprintf(stderr,
                             "Spacecraft %ld only has %ld bodies, but the "
                             "primary target was attempted to be set as body "
                             "%ld. Exiting...\n",
                             vecs[k]->TrgSC, SC[vecs[k]->TrgSC].Nb,
                             vecs[k]->TrgBody);
                     exit(EXIT_FAILURE);
                  }
               }
               else {
                  fprintf(stderr, "%s is in incorrect format. Exiting...",
                          target);
                  exit(EXIT_FAILURE);
               }
            }
            else {
               fprintf(stderr,
                       "%s Vector for command %s has improper format for SC or "
                       "BODY targeting. Exiting...\n",
                       (k == 0) ? ("Primary") : ("Secondary"), cmdName);
               exit(EXIT_FAILURE);
            }
            *attcmdProc[k] = TRUE;
         }
         else if (!strcmp(tgtType, "VEC")) {
            vecs[k]->CmdMode = CMD_DIRECTION;
            vecs[k]->TrgType = TARGET_VEC;
            *attcmdProc[k] =
                fy_node_scanf(tgtNode, "/Frame %19s", cmdRefFrm[k]);
            *attcmdProc[k] &= assignYAMLToDoubleArray(
                                  3, fy_node_by_path_def(tgtNode, "/Axis"),
                                  vecs[k]->cmd_vec.v) == 3;
            if (*attcmdProc[k] == FALSE) {
               fprintf(stderr,
                       "%s Vector for command %s has improper format for VEC "
                       "targeting. Exiting...\n",
                       (k == 0) ? ("Primary") : ("Secondary"), cmdName);
               exit(EXIT_FAILURE);
            }
         }
         else {
            fprintf(stderr,
                    "For %s Vector for command %s, %s is an invalid targeting "
                    "type. Exiting...\n",
                    (k == 0) ? ("Primary") : ("Secondary"), cmdName, tgtType);
            exit(EXIT_FAILURE);
         }
      }

      ctrlNode = fy_node_by_path_def(cmdNode, "/Controller");
      actNode  = fy_node_by_path_def(cmdNode, "/Actuator");

      if (AttPriCmdProcessed == TRUE && AttSecCmdProcessed == TRUE)
         AttitudeCmdProcessed = TRUE;

      Cmd->AttitudeCtrlActive = TRUE;
   }
   else if (!strcmp(subType, "Quaternion")) {
      Cmd->Method = PARM_QUATERNION;

      AttitudeCmdProcessed =
          assignYAMLToDoubleArray(
              4, fy_node_by_path_def(cmdNode, "/Quaternion"), Cmd->q.q) == 4;
      AttitudeCmdProcessed &=
          fy_node_scanf(cmdNode, "/Frame %19s", Cmd->AttRefFrame) == 1;
      ctrlNode              = fy_node_by_path_def(cmdNode, "/Controller");
      actNode               = fy_node_by_path_def(cmdNode, "/Actuator");
      AttitudeCmdProcessed &= ctrlNode != NULL && actNode != NULL;

      if (AttitudeCmdProcessed == FALSE) {
         fprintf(stderr,
                 "Quaternion Command %s has invalid format. Exiting...\n",
                 cmdName);
         exit(EXIT_FAILURE);
      }

      Cmd->AttitudeCtrlActive = TRUE;
   }
   else if (!strcmp(subType, "Mirror")) {
      Cmd->Method = PARM_MIRROR;

      AttitudeCmdProcessed =
          fy_node_scanf(cmdNode, "/Target %19s", Cmd->AttRefScID) == 1;
      ctrlNode              = fy_node_by_path_def(cmdNode, "/Controller");
      actNode               = fy_node_by_path_def(cmdNode, "/Actuator");
      AttitudeCmdProcessed &= ctrlNode != NULL && actNode != NULL;

      if (AttitudeCmdProcessed == FALSE) {
         fprintf(stderr, "Mirror Command %s has invalid format. Exiting...\n",
                 cmdName);
         exit(EXIT_FAILURE);
      }

      Cmd->AttitudeCtrlActive = TRUE;
   }
   else if (!strcmp(subType, "Detumble")) {
      Cmd->Method = PARM_DETUMBLE;

      ctrlNode             = fy_node_by_path_def(cmdNode, "/Controller");
      actNode              = fy_node_by_path_def(cmdNode, "/Actuator");
      AttitudeCmdProcessed = ctrlNode != NULL && actNode != NULL;

      if (AttitudeCmdProcessed == FALSE) {
         fprintf(stderr, "Detumble Command %s has invalid format. Exiting...\n",
                 cmdName);
         exit(EXIT_FAILURE);
      }

      Cmd->AttitudeCtrlActive = TRUE;
   }
   else if (!strcmp(subType, "Whl H Manage")) {
      AttitudeCmdProcessed =
          fy_node_scanf(cmdNode,
                        "/Minimum H_norm %lf "
                        "/Maximum H_norm %lf",
                        &Cmd->H_DumpLims[0], &Cmd->H_DumpLims[1]) == 2;

      ctrlNode              = fy_node_by_path_def(cmdNode, "/Controller");
      actNode               = fy_node_by_path_def(cmdNode, "/Actuator");
      AttitudeCmdProcessed &= ctrlNode != NULL && actNode != NULL;

      struct fy_node *dumpNode  = fy_node_by_path_def(cmdNode, "/Dumping");
      AttitudeCmdProcessed     &= dumpNode != NULL;
      Cmd->H_DumpActive         = getYAMLBool(dumpNode);
      state                     = DMP_STATE;
      if (Cmd->H_DumpLims[1] < Cmd->H_DumpLims[0]) {
         fprintf(stderr,
                 "Maximum momentum dump limit must be more than the minimum "
                 "for Whl H Manage Command %s Exiting...\n",
                 cmdName);
         exit(EXIT_FAILURE);
      }
      if (AttitudeCmdProcessed == FALSE) {
         fprintf(stderr,
                 "Whl H Manage Command %s has invalid format. Exiting...\n",
                 cmdName);
         exit(EXIT_FAILURE);
      }
   }
   else if (!strcmp(subType, "Spin Vector")) {
      Cmd->Method = PARM_AXIS_SPIN;

      // Configure Primary Vector
      struct DSMCmdVecType *vec = &Cmd->PriVec;
      struct fy_node *tgtNode   = fy_node_by_path_def(cmdNode, "/Target");
      assignYAMLToDoubleArray(3, fy_node_by_path_def(cmdNode, "/Axis"),
                              vec->cmd_axis.v);
      char tgtType[50] = {0};
      fy_node_scanf(tgtNode, "/Type %49s", tgtType);

      if (!strcmp(tgtType, "BODY") || !strcmp(tgtType, "SC")) {
         vec->CmdMode    = CMD_TARGET;
         char target[50] = {0};
         fy_node_scanf(tgtNode, "/Target %49s", target);
         if (!strcmp(tgtType, "BODY")) {
            vec->TrgType = TARGET_WORLD;
            long gsNum;
            strcpy(GroundStationCmd, "GroundStation_[%ld]");
            if (sscanf(target, GroundStationCmd, &gsNum) == 1) {
               vec->TrgWorld = GroundStation[gsNum].World;
               vec->W        = GroundStation[gsNum].PosW;
            }
            else {
               vec->TrgWorld = DecodeString(target);
               vec->W        = VEC3_ZERO;
            }
         }
         else if (!strcmp(tgtType, "SC")) {
            vec->TrgType = TARGET_SC;
            if (sscanf(target, "SC[%ld].B[%ld]", &vec->TrgSC, &vec->TrgBody) ==
                2) {
               // Decode Current SC ID Number
               if (vec->TrgSC >= Nsc) {
                  fprintf(stderr,
                          "This mission only has %ld spacecraft, but "
                          "spacecraft %ld was attempted to be set as the "
                          "primary target vector. Exiting...\n",
                          Nsc, vec->TrgSC);
                  exit(EXIT_FAILURE);
               }

               if (vec->TrgBody >= SC[vec->TrgSC].Nb) {
                  fprintf(stderr,
                          "Spacecraft %ld only has %ld bodies, but the primary "
                          "target was attempted to be set as body %ld. "
                          "Exiting...\n",
                          vec->TrgSC, SC[vec->TrgSC].Nb, vec->TrgBody);
                  exit(EXIT_FAILURE);
               }
            }
            else {
               fprintf(stderr, "%s is in incorrect format. Exiting...", target);
               exit(EXIT_FAILURE);
            }
         }
         else {
            fprintf(stderr,
                    "Vector for command %s has improper format for SC or BODY "
                    "targeting. Exiting...\n",
                    cmdName);
            exit(EXIT_FAILURE);
         }
         AttitudeCmdProcessed = TRUE;
      }
      else if (!strcmp(tgtType, "VEC")) {
         vec->CmdMode = CMD_DIRECTION;
         vec->TrgType = TARGET_VEC;
         AttitudeCmdProcessed =
             fy_node_scanf(tgtNode, "/Frame %19s", Cmd->PriAttRefFrame);
         AttitudeCmdProcessed &=
             assignYAMLToDoubleArray(3, fy_node_by_path_def(tgtNode, "/Axis"),
                                     vec->cmd_vec.v) == 3;
         if (AttitudeCmdProcessed == FALSE) {
            fprintf(stderr,
                    "Vector for command %s has improper format for VEC "
                    "targeting. Exiting...\n",
                    cmdName);
            exit(EXIT_FAILURE);
         }
      }
      else {
         fprintf(stderr,
                 "For Vector for command %s, %s is an invalid targeting type. "
                 "Exiting...\n",
                 cmdName, tgtType);
         exit(EXIT_FAILURE);
      }
      // Load Desired Angular Rate in Cmd->AngRate[2], then construct vector as
      // parallel to Cmd->PriVec.cmd_axis
      AttitudeCmdProcessed &=
          fy_node_scanf(cmdNode, "/Rate %lf", &Cmd->AngRate.z) == 1;
      Cmd->AngRate.z *= D2R;
      Cmd->AngRate    = SxV(Cmd->AngRate.z, Cmd->PriVec.cmd_axis);

      Cmd->AttitudeCtrlActive = TRUE;
      ctrlNode                = fy_node_by_path_def(cmdNode, "/Controller");
      actNode                 = fy_node_by_path_def(cmdNode, "/Actuator");
   }
   else {
      AttitudeCmdProcessed = FALSE;
   }
   if (AttitudeCmdProcessed == TRUE && Cmd->AttitudeCtrlActive == TRUE) {
      if (GetController(DSM, ctrlNode, state) == FALSE) {
         fprintf(stderr,
                 "For %s command %s, could not find Controller alias %s or "
                 "invalid format. Exiting...\n",
                 subType, cmdName,
                 fy_anchor_get_text(fy_node_get_anchor(ctrlNode), NULL));
         exit(EXIT_FAILURE);
      }

      if (GetActuators(AC, DSM, actNode, state) == FALSE) {
         fprintf(stderr,
                 "For %s command %s, could not find Actuator alias %s or "
                 "invalid format. Exiting...\n",
                 subType, cmdName,
                 fy_anchor_get_text(fy_node_get_anchor(actNode), NULL));
         exit(EXIT_FAILURE);
      }
   }
   return (AttitudeCmdProcessed);
}
//-------------------------------- ACTUATOR CMD --------------------------------
long GetActuatorCmd(struct AcType *const AC, struct DSMType *const DSM,
                    struct fy_node *actCmdNode)
{
   struct fy_node *iterNode = NULL, *actSeqNode = NULL;
   long ActuatorCmdProcessed = FALSE;
   long i                    = 0;

   struct DSMCmdType *Cmd = &DSM->Cmd;

   struct fy_node *cmdNode = fy_node_by_path_def(actCmdNode, "/Command Data");

   if (cmdNode == NULL) {
      fprintf(stderr,
              "Could not find Command Data for Actuator command. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   const char *cmdName =
       fy_node_get_scalar0(fy_node_by_path_def(cmdNode, "/Description"));

   actSeqNode      = fy_node_by_path_def(cmdNode, "/Actuators");
   Cmd->ActNumCmds = fy_node_sequence_item_count(actSeqNode);
   iterNode        = NULL;
   WHILE_FY_ITER(actSeqNode, iterNode)
   {
      char type[FIELDWIDTH + 1] = {};
      if (fy_node_scanf(iterNode, "/Type %" STR(FIELDWIDTH) "s", type) == 1) {
         if (!strcmp(type, "WHL"))
            Cmd->ActTypes[i] = ACT_WHL;
         else if (!strcmp(type, "THR"))
            Cmd->ActTypes[i] = ACT_THR;
         else if (!strcmp(type, "MTB"))
            Cmd->ActTypes[i] = ACT_MTB;
         else if (!strcmp(type, "IDEALFRC"))
            Cmd->ActTypes[i] = ACT_IDEALFRC;
         else if (!strcmp(type, "IDEALTRQ"))
            Cmd->ActTypes[i] = ACT_IDEALTRQ;
         else {
            fprintf(stderr,
                    "Actuator Command index %s has improper actuator type %s. "
                    "Exiting...",
                    cmdName, type);
            exit(EXIT_FAILURE);
         }
         switch (Cmd->ActTypes[i]) {
            case ACT_WHL:
            case ACT_THR:
            case ACT_MTB:
               if (fy_node_scanf(iterNode, "/Index %d /Duty Cycle %lf",
                                 &Cmd->ActInds[i], &Cmd->ActDuties[i]) != 2) {
                  fprintf(stderr,
                          "Actuator Command index %s for non-ideal actuator is "
                          "impropertly formatted. Exiting...",
                          cmdName);
                  exit(EXIT_FAILURE);
               }
               break;
            case ACT_IDEALFRC:
            case ACT_IDEALTRQ: {
               long isgood  = fy_node_scanf(iterNode, "/Action %lf /Frame %19s",
                                            &Cmd->ActDuties[i],
                                            Cmd->ActIdealFrame[i]) == 2;
               isgood      &= assignYAMLToDoubleArray(
                                  3, fy_node_by_path_def(iterNode, "/Direction"),
                                  Cmd->ActIdealDirs[i].v) == 3;
               if (!isgood) {
                  fprintf(stderr,
                          "Actuator Command index %s for ideal actuator is "
                          "impropertly formatted. Exiting...",
                          cmdName);
                  exit(EXIT_FAILURE);
               }
               Cmd->ActIdealDirs[i] = UNITV(Cmd->ActIdealDirs[i]).v;
            } break;
            default:
               break;
         }
      }
      else {
         fprintf(
             stderr,
             "Actuator Command index %s is impropertly formatted. Exiting...",
             cmdName);
         exit(EXIT_FAILURE);
      }
      if (Cmd->ActTypes[i] == ACT_WHL && Cmd->ActInds[i] > AC->Nwhl) {
         fprintf(
             stderr,
             "SC[%ld] only has %ld wheels, but an actuator command was sent "
             "to wheel %d. Exiting...\n",
             AC->ID, AC->Nwhl, Cmd->ActInds[i]);
         exit(EXIT_FAILURE);
      }
      if (Cmd->ActTypes[i] == ACT_THR && Cmd->ActInds[i] > AC->Nthr) {
         fprintf(stderr,
                 "SC[%ld] only has %ld thrusters, but an actuator command was "
                 "sent to thruster %d. Exiting...\n",
                 AC->ID, AC->Nthr, Cmd->ActInds[i]);
         exit(EXIT_FAILURE);
      }
      if (Cmd->ActTypes[i] == ACT_MTB && Cmd->ActInds[i] > AC->Nmtb) {
         fprintf(stderr,
                 "SC[%ld] only has %ld MTBs, but an actuator command was sent "
                 "to MTB %d. Exiting...\n",
                 AC->ID, AC->Nmtb, Cmd->ActInds[i]);
         exit(EXIT_FAILURE);
      }
      i++;
   }
   if (i == Cmd->ActNumCmds)
      ActuatorCmdProcessed = TRUE;

   return (ActuatorCmdProcessed);
}
//--------------------------------- STATE NAMES --------------------------------
enum States GetStateValue(const char *string)
{
   if (!strcmp(string, "Attitude"))
      return ATTITUDE_STATE;
   else if (!strcmp(string, "Time"))
      return TIME_STATE;
   else if (!strcmp(string, "RotMat"))
      return ROTMAT_STATE;
   else if (!strcmp(string, "Quat"))
      return QUAT_STATE;
   else if (!strncmp(string, "Pos", 3))
      return POS_STATE;
   else if (!strncmp(string, "Vel", 3))
      return VEL_STATE;
   else if (!strcmp(string, "Omega"))
      return OMEGA_STATE;
   else
      return NULL_STATE;
}

//-------------------------------- SENSOR NAMES --------------------------------
enum SensorType GetSensorValue(const char *string)
{
   if (!strcmp(string, "STARTRACK"))
      return STARTRACK_SENSOR;
   else if (!strcmp(string, "GPS"))
      return GPS_SENSOR;
   else if (!strcmp(string, "FSS"))
      return FSS_SENSOR;
   else if (!strcmp(string, "CSS"))
      return CSS_SENSOR;
   else if (!strcmp(string, "GYRO"))
      return GYRO_SENSOR;
   else if (!strcmp(string, "MAG"))
      return MAG_SENSOR;
   else if (!strcmp(string, "ACCEL"))
      return ACCEL_SENSOR;
   else
      return NULL_SENSOR;
}

//------------------------------- NAVIGATION DATA ------------------------------
void ConfigureMeas(struct DSMMeasType *meas, enum SensorType sensor)
{
   switch (sensor) {
      case GPS_SENSOR:
         meas->dim             = 6;
         meas->errDim          = 6;
         meas->noiseDim        = 6;
         meas->measJacobianFun = &gpsJacobianFun;
         meas->measFun         = &gpsFun;
         break;
      case STARTRACK_SENSOR:
         meas->dim             = 4;
         meas->errDim          = 3;
         meas->noiseDim        = 3;
         meas->measJacobianFun = &startrackJacobianFun;
         meas->measFun         = &startrackFun;
         break;
      case FSS_SENSOR:
         meas->dim             = 2;
         meas->errDim          = 2;
         meas->noiseDim        = 2;
         meas->measJacobianFun = &fssJacobianFun;
         meas->measFun         = &fssFun;
         break;
      case CSS_SENSOR:
         meas->dim             = 1;
         meas->errDim          = 1;
         meas->noiseDim        = 1;
         meas->measJacobianFun = &cssJacobianFun;
         meas->measFun         = &cssFun;
         break;
      case GYRO_SENSOR:
         meas->dim             = 1;
         meas->errDim          = 1;
         meas->noiseDim        = 1;
         meas->measJacobianFun = &gyroJacobianFun;
         meas->measFun         = &gyroFun;
         break;
      case MAG_SENSOR:
         meas->dim             = 1;
         meas->errDim          = 1;
         meas->noiseDim        = 1;
         meas->measJacobianFun = &magJacobianFun;
         meas->measFun         = &magFun;
         break;
      case ACCEL_SENSOR:
         meas->dim             = 1;
         meas->errDim          = 1;
         meas->noiseDim        = 1;
         meas->measJacobianFun = &accelJacobianFun;
         meas->measFun         = &accelFun;
         break;
      default:
         break;
   }
   meas->type = sensor;
   meas->R    = calloc(meas->noiseDim, sizeof(double));
   meas->N    = CreateMatrix(meas->errDim, meas->noiseDim);
   // TODO: might move this out later
   for (int i = 0; i < MIN(meas->errDim, meas->noiseDim); i++)
      meas->N[i][i] = 1.0;
}

long ConfigureNavigationSensors(struct AcType *const AC,
                                struct DSMNavType *const Nav,
                                struct fy_node *senSetNode)
{
   struct fy_node *iterNode = NULL;
   long DataProcessed = FALSE, numSensors[FIN_SENSOR + 1] = {0};
   long i, j;

   FOR_SENSORS(sensor)
   {
      long nSensor;
      Nav->sensorActive[sensor] = FALSE;
      switch (sensor) {
         case STARTRACK_SENSOR:
            nSensor = AC->Nst;
            break;
         case GPS_SENSOR:
            nSensor = AC->Ngps;
            break;
         case FSS_SENSOR:
            nSensor = AC->Nfss;
            break;
         case CSS_SENSOR:
            nSensor = AC->Ncss;
            break;
         case GYRO_SENSOR:
            nSensor = AC->Ngyro;
            break;
         case MAG_SENSOR:
            nSensor = AC->Nmag;
            break;
         case ACCEL_SENSOR:
            nSensor = AC->Nacc;
            break;
         default:
            nSensor = 0;
            break;
      }
      if (Nav->measTypes[sensor] != NULL) {
         for (i = 0; i < nSensor; i++) {
            free(Nav->measTypes[sensor][i].R);
            free(Nav->innovations[i]);
            DestroyMatrix(Nav->measTypes[sensor][i].N);
         }
         free(Nav->sensorActive[sensor]);
         free(Nav->measTypes[sensor]);
         free(Nav->innovations[sensor]);
      }
      Nav->nSensor[sensor]      = nSensor;
      Nav->sensorActive[sensor] = calloc(nSensor, sizeof(int));
      for (i = 0; i < nSensor; i++)
         Nav->sensorActive[sensor][i] = FALSE;
      if (nSensor > 0) {
         Nav->measTypes[sensor]   = calloc(nSensor, sizeof(struct DSMMeasType));
         Nav->innovations[sensor] = calloc(nSensor, sizeof(double *));
         for (i = 0; i < nSensor; i++)
            Nav->innovations[sensor][i] = NULL;
      }
      else {
         Nav->measTypes[sensor]   = NULL;
         Nav->innovations[sensor] = NULL;
      }
   }

   char sensorSetName[1024] = {0};
   fy_node_scanf(senSetNode, "/Description %1023s", sensorSetName);
   WHILE_FY_ITER(fy_node_by_path_def(senSetNode, "/Sensors"), iterNode)
   {
      DataProcessed = TRUE;
      char sensorType[FIELDWIDTH + 1];
      long sensorNum;
      fy_node_scanf(iterNode,
                    "/Type %" STR(FIELDWIDTH) "s "
                                              "/Sensor Index %ld",
                    sensorType, &sensorNum);
      const enum SensorType sensor = GetSensorValue(sensorType);
      struct DSMMeasType *meas     = NULL;
      char sensorName[1024]        = {0};
      fy_node_scanf(iterNode, "/Description %1023s", sensorName);
      long maxSensors = 0;
      // the strcpys are here just for error reporting later
      switch (sensor) {
         case GPS_SENSOR:
            maxSensors = AC->Ngps;
            strcpy(sensorType, "GPS");
            break;
         case STARTRACK_SENSOR:
            maxSensors = AC->Nst;
            strcpy(sensorType, "Startracker");
            break;
         case FSS_SENSOR:
            maxSensors = AC->Nfss;
            strcpy(sensorType, "Fine Sun Sensor");
            break;
         case CSS_SENSOR:
            maxSensors = AC->Ncss;
            strcpy(sensorType, "Coarse Sun Sensor");
            break;
         case GYRO_SENSOR:
            maxSensors = AC->Ngyro;
            strcpy(sensorType, "Gyro");
            break;
         case MAG_SENSOR:
            maxSensors = AC->Nmag;
            strcpy(sensorType, "Magnetometer");
            break;
         case ACCEL_SENSOR:
            maxSensors = AC->Nacc;
            strcpy(sensorType, "Accelerometer");
            break;
         default:
            break;
      }
      if (sensorNum >= maxSensors) {
         printf("Sensor Set %s has requested more %ss than spacecraft SC_[%ld] "
                "has. Exiting...\n",
                sensorSetName, sensorType, AC->ID);
         exit(EXIT_FAILURE);
      }
      meas = &Nav->measTypes[sensor][sensorNum];
      ConfigureMeas(meas, sensor);
      long isGood;
      switch (sensor) {
         case STARTRACK_SENSOR: {
            double tmp[3] = {0.0};
            isGood        = assignYAMLToDoubleArray(
                                3, fy_node_by_path_def(iterNode, "/Sensor Noise"),
                                tmp) == 3;
            for (j = 0; j < meas->noiseDim; j++) {
               long const ind = (AC->ST[sensorNum].BoreAxis + j) % 3;
               meas->R[ind]   = tmp[j] * D2R / 3600.0;
            }
         } break;
         case GPS_SENSOR: {
            double tmp[2] = {0.0};
            isGood        = assignYAMLToDoubleArray(
                                2, fy_node_by_path_def(iterNode, "/Sensor Noise"),
                                tmp) == 2;
            for (j = 0; j < meas->noiseDim; j++) {
               if (j < 3)
                  meas->R[j] = tmp[0];
               else
                  meas->R[j] = tmp[1];
            }
         } break;
         case FSS_SENSOR: {
            isGood = assignYAMLToDoubleArray(
                         1, fy_node_by_path_def(iterNode, "/Sensor Noise"),
                         meas->R) == 1;
            for (j = meas->noiseDim - 1; j >= 0; j--)
               meas->R[j] = meas->R[0] * D2R;
         } break;
         case CSS_SENSOR:
         case GYRO_SENSOR:
         case MAG_SENSOR:
         case ACCEL_SENSOR: {
            isGood = assignYAMLToDoubleArray(
                         1, fy_node_by_path_def(iterNode, "/Sensor Noise"),
                         meas->R) == 1;
         } break;
         default:
            printf("%s in %s is of invalid sensor type %s. Exiting..\n",
                   sensorName, sensorSetName, sensorType);
            exit(EXIT_FAILURE);
            break;
      }
      isGood &= fy_node_scanf(iterNode,
                              "/Underweighting Factor %lf "
                              "/Residual Editing Gate %lf",
                              &meas->underWeighting, &meas->probGate) == 2;
      if (!isGood) {
         printf("%s is of invalid format for sensor type %s. Exiting...\n",
                sensorName, sensorType);
         exit(EXIT_FAILURE);
      }

      meas->nextMeas   = NULL;
      meas->data       = NULL;
      meas->time       = 0.0;
      meas->ccsds_time = (CCSDSTime){.coarse = 0, .fine = 0};
      meas->sensorNum  = sensorNum;
      meas->type       = sensor;
      numSensors[sensor]++;
      Nav->innovations[sensor][sensorNum] =
          calloc(meas->errDim, sizeof(double));
      Nav->sensorActive[sensor][sensorNum] = TRUE;
   }

   return (DataProcessed);
}

//------------------------------- NAVIGATION DATA ------------------------------
long GetNavigationData(struct DSMNavType *const Nav, struct fy_node *datNode,
                       enum matType type)
{
   long DataProcessed = FALSE, (*inds)[] = NULL, (*sizes)[] = NULL;
   double *dataDest;
   long dataDim = 0;
   long i, maxI, startInd;

   switch (type) {
      case Q_DAT:
         dataDim = Nav->navDim;
         inds    = &Nav->navInd;
         sizes   = &Nav->navSize;
         break;
      case P0_DAT:
         dataDim = Nav->navDim;
         inds    = &Nav->navInd;
         sizes   = &Nav->navSize;
         break;
      case IC_DAT:
         dataDim = Nav->stateDim;
         inds    = &Nav->stateInd;
         sizes   = &Nav->stateSize;
         break;
   }
   dataDest = calloc(dataDim, sizeof(double));

   // Ensure default Rotation Matrix and Quaternion are valid
   if (type == IC_DAT) {
      if (Nav->stateActive[ROTMAT_STATE] == TRUE) {
         dataDest[(*inds)[ROTMAT_STATE] + 0] = 1.0;
         dataDest[(*inds)[ROTMAT_STATE] + 4] = 1.0;
         dataDest[(*inds)[ROTMAT_STATE] + 8] = 1.0;
      }
      if (Nav->stateActive[QUAT_STATE] == TRUE) {
         dataDest[(*inds)[QUAT_STATE] + 3] = 1.0;
      }
   }

   const char stateNames[4][20] = {"/Attitude", "/Position", "/Velocity",
                                   "/Omega"};

   for (int k = 0; k < 4; k++) {
      struct fy_node *tmpNode = fy_node_by_path_def(datNode, stateNames[k]);
      if (tmpNode != NULL) {
         // You can do neat things with null terminated strings
         enum States state = GetStateValue(&stateNames[k][1]);
         if (state != NULL_STATE) {
            if (state == ATTITUDE_STATE) {
               if (Nav->stateActive[ROTMAT_STATE] == TRUE)
                  state = ROTMAT_STATE;
               else
                  state = QUAT_STATE;
            }
            // Nesting the ifs so default initial values can be used for ICs and
            // sqrQ
            if (Nav->stateActive[state] == TRUE) {
               maxI     = (*sizes)[state];
               startInd = (*inds)[state];
               if (type == IC_DAT &&
                   (state == ROTMAT_STATE || state == QUAT_STATE)) {
                  double ang[3] = {0.0};
                  long SEQ;
                  getYAMLEulerAngles(tmpNode, ang, &SEQ);
                  Nav->CRB = A2C(SEQ, ang[0], ang[1], ang[2]);
               }
               else
                  assignYAMLToDoubleArray(maxI, tmpNode, &dataDest[startInd]);
            }
         }
      }
   }

   switch (type) {
      case Q_DAT:
         for (i = 0; i < Nav->navDim; i++)
            Nav->sqrQ[i] = fabs(dataDest[i]);
         DataProcessed = TRUE;
         break;
      case P0_DAT:
         for (i = 0; i < Nav->navDim; i++) {
            if (dataDest[i] < 0.0) {
               printf("The initial estimation error covariance matrix in "
                      "navigation data %s is not positive definite. Ensure "
                      "that all states are supplied. Exiting...\n",
                      fy_node_get_parent_address(datNode));
               exit(EXIT_FAILURE);
            }
            Nav->S[i][i] = fabs(dataDest[i]);
         }
         DataProcessed = TRUE;
         break;
      case IC_DAT:
         FOR_STATES(state)
         {
            if (Nav->stateActive[state] == TRUE) {
               startInd = Nav->stateInd[state];
               switch (state) {
                  case TIME_STATE: {
                     Nav->jd_tt_mjd_0 = JDFromDays(dataDest[startInd], TT_TIME,
                                                   GMAT_MJD_EPOCH);
                     Nav->jd_tt_mjd   = Nav->jd_tt_mjd_0;
                  } break;
                  case ROTMAT_STATE:
                  case QUAT_STATE: {
                     mat3x3_t tmpM = MT(Nav->CRB);
                     Nav->qbr      = C2Q(tmpM);
                  } break;
                  case POS_STATE:
                     CopyVG(Nav->PosR.v, &dataDest[startInd], 3);
                     break;
                  case VEL_STATE:
                     CopyVG(Nav->VelR.v, &dataDest[startInd], 3);
                     break;
                  case OMEGA_STATE:
                     CopyVG(Nav->wbr.v, &dataDest[startInd], 3);
                     break;
                  default:
                     break;
               }
            }
         }
         DataProcessed = TRUE;
         break;
   }
   free(dataDest);
   return (DataProcessed);
}

//------------------------------- NAVIGATION CMD -------------------------------
long GetNavigationCmd(struct AcType *const AC, struct DSMType *const DSM,
                      struct fy_node *navCmdNode)
{
   char navType[FIELDWIDTH + 1] = {}, batchingType[FIELDWIDTH + 1] = {},
                             refOri[FIELDWIDTH + 1] = {}, refFrame = 0;
   long NavigationCmdProcessed = FALSE;
   long i, j;
   struct fy_node *qNode = NULL, *pNode = NULL, *x0Node = NULL,
                  *senSetNode = NULL, *statesNode = NULL;

   struct DSMNavType *Nav = &DSM->DsmNav;

   char subType[FIELDWIDTH + 1] = {};
   if (fy_node_scanf(navCmdNode, "/Subtype %" STR(FIELDWIDTH) "s", subType)) {
      if (!strcmp(subType, "NO_CHANGE")) {
         NavigationCmdProcessed = TRUE;
      }
      else if (!strcmp(subType, "PASSIVE_NAV")) {
         Nav->NavigationActive  = FALSE;
         NavigationCmdProcessed = TRUE;
      }
      return (NavigationCmdProcessed);
   }

   struct fy_node *cmdNode = fy_node_by_path_def(navCmdNode, "/Command Data");
   if (cmdNode == NULL) {
      printf("Could not find Command Data for Translation command of subtype "
             "%s. Exiting...\n",
             subType);
      exit(EXIT_FAILURE);
   }
   const char *cmdName =
       fy_node_get_scalar0(fy_node_by_path_def(cmdNode, "/Description"));

   Nav->NavigationActive = TRUE;

   NavigationCmdProcessed =
       fy_node_scanf(
           cmdNode,
           "/Type %" STR(FIELDWIDTH) "s "
                                     "/Batching %" STR(
                                         FIELDWIDTH) "s "
                                                     "/Frame %c "
                                                     "/Reference Origin %" STR(
                                                         FIELDWIDTH) "s",
           navType, batchingType, &refFrame, refOri) == 4;

   qNode                   = fy_node_by_path_def(cmdNode, "/Data/Q");
   pNode                   = fy_node_by_path_def(cmdNode, "/Data/P");
   x0Node                  = fy_node_by_path_def(cmdNode, "/Data/x0");
   senSetNode              = fy_node_by_path_def(cmdNode, "/Sensor Set");
   statesNode              = fy_node_by_path_def(cmdNode, "/States");
   NavigationCmdProcessed &= qNode != NULL && pNode != NULL && x0Node != NULL &&
                             senSetNode != NULL && statesNode != NULL;

   if (NavigationCmdProcessed == TRUE) {
      Nav->DT     = DSM->DT;
      Nav->DT_RAT = double2rational(Nav->DT);
      // round to nearest ccsds step
      Nav->subStepSteps = DTSIM * CCSDS_FINE_MAX + 0.5;
      Nav->subStepSize  = DTSIM;
      Nav->steps        = 0;
      const double t0   = gpsTime2J2000Sec(GpsRollover, GpsWeek, GpsSecond);

      Nav->jd_tt_mjd_0 = JDFromSeconds(t0, TT_TIME, J2000_EPOCH);
      Nav->jd_tt_mjd_0 =
          JDChangeSystemEpoch(TT_TIME, GMAT_MJD_EPOCH, Nav->jd_tt_mjd_0);
      Nav->jd_tt_mjd_0 = JDSubRationalSeconds(Nav->jd_tt_mjd_0, Nav->DT_RAT);
      Nav->jd_tt_mjd   = Nav->jd_tt_mjd_0;
      Nav->ccsds_time  = jd2ccsds(Nav->jd_tt_mjd_0);

      Nav->Date = JDToDate(Nav->jd_tt_mjd_0, TT_TIME);

      Nav->Init             = FALSE;
      Nav->reportConfigured = FALSE;
      if (Nav->sqrQ != NULL) {
         free(Nav->sqrQ);
         free(Nav->delta);
         DestroyMatrix(Nav->P);
         DestroyMatrix(Nav->STM);
         DestroyMatrix(Nav->STMStep);
         Nav->sqrQ     = NULL;
         Nav->M        = NULL;
         Nav->delta    = NULL;
         Nav->P        = NULL;
         Nav->S        = NULL;
         Nav->jacobian = NULL;
         Nav->STM      = NULL;
         Nav->STMStep  = NULL;
         Nav->NxN      = NULL;
         Nav->NxN2     = NULL;
         Nav->whlH     = NULL;
      }
      DestroyMeasList(&Nav->measList);

      if (!strcmp(navType, "RIEKF")) {
         Nav->type = RIEKF_NAV;
      }
      else if (!strcmp(navType, "LIEKF")) {
         Nav->type = LIEKF_NAV;
      }
      else if (!strcmp(navType, "MEKF")) {
         Nav->type = MEKF_NAV;
      }
      else {
         printf("%s is an invalid filter type for Navigation Command %s. "
                "Exiting...\n",
                navType, cmdName);
         exit(EXIT_FAILURE);
      }

      if (!strcmp(batchingType, "None")) {
         Nav->batching = NONE_BATCH;
      }
      else if (!strcmp(batchingType, "Sensor")) {
         Nav->batching = SENSOR_BATCH;
      }
      else if (!strcmp(batchingType, "Time")) {
         Nav->batching = TIME_BATCH;
      }
      else {
         printf("%s is an invalid batching type for Navigation Command %s. "
                "Exiting...\n",
                batchingType, cmdName);
         exit(EXIT_FAILURE);
      }

      if (refFrame == 'N') {
         Nav->refFrame = FRAME_N;
         // } else if (refFrame == 'L') {
         //    Nav->refFrame = FRAME_L;
         // } else if (refFrame == 'B') {
         //    Nav->refFrame = FRAME_B;
      }
      else {
         printf("Frame %c is an invalid navigation reference frame for "
                "Navigation Command %s. Exiting...\n",
                refFrame, cmdName);
         exit(EXIT_FAILURE);
      }

      if (!strcmp(refOri, "OP")) {
         Nav->refOriType = ORI_OP;
         Nav->refOriBody = 0;
         Nav->refOriPtr  = DSM->refOrb;
         Nav->refBodyPtr = NULL;
      }
      else if (!strncmp(refOri, "SC", 2)) {
         sscanf(refOri, "SC[%ld].B[%ld]", &Nav->refOriType, &Nav->refOriBody);
         if (Nav->refOriType >= Nsc) {
            printf("This mission only has %ld spacecraft, but spacecraft %ld "
                   "was attempted to be set as the navigation reference frame. "
                   "Exiting...\n",
                   Nsc, Nav->refOriType);
            exit(EXIT_FAILURE);
         }
         if (Nav->refOriBody >= SC[Nav->refOriType].Nb) {
            printf("Spacecraft %ld only has %ld bodies, but the navigation "
                   "reference frame was attempted to be set as body %ld. "
                   "Exiting...\n",
                   Nav->refOriType, SC[Nav->refOriType].Nb, Nav->refOriBody);
            exit(EXIT_FAILURE);
         }
         // This is all to avoid calling SC[] directly in Nav
         {
            struct SCType *TrgS = &SC[Nav->refOriType];
            Nav->refOriPtr      = &TrgS->DSM.commState;
            Nav->refBodyPtr     = &TrgS->B[Nav->refOriBody];
         }
      }
      else {
         Nav->refOriType = ORI_WORLD;
         Nav->refOriBody = 0;
         long wID        = GetWorldID(refOri);
         Nav->refOriPtr  = &World[wID];
         Nav->refBodyPtr = NULL;
         // error check?
      }

      FOR_STATES(state)
      {
         Nav->stateActive[state] = FALSE;
      }

      struct fy_node *iterNode = NULL;
      WHILE_FY_ITER(statesNode, iterNode)
      {
         char p[FIELDWIDTH + 1] = {0};
         fy_node_scanf(iterNode, "/ %" STR(FIELDWIDTH) "s", p);
         const enum States state = GetStateValue(p);
         if (state == -1 || (state == ROTMAT_STATE && Nav->type == MEKF_NAV) ||
             (state == QUAT_STATE && Nav->type != MEKF_NAV)) {
            printf("%s is an invalid state to estimate for Navigation Command "
                   "%s of type %s. Exiting...\n",
                   p, cmdName, navType);
            exit(EXIT_FAILURE);
         }
         else {
            Nav->stateActive[state] = TRUE;
         }
      }

      if (Nav->stateActive[ROTMAT_STATE] && Nav->stateActive[QUAT_STATE]) {
         printf("Cannot filter the Rotation Matrix and the attitude Quaternion "
                "simultaneously. Exiting...\n");
         exit(EXIT_FAILURE);
      }

      FOR_STATES(state)
      {
         switch (state) {
            case TIME_STATE:
               Nav->stateSize[state] = 1;
               Nav->navSize[state]   = 1;
               break;
            case ROTMAT_STATE:
               Nav->stateSize[state] = 9;
               Nav->navSize[state]   = 3;
               break;
            case QUAT_STATE:
               Nav->stateSize[state] = 4;
               Nav->navSize[state]   = 3;
               break;
            case POS_STATE:
            case VEL_STATE:
            case OMEGA_STATE:
               Nav->stateSize[state] = 3;
               Nav->navSize[state]   = 3;
               break;
            default:
               fprintf(stderr,
                       "Invalid State in GetNavigationCmd. Misconfigured "
                       "INIT_STATE or FIN_STATE. Exiting...\n");
               exit(EXIT_FAILURE);
         }
      }
      Nav->whlH = calloc(AC->Nwhl, sizeof(double));

      long stateInd = 0;
      long navInd   = 0;
      FOR_STATES(state)
      {
         if (Nav->stateActive[state] == TRUE) {
            Nav->stateInd[state]  = stateInd;
            Nav->navInd[state]    = navInd;
            stateInd             += Nav->stateSize[state];
            navInd               += Nav->navSize[state];
         }
         else {
            // TODO: I need to figure out how to deal with this for varied
            // frames & origins
            Nav->stateInd[state] = -1;
            Nav->navInd[state]   = -1;
            switch (state) {
               case POS_STATE:
                  Nav->PosR = VEC3_ZERO;
                  // for (j = 0; j < 3; j++)
                  //    Nav->PosR[j] = S->PosR[j] + (S->PosN[j] - AC->PosN[j]);
                  break;
               case VEL_STATE:
                  Nav->VelR = VEC3_ZERO;
                  // for (j = 0; j < 3; j++)
                  //    Nav->VelR[j] = S->VelR[j] + (S->VelN[j] - AC->VelN[j]);
                  break;
               case OMEGA_STATE:
                  Nav->wbr = AC->wbn;
                  break;
               default:
                  break;
            }
         }
      }
      if (navInd == 0) {
         printf("Navigation Command is not filtering anything. Exiting...\n");
         exit(EXIT_FAILURE);
      }

      Nav->stateDim = stateInd;
      Nav->navDim   = navInd;
      if (!Nav->stateActive[ROTMAT_STATE] && !Nav->stateActive[QUAT_STATE])
         Nav->qbr = AC->qbn;

      // sqrQ and P0 diagonal elements from Inp_DSM.txt
      Nav->sqrQ  = calloc(Nav->navDim, sizeof(double));
      Nav->M     = CreateMatrix(Nav->navDim, Nav->navDim);
      Nav->P     = CreateMatrix(Nav->navDim, Nav->navDim);
      Nav->S     = CreateMatrix(Nav->navDim, Nav->navDim);
      Nav->delta = calloc(Nav->navDim, sizeof(double));

      Nav->jacobian = CreateMatrix(Nav->navDim, Nav->navDim);
      Nav->STM      = CreateMatrix(Nav->navDim, Nav->navDim);
      Nav->STMStep  = CreateMatrix(Nav->navDim, Nav->navDim);
      Nav->NxN      = CreateMatrix(Nav->navDim, Nav->navDim);
      Nav->NxN2     = CreateMatrix(Nav->navDim, Nav->navDim);
      for (i = 0; i < Nav->navDim; i++) {
         Nav->STM[i][i]     = 1.0;
         Nav->STMStep[i][i] = 1.0;
      }

      if (GetNavigationData(Nav, x0Node, IC_DAT) == FALSE) {
         printf("Navigation data is an invalid data set for the initial "
                "estimation states for Navigation Command %s. Exiting...\n",
                cmdName);
         exit(EXIT_FAILURE);
      }

      if (Nav->refOriType == ORI_WORLD && Nav->refFrame == FRAME_N) {
         Nav->PosR = VAddV_Elem(Nav->PosR, DSM->refOrb->PosN);
         Nav->VelR = VAddV_Elem(Nav->VelR, DSM->refOrb->VelN);
      }

      if (Nav->stateActive[ROTMAT_STATE] == TRUE) {
         // Simple test if given rot mat is a rot mat
         mat3x3_t testM;
         double test = 0.0;
         // if Nav->CRB is valid, testM should be identity
         testM = MTxM(Nav->CRB, Nav->CRB);
         // if Nav->CRB is valid, testM should now be zero matrix
         for (i = 0; i < 3; i++)
            testM.mat[i][i] -= 1.0;

         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               test += fabs(testM.mat[i][j]); // 1-norm of vec(testM)
         if (test >= EPS_DSM) {
            printf("The supplied initial rotation matrix for Navigation "
                   "Command %s is not a valid Rotation Matrix. "
                   "Exiting...\n",
                   cmdName);
            exit(EXIT_FAILURE);
         }
      }

      if (GetNavigationData(Nav, qNode, Q_DAT) == FALSE) {
         printf("Navigation data is an invalid data set for the process noise "
                "covariance matrix for Navigation command %s. Exiting...\n",
                cmdName);
         exit(EXIT_FAILURE);
      }
      if (GetNavigationData(Nav, pNode, P0_DAT) == FALSE) {
         printf(
             "Navigation data is an invalid data set for the inital estimation "
             "error covariance matrix for Navigation command index %s. "
             "Exiting...",
             cmdName);
         exit(EXIT_FAILURE);
      }

      // Transform P0 to correct error state expression
      double **linTForm;
      linTForm = GetStateLinTForm(Nav);
      MINVxMG(linTForm, Nav->S, Nav->NxN, Nav->navDim, Nav->navDim);
      MxMTG(Nav->NxN, Nav->NxN, Nav->NxN2, Nav->navDim, Nav->navDim,
            Nav->navDim);
      for (i = 0; i < Nav->navDim; i++)
         for (j = 0; j < Nav->navDim; j++)
            Nav->S[i][j] = 0.0;
      chol(Nav->NxN2, Nav->S, Nav->navDim);

      DestroyMatrix(linTForm);

      ConfigureNavigationSensors(AC, Nav, senSetNode);
      AssignNavFunctions(Nav, Nav->type);
   }

   return (NavigationCmdProcessed);
}
// the compare function for sorting command time array
static int compareCmdNodes(const void *a, const void *b)
{
   double timeA = 0.0, timeB = 0.0;
   fy_node_scanf(*((struct fy_node **)a), "/Time %lf", &timeA);
   fy_node_scanf(*((struct fy_node **)b), "/Time %lf", &timeB);
   if (timeA > timeB)
      return 1;
   else if (timeA < timeB)
      return -1;
   else
      return 0;
}
//------------------------ INTERPRETER (FIRST ITERATION) -----------------------
void DsmCmdInterpreterMrk1(struct DSMType *const DSM, struct fy_node *dsmCmds)
{
   struct fy_node *iterNode = NULL, *scCmdsNode = NULL;

   DSM->CmdCnt      = 0;
   DSM->CmdNum      = 0;
   DSM->CmdCnt      = 0;
   DSM->CmdNextTime = 0.0;
   if (DSM->CmdArray != NULL) {
      free(DSM->CmdArray);
      DSM->CmdArray = NULL;
   }
   WHILE_FY_ITER(dsmCmds, iterNode)
   {
      long scInd = 0;
      if (!fy_node_scanf(iterNode, "/SC %ld", &scInd)) {
         fprintf(
             stderr,
             "Improperly configured DSM Commands SC sequence. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      if (scInd == DSM->ID) {
         scCmdsNode = fy_node_by_path_def(iterNode, "/Command Sequence");
         if (scCmdsNode == NULL) {
            fprintf(stderr,
                    "Could not find Command Sequence for SC[%li]. Exiting...\n",
                    DSM->ID);
            exit(EXIT_FAILURE);
         }
         long i       = DSM->CmdCnt;
         DSM->CmdCnt += fy_node_sequence_item_count(scCmdsNode);
         if (DSM->CmdCnt != i) {
            DSM->CmdArray =
                realloc(DSM->CmdArray, DSM->CmdCnt * sizeof(struct fy_node *));
            struct fy_node *cmdIterNode = NULL;
            WHILE_FY_ITER(scCmdsNode, cmdIterNode)
            {
               DSM->CmdArray[i++] = cmdIterNode;
            }
            qsort(DSM->CmdArray, DSM->CmdCnt, sizeof(struct fy_node *),
                  &compareCmdNodes);
            fy_node_scanf(DSM->CmdArray[0], "/Time %lf", &DSM->CmdNextTime);
         }
      }
   }
}
//--------------------- INTERPRETER (SUBSEQUENT ITERATIONS) --------------------
void DsmCmdInterpreterMrk2(struct AcType *const AC, struct DSMType *const DSM)
{
   struct DSMCmdType *Cmd = &DSM->Cmd;
   struct fy_node *cmdsNode =
       fy_node_by_path_def(DSM->CmdArray[DSM->CmdNum], "/Commands");

   if (cmdsNode == NULL) {
      fprintf(stderr,
              "Could not find command for SC[%ld] at time %lf. "
              "How did this happen? Exiting...\n",
              DSM->ID, DSM->CmdNextTime);
      exit(EXIT_FAILURE);
   }

   struct fy_node *iterNode = NULL;
   WHILE_FY_ITER(cmdsNode, iterNode)
   {
      char typeToken[FIELDWIDTH + 1] = {}, subType[FIELDWIDTH + 1] = {};

      const char *searchTypeStr    = "/Type %" STR(FIELDWIDTH) "[^\n]";
      const char *searchSubtypeStr = "/Subtype %" STR(FIELDWIDTH) "[^\n]";
      fy_node_scanf(iterNode, searchTypeStr, typeToken);
      if (!strcmp(typeToken, "Translation")) {
         if (GetTranslationCmd(AC, DSM, iterNode, DSM->CmdNextTime) == FALSE) {
            fy_node_scanf(iterNode, searchSubtypeStr, subType);
            fprintf(stderr,
                    "Translation command of subtype %s cannot be found in "
                    "Inp_DSM.yaml. Exiting...\n",
                    subType);
            exit(EXIT_FAILURE);
         }
      }
      else if (!strcmp(typeToken, "Attitude")) {
         if (GetAttitudeCmd(AC, DSM, iterNode) == FALSE) {
            fy_node_scanf(iterNode, searchSubtypeStr, subType);
            fprintf(stderr,
                    "Attitude command of subtype %s cannot be found in "
                    "Inp_DSM.yaml. Exiting...\n",
                    subType);
            exit(EXIT_FAILURE);
         }
      }
      else if (!strcmp(typeToken, "Actuator")) {
         if (GetActuatorCmd(AC, DSM, iterNode) == FALSE) {
            fprintf(stderr, "Actuator command cannot be found in Inp_DSM.yaml. "
                            "Exiting...\n");
            exit(EXIT_FAILURE);
         }
      }
      else if (!strcmp(typeToken, "Navigation")) {
         if (GetNavigationCmd(AC, DSM, iterNode) == FALSE) {
            printf("Navigation command cannot be found in Inp_DSM.yaml. "
                   "Exiting...\n");
            exit(EXIT_FAILURE);
         }
      }
      else {
         fprintf(stderr, "%s is not a supported command type. Exiting...\n",
                 typeToken);
         exit(EXIT_FAILURE);
      }
   }
   // This sure is one of the if() statements of all time. I feel like it can
   // be reduced...
   if ((Cmd->TranslationCtrlActive && Cmd->AttitudeCtrlActive) &&
       ((!strcmp(Cmd->trn_actuator, "THR_3DOF") &&
         (!strcmp(Cmd->att_actuator, "THR_6DOF") ||
          (!strcmp(Cmd->dmp_actuator, "THR_6DOF") && Cmd->H_DumpActive))) ||
        (!strcmp(Cmd->trn_actuator, "THR_6DOF") &&
         (!strcmp(Cmd->att_actuator, "THR_3DOF") ||
          (!strcmp(Cmd->dmp_actuator, "THR_3DOF") && Cmd->H_DumpActive))) ||
        (!strcmp(Cmd->trn_actuator, "THR_3DOF") &&
         (!strcmp(Cmd->att_actuator, "THR_3DOF") ||
          (!strcmp(Cmd->dmp_actuator, "THR_3DOF"))) &&
         Cmd->H_DumpActive))) {
      fprintf(stderr,
              "If the Translation actuator is 6DOF Thruster and Attitude "
              "actuator is Thruster, then it must be 6DOF (and vice "
              "versa).\nAdditionally, if the translation actuator is 3DOF "
              "thruster, then Attitude cannot also be 3DOF. Exiting...\n");
      exit(EXIT_FAILURE);
   }
}
#undef FIELDWIDTH
//------------------------------------------------------------------------------
//                                SENSORS
//------------------------------------------------------------------------------
void DsmSensorModule(struct AcType *const AC, struct DSMType *const DSM)
{
   struct DSMNavType *const Nav = &DSM->DsmNav;
   struct DSMMeasListType measList;
   long haveFSSMeas = FALSE;

   InitMeasList(&measList);

   FOR_SENSORS(sensor)
   {
      struct DSMMeasListType *newMeasList = NULL;
      switch (sensor) {
         case GYRO_SENSOR:
            newMeasList = DSM_GyroProcessing(AC, DSM);
            break;
         case MAG_SENSOR: // maybe add a condition to not run if
                          // magnetorquers are active??
            newMeasList = DSM_MagnetometerProcessing(AC, DSM);
            break;
         case FSS_SENSOR:
            newMeasList = DSM_FssProcessing(AC, DSM);
            if (newMeasList != NULL && newMeasList->head != NULL)
               haveFSSMeas = TRUE;
            break;
         case CSS_SENSOR: // Fine sun sensors preempt coarse sun sensors
            if (haveFSSMeas == FALSE)
               newMeasList = DSM_CssProcessing(AC, DSM);
            break;
         case STARTRACK_SENSOR:
            newMeasList = DSM_StarTrackerProcessing(AC, DSM);
            break;
         case GPS_SENSOR:
            newMeasList = DSM_GpsProcessing(AC, DSM);
            break;
         case ACCEL_SENSOR:
            newMeasList = DSM_AccelProcessing(AC, DSM);
            break;
         default:
            printf("Invalid Sensor in INIT_SENSOR and FIN_SENSOR interval. "
                   "Exiting...\n");
            exit(EXIT_FAILURE);
            break;
      }
      if (newMeasList != NULL) {
         appendList(&measList, newMeasList);
         free(newMeasList);
         newMeasList = NULL;
      }
   }

   if (Nav->NavigationActive == TRUE && ((measList.head) != NULL)) {
      bubbleSort(&measList);
      appendList(&Nav->measList, &measList);
   }
}
//------------------------------------------------------------------------------
//                                ACTUATORS
//------------------------------------------------------------------------------
void ActuatorModule(struct AcType *const AC, struct DSMType *const DSM)
{

   long i;
   vec3_t unit_bvb;

   struct DSMCmdType *Cmd = &DSM->Cmd;

   // Zero out all actuators first, so that they will be set nonzero only if
   // desired
   if (AC->Nthr > 0) {
      for (i = 0; i < AC->Nthr; i++) {
         AC->Thr[i].PulseWidthFinTimeStamp = JD_ZERO;
         AC->Thr[i].PulseWidthFinTimeStamp.system =
             UTC_TIME; // flag for not set
         AC->Thr[i].PulseWidthCmd  = 0.0;
         AC->Thr[i].ThrustLevelCmd = 0.0;
      }
   }

   if (AC->Nwhl > 0)
      for (i = 0; i < AC->Nwhl; i++)
         AC->Whl[i].Tcmd = 0.0;
   if (AC->Nmtb > 0)
      for (i = 0; i < AC->Nmtb; i++)
         AC->MTB[i].Mcmd = 0.0;

   AC->IdealFrc = VEC3_ZERO;
   AC->IdealTrq = VEC3_ZERO;
   AC->Fcmd     = VEC3_ZERO;
   AC->Tcmd     = VEC3_ZERO;
   AC->Mcmd     = VEC3_ZERO;

   // Translation
   if (Cmd->TranslationCtrlActive == TRUE) {
      if ((!strcmp(Cmd->trn_actuator, "THR_3DOF") ||
           !strcmp(Cmd->trn_actuator, "THR_6DOF")) &&
          AC->Nthr > 0) {
         AC->Fcmd = DSM->FcmdB;
         ThrProcessingMinPower(AC);
      }
      else if (!strcmp(Cmd->trn_actuator, "Ideal"))
         AC->IdealFrc = DSM->FcmdB;
      else {
         // What is Cmd->trn_actuator if no valid Cmd->ActuatorMode?? Error????
      }
   }

   // Attitude
   if (Cmd->AttitudeCtrlActive == TRUE) {
      if ((!strcmp(Cmd->att_actuator, "THR_3DOF") ||
           !strcmp(Cmd->att_actuator, "THR_6DOF")) &&
          AC->Nthr > 0) {
         AC->Tcmd = DSM->Tcmd;
         // if THR_TRN, this does both force & torque since AC->Fcmd set
         ThrProcessingMinPower(AC);
      }
      else if (!strcmp(Cmd->att_actuator, "WHL") && AC->Nwhl > 0) {
         AC->Tcmd = DSM->Tcmd;
         DSM_WheelProcessing(AC);
      }
      else if (!strcmp(Cmd->att_actuator, "MTB") && AC->Nmtb > 0) {
         unit_bvb  = UNITV(AC->bvb).v;
         DSM->Mcmd = VxV(unit_bvb, DSM->Tcmd);
         AC->Mcmd  = SxV(1.0 / MAGV(AC->bvb), DSM->Mcmd);
         DSM_MtbProcessing(AC);
      }
      else if (!strcmp(Cmd->att_actuator, "Ideal"))
         AC->IdealTrq = DSM->Tcmd;

      else {
         // What is Cmd->att_actuator if no valid Cmd->AttActuatorMode??
         // Error????
      }
   }

   // TODO: move momentum dumping to its own type
   // Momentum Dumping
   if (Cmd->H_DumpActive == TRUE) {
      if ((Cmd->AttitudeCtrlActive == TRUE &&
           strcmp(Cmd->att_actuator, "WHL")) ||
          Cmd->AttitudeCtrlActive == FALSE) {
         fprintf(stderr,
                 "You many only enable momentum dumping when the attitude is "
                 "actively controlled with WHLs. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      if (DSM->DsmCtrl.H_DumpActive == TRUE &&
          !strcmp(Cmd->dmp_actuator, "MTB") && AC->Nmtb > 0) {
         unit_bvb  = UNITV(AC->bvb).v;
         DSM->Mcmd = VxV(unit_bvb, DSM->dTcmd);
         AC->Mcmd  = SxV(1.0 / MAGV(AC->bvb), DSM->Mcmd);
         DSM_MtbProcessing(AC);
      }
      else if (DSM->DsmCtrl.H_DumpActive == TRUE &&
               (!strcmp(Cmd->dmp_actuator, "THR_3DOF") ||
                !strcmp(Cmd->dmp_actuator, "THR_6DOF")) &&
               AC->Nthr > 0) {
         // maybe have thrusters just thrust at
         // min(thrustertorquemax,SCALE*AC->Whl[i].Tmax)??? this could run into
         // issues if Thruster is being used for translation
         AC->Tcmd = DSM->dTcmd;
         // if THR_TRN, this does both force & torque since AC->Fcmd set
         ThrProcessingMinPower(AC);
      }
      else if (DSM->DsmCtrl.H_DumpActive == TRUE &&
               !strcmp(Cmd->dmp_actuator, "Ideal"))
         AC->IdealTrq = DSM->dTcmd;
   }

   // Process ActuatorCmd
   // loops through stored Actuator commands
   // Do it last to override other commands
   for (i = 0; i < Cmd->ActNumCmds; i++) {
      switch (Cmd->ActTypes[i]) {
         case ACT_WHL:
         case ACT_THR:
         case ACT_MTB:
            break;
         case ACT_IDEALFRC:
            AC->IdealFrc = VEC3_ZERO;
            break;
         case ACT_IDEALTRQ:
            AC->IdealTrq = VEC3_ZERO;
            break;
         default:
            break;
      }
   }
   for (i = 0; i < Cmd->ActNumCmds; i++) {
      switch (Cmd->ActTypes[i]) {
         case ACT_WHL:
            AC->Whl[Cmd->ActInds[i]].Tcmd =
                AC->Whl[i].Tmax * Cmd->ActDuties[i] / 100.0;
            break;
         case ACT_THR: {
            AC->Thr[Cmd->ActInds[i]].PulseWidthCmd =
                Cmd->ActDuties[i] / 100.0 * AC->DT;
            AC->Thr[Cmd->ActInds[i]].PulseWidthFinTimeStamp =
                JDAddSeconds(JD_TT_MJD, AC->Thr[Cmd->ActInds[i]].PulseWidthCmd);
            AC->Thr[Cmd->ActInds[i]].ThrustLevelCmd = Cmd->ActDuties[i] / 100.0;
         } break;
         case ACT_MTB:
            AC->MTB[Cmd->ActInds[i]].Mcmd =
                AC->MTB[i].Mmax * Cmd->ActDuties[i] / 100.0;
            break;
         case ACT_IDEALFRC: {
            vec3_t act_frc = SxV(Cmd->ActDuties[i], Cmd->ActIdealDirs[i]);
            switch (Cmd->ActIdealFrame[i][0]) {
               case 'L':
               case 'l':
                  act_frc = MTxV(AC->CLN, act_frc);
                  [[fallthrough]];
               case 'N':
               case 'n':
                  act_frc = MxV(AC->CBN, act_frc);
                  break;
               default:
                  break;
            }
            AC->IdealFrc = VAddV_Elem(
                AC->IdealFrc, SxV(Cmd->ActDuties[i], Cmd->ActIdealDirs[i]));
         } break;
         case ACT_IDEALTRQ: {
            AC->IdealTrq = VAddV_Elem(
                AC->IdealTrq, SxV(Cmd->ActDuties[i], Cmd->ActIdealDirs[i]));
         } break;
         default:
            break;
      }
   }
}
//------------------------------------------------------------------------------
//                                GUIDANCE
//------------------------------------------------------------------------------
void FindDsmCmdVecN(struct DSMType *DSM, struct DSMCmdVecType *CV)
{
   /*Clone of FindCmdVecN()from 42fsw.c with new structure type */

   // TODO: find angular rate of command vector
   vec3_t RelPosB, vb, Rhat;
   vec3_t RelPosN, RelPosH, RelVelN, RelVelH;
   vec3_t pn, vn, ph, vh;
   magvec3_t uv;
   double CosPriMerAng, SinPriMerAng;
   double MaxToS, ToS;
   long It;

   struct OrbitType const *RefOrb = DSM->refOrb;
   struct DSMStateType *state     = &DSM->state;

   switch (CV->TrgType) {
      case TARGET_WORLD: {
         struct WorldType *TrgW = &World[CV->TrgWorld];
         CosPriMerAng           = cos(TrgW->PriMerAng);
         SinPriMerAng           = sin(TrgW->PriMerAng);
         pn.x = CV->W.x * CosPriMerAng - CV->W.y * SinPriMerAng;
         pn.y = CV->W.x * SinPriMerAng + CV->W.y * CosPriMerAng;
         pn.z = CV->W.z;
         vn.x = -CV->W.x * SinPriMerAng - CV->W.y * CosPriMerAng;
         vn.y = CV->W.x * CosPriMerAng - CV->W.y * SinPriMerAng;
         vn.z = 0.0;
         if (CV->TrgWorld == RefOrb->World) {
            RelPosN = VSubV_Elem(pn, state->PosN);
            RelVelN = VSubV_Elem(vn, state->VelN);
         }
         else {
            struct WorldType *W = &World[RefOrb->World];

            ph      = MTxV(TrgW->CNH, pn);
            vh      = MTxV(TrgW->CNH, vn);
            RelPosH = MTxV(W->CNH, state->PosN);
            RelVelH = MTxV(W->CNH, state->VelN);

            for (int i = 0; i < 3; i++) {
               RelPosH.v[i] =
                   (TrgW->PosH.v[i] - W->PosH.v[i]) + (ph.v[i] - RelPosH.v[i]);
               RelVelH.v[i] =
                   (TrgW->VelH.v[i] - W->VelH.v[i]) + (vh.v[i] - RelVelH.v[i]);
            }

            RelPosN = MxV(W->CNH, RelPosH);
            RelVelN = MxV(W->CNH, RelVelH);
         }
         CV->N  = UNITV(RelPosN).v;
         CV->wn = DSM_RelMotionToAngRate(RelPosN, RelVelN);
      } break;
      case TARGET_SC: {
         struct DSMStateType *TrgState = NULL;
         struct OrbitType *TrgOrb      = NULL;
         {
            // Limit the scope where SC is accessed
            struct SCType *TrgS    = &SC[CV->TrgSC];
            struct DSMType *TrgDSM = &TrgS->DSM;
            TrgOrb                 = TrgDSM->refOrb;
            TrgState               = &TrgDSM->commState;
         }
         if (TrgOrb == RefOrb) {
            RelPosN = VSubV_Elem(TrgState->PosR, state->PosR);
            RelVelN = VSubV_Elem(TrgState->VelR, state->VelR);
         }
         else if (TrgOrb->World == RefOrb->World) {
            RelPosN = VSubV_Elem(TrgState->PosN, state->PosN);
            RelVelN = VSubV_Elem(TrgState->VelN, state->VelN);
         }
         else {
            struct WorldType *TrgW = &World[TrgOrb->World];
            struct WorldType *W    = &World[RefOrb->World];

            RelPosH = MTxV(TrgW->CNH, TrgState->PosN);
            RelVelH = MTxV(TrgW->CNH, TrgState->VelN);
            ph      = MTxV(TrgW->CNH, state->PosN);
            vh      = MTxV(TrgW->CNH, state->VelN);

            for (int i = 0; i < 3; i++) {
               RelPosH.v[i] -= ph.v[i];
               RelVelH.v[i] -= vh.v[i];
               RelPosH.v[i] += (TrgW->PosH.v[i] - W->PosH.v[i]);
               RelVelH.v[i] += (TrgW->VelH.v[i] - W->VelH.v[i]);
            }

            RelPosN = MxV(W->CNH, RelPosH);
            RelVelN = MxV(W->CNH, RelVelH);
         }
         CV->N  = UNITV(RelPosN).v;
         CV->wn = DSM_RelMotionToAngRate(RelPosN, RelVelN);
      } break;
      case TARGET_BODY: {
         struct OrbitType *TrgOrb      = NULL;
         struct DSMStateType *TrgState = NULL;
         struct BodyType *TrgSB        = NULL;
         vec3_t pcmn;
         {
            // Limit the scope where SC is accessed
            struct SCType *TrgS    = &SC[CV->TrgSC];
            TrgSB                  = TrgS->B;
            struct DSMType *TrgDSM = &TrgS->DSM;
            TrgOrb                 = TrgDSM->refOrb;
            TrgState               = &TrgDSM->commState;
            // TODO: don't like accessing SCType::cm
            pcmn = QTxV(TrgState->qbn, TrgS->cm);
         }
         // TODO: make this better
         quat_t qBb, qbN;
         qBb = QxQT(TrgSB[0].qn, TrgSB[CV->TrgBody].qn);
         qbN = QTxQ(qBb, TrgState->qbn);

         pn = VSubV_Elem(VAddV_Elem(QTxV(qbN, CV->T), TrgSB[CV->TrgBody].pn),
                         pcmn);

         RelPosB = VSubV_Elem(CV->T, TrgSB[CV->TrgBody].cm);
         vb      = VxV(TrgSB[CV->TrgBody].wn, RelPosB);
         vn      = VAddV_Elem(QTxV(qbN, vb), TrgSB[CV->TrgBody].vn);

         if (TrgOrb == RefOrb) {
            for (int i = 0; i < 3; i++) {
               RelPosN.v[i] = TrgState->PosR.v[i] + pn.v[i] - state->PosR.v[i];
               RelVelN.v[i] = TrgState->VelR.v[i] + vn.v[i] - state->VelR.v[i];
            }
         }
         else if (TrgOrb->World == RefOrb->World) {
            for (int i = 0; i < 3; i++) {
               RelPosN.v[i] = TrgState->PosN.v[i] + pn.v[i] - state->PosN.v[i];
               RelVelN.v[i] = TrgState->VelN.v[i] + vn.v[i] - state->VelN.v[i];
            }
         }
         else {
            struct WorldType *TrgW = &World[TrgOrb->World];
            pn                     = VAddV_Elem(pn, TrgState->PosN);
            vn                     = VAddV_Elem(vn, TrgState->VelN);
            RelPosH                = MTxV(TrgW->CNH, pn);
            RelVelH                = MTxV(TrgW->CNH, vn);
            struct WorldType *W    = &World[RefOrb->World];
            ph                     = MTxV(W->CNH, state->PosN);
            vh                     = MTxV(W->CNH, state->VelN);
            for (int i = 0; i < 3; i++) {
               RelPosH.v[i] -= ph.v[i];
               RelVelH.v[i] -= vh.v[i];
               RelPosH.v[i] += (TrgW->PosH.v[i] - W->PosH.v[i]);
               RelVelH.v[i] += (TrgW->VelH.v[i] - W->VelH.v[i]);
            }
            RelPosN = MxV(W->CNH, RelPosH);
            RelVelN = MxV(W->CNH, RelVelH);
         }
         CV->N  = UNITV(RelPosN).v;
         CV->wn = DSM_RelMotionToAngRate(RelPosN, RelVelN);
      } break;
      case TARGET_VELOCITY:
         CV->N = state->VelN;
         uv    = UNITV(CV->N);
         CV->N = uv.v;
         break;
      case TARGET_MAGFIELD:
         CV->N = state->bvn;
         uv    = UNITV(CV->N);
         CV->N = uv.v;
         break;
      case TARGET_TDRS:
         CV->N  = VEC3_PZAXIS;
         CV->wn = VEC3_ZERO;

         MaxToS = -2.0; /* Bogus */
         Rhat   = UNITV(state->PosN).v;
         /* Aim at TDRS closest to Zenith */
         for (It = 0; It < 10; It++) {
            if (Tdrs[It].Exists) {
               RelPosN = VSubV_Elem(Tdrs[It].PosN, state->PosN);
               uv      = UNITV(RelPosN);
               RelPosN = uv.v;
               ToS     = VoV(RelPosN, Rhat);
               if (ToS > MaxToS) {
                  MaxToS = ToS;
                  CV->N  = RelPosN;
               }
            }
         }
         break;
      default:
         break;
   }
}
//------------------------------------------------------------------------------
void TranslationGuidance(struct DSMType *DSM, struct FormationType *F)
{
   struct DSMCmdType *Cmd = &DSM->Cmd;
   if (Cmd->TranslationCtrlActive == FALSE || Cmd->ManeuverMode != MAN_INACTIVE)
      return;

   long Isc_Ref, goodOriginFrame = FALSE;
   long frame_body, origin_body;
   struct DSMCtrlType *CTRL   = &DSM->DsmCtrl;
   struct DSMStateType *state = &DSM->state;

   // Convert Disp vec into N/R coords.
   switch (Cmd->RefFrame[0]) {
      case 'F': {
         vec3_t wfn;
         CTRL->CmdPosR = MTxV(F->CN, Cmd->Pos); // Convert F to R Inertial
         switch (F->FixedInFrame) {
            case 'L': {
               // L rotates wrt R
               wfn             = DSM->refOrb->wln;
               goodOriginFrame = TRUE;
            } break;
            case 'N': {
               // R does not rotate wrt R Inertial
               wfn             = VEC3_ZERO;
               goodOriginFrame = TRUE;
            } break;
            default: {
               fprintf(stderr,
                       "Invalid Formation fixed frame. How did this happen? "
                       "Exiting...\n");
               exit(EXIT_FAILURE);
            } break;
         }
         CTRL->CmdVelR = VxV(wfn, CTRL->CmdPosR);
      } break;
      case 'N': {
         CTRL->CmdPosR   = Cmd->Pos;  // Already in R Inertial
         CTRL->CmdVelR   = VEC3_ZERO; // R does not rotate wrt R Inertial
         goodOriginFrame = TRUE;
      } break;
      case 'L': {
         // Convert LVLH to R Inertial
         CTRL->CmdPosR   = MTxV(DSM->refOrb->CLN, Cmd->Pos);
         CTRL->CmdVelR   = VxV(DSM->refOrb->wln, CTRL->CmdPosR);
         goodOriginFrame = TRUE;
      } break;
      case 'E': {
         // Hailey's EH Code Begin ****************************************
         vec3_t cmd_pos_EH = VEC3_ZERO;
         vec3_t cmd_vel_EH = VEC3_ZERO;
         vec3_t wln        = VEC3_ZERO;

         double n = sqrt(DSM->refOrb->mu / pow(DSM->refOrb->SMA, 3));

         if (!strcmp(Cmd->TranslationType, "Position")) {
            cmd_pos_EH.x = -Cmd->Distance * cos(Cmd->Phase) / 2;
            cmd_pos_EH.y = Cmd->Distance * sin(Cmd->Phase);
            cmd_pos_EH.z = Cmd->Distance * sqrt(3) * cos(Cmd->Phase) / 2;

            cmd_vel_EH.x = Cmd->Distance * n * sin(Cmd->Phase) / 2;
            cmd_vel_EH.y = Cmd->Distance * n * cos(Cmd->Phase);
            cmd_vel_EH.z = -Cmd->Distance * n * sqrt(3) * sin(Cmd->Phase) / 2;
         }
         else if (!strcmp(Cmd->TranslationType, "Circumnavigation")) {
            /* "Development and Flight of a Stereoscopic Imager for Use in
             * Spacecraft Close Proximity Operations," Darling et. al. p. 497 */
            if (Cmd->ResetTimer == 1) {
               Cmd->InitTime   = state->Time;
               Cmd->ResetTimer = 0;
            }
            Cmd->CurrentTimer = state->Time - Cmd->InitTime;
            if (sscanf(Cmd->RefOrigin, "SC[%ld].B[%ld]", &Isc_Ref,
                       &frame_body) == 2) {
               if (Isc_Ref == DSM->ID) {
                  fprintf(
                      stderr,
                      "Spacecraft %ld called Euler Hill guidance law on itself."
                      "Exiting...\n",
                      Isc_Ref);
                  exit(EXIT_FAILURE);
               }
               /* Calculate coefficients */
               double tau_k = n * Cmd->CurrentTimer + Cmd->Phase;
               wln          = DSM->refOrb->wln;

               /* CW equations (note: vel. is incorrect in paper) */
               cmd_pos_EH.x = -Cmd->Distance * cos(tau_k) / 2;
               cmd_pos_EH.y = Cmd->Distance * sin(tau_k);
               cmd_pos_EH.z = Cmd->Distance * sqrt(3) * cos(tau_k) / 2;

               cmd_vel_EH.x = Cmd->Distance * n * sin(tau_k) / 2;
               cmd_vel_EH.y = Cmd->Distance * n * cos(tau_k);
               cmd_vel_EH.z = -Cmd->Distance * n * sqrt(3) * sin(tau_k) / 2;
            }
            else {
               fprintf(stderr, "Invalid Translational Control Reference Frame. "
                               "Exiting...\n");
               exit(EXIT_FAILURE);
            }
         }
         else if (!strcmp(Cmd->TranslationType, "Docking")) {
            /* "Fundamentals of Astrodynamics and Applications", Vallado p. 397,
             * 410 */
            if (sscanf(Cmd->RefOrigin, "SC[%ld].B[%ld]", &Isc_Ref,
                       &frame_body) == 2) {
               if (Isc_Ref == DSM->ID) {
                  fprintf(stderr,
                          "Spacecraft %ld called Euler Hill guidance law on "
                          "itself. "
                          "Exiting...\n",
                          Isc_Ref);
                  exit(EXIT_FAILURE);
               }

               if (Cmd->ResetTimer == 1) {
                  Cmd->InitTime   = state->Time;
                  Cmd->ResetTimer = 0;
                  /* R Interial -> LVLH */
                  Cmd->Pos = MxV(SC[Isc_Ref].CLN, state->PosR);

                  Cmd->PosRate.y =
                      ((6 * Cmd->Pos.x *
                            (n * Cmd->TimeDock - sin(n * Cmd->TimeDock)) -
                        Cmd->Pos.y) *
                           n * sin(n * Cmd->TimeDock) -
                       2 * n * Cmd->Pos.x * (4 - 3 * cos(n * Cmd->TimeDock)) *
                           (1 - cos(n * Cmd->TimeDock))) /
                      ((4 * sin(n * Cmd->TimeDock) - 3 * n * Cmd->TimeDock) *
                           sin(n * Cmd->TimeDock) +
                       4 * pow(1 - cos(n * Cmd->TimeDock), 2));
                  Cmd->PosRate.x =
                      -(n * Cmd->Pos.x * (4 - 3 * cos(n * Cmd->TimeDock)) +
                        2 * (1 - cos(n * Cmd->TimeDock)) * Cmd->PosRate.y) /
                      sin(n * Cmd->TimeDock);
                  Cmd->PosRate.z = -Cmd->Pos.z * n / tan(n * Cmd->TimeDock);
               }

               wln               = DSM->refOrb->wln;
               Cmd->CurrentTimer = state->Time - Cmd->InitTime;
               if (Cmd->CurrentTimer <= Cmd->TimeDock) {
                  /* Update Position */
                  cmd_pos_EH.x =
                      (Cmd->PosRate.x / n) * sin(n * Cmd->CurrentTimer) -
                      (3 * Cmd->Pos.x + 2 * Cmd->PosRate.y / n) *
                          cos(n * Cmd->CurrentTimer) +
                      4 * Cmd->Pos.x + 2 * Cmd->PosRate.y / n;
                  cmd_pos_EH.y =
                      (6 * Cmd->Pos.x + 4 * Cmd->PosRate.y / n) *
                          sin(n * Cmd->CurrentTimer) +
                      (2 * Cmd->PosRate.x / n) * cos(n * Cmd->CurrentTimer) -
                      (6 * n * Cmd->Pos.x + 3 * Cmd->PosRate.y) *
                          Cmd->CurrentTimer +
                      Cmd->Pos.y - 2 * Cmd->PosRate.x / n;
                  cmd_pos_EH.z =
                      Cmd->Pos.z * cos(n * Cmd->CurrentTimer) +
                      (Cmd->PosRate.z / n) * sin(n * Cmd->CurrentTimer);
                  /* Update Velocity */
                  cmd_vel_EH.x = Cmd->PosRate.x * cos(n * Cmd->CurrentTimer) +
                                 (3 * n * Cmd->Pos.x + 2 * Cmd->PosRate.y) *
                                     sin(n * Cmd->CurrentTimer);
                  cmd_vel_EH.y =
                      (6 * n * Cmd->Pos.x + 4 * Cmd->PosRate.y) *
                          cos(n * Cmd->CurrentTimer) -
                      (2 * Cmd->PosRate.x) * sin(n * Cmd->CurrentTimer) -
                      (6 * n * Cmd->Pos.x + 3 * Cmd->PosRate.y);
                  cmd_vel_EH.z =
                      (-Cmd->Pos.z * n) * sin(n * Cmd->CurrentTimer) +
                      Cmd->PosRate.z * cos(n * Cmd->CurrentTimer);
               }
               else { // arrived at docking location
                  cmd_pos_EH = VEC3_ZERO;
                  cmd_vel_EH = VEC3_ZERO;
               }
            }
            else {
               fprintf(stderr, "Invalid Translational Control Reference Frame. "
                               "Exiting...\n");
               exit(EXIT_FAILURE);
            }
         }
         /* LVLH -> R Inertial */
         CTRL->CmdPosR     = MTxV(DSM->refOrb->CLN, cmd_pos_EH);
         CTRL->CmdVelR     = VxV(wln, CTRL->CmdPosR);
         const vec3_t temp = MTxV(DSM->refOrb->CLN, cmd_vel_EH);
         CTRL->CmdVelR     = VAddV_Elem(CTRL->CmdVelR, temp);
      } break;
      default: {
         // Decode ref SC ID Number
         if (sscanf(Cmd->RefFrame, "SC[%ld].B[%ld]", &Isc_Ref, &frame_body) ==
             2) {
            if (Isc_Ref == DSM->ID) {
               fprintf(stderr,
                       "SC[%ld] is attempting to translate relative to its "
                       "own body frame. Exiting...\n",
                       DSM->ID);
               exit(EXIT_FAILURE);
            }
            // Specify disp from OP, in SC B frame directions, control to OP
            if (Isc_Ref >= Nsc) {
               fprintf(stderr,
                       "This mission only has %ld spacecraft, but spacecraft "
                       "%ld was attempted to be set as the reference frame. "
                       "Exiting...\n",
                       Nsc, Isc_Ref);
               exit(EXIT_FAILURE);
            }
            if (frame_body >= SC[Isc_Ref].Nb) {
               fprintf(
                   stderr,
                   "Spacecraft %ld only has %ld bodies, but the reference "
                   "frame was attempted to be set as body %ld. Exiting...\n",
                   Isc_Ref, SC[Isc_Ref].Nb, frame_body);
               exit(EXIT_FAILURE);
            }
            // TODO: don't use other sc truth
            quat_t qbn;
            vec3_t wbn;
            struct BodyType *TrgSB        = NULL;
            struct DSMStateType *TrgState = NULL;
            {
               // Limit scope where we need SCType
               struct SCType *TrgS    = &SC[Isc_Ref];
               TrgSB                  = TrgS->B;
               struct DSMType *TrgDSM = &TrgS->DSM;
               TrgState               = &TrgDSM->commState;
            }
            if (frame_body != 0) {
               // get relative orientation of body to B[0] then apply this to
               // AC.qbn
               vec3_t wBnb, wBnbAC;
               quat_t qbB = QxQT(TrgSB[frame_body].qn, TrgSB[0].qn);
               qbn        = QxQ(qbB, TrgState->qbn);

               // get angular velocity of body relative to B[0], then apply
               // this to AC.wbn; all in B[frame_body] frame
               wBnb   = QxV(qbB, TrgSB[0].wn);
               wBnbAC = QxV(qbB, TrgState->wbn);
               // TODO: double check what BodyType::wn actually is
               for (int i = 0; i < 3; i++)
                  wbn.v[i] =
                      wBnbAC.v[i] + (TrgSB[frame_body].wn.v[i] - wBnb.v[i]);
            }
            else {
               qbn = TrgState->qbn;
               wbn = TrgState->wbn;
            }
            // angular velocity of trgDSM wrt N expressed in N
            vec3_t wbnn;
            // Convert SC# B to R Inertial
            CTRL->CmdPosR   = QTxV(qbn, Cmd->Pos);
            wbnn            = QTxV(qbn, wbn); // SC rotates wrt R
            CTRL->CmdVelR   = VxV(wbnn, CTRL->CmdPosR);
            goodOriginFrame = TRUE;
         }
         else {
            fprintf(stderr, "Invalid Translational Control Reference Frame. "
                            "Exiting...\n");
            exit(EXIT_FAILURE);
         }
      } break;
   }

   if (!strcmp(Cmd->RefOrigin, "OP")) {
      // Specify disp from OP, in X frame directions, control to OP
      // Add pos of F frame origin in R frame
      CTRL->CmdPosR   = VAddV_Elem(CTRL->CmdPosR, F->PosR);
      goodOriginFrame = TRUE;
   }
   else if (!strncmp(Cmd->RefOrigin, "SC", 2)) {
      // Specify disp from SC, in X frame directions, control to SC
      // Add pos of SC in R frame
      sscanf(Cmd->RefOrigin, "SC[%ld].B[%ld]", &Isc_Ref,
             &origin_body); // Decode ref SC ID Number
      if (Isc_Ref >= Nsc) {
         fprintf(stderr,
                 "This mission only has %ld spacecraft, but spacecraft %ld was "
                 "attempted to be set as the reference origin. Exiting...\n",
                 Nsc, Isc_Ref);
         exit(EXIT_FAILURE);
      }
      if (origin_body >= SC[Isc_Ref].Nb) {
         fprintf(stderr,
                 "Spacecraft %ld only has %ld bodies, but the reference origin "
                 "was attempted to be set as body %ld. Exiting...\n",
                 Isc_Ref, SC[Isc_Ref].Nb, origin_body);
         exit(EXIT_FAILURE);
      }
      struct BodyType *TrgSB        = NULL;
      struct DSMStateType *TrgState = NULL;
      {
         // Limit scope where we need SCType
         struct SCType *TrgS    = &SC[Isc_Ref];
         TrgSB                  = TrgS->B;
         struct DSMType *TrgDSM = &TrgS->DSM;
         TrgState               = &TrgDSM->commState;
      }
      for (int i = 0; i < 3; i++) {
         CTRL->CmdPosR.v[i] += TrgState->PosR.v[i] + TrgSB[origin_body].pn.v[i];
         CTRL->CmdVelR.v[i] += TrgState->VelR.v[i] + TrgSB[origin_body].vn.v[i];
      }
      goodOriginFrame = TRUE;
   }
   else {
      goodOriginFrame = FALSE;
   }

   if (goodOriginFrame == FALSE) {
      fprintf(stderr,
              "Invalid Ref origin/frame combo %s/%s in Translation Command "
              "at %lf. Exiting...\n",
              Cmd->RefOrigin, Cmd->RefFrame, SimTime);
      exit(EXIT_FAILURE);
   }
   CTRL->CmdPosN = VAddV_Elem(CTRL->CmdPosR, state->PosN);
   CTRL->CmdVelN = VAddV_Elem(CTRL->CmdVelR, state->VelN);

   CTRL->trn_kp   = Cmd->trn_kp;
   CTRL->trn_kr   = Cmd->trn_kr;
   CTRL->trn_ki   = Cmd->trn_ki;
   CTRL->FrcB_max = Cmd->FrcB_max;
   CTRL->vel_max  = Cmd->vel_max;
}
//------------------------------------------------------------------------------
long getCmdVecs(struct DSMType *DSM, struct FormationType *F,
                struct DSMCmdVecType *vec, const char *attRefFrame,
                struct DSMStateType *state, vec3_t *cmdVecB, vec3_t *cmdVecN)
{
   switch (vec->TrgType) {
      case TARGET_SC:
      case TARGET_WORLD: {
         // to get PV->wn, PV->N (in F Frame)
         FindDsmCmdVecN(DSM, vec);
         // (Converting Cmd vec to body frame)
         *cmdVecB = QxV(state->qbn, vec->N);
      } break;
      case TARGET_VEC: {
         switch (attRefFrame[0]) {
            case 'N': {
               // (Converting Cmd vec to body frame)
               *cmdVecB = QxV(state->qbn, vec->cmd_vec);
            } break;
            case 'F': {
               // (Converting to Inertial frame)
               *cmdVecN = MTxV(F->CN, vec->cmd_vec);
               // (Converting to body frame)
               *cmdVecB = QxV(state->qbn, *cmdVecN);
            } break;
            case 'L': {
               // (Converting to LVLH to Inertial frame)
               *cmdVecN = MTxV(DSM->refOrb->CLN, vec->cmd_vec);
               // (Converting to body frame)
               *cmdVecB = QxV(state->qbn, *cmdVecN);
            } break;
            case 'M': {
               /* Magnetic field frame                               */
               /*    x: magnetic field line                          */
               /*    y: radial cross magnetic field                  */
               /*    z: completes  triad                             */
               mat3x3_t CbN;
               CbN.rows[0] = state->bvn;
               CbN.rows[0] = UNITV(CbN.rows[0]).v;
               CbN.rows[1] = VxV(state->PosN, CbN.rows[0]);
               CbN.rows[1] = UNITV(CbN.rows[1]).v;
               CbN.rows[2] = VxV(CbN.rows[0], CbN.rows[1]);
               CbN.rows[2] = UNITV(CbN.rows[2]).v;
               // (Converting from magnetic frame to Inertial frame)
               *cmdVecN = MTxV(CbN, vec->cmd_vec);
               // (Converting to body frame)
               *cmdVecB = QxV(state->qbn, *cmdVecN);
            } break;
            default: {
               long frame_body = 0;
               long Isc_Ref;
               // Decode ref SC ID Number
               if (sscanf(attRefFrame, "SC[%ld].B[%ld]", &Isc_Ref,
                          &frame_body) == 2) {
                  if (Isc_Ref == DSM->ID) {
                     fprintf(stderr,
                             "SC[%ld] is attempting to point relative "
                             "to its own body frame. Exiting...\n",
                             DSM->ID);
                     exit(EXIT_FAILURE);
                  }
                  if (Isc_Ref >= Nsc) {
                     fprintf(stderr,
                             "This mission only has %ld spacecraft, "
                             "but spacecraft %ld was attempted to be "
                             "set as the reference frame. Exiting...\n",
                             Nsc, Isc_Ref);
                     exit(EXIT_FAILURE);
                  }
                  if (frame_body >= SC[Isc_Ref].Nb) {
                     fprintf(stderr,
                             "Spacecraft %ld only has %ld bodies, but "
                             "the reference frame was attempted to be "
                             "set as body %ld. Exiting...\n",
                             Isc_Ref, SC[Isc_Ref].Nb, frame_body);
                     exit(EXIT_FAILURE);
                  }
                  // TODO: don't use other sc truth
                  quat_t qbn;
                  struct BodyType *TrgSB        = NULL;
                  struct DSMStateType *TrgState = NULL;
                  {
                     // Limit scope where we need SCType
                     struct SCType *TrgS    = &SC[Isc_Ref];
                     TrgSB                  = TrgS->B;
                     struct DSMType *TrgDSM = &TrgS->DSM;
                     TrgState               = &TrgDSM->commState;
                  }
                  if (frame_body != 0) {
                     // get relative orientation of body to B[0]
                     // then apply this to AC.qbn
                     quat_t qbB;
                     qbB = QxQT(TrgSB[frame_body].qn, TrgSB[0].qn);
                     qbn = QxQ(qbB, TrgState->qbn);
                  }
                  else
                     qbn = TrgState->qbn;

                  // rotation from trg Body to DSM body frame
                  quat_t qbbs;
                  qbbs     = QxQT(state->qbn, qbn);
                  *cmdVecB = QxV(qbbs, vec->cmd_vec);
               }
               else {
                  fprintf(stderr, "Invalid attitude reference frame for "
                                  "pointing vector. Exiting...\n");
                  exit(EXIT_FAILURE);
               }
            } break;
         }
      } break;
      default:
         return FALSE;
         break;
   }
   *cmdVecB = UNITV(*cmdVecB).v;
   *cmdVecN = QTxV(state->qbn, *cmdVecB);
   *cmdVecN = UNITV(*cmdVecN).v;
   return TRUE;
}
//------------------------------------------------------------------------------
void AttitudeGuidance(struct DSMType *DSM, struct FormationType *F)
{
   struct DSMCmdType *Cmd = &DSM->Cmd;
   if (Cmd->AttitudeCtrlActive == FALSE)
      return;

   long i, target_num;
   quat_t qfn, qrn, qfl, qrf;
   struct DSMCtrlType *CTRL   = &DSM->DsmCtrl;
   struct DSMStateType *state = &DSM->state;

   switch (Cmd->Method) {
      case (PARM_VECTORS): {
         vec3_t cmdVecB[2]             = {VEC3_ZERO};
         vec3_t cmdVecN[2]             = {VEC3_ZERO};
         struct DSMCmdVecType *vecs[2] = {&Cmd->PriVec, &Cmd->SecVec};
         char *attRefFrame[2] = {Cmd->PriAttRefFrame, Cmd->SecAttRefFrame};
         mat3x3_t C_tb, C_tn, dC;
         quat_t q_tb = QUAT_EYE, q_tn = QUAT_EYE, qbn_cmd;
         for (int k = 0; k < 2; k++) {
            if (!getCmdVecs(DSM, F, vecs[k], attRefFrame[k], state, &cmdVecB[k],
                            &cmdVecN[k])) {
               fprintf(stderr,
                       "Invalid Target type for %s vector. Exiting...\n",
                       k == 0 ? "Primary" : "Secondary");
               exit(EXIT_FAILURE);
            }
         }

         /*construct body to target DCM and Inertial to Target DCMS*/
         C_tb.rows[0] = vecs[0]->cmd_axis; // = PV->cmd_axis
         C_tn.rows[0] = cmdVecN[0];        // = PriCmdVec

         if (fabs(VoV(vecs[0]->cmd_axis, vecs[1]->cmd_axis) - 1.0) < EPS_DSM) {
            fprintf(stderr,
                    "PV Axis [%lf  %lf  %lf] in %s and SV Axis [%lf  %lf  %lf] "
                    "in %s are parallel, resulting in an infeasible attitude "
                    "command. Exiting...\n",
                    vecs[0]->cmd_axis.x, vecs[0]->cmd_axis.y,
                    vecs[0]->cmd_axis.z, Cmd->PriAttRefFrame,
                    vecs[1]->cmd_axis.x, vecs[1]->cmd_axis.y,
                    vecs[1]->cmd_axis.z, Cmd->SecAttRefFrame);
            exit(EXIT_FAILURE);
         }

         if (fabs(VoV(cmdVecB[0], cmdVecB[1]) - 1.0) < EPS_DSM) {
            char tgts[2][50] = {{0}};
            for (i = 0; i < 2; i++) {
               switch (vecs[i]->TrgType) {
                  case TARGET_SC: {
                     sprintf(tgts[i], "SC[%ld].B[%ld]", vecs[i]->TrgSC,
                             vecs[i]->TrgBody);
                  } break;
                  case TARGET_WORLD: {
                     sprintf(tgts[i],
                             "World %s, Position [%.3le, %.3le, %.3le]",
                             World[vecs[i]->TrgWorld].Name, vecs[i]->W.x,
                             vecs[i]->W.y, vecs[i]->W.z);
                  } break;
                  case TARGET_VEC: {
                     sprintf(tgts[i], "Vector [%.3le, %.3le, %.3le]",
                             vecs[i]->cmd_vec.x, vecs[i]->cmd_vec.y,
                             vecs[i]->cmd_vec.z);
                  } break;
                  default:
                     strcpy(tgts[i], "ERROR");
                     break;
               }
            }
            fprintf(stderr,
                    "PV Target, %s, and SV Target, %s, are parallel, resulting "
                    "in an infeasible attitude command. Exiting...\n",
                    tgts[0], tgts[1]);
            exit(EXIT_FAILURE);
         }

         C_tb.rows[2] = VxV(vecs[0]->cmd_axis, vecs[1]->cmd_axis);
         C_tn.rows[2] = VxV(cmdVecN[0], cmdVecN[1]);
         C_tb.rows[1] = VxV(C_tb.rows[2], C_tb.rows[0]);
         C_tn.rows[1] = VxV(C_tn.rows[2], C_tn.rows[0]);

         for (i = 0; i < 3; i++) {
            C_tb.rows[i] = UNITV(C_tb.rows[i]).v;
            C_tn.rows[i] = UNITV(C_tn.rows[i]).v;
         }
         q_tb = C2Q(C_tb);
         q_tn = C2Q(C_tn);

         /* Approximation of log map from SO(3) to so(3) to calculate Cmd->wrn*/
         dC       = MTxM(C_tn, Cmd->OldCRN);
         Cmd->wrn = logso3(dC);
         // get the short path rotation axis
         double mag = MAGV(Cmd->wrn);
         if (mag > __DBL_EPSILON__) {
            const magvec3_t wrnu = UNITV(Cmd->wrn);
            mag                  = WrapToPMPi(wrnu.m);
            if (mag > (TWOPI - mag))
               Cmd->wrn = SxV(TWOPI - mag, wrnu.v);
         }
         Cmd->wrn = SxV(1.0 / DSM->DT, Cmd->wrn);
         memcpy(Cmd->OldCRN.flat, C_tn.flat, sizeof(Cmd->OldCRN));

         /* Calculate Inertial to Body Quaternion */
         qbn_cmd  = UNITQ(QTxQ(q_tb, q_tn));
         Cmd->qbr = QxQT(state->qbn, qbn_cmd);
      } break;
      case (PARM_AXIS_SPIN): {
         vec3_t cmdVecB;
         vec3_t cmdVecN;

         if (!getCmdVecs(DSM, F, &Cmd->PriVec, Cmd->PriAttRefFrame, state,
                         &cmdVecB, &cmdVecN)) {
            fprintf(
                stderr,
                "Invalid Target type for Primary Spin vector. Exiting...\n");
            exit(EXIT_FAILURE);
         }

         vec3_t therr              = VSubV_Elem(cmdVecB, Cmd->PriVec.cmd_axis);
         const double therr_o_axis = VoV(Cmd->PriVec.cmd_axis, therr);
         therr = VAddV_Elem(therr, SxV(-therr_o_axis, Cmd->PriVec.cmd_axis));
         magvec3_t utherr       = UNITV(therr);
         therr                  = utherr.v;
         const double therr_mag = utherr.m;

         Cmd->qbr.qs      = cos(therr_mag / 2);
         const double tmp = sqrt(1.0 - Cmd->qbr.qs * Cmd->qbr.qs);
         Cmd->qbr.qv      = SxV(tmp, therr);
         Cmd->wrn         = QTxV(state->qbn, Cmd->AngRate);
      } break;
      case (PARM_UNITVECTOR): {
         fprintf(stderr,
                 "Feature for Singular Primary Unit Vector Pointing not "
                 "currently fully implemented. Exiting...\n");
         exit(EXIT_FAILURE);
      } break;
      case (PARM_QUATERNION): {
         switch (Cmd->AttRefFrame[0]) {
            case 'N': {
               Cmd->qrn = Cmd->q;
               Cmd->qbr = QxQT(state->qbn, Cmd->qrn);
               Cmd->wrn = VEC3_ZERO;
            } break;
            case 'F': {
               Cmd->qrf = Cmd->q;
               qfn      = C2Q(F->CN);
               qrn      = QxQ(Cmd->qrf, qfn);
               Cmd->qbr = QxQT(state->qbn, qrn);
               switch (F->FixedInFrame) {
                  case 'L': {
                     // F rotates wrt N
                     Cmd->wrn = DSM->refOrb->wln;
                  } break;
                  case 'N': {
                     // N does not rotate wrt N Inertial
                     Cmd->wrn = VEC3_ZERO;
                  } break;
                  default: {
                     fprintf(stderr,
                             "Invalid Formation fixed frame. How did this "
                             "happen? Exiting...\n");
                     exit(EXIT_FAILURE);
                  } break;
               }
            } break;
            case 'L': {
               Cmd->qrl = Cmd->q;
               qfl      = C2Q(F->CL);
               qrf      = QxQT(Cmd->qrl, qfl);
               qfn      = C2Q(F->CN);
               qrn      = QxQ(qrf, qfn);
               Cmd->qbr = QxQT(state->qbn, qrn);
               Cmd->wrn = DSM->refOrb->wln;
            } break;
            default: {
               long frame_body = 0;
               long Isc_Ref;
               // Decode ref SC ID Number
               if (sscanf(Cmd->AttRefFrame, "SC[%ld].B[%ld]", &Isc_Ref,
                          &frame_body) == 2) {
                  if (Isc_Ref == DSM->ID) {
                     fprintf(stderr,
                             "SC[%ld] is attempting to point relative to its "
                             "own body frame. Exiting...\n",
                             DSM->ID);
                     exit(EXIT_FAILURE);
                  }
                  if (Isc_Ref >= Nsc) {
                     fprintf(stderr,
                             "This mission only has %ld spacecraft, but "
                             "spacecraft %ld was attempted to be set as the "
                             "reference frame. Exiting...\n",
                             Nsc, Isc_Ref);
                     exit(EXIT_FAILURE);
                  }
                  if (frame_body >= SC[Isc_Ref].Nb) {
                     fprintf(stderr,
                             "Spacecraft %ld only has %ld bodies, but the "
                             "reference frame was attempted to be set as body "
                             "%ld. Exiting...\n",
                             Isc_Ref, SC[Isc_Ref].Nb, frame_body);
                     exit(EXIT_FAILURE);
                  }
                  // TODO: don't use other sc truth
                  quat_t qbn;
                  struct BodyType *TrgSB        = NULL;
                  struct DSMStateType *TrgState = NULL;
                  {
                     // Limit scope where we need SCType
                     struct SCType *TrgS    = &SC[Isc_Ref];
                     TrgSB                  = TrgS->B;
                     struct DSMType *TrgDSM = &TrgS->DSM;
                     TrgState               = &TrgDSM->commState;
                  }
                  if (frame_body != 0) {
                     // get relative orientation of body to B[0]
                     // then apply this to AC.qbn
                     quat_t qbB = QxQT(TrgSB[frame_body].qn, TrgSB[0].qn);
                     qbn        = QxQ(qbB, TrgState->qbn);
                  }
                  else
                     qbn = TrgState->qbn;

                  // rotation from trg Body to DSM body frame
                  quat_t qbbs = QxQT(state->qbn, qbn);
                  Cmd->qbr    = QxQT(qbbs, Cmd->q);
               }
               else {
                  fprintf(stderr,
                          "Invlaid attitude reference frame for quaternion. "
                          "Exiting...\n");
                  exit(EXIT_FAILURE);
               }
            } break;
         }
      } break;
      case (PARM_MIRROR): {
         long Isc_Ref;
         // Decode ref SC ID Number
         sscanf(Cmd->AttRefScID, "SC[%ld].B[%ld]", &Isc_Ref, &target_num);
         if (Isc_Ref >= Nsc) {
            fprintf(
                stderr,
                "This mission only has %ld spacecraft, but spacecraft %ld was "
                "attempted to be set as the spacecraft to mirror. Exiting...\n",
                Nsc, Isc_Ref);
            exit(EXIT_FAILURE);
         }
         if (target_num >= SC[Isc_Ref].Nb) {
            fprintf(stderr,
                    "Spacecraft %ld only has %ld bodies, but the mirror target "
                    "was attempted to be set as body %ld. Exiting...\n",
                    Isc_Ref, SC[Isc_Ref].Nb, target_num);
            exit(EXIT_FAILURE);
         }
         // TODO: not truth of other body
         quat_t qbn;
         vec3_t wbn;
         struct BodyType *TrgSB        = NULL;
         struct DSMStateType *TrgState = NULL;
         {
            // Limit scope where we need SCType
            struct SCType *TrgS    = &SC[Isc_Ref];
            TrgSB                  = TrgS->B;
            struct DSMType *TrgDSM = &TrgS->DSM;
            TrgState               = &TrgDSM->commState;
         }
         if (target_num != 0) {
            // get relative orientation of body to B[0] then apply this to
            // AC.qbn
            vec3_t wBnb, wBnbDSM;
            quat_t qbB = QxQT(TrgSB[target_num].qn, TrgSB[0].qn);
            qbn        = QxQ(qbB, TrgState->qbn);

            // get relative angular velocity of body to B[0], then apply this
            // to AC.wbn; all in B[frame_body] frame
            wBnb    = QxV(qbB, TrgSB[0].wn);
            wBnbDSM = QxV(qbB, TrgState->wbn);

            for (i = 0; i < 3; i++)
               wbn.v[i] =
                   wBnbDSM.v[i] + (TrgSB[target_num].wn.v[i] - wBnb.v[i]);
         }
         else {
            qbn = TrgState->qbn;
            wbn = TrgState->wbn;
         }
         Cmd->qbr = QxQT(state->qbn, qbn);
         Cmd->wrn = QTxV(TrgState->qbn, wbn);
      } break;
      case (PARM_DETUMBLE): {
         Cmd->qbr = QUAT_EYE;
         Cmd->wrn = VEC3_ZERO;
      } break;
      default:
         fprintf(stderr,
                 "Invalid Command Method for Attitude Guidance. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   CTRL->qbr      = Cmd->qbr;
   CTRL->dmp_kp   = Cmd->dmp_kp;
   CTRL->att_kp   = Cmd->att_kp;
   CTRL->att_kr   = Cmd->att_kr;
   CTRL->att_ki   = Cmd->att_ki;
   CTRL->Trq_max  = Cmd->Trq_max;
   CTRL->dTrq_max = Cmd->dTrq_max;
   CTRL->w_max    = Cmd->w_max;
}
//------------------------------------------------------------------------------
//                                NAVIGATION
//------------------------------------------------------------------------------
void NavigationModule(struct AcType *const AC, struct DSMType *const DSM)
{
   const struct DSMNavType *Nav  = &DSM->DsmNav;
   struct DSMStateType *DSMState = &DSM->state;

   if (Nav->NavigationActive == FALSE) {
      // TODO
      DSMState->Time = AC->Time;
      DSMState->PosN = AC->PosN;
      DSMState->VelN = AC->VelN;
      DSMState->PosR = VSubV_Elem(DSMState->PosN, DSM->refOrb->PosN);
      DSMState->VelR = VSubV_Elem(DSMState->VelN, DSM->refOrb->VelN);

      DSMState->CBN = AC->CBN;
      DSMState->qbn = AC->qbn;
      DSMState->wbn = AC->wbn;

      DSMState->svb = AC->svb;
      DSMState->svn = AC->svn;
      DSMState->bvb = AC->bvb;
      DSMState->bvn = AC->bvn;
      return;
   }

   KalmanFilt(AC, DSM);
   DSMState->Time = Date2Time(Nav->Date);
   AC->Time       = DSMState->Time;
   // Overwrite data in AC structure with filtered data
   FOR_STATES(state)
   {
      if (Nav->stateActive[state] == TRUE) {
         // TODO: what to do for states that are not active in Nav?
         vec3_t tmp3Vec;
         quat_t tmpQ;
         switch (state) {
            case TIME_STATE:
               // AC->Time = Nav->Time;
               break;
            case ROTMAT_STATE:
               DSMState->CBN = MTxM(Nav->CRB, Nav->refCRN);
               DSMState->qbn = C2Q(DSMState->CBN);
               break;
            case QUAT_STATE:
               tmpQ          = C2Q(Nav->refCRN);
               DSMState->qbn = QxQ(Nav->qbr, tmpQ);
               DSMState->CBN = Q2C(DSMState->qbn);
               break;
            case POS_STATE:
               tmp3Vec        = VAddV_Elem(Nav->PosR, Nav->refPos);
               DSMState->PosN = MTxV(Nav->refCRN, tmp3Vec);
               DSMState->PosR = VSubV_Elem(DSMState->PosN, DSM->refOrb->PosN);
               break;
            case VEL_STATE:
               // will need more (BKE) for non-inertial frame
               tmp3Vec        = VAddV_Elem(Nav->VelR, Nav->refVel);
               DSMState->VelN = MTxV(Nav->refCRN, tmp3Vec);

               DSMState->VelR = VSubV_Elem(DSMState->VelN, DSM->refOrb->VelN);
               break;
            case OMEGA_STATE:
               tmp3Vec       = MTxV(Nav->CRB, Nav->refOmega);
               DSMState->wbn = VAddV_Elem(Nav->wbr, tmp3Vec);
               break;
            default:
               break;
         }
      }
   }

   if (Nav->stateActive[ROTMAT_STATE] == TRUE ||
       Nav->stateActive[QUAT_STATE] == TRUE) {
      if (any_int(Nav->nSensor[MAG_SENSOR], Nav->sensorActive[MAG_SENSOR])) {
         DSMState->bvn = AC->bvn;
         DSMState->bvb = MxV(DSMState->CBN, DSMState->bvn);
         AC->bvb       = DSMState->bvb;
      }
      if (any_int(Nav->nSensor[CSS_SENSOR], Nav->sensorActive[CSS_SENSOR]) ||
          any_int(Nav->nSensor[FSS_SENSOR], Nav->sensorActive[FSS_SENSOR])) {
         DSMState->svn = AC->svn;
         DSMState->svb = MxV(DSMState->CBN, DSMState->svn);
         AC->svb       = DSMState->svb;
      }
   }
}
//------------------------------------------------------------------------------
void TranslationalNavigation(struct AcType *AC, struct DSMStateType *state)
{
   AC->PosN = state->PosN;
   AC->VelN = state->VelN;
}
//------------------------------------------------------------------------------
void AttitudeNavigation(struct AcType *AC, struct DSMStateType *state)
{
   AC->CBN = state->CBN;
   AC->qbn = state->qbn;
   AC->wbn = state->wbn;
}
//------------------------------------------------------------------------------
void MurAKF(struct AcType *AC __attribute__((unused)),
            struct DSMStateType *state __attribute__((unused)))
    __attribute__((unused));
void MurAKF(struct AcType *AC __attribute__((unused)),
            struct DSMStateType *state __attribute__((unused)))
{
   /* Propagate quaternion, bias, and error covariance */
   // (Hasnaa uses mag, ST, and FSS data)
   // long N = AC->Nmag + AC->Nst + AC->Nfss; // # observations?

   /* Compute attitude matrix A(qk^-) */

   /* Initialize error state vector */
   // double delta_xk_minus[3] = {0.0};

   // for (int i = 1; i < N; i++) {
   /* Sensitivity matrix */

   /* Compute Kalman gain */

   /* Update covariance, residual, and state */
   //}

   /* Reset */
}

//------------------------------------------------------------------------------
//                                 CONTROL
//------------------------------------------------------------------------------
void TranslationCtrl(struct DSMType *DSM)
{
   struct DSMCtrlType *CTRL   = &DSM->DsmCtrl;
   struct DSMCmdType *Cmd     = &DSM->Cmd;
   struct DSMStateType *state = &DSM->state;

   if (Cmd->TranslationCtrlActive == TRUE &&
       Cmd->ManeuverMode == MAN_INACTIVE) {
      switch (Cmd->trn_controller) {
         case PID_CNTRL: {
            // PID Controller
            if (Cmd->NewTrnGainsProcessed == TRUE) {
               DSM->trn_ei               = VEC3_ZERO;
               Cmd->NewTrnGainsProcessed = FALSE;
            }

            // Position Error
            DSM->perr = VSubV_Elem(state->PosR, CTRL->CmdPosR);

            // Velocity Error
            DSM->verr = VSubV_Elem(state->VelR, CTRL->CmdVelR);

            // Integrated Error
            for (int i = 0; i < 3; i++)
               DSM->trn_ei.v[i] =
                   (DSM->perr.v[i] + DSM->Oldperr.v[i]) * DSM->DT / 2.0;

            DSM->trn_ei = LimitElem_bidir(
                DSM->trn_ei, VDivV_Elem(Cmd->trn_kilimit, CTRL->trn_ki));

            for (int i = 0; i < 3; i++)
               CTRL->u1.v[i] =
                   CTRL->trn_kp.v[i] / CTRL->trn_kr.v[i] * DSM->perr.v[i];

            CTRL->u1 = LimitElem_bidir(CTRL->u1, CTRL->vel_max);

            for (int i = 0; i < 3; i++)
               CTRL->FcmdN.v[i] =
                   -CTRL->trn_kr.v[i] * (CTRL->u1.v[i] + DSM->verr.v[i]) -
                   CTRL->trn_ki.v[i] * DSM->trn_ei.v[i];

            // Converting from Inertial to body frame for Report
            CTRL->FcmdB = QxV(state->qbn, CTRL->FcmdN);

            // Limiting AC->Frc in body frame
            CTRL->FcmdB = LimitElem_bidir(CTRL->FcmdB, CTRL->FrcB_max);
            CTRL->FcmdN = QTxV(state->qbn, CTRL->FcmdB);
         } break;
         case LYA_2BODY_CNTRL: {
            // Calculate relative radius, velocity
            // Position Error, Relative
            DSM->perr = VSubV_Elem(state->PosR, CTRL->CmdPosR);

            // Velocity Error
            DSM->verr = VSubV_Elem(state->VelR, CTRL->CmdVelR);

            const double r_norm  = MAGV(state->PosN);
            const double r_cntrl = MAGV(CTRL->CmdPosN);
            const double mu      = DSM->refOrb->mu;

            for (int i = 0; i < 3; i++) {
               const double dg  = -mu / pow(r_norm, 3) * state->PosN.v[i] +
                                  mu / pow(r_cntrl, 3) * CTRL->CmdPosN.v[i];
               CTRL->FcmdN.v[i] = -CTRL->trn_kp.v[i] * DSM->perr.v[i] -
                                  CTRL->trn_kr.v[i] * DSM->verr.v[i] -
                                  dg * DSM->mass;
            }

            // Converting from Inertial to body frame for Report
            CTRL->FcmdB = QxV(state->qbn, CTRL->FcmdN);

            // Limiting AC->Frc in body frame
            CTRL->FcmdB = LimitElem_bidir(CTRL->FcmdB, CTRL->FrcB_max);
            CTRL->FcmdN = QTxV(state->qbn, CTRL->FcmdB);
         } break;
         default:
            fprintf(stderr,
                    "Invalid Translational Controller type. Exiting...\n");
            exit(EXIT_FAILURE);
            break;
      }
      DSM->Oldperr = DSM->perr;
   }
   else if (Cmd->TranslationCtrlActive == TRUE &&
            Cmd->ManeuverMode != MAN_INACTIVE) {
      if (SimTime < Cmd->BurnStopTime) {
         switch (Cmd->ManeuverMode) {
            case MAN_CONSTANT: {
               if (!strcmp(Cmd->RefFrame, "N")) {
                  CTRL->FcmdN = SxV(DSM->mass / Cmd->BurnTime, Cmd->DeltaV);
                  // Converting from Inertial to body frame for Report
                  CTRL->FcmdB = QxV(state->qbn, CTRL->FcmdN);
               }
               else if (!strcmp(Cmd->RefFrame, "B"))
                  CTRL->FcmdB = SxV(DSM->mass / Cmd->BurnTime, Cmd->DeltaV);

               // Limiting AC->Frc in body frame
               CTRL->FcmdB = LimitElem_bidir(CTRL->FcmdB, CTRL->FrcB_max);

               // Converting back to Inertial from body frame
               CTRL->FcmdN = QTxV(state->qbn, CTRL->FcmdB);
            } break;
            case MAN_SMOOTHED: {
               // .99998 corresponds to capturing 99.999% of the burn since tanh
               // has an asymptote
               const double coef = -2 * atanh(-0.99998);

               const double sharp = coef / Cmd->BurnTime;
               const double t_mid = Cmd->BurnStopTime - Cmd->BurnTime / 2.0;
               // Time elapsed since middle of burn
               const double t_since_mid = SimTime - t_mid;
               const double coshSharp   = cosh(sharp * t_since_mid);
               const double coshSharp2  = coshSharp * coshSharp;

               if (!strcmp(Cmd->RefFrame, "N")) {
                  CTRL->FcmdN = SxV((DSM->mass * sharp) / (2.0 * coshSharp2),
                                    Cmd->DeltaV);
                  // Converting from Inertial to body frame for Report
                  CTRL->FcmdB = QxV(state->qbn, CTRL->FcmdN);
               }
               else if (!strcmp(Cmd->RefFrame, "B"))
                  CTRL->FcmdB = SxV((DSM->mass * sharp) / (2.0 * coshSharp2),
                                    Cmd->DeltaV);
               CTRL->FcmdB = LimitElem_bidir(CTRL->FcmdB, CTRL->FrcB_max);

               // Converting back to Inertial from body frame
               CTRL->FcmdN = QTxV(state->qbn, CTRL->FcmdB);
            } break;
            default:
               fprintf(stderr, "Invalid maneuver mode. Exiting...\n");
               exit(EXIT_FAILURE);
               break;
         }
      }
      else {
         Cmd->ManeuverMode          = MAN_INACTIVE;
         Cmd->TranslationCtrlActive = FALSE;
         CTRL->FcmdN                = VEC3_ZERO;
         CTRL->FcmdB                = VEC3_ZERO;
      }
   }
   else {
      CTRL->FcmdN = VEC3_ZERO;
      CTRL->FcmdB = VEC3_ZERO;
   }
   // Assigning CMDs to upper structure
   DSM->FcmdN = CTRL->FcmdN;
   DSM->FcmdB = CTRL->FcmdB;
}
//------------------------------------------------------------------------------
void AttitudeCtrl(struct DSMType *DSM)
{
   vec3_t wrb;

   struct DSMCtrlType *CTRL   = &DSM->DsmCtrl;
   struct DSMCmdType *Cmd     = &DSM->Cmd;
   struct DSMStateType *state = &DSM->state;

   if (Cmd->AttitudeCtrlActive == TRUE) {
      switch (Cmd->att_controller) {
         case PID_CNTRL: {
            // PID Controller
            if (Cmd->NewAttGainsProcessed == TRUE) {
               DSM->att_ei               = VEC3_ZERO;
               Cmd->NewAttGainsProcessed = FALSE;
            }
            // Angular Position Error
            DSM->therr = Q2AngleVec(CTRL->qbr);
            // Rotate angular velocity into Body frame
            wrb = QxV(state->qbn, Cmd->wrn);

            // Angular Velocity Error (in body frame)
            DSM->werr = VSubV_Elem(state->wbn, wrb);

            // Integrated angle error
            for (int i = 0; i < 3; i++)
               DSM->att_ei.v[i] +=
                   (DSM->Oldtherr.v[i] + DSM->therr.v[i]) / 2.0 * DSM->DT;

            DSM->att_ei = LimitElem_bidir(
                DSM->att_ei, VDivV_Elem(Cmd->att_kilimit, CTRL->att_ki));

            for (int i = 0; i < 3; i++)
               CTRL->u2.v[i] =
                   CTRL->att_kp.v[i] / CTRL->att_kr.v[i] * DSM->therr.v[i];

            CTRL->u2 = LimitElem_bidir(CTRL->u2, CTRL->w_max);

            for (int i = 0; i < 3; i++)
               CTRL->Tcmd.v[i] =
                   -CTRL->att_kr.v[i] * (CTRL->u2.v[i] + DSM->werr.v[i]) -
                   CTRL->att_ki.v[i] * DSM->att_ei.v[i];

            DSM->Oldtherr = DSM->therr;
         } break;
         case LYA_ATT_CNTRL: {
            // Angular Position Error
            DSM->therr = Q2AngleVec(CTRL->qbr);
            // Rotate angular velocity into Body frame
            wrb = QxV(state->qbn, Cmd->wrn);

            // Angular Velocity Error (in body frame)
            DSM->werr = VSubV_Elem(state->wbn, wrb);

            // calculate nonlinear term in Quaternion Lyapunov stability
            vec3_t om_x_I_om = vxMov(DSM->werr, DSM->MOI);
            for (int i = 0; i < 3; i++) {
               CTRL->Tcmd.v[i] = -Cmd->att_kp.v[i] * CTRL->qbr.qv.v[i] -
                                 Cmd->att_kr.v[i] * DSM->werr.v[i] +
                                 om_x_I_om.v[i];
            }
         } break;
         default:
            fprintf(stderr, "Invalid Attitude controller type. Exiting...\n");
            exit(EXIT_FAILURE);
            break;
      }
      CTRL->dTcmd = LimitElem_bidir(CTRL->dTcmd, CTRL->dTrq_max);
   }
   else
      CTRL->Tcmd = VEC3_ZERO;

   // Assigning CMDs to upper structure
   DSM->Tcmd = CTRL->Tcmd;
   DSM->Mcmd = VEC3_ZERO; // For now, this is unused, so it needs to be cleared
}
//------------------------------------------------------------------------------
void MomentumDumpCtrl(struct DSMType *DSM, vec3_t TotalWhlH)
{
   double whlHNorm = 0.0;

   struct DSMCmdType *Cmd;
   struct DSMCtrlType *CTRL;

   Cmd  = &DSM->Cmd;
   CTRL = &DSM->DsmCtrl;

   whlHNorm = MAGV(TotalWhlH);

   if (Cmd->H_DumpActive == FALSE ||
       ((CTRL->H_DumpActive == TRUE) && (whlHNorm < Cmd->H_DumpLims[0])))
      CTRL->H_DumpActive = FALSE;
   else if (Cmd->H_DumpActive == TRUE &&
            ((CTRL->H_DumpActive == FALSE) && (whlHNorm > Cmd->H_DumpLims[1])))
      CTRL->H_DumpActive = TRUE;

   if (CTRL->H_DumpActive == TRUE) {
      switch (Cmd->dmp_controller) {
         case H_DUMP_CNTRL:
            CTRL->dTcmd = NegV_Elem(VMulV_Elem(CTRL->dmp_kp, TotalWhlH));
            break;
         default:
            fprintf(
                stderr,
                "Invalid controller type for Momentum Dumping. How did this "
                "happen? Exiting...\n");
            exit(EXIT_FAILURE);
            break;
      }

      CTRL->dTcmd = LimitElem_bidir(CTRL->dTcmd, CTRL->dTrq_max);
   }
   else
      CTRL->dTcmd = VEC3_ZERO;

   DSM->dTcmd = CTRL->dTcmd;
}
//------------------------------------------------------------------------------
//                             FLIGHT SOFTWARE
//------------------------------------------------------------------------------
void DsmFSW(struct SCType *S)
{
   // load the DSM file statically so that all DsmFSW calls have access to
   // same object. Document is destroyed at program exit
   static struct fy_node *dsmRoot = NULL, *dsmCmds = NULL;
   if (dsmRoot == NULL) {
      struct fy_document *fyd =
          fy_document_build_and_check(NULL, InOutPath, "Inp_DSM.yaml");
      dsmRoot = fy_document_root(fyd);
      dsmCmds = fy_node_by_path_def(dsmRoot, "/DSM Commands");
   }

   struct DSMType *const DSM = &S->DSM;
   struct AcType *const AC   = &S->AC;

   // Run Command Interperter
   if (DSM->CmdInit) {
      DSM->CmdInit = 0;
      DsmCmdInterpreterMrk1(DSM, dsmCmds);

      // put place holders in integrator "old" values, set ei values to zero to
      // initialize integrated error
      DSM->Oldtherr = VEC3_ZERO;
      DSM->Oldperr  = VEC3_ZERO;

      DSM->att_ei = VEC3_ZERO;
      DSM->trn_ei = VEC3_ZERO;
   }

   if (DSM->CmdNum < DSM->CmdCnt && SimTime >= DSM->CmdNextTime) {
      DsmCmdInterpreterMrk2(AC, DSM);
      DSM->CmdNum++;
      if (DSM->CmdNum < DSM->CmdCnt)
         fy_node_scanf(DSM->CmdArray[DSM->CmdNum], "/Time %lf",
                       &DSM->CmdNextTime);
   }

   // Generate Data From Sensors
   // Navigation Modules
   NavigationModule(AC, DSM);
   TranslationalNavigation(AC, &DSM->state);
   AttitudeNavigation(AC, &DSM->state);

   // Generate Guidance Solution
   TranslationGuidance(DSM, &Frm[S->RefOrb]);
   AttitudeGuidance(DSM, &Frm[S->RefOrb]);

   // Run Control Systems
   TranslationCtrl(DSM);
   AttitudeCtrl(DSM);
   {
      // TODO: this
      vec3_t TotalWhlH = VEC3_ZERO;
      for (int i = 0; i < AC->Nwhl; i++)
         for (int j = 0; j < 3; j++)
            TotalWhlH.v[j] += AC->Whl[i].Axis.v[j] * AC->Whl[i].H;

      MomentumDumpCtrl(DSM, TotalWhlH);
   }

   // Implement Control Through Actuators
   ActuatorModule(AC, DSM);
}

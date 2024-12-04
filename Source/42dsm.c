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

#include "42.h"
#include "dsmkit.h"

#define EPS_DSM 1e-12

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

   for (i = 0; i < 3; i++) {
      cmdVec[i]     = AC->Fcmd[i];
      cmdVec[i + 3] = AC->Tcmd[i];
   }

   // Assigning PulseWidth to each Thruster
   for (i = 0; i < AC->Nthr; i++) {
      distDotCmd = 0.0;
      for (j = 0; j < 6; j++)
         distDotCmd += AC->Thr[i].DistVec[j] * cmdVec[j];
      AC->Thr[i].PulseWidthCmd  = Limit(distDotCmd * AC->DT, 0.0, AC->DT);
      AC->Thr[i].ThrustLevelCmd = Limit(distDotCmd, 0.0, 1.0);
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
            if (controllerState == TRN_STATE) {
               A[j][i] = AC->Thr[i].Axis[j];
            }
            else if (controllerState == ATT_STATE ||
                     controllerState == DMP_STATE) {
               A[j][i] = AC->Thr[i].rxA[j];
            }
         }
         else if (DOF == 6) {
            A[j][i]     = AC->Thr[i].Axis[j];
            A[j + 3][i] = AC->Thr[i].rxA[j];
         }
      }
   }
   for (i = 0; i < AC->Nthr; i++) {
      for (j = 0; j < DOF; j++) {
         A[j][i] *= AC->Thr[i].Fmax; // Without this, errors would arise if
                                     // thrusters have different max thrusts
      }
   }

   PINVG(A, APlus, DOF, AC->Nthr);

   for (i = 0; i < AC->Nthr; i++) {
      if (DOF == 3) {
         // Unused entries of DistVec will be zero, so won't cause issues with
         // eventual dot product
         for (j = 0; j < DOF; j++) {
            if (controllerState == TRN_STATE) {
               AC->Thr[i].DistVec[j] = APlus[i][j];
            }
            else if (controllerState == ATT_STATE ||
                     controllerState == DMP_STATE) {
               AC->Thr[i].DistVec[j + 3] = APlus[i][j];
            }
         }
      }
      else if (DOF == 6) {
         for (j = 0; j < DOF; j++) {
            AC->Thr[i].DistVec[j] = APlus[i][j];
         }
      }
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
   for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
         DSM->MOI[i][j] = S->AC.MOI[i][j];

   /* Controllers */
   DSM->DsmCtrl.Init         = 1;
   DSM->DsmCtrl.H_DumpActive = FALSE;
   DSM->CmdArray             = NULL;

   Cmd->TranslationCtrlActive = FALSE;
   Cmd->AttitudeCtrlActive    = FALSE;
   Cmd->H_DumpActive          = FALSE;
   strcpy(Cmd->dmp_actuator, "");
   Cmd->ActNumCmds = 0;
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
         kp         = Cmd->trn_kp;
         kr         = Cmd->trn_kr;
         ki         = Cmd->trn_ki;
         limit_vec  = Cmd->trn_kilimit;
         break;
      case ATT_STATE:
         controller = &Cmd->att_controller;
         kp         = Cmd->att_kp;
         kr         = Cmd->att_kr;
         ki         = Cmd->att_ki;
         limit_vec  = Cmd->att_kilimit;
         break;
      case FULL_STATE:
         // PLACEHOLDER
         break;
      case DMP_STATE:
         controller = &Cmd->dmp_controller;
         kp         = Cmd->dmp_kp;
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
                  kp[i] *= DSM->MOI[i][i];
                  kr[i] *= DSM->MOI[i][i];
                  ki[i] *= DSM->MOI[i][i];
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
                  kr[i] = sqrt(2.0 * k_lya * DSM->MOI[i][i]);
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
         printf(
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
         fMax       = Cmd->FrcB_max;
         vMax       = Cmd->vel_max;
         break;
      case ATT_STATE:
         controller = &Cmd->att_controller;
         fMax       = Cmd->Trq_max;
         vMax       = Cmd->w_max;
         break;
      case FULL_STATE:
         // PLACEHOLDER
         break;
      case DMP_STATE:
         controller = &Cmd->dmp_controller;
         fMax       = Cmd->dTrq_max;
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
   if (fy_node_scanf(ctrlNode, "/Type %41s", ctrlType) == 1) {
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
         printf("%s is an invalid control type. Exiting...\n", ctrlType);
         exit(EXIT_FAILURE);
      }
      // There should be a nicer way to handle this that doesn't require
      // hardcoding for things...
      if (controller == LYA_ATT_CNTRL && controllerState != ATT_STATE) {
         printf(
             "%s\n",
             "Can only use LYA_ATT_CNTRL for attitude control. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      if (controller == H_DUMP_CNTRL && controllerState != DMP_STATE) {
         printf("%s\n", "Can only use H_DUMP_CNTRL for momentum dumping "
                        "control. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      if (controller == LYA_2BODY_CNTRL && controllerState != TRN_STATE) {
         printf("%s\n", "Can only use LYA_2BODY_CNTRL for translation control. "
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
         printf("For Controller alias %s, could not find Gain alias %s or "
                "invalid format. Exiting...\n",
                fy_anchor_get_text(fy_node_get_anchor(ctrlNode), NULL),
                fy_anchor_get_text(fy_node_get_anchor(gainNode), NULL));
         exit(EXIT_FAILURE);
      }
      if (GetLimits(DSM, limNode, controllerState) == FALSE) {
         printf("For Controller alias %s, could not find Limit alias %s or "
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
      printf("Could not find Command Data for Translation command of subtype "
             "%s. Exiting...\n",
             subType);
      exit(EXIT_FAILURE);
   }
   const char *cmdName =
       fy_node_get_scalar0(fy_node_by_path_def(cmdNode, "/Description"));

   if (!strcmp(subType, "Position")) {
      Cmd->TranslationCtrlActive = TRUE;
      long isGood                = fy_node_scanf(cmdNode,
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
                 3, fy_node_by_path_def(cmdNode, "/Position"), Cmd->Pos) == 3;
      }
      ctrlNode  = fy_node_by_path_def(cmdNode, "/Controller");
      actNode   = fy_node_by_path_def(cmdNode, "/Actuator");
      isGood   &= ctrlNode != NULL && actNode != NULL;

      if (isGood) {
         TranslationCmdProcessed = TRUE;
         Cmd->ManeuverMode       = INACTIVE;
      }

      if (TranslationCmdProcessed == FALSE) {
         printf("Position Command %s has invalid format. Exiting...\n",
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
         Cmd->ManeuverMode       = INACTIVE;
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
                                  Cmd->DeltaV) == 3;
      if (isGood) {
         TranslationCmdProcessed = TRUE;
         Cmd->BurnStopTime       = DsmCmdTime + Cmd->BurnTime;
         if (!strcmp(manType, "CONSTANT"))
            Cmd->ManeuverMode = CONSTANT;
         else if (!strcmp(manType, "SMOOTHED"))
            Cmd->ManeuverMode = SMOOTHED;
         else {
            printf("%s is an invalid maneuver mode for Maneuver %s. "
                   "Exiting...",
                   manType, cmdName);
            exit(EXIT_FAILURE);
         }
      }
      if (TranslationCmdProcessed == FALSE) {
         printf("Translation Command %s has invalid format. Exiting...\n",
                cmdName);
         exit(EXIT_FAILURE);
      }
   }

   if (TranslationCmdProcessed == TRUE && Cmd->TranslationCtrlActive == TRUE) {
      if (Cmd->ManeuverMode == INACTIVE) {
         if (GetController(DSM, ctrlNode, TRN_STATE) == FALSE) {
            printf("For %s command %s, could not find Controller alias %s or "
                   "invalid format. Exiting...\n",
                   subType, cmdName,
                   fy_anchor_get_text(fy_node_get_anchor(ctrlNode), NULL));
            exit(EXIT_FAILURE);
         }
      }
      else {
         if (GetLimits(DSM, limNode, TRN_STATE) == FALSE) {
            printf(
                "For %s command %s, could not find Limit alias %s or invalid "
                "format. Exiting...\n",
                subType, cmdName,
                fy_anchor_get_text(fy_node_get_anchor(limNode), NULL));
            exit(EXIT_FAILURE);
         }
      }
      if (GetActuators(AC, DSM, actNode, TRN_STATE) == FALSE) {
         printf(
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
      printf("Could not find Command Data for Attitude command of subtype %s. "
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
         printf("For Vector command %s, could not find Primary Vector. "
                "Exiting...\n",
                cmdName);
         exit(EXIT_FAILURE);
      }

      if (!strcmp(subType, "Two Vector")) {
         kMax        = 2;
         Cmd->Method = PARM_VECTORS;

         if (nodes[1] == NULL) {
            printf("For Two Vector command %s, could not find Secondary "
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
                                 vecs[k]->cmd_axis);
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
                  for (int i = 0; i < 3; i++)
                     vecs[k]->W[i] = GroundStation[gsNum].PosW[i];
               }
               else {
                  vecs[k]->TrgWorld = DecodeString(target);
                  for (int i = 0; i < 3; i++)
                     vecs[k]->W[i] = 0.0;
               }
            }
            else if (!strcmp(tgtType, "SC")) {
               vecs[k]->TrgType = TARGET_SC;
               if (sscanf(target, "SC[%ld].B[%ld]", &vecs[k]->TrgSC,
                          &vecs[k]->TrgBody) == 2) {
                  // Decode Current SC ID Number
                  if (vecs[k]->TrgSC >= Nsc) {
                     printf("This mission only has %ld spacecraft, but "
                            "spacecraft %ld was attempted to be set as the "
                            "primary target vector. Exiting...\n",
                            Nsc, vecs[k]->TrgSC);
                     exit(EXIT_FAILURE);
                  }

                  if (vecs[k]->TrgBody >= SC[vecs[k]->TrgSC].Nb) {
                     printf("Spacecraft %ld only has %ld bodies, but the "
                            "primary target was attempted to be set as body "
                            "%ld. Exiting...\n",
                            vecs[k]->TrgSC, SC[vecs[k]->TrgSC].Nb,
                            vecs[k]->TrgBody);
                     exit(EXIT_FAILURE);
                  }
               }
               else {
                  printf("%s is in incorrect format. Exiting...", target);
                  exit(EXIT_FAILURE);
               }
            }
            else {
               printf("%s Vector for command %s has improper format for SC or "
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
                                  vecs[k]->cmd_vec) == 3;
            if (*attcmdProc[k] == FALSE) {
               printf("%s Vector for command %s has improper format for VEC "
                      "targeting. Exiting...\n",
                      (k == 0) ? ("Primary") : ("Secondary"), cmdName);
               exit(EXIT_FAILURE);
            }
         }
         else {
            printf("For %s Vector for command %s, %s is an invalid targeting "
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
              4, fy_node_by_path_def(cmdNode, "/Quaternion"), Cmd->q) == 4;
      AttitudeCmdProcessed &=
          fy_node_scanf(cmdNode, "/Frame %19s", Cmd->AttRefFrame) == 1;
      ctrlNode              = fy_node_by_path_def(cmdNode, "/Controller");
      actNode               = fy_node_by_path_def(cmdNode, "/Actuator");
      AttitudeCmdProcessed &= ctrlNode != NULL && actNode != NULL;

      if (AttitudeCmdProcessed == FALSE) {
         printf("Quaternion Command %s has invalid format. Exiting...\n",
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
         printf("Mirror Command %s has invalid format. Exiting...\n", cmdName);
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
         printf("Detumble Command %s has invalid format. Exiting...\n",
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
         printf("Maximum momentum dump limit must be more than the minimum for "
                "Whl H Manage Command %s Exiting...\n",
                cmdName);
         exit(EXIT_FAILURE);
      }
      if (AttitudeCmdProcessed == FALSE) {
         printf("Whl H Manage Command %s has invalid format. Exiting...\n",
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
                              vec->cmd_axis);
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
               for (int i = 0; i < 3; i++)
                  vec->W[i] = GroundStation[gsNum].PosW[i];
            }
            else {
               vec->TrgWorld = DecodeString(target);
               for (int i = 0; i < 3; i++)
                  vec->W[i] = 0.0;
            }
         }
         else if (!strcmp(tgtType, "SC")) {
            vec->TrgType = TARGET_SC;
            if (sscanf(target, "SC[%ld].B[%ld]", &vec->TrgSC, &vec->TrgBody) ==
                2) {
               // Decode Current SC ID Number
               if (vec->TrgSC >= Nsc) {
                  printf("This mission only has %ld spacecraft, but "
                         "spacecraft %ld was attempted to be set as the "
                         "primary target vector. Exiting...\n",
                         Nsc, vec->TrgSC);
                  exit(EXIT_FAILURE);
               }

               if (vec->TrgBody >= SC[vec->TrgSC].Nb) {
                  printf("Spacecraft %ld only has %ld bodies, but the "
                         "primary target was attempted to be set as body "
                         "%ld. Exiting...\n",
                         vec->TrgSC, SC[vec->TrgSC].Nb, vec->TrgBody);
                  exit(EXIT_FAILURE);
               }
            }
            else {
               printf("%s is in incorrect format. Exiting...", target);
               exit(EXIT_FAILURE);
            }
         }
         else {
            printf("Vector for command %s has improper format for SC or "
                   "BODY targeting. Exiting...\n",
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
                                     vec->cmd_vec) == 3;
         if (AttitudeCmdProcessed == FALSE) {
            printf("Vector for command %s has improper format for VEC "
                   "targeting. Exiting...\n",
                   cmdName);
            exit(EXIT_FAILURE);
         }
      }
      else {
         printf("For Vector for command %s, %s is an invalid targeting "
                "type. Exiting...\n",
                cmdName, tgtType);
         exit(EXIT_FAILURE);
      }
      // Load Desired Angular Rate in Cmd->AngRate[2], then construct vector as
      // parallel to Cmd->PriVec.cmd_axis
      AttitudeCmdProcessed &=
          fy_node_scanf(cmdNode, "/Rate %lf", &Cmd->AngRate[2]) == 1;
      Cmd->AngRate[2] *= D2R;
      for (int i = 0; i < 3; i++)
         Cmd->AngRate[i] = Cmd->PriVec.cmd_axis[i] * Cmd->AngRate[2];

      Cmd->AttitudeCtrlActive = TRUE;
      ctrlNode                = fy_node_by_path_def(cmdNode, "/Controller");
      actNode                 = fy_node_by_path_def(cmdNode, "/Actuator");
   }
   else {
      AttitudeCmdProcessed = FALSE;
   }
   if (AttitudeCmdProcessed == TRUE && Cmd->AttitudeCtrlActive == TRUE) {
      if (GetController(DSM, ctrlNode, state) == FALSE) {
         printf("For %s command %s, could not find Controller alias %s or "
                "invalid format. Exiting...\n",
                subType, cmdName,
                fy_anchor_get_text(fy_node_get_anchor(ctrlNode), NULL));
         exit(EXIT_FAILURE);
      }

      if (GetActuators(AC, DSM, actNode, state) == FALSE) {
         printf(
             "For %s command %s, could not find Actuator alias %s or invalid "
             "format. Exiting...\n",
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
      printf("Could not find Command Data for Actuator command. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   const char *cmdName =
       fy_node_get_scalar0(fy_node_by_path_def(cmdNode, "/Description"));

   actSeqNode      = fy_node_by_path_def(cmdNode, "/Actuators");
   Cmd->ActNumCmds = fy_node_sequence_item_count(actSeqNode);
   iterNode        = NULL;
   WHILE_FY_ITER(actSeqNode, iterNode)
   {
      const char *searchStr =
          "/Type %" STR(FIELDWIDTH) "s /Index %ld /Duty Cycle %lf";
      char type[FIELDWIDTH + 1] = {};
      if (fy_node_scanf(iterNode, searchStr, type, &Cmd->ActInds[i],
                        &Cmd->ActDuties[i]) == 3) {
         if (!strcmp(type, "WHL"))
            Cmd->ActTypes[i] = WHL_TYPE;
         else if (!strcmp(type, "THR"))
            Cmd->ActTypes[i] = THR_TYPE;
         else if (!strcmp(type, "MTB"))
            Cmd->ActTypes[i] = MTB_TYPE;
         else {
            printf("Actuator Command index %s has improper actuator type %s. "
                   "Exiting...",
                   cmdName, type);
            exit(EXIT_FAILURE);
         }
      }
      else {
         printf(
             "Actuator Command index %s is impropertly formatted. Exiting...",
             cmdName);
         exit(EXIT_FAILURE);
      }
      if (Cmd->ActTypes[i] == WHL_TYPE && Cmd->ActInds[i] > AC->Nwhl) {
         printf("SC[%ld] only has %ld wheels, but an actuator command was sent "
                "to wheel %d. Exiting...\n",
                AC->ID, AC->Nwhl, Cmd->ActInds[i]);
         exit(EXIT_FAILURE);
      }
      if (Cmd->ActTypes[i] == THR_TYPE && Cmd->ActInds[i] > AC->Nthr) {
         printf("SC[%ld] only has %ld thrusters, but an actuator command was "
                "sent to thruster %d. Exiting...\n",
                AC->ID, AC->Nthr, Cmd->ActInds[i]);
         exit(EXIT_FAILURE);
      }
      if (Cmd->ActTypes[i] == MTB_TYPE && Cmd->ActInds[i] > AC->Nmtb) {
         printf("SC[%ld] only has %ld MTBs, but an actuator command was sent "
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
         printf("Improperly configured DSM Commands SC sequence. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      if (scInd == DSM->ID) {
         scCmdsNode = fy_node_by_path_def(iterNode, "/Command Sequence");
         if (scCmdsNode == NULL) {
            printf("Could not find Command Sequence for SC[%li]. Exiting...\n",
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
void DsmCmdInterpreterMrk2(struct AcType *const AC, struct DSMType *const DSM,
                           struct fy_node *dsmRoot)
{
   struct DSMCmdType *Cmd = &DSM->Cmd;
   struct fy_node *cmdsNode =
       fy_node_by_path_def(DSM->CmdArray[DSM->CmdNum], "/Commands");

   if (cmdsNode == NULL) {
      printf("Could not find command for SC[%ld] at time %lf. "
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
            printf("Translation command of subtype %s cannot be found in "
                   "Inp_DSM.yaml. Exiting...\n",
                   subType);
            exit(EXIT_FAILURE);
         }
      }
      else if (!strcmp(typeToken, "Attitude")) {
         if (GetAttitudeCmd(AC, DSM, iterNode) == FALSE) {
            fy_node_scanf(iterNode, searchSubtypeStr, subType);
            printf("Attitude command of subtype %s cannot be found in "
                   "Inp_DSM.yaml. Exiting...\n",
                   subType);
            exit(EXIT_FAILURE);
         }
      }
      else if (!strcmp(typeToken, "Actuator")) {
         if (GetActuatorCmd(AC, DSM, iterNode) == FALSE) {
            printf("Actuator command cannot be found in Inp_DSM.yaml. "
                   "Exiting...\n");
            exit(EXIT_FAILURE);
         }
      }
      else {
         printf("%s is not a supported command type. Exiting...\n", typeToken);
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
      printf("If the Translation actuator is 6DOF Thruster and Attitude "
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
void SensorModule(struct AcType *const AC)
{
   if (AC->Ngyro > 0)
      DSM_GyroProcessing(AC);
   if (AC->Nmag > 0)
      DSM_MagnetometerProcessing(AC);
   if (AC->Ncss > 0)
      DSM_CssProcessing(AC);
   if (AC->Nfss > 0)
      DSM_FssProcessing(AC);
   if (AC->Nst > 0)
      DSM_StarTrackerProcessing(AC);
   if (AC->Ngps > 0)
      DSM_GpsProcessing(AC);
   if (AC->Nst > 0)
      DSM_AccelProcessing(AC);
}
//------------------------------------------------------------------------------
//                                ACTUATORS
//------------------------------------------------------------------------------
void ActuatorModule(struct AcType *const AC, struct DSMType *const DSM)
{

   long i;
   double unit_bvb[3];

   struct DSMCmdType *Cmd = &DSM->Cmd;

   // Zero out all actuators first, so that they will be set nonzero only if
   // desired
   if (AC->Nthr > 0) {
      for (i = 0; i < AC->Nthr; i++) {
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

   for (i = 0; i < 3; i++) {
      AC->IdealFrc[i] = 0.0;
      AC->IdealTrq[i] = 0.0;
      AC->Fcmd[i]     = 0.0;
      AC->Tcmd[i]     = 0.0;
      AC->Mcmd[i]     = 0.0;
   }

   // Translation
   if (Cmd->TranslationCtrlActive == TRUE) {
      if ((!strcmp(Cmd->trn_actuator, "THR_3DOF") ||
           !strcmp(Cmd->trn_actuator, "THR_6DOF")) &&
          AC->Nthr > 0) {
         for (i = 0; i < 3; i++)
            AC->Fcmd[i] = DSM->FcmdB[i];
         ThrProcessingMinPower(AC);
      }
      else if (!strcmp(Cmd->trn_actuator, "Ideal")) {
         for (i = 0; i < 3; i++)
            AC->IdealFrc[i] = DSM->FcmdB[i];
      }
      else {
      } // What is Cmd->trn_actuator if no valid Cmd->ActuatorMode?? Error????
   }

   // Attitude
   if (Cmd->AttitudeCtrlActive == TRUE) {
      if ((!strcmp(Cmd->att_actuator, "THR_3DOF") ||
           !strcmp(Cmd->att_actuator, "THR_6DOF")) &&
          AC->Nthr > 0) {
         for (i = 0; i < 3; i++)
            AC->Tcmd[i] = DSM->Tcmd[i];
         // if THR_TRN, this does both force & torque since AC->Fcmd set
         ThrProcessingMinPower(AC);
      }
      else if (!strcmp(Cmd->att_actuator, "WHL") && AC->Nwhl > 0) {
         for (i = 0; i < 3; i++)
            AC->Tcmd[i] = DSM->Tcmd[i];
         DSM_WheelProcessing(AC);
      }
      else if (!strcmp(Cmd->att_actuator, "MTB") && AC->Nmtb > 0) {
         CopyUnitV(AC->bvb, unit_bvb);
         VxV(unit_bvb, DSM->Tcmd, DSM->Mcmd);
         for (i = 0; i < 3; i++)
            AC->Mcmd[i] = DSM->Mcmd[i] / MAGV(AC->bvb);
         DSM_MtbProcessing(AC);
      }
      else if (!strcmp(Cmd->att_actuator, "Ideal")) {
         for (i = 0; i < 3; i++)
            AC->IdealTrq[i] = DSM->Tcmd[i];
      }
      else {
      } // What is Cmd->att_actuator if no valid Cmd->AttActuatorMode??
        // Error????
   }

   // TODO: move momentum dumping to its own type
   // Momentum Dumping
   if (Cmd->H_DumpActive == TRUE) {
      if ((Cmd->AttitudeCtrlActive == TRUE &&
           strcmp(Cmd->att_actuator, "WHL")) ||
          Cmd->AttitudeCtrlActive == FALSE) {
         printf("You many only enable momentum dumping when the attitude is "
                "actively controlled with WHLs. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      if (DSM->DsmCtrl.H_DumpActive == TRUE &&
          !strcmp(Cmd->dmp_actuator, "MTB") && AC->Nmtb > 0) {
         CopyUnitV(AC->bvb, unit_bvb);
         VxV(unit_bvb, DSM->dTcmd, DSM->Mcmd);
         for (i = 0; i < 3; i++)
            AC->Mcmd[i] = DSM->Mcmd[i] / MAGV(AC->bvb);
         DSM_MtbProcessing(AC);
      }
      else if (DSM->DsmCtrl.H_DumpActive == TRUE &&
               (!strcmp(Cmd->dmp_actuator, "THR_3DOF") ||
                !strcmp(Cmd->dmp_actuator, "THR_6DOF")) &&
               AC->Nthr > 0) {
         // maybe have thrusters just thrust at
         // min(thrustertorquemax,SCALE*AC->Whl[i].Tmax)??? this could run into
         // issues if Thruster is being used for translation
         for (i = 0; i < 3; i++)
            AC->Tcmd[i] = DSM->dTcmd[i];
         // if THR_TRN, this does both force & torque since AC->Fcmd set
         ThrProcessingMinPower(AC);
      }
      else if (DSM->DsmCtrl.H_DumpActive == TRUE &&
               !strcmp(Cmd->dmp_actuator, "Ideal")) {
         for (i = 0; i < 3; i++)
            AC->IdealTrq[i] = DSM->dTcmd[i];
      }
   }

   // Process ActuatorCmd
   for (i = 0; i < Cmd->ActNumCmds;
        i++) // loops through stored Actuator commands
   {
      if (Cmd->ActTypes[i] == WHL_TYPE) {
         AC->Whl[Cmd->ActInds[i]].Tcmd =
             AC->Whl[i].Tmax * Cmd->ActDuties[i] / 100;
      }
      else if (Cmd->ActTypes[i] == THR_TYPE) {
         AC->Thr[Cmd->ActInds[i]].PulseWidthCmd =
             Cmd->ActDuties[i] / 100 * AC->DT;
         AC->Thr[Cmd->ActInds[i]].ThrustLevelCmd = Cmd->ActDuties[i] / 100;
      }
      else if (Cmd->ActTypes[i] == MTB_TYPE) {
         AC->MTB[Cmd->ActInds[i]].Mcmd =
             AC->MTB[i].Mmax * Cmd->ActDuties[i] / 100;
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
   double RelPosB[3], vb[3];
   double RelPosN[3], RelPosH[3], RelVelN[3], RelVelH[3];
   double pn[3], vn[3], ph[3], vh[3];
   double CosPriMerAng, SinPriMerAng;
   double MaxToS, Rhat[3], ToS;
   long It, i;

   struct OrbitType const *RefOrb = DSM->refOrb;
   struct DSMStateType *state     = &DSM->state;

   switch (CV->TrgType) {
      case TARGET_WORLD: {
         struct WorldType *TrgW = &World[CV->TrgWorld];
         CosPriMerAng           = cos(TrgW->PriMerAng);
         SinPriMerAng           = sin(TrgW->PriMerAng);
         pn[0] = CV->W[0] * CosPriMerAng - CV->W[1] * SinPriMerAng;
         pn[1] = CV->W[0] * SinPriMerAng + CV->W[1] * CosPriMerAng;
         pn[2] = CV->W[2];
         vn[0] = -CV->W[0] * SinPriMerAng - CV->W[1] * CosPriMerAng;
         vn[1] = CV->W[0] * CosPriMerAng - CV->W[1] * SinPriMerAng;
         vn[2] = 0.0;
         if (CV->TrgWorld == RefOrb->World) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = pn[i] - state->PosN[i];
               RelVelN[i] = vn[i] - state->VelN[i];
            }
         }
         else {
            MTxV(TrgW->CNH, pn, ph);
            MTxV(TrgW->CNH, vn, vh);
            struct WorldType *W = &World[RefOrb->World];
            MTxV(W->CNH, state->PosN, RelPosH);
            MTxV(W->CNH, state->VelN, RelVelH);
            for (i = 0; i < 3; i++) {
               RelPosH[i] = (TrgW->PosH[i] - W->PosH[i]) + (ph[i] - RelPosH[i]);
               RelVelH[i] = (TrgW->VelH[i] - W->VelH[i]) + (vh[i] - RelVelH[i]);
            }
            MxV(W->CNH, RelPosH, RelPosN);
            MxV(W->CNH, RelVelH, RelVelN);
         }
         CopyUnitV(RelPosN, CV->N);
         DSM_RelMotionToAngRate(RelPosN, RelVelN, CV->wn);
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
            for (i = 0; i < 3; i++) {
               RelPosN[i] = TrgState->PosR[i] - state->PosR[i];
               RelVelN[i] = TrgState->VelR[i] - state->VelR[i];
            }
         }
         else if (TrgOrb->World == RefOrb->World) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = TrgState->PosN[i] - state->PosN[i];
               RelVelN[i] = TrgState->VelN[i] - state->VelN[i];
            }
         }
         else {
            struct WorldType *TrgW = &World[TrgOrb->World];
            struct WorldType *W    = &World[RefOrb->World];
            MTxV(TrgW->CNH, TrgState->PosN, RelPosH);
            MTxV(TrgW->CNH, TrgState->VelN, RelVelH);
            MTxV(TrgW->CNH, state->PosN, ph);
            MTxV(TrgW->CNH, state->VelN, vh);
            for (i = 0; i < 3; i++) {
               RelPosH[i] -= ph[i];
               RelVelH[i] -= vh[i];
               RelPosH[i] += (TrgW->PosH[i] - W->PosH[i]);
               RelVelH[i] += (TrgW->VelH[i] - W->VelH[i]);
            }
            MxV(W->CNH, RelPosH, RelPosN);
            MxV(W->CNH, RelVelH, RelVelN);
         }
         CopyUnitV(RelPosN, CV->N);
         DSM_RelMotionToAngRate(RelPosN, RelVelN, CV->wn);
      } break;
      case TARGET_BODY: {
         struct OrbitType *TrgOrb      = NULL;
         struct DSMStateType *TrgState = NULL;
         struct BodyType *TrgSB        = NULL;
         double pcmn[3]                = {0.0};
         {
            // Limit the scope where SC is accessed
            struct SCType *TrgS    = &SC[CV->TrgSC];
            TrgSB                  = TrgS->B;
            struct DSMType *TrgDSM = &TrgS->DSM;
            TrgOrb                 = TrgDSM->refOrb;
            TrgState               = &TrgDSM->commState;
            // TODO: don't like accessing SCType::cm
            QTxV(TrgState->qbn, TrgS->cm, pcmn);
         }
         // TODO: make this better
         double qBb[4] = {0.0}, qbN[4] = {0.0};
         QxQT(TrgSB[0].qn, TrgSB[CV->TrgBody].qn, qBb);
         QTxQ(qBb, TrgState->qbn, qbN);
         for (i = 0; i < 3; i++)
            RelPosB[i] = CV->T[i] - TrgSB[CV->TrgBody].cm[i];
         VxV(TrgSB[CV->TrgBody].wn, RelPosB, vb);
         QTxV(qbN, CV->T, pn);
         QTxV(qbN, vb, vn);
         for (i = 0; i < 3; i++) {
            pn[i] += TrgSB[CV->TrgBody].pn[i] - pcmn[i];
            vn[i] += TrgSB[CV->TrgBody].vn[i];
         }

         if (TrgOrb == RefOrb) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = TrgState->PosR[i] + pn[i] - state->PosR[i];
               RelVelN[i] = TrgState->VelR[i] + vn[i] - state->VelR[i];
            }
         }
         else if (TrgOrb->World == RefOrb->World) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = TrgState->PosN[i] + pn[i] - state->PosN[i];
               RelVelN[i] = TrgState->VelN[i] + vn[i] - state->VelN[i];
            }
         }
         else {
            for (i = 0; i < 3; i++) {
               pn[i] += TrgState->PosN[i];
               vn[i] += TrgState->VelN[i];
            }
            struct WorldType *TrgW = &World[TrgOrb->World];
            MTxV(TrgW->CNH, pn, RelPosH);
            MTxV(TrgW->CNH, vn, RelVelH);
            struct WorldType *W = &World[RefOrb->World];
            MTxV(W->CNH, state->PosN, ph);
            MTxV(W->CNH, state->VelN, ph);
            for (i = 0; i < 3; i++) {
               RelPosH[i] -= ph[i];
               RelVelH[i] -= vh[i];
               RelPosH[i] += (TrgW->PosH[i] - W->PosH[i]);
               RelVelH[i] += (TrgW->VelH[i] - W->VelH[i]);
            }
            MxV(W->CNH, RelPosH, RelPosN);
            MxV(W->CNH, RelVelH, RelVelN);
         }
         CopyUnitV(RelPosN, CV->N);
         DSM_RelMotionToAngRate(RelPosN, RelVelN, CV->wn);
      } break;
      case TARGET_VELOCITY:
         for (i = 0; i < 3; i++)
            CV->N[i] = state->VelN[i];
         UNITV(CV->N);
         break;
      case TARGET_MAGFIELD:
         for (i = 0; i < 3; i++)
            CV->N[i] = state->bvn[i];
         UNITV(CV->N);
         break;
      case TARGET_TDRS:
         CV->N[0] = 0.0;
         CV->N[1] = 0.0;
         CV->N[2] = 1.0;
         for (i = 0; i < 3; i++)
            CV->wn[i] = 0.0;
         MaxToS = -2.0; /* Bogus */
         CopyUnitV(state->PosN, Rhat);
         /* Aim at TDRS closest to Zenith */
         for (It = 0; It < 10; It++) {
            if (Tdrs[It].Exists) {
               for (i = 0; i < 3; i++)
                  RelPosN[i] = Tdrs[It].PosN[i] - state->PosN[i];
               UNITV(RelPosN);
               ToS = VoV(RelPosN, Rhat);
               if (ToS > MaxToS) {
                  MaxToS = ToS;
                  for (i = 0; i < 3; i++)
                     CV->N[i] = RelPosN[i];
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
   if (Cmd->TranslationCtrlActive == FALSE || Cmd->ManeuverMode != INACTIVE)
      return;

   long i, Isc_Ref, goodOriginFrame = FALSE;
   long frame_body, origin_body;
   struct DSMCtrlType *CTRL   = &DSM->DsmCtrl;
   struct DSMStateType *state = &DSM->state;

   // Convert Disp vec into N/R coords.
   switch (Cmd->RefFrame[0]) {
      case 'F': {
         double wfn[3] = {0.0};
         MTxV(F->CN, Cmd->Pos, CTRL->CmdPosR); // Convert F to R Inertial
         switch (F->FixedInFrame) {
            case 'L': {
               // L rotates wrt R
               for (i = 0; i < 3; i++)
                  wfn[i] = DSM->refOrb->wln[i];
               goodOriginFrame = TRUE;
            } break;
            case 'N': {
               // R does not rotate wrt R Inertial
               for (i = 0; i < 3; i++)
                  wfn[i] = 0.0;
               goodOriginFrame = TRUE;
            } break;
            default: {
               printf("Invalid Formation fixed frame. How did this happen? "
                      "Exiting...\n");
               exit(EXIT_FAILURE);
            } break;
         }
         VxV(wfn, CTRL->CmdPosR, CTRL->CmdVelR);
      } break;
      case 'N': {
         for (i = 0; i < 3; i++)
            CTRL->CmdPosR[i] = Cmd->Pos[i]; // Already in R Inertial
         for (i = 0; i < 3; i++)
            CTRL->CmdVelR[i] = 0.0; // R does not rotate wrt R Inertial
         goodOriginFrame = TRUE;
      } break;
      case 'L': {
         // Convert LVLH to R Inertial
         MTxV(DSM->refOrb->CLN, Cmd->Pos, CTRL->CmdPosR);
         VxV(DSM->refOrb->wln, CTRL->CmdPosR, CTRL->CmdVelR);
         goodOriginFrame = TRUE;
      } break;
      case 'E': {
         // Hailey's EH Code Begin ****************************************
         double cmd_pos_EH[3] = {0.0};
         double cmd_vel_EH[3] = {0.0};
         double wln[3]        = {0.0};

         double n = sqrt(DSM->refOrb->mu / pow(DSM->refOrb->SMA, 3));

         if (!strcmp(Cmd->TranslationType, "Position")) {
            cmd_pos_EH[0] = -Cmd->Distance * cos(Cmd->Phase) / 2;
            cmd_pos_EH[1] = Cmd->Distance * sin(Cmd->Phase);
            cmd_pos_EH[2] = Cmd->Distance * sqrt(3) * cos(Cmd->Phase) / 2;

            cmd_vel_EH[0] = Cmd->Distance * n * sin(Cmd->Phase) / 2;
            cmd_vel_EH[1] = Cmd->Distance * n * cos(Cmd->Phase);
            cmd_vel_EH[2] = -Cmd->Distance * n * sqrt(3) * sin(Cmd->Phase) / 2;
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
                  printf(
                      "Spacecraft %ld called Euler Hill guidance law on itself."
                      "Exiting...\n",
                      Isc_Ref);
                  exit(EXIT_FAILURE);
               }
               /* Calculate coefficients */
               double tau_k = n * Cmd->CurrentTimer + Cmd->Phase;
               for (i = 0; i < 3; i++)
                  wln[i] = DSM->refOrb->wln[i];

               /* CW equations (note: vel. is incorrect in paper) */
               cmd_pos_EH[0] = -Cmd->Distance * cos(tau_k) / 2;
               cmd_pos_EH[1] = Cmd->Distance * sin(tau_k);
               cmd_pos_EH[2] = Cmd->Distance * sqrt(3) * cos(tau_k) / 2;

               cmd_vel_EH[0] = Cmd->Distance * n * sin(tau_k) / 2;
               cmd_vel_EH[1] = Cmd->Distance * n * cos(tau_k);
               cmd_vel_EH[2] = -Cmd->Distance * n * sqrt(3) * sin(tau_k) / 2;
            }
            else {
               printf("Invalid Translational Control Reference Frame. "
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
                  printf("Spacecraft %ld called Euler Hill guidance law on "
                         "itself. "
                         "Exiting...\n",
                         Isc_Ref);
                  exit(EXIT_FAILURE);
               }

               if (Cmd->ResetTimer == 1) {
                  Cmd->InitTime   = state->Time;
                  Cmd->ResetTimer = 0;
                  /* R Interial -> LVLH */
                  MxV(SC[Isc_Ref].CLN, state->PosR, Cmd->Pos);

                  Cmd->PosRate[1] =
                      ((6 * Cmd->Pos[0] *
                            (n * Cmd->TimeDock - sin(n * Cmd->TimeDock)) -
                        Cmd->Pos[1]) *
                           n * sin(n * Cmd->TimeDock) -
                       2 * n * Cmd->Pos[0] * (4 - 3 * cos(n * Cmd->TimeDock)) *
                           (1 - cos(n * Cmd->TimeDock))) /
                      ((4 * sin(n * Cmd->TimeDock) - 3 * n * Cmd->TimeDock) *
                           sin(n * Cmd->TimeDock) +
                       4 * pow(1 - cos(n * Cmd->TimeDock), 2));
                  Cmd->PosRate[0] =
                      -(n * Cmd->Pos[0] * (4 - 3 * cos(n * Cmd->TimeDock)) +
                        2 * (1 - cos(n * Cmd->TimeDock)) * Cmd->PosRate[1]) /
                      sin(n * Cmd->TimeDock);
                  Cmd->PosRate[2] = -Cmd->Pos[2] * n / tan(n * Cmd->TimeDock);
               }
               for (i = 0; i < 3; i++)
                  wln[i] = DSM->refOrb->wln[i];
               Cmd->CurrentTimer = state->Time - Cmd->InitTime;
               if (Cmd->CurrentTimer <= Cmd->TimeDock) {
                  /* Update Position */
                  cmd_pos_EH[0] =
                      (Cmd->PosRate[0] / n) * sin(n * Cmd->CurrentTimer) -
                      (3 * Cmd->Pos[0] + 2 * Cmd->PosRate[1] / n) *
                          cos(n * Cmd->CurrentTimer) +
                      4 * Cmd->Pos[0] + 2 * Cmd->PosRate[1] / n;
                  cmd_pos_EH[1] =
                      (6 * Cmd->Pos[0] + 4 * Cmd->PosRate[1] / n) *
                          sin(n * Cmd->CurrentTimer) +
                      (2 * Cmd->PosRate[0] / n) * cos(n * Cmd->CurrentTimer) -
                      (6 * n * Cmd->Pos[0] + 3 * Cmd->PosRate[1]) *
                          Cmd->CurrentTimer +
                      Cmd->Pos[1] - 2 * Cmd->PosRate[0] / n;
                  cmd_pos_EH[2] =
                      Cmd->Pos[2] * cos(n * Cmd->CurrentTimer) +
                      (Cmd->PosRate[2] / n) * sin(n * Cmd->CurrentTimer);
                  /* Update Velocity */
                  cmd_vel_EH[0] = Cmd->PosRate[0] * cos(n * Cmd->CurrentTimer) +
                                  (3 * n * Cmd->Pos[0] + 2 * Cmd->PosRate[1]) *
                                      sin(n * Cmd->CurrentTimer);
                  cmd_vel_EH[1] =
                      (6 * n * Cmd->Pos[0] + 4 * Cmd->PosRate[1]) *
                          cos(n * Cmd->CurrentTimer) -
                      (2 * Cmd->PosRate[0]) * sin(n * Cmd->CurrentTimer) -
                      (6 * n * Cmd->Pos[0] + 3 * Cmd->PosRate[1]);
                  cmd_vel_EH[2] =
                      (-Cmd->Pos[2] * n) * sin(n * Cmd->CurrentTimer) +
                      Cmd->PosRate[2] * cos(n * Cmd->CurrentTimer);
               }
               else { // arrived at docking location
                  for (i = 0; i < 3; i++) {
                     cmd_pos_EH[i] = 0;
                     cmd_vel_EH[i] = 0;
                  }
               }
            }
            else {
               printf("Invalid Translational Control Reference Frame. "
                      "Exiting...\n");
               exit(EXIT_FAILURE);
            }
         }
         /* LVLH -> R Inertial */
         MTxV(DSM->refOrb->CLN, cmd_pos_EH, CTRL->CmdPosR);
         VxV(wln, CTRL->CmdPosR, CTRL->CmdVelR);
         double temp[3];
         MTxV(DSM->refOrb->CLN, cmd_vel_EH, temp);
         for (i = 0; i < 3; i++)
            CTRL->CmdVelR[i] += temp[i];
      } break;
      default: {
         // Decode ref SC ID Number
         if (sscanf(Cmd->RefFrame, "SC[%ld].B[%ld]", &Isc_Ref, &frame_body) ==
             2) {
            if (Isc_Ref == DSM->ID) {
               printf("SC[%ld] is attempting to translate relative to its "
                      "own body frame. Exiting...\n",
                      DSM->ID);
               exit(EXIT_FAILURE);
            }
            // Specify disp from OP, in SC B frame directions, control to OP
            if (Isc_Ref >= Nsc) {
               printf("This mission only has %ld spacecraft, but spacecraft "
                      "%ld was attempted to be set as the reference frame. "
                      "Exiting...\n",
                      Nsc, Isc_Ref);
               exit(EXIT_FAILURE);
            }
            if (frame_body >= SC[Isc_Ref].Nb) {
               printf("Spacecraft %ld only has %ld bodies, but the reference "
                      "frame was attempted to be set as body %ld. Exiting...\n",
                      Isc_Ref, SC[Isc_Ref].Nb, frame_body);
               exit(EXIT_FAILURE);
            }
            // TODO: don't use other sc truth
            double qbn[4] = {0.0}, wbn[3] = {0.0};
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
               double qbB[4] = {0.0}, wBnb[3] = {0.0}, wBnbAC[3] = {0.0};
               QxQT(TrgSB[frame_body].qn, TrgSB[0].qn, qbB);
               QxQ(qbB, TrgState->qbn, qbn);

               // get angular velocity of body relative to B[0], then apply
               // this to AC.wbn; all in B[frame_body] frame
               QxV(qbB, TrgSB[0].wn, wBnb);
               QxV(qbB, TrgState->wbn, wBnbAC);
               for (i = 0; i < 3; i++) {
                  // TODO: double check what BodyType::wn actually is
                  wbn[i] = wBnbAC[i] + (TrgSB[frame_body].wn[i] - wBnb[i]);
               }
            }
            else {
               for (i = 0; i < 3; i++) {
                  qbn[i] = TrgState->qbn[i];
                  wbn[i] = TrgState->wbn[i];
               }
               qbn[3] = TrgState->qbn[3];
            }
            // angular velocity of trgDSM wrt N expressed in N
            double wbnn[3] = {0.0};
            // Convert SC# B to R Inertial
            QTxV(qbn, Cmd->Pos, CTRL->CmdPosR);
            QTxV(qbn, wbn, wbnn); // SC rotates wrt R
            VxV(wbnn, CTRL->CmdPosR, CTRL->CmdVelR);
            goodOriginFrame = TRUE;
         }
         else {
            printf("Invalid Translational Control Reference Frame. "
                   "Exiting...\n");
            exit(EXIT_FAILURE);
         }
      } break;
   }

   if (!strcmp(Cmd->RefOrigin, "OP")) {
      // Specify disp from OP, in X frame directions, control to OP
      // Add pos of F frame origin in R frame
      for (i = 0; i < 3; i++)
         CTRL->CmdPosR[i] += F->PosR[i];
      goodOriginFrame = TRUE;
   }
   else if (!strncmp(Cmd->RefOrigin, "SC", 2)) {
      // Specify disp from SC, in X frame directions, control to SC
      // Add pos of SC in R frame
      sscanf(Cmd->RefOrigin, "SC[%ld].B[%ld]", &Isc_Ref,
             &origin_body); // Decode ref SC ID Number
      if (Isc_Ref >= Nsc) {
         printf("This mission only has %ld spacecraft, but spacecraft %ld was "
                "attempted to be set as the reference origin. Exiting...\n",
                Nsc, Isc_Ref);
         exit(EXIT_FAILURE);
      }
      if (origin_body >= SC[Isc_Ref].Nb) {
         printf("Spacecraft %ld only has %ld bodies, but the reference origin "
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
      for (i = 0; i < 3; i++) {
         CTRL->CmdPosR[i] += TrgState->PosR[i] + TrgSB[origin_body].pn[i];
         CTRL->CmdVelR[i] += TrgState->VelR[i] + TrgSB[origin_body].vn[i];
      }
      goodOriginFrame = TRUE;
   }
   else {
      goodOriginFrame = FALSE;
   }

   if (goodOriginFrame == FALSE) {
      printf("Invalid Ref origin/frame combo %s/%s in Translation Command "
             "at %lf. Exiting...\n",
             Cmd->RefOrigin, Cmd->RefFrame, SimTime);
      exit(EXIT_FAILURE);
   }
   for (i = 0; i < 3; i++) {
      CTRL->CmdPosN[i] = CTRL->CmdPosR[i] + state->PosN[i];
      CTRL->CmdVelN[i] = CTRL->CmdVelR[i] + state->VelN[i];
   }
   for (i = 0; i < 3; i++) {
      CTRL->trn_kp[i]   = Cmd->trn_kp[i];
      CTRL->trn_kr[i]   = Cmd->trn_kr[i];
      CTRL->trn_ki[i]   = Cmd->trn_ki[i];
      CTRL->FrcB_max[i] = Cmd->FrcB_max[i];
      CTRL->vel_max[i]  = Cmd->vel_max[i];
   }
}
//------------------------------------------------------------------------------
long getCmdVecs(struct DSMType *DSM, struct FormationType *F,
                struct DSMCmdVecType *vec, const char *attRefFrame,
                struct DSMStateType *state, double cmdVecB[3],
                double cmdVecN[3])
{
   switch (vec->TrgType) {
      case TARGET_SC:
      case TARGET_WORLD: {
         // to get PV->wn, PV->N (in F Frame)
         FindDsmCmdVecN(DSM, vec);
         // (Converting Cmd vec to body frame)
         QxV(state->qbn, vec->N, cmdVecB);
      } break;
      case TARGET_VEC: {
         switch (attRefFrame[0]) {
            case 'N': {
               // (Converting Cmd vec to body frame)
               QxV(state->qbn, vec->cmd_vec, cmdVecB);
            } break;
            case 'F': {
               // (Converting to Inertial frame)
               MTxV(F->CN, vec->cmd_vec, cmdVecN);
               // (Converting to body frame)
               QxV(state->qbn, cmdVecN, cmdVecB);
            } break;
            case 'L': {
               // (Converting to LVLH to Inertial frame)
               MTxV(DSM->refOrb->CLN, vec->cmd_vec, cmdVecN);
               // (Converting to body frame)
               QxV(state->qbn, cmdVecN, cmdVecB);
            } break;
            case 'M': {
               /* Magnetic field frame                               */
               /*    x: magnetic field line                          */
               /*    y: radial cross magnetic field                  */
               /*    z: completes  triad                             */
               double CbN[3][3] = {{0.0}};
               for (int i = 0; i < 3; i++)
                  CbN[0][i] = state->bvn[i];
               UNITV(CbN[0]);
               VxV(state->PosN, CbN[0], CbN[1]);
               UNITV(CbN[1]);
               VxV(CbN[0], CbN[1], CbN[2]);
               UNITV(CbN[2]);
               // (Converting from magnetic frame to Inertial frame)
               MTxV(CbN, vec->cmd_vec, cmdVecN);
               // (Converting to body frame)
               QxV(state->qbn, cmdVecN, cmdVecB);
            } break;
            default: {
               long frame_body = 0;
               long Isc_Ref;
               // Decode ref SC ID Number
               if (sscanf(attRefFrame, "SC[%ld].B[%ld]", &Isc_Ref,
                          &frame_body) == 2) {
                  if (Isc_Ref == DSM->ID) {
                     printf("SC[%ld] is attempting to point relative "
                            "to its own body frame. Exiting...\n",
                            DSM->ID);
                     exit(EXIT_FAILURE);
                  }
                  if (Isc_Ref >= Nsc) {
                     printf("This mission only has %ld spacecraft, "
                            "but spacecraft %ld was attempted to be "
                            "set as the reference frame. Exiting...\n",
                            Nsc, Isc_Ref);
                     exit(EXIT_FAILURE);
                  }
                  if (frame_body >= SC[Isc_Ref].Nb) {
                     printf("Spacecraft %ld only has %ld bodies, but "
                            "the reference frame was attempted to be "
                            "set as body %ld. Exiting...\n",
                            Isc_Ref, SC[Isc_Ref].Nb, frame_body);
                     exit(EXIT_FAILURE);
                  }
                  // TODO: don't use other sc truth
                  double qbn[4]                 = {0.0};
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
                     double qbB[4] = {0.0};
                     QxQT(TrgSB[frame_body].qn, TrgSB[0].qn, qbB);
                     QxQ(qbB, TrgState->qbn, qbn);
                  }
                  else {
                     for (int i = 0; i < 3; i++) {
                        qbn[i] = TrgState->qbn[i];
                     }
                     qbn[3] = TrgState->qbn[3];
                  }
                  // rotation from trg Body to DSM body frame
                  double qbbs[4] = {0.0};
                  QxQT(state->qbn, qbn, qbbs);
                  QxV(qbbs, vec->cmd_vec, cmdVecB);
               }
               else {
                  printf("Invalid attitude reference frame for "
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
   UNITV(cmdVecB);
   QTxV(state->qbn, cmdVecB, cmdVecN);
   UNITV(cmdVecN);
   return TRUE;
}
//------------------------------------------------------------------------------
void AttitudeGuidance(struct DSMType *DSM, struct FormationType *F)
{
   struct DSMCmdType *Cmd = &DSM->Cmd;
   if (Cmd->AttitudeCtrlActive == FALSE)
      return;

   long i, target_num;
   double qfn[4], qrn[4], qfl[4], qrf[4];
   struct DSMCtrlType *CTRL   = &DSM->DsmCtrl;
   struct DSMStateType *state = &DSM->state;

   switch (Cmd->Method) {
      case (PARM_VECTORS): {
         double cmdVecB[2][3]          = {{0.0}};
         double cmdVecN[2][3]          = {{0.0}};
         struct DSMCmdVecType *vecs[2] = {&Cmd->PriVec, &Cmd->SecVec};
         char *attRefFrame[2] = {Cmd->PriAttRefFrame, Cmd->SecAttRefFrame};
         double C_tb[3][3], C_tn[3][3], dC[3][3];
         double q_tb[4] = {0, 0, 0, 1}, q_tn[4] = {0, 0, 0, 1}, qbn_cmd[4];
         for (int k = 0; k < 2; k++) {
            if (!getCmdVecs(DSM, F, vecs[k], attRefFrame[k], state, cmdVecB[k],
                            cmdVecN[k])) {
               printf("Invalid Target type for %s vector. Exiting...\n",
                      k == 0 ? "Primary" : "Secondary");
               exit(EXIT_FAILURE);
            }
         }

         /*construct body to target DCM and Inertial to Target DCMS*/
         for (i = 0; i < 3; i++) {
            C_tb[0][i] = vecs[0]->cmd_axis[i]; // = PV->cmd_axis
            C_tn[0][i] = cmdVecN[0][i];        // = PriCmdVec
         }

         if (fabs(VoV(vecs[0]->cmd_axis, vecs[1]->cmd_axis) - 1.0) < EPS_DSM) {
            printf("PV Axis [%lf  %lf  %lf] in %s and SV Axis [%lf  %lf  %lf] "
                   "in %s are parallel, resulting in an infeasible attitude "
                   "command. Exiting...\n",
                   vecs[0]->cmd_axis[0], vecs[0]->cmd_axis[1],
                   vecs[0]->cmd_axis[2], Cmd->PriAttRefFrame,
                   vecs[1]->cmd_axis[0], vecs[1]->cmd_axis[1],
                   vecs[1]->cmd_axis[2], Cmd->SecAttRefFrame);
            exit(EXIT_FAILURE);
         }

         if (fabs(VoV(cmdVecB[0], cmdVecB[1]) - 1.0) < EPS_DSM) {
            char *tgts[2] = {0};
            for (i = 0; i < 2; i++) {
               switch (vecs[i]->TrgType) {
                  case TARGET_SC: {
                     sprintf(tgts[i], "SC[%ld].B[%ld]", vecs[i]->TrgSC,
                             vecs[i]->TrgBody);
                  } break;
                  case TARGET_WORLD: {
                     sprintf(tgts[i],
                             "World %s, Position [%.3le, %.3le, %.3le]",
                             World[vecs[i]->TrgWorld].Name, vecs[i]->W[0],
                             vecs[i]->W[1], vecs[i]->W[2]);
                  } break;
                  case TARGET_VEC: {
                     sprintf(tgts[i], "Vector [%.3le, %.3le, %.3le]",
                             vecs[i]->cmd_vec[0], vecs[i]->cmd_vec[1],
                             vecs[i]->cmd_vec[2]);
                  } break;
                  default:
                     strcpy(tgts[i], "ERROR");
                     break;
               }
            }
            printf("PV Target, %s, and SV Target, %s, are parallel, resulting "
                   "in an infeasible attitude command. Exiting...\n",
                   tgts[0], tgts[1]);
            exit(EXIT_FAILURE);
         }

         VxV(vecs[0]->cmd_axis, vecs[1]->cmd_axis, C_tb[2]);
         VxV(cmdVecN[0], cmdVecN[1], C_tn[2]);
         VxV(C_tb[2], C_tb[0], C_tb[1]);
         VxV(C_tn[2], C_tn[0], C_tn[1]);

         for (i = 0; i < 3; i++) {
            UNITV(C_tb[i]);
            UNITV(C_tn[i]);
         }

         C2Q(C_tb, q_tb);
         C2Q(C_tn, q_tn);

         /* Approximation of log map from SO(3) to so(3) to calculate Cmd->wrn*/
         MTxM(C_tn, Cmd->OldCRN, dC);
         logso3(dC, Cmd->wrn);
         for (i = 0; i < 3; i++)
            Cmd->wrn[i] /= DSM->DT;
         memcpy(Cmd->OldCRN, C_tn, sizeof(Cmd->OldCRN));

         /* Calculate Inertial to Body Quaternion */
         QTxQ(q_tb, q_tn, qbn_cmd);
         UNITQ(qbn_cmd);
         QxQT(state->qbn, qbn_cmd, Cmd->qbr);
      } break;
      case (PARM_AXIS_SPIN): {
         double cmdVecB[3] = {0.0};
         double cmdVecN[3] = {0.0};

         if (!getCmdVecs(DSM, F, &Cmd->PriVec, Cmd->PriAttRefFrame, state,
                         cmdVecB, cmdVecN)) {
            printf("Invalid Target type for Primary Spin vector. Exiting...\n");
            exit(EXIT_FAILURE);
         }

         double therr[3] = {0.0};
         for (i = 0; i < 3; i++)
            therr[i] = cmdVecB[i] - Cmd->PriVec.cmd_axis[i];
         double therr_o_axis = VoV(Cmd->PriVec.cmd_axis, therr);
         for (i = 0; i < 3; i++)
            therr[i] -= Cmd->PriVec.cmd_axis[i] * therr_o_axis;
         double therr_mag = UNITV(therr);
         Cmd->qbr[3]      = cos(therr_mag / 2);
         double tmp       = sqrt(1.0 - Cmd->qbr[3] * Cmd->qbr[3]);
         for (i = 0; i < 3; i++)
            Cmd->qbr[i] = tmp * therr[i];

         QTxV(state->qbn, Cmd->AngRate, Cmd->wrn);
      } break;
      case (PARM_UNITVECTOR): {
         printf("Feature for Singular Primary Unit Vector Pointing not "
                "currently fully implemented. Exiting...\n");
         exit(EXIT_FAILURE);
      } break;
      case (PARM_QUATERNION): {
         switch (Cmd->AttRefFrame[0]) {
            case 'N': {
               for (i = 0; i < 4; i++)
                  Cmd->qrn[i] = Cmd->q[i];
               QxQT(state->qbn, Cmd->qrn, Cmd->qbr);
               for (i = 0; i < 3; i++)
                  Cmd->wrn[i] = 0.0;
            } break;
            case 'F': {
               for (i = 0; i < 4; i++)
                  Cmd->qrf[i] = Cmd->q[i];
               C2Q(F->CN, qfn);
               QxQ(Cmd->qrf, qfn, qrn);
               QxQT(state->qbn, qrn, Cmd->qbr);
               switch (F->FixedInFrame) {
                  case 'L': {
                     // F rotates wrt N
                     for (i = 0; i < 3; i++)
                        Cmd->wrn[i] = DSM->refOrb->wln[i];
                  } break;
                  case 'N': {
                     // N does not rotate wrt N Inertial
                     for (i = 0; i < 3; i++)
                        Cmd->wrn[i] = 0.0;
                  } break;
                  default: {
                     printf("Invalid Formation fixed frame. How did this "
                            "happen? Exiting...\n");
                     exit(EXIT_FAILURE);
                  } break;
               }
            } break;
            case 'L': {
               for (i = 0; i < 4; i++)
                  Cmd->qrl[i] = Cmd->q[i];
               C2Q(F->CL, qfl);
               QxQT(Cmd->qrl, qfl, qrf);
               C2Q(F->CN, qfn);
               QxQ(qrf, qfn, qrn);
               QxQT(state->qbn, qrn, Cmd->qbr);
               for (i = 0; i < 3; i++)
                  Cmd->wrn[i] = DSM->refOrb->wln[i];
            } break;
            default: {
               long frame_body = 0;
               long Isc_Ref;
               // Decode ref SC ID Number
               if (sscanf(Cmd->AttRefFrame, "SC[%ld].B[%ld]", &Isc_Ref,
                          &frame_body) == 2) {
                  if (Isc_Ref == DSM->ID) {
                     printf("SC[%ld] is attempting to point relative to its "
                            "own body frame. Exiting...\n",
                            DSM->ID);
                     exit(EXIT_FAILURE);
                  }
                  if (Isc_Ref >= Nsc) {
                     printf("This mission only has %ld spacecraft, but "
                            "spacecraft %ld was attempted to be set as the "
                            "reference frame. Exiting...\n",
                            Nsc, Isc_Ref);
                     exit(EXIT_FAILURE);
                  }
                  if (frame_body >= SC[Isc_Ref].Nb) {
                     printf("Spacecraft %ld only has %ld bodies, but the "
                            "reference frame was attempted to be set as body "
                            "%ld. Exiting...\n",
                            Isc_Ref, SC[Isc_Ref].Nb, frame_body);
                     exit(EXIT_FAILURE);
                  }
                  // TODO: don't use other sc truth
                  double qbn[4]                 = {0.0};
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
                     double qbB[4] = {0.0};
                     QxQT(TrgSB[frame_body].qn, TrgSB[0].qn, qbB);
                     QxQ(qbB, TrgState->qbn, qbn);
                  }
                  else {
                     for (i = 0; i < 3; i++) {
                        qbn[i] = TrgState->qbn[i];
                     }
                     qbn[3] = TrgState->qbn[3];
                  }
                  // rotation from trg Body to DSM body frame
                  double qbbs[4] = {0.0};
                  QxQT(state->qbn, qbn, qbbs);
                  QxQT(qbbs, Cmd->q, Cmd->qbr);
               }
               else {
                  printf("Invlaid attitude reference frame for quaternion. "
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
            printf(
                "This mission only has %ld spacecraft, but spacecraft %ld was "
                "attempted to be set as the spacecraft to mirror. Exiting...\n",
                Nsc, Isc_Ref);
            exit(EXIT_FAILURE);
         }
         if (target_num >= SC[Isc_Ref].Nb) {
            printf("Spacecraft %ld only has %ld bodies, but the mirror target "
                   "was attempted to be set as body %ld. Exiting...\n",
                   Isc_Ref, SC[Isc_Ref].Nb, target_num);
            exit(EXIT_FAILURE);
         }
         // TODO: not truth of other body
         double qbn[4] = {0.0}, wbn[3] = {0.0};
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
            double qbB[4] = {0.0}, wBnb[3] = {0.0}, wBnbDSM[3] = {0.0};
            QxQT(TrgSB[target_num].qn, TrgSB[0].qn, qbB);
            QxQ(qbB, TrgState->qbn, qbn);

            // get relative angular velocity of body to B[0], then apply this
            // to AC.wbn; all in B[frame_body] frame
            QxV(qbB, TrgSB[0].wn, wBnb);
            QxV(qbB, TrgState->wbn, wBnbDSM);
            for (i = 0; i < 3; i++)
               wbn[i] = wBnbDSM[i] + (TrgSB[target_num].wn[i] - wBnb[i]);
         }
         else {
            for (i = 0; i < 3; i++) {
               qbn[i] = TrgState->qbn[i];
               wbn[i] = TrgState->wbn[i];
            }
            qbn[3] = TrgState->qbn[3];
         }
         QxQT(state->qbn, qbn, Cmd->qbr);
         QTxV(TrgState->qbn, wbn, Cmd->wrn);
      } break;
      case (PARM_DETUMBLE): {
         for (i = 0; i < 3; i++)
            Cmd->qbr[i] = 0;
         Cmd->qbr[3] = 1;
         for (i = 0; i < 3; i++)
            Cmd->wrn[i] = 0.0;
      } break;
      default:
         printf("Invalid Command Method for Attitude Guidance. Exiting...\n");
         exit(EXIT_FAILURE);
         break;
   }

   for (i = 0; i < 4; i++)
      CTRL->qbr[i] = Cmd->qbr[i];
   for (i = 0; i < 3; i++) {
      CTRL->dmp_kp[i]   = Cmd->dmp_kp[i];
      CTRL->att_kp[i]   = Cmd->att_kp[i];
      CTRL->att_kr[i]   = Cmd->att_kr[i];
      CTRL->att_ki[i]   = Cmd->att_ki[i];
      CTRL->Trq_max[i]  = Cmd->Trq_max[i];
      CTRL->dTrq_max[i] = Cmd->dTrq_max[i];
      CTRL->w_max[i]    = Cmd->w_max[i];
   }
}
//------------------------------------------------------------------------------
//                                NAVIGATION
//------------------------------------------------------------------------------
void NavigationModule(struct AcType *const AC, struct DSMType *const DSM)
{
   struct DSMStateType *state = &DSM->state;

   state->Time = AC->Time;
   for (int i = 0; i < 3; i++) {
      state->PosN[i] = AC->PosN[i];
      state->VelN[i] = AC->VelN[i];
      state->PosR[i] = state->PosN[i] - DSM->refOrb->PosN[i];
      state->VelR[i] = state->VelN[i] - DSM->refOrb->VelN[i];

      for (int j = 0; j < 3; j++)
         state->CBN[i][j] = AC->CBN[i][j];
      state->qbn[i] = AC->qbn[i];
      state->wbn[i] = AC->wbn[i];

      state->svb[i] = AC->svb[i];
      state->svn[i] = AC->svn[i];
      state->bvb[i] = AC->bvb[i];
      state->bvn[i] = AC->bvn[i];
   }
   state->qbn[3] = AC->qbn[3];
   return;
}
void TranslationalNavigation(struct AcType *AC, struct DSMStateType *state)
{
   for (int i = 0; i < 3; i++) {
      AC->PosN[i] = state->PosN[i];
      AC->VelN[i] = state->VelN[i];
   }
}
//------------------------------------------------------------------------------
void AttitudeNavigation(struct AcType *AC, struct DSMStateType *state)
{
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++)
         AC->CBN[i][j] = state->CBN[i][j];
      AC->qbn[i] = state->qbn[i];
      AC->wbn[i] = state->wbn[i];
   }
   AC->qbn[3] = state->qbn[3];
}
//------------------------------------------------------------------------------
void MurAKF(struct AcType *AC, struct DSMStateType *state)
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
   long i;

   struct DSMCtrlType *CTRL   = &DSM->DsmCtrl;
   struct DSMCmdType *Cmd     = &DSM->Cmd;
   struct DSMStateType *state = &DSM->state;

   if (Cmd->TranslationCtrlActive == TRUE && Cmd->ManeuverMode == INACTIVE) {
      switch (Cmd->trn_controller) {
         case PID_CNTRL: {
            // PID Controller
            if (Cmd->NewTrnGainsProcessed == TRUE) {
               for (i = 0; i < 3; i++)
                  DSM->trn_ei[i] = 0.0;
               Cmd->NewTrnGainsProcessed = FALSE;
            }

            for (i = 0; i < 3; i++) {
               DSM->perr[i] =
                   state->PosR[i] - CTRL->CmdPosR[i]; // Position Error
               DSM->verr[i] =
                   state->VelR[i] - CTRL->CmdVelR[i]; // Velocity Error
               DSM->trn_ei[i] += (DSM->Oldperr[i] + DSM->perr[i]) / 2.0 *
                                 DSM->DT; // Integrated Error

               if (fabs(Cmd->trn_kilimit[i]) > EPS_DSM &&
                   fabs(CTRL->trn_ki[i]) > EPS_DSM)
                  DSM->trn_ei[i] = Limit(
                      DSM->trn_ei[i], -Cmd->trn_kilimit[i] / CTRL->trn_ki[i],
                      Cmd->trn_kilimit[i] /
                          CTRL->trn_ki[i]); // limits integrated error to
                                            // limit/ki, since limit is given
                                            // in terms of force

               CTRL->u1[i] = CTRL->trn_kp[i] / CTRL->trn_kr[i] * DSM->perr[i];
               if (CTRL->vel_max[i] > 0) {
                  CTRL->u1[i] =
                      Limit(CTRL->u1[i], -CTRL->vel_max[i], CTRL->vel_max[i]);
               }
               CTRL->FcmdN[i] =
                   -CTRL->trn_kr[i] * (CTRL->u1[i] + DSM->verr[i]) -
                   CTRL->trn_ki[i] * DSM->trn_ei[i];
            }
            QxV(state->qbn, CTRL->FcmdN,
                CTRL->FcmdB); // Converting from Inertial to body frame for
                              // Report
            for (i = 0; i < 3; i++) {
               if (CTRL->FrcB_max[i] > 0)
                  CTRL->FcmdB[i] = Limit(
                      CTRL->FcmdB[i], -CTRL->FrcB_max[i],
                      CTRL->FrcB_max[i]); // Limiting AC->Frc in body frame
            }
            QTxV(state->qbn, CTRL->FcmdB, CTRL->FcmdN);
         } break;
         case LYA_2BODY_CNTRL: {
            // Calculate relative radius, velocity
            for (i = 0; i < 3; i++) {
               DSM->perr[i] = state->PosR[i] -
                              CTRL->CmdPosR[i]; // Position Error, Relative
               DSM->verr[i] =
                   state->VelR[i] - CTRL->CmdVelR[i]; // Velocity Error
            }

            double r_norm  = MAGV(state->PosN);
            double r_cntrl = MAGV(CTRL->CmdPosN);
            double mu      = DSM->refOrb->mu;

            double dg[3];
            for (i = 0; i < 3; i++)
               dg[i] = -mu / pow(r_norm, 3) * state->PosN[i] +
                       mu / pow(r_cntrl, 3) * CTRL->CmdPosN[i];

            for (i = 0; i < 3; i++) {
               CTRL->FcmdN[i] = -CTRL->trn_kp[i] * DSM->perr[i] -
                                CTRL->trn_kr[i] * DSM->verr[i] -
                                dg[i] * DSM->mass;
            }
            QxV(state->qbn, CTRL->FcmdN,
                CTRL->FcmdB); // Converting from Inertial to body frame for
                              // Report
            for (i = 0; i < 3; i++) {
               if (CTRL->FrcB_max[i] > 0)
                  CTRL->FcmdB[i] = Limit(
                      CTRL->FcmdB[i], -CTRL->FrcB_max[i],
                      CTRL->FrcB_max[i]); // Limiting AC->Frc in body frame
            }
            QTxV(state->qbn, CTRL->FcmdB, CTRL->FcmdN);
         } break;
         default:
            printf("Invalid Translational Controller type. Exiting...\n");
            exit(EXIT_FAILURE);
            break;
      }
      for (i = 0; i < 3; i++)
         DSM->Oldperr[i] = DSM->perr[i];
   }
   else if (Cmd->TranslationCtrlActive == TRUE &&
            Cmd->ManeuverMode != INACTIVE) {
      if (SimTime < Cmd->BurnStopTime) {
         switch (Cmd->ManeuverMode) {
            case CONSTANT: {
               if (!strcmp(Cmd->RefFrame, "N")) {
                  for (i = 0; i < 3; i++) {
                     CTRL->FcmdN[i] =
                         DSM->mass * Cmd->DeltaV[i] / Cmd->BurnTime;
                  }
                  QxV(state->qbn, CTRL->FcmdN,
                      CTRL->FcmdB); // Converting from Inertial to body frame
                                    // for Report
               }
               else if (!strcmp(Cmd->RefFrame, "B")) {
                  for (i = 0; i < 3; i++) {
                     CTRL->FcmdB[i] =
                         DSM->mass * Cmd->DeltaV[i] / Cmd->BurnTime;
                  }
               }
               for (i = 0; i < 3; i++) {
                  if (CTRL->FrcB_max[i] > 0) {
                     CTRL->FcmdB[i] = Limit(
                         CTRL->FcmdB[i], -CTRL->FrcB_max[i],
                         CTRL->FrcB_max[i]); // Limiting AC->Frc in body frame
                  }
               }
               QTxV(state->qbn, CTRL->FcmdB,
                    CTRL->FcmdN); // Converting back to Inertial from body frame
            } break;
            case SMOOTHED: {
               const double coef =
                   -2 *
                   atanh(-0.99998); // .99998 corresponds to capturing 99.999%
                                    // of the burn since tanh has an asymptote
               double sharp = coef / Cmd->BurnTime;
               double t_mid = Cmd->BurnStopTime - Cmd->BurnTime / 2.0;
               double t_since_mid =
                   SimTime - t_mid; // Time elapsed since middle of burn
               double coshSharp  = cosh(sharp * t_since_mid);
               double coshSharp2 = coshSharp * coshSharp;

               if (!strcmp(Cmd->RefFrame, "N")) {
                  for (i = 0; i < 3; i++) {
                     CTRL->FcmdN[i] = DSM->mass *
                                      (Cmd->DeltaV[i] * sharp / 2.0) /
                                      coshSharp2;
                     CTRL->FcmdN[i] = DSM->mass *
                                      (Cmd->DeltaV[i] * sharp / 2.0) /
                                      coshSharp2;
                  }
                  QxV(state->qbn, CTRL->FcmdN,
                      CTRL->FcmdB); // Converting from Inertial to body frame
                                    // for Report
               }
               else if (!strcmp(Cmd->RefFrame, "B")) {
                  for (i = 0; i < 3; i++) {
                     CTRL->FcmdB[i] = DSM->mass *
                                      (Cmd->DeltaV[i] * sharp / 2.0) /
                                      coshSharp2;
                  }
               }
               for (i = 0; i < 3; i++) {
                  if (CTRL->FrcB_max[i] > 0) {
                     CTRL->FcmdB[i] = Limit(
                         CTRL->FcmdB[i], -CTRL->FrcB_max[i],
                         CTRL->FrcB_max[i]); // Limiting AC->Frc in body frame
                  }
               }
               QTxV(state->qbn, CTRL->FcmdB,
                    CTRL->FcmdN); // Converting back to Inertial from body frame
            } break;
            default:
               printf("Invalid maneuver mode. Exiting...\n");
               exit(EXIT_FAILURE);
               break;
         }
      }
      else {
         Cmd->ManeuverMode          = INACTIVE;
         Cmd->TranslationCtrlActive = FALSE;
         for (i = 0; i < 3; i++) {
            CTRL->FcmdN[i] = 0;
            CTRL->FcmdB[i] = 0;
         }
      }
   }
   else {
      for (i = 0; i < 3; i++) {
         CTRL->FcmdN[i] = 0;
         CTRL->FcmdB[i] = 0;
      }
   }
   // Assigning CMDs to upper structure
   for (i = 0; i < 3; i++) {
      DSM->FcmdN[i] = CTRL->FcmdN[i];
      DSM->FcmdB[i] = CTRL->FcmdB[i];
   }
}
//------------------------------------------------------------------------------
void AttitudeCtrl(struct DSMType *DSM)
{
   long i;
   double wrb[3];

   struct DSMCtrlType *CTRL   = &DSM->DsmCtrl;
   struct DSMCmdType *Cmd     = &DSM->Cmd;
   struct DSMStateType *state = &DSM->state;

   if (Cmd->AttitudeCtrlActive == TRUE) {
      switch (Cmd->att_controller) {
         case PID_CNTRL: {
            // PID Controller
            if (Cmd->NewAttGainsProcessed == TRUE) {
               for (i = 0; i < 3; i++) {
                  DSM->att_ei[i] = 0.0;
               }
               Cmd->NewAttGainsProcessed = FALSE;
            }

            Q2AngleVec(CTRL->qbr, DSM->therr); // Angular Position Error
            QxV(state->qbn, Cmd->wrn,
                wrb); // Rotate angular velocity into Body frame
            for (i = 0; i < 3; i++) {
               DSM->werr[i] = state->wbn[i] -
                              wrb[i]; // Angular Velocity Error (in body frame)
               DSM->att_ei[i] += (DSM->Oldtherr[i] + DSM->therr[i]) / 2.0 *
                                 DSM->DT; // Integrated angle error

               if (fabs(Cmd->att_kilimit[i]) > EPS_DSM &&
                   CTRL->att_ki[i] > EPS_DSM)
                  DSM->att_ei[i] = Limit(DSM->att_ei[i],
                                         -Cmd->att_kilimit[i] / CTRL->att_ki[i],
                                         Cmd->att_kilimit[i] / CTRL->att_ki[i]);

               CTRL->u2[i] = CTRL->att_kp[i] / CTRL->att_kr[i] * DSM->therr[i];
               if (CTRL->w_max[i] > 0) {
                  CTRL->u2[i] =
                      Limit(CTRL->u2[i], -CTRL->w_max[i], CTRL->w_max[i]);
               }
               CTRL->Tcmd[i] = -CTRL->att_kr[i] * (CTRL->u2[i] + DSM->werr[i]) -
                               CTRL->att_ki[i] * DSM->att_ei[i];
            }
            for (i = 0; i < 3; i++)
               DSM->Oldtherr[i] = DSM->therr[i];
         } break;
         case LYA_ATT_CNTRL: {
            double om_x_I_om[3];

            Q2AngleVec(CTRL->qbr, DSM->therr); // Angular Position Error
            QxV(state->qbn, Cmd->wrn,
                wrb); // Rotate angular velocity into Body frame
            for (i = 0; i < 3; i++)
               DSM->werr[i] = state->wbn[i] -
                              wrb[i]; // Angular Velocity Error (in body frame)
            // calculate nonlinear term in Quaternion Lyapunov stability
            vxMov(DSM->werr, DSM->MOI, om_x_I_om);

            for (i = 0; i < 3; i++) {
               CTRL->Tcmd[i] = -Cmd->att_kp[i] * CTRL->qbr[i] -
                               Cmd->att_kr[i] * DSM->werr[i] + om_x_I_om[i];
            }
         } break;
         default:
            printf("Invalid Attitude controller type. Exiting...\n");
            exit(EXIT_FAILURE);
            break;
      }
      for (i = 0; i < 3; i++) {
         if (CTRL->Trq_max[i] > 0)
            CTRL->Tcmd[i] =
                Limit(CTRL->Tcmd[i], -CTRL->Trq_max[i], CTRL->Trq_max[i]);
      }
   }
   else {
      for (i = 0; i < 3; i++)
         CTRL->Tcmd[i] = 0;
   }
   // Assigning CMDs to upper structure
   for (i = 0; i < 3; i++) {
      DSM->Tcmd[i] = CTRL->Tcmd[i];
      DSM->Mcmd[i] = 0.0; // For now, this is unused, so it needs to be cleared
   }
}
//------------------------------------------------------------------------------
void MomentumDumpCtrl(struct DSMType *DSM, double TotalWhlH[3])
{
   long i;
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
            for (i = 0; i < 3; i++)
               CTRL->dTcmd[i] = -CTRL->dmp_kp[i] * TotalWhlH[i];
            break;
         default:
            printf("Invalid controller type for Momentum Dumping. How did this "
                   "happen? Exiting...\n");
            exit(EXIT_FAILURE);
            break;
      }

      for (i = 0; i < 3; i++) {
         if (CTRL->dTrq_max[i] > 0)
            CTRL->dTcmd[i] =
                Limit(CTRL->dTcmd[i], -CTRL->dTrq_max[i], CTRL->dTrq_max[i]);
      }
   }
   else {
      for (i = 0; i < 3; i++)
         CTRL->dTcmd[i] = 0.0;
   }

   for (i = 0; i < 3; i++)
      DSM->dTcmd[i] = CTRL->dTcmd[i];
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

      for (int i = 0; i < 3;
           i++) { // put place holders in integrator "old" values, set ei
                  // values to zero to initialize integrated error
         DSM->Oldtherr[i] = 0.0;
         DSM->Oldperr[i]  = 0.0;

         DSM->att_ei[i] = 0.0;
         DSM->trn_ei[i] = 0.0;
      }
   }

   if (DSM->CmdNum < DSM->CmdCnt && SimTime >= DSM->CmdNextTime) {
      DsmCmdInterpreterMrk2(AC, DSM, dsmRoot);
      DSM->CmdNum++;
      if (DSM->CmdNum < DSM->CmdCnt)
         fy_node_scanf(DSM->CmdArray[DSM->CmdNum], "/Time %lf",
                       &DSM->CmdNextTime);
   }

   // Generate Data From Sensors
   SensorModule(AC);

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
      double TotalWhlH[3] = {0.0};
      for (int i = 0; i < AC->Nwhl; i++) {
         for (int j = 0; j < 3; j++) {
            TotalWhlH[j] += AC->Whl[i].Axis[j] * AC->Whl[i].H;
         }
      }
      MomentumDumpCtrl(DSM, TotalWhlH);
   }

   // Implement Control Through Actuators
   ActuatorModule(AC, DSM);
}

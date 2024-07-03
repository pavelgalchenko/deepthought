/*    This file is distributed with DeepThought,                      */
/*    a fork of 42,                                                   */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Contributors [aka cool froods]:                                 */
/*    - Daniel Newberry - NASA WFF Intern, Summer 2023                */
/*      drnmvd@mst.edu                                                */
/*    - Jerry Varghese - NASA WFF Intern, Summer 2023                 */
/*      varghes5@purdue.edu                                           */
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
void ThrProcessingMinPower(struct SCType *S)
{
   long i, j;
   double cmdVec[6], distDotCmd;

   struct AcType *AC;

   AC = &S->AC;

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
void InitThrDistVecs(struct AcType *AC, int DOF, long controllerState)
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

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;

   S->InitDSM   = 0;
   DSM->Init    = 1;
   DSM->ID      = S->ID;
   DSM->CmdInit = 1;

   /* Controllers */
   DSM->DsmCtrl.Init         = 1;
   DSM->DsmCtrl.H_DumpActive = FALSE;

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
//----------------------------------- GAINS -----------------------------------
long GetGains(struct SCType *S, struct fy_node *gainsNode, long controllerState)
{
   long GainsProcessed = FALSE;

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   struct AcType *AC;
   int controller = -1;

   DSM        = &S->DSM;
   Cmd        = &DSM->Cmd;
   AC         = &S->AC;
   double *kp = NULL, *kr = NULL, *ki = NULL, *limit_vec = NULL;

   switch (controllerState) {
      case TRN_STATE:
         controller = Cmd->trn_controller;
         kp         = Cmd->trn_kp;
         kr         = Cmd->trn_kr;
         ki         = Cmd->trn_ki;
         limit_vec  = Cmd->trn_kilimit;
         break;
      case ATT_STATE:
         controller = Cmd->att_controller;
         kp         = Cmd->att_kp;
         kr         = Cmd->att_kr;
         ki         = Cmd->att_ki;
         limit_vec  = Cmd->att_kilimit;
         break;
      case FULL_STATE:
         // PLACEHOLDER
         break;
      case DMP_STATE:
         kp = Cmd->dmp_kp;
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
      if (gainsGood && controller == PID_CNTRL)
         GainsProcessed = TRUE;
   }
   else if (!strcmp(gainMode, "PID_WN")) {
      if (fy_node_scanf(gainsDataNode,
                        "/Omega %lf /Zeta %lf /Alpha %lf /Ki_Limit %lf", &omega,
                        &zeta, &alpha, &limit) == 4 &&
          controller == PID_CNTRL) {
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
                  kp[i] *= AC->mass;
                  kr[i] *= AC->mass;
                  ki[i] *= AC->mass;
               }
               break;
            case ATT_STATE:
               for (i = 0; i < 3; i++) {
                  kp[i] *= AC->MOI[i][i];
                  kr[i] *= AC->MOI[i][i];
                  ki[i] *= AC->MOI[i][i];
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
      switch (controller) {
         case LYA_2BODY_CNTRL:
            if (fy_node_sequence_item_count(kNode) == 2) {
               fy_node_scanf(fy_node_sequence_get_by_index(kNode, 0), "/ %lf",
                             &omega);
               fy_node_scanf(fy_node_sequence_get_by_index(kNode, 1), "/ %lf",
                             &zeta);
               for (i = 0; i < 3; i++) {
                  kp[i] = omega * omega * AC->mass;
                  kr[i] = 2 * zeta * omega * AC->mass;
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
                  kr[i] = sqrt(2.0 * k_lya * AC->MOI[i][i]);
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

   if (GainsProcessed ==
       TRUE) { // Set GainsProcessed flag for relevant controller
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
long GetLimits(struct SCType *S, struct fy_node *limsNode, long controllerState)
{
   long LimitsProcessed = FALSE;

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   double *fMax = NULL, *vMax = NULL;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;
   switch (controllerState) {
      case TRN_STATE:
         fMax = Cmd->FrcB_max;
         vMax = Cmd->vel_max;
         break;
      case ATT_STATE:
         fMax = Cmd->Trq_max;
         vMax = Cmd->w_max;
         break;
      case FULL_STATE:
         // PLACEHOLDER
         break;
      case DMP_STATE:
         fMax = Cmd->dTrq_max;
         break;
      default:
         break;
   }
   if (assignYAMLToDoubleArray(3, fy_node_by_path_def(limsNode, "/Force Max"),
                               fMax) == 3 &&
       (controllerState == H_DUMP_CNTRL ||
        assignYAMLToDoubleArray(
            3, fy_node_by_path_def(limsNode, "/Velocity Max"), vMax) == 3))
      LimitsProcessed = TRUE;
   if (controllerState == ATT_STATE) {
      for (long i = 0; i < 3; i++)
         vMax[i] *= D2R;
   }
   return (LimitsProcessed);
}
//----------------------------------- CONTROLLER
//-----------------------------------
long GetController(struct SCType *S, struct fy_node *ctrlNode,
                   long controllerState)
{
   struct fy_node *gainNode = NULL, *limNode = NULL;

   long CntrlProcessed = FALSE;

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;

   long controller;
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
      if (GetGains(S, gainNode, controllerState) == FALSE) {
         printf("For Controller alias %s, could not find Gain alias %s or "
                "invalid format."
                " Exiting...\n",
                fy_anchor_get_text(fy_node_get_anchor(ctrlNode), NULL),
                fy_anchor_get_text(fy_node_get_anchor(gainNode), NULL));
         exit(EXIT_FAILURE);
      }
      if (GetLimits(S, limNode, controllerState) == FALSE) {
         printf("For Controller alias %s, could not find Limit alias %s or "
                "invalid format."
                " Exiting...\n",
                fy_anchor_get_text(fy_node_get_anchor(ctrlNode), NULL),
                fy_anchor_get_text(fy_node_get_anchor(limNode), NULL));
         exit(EXIT_FAILURE);
      }
   }

   return (CntrlProcessed);
}
//----------------------------------- ACTUATORS
//-----------------------------------
long GetActuators(struct SCType *S, struct fy_node *actNode,
                  long controllerState)
{
   long ActuatorsProcessed = FALSE;

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   struct AcType *AC;

   DSM              = &S->DSM;
   Cmd              = &DSM->Cmd;
   AC               = &S->AC;
   char actName[40] = {0};
   if (fy_node_scanf(actNode, "/Type %39s", actName) == 1) {
      strcpy(Cmd->dmp_actuator,
             ""); // Null it out if no dumping to avoid other errors
      if (controllerState == ATT_STATE)
         Cmd->H_DumpActive = FALSE;
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
long GetTranslationCmd(struct SCType *S, struct fy_node *trnCmdNode,
                       const double DsmCmdTime, struct fy_node *dsmRoot)
{
   struct fy_node *iterNode = NULL, *ctrlNode = NULL, *actNode = NULL,
                  *limNode      = NULL;
   long TranslationCmdProcessed = FALSE;

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;

   char subType[FIELDWIDTH + 1] = {};
   long cmdInd;
   const char *searchStr = "/Subtype %" STR(FIELDWIDTH) "s /Index %ld";
   fy_node_scanf(trnCmdNode, searchStr, subType, &cmdInd);
   if (!strcmp(subType, "NO_CHANGE")) {
      TranslationCmdProcessed = TRUE;
      return (TranslationCmdProcessed);
   }
   else if (!strcmp(subType, "Passive")) {
      Cmd->TranslationCtrlActive = FALSE;
      TranslationCmdProcessed    = TRUE;
      return (TranslationCmdProcessed);
   }
   else if (!strcmp(subType, "Translation")) {
      Cmd->TranslationCtrlActive = TRUE;
      struct fy_node *cmdNode =
          fy_node_by_path_def(dsmRoot, "/Translation Configurations");

      WHILE_FY_ITER(cmdNode, iterNode)
      {
         long ind = 0;
         fy_node_scanf(iterNode, "/Translation/Index %ld", &ind);
         if (ind == cmdInd) {
            iterNode     = fy_node_by_path_def(iterNode, "/Translation");
            long isGood  = fy_node_scanf(iterNode,
                                         "/Origin %19s "
                                          "/Frame %19s",
                                         Cmd->RefOrigin, Cmd->RefFrame) == 2;
            ctrlNode     = fy_node_by_path_def(iterNode, "/Controller");
            actNode      = fy_node_by_path_def(iterNode, "/Actuator");
            isGood      &= ctrlNode != NULL && actNode != NULL;

            isGood &= assignYAMLToDoubleArray(
                          3, fy_node_by_path_def(iterNode, "/Position"),
                          Cmd->Pos) == 3;
            if (isGood) {
               TranslationCmdProcessed = TRUE;
               Cmd->ManeuverMode       = INACTIVE;
            }
            break;
         }
      }
      if (TranslationCmdProcessed == FALSE) {
         printf(
             "Could not find Translation Command index %ld or invalid format. "
             "Exiting...\n",
             cmdInd);
         exit(EXIT_FAILURE);
      }
   }
   else if (!strcmp(subType, "Maneuver")) {
      Cmd->TranslationCtrlActive = TRUE;
      struct fy_node *cmdNode =
          fy_node_by_path_def(dsmRoot, "/Maneuver Configurations");
      WHILE_FY_ITER(cmdNode, iterNode)
      {
         long ind = 0;
         fy_node_scanf(iterNode, "/Maneuver/Index %ld", &ind);
         if (ind == cmdInd) {
            iterNode = fy_node_by_path_def(iterNode, "/Maneuver");
            char manType[FIELDWIDTH + 1] = {};
            const char *searchManStr =
                "/Type %" STR(FIELDWIDTH) "s /Frame %19s /Duration %lf";

            long isGood  = fy_node_scanf(iterNode, searchManStr, manType,
                                         Cmd->RefFrame, &Cmd->BurnTime) == 3;
            limNode      = fy_node_by_path_def(iterNode, "/Limits");
            actNode      = fy_node_by_path_def(iterNode, "/Actuator");
            isGood      &= assignYAMLToDoubleArray(
                          3, fy_node_by_path_def(iterNode, "/Delta V"),
                          Cmd->DeltaV) == 3;
            if (isGood) {
               TranslationCmdProcessed = TRUE;
               Cmd->BurnStopTime       = DsmCmdTime + Cmd->BurnTime;
               if (!strcmp(manType, "CONSTANT"))
                  Cmd->ManeuverMode = CONSTANT;
               else if (!strcmp(manType, "SMOOTHED"))
                  Cmd->ManeuverMode = SMOOTHED;
               else {
                  printf(
                      "%s is an invalid maneuver mode for Maneuver index %ld. "
                      "Exiting...",
                      manType, cmdInd);
                  exit(EXIT_FAILURE);
               }
            }
            break;
         }
      }
      if (TranslationCmdProcessed == FALSE) {
         printf(
             "Could not find Translation Command index %ld or invalid format. "
             "Exiting...\n",
             cmdInd);
         exit(EXIT_FAILURE);
      }
   }

   if (TranslationCmdProcessed == TRUE && Cmd->TranslationCtrlActive == TRUE) {
      if (Cmd->ManeuverMode == INACTIVE) {
         if (GetController(S, ctrlNode, TRN_STATE) == FALSE) {
            printf("For %s index %ld, could not find Controller alias %s or "
                   "invalid format. "
                   "Exiting...\n",
                   subType, cmdInd,
                   fy_anchor_get_text(fy_node_get_anchor(ctrlNode), NULL));
            exit(EXIT_FAILURE);
         }
      }
      else {
         if (GetLimits(S, limNode, TRN_STATE) == FALSE) {
            printf("For %s index %ld, could not find Limit alias %s or invalid "
                   "format. "
                   "Exiting...\n",
                   subType, cmdInd,
                   fy_anchor_get_text(fy_node_get_anchor(limNode), NULL));
            exit(EXIT_FAILURE);
         }
      }
      if (GetActuators(S, actNode, TRN_STATE) == FALSE) {
         printf("For %s index %ld, could not find Actuator alias %s or invalid "
                "format. "
                "Exiting...\n",
                subType, cmdInd,
                fy_anchor_get_text(fy_node_get_anchor(actNode), NULL));
         exit(EXIT_FAILURE);
      }
   }

   return (TranslationCmdProcessed);
}
//-----------------------ATTITUDE CMD ---------------------------------------
long GetAttitudeCmd(struct SCType *S, struct fy_node *attCmdNode,
                    struct fy_node *dsmRoot)
{
   struct fy_node *iterNode = NULL, *ctrlNode = NULL, *actNode = NULL;
   long AttitudeCmdProcessed = FALSE, AttPriCmdProcessed = FALSE,
        AttSecCmdProcessed = FALSE;
   char GroundStationCmd[30];
   long AttPriCmdMode, AttSecCmdMode, AttCmdMode;
   long *cmdInd;

   int state = ATT_STATE;
   struct DSMType *DSM;
   struct DSMCmdType *Cmd;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;

   char subType[FIELDWIDTH + 1] = {};
   const char *searchStr        = "/Subtype %" STR(FIELDWIDTH) "[^\n]";
   fy_node_scanf(attCmdNode, searchStr, subType);
   if (!strcmp(subType, "NO_CHANGE")) {
      AttitudeCmdProcessed = TRUE;
   }
   else if (!strcmp(subType, "Passive")) {
      Cmd->AttitudeCtrlActive = FALSE;
      AttitudeCmdProcessed    = TRUE;
   }
   else if (!strcmp(subType, "Two Vector Pointing") ||
            !strcmp(subType, "One Vector Pointing")) {
      long kMax;
      long inds[2]                 = {0};
      struct DSMCmdVecType *vecs[] = {&Cmd->PriVec, NULL};
      long *cmdModes[]             = {&AttPriCmdMode, &AttSecCmdMode};
      struct fy_node *nodes[]      = {NULL, NULL};
      char *cmdRefFrm[]            = {Cmd->PriAttRefFrame, Cmd->SecAttRefFrame};
      long *attcmdProc[]           = {&AttPriCmdProcessed, &AttSecCmdProcessed};
      struct fy_node *indNode      = fy_node_by_path_def(attCmdNode, "/Index");

      if (!strcmp(subType, "Two Vector Pointing")) {
         kMax    = 2;
         vecs[1] = &Cmd->SecVec;
         assignYAMLToLongArray(2, indNode, inds);
         for (int k = 0; k < kMax; k++)
            *cmdModes[k] = inds[k];
         Cmd->Method = PARM_VECTORS;
      }
      else {
         kMax        = 1;
         Cmd->Method = PARM_UNITVECTOR;
         cmdModes[0] = &AttCmdMode;
         fy_node_scanf(indNode, "/ %ld", cmdModes[0]);
      }
      for (int k = 0; k < kMax; k++) {
         struct fy_node *searchNode = fy_node_by_path_def(
             dsmRoot, (k == 0) ? ("/Primary Vector Configurations")
                               : ("/Secondary Vector Configurations"));
         WHILE_FY_ITER(searchNode, nodes[k])
         {
            long ind = 0;
            fy_node_scanf(nodes[k],
                          (k == 0) ? ("/Primary Vector/Index %ld")
                                   : ("/Secondary Vector/Index %ld"),
                          &ind);
            if (ind == *cmdModes[k]) {
               nodes[k] = fy_node_by_path_def(nodes[k],
                                              (k == 0) ? ("/Primary Vector")
                                                       : ("/Secondary Vector"));
               break;
            }
         }
      }
      if (nodes[0] != NULL &&
          (Cmd->Method == PARM_UNITVECTOR || nodes[1] != NULL)) {
         struct fy_node *tgtNode = NULL;
         ctrlNode                = fy_node_by_path_def(nodes[0], "/Controller");
         actNode                 = fy_node_by_path_def(nodes[0], "/Actuator");
         for (int k = 0; k < kMax; k++) {
            tgtNode = fy_node_by_path_def(nodes[k], "/Target");
            assignYAMLToDoubleArray(3, fy_node_by_path_def(nodes[k], "/Axis"),
                                    vecs[k]->cmd_axis);
            char tgtType[50] = {};
            fy_node_scanf(tgtNode, "/Type %49s", tgtType);

            if (!strcmp(tgtType, "BODY") || !strcmp(tgtType, "SC")) {
               vecs[k]->CmdMode = CMD_TARGET;
               char target[50]  = {};
               fy_node_scanf(tgtNode, "/Target %51s", target);
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
                             &vecs[k]->TrgBody) ==
                      2) { // Decode Current SC ID Number
                     if (vecs[k]->TrgSC >= Nsc) {
                        printf("This mission only has %ld spacecraft, but "
                               "spacecraft %ld was "
                               "attempted to be set as the primary target "
                               "vector. Exiting...\n",
                               Nsc, vecs[k]->TrgSC);
                        exit(EXIT_FAILURE);
                     }
                     if (vecs[k]->TrgBody >= SC[vecs[k]->TrgSC].Nb) {
                        printf("Spacecraft %ld only has %ld bodies, but the "
                               "primary target was "
                               "attempted to be set as body %ld. Exiting...\n",
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
                  printf("%s Vector index %ld has improper format for SC or "
                         "BODY targeting. "
                         "Exiting...\n",
                         (k == 0) ? ("Primary") : ("Secondary"), *cmdModes[k]);
                  exit(EXIT_FAILURE);
               }
               *attcmdProc[k] = TRUE;
            }
            else if (!strcmp(tgtType, "VEC")) {
               vecs[k]->CmdMode = CMD_DIRECTION;
               vecs[k]->TrgType = TARGET_VEC;
               *attcmdProc[k] =
                   fy_node_scanf(tgtNode, "/Frame %c", cmdRefFrm[k]);
               *attcmdProc[k] &= assignYAMLToDoubleArray(
                                     3, fy_node_by_path_def(tgtNode, "/Axis"),
                                     vecs[k]->cmd_vec) == 3;
               if (*attcmdProc[k] == FALSE) {
                  printf("%s Vector index %ld has improper format for VEC "
                         "targeting. "
                         "Exiting...\n",
                         (k == 0) ? ("Primary") : ("Secondary"), *cmdModes[k]);
                  exit(EXIT_FAILURE);
               }
            }
            else {
               printf(
                   "For %s Vector index %ld, %s is an invalid targeting type. "
                   "Exiting...\n",
                   (k == 0) ? ("Primary") : ("Secondary"), *cmdModes[k],
                   tgtType);
               exit(EXIT_FAILURE);
            }
         }
      }
      else {
         printf("Could not find either Primary Vector command index %ld or "
                "Secondary Vector "
                "command index %ld. "
                "Exiting...\n",
                *cmdModes[0], *cmdModes[1]);
         exit(EXIT_FAILURE);
      }
      if (AttPriCmdProcessed == TRUE && AttSecCmdProcessed == TRUE) {
         AttitudeCmdProcessed = TRUE;
      }
   }
   else if (!strcmp(subType, "Quaternion")) {
      fy_node_scanf(attCmdNode, "/Index %ld", &AttCmdMode);
      cmdInd      = &AttCmdMode;
      Cmd->Method = PARM_QUATERNION;

      struct fy_node *searchNode =
          fy_node_by_path_def(dsmRoot, "/Quaternion Configurations");
      WHILE_FY_ITER(searchNode, iterNode)
      {
         long ind = 0;
         fy_node_scanf(iterNode, "/Quaternion/Index %ld", &ind);
         if (ind == *cmdInd) {
            iterNode = fy_node_by_path_def(iterNode, "/Quaternion");
            break;
         }
      }
      searchNode = fy_node_by_path_def(iterNode, "/Quaternion");
      AttitudeCmdProcessed =
          assignYAMLToDoubleArray(4, searchNode, Cmd->q) == 4;
      AttitudeCmdProcessed &=
          fy_node_scanf(iterNode, "/Frame %19s", Cmd->AttRefFrame) == 1;
      ctrlNode              = fy_node_by_path_def(iterNode, "/Controller");
      actNode               = fy_node_by_path_def(iterNode, "/Actuator");
      AttitudeCmdProcessed &= ctrlNode != NULL && actNode != NULL;

      Cmd->AttitudeCtrlActive = TRUE;
   }
   else if (!strcmp(subType, "Mirror")) {
      fy_node_scanf(attCmdNode, "/Index %ld", &AttCmdMode);
      cmdInd      = &AttCmdMode;
      Cmd->Method = PARM_MIRROR;

      struct fy_node *searchNode =
          fy_node_by_path_def(dsmRoot, "/Mirror Configurations");
      WHILE_FY_ITER(searchNode, iterNode)
      {
         long ind = 0;
         fy_node_scanf(iterNode, "/Mirror/Index %ld", &ind);
         if (ind == *cmdInd) {
            iterNode = fy_node_by_path_def(iterNode, "/Mirror");
            break;
         }
      }
      AttitudeCmdProcessed =
          fy_node_scanf(iterNode, "/Target %5s", Cmd->AttRefScID) == 1;
      ctrlNode              = fy_node_by_path_def(iterNode, "/Controller");
      actNode               = fy_node_by_path_def(iterNode, "/Actuator");
      AttitudeCmdProcessed &= ctrlNode != NULL && actNode != NULL;

      Cmd->AttitudeCtrlActive = TRUE;
   }
   else if (!strcmp(subType, "Detumble")) {
      fy_node_scanf(attCmdNode, "/Index %ld", &AttCmdMode);
      cmdInd      = &AttCmdMode;
      Cmd->Method = PARM_DETUMBLE;

      struct fy_node *searchNode =
          fy_node_by_path_def(dsmRoot, "/Detumble Configurations");
      WHILE_FY_ITER(searchNode, iterNode)
      {
         long ind = 0;
         fy_node_scanf(iterNode, "/Detumble/Index %ld", &ind);
         if (ind == *cmdInd) {
            iterNode = fy_node_by_path_def(iterNode, "/Detumble");
            break;
         }
      }
      ctrlNode             = fy_node_by_path_def(iterNode, "/Controller");
      actNode              = fy_node_by_path_def(iterNode, "/Actuator");
      AttitudeCmdProcessed = ctrlNode != NULL && actNode != NULL;

      Cmd->AttitudeCtrlActive = TRUE;
   }
   else if (!strcmp(subType, "Whl H Manage")) {
      fy_node_scanf(attCmdNode, "/Index %ld", &AttCmdMode);
      cmdInd = &AttCmdMode;

      struct fy_node *searchNode =
          fy_node_by_path_def(dsmRoot, "/Whl H Manage Configurations");
      WHILE_FY_ITER(searchNode, iterNode)
      {
         long ind = 0;
         fy_node_scanf(iterNode, "/Whl H Manage/Index %ld", &ind);
         if (ind == *cmdInd) {
            iterNode = fy_node_by_path_def(iterNode, "/Whl H Manage");
            break;
         }
      }
      AttitudeCmdProcessed =
          fy_node_scanf(iterNode,
                        "/Minimum H_norm %lf "
                        "/Maximum H_norm %lf",
                        &Cmd->H_DumpLims[0], &Cmd->H_DumpLims[1]) == 2;

      ctrlNode              = fy_node_by_path_def(iterNode, "/Controller");
      actNode               = fy_node_by_path_def(iterNode, "/Actuator");
      AttitudeCmdProcessed &= ctrlNode != NULL && actNode != NULL;

      struct fy_node *dumpNode  = fy_node_by_path_def(iterNode, "/Dumping");
      AttitudeCmdProcessed     &= dumpNode != NULL;
      Cmd->H_DumpActive         = fy_node_compare_string(dumpNode, "true", -1);
      state                     = DMP_STATE;
      if (Cmd->H_DumpLims[1] < Cmd->H_DumpLims[0]) {
         printf("Maximum momentum dump limit must be more than the minimum for "
                "Whl H Manage "
                "Command index %ld. "
                "Exiting...\n",
                *cmdInd);
         exit(EXIT_FAILURE);
      }
   }
   else {
      AttitudeCmdProcessed = FALSE;
   }
   if (AttitudeCmdProcessed == TRUE && Cmd->AttitudeCtrlActive == TRUE) {
      if (GetController(S, ctrlNode, state) == FALSE) {
         printf("For %s index %ld, could not find Controller alias %s or "
                "invalid format. "
                "Exiting...\n",
                subType, *cmdInd,
                fy_anchor_get_text(fy_node_get_anchor(ctrlNode), NULL));
         exit(EXIT_FAILURE);
      }

      if (GetActuators(S, actNode, state) == FALSE) {
         printf("For %s index %ld, could not find Actuator alias %s or invalid "
                "format. "
                "Exiting...\n",
                subType, *cmdInd,
                fy_anchor_get_text(fy_node_get_anchor(actNode), NULL));
         exit(EXIT_FAILURE);
      }
   }
   return (AttitudeCmdProcessed);
}
//-------------------------------- ACTUATOR CMD --------------------------------
long GetActuatorCmd(struct SCType *S, struct fy_node *actCmdNode,
                    struct fy_node *dsmRoot)
{
   struct fy_node *iterNode = NULL, *actSeqNode = NULL;
   long ActuatorCmdProcessed = FALSE;
   long i = 0, actCmdInd = 0;

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   struct AcType *AC;

   AC  = &S->AC;
   DSM = &S->DSM;
   Cmd = &DSM->Cmd;

   fy_node_scanf(actCmdNode, "/Index %ld", &actCmdInd);
   struct fy_node *actConfigNode =
       fy_node_by_path_def(dsmRoot, "/Actuator Cmd Configurations");
   WHILE_FY_ITER(actConfigNode, iterNode)
   {
      long ind = 0;
      fy_node_scanf(iterNode, "/Actuator Cmd/Index %ld", &ind);
      if (ind == actCmdInd) {
         actSeqNode = fy_node_by_path_def(iterNode, "/Actuator Cmd/Actuators");
         break;
      }
   }
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
            printf("Actuator Command index %ld has improper actuator type %s. "
                   "Exiting...",
                   actCmdInd, type);
            exit(EXIT_FAILURE);
         }
      }
      else {
         printf(
             "Actuator Command index %ld is impropertly formatted. Exiting...",
             actCmdInd);
         exit(EXIT_FAILURE);
      }
      if (Cmd->ActTypes[i] == WHL_TYPE && Cmd->ActInds[i] > AC->Nwhl) {
         printf("SC[%ld] only has %ld wheels, but an actuator command was sent "
                "to wheel %d. "
                "Exiting...\n",
                AC->ID, AC->Nwhl, Cmd->ActInds[i]);
         exit(EXIT_FAILURE);
      }
      if (Cmd->ActTypes[i] == THR_TYPE && Cmd->ActInds[i] > AC->Nthr) {
         printf("SC[%ld] only has %ld thrusters, but an actuator command was "
                "sent to thruster %d. "
                "Exiting...\n",
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

// the compare function for double values
static int compare(const void *a, const void *b)
{
   if (*(double *)a > *(double *)b)
      return 1;
   else if (*(double *)a < *(double *)b)
      return -1;
   else
      return 0;
}
//------------------------ INTERPRETER (FIRST ITERATION) -----------------------
void DsmCmdInterpreterMrk1(struct SCType *S, struct fy_node *dsmCmds)
{
   struct DSMType *DSM;
   struct fy_node *iterNode = NULL, *scCmdsNode = NULL;

   DSM         = &S->DSM;
   DSM->CmdCnt = 0;
   DSM->CmdNum = 0;
   // TODO: preload and presort the command node pointers

   WHILE_FY_ITER(dsmCmds, iterNode)
   {
      long scInd = 0;
      fy_node_scanf(iterNode, "/SC %ld", &scInd);
      if (scInd == S->ID) {
         scCmdsNode = fy_node_by_path_def(iterNode, "/Command Sequence");
         break;
      }
   }
   if (scCmdsNode == NULL) {
      DSM->CmdTime_f   = NULL;
      DSM->CmdNextTime = 0.0;
   }
   else {
      DSM->CmdCnt    = fy_node_sequence_item_count(scCmdsNode);
      DSM->CmdTime_f = calloc(DSM->CmdCnt, sizeof(double));
      long i         = 0;
      iterNode       = NULL;
      WHILE_FY_ITER(scCmdsNode, iterNode)
      {
         fy_node_scanf(iterNode, "/Time %lf", &DSM->CmdTime_f[i++]);
      }
      qsort(DSM->CmdTime_f, DSM->CmdCnt, sizeof(double), compare);
      DSM->CmdNextTime = DSM->CmdTime_f[0];
   }
}
//--------------------- INTERPRETER (SUBSEQUENT ITERATIONS) --------------------
void DsmCmdInterpreterMrk2(struct SCType *S, struct fy_node *dsmRoot,
                           struct fy_node *dsmCmds)
{
   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   struct fy_node *iterNode = NULL, *scCmdsNode = NULL, *cmdsNode = NULL;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;

   WHILE_FY_ITER(dsmCmds, iterNode)
   {
      long scInd = 0;
      fy_node_scanf(iterNode, "/SC %ld", &scInd);
      if (scInd == S->ID) {
         scCmdsNode = fy_node_by_path_def(iterNode, "/Command Sequence");
         break;
      }
   }
   iterNode = NULL;
   WHILE_FY_ITER(scCmdsNode, iterNode)
   {
      double cmdTime = 0.0;
      fy_node_scanf(iterNode, "/Time %lf", &cmdTime);
      if (cmdTime == DSM->CmdNextTime) {
         cmdsNode = fy_node_by_path_def(iterNode, "/Commands");
         break;
      }
   }

   if (cmdsNode == NULL) {
      printf("Could not find command for SC[%ld] at time %lf. "
             "How did this happen? Exiting...\n",
             S->ID, DSM->CmdNextTime);
      exit(EXIT_FAILURE);
   }

   iterNode = NULL;
   WHILE_FY_ITER(cmdsNode, iterNode)
   {
      char typeToken[FIELDWIDTH + 1] = {}, subType[FIELDWIDTH + 1] = {};

      const char *searchTypeStr = "/Type %" STR(FIELDWIDTH) "[^\n]";
      const char *searchSubtypeIndexStr =
          "/Subtype %" STR(FIELDWIDTH) "[^\n] /Index %ld";
      fy_node_scanf(iterNode, searchTypeStr, typeToken);
      if (!strcmp(typeToken, "Translation")) {
         if (GetTranslationCmd(S, iterNode, DSM->CmdNextTime, dsmRoot) ==
             FALSE) {
            long index;
            fy_node_scanf(iterNode, searchSubtypeIndexStr, subType, &index);
            printf("Translational command of subtype %*s and index %ld "
                   "cannot be found in Inp_DSM.yaml. Exiting...\n",
                   FIELDWIDTH, subType, index);
            exit(EXIT_FAILURE);
         }
      }
      else if (!strcmp(typeToken, "Attitude")) {
         if (GetAttitudeCmd(S, iterNode, dsmRoot) == FALSE) {
            const char *searchSubtypeStr = "/Sub Type %" STR(FIELDWIDTH) "s";
            fy_node_scanf(iterNode, searchSubtypeStr, subType);
            if (!strncmp(subType, "Two", 3)) {
               long indicies[2];
               assignYAMLToLongArray(2, fy_node_by_path_def(iterNode, "/Index"),
                                     indicies);
               printf("Actuator command of subtype %*s and indicies [%ld %ld]"
                      "cannot be found in Inp_DSM.yaml. Exiting...\n",
                      FIELDWIDTH, subType, indicies[0], indicies[1]);
            }
            else {
               long index;
               fy_node_scanf(iterNode, "/Index %ld", &index);
               printf("Actuator command of subtype %*s and index %ld "
                      "cannot method found in Inp_DSM.yaml. Exiting...\n",
                      FIELDWIDTH, subType, index);
            }
            exit(EXIT_FAILURE);
         }
      }
      else if (!strcmp(typeToken, "Actuator")) {
         if (GetActuatorCmd(S, iterNode, dsmRoot) == FALSE) {
            long index;
            fy_node_scanf(iterNode, "/Index %ld", &index);
            printf("Actuator command of index %ld "
                   "cannot be found in Inp_DSM.yaml. Exiting...\n",
                   index);
            exit(EXIT_FAILURE);
         }
      }
      else {
         printf("%s is not a supported command type. Exiting...\n", typeToken);
         exit(EXIT_FAILURE);
      }
      // This sure is one of the if() statements of all time. I feel like it can
      // be reduced...
      if ((Cmd->TranslationCtrlActive && Cmd->AttitudeCtrlActive) &&
          ((!strcmp(Cmd->trn_actuator, "THR_3DOF") &&
            (!strcmp(Cmd->att_actuator, "THR_6DOF") ||
             !strcmp(Cmd->dmp_actuator, "THR_6DOF"))) ||
           (!strcmp(Cmd->trn_actuator, "THR_6DOF") &&
            (!strcmp(Cmd->att_actuator, "THR_3DOF") ||
             !strcmp(Cmd->dmp_actuator, "THR_3DOF"))) ||
           (!strcmp(Cmd->trn_actuator, "THR_3DOF") &&
            (!strcmp(Cmd->att_actuator, "THR_3DOF") ||
             !strcmp(Cmd->dmp_actuator, "THR_3DOF"))))) {
         printf("If the Translation actuator is 6DOF Thruster and Attitude "
                "actuator is Thruster, "
                "then it must be 6DOF (and vice versa).\nAdditionally, if the "
                "translation actuator "
                "is 3DOF thruster, then Attitude cannot also be 3DOF. "
                "Exiting...\n");
         exit(EXIT_FAILURE);
      }
   }
}
#undef FIELDWIDTH
//------------------------------------------------------------------------------
//                                SENSORS
//------------------------------------------------------------------------------
void SensorModule(struct SCType *S)
{

   struct AcType *AC;

   AC = &S->AC;

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
void ActuatorModule(struct SCType *S)
{

   long i;
   double unit_bvb[3];

   struct AcType *AC;
   struct DSMType *DSM;
   struct DSMCmdType *Cmd;

   AC  = &S->AC;
   DSM = &S->DSM;
   Cmd = &DSM->Cmd;

   // Zero out all actuators first, so that they will be set nonzero only if
   // desired
   for (i = 0; i < 3; i++)
      AC->IdealFrc[i] = 0.0;
   if (AC->Nthr > 0) {
      for (i = 0; i < AC->Nthr; i++)
         AC->Thr[i].PulseWidthCmd = 0.0;
      for (i = 0; i < AC->Nthr; i++)
         AC->Thr[i].ThrustLevelCmd = 0.0;
   }

   for (i = 0; i < 3; i++)
      AC->IdealTrq[i] = 0.0;
   if (AC->Nwhl > 0)
      for (i = 0; i < AC->Nwhl; i++)
         AC->Whl[i].Tcmd = 0.0;
   if (AC->Nmtb > 0)
      for (i = 0; i < AC->Nmtb; i++)
         AC->MTB[i].Mcmd = 0.0;

   for (i = 0; i < 3; i++)
      AC->Fcmd[i] = 0.0;
   for (i = 0; i < 3; i++)
      AC->Tcmd[i] = 0.0;
   for (i = 0; i < 3; i++)
      AC->Mcmd[i] = 0.0;

   // Translation
   if (Cmd->TranslationCtrlActive == TRUE) {
      if ((!strcmp(Cmd->trn_actuator, "THR_3DOF") ||
           !strcmp(Cmd->trn_actuator, "THR_6DOF")) &&
          AC->Nthr > 0) {
         for (i = 0; i < 3; i++)
            AC->Fcmd[i] = DSM->FcmdB[i];
         ThrProcessingMinPower(S);
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
         ThrProcessingMinPower(
             S); // if THR_TRN, this does both force & torque since AC->Fcmd set
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

   // Momentum Dumping
   if (Cmd->H_DumpActive == TRUE) {
      if (Cmd->AttitudeCtrlActive == TRUE && strcmp(Cmd->att_actuator, "WHL")) {
         printf("You many only enable momentum dumping in a PASSIVE attitude "
                "state or the attitude is controlled with WHLs. Exiting...\n");
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
         ThrProcessingMinPower(
             S); // if THR_TRN, this does both force & torque since AC->Fcmd set
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
void FindDsmCmdVecN(struct SCType *S, struct DSMCmdVecType *CV)
{
   /*Clone of FindCmdVecN()from 42fsw.c with new structure type */
   struct WorldType *W;
   double RelPosB[3], vb[3];
   double RelPosN[3], RelPosH[3], RelVelN[3], RelVelH[3];
   double pcmn[3], pn[3], vn[3], ph[3], vh[3];
   double CosPriMerAng, SinPriMerAng;
   double MaxToS, Rhat[3], ToS;
   long It, i;

   switch (CV->TrgType) {
      case TARGET_WORLD:
         W            = &World[CV->TrgWorld];
         CosPriMerAng = cos(W->PriMerAng);
         SinPriMerAng = sin(W->PriMerAng);
         pn[0]        = CV->W[0] * CosPriMerAng - CV->W[1] * SinPriMerAng;
         pn[1]        = CV->W[0] * SinPriMerAng + CV->W[1] * CosPriMerAng;
         pn[2]        = CV->W[2];
         vn[0]        = -CV->W[0] * SinPriMerAng - CV->W[1] * CosPriMerAng;
         vn[1]        = CV->W[0] * CosPriMerAng - CV->W[1] * SinPriMerAng;
         vn[2]        = 0.0;
         if (CV->TrgWorld == Orb[SC->RefOrb].World) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = pn[i] - S->PosN[i];
               RelVelN[i] = vn[i] - S->VelN[i];
            }
         }
         else {
            MTxV(W->CNH, pn, ph);
            MTxV(W->CNH, vn, vh);
            for (i = 0; i < 3; i++) {
               RelPosH[i] = (W->PosH[i] + ph[i]) - S->PosH[i];
               RelVelH[i] = (W->VelH[i] + vh[i]) - S->VelH[i];
            }
            MxV(World[Orb[S->RefOrb].World].CNH, RelPosH, RelPosN);
            MxV(World[Orb[S->RefOrb].World].CNH, RelVelH, RelVelN);
         }
         CopyUnitV(RelPosN, CV->N);
         DSM_RelMotionToAngRate(RelPosN, RelVelN, CV->wn);
         break;
      case TARGET_SC:
         if (SC[CV->TrgSC].RefOrb == S->RefOrb) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = SC[CV->TrgSC].PosR[i] - S->PosR[i];
               RelVelN[i] = SC[CV->TrgSC].VelR[i] - S->VelR[i];
            }
         }
         else if (Orb[SC[CV->TrgSC].RefOrb].World == Orb[S->RefOrb].World) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = SC[CV->TrgSC].PosN[i] - S->PosN[i];
               RelVelN[i] = SC[CV->TrgSC].VelN[i] - S->VelN[i];
            }
         }
         else {
            for (i = 0; i < 3; i++) {
               RelPosH[i] = SC[CV->TrgSC].PosH[i] - S->PosH[i];
               RelVelH[i] = SC[CV->TrgSC].VelH[i] - S->VelH[i];
            }
            MxV(World[Orb[S->RefOrb].World].CNH, RelPosH, RelPosN);
            MxV(World[Orb[S->RefOrb].World].CNH, RelVelH, RelVelN);
         }
         CopyUnitV(RelPosN, CV->N);
         DSM_RelMotionToAngRate(RelPosN, RelVelN, CV->wn);
         break;
      case TARGET_BODY:
         MTxV(SC[CV->TrgSC].B[0].CN, SC[CV->TrgSC].cm, pcmn);
         MTxV(SC[CV->TrgSC].B[CV->TrgBody].CN, CV->T, pn);
         for (i = 0; i < 3; i++)
            RelPosB[i] = CV->T[i] - SC[CV->TrgSC].B[CV->TrgBody].cm[i];
         VxV(SC[CV->TrgSC].B[CV->TrgBody].wn, RelPosB, vb);
         MTxV(SC[CV->TrgSC].B[CV->TrgBody].CN, vb, vn);
         for (i = 0; i < 3; i++) {
            pn[i] += SC[CV->TrgSC].B[CV->TrgBody].pn[i] - pcmn[i];
            vn[i] += SC[CV->TrgSC].B[CV->TrgBody].vn[i];
         }
         if (SC[CV->TrgSC].RefOrb == S->RefOrb) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = SC[CV->TrgSC].PosR[i] + pn[i] - S->PosR[i];
               RelVelN[i] = SC[CV->TrgSC].VelR[i] + vn[i] - S->VelR[i];
            }
         }
         else if (Orb[SC[CV->TrgSC].RefOrb].World == Orb[S->RefOrb].World) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = SC[CV->TrgSC].PosN[i] + pn[i] - S->PosN[i];
               RelVelN[i] = SC[CV->TrgSC].VelN[i] + vn[i] - S->VelN[i];
            }
         }
         else {
            MTxV(World[Orb[SC[CV->TrgSC].RefOrb].World].CNH, pn, ph);
            MTxV(World[Orb[SC[CV->TrgSC].RefOrb].World].CNH, vn, vh);
            for (i = 0; i < 3; i++) {
               RelPosH[i] = SC[CV->TrgSC].PosH[i] + ph[i] - S->PosH[i];
               RelVelH[i] = SC[CV->TrgSC].VelH[i] + vh[i] - S->VelH[i];
            }
            MxV(World[Orb[S->RefOrb].World].CNH, RelPosH, RelPosN);
            MxV(World[Orb[S->RefOrb].World].CNH, RelVelH, RelVelN);
         }
         CopyUnitV(RelPosN, CV->N);
         DSM_RelMotionToAngRate(RelPosN, RelVelN, CV->wn);
         break;
      case TARGET_VELOCITY:
         for (i = 0; i < 3; i++)
            CV->N[i] = S->VelN[i];
         UNITV(CV->N);
         break;
      case TARGET_MAGFIELD:
         for (i = 0; i < 3; i++)
            CV->N[i] = S->bvn[i];
         UNITV(CV->N);
         break;
      case TARGET_TDRS:
         CV->N[0] = 0.0;
         CV->N[1] = 0.0;
         CV->N[2] = 1.0;
         for (i = 0; i < 3; i++)
            CV->wn[i] = 0.0;
         MaxToS = -2.0; /* Bogus */
         CopyUnitV(S->PosN, Rhat);
         /* Aim at TDRS closest to Zenith */
         for (It = 0; It < 10; It++) {
            if (Tdrs[It].Exists) {
               for (i = 0; i < 3; i++)
                  RelPosN[i] = Tdrs[It].PosN[i] - S->PosN[i];
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
void TranslationGuidance(struct SCType *S)
{

   double wcn[3];
   long i, Isc_Ref, goodOriginFrame = FALSE;
   long frame_body, origin_body;

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   struct DSMCtrlType *CTRL;
   struct FormationType *F;

   DSM  = &S->DSM;
   Cmd  = &DSM->Cmd;
   CTRL = &DSM->DsmCtrl;
   F    = &Frm[S->RefOrb];

   if (Cmd->TranslationCtrlActive == TRUE && Cmd->ManeuverMode == INACTIVE) {
      // Convert Disp vec into N/R coords.
      if (!strcmp(Cmd->RefFrame, "F")) {
         MTxV(F->CN, Cmd->Pos, CTRL->CmdPosR); // Convert F to R Inertial
         if (F->FixedInFrame == 'L') {
            for (i = 0; i < 3; i++)
               wcn[i] = Orb[S->RefOrb].wln[i]; // L rotates wrt R
            goodOriginFrame = TRUE;
         }
         else if (F->FixedInFrame == 'N') {
            for (i = 0; i < 3; i++)
               wcn[i] = 0.0; // R does not rotate wrt R Inertial
            goodOriginFrame = TRUE;
         }
      }
      else if (!strcmp(Cmd->RefFrame, "N")) {
         for (i = 0; i < 3; i++)
            CTRL->CmdPosR[i] = Cmd->Pos[i]; // Already in R Inertial
         for (i = 0; i < 3; i++)
            wcn[i] = 0.0; // R does not rotate wrt R Inertial
         goodOriginFrame = TRUE;
      }
      else if (!strcmp(Cmd->RefFrame, "L")) {
         MTxV(S->CLN, Cmd->Pos, CTRL->CmdPosR); // Convert LVLH to R Inertial
         for (i = 0; i < 3; i++)
            wcn[i] = Orb[S->RefOrb].wln[i]; // L rotates wrt R
         goodOriginFrame = TRUE;
      }
      else if (!strncmp(Cmd->RefFrame, "SC",
                        2)) { // Specify disp from OP, in SC B frame
                              // directions, control to OP
         sscanf(Cmd->RefFrame, "SC[%ld].B[%ld]", &Isc_Ref,
                &frame_body); // Decode ref SC ID Number
         if (Isc_Ref >= Nsc) {
            printf(
                "This mission only has %ld spacecraft, but spacecraft %ld was "
                "attempted to be set as the reference frame. Exiting...\n",
                Nsc, Isc_Ref);
            exit(EXIT_FAILURE);
         }
         if (frame_body >= SC[Isc_Ref].Nb) {
            printf("Spacecraft %ld only has %ld bodies, but the reference "
                   "frame was attempted to be set as body %ld. Exiting...\n",
                   Isc_Ref, SC[Isc_Ref].Nb, frame_body);
            exit(EXIT_FAILURE);
         }
         QTxV(SC[Isc_Ref].B[frame_body].qn, Cmd->Pos,
              CTRL->CmdPosR); // Convert SC# B to R Inertial
         QTxV(SC[Isc_Ref].B[frame_body].qn, SC[Isc_Ref].B[frame_body].wn,
              wcn); // SC rotates wrt R
         goodOriginFrame = TRUE;
      }
      if (!strcmp(Cmd->RefOrigin, "OP")) { // Specify disp from OP, in X frame
                                           // directions, control to OP
         // Add pos of F frame origin in R frame
         VxV(wcn, CTRL->CmdPosR, CTRL->CmdVelR);
         for (i = 0; i < 3; i++)
            CTRL->CmdPosR[i] += F->PosR[i];
         goodOriginFrame = TRUE;
      }
      else if (!strncmp(Cmd->RefOrigin, "SC",
                        2)) { // Specify disp from SC, in X frame directions,
                              // control to SC
         // Add pos of SC in R frame
         sscanf(Cmd->RefOrigin, "SC[%ld].B[%ld]", &Isc_Ref,
                &origin_body); // Decode ref SC ID Number
         if (Isc_Ref >= Nsc) {
            printf(
                "This mission only has %ld spacecraft, but spacecraft %ld was "
                "attempted to be set as the reference origin. Exiting...\n",
                Nsc, Isc_Ref);
            exit(EXIT_FAILURE);
         }
         if (origin_body >= SC[Isc_Ref].Nb) {
            printf("Spacecraft %ld only has %ld bodies, but the reference "
                   "origin was attempted to be set as body %ld. Exiting...\n",
                   Isc_Ref, SC[Isc_Ref].Nb, origin_body);
            exit(EXIT_FAILURE);
         }
         VxV(wcn, CTRL->CmdPosR, CTRL->CmdVelR);
         for (i = 0; i < 3; i++)
            CTRL->CmdVelR[i] +=
                SC[Isc_Ref].VelR[i] + SC[Isc_Ref].B[origin_body].vn[i];
         for (i = 0; i < 3; i++)
            CTRL->CmdPosR[i] +=
                SC[Isc_Ref].PosR[i] + SC[Isc_Ref].B[origin_body].pn[i];
         goodOriginFrame = TRUE;
      }
      else {
         goodOriginFrame = FALSE;
      }
      if (goodOriginFrame == FALSE) {
         printf("Invalid Ref origin/frame combo %s/%s in Translation Command "
                "at %f. Exiting...\n",
                Cmd->RefOrigin, Cmd->RefFrame, SimTime);
         exit(EXIT_FAILURE);
      }
      for (i = 0; i < 3; i++) {
         CTRL->CmdPosN[i] =
             CTRL->CmdPosR[i] + S->PosN[i]; // Actually calculate these values
         CTRL->CmdVelN[i] = CTRL->CmdVelR[i] + S->VelN[i];
      }
      for (i = 0; i < 3; i++) {
         CTRL->trn_kp[i]   = Cmd->trn_kp[i];
         CTRL->trn_kr[i]   = Cmd->trn_kr[i];
         CTRL->trn_ki[i]   = Cmd->trn_ki[i];
         CTRL->FrcB_max[i] = Cmd->FrcB_max[i];
         CTRL->vel_max[i]  = Cmd->vel_max[i];
      }
   }
}
//------------------------------------------------------------------------------
void AttitudeGuidance(struct SCType *S)
{

   long i, Isc_Ref, target_num;
   double qfn[4], qrn[4], qfl[4], qrf[4];
   double PriCmdVecB[3], PriCmdVecN[3];
   double SecCmdVecB[3], SecCmdVecN[3];
   double tgtX_b[3], tgtY_b[3], tgtZ_b[3];
   double tgtX_n[3], tgtY_n[3], tgtZ_n[3];
   double C_tb[3][3], C_tn[3][3], dC[3][3];
   double tmag;
   double q_tb[4] = {0, 0, 0, 1}, q_tn[4] = {0, 0, 0, 1}, qbn_cmd[4];
   double vec_cmp[3];

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   struct DSMCtrlType *CTRL;
   struct FormationType *F;
   struct DSMCmdVecType *PV, *SV;

   DSM  = &S->DSM;
   Cmd  = &DSM->Cmd;
   CTRL = &DSM->DsmCtrl;
   F    = &Frm[S->RefOrb];
   PV   = &Cmd->PriVec;
   SV   = &Cmd->SecVec;

   if (Cmd->AttitudeCtrlActive == TRUE) {
      if (Cmd->Method == PARM_VECTORS) {
         if (PV->TrgType == TARGET_SC || PV->TrgType == TARGET_WORLD) {
            FindDsmCmdVecN(S, PV); // to get PV->wn, PV->N (in F Frame)
            MxV(S->B[0].CN, PV->N,
                PriCmdVecB); // (Converting Cmd vec to body frame)
         }
         else if (PV->TrgType == TARGET_VEC) {
            if (!strcmp(Cmd->PriAttRefFrame, "N")) {
               MxV(S->B[0].CN, PV->cmd_vec,
                   PriCmdVecB); // (Converting Cmd vec to body frame)
            }
            else if (!strcmp(Cmd->PriAttRefFrame, "F")) {
               MTxV(F->CN, PV->cmd_vec,
                    PriCmdVecN); // (Converting to Inertial frame)
               MxV(S->B[0].CN, PriCmdVecN,
                   PriCmdVecB); // (Converting to body frame) CHECK THIS
            }
            else if (!strcmp(Cmd->PriAttRefFrame, "L")) {
               MTxV(S->CLN, PV->cmd_vec,
                    PriCmdVecN); // (Converting to LVLH to Inertial frame)
               UNITV(PriCmdVecN);

               MxV(S->B[0].CN, PriCmdVecN,
                   PriCmdVecB); // (Converting to body frame)
            }
            else if (!strcmp(Cmd->PriAttRefFrame, "B")) {
               for (i = 0; i < 3; i++)
                  PriCmdVecB[i] = PV->cmd_vec[i];
            }
         }
         UNITV(PriCmdVecB);

         if (SV->TrgType == TARGET_SC || SV->TrgType == TARGET_WORLD) {
            FindDsmCmdVecN(S, SV); // to get SV->wn, SV->N
            MxV(S->B[0].CN, SV->N, SecCmdVecB);
         }
         else if (SV->TrgType == TARGET_VEC) {
            if (!strcmp(Cmd->SecAttRefFrame, "N")) {
               MxV(S->B[0].CN, SV->cmd_vec,
                   SecCmdVecB); // (Converting Cmd vec to body frame)
            }
            else if (!strcmp(Cmd->SecAttRefFrame, "F")) {
               MTxV(F->CN, SV->cmd_vec,
                    SecCmdVecN); // (Converting to Inertial frame)
               MxV(S->B[0].CN, SecCmdVecN,
                   SecCmdVecB); // (Converting to body frame)
            }
            else if (!strcmp(Cmd->SecAttRefFrame, "L")) {
               MTxV(S->CLN, SV->cmd_vec,
                    SecCmdVecN); // (Converting from LVLH to Inertial frame)
               UNITV(SecCmdVecN);
               MxV(S->B[0].CN, SecCmdVecN,
                   SecCmdVecB); // (Converting to body frame)
            }
            else if (!strcmp(Cmd->SecAttRefFrame, "B")) {
               for (i = 0; i < 3; i++)
                  SecCmdVecB[i] = SV->cmd_vec[i];
            }
            UNITV(SecCmdVecB);
         }

         MTxV(S->B[0].CN, PriCmdVecB, PriCmdVecN);
         MTxV(S->B[0].CN, SecCmdVecB, SecCmdVecN);
         UNITV(PriCmdVecN);
         UNITV(SecCmdVecN);

         for (i = 0; i < 3; i++) {
            tgtX_b[i] = PV->cmd_axis[i]; // = PV->cmd_axis
            tgtX_n[i] = PriCmdVecN[i];   // = PriCmdVec
         }

         VxV(PV->cmd_axis, SV->cmd_axis, tgtZ_b);
         VxV(PriCmdVecN, SecCmdVecN, tgtZ_n);
         VxV(tgtZ_b, tgtX_b, tgtY_b);
         VxV(tgtZ_n, tgtX_n, tgtY_n);

         for (i = 0; i < 3; i++)
            vec_cmp[i] = PV->cmd_axis[i] - SV->cmd_axis[i];
         if (fabs(MAGV(vec_cmp)) < EPS_DSM) {
            printf("PV [%lf  %lf  %lf] in %s and SV [%lf  %lf  %lf] in %s are "
                   "identical, resulting in a infeasible attitude command. "
                   "Exiting...\n",
                   PV->cmd_vec[0], PV->cmd_vec[1], PV->cmd_vec[2],
                   Cmd->PriAttRefFrame, SV->cmd_vec[0], SV->cmd_vec[1],
                   SV->cmd_vec[2], Cmd->SecAttRefFrame);
            exit(EXIT_FAILURE);
         }

         UNITV(tgtX_b);
         UNITV(tgtY_b);
         UNITV(tgtZ_b);
         UNITV(tgtX_n);
         UNITV(tgtY_n);
         UNITV(tgtZ_n);

         /*construct body to target DCM and Inertial to Target DCMS*/
         for (i = 0; i < 3; i++) {
            C_tb[0][i] = tgtX_b[i];
            C_tb[1][i] = tgtY_b[i];
            C_tb[2][i] = tgtZ_b[i];
            C_tn[0][i] = tgtX_n[i];
            C_tn[1][i] = tgtY_n[i];
            C_tn[2][i] = tgtZ_n[i];
         }

         C2Q(C_tb, q_tb);
         C2Q(C_tn, q_tn);

         /* Approximation of log map from SO3 to so3 to calculate Cmd->wrn */
         MTxM(C_tn, Cmd->OldCRN, dC);
         tmag        = acos((dC[0][0] + dC[1][1] + dC[2][2] - 1) / 2.0);
         Cmd->wrn[0] = (dC[2][1] - dC[1][2]) / 2.0 / S->AC.DT;
         Cmd->wrn[1] = (dC[0][2] - dC[2][0]) / 2.0 / S->AC.DT;
         Cmd->wrn[2] = (dC[1][0] - dC[0][1]) / 2.0 / S->AC.DT;
         if (tmag >
             EPS_DSM) { // check if wmag large enough to avoid singularity
            for (i = 0; i < 3; i++)
               Cmd->wrn[i] *= tmag / sin(tmag);
         }
         memcpy(Cmd->OldCRN, C_tn, sizeof(Cmd->OldCRN));

         /* Calculate Inertial to Body Quaternion */
         QTxQ(q_tb, q_tn, qbn_cmd);
         UNITQ(qbn_cmd);
         QxQT(DSM->qbn, qbn_cmd, Cmd->qbr);
      }
      else if (Cmd->Method == PARM_UNITVECTOR) {
         printf("Feature for Singular Primary Unit Vector Pointing not "
                "currently fully implemented. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      else if (Cmd->Method == PARM_QUATERNION) {
         if (!strcmp(Cmd->AttRefFrame, "N")) {
            for (i = 0; i < 4; i++)
               Cmd->qrn[i] = Cmd->q[i];
            QxQT(DSM->qbn, Cmd->qrn, Cmd->qbr);
            for (i = 0; i < 3; i++)
               Cmd->wrn[i] = 0.0;
         }
         else if (!strcmp(Cmd->AttRefFrame, "F")) {
            for (i = 0; i < 4; i++)
               Cmd->qrf[i] = Cmd->q[i];
            C2Q(F->CN, qfn);
            QxQ(Cmd->qrf, qfn, qrn);
            QxQT(DSM->qbn, qrn, Cmd->qbr);
            if (F->FixedInFrame == 'L') {
               for (i = 0; i < 3; i++)
                  Cmd->wrn[i] = Orb[S->RefOrb].wln[i]; // F rotates wrt N
            }
            else if (F->FixedInFrame == 'N') {
               for (i = 0; i < 3; i++)
                  Cmd->wrn[i] = 0.0; // N does not rotate wrt N Inertial
            }
         }
         else if (!strcmp(Cmd->AttRefFrame, "L")) {
            for (i = 0; i < 4; i++)
               Cmd->qrl[i] = Cmd->q[i];
            C2Q(F->CL, qfl);
            QxQT(Cmd->qrl, qfl, qrf);
            C2Q(F->CN, qfn);
            QxQ(qrf, qfn, qrn);
            QxQT(DSM->qbn, qrn, Cmd->qbr);
            for (i = 0; i < 3; i++)
               Cmd->wrn[i] = Orb[S->RefOrb].wln[i];
         }
      }
      else if (Cmd->Method == PARM_MIRROR) {
         sscanf(Cmd->AttRefScID, "SC[%ld].B[%ld]", &Isc_Ref,
                &target_num); // Decode ref SC ID Number - does this need to be
                              // SC[%ld].B[%ld]?
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
         QxQT(SC[S->ID].DSM.qbn, SC[Isc_Ref].B[target_num].qn, Cmd->qbr);
         QTxV(SC[Isc_Ref].DSM.qbn, SC[Isc_Ref].B[target_num].wn, Cmd->wrn);
      }
      else if (Cmd->Method == PARM_DETUMBLE) {
         for (i = 0; i < 3; i++)
            Cmd->qbr[i] = 0;
         Cmd->qbr[3] = 1;
         for (i = 0; i < 3; i++)
            Cmd->wrn[i] = 0.0;
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
}
//------------------------------------------------------------------------------
//                                NAVIGATION
//------------------------------------------------------------------------------
void TranslationalNavigation(struct SCType *S)
{

   long i;

   struct AcType *AC;
   struct DSMType *DSM;

   AC  = &S->AC;
   DSM = &S->DSM;

   for (i = 0; i < 3; i++) {
      DSM->PosR[i] = S->PosR[i] + (S->PosN[i] - AC->PosN[i]);
      DSM->VelR[i] = S->VelR[i] + (S->VelN[i] - AC->VelN[i]);
   }
}
//------------------------------------------------------------------------------
void AttitudeNavigation(struct SCType *S)
{

   long i;

   struct AcType *AC;
   struct DSMType *DSM;

   AC  = &S->AC;
   DSM = &S->DSM;

   for (i = 0; i < 4; i++)
      DSM->qbn[i] = AC->qbn[i];
   for (i = 0; i < 3; i++)
      DSM->wbn[i] = AC->wbn[i];
}
//------------------------------------------------------------------------------
//                                 CONTROL
//------------------------------------------------------------------------------
void TranslationCtrl(struct SCType *S)
{

   long i;

   struct DSMType *DSM;
   struct DSMCtrlType *CTRL;
   struct DSMCmdType *Cmd;
   struct AcType *AC;

   DSM  = &S->DSM;
   CTRL = &DSM->DsmCtrl;
   Cmd  = &DSM->Cmd;
   AC   = &S->AC;

   if (Cmd->TranslationCtrlActive == TRUE && Cmd->ManeuverMode == INACTIVE) {
      // TO DO: do we want S->PosR to be AC->PosR? and DSM->FcmdB to be
      // CTRL->FcmdB??
      if (Cmd->trn_controller == PID_CNTRL) { // PID Controller
         // TO DO: do we want S->PosR to be AC->PosR? and DSM->FcmdB to be
         // CTRL->FcmdB?

         if (Cmd->NewTrnGainsProcessed == TRUE) {
            for (i = 0; i < 3; i++) {
               DSM->trn_ei[i] = 0.0;
            }
            Cmd->NewTrnGainsProcessed = FALSE;
         }

         for (i = 0; i < 3; i++) {
            DSM->perr[i]    = S->PosR[i] - CTRL->CmdPosR[i]; // Position Error
            DSM->verr[i]    = S->VelR[i] - CTRL->CmdVelR[i]; // Velocity Error
            DSM->trn_ei[i] += (DSM->Oldperr[i] + DSM->perr[i]) / 2.0 *
                              AC->DT; // Integrated Error

            if (fabs(Cmd->trn_kilimit[i]) > EPS_DSM &&
                fabs(CTRL->trn_ki[i]) > EPS_DSM)
               DSM->trn_ei[i] =
                   Limit(DSM->trn_ei[i], -Cmd->trn_kilimit[i] / CTRL->trn_ki[i],
                         Cmd->trn_kilimit[i] /
                             CTRL->trn_ki[i]); // limits integrated error to
                                               // limit/ki, since limit is given
                                               // in terms of force

            CTRL->u1[i] = CTRL->trn_kp[i] / CTRL->trn_kr[i] * DSM->perr[i];
            if (CTRL->vel_max[i] > 0) {
               CTRL->u1[i] =
                   Limit(CTRL->u1[i], -CTRL->vel_max[i], CTRL->vel_max[i]);
            }
            CTRL->FcmdN[i] = -CTRL->trn_kr[i] * (CTRL->u1[i] + DSM->verr[i]) -
                             CTRL->trn_ki[i] * DSM->trn_ei[i];
         }
         MxV(S->B[0].CN, CTRL->FcmdN,
             CTRL->FcmdB); // Converting from Inertial to body frame for Report
         for (i = 0; i < 3; i++) {
            if (CTRL->FrcB_max[i] > 0)
               CTRL->FcmdB[i] =
                   Limit(CTRL->FcmdB[i], -CTRL->FrcB_max[i],
                         CTRL->FrcB_max[i]); // Limiting AC->Frc in body frame
         }
         MTxV(S->B[0].CN, CTRL->FcmdB, CTRL->FcmdN);
      }
      else if (Cmd->trn_controller == LYA_2BODY_CNTRL) {
         // Calculate relative radius, velocity
         for (i = 0; i < 3; i++) {
            DSM->perr[i] =
                S->PosR[i] - CTRL->CmdPosR[i]; // Position Error, Relative
            DSM->verr[i] = S->VelR[i] - CTRL->CmdVelR[i]; // Velocity Error
         }

         double r_norm  = MAGV(S->PosN);
         double r_cntrl = MAGV(CTRL->CmdPosN);
         double mu      = Orb[SC->RefOrb].mu;

         double dg[3];
         for (i = 0; i < 3; i++)
            dg[i] = -mu / pow(r_norm, 3) * S->PosN[i] +
                    mu / pow(r_cntrl, 3) * CTRL->CmdPosN[i];

         for (i = 0; i < 3; i++) {
            CTRL->FcmdN[i] = -CTRL->trn_kp[i] * DSM->perr[i] -
                             CTRL->trn_kr[i] * DSM->verr[i] - dg[i] * AC->mass;
         }
         MxV(S->B[0].CN, CTRL->FcmdN,
             CTRL->FcmdB); // Converting from Inertial to body frame for Report
         for (i = 0; i < 3; i++) {
            if (CTRL->FrcB_max[i] > 0)
               CTRL->FcmdB[i] =
                   Limit(CTRL->FcmdB[i], -CTRL->FrcB_max[i],
                         CTRL->FrcB_max[i]); // Limiting AC->Frc in body frame
         }
         MTxV(S->B[0].CN, CTRL->FcmdB, CTRL->FcmdN);
      }
      for (i = 0; i < 3; i++)
         DSM->Oldperr[i] = DSM->perr[i];
   }
   else if (Cmd->TranslationCtrlActive == TRUE &&
            Cmd->ManeuverMode != INACTIVE) {
      if (SimTime < Cmd->BurnStopTime) {
         if (Cmd->ManeuverMode == CONSTANT) {
            if (!strcmp(Cmd->RefFrame, "N")) {
               for (i = 0; i < 3; i++) {
                  CTRL->FcmdN[i] = S->mass * Cmd->DeltaV[i] / Cmd->BurnTime;
               }
               MxV(S->B[0].CN, CTRL->FcmdN,
                   CTRL->FcmdB); // Converting from Inertial to body frame for
                                 // Report
            }
            else if (!strcmp(Cmd->RefFrame, "B")) {
               for (i = 0; i < 3; i++) {
                  CTRL->FcmdB[i] = S->mass * Cmd->DeltaV[i] / Cmd->BurnTime;
               }
            }
            for (i = 0; i < 3; i++) {
               if (CTRL->FrcB_max[i] > 0) {
                  CTRL->FcmdB[i] = Limit(
                      CTRL->FcmdB[i], -CTRL->FrcB_max[i],
                      CTRL->FrcB_max[i]); // Limiting AC->Frc in body frame
               }
            }
            MTxV(S->B[0].CN, CTRL->FcmdB,
                 CTRL->FcmdN); // Converting back to Inertial from body frame
         }
         else if (Cmd->ManeuverMode == SMOOTHED) {
            double sharp =
                -2 * atanh(-.99998) /
                Cmd->BurnTime; // .99998 corresponds to capturing 99.999% of the
                               // burn since tanh has an asymptote
            double t_mid = Cmd->BurnStopTime - Cmd->BurnTime / 2.0;
            double t_since_mid =
                SimTime - t_mid; // Time elapsed since middle of burn

            if (!strcmp(Cmd->RefFrame, "N")) {
               for (i = 0; i < 3; i++) {
                  CTRL->FcmdN[i] = S->mass * (Cmd->DeltaV[i] * sharp / 2.0) /
                                   ((cosh(sharp * t_since_mid)) *
                                    (cosh(sharp * t_since_mid)));
               }
               MxV(S->B[0].CN, CTRL->FcmdN,
                   CTRL->FcmdB); // Converting from Inertial to body frame for
                                 // Report
            }
            else if (!strcmp(Cmd->RefFrame, "B")) {
               for (i = 0; i < 3; i++) {
                  CTRL->FcmdB[i] = S->mass * (Cmd->DeltaV[i] * sharp / 2.0) /
                                   ((cosh(sharp * t_since_mid)) *
                                    (cosh(sharp * t_since_mid)));
               }
            }
            for (i = 0; i < 3; i++) {
               if (CTRL->FrcB_max[i] > 0) {
                  CTRL->FcmdB[i] = Limit(
                      CTRL->FcmdB[i], -CTRL->FrcB_max[i],
                      CTRL->FrcB_max[i]); // Limiting AC->Frc in body frame
               }
            }
            MTxV(S->B[0].CN, CTRL->FcmdB,
                 CTRL->FcmdN); // Converting back to Inertial from body frame
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
void AttitudeCtrl(struct SCType *S)
{

   long i;
   double wrb[3];

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   struct DSMCtrlType *CTRL;
   struct AcType *AC;

   DSM  = &S->DSM;
   Cmd  = &DSM->Cmd;
   CTRL = &DSM->DsmCtrl;
   AC   = &S->AC;

   if (Cmd->AttitudeCtrlActive == TRUE) {
      if (Cmd->att_controller == PID_CNTRL) // PID Controller
      {
         if (Cmd->NewAttGainsProcessed == TRUE) {
            for (i = 0; i < 3; i++) {
               DSM->att_ei[i] = 0.0;
            }
            Cmd->NewAttGainsProcessed = FALSE;
         }

         Q2AngleVec(CTRL->qbr, DSM->therr); // Angular Position Error
         QxV(DSM->qbn, Cmd->wrn,
             wrb); // Rotate angular velocity into Body frame
         for (i = 0; i < 3; i++) {
            DSM->werr[i] =
                DSM->wbn[i] - wrb[i]; // Angular Velocity Error (in body frame)
            DSM->att_ei[i] += (DSM->Oldtherr[i] + DSM->therr[i]) / 2.0 *
                              AC->DT; // Integrated angle error

            if (fabs(Cmd->att_kilimit[i]) > EPS_DSM &&
                CTRL->att_ki[i] > EPS_DSM)
               DSM->att_ei[i] =
                   Limit(DSM->att_ei[i], -Cmd->att_kilimit[i] / CTRL->att_ki[i],
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
      }
      else if (Cmd->att_controller == LYA_ATT_CNTRL) {
         double om_x_I_om[3];

         Q2AngleVec(CTRL->qbr, DSM->therr); // Angular Position Error
         QxV(DSM->qbn, Cmd->wrn,
             wrb); // Rotate angular velocity into Body frame
         for (i = 0; i < 3; i++)
            DSM->werr[i] =
                DSM->wbn[i] - wrb[i]; // Angular Velocity Error (in body frame)

         vxMov(DSM->werr, AC->MOI, om_x_I_om); // calculate nonlinear term in
                                               // Quaternion Lyapunov stability

         for (i = 0; i < 3; i++) {
            CTRL->Tcmd[i] = -Cmd->att_kp[i] * CTRL->qbr[i] -
                            Cmd->att_kr[i] * DSM->werr[i] + om_x_I_om[i];
         }
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
void MomentumDumpCtrl(struct SCType *S)
{

   long i, j;
   double TotalWhlH[3] = {0.0, 0.0, 0.0};
   double whlHNorm     = 0.0;

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   struct DSMCtrlType *CTRL;
   struct AcType *AC;

   DSM  = &S->DSM;
   Cmd  = &DSM->Cmd;
   CTRL = &DSM->DsmCtrl;
   AC   = &S->AC;

   for (i = 0; i < AC->Nwhl; i++) {
      for (j = 0; j < 3; j++) {
         TotalWhlH[j] += AC->Whl[i].Axis[j] * AC->Whl[i].H;
      }
   }
   for (i = 0; i < 3; i++)
      whlHNorm += TotalWhlH[i] * TotalWhlH[i];
   whlHNorm = sqrt(whlHNorm);

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
   // load the DSM file statically so that all DsmFSW calls have access to same
   // object
   static struct fy_node *dsmRoot = NULL, *dsmCmds = NULL;
   if (dsmRoot == NULL) {
      FILE *dsm_in            = FileOpen(InOutPath, "Inp_DSM.yaml", "r");
      struct fy_document *fyd = fy_document_build_from_fp(NULL, dsm_in);
      fclose(dsm_in);
      if (fy_document_resolve(fyd)) {
         printf("Unable to resolve links in Inp_DSM.yaml. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      dsmRoot = fy_document_root(fyd);
      dsmCmds = fy_node_by_path_def(dsmRoot, "/DSM Commands");
   }

   struct DSMType *DSM;

   DSM = &S->DSM;

   // Run Command Interperter
   if (DSM->CmdInit) {
      DSM->CmdInit = 0;
      DsmCmdInterpreterMrk1(S, dsmCmds);

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
      DsmCmdInterpreterMrk2(S, dsmRoot, dsmCmds);
      DSM->CmdNum++;
      if (DSM->CmdNum < DSM->CmdCnt)
         DSM->CmdNextTime = DSM->CmdTime_f[DSM->CmdNum];
   }

   // Generate Data From Sensors
   SensorModule(S);

   // Navigation Modules
   TranslationalNavigation(S);
   AttitudeNavigation(S);

   // Generate Guidance Solution
   TranslationGuidance(S);
   AttitudeGuidance(S);

   // Run Control Systems
   TranslationCtrl(S);
   AttitudeCtrl(S);
   MomentumDumpCtrl(S);

   // Implement Control Through Actuators
   ActuatorModule(S);
}

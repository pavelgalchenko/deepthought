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

#include "42dsm.h"

#define EPS_DSM 1e-12

//------------------------------------------------------------------------------
//                                NAV FUNCTIONS
//------------------------------------------------------------------------------
// Assigns the Jacobian and update functions as needed. Also initializes the
// the default measurement data
void AssignNavFunctions(struct DSMNavType *const Nav,
                        const enum navType navType)
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
void InitThrDistVecs(struct AcType *AC, int DOF, enum ctrlState controllerState)
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
   struct DSMNavType *Nav;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;
   Nav = &DSM->DsmNav;

   S->InitDSM   = 0;
   DSM->Init    = 1;
   DSM->ID      = S->ID;
   DSM->CmdInit = 1;
   DSM->DT      = S->AC.DT;
   DSM->mass    = S->AC.mass;
   DSM->refOrb  = &Orb[S->RefOrb];
   for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
         DSM->MOI[i][j] = S->AC.MOI[i][j];

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

   /* Controllers */
   DSM->DsmCtrl.Init         = 1;
   DSM->DsmCtrl.H_DumpActive = FALSE;
   DSM->CmdArray             = NULL;

   Cmd->TranslationCtrlActive = FALSE;
   Cmd->AttitudeCtrlActive    = FALSE;
   Cmd->H_DumpActive          = FALSE;
   strcpy(Cmd->dmp_actuator, "");
   Cmd->ActNumCmds = 0;

   Nav->type             = IDEAL_NAV;
   Nav->batching         = NONE_BATCH;
   Nav->refFrame         = FRAME_N;
   Nav->NavigationActive = FALSE;
   Nav->DT               = S->AC.DT;
   // Nav->Time             = 0.0;
   // Nav->step         = 0;
   Nav->subStep      = 0;
   Nav->Date0.JulDay = 0;
   Nav->Date0.Year   = 0;
   Nav->Date0.Month  = 0;
   Nav->Date0.Day    = 0;
   Nav->Date0.doy    = 0;
   Nav->Date0.Hour   = 0;
   Nav->Date0.Minute = 0;
   Nav->Date0.Second = 0;
   Nav->Date         = Nav->Date0;

   for (enum navState i = INIT_STATE; i <= FIN_STATE; i++)
      Nav->stateActive[i] = FALSE;
   for (enum sensorType i = INIT_SENSOR; i <= FIN_SENSOR; i++) {
      Nav->sensorActive[i] = FALSE;
      Nav->measTypes[i]    = NULL;
      Nav->residuals[i]    = NULL;
   }
   InitMeasList(&Nav->measList);
   /* Initialize pointers to NULL */
   Nav->sqrQ       = NULL;
   Nav->M          = NULL;
   Nav->P          = NULL;
   Nav->delta      = NULL;
   Nav->jacobian   = NULL;
   Nav->STM        = NULL;
   Nav->NxN        = NULL;
   Nav->NxN2       = NULL;
   Nav->whlH       = NULL;
   Nav->refOriBody = 0;
   Nav->refOriType = 0;
   Nav->refOriPtr  = NULL;
   Nav->refBodyPtr = NULL;
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++)
         Nav->oldRefCRN[i][j] = 0.0;
      Nav->oldRefCRN[i][i]   = 1.0;
      Nav->oldRefPos[i]      = 0.0;
      Nav->oldRefVel[i]      = 0.0;
      Nav->oldRefOmega[i]    = 0.0;
      Nav->oldRefOmegaDot[i] = 0.0;
      Nav->forceB[i]         = 0.0;
      Nav->torqueB[i]        = 0.0;
   }
   Nav->reportConfigured = FALSE;
}
//------------------------------------------------------------------------------
//                           COMMAND INTERPRETER
//------------------------------------------------------------------------------

#define FIELDWIDTH 63
//------------------------------------ GAINS -----------------------------------
long GetGains(struct DSMType *const DSM, struct fy_node *gainsNode,
              enum ctrlState controllerState)
{
   long GainsProcessed = FALSE;

   enum ctrlType *controller = NULL;
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
               enum ctrlState controllerState)
{
   long LimitsProcessed = FALSE;

   double *fMax = NULL, *vMax = NULL;
   struct DSMCmdType *Cmd    = &DSM->Cmd;
   enum ctrlType *controller = NULL;
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
                   enum ctrlState controllerState)
{
   struct fy_node *gainNode = NULL, *limNode = NULL;

   long CntrlProcessed = FALSE;

   struct DSMCmdType *Cmd = &DSM->Cmd;

   enum ctrlType controller;
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
                  struct fy_node *actNode, enum ctrlState controllerState)
{
   long ActuatorsProcessed = FALSE;

   struct DSMCmdType *Cmd = &DSM->Cmd;
   char actName[40]       = {0};
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
long GetTranslationCmd(struct AcType *const AC, struct DSMType *const DSM,
                       struct fy_node *trnCmdNode, const double DsmCmdTime,
                       struct fy_node *dsmRoot)
{
   struct fy_node *iterNode = NULL, *ctrlNode = NULL, *actNode = NULL,
                  *limNode      = NULL;
   long TranslationCmdProcessed = FALSE;

   struct DSMCmdType *Cmd = &DSM->Cmd;

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
         if (GetController(DSM, ctrlNode, TRN_STATE) == FALSE) {
            printf("For %s index %ld, could not find Controller alias %s or "
                   "invalid format. Exiting...\n",
                   subType, cmdInd,
                   fy_anchor_get_text(fy_node_get_anchor(ctrlNode), NULL));
            exit(EXIT_FAILURE);
         }
      }
      else {
         if (GetLimits(DSM, limNode, TRN_STATE) == FALSE) {
            printf("For %s index %ld, could not find Limit alias %s or invalid "
                   "format. Exiting...\n",
                   subType, cmdInd,
                   fy_anchor_get_text(fy_node_get_anchor(limNode), NULL));
            exit(EXIT_FAILURE);
         }
      }
      if (GetActuators(AC, DSM, actNode, TRN_STATE) == FALSE) {
         printf("For %s index %ld, could not find Actuator alias %s or invalid "
                "format. Exiting...\n",
                subType, cmdInd,
                fy_anchor_get_text(fy_node_get_anchor(actNode), NULL));
         exit(EXIT_FAILURE);
      }
   }

   return (TranslationCmdProcessed);
}
//-----------------------ATTITUDE CMD ---------------------------------------
long GetAttitudeCmd(struct AcType *const AC, struct DSMType *const DSM,
                    struct fy_node *attCmdNode, struct fy_node *dsmRoot)
{
   struct fy_node *iterNode = NULL, *ctrlNode = NULL, *actNode = NULL;
   long AttitudeCmdProcessed = FALSE, AttPriCmdProcessed = FALSE,
        AttSecCmdProcessed = FALSE;
   char GroundStationCmd[30];
   long AttPriCmdMode, AttSecCmdMode, AttCmdMode;
   long *cmdInd;

   enum ctrlState state   = ATT_STATE;
   struct DSMCmdType *Cmd = &DSM->Cmd;

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
                  printf("%s Vector index %ld has improper format for SC or "
                         "BODY targeting. Exiting...\n",
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
                         "targeting. Exiting...\n",
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
                "Secondary Vector command index %ld. Exiting...\n",
                *cmdModes[0], *cmdModes[1]);
         exit(EXIT_FAILURE);
      }
      if (AttPriCmdProcessed == TRUE && AttSecCmdProcessed == TRUE)
         AttitudeCmdProcessed = TRUE;

      Cmd->AttitudeCtrlActive = TRUE;
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
                "Whl H Manage Command index %ld. Exiting...\n",
                *cmdInd);
         exit(EXIT_FAILURE);
      }
   }
   else {
      AttitudeCmdProcessed = FALSE;
   }
   if (AttitudeCmdProcessed == TRUE && Cmd->AttitudeCtrlActive == TRUE) {
      if (GetController(DSM, ctrlNode, state) == FALSE) {
         printf("For %s index %ld, could not find Controller alias %s or "
                "invalid format. Exiting...\n",
                subType, *cmdInd,
                fy_anchor_get_text(fy_node_get_anchor(ctrlNode), NULL));
         exit(EXIT_FAILURE);
      }

      if (GetActuators(AC, DSM, actNode, state) == FALSE) {
         printf("For %s index %ld, could not find Actuator alias %s or invalid "
                "format. Exiting...\n",
                subType, *cmdInd,
                fy_anchor_get_text(fy_node_get_anchor(actNode), NULL));
         exit(EXIT_FAILURE);
      }
   }
   return (AttitudeCmdProcessed);
}
//-------------------------------- ACTUATOR CMD --------------------------------
long GetActuatorCmd(struct AcType *const AC, struct DSMType *const DSM,
                    struct fy_node *actCmdNode, struct fy_node *dsmRoot)
{
   struct fy_node *iterNode = NULL, *actSeqNode = NULL;
   long ActuatorCmdProcessed = FALSE;
   long i = 0, actCmdInd = 0;

   struct DSMCmdType *Cmd = &DSM->Cmd;

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
//--------------------------------- STATE NAMES --------------------------------
enum navState GetStateValue(const char *string)
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
enum sensorType GetSensorValue(const char *string)
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
void ConfigureMeas(struct DSMMeasType *meas, enum sensorType sensor)
{
   switch (sensor) {
      case GPS_SENSOR:
         meas->dim             = 6;
         meas->errDim          = 6;
         meas->measJacobianFun = &gpsJacobianFun;
         meas->measFun         = &gpsFun;
         break;
      case STARTRACK_SENSOR:
         meas->dim             = 4;
         meas->errDim          = 3;
         meas->measJacobianFun = &startrackJacobianFun;
         meas->measFun         = &startrackFun;
         break;
      case FSS_SENSOR:
         meas->dim             = 2;
         meas->errDim          = 2;
         meas->measJacobianFun = &fssJacobianFun;
         meas->measFun         = &fssFun;
         break;
      case CSS_SENSOR:
         meas->dim             = 1;
         meas->errDim          = 1;
         meas->measJacobianFun = &cssJacobianFun;
         meas->measFun         = &cssFun;
         break;
      case GYRO_SENSOR:
         meas->dim             = 1;
         meas->errDim          = 1;
         meas->measJacobianFun = &gyroJacobianFun;
         meas->measFun         = &gyroFun;
         break;
      case MAG_SENSOR:
         meas->dim             = 1;
         meas->errDim          = 1;
         meas->measJacobianFun = &magJacobianFun;
         meas->measFun         = &magFun;
         break;
      case ACCEL_SENSOR:
         meas->dim             = 1;
         meas->errDim          = 1;
         meas->measJacobianFun = &accelJacobianFun;
         meas->measFun         = &accelFun;
         break;
      default:
         break;
   }
   meas->type = sensor;
   meas->R    = calloc(meas->errDim, sizeof(double));
   meas->N    = CreateMatrix(meas->errDim, meas->errDim);
   // TODO: might move this out later
   for (int i = 0; i < meas->errDim; i++)
      meas->N[i][i] = 1.0;
}

long ConfigureNavigationSensors(struct AcType *const AC,
                                struct DSMNavType *const Nav,
                                struct fy_node *senSetNode)
{
   struct fy_node *iterNode = NULL;
   long DataProcessed = FALSE, numSensors[FIN_SENSOR + 1] = {0};
   long i, j;
   enum sensorType sensor;

   for (sensor = INIT_SENSOR; sensor <= FIN_SENSOR; sensor++) {
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
            free(Nav->residuals[i]);
            DestroyMatrix(Nav->measTypes[sensor][i].N);
         }
         free(Nav->measTypes[sensor]);
         free(Nav->residuals[sensor]);
      }
      Nav->nSensor[sensor] = nSensor;
      if (nSensor > 0) {
         Nav->measTypes[sensor] = calloc(nSensor, sizeof(struct DSMMeasType));
         Nav->residuals[sensor] = calloc(nSensor, sizeof(double *));
      }
      else {
         Nav->measTypes[sensor] = NULL;
         Nav->residuals[sensor] = NULL;
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
      sensor                   = GetSensorValue(sensorType);
      struct DSMMeasType *meas = NULL;
      char sensorName[1024]    = {0};
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
            isGood = assignYAMLToDoubleArray(
                         3, fy_node_by_path_def(iterNode, "/Sensor Noise"),
                         meas->R) == 3;
            for (j = 0; j < meas->errDim; j++)
               meas->R[j] = meas->R[j] / 3600.0 * D2R;
         } break;
         case GPS_SENSOR: {
            isGood = assignYAMLToDoubleArray(
                         2, fy_node_by_path_def(iterNode, "/Sensor Noise"),
                         meas->R) == 2;
            for (j = meas->errDim - 1; j >= 0; j--) {
               if (j < 3)
                  meas->R[j] = meas->R[0];
               else
                  meas->R[j] = meas->R[1];
            }
         } break;
         case FSS_SENSOR: {
            isGood = assignYAMLToDoubleArray(
                         1, fy_node_by_path_def(iterNode, "/Sensor Noise"),
                         meas->R) == 1;
            for (j = 0; j < meas->errDim; j++)
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

      meas->nextMeas  = NULL;
      meas->data      = NULL;
      meas->time      = 0.0;
      meas->step      = 0;
      meas->sensorNum = sensorNum;
      meas->type      = sensor;
      numSensors[sensor]++;
      Nav->residuals[sensor][sensorNum] = calloc(meas->errDim, sizeof(double));
   }
   for (sensor = INIT_SENSOR; sensor <= FIN_SENSOR; sensor++)
      Nav->sensorActive[sensor] = (numSensors[sensor] > 0 ? TRUE : FALSE);

   return (DataProcessed);
}

//------------------------------- NAVIGATION DATA ------------------------------
long GetNavigationData(struct DSMNavType *const Nav, struct fy_node *datNode,
                       enum matType type)
{
   long DataProcessed = FALSE, (*inds)[] = NULL, (*sizes)[] = NULL;
   enum navState state;
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
         state = GetStateValue(&stateNames[k][1]);
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
                  A2C(SEQ, ang[0], ang[1], ang[2], Nav->CRB);
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
         for (state = INIT_STATE; state <= FIN_STATE; state++) {
            if (Nav->stateActive[state] == TRUE) {
               startInd = Nav->stateInd[state];
               switch (state) {
                  case TIME_STATE:
                     Nav->Date0.JulDay = dataDest[startInd];
                     JDToDate(dataDest[startInd], &Nav->Date0.Year,
                              &Nav->Date0.Month, &Nav->Date0.Day,
                              &Nav->Date0.Hour, &Nav->Date0.Minute,
                              &Nav->Date0.Second);
                     Nav->Date = Nav->Date0;
                     break;
                  case ROTMAT_STATE:
                  case QUAT_STATE: {
                     double tmpM[3][3] = {{0.0}};
                     MT(Nav->CRB, tmpM);
                     C2Q(tmpM, Nav->qbr);
                  } break;
                  case POS_STATE:
                     for (i = 0; i < 3; i++)
                        Nav->PosR[i] = dataDest[startInd + i];
                     break;
                  case VEL_STATE:
                     for (i = 0; i < 3; i++)
                        Nav->VelR[i] = dataDest[startInd + i];
                     break;
                  case OMEGA_STATE:
                     for (i = 0; i < 3; i++)
                        Nav->wbr[i] = dataDest[startInd + i];
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
                      struct fy_node *navCmdNode, struct fy_node *dsmRoot)
{
   char navType[FIELDWIDTH + 1] = {}, batchingType[FIELDWIDTH + 1] = {},
                             refOri[FIELDWIDTH + 1] = {}, refFrame = 0;
   struct fy_node *iterNode    = NULL;
   long NavigationCmdProcessed = FALSE;
   enum navState state;
   long i, j;
   long cmdInd;
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
   else {
      Nav->NavigationActive = TRUE;
      struct fy_node *cmdNode =
          fy_node_by_path_def(dsmRoot, "/Navigation Configurations");
      fy_node_scanf(navCmdNode, "/Index %ld", &cmdInd);
      WHILE_FY_ITER(cmdNode, iterNode)
      {
         long ind = 0;
         fy_node_scanf(iterNode, "/Navigation/Index %ld", &ind);
         if (ind == cmdInd) {
            iterNode = fy_node_by_path_def(iterNode, "/Navigation");
            NavigationCmdProcessed =
                fy_node_scanf(
                    iterNode,
                    "/Type %" STR(
                        FIELDWIDTH) "s "
                                    "/Batching %" STR(
                                        FIELDWIDTH) "s "
                                                    "/Frame %c "
                                                    "/Reference Origin %" STR(
                                                        FIELDWIDTH) "s",
                    navType, batchingType, &refFrame, refOri) == 4;
            qNode      = fy_node_by_path_def(iterNode, "/Data/Q");
            pNode      = fy_node_by_path_def(iterNode, "/Data/P");
            x0Node     = fy_node_by_path_def(iterNode, "/Data/x0");
            senSetNode = fy_node_by_path_def(iterNode, "/Sensor Set");
            statesNode = fy_node_by_path_def(iterNode, "/States");
            NavigationCmdProcessed &= qNode != NULL && pNode != NULL &&
                                      x0Node != NULL && senSetNode != NULL &&
                                      statesNode != NULL;
            break;
         }
      }
   }

   if (NavigationCmdProcessed == TRUE) {
      // TODO: not the biggest fan of DTSIM being here, but I'm coming around
      Nav->DT          = DSM->DT;
      Nav->subStepSize = DTSIM;
      Nav->subStep     = 0;
      Nav->step        = 0;
      double t0        = gpsTime2J2000Sec(GpsRollover, GpsWeek, GpsSecond);
      TimeToDate(t0, &Nav->Date0.Year, &Nav->Date0.Month, &Nav->Date0.Day,
                 &Nav->Date0.Hour, &Nav->Date0.Minute, &Nav->Date0.Second,
                 DTSIM);
      Nav->Date0.doy =
          MD2DOY(Nav->Date0.Year, Nav->Date0.Month, Nav->Date0.Day);
      Nav->Date0.JulDay =
          DateToJD(Nav->Date0.Year, Nav->Date0.Month, Nav->Date0.Day,
                   Nav->Date0.Hour, Nav->Date0.Minute, Nav->Date0.Second);
      Nav->Date = Nav->Date0;

      Nav->subStepMax       = (long)(Nav->DT / Nav->subStepSize + 0.5);
      Nav->Init             = FALSE;
      Nav->reportConfigured = FALSE;
      if (Nav->sqrQ != NULL) {
         free(Nav->sqrQ);
         free(Nav->delta);
         DestroyMatrix(Nav->P);
         DestroyMatrix(Nav->STM);
         Nav->sqrQ     = NULL;
         Nav->M        = NULL;
         Nav->delta    = NULL;
         Nav->P        = NULL;
         Nav->S        = NULL;
         Nav->jacobian = NULL;
         Nav->STM      = NULL;
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
         printf(
             "%s is an invalid filter type for filter index %ld. Exiting...\n",
             navType, cmdInd);
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
         printf("%s is an invalid batching type for filter index %ld. "
                "Exiting...\n",
                batchingType, cmdInd);
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
         printf("Frame %c is an invalid navigation reference frame for filter "
                "index %ld. Exiting...\n",
                refFrame, cmdInd);
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
            Nav->refOriPtr      = &TrgS->DSM;
            Nav->refBodyPtr     = TrgS->B;
         }
      }
      else {
         Nav->refOriType = ORI_WORLD;
         Nav->refOriBody = 0;
         long wID        = DecodeString(refOri);
         Nav->refOriPtr  = &World[wID];
         Nav->refBodyPtr = NULL;
         // error check?
      }

      for (i = INIT_STATE; i <= FIN_STATE; i++)
         Nav->stateActive[i] = FALSE;

      iterNode = NULL;
      WHILE_FY_ITER(statesNode, iterNode)
      {
         char p[FIELDWIDTH + 1] = {0};
         fy_node_scanf(iterNode, "/ %" STR(FIELDWIDTH) "s", p);
         state = GetStateValue(p);
         if (state == -1 || (state == ROTMAT_STATE && Nav->type == MEKF_NAV) ||
             (state == QUAT_STATE && Nav->type != MEKF_NAV)) {
            printf("%s is an invalid state to estimate for navigation filter "
                   "index %ld of type %s. Exiting...\n",
                   p, cmdInd, navType);
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

      for (i = INIT_STATE; i <= FIN_STATE; i++) {
         switch (i) {
            case TIME_STATE:
               Nav->stateSize[i] = 1;
               Nav->navSize[i]   = 1;
               break;
            case ROTMAT_STATE:
               Nav->stateSize[i] = 9;
               Nav->navSize[i]   = 3;
               break;
            case QUAT_STATE:
               Nav->stateSize[i] = 4;
               Nav->navSize[i]   = 3;
               break;
            case POS_STATE:
            case VEL_STATE:
            case OMEGA_STATE:
               Nav->stateSize[i] = 3;
               Nav->navSize[i]   = 3;
               break;
         }
      }
      Nav->whlH = calloc(AC->Nwhl, sizeof(double));

      long stateInd = 0;
      long navInd   = 0;
      for (i = INIT_STATE; i <= FIN_STATE; i++) {
         if (Nav->stateActive[i] == TRUE) {
            Nav->stateInd[i]  = stateInd;
            Nav->navInd[i]    = navInd;
            stateInd         += Nav->stateSize[i];
            navInd           += Nav->navSize[i];
         }
         else {
            // TODO: I need to figure out how to deal with this for varied
            // frames & origins
            Nav->stateInd[i] = -1;
            Nav->navInd[i]   = -1;
            switch (i) {
               case POS_STATE:
                  for (j = 0; j < 3; j++)
                     Nav->PosR[j] =
                         0; // S->PosR[j] + (S->PosN[j] - AC->PosN[j]);
                  break;
               case VEL_STATE:
                  for (j = 0; j < 3; j++)
                     Nav->VelR[j] =
                         0; // S->VelR[j] + (S->VelN[j] - AC->VelN[j]);
                  break;
               case OMEGA_STATE:
                  for (j = 0; j < 3; j++)
                     Nav->wbr[j] = AC->wbn[j];
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
         for (j = 0; j < 4; j++)
            Nav->qbr[j] = AC->qbn[j];

      // sqrQ and P0 diagonal elements from Inp_DSM.txt
      Nav->sqrQ  = calloc(Nav->navDim, sizeof(double));
      Nav->M     = CreateMatrix(Nav->navDim, Nav->navDim);
      Nav->P     = CreateMatrix(Nav->navDim, Nav->navDim);
      Nav->S     = CreateMatrix(Nav->navDim, Nav->navDim);
      Nav->delta = calloc(Nav->navDim, sizeof(double));

      Nav->jacobian = CreateMatrix(Nav->navDim, Nav->navDim);
      Nav->STM      = CreateMatrix(Nav->navDim, Nav->navDim);
      Nav->NxN      = CreateMatrix(Nav->navDim, Nav->navDim);
      Nav->NxN2     = CreateMatrix(Nav->navDim, Nav->navDim);
      for (i = 0; i < Nav->navDim; i++)
         Nav->STM[i][i] = 1.0;

      if (GetNavigationData(Nav, x0Node, IC_DAT) == FALSE) {
         printf("Navigation data is an invalid data set for the initial "
                "estimation states for Navigation index %ld. Exiting...\n",
                cmdInd);
         exit(EXIT_FAILURE);
      }

      if (Nav->refOriType == ORI_WORLD && Nav->refFrame == FRAME_N) {
         for (i = 0; i < 3; i++) {
            Nav->PosR[i] += DSM->refOrb->PosN[i];
            Nav->VelR[i] += DSM->refOrb->VelN[i];
         }
      }

      if (Nav->stateActive[ROTMAT_STATE] == TRUE) {
         // Simple test if given rot mat is a rot mat
         double testM[3][3] = {{0.0}}, test = 0.0;
         MTxM(Nav->CRB, Nav->CRB,
              testM); // if Nav->CRB is valid, testM should be identity
         for (i = 0; i < 3; i++)
            testM[i][i] -=
                1.0; // if Nav->CRB is valid, testM should now be zero matrix
         for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
               test += fabs(testM[i][j]); // 1-norm of vec(testM)
         if (test >= EPS_DSM) {
            printf("The supplied initial rotation matrix for Navigation "
                   "Command index %ld is not a valid Rotation Matrix. "
                   "Exiting...\n",
                   cmdInd);
            exit(EXIT_FAILURE);
         }
      }

      if (GetNavigationData(Nav, qNode, Q_DAT) == FALSE) {
         printf("Navigation data is an invalid data set for the process noise "
                "covariance matrix for Navigation command %ld. Exiting...\n",
                cmdInd);
         exit(EXIT_FAILURE);
      }
      if (GetNavigationData(Nav, pNode, P0_DAT) == FALSE) {
         printf(
             "Navigation data is an invalid data set for the inital estimation "
             "error covariance matrix for Navigation command index %ld. "
             "Exiting...",
             cmdInd);
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
            fy_node_scanf(DSM->CmdArray[0], "/Time %lf", &DSM->CmdNextTime);
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

      const char *searchTypeStr = "/Type %" STR(FIELDWIDTH) "[^\n]";
      const char *searchSubtypeIndexStr =
          "/Subtype %" STR(FIELDWIDTH) "[^\n] /Index %ld";
      fy_node_scanf(iterNode, searchTypeStr, typeToken);
      if (!strcmp(typeToken, "Translation")) {
         if (GetTranslationCmd(AC, DSM, iterNode, DSM->CmdNextTime, dsmRoot) ==
             FALSE) {
            long index;
            fy_node_scanf(iterNode, searchSubtypeIndexStr, subType, &index);
            printf("Translational command of subtype %*s and index %ld cannot "
                   "be found in Inp_DSM.yaml. Exiting...\n",
                   FIELDWIDTH, subType, index);
            exit(EXIT_FAILURE);
         }
      }
      else if (!strcmp(typeToken, "Attitude")) {
         if (GetAttitudeCmd(AC, DSM, iterNode, dsmRoot) == FALSE) {
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
               printf("Actuator command of subtype %*s and index %ld cannot "
                      "method found in Inp_DSM.yaml. Exiting...\n",
                      FIELDWIDTH, subType, index);
            }
            exit(EXIT_FAILURE);
         }
      }
      else if (!strcmp(typeToken, "Actuator")) {
         if (GetActuatorCmd(AC, DSM, iterNode, dsmRoot) == FALSE) {
            long index;
            fy_node_scanf(iterNode, "/Index %ld", &index);
            printf("Actuator command of index %ld cannot be found in "
                   "Inp_DSM.yaml. Exiting...\n",
                   index);
            exit(EXIT_FAILURE);
         }
      }
      else if (!strcmp(typeToken, "Navigation")) {
         if (GetNavigationCmd(AC, DSM, iterNode, dsmRoot) == FALSE) {
            long index;
            fy_node_scanf(iterNode, "/Index %ld", &index);
            printf("Navigation command of index %ld cannot be found in "
                   "Inp_DSM.yaml. Exiting...\n",
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
                "actuator is Thruster, then it must be 6DOF (and vice "
                "versa).\nAdditionally, if the translation actuator is 3DOF "
                "thruster, then Attitude cannot also be 3DOF. Exiting...\n");
         exit(EXIT_FAILURE);
      }
   }
}
#undef FIELDWIDTH
//------------------------------------------------------------------------------
//                                SENSORS
//------------------------------------------------------------------------------
void DsmSensorModule(struct AcType *const AC, struct DSMType *const DSM)
{
   struct DSMNavType *Nav = &DSM->DsmNav;
   struct DSMMeasListType measList;
   long haveFSSMeas = FALSE;

   InitMeasList(&measList);

   for (enum sensorType sensor = INIT_SENSOR; sensor <= FIN_SENSOR; sensor++) {
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

   ;
   double RelPosB[3], vb[3];
   double RelPosN[3], RelPosH[3], RelVelN[3], RelVelH[3];
   double pn[3], vn[3], ph[3], vh[3];
   double CosPriMerAng, SinPriMerAng;
   double MaxToS, Rhat[3], ToS;
   long It, i;

   struct OrbitType const *RefOrb = DSM->refOrb;

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
               RelPosN[i] = pn[i] - DSM->PosN[i];
               RelVelN[i] = vn[i] - DSM->VelN[i];
            }
         }
         else {
            MTxV(TrgW->CNH, pn, ph);
            MTxV(TrgW->CNH, vn, vh);
            struct WorldType *W = &World[RefOrb->World];
            MTxV(W->CNH, DSM->PosN, RelPosH);
            MTxV(W->CNH, DSM->VelN, RelVelH);
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
         struct DSMType *TrgDSM = &SC[CV->TrgSC].DSM;
         if (TrgDSM->refOrb == RefOrb) {
            for (i = 0; i < 3; i++) {
               // TODO
               RelPosN[i] = TrgDSM->PosR[i] - DSM->PosR[i];
               RelVelN[i] = TrgDSM->VelR[i] - DSM->VelR[i];
            }
         }
         else if (TrgDSM->refOrb->World == RefOrb->World) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = TrgDSM->PosN[i] - DSM->PosN[i];
               RelVelN[i] = TrgDSM->VelN[i] - DSM->VelN[i];
            }
         }
         else {
            struct WorldType *TrgW = &World[TrgDSM->refOrb->World];
            struct WorldType *W    = &World[RefOrb->World];
            MTxV(TrgW->CNH, TrgDSM->PosN, RelPosH);
            MTxV(TrgW->CNH, TrgDSM->VelN, RelVelH);
            MTxV(TrgW->CNH, DSM->PosN, ph);
            MTxV(TrgW->CNH, DSM->VelN, vh);
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
         struct DSMType *TrgDSM = NULL;
         struct BodyType *TrgSB = NULL;
         double pcmn[3]         = {0.0};
         {
            // Limit the scope where SC is accessed
            struct SCType *TrgS = &SC[CV->TrgSC];
            TrgSB               = TrgS->B;
            TrgDSM              = &TrgS->DSM;
            // TODO: don't like accessing SCType::cm
            MTxV(TrgDSM->CBN, TrgS->cm, pcmn);
         }
         // TODO: make this better
         double qBb[4] = {0.0}, qbN[4] = {0.0};
         QxQT(TrgSB[0].qn, TrgSB[CV->TrgBody].qn, qBb);
         QTxQ(qBb, TrgDSM->qbn, qbN);
         for (i = 0; i < 3; i++)
            RelPosB[i] = CV->T[i] - TrgSB[CV->TrgBody].cm[i];
         VxV(TrgSB[CV->TrgBody].wn, RelPosB, vb);
         QTxV(qbN, CV->T, pn);
         QTxV(qbN, vb, vn);
         for (i = 0; i < 3; i++) {
            pn[i] += TrgSB[CV->TrgBody].pn[i] - pcmn[i];
            vn[i] += TrgSB[CV->TrgBody].vn[i];
         }

         if (TrgDSM->refOrb == RefOrb) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = TrgDSM->PosR[i] + pn[i] - DSM->PosR[i];
               RelVelN[i] = TrgDSM->VelR[i] + vn[i] - DSM->VelR[i];
            }
         }
         else if (TrgDSM->refOrb->World == RefOrb->World) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = TrgDSM->PosN[i] + pn[i] - DSM->PosN[i];
               RelVelN[i] = TrgDSM->VelN[i] + vn[i] - DSM->VelN[i];
            }
         }
         else {
            for (i = 0; i < 3; i++) {
               pn[i] += TrgDSM->PosN[i];
               vn[i] += TrgDSM->VelN[i];
            }
            struct WorldType *TrgW = &World[TrgDSM->refOrb->World];
            MTxV(TrgW->CNH, pn, RelPosH);
            MTxV(TrgW->CNH, vn, RelVelH);
            struct WorldType *W = &World[RefOrb->World];
            MTxV(W->CNH, DSM->PosN, ph);
            MTxV(W->CNH, DSM->VelN, ph);
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
            CV->N[i] = DSM->VelN[i];
         UNITV(CV->N);
         break;
      case TARGET_MAGFIELD:
         for (i = 0; i < 3; i++)
            CV->N[i] = DSM->bvn[i];
         UNITV(CV->N);
         break;
      case TARGET_TDRS:
         CV->N[0] = 0.0;
         CV->N[1] = 0.0;
         CV->N[2] = 1.0;
         for (i = 0; i < 3; i++)
            CV->wn[i] = 0.0;
         MaxToS = -2.0; /* Bogus */
         CopyUnitV(DSM->PosN, Rhat);
         /* Aim at TDRS closest to Zenith */
         for (It = 0; It < 10; It++) {
            if (Tdrs[It].Exists) {
               for (i = 0; i < 3; i++)
                  RelPosN[i] = Tdrs[It].PosN[i] - DSM->PosN[i];
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
   double wcn[3];
   long i, Isc_Ref, goodOriginFrame = FALSE;
   long frame_body, origin_body;

   struct DSMCmdType *Cmd;
   struct DSMCtrlType *CTRL;

   Cmd  = &DSM->Cmd;
   CTRL = &DSM->DsmCtrl;

   if (Cmd->TranslationCtrlActive == TRUE && Cmd->ManeuverMode == INACTIVE) {
      // Convert Disp vec into N/R coords.
      if (!strcmp(Cmd->RefFrame, "F")) {
         MTxV(F->CN, Cmd->Pos, CTRL->CmdPosR); // Convert F to R Inertial
         if (F->FixedInFrame == 'L') {
            for (i = 0; i < 3; i++)
               wcn[i] = DSM->refOrb->wln[i]; // L rotates wrt R
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
         double CLN[3][3] = {{0.0}}, wln[3] = {0.0};
         FindCLN(DSM->PosN, DSM->VelN, CLN, wln);
         MTxV(CLN, Cmd->Pos, CTRL->CmdPosR); // Convert LVLH to R Inertial
         for (i = 0; i < 3; i++)
            wcn[i] = DSM->refOrb->wln[i]; // L rotates wrt R
         goodOriginFrame = TRUE;
      }
      else if (!strncmp(Cmd->RefFrame, "SC",
                        2)) { // Specify disp from OP, in SC B frame directions,
                              // control to OP
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
         // TODO: don't use other sc truth
         double qbn[4] = {0.0}, wbn[3] = {0.0};
         struct BodyType *TrgSB = NULL;
         struct DSMType *TrgDSM = NULL;
         {
            // Limit scope where we need SCType
            struct SCType *TrgS = &SC[Isc_Ref];
            TrgSB               = TrgS->B;
            TrgDSM              = &TrgS->DSM;
         }
         if (frame_body != 0) {
            // get relative orientation of body to B[0] then apply this to
            // AC.qbn
            double qbB[4] = {0.0}, wBnb[3] = {0.0}, wBnbAC[3] = {0.0};
            QxQT(TrgSB[frame_body].qn, TrgSB[0].qn, qbB);
            QxQ(qbB, TrgDSM->qbn, qbn);

            // get angular velocity of body relative to B[0], then apply this to
            // AC.wbn; all in B[frame_body] frame
            QxV(qbB, TrgSB[0].wn, wBnb);
            QxV(qbB, TrgDSM->wbn, wBnbAC);
            for (i = 0; i < 3; i++) {
               // TODO: double check what BodyType::wn actually is
               wbn[i] = wBnbAC[i] + (TrgSB[frame_body].wn[i] - wBnb[i]);
            }
         }
         else {
            for (i = 0; i < 3; i++) {
               qbn[i] = TrgDSM->qbn[i];
               wbn[i] = TrgDSM->wbn[i];
            }
            qbn[3] = TrgDSM->qbn[3];
         }
         QTxV(qbn, Cmd->Pos, CTRL->CmdPosR); // Convert SC# B to R Inertial
         QTxV(qbn, wbn, wcn);                // SC rotates wrt R
         goodOriginFrame = TRUE;
      }
      if (!strcmp(Cmd->RefOrigin, "OP")) {
         // Specify disp from OP, in X frame directions, control to OP
         // Add pos of F frame origin in R frame
         VxV(wcn, CTRL->CmdPosR, CTRL->CmdVelR);
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
         struct BodyType *TrgSB = NULL;
         struct DSMType *TrgDSM = NULL;
         {
            // Limit scope where we need SCType
            struct SCType *TrgS = &SC[Isc_Ref];
            TrgSB               = TrgS->B;
            TrgDSM              = &TrgS->DSM;
         }
         for (i = 0; i < 3; i++) {
            CTRL->CmdPosR[i] += TrgDSM->PosR[i] + TrgSB[origin_body].pn[i];
            CTRL->CmdVelR[i] += TrgDSM->VelR[i] + TrgSB[origin_body].vn[i];
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
         CTRL->CmdPosN[i] = CTRL->CmdPosR[i] + DSM->PosN[i];
         CTRL->CmdVelN[i] = CTRL->CmdVelR[i] + DSM->VelN[i];
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
void AttitudeGuidance(struct DSMType *DSM, struct FormationType *F)
{
   long i, Isc_Ref, target_num;
   double qfn[4], qrn[4], qfl[4], qrf[4];
   double PriCmdVecB[3], PriCmdVecN[3];
   double SecCmdVecB[3], SecCmdVecN[3];
   double tgtX_b[3], tgtY_b[3], tgtZ_b[3];
   double tgtX_n[3], tgtY_n[3], tgtZ_n[3];
   double C_tb[3][3], C_tn[3][3], dC[3][3];
   double q_tb[4] = {0, 0, 0, 1}, q_tn[4] = {0, 0, 0, 1}, qbn_cmd[4];
   double vec_cmp[3];

   struct DSMCmdType *Cmd;
   struct DSMCtrlType *CTRL;
   struct DSMCmdVecType *PV, *SV;

   Cmd  = &DSM->Cmd;
   CTRL = &DSM->DsmCtrl;
   PV   = &Cmd->PriVec;
   SV   = &Cmd->SecVec;

   if (Cmd->AttitudeCtrlActive == TRUE) {
      if (Cmd->Method == PARM_VECTORS) {
         if (PV->TrgType == TARGET_SC || PV->TrgType == TARGET_WORLD) {
            FindDsmCmdVecN(DSM,
                           PV); // to get PV->wn, PV->N (in F Frame)
            QxV(DSM->qbn, PV->N,
                PriCmdVecB); // (Converting Cmd vec to body frame)
         }
         else if (PV->TrgType == TARGET_VEC) {
            if (!strcmp(Cmd->PriAttRefFrame, "N")) {
               QxV(DSM->qbn, PV->cmd_vec,
                   PriCmdVecB); // (Converting Cmd vec to body frame)
            }
            else if (!strcmp(Cmd->PriAttRefFrame, "F")) {
               MTxV(F->CN, PV->cmd_vec,
                    PriCmdVecN); // (Converting to Inertial frame)
               QxV(DSM->qbn, PriCmdVecN,
                   PriCmdVecB); // (Converting to body frame) CHECK THIS
            }
            else if (!strcmp(Cmd->PriAttRefFrame, "L")) {
               double CLN[3][3] = {{0.0}}, wln[3] = {0.0};
               FindCLN(DSM->PosN, DSM->VelN, CLN, wln);
               MTxV(CLN, PV->cmd_vec,
                    PriCmdVecN); // (Converting to LVLH to Inertial frame)
               UNITV(PriCmdVecN);
               QxV(DSM->qbn, PriCmdVecN,
                   PriCmdVecB); // (Converting to body frame)
            }
            else if (!strcmp(Cmd->PriAttRefFrame, "B")) {
               for (i = 0; i < 3; i++)
                  PriCmdVecB[i] = PV->cmd_vec[i];
            }
         }
         UNITV(PriCmdVecB);

         if (SV->TrgType == TARGET_SC || SV->TrgType == TARGET_WORLD) {
            FindDsmCmdVecN(DSM, SV); // to get SV->wn, SV->N
            QxV(DSM->qbn, SV->N, SecCmdVecB);
         }
         else if (SV->TrgType == TARGET_VEC) {
            if (!strcmp(Cmd->SecAttRefFrame, "N")) {
               QxV(DSM->qbn, SV->cmd_vec,
                   SecCmdVecB); // (Converting Cmd vec to body frame)
            }
            else if (!strcmp(Cmd->SecAttRefFrame, "F")) {
               MTxV(F->CN, SV->cmd_vec,
                    SecCmdVecN); // (Converting to Inertial frame)
               QxV(DSM->qbn, SecCmdVecN,
                   SecCmdVecB); // (Converting to body frame)
            }
            else if (!strcmp(Cmd->SecAttRefFrame, "L")) {
               double CLN[3][3] = {{0.0}}, wln[3] = {0.0};
               FindCLN(DSM->PosN, DSM->VelN, CLN, wln);
               MTxV(CLN, SV->cmd_vec,
                    SecCmdVecN); // (Converting from LVLH to Inertial frame)
               UNITV(SecCmdVecN);
               QxV(DSM->qbn, SecCmdVecN,
                   SecCmdVecB); // (Converting to body frame)
            }
            else if (!strcmp(Cmd->SecAttRefFrame, "B")) {
               for (i = 0; i < 3; i++)
                  SecCmdVecB[i] = SV->cmd_vec[i];
            }
            UNITV(SecCmdVecB);
         }

         QTxV(DSM->qbn, PriCmdVecB, PriCmdVecN);
         QTxV(DSM->qbn, SecCmdVecB, SecCmdVecN);
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
         logso3(dC, Cmd->wrn);
         for (i = 0; i < 3; i++)
            Cmd->wrn[i] /= DSM->DT;
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
                  Cmd->wrn[i] = DSM->refOrb->wln[i]; // F rotates wrt N
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
               Cmd->wrn[i] = DSM->refOrb->wln[i];
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
         // TODO: not truth of other body
         double qbn[4] = {0.0}, wbn[3] = {0.0};
         struct BodyType *TrgSB = NULL;
         struct DSMType *TrgDSM = NULL;
         {
            // Limit scope where we need SCType
            struct SCType *TrgS = &SC[Isc_Ref];
            TrgSB               = TrgS->B;
            TrgDSM              = &TrgS->DSM;
         }
         if (target_num != 0) {
            // get relative orientation of body to B[0] then apply this to
            // AC.qbn
            double qbB[4] = {0.0}, wBnb[3] = {0.0}, wBnbDSM[3] = {0.0};
            QxQT(TrgSB[target_num].qn, TrgSB[0].qn, qbB);
            QxQ(qbB, TrgDSM->qbn, qbn);

            // get relative angular velocity of body to B[0], then apply this to
            // AC.wbn; all in B[frame_body] frame
            QxV(qbB, TrgSB[0].wn, wBnb);
            QxV(qbB, TrgDSM->wbn, wBnbDSM);
            for (i = 0; i < 3; i++)
               wbn[i] = wBnbDSM[i] + (TrgSB[target_num].wn[i] - wBnb[i]);
         }
         else {
            for (i = 0; i < 3; i++) {
               qbn[i] = TrgDSM->qbn[i];
               wbn[i] = TrgDSM->wbn[i];
            }
            qbn[3] = TrgDSM->qbn[3];
         }
         QxQT(DSM->qbn, qbn, Cmd->qbr);
         QTxV(TrgDSM->qbn, wbn, Cmd->wrn);
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
void NavigationModule(struct AcType *const AC, struct DSMType *const DSM)
{
   struct DSMNavType *Nav = &DSM->DsmNav;

   if (Nav->NavigationActive == FALSE) {
      // TODO
      for (int i = 0; i < 3; i++) {
         DSM->PosN[i] = AC->PosN[i];
         DSM->VelN[i] = AC->VelN[i];
         DSM->PosR[i] = DSM->PosN[i] - DSM->refOrb->PosN[i];
         DSM->VelR[i] = DSM->VelN[i] - DSM->refOrb->VelN[i];

         for (int j = 0; j < 3; j++)
            DSM->CBN[i][j] = AC->CBN[i][j];
         DSM->qbn[i] = AC->qbn[i];
         DSM->wbn[i] = AC->wbn[i];

         DSM->svb[i] = AC->svb[i];
         DSM->svn[i] = AC->svn[i];
         DSM->bvb[i] = AC->bvb[i];
         DSM->bvn[i] = AC->bvn[i];
      }
      DSM->qbn[3] = AC->qbn[3];
      return;
   }

   KalmanFilt(AC, DSM);
   struct DateType *navDate = &Nav->Date;
   DSM->Time = DateToTime(navDate->Year, navDate->Month, navDate->Day,
                          navDate->Hour, navDate->Minute, navDate->Second);
   // Overwrite data in AC structure with filtered data
   for (enum navState state = INIT_STATE; state <= FIN_STATE; state++) {
      if (Nav->stateActive[state] == TRUE) {
         // TODO: what to do for states that are not active in Nav?
         double tmp3Vec[3] = {0.0}, tmpQ[4] = {0.0};
         switch (state) {
            case TIME_STATE:
               // AC->Time = Nav->Time;
               break;
            case ROTMAT_STATE:
               MTxM(Nav->CRB, Nav->refCRN, DSM->CBN);
               C2Q(DSM->CBN, DSM->qbn);
               break;
            case QUAT_STATE:
               C2Q(Nav->refCRN, tmpQ);
               QxQ(Nav->qbr, tmpQ, DSM->qbn);
               Q2C(DSM->qbn, DSM->CBN);
               break;
            case POS_STATE:
               for (int i = 0; i < 3; i++)
                  tmp3Vec[i] = Nav->PosR[i] + Nav->refPos[i];
               MTxV(Nav->refCRN, tmp3Vec, DSM->PosN);
               for (int i = 0; i < 3; i++)
                  DSM->PosR[i] = DSM->PosN[i] - DSM->refOrb->PosN[i];
               break;
            case VEL_STATE:
               // will need more (BKE) for non-inertial frame
               for (int i = 0; i < 3; i++)
                  tmp3Vec[i] = Nav->VelR[i] + Nav->refVel[i];
               MTxV(Nav->refCRN, tmp3Vec, DSM->VelN);
               for (int i = 0; i < 3; i++)
                  DSM->VelR[i] = DSM->VelN[i] - DSM->refOrb->VelN[i];
               break;
            case OMEGA_STATE:
               MTxV(Nav->CRB, Nav->refOmega, tmp3Vec);
               for (int i = 0; i < 3; i++)
                  DSM->wbn[i] = Nav->wbr[i] + tmp3Vec[i];
               break;
            default:
               break;
         }
      }
   }

   if (Nav->stateActive[ROTMAT_STATE] == TRUE ||
       Nav->stateActive[QUAT_STATE] == TRUE) {
      if (Nav->sensorActive[MAG_SENSOR] == TRUE)
         MxV(DSM->CBN, DSM->bvn, DSM->bvb);
      if (Nav->sensorActive[CSS_SENSOR] == TRUE ||
          Nav->sensorActive[FSS_SENSOR] == TRUE)
         MxV(DSM->CBN, DSM->svn, DSM->svb);
   }
}
//------------------------------------------------------------------------------
void TranslationalNavigation(struct AcType *AC, struct DSMType *DSM)
{
   for (int i = 0; i < 3; i++) {
      AC->PosN[i] = DSM->PosN[i];
      AC->VelN[i] = DSM->VelN[i];
   }
}
//------------------------------------------------------------------------------
void AttitudeNavigation(struct AcType *AC, struct DSMType *DSM)
{
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++)
         AC->CBN[i][j] = DSM->CBN[i][j];
      AC->qbn[i] = DSM->qbn[i];
      AC->wbn[i] = DSM->wbn[i];
   }
   AC->qbn[3] = DSM->qbn[3];
}
//------------------------------------------------------------------------------
//                                 CONTROL
//------------------------------------------------------------------------------
void TranslationCtrl(struct DSMType *DSM)
{
   long i;

   struct DSMCtrlType *CTRL;
   struct DSMCmdType *Cmd;

   CTRL = &DSM->DsmCtrl;
   Cmd  = &DSM->Cmd;

   if (Cmd->TranslationCtrlActive == TRUE && Cmd->ManeuverMode == INACTIVE) {
      // TODO: do we want S->PosR to be AC->PosR? and DSM->FcmdB to be
      // CTRL->FcmdB??
      if (Cmd->trn_controller == PID_CNTRL) { // PID Controller
         if (Cmd->NewTrnGainsProcessed == TRUE) {
            for (i = 0; i < 3; i++)
               DSM->trn_ei[i] = 0.0;
            Cmd->NewTrnGainsProcessed = FALSE;
         }

         for (i = 0; i < 3; i++) {
            DSM->perr[i]    = DSM->PosR[i] - CTRL->CmdPosR[i]; // Position Error
            DSM->verr[i]    = DSM->VelR[i] - CTRL->CmdVelR[i]; // Velocity Error
            DSM->trn_ei[i] += (DSM->Oldperr[i] + DSM->perr[i]) / 2.0 *
                              DSM->DT; // Integrated Error

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
         QxV(DSM->qbn, CTRL->FcmdN,
             CTRL->FcmdB); // Converting from Inertial to body frame for Report
         for (i = 0; i < 3; i++) {
            if (CTRL->FrcB_max[i] > 0)
               CTRL->FcmdB[i] =
                   Limit(CTRL->FcmdB[i], -CTRL->FrcB_max[i],
                         CTRL->FrcB_max[i]); // Limiting AC->Frc in body frame
         }
         QTxV(DSM->qbn, CTRL->FcmdB, CTRL->FcmdN);
      }
      else if (Cmd->trn_controller == LYA_2BODY_CNTRL) {
         // Calculate relative radius, velocity
         for (i = 0; i < 3; i++) {
            DSM->perr[i] =
                DSM->PosR[i] - CTRL->CmdPosR[i]; // Position Error, Relative
            DSM->verr[i] = DSM->VelR[i] - CTRL->CmdVelR[i]; // Velocity Error
         }

         double r_norm  = MAGV(DSM->PosN);
         double r_cntrl = MAGV(CTRL->CmdPosN);
         double mu      = DSM->refOrb->mu;

         double dg[3];
         for (i = 0; i < 3; i++)
            dg[i] = -mu / pow(r_norm, 3) * DSM->PosN[i] +
                    mu / pow(r_cntrl, 3) * CTRL->CmdPosN[i];

         for (i = 0; i < 3; i++) {
            CTRL->FcmdN[i] = -CTRL->trn_kp[i] * DSM->perr[i] -
                             CTRL->trn_kr[i] * DSM->verr[i] - dg[i] * DSM->mass;
         }
         QxV(DSM->qbn, CTRL->FcmdN,
             CTRL->FcmdB); // Converting from Inertial to body frame for Report
         for (i = 0; i < 3; i++) {
            if (CTRL->FrcB_max[i] > 0)
               CTRL->FcmdB[i] =
                   Limit(CTRL->FcmdB[i], -CTRL->FrcB_max[i],
                         CTRL->FrcB_max[i]); // Limiting AC->Frc in body frame
         }
         QTxV(DSM->qbn, CTRL->FcmdB, CTRL->FcmdN);
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
                  CTRL->FcmdN[i] = DSM->mass * Cmd->DeltaV[i] / Cmd->BurnTime;
               }
               QxV(DSM->qbn, CTRL->FcmdN,
                   CTRL->FcmdB); // Converting from Inertial to body frame for
                                 // Report
            }
            else if (!strcmp(Cmd->RefFrame, "B")) {
               for (i = 0; i < 3; i++) {
                  CTRL->FcmdB[i] = DSM->mass * Cmd->DeltaV[i] / Cmd->BurnTime;
               }
            }
            for (i = 0; i < 3; i++) {
               if (CTRL->FrcB_max[i] > 0) {
                  CTRL->FcmdB[i] = Limit(
                      CTRL->FcmdB[i], -CTRL->FrcB_max[i],
                      CTRL->FrcB_max[i]); // Limiting AC->Frc in body frame
               }
            }
            QTxV(DSM->qbn, CTRL->FcmdB,
                 CTRL->FcmdN); // Converting back to Inertial from body frame
         }
         else if (Cmd->ManeuverMode == SMOOTHED) {
            const double coef =
                -2 * atanh(-0.99998); // .99998 corresponds to capturing 99.999%
                                      // of the burn since tanh has an asymptote
            double sharp = coef / Cmd->BurnTime;
            double t_mid = Cmd->BurnStopTime - Cmd->BurnTime / 2.0;
            double t_since_mid =
                SimTime - t_mid; // Time elapsed since middle of burn
            double coshSharp  = cosh(sharp * t_since_mid);
            double coshSharp2 = coshSharp * coshSharp;

            if (!strcmp(Cmd->RefFrame, "N")) {
               for (i = 0; i < 3; i++) {
                  CTRL->FcmdN[i] =
                      DSM->mass * (Cmd->DeltaV[i] * sharp / 2.0) / coshSharp2;
                  CTRL->FcmdN[i] =
                      DSM->mass * (Cmd->DeltaV[i] * sharp / 2.0) / coshSharp2;
               }
               QxV(DSM->qbn, CTRL->FcmdN,
                   CTRL->FcmdB); // Converting from Inertial to body frame for
                                 // Report
            }
            else if (!strcmp(Cmd->RefFrame, "B")) {
               for (i = 0; i < 3; i++) {
                  CTRL->FcmdB[i] =
                      DSM->mass * (Cmd->DeltaV[i] * sharp / 2.0) / coshSharp2;
               }
            }
            for (i = 0; i < 3; i++) {
               if (CTRL->FrcB_max[i] > 0) {
                  CTRL->FcmdB[i] = Limit(
                      CTRL->FcmdB[i], -CTRL->FrcB_max[i],
                      CTRL->FrcB_max[i]); // Limiting AC->Frc in body frame
               }
            }
            QTxV(DSM->qbn, CTRL->FcmdB,
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
void AttitudeCtrl(struct DSMType *DSM)
{
   long i;
   double wrb[3];

   struct DSMCmdType *Cmd;
   struct DSMCtrlType *CTRL;

   Cmd  = &DSM->Cmd;
   CTRL = &DSM->DsmCtrl;

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
                              DSM->DT; // Integrated angle error

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
         // calculate nonlinear term in Quaternion Lyapunov stability
         vxMov(DSM->werr, DSM->MOI, om_x_I_om);

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
   // load the DSM file statically so that all DsmFSW calls have access to same
   // object. Document is destroyed at program exit
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
   // DsmSensorModule(S);

   // Navigation Modules
   {
      // TODO: this
      for (int i = 0; i < 3; i++) {
         DSM->bvn[i] = AC->bvn[i];
         DSM->svn[i] = AC->svn[i];
      }
   }
   NavigationModule(AC, DSM);
   TranslationalNavigation(AC, DSM);
   AttitudeNavigation(AC, DSM);

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

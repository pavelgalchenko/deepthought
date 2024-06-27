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
void ThrProcessingMinPower(struct SCType *S) {
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
void InitThrDistVecs(struct AcType *AC, int DOF, long controllerState) {
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
            } else if (controllerState == ATT_STATE ||
                       controllerState == DMP_STATE) {
               A[j][i] = AC->Thr[i].rxA[j];
            }
         } else if (DOF == 6) {
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
            } else if (controllerState == ATT_STATE ||
                       controllerState == DMP_STATE) {
               AC->Thr[i].DistVec[j + 3] = APlus[i][j];
            }
         }
      } else if (DOF == 6) {
         for (j = 0; j < DOF; j++) {
            AC->Thr[i].DistVec[j] = APlus[i][j];
         }
      }
   }
   DestroyMatrix(A, DOF);
   DestroyMatrix(APlus, AC->Nthr);
}
//------------------------------------------------------------------------------
//                           Initialize DSM Structure
//------------------------------------------------------------------------------
void InitDSM(struct SCType *S) {

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

//----------------------------------- GAINS -----------------------------------
long GetGains(struct SCType *S, const char GainCmdName[255],
              long controllerState, FILE *InpDsmFilePtr) {
   long GainsProcessed = FALSE;
   int gainNum;
   char DsmCmdLine[255] = "";
   char GainMode[30], GainCmd[255];
   long i;
   double omega, zeta, alpha, k_lya, limit;

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   struct AcType *AC;
   double kp[3], kr[3], ki[3], limit_vec[3];
   int controller;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;
   AC  = &S->AC;

   strcpy(GainCmd, GainCmdName);
   if (sscanf(GainCmd, "Gains_[%d]", &gainNum) != 1) {
      GainsProcessed = FALSE;
      return (GainsProcessed);
   }

   switch (controllerState) {
      case TRN_STATE:
         controller = Cmd->trn_controller;
         break;
      case ATT_STATE:
         controller = Cmd->att_controller;
         break;
      case FULL_STATE:
         // PLACEHOLDER
         break;
      case DMP_STATE:
         // PLACEHOLDER
         break;
      default:
         break;
   }

   strcat(GainCmd, " %s");
   rewind(InpDsmFilePtr);
   while (fgets(DsmCmdLine, 255, InpDsmFilePtr)) {
      if (sscanf(DsmCmdLine, GainCmd, &GainMode) == 1) {
         if (!strcmp(GainMode, "PID")) {
            strcat(GainCmd, " Kp %lf %lf %lf Kr %lf %lf %lf Ki %lf %lf %lf "
                            "Ki_Limit %lf %lf %lf");
            if (sscanf(DsmCmdLine, GainCmd, GainMode, &kp[0], &kp[1], &kp[2],
                       &kr[0], &kr[1], &kr[2], &ki[0], &ki[1], &ki[2],
                       &limit_vec[0], &limit_vec[1], &limit_vec[2]) == 13)
               if (controller == PID_CNTRL)
                  GainsProcessed = TRUE;
         } else if (!strcmp(GainMode, "PID_WN")) {
            strcat(GainCmd, " %lf %lf %lf %lf");
            if (sscanf(DsmCmdLine, GainCmd, &GainMode, &omega, &zeta, &alpha,
                       &limit) == 5) {
               for (i = 0; i < 3; i++) {
                  kp[i]        = (2 * zeta * alpha + 1) * pow(omega, 2);
                  kr[i]        = (2 * zeta + alpha) * omega;
                  ki[i]        = alpha * pow(omega, 3);
                  limit_vec[i] = limit;
               }
               if (controller == PID_CNTRL)
                  GainsProcessed = TRUE;
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
         } else if (!strcmp(GainMode, "FC_LYA")) {
            switch (controller) {
               case LYA_2BODY_CNTRL:
                  strcat(GainCmd, " %lf %lf");
                  if (sscanf(DsmCmdLine, GainCmd, GainMode, &omega, &zeta) ==
                      3) {
                     for (i = 0; i < 3; i++) {
                        kp[i] = pow(omega, 2) * AC->mass;
                        kr[i] = 2 * zeta * omega * AC->mass;
                     }
                     GainsProcessed = TRUE;
                  }
                  break;
               case LYA_ATT_CNTRL:
                  strcat(GainCmd, " %lf");
                  if (sscanf(DsmCmdLine, GainCmd, GainMode, &k_lya) == 2) {
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
         } else if (!strcmp(GainMode, "MomentumDump")) {
            if (controllerState != DMP_STATE) {
               printf("%s gain sets can only be used for momentum dumping. "
                      "Exiting...",
                      GainMode);
               exit(EXIT_FAILURE);
            }
            strcat(GainCmd, " %lf %lf %lf");
            if (sscanf(DsmCmdLine, GainCmd, GainMode, &kp[0], &kp[1], &kp[2]) ==
                4)
               GainsProcessed = TRUE;
         }
         break;
      }
   }

   switch (controllerState) {
      case TRN_STATE:
         Cmd->trn_controller = controller;
         for (i = 0; i < 3; i++) {
            Cmd->trn_kp[i]      = kp[i];
            Cmd->trn_kr[i]      = kr[i];
            Cmd->trn_ki[i]      = ki[i];
            Cmd->trn_kilimit[i] = limit_vec[i];
         }
         break;
      case ATT_STATE:
         Cmd->att_controller = controller;
         for (i = 0; i < 3; i++) {
            Cmd->att_kp[i]      = kp[i];
            Cmd->att_kr[i]      = kr[i];
            Cmd->att_ki[i]      = ki[i];
            Cmd->att_kilimit[i] = limit;
         }
         break;
      case FULL_STATE:
         // PLACEHOLDER
         break;
      case DMP_STATE:
         for (i = 0; i < 3; i++) {
            Cmd->dmp_kp[i] = kp[i];
         }
         break;
      default:
         break;
   }

   if (GainsProcessed ==
       TRUE) { // Set GainsProcessed flag for relevant controller
      if (controllerState == TRN_STATE) {
         Cmd->NewTrnGainsProcessed = TRUE;
      } else if (controllerState == ATT_STATE) {
         Cmd->NewAttGainsProcessed = TRUE;
      }
   }

   return (GainsProcessed);
}
//----------------------------------- LIMITS -----------------------------------
long GetLimits(struct SCType *S, const char LimitCmdName[255],
               long controllerState, FILE *InpDsmFilePtr) {
   long LimitsProcessed = FALSE;
   int limitNum;
   char LimitCmd[255], DsmCmdLine[255] = "";
   long i;

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   double fMax[3], vMax[3];

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;

   strcpy(LimitCmd, LimitCmdName);
   if (sscanf(LimitCmd, "Limits_[%d]", &limitNum) != 1) {
      LimitsProcessed = FALSE;
      return (LimitsProcessed);
   }

   strcat(LimitCmd, " %lf %lf %lf %lf %lf %lf");
   rewind(InpDsmFilePtr);
   while (fgets(DsmCmdLine, 255, InpDsmFilePtr)) {
      if (sscanf(DsmCmdLine, LimitCmd, &fMax[0], &fMax[1], &fMax[2], &vMax[0],
                 &vMax[1], &vMax[2]) == 6) {
         LimitsProcessed = TRUE;
         if (controllerState == ATT_STATE) {
            for (i = 0; i < 3; i++)
               vMax[i] *= D2R;
         }
         break;
      }
   }

   switch (controllerState) {
      case TRN_STATE:
         for (i = 0; i < 3; i++) {
            Cmd->FrcB_max[i] = fMax[i];
            Cmd->vel_max[i]  = vMax[i];
         }
         break;
      case ATT_STATE:
         for (i = 0; i < 3; i++) {
            Cmd->Trq_max[i] = fMax[i];
            Cmd->w_max[i]   = vMax[i];
         }
         break;
      case FULL_STATE:
         // PLACEHOLDER
         break;
      case DMP_STATE:
         for (i = 0; i < 3; i++) {
            Cmd->dTrq_max[i] = fMax[i];
         }
         break;
      default:
         break;
   }

   return (LimitsProcessed);
}
//----------------------------------- CONTROLLER
//-----------------------------------
long GetController(struct SCType *S, const char CtrlCmdName[255],
                   long controllerState, FILE *InpDsmFilePtr) {
   long CntrlProcessed = FALSE;
   int cntrlNum;

   char CtrlCmd[255], DsmCmdLine[255] = "";
   char CtrlGainCmd[255], CtrlLimitCmd[255];
   char ControllerCmdMode[20];

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   int controller;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;

   strcpy(CtrlCmd, CtrlCmdName);
   if (sscanf(CtrlCmd, "Controller_[%d]", &cntrlNum) != 1) {
      CntrlProcessed = FALSE;
      return (CntrlProcessed);
   }

   strcat(CtrlCmd, " %s %s %s");
   rewind(InpDsmFilePtr);
   while (fgets(DsmCmdLine, 255, InpDsmFilePtr)) {
      if (sscanf(DsmCmdLine, CtrlCmd, &ControllerCmdMode, &CtrlGainCmd,
                 &CtrlLimitCmd) == 3) {
         if (!strcmp(ControllerCmdMode, "PID_CNTRL"))
            controller = PID_CNTRL;
         else if (!strcmp(ControllerCmdMode, "LYA_ATT_CNTRL"))
            controller = LYA_ATT_CNTRL;
         else if (!strcmp(ControllerCmdMode, "LYA_2BODY_CNTRL"))
            controller = LYA_2BODY_CNTRL;
         else if (!strcmp(ControllerCmdMode, "H_DUMP_CNTRL"))
            controller = H_DUMP_CNTRL;
         else {
            printf("%s is an invalid control type. Exiting...\n",
                   ControllerCmdMode);
            exit(EXIT_FAILURE);
         }
         // There should be a nicer way to handle this that doesn't require
         // hardcoding for things...
         if (controller == LYA_ATT_CNTRL && controllerState != ATT_STATE) {
            printf(
                "%s\n",
                "Can only use LYA_ATT_CNTRL for attitude control. Exiting...");
            exit(EXIT_FAILURE);
         }
         if (controller == H_DUMP_CNTRL && controllerState != DMP_STATE) {
            printf("%s\n", "Can only use H_DUMP_CNTRL for momentum dumping "
                           "control. Exiting...");
            exit(EXIT_FAILURE);
         }
         if (controller == LYA_2BODY_CNTRL && controllerState != TRN_STATE) {
            printf("%s\n", "Can only use LYA_2BODY_CNTRL for translation "
                           "control. Exiting...");
            exit(EXIT_FAILURE);
         }
         CntrlProcessed = TRUE;
         break;
      }
   }

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

   if (GetGains(S, CtrlGainCmd, controllerState, InpDsmFilePtr) == FALSE) {
      printf("For %s, could not find %s or invalid format. Exiting.g..\n",
             CtrlCmdName, CtrlGainCmd);
      exit(EXIT_FAILURE);
   }
   if (GetLimits(S, CtrlLimitCmd, controllerState, InpDsmFilePtr) == FALSE) {
      printf("For %s, could not find %s or invalid format. Exiting.d..\n",
             CtrlCmdName, CtrlLimitCmd);
      exit(EXIT_FAILURE);
   }

   return (CntrlProcessed);
}
//----------------------------------- ACTUATORS
//-----------------------------------
long GetActuators(struct SCType *S, const char ActuatorCmdName[255],
                  long controllerState, FILE *InpDsmFilePtr) {
   long ActuatorsProcessed = FALSE;
   int actuatorNum, H_DumpCmdMode;
   char ActuatorCmd[255], DsmCmdLine[255] = "";
   char ActuatorName[20];
   char actuator[20];

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   struct AcType *AC;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;
   AC  = &S->AC;

   strcpy(ActuatorCmd, ActuatorCmdName);
   if (sscanf(ActuatorCmd, "Actuators_[%d]", &actuatorNum) != 1) {
      ActuatorsProcessed = FALSE;
      return (ActuatorsProcessed);
   }

   strcat(ActuatorCmd, " %s");
   rewind(InpDsmFilePtr);
   while (fgets(DsmCmdLine, 255, InpDsmFilePtr)) {
      if (sscanf(DsmCmdLine, ActuatorCmd, &ActuatorName) == 1) {
         if (sscanf(ActuatorName, "WHL_[%d]", &H_DumpCmdMode) == 1) {
            strcpy(actuator, "WHL");
         } else {
            strcpy(actuator, ActuatorName);
            strcpy(Cmd->dmp_actuator,
                   ""); // Null it out if no dumping to avoid other errors
            if (controllerState == ATT_STATE)
               Cmd->H_DumpActive = FALSE;
         }
         ActuatorsProcessed = TRUE;
         break;
      }
   }
   // This handles invalid actuator names
   if (!strcmp(actuator, "WHL")) {
      if (controllerState == TRN_STATE || controllerState == DMP_STATE)
         ActuatorsProcessed = FALSE;
   } else if (!strcmp(actuator, "MTB")) {
      if (controllerState == TRN_STATE)
         ActuatorsProcessed = FALSE;
   } else if (!strcmp(actuator, "THR_3DOF")) {
      InitThrDistVecs(AC, 3, controllerState);
   } else if (!strcmp(actuator, "THR_6DOF")) {
      InitThrDistVecs(AC, 6, controllerState);
   } else if (!strcmp(actuator, "Ideal")) {
      // Ideal do what it wants
   } else {
      ActuatorsProcessed = FALSE;
   }

   switch (controllerState) {
      case TRN_STATE:
         strcpy(Cmd->trn_actuator, actuator);
         break;
      case ATT_STATE:
         strcpy(Cmd->att_actuator, actuator);
         break;
      case FULL_STATE:
         // PLACEHOLDER
         break;
      case DMP_STATE:
         strcpy(Cmd->dmp_actuator, actuator);
         break;
      default:
         break;
   }

   return (ActuatorsProcessed);
}
//------------------------- TRANSLATIONAL CMD ----------------------------------
long GetTranslationCmd(struct SCType *S, char TranslationCmd[255],
                       double DsmCmdTime, FILE *InpDsmFilePtr) {
   char DsmCmdLine[255] = "";
   char ActuatorCmd[255];
   char TranslationCmdName[255];
   long TranslationCmdProcessed = FALSE;
   char CtrlLimitCmd[255], ManeuverCmdMode[20];

   char ControllerCmd[255];

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;

   strcpy(TranslationCmdName, TranslationCmd);

   if (!strcmp(TranslationCmd, "NO_CHANGE")) {
      TranslationCmdProcessed = TRUE;
      return (TranslationCmdProcessed);
   } else if (!strcmp(TranslationCmd, "PASSIVE_TRN")) {
      Cmd->TranslationCtrlActive = FALSE;
      TranslationCmdProcessed    = TRUE;
      return (TranslationCmdProcessed);
   } else if (!strncmp(TranslationCmd, "TranslationCmd", 14)) {
      Cmd->TranslationCtrlActive = TRUE;
      strcat(TranslationCmd, " %lf %lf %lf %s %s %s %s");
      rewind(InpDsmFilePtr);
      while (
          fgets(DsmCmdLine, 255,
                InpDsmFilePtr)) { // Start Looping through file until reach EOF
         // Extract Translation Command
         if (sscanf(DsmCmdLine, TranslationCmd, &Cmd->Pos[0], &Cmd->Pos[1],
                    &Cmd->Pos[2], &Cmd->RefOrigin, &Cmd->RefFrame,
                    &ControllerCmd, &ActuatorCmd) == 7) {
            TranslationCmdProcessed = TRUE;
            Cmd->ManeuverMode       = INACTIVE;
            break;
         }
      }
      if (TranslationCmdProcessed == FALSE) {
         printf("Could not find %s or invalid format. Exiting...\n",
                TranslationCmdName);
         exit(EXIT_FAILURE);
      }
   } else if (!strncmp(TranslationCmd, "ManeuverCmd", 11)) {
      Cmd->TranslationCtrlActive = TRUE;
      strcat(TranslationCmd, " %lf %lf %lf %s %s %lf %s %s");
      rewind(InpDsmFilePtr);
      while (
          fgets(DsmCmdLine, 255,
                InpDsmFilePtr)) { // Start Looping through file until reach EOF
         // Extract Maneuver Command
         if (sscanf(DsmCmdLine, TranslationCmd, &Cmd->DeltaV[0],
                    &Cmd->DeltaV[1], &Cmd->DeltaV[2], &Cmd->RefFrame,
                    &ManeuverCmdMode, &Cmd->BurnTime, &CtrlLimitCmd,
                    &ActuatorCmd) == 8) {
            TranslationCmdProcessed = TRUE;
            Cmd->BurnStopTime       = DsmCmdTime + Cmd->BurnTime;
            if (!strcmp(ManeuverCmdMode, "CONSTANT"))
               Cmd->ManeuverMode = CONSTANT;
            else if (!strcmp(ManeuverCmdMode, "SMOOTHED"))
               Cmd->ManeuverMode = SMOOTHED;
            else {
               printf("%s is an invalid maneuver mode for %s. Exiting...",
                      ManeuverCmdMode, TranslationCmdName);
               exit(EXIT_FAILURE);
            }
            break;
         }
      }
      if (TranslationCmdProcessed == FALSE) {
         printf("Could not find %s or invalid format. Exiting...\n",
                TranslationCmdName);
         exit(EXIT_FAILURE);
      }
   }

   if (TranslationCmdProcessed == TRUE && Cmd->TranslationCtrlActive == TRUE) {
      if (Cmd->ManeuverMode == INACTIVE) {
         if (GetController(S, ControllerCmd, TRN_STATE, InpDsmFilePtr) ==
             FALSE) {
            printf("For %s, could not find %s or invalid format. Exiting...\n",
                   TranslationCmdName, ControllerCmd);
            exit(EXIT_FAILURE);
         }
      } else {
         if (GetLimits(S, CtrlLimitCmd, TRN_STATE, InpDsmFilePtr) == FALSE) {
            printf("For %s, could not find %s or invalid format. Exiting...\n",
                   TranslationCmdName, CtrlLimitCmd);
            exit(EXIT_FAILURE);
         }
      }
      if (GetActuators(S, ActuatorCmd, TRN_STATE, InpDsmFilePtr) == FALSE) {
         printf("For %s, could not find %s or invalid format. Exiting...\n",
                TranslationCmdName, ActuatorCmd);
         exit(EXIT_FAILURE);
      }
   }

   return (TranslationCmdProcessed);
}
//-----------------------ATTITUDE CMD ---------------------------------------
long GetAttitudeCmd(struct SCType *S, char AttitudeCmd[255],
                    FILE *InpDsmFilePtr) {
   char Junk[50], PriTrgType[20], SecTrgType[20], OnOff[20];
   char PriTargetCmd[255], SecTargetCmd[255], DsmCmdLine[255] = "";
   char ActuatorCmd[255];
   char AttCmdMethod[30], Target[50];
   char AttitudeCmdName[255];
   char PriTargetCmdName[255], SecTargetCmdName[255], GroundStationCmd[30];
   int AttPriCmdMode, AttSecCmdMode, AttCmdMode;
   long AttitudeCmdProcessed = FALSE, AttPriCmdProcessed = FALSE,
        AttSecCmdProcessed = FALSE;
   long Isc_trgt, PriTrgTypeFlag = 0, SecTrgTypeFlag = 0, GroundStationNum,
                  target_num;
   int state = ATT_STATE;

   char ControllerCmd[255];

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   struct DSMCmdVecType *PV, *SV;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;
   PV  = &Cmd->PriVec;
   SV  = &Cmd->SecVec;

   strcpy(AttitudeCmdName, AttitudeCmd);

   // Decode Attitude Command Method/Mode
   // TO DO: can we make it read %s_[%d]_[%d] instead of hard coded str??
   if (!strcmp(AttitudeCmd, "NO_CHANGE")) {
      AttitudeCmdProcessed = TRUE;
      return (AttitudeCmdProcessed);
   } else if (!strcmp(AttitudeCmd, "PASSIVE_ATT")) {
      Cmd->AttitudeCtrlActive = FALSE;
      AttitudeCmdProcessed    = TRUE;
      return (AttitudeCmdProcessed);
   } else if (sscanf(AttitudeCmd, "AttitudeCmd_PV[%d]_SV[%d]", &AttPriCmdMode,
                     &AttSecCmdMode) == 2 ||
              sscanf(AttitudeCmd, "AttitudeCmd_SV[%d]_PV[%d]", &AttSecCmdMode,
                     &AttPriCmdMode) ==
                  2) { // Decode Cmd into Method and Mode for setting params
      strcpy(AttCmdMethod, "AttitudeCmd");
      Cmd->AttitudeCtrlActive = TRUE;
   } else if (sscanf(AttitudeCmd, "AttitudeCmd_PV[%d]", &AttCmdMode) ==
              1) { // Decode Cmd into Method and Mode for setting params
      strcpy(AttCmdMethod, "UnitVectorCmd");
      Cmd->AttitudeCtrlActive = TRUE;
   } else if (sscanf(AttitudeCmd, "QuaternionCmd_[%d]", &AttCmdMode) == 1) {
      strcpy(AttCmdMethod, "QuaternionCmd");
      Cmd->AttitudeCtrlActive = TRUE;
   } else if (sscanf(AttitudeCmd, "MirrorCmd_[%d]", &AttCmdMode) == 1) {
      strcpy(AttCmdMethod, "MirrorCmd");
      Cmd->AttitudeCtrlActive = TRUE;
   } else if (sscanf(AttitudeCmd, "DetumbleCmd_[%d]", &AttCmdMode) == 1) {
      strcpy(AttCmdMethod, "DetumbleCmd");
      Cmd->AttitudeCtrlActive = TRUE;
   } else if (sscanf(AttitudeCmd, "WhlHManageCmd_[%d]", &AttCmdMode) == 1) {
      strcpy(AttCmdMethod, "WhlHManageCmd");
   } else {
      AttitudeCmdProcessed = FALSE;
      return (AttitudeCmdProcessed);
   }

   // Check Attitude Command Method
   if (!strcmp(AttCmdMethod, "AttitudeCmd")) {
      Cmd->Method = PARM_VECTORS;
      PV          = &Cmd->PriVec;
      SV          = &Cmd->SecVec;
      // Reconstruct Primary Axis Cmd
      sprintf(PriTargetCmd, "AttitudeCmd_PV[%d]", AttPriCmdMode);
      strcpy(PriTargetCmdName, PriTargetCmd);
      strcat(PriTargetCmd, " %s");
      // Reconstruct Secondary Axis Cmd
      sprintf(SecTargetCmd, "AttitudeCmd_SV[%d]", AttSecCmdMode);
      strcpy(SecTargetCmdName, SecTargetCmd);
      strcat(SecTargetCmd, " %s");

      // Find Primary & Secondary Axis Cmd Target Type
      rewind(InpDsmFilePtr);
      while (
          fgets(DsmCmdLine, 255,
                InpDsmFilePtr)) { // Start Looping through file until reach EOF
         if (sscanf(DsmCmdLine, PriTargetCmd, &PriTrgType) == 1) {
            PriTrgTypeFlag = 1;
         } else if (sscanf(DsmCmdLine, SecTargetCmd, &SecTrgType) == 1) {
            SecTrgTypeFlag = 1;
         }
         if (PriTrgTypeFlag == 1 && SecTrgTypeFlag == 1) {
            break;
         }
      }
      if (PriTrgTypeFlag == 0) {
         printf("For %s, could not find %s or invalid format. Exiting...\n",
                AttitudeCmdName, PriTargetCmdName);
         exit(EXIT_FAILURE);
      }
      if (SecTrgTypeFlag == 0) {
         printf("For %s, could not find %s or invalid format. Exiting...\n",
                AttitudeCmdName, SecTargetCmdName);
         exit(EXIT_FAILURE);
      }
      // Check Primary Target Type
      if (!strcmp(PriTrgType, "BODY") || !strcmp(PriTrgType, "SC")) {
         PV->CmdMode = CMD_TARGET;
         strcat(PriTargetCmd, " %lf %lf %lf %s %s %s");
         rewind(InpDsmFilePtr);
         while (fgets(
             DsmCmdLine, 255,
             InpDsmFilePtr)) { // Start Looping through file until reach EOF
            if (sscanf(DsmCmdLine, PriTargetCmd, Junk, &PV->cmd_axis[0],
                       &PV->cmd_axis[1], &PV->cmd_axis[2], &Target,
                       &ControllerCmd, &ActuatorCmd) == 7) {
               if (!strcmp(PriTrgType, "BODY")) {
                  PV->TrgType = TARGET_WORLD;
                  strcpy(GroundStationCmd, "GroundStation_[%ld]");
                  if (sscanf(Target, GroundStationCmd, &GroundStationNum) ==
                      1) {
                     PV->TrgWorld = GroundStation[GroundStationNum].World;
                     for (int i = 0; i < 3; i++)
                        PV->W[i] = GroundStation[GroundStationNum].PosW[i];
                  } else {
                     PV->TrgWorld = DecodeString(Target);
                     for (int i = 0; i < 3; i++)
                        PV->W[i] = 0.0;
                  }
               } else if (!strcmp(PriTrgType, "SC")) {
                  if (sscanf(Target, "SC[%ld].B[%ld]", &Isc_trgt,
                             &target_num) == 2) { // Decode Current SC ID Number
                     if (Isc_trgt >= Nsc) {
                        printf("This mission only has %ld spacecraft, but "
                               "spacecraft %ld was attempted to be set as the "
                               "primary target vector. Exiting...\n",
                               Nsc, Isc_trgt);
                        exit(EXIT_FAILURE);
                     }
                     if (target_num >= SC[Isc_trgt].Nb) {
                        printf("Spacecraft %ld only has %ld bodies, but the "
                               "primary target was attempted to be set as body "
                               "%ld. Exiting...\n",
                               Isc_trgt, SC[Isc_trgt].Nb, target_num);
                        exit(EXIT_FAILURE);
                     }
                     PV->TrgType = TARGET_SC;
                     PV->TrgSC   = Isc_trgt;
                     PV->TrgBody = target_num;
                  } else {
                     printf("%s is in incorrect format. Exiting...", Target);
                     exit(EXIT_FAILURE);
                  }
               }
               AttPriCmdProcessed = TRUE;
               break;
            }
         }
         if (AttPriCmdProcessed == FALSE) {
            printf(
                "%s has improper format for SC or BODY targeting. Exiting...\n",
                PriTargetCmdName);
            exit(EXIT_FAILURE);
         }
      } else if (!strcmp(PriTrgType, "VEC")) {
         PV->CmdMode = CMD_DIRECTION;
         PV->TrgType = TARGET_VEC;
         strcat(PriTargetCmd, " %lf %lf %lf %s %lf %lf %lf %s %s");
         rewind(InpDsmFilePtr);
         while (fgets(
             DsmCmdLine, 255,
             InpDsmFilePtr)) { // Start Looping through file until reach EOF
            if (sscanf(DsmCmdLine, PriTargetCmd, Junk, &PV->cmd_axis[0],
                       &PV->cmd_axis[1], &PV->cmd_axis[2], &Cmd->PriAttRefFrame,
                       &PV->cmd_vec[0], &PV->cmd_vec[1], &PV->cmd_vec[2],
                       &ControllerCmd, &ActuatorCmd) == 10) {
               AttPriCmdProcessed = TRUE;
               break;
            }
         }
         if (AttPriCmdProcessed == FALSE) {
            printf("%s has improper format for VEC targeting. Exiting...\n",
                   PriTargetCmdName);
            exit(EXIT_FAILURE);
         }
      } else {
         printf("For %s, %s is an invalid targeting type. Exiting...\n",
                PriTargetCmdName, PriTrgType);
         exit(EXIT_FAILURE);
      }
      // Check Secondary Target Type
      if (!strcmp(SecTrgType, "BODY") || !strcmp(SecTrgType, "SC")) {
         SV->CmdMode = CMD_TARGET;
         strcat(SecTargetCmd, " %lf %lf %lf %s");
         rewind(InpDsmFilePtr);
         while (fgets(
             DsmCmdLine, 255,
             InpDsmFilePtr)) { // Start Looping through file until reach EOF
            if (fscanf(InpDsmFilePtr, SecTargetCmd, Junk, &SV->cmd_axis[0],
                       &SV->cmd_axis[1], &SV->cmd_axis[2], &Target) == 5) {
               if (!strcmp(SecTrgType, "BODY")) {
                  SV->TrgType = TARGET_WORLD;
                  strcpy(GroundStationCmd, "GroundStation_[%ld]");
                  if (sscanf(Target, GroundStationCmd, &GroundStationNum) ==
                      1) {
                     SV->TrgWorld = GroundStation[GroundStationNum].World;
                     for (int i = 0; i < 3; i++)
                        SV->W[i] = GroundStation[GroundStationNum].PosW[i];
                  } else {
                     SV->TrgWorld = DecodeString(Target);
                     for (int i = 0; i < 3; i++)
                        SV->W[i] = 0.0;
                  }
               } else if (!strcmp(SecTrgType, "SC")) {
                  if (sscanf(Target, "SC[%ld].B[%ld]", &Isc_trgt,
                             &target_num) == 2) { // Decode Current SC ID Number
                     if (Isc_trgt >= Nsc) {
                        printf("This mission only has %ld spacecraft, but "
                               "spacecraft %ld was attempted to be set as the "
                               "secondary target vector. Exiting...\n",
                               Nsc, Isc_trgt);
                        exit(EXIT_FAILURE);
                     }
                     if (target_num >= SC[Isc_trgt].Nb) {
                        printf("Spacecraft %ld only has %ld bodies, but the "
                               "secondary target was attempted to be set as "
                               "body %ld. Exiting...\n",
                               Isc_trgt, SC[Isc_trgt].Nb, target_num);
                        exit(EXIT_FAILURE);
                     }
                     SV->TrgType = TARGET_SC;
                     SV->TrgSC   = Isc_trgt;
                     SV->TrgBody = target_num;
                  } else {
                     printf("%s is in incorrect format. Exiting...", Target);
                     exit(EXIT_FAILURE);
                  }
               }
               AttSecCmdProcessed = TRUE;
               break;
            }
         }
         if (AttSecCmdProcessed == FALSE) {
            printf(
                "%s has improper format for SC or BODY targeting. Exiting...\n",
                SecTargetCmdName);
            exit(EXIT_FAILURE);
         }
      } else if (!strcmp(SecTrgType, "VEC")) {
         SV->CmdMode = CMD_DIRECTION;
         SV->TrgType = TARGET_VEC;
         strcat(SecTargetCmd, " %lf %lf %lf %s %lf %lf %lf");
         rewind(InpDsmFilePtr);
         while (fgets(
             DsmCmdLine, 255,
             InpDsmFilePtr)) { // Start Looping through file until reach EOF
            if (fscanf(InpDsmFilePtr, SecTargetCmd, Junk, &SV->cmd_axis[0],
                       &SV->cmd_axis[1], &SV->cmd_axis[2], &Cmd->SecAttRefFrame,
                       &SV->cmd_vec[0], &SV->cmd_vec[1],
                       &SV->cmd_vec[2]) == 8) {
               AttSecCmdProcessed = TRUE;
               break;
            }
         }
         if (AttSecCmdProcessed == FALSE) {
            printf("%s has improper format for VEC targeting. Exiting...\n",
                   SecTargetCmdName);
            exit(EXIT_FAILURE);
         }
      } else {
         printf("For %s, %s is an invalid targeting type. Exiting...\n",
                SecTargetCmdName, SecTrgType);
         exit(EXIT_FAILURE);
      }
      if (AttPriCmdProcessed == TRUE && AttSecCmdProcessed == TRUE) {
         AttitudeCmdProcessed = TRUE;
      }
   } else if (!strcmp(AttCmdMethod, "UnitVectorCmd")) {
      Cmd->Method = PARM_UNITVECTOR;
      PV          = &Cmd->PriVec;
      // Reconstruct Primary Axis Cmd
      sprintf(PriTargetCmd, "AttitudeCmd_PV[%d]", AttCmdMode);
      strcpy(PriTargetCmdName, PriTargetCmd);
      strcat(PriTargetCmd, " %s");

      // Find Primary & Secondary Axis Cmd Target Type
      rewind(InpDsmFilePtr);
      while (
          fgets(DsmCmdLine, 255,
                InpDsmFilePtr)) { // Start Looping through file until reach EOF
         if (sscanf(DsmCmdLine, PriTargetCmd, &PriTrgType) == 1) {
            PriTrgTypeFlag = 1;
            break;
         }
      }

      if (PriTrgTypeFlag == 0) {
         printf("For %s, could not find %s or invalid format. Exiting...\n",
                AttitudeCmdName, PriTargetCmdName);
         exit(EXIT_FAILURE);
      }

      // Check Primary Target Type
      if (!strcmp(PriTrgType, "BODY") || !strcmp(PriTrgType, "SC")) {
         PV->CmdMode = CMD_TARGET;
         strcat(PriTargetCmd, " %lf %lf %lf %s %s %s");
         rewind(InpDsmFilePtr);
         while (fgets(
             DsmCmdLine, 255,
             InpDsmFilePtr)) { // Start Looping through file until reach EOF
            if (sscanf(DsmCmdLine, PriTargetCmd, Junk, &PV->cmd_axis[0],
                       &PV->cmd_axis[1], &PV->cmd_axis[2], &Target,
                       &ControllerCmd, &ActuatorCmd) == 7) {
               if (!strcmp(PriTrgType, "BODY")) {
                  PV->TrgType = TARGET_WORLD;
                  strcpy(GroundStationCmd, "GroundStation_[%ld]");
                  if (sscanf(Target, GroundStationCmd, &GroundStationNum) ==
                      1) {
                     PV->TrgWorld = GroundStation[GroundStationNum].World;
                     for (int i = 0; i < 3; i++)
                        PV->W[i] = GroundStation[GroundStationNum].PosW[i];
                  } else {
                     PV->TrgWorld = DecodeString(Target);
                     for (int i = 0; i < 3; i++)
                        PV->W[i] = 0.0;
                  }
               } else if (!strcmp(PriTrgType, "SC")) {
                  if (sscanf(Target, "SC[%ld].B[%ld]", &Isc_trgt,
                             &target_num) == 2) { // Decode Current SC ID Number
                     if (Isc_trgt >= Nsc) {
                        printf("This mission only has %ld spacecraft, but "
                               "spacecraft %ld was attempted to be set as the "
                               "primary target vector. Exiting...\n",
                               Nsc, Isc_trgt);
                        exit(EXIT_FAILURE);
                     }
                     if (target_num >= SC[Isc_trgt].Nb) {
                        printf("Spacecraft %ld only has %ld bodies, but the "
                               "primary target was attempted to be set as body "
                               "%ld. Exiting...\n",
                               Isc_trgt, SC[Isc_trgt].Nb, target_num);
                        exit(EXIT_FAILURE);
                     }
                     PV->TrgType = TARGET_SC;
                     PV->TrgSC   = Isc_trgt;
                     PV->TrgBody = target_num;
                  } else {
                     printf("%s is in incorrect format. Exiting...", Target);
                     exit(EXIT_FAILURE);
                  }
               }
               AttitudeCmdProcessed = TRUE;
               break;
            }
         }
         if (AttitudeCmdProcessed == FALSE) {
            printf(
                "%s has improper format for SC or BODY targeting. Exiting...\n",
                PriTargetCmdName);
            exit(EXIT_FAILURE);
         }
      } else if (!strcmp(PriTrgType, "VEC")) {
         PV->CmdMode = CMD_DIRECTION;
         PV->TrgType = TARGET_VEC;
         strcat(PriTargetCmd, " %lf %lf %lf %s %lf %lf %lf %s %s");
         rewind(InpDsmFilePtr);
         while (fgets(
             DsmCmdLine, 255,
             InpDsmFilePtr)) { // Start Looping through file until reach EOF
            if (sscanf(DsmCmdLine, PriTargetCmd, Junk, &PV->cmd_axis[0],
                       &PV->cmd_axis[1], &PV->cmd_axis[2], &Cmd->PriAttRefFrame,
                       &PV->cmd_vec[0], &PV->cmd_vec[1], &PV->cmd_vec[2],
                       &ControllerCmd, &ActuatorCmd) == 10) {
               AttitudeCmdProcessed = TRUE;
               break;
            }
         }
         if (AttitudeCmdProcessed == FALSE) {
            printf("%s has improper format for VEC targeting. Exiting...\n",
                   PriTargetCmdName);
            exit(EXIT_FAILURE);
         }
      } else {
         printf("For %s, %s is an invalid targeting type. Exiting...\n",
                PriTargetCmdName, PriTrgType);
         exit(EXIT_FAILURE);
      }
   } else if (!strcmp(AttCmdMethod, "QuaternionCmd")) {
      strcat(AttitudeCmd, " %lf %lf %lf %lf %s %s %s");
      rewind(InpDsmFilePtr);
      while (
          fgets(DsmCmdLine, 255,
                InpDsmFilePtr)) { // Start Looping through file until reach EOF
         if (sscanf(DsmCmdLine, AttitudeCmd, &Cmd->q[0], &Cmd->q[1], &Cmd->q[2],
                    &Cmd->q[3], &Cmd->AttRefFrame, &ControllerCmd,
                    &ActuatorCmd) == 7) {
            Cmd->Method          = PARM_QUATERNION;
            AttitudeCmdProcessed = TRUE;
            break;
         }
      }
      if (AttitudeCmdProcessed == FALSE) {
         printf("Could not find %s or invalid format. Exiting...\n",
                AttitudeCmdName);
         exit(EXIT_FAILURE);
      }
   } else if (!strcmp(AttCmdMethod, "MirrorCmd")) {
      strcat(AttitudeCmd, " %s %s %s");
      rewind(InpDsmFilePtr);
      while (
          fgets(DsmCmdLine, 255,
                InpDsmFilePtr)) { // Start Looping through file until reach EOF
         if (sscanf(DsmCmdLine, AttitudeCmd, &Cmd->AttRefScID, &ControllerCmd,
                    &ActuatorCmd) == 3) {
            Cmd->Method          = PARM_MIRROR;
            AttitudeCmdProcessed = TRUE;
            break;
         }
      }
      if (AttitudeCmdProcessed == FALSE) {
         printf("Could not find %s or invalid format. Exiting...\n",
                AttitudeCmdName);
         exit(EXIT_FAILURE);
      }
   } else if (!strcmp(AttCmdMethod, "DetumbleCmd")) {
      strcat(AttitudeCmd, " %s %s");
      rewind(InpDsmFilePtr);
      while (
          fgets(DsmCmdLine, 255,
                InpDsmFilePtr)) { // Start Looping through file until reach EOF
         if (sscanf(DsmCmdLine, AttitudeCmd, &ControllerCmd, &ActuatorCmd) ==
             2) {
            Cmd->Method          = PARM_DETUMBLE;
            AttitudeCmdProcessed = TRUE;
            break;
         }
      }
      if (AttitudeCmdProcessed == FALSE) {
         printf("Could not find %s or invalid format. Exiting...\n",
                AttitudeCmdName);
         exit(EXIT_FAILURE);
      }
   } else if (!strcmp(AttCmdMethod, "WhlHManageCmd")) {
      strcat(AttitudeCmd, " %s %lf %lf %s %s");
      rewind(InpDsmFilePtr);
      while (fgets(DsmCmdLine, 255, InpDsmFilePtr)) {
         if (sscanf(DsmCmdLine, AttitudeCmd, &OnOff, &Cmd->H_DumpLims[0],
                    &Cmd->H_DumpLims[1], &ControllerCmd, &ActuatorCmd) == 5) {
            state = DMP_STATE;
            if (!strcmp(OnOff, "ON"))
               Cmd->H_DumpActive = TRUE;
            else
               Cmd->H_DumpActive = FALSE;
            AttitudeCmdProcessed = TRUE;
            if (Cmd->H_DumpLims[1] < Cmd->H_DumpLims[0]) {
               printf("Maximum momentum dump limit must be more than the "
                      "minimum for %s. Exiting...\n",
                      AttitudeCmdName);
               exit(EXIT_FAILURE);
            }
         }
      }
      if (AttitudeCmdProcessed == FALSE) {
         printf("Could not find %s or invalid format. Exiting...\n",
                AttitudeCmdName);
         exit(EXIT_FAILURE);
      }
   }

   // Extract Controllers, Gains, Limits, & Actuators Corresponding to the Ctrl
   // mode commanded
   if (AttitudeCmdProcessed == TRUE && Cmd->AttitudeCtrlActive == TRUE) {
      if (GetController(S, ControllerCmd, state, InpDsmFilePtr) == FALSE) {
         printf("For %s, could not find %s or invalid format. Exiting...\n",
                AttitudeCmdName, ControllerCmd);
         exit(EXIT_FAILURE);
      }

      if (GetActuators(S, ActuatorCmd, state, InpDsmFilePtr) == FALSE) {
         printf("For %s, could not find %s or invalid format. Exiting...\n",
                AttitudeCmdName, ActuatorCmd);
         exit(EXIT_FAILURE);
      }
   }

   return (AttitudeCmdProcessed);
}
//----------------------- ACTUATOR CMD -----------------------------------------
long GetActuatorCmd(struct SCType *S, char ActuatorCmd[255],
                    FILE *InpDsmFilePtr) {
   char DsmCmdLine[2048] = "";
   char SubCommands[2048];
   char *each_command[20];
   int DsmCmdLinelength = 2048;
   char *token;
   int NumCommands;
   int i;
   long ActuatorCmdProcessed = FALSE;

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;
   struct AcType *AC;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;
   AC  = &S->AC;

   rewind(InpDsmFilePtr);
   strcat(
       ActuatorCmd,
       " NUM_CMD[%d] %[\040-\377]"); // First, find the number of subcommands in
                                     // the appropriate line and following cmnds

   while (fgets(DsmCmdLine, DsmCmdLinelength,
                InpDsmFilePtr)) { // Start Looping through file until reach EOF
      // Extract number of commands
      if (sscanf(DsmCmdLine, ActuatorCmd, &NumCommands, &SubCommands) == 2) {
         Cmd->ActNumCmds = NumCommands;
         i               = 0;
         token           = strtok(SubCommands, " ");
         while (token != NULL) {
            each_command[i++] = token;
            token             = strtok(NULL, " ");
         }
         if (i < NumCommands) {
            printf("Declared NUM_CMD[%d], but cound only find [%d] commands. "
                   "Exiting...\n",
                   NumCommands, i);
            exit(EXIT_FAILURE);
         }
         for (i = 0; i < NumCommands; i++) {
            if (sscanf(each_command[i], "WHL_[%d]_[%lf]", &Cmd->ActInds[i],
                       &Cmd->ActDuties[i]) == 2)
               Cmd->ActTypes[i] = WHL_TYPE;
            else if (sscanf(each_command[i], "THR_[%d]_[%lf]", &Cmd->ActInds[i],
                            &Cmd->ActDuties[i]) == 2)
               Cmd->ActTypes[i] = THR_TYPE;
            else if (sscanf(each_command[i], "MTB_[%d]_[%lf]", &Cmd->ActInds[i],
                            &Cmd->ActDuties[i]) == 2)
               Cmd->ActTypes[i] = MTB_TYPE;
            else {
               printf("The command %s is not a valid command. Exiting...\n",
                      each_command[i]);
               exit(EXIT_FAILURE);
            }
            if (Cmd->ActTypes[i] == WHL_TYPE && Cmd->ActInds[i] > AC->Nwhl) {
               printf("SC[%ld] only has %ld wheels, but an actuator command "
                      "was sent to wheel %d. Exiting...\n",
                      AC->ID, AC->Nwhl, Cmd->ActInds[i]);
               exit(EXIT_FAILURE);
            }
            if (Cmd->ActTypes[i] == THR_TYPE && Cmd->ActInds[i] > AC->Nthr) {
               printf("SC[%ld] only has %ld thrusters, but an actuator command "
                      "was sent to thruster %d. Exiting...\n",
                      AC->ID, AC->Nthr, Cmd->ActInds[i]);
               exit(EXIT_FAILURE);
            }
            if (Cmd->ActTypes[i] == MTB_TYPE && Cmd->ActInds[i] > AC->Nmtb) {
               printf("SC[%ld] only has %ld MTBs, but an actuator command was "
                      "sent to MTB %d. Exiting...\n",
                      AC->ID, AC->Nmtb, Cmd->ActInds[i]);
               exit(EXIT_FAILURE);
            }
         }
         ActuatorCmdProcessed = TRUE;
         break;
      } else if (sscanf(DsmCmdLine, ActuatorCmd, &NumCommands) == 1) {
         if (NumCommands == 0) {
            Cmd->ActNumCmds      = NumCommands;
            ActuatorCmdProcessed = TRUE;
            break;
         } else {
            printf("%d sub-commands were specified, but no subcommands were "
                   "found. Exiting...\n",
                   NumCommands);
            exit(EXIT_FAILURE);
         }
      }
   }

   return (ActuatorCmdProcessed);
}

//----------------------------- INTERPRETER (FIRST
//ITERATION)---------------------------------------
void DsmCmdInterpreterMrk1(struct SCType *S, FILE *InpDsmFilePtr) {
   char DSM_FileLine[1024] = "";
   char DSM_CmdLine[255];
   char Junk[50], ScID[50];
   int DSM_CmdLineLength = 1024;
   long i;
   long Isc;
   double DSM_CmdTime;

   struct DSMType *DSM;

   DSM = &S->DSM;

   DSM->CmdCnt = 0;
   for (i = 0; i < 100; i++)
      DSM->CmdTime_f[i] = 0.0;

   rewind(InpDsmFilePtr);
   while (fgets(DSM_FileLine, DSM_CmdLineLength,
                InpDsmFilePtr)) { // Start Looping through file until reach EOF
      // Read Spacecraft Command------------------------------------------------
      strcpy(DSM_CmdLine, "DSM_Cmd %s %s %lf");
      if (sscanf(DSM_FileLine, DSM_CmdLine, &ScID, Junk, &DSM_CmdTime) == 3) {
         if (sscanf(ScID, "SC[%ld]", &Isc) !=
             1) { // Decode Current SC ID Number
            printf("%s is not a valid Spacecraft Identifier. Exiting...\n",
                   ScID);
            exit(EXIT_FAILURE);
         }
         if (S->ID == Isc) {
            DSM->CmdCnt++;
            DSM->CmdTime_f[DSM->CmdCnt - 1] = DSM_CmdTime;
         }
      } else if (!strncmp(DSM_FileLine, " ", 1) ||
                 !strncmp(DSM_FileLine, "//", 2) ||
                 !strncmp(DSM_FileLine, "#", 1) ||
                 !strncmp(DSM_FileLine, "<", 1) ||
                 !strncmp(DSM_FileLine, "\n", 1) ||
                 !strncmp(DSM_FileLine, "\r", 1) ||
                 !strncmp(DSM_FileLine, "%", 1)) {
      } else if (!strncmp(DSM_FileLine, "End_Of_File", 11)) {
         break;
      } else {
         printf("%s from Inp_DSM.txt is not being interpreted. Exiting...\n",
                DSM_FileLine);
         exit(EXIT_FAILURE);
      }
   }

   DSM->CmdNextTime = DSM->CmdTime_f[0];
   DSM->CmdNum      = 0;
}

//----------------------------- INTERPRETER (SUBSEQUENT ITERATIONS)
//---------------------------------------
void DsmCmdInterpreterMrk2(struct SCType *S, FILE *InpDsmFilePtr) {
   char DSM_FileLine[1024] = "";
   char DSM_CmdLine[255];
   char Junk[50], ScID[50];
   char TranslationCmd[255], AttitudeCmd[255], ActuatorCmd[255];
   int DsmCmdLinelength = 1024;
   long Isc;
   double DSM_CmdTime;
   char *token;
   char SubCommands[2048];
   char *each_command[40];
   int NumCommands;
   char curCommand[40];
   long i;

   struct DSMType *DSM;
   struct DSMCmdType *Cmd;

   DSM = &S->DSM;
   Cmd = &DSM->Cmd;

   rewind(InpDsmFilePtr);
   while (fgets(DSM_FileLine, DsmCmdLinelength,
                InpDsmFilePtr)) { // Start Looping through file until reach EOF
      // Read Spacecraft Command------------------------------------------------
      strcpy(DSM_CmdLine, "DSM_Cmd %s %s %lf");
      if (sscanf(DSM_FileLine, DSM_CmdLine, &ScID, Junk, &DSM_CmdTime) == 3) {
         if (sscanf(ScID, "SC[%ld]", &Isc) !=
             1) { // Decode Current SC ID Number
            printf("%s is not a valid Spacecraft Identifier. Exiting...\n",
                   ScID);
            exit(EXIT_FAILURE);
         }
         if (S->ID == Isc && DSM->CmdNextTime == DSM_CmdTime) {
            strcat(DSM_CmdLine, " NUM_CMD[%d] %[\040-\377]");
            if (sscanf(DSM_FileLine, DSM_CmdLine, Junk, Junk, Junk,
                       &NumCommands, &SubCommands) == 5) {
               i     = 0;
               token = strtok(SubCommands, " ");
               while (token != NULL) {
                  each_command[i++] = token;
                  token             = strtok(NULL, " ");
               }
            }

            // Set Attitude and Translation as NO_CHANGE - will get overwritten
            // if there is a change
            strcpy(AttitudeCmd, "NO_CHANGE");
            strcpy(TranslationCmd, "NO_CHANGE");

            for (i = 0; i < NumCommands; i++) {
               strcpy(curCommand, each_command[i]);
               // Parse Attitude Commands
               if (!strncmp(curCommand, "DetumbleCmd", 11) ||
                   !strncmp(curCommand, "AttitudeCmd", 11) ||
                   !strncmp(curCommand, "QuaternionCmd", 13) ||
                   !strncmp(curCommand, "MirrorCmd", 9) ||
                   !strncmp(curCommand, "WhlHManageCmd", 13) ||
                   !strcmp(curCommand, "PASSIVE_ATT")) {
                  strcpy(AttitudeCmd, curCommand);
                  if (GetAttitudeCmd(S, AttitudeCmd, InpDsmFilePtr) == FALSE) {
                     printf("%s does not match any valid attitude command "
                            "methods found in Inp_DSM.txt. Exiting...\n",
                            AttitudeCmd);
                     exit(EXIT_FAILURE);
                  }
               }
               // Parse Translation Commands
               else if (!strncmp(curCommand, "TranslationCmd", 14) ||
                        !strncmp(curCommand, "ManeuverCmd", 11) ||
                        !strcmp(curCommand, "PASSIVE_TRN")) {
                  strcpy(TranslationCmd, each_command[i]);
                  if (GetTranslationCmd(S, TranslationCmd, DSM_CmdTime,
                                        InpDsmFilePtr) == FALSE) {
                     printf("%s does not match any valid translational command "
                            "methods found in Inp_DSM.txt. Exiting...\n",
                            TranslationCmd);
                     exit(EXIT_FAILURE);
                  }
               }
               // Parse Actuator Commands
               else if (!strncmp(curCommand, "ActuatorCmd", 11)) {
                  strcpy(ActuatorCmd, curCommand);
                  if (GetActuatorCmd(S, ActuatorCmd, InpDsmFilePtr) == FALSE) {
                     printf("%s does not match any valid actuator command "
                            "methods found in Inp_DSM.txt. Exiting...\n",
                            ActuatorCmd);
                     exit(EXIT_FAILURE);
                  }
               } else {
                  printf("%s is not a supported command. Exiting...\n",
                         curCommand);
                  exit(EXIT_FAILURE);
               }
            }
            // This sure is one of the if() statements of all time. I feel like
            // it can be reduced...
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
               printf(
                   "If the Translation actuator is 6DOF Thruster and Attitude "
                   "actuator is Thruster, then it must be 6DOF (and vice "
                   "versa).\nAdditionally, if the translation actuator is 3DOF "
                   "thruster, then Attitude cannot also be 3DOF. Exiting...\n");
               exit(EXIT_FAILURE);
            }
            break;
         }
      } else if (!strncmp(DSM_FileLine, " ", 1) ||
                 !strncmp(DSM_FileLine, "//", 2) ||
                 !strncmp(DSM_FileLine, "#", 1) ||
                 !strncmp(DSM_FileLine, "<", 1) ||
                 !strncmp(DSM_FileLine, "\n", 1) ||
                 !strncmp(DSM_FileLine, "\r", 1) ||
                 !strncmp(DSM_FileLine, "%", 1)) {
      } else if (!strncmp(DSM_FileLine, "End_Of_File", 11)) {
         printf("Reached End_Of_File without finding "
                "'DSM_Cmd\tSC[%li]\tCmdTime\t%e'. Exiting...\n",
                S->ID, DSM->CmdNextTime);
         exit(EXIT_FAILURE);
      } else {
         printf("%s from Inp_DSM.txt is not being interpreted. Exiting...\n",
                DSM_FileLine);
         exit(EXIT_FAILURE);
      }
   }
}
//------------------------------------------------------------------------------
//                                SENSORS
//------------------------------------------------------------------------------
void SensorModule(struct SCType *S) {

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
void ActuatorModule(struct SCType *S) {

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
      } else if (!strcmp(Cmd->trn_actuator, "Ideal")) {
         for (i = 0; i < 3; i++)
            AC->IdealFrc[i] = DSM->FcmdB[i];
      } else {
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
      } else if (!strcmp(Cmd->att_actuator, "WHL") && AC->Nwhl > 0) {
         for (i = 0; i < 3; i++)
            AC->Tcmd[i] = DSM->Tcmd[i];
         DSM_WheelProcessing(AC);
      } else if (!strcmp(Cmd->att_actuator, "MTB") && AC->Nmtb > 0) {
         CopyUnitV(AC->bvb, unit_bvb);
         VxV(unit_bvb, DSM->Tcmd, DSM->Mcmd);
         for (i = 0; i < 3; i++)
            AC->Mcmd[i] = DSM->Mcmd[i] / MAGV(AC->bvb);
         DSM_MtbProcessing(AC);
      } else if (!strcmp(Cmd->att_actuator, "Ideal")) {
         for (i = 0; i < 3; i++)
            AC->IdealTrq[i] = DSM->Tcmd[i];
      } else {
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
      } else if (DSM->DsmCtrl.H_DumpActive == TRUE &&
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
      } else if (DSM->DsmCtrl.H_DumpActive == TRUE &&
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
      } else if (Cmd->ActTypes[i] == THR_TYPE) {
         AC->Thr[Cmd->ActInds[i]].PulseWidthCmd =
             Cmd->ActDuties[i] / 100 * AC->DT;
         AC->Thr[Cmd->ActInds[i]].ThrustLevelCmd = Cmd->ActDuties[i] / 100;
      } else if (Cmd->ActTypes[i] == MTB_TYPE) {
         AC->MTB[Cmd->ActInds[i]].Mcmd =
             AC->MTB[i].Mmax * Cmd->ActDuties[i] / 100;
      }
   }
}
//------------------------------------------------------------------------------
//                                GUIDANCE
//------------------------------------------------------------------------------
void FindDsmCmdVecN(struct SCType *S, struct DSMCmdVecType *CV) {
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
         } else {
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
         } else if (Orb[SC[CV->TrgSC].RefOrb].World == Orb[S->RefOrb].World) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = SC[CV->TrgSC].PosN[i] - S->PosN[i];
               RelVelN[i] = SC[CV->TrgSC].VelN[i] - S->VelN[i];
            }
         } else {
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
         } else if (Orb[SC[CV->TrgSC].RefOrb].World == Orb[S->RefOrb].World) {
            for (i = 0; i < 3; i++) {
               RelPosN[i] = SC[CV->TrgSC].PosN[i] + pn[i] - S->PosN[i];
               RelVelN[i] = SC[CV->TrgSC].VelN[i] + vn[i] - S->VelN[i];
            }
         } else {
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
void TranslationGuidance(struct SCType *S) {

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
         } else if (F->FixedInFrame == 'N') {
            for (i = 0; i < 3; i++)
               wcn[i] = 0.0; // R does not rotate wrt R Inertial
            goodOriginFrame = TRUE;
         }
      } else if (!strcmp(Cmd->RefFrame, "N")) {
         for (i = 0; i < 3; i++)
            CTRL->CmdPosR[i] = Cmd->Pos[i]; // Already in R Inertial
         for (i = 0; i < 3; i++)
            wcn[i] = 0.0; // R does not rotate wrt R Inertial
         goodOriginFrame = TRUE;
      } else if (!strcmp(Cmd->RefFrame, "L")) {
         MTxV(S->CLN, Cmd->Pos, CTRL->CmdPosR); // Convert LVLH to R Inertial
         for (i = 0; i < 3; i++)
            wcn[i] = Orb[S->RefOrb].wln[i]; // L rotates wrt R
         goodOriginFrame = TRUE;
      } else if (!strncmp(Cmd->RefFrame, "SC",
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
      } else if (!strncmp(Cmd->RefOrigin, "SC",
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
      } else {
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
void AttitudeGuidance(struct SCType *S) {

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
         } else if (PV->TrgType == TARGET_VEC) {
            if (!strcmp(Cmd->PriAttRefFrame, "N")) {
               MxV(S->B[0].CN, PV->cmd_vec,
                   PriCmdVecB); // (Converting Cmd vec to body frame)
            } else if (!strcmp(Cmd->PriAttRefFrame, "F")) {
               MTxV(F->CN, PV->cmd_vec,
                    PriCmdVecN); // (Converting to Inertial frame)
               MxV(S->B[0].CN, PriCmdVecN,
                   PriCmdVecB); // (Converting to body frame) CHECK THIS
            } else if (!strcmp(Cmd->PriAttRefFrame, "L")) {
               MTxV(S->CLN, PV->cmd_vec,
                    PriCmdVecN); // (Converting to LVLH to Inertial frame)
               UNITV(PriCmdVecN);

               MxV(S->B[0].CN, PriCmdVecN,
                   PriCmdVecB); // (Converting to body frame)
            } else if (!strcmp(Cmd->PriAttRefFrame, "B")) {
               for (i = 0; i < 3; i++)
                  PriCmdVecB[i] = PV->cmd_vec[i];
            }
         }
         UNITV(PriCmdVecB);

         if (SV->TrgType == TARGET_SC || SV->TrgType == TARGET_WORLD) {
            FindDsmCmdVecN(S, SV); // to get SV->wn, SV->N
            MxV(S->B[0].CN, SV->N, SecCmdVecB);
         } else if (SV->TrgType == TARGET_VEC) {
            if (!strcmp(Cmd->SecAttRefFrame, "N")) {
               MxV(S->B[0].CN, SV->cmd_vec,
                   SecCmdVecB); // (Converting Cmd vec to body frame)
            } else if (!strcmp(Cmd->SecAttRefFrame, "F")) {
               MTxV(F->CN, SV->cmd_vec,
                    SecCmdVecN); // (Converting to Inertial frame)
               MxV(S->B[0].CN, SecCmdVecN,
                   SecCmdVecB); // (Converting to body frame)
            } else if (!strcmp(Cmd->SecAttRefFrame, "L")) {
               MTxV(S->CLN, SV->cmd_vec,
                    SecCmdVecN); // (Converting from LVLH to Inertial frame)
               UNITV(SecCmdVecN);
               MxV(S->B[0].CN, SecCmdVecN,
                   SecCmdVecB); // (Converting to body frame)
            } else if (!strcmp(Cmd->SecAttRefFrame, "B")) {
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
      } else if (Cmd->Method == PARM_UNITVECTOR) {
         printf("Feature for Singular Primary Unit Vector Pointing not "
                "currently fully implemented. Exiting...\n");
         exit(EXIT_FAILURE);
      } else if (Cmd->Method == PARM_QUATERNION) {
         if (!strcmp(Cmd->AttRefFrame, "N")) {
            for (i = 0; i < 4; i++)
               Cmd->qrn[i] = Cmd->q[i];
            QxQT(DSM->qbn, Cmd->qrn, Cmd->qbr);
            for (i = 0; i < 3; i++)
               Cmd->wrn[i] = 0.0;
         } else if (!strcmp(Cmd->AttRefFrame, "F")) {
            for (i = 0; i < 4; i++)
               Cmd->qrf[i] = Cmd->q[i];
            C2Q(F->CN, qfn);
            QxQ(Cmd->qrf, qfn, qrn);
            QxQT(DSM->qbn, qrn, Cmd->qbr);
            if (F->FixedInFrame == 'L') {
               for (i = 0; i < 3; i++)
                  Cmd->wrn[i] = Orb[S->RefOrb].wln[i]; // F rotates wrt N
            } else if (F->FixedInFrame == 'N') {
               for (i = 0; i < 3; i++)
                  Cmd->wrn[i] = 0.0; // N does not rotate wrt N Inertial
            }
         } else if (!strcmp(Cmd->AttRefFrame, "L")) {
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
      } else if (Cmd->Method == PARM_MIRROR) {
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
      } else if (Cmd->Method == PARM_DETUMBLE) {
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
void TranslationalNavigation(struct SCType *S) {

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
void AttitudeNavigation(struct SCType *S) {

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
void TranslationCtrl(struct SCType *S) {

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
      } else if (Cmd->trn_controller == LYA_2BODY_CNTRL) {
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
   } else if (Cmd->TranslationCtrlActive == TRUE &&
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
            } else if (!strcmp(Cmd->RefFrame, "B")) {
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
         } else if (Cmd->ManeuverMode == SMOOTHED) {
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
            } else if (!strcmp(Cmd->RefFrame, "B")) {
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
      } else {
         Cmd->ManeuverMode          = INACTIVE;
         Cmd->TranslationCtrlActive = FALSE;
         for (i = 0; i < 3; i++) {
            CTRL->FcmdN[i] = 0;
            CTRL->FcmdB[i] = 0;
         }
      }
   } else {
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
void AttitudeCtrl(struct SCType *S) {

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
      } else if (Cmd->att_controller == LYA_ATT_CNTRL) {
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
   } else {
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
void MomentumDumpCtrl(struct SCType *S) {

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
   } else {
      for (i = 0; i < 3; i++)
         CTRL->dTcmd[i] = 0.0;
   }

   for (i = 0; i < 3; i++)
      DSM->dTcmd[i] = CTRL->dTcmd[i];
}
//------------------------------------------------------------------------------
//                             FLIGHT SOFTWARE
//------------------------------------------------------------------------------
void DsmFSW(struct SCType *S) {

   FILE *dsm_in;

   struct DSMType *DSM;

   DSM = &S->DSM;

   // Run Command Interperter
   if (DSM->CmdInit) {
      DSM->CmdInit = 0;
      dsm_in       = FileOpen(InOutPath, "Inp_DSM.txt", "r");
      DsmCmdInterpreterMrk1(S, dsm_in);
      fclose(dsm_in);

      for (int i = 0; i < 3;
           i++) { // put place holders in integrator "old" values, set ei values
                  // to zero to initialize integrated error
         DSM->Oldtherr[i] = 0.0;
         DSM->Oldperr[i]  = 0.0;

         DSM->att_ei[i] = 0.0;
         DSM->trn_ei[i] = 0.0;
      }
   }

   if (SimTime >= DSM->CmdNextTime && DSM->CmdNum < DSM->CmdCnt) {
      dsm_in = FileOpen(InOutPath, "Inp_DSM.txt", "r");
      DsmCmdInterpreterMrk2(S, dsm_in);
      fclose(dsm_in);
      DSM->CmdNum++;
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

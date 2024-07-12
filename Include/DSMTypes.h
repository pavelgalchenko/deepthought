/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __DSMTYPES_H__
#define __DSMTYPES_H__

// Controller Type Definitions
enum ctrlType {
   PID_CNTRL = 0,   // Translational and Rotational
   LYA_ATT_CNTRL,   // Attitude
   LYA_2BODY_CNTRL, // 2 Body Relative Orbit Control
   H_DUMP_CNTRL,    // Proportional only, for momentum dumping
};

// Controller State Definitions
enum ctrlState {
   TRN_STATE = 0, // Translational
   ATT_STATE,     // Attitude
   FULL_STATE,    // 6DOF controller, currently unused
   DMP_STATE,     // Dumping Controller
};

// Manuever Type Definitions
enum maneuverType {
   INACTIVE = -1,
   CONSTANT,
   SMOOTHED,
};

// Actuator Type Definitions
enum actuatorType {
   WHL_TYPE = 0,
   THR_TYPE,
   MTB_TYPE,
};

/*
** #ifdef __cplusplus
** namespace _42 {
** #endif
*/

/* Variable tags for message building are delimited by [~ ~] */
/* > : Send from 42 to standalone */
/* < : Send from standalone to 42 */
/* = : Send both ways (eg. 42 Tx to 42 Rx) */
/* ! : Read from command file */
/* Example: [~!=~] means this variable can be read from command, and is sent
 * both ways */

struct DSMCmdVecType {
   /*~ Internal Variables ~*/
   long CmdMode;
   long Frame;
   long TrgType;
   long TrgWorld;
   long TrgSC;
   long TrgBody;
   double N[3];  /* Components in N */
   double W[3];  /* Components in W */
   double L[3];  /* Components in L */
   double R[3];  /* Components in R */
   double T[3];  /* Components in T */
   double wn[3]; /* Angular velocity in N, expressed in N */
   double cmd_vec[3];
   double cmd_axis[3];
};

struct DSMCmdType {
   /*~ Internal Variables ~*/
   long Method;
   char RefFrame[20];
   char RefOrigin[20];
   char AttRefFrame[20];
   char SecAttRefFrame[20];
   char PriAttRefFrame[20];
   long TranslationCtrlActive;
   long AttitudeCtrlActive;
   long H_DumpActive;
   long init;
   double AngRate[3];
   double Ang[3];
   double PosRate[3];
   long RotSeq;
   double qrl[4];
   double qrn[4];
   double qrf[4];
   double qbr[4];
   double wrn[3];
   double SpinRate;
   double Hvr[3];
   double Hvn[3];
   double OldCRN[3][3];
   double k_nute[3];      // Nutation Gain
   double k_prec[3];      // Precession Gain
   double trn_kp[3];      // Proportional Gain
   double trn_ki[3];      // Intergral Gain
   double trn_kr[3];      // Rate / Derivitive Gain
   double trn_kilimit[3]; // Integral Limit
   double dmp_kp[3];      // Dumping Proportional Gain
   double att_kp[3];      // Attitude Proportional Gain
   double att_ki[3];      // Attitude Intergral Gain
   double att_kr[3];      // Attitude Rate / Derivitive Gain
   double att_kilimit[3]; // Attitude Integral Limit
   double FrcB_max[3];    // Force limit in SC body frame
   double vel_max[3];     // Velocity limit in SC body frame
   double Trq_max[3];     // Torque limit in SC body frame
   double dTrq_max[3];    // Detumble torque limit in SC body frame
   double w_max[3];       // Angular velocity limit in SC body frame
   double Pos[3];         // Position Vector of wrt any frame
   double PosN[3];        // Position Vector of wrt Inertial frame N
   double PosR[3];        // Position Vector of wrt Inertial frame R
   double q[4];           // Quaternion wrt any frame
   char trn_actuator[20];
   char att_actuator[20];
   char dmp_actuator[20];
   enum ctrlType trn_controller;
   enum ctrlType att_controller;
   enum ctrlType dmp_controller;
   enum maneuverType ManeuverMode;
   char AttRefScID[6];
   char H_DumpGain[20];
   char H_DumpMode[20];
   double H_DumpLims[2];
   double DeltaV[3];
   double BurnTime;
   double TrgVelR[3];
   double BurnStopTime;
   enum actuatorType ActTypes[100];
   int ActInds[100];
   int ActNumCmds;
   double ActDuties[100];

   long NewAttGainsProcessed;
   long NewTrnGainsProcessed;

   /*~ Structures ~*/
   struct DSMCmdVecType PriVec;
   struct DSMCmdVecType SecVec;
};

struct DSMCtrlType {
   /*~ Parameters ~*/
   double trn_kp[3];   // Translational Proportional Gain
   double trn_kr[3];   // Translational Rate/derivitive Gain
   double trn_ki[3];   // Translational Integral Gain
   double dmp_kp[3];   // Momentum Gain
   double att_kp[3];   // Attitude Proportional Gain
   double att_kr[3];   // Attitude Derivative Gain
   double att_ki[3];   // Attitude Integral Gain
   double FrcB_max[3]; // Maximum Force / Force limit
   double FrcN_max[3]; // SC body Force limit in Inertial frame
   double vel_max[3];  // Maximum Velocity / Velocity limit
   double w_max[3];    // Maximum Angular Velocity / Angular Velocity limit
   double Trq_max[3];  // Maximum Torque / Torque limit
   double dTrq_max[3]; // Detumble torque limit in SC body frame

   /*~ Internal Variables ~*/
   long Init;
   long H_DumpActive; // Used interally to MomentumDumpCtrl()
   double qbr[4];
   double wrn[3];
   double therr[3];
   double werr[3];  // Angular velocity error
   double perr[3];  // Position error
   double verr[3];  // Velocity error
   double Tcmd[3];  // Torque Command
   double Mcmd[3];  // Magnetorquer Command
   double dTcmd[3]; // Dump Torque Command
   double FcmdN[3]; // Force Command in N frame
   double FcmdB[3]; // Force Command in SC B Frame
   double u1[3];
   double u2[3];
   double CmdPosN[3]; // Commanded Position in the Inertial frame (N)
   double CmdPosR[3]; // Commanded Position in the Inertial frame (R)
   double CmdVelN[3]; // Commanded Velocity in the Inertial frame (N)
   double CmdVelR[3]; // Commanded Velocity in the Inertial frame (R)
};

struct DSMType {
   /*~ Parameters ~*/
   long ID; /* Spacecraft ID */
   long Nb;
   long DsmTag; // Tag to designate DSM fsw modes

   /*~ Inputs ~*/
   double Time; /* Time since J2000 [[sec]] */
   long Mode;

   /*~ Outputs ~*/
   double Tcmd[3];  // Torque Command
   double Mcmd[3];  // Magnetorquer Command
   double dTcmd[3]; // Dump Torque Command
   double FcmdN[3]; // Force Command in N frame
   double FcmdB[3]; // Force Command in SC B Frame
   double VelR[3];  // Velocity in R Frame
   double PosR[3];  // Position in R Frame

   double therr[3]; // Angular Position Error
   double werr[3];  // Angular Velocity Error
   double perr[3];  // Position Error
   double verr[3];  // Velocity Error

   double trn_ei[3]; // translation error integral
   double att_ei[3]; // attitude error integral

   double Oldtherr[3]; // stores previous iteration's therr for integration
                       // purposes
   double
       Oldperr[3]; // stores previous iteration's therr for integration purposes

   double IdealTrq[3]; // Ideal Torque
   double IdealFrc[3]; // Ideal Force
   double wbn[3];      // Angular Velocity in the SC Body Frame
   double qbn[4];      // Quarternion from B to N

   double *CmdTime_f;
   long CmdNum;
   long CmdInit;
   long CmdCnt;
   double CmdNextTime;

   /*~ Internal Variables ~*/
   long Init;

   /*~ Structures ~*/
   struct DSMCtrlType DsmCtrl;
   struct DSMCmdType Cmd;
};

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __DSMTYPES_H__ */

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
extern struct SCType SCType;
#include "timekit.h"

#define REPORT_RESIDUALS FALSE

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

// Sensor Type Definitions
// need to arrange this in order of filtering preference
enum sensorType {
   NULL_SENSOR = -1,
   GPS_SENSOR,
   STARTRACK_SENSOR,
   FSS_SENSOR, // FSS before CSS so it can supersede css measurements
   CSS_SENSOR,
   GYRO_SENSOR,
   MAG_SENSOR,
   ACCEL_SENSOR,
};
// Update these to be the zeroth and last items in sensorType
#define INIT_SENSOR GPS_SENSOR
#define FIN_SENSOR  ACCEL_SENSOR

// Nav Filter Type Definitions
enum navType {
   IDEAL_NAV = 0, // get data direct from AC
   MEKF_NAV, // TODO: maybe make it EKF and have it go MEKF if Quaternion is
             // defined to be filtered??
   RIEKF_NAV,
   LIEKF_NAV,
};

// TODO: see about merging ROTMAT_STATE and QUAT_STATE
// Nav states to filter
enum navState {
   NULL_STATE = -2,
   ATTITUDE_STATE, // allows for nav dat to be either rotmat or quaternion data
   TIME_STATE,
   ROTMAT_STATE,
   QUAT_STATE,
   OMEGA_STATE,
   POS_STATE,
   VEL_STATE,
   // bias filtering???
   // MOI filtering???
   // actuation filtering???
};
// Update these to be the zeroth and last items in navState
#define INIT_STATE TIME_STATE
#define FIN_STATE  VEL_STATE

enum batchType {
   NONE_BATCH = 0,
   SENSOR_BATCH,
   TIME_BATCH,
};

enum originType {  // Start at -2 so Nav->refOriType >= 0 is the SC[#]
   ORI_WORLD = -2, // reference origin is celestial body
   ORI_OP,         // reference origin is orbit point
   ORI_SC,         // reference origin is SC, SC[#] = Nav->refOriType
};

#define ORDRK 4
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
struct DSMType;
struct AcType;
struct DSMMeasType {
   /*~ Parameters ~*/
   double time;
   long step;
   long subStep;
   double *data;

   /*~ Internal Variables ~*/
   long sensorNum;
   double *(*measFun)(struct AcType *const, struct DSMType *const, const long);
   double **(*measJacobianFun)(struct AcType *const, struct DSMType *const,
                               const long);
   enum sensorType type;
   int dim;
   int errDim;
   double *R;  // diagonal elements of measurement noise covariance
   double **N; // measurement noise mapping matrix
   double underWeighting;
   double probGate;

   struct DSMMeasType *nextMeas; // oh boy, a linked list
};
struct DSMMeasListType {
   // TODO: add a tail maybe??
   struct DSMMeasType *head;
   long length;
   long measDim;
};

struct DSMNavType {
   // This is set up for KF nav filter types, what about observers?
   // Would be nice if could set up for QUEST

   /*~ Parameters ~*/
   long NavigationActive;

   enum navType type;
   enum batchType batching;

   // These need to be figured out still
   long refFrame;   // nav reference frame
   long refOriType; // nav reference origin type
   void *refOriPtr; // pointer to object of nav reference origin, can be NULL,
   // ACType, WorldType, or OrbitType
   struct BodyType *refBodyPtr; // pointer to reference body, NULL if not used
   double refPos[3];            // PosN of nav origin
   double refVel[3];            // VelN of nav origin
   double refCRN[3][3];         // rotation from body to nav reference frame
   double refOmega[3];          // angular velocity of nav reference frame
   double refOmegaDot[3];

   double oldRefPos[3]; // PosN of nav origin
   double oldRefVel[3]; // VelN of nav origin
   double oldRefCRN[3][3];
   double oldRefOmega[3]; // angular velocity of nav reference frame
   double oldRefOmegaDot[3];
   double refLerpAlpha;
   double refAccel[3]; // VelN of nav origin

   /*~ Internal Variables ~*/
   long Init;
   long subStep;
   long subStepMax;
   long stateDim; // total dimension of navigation state space
   long navDim;   // total dimension of estimation error space
   long stateSize[FIN_STATE + 1];
   long navSize[FIN_STATE + 1];
   long stateInd[FIN_STATE + 1];
   long navInd[FIN_STATE + 1];
   long step;
   struct DateType Date0;
   struct DateType Date;
   double DT;
   double subStepSize;
   double **P; // Estimation Error Covariance
   double **S; // U from UDU factorization with D along diagonal
   double *delta;

   // maybe will make this an array with the size being the number of bodys for
   // the sc
   double ballisticCoef; // ballistic coefficient / mass []

   /*~ state information ~*/
   double CRB[3][3]; // Rotation from body to nav reference frame
   double qbr[4];    // Quaternion for CRB^T
   double PosR[3];   // Position of body relative to nav origin in terms of nav
                     // reference frame
   double VelR[3];   // Velocity of body relative to nav origin in terms of nav
                     // reference frame with respect to nav reference frame
   double
       wbr[3]; // Angular velocity of body frame relative to nav reference frame
               // in terms of body frame with respect to nav reference frame
   double *whlH;

   double torqueB[3];
   double forceB[3];

   double **NxN; // Pre-allocated navDim x navDim matrix for use in intermediary
                 // steps
   double **NxN2;     // Pre-allocated navDim x navDim matrix for use in
                      // intermediary steps
   double **jacobian; // EOM jacobian
   double **STM;      // state transition matrix
   double **M;        // dynamics noise mapping matrix
   double *sqrQ;      // Diagonal elements of noise covariance
   void (*EOMJacobianFun)(struct AcType *const, struct DSMType *const,
                          const struct DateType *, double const[3][3],
                          double const[4], double const[3], double const[3],
                          double const[3], double const[], const double);
   void (*updateLaw)(struct DSMNavType *const);
   // TODO: calculate the number of measurements per step and allocate an array
   // for measurements
   struct DSMMeasListType
       measList; // linked list of measurement buffer. Ordered by time. Head is
                 // measurement with the smallest time in the queue.

   // Use final element of relevant enums+1 to ensure these arrays are just as
   // big as needed
   struct DSMMeasType
       *measTypes[FIN_SENSOR + 1];   // index corresponding to enum sensorType
                                     // holds default sensor data
   int sensorActive[FIN_SENSOR + 1]; // TRUE/FALSE; index corresponding to enum
                                     // sensorType indicates sensor is used
   int nSensor[FIN_SENSOR +
               1]; // each index incates the number of the corresponding sensor
   int stateActive[FIN_STATE + 1]; // TRUE/FALSE; index corresponding to enum
                                   // navState indicates state is filtered

   double **residuals[FIN_SENSOR + 1];
   long reportConfigured;
};

struct DSMType {
   /*~ Parameters ~*/
   long ID; /* Spacecraft ID */
   long Nb;
   long DsmTag; // Tag to designate DSM fsw modes

   /*~ Inputs ~*/
   double Time; /* Time since J2000 [[sec]] */
   double DT;
   double mass;
   double MOI[3][3];
   long Mode;

   /*~ Outputs ~*/
   double Tcmd[3];  // Torque Command
   double Mcmd[3];  // Magnetorquer Command
   double dTcmd[3]; // Dump Torque Command
   double FcmdN[3]; // Force Command in N frame
   double FcmdB[3]; // Force Command in SC B Frame
   double VelR[3];  // Velocity in R Frame
   double PosR[3];  // Position in R Frame
   double VelN[3];  // Velocity in N Frame
   double PosN[3];  // Position in N Frame

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
   double qbn[4];      // Quarternion from N to B
   double CBN[3][3];   // Rotation Matrix from N to B

   double *CmdTime_f;
   long CmdNum;
   long CmdInit;
   long CmdCnt;
   double CmdNextTime;

   double svn[3];
   double svb[3];
   double bvn[3];
   double bvb[3];

   /*~ Internal Variables ~*/
   struct OrbitType *refOrb; // spacecraft's reference orbit
   long Init;

   /*~ Structures ~*/
   struct DSMCtrlType DsmCtrl;
   struct DSMCmdType Cmd;
   struct DSMNavType DsmNav;
};

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __DSMTYPES_H__ */

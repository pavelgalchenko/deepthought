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
#include "orbkit.h"
#include "timekit.h"

// Controller Type Definitions
enum CtrlType {
   PID_CNTRL = 0,   // Translational and Rotational
   LYA_ATT_CNTRL,   // Attitude
   LYA_2BODY_CNTRL, // 2 Body Relative Orbit Control
   H_DUMP_CNTRL,    // Proportional only, for momentum dumping
};

// Controller State Definitions
enum CtrlState {
   TRN_STATE = 0, // Translational
   ATT_STATE,     // Attitude
   FULL_STATE,    // 6DOF controller, currently unused
   DMP_STATE,     // Dumping Controller
};

// Manuever Type Definitions
enum ManeuverType {
   MAN_INACTIVE = -1,
   MAN_CONSTANT,
   MAN_SMOOTHED,
};

// Actuator Type Definitions
enum ActuatorType {
   ACT_WHL = 0,
   ACT_THR,
   ACT_MTB,
   ACT_IDEALFRC,
   ACT_IDEALTRQ,
};

// Sensor Type Definitions
// need to arrange this in order of filtering preference
enum SensorType {
   NULL_SENSOR = -1,
   GPS_SENSOR,
   STARTRACK_SENSOR,
   FSS_SENSOR, // FSS before CSS so it can supersede css measurements
   CSS_SENSOR,
   GYRO_SENSOR,
   MAG_SENSOR,
   ACCEL_SENSOR,
};
// Update these to be the zeroth and last items in SensorType
#define INIT_SENSOR (GPS_SENSOR)
#define FIN_SENSOR  (ACCEL_SENSOR)

#define FOR_SENSORS(x)                                                         \
   for (enum SensorType(x) = INIT_SENSOR; (x) <= FIN_SENSOR; (x)++)

// Nav Filter Type Definitions
enum NavType {
   IDEAL_NAV = 0, // get data direct from AC
   MEKF_NAV, // TODO: maybe make it EKF and have it go MEKF if Quaternion is
             // defined to be filtered??
   RIEKF_NAV,
   LIEKF_NAV,
};

// Nav states to filter
enum States {
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
// Update these to be the zeroth and last items in States
#define INIT_STATE (TIME_STATE)
#define FIN_STATE  (VEL_STATE)

#define FOR_STATES(x) for (enum States(x) = INIT_STATE; (x) <= FIN_STATE; (x)++)

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
   WorldID TrgWorld;
   long TrgSC;
   long TrgBody;
   vec3_t N;  /* Components in N */
   vec3_t W;  /* Components in W */
   vec3_t L;  /* Components in L */
   vec3_t R;  /* Components in R */
   vec3_t T;  /* Components in T */
   vec3_t wn; /* Angular velocity in N, expressed in N */
   vec3_t cmd_vec;
   vec3_t cmd_axis;
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
   vec3_t AngRate;
   vec3_t Ang;
   vec3_t PosRate;
   long RotSeq;
   quat_t qrl;
   quat_t qrn;
   quat_t qrf;
   quat_t qbr;
   vec3_t wrn;
   double SpinRate;
   vec3_t Hvr;
   vec3_t Hvn;
   mat3x3_t OldCRN;
   vec3_t k_nute;            // Nutation Gain
   vec3_t k_prec;            // Precession Gain
   vec3_t trn_kp;            // Proportional Gain
   vec3_t trn_ki;            // Intergral Gain
   vec3_t trn_kr;            // Rate / Derivitive Gain
   vec3_t trn_kilimit;       // Integral Limit
   vec3_t dmp_kp;            // Dumping Proportional Gain
   vec3_t att_kp;            // Attitude Proportional Gain
   vec3_t att_ki;            // Attitude Intergral Gain
   vec3_t att_kr;            // Attitude Rate / Derivitive Gain
   vec3_t att_kilimit;       // Attitude Integral Limit
   vec3_t FrcB_max;          // Force limit in SC body frame
   vec3_t vel_max;           // Velocity limit in SC body frame
   vec3_t Trq_max;           // Torque limit in SC body frame
   vec3_t dTrq_max;          // Detumble torque limit in SC body frame
   vec3_t w_max;             // Angular velocity limit in SC body frame
   vec3_t Pos;               // Position Vector of wrt any frame
   vec3_t PosN;              // Position Vector of wrt Inertial frame N
   vec3_t PosR;              // Position Vector of wrt Inertial frame R
   quat_t q;                 // Quaternion wrt any frame
   double Distance;          // target distance for EH maneuver
   double Phase;             // target degree for EH maneuver
   double TimeDock;          // target time period for EH docking
   long ResetTimer;          // resets EH timer
   double InitTime;          // start of EH guidance law execution
   double CurrentTimer;      // time after EH start
   char TranslationType[20]; // Docking or Circumnavigation or Position
   char trn_actuator[20];
   char att_actuator[20];
   char dmp_actuator[20];
   enum CtrlType trn_controller;
   enum CtrlType att_controller;
   enum CtrlType dmp_controller;
   enum ManeuverType ManeuverMode;
   char AttRefScID[20];
   char H_DumpGain[20];
   char H_DumpMode[20];
   double H_DumpLims[2];
   vec3_t DeltaV;
   double BurnTime;
   vec3_t TrgVelR;
   double BurnStopTime;
   enum ActuatorType ActTypes[100];
   int ActInds[100];
   int ActNumCmds;
   double ActDuties[100]; // Duty Cycle for non-ideal actuators, newtons for
                          // ideal force, newton-meters for ideal torque
   char ActIdealFrame[100][20]; // frame for ideal actuator
   vec3_t ActIdealDirs[100]; // unit vector of ideal actuator action direction

   long NewAttGainsProcessed;
   long NewTrnGainsProcessed;

   /*~ Structures ~*/
   struct DSMCmdVecType PriVec;
   struct DSMCmdVecType SecVec;
};

struct DSMCtrlType {
   /*~ Parameters ~*/
   vec3_t trn_kp;   // Translational Proportional Gain
   vec3_t trn_kr;   // Translational Rate/derivitive Gain
   vec3_t trn_ki;   // Translational Integral Gain
   vec3_t dmp_kp;   // Momentum Gain
   vec3_t att_kp;   // Attitude Proportional Gain
   vec3_t att_kr;   // Attitude Derivative Gain
   vec3_t att_ki;   // Attitude Integral Gain
   vec3_t FrcB_max; // Maximum Force / Force limit
   vec3_t FrcN_max; // SC body Force limit in Inertial frame
   vec3_t vel_max;  // Maximum Velocity / Velocity limit
   vec3_t w_max;    // Maximum Angular Velocity / Angular Velocity limit
   vec3_t Trq_max;  // Maximum Torque / Torque limit
   vec3_t dTrq_max; // Detumble torque limit in SC body frame

   /*~ Internal Variables ~*/
   long Init;
   long H_DumpActive; // Used interally to MomentumDumpCtrl()
   quat_t qbr;
   vec3_t wrn;
   vec3_t therr;
   vec3_t werr;  // Angular velocity error
   vec3_t perr;  // Position error
   vec3_t verr;  // Velocity error
   vec3_t Tcmd;  // Torque Command
   vec3_t Mcmd;  // Magnetorquer Command
   vec3_t dTcmd; // Dump Torque Command
   vec3_t FcmdN; // Force Command in N frame
   vec3_t FcmdB; // Force Command in SC B Frame
   vec3_t u1;
   vec3_t u2;
   vec3_t CmdPosN; // Commanded Position in the Inertial frame (N)
   vec3_t CmdPosR; // Commanded Position in the Inertial frame (R)
   vec3_t CmdVelN; // Commanded Velocity in the Inertial frame (N)
   vec3_t CmdVelR; // Commanded Velocity in the Inertial frame (R)
};
struct DSMType;
struct AcType;
struct DSMMeasType {
   /*~ Parameters ~*/
   double time;
   CCSDSTime ccsds_time;
   double *data;

   /*~ Internal Variables ~*/
   long sensorNum;
   double *(*measFun)(struct AcType *const, struct DSMType *const, const long);
   double **(*measJacobianFun)(struct AcType *const, struct DSMType *const,
                               const long, double **);
   enum SensorType type;
   int dim;
   int errDim;
   int noiseDim;
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

struct DSMStateType {
   // TODO: A HASH TABLE WILL DO WHAT I WANT!!!!! to make this more configurable
   // if this were C++, this would be MUCH easier
   // TODO: make commState a different structure that is user configurable
   // I would have already done this, but CBN complicates the issue
   // (double ** != double [][])
   // Figuring out how DSM guidance works with data not in commState would also
   // be interesting

   /*~ Parameters ~*/
   long ID; /* Spacecraft ID */

   /*~ Inputs ~*/
   double Time; /* Time since J2000 [[sec]] */

   /*~ Outputs ~*/
   vec3_t VelR;  // Velocity in R Frame
   vec3_t PosR;  // Position in R Frame
   vec3_t VelN;  // Velocity in N Frame
   vec3_t PosN;  // Position in N Frame
   vec3_t wbn;   // Angular Velocity in the SC Body Frame
   quat_t qbn;   // Quarternion from N to B
   mat3x3_t CBN; // Rotation Matrix from N to B

   vec3_t svn; // Sun vector in N frame
   vec3_t svb; // Sun vector in B frame
   vec3_t bvn; // Magnetic field vector in N frame
   vec3_t bvb; // Magnetic field vector in B frame
};

struct DSMNavType {
   // This is set up for KF nav filter types, what about observers?
   // Would be nice if could set up for QUEST

   /*~ Parameters ~*/
   long NavigationActive;

   enum NavType type;
   enum batchType batching;

   // These need to be figured out still
   long refFrame;   // nav reference frame
   long refOriType; // nav reference origin type
   long refOriBody; // nav reference origin type
   void *refOriPtr; // pointer to object of nav reference origin, can be NULL,
   // ACType, WorldType, or OrbitType
   struct BodyType *refBodyPtr; // pointer to reference body, NULL if not used
   vec3_t refPos;               // PosN of nav origin
   vec3_t refVel;               // VelN of nav origin
   mat3x3_t refCRN;             // rotation from body to nav reference frame
   vec3_t refOmega;             // angular velocity of nav reference frame
   vec3_t refOmegaDot;

   vec3_t oldRefPos; // PosN of nav origin
   vec3_t oldRefVel; // VelN of nav origin
   mat3x3_t oldRefCRN;
   vec3_t oldRefOmega; // angular velocity of nav reference frame
   vec3_t oldRefOmegaDot;
   double refLerpAlpha;
   vec3_t refAccel; // VelN of nav origin

   /*~ Internal Variables ~*/
   long Init;
   unsigned long steps;
   double subStepSize;
   long subStepSteps;    // number of ccsdsSubseconds counts per subStepSize
   CCSDSTime ccsds_time; // UTC, Epoch: Midnight Jan 1, 1958
   long stateDim;        // total dimension of navigation state space
   long navDim;          // total dimension of estimation error space
   long stateSize[FIN_STATE + 1];
   long navSize[FIN_STATE + 1];
   long stateInd[FIN_STATE + 1];
   long navInd[FIN_STATE + 1];
   JDType jd_tt_mjd_0; // TT
   JDType jd_tt_mjd;   // TT
   DateType Date;      // TT
   double DT;
   Rational DT_RAT;
   double **P; // Estimation Error Covariance, used only as scratch for
               // reporting and graphics
   double **S; // Lower-triangular Cholesky factorization of P
   double *delta;

   // maybe will make this an array with the size being the number of bodys for
   // the sc
   double ballisticCoef; // ballistic coefficient / mass []

   /*~ state information ~*/
   mat3x3_t CRB; // Rotation from body to nav reference frame
   quat_t qbr;   // Quaternion for CRB^T
   vec3_t PosR;  // Position of body relative to nav origin in terms of nav
                 // reference frame
   vec3_t VelR;  // Velocity of body relative to nav origin in terms of nav
                 // reference frame with respect to nav reference frame
   vec3_t wbr; // Angular velocity of body frame relative to nav reference frame
               // in terms of body frame with respect to nav reference frame
   double *whlH;

   vec3_t torqueB;
   vec3_t forceB;

   double **NxN; // Pre-allocated navDim x navDim matrix for use in intermediary
                 // steps
   double **NxN2;     // Pre-allocated navDim x navDim matrix for use in
                      // intermediary steps
   double **jacobian; // EOM jacobian
   double **STM;      // state transition matrix for subStepSize
   double **STMStep;  // STM for +1 CCSDS counts
   double **M;        // dynamics noise mapping matrix
   double *sqrQ;      // Diagonal elements of noise covariance
   void (*EOMJacobianFun)(struct AcType *const, struct DSMType *const,
                          const DateType *, const mat3x3_t, const quat_t,
                          const vec3_t, const vec3_t, const vec3_t,
                          double const[], const double, double **);
   void (*updateLaw)(struct DSMNavType *const);
   // linked list of measurement buffer. Ordered by time. Head is measurement
   // with the smallest time in the queue.
   struct DSMMeasListType measList;

   // Use final element of relevant enums+1 to ensure these arrays are just as
   // big as needed
   struct DSMMeasType
       *measTypes[FIN_SENSOR + 1];    // index corresponding to enum SensorType
                                      // holds default sensor data
   int *sensorActive[FIN_SENSOR + 1]; // TRUE/FALSE; index corresponding to enum
                                      // SensorType indicates sensor is used
   int nSensor[FIN_SENSOR + 1];       // each index incates the number of the
                                      // corresponding sensor
   int stateActive[FIN_STATE + 1];    // TRUE/FALSE; index corresponding to enum
                                      // States indicates state is filtered

   long innovationsReportFirst;
   double innovationTime;
   long innovationsExist;
   double **innovations[FIN_SENSOR + 1];
   long reportConfigured;
};

struct DSMType {
   /*~ Parameters ~*/
   long ID; /* Spacecraft ID */

   /*~ Inputs ~*/
   double DT;
   double mass;
   mat3x3_t MOI;
   long Mode;

   /*~ Outputs ~*/
   struct DSMStateType state;
   struct DSMStateType commState;
   // assign a function pointer to allow for this to be more general later
   struct DSMStateType (*CommStateProcessing)(struct DSMStateType);
   vec3_t Tcmd;  // Torque Command
   vec3_t Mcmd;  // Magnetorquer Command
   vec3_t dTcmd; // Dump Torque Command
   vec3_t FcmdN; // Force Command in N frame
   vec3_t FcmdB; // Force Command in SC B Frame

   vec3_t therr; // Angular Position Error
   vec3_t werr;  // Angular Velocity Error
   vec3_t perr;  // Position Error
   vec3_t verr;  // Velocity Error

   vec3_t trn_ei; // translation error integral
   vec3_t att_ei; // attitude error integral

   vec3_t Oldtherr; // stores previous iteration's therr for integration
                    // purposes
   vec3_t Oldperr; // stores previous iteration's therr for integration purposes

   vec3_t IdealTrq; // Ideal Torque
   vec3_t IdealFrc; // Ideal Force

   struct fy_node **CmdArray;
   long CmdNum;
   long CmdInit;
   long CmdCnt;
   double CmdNextTime;

   vec3_t svn;
   vec3_t svb;
   vec3_t bvn;
   vec3_t bvb;

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

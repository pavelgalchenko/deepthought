/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "AcTypes.h"
#include "DSMTypes.h"
#include "geomkit.h"
#include "iokit.h"
#include "mathkit.h"
#include "orbkit.h"
#include "rkkit.h"
#include "sigkit.h"

#ifndef __42TYPES_H__
#define __42TYPES_H__

/*
** #ifdef __cplusplus
** namespace _42 {
** #endif
*/

/* Ephem Tags */
typedef enum ephemType {
   EPH_NULL = -1, // dummy value for initialization/logging errors
   EPH_MEAN = 0,
   EPH_DE430,
   EPH_DE440,
   EPH_DE421,
   EPH_DE424,
   EPH_GMAT421,
   EPH_GMAT424,
   EPH_SPICE,
} ephemType;

/* FSW Tags */
enum fswType {
   PASSIVE_FSW = 0,
   PROTOTYPE_FSW,
   AD_HOC_FSW,
   SPINNER_FSW,
   MOMBIAS_FSW,
   THREE_AXIS_FSW,
   ISS_FSW,
   CMG_FSW,
   THR_FSW,
   CFS_FSW,
   RBT_FSW,
   DSM_FSW,
};

struct FormationType {
   /*~ Internal Variables ~*/
   char FixedInFrame;
   mat3x3_t CN;
   mat3x3_t CL;
   vec3_t PosR; /* Position of F wrt R, expressed in N */
};

/* Store information about the JPL DE file by parsing the header file */
typedef struct JPLHeaderType {
   char eph_path[80];
   char eph_str[5];
   char hdr_name[16];
   ephemType eph;
   long n_coeff;
   long blk_len;
   long blk_lines;
   JDType jd_range[2];
   double n_days;
   long n_data;
   char (*group_1040)[10];
   double *group_1041;
   int group_1050[11][3];
} JPLHeaderType;

/* "Analysis" nodes, used both for Flex, ("Force" nodes and "Measurement" nodes)
 */
/* and for more general purposes (sensor, actuator positions) */
struct NodeType {
   /*~ Internal Variables ~*/
   char comment[80];
   vec3_t NomPosB;
   vec3_t PosCm;          /* Pos wrt B's cm, expressed in B */
   double **PSI, **THETA; /* Mode shapes, 3 x B.Nf */
   vec3_t Frc, Trq;       /* Both expressed in B */
   double *FlexFrc;       /* "Fbendy + Tbendy", B.Nf x 1 */
   vec3_t FlexPos, FlexVel, FlexAng, FlexAngRate; /* Deflection variables */
   vec3_t PosB, VelB, VelN, AngVelB;
   quat_t qb;
};

struct ShakerType {
   /*~ Parameters ~*/
   long Body;
   long Node;
   long FrcTrq;
   vec3_t Axis;
   long Ntone;
   long RandomActive;
   double *ToneAmp;   /* N or Nm */
   double *ToneFreq;  /* For tonic, rad/sec */
   double *TonePhase; /* For tonic, rad */
   struct RandomProcessType *RandomProc;
   struct FilterType *Lowpass;
   struct FilterType *Highpass;
   double LowBandLimit;  /* rad/sec */
   double HighBandLimit; /* rad/sec */
   double RandStd;       /* Std Dev of band-limited random input, N or Nm */

   /*~ Internal Variables ~*/
   double Output;
   struct FilterType *Rand; /* White noise in, band-limited noise out */
};

struct BodyType {
   /*~ Internal Variables ~*/
   double mass;
   vec3_t cm;  /* wrt origin of convenience, expressed in B frame */
   vec3_t c;   /* First mass moment about ref pt, expressed in B */
   mat3x3_t I; /* Moment of Inertia, about ref pt, expressed in B frame */
   vec3_t EmbeddedMom;    /* Constant embedded momentum, for CMGs and rotating
                                instruments */
   vec3_t EmbeddedDipole; /* Constant embedded magnetic moment [[A-m^2]] */
   vec3_t wn;   /* Angular Velocity of B wrt N expressed in B frame [[rad/sec]]
                      [~=~] */
   quat_t qn;   /* [~=~] */
   vec3_t vn;   /* velocity of B ref pt expressed in N frame */
   vec3_t pn;   /* position of B ref pt in N frame expressed in N frame */
   mat3x3_t CN; /* Direction Cosine of B frame in N frame */
   vec3_t Trq;  /* expressed in B */
   vec3_t SCContactTrq;  /* expressed in B */
   vec3_t FrcN;          /* expressed in N */
   vec3_t SCContactFrcN; /* expressed in N */
   vec3_t gravPriAccN;  /* Acceleration from primary 1/r2 gravitational force */
   vec3_t gravPertAccN; /* Acceleration from other primary grav forces */
   vec3_t alpha;        /* Angular acceleration of B wrt N, expressed in B */
   vec3_t accel;        /* Linear acceleration of B wrt N, expressed in N */
   char GeomFileName[40];
   char NodeFileName[40];
   char FlexFileName[40];
   float ModelMatrix[16]; /* For OpenGL */
   long GeomTag;
   /* For KaneNBody Dynamics */
   long Gin;      /* Joint that B is Bout of */
   vec3_t beta;   /* Vector from B ref pt to B[0] ref pt, expressed in N */
   vec3_t AlphaR; /* Remainder alpha, expressed in B */
   vec3_t AccR;   /* Remainder acc, expressed in N */
   vec3_t InertiaTrq;
   vec3_t InertiaFrc;
   vec3_t JointTrq; /* From all joints exerting trq on this body */

   /* For OrderN Dynamics */
   long Nd;  /* Number of distal joints (i.e. for which this body is Bi) */
   long *Gd; /* Indices of distal joints (i.e. for which this body is Bi) */
   vec3_t RemAlf;
   vec3_t RemAcc;
   vec3_t alfn;
   vec3_t accn;
   vec3_t H;
   double RemInertiaFrc[6];
   vec3_t WhlMom;
   vec3_t FrcB;          /* Expressed in B */
   vec3_t SCContactFrcB; /* Expressed in B */
   double SpatFrc[6];    /* [Trq;Frc] + [PassiveTrq;PassiveFrc] */

   double AccU[6];

   /* For Flex Formulation */
   long Nf;            /* Number of flex modes superimposed on this body */
   double *xi;         /* Flex speed coordinate, Nf x 1 */
   double *eta;        /* Flex position coordinate, Nf x 1 */
   double **Mf;        /* Flex Mass Matrix, Nf x Nf */
   double **Kf;        /* Flex Stiffness Matrix, Nf x Nf */
   double **Cf;        /* Flex Damping Matrix, Nf x Nf */
   double **Pf;        /* Flex tensor, 3 x Nf */
   double **Hf;        /* Flex tensor, 3 x Nf */
   double *Qf;         /* Flex tensor, 3 x Nf x Nf */
   double *Rf;         /* Flex tensor, 3 x Nf x 3 */
   double *Sf;         /* Flex tensor, 3 x Nf x Nf x 3 */
   long f0;            /* Index of first element in uf */
   vec3_t Peta;        /* Pf*eta */
   mat3x3_t cplusPeta; /* SkewMatrix of (c + Pf*eta) */
   double **CnbP;      /* CNB*Pf, 3 x Nf */
   double **HplusQeta; /* Hf + Qf*eta, 3 x Nf */
   double **Qxi;       /* Qf*xi, 3 x Nf */
   double **Rw;        /* Rf*w, 3 x Nf */
   double *Sw;         /* Sf*w, 3 x Nf * Nf */
   double **Swe;       /* Sf*w*eta, 3 x Nf */
   long NumNodes;      /* Number of flex "analysis" nodes on Body */
   struct NodeType *Node;
   long MfIsDiagonal; /* Simpler EOM for One-body case if Mf is diagonal */
};

struct JointType {
   /*~ Internal Variables ~*/
   long Type; /* PASSIVE_JOINT, ACTUATED_JOINT, etc */
   long Init;
   long IsSpherical; /* TRUE or FALSE */
   long RotDOF;      /* 0,1,2,3 */
   long TrnDOF;      /* 0,1,2,3 */
   long Bin;         /* Index of inner body */
   long Bout;        /* Index of outer body */
   struct BodyType *Bi;
   struct BodyType *Bo;
   long
       Nanc; /* Number of "ancestor" joints: joints between this one and B[0] */
   long *Anc;         /* Indices of ancestor joints */
   vec3_t RigidRin;   /* Position wrt inner body ref pt (rigid) */
   vec3_t RigidRout;  /* Position wrt outer body ref pt (rigid) */
   vec3_t ri;         /* Position wrt inner body ref pt (incl flex & TrnDOF) */
   vec3_t ro;         /* Position wrt outer body ref pt (incl flex) */
   long RotSeq;       /* Joint Euler sequence */
   long RotLocked[3]; /* Set TRUE if individual DOF is to be locked in place */
   long TrnSeq;       /* Translational joint sequence */
   long TrnLocked[3];
   vec3_t Pos;        /* translational kinematic state variables [~=~] */
   vec3_t PosRate;    /* translational dynamic state variables [~=~] */
   vec3_t xb;         /* translational displacement in the Bi frame */
   vec3_t xn;         /* translational displacement in the N frame */
   vec3_t Ang;        /* Joint Euler angles [~=~] */
   vec3_t AngRate;    /* Euler angle rates about gim axes [~=~] */
   vec3_t AngRateCmd; /* Euler angle rate commands, rad/sec */
   vec3_t PosRateCmd; /* Translation rate commands, m/sec */
   vec3_t AngRateGain;
   vec3_t PosRateGain;
   vec3_t MaxAngRate;
   vec3_t MaxPosRate;
   vec3_t MaxTrq;
   vec3_t MaxFrc;
   vec3_t RotSpringCoef; /* For passive joint torques */
   vec3_t RotDampCoef;   /* For passive joint torques */
   vec3_t TrnSpringCoef; /* For passive joint forces */
   vec3_t TrnDampCoef;   /* For passive joint forces */
   /* Frames involved in a joint: Bo <-> (Bfo) <-> Go <-> Gi <-> (Bfi) <-> Bi */
   mat3x3_t CGiBi;  /* Constant orientation of joint in Bi (Bfi) */
   mat3x3_t CBoGo;  /* Constant orientation of joint in Bo (Bfo) */
   mat3x3_t CGoGi;  /* Euler rot through ang[0], ang[1], ang[2] */
   mat3x3_t CTrqBo; /* Used for transforming joint torques or forces from Go
                           to Bo */
   mat3x3_t CTrqBi; /* Used for transforming joint torques from Go to Bi */
   mat3x3_t COI;    /* DCM from inner body to outer body, (incl flex) */
   vec3_t Trq;      /* Exerted on Bout, components along gimbal axes */
   vec3_t Frc; /* Force exerted on Bout, components along translational axes */
   mat3x3_t Gamma;    /* w = Gamma*sigma */
   mat3x3_t Delta;    /* v = Delta*s -- matrix of joint partials for
                             translational joints */
   vec3_t Gs;         /* Gamma*sigma */
   vec3_t Gds;        /* Gammadot*sigma */
   vec3_t Ds;         /* Delta*s */
   vec3_t Dds;        /* Deltadot*s */
   long Rotu0;        /* Index of first Rot element in u */
   long Rotx0;        /* Index of first Rot element in x */
   long Trnu0;        /* Index of first Trn element in u */
   long Trnx0;        /* Index of first Trn element in x */
   long ActiveRotu0;  /* Index in DynStateIdx of first unlocked Rot DOF in
                         DynState */
   long ActiveTrnu0;  /* Index in DynStateIdx of first unlocked Trn DOF in
                         DynState */
   long ActiveRotDOF; /* Number of unlocked RotDOF */
   long ActiveTrnDOF; /* Number of unlocked TrnDOF */

   /* For OrderN Dynamics */
   mat3x3_t Pw;
   mat3x3_t Pv;
   mat3x3_t Pwdot;
   double P[6][6];         /* [Pw 0; 0 Pv] */
   double ArtFrc[6];       /* Articulated Body Force */
   double ArtMass[6][6];   /* Articulated Body Mass */
   double DynMtx[6][6];    /* Nu x Nu */
   double InvDynMtx[6][6]; /* Nu x Nu */
   double InvDynPT[6][6];  /* Nu x 6 */
   double AbsorpMtx[6][6];
   double TransMtx[6][6];
   vec3_t riplusPx; /* r_{ik} + P_{vk}x_k */
   long Nu;         /* RotDOF + TrnDOF */
   quat_t q;        /* Quaternion or angle states, depending on IsSpherical */
   double udot[6];
   quat_t qdot;
   vec3_t xdot;
   double RKum[6];
   quat_t RKqm;
   vec3_t RKxm;
   double RKdu[6];
   quat_t RKdq;
   vec3_t RKdx;

   /* For Flex */
   double **PSIi;      /* Translation Mode Shapes, 3 x Bi.Nf */
   double **THETAi;    /* Rotational Mode Shapes, 3 x Bi.Nf */
   double **PSIo;      /* Translation Mode Shapes, 3 x Bo.Nf */
   double **THETAo;    /* Rotational Mode Shapes, 3 x Bo.Nf */
   vec3_t FlexPosi;    /* Translational Flex Deflection of Bi at G (d) */
   vec3_t FlexVeli;    /* Translational Flex Velocity of Bi at G   (e) */
   vec3_t FlexAngi;    /* Rotational Flex Deflection of Bi at G    (delta) */
   vec3_t FlexAngVeli; /* Rotational Flex Velocity of Bi at G      (eta) */
   vec3_t FlexPoso;    /* Translational Flex Deflection of Bo at G (d) */
   vec3_t FlexVelo;    /* Translational Flex Velocity of Bo at G   (e) */
   vec3_t FlexAngo;    /* Rotational Flex Deflection of Bo at G    (delta) */
   vec3_t FlexAngVelo; /* Rotational Flex Velocity of Bo at G      (eta) */
   /* For Constraints */
   long Rotc0;
   long Trnc0;

   char ParmFileName[40];
};

struct IdealActType {
   /*~ Internal Variables ~*/
   double Tcmd;
   double Fcmd;
   struct DelayType *FrcDelay;
   struct DelayType *TrqDelay;
};

struct WhlHarmType {
   double n;     /* Harmonic Number [none] */
   double Ks;    /* Static imbalance coefficient, [kg-m] */
   double Kd;    /* Dynamic imbalance coefficient, [kg-m^2] */
   double phase; /* Phase angle of harmonic wrt dynamic imbalance, [rad] */
};

struct WhlType {
   /*~ Internal Variables ~*/
   long Body;    /* Body that wheel is mounted in */
   double H;     /* Angular Momentum, [[Nms]] [~=~] */
   double J;     /* Rotary inertia, kg-m^2 */
   double w;     /* Angular speed, rad/sec */
   double Ang;   /* Spin phase angle, rad */
   vec3_t A;     /* Axis vector wrt Body */
   vec3_t Uaxis; /* Transverse axes */
   vec3_t Vaxis; /* Transverse axes */
   double Tmax;
   double Hmax;
   double Tcmd;
   double Trq; /* Exerted on wheel, expressed along wheel axis */
   long Node;
   struct DelayType *Delay; /* For injecting delay into control loops */

   char DragJitterFileName[40];

   /* For Drag */
   double CoulCoef;        /* Coulomb friction, Nm */
   double StribeckCoef;    /* Stiction - Coulomb, Nm */
   double ViscCoef;        /* Viscous friction coefficient, Nm/(rad/sec) */
   double StribeckZone;    /* Stribeck zone, rad/sec */
   double LugreSpringCoef; /* Lugre stiffness, Nm/rad */
   double LugreDampCoef;   /* Lugre damping, Nm/(rad/sec) */
   double LugreDampZone;   /* Lugre damping zone, rad/sec */
   double z;               /* Lugre internal state, rad */
   double FricTrq;         /* Friction Torque, Nm */

   /* For Jitter */
   double gamma;    /* 2*Jt/Jr (<1.0) */
   double Jt;       /* Transverse rotor inertia, [kg-m^2] */
   double ImbPhase; /* Phase of static imbalance wrt dynamic imbalance [rad] */
   double LatFreq;  /* [rad/sec] */
   double LatDamp;
   double RockFreq; /* [rad/sec] */
   double RockDamp;
   long NumHarm;
   struct WhlHarmType *Harm;
   vec3_t JitFrc;
   vec3_t JitTrq;

   /* For OrderN Dynamics */
   double Hdot;
   double RKHm;
   double RKdH;
};

struct MTBType {
   /*~ Internal Variables ~*/
   double M;
   vec3_t A; /* Axis vector wrt Body 0 */
   double Mmax;
   double Mcmd;
   vec3_t Trq; /* Exerted on Body 0, expressed in B[0] frame */
   long Node;
   struct DelayType *Delay; /* For injecting delay into control loops */
};

struct ThrType {
   /*~ Internal Variables ~*/
   long Mode; /* THR_PULSED or THR_PROPORTIONAL */
   double Fmax;
   double F;
   long Body; /* Body that thruster is mounted on */
   long Node;
   vec3_t A; /* Axis vector wrt Body 0 */
   JDType PulseWidthFinTimeStamp;
   double PulseWidthCmd;    /* [[sec]], for THR_PULSED */
   double ThrustLevelCmd;   /* [{0.0:1.0}], for THR_PROPORTIONAL */
   vec3_t Frc;              /* Force exerted */
   vec3_t Trq;              /* Torque exerted */
   struct DelayType *Delay; /* For injecting delay into control loops */
};

struct GyroType {
   /*~ Parameters ~*/
   double SampleTime;
   long MaxCounter;
   vec3_t Axis;
   double MaxRate;
   double Scale;
   double Quant;
   double SigV; /* ARW, rad/rt-sec */
   double SigU; /* Bias Stability, rad/sec^1.5 */
   double SigE; /* Angle Readout Noise, rad */
   long Node;

   double BiasStabCoef;
   double ARWCoef;
   double AngNoiseCoef;
   double CorrCoef; /* Correlation Coef, exp(-SampleTime/BiasTime) */

   /*~ Internal Variables ~*/
   long SampleCounter;
   double TrueRate; /* rad/sec [~>~] */
   double Bias;     /* rad/sec */
   double Angle;    /* rad */
   double MeasRate; /* rad/sec */
};

struct MagnetometerType {
   /*~ Parameters ~*/
   double SampleTime;
   long MaxCounter;
   vec3_t Axis;
   double Saturation;
   double Scale;
   double Quant;
   double Noise;
   long Node;

   /*~ Internal Variables ~*/
   long SampleCounter;
   double Field; /* Magfield Component, Tesla */
};

struct CssType {
   /*~ Parameters ~*/
   double SampleTime;
   long MaxCounter;
   long Body;
   vec3_t Axis;
   double FovHalfAng;
   double CosFov;
   double Scale;
   double Quant;
   long Node;

   /*~ Internal Variables ~*/
   long SampleCounter;
   long Valid;
   double Illum;  /* Units defined by scale */
   double Albedo; /* [0.0:1.0] */
};

struct FssType {
   /*~ Parameters ~*/
   double SampleTime;
   long MaxCounter;
   quat_t qb;
   mat3x3_t CB;
   double FovHalfAng[2];
   double NEA;
   double Quant;
   long Node;
   long BoreAxis; /* X_AXIS, Y_AXIS, Z_AXIS */
   long H_Axis;   /* (BoreAxis+1)%3 */
   long V_Axis;   /* (BoreAxis+2)%3 */

   /*~ Internal Variables ~*/
   long SampleCounter;
   long Valid;
   enum fssTypes type;
   double SunAng[2];
   vec3_t SunVecS;
   vec3_t SunVecB;
   double AlbA;
   double AlbB;
   double AlbC;
   double AlbD;
};

struct StarTrackerType {
   /*~ Parameters ~*/
   double SampleTime;
   long MaxCounter;
   quat_t qb;
   mat3x3_t CB;
   double FovHalfAng[2];
   double CosFov[2];
   double SunExclAng;
   double CosSunExclAng;
   double EarthExclAng;
   double CosEarthExclAng;
   double MoonExclAng;
   double CosMoonExclAng;
   double NEA[3];
   long Node;
   long BoreAxis; /* X_AXIS, Y_AXIS, Z_AXIS */
   long H_Axis;   /* (BoreAxis+1)%3 */
   long V_Axis;   /* (BoreAxis+2)%3 */

   /*~ Internal Variables ~*/
   long SampleCounter;
   long Valid;
   quat_t qn;
};

struct GpsType {
   /*~ Parameters ~*/
   double SampleTime;
   long MaxCounter;
   double PosNoise;
   double VelNoise;
   double TimeNoise;
   long Node;

   /*~ Internal Variables ~*/
   long SampleCounter;
   long Valid;
   long Rollover;
   long Week;
   double Sec;
   vec3_t PosN;
   vec3_t VelN;
   vec3_t PosW;
   vec3_t VelW;
   double Lng, Lat, Alt;          /* Geocentric */
   double WgsLng, WgsLat, WgsAlt; /* Geodetic, WGS-84 */
};

struct AccelType {
   /*~ Parameters ~*/
   double SampleTime;
   long SampleCounter;
   long MaxCounter;
   long Node;
   vec3_t Axis; /* Mounting matrix */
   double Quant;
   double Scale;
   double SigV; /* DVRW m/s/rt-sec */
   double SigU; /* Bias Stability m/s^1.5 */
   double SigE; /* DV Readout Noise, m/s  */

   /*~ Internal Variables ~*/
   vec3_t AccumAccN;
   double Bias;     /* m/s^2 */
   vec3_t PrevVelN; /* m/s */
   quat_t PrevQN;
   double DV;      /* Change in velocity m/s */
   double TrueAcc; /* m/s^2 */
   double MeasAcc; /* m/s^2 */
   double MaxAcc;  /* m/s^2 max acceleration */
   double AccError;
   long Counts;

   /* Coef */
   double BiasStabCoef;
   double DVRWCoef;
   double DVNoiseCoef;
   double CorrCoef; /* Correlation Coef, exp(-SampleTime/BiasTime) */
};

struct OpticsType {
   long SC;
   long Body;
   long Node;
   long Type;
   vec3_t Axis;
   double FocLen;
   double ConicConst;
   double ConicSign;
   double ApRad;
};

struct GuideWindowType {
   /*~ Parameters ~*/
   long Nrow;
   long Ncol;

   /*~ Internal Variables ~*/
   long Row0;
   long Col0;
   double *Image; /* Nrow x Ncol, grayscale */
};

struct PsfType {
   long Nrow;
   long Ncol;
   long BytesPerPixel;
   double Scl; /* rad/pixel */
   double *Image;
};

struct FgsType {
   /*~ Parameters ~*/
   long HasOptics;
   double SampleTime;
   long MaxCounter;
   quat_t qb;
   mat3x3_t CB;
   quat_t qr;   /* q_fr_r */
   mat3x3_t CR; /* CFrR */
   double NEA;
   long Body;
   long Node;
   long BoreAxis; /* X_AXIS, Y_AXIS, Z_AXIS */
   long H_Axis;   /* (BoreAxis+1)%3 */
   long V_Axis;   /* (BoreAxis+2)%3 */
   double FovHalfAng[2];
   double Scl; /* rad/pixel */
   double Hr;  /* Guide Star in Fr */
   double Vr;  /* Guide Star in Fr */

   /*~ Internal Variables ~*/
   long SampleCounter;
   long Valid;
   vec3_t StarVecR;
   double H;
   double V;
   vec3_t Ang;
   char OpticsFileName[40];
   char PsfFileName[40];

   long Nopt;
   struct OpticsType *Opt;
   long ApFocus;
   long DetFocus;

   struct PsfType PSF;
   struct GuideWindowType Gw;
};

struct JointPathTableType { /* tells if joint is in path of body*/
   /*~ Internal Variables ~*/
   long InPath;
   vec3_t rho;
};

struct BodyPathTableType { /* tells if inner body is in path of outer body*/
   /*~ Internal Variables ~*/
   long InPath;
   mat3x3_t Coi;
};

struct DynType {
   /*~ Internal Variables ~*/
   long Nu;             /* 6 + Sum(Joint DOFs) */
   long Nx;             /* 7 + Sum(Joint DOFs), (spherical gives 4) */
   long **ConnectTable; /* Nb x Ng */
   struct JointPathTableType **JointPathTable; /* Nb x Ng */
   struct BodyPathTableType **BodyPathTable;   /* Nb x Nb */
   double **PAngVel;                           /* 3*Nb x Nu */
   double **IPAngVel;                          /* 3*Nb x Nu */
   double **PVel;                              /* 3*Nb x Nu */
   double **mPVel;                             /* 3*Nb x Nu */
   double *BodyTrq;                            /* 3*Nb x 1 */
   double *BodyFrc;                            /* 3*Nb x 1 */
   double **COEF;                              /* (Nu+Nf) x (Nu+Nf) */
   double *RHS;                                /* (Nu+Nf) x 1 */
   long SomeJointsLocked;                      /* 1 if any DOFs are locked */
   long Ns;             /* Number of active states (joint + flex), <= (Nu+Nf) */
   double *ActiveState; /* u and uf concatenated, (Nu+Nf) x 1 */
   long *ActiveStateIdx;       /* Keeps track of active states, (Nu+Nf) x 1 */
   double *u, *uu, *du, *udot; /* Nu  (Dynamic States) */
   double *x, *xx, *dx, *xdot; /* Nx  (Kinematic States) */
   double *h, *hh, *dh, *hdot; /* Nw  (Wheel Momentum States) */
   double *a, *aa, *da, *adot; /* Nw  (Wheel spin angle states) */
   /* For Flex */
   long Nf;                        /* Total Number of Flex Modes, Sum(B.Nf) */
   double **PAngVelf;              /* 3*Nb x Nf */
   double **IPAngVelf;             /* 3*Nb x Nf */
   double **PVelf;                 /* 3*Nb x Nf */
   double **mPVelf;                /* 3*Nb x Nf */
   double **Mf;                    /* Nf x Nf */
   double **PCPVelf;               /* Nf x Nf */
   double **HplusQetaPAngVelf;     /* Nf x Nf */
   double *uf, *uuf, *duf, *ufdot; /* Nf (Dynamic Flex States) */
   double *xf, *xxf, *dxf, *xfdot; /* Nf (Kinematic Flex States) */
   double *FlexAcc;                /* Nf x 1 */
   double *FlexFrc;                /* Nf x 1 */
   double *FlexInertiaFrc;         /* Nf x 1 */
   /* For Constraints */
   long Nc;                  /* 6*Nb - Nu */
   double **PAngVelc;        /* 3*Nb x Nc */
   double **PVelc;           /* 3*Nb x Nc */
   double *TotalTrq;         /* 3*Nb x 1 */
   double *TotalFrc;         /* 3*Nb x 1 */
   double *GenConstraintFrc; /* Nc x 1 */
};

struct EnvTrqType {
   /*~ Internal Variables ~*/
   long First;
   FILE *envfile;
   vec3_t Hs;
};

struct TargetType {
   /*~ Internal Variables ~*/
   long Type;
   WorldID World;
   long RefOrb;
   long SC;
   long Body;
   vec3_t PosR;
   vec3_t PosN;
   vec3_t PosH;
   mat3x3_t CN;
};

struct POVType {
   /*~ Internal Variables ~*/
   long Mode; /* Track Host, Track Target, or Fixed in Host */
   struct TargetType Host;
   struct TargetType Target;
   long View;
   long Frame;    /* Which frame is POV frame oriented in? (N=0,L=1,F=2,B=3) */
   long BoreAxis; /* POV boresight axis (out of screen): POS_X, POS_Y, ...,
                     NEG_Z */
   long UpAxis;   /* POV axis pointing to top of window: POS_X, ... NEG_Z */
   double Width;  /* Width of POV Field of View */
   double Height; /* Height of POV Field of View */
   double Near, Far; /* Near and Far limits of POV FOV */
   double CosFov, SinFov;
   double Angle;       /* Angle subtended in vertical, deg */
   double AR;          /* Aspect ratio of POV FOV */
   vec3_t PosLeftEye;  /* in POV frame, expressed in POV */
   vec3_t PosRightEye; /* in POV frame, expressed in POV */
   vec3_t w;           /* Angular velocity */
   quat_t q;           /* Quaternion */
   vec3_t PosB;        /* Position wrt Host, expressed in Host B[0] Frame */
   double Range;
   double GridSpacing; /* For ProxOps Grid */
   vec3_t wmax;
   mat3x3_t C;
   mat3x3_t CN;
   mat3x3_t CH;
   mat3x3_t CL;
   mat3x3_t CF;
   mat3x3_t CB;
   vec3_t PosR; /* Position vector in R, expressed in N */
   vec3_t PosN; /* Position vector in N, expressed in N */
   vec3_t PosH; /* Position vector in H, expressed in H */
   float ViewMatrix[16];
   /* For PanZoomPOV */
   double TimeToGo;
   long CmdSeq;
   vec3_t CmdAngle;
   double CmdRange;
   mat3x3_t CmdPermute;
};

struct RegionType {
   /*~ Internal Variables ~*/
   long Exists;
   WorldID World;
   double Lng, Lat, Alt; /* Origin location */
   vec3_t PosW;
   mat3x3_t CW; /* Region frame is East-North-Up */
   vec3_t PosN;
   vec3_t VelN;
   mat3x3_t CN;
   vec3_t wn; /* Expressed in R frame */
   double ElastCoef, DampCoef, FricCoef;
   char Name[20];
   char GeomFileName[40];
   long GeomTag;
   float ModelMatrix[16]; /* For OpenGL */
};

struct SCType;
typedef struct SCRKParams {
   RKParams base;
   struct WorldType *worlds;          // pointer to the global World
   struct RegionType *rgn;            // pointer to the global Rgn
   struct LagrangeSystemType *lagsys; // pointer to all lagsystems
   struct OrbitType *orb;             // pointer to sc's orbit
   struct FormationType *frm;         // pointer to sc's formation
   struct SCType *sc;                 // pointer to sc itself
   ephemType ephem;
} SCRKParams;

struct SCType {
   /*~ Internal Variables ~*/
   long ID; /* SC[x].ID = x */
   long Exists;
   char Label[40];
   long DynMethod; /* GAUSS_ELIM, ORDER_N */
   long OrbDOF;    /* FIXED, EULER_HILL, ENCKE, COWELL */
   long RefOrb;
   enum fswType FswTag; /* Tag for FSW function, eg. PROTOTYPE_FSW */
   double FswSampleTime;
   long FswMaxCounter;
   long FswSampleCounter;
   long InitAC;
   long InitDSM;

   double aeroProjectedArea;
   double srpProjectedArea;
   vec3_t gravTrqB;
   vec3_t gravTrqN;
   vec3_t srpTrqB;
   vec3_t srpTrqN;
   vec3_t aeroTrqB;
   vec3_t aeroTrqN;
   vec3_t srpFrcB;
   vec3_t srpFrcN;
   vec3_t aeroFrcB;
   vec3_t aeroFrcN;

   long Nb; /* Number of bodies */
   long Ng; /* Number of joints, = Nb-1 */

   long Nw;   /* Number of wheels */
   long Nmtb; /* Number of MTB's */
   long Nthr; /* Number of thrusters */

   long Ngyro; /* Number of Gyro axes */
   long Nmag;  /* Number of magnetometer axes */
   long Ncss;  /* Number of coarse sun sensors */
   long Nfss;  /* Number of Fine Sun Sensors */
   long Nst;   /* Number of star trackers */
   long Ngps;  /* Number of GPS receivers */
   long Nacc;  /* Number of accelerometer axes */
   long Nfgs;  /* Number of Fine Guidance Sensors */
   long Nsh;   /* Number of shakers */

   double mass;
   vec3_t cm;    /* wrt B0 origin, expressed in B0 frame */
   mat3x3_t I;   /* Inertia matrix, wrt SC.cm, expressed in B0 frame */
   vec3_t PosR;  /* Position of cm wrt Reference Orbit [[m]], expressed in N
                       [~=~] */
   vec3_t VelR;  /* Velocity of cm wrt R [[m/s]], expressed in N [~=~] */
   vec3_t PosEH; /* Position of cm wrt R, m, in Euler-Hill coords */
   vec3_t VelEH; /* Velocity of cm wrt R, m, in Euler-Hill coords */
   vec3_t PosN;  /* Position of cm wrt origin of N, m, expressed in N */
   vec3_t VelN;  /* Velocity of cm wrt origin of N, m/sec, expressed in N */
   mat3x3_t CLN; /* Note that SC.CLN != Orb[RefOrb].CLN if SC.PosR != 0.0 */
   mat3x3_t CEN; /* E = Equatorial frame: e1 = North, e2 = East, e3 = Nadir */
   vec3_t wln;   /* Expressed in N */
   vec3_t PosH;  /* Position of cm wrt H frame, expressed in H */
   vec3_t VelH;  /* Velocity of cm wrt H frame, expressed in H */
   vec3_t FrcN;  /* Force, N, expressed in N */
   vec3_t AccN;  /* Acceleration due to external force, for accelerometer
                       model */
   vec3_t gravPriAccN;  /* Acceleration from primary 1/r2 gravitational force */
   vec3_t gravPertAccN; /* Acceleration from other primary grav forces */
   vec3_t svn;          /* Sun-pointing unit vector, expressed in N */
   vec3_t svb; /* Sun-pointing unit vector, expressed in SC.B[0] [~=~] */
   vec3_t bvn; /* Magfield, Tesla, expressed in N */
   vec3_t bvb; /* Magfield [[Tesla]], expressed in SC.B[0] [~=~] */
   vec3_t Hvn; /* Total SC angular momentum, Nms, expressed in N */
   vec3_t Hvb; /* Total SC angular momentum [[Nms]], expressed in SC.B[0]
                     [~=~] */
   long Eclipse;
   double AtmoDensity;
   double DragCoef;
   char FileName[50];
   char SpriteFileName[40];
   unsigned int SpriteTexTag;
   /* The following are for OSCAR */
   vec3_t PosF; /* Position of B0 origin wrt F, expressed in F */
   vec3_t VelF; /* Velocity of B0 origin wrt F, expressed in F */
   mat3x3_t CF; /* Attitude of B0 wrt F */
   /* Constraint forces and torques are computed if requested */
   long ConstraintsRequested;
   /* Mass and flex properties referred to REFPT_CM or REFPT_JOINT */
   long RefPt;
   /* Flexible Dynamics Active */
   long FlexActive;
   /* Include higher-order coupling terms in rigid-flex dynamics */
   long IncludeSecondOrderFlexTerms;
   char ShakerFileName[40];
   long WhlDragActive;
   long WhlJitterActive;
   /* Workspace for KaneNBody */
   struct DynType Dyn;
   /* Workspace for Actuator Sizing */
   struct EnvTrqType EnvTrq;
   /* Bounding Box used for shadowmap */
   struct BoundingBoxType BBox;
   /* See ReadStatesFromSocket */
   long RequestStateRefresh;

   /* For stability analysis */
   long GainAndDelayActive;
   double LoopGain;
   double LoopDelay;

   /*~ Structures ~*/
   RungeKutta RKIntegrator;
   SCRKParams rkparams;
   double *rk_state;
   struct AcType AC;
   struct DSMType DSM;
   struct BodyType *B;  /* [*Nb*] */
   struct JointType *G; /* [*Ng*] */
   struct JointType GN; /* Joint between N and B[0] */
   struct IdealActType IdealAct[3];
   struct WhlType *Whl;          /* [*Nw*] */
   struct MTBType *MTB;          /* [*Nmtb*] */
   struct ThrType *Thr;          /* [*Nthr*] */
   struct GyroType *Gyro;        /* [*Ngyro*] */
   struct MagnetometerType *MAG; /* [*Nmag*] */
   struct CssType *CSS;          /* [*Ncss*] */
   struct FssType *FSS;          /* [*Nfss*] */
   struct StarTrackerType *ST;   /* [*Nst*] */
   struct GpsType *GPS;          /* [*Ngps*] */
   struct AccelType *Accel;      /* [*Nacc*] */
   struct FgsType *Fgs;          /* [*Nfgs*] */
   struct ShakerType *Shaker;    /* [*Nsh*] */
};

struct SpotType {
   /*~ Internal Variables ~*/
   long xmin, ymin;
   long xmax, ymax;
   long Visible;
   long Selected;
};

struct WidgetType {
   /*~ Internal Variables ~*/
   long xmin, ymin;
   long xmax, ymax;
   float BorderColor[4];
   float TextColor[4];
   float SelectedColor[4];
   float UnselectedColor[4];
   long Nspot;
   struct SpotType *Spot;
};

struct FovType {
   /*~ Internal Variables ~*/
   long Type; /* WIREFRAME, SOLID, or VECTOR */
   long NearExists;
   long FarExists;
   long RefOrb;
   long SC;
   long Body;
   char Label[40];
   long Nv;
   long BoreAxis; /* X_AXIS, Y_AXIS, Z_AXIS */
   long H_Axis;   /* (BoreAxis+1)%3 */
   long V_Axis;   /* (BoreAxis+2)%3 */
   double Width;  /* X angular dimension, rad */
   double Height; /* Y angular dimension, rad */
   double Length;
   vec3_t pb;
   mat3x3_t CB;
   float Color[4];
};

struct TdrsType {
   /*~ Internal Variables ~*/
   long Exists;
   double JD;
   vec3_t rw;   /* Position vector in ECEF frame */
   vec3_t PosN; /* Position vector in N frame */
   vec3_t VelN; /* Velocity vector in N frame (never used?) */
   double lat;
   double lng;
   char Designation[40];
};

struct GroundStationType {
   /*~ Internal Variables ~*/
   long Exists;
   WorldID World;
   long Show;
   double lng, lat;
   vec3_t PosW; /* Position vector in World frame */
   char Label[40];
};

/* Framebuffer Objects for Spacecraft Shadows */
struct ShadowFBOType {
   /*~ Internal Variables ~*/
   unsigned int FrameTag;
   unsigned int Height, Width;
   unsigned int RenderTag;
   unsigned int TexTag;
   float *Tex;
};

struct AlbedoFBOType {
   /*~ Internal Variables ~*/
   unsigned int FrameTag;
   unsigned int Height, Width;
   unsigned int RenderTag;
   unsigned int TexTag;
   float *Tex;
};

/* Orrery POV is different from POV */
struct OrreryPOVType {
   /*~ Internal Variables ~*/
   long Regime; /* CENTRAL or THREE_BODY */
   long CenterType;
   WorldID World;
   long LagSys;
   long MinorBody;
   long LP;
   vec3_t PosN; /* Position wrt World, expressed in World N */
   double Radius;
   long Zoom;
   double Scale[30];
   char ScaleLabel[30][8];
   double Angle;
   mat3x3_t CNH;
   mat3x3_t CN;
   mat3x3_t CH;
   mat3x3_t CL;
};

struct ConstellationType {
   char Tag[4];
   long Class; /* MAJOR, ZODIAC, or MINOR */
   long Nstars;
   long Nlines;
   vec3_t *StarVec;
   /* For each line */
   long *Star1;
   long *Star2;
};

struct IpcType {
   long Init;
   long Mode;       /* OFF, TX, RX, TXRX, ACS, WRITEFILE, READFILE */
   long SocketRole; /* SERVER, CLIENT, GMSEC_CLIENT */
   long AcsID;      /* AC.ID for ACS mode */
   char HostName[40];
   long Port;
   long AllowBlocking;
   long EchoEnabled;
   SOCKET Socket;
   FILE *File;
   long Nprefix;
   char **Prefix;
};

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __42TYPES_H__ */

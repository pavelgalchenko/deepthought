/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __ACTYPES_H__
#define __ACTYPES_H__

#include "jdkit.h"
#include "mathkit.h"

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

enum fssTypes {
   CONVENTIONAL_FSS = 0,
   GS_FSS,
};

struct CmdVecType {
   /*~ Internal Variables ~*/
   long Mode;
   long Frame;
   long TrgType;
   long TrgWorld;
   long TrgSC;
   long TrgBody;
   vec3_t N;  /* Components in N */
   vec3_t W;  /* Components in W */
   vec3_t L;  /* Components in L */
   vec3_t R;  /* Components in R */
   vec3_t T;  /* Components in T */
   vec3_t wn; /* Angular velocity in N, expressed in N */
};

struct CmdType {
   /*~ Internal Variables ~*/
   long Parm;
   long Frame;
   vec3_t AngRate; /* [~<~] */
   vec3_t Ang;     /* [~<~] */
   vec3_t PosRate;
   vec3_t Pos;
   long RotSeq;
   quat_t qrl; /* [~!~] */
   quat_t qrn; /* [~!~] */
   vec3_t wrn;
   double SpinRate;
   vec3_t Hvr;
   vec3_t Hvn;
   mat3x3_t OldCRN;

   /*~ Structures ~*/
   struct CmdVecType PriVec;
   struct CmdVecType SecVec;
};

struct AcBodyType {
   /*~ Parameters ~*/
   double mass;  /* [[kg]] */
   vec3_t cm;    /* [[m]] */
   mat3x3_t MOI; /* [[kg-m^2]] */
};

struct AcJointType {
   /*~ Parameters ~*/
   long IsSpherical;
   long RotDOF;
   long TrnDOF;
   long RotSeq;
   long TrnSeq;
   mat3x3_t CGiBi;
   mat3x3_t CBoGo;
   vec3_t AngGain;
   vec3_t AngRateGain;
   vec3_t PosGain;
   vec3_t PosRateGain;
   vec3_t MaxAngRate;
   vec3_t MaxPosRate;
   vec3_t MaxTrq;
   vec3_t MaxFrc;

   /*~ Internal Variables ~*/
   vec3_t Ang; /* [[rad]] [~>~] */
   vec3_t AngRate;
   vec3_t Pos;
   vec3_t PosRate;
   mat3x3_t COI;

   /*~ Structures ~*/
   struct CmdType Cmd;
};

struct AcGyroType {
   /*~ Parameters ~*/
   vec3_t Axis;

   /*~ Internal Variables ~*/
   long Valid;
   double Rate; /* [[rad/sec]] [~>~] */
};

struct AcMagnetometerType {
   /*~ Parameters ~*/
   vec3_t Axis;

   /*~ Internal Variables ~*/
   long Valid;
   double Field; /* [[Tesla]] [~>~] */
};

struct AcCssType {
   /*~ Parameters ~*/
   long Body;
   vec3_t Axis;
   double Scale;

   /*~ Internal Variables ~*/
   long Valid;   /* [~>~] */
   double Illum; /* [~>~] */
};

struct AcFssType {
   /*~ Parameters ~*/
   quat_t qb;
   mat3x3_t CB;
   long H_Axis;
   long V_Axis;
   long BoreAxis;

   /*~ Internal Variables ~*/
   enum fssTypes type;
   long Valid;       /* [~>~] */
   double SunAng[2]; /* [[rad]] [~>~] */
   vec3_t SunVecS;
   vec3_t SunVecB;
};

struct AcStarTrackerType {
   /*~ Parameters ~*/
   quat_t qb;
   mat3x3_t CB;

   /*~ Internal Variables ~*/
   long Valid; /* [~>~] */
   quat_t qn;  /* [~>~] */
   quat_t qbn;
   long BoreAxis; /* X_AXIS, Y_AXIS, Z_AXIS */
};

struct AcGpsType {
   /*~ Internal Variables ~*/
   long Valid;    /* [~>~] */
   long Rollover; /* [~>~] */
   long Week;     /* [~>~] */
   double Sec;    /* [~>~] */
   vec3_t PosN;   /* [[m]] [~>~] */
   vec3_t VelN;   /* [[m/s]] [~>~] */
   vec3_t PosW;   /* [[m]] [~>~] */
   vec3_t VelW;   /* [[m/s]] [~>~] */
   double Lng;    /* Geocentric [[rad]] [~>~] */
   double Lat;    /* Geocentric [[rad]] [~>~] */
   double Alt;    /* Geocentric [[m]] [~>~] */
   double WgsLng; /* Geodetic, WGS-84 [[rad]] [~>~] */
   double WgsLat; /* Geodetic, WGS-84 [[rad]] [~>~] */
   double WgsAlt; /* Geodetic, WGS-84 [[m]] [~>~] */
};

struct AcAccelType {
   /*~ Parameters ~*/
   vec3_t PosB;
   vec3_t Axis;

   /*~ Internal Variables ~*/
   long Valid;
   double Acc; /* [[m/s^2]] [~>~] */
};

struct AcEarthSensorType {
   /*~ Internal Variables ~*/
   long Valid;
   double Roll;
   double Pitch;
};

struct AcWhlType {
   /*~ Parameters ~*/
   long Body;
   vec3_t Axis;
   vec3_t DistVec;
   double J;
   double Tmax;
   double Hmax;

   /*~ Internal Variables ~*/
   double w;
   double H;    /* [[Nms]] [~>~] */
   double Tcmd; /* [[N-m]] [~<~] */
};

struct AcMtbType {
   /*~ Parameters ~*/
   vec3_t Axis;
   vec3_t DistVec;
   double Mmax;

   /*~ Internal Variables ~*/
   double Mcmd; /* [[A-m^2]] [~<~] */
};

struct AcThrType {
   /*~ Parameters ~*/
   long Body;
   vec3_t PosB;
   vec3_t Axis;
   vec3_t rxA;
   double DistVec[6];
   double Fmax;

   /*~ Internal Variables ~*/
   double Fcmd;
   JDType PulseWidthFinTimeStamp;
   double PulseWidthCmd;  /* for PULSED [[sec]] [~<~] */
   double ThrustLevelCmd; /* for PROPORTIONAL [[None]] [~<~] */
};

struct AcPrototypeCtrlType {
   /*~ Parameters ~*/
   double wc;
   double amax;
   double vmax;
   double Kprec;
   double Knute;

   /*~ Internal Variables ~*/
   long Init;
   vec3_t Tcmd;
   quat_t qbr;
   vec3_t therr;
   vec3_t werr;
};

struct AcAdHocCtrlType {
   /*~ Parameters ~*/
   vec3_t Kr;
   vec3_t Kp;

   /*~ Internal Variables ~*/
   long Init;
   vec3_t therr;
   vec3_t werr;
   vec3_t Tcmd;
};

struct AcSpinnerCtrlType {
   /*~ Parameters ~*/
   double Ispin;
   double Itrans;
   double SpinRate;
   double Knute;
   double Kprec;

   /*~ Internal Variables ~*/
   long Init;
   double Bold1, Bold2;
   double xold, yold;
   vec3_t rvn, rvb;
   vec3_t Tcmd;
   vec3_t Mcmd;
};

struct AcMomBiasCtrlType {
   /*~ Internal Variables ~*/
   long Init;
};

struct AcThreeAxisCtrlType {
   /*~ Parameters ~*/
   vec3_t Kr;
   vec3_t Kp;
   double Kunl;

   /*~ Internal Variables ~*/
   long Init;
   vec3_t Tcmd;
   vec3_t Hwcmd;
};

struct AcIssCtrlType {
   /*~ Parameters ~*/
   vec3_t Kr;
   vec3_t Kp;
   double Tmax;

   /*~ Internal Variables ~*/
   long Init;
   vec3_t therr;
   vec3_t werr;
};

struct AcCmgCtrlType {
   /*~ Parameters ~*/
   vec3_t Kr;
   vec3_t Kp;

   /*~ Internal Variables ~*/
   long Init;
   vec3_t therr, werr;
   vec3_t Tcmd;
   quat_t AngRateCmd;
};

struct AcThrCtrlType {
   /*~ Parameters ~*/
   vec3_t Kw;
   vec3_t Kth;
   double Kv;
   double Kp;

   /*~ Internal Variables ~*/
   long Init;
};

struct AcCfsCtrlType {
   /*~ Parameters ~*/
   vec3_t Kr;
   vec3_t Kp;
   double Kunl;

   /*~ Internal Variables ~*/
   long Init;
   vec3_t therr;
   vec3_t werr;
};

struct AcThrSteerCtrlType {
   /*~ Parameters ~*/
   vec3_t Kr;
   vec3_t Kp;

   /*~ Internal Variables ~*/
   long Init;
   vec3_t therr;
   vec3_t werr;
   vec3_t ierr;
   vec3_t Tcmd;
};

struct AcType {
   /*~ Parameters ~*/
   long ID;              /* Spacecraft ID */
   long EchoEnabled;     /* For IPC */
   long ParmLoadEnabled; /* [~>~] */
   long ParmDumpEnabled; /* [~>~] */
   long Nb;
   long Ng;
   long Nwhl;
   long Nmtb;
   long Nthr;
   long Ncmg;
   long Ngyro;
   long Nmag;
   long Ncss;
   long Nfss;
   long Nst;
   long Ngps;
   long Nacc;

   double Pi;
   double TwoPi;

   double DT;
   double mass;
   vec3_t cm;
   mat3x3_t MOI;

   /*~ Inputs ~*/
   double Time; /* Time since J2000 [[sec]] */
   long Mode;
   vec3_t wbn;
   quat_t qbn;
   mat3x3_t CBN;
   mat3x3_t CLN;
   vec3_t wln;
   quat_t qln;
   vec3_t svn;
   vec3_t svb; /* [~<~] */
   vec3_t bvn;
   vec3_t bvb; /* [~<~] */
   vec3_t PosN;
   vec3_t VelN;
   long SunValid;
   long MagValid;
   long EphValid;
   long StValid;

   /*~ Outputs ~*/
   long ReqMode;

   vec3_t Tcmd;
   vec3_t Mcmd;
   vec3_t Fcmd;

   vec3_t IdealTrq;
   vec3_t IdealFrc;

   /*~ Internal Variables ~*/
   long Init;
   quat_t qrn;
   vec3_t wrn;
   quat_t qbr;
   vec3_t Hvb; /* [~<~] */

   /*~ Structures ~*/

   /* Dynamics */
   struct AcBodyType *B;  /* [*Nb*] */
   struct AcJointType *G; /* [*Ng*] */

   /* Sensors */
   struct AcGyroType *Gyro;        /* [*Ngyro*] */
   struct AcMagnetometerType *MAG; /* [*Nmag*] */
   struct AcCssType *CSS;          /* [*Ncss*] */
   struct AcFssType *FSS;          /* [*Nfss*] */
   struct AcStarTrackerType *ST;   /* [*Nst*] */
   struct AcGpsType *GPS;          /* [*Ngps*] */
   struct AcAccelType *Accel;      /* [*Nacc*] */
   struct AcEarthSensorType ES;

   /* Actuators */
   struct AcWhlType *Whl; /* [*Nwhl*] */
   struct AcMtbType *MTB; /* [*Nmtb*] */
   struct AcThrType *Thr; /* [*Nthr*] */

   /* Control Modes */
   struct AcPrototypeCtrlType PrototypeCtrl;
   struct AcAdHocCtrlType AdHocCtrl;
   struct AcSpinnerCtrlType SpinnerCtrl;
   struct AcMomBiasCtrlType MomBiasCtrl;
   struct AcThreeAxisCtrlType ThreeAxisCtrl;
   struct AcIssCtrlType IssCtrl;
   struct AcCmgCtrlType CmgCtrl;
   struct AcThrCtrlType ThrCtrl;
   struct AcCfsCtrlType CfsCtrl;
   struct AcThrSteerCtrlType ThrSteerCtrl;

   struct CmdType Cmd;
};

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __ACTYPES_H__ */

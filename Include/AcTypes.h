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
   vec3 N;  /* Components in N */
   vec3 W;  /* Components in W */
   vec3 L;  /* Components in L */
   vec3 R;  /* Components in R */
   vec3 T;  /* Components in T */
   vec3 wn; /* Angular velocity in N, expressed in N */
};

struct CmdType {
   /*~ Internal Variables ~*/
   long Parm;
   long Frame;
   vec3 AngRate; /* [~<~] */
   vec3 Ang;     /* [~<~] */
   vec3 PosRate;
   vec3 Pos;
   long RotSeq;
   quat qrl; /* [~!~] */
   quat qrn; /* [~!~] */
   vec3 wrn;
   double SpinRate;
   vec3 Hvr;
   vec3 Hvn;
   mat3x3 OldCRN;

   /*~ Structures ~*/
   struct CmdVecType PriVec;
   struct CmdVecType SecVec;
};

struct AcBodyType {
   /*~ Parameters ~*/
   double mass; /* [[kg]] */
   vec3 cm;     /* [[m]] */
   mat3x3 MOI;  /* [[kg-m^2]] */
};

struct AcJointType {
   /*~ Parameters ~*/
   long IsSpherical;
   long RotDOF;
   long TrnDOF;
   long RotSeq;
   long TrnSeq;
   mat3x3 CGiBi;
   mat3x3 CBoGo;
   vec3 AngGain;
   vec3 AngRateGain;
   vec3 PosGain;
   vec3 PosRateGain;
   vec3 MaxAngRate;
   vec3 MaxPosRate;
   vec3 MaxTrq;
   vec3 MaxFrc;

   /*~ Internal Variables ~*/
   vec3 Ang; /* [[rad]] [~>~] */
   vec3 AngRate;
   vec3 Pos;
   vec3 PosRate;
   mat3x3 COI;

   /*~ Structures ~*/
   struct CmdType Cmd;
};

struct AcGyroType {
   /*~ Parameters ~*/
   vec3 Axis;

   /*~ Internal Variables ~*/
   long Valid;
   double Rate; /* [[rad/sec]] [~>~] */
};

struct AcMagnetometerType {
   /*~ Parameters ~*/
   vec3 Axis;

   /*~ Internal Variables ~*/
   long Valid;
   double Field; /* [[Tesla]] [~>~] */
};

struct AcCssType {
   /*~ Parameters ~*/
   long Body;
   vec3 Axis;
   double Scale;

   /*~ Internal Variables ~*/
   long Valid;   /* [~>~] */
   double Illum; /* [~>~] */
};

struct AcFssType {
   /*~ Parameters ~*/
   quat qb;
   mat3x3 CB;
   long H_Axis;
   long V_Axis;
   long BoreAxis;

   /*~ Internal Variables ~*/
   enum fssTypes type;
   long Valid;       /* [~>~] */
   double SunAng[2]; /* [[rad]] [~>~] */
   vec3 SunVecS;
   vec3 SunVecB;
};

struct AcStarTrackerType {
   /*~ Parameters ~*/
   quat qb;
   mat3x3 CB;

   /*~ Internal Variables ~*/
   long Valid; /* [~>~] */
   quat qn;    /* [~>~] */
   quat qbn;
   long BoreAxis; /* X_AXIS, Y_AXIS, Z_AXIS */
};

struct AcGpsType {
   /*~ Internal Variables ~*/
   long Valid;    /* [~>~] */
   long Rollover; /* [~>~] */
   long Week;     /* [~>~] */
   double Sec;    /* [~>~] */
   vec3 PosN;     /* [[m]] [~>~] */
   vec3 VelN;     /* [[m/s]] [~>~] */
   vec3 PosW;     /* [[m]] [~>~] */
   vec3 VelW;     /* [[m/s]] [~>~] */
   double Lng;    /* Geocentric [[rad]] [~>~] */
   double Lat;    /* Geocentric [[rad]] [~>~] */
   double Alt;    /* Geocentric [[m]] [~>~] */
   double WgsLng; /* Geodetic, WGS-84 [[rad]] [~>~] */
   double WgsLat; /* Geodetic, WGS-84 [[rad]] [~>~] */
   double WgsAlt; /* Geodetic, WGS-84 [[m]] [~>~] */
};

struct AcAccelType {
   /*~ Parameters ~*/
   vec3 PosB;
   vec3 Axis;

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
   vec3 Axis;
   vec3 DistVec;
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
   vec3 Axis;
   vec3 DistVec;
   double Mmax;

   /*~ Internal Variables ~*/
   double Mcmd; /* [[A-m^2]] [~<~] */
};

struct AcThrType {
   /*~ Parameters ~*/
   long Body;
   vec3 PosB;
   vec3 Axis;
   vec3 rxA;
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
   vec3 Tcmd;
   quat qbr;
   vec3 therr;
   vec3 werr;
};

struct AcAdHocCtrlType {
   /*~ Parameters ~*/
   vec3 Kr;
   vec3 Kp;

   /*~ Internal Variables ~*/
   long Init;
   vec3 therr;
   vec3 werr;
   vec3 Tcmd;
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
   vec3 rvn, rvb;
   vec3 Tcmd;
   vec3 Mcmd;
};

struct AcMomBiasCtrlType {
   /*~ Internal Variables ~*/
   long Init;
};

struct AcThreeAxisCtrlType {
   /*~ Parameters ~*/
   vec3 Kr;
   vec3 Kp;
   double Kunl;

   /*~ Internal Variables ~*/
   long Init;
   vec3 Tcmd;
   vec3 Hwcmd;
};

struct AcIssCtrlType {
   /*~ Parameters ~*/
   vec3 Kr;
   vec3 Kp;
   double Tmax;

   /*~ Internal Variables ~*/
   long Init;
   vec3 therr;
   vec3 werr;
};

struct AcCmgCtrlType {
   /*~ Parameters ~*/
   vec3 Kr;
   vec3 Kp;

   /*~ Internal Variables ~*/
   long Init;
   vec3 therr, werr;
   vec3 Tcmd;
   quat AngRateCmd;
};

struct AcThrCtrlType {
   /*~ Parameters ~*/
   vec3 Kw;
   vec3 Kth;
   double Kv;
   double Kp;

   /*~ Internal Variables ~*/
   long Init;
};

struct AcCfsCtrlType {
   /*~ Parameters ~*/
   vec3 Kr;
   vec3 Kp;
   double Kunl;

   /*~ Internal Variables ~*/
   long Init;
   vec3 therr;
   vec3 werr;
};

struct AcThrSteerCtrlType {
   /*~ Parameters ~*/
   vec3 Kr;
   vec3 Kp;

   /*~ Internal Variables ~*/
   long Init;
   vec3 therr;
   vec3 werr;
   vec3 ierr;
   vec3 Tcmd;
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
   vec3 cm;
   mat3x3 MOI;

   /*~ Inputs ~*/
   double Time; /* Time since J2000 [[sec]] */
   long Mode;
   vec3 wbn;
   quat qbn;
   mat3x3 CBN;
   mat3x3 CLN;
   vec3 wln;
   quat qln;
   vec3 svn;
   vec3 svb; /* [~<~] */
   vec3 bvn;
   vec3 bvb; /* [~<~] */
   vec3 PosN;
   vec3 VelN;
   long SunValid;
   long MagValid;
   long EphValid;
   long StValid;

   /*~ Outputs ~*/
   long ReqMode;

   vec3 Tcmd;
   vec3 Mcmd;
   vec3 Fcmd;

   vec3 IdealTrq;
   vec3 IdealFrc;

   /*~ Internal Variables ~*/
   long Init;
   quat qrn;
   vec3 wrn;
   quat qbr;
   vec3 Hvb; /* [~<~] */

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

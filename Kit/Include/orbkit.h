/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __ORBKIT_H__
#define __ORBKIT_H__

#include "42constants.h"
#include "dcmkit.h"
#include "defineskit.h"
#include "iokit.h"
#include "mathkit.h"
#include "timekit.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

/* World Tags */
typedef enum WorldID {
   // TODO: maybe organize the moons to immediately follow the parent planet?
   // for example, the moons of Jupiter would be from
   // World[JUPITER+1] to World[SATURN-1]
   // this would make iterating over just the planets more difficult...
   SOL = 0,
   MERCURY,
   VENUS,
   EARTH,
   MARS,
   JUPITER,
   SATURN,
   URANUS,
   NEPTUNE,
   PLUTO,
   /* Moon of Earth */
   LUNA,
   /* Moons of Mars */
   PHOBOS,
   DEIMOS,
   /* Major Moons of Jupiter */
   IO,
   EUROPA,
   GANYMEDE,
   CALLISTO,
   AMALTHEA,
   HIMALIA,
   ELARA,
   PASIPHAE,
   SINOPE,
   LYSITHEA,
   CARME,
   ANANKE,
   LEDA,
   THEBE,
   ADRASTEA,
   METIS,
   /* Major Moons of Saturn */
   MIMAS,
   ENCELADUS,
   TETHYS,
   DIONE,
   RHEA,
   TITAN,
   HYPERION,
   IAPETUS,
   PHOEBE,
   JANUS,
   EPIMETHEUS,
   HELENE,
   TELESTO,
   CALYPSO,
   ATLAS,
   PROMETHEUS,
   PANDORA,
   PAN,
   /* Major Moons of Uranus */
   ARIEL,
   UMBRIEL,
   TITANIA,
   OBERON,
   MIRANDA,
   /* Major Moons of Neptune */
   TRITON,
   NEREID,
   /* Pluto's moon */
   CHARON,
   // set the number of non Minor Bodies
   NMAJORWORLD,
   /* Minor Bodies */
   MINORBODY_0 = NMAJORWORLD,
   MINORBODY_1,
   MINORBODY_2,
   MINORBODY_3,
   MINORBODY_4,
   MINORBODY_5,
   MINORBODY_6,
   MINORBODY_7,
   MINORBODY_8,
   MINORBODY_9,
   // leave this at the end to get the count of worlds
   NWORLD,
} WorldID;
// TODO: remove minor bodies from this list, make global World list dynamically
// allocated by counting the number of minor bodies in the minorbody file. Maybe
// make the minor body file a sim level configuration.
//    to do this, make NWORLD a global scope variable and init it to
//    NMAJORWORLD, changing it and reallocing World as needed to handle minor
//    bodies
#define N_PLANETS (PLUTO - SOL)

enum orbitRegime {
   ORB_ZERO = 0,
   ORB_FLIGHT,
   ORB_CENTRAL,
   ORB_THREE_BODY,
   ORB_N_BODY,
};

enum orbitInputType {
   INP_KEPLER = 0,
   INP_POSVEL,
   INP_FILE,
   INP_TLE,
   INP_TRV,
   INP_MODES,
   INP_XYZ,
   INP_XYZ_ROT,
   INP_SPLINE,
};

typedef enum LagrangeSystem {
   EARTHMOON = 0,
   SUNEARTH,
   SUNJUPITER,
   NLAGSYS,
} LagrangeSystem;

struct LagrangePointType {
   /*~ Internal Variables ~*/
   double PosN[3]; /* Pos wrt N frame of Body 1 (larger grav center), [[m]] */
   double
       VelN[3]; /* Vel wrt N frame of Body 1 (larger grav center), [[m/sec]] */
   double X0;   /* Radial location wrt Body 1, in Synodic Frame, m */
   double Y0;   /* Transversed location wrt Body 1, in Synodic Frame, m */
   double R1;
   double R2;
   double Kxx;
   double Kxy;
   double Kyy;
   double Zw1;
   double Zw2;
   double Zs;
   /* Modal frequencies (or time constant), rad/sec */
   double w1;
   double w2;
   double sigma;
   double wz;

   double ca1;
   double sa1;
   double ca2;
   double sa2;
   /* Modal aspect ratios */
   double AR1;
   double AR2;
};

struct LagrangeSystemType {
   /*~ Internal Variables ~*/
   long Exists;
   char Name[20];
   long Body1;
   long Body2;
   double mu1;
   double mu2;
   double rho;
   double SLR;
   double ecc;
   double inc;
   double RAAN;
   double ArgP;
   double tp;
   double SMA;
   double MeanRate;
   double Period;
   /* Nondimensional distance from m1 to m2, and derivatives */
   double D;
   double Ddot;
   double Ddotdot;
   /* CR3BP ND Units */
   double LU;
   double TU;
   double VU;

   /* True anomaly, and derivatives */
   double th;
   double thdot;
   double thdotdot;
   struct LagrangePointType LP[5];
   double CLH[3][3];
   double CLN[3][3];
};

/* Chebyshev coefficients.  Used for DE430 planetary ephemerides. */
struct Cheb3DType {
   /* Coefficients only valid for JD1 <= JulDay < JD2 */
   JDType JD1; // GMAT_MJD, TDB
   JDType JD2; // GMAT_MJD, TDB
   long N;     /* Order <= 20 */
   double Coef[3][20];
};

struct OrbitType {
   /*~ Parameters ~*/

   long Tag; /* Orb[Tag].Tag = Tag */
   long Exists;
   TimeSystem EphemSystem; /* Time system used for this orbit */
   double
       Epoch; /* Sec since J2000 epoch at which orbit elements are referenced */
   enum orbitRegime Regime; /* ZERO, FLIGHT, CENTRAL (Two-body) or THREE_BODY */
   long PolyhedronGravityEnabled;
   WorldID World;
   long Region;

   /* For Three-Body Orbit Description */
   long Sys; /* e.g. SUNEARTH, EARTHMOON, SUNJUPITER */
   long LP;  /* Lagrange Point [0-4] */
   long Body1;
   long Body2;
   double mu1;
   double mu2;
   long LagDOF;
   /* Modal parameters, m */
   double Ax;
   double Bx;
   double Cx;
   double Dx;
   double Ay;
   double By;
   double Cy;
   double Dy;
   double Az;
   double Bz;

   /* For Central Orbit Description */
   double mu;
   double SMA;   /* Semi-major axis [[m]] */
   double ecc;   /* Eccentricity */
   double inc;   /* Inclination, [[rad]] */
   double RAAN;  /* Right Ascension of Ascending Node, [[rad]] */
   double ArgP;  /* Argument of Periapsis, [[rad]] */
   double tp;    /* Time of Periapsis Passage, [[sec]] since J2000 epoch */
   double alpha; /* 1/SMA.  Better behaved than SMA when e near 1.0 [[1/m]] */
   double SLR;   /* Semilatus rectum.  Always well behaved [[m]] */
   double rmin;  /* Periapsis radius.  Always well behaved [[m]] */
   double Period;
   double MeanMotion;
   char FileName[40];
   long J2DriftEnabled;
   /* J2 Drift Parameters */
   double MeanSMA;
   double RAAN0;
   double ArgP0;
   double MeanAnom0;
   double RAANdot; /* Due to average J2 effect, rad/sec */
   double ArgPdot; /* Due to average J2 effect, rad/sec */
   double J2Rw2bya;
   char SplineFileName[1050];
   FILE *SplineFile;

   /*~ Internal Variables ~*/
   /* Linearized three-body motion about LP (X0,Y0), m and m/sec */
   double x;
   double y;
   double z;
   double xdot;
   double ydot;
   double zdot;

   /* SC Pos/Vel direct import from TRVs */
   /* This bypasses converting Pos/Vel into a viable two-body orbit and
   then converting back to SC Pos/Vel from whatever the new two-body
   orbit values are. This allows SC to have the EXACT Pos/Vel that is
   input in the the TRV file. Note that this assumes you are using
   COWELL method for orbit propagation in SC configuration and that
   you are using a JPL Ephemerides as the Ephem option. */
   int use_N_BODY_Vec;
   double N_BODY_PosN[3]; /* SC Position from TRV file, [[m]], expressed in N of
                             CENTRAL body */
   double N_BODY_VelN[3]; /* SC Velocity from TRV file, [[m/sec]], expressed in
                             N of CENTRAL body */

   /* For Central Orbit Description */
   double MeanAnom;
   double anom;      /* True Anomaly, rad */
   double PosN[3];   /* Position, [[m]], expressed in N [~=~] */
   double VelN[3];   /* Velocity, [[m/sec]], expressed in N [~=~] */
   double CLN[3][3]; /* For ZERO, L = N.  For FLIGHT, L = ENU.  For CENTRAL, L =
                        LVLH.  For THREE_BODY, L = XYZ */
   double wln[3];    /* Expressed in N */
   /* Fit spline to data file */
   long SplineActive;
   double NodeDynTime[4]; /* Sec since J2000 (TT) */
   double NodePos[4][3];
   double NodeVel[4][3];
   /* Chebyshev Coefficients */
   long Ncheb;
   struct Cheb3DType *Cheb;
};

struct SphereHarmType {
   /*~ Internal Variables ~*/
   char modelFile[40];
   long Type;
   long N;
   long M;
   double **Norm;
   double **C;
   double **S;
   double r_ref;
};

struct AtmoType {
   /*~ Internal Variables ~*/
   long Exists;
   float GasColor[3];
   float DustColor[3];
   float RayScat[3];
   float MieScat;
   float RayScaleHt;
   float MieScaleHt;
   float MieG;
   double MaxHt;
   double rad;
};

/* Contains data for calculating the prime meridian angle             */
/*    primarily for cspice                                            */
typedef struct AngDataType {
   /*~ Internal Variables ~*/
   char ang_char; // 'P', 'R', 'D', or '\0'
   // for 'P',       t = day
   // for 'R' & 'D', t = julian century
   // '\0' is Invalid flag
   double ang[3]; // deg, deg/t, deg/t^2
   int n_ang;
   int n_E;
   double *nut_prec_ang;    // deg
   double (*nut_prec_E)[2]; // {deg, deg/(jd century)}
} AngDataType;

// equivalent to (AngDataType){0}
#define ANGDATATYPE_INVALID                                                    \
   (AngDataType)                                                               \
   {                                                                           \
      .ang_char = '\0', .ang = {0.0, 0.0, 0.0}, .n_ang = 0, .n_E = 0,          \
      .nut_prec_ang = NULL, .nut_prec_E = NULL                                 \
   }

struct WorldType {
   /*~ Parameters ~*/

   /* Relationships */
   long Exists;
   long Type; /* STAR, PLANET, MOON, ASTEROID, COMET */
   WorldID Parent;
   long Nsat;
   WorldID *Sat; /* [*Nsat*] */

   /* Physical Properties */
   double mu;  /* Gravitation constant  */
   double J2;  /* Gravitation oblateness parameter */
   double rad; /* Radius */
   // double w;               /* Spin Rate */
   // double PriMerAngJ2000;  /* Prime Meridian Angle at J2000 epoch, rad */
   double RadOfInfluence;  /* Radius of Sphere of Influence */
   double DipoleMoment;    /* Magnetic Field Dipole Moment, Wb-m */
   double DipoleAxis[3];   /* Magnetic Field Dipole Axis */
   double DipoleOffset[3]; /* Dipole Offset, m */
   double RingInner, RingOuter;
   double Density; /* For minor bodies, polyhedron gravity */
   struct SphereHarmType GravModel;

   /* Graphical Properties */
   long HasRing;
   char Name[20];
   char MapFileName[40];
   char GeomFileName[40];
   char ColTexFileName[40];
   char BumpTexFileName[40];
   float Color[4];
   unsigned char Glyph[14];
   unsigned int TexTag;
   unsigned int MapTexTag;
   unsigned int ColTexTag;
   unsigned int BumpTexTag;
   unsigned int ColCubeTag;
   unsigned int BumpCubeTag;
   unsigned int CloudGlossCubeTag;
   long GeomTag;
   unsigned int RingTexTag;
   double NearExtent, FarExtent;

   long OrientWorld; /* Compute the worlds orientation for this SimStep
                        (used only with SPICE)*/
   double CNH[3][3]; /* DCM from heliocentric ecliptic frame
                        to world-centric equatorial inertial frame */
   double qnh[4];    /* ~*/
   double CNJ[3][3]; /* DCM from J2000 frame to world-centric equatorial
                        inertial frame */
   double qnj[4];

   /*~ Internal Variables ~*/
   /* contains information defining prime meridian angle information */
   /*    order: Prime Meridian, Right Ascension, Declination         */
   AngDataType ang_data[3];

   double PosH[3];   /* Position in H frame [~=~] */
   double VelH[3];   /* Velocity in H frame */
   double PriMerAng; /* Angle from N1 to prime meridian */
   double CWN[3][3]; /* DCM from world-centric inertial frame
                        to world-centric rotating frame */
   double qwn[4];    /* ~*/
   long Visibility;  /* Too small to see, point-sized, or shows disk */
   float ModelMatrix[16];

   /*~ Structures ~*/
   struct OrbitType eph; /* Ephemeris */
   struct AtmoType Atmo;
};

/*~ Prototypes ~*/
WorldID GetWorldID(const char *s);

void CloneWorld(struct WorldType *const destWorld,
                const struct WorldType srcWorld);
void CopyWorld(struct WorldType *const destWorld,
               const struct WorldType srcWorld);
double GetWorldW(JDType jd, const struct WorldType *const world)
    __attribute__((pure));
void GetWorldWln(JDType jd, const struct WorldType *const world, double wln[3]);
AngDataType CopyAngData(const AngDataType src) __attribute__((const));
double GetWorldAng(JDType jd, const AngDataType *const ang_data)
    __attribute__((pure));
double GetWorldCWN(JDType jd, const AngDataType *const ang_data,
                   double CWN[3][3]);
void GetWorldCNJ(JDType jd, const AngDataType *const ang_data,
                 double CNJ[3][3]);

void CloneOrbit(struct OrbitType *const destOrb, const struct OrbitType srcOrb);
void CopyOrbit(struct OrbitType *const destOrb, const struct OrbitType srcOrb);
void WorldID2String(WorldID w_id, char w_str[32]);
double MeanAnomToTrueAnom(double MeanAnom, double ecc) __attribute__((const));
double TrueAnomaly(double mu, double p, double e, double t)
    __attribute__((const));
double atanh(double x) __attribute__((const));
double TimeSincePeriapsis(double mu, double p, double e, double th)
    __attribute__((const));
void RV02RV(double mu, double xr0[3], double xv0[3], double anom, double xr[3],
            double xv[3]);
void Eph2RV(double mu, double p, double e, double i, double RAAN, double ArgP,
            double dt, double r[3], double v[3], double *anom);
void RV2Eph(double time, double mu, double xr[3], double xv[3], double *SMA,
            double *e, double *i, double *RAAN, double *ArgP, double *th,
            double *tp, double *SLR, double *alpha, double *rmin,
            double *MeanMotion, double *Period);
void TLE2MeanEph(const char Line1[80], const char Line2[80], JDType jd,
                 struct OrbitType *O);
void MeanEph2RV(struct OrbitType *O, double DynTime);
long LoadTleFromFile(const char *Path, const char *TleFileName,
                     const char *TleLabel, double DynTime, JDType jd,
                     struct OrbitType *O);
double RV2RVp(double mu, double r[3], double v[3], double rp[3], double vp[3])
    __attribute__((pure));
void PlanetEphemerides(long i, JDType jd, double mu, double *SMA, double *ecc,
                       double *inc, double *RAAN, double *omg, double *tp,
                       double *anom, double *p, double *alpha, double *rmin,
                       double *MeanMotion, double *Period);
void LunaPosition(const JDType jd, double r[3]);
int LoadLunaInertialFrameData(AngDataType *const ang_data);
void LunaInertialFrame(const JDType jd, double CNJ[3][3]);
int LoadLunaPriMerAngData(AngDataType *const ang_data);
double LunaPriMerAng(JDType JulDay) __attribute__((const, deprecated));
void FindCLN(double r[3], double v[3], double CLN[3][3], double wln[3]);
void FindCEN(double r[3], double CEN[3][3]);
void FindENU(double PosN[3], double WorldW, double CLN[3][3], double wln[3]);
void FindLagPtParms(struct LagrangeSystemType *LS);
void FindLagPtPosVel(double SecSinceJ2000, struct LagrangeSystemType *S,
                     long Ilp, double PosN[3], double VelN[3],
                     double CLN[3][3]);
void LagModes2RV(double SecSinceJ2000, struct LagrangeSystemType *LS,
                 struct OrbitType *O, double r[3], double v[3]);
void RV2LagModes(double SecSinceJ2000, struct LagrangeSystemType *LS,
                 struct OrbitType *O);
void R2StableLagMode(double SecSinceJ2000, struct LagrangeSystemType *LS,
                     struct OrbitType *O);
void XYZ2LagModes(double TimeSinceEpoch, struct LagrangeSystemType *LS,
                  struct OrbitType *O);
void AmpPhase2LagModes(double TimeSinceEpoch, double AmpXY1, double PhiXY1,
                       double SenseXY1, double AmpXY2, double PhiXY2,
                       double SenseXY2, double AmpZ, double PhiZ,
                       struct LagrangeSystemType *S, struct OrbitType *O);
void TDRSPosVel(double PriMerAng, double TIME, double ptn[10][3],
                double vtn[10][3]);
void TETE2J2000(double JD, double CTJ[3][3]);
double RadiusOfInfluence(double mu1, double mu2, double r)
    __attribute__((const));
void RelRV2EHRV(double OrbRadius, double OrbRate, double OrbCLN[3][3],
                double Rrel[3], double Vrel[3], double re[3], double ve[3]);
void EHRV2RelRV(double OrbRadius, double OrbRate, double OrbCLN[3][3],
                double re[3], double ve[3], double Rrel[3], double Vrel[3]);
void EHRV2EHModes(double r[3], double v[3], double n, double nt, double *A,
                  double *Bc, double *Bs, double *C, double *Dc, double *Ds);
void EHModes2EHRV(double A, double Bc, double Bs, double C, double Dc,
                  double Ds, double n, double nt, double r[3], double v[3]);
double LambertTOF(double mu, double amin, double lambda, double x)
    __attribute__((const));
void LambertProblem(double t0, double mu, double xr1[3], double xr2[3],
                    double TOF, double TransferType, double *SLR, double *e,
                    double *inc, double *RAAN, double *ArgP, double *tp);
double RendezvousCostFunction(double *InVec, double *AuxVec);
void PlanTwoImpulseRendezvous(double mu, double r1e[3], double v1e[3],
                              double r2e[3], double v2e[3], double *t1,
                              double *t2, double DV1[3], double DV2[3]);
void FindLightLagOffsets(double DynTime, struct OrbitType *Observer,
                         struct OrbitType *Target, double PastPos[3],
                         double FuturePos[3]);
void OscEphToMeanEph(double mu, double J2, double Rw, JDType jd,
                     struct OrbitType *O);
void MeanEphToOscEph(struct OrbitType *O, double DynTime);

void StateRnd2StateN(struct LagrangeSystemType *LS, double W2_pos[3],
                     double W2_vel[3], double R_R_nd[3], double V_R_nd[3],
                     double R_N[3], double V_N[3]);
void StateN2StateRnd(struct LagrangeSystemType *LS, double W2_pos[3],
                     double W2_vel[3], double R_N[3], double V_N[3],
                     double R_R_nd[3], double V_R_nd[3]);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __ORBKIT_H__ */

/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

/* Disable extern keyword to declare globals */
#ifdef DECLARE_GLOBALS
#define EXTERN
#else
#define EXTERN extern
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "42defines.h"
#include "42types.h"
#include "dcmkit.h"
#include "docoptkit.h"
#include "dsmkit.h"
#include "envkit.h"
#include "fswkit.h"
#include "geomkit.h"
#include "iokit.h"
#include "mathkit.h"
#include "orbkit.h"
#include "sigkit.h"
#include "sphkit.h"
#include "timekit.h"

#define BUFSIZE 1000 // Default buffer size for chars.

/*
** #ifdef __cplusplus
** namespace _42 {
** using namespace Kit;
** #endif
*/

/* Number of Reference Orbits */
EXTERN long Norb;
/* Number of spacecraft */
EXTERN long Nsc;

/* Number of materials */
EXTERN long Nmatl;
/* Number of geometric objects */
EXTERN long Ngeom;

/* Directories */
EXTERN char InOutPath[BUFSIZE];
EXTERN char ExeDir[BUFSIZE];
EXTERN char OutPath[BUFSIZE];
EXTERN char ModelPath[BUFSIZE];
EXTERN char SCModelPath[BUFSIZE];
EXTERN char CmdFileName[BUFSIZE];

/* ******************* */

EXTERN struct DocoptArgs CLI_ARGS;

/* Math Basics */
EXTERN double Pi, TwoPi, HalfPi, SqrtTwo, SqrtHalf, A2R, R2A, GoldenRatio;

/* Simulation Control */
EXTERN long TimeMode; /* FAST_TIME, REAL_TIME, EXTERNAL_SYNCH, NOS3_TIME */
EXTERN double SimTime, STOPTIME, DTSIM, DTOUT, DTOUTGL;
EXTERN long OutFlag, GLOutFlag, GLEnable, CleanUpFlag;

// /* Adding parameters to help with variable DTSIM  */
// // TODO: Introduce a real variable-time integration method!!!
// EXTERN double t_cnt_RPT;  /* This is a time counter for report function */
// EXTERN double t_cnt_GUI;  /* This is a time counter for GUI function */
// EXTERN double DTSIM_min;  /* This is the minimum variable timestep */
// EXTERN double oldDTSIM;   /* Copy of original DTSIM set in InpSim */

/* Making global parameters for updated JPL EPHEM methods */
EXTERN double EMRAT;   /* Earth/Moon Mass Ratio */
EXTERN double AU;      /* Number of kilometers in 1 AU */
EXTERN double AUd2ms;  /* Conversion from [au**3/day**2] to [m**3/s**2] */

// If TRV method is used, SC Pos/Vel is directly intialized from TRV values
EXTERN long DirectTRVmethod;

/* Environment */
EXTERN struct SphereHarmType MagModel; /* -3,...,10 */
EXTERN long SurfaceModel;              /* 0=Brick, 1=CylPlate */
EXTERN long AeroActive;
EXTERN long AeroShadowsActive;
EXTERN long GGActive;
EXTERN long SolPressActive;
EXTERN long SolPressShadowsActive;
EXTERN long GravPertActive;
EXTERN long ThrusterPlumesActive;
EXTERN long ResidualDipoleActive;
EXTERN long ContactActive;
EXTERN long SloshActive;
EXTERN long AlbedoActive; /* Affects CSS measurements */
EXTERN long ComputeEnvTrq;
EXTERN long EphemOption; /* MEAN, DE421, DE424, DE430, DE440, GMAT421, GMAT424, or SPICE */

/* Calendar Time is all based in Terrestrial Dynamical Time (TT or TDT) unless
 * otherwise noted */
EXTERN double DynTime0;    /* Time in sec since J2000 Epoch at Sim Start (TT) */
EXTERN double DynTime;     /* Absolute Time (TT), sec since J2000 Epoch */
EXTERN double AtomicTime;  /* TAI = TT - 32.184 sec, sec since J2000 */
EXTERN double LeapSec;     /* Add to civil time (UTC) to synch with TAI */
EXTERN double CivilTime;   /* UTC = TAI - LeapSec */
EXTERN double GpsTime;     /* GPS Time = TAI - 19.0 sec */
EXTERN struct DateType TT; /* Terrestrial Dynamical Time */
EXTERN struct DateType UTC; /* Universal Time Coordinated */
EXTERN long GpsRollover, GpsWeek;
EXTERN double GpsSecond;

EXTERN long double DynTime0_ld; /* Time in sec since J2000 Epoch at Sim Start (TT) */
EXTERN long double DynTime_ld; /* Absolute Time (TT), sec since J2000 Epoch */
EXTERN struct DateType_ld TDB; /* Barycentric Dynamical Time */

/* Parameters for environmental models  */
EXTERN long AtmoOption; /* TWOSIGMA_ATMO, NOMINAL_ATMO, USER_ATMO */
EXTERN double Flux10p7, GeomagIndex;
EXTERN double
    SchattenTable[5][1009]; /* JD, +2sig F10.7, Nom F10.7, +2sig Kp, Nom Kp */

EXTERN struct WorldType World[NWORLD];
EXTERN struct LagrangeSystemType LagSys[3];

/* Galactic Coordinate Frame */
EXTERN double CGH[3][3];

/* J2000 to Heliocentric Ecliptic */
EXTERN double qjh[4];

/* SC structure manages attitude and translation wrt Reference Orbit */
EXTERN struct SCType *SC;
/* Orb structure manages Reference Orbits */
EXTERN struct OrbitType *Orb;
/* Frm structure describes a Formation of S/C's */
EXTERN struct FormationType *Frm;

/* Geom structure manages geometric objects, used for display and    */
/* for surface force computation (e.g. aerodynamic)                  */
EXTERN struct GeomType *Geom;

EXTERN struct POVType POV;

EXTERN struct TdrsType Tdrs[10];
EXTERN struct GroundStationType *GroundStation;
EXTERN long Ngnd;

/* For drawing Fields of View */
EXTERN long Nfov;
EXTERN struct FovType *FOV;

/* Materials for graphics and solar pressure forces.                  */
EXTERN struct MatlType *Matl;

/* Framebuffer Objects for Shadows and Surface Forces */
EXTERN struct ShadowFBOType ShadowMap;
EXTERN struct AlbedoFBOType AlbedoFBO;

/* Minor Bodies (Asteroids and Comets) */
EXTERN long Nmb;

EXTERN long Nrgn;
EXTERN struct RegionType *Rgn;

/* CFD execution control */
EXTERN long ExecuteCFDStep;
EXTERN long EndCFD;

/* Inter-Process Comm */
EXTERN long Nipc;
EXTERN struct IpcType *IPC;

/* Master Random Process */
EXTERN struct RandomProcessType *RNG;
EXTERN long RngSeed;

EXTERN double MapTime, JointTime, PathTime, PVelTime, FrcTrqTime;
EXTERN double AssembleTime, LockTime, TriangleTime, SubstTime, SolveTime;

EXTERN struct ConstellationType Constell[89];

void GravPertForceRK4(struct SCType *S, long double u[6], double FrcN[3], double RKFdt);
void ThirdBodyGravForce(double p[3], double s[3], double mu, double mass,
                        double Frc[3]);
void Rk4JplEphems(long double JD, long trgtWORLD, long double trgtPosN[3], long double trgtPosH[3],
                  long double *trgtPriMerAng, long double trgtCNH[3][3]);
void Rk4SpiceEphems(long double JD, long trgtWORLD, long double trgtPosN[3], long double trgtPosH[3],
                    long double *trgtPriMerAng, long double trgtCNH[3][3]);

long SimStep(void);
void Ephemerides(void);
void OrbitMotion(double Time);
void Environment(struct SCType *S);
void Perturbations(struct SCType *S);
void Sensors(struct SCType *S);
void SensorDriver(struct SCType *S);
void FlightSoftWare(struct SCType *S);
void ActuatorDriver(struct SCType *S);
void Actuators(struct SCType *S);
void CmdInterpreter(void);
void Report(void);
void DrawScene(void);
void ThreeBodyOrbitRK4(struct OrbitType *O);
void MotionConstraints(struct SCType *S);
void SCMassProps(struct SCType *S);
void MapJointStatesToStateVector(struct SCType *S);
void MapStateVectorToBodyStates(double *u, double *x, double *h, double *a,
                                double *uf, double *xf, struct SCType *S);
void BodyStatesToNodeStates(struct SCType *S);
void PartitionForces(struct SCType *S);
void Dynamics(struct SCType *S);
void Cleanup(void);
void FindInterBodyDCMs(struct SCType *S);
void FindPathVectors(struct SCType *S);
void FindTotalAngMom(struct SCType *S);
double FindTotalKineticEnergy(struct SCType *S);
void UpdateScBoundingBox(struct SCType *S);
void FindUnshadedAreas(struct SCType *S, double DirVecN[3]);
void RadBelt(float RadiusKm, float MagLatDeg, int NumEnergies,
             float *ElectronEnergy, float *ProtonEnergy, double **Flux);
void InitAlbedo(void);
void FindCssAlbedo(struct SCType *S, struct CssType *CSS);
void FindFssAlbedo(struct SCType *S, struct FssType *FSS);
void JointFrcTrq(struct JointType *G, struct SCType *S);
void InitActuatedJoint(struct JointType *G, struct SCType *S);
void WheelJitter(struct WhlType *W, struct SCType *S);
void ShakerJitter(struct ShakerType *Sh, struct SCType *S);
long OpticalFieldPoint(double StarVecB[3], struct OpticsType *O,
                       double FldPntB[3], double FldDirB[3]);
long OpticalTrain(long FldSC, long FldBody, double FldPntB[3],
                  double FldDirB[3], long Nopt, struct OpticsType *Opt,
                  long *OutSC, long *OutBody, double OutPntB[3],
                  double OutDirB[3]);

/* Debug Function Prototypes */
void EchoPVel(struct SCType *S);
void EchoEOM(double **COEF, double *State, double *RHS, long Ns);
void EchoStates(int Nx, double *x, int Nu, double *u);
void EchoRemAcc(struct SCType *S);

void InitSim(int argc, char **argv);
void InitOrbits(void);
void InitSpacecraft(struct SCType *S);
void LoadPlanets(void);
/* Load defined SPICE kernels from Model/spice_kernels/kernels.txt */
long LoadSpiceKernels(char SpicePath[80]);
/* Update celestial body locations at TT.JulDay using SPICE*/
long UpdateSpiceEphems(long double JS);
/* Load appropriate JPL Ephem (421,424,430,440, +GMAT varients)
to get Chebyshev coefficients for current JD range (TT) */
long LoadJplEphems(char EphemPath[80], long double JD);
/* Update celestial body locations at TT.JulDay using JPL Ephem*/
void UpdateJplEphems(void);
/* Update celestial body locations using MEAN method */
void UpdateMeanEphems(void);
/* Updates minor body locations using two-body methods */
void UpdateMinorBodies(void);
/* Updates all (non Earth) planertary moon locations using two-body methods */
void UpdateNonEphemMoons(void);
long DecodeString(char *s);
void InitFSW(struct SCType *S);
void InitAC(struct SCType *S);
void InitDSM(struct SCType *S);
void InitLagrangePoints(void);
/* Updates Lagrange System constants based on updated/variable orbit ephems */
void UpdateLagrangePoints(void);

long LoadTRVfromFile(const char *Path, const char *TrvFileName,
                     const char *ElemLabel, double DynTime,
                     struct OrbitType *O);
void SplineToPosVel(struct OrbitType *O);

void CfdSlosh(struct SCType *S);
void FakeCfdSlosh(struct SCType *S);
void SendStatesToSpirent(void);

void NOS3Time(long *year, long *day_of_year, long *month, long *day, long *hour,
              long *minute, double *second);

void InterProcessComm(void);
void InitInterProcessComm(void);

#undef EXTERN

/*
** #ifdef __cplusplus
** }
** #endif
*/

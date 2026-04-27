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
#include "jdkit.h"
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

/* Making global parameters for updated JPL EPHEM methods */
EXTERN double EMRAT;  /* Earth/Moon Mass Ratio */
EXTERN double AU;     /* Number of kilometers in 1 AU */
EXTERN double AUd2ms; /* Conversion from [au**3/day**2] to [m**3/s**2] */

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
EXTERN ephemType EphemOption;   /* MEAN, DE421, DE424, DE430, DE440, GMAT421,
                                   GMAT424, or SPICE */
EXTERN JPLHeaderType JplHeader; /* Stores header information for
                                   DE ephem types*/

/* Calendar Time is all based in Terrestrial Dynamical Time (TT or TDT) unless
 * otherwise noted */
EXTERN JDType
    JD_TDB_MJD;         /* Julian day in TDB with reference to GMAT MJD epoch*/
EXTERN double DynTime0; /* Time in sec since J2000 Epoch at Sim Start (TT) */
EXTERN double DynTime;  /* Absolute Time (TT), sec since J2000 Epoch */
EXTERN double AtomicTime; /* TAI = TT - 32.184 sec, sec since J2000 */
EXTERN double LeapSec;    /* Add to civil time (UTC) to synch with TAI */
EXTERN double CivilTime;  /* UTC = TAI - LeapSec */
EXTERN double GpsTime;    /* GPS Time = TAI - 19.0 sec */
EXTERN DateType TDB;      /* Barycentric Dynamical Time */
EXTERN DateType TT;       /* Terrestrial Dynamical Time */
EXTERN DateType UTC;      /* Universal Time Coordinated */
EXTERN long GpsRollover, GpsWeek;
EXTERN double GpsSecond;

/* Parameters for environmental models  */
EXTERN long AtmoOption; /* TWOSIGMA_ATMO, NOMINAL_ATMO, USER_ATMO */
EXTERN double Flux10p7, GeomagIndex;
EXTERN double SchattenTable[5][1009]; /* JD TT GMAT MJD, +2sig F10.7, Nom F10.7,
                                         +2sig Kp, Nom Kp */

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

void GravPertForceRK4(struct WorldType *const worlds,
                      struct OrbitType *const orbs, struct SCType *S,
                      double u[6], double FrcN[3], double RKFdt);
void ThirdBodyGravForce(double p[3], double s[3], double mu, double mass,
                        double Frc[3]);
void Rk4JplEphems(JDType jd, long trgtWORLD, struct WorldType *const worlds,
                  double trgtPosN[3], double trgtPosH[3], double *trgtPriMerAng,
                  double trgtCNH[3][3]);
void Rk4SpiceEphems(JDType jd, WorldID trgtWORLD,
                    struct WorldType *const worlds, double trgtPosN[3],
                    double trgtPosH[3], double *trgtPriMerAng,
                    double trgtCNH[3][3]);

long SimStep(void);
void Ephemerides(struct SCType *scs, struct WorldType *const worlds,
                 struct OrbitType *const orbs);
void OrbitMotion(struct WorldType *const worlds, struct OrbitType *const orbs,
                 double Time);
void Environment(JDType jd, struct WorldType *const worlds,
                 struct OrbitType *const orbs, struct SCType *S);
void Perturbations(struct WorldType *const worlds, struct OrbitType *const orbs,
                   struct SCType *S);
void Sensors(struct WorldType *const worlds, struct OrbitType *const orbs,
             struct SCType *S);
void SensorDriver(struct SCType *S);
void FlightSoftWare(struct SCType *S);
void ActuatorDriver(struct SCType *S);
void Actuators(struct SCType *S);
void CmdInterpreter(void);
void Report(void);
void DrawScene(void);
void ThreeBodyOrbitRK4(struct WorldType *worlds, struct OrbitType *O);
void MotionConstraints(struct SCType *S);
void SCMassProps(struct SCType *S);
void MapJointStatesToStateVector(struct SCType *S);
void MapStateVectorToBodyStates(double *u, double *x, double *h, double *a,
                                double *uf, double *xf, struct SCType *S);
void BodyStatesToNodeStates(struct SCType *S);
void PartitionForces(struct SCType *S);
void Dynamics(struct WorldType *const worlds, struct OrbitType *const orbs,
              struct SCType *S);
void Cleanup(void);
void FindInterBodyDCMs(struct SCType *S);
void FindPathVectors(struct SCType *S);
void FindTotalAngMom(struct SCType *S);
double FindTotalKineticEnergy(struct OrbitType *orbs, struct SCType *S);
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
void InitOrbits(struct OrbitType *O, const JDType jd);
void InitSpacecraft(struct SCType *S);
void LoadPlanets(const ephemType ephem, const JDType jd,
                 const JPLHeaderType *const jpl_hdr,
                 struct WorldType *const worlds);
/* Load defined SPICE kernels from Model/spice_kernels/kernels.txt */
long LoadSpiceKernels(char SpicePath[80]);
/* handler to determine which Update*Ephems() subfunction to call */
long UpdateEphems(const ephemType ephem, const JDType jd,
                  const JPLHeaderType *const jpl_hdr,
                  struct WorldType *const worlds);
/* Update celestial body locations at TT.JulDay using SPICE*/
long UpdateSpiceEphems(const JDType jd, struct WorldType *const worlds);
/* Load appropriate JPL Ephem (421,424,430,440, +GMAT varients)
to get Chebyshev coefficients for current JD range (TDB) */
long LoadJplEphems(char EphemPath[128], JPLHeaderType *const jpl_hdr,
                   const JDType jd, struct WorldType *const worlds);
/* Update celestial body locations at TT.JulDay using JPL Ephem*/
long UpdateJplEphems(const JDType jd, const JPLHeaderType *const jpl_hdr,
                     struct WorldType *const worlds);
/* Update celestial body locations using MEAN method */
long UpdateMeanEphems(const JDType jd, struct WorldType *const worlds);
/* Updates minor body locations using two-body methods */
long UpdateMinorBodies(const JDType jd, struct WorldType *const worlds);
/* Updates all (non Earth) planertary moon locations using two-body methods */
long UpdateNonEphemMoons(const JDType jd, struct WorldType *const worlds);
long DecodeString(char *s);
WorldID GetWorldID(const char *s);
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

DateType NOS3Time();

void InterProcessComm(void);
void InitInterProcessComm(void);

#undef EXTERN

/*
** #ifdef __cplusplus
** }
** #endif
*/

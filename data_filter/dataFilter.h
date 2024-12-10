/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __DATAFILTER_H__
#define __DATAFILTER_H__

// TODO: remove unneeded
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "42defines.h"
#include "42dsm.h"
#include "42types.h"
#include "AcTypes.h"
#include "DSMTypes.h"
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
#include "sqlite3.h"
#include "timekit.h"
#include <ctype.h>
#include <stdint.h>
#include <unistd.h>

#if defined(__MINGW32__) || (_WIN32) || (_WIN64)
#include <Windows.h>
#elif defined __APPLE__
#include <mach-o/dyld.h>
#elif defined __linux__
#else
#error "Computing platform not detected!"
#endif

#include <dirent.h>
#include <errno.h>
#include <strings.h>

#define BUFSIZE 1000 // Default buffer size for chars.

/* Number of Reference Orbits */
long Norb;
/* Number of spacecraft */
long Nsc;

/* Number of materials */
long Nmatl;
/* Number of geometric objects */
long Ngeom;

/* Directories */
char InOutPath[BUFSIZE];
char ExeDir[BUFSIZE];
char OutPath[BUFSIZE];
char ModelPath[BUFSIZE];
char SCModelPath[BUFSIZE];
char CmdFileName[BUFSIZE];

int main(int argc, char **argv);

/* ******************* */

struct DocoptArgs CLI_ARGS;

/* Math Basics */
double Pi, TwoPi, HalfPi, SqrtTwo, SqrtHalf, GoldenRatio;

/* Simulation Control */
long TimeMode; /* FAST_TIME, REAL_TIME, EXTERNAL_SYNCH, NOS3_TIME */
double SimTime, STOPTIME, DTSIM, DTOUT, DTOUTGL;
long OutFlag, GLOutFlag, GLEnable, CleanUpFlag;

/* Environment */
struct SphereHarmType MagModel; /* -3,...,10 */
long SurfaceModel;              /* 0=Brick, 1=CylPlate */
long AeroActive;
long AeroShadowsActive;
long GGActive;
long SolPressActive;
long SolPressShadowsActive;
long GravPertActive;
long ThrusterPlumesActive;
long ResidualDipoleActive;
long ContactActive;
long SloshActive;
long AlbedoActive; /* Affects CSS measurements */
long ComputeEnvTrq;
long EphemOption; /* MEAN or DE430 */

/* Calendar Time is all based in Terrestrial Dynamical Time (TT or TDT) unless
 * otherwise noted */
double DynTime0;     /* Time in sec since J2000 Epoch at Sim Start (TT) */
double DynTime;      /* Absolute Time (TT), sec since J2000 Epoch */
double AtomicTime;   /* TAI = TT - 32.184 sec, sec since J2000 */
double LeapSec;      /* Add to civil time (UTC) to synch with TAI */
double CivilTime;    /* UTC = TAI - LeapSec */
double GpsTime;      /* GPS Time = TAI - 19.0 sec */
struct DateType TT;  /* Terrestrial Dynamical Time */
struct DateType UTC; /* Universal Time Coordinated */
long GpsRollover, GpsWeek;
double GpsSecond;

/* Parameters for environmental models  */
long AtmoOption; /* TWOSIGMA_ATMO, NOMINAL_ATMO, USER_ATMO */
double Flux10p7, GeomagIndex;
double SchattenTable[5][410]; /* JD, +2sig F10.7, Nom F10.7, +2sig Kp, Nom Kp */

struct WorldType World[NWORLD];
struct LagrangeSystemType LagSys[3];

/* Galactic Coordinate Frame */
double CGH[3][3];

/* J2000 to Heliocentric Ecliptic */
double qJ2000H[4];

/* SC structure manages attitude and translation wrt Reference Orbit */
struct SCType *SC;
/* Orb structure manages Reference Orbits */
struct OrbitType *Orb;
/* Frm structure describes a Formation of S/C's */
struct FormationType *Frm;

/* Geom structure manages geometric objects, used for display and    */
/* for surface force computation (e.g. aerodynamic)                  */
struct GeomType *Geom;

struct TdrsType Tdrs[10];
struct GroundStationType *GroundStation;
long Ngnd;

/* Materials for graphics and solar pressure forces.                  */
struct MatlType *Matl;

/* Framebuffer Objects for Shadows and Surface Forces */
struct ShadowFBOType ShadowMap;
struct AlbedoFBOType AlbedoFBO;

/* Minor Bodies (Asteroids and Comets) */
long Nmb;

long Nrgn;
struct RegionType *Rgn;

/* Inter-Process Comm */
long Nipc;
struct IpcType *IPC;

/* Master Random Process */
struct RandomProcessType *RNG;
long RngSeed;

extern void InitOrbits(void);
extern void InitSpacecraft(struct SCType *S);
extern void LoadPlanets(void);
extern long LoadJplEphems(char EphemPath[80], double JD);
extern long LoadSpiceKernels(char SpicePath[80]);
extern long LoadSpiceEphems(double JS);
extern long DecodeString(char *s);
extern void InitFSW(struct SCType *S);
extern void InitAC(struct SCType *S);
extern void InitDSM(struct SCType *S);

void InterProcessComm(void);
void InitInterProcessComm(void);

extern void LoadSun(void);
extern void LoadPlanets(void);
extern void LoadMoonOfEarth(void);
extern void LoadMoonsOfMars(void);
extern void LoadMoonsOfJupiter(void);
extern void LoadMoonsOfSaturn(void);
extern void LoadMoonsOfUranus(void);
extern void LoadMoonsOfNeptune(void);
extern void LoadMoonsOfPluto(void);
extern void LoadMinorBodies(void);

extern void InitOrbit(struct OrbitType *);

extern void LoadRegions();
extern void InitLagrangePoints();
extern void OrbitMotion(double);
extern void LoadTdrs();
extern void LoadSchatten();
// #undef BUFSIZE
#endif
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
#include <format>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

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

#ifndef BIT_SHIFT
#define BIT_SHIFT(x, y) ((1U) << (((x) * 8U) + (y)))
#endif

#define BYTE_MASK(x) ((0xfU) << ((x) * 8U))

int main(int argc, char **argv);

class DataFilter {

   private:
   enum db_read_config_flags {
      // output condition flags
      DRCF_COND_VALID            = BIT_SHIFT(0, 0),
      DRCF_COND_INVALID          = BIT_SHIFT(0, 1),
      DRCF_COND_TIME_UNAMBIGUOUS = BIT_SHIFT(0, 2),
      DRCF_COND_TIME_AMBIGUOUS   = BIT_SHIFT(0, 3),
      DRCF_COND_ALL              = BYTE_MASK(0),

      // auxillary data output flags
      DRCF_OUT_VALID          = BIT_SHIFT(1, 0),
      DRCF_OUT_FILE           = BIT_SHIFT(1, 1),
      DRCF_OUT_DIR_TIME       = BIT_SHIFT(1, 2),
      DRCF_OUT_LIVE_PASS      = BIT_SHIFT(1, 3),
      DRCF_OUT_TIME_AMBIGUOUS = BIT_SHIFT(1, 4),
      DRCF_OUT_ALL            = BYTE_MASK(1),

      // sensor output flags
      DRCF_DATA_GYRO    = BIT_SHIFT(2, 0),
      DRCF_DATA_MAG     = BIT_SHIFT(2, 1),
      DRCF_DATA_CSS     = BIT_SHIFT(2, 2),
      DRCF_DATA_FSS     = BIT_SHIFT(2, 3),
      DRCF_DATA_ST      = BIT_SHIFT(2, 4),
      DRCF_DATA_GPS     = BIT_SHIFT(2, 5),
      DRCF_DATA_ACCEL   = BIT_SHIFT(2, 6),
      DRCF_DATA_SENSORS = BYTE_MASK(2),

      // actuator output flags
      DRCF_DATA_WHL = BIT_SHIFT(3, 0),
      DRCF_DATA_MTB = BIT_SHIFT(3, 1),
      DRCF_DATA_THR = BIT_SHIFT(3, 2),
      DRCF_DATA_ACT = BYTE_MASK(3),
   };

   public:
   DataFilter(int argc, char **argv);
   ~DataFilter();
   void read_db(struct SCType *S, struct DateType &date);

   private:
   static std::string DRCF2String(const enum db_read_config_flags flag)
   {
      switch (flag) {
         case (DRCF_COND_VALID):
            return "Condition Valid";
         case (DRCF_COND_INVALID):
            return "Condition Invalid";
         case (DRCF_COND_TIME_UNAMBIGUOUS):
            return "Condition Time Unambiguous";
         case (DRCF_COND_TIME_AMBIGUOUS):
            return "Condition Time Ambiguous";
         case (DRCF_COND_ALL):
            return "Condition All";
         case (DRCF_OUT_VALID):
            return "Out Valid";
         case (DRCF_OUT_FILE):
            return "Out File";
         case (DRCF_OUT_DIR_TIME):
            return "Out Directory Time";
         case (DRCF_OUT_LIVE_PASS):
            return "Out Live Pass";
         case (DRCF_OUT_TIME_AMBIGUOUS):
            return "Out Time Ambiguous";
         case (DRCF_OUT_ALL):
            return "Out All";
         case (DRCF_DATA_GYRO):
            return "Gyro";
         case (DRCF_DATA_MAG):
            return "MAG";
         case (DRCF_DATA_CSS):
            return "CSS";
         case (DRCF_DATA_FSS):
            return "FSS";
         case (DRCF_DATA_ST):
            return "ST";
         case (DRCF_DATA_GPS):
            return "GPS";
         case (DRCF_DATA_ACCEL):
            return "Accel";
         case (DRCF_DATA_SENSORS):
            return "Sensors";
         case (DRCF_DATA_WHL):
            return "Whl";
         case (DRCF_DATA_MTB):
            return "MTB";
         case (DRCF_DATA_THR):
            return "THR";
         case (DRCF_DATA_ACT):
            return "Actuators";
      }
   }

   static enum db_read_config_flags String2DRCF(const std::string string)
   {
      if (!string.compare("Condition Valid")) {
         return DRCF_COND_VALID;
      }
      else if (!string.compare("Condition Invalid")) {
         return DRCF_COND_INVALID;
      }
      else if (!string.compare("Condition Time Unambiguous")) {
         return DRCF_COND_TIME_UNAMBIGUOUS;
      }
      else if (!string.compare("Condition Time Ambiguous")) {
         return DRCF_COND_TIME_AMBIGUOUS;
      }
      else if (!string.compare("Condition All")) {
         return DRCF_COND_ALL;
      }
      else if (!string.compare("Out Valid")) {
         return DRCF_OUT_VALID;
      }
      else if (!string.compare("Out File")) {
         return DRCF_OUT_FILE;
      }
      else if (!string.compare("Out Directory Time")) {
         return DRCF_OUT_DIR_TIME;
      }
      else if (!string.compare("Out Live Pass")) {
         return DRCF_OUT_LIVE_PASS;
      }
      else if (!string.compare("Out Time Ambiguous")) {
         return DRCF_OUT_TIME_AMBIGUOUS;
      }
      else if (!string.compare("Out All")) {
         return DRCF_OUT_ALL;
      }
      else if (!string.compare("Gyro")) {
         return DRCF_DATA_GYRO;
      }
      else if (!string.compare("MAG")) {
         return DRCF_DATA_MAG;
      }
      else if (!string.compare("CSS")) {
         return DRCF_DATA_CSS;
      }
      else if (!string.compare("FSS")) {
         return DRCF_DATA_FSS;
      }
      else if (!string.compare("ST")) {
         return DRCF_DATA_ST;
      }
      else if (!string.compare("GPS")) {
         return DRCF_DATA_GPS;
      }
      else if (!string.compare("Accel")) {
         return DRCF_DATA_ACCEL;
      }
      else if (!string.compare("Sensors")) {
         return DRCF_DATA_SENSORS;
      }
      else if (!string.compare("Whl")) {
         return DRCF_DATA_WHL;
      }
      else if (!string.compare("MTB")) {
         return DRCF_DATA_MTB;
      }
      else if (!string.compare("THR")) {
         return DRCF_DATA_THR;
      }
      else if (!string.compare("Actuators")) {
         return DRCF_DATA_ACT;
      }
   }

   template <typename T> bool contains(const T arr[], const T item)
   {
      auto end = std::end(arr);
      T *found = std::find(std::begin(arr), end, item);
      return (found != end);
   };

   fy_document *config_doc;
   std::string db_name;
   struct DateType db_time;

   void SelectStatementFromMapping(struct fy_node *map_node,
                                   std::string &select_statement);
   std::string ConstructSQLStatement(struct fy_node *config_root,
                                     const long flags,
                                     const ccsdsCoarse ccsds_coarse_min,
                                     const ccsdsFine ccsds_fine_min,
                                     const ccsdsCoarse ccsds_coarse_max,
                                     const ccsdsFine ccsds_fine_max);
   void readData(struct SCType *S, sqlite3 *db, struct fy_node *data_node,
                 const long flags, const struct DateType &min_date,
                 const struct DateType &max_date);
};

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

/* Calendar Time is all based in Terrestrial Dynamical Time (TT or TDT)
 * unless otherwise noted */
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

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

// #undef BUFSIZE
#endif
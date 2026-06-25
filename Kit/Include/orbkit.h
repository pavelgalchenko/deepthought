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

#include "iokit.h"
#include "jdkit.h"
#include "mathkit.h"

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

/* Use X-Macros to define the World List, and some other associated   */
/*    Parameters.                                                     */
/*    Format:                                                         */
/*    X(                                                              */
/*       world, ---- Identifier for the enum                          */
/*       naif_str, - NAIF string representation                       */
/*       parent, --- Parent of the world                              */
/*    )                                                               */
#define X_WORLD_LIST                                                           \
   X(SOL, "SOL", "SUN", (-1))                                                  \
   X(MERCURY, "MERCURY", "MERCURY", SOL)                                       \
   X(VENUS, "VENUS", "VENUS", SOL)                                             \
   X(EARTH, "EARTH", "EARTH", SOL)                                             \
   X(MARS, "MARS", "MARS", SOL)                                                \
   X(JUPITER, "JUPITER", "JUPITER", SOL)                                       \
   X(SATURN, "SATURN", "SATURN", SOL)                                          \
   X(URANUS, "URANUS", "URANUS", SOL)                                          \
   X(NEPTUNE, "NEPTUNE", "NEPTUNE", SOL)                                       \
   X(PLUTO, "PLUTO", "PLUTO", SOL)                                             \
   /* Moon of Earth */                                                         \
   X(LUNA, "LUNA", "MOON", EARTH)                                              \
   /* Moons of Mars */                                                         \
   X(PHOBOS, "PHOBOS", "PHOBOS", MARS)                                         \
   X(DEIMOS, "DEIMOS", "DEIMOS", MARS)                                         \
   /* Major Moons of Jupiter */                                                \
   X(IO, "IO", "IO", JUPITER)                                                  \
   X(EUROPA, "EUROPA", "EUROPA", JUPITER)                                      \
   X(GANYMEDE, "GANYMEDE", "GANYMEDE", JUPITER)                                \
   X(CALLISTO, "CALLISTO", "CALLISTO", JUPITER)                                \
   X(AMALTHEA, "AMALTHEA", "AMALTHEA", JUPITER)                                \
   X(HIMALIA, "HIMALIA", "HIMALIA", JUPITER)                                   \
   X(ELARA, "ELARA", "ELARA", JUPITER)                                         \
   X(PASIPHAE, "PASIPHAE", "PASIPHAE", JUPITER)                                \
   X(SINOPE, "SINOPE", "SINOPE", JUPITER)                                      \
   X(LYSITHEA, "LYSITHEA", "LYSITHEA", JUPITER)                                \
   X(CARME, "CARME", "CARME", JUPITER)                                         \
   X(ANANKE, "ANANKE", "ANANKE", JUPITER)                                      \
   X(LEDA, "LEDA", "LEDA", JUPITER)                                            \
   X(THEBE, "THEBE", "THEBE", JUPITER)                                         \
   X(ADRASTEA, "ADRASTEA", "ADRASTEA", JUPITER)                                \
   X(METIS, "METIS", "METIS", JUPITER)                                         \
   /* Major Moons of Saturn */                                                 \
   X(MIMAS, "MIMAS", "MIMAS", SATURN)                                          \
   X(ENCELADUS, "ENCELADUS", "ENCELADUS", SATURN)                              \
   X(TETHYS, "TETHYS", "TETHYS", SATURN)                                       \
   X(DIONE, "DIONE", "DIONE", SATURN)                                          \
   X(RHEA, "RHEA", "RHEA", SATURN)                                             \
   X(TITAN, "TITAN", "TITAN", SATURN)                                          \
   X(HYPERION, "HYPERION", "HYPERION", SATURN)                                 \
   X(IAPETUS, "IAPETUS", "IAPETUS", SATURN)                                    \
   X(PHOEBE, "PHOEBE", "PHOEBE", SATURN)                                       \
   X(JANUS, "JANUS", "JANUS", SATURN)                                          \
   X(EPIMETHEUS, "EPIMETHEUS", "EPIMETHEUS", SATURN)                           \
   X(HELENE, "HELENE", "HELENE", SATURN)                                       \
   X(TELESTO, "TELESTO", "TELESTO", SATURN)                                    \
   X(CALYPSO, "CALYPSO", "CALYPSO", SATURN)                                    \
   X(ATLAS, "ATLAS", "ATLAS", SATURN)                                          \
   X(PROMETHEUS, "PROMETHEUS", "PROMETHEUS", SATURN)                           \
   X(PANDORA, "PANDORA", "PANDORA", SATURN)                                    \
   X(PAN, "PAN", "PAN", SATURN)                                                \
   /* Major Moons of Uranus */                                                 \
   X(ARIEL, "ARIEL", "ARIEL", URANUS)                                          \
   X(UMBRIEL, "UMBRIEL", "UMBRIEL", URANUS)                                    \
   X(TITANIA, "TITANIA", "TITANIA", URANUS)                                    \
   X(OBERON, "OBERON", "OBERON", URANUS)                                       \
   X(MIRANDA, "MIRANDA", "MIRANDA", URANUS)                                    \
   /* Major Moons of Neptune */                                                \
   X(TRITON, "TRITON", "TRITON", NEPTUNE)                                      \
   X(NEREID, "NEREID", "NEREID", NEPTUNE)                                      \
   /* Pluto's moon */                                                          \
   X(CHARON, "CHARON", "CHARON", PLUTO)

#define WORLD_CONFIGURE_SATELLITES(worlds, w_id, n_sat, sat_list)              \
   do {                                                                        \
      (n_sat)    = 0;                                                          \
      (sat_list) = NULL;                                                       \
      for (WorldID Iw = SOL; Iw < NMAJORWORLD; Iw++)                           \
         if (GetWorldParent(Iw) == w_id &&                                     \
             (w_id == SOL || (worlds)[Iw].Exists))                             \
            (n_sat)++;                                                         \
                                                                               \
      if ((n_sat) > 0) {                                                       \
         (sat_list) = calloc((n_sat), sizeof(long));                           \
         if ((sat_list) == NULL) {                                             \
            fprintf(                                                           \
                stderr,                                                        \
                "World[%i].Sat calloc returned null pointer . Exiting...\n",   \
                (w_id));                                                       \
            exit(EXIT_FAILURE);                                                \
         }                                                                     \
                                                                               \
         long i_sat = 0;                                                       \
         for (WorldID Iw = SOL; Iw < NMAJORWORLD; Iw++) {                      \
            if (GetWorldParent(Iw) == w_id &&                                  \
                (w_id == SOL || (worlds)[Iw].Exists)) {                        \
               sat_list[i_sat] = Iw;                                           \
               i_sat++;                                                        \
            }                                                                  \
         }                                                                     \
      }                                                                        \
   } while (0)

/* World Tags */
typedef enum WorldID {
   // TODO: maybe organize the moons to immediately follow the parent planet?
   // for example, the moons of Jupiter would be from
   // World[JUPITER+1] to World[SATURN-1]
   // this would make iterating over just the planets more difficult...

   NULL_WORLD = -1,
#define X(world, str_val, naif, parent) world,
   X_WORLD_LIST
#undef X
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
   vec3_t PosN; /* Pos wrt N frame of Body 1 (larger grav center), [[m]] */
   vec3_t VelN; /* Vel wrt N frame of Body 1 (larger grav center), [[m/sec]] */
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
   mat3x3_t CLH;
   mat3x3_t CLN;
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
   vec3_t N_BODY_PosN; /* SC Position from TRV file, [[m]], expressed in N of
                             CENTRAL body */
   vec3_t N_BODY_VelN; /* SC Velocity from TRV file, [[m/sec]], expressed in
                             N of CENTRAL body */

   /* For Central Orbit Description */
   double MeanAnom;
   double anom;  /* True Anomaly, rad */
   vec3_t PosN;  /* Position, [[m]], expressed in N [~=~] */
   vec3_t VelN;  /* Velocity, [[m/sec]], expressed in N [~=~] */
   mat3x3_t CLN; /* For ZERO, L = N.  For FLIGHT, L = ENU.  For CENTRAL, L =
                        LVLH.  For THREE_BODY, L = XYZ */
   vec3_t wln;   /* Expressed in N */
   /* Fit spline to data file */
   long SplineActive;
   double NodeDynTime[4]; /* Sec since J2000 (TT) */
   vec3_t NodePos[4];
   vec3_t NodeVel[4];
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
   double r_ref;
   double factor; // -mu for gravity
   double **Norm;
   double **Norm1;
   double **Norm2;
   double **A;
   double **C;
   double **S;
   double *Re;
   double *Im;

   double **VR01;
   double **VR11;
   double **VR02;
   double **VR12;
   double **VR22;
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
   double RadOfInfluence; /* Radius of Sphere of Influence */
   double DipoleMoment;   /* Magnetic Field Dipole Moment, Wb-m */
   vec3_t DipoleAxis;     /* Magnetic Field Dipole Axis */
   vec3_t DipoleOffset;   /* Dipole Offset, m */
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
   mat3x3_t CNH;     /* DCM from heliocentric ecliptic frame
                            to world-centric equatorial inertial frame */
   quat_t qnh;       /* ~*/
   mat3x3_t CNJ;     /* DCM from J2000 frame to world-centric equatorial
                            inertial frame */
   quat_t qnj;

   /*~ Internal Variables ~*/
   /* contains information defining prime meridian angle information */
   /*    order: Prime Meridian, Right Ascension, Declination         */
   AngDataType ang_data[3];

   vec3_t PosH; /* Position in H frame [~=~] */
   vec3_t VelH; /* Velocity in H frame */
   /* Angle from N1 to prime meridian (AST angled for Earth) */
   double PriMerAng;
   mat3x3_t CWN;    /* DCM from world-centric inertial frame
                           to world-centric rotating frame */
   quat_t qwn;      /* ~*/
   long Visibility; /* Too small to see, point-sized, or shows disk */
   float ModelMatrix[16];

   /*~ Structures ~*/
   struct OrbitType eph; /* Ephemeris */
   struct AtmoType Atmo;
};

/*~ Prototypes ~*/
__attribute__((pure)) WorldID GetWorldID(const char *s);
__attribute__((pure)) const char *WorldID2String(WorldID w_id);
__attribute__((const)) WorldID GetWorldParent(const WorldID w_id);

void CloneWorld(struct WorldType *const destWorld,
                const struct WorldType srcWorld);
void CopyWorld(struct WorldType *const destWorld,
               const struct WorldType srcWorld);
__attribute__((pure)) double GetWorldW(JDType jd,
                                       const struct WorldType *const world);
__attribute__((pure)) vec3_t GetWorldWln(JDType jd,
                                         const struct WorldType *const world);
__attribute__((const)) AngDataType CopyAngData(const AngDataType src);
__attribute__((pure)) double GetWorldAng(JDType jd,
                                         const AngDataType *const ang_data);
__attribute__((pure)) pair_dbl_mat3x3_t
GetWorldCWN(JDType jd, const AngDataType *const ang_data);
__attribute__((pure)) mat3x3_t GetWorldCNJ(JDType jd,
                                           const AngDataType *const ang_data);

void CloneOrbit(struct OrbitType *const destOrb, const struct OrbitType srcOrb);
void CopyOrbit(struct OrbitType *const destOrb, const struct OrbitType srcOrb);
__attribute__((const)) double MeanAnomToTrueAnom(double MeanAnom, double ecc);
__attribute__((const)) double TrueAnomaly(double mu, double p, double e,
                                          double t);
__attribute__((const)) double atanh(double x);
__attribute__((const)) double TimeSincePeriapsis(double mu, double p, double e,
                                                 double th);
void RV02RV(double mu, vec3_t xr0, vec3_t xv0, double anom, vec3_t *xr,
            vec3_t *xv);
void Eph2RV(double mu, double p, double e, double i, double RAAN, double ArgP,
            double dt, vec3_t *r, vec3_t *v, double *anom);
void RV2Eph(double time, double mu, vec3_t xr, vec3_t xv, double *SMA,
            double *e, double *i, double *RAAN, double *ArgP, double *th,
            double *tp, double *SLR, double *alpha, double *rmin,
            double *MeanMotion, double *Period);
void TLE2MeanEph(const char Line1[80], const char Line2[80], JDType jd,
                 struct OrbitType *O);
void MeanEph2RV(struct OrbitType *O, double DynTime);
long LoadTleFromFile(const char *Path, const char *TleFileName,
                     const char *TleLabel, double DynTime, JDType jd,
                     struct OrbitType *O);
double RV2RVp(double mu, vec3_t r, vec3_t v, vec3_t *rp, vec3_t *vp);
void PlanetEphemerides(long i, JDType jd, double mu, double *SMA, double *ecc,
                       double *inc, double *RAAN, double *omg, double *tp,
                       double *anom, double *p, double *alpha, double *rmin,
                       double *MeanMotion, double *Period);
vec3_t LunaPosition(const JDType jd);
int LoadLunaInertialFrameData(AngDataType *const ang_data);
mat3x3_t LunaInertialFrame(const JDType jd);
int LoadLunaPriMerAngData(AngDataType *const ang_data);
__attribute__((const, deprecated)) double LunaPriMerAng(JDType JulDay);
void FindCLN(vec3_t r, vec3_t v, mat3x3_t *CLN, vec3_t *wln);
__attribute__((const)) mat3x3_t FindCEN(vec3_t r);
void FindENU(vec3_t PosN, double WorldW, mat3x3_t *CLN, vec3_t *wln);
void FindLagPtParms(struct LagrangeSystemType *LS);
void FindLagPtPosVel(double SecSinceJ2000, struct LagrangeSystemType *S,
                     long Ilp, vec3_t *PosN, vec3_t *VelN, mat3x3_t *CLN);
void LagModes2RV(double SecSinceJ2000, struct LagrangeSystemType *LS,
                 struct OrbitType *O, vec3_t *r, vec3_t *v);
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
void TDRSPosVel(double PriMerAng, double TIME, vec3_t ptn[10], vec3_t vtn[10]);
__attribute__((const)) mat3x3_t TETE2J2000(double JD);
__attribute__((const)) double RadiusOfInfluence(double mu1, double mu2,
                                                double r);
__attribute__((const)) pair_vec3_t RelRV2EHRV(double OrbRadius, double OrbRate,
                                              mat3x3_t OrbCLN, vec3_t Rrel,
                                              vec3_t Vrel);
__attribute__((const)) pair_vec3_t EHRV2RelRV(double OrbRadius, double OrbRate,
                                              mat3x3_t OrbCLN, vec3_t re,
                                              vec3_t ve);
void EHRV2EHModes(vec3_t r, vec3_t v, double n, double nt, double *A,
                  double *Bc, double *Bs, double *C, double *Dc, double *Ds);
void EHModes2EHRV(double A, double Bc, double Bs, double C, double Dc,
                  double Ds, double n, double nt, vec3_t *const r,
                  vec3_t *const v);
__attribute__((const)) double LambertTOF(double mu, double amin, double lambda,
                                         double x);
void LambertProblem(double t0, double mu, vec3_t xr1, vec3_t xr2, double TOF,
                    double TransferType, double *SLR, double *e, double *inc,
                    double *RAAN, double *ArgP, double *tp);
double RendezvousCostFunction(double *InVec, double *AuxVec);
void PlanTwoImpulseRendezvous(double mu, vec3_t r1e, vec3_t v1e, vec3_t r2e,
                              vec3_t v2e, double *t1, double *t2, vec3_t DV1,
                              vec3_t DV2);
void FindLightLagOffsets(double DynTime, struct OrbitType *Observer,
                         struct OrbitType *Target, vec3_t PastPos,
                         vec3_t FuturePos);
void OscEphToMeanEph(double mu, double J2, double Rw, JDType jd,
                     struct OrbitType *O);
void MeanEphToOscEph(struct OrbitType *O, double DynTime);

void StateRnd2StateN(struct LagrangeSystemType *LS, vec3_t W2_pos,
                     vec3_t W2_vel, vec3_t R_R_nd, vec3_t V_R_nd, vec3_t *R_N,
                     vec3_t *V_N);
void StateN2StateRnd(struct LagrangeSystemType *LS, vec3_t W2_pos,
                     vec3_t W2_vel, vec3_t R_N, vec3_t V_N, vec3_t *R_R_nd,
                     vec3_t *V_R_nd);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __ORBKIT_H__ */

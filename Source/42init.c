/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "42.h"
#include <ctype.h>
#include <glob.h>
#include <stdint.h>
#include <unistd.h>

#if defined __MINGW32__
#include <Windows.h>
#elif defined _WIN32
#include <Windows.h>
#elif defined _WIN64
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

#ifdef _ENABLE_SPICE_
#include "SpiceUsr.h"
#endif

/* #ifdef __cplusplus
** namespace _42 {
** using namespace Kit;
** #endif
*/

/**********************************************************************/
long DecodeString(char *s)
{

   unsigned long i;

   for (i = 0; i < strlen(s); i++)
      s[i] = toupper(s[i]);

   if (!strcmp(s, "FALSE"))
      return FALSE;
   else if (!strcmp(s, "TRUE"))
      return TRUE;
   else if (!strcmp(s, "OFF"))
      return OFF;
   else if (!strcmp(s, "ON"))
      return ON;
   else if (!strcmp(s, "FAST"))
      return FAST_TIME;
   else if (!strcmp(s, "REAL"))
      return REAL_TIME;
   else if (!strcmp(s, "EXTERNAL"))
      return EXTERNAL_TIME;
   else if (!strcmp(s, "NOS3"))
      return NOS3_TIME;
   else if (!strcmp(s, "KEP"))
      return INP_KEPLER;
   else if (!strcmp(s, "RV"))
      return INP_POSVEL;
   else if (!strcmp(s, "FILE"))
      return INP_FILE;
   else if (!strcmp(s, "TLE"))
      return INP_TLE;
   else if (!strcmp(s, "TRV"))
      return INP_TRV;
   else if (!strcmp(s, "PA"))
      return TRUE;
   else if (!strcmp(s, "AE"))
      return FALSE;
   else if (!strcmp(s, "POSW"))
      return TRUE;
   else if (!strcmp(s, "LLA"))
      return FALSE;
   else if (!strcmp(s, "POS"))
      return POSITIVE;
   else if (!strcmp(s, "NEG"))
      return NEGATIVE;
   else if (!strcmp(s, "CM"))
      return TRUE;
   else if (!strcmp(s, "ORIGIN"))
      return FALSE;
   else if (!strcmp(s, "CENTRAL"))
      return ORB_CENTRAL;
   else if (!strcmp(s, "THREE_BODY"))
      return ORB_THREE_BODY;
   else if (!strcmp(s, "N_BODY"))
      return ORB_N_BODY;
   else if (!strcmp(s, "ZERO"))
      return ORB_ZERO;
   else if (!strcmp(s, "FLIGHT"))
      return ORB_FLIGHT;
   else if (!strcmp(s, "EARTHMOON"))
      return EARTHMOON;
   else if (!strcmp(s, "SUNEARTH"))
      return SUNEARTH;
   else if (!strcmp(s, "SUNJUPITER"))
      return SUNJUPITER;
   else if (!strcmp(s, "MODES"))
      return INP_MODES;
   else if (!strcmp(s, "XYZ"))
      return INP_XYZ;
   else if (!strcmp(s, "XYZ_ROT"))
      return INP_XYZ_ROT;
   else if (!strcmp(s, "SPLINE"))
      return INP_SPLINE;
   else if (!strcmp(s, "L1"))
      return LAGPT_L1;
   else if (!strcmp(s, "L2"))
      return LAGPT_L2;
   else if (!strcmp(s, "L3"))
      return LAGPT_L3;
   else if (!strcmp(s, "L4"))
      return LAGPT_L4;
   else if (!strcmp(s, "L5"))
      return LAGPT_L5;
   else if (!strcmp(s, "CW"))
      return DIR_CW;
   else if (!strcmp(s, "CCW"))
      return DIR_CCW;
   else if (!strcmp(s, "NONE"))
      return NONE;
   else if (!strcmp(s, "DIPOLE"))
      return DIPOLE;
   else if (!strcmp(s, "IGRF"))
      return IGRF;
   else if (!strcmp(s, "SPHERICAL"))
      return 1;
   else if (!strcmp(s, "GIMBAL"))
      return 0;
   else if (!strcmp(s, "X_AXIS"))
      return X_AXIS;
   else if (!strcmp(s, "Y_AXIS"))
      return Y_AXIS;
   else if (!strcmp(s, "Z_AXIS"))
      return Z_AXIS;
   else if (!strcmp(s, "POS_X"))
      return POS_X;
   else if (!strcmp(s, "POS_Y"))
      return POS_Y;
   else if (!strcmp(s, "POS_Z"))
      return POS_Z;
   else if (!strcmp(s, "NEG_X"))
      return NEG_X;
   else if (!strcmp(s, "NEG_Y"))
      return NEG_Y;
   else if (!strcmp(s, "NEG_Z"))
      return NEG_Z;
   else if (!strcmp(s, "REFORB"))
      return TARGET_REFORB;
   else if (!strcmp(s, "FRM"))
      return TARGET_FRM;
   else if (!strcmp(s, "SC"))
      return TARGET_SC;
   else if (!strcmp(s, "BODY"))
      return TARGET_BODY;

   else if (!strcmp(s, "TRACK_HOST"))
      return TRACK_HOST;
   else if (!strcmp(s, "TRACK_TARGET"))
      return TRACK_TARGET;
   else if (!strcmp(s, "FIXED_IN_HOST"))
      return FIXED_IN_HOST;

   else if (!strcmp(s, "DOWN"))
      return VIEW_DOWN;
   else if (!strcmp(s, "REAR_LEFT"))
      return VIEW_REAR_LEFT;
   else if (!strcmp(s, "REAR"))
      return VIEW_REAR;
   else if (!strcmp(s, "REAR_RIGHT"))
      return VIEW_REAR_RIGHT;
   else if (!strcmp(s, "LEFT"))
      return VIEW_LEFT;
   else if (!strcmp(s, "UP"))
      return VIEW_UP;
   else if (!strcmp(s, "RIGHT"))
      return VIEW_RIGHT;
   else if (!strcmp(s, "FRONT_LEFT"))
      return VIEW_FRONT_LEFT;
   else if (!strcmp(s, "FRONT"))
      return VIEW_FRONT;
   else if (!strcmp(s, "FRONT_RIGHT"))
      return VIEW_FRONT_RIGHT;

   else if (!strcmp(s, "WIREFRAME"))
      return FOV_WIREFRAME;
   else if (!strcmp(s, "SOLID"))
      return FOV_SOLID;
   else if (!strcmp(s, "VECTOR"))
      return FOV_VECTOR;
   else if (!strcmp(s, "PLANE"))
      return FOV_PLANE;

   else if (!strcmp(s, "REFPT_CM"))
      return REFPT_CM;
   else if (!strcmp(s, "REFPT_JOINT"))
      return REFPT_JOINT;

   else if (!strcmp(s, "LAGDOF_MODES"))
      return LAGDOF_MODES;
   else if (!strcmp(s, "LAGDOF_COWELL"))
      return LAGDOF_COWELL;
   else if (!strcmp(s, "LAGDOF_SPLINE"))
      return LAGDOF_SPLINE;

   else if (!strcmp(s, "GAUSS_ELIM"))
      return DYN_GAUSS_ELIM;
   else if (!strcmp(s, "ORDER_N"))
      return DYN_ORDER_N;

   else if (!strcmp(s, "FIXED"))
      return ORBDOF_FIXED;
   else if (!strcmp(s, "EULER_HILL"))
      return ORBDOF_EULER_HILL;
   else if (!strcmp(s, "ENCKE"))
      return ORBDOF_ENCKE;
   else if (!strcmp(s, "COWELL"))
      return ORBDOF_COWELL;

   else if (!strcmp(s, "PASSIVE_FSW"))
      return PASSIVE_FSW;
   else if (!strcmp(s, "PROTOTYPE_FSW"))
      return PROTOTYPE_FSW;
   else if (!strcmp(s, "AD_HOC_FSW"))
      return AD_HOC_FSW;
   else if (!strcmp(s, "SPINNER_FSW"))
      return SPINNER_FSW;
   else if (!strcmp(s, "MOMBIAS_FSW"))
      return MOMBIAS_FSW;
   else if (!strcmp(s, "THREE_AXIS_FSW"))
      return THREE_AXIS_FSW;
   else if (!strcmp(s, "ISS_FSW"))
      return ISS_FSW;
   else if (!strcmp(s, "CMG_FSW"))
      return CMG_FSW;
   else if (!strcmp(s, "THR_FSW"))
      return THR_FSW;
   else if (!strcmp(s, "CFS_FSW"))
      return CFS_FSW;
   else if (!strcmp(s, "RBT_FSW"))
      return RBT_FSW;
   else if (!strcmp(s, "DSM_FSW"))
      return DSM_FSW;

   else if (!strcmp(s, "SUN"))
      return SUN;
   else if (!strcmp(s, "PLANET"))
      return PLANET;
   else if (!strcmp(s, "MOON"))
      return MOON;
   else if (!strcmp(s, "ASTEROID"))
      return ASTEROID;
   else if (!strcmp(s, "COMET"))
      return COMET;

   else if (!strcmp(s, "SIDE"))
      return VIEW_SIDE;
   else if (!strcmp(s, "TOP"))
      return VIEW_TOP;
   else if (!strcmp(s, "TWOSIGMA"))
      return TWOSIGMA_ATMO;
   else if (!strcmp(s, "NOMINAL"))
      return NOMINAL_ATMO;
   else if (!strcmp(s, "USER"))
      return USER_ATMO;
   else if (!strcmp(s, "TX"))
      return IPC_TX;
   else if (!strcmp(s, "RX"))
      return IPC_RX;
   else if (!strcmp(s, "TXRX"))
      return IPC_TXRX;
   else if (!strcmp(s, "ACS"))
      return IPC_ACS;
   else if (!strcmp(s, "WRITEFILE"))
      return IPC_WRITEFILE;
   else if (!strcmp(s, "READFILE"))
      return IPC_READFILE;
   else if (!strcmp(s, "SPIRENT"))
      return IPC_SPIRENT;
   else if (!strcmp(s, "FFTB"))
      return IPC_FFTB;
   else if (!strcmp(s, "SERVER"))
      return IPC_SERVER;
   else if (!strcmp(s, "CLIENT"))
      return IPC_CLIENT;
   else if (!strcmp(s, "GMSEC_CLIENT"))
      return IPC_GMSEC_CLIENT;

   else if (!strcmp(s, "MAJOR"))
      return MAJOR_CONSTELL;
   else if (!strcmp(s, "ZODIAC"))
      return ZODIAC_CONSTELL;
   else if (!strcmp(s, "MINOR"))
      return MINOR_CONSTELL;

   else if (!strcmp(s, "PASSIVE"))
      return PASSIVE_JOINT;
   else if (!strcmp(s, "ACTUATED"))
      return ACTUATED_JOINT;
   else if (!strcmp(s, "STEPPER_MOTOR"))
      return STEPPER_MOTOR_JOINT;
   else if (!strcmp(s, "TVC_JOINT"))
      return TVC_JOINT;
   else if (!strcmp(s, "VIBRATION_ISOLATOR"))
      return VIBRATION_ISOLATOR_JOINT;
   else if (!strcmp(s, "SLOSH"))
      return SLOSH_JOINT;
   else if (!strcmp(s, "STEERING_MIRROR"))
      return STEERING_MIRROR_JOINT;
   else if (!strcmp(s, "AD_HOC_JOINT"))
      return AD_HOC_JOINT;

   else if (!strcmp(s, "FORCE"))
      return FORCE;
   else if (!strcmp(s, "TORQUE"))
      return TORQUE;

   else if (!strcmp(s, "PULSED"))
      return THR_PULSED;
   else if (!strcmp(s, "PROPORTIONAL"))
      return THR_PROPORTIONAL;

   else if (!strcmp(s, "CONVENTIONAL"))
      return CONVENTIONAL_FSS;
   else if (!strcmp(s, "GSFSS"))
      return GS_FSS;

   else if (!strcmp(s, "APERTURE"))
      return OPT_APERTURE;
   else if (!strcmp(s, "CONIC"))
      return OPT_CONIC;
   else if (!strcmp(s, "DETECTOR"))
      return OPT_DETECTOR;
   else if (!strcmp(s, "THINLENS"))
      return OPT_THINLENS;

   else if (!strcmp(s, "CONCAVE"))
      return OPT_CONCAVE;
   else if (!strcmp(s, "CONVEX"))
      return OPT_CONVEX;

   else {
      fprintf(stderr, "Bogus input %s in DecodeString (42init.c:%d)\n", s,
              __LINE__);
      exit(EXIT_FAILURE);
   }
}
/**********************************************************************/
WorldID GetWorldID(const char *s)
{
   unsigned long i;
   if (!strcmp(s, "SOL") || !strcmp(s, "SUN"))
      return SOL;
   else if (!strcmp(s, "MERCURY"))
      return MERCURY;
   else if (!strcmp(s, "VENUS"))
      return VENUS;
   else if (!strcmp(s, "EARTH"))
      return EARTH;
   else if (!strcmp(s, "MARS"))
      return MARS;
   else if (!strcmp(s, "JUPITER"))
      return JUPITER;
   else if (!strcmp(s, "SATURN"))
      return SATURN;
   else if (!strcmp(s, "URANUS"))
      return URANUS;
   else if (!strcmp(s, "NEPTUNE"))
      return NEPTUNE;
   else if (!strcmp(s, "PLUTO"))
      return PLUTO;
   else if (!strcmp(s, "LUNA"))
      return LUNA;
   else if (!strcmp(s, "PHOBOS"))
      return PHOBOS;
   else if (!strcmp(s, "DEIMOS"))
      return DEIMOS;
   else if (!strcmp(s, "IO"))
      return IO;
   else if (!strcmp(s, "EUROPA"))
      return EUROPA;
   else if (!strcmp(s, "GANYMEDE"))
      return GANYMEDE;
   else if (!strcmp(s, "CALLISTO"))
      return CALLISTO;
   else if (!strcmp(s, "AMALTHEA"))
      return AMALTHEA;
   else if (!strcmp(s, "HIMALIA"))
      return HIMALIA;
   else if (!strcmp(s, "ELARA"))
      return ELARA;
   else if (!strcmp(s, "PASIPHAE"))
      return PASIPHAE;
   else if (!strcmp(s, "SINOPE"))
      return SINOPE;
   else if (!strcmp(s, "LYSITHEA"))
      return LYSITHEA;
   else if (!strcmp(s, "CARME"))
      return CARME;
   else if (!strcmp(s, "ANANKE"))
      return ANANKE;
   else if (!strcmp(s, "LEDA"))
      return LEDA;
   else if (!strcmp(s, "THEBE"))
      return THEBE;
   else if (!strcmp(s, "ADRASTEA"))
      return ADRASTEA;
   else if (!strcmp(s, "METIS"))
      return METIS;
   else if (!strcmp(s, "MIMAS"))
      return MIMAS;
   else if (!strcmp(s, "ENCELADUS"))
      return ENCELADUS;
   else if (!strcmp(s, "TETHYS"))
      return TETHYS;
   else if (!strcmp(s, "DIONE"))
      return DIONE;
   else if (!strcmp(s, "RHEA"))
      return RHEA;
   else if (!strcmp(s, "TITAN"))
      return TITAN;
   else if (!strcmp(s, "HYPERION"))
      return HYPERION;
   else if (!strcmp(s, "IAPETUS"))
      return IAPETUS;
   else if (!strcmp(s, "PHOEBE"))
      return PHOEBE;
   else if (!strcmp(s, "JANUS"))
      return JANUS;
   else if (!strcmp(s, "EPIMETHEUS"))
      return EPIMETHEUS;
   else if (!strcmp(s, "HELENE"))
      return HELENE;
   else if (!strcmp(s, "TELESTO"))
      return TELESTO;
   else if (!strcmp(s, "CALYPSO"))
      return CALYPSO;
   else if (!strcmp(s, "ATLAS"))
      return ATLAS;
   else if (!strcmp(s, "PROMETHEUS"))
      return PROMETHEUS;
   else if (!strcmp(s, "PANDORA"))
      return PANDORA;
   else if (!strcmp(s, "PAN"))
      return PAN;
   else if (!strcmp(s, "ARIEL"))
      return ARIEL;
   else if (!strcmp(s, "UMBRIEL"))
      return UMBRIEL;
   else if (!strcmp(s, "TITANIA"))
      return TITANIA;
   else if (!strcmp(s, "OBERON"))
      return OBERON;
   else if (!strcmp(s, "MIRANDA"))
      return MIRANDA;
   else if (!strcmp(s, "TRITON"))
      return TRITON;
   else if (!strcmp(s, "NEREID"))
      return NEREID;
   else if (!strcmp(s, "CHARON"))
      return CHARON;
   else if (sscanf(s, "MINORBODY_%lu", &i) == 1)
      return (MINORBODY_0 + i);
   fprintf(stderr, "Bogus input %s in GetWorldID (42init.c:%d)\n", s, __LINE__);
   exit(EXIT_FAILURE);
}
/**********************************************************************/
void WorldID2String(WorldID w_id, char w_str[32])
{
   // Returns the NAIF names of the celestial bodies
   switch (w_id) {
      case SOL:
         strcpy(w_str, "SUN");
         break;
      case MERCURY:
         strcpy(w_str, "MERCURY");
         break;
      case VENUS:
         strcpy(w_str, "VENUS");
         break;
      case EARTH:
         strcpy(w_str, "EARTH");
         break;
      case MARS:
         strcpy(w_str, "MARS");
         break;
      case JUPITER:
         strcpy(w_str, "JUPITER");
         break;
      case SATURN:
         strcpy(w_str, "SATURN");
         break;
      case URANUS:
         strcpy(w_str, "URANUS");
         break;
      case NEPTUNE:
         strcpy(w_str, "NEPTUNE");
         break;
      case PLUTO:
         strcpy(w_str, "PLUTO");
         break;
      case LUNA:
         strcpy(w_str, "LUNA");
         break;
      case PHOBOS:
         strcpy(w_str, "PHOBOS");
         break;
      case DEIMOS:
         strcpy(w_str, "DEIMOS");
         break;
      case IO:
         strcpy(w_str, "IO");
         break;
      case EUROPA:
         strcpy(w_str, "EUROPA");
         break;
      case GANYMEDE:
         strcpy(w_str, "GANYMEDE");
         break;
      case CALLISTO:
         strcpy(w_str, "CALLISTO");
         break;
      case AMALTHEA:
         strcpy(w_str, "AMALTHEA");
         break;
      case HIMALIA:
         strcpy(w_str, "HIMALIA");
         break;
      case ELARA:
         strcpy(w_str, "ELARA");
         break;
      case PASIPHAE:
         strcpy(w_str, "PASIPHAE");
         break;
      case SINOPE:
         strcpy(w_str, "SINOPE");
         break;
      case LYSITHEA:
         strcpy(w_str, "LYSITHEA");
         break;
      case CARME:
         strcpy(w_str, "CARME");
         break;
      case ANANKE:
         strcpy(w_str, "ANANKE");
         break;
      case LEDA:
         strcpy(w_str, "LEDA");
         break;
      case THEBE:
         strcpy(w_str, "THEBE");
         break;
      case ADRASTEA:
         strcpy(w_str, "ADRASTEA");
         break;
      case METIS:
         strcpy(w_str, "METIS");
         break;
      case MIMAS:
         strcpy(w_str, "MIMAS");
         break;
      case ENCELADUS:
         strcpy(w_str, "ENCELADUS");
         break;
      case TETHYS:
         strcpy(w_str, "TETHYS");
         break;
      case DIONE:
         strcpy(w_str, "DIONE");
         break;
      case RHEA:
         strcpy(w_str, "RHEA");
         break;
      case TITAN:
         strcpy(w_str, "TITAN");
         break;
      case HYPERION:
         strcpy(w_str, "HYPERION");
         break;
      case IAPETUS:
         strcpy(w_str, "IAPETUS");
         break;
      case PHOEBE:
         strcpy(w_str, "PHOEBE");
         break;
      case JANUS:
         strcpy(w_str, "JANUS");
         break;
      case EPIMETHEUS:
         strcpy(w_str, "EPIMETHEUS");
         break;
      case HELENE:
         strcpy(w_str, "HELENE");
         break;
      case TELESTO:
         strcpy(w_str, "TELESTO");
         break;
      case CALYPSO:
         strcpy(w_str, "CALYPSO");
         break;
      case ATLAS:
         strcpy(w_str, "ATLAS");
         break;
      case PROMETHEUS:
         strcpy(w_str, "PROMETHEUS");
         break;
      case PANDORA:
         strcpy(w_str, "PANDORA");
         break;
      case PAN:
         strcpy(w_str, "PAN");
         break;
      case ARIEL:
         strcpy(w_str, "ARIEL");
         break;
      case UMBRIEL:
         strcpy(w_str, "UMBRIEL");
         break;
      case TITANIA:
         strcpy(w_str, "TITANIA");
         break;
      case OBERON:
         strcpy(w_str, "OBERON");
         break;
      case MIRANDA:
         strcpy(w_str, "MIRANDA");
         break;
      case TRITON:
         strcpy(w_str, "TRITON");
         break;
      case NEREID:
         strcpy(w_str, "NEREID");
         break;
      case CHARON:
         strcpy(w_str, "CHARON");
         break;
      default:
         if (w_id >= MINORBODY_0) {
            sprintf(w_str, "MINORBODY_%u", w_id - MINORBODY_0);
            break;
         }
         else {
            fprintf(stderr,
                    "Unknown WorldID %u in WorldID2String. Exiting...\n", w_id);
            exit(EXIT_FAILURE);
         }
   }
}
/**********************************************************************/
#ifdef _ENABLE_SPICE_
// Do some preconfiguration to interact with spice easier
SpiceInt WorldID2NAIFID(WorldID w_id)
{
   static int first                          = 0;
   static SpiceInt naif_id_list[MINORBODY_0] = {0};
   if (w_id >= MINORBODY_0) {
      fprintf(stderr, "WorldID2NAIFID() is not configured to handle the "
                      "user configured minor bodies. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   if (!first) {
      first = 1;
      for (WorldID Iw = SOL; Iw < MINORBODY_0; Iw++) {
         char world_name[32];
         WorldID2String(Iw, world_name);
         SpiceBoolean found = FALSE;
         bodn2c_c(world_name, &naif_id_list[Iw], &found);
         if (!found) {
            fprintf(stderr, "Could not find NAIF ID for body %s. Exiting...\n",
                    world_name);
            exit(EXIT_FAILURE);
         }
      }
   }
   return naif_id_list[w_id];
}
/**********************************************************************/
void WorldID2IAUFrame(WorldID w_id, char iau_frame[25])
{
   static int first                            = 0;
   static char iau_frame_list[MINORBODY_0][25] = {{'\0'}};
   if (w_id >= MINORBODY_0) {
      fprintf(stderr, "WorldID2OrientationNAIFID() is not configured to handle "
                      "the user configured minor bodies. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   if (!first) {
      // Some smaller moons do not have valid orientation data.
      // We replace these with the orientation of their planet
      // Substitutions:
      // HIMALIA, ELARA, PASIPHAE, SINOPE, LYSITHEA, CARME, ANANKE, LEDA ->
      // JUPITER HYPERION -> SATURN NEREID -> NEPTUNE
      first = 1;
      for (WorldID Iw = SOL; Iw < MINORBODY_0; Iw++) {
         char world_name[32];
         WorldID id = Iw;
         switch (Iw) {
            case HIMALIA:
            case ELARA:
            case PASIPHAE:
            case SINOPE:
            case LYSITHEA:
            case CARME:
            case ANANKE:
            case LEDA:
               id = JUPITER;
               break;
            case HYPERION:
               id = SATURN;
               break;
            case NEREID:
               id = NEPTUNE;
               break;
            default:
               break;
         }
         WorldID2String(id, world_name);
         strcpy(iau_frame_list[Iw], "IAU_");
         strcat(iau_frame_list[Iw], world_name);
      }
   }
   strcpy(iau_frame, iau_frame_list[w_id]);
}
#endif
/**********************************************************************/
ephemType GetEphemType(const char *s)
{
   if (!strcmp(s, "MEAN"))
      return EPH_MEAN;
   else if (!strcmp(s, "DE421"))
      return EPH_DE421;
   else if (!strcmp(s, "DE424"))
      return EPH_DE424;
   else if (!strcmp(s, "DE430"))
      return EPH_DE430;
   else if (!strcmp(s, "DE440"))
      return EPH_DE440;
   else if (!strcmp(s, "GMAT421"))
      return EPH_GMAT421;
   else if (!strcmp(s, "GMAT424"))
      return EPH_GMAT424;
   else if (!strcmp(s, "SPICE"))
      return EPH_SPICE;
   fprintf(stderr, "Bogus input %s in GetEphemType (42init.c:%d)\n", s,
           __LINE__);
   exit(EXIT_FAILURE);
}
/**********************************************************************/
void EchoDyn(struct SCType *S)
{
   FILE *outfile;
   char OutFileName[80];
   struct DynType *D;
   struct BodyType *B;
   struct JointType *G;
   long i, j, Ib, Ig, Nf;

   sprintf(OutFileName, "Dyn%02ld.42", S->ID);
   outfile = FileOpen(OutPath, OutFileName, "w");

   /* .. SC Structure */
   fprintf(outfile, "Dynamics Check for SC[%ld]\n\n", S->ID);
   fprintf(outfile, "Nb: %2ld   Ng: %2ld\n", S->Nb, S->Ng);
   fprintf(outfile, "Mass:  %lf\n", S->mass);
   fprintf(outfile, "cm:  %lf %lf %lf\n", S->cm[0], S->cm[1], S->cm[2]);
   fprintf(outfile,
           "I  :  %lf  %lf  %lf \n     %lf  %lf  %lf \n      %lf %lf  %lf\n",
           S->I[0][0], S->I[0][1], S->I[0][2], S->I[1][0], S->I[1][1],
           S->I[1][2], S->I[2][0], S->I[2][1], S->I[2][2]);
   fprintf(outfile, "PosR:  %lf %lf %lf\n", S->PosR[0], S->PosR[1], S->PosR[2]);
   fprintf(outfile, "VelR:  %lf %lf %lf\n\n", S->VelR[0], S->VelR[1],
           S->VelR[2]);

   /* .. Dyn Structure */
   D = &S->Dyn;
   fprintf(outfile, "Dyn Structure\n-------------\n");
   fprintf(outfile, "Nu:  %ld   Nx:  %ld  Nf:  %ld\n", D->Nu, D->Nx, D->Nf);
   for (i = 0; i < D->Nu; i++)
      fprintf(outfile, "u[%02ld]: %lf\n", i, D->u[i]);
   fprintf(outfile, "\n");
   for (i = 0; i < D->Nx; i++)
      fprintf(outfile, "x[%02ld]: %lf\n", i, D->x[i]);
   fprintf(outfile, "\n");
   for (i = 0; i < D->Nf; i++)
      fprintf(outfile, "[%02ld] uf: %lf   xf: %lf\n", i, D->uf[i], D->xf[i]);
   fprintf(outfile, "\n\n");

   /* .. Body Structures */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      fprintf(outfile, "Body Structure [%02ld]\n-------------------\n", Ib);
      fprintf(outfile, "Mass:  %lf\n", B->mass);
      fprintf(outfile, "cm:  %lf %lf %lf\n", B->cm[0], B->cm[1], B->cm[2]);
      fprintf(outfile, "c:   %lf %lf %lf\n", B->c[0], B->c[1], B->c[2]);
      fprintf(outfile, "I:  %lf %lf %lf\n     %lf %lf %lf\n   %lf %lf %lf\n",
              B->I[0][0], B->I[0][1], B->I[0][2], B->I[1][0], B->I[1][1],
              B->I[1][2], B->I[2][0], B->I[2][1], B->I[2][2]);
      fprintf(outfile, "wn:  %lf %lf %lf\n", B->wn[0], B->wn[1], B->wn[2]);
      fprintf(outfile, "qn:  %lf %lf %lf %lf\n", B->qn[0], B->qn[1], B->qn[2],
              B->qn[3]);
      fprintf(outfile, "vn:  %lf %lf %lf\n", B->vn[0], B->vn[1], B->vn[2]);
      fprintf(outfile, "pn:  %lf %lf %lf\n\n", B->pn[0], B->pn[1], B->pn[2]);
      fprintf(outfile, "CN:\n");
      fprintf(outfile, "%lf %lf %lf\n", B->CN[0][0], B->CN[0][1], B->CN[0][2]);
      fprintf(outfile, "%lf %lf %lf\n", B->CN[1][0], B->CN[1][1], B->CN[1][2]);
      fprintf(outfile, "%lf %lf %lf\n", B->CN[2][0], B->CN[2][1], B->CN[2][2]);
      fprintf(outfile, "Nf: %ld   f0: %ld\n", B->Nf, B->f0);
      fprintf(outfile, "Mf:\n");
      for (i = 0; i < B->Nf; i++) {
         for (j = 0; j < B->Nf; j++)
            fprintf(outfile, "  %lf", B->Mf[i][j]);
         fprintf(outfile, "\n");
      }
      fprintf(outfile, "\n");
      fprintf(outfile, "Kf:\n");
      for (i = 0; i < B->Nf; i++) {
         for (j = 0; j < B->Nf; j++)
            fprintf(outfile, "  %lf", B->Kf[i][j]);
         fprintf(outfile, "\n");
      }
      fprintf(outfile, "\n");
      fprintf(outfile, "Cf:\n");
      for (i = 0; i < B->Nf; i++) {
         for (j = 0; j < B->Nf; j++)
            fprintf(outfile, "  %lf", B->Cf[i][j]);
         fprintf(outfile, "\n");
      }
      fprintf(outfile, "\n");
      fprintf(outfile, "Pf:\n");
      for (i = 0; i < 3; i++) {
         for (j = 0; j < B->Nf; j++)
            fprintf(outfile, "  %lf", B->Pf[i][j]);
         fprintf(outfile, "\n");
      }
      fprintf(outfile, "\n");
      fprintf(outfile, "Hf:\n");
      for (i = 0; i < 3; i++) {
         for (j = 0; j < B->Nf; j++)
            fprintf(outfile, "  %lf", B->Hf[i][j]);
         fprintf(outfile, "\n");
      }
      fprintf(outfile, "\n");
   }
   fprintf(outfile, "\n");

   /* .. Joint Structures */
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G = &S->G[Ig];
      fprintf(outfile, "Joint Structure [%02ld]\n---------------\n", Ig);
      fprintf(outfile, "Rotu0: %ld   Rotx0: %ld\n", G->Rotu0, G->Rotx0);
      fprintf(outfile, "Trnu0: %ld   Trnx0: %ld\n", G->Trnu0, G->Trnx0);
      fprintf(outfile, "ang:  %lf %lf %lf\n", G->Ang[0], G->Ang[1], G->Ang[2]);
      fprintf(outfile, "rate:  %lf %lf %lf\n", G->AngRate[0], G->AngRate[1],
              G->AngRate[2]);
      fprintf(outfile, "COI:\n");
      fprintf(outfile, "%lf %lf %lf\n", G->COI[0][0], G->COI[0][1],
              G->COI[0][2]);
      fprintf(outfile, "%lf %lf %lf\n", G->COI[1][0], G->COI[1][1],
              G->COI[1][2]);
      fprintf(outfile, "%lf %lf %lf\n", G->COI[2][0], G->COI[2][1],
              G->COI[2][2]);
      fprintf(outfile, "SpringCoef: %lf %lf %lf\n", G->RotSpringCoef[0],
              G->RotSpringCoef[1], G->RotSpringCoef[2]);
      fprintf(outfile, "DampCoef: %lf %lf %lf\n", G->RotDampCoef[0],
              G->RotDampCoef[1], G->RotDampCoef[2]);
      Nf = S->B[G->Bin].Nf;
      fprintf(outfile, "PSIi:\n");
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Nf; j++)
            fprintf(outfile, " %lf", G->PSIi[i][j]);
         fprintf(outfile, "\n");
      }
      fprintf(outfile, "THETAi:\n");
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Nf; j++)
            fprintf(outfile, " %lf", G->THETAi[i][j]);
         fprintf(outfile, "\n");
      }
      Nf = S->B[G->Bout].Nf;
      fprintf(outfile, "PSIo:\n");
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Nf; j++)
            fprintf(outfile, " %lf", G->PSIo[i][j]);
         fprintf(outfile, "\n");
      }
      fprintf(outfile, "THETAo:\n");
      for (i = 0; i < 3; i++) {
         for (j = 0; j < Nf; j++)
            fprintf(outfile, " %lf", G->THETAo[i][j]);
         fprintf(outfile, "\n");
      }
      fprintf(outfile, "\n\n");
   }

   fclose(outfile);
}
/**********************************************************************/
long LoadTRVfromFile(const char *Path, const char *TrvFileName,
                     const char *ElemLabel, double Time, struct OrbitType *O)
{
   FILE *infile;
   char line[80], response1[80], response2[80];
   char Label[25];
   long i, Nchar;
   long Success = 0;
   double R[3], V[3];
   JDType Epoch_JD;

   DateType EpochDate = {0};
   EpochDate.system   = UTC_TIME;
   double sec         = 0;

   infile = FileOpen(Path, TrvFileName, "r");

   Nchar = strlen(ElemLabel);
   /* Pad label to 24 characters to assure unique match */
   while (!feof(infile) && !Success) {
      fgets(line, 79, infile);
      if (sscanf(line, "\"%[^\"]\"", Label) == 1) {
         if (!strncmp(Label, ElemLabel, Nchar)) {
            Success = 1;
            fscanf(infile, "%s %s %ld-%ld-%ld %ld:%ld:%lf\n", response1,
                   response2, &EpochDate.Year, &EpochDate.Month, &EpochDate.Day,
                   &EpochDate.Hour, &EpochDate.Minute, &sec);
            fscanf(infile, "%lf %lf %lf\n", &R[0], &R[1], &R[2]);
            fscanf(infile, "%lf %lf %lf\n", &V[0], &V[1], &V[2]);
         }
      }
   }
   fclose(infile);

   if (Success) {
      /* Epoch is in UTC */
      EpochDate.Second = double2rational(sec);
      Epoch_JD         = Date2JD(EpochDate, J2000_EPOCH);
      ChangeSystem(TT_TIME, &Epoch_JD);
      O->Epoch  = JDToDynTime(Epoch_JD);
      O->Regime = DecodeString(response1);
      if (O->Regime == ORB_CENTRAL || O->Regime == ORB_N_BODY) {
         O->World = GetWorldID(response2);
         O->mu    = World[O->World].mu;
         // TODO: Following assumes a Lunar N_Body orbit is given in ECI,
         //       consider other options for initialization.
         // double CNJ[3][3] = {0}, R_temp[3], V_temp[3];
         // if (O->World == LUNA && O->Regime == ORB_N_BODY) {
         //    LunaInertialFrame(TDB.JulDay, CNJ);
         //    MxV(CNJ,R,R_temp);
         //    MxV(CNJ,V,V_temp);
         //    for(i=0;i<3;i++) {
         //       R[i] = R_temp[i];
         //       V[i] = V_temp[i];
         //    }
         // }
         RV2Eph(O->Epoch, O->mu, R, V, &O->SMA, &O->ecc, &O->inc, &O->RAAN,
                &O->ArgP, &O->anom, &O->tp, &O->SLR, &O->alpha, &O->rmin,
                &O->MeanMotion, &O->Period);
         Eph2RV(O->mu, O->SLR, O->ecc, O->inc, O->RAAN, O->ArgP,
                Time - O->Epoch, O->PosN, O->VelN, &O->anom);
         // Save original SC Pos/Vel from TRV files
         if (O->Regime == ORB_N_BODY) {
            O->use_N_BODY_Vec = true;
            for (i = 0; i < 3; i++) {
               O->N_BODY_PosN[i] = R[i];
               O->N_BODY_VelN[i] = V[i];
            }
         }
      }
      else {
         O->Sys   = DecodeString(response2);
         O->Body1 = LagSys[O->Sys].Body1;
         O->Body2 = LagSys[O->Sys].Body2;
         O->mu1   = World[O->Body1].mu;
         O->mu2   = World[O->Body2].mu;
         O->World = O->Body1;
         O->mu    = O->mu1;
         for (i = 0; i < 3; i++) {
            O->PosN[i] = R[i];
            O->VelN[i] = V[i];
         }
         /* RV2LagModes(O->Epoch,&LagSys[O->Sys],O); */
         R2StableLagMode(O->Epoch, &LagSys[O->Sys], O);
         LagModes2RV(Time, &LagSys[O->Sys], O, O->PosN, O->VelN);
      }
   }

   return (Success);
}
/*********************************************************************/
void InitOrbit(struct OrbitType *O, const JDType jd)
{
   long i, j, k;

   JDType jd_tt_j2000 = jd;
   ChangeSystemEpoch(TT_TIME, J2000_EPOCH, &jd_tt_j2000);
   const double j2000_tt = JDToDynTime(jd_tt_j2000);

   char fileName[50] = {0};
   strcpy(fileName, O->FileName);
   // Replace ".txt" with ".yaml"
   char *typeStr = strstr(fileName, ".txt");
   if (typeStr != NULL)
      strcpy(typeStr, ".yaml");

   struct fy_document *fyd =
       fy_document_build_and_check(NULL, InOutPath, fileName);
   struct fy_node *root = fy_document_root(fyd);
   struct fy_node *node = NULL;
   node                 = fy_node_by_path_def(root, "/Orbit");

   /* .. Orbit Parameters */
   O->use_N_BODY_Vec = FALSE; // Start with TRV method disabled
   O->Epoch          = j2000_tt;
   O->SplineActive   = FALSE;
   char response[50] = {0};
   if (!fy_node_scanf(node, "/Type %49s", response)) {
      fprintf(stderr, "Could not find orbit type. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   O->Regime = DecodeString(response);
   enum orbitInputType inputType;
   switch (O->Regime) {
      case ORB_ZERO: {
         if (!fy_node_scanf(node, "/World %49s", response)) {
            fprintf(stderr, "Could not find World for orbit. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         O->World = GetWorldID(response);
         if (!World[O->World].Exists) {
            fprintf(stderr,
                    "Oops.  Orbit %ld depends on a World that doesn't exist.\n",
                    O->Tag);
            exit(EXIT_FAILURE);
         }

         O->mu = World[O->World].mu;
         for (j = 0; j < 3; j++) {
            O->PosN[j] = 0.0;
            O->VelN[j] = 0.0;
            for (k = 0; k < 3; k++)
               O->CLN[j][k] = 0.0;
            O->CLN[j][j] = 1.0;
            O->wln[j]    = 0.0;
         }
         O->PolyhedronGravityEnabled =
             getYAMLBool(fy_node_by_path_def(node, "/Polyhedron Grav"));
      } break;
      case ORB_FLIGHT: {
         long Ir = 0;
         if (!fy_node_scanf(node, "/Region %ld", &Ir)) {
            fprintf(stderr, "Could not find region for orbit. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         if (!Rgn[Ir].Exists) {
            fprintf(
                stderr,
                "Oops.  Orbit %ld depends on a Region that doesn't exist.\n",
                O->Tag);
            exit(EXIT_FAILURE);
         }
         O->Region            = Ir;
         struct RegionType *R = &Rgn[Ir];
         O->World             = R->World;
         O->mu                = World[O->World].mu;
         for (j = 0; j < 3; j++) {
            O->PosN[j] = R->PosN[j];
            O->VelN[j] = R->VelN[j];
            for (k = 0; k < 3; k++)
               O->CLN[j][k] = R->CN[j][k];
         }
         O->wln[0] = 0.0;
         O->wln[1] = 0.0;
         O->wln[2] = World[O->World].w;
         O->PolyhedronGravityEnabled =
             getYAMLBool(fy_node_by_path_def(node, "/Polyhedron Grav"));
      } break;
      case ORB_N_BODY:
      case ORB_CENTRAL: {
         if (!fy_node_scanf(node, "/World %49s", response)) {
            fprintf(stderr, "Could not find World for orbit. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         O->World = GetWorldID(response);
         if (!World[O->World].Exists) {
            fprintf(stderr,
                    "Oops.  Orbit %ld depends on a World that doesn't exist.\n",
                    O->Tag);
            exit(EXIT_FAILURE);
         }
         O->J2DriftEnabled =
             getYAMLBool(fy_node_by_path_def(node, "/J2 Secular Drift"));
         O->mu      = World[O->World].mu;
         double rad = World[O->World].rad;
         double J2  = World[O->World].J2;
         node       = fy_node_by_path_def(node, "/Init");
         if (!fy_node_scanf(node, "/Method %49s", response)) {
            fprintf(stderr,
                    "Could not find Central orbit initialization method. "
                    "Exiting...\n");
            exit(EXIT_FAILURE);
         }
         inputType = DecodeString(response);
         switch (inputType) {
            case INP_KEPLER: {
               if (fy_node_scanf(node,
                                 "/SMA Parameterization %49s "
                                 "/Inclination %lf "
                                 "/RAAN %lf "
                                 "/Arg of Periapsis %lf "
                                 "/True Anomaly %lf",
                                 response, &O->inc, &O->RAAN, &O->ArgP,
                                 &O->anom) != 5) {
                  fprintf(stderr,
                          "Invalid Keplarian initialization. Exiting...\n");
                  exit(EXIT_FAILURE);
               }
               long usePA = DecodeString(response);
               if (usePA) {
                  double alt1, alt2;
                  if (!fy_node_scanf(node,
                                     "/Periapsis %lf "
                                     "/Apoapsis %lf",
                                     &alt1, &alt2)) {
                     fprintf(stderr,
                             "Could not find Periapsis and/or Apoapsis. "
                             "Exiting...\n");
                     exit(EXIT_FAILURE);
                  }
                  O->SMA        = rad + 0.5 * (alt1 + alt2) * 1.0E3;
                  O->ecc        = 1.0E3 * fabs(alt1 - alt2) / (2.0 * O->SMA);
                  O->SLR        = O->SMA * (1.0 - O->ecc * O->ecc);
                  O->alpha      = 1.0 / O->SMA;
                  O->rmin       = rad + alt1 * 1.0E3;
                  O->MeanMotion = sqrt(O->mu * O->alpha) * O->alpha;
               }
               else {
                  double alt1;
                  if (!fy_node_scanf(node,
                                     "/Minimum Altitude %lf "
                                     "/Eccentricity %lf",
                                     &alt1, &O->ecc)) {
                     fprintf(stderr, "Could not find Minimum Altitude and/or "
                                     "Eccentricity. Exiting...\n");
                     exit(EXIT_FAILURE);
                  }
                  O->rmin  = rad + alt1 * 1.0E3;
                  O->SLR   = O->rmin * (1.0 + O->ecc);
                  O->alpha = (1.0 - O->ecc) / O->rmin;
                  if (O->alpha != 0.0)
                     O->SMA = 1.0 / O->alpha;
                  if (O->alpha > 0.0)
                     O->MeanMotion = sqrt(O->mu * O->alpha) * O->alpha;
                  else
                     O->MeanMotion = sqrt(-O->mu * O->alpha) * O->alpha;
               }
               O->inc  *= D2R;
               O->RAAN *= D2R;
               O->ArgP *= D2R;
               O->anom *= D2R;
               O->tp    = O->Epoch -
                          TimeSincePeriapsis(O->mu, O->SLR, O->ecc, O->anom);
               /* Some anomalies are unreachable for hyperbolic trajectories */
               if (O->ecc > 1.0) {
                  double maxAnom = Pi - acos(1.0 / O->ecc);
                  if (fabs(O->anom) > maxAnom) {
                     fprintf(stderr,
                             "True Anomaly out of range for Orbit %ld\n",
                             O->Tag);
                     exit(EXIT_FAILURE);
                  }
               }

               if (O->J2DriftEnabled) {
                  OscEphToMeanEph(O->mu, J2, rad, JD_TT_MJD_0, O);
               }
               Eph2RV(O->mu, O->SLR, O->ecc, O->inc, O->RAAN, O->ArgP,
                      O->Epoch - O->tp, O->PosN, O->VelN, &O->anom);

            } break;
            case INP_POSVEL: {
               assignYAMLToDoubleArray(
                   3, fy_node_by_path_def(node, "/Position"), O->PosN);
               assignYAMLToDoubleArray(
                   3, fy_node_by_path_def(node, "/Velocity"), O->VelN);
               for (j = 0; j < 3; j++) {
                  O->PosN[j] *= 1.0E3;
                  O->VelN[j] *= 1.0E3;
               }
               RV2Eph(O->Epoch, O->mu, O->PosN, O->VelN, &O->SMA, &O->ecc,
                      &O->inc, &O->RAAN, &O->ArgP, &O->anom, &O->tp, &O->SLR,
                      &O->alpha, &O->rmin, &O->MeanMotion, &O->Period);
               if (O->J2DriftEnabled) {
                  OscEphToMeanEph(O->mu, J2, rad, JD_TT_MJD_0, O);
               }
            } break;
            case INP_FILE: {
               char elementFileName[50] = {0}, elementLabel[50] = {0};
               if (!fy_node_scanf(node,
                                  "/File Type %49s "
                                  "/File Name %49[^\n] "
                                  "/Label in File %49[^\n]",
                                  response, elementFileName, elementLabel)) {
                  fprintf(
                      stderr,
                      "Could not configure File initialization. Exiting...\n");
                  exit(EXIT_FAILURE);
               }
               inputType = DecodeString(response);
               switch (inputType) {
                  case INP_TLE: {
                     if (O->World != EARTH) {
                        fprintf(
                            stderr,
                            "TLEs are only defined for Earth-orbiting S/C.\n");
                        exit(EXIT_FAILURE);
                     }
                     if (!LoadTleFromFile(InOutPath, elementFileName,
                                          elementLabel, j2000_tt, jd_tt_j2000,
                                          O)) {
                        fprintf(stderr, "Error loading TLE %s from file %s.\n",
                                elementLabel, elementFileName);
                        exit(EXIT_FAILURE);
                     }
                     MeanEph2RV(O, j2000_tt);
                  } break;
                  case INP_TRV: {
                     if (!LoadTRVfromFile(InOutPath, elementFileName,
                                          elementLabel, CivilTime, O)) {
                        fprintf(stderr, "Error loading TRV %s from file %s.\n",
                                elementLabel, elementFileName);
                        exit(EXIT_FAILURE);
                     }
                  } break;
                  case INP_SPLINE: {
                     O->SplineFile = FileOpen(InOutPath, elementFileName, "rt");
                     O->SplineActive   = TRUE;
                     DateType NodeDate = {0};
                     double sec        = 0;
                     NodeDate.system   = UTC_TIME;
                     char newline;
                     for (i = 0; i < 4; i++) {
                        fscanf(
                            O->SplineFile,
                            "%ld-%ld-%ldT%ld:%ld:%lf %lf %lf %lf %lf %lf %lf "
                            "%[\n]",
                            &NodeDate.Year, &NodeDate.Month, &NodeDate.Day,
                            &NodeDate.Hour, &NodeDate.Minute, &sec,
                            &O->NodePos[i][0], &O->NodePos[i][1],
                            &O->NodePos[i][2], &O->NodeVel[i][0],
                            &O->NodeVel[i][1], &O->NodeVel[i][2], &newline);
                        NodeDate.Second = double2rational(sec);
                        JDType node_jd  = Date2JD(NodeDate, J2000_EPOCH);
                        ChangeSystem(TT_TIME, &node_jd);
                        O->NodeDynTime[i] = JDToDynTime(node_jd);
                        for (j = 0; j < 3; j++) {
                           O->NodePos[i][j] *= 1000.0;
                           O->NodeVel[i][j] *= 1000.0;
                        }
                        if (j2000_tt < O->NodeDynTime[1]) {
                           fprintf(stderr,
                                   "Oops.  Spline file beginning is in the "
                                   "future.\n");
                           exit(EXIT_FAILURE);
                        }
                     }
                     SplineToPosVel(O, j2000_tt);
                  } break;
                  default:
                     fprintf(stderr,
                             "Invalid filetype in Orbit %ld. Exiting...\n",
                             O->Tag);
                     exit(EXIT_FAILURE);
                     break;
               }

            } break;
            default:
               fprintf(
                   stderr,
                   "Invalid central orbit initialization type in Orbit %ld. "
                   "Exiting...\n",
                   O->Tag);
               exit(EXIT_FAILURE);
               break;
         }
         FindCLN(O->PosN, O->VelN, O->CLN, O->wln);

      } break;
      case ORB_THREE_BODY: {
         if (!fy_node_scanf(node, "/Lagrange System %49s", response)) {
            fprintf(stderr,
                    "Could not find Lagrange System for Three Body orbit. "
                    "Exiting...\n");
            exit(EXIT_FAILURE);
         }
         O->Sys = DecodeString(response);
         if (!LagSys[O->Sys].Exists) {
            fprintf(
                stderr,
                "Oops.  Orbit %ld depends on a Lagrange System that doesn't "
                "exist.\n",
                O->Tag);
            exit(EXIT_FAILURE);
         }
         O->Body1 = LagSys[O->Sys].Body1;
         O->Body2 = LagSys[O->Sys].Body2;
         O->mu1   = LagSys[O->Sys].mu1;
         O->mu2   = LagSys[O->Sys].mu2;
         if (!fy_node_scanf(node, "/Propagation Method %49s", response)) {
            fprintf(stderr,
                    "Could not find Propagation Method for Three Body orbit. "
                    "Exiting...\n");
            exit(EXIT_FAILURE);
         }
         O->LagDOF = DecodeString(response);

         node = fy_node_by_path_def(node, "/Init");
         if (!fy_node_scanf(node, "/Method %49s", response)) {
            fprintf(stderr,
                    "Could not find Three Body orbit Initialization Method. "
                    "Exiting...\n");
            exit(EXIT_FAILURE);
         }
         inputType = DecodeString(response);
         switch (inputType) {
            case INP_MODES: {
               double senseXY1 = 1.0, senseXY2 = 1.0;
               double ampXY1, ampXY2 = 0.0, phiXY1, phiXY2 = 0.0, ampZ, phiZ;

               if (!fy_node_scanf(node, "/Lagrange Point %49s", response)) {
                  fprintf(stderr, "Could not find Lagrange point of modal "
                                  "initialization. Exiting...\n");
                  exit(EXIT_FAILURE);
               }
               O->LP = DecodeString(response);
               if (fy_node_scanf(node,
                                 "/XY SMA %lf "
                                 "/XY Phase %lf "
                                 "/Sense %49s "
                                 "/Z SMA %lf "
                                 "/Z Phase %lf",
                                 &ampXY1, &phiXY1, response, &ampZ,
                                 &phiZ) != 5) {
                  fprintf(stderr, "Invalid configuration for Modal Three Body "
                                  "Initialization. Exiting...\n");
                  exit(EXIT_FAILURE);
               }
               if (DecodeString(response) == DIR_CW)
                  senseXY1 = -1.0;
               if (O->LP == LAGPT_L4 || O->LP == LAGPT_L5) {
                  if (fy_node_scanf(node,
                                    "/XY 2nd SMA %lf "
                                    "/XY 2nd Phase %lf "
                                    "/2nd Sense %49s",
                                    &ampXY2, &phiXY2, response) != 3) {
                     fprintf(stderr,
                             "Invalid configuration for Triangular Lagrange "
                             "Point modal initialization. Exiting...\n");
                     exit(EXIT_FAILURE);
                  }
                  if (DecodeString(response) == DIR_CW)
                     senseXY2 = -1.0;
               }
               AmpPhase2LagModes(0.0, ampXY1, phiXY1, senseXY1, ampXY2, phiXY2,
                                 senseXY2, ampZ, phiZ, &LagSys[O->Sys], O);
               /* Find r,v from modal description */
               LagModes2RV(j2000_tt, &LagSys[O->Sys], O, O->PosN, O->VelN);

            } break;
            case INP_XYZ: {
               double vec3[3] = {0.0};
               assignYAMLToDoubleArray(
                   3, fy_node_by_path_def(node, "/Position"), vec3);
               O->x = vec3[0];
               O->y = vec3[1];
               O->z = vec3[2];
               assignYAMLToDoubleArray(
                   3, fy_node_by_path_def(node, "/Velocity"), vec3);
               O->xdot = vec3[0];
               O->ydot = vec3[1];
               O->zdot = vec3[2];
               XYZ2LagModes(0.0, &LagSys[O->Sys], O);
               LagModes2RV(j2000_tt, &LagSys[O->Sys], O, O->PosN, O->VelN);
            } break;
            case INP_XYZ_ROT: {
               double vec3_p[3] = {0.0};
               double vec3_v[3] = {0.0};

               struct LagrangeSystemType *LS;
               LS = &LagSys[O->Sys];

               assignYAMLToDoubleArray(
                   3, fy_node_by_path_def(node, "/Position"), vec3_p);
               assignYAMLToDoubleArray(
                   3, fy_node_by_path_def(node, "/Velocity"), vec3_v);

               StateRnd2StateN(LS, World[LS->Body2].eph.PosN,
                               World[LS->Body2].eph.VelN, vec3_p, vec3_v,
                               O->PosN, O->VelN);
            } break;
            case INP_FILE: {
               char elementFileName[50] = {0}, elementLabel[50] = {0};
               if (fy_node_scanf(node,
                                 "/Lagrange Point %49s "
                                 "/File Name %49[^\n] "
                                 "/Label in File %49[^\n]",
                                 response, elementFileName,
                                 elementLabel) != 3) {
                  fprintf(
                      stderr,
                      "Could not configure File initialization. Exiting...\n");
                  exit(EXIT_FAILURE);
               }
               O->LP = DecodeString(response);
               if (!fy_node_scanf(node, "/File Type %49s", response)) {
                  fprintf(stderr,
                          "Could not find File Type for Three Body Orbit "
                          "Initialization. Exiting...\n");
                  exit(EXIT_FAILURE);
               }
               inputType = DecodeString(response);
               switch (inputType) {
                  case INP_TRV: {
                     if (!LoadTRVfromFile(InOutPath, elementFileName,
                                          elementLabel, CivilTime, O)) {
                        fprintf(stderr, "Error loading TRV %s from file %s.\n",
                                elementLabel, elementFileName);
                        exit(EXIT_FAILURE);
                     }
                  } break;
                  case INP_SPLINE: {
                     O->SplineFile = FileOpen(InOutPath, elementFileName, "rt");
                     O->SplineActive   = TRUE;
                     DateType NodeDate = {0};
                     double sec        = 0;
                     NodeDate.system   = UTC_TIME;
                     char newline;
                     for (i = 0; i < 4; i++) {
                        fscanf(
                            O->SplineFile,
                            "%ld-%ld-%ldT%ld:%ld:%lf %lf %lf %lf %lf %lf %lf "
                            "%[\n]",
                            &NodeDate.Year, &NodeDate.Month, &NodeDate.Day,
                            &NodeDate.Hour, &NodeDate.Minute, &sec,
                            &O->NodePos[i][0], &O->NodePos[i][1],
                            &O->NodePos[i][2], &O->NodeVel[i][0],
                            &O->NodeVel[i][1], &O->NodeVel[i][2], &newline);
                        NodeDate.Second = double2rational(sec);
                        JDType node_jd  = Date2JD(NodeDate, J2000_EPOCH);
                        ChangeSystem(TT_TIME, &node_jd);
                        O->NodeDynTime[i] = JDToDynTime(node_jd);
                        for (j = 0; j < 3; j++) {
                           O->NodePos[i][j] *= 1000.0;
                           O->NodeVel[i][j] *= 1000.0;
                        }
                        if (j2000_tt < O->NodeDynTime[1]) {
                           fprintf(stderr,
                                   "Oops.  Spline file beginning is in the "
                                   "future.\n");
                           exit(EXIT_FAILURE);
                        }
                     }
                     SplineToPosVel(O, j2000_tt);
                  }

                  break;
                  default:
                     break;
               }
            } break;
            default:
               fprintf(
                   stderr,
                   "Invalid three body orbit initialization type in Orbit %ld. "
                   "Exiting...\n",
                   O->Tag);
               exit(EXIT_FAILURE);
               break;
         }
         O->World = O->Body1;
         O->mu    = O->mu1;
         O->SMA   = MAGV(O->PosN); /* For sake of EH */
         FindCLN(O->PosN, O->VelN, O->CLN, O->wln);
         O->MeanMotion = LagSys[O->Sys].MeanRate;
         O->Period     = TwoPi / O->MeanMotion;
      } break;
      default:
         fprintf(stderr, "Bogus Orbit Regime in file %s\n", O->FileName);
         exit(EXIT_FAILURE);
         break;
   }

   /* .. Formation Frame Parameters */
   struct FormationType *F = &Frm[O->Tag];
   node                    = fy_node_by_path_def(root, "/Formation");

   char FrmExpressedIn = 0;
   if (fy_node_scanf(node,
                     "/Fixed Frame %c "
                     "/Expression Frame %c",
                     &F->FixedInFrame, &FrmExpressedIn) != 2) {
      fprintf(stderr,
              "Could not find configuration for formation frame. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   double ang[3] = {0.0};
   long seq;
   getYAMLEulerAngles(fy_node_by_path_def(node, "/Euler Angles"), ang, &seq);
   A2C(seq, ang[0] * D2R, ang[1] * D2R, ang[2] * D2R, F->CN);

   if (F->FixedInFrame == 'L') {
      /* Adjust CFN */
      for (j = 0; j < 3; j++) {
         for (k = 0; k < 3; k++)
            F->CL[j][k] = F->CN[j][k];
      }
      MxM(F->CL, O->CLN, F->CN);
   }
   assignYAMLToDoubleArray(3, fy_node_by_path_def(node, "/Position"), F->PosR);
   if (FrmExpressedIn == 'L') {
      double p[3] = {0.0};
      for (j = 0; j < 3; j++)
         p[j] = F->PosR[j];
      MTxV(O->CLN, p, F->PosR);
   }
   fy_document_destroy(fyd);
}
/**********************************************************************/
void InitRigidDyn(struct SCType *S)
{
   long i, j, Ig, Ia, Jg, Jb, Ibody, Ib, u0, x0, c0, Nu, Nx;
   struct JointType *G;
   struct DynType *D;
   FILE *outfile;
   char filename[80];

   D = &S->Dyn;

   /* .. Tree Tables */
   /* Connectivity Table */
   /* Inner body -> -1, Outer body -> +1, else 0 */
   D->ConnectTable = (long **)calloc(S->Nb, sizeof(long *));
   for (i = 0; i < S->Nb; i++)
      D->ConnectTable[i] = (long *)calloc(S->Ng, sizeof(long));
   for (Ib = 0; Ib < S->Nb; Ib++) {
      for (Ig = 0; Ig < S->Ng; Ig++) {
         D->ConnectTable[Ib][Ig] = 0;
      }
   }
   for (Ig = 0; Ig < S->Ng; Ig++) {
      D->ConnectTable[S->G[Ig].Bin][Ig]  = -1;
      D->ConnectTable[S->G[Ig].Bout][Ig] = 1;
   }

   /* Joint Path Table */
   /* Joint in path -> 1, else 0 */
   D->JointPathTable = (struct JointPathTableType **)calloc(
       S->Nb, sizeof(struct JointPathTableType *));
   for (i = 0; i < S->Nb; i++)
      D->JointPathTable[i] = (struct JointPathTableType *)calloc(
          S->Ng, sizeof(struct JointPathTableType));
   for (Ibody = 0; Ibody < S->Nb; Ibody++) {
      Ib = Ibody;
      while (Ib > 0) {
         Ig                                  = S->B[Ib].Gin;
         D->JointPathTable[Ibody][Ig].InPath = 1;
         Ib                                  = S->G[Ig].Bin;
      }
   }
   /* Body Path Table */
   /* Body in path -> 1, else 0 */
   D->BodyPathTable = (struct BodyPathTableType **)calloc(
       (int)S->Nb, sizeof(struct BodyPathTableType *));
   for (i = 0; i < S->Nb; i++)
      D->BodyPathTable[i] = (struct BodyPathTableType *)calloc(
          S->Nb, sizeof(struct BodyPathTableType));
   for (Ibody = 0; Ibody < S->Nb; Ibody++) {
      Ib = Ibody;
      while (Ib > 0) {
         Ig = 0;
         while (S->G[Ig].Bout != Ib)
            Ig++;
         D->BodyPathTable[Ibody][Ib].InPath = 1;
         Ib                                 = S->G[Ig].Bin;
      }
      D->BodyPathTable[Ibody][0].InPath = 1;
   }

   /* Find ancestor joint list for each joint */
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G       = &S->G[Ig];
      Ib      = S->G[Ig].Bin;
      G->Nanc = 0;
      while (Ib > 0) {
         G->Nanc++;
         Jg = S->B[Ib].Gin;
         Ib = S->G[Jg].Bin;
      }
      G->Anc = (long *)calloc(G->Nanc, sizeof(long));
      Ib     = S->G[Ig].Bin;
      Ia     = 0;
      while (Ib > 0) {
         Jg         = S->B[Ib].Gin;
         G->Anc[Ia] = Jg;
         Ib         = S->G[Jg].Bin;
         Ia++;
      }
   }

   /* Determine sizes of state vectors */
   D->Nu = 6;
   D->Nx = 7;
   u0    = 3;
   x0    = 4;
   D->Nc = 0;
   c0    = 0;
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G = &S->G[Ig];
      if (G->IsSpherical) {
         G->RotDOF  = 3;
         G->RotSeq  = 123;
         D->Nu     += 3;
         D->Nx     += 4;
         G->Rotu0   = u0;
         G->Rotx0   = x0;
         u0        += 3;
         x0        += 4;
      }
      else { /* Is Gimbal */
         D->Nu    += G->RotDOF;
         D->Nx    += G->RotDOF;
         G->Rotu0  = u0;
         G->Rotx0  = x0;
         u0       += G->RotDOF;
         x0       += G->RotDOF;
         G->Rotc0  = c0;
         c0       += 3 - G->RotDOF;
      }
      D->Nu    += G->TrnDOF;
      D->Nx    += G->TrnDOF;
      G->Trnu0  = u0;
      G->Trnx0  = x0;
      u0       += G->TrnDOF;
      x0       += G->TrnDOF;
      G->Trnc0  = c0;
      c0       += 3 - G->TrnDOF;
   }

   /* .. Allocate Workspace */
   D->PAngVel  = CreateMatrix(3 * S->Nb, D->Nu);
   D->IPAngVel = CreateMatrix(3 * S->Nb, D->Nu);
   D->PVel     = CreateMatrix(3 * S->Nb, D->Nu);
   D->mPVel    = CreateMatrix(3 * S->Nb, D->Nu);
   D->BodyTrq  = (double *)calloc(3 * S->Nb, sizeof(double));
   D->BodyFrc  = (double *)calloc(3 * S->Nb, sizeof(double));

   D->u    = (double *)calloc(D->Nu, sizeof(double));
   D->uu   = (double *)calloc(D->Nu, sizeof(double));
   D->du   = (double *)calloc(D->Nu, sizeof(double));
   D->udot = (double *)calloc(D->Nu, sizeof(double));
   D->x    = (double *)calloc(D->Nx, sizeof(double));
   D->xx   = (double *)calloc(D->Nx, sizeof(double));
   D->dx   = (double *)calloc(D->Nx, sizeof(double));
   D->xdot = (double *)calloc(D->Nx, sizeof(double));
   D->h    = (double *)calloc(S->Nw, sizeof(double));
   D->hh   = (double *)calloc(S->Nw, sizeof(double));
   D->dh   = (double *)calloc(S->Nw, sizeof(double));
   D->hdot = (double *)calloc(S->Nw, sizeof(double));
   D->a    = (double *)calloc(S->Nw, sizeof(double));
   D->aa   = (double *)calloc(S->Nw, sizeof(double));
   D->da   = (double *)calloc(S->Nw, sizeof(double));
   D->adot = (double *)calloc(S->Nw, sizeof(double));

   for (i = 0; i < 3; i++) {
      D->PAngVel[i][i] = 1.0;
      for (j = 0; j < 3; j++)
         D->IPAngVel[i][j] = S->B[0].I[i][j];
   }
   for (Ib = 0; Ib < S->Nb; Ib++) {
      for (i = 0; i < 3; i++) {
         D->PVel[3 * Ib + i][D->Nu - 3 + i]  = 1.0;
         D->mPVel[3 * Ib + i][D->Nu - 3 + i] = S->B[Ib].mass;
      }
   }

   if (S->ConstraintsRequested) {
      D->Nc               = 6 * S->Nb - D->Nu;
      D->PAngVelc         = CreateMatrix(3 * S->Nb, D->Nc);
      D->PVelc            = CreateMatrix(3 * S->Nb, D->Nc);
      D->TotalTrq         = (double *)calloc(3 * S->Nb, sizeof(double));
      D->TotalFrc         = (double *)calloc(3 * S->Nb, sizeof(double));
      D->GenConstraintFrc = (double *)calloc(D->Nc, sizeof(double));
   }

   MapJointStatesToStateVector(S);

   /* .. Echo tree tables */
   sprintf(filename, "Tree%02ld.42", S->ID);
   outfile = FileOpen(OutPath, filename, "w");
   fprintf(outfile, "SC %2ld:  Nb = %2ld  Ng = %2ld\n\n", S->ID, S->Nb, S->Ng);
   fprintf(outfile, "Connect Table:\n\n");
   fprintf(outfile, "     ");
   for (Ig = 0; Ig < S->Ng; Ig++)
      fprintf(outfile, "  G[%02ld]", Ig);
   fprintf(outfile, "\n");
   for (Ib = 0; Ib < S->Nb; Ib++) {
      fprintf(outfile, "B[%02ld]:", Ib);
      for (Ig = 0; Ig < S->Ng; Ig++) {
         fprintf(outfile, "  %3ld  ", D->ConnectTable[Ib][Ig]);
      }
      fprintf(outfile, "\n");
   }
   fprintf(outfile, "\n\n\nJoint Path Table:\n\n");
   fprintf(outfile, "     ");
   for (Ig = 0; Ig < S->Ng; Ig++)
      fprintf(outfile, "  G[%02ld]", Ig);
   fprintf(outfile, "\n");
   for (Ib = 0; Ib < S->Nb; Ib++) {
      fprintf(outfile, "B[%02ld]:", Ib);
      for (Ig = 0; Ig < S->Ng; Ig++) {
         fprintf(outfile, "  %3ld  ", D->JointPathTable[Ib][Ig].InPath);
      }
      fprintf(outfile, "\n");
   }
   fprintf(outfile, "\n\n\nBody Path Table:\n\n");
   fprintf(outfile, "     ");
   for (Jb = 0; Jb < S->Nb; Jb++)
      fprintf(outfile, "  B[%02ld]", Jb);
   fprintf(outfile, "\n");
   for (Ib = 0; Ib < S->Nb; Ib++) {
      fprintf(outfile, "B[%02ld]:", Ib);
      for (Jb = 0; Jb < S->Nb; Jb++) {
         fprintf(outfile, "  %3ld  ", D->BodyPathTable[Ib][Jb].InPath);
      }
      fprintf(outfile, "\n");
   }

   /* .. Echo State Vector */
   fprintf(outfile, "\n\n\nState Vector Map:\n\n");
   fprintf(
       outfile,
       "Body/Joint   RotDOF   TrnDOF   RotSeq   TrnSeq       u[]      x[]\n");
   fprintf(
       outfile,
       "  B[00]        wn       ---      123      ---       00-02    00-03\n");
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G  = &S->G[Ig];
      Nu = G->RotDOF + G->TrnDOF;
      Nx = (G->IsSpherical ? Nu + 1 : Nu);
      fprintf(outfile,
              "  G[%02ld]       %3ld      %3ld       %3ld      %3ld       "
              "%02ld-%02ld    %02ld-%02ld\n",
              Ig, G->RotDOF, G->TrnDOF, G->RotSeq, G->TrnSeq, G->Rotu0,
              G->Rotu0 + Nu - 1, G->Rotx0, G->Rotx0 + Nx - 1);
   }
   fprintf(outfile,
           "  B[00]       ---       vn       ---      123       %02ld-%02ld    "
           "%02ld-%02ld\n",
           D->Nu - 3, D->Nu - 1, D->Nx - 3, D->Nx - 1);

   /* .. DOF/Constraint Map */
   fprintf(
       outfile,
       "\n\n\nMap all 6*Nb potential DOF Axes into DOFs or Constraints\n\n");
   fprintf(
       outfile,
       "*****************************************************************\n");
   fprintf(outfile, "Body 00:   RotSeq = 123   TrnSeq = 123\n");
   fprintf(outfile,
           "                                Col in   Col in       Col in\n");
   fprintf(outfile,
           "Axis      F/C    u[]  x[]       u%02ld.42   x%02ld.42   "
           "Constraint%02ld.42\n",
           S->ID, S->ID, S->ID);
   fprintf(
       outfile,
       "-----------------------------------------------------------------\n");
   fprintf(outfile,
           "Rot1       F      00   00         01       01           --\n");
   fprintf(outfile,
           "Rot2       F      01   01         02       02           --\n");
   fprintf(outfile,
           "Rot3       F      02   02         03       03           --\n");
   fprintf(outfile,
           "(Sph)      -      --   03         --       04           --\n\n");
   fprintf(outfile,
           "Trn1       F      %02ld   %02ld         %02ld       %02ld          "
           " --\n",
           D->Nu - 3, D->Nx - 3, D->Nu - 2, D->Nx - 2);
   fprintf(outfile,
           "Trn2       F      %02ld   %02ld         %02ld       %02ld          "
           " --\n",
           D->Nu - 2, D->Nx - 2, D->Nu - 1, D->Nx - 1);
   fprintf(outfile,
           "Trn3       F      %02ld   %02ld         %02ld       %02ld          "
           " --\n",
           D->Nu - 1, D->Nx - 1, D->Nu, D->Nx);
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G = &S->G[Ig];
      fprintf(outfile, "*******************************************************"
                       "**********\n");
      fprintf(outfile, "Joint %02ld:   RotSeq = %3ld   TrnSeq = %3ld\n", Ig,
              G->RotSeq, G->TrnSeq);
      fprintf(outfile,
              "                                Col in   Col in       Col in\n");
      fprintf(outfile,
              "Axis      F/C    u[]  x[]       u%02ld.42   x%02ld.42   "
              "Constraint%02ld.42\n",
              S->ID, S->ID, S->ID);
      fprintf(outfile, "-------------------------------------------------------"
                       "----------\n");
      for (i = 0; i < G->RotDOF; i++) {
         fprintf(outfile,
                 "Rot%ld       F      %02ld   %02ld         %02ld       %02ld  "
                 "         --\n",
                 i + 1, G->Rotu0 + i, G->Rotx0 + i, G->Rotu0 + i + 1,
                 G->Rotx0 + i + 1);
      }
      for (i = 0; i < 3 - G->RotDOF; i++) {
         fprintf(outfile,
                 "Rot%ld       C      --   --         --       --           "
                 "%02ld\n",
                 G->RotDOF + i + 1, G->Rotc0 + i + 1);
      }
      if (G->IsSpherical) {
         fprintf(outfile,
                 "(Sph)      -      --   %02ld         --       %02ld          "
                 " --\n",
                 G->Rotx0 + 3, G->Rotx0 + 4);
      }
      fprintf(outfile, "\n");
      for (i = 0; i < G->TrnDOF; i++) {
         fprintf(outfile,
                 "Trn%ld       F      %02ld   %02ld         %02ld       %02ld  "
                 "         --\n",
                 i, G->Trnu0 + i + 1, G->Trnx0 + i, G->Trnu0 + i + 1,
                 G->Trnx0 + i + 1);
      }
      for (i = 0; i < 3 - G->TrnDOF; i++) {
         fprintf(outfile,
                 "Trn%ld       C      --   --         --       --           "
                 "%02ld\n",
                 G->TrnDOF + i + 1, G->Trnc0 + i + 1);
      }
   }
   fclose(outfile);
}
/**********************************************************************/
void InitFlexModes(struct SCType *S)
{
   FILE *infile;
   struct DynType *D;
   struct BodyType *B;
   struct JointType *G;
   long Ib, Ig, If, Im, Jm, Ia, In;
   long Nnonzero, Iz;
   long i, j;
   double value, wf;
   char junk[80], newline;
   double ***L, ****N;
   struct NodeType *FN;

   D     = &S->Dyn;
   D->Nf = 0;

   /* .. First pass through all flex input files to allocate matrices */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B     = &S->B[Ib];
      B->Nf = 0;
      if (strcmp(B->FlexFileName, "NONE")) {
         infile = FileOpen(InOutPath, B->FlexFileName, "r");
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         /* Number of Flex Modes */
         fscanf(infile, "%ld %[^\n] %[\n]", &B->Nf, junk, &newline);
         B->f0  = D->Nf;
         D->Nf += B->Nf;
         /* Allocate matrices */
         B->xi        = (double *)calloc(B->Nf, sizeof(double));
         B->eta       = (double *)calloc(B->Nf, sizeof(double));
         B->Mf        = CreateMatrix(B->Nf, B->Nf);
         B->Kf        = CreateMatrix(B->Nf, B->Nf);
         B->Cf        = CreateMatrix(B->Nf, B->Nf);
         B->Pf        = CreateMatrix(3, B->Nf);
         B->Hf        = CreateMatrix(3, B->Nf);
         B->CnbP      = CreateMatrix(3, B->Nf);
         B->HplusQeta = CreateMatrix(3, B->Nf);
         B->Qxi       = CreateMatrix(3, B->Nf);
         B->Rw        = CreateMatrix(3, B->Nf);
         B->Swe       = CreateMatrix(3, B->Nf);
         /* Allocate higher-order tensors */
         B->Qf = (double *)calloc(3 * B->Nf * B->Nf, sizeof(double));
         B->Rf = (double *)calloc(3 * B->Nf * 3, sizeof(double));
         B->Sf = (double *)calloc(3 * B->Nf * B->Nf * 3, sizeof(double));
         B->Sw = (double *)calloc(3 * B->Nf * B->Nf, sizeof(double));
         fclose(infile);
      }
   }
   /* For Joint Nodes */
   for (Ig = 0; Ig < S->Ng; Ig++) {
      G = &S->G[Ig];
      B = &S->B[G->Bin];
      if (B->Nf > 0) {
         G->PSIi   = CreateMatrix(3, B->Nf);
         G->THETAi = CreateMatrix(3, B->Nf);
      }
      B = &S->B[G->Bout];
      if (B->Nf > 0) {
         G->PSIo   = CreateMatrix(3, B->Nf);
         G->THETAo = CreateMatrix(3, B->Nf);
      }
   }
   D->PAngVelf          = CreateMatrix(3 * S->Nb, D->Nf);
   D->IPAngVelf         = CreateMatrix(3 * S->Nb, D->Nf);
   D->PVelf             = CreateMatrix(3 * S->Nb, D->Nf);
   D->mPVelf            = CreateMatrix(3 * S->Nb, D->Nf);
   D->uf                = (double *)calloc(D->Nf, sizeof(double));
   D->uuf               = (double *)calloc(D->Nf, sizeof(double));
   D->duf               = (double *)calloc(D->Nf, sizeof(double));
   D->ufdot             = (double *)calloc(D->Nf, sizeof(double));
   D->xf                = (double *)calloc(D->Nf, sizeof(double));
   D->xxf               = (double *)calloc(D->Nf, sizeof(double));
   D->dxf               = (double *)calloc(D->Nf, sizeof(double));
   D->xfdot             = (double *)calloc(D->Nf, sizeof(double));
   D->FlexAcc           = (double *)calloc(D->Nf, sizeof(double));
   D->FlexFrc           = (double *)calloc(D->Nf, sizeof(double));
   D->FlexInertiaFrc    = (double *)calloc(D->Nf, sizeof(double));
   D->Mf                = CreateMatrix(D->Nf, D->Nf);
   D->PCPVelf           = CreateMatrix(D->Nf, D->Nf);
   D->HplusQetaPAngVelf = CreateMatrix(D->Nf, D->Nf);

   /* .. Second pass through all flex files to read in values */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      if (strcmp(B->FlexFileName, "NONE")) {
         infile = FileOpen(InOutPath, B->FlexFileName, "r");
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);

         /* Initial Modal States x, u */
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         for (If = 0; If < B->Nf; If++) {
            fscanf(infile, "%lf %lf %[^\n] %[\n]", &B->eta[If], &B->xi[If],
                   junk, &newline);
            D->xf[B->f0 + If] = B->eta[If];
            D->uf[B->f0 + If] = B->xi[If];
         }

         /* Node-related matrices */
         for (In = 0; In < B->NumNodes; In++) {
            FN        = &B->Node[In];
            FN->PSI   = CreateMatrix(3, B->Nf);
            FN->THETA = CreateMatrix(3, B->Nf);
            for (i = 0; i < 3; i++) {
               FN->Frc[i] = 0.0;
               FN->Trq[i] = 0.0;
            }
            FN->FlexFrc = (double *)calloc(B->Nf, sizeof(double));
         }

         /**** Joint Node Mode Shapes ****/
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         /* Non-zero Translation Mode Shape (PSI) Elements */
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%ld %[^\n] %[\n]", &Nnonzero, junk, &newline);
         for (Iz = 0; Iz < Nnonzero; Iz++) {
            fscanf(infile, "%ld %ld %ld %lf %[^\n] %[\n]", &Im, &Ig, &Ia,
                   &value, junk, &newline);
            if (Ig >= S->Ng) {
               fprintf(stderr,
                       "Error in InitFlexModes: Joint %ld out of range\n", Ig);
               exit(EXIT_FAILURE);
            }
            if (Ia > 2) {
               fprintf(stderr,
                       "Error in InitFlexModes (PSI): Axis %ld out of range\n",
                       Ia);
               exit(EXIT_FAILURE);
            }
            if (Im >= B->Nf) {
               fprintf(
                   stderr,
                   "Error in InitFlexModes (PSI): Flex Mode %ld out of range\n",
                   Im);
               exit(EXIT_FAILURE);
            }
            G = &S->G[Ig];
            if (Ib == G->Bin)
               G->PSIi[Ia][Im] = value;
            else if (Ib == G->Bout)
               G->PSIo[Ia][Im] = value;
            else {
               fprintf(
                   stderr,
                   "Error in InitFlexModes (PSI): Body %ld not connected to "
                   "Joint %ld\n",
                   Ib, Ig);
               exit(EXIT_FAILURE);
            }
         }
         /* Non-zero Rotation Mode Shape (THETA) Elements */
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%ld %[^\n] %[\n]", &Nnonzero, junk, &newline);
         for (Iz = 0; Iz < Nnonzero; Iz++) {
            fscanf(infile, "%ld %ld %ld %lf %[^\n] %[\n]", &Im, &Ig, &Ia,
                   &value, junk, &newline);
            if (Ig >= S->Ng) {
               fprintf(stderr,
                       "Error in InitFlexModes: Joint %ld out of range\n", Ig);
               exit(EXIT_FAILURE);
            }
            if (Ia > 2) {
               fprintf(
                   stderr,
                   "Error in InitFlexModes (THETA): Axis %ld out of range\n",
                   Ia);
               exit(EXIT_FAILURE);
            }
            if (Im >= B->Nf) {
               fprintf(stderr,
                       "Error in InitFlexModes (THETA): Flex Mode %ld out of "
                       "range\n",
                       Im);
               exit(EXIT_FAILURE);
            }
            G = &S->G[Ig];
            if (Ib == G->Bin)
               G->THETAi[Ia][Im] = value;
            else if (Ib == G->Bout)
               G->THETAo[Ia][Im] = value;
            else {
               fprintf(stderr,
                       "Error in InitFlexModes (THETA): Body %ld not connected "
                       "to Joint %ld\n",
                       Ib, Ig);
               exit(EXIT_FAILURE);
            }
         }

         /**** Analysis Node Mode Shapes ****/
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         /* Non-zero Translation Mode Shape (PSI) Elements */
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%ld %[^\n] %[\n]", &Nnonzero, junk, &newline);
         for (Iz = 0; Iz < Nnonzero; Iz++) {
            fscanf(infile, "%ld %ld %ld %lf %[^\n] %[\n]", &Im, &In, &Ia,
                   &value, junk, &newline);
            if (In > B->NumNodes - 1) {
               fprintf(stderr,
                       "Error in InitFlexModes (PSI):  Node %ld out of range\n",
                       In);
               exit(EXIT_FAILURE);
            }
            FN = &B->Node[In];
            if (Ia > 2) {
               fprintf(stderr,
                       "Error in InitFlexModes (PSI): Axis %ld out of range\n",
                       Ia);
               exit(EXIT_FAILURE);
            }
            if (Im >= B->Nf) {
               fprintf(
                   stderr,
                   "Error in InitFlexModes (PSI): Flex Mode %ld out of range\n",
                   Im);
               exit(EXIT_FAILURE);
            }
            FN->PSI[Ia][Im] = value;
         }

         /* Non-zero Rotation Mode Shape (THETA) Elements */
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%ld %[^\n] %[\n]", &Nnonzero, junk, &newline);
         for (Iz = 0; Iz < Nnonzero; Iz++) {
            fscanf(infile, "%ld %ld %ld %lf %[^\n] %[\n]", &Im, &In, &Ia,
                   &value, junk, &newline);
            FN = &B->Node[In];
            if (In > B->NumNodes - 1) {
               fprintf(
                   stderr,
                   "Error in InitFlexModes (THETA):  Node %ld out of range\n",
                   In);
               exit(EXIT_FAILURE);
            }
            if (Ia > 2) {
               fprintf(
                   stderr,
                   "Error in InitFlexModes (THETA): Axis %ld out of range\n",
                   Ia);
               exit(EXIT_FAILURE);
            }
            if (Im >= B->Nf) {
               fprintf(stderr,
                       "Error in InitFlexModes (THETA): Flex Mode %ld out of "
                       "range\n",
                       Im);
               exit(EXIT_FAILURE);
            }
            FN->THETA[Ia][Im] = value;
         }

         /* Non-zero Mass Matrix Elements */
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%ld %[^\n] %[\n]", &Nnonzero, junk, &newline);
         for (Iz = 0; Iz < Nnonzero; Iz++) {
            fscanf(infile, "%ld %ld %lf %[^\n] %[\n]", &i, &j, &value, junk,
                   &newline);
            if (i >= B->Nf || j >= B->Nf) {
               fprintf(stderr,
                       "Error in InitFlexModes: Mass Matrix index [%ld][%ld] "
                       "out of range\n",
                       i, j);
               exit(EXIT_FAILURE);
            }
            B->Mf[i][j] = value;
         }
         /* Non-zero Stiffness Matrix Elements */
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%ld %[^\n] %[\n]", &Nnonzero, junk, &newline);
         for (Iz = 0; Iz < Nnonzero; Iz++) {
            fscanf(infile, "%ld %ld %lf %[^\n] %[\n]", &i, &j, &value, junk,
                   &newline);
            if (i >= B->Nf || j >= B->Nf) {
               fprintf(stderr,
                       "Error in InitFlexModes: Stiffness Matrix index "
                       "[%ld][%ld] out of range\n",
                       i, j);
               exit(EXIT_FAILURE);
            }
            B->Kf[i][j] = value;
         }
         /* Non-zero Damping Matrix Elements */
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%ld %[^\n] %[\n]", &Nnonzero, junk, &newline);
         for (Iz = 0; Iz < Nnonzero; Iz++) {
            fscanf(infile, "%ld %ld %lf %[^\n] %[\n]", &i, &j, &value, junk,
                   &newline);
            if (i >= B->Nf || j >= B->Nf) {
               fprintf(
                   stderr,
                   "Error in InitFlexModes: Damping Matrix index [%ld][%ld] "
                   "out of range\n",
                   i, j);
               exit(EXIT_FAILURE);
            }
            B->Cf[i][j] = value;
         }

         /* Check modal frequencies to make sure DTSIM is small enough */
         for (i = 0; i < B->Nf; i++) {
            wf = sqrt(B->Kf[i][i] / B->Mf[i][i]);
            if (Pi / wf < DTSIM) {
               fprintf(
                   stderr,
                   "Oops.  Natural frequency of Flex Mode %ld of Body %ld of "
                   "SC %ld is too high to be sampled at time step of %lf.\n",
                   i, Ib, S->ID, DTSIM);
               fprintf(stderr, "Suggest setting DTSIM < %lf sec\n",
                       0.2 * TwoPi / wf); /* 5 samples/cycle */
               exit(EXIT_FAILURE);
            }
         }

         /* Linear Momentum Modal Integral, Pf, 3 x Nf */
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         for (Im = 0; Im < B->Nf; Im++) {
            fscanf(infile, "%lf %lf %lf %[^\n] %[\n]", &B->Pf[0][Im],
                   &B->Pf[1][Im], &B->Pf[2][Im], junk, &newline);
         }
         /* Angular Momentum Modal Integral, Hf, 3 x Nf */
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         for (Im = 0; Im < B->Nf; Im++) {
            fscanf(infile, "%lf %lf %lf %[^\n] %[\n]", &B->Hf[0][Im],
                   &B->Hf[1][Im], &B->Hf[2][Im], junk, &newline);
         }
         /* Don't trust input file to make Hf, and Pf be zero */
         if (S->RefPt == REFPT_CM) {
            for (Im = 0; Im < B->Nf; Im++) {
               B->Pf[0][Im] = 0.0;
               B->Pf[1][Im] = 0.0;
               B->Pf[2][Im] = 0.0;
               B->Hf[0][Im] = 0.0;
               B->Hf[1][Im] = 0.0;
               B->Hf[2][Im] = 0.0;
            }
         }

         /* Non-zero Elements of Linear Modal Integral, L, 3 x 3 x Nf */
         L = (double ***)calloc(3, sizeof(double **));
         for (i = 0; i < 3; i++)
            L[i] = CreateMatrix(3, B->Nf);
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%ld %[^\n] %[\n]", &Nnonzero, junk, &newline);
         for (Iz = 0; Iz < Nnonzero; Iz++) {
            fscanf(infile, "%ld %ld %ld %lf %[^\n] %[\n]", &i, &j, &Im, &value,
                   junk, &newline);
            if (i >= 3 || j >= 3 || Im >= B->Nf) {
               fprintf(stderr,
                       "Error in InitFlexModes: L index [%ld][%ld][%ld] out of "
                       "range\n",
                       i, j, Im);
               exit(EXIT_FAILURE);
            }
            L[i][j][Im] = value;
         }
         /* B->Rf, 3 x Nf x 3 */
         for (i = 0; i < B->Nf; i++) {
            B->Rf[IDX3(0, i, 0, B->Nf, 3)] = -L[1][1][i] - L[2][2][i];
            B->Rf[IDX3(1, i, 1, B->Nf, 3)] = -L[2][2][i] - L[0][0][i];
            B->Rf[IDX3(2, i, 2, B->Nf, 3)] = -L[0][0][i] - L[1][1][i];
            B->Rf[IDX3(0, i, 1, B->Nf, 3)] = L[1][0][i];
            B->Rf[IDX3(0, i, 2, B->Nf, 3)] = L[2][0][i];
            B->Rf[IDX3(1, i, 0, B->Nf, 3)] = L[0][1][i];
            B->Rf[IDX3(1, i, 2, B->Nf, 3)] = L[2][1][i];
            B->Rf[IDX3(2, i, 0, B->Nf, 3)] = L[0][2][i];
            B->Rf[IDX3(2, i, 1, B->Nf, 3)] = L[1][2][i];
         }
         for (i = 0; i < 3; i++)
            DestroyMatrix(L[i]);
         free(L);

         /* Non-zero Elements of Angular Modal Integral, N, 3 x 3 x Nf x Nf*/
         N = (double ****)calloc(3, sizeof(double ***));
         for (i = 0; i < 3; i++) {
            N[i] = (double ***)calloc(3, sizeof(double **));
            for (j = 0; j < 3; j++) {
               N[i][j] = CreateMatrix(B->Nf, B->Nf);
            }
         }
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%ld %[^\n] %[\n]", &Nnonzero, junk, &newline);
         for (Iz = 0; Iz < Nnonzero; Iz++) {
            fscanf(infile, "%ld %ld %ld %ld %lf %[^\n] %[\n]", &i, &j, &Im, &Jm,
                   &value, junk, &newline);
            if (i >= 3 || j >= 3 || Im >= B->Nf || Jm >= B->Nf) {
               fprintf(stderr,
                       "Error in InitFlexModes: N index [%ld][%ld][%ld][%ld] "
                       "out of range\n",
                       i, j, Im, Jm);
               exit(EXIT_FAILURE);
            }
            N[i][j][Im][Jm] = value;
         }
         for (i = 0; i < B->Nf; i++) {
            for (j = 0; j < B->Nf; j++) {
               /* B->Qf, 3 x Nf x Nf */
               B->Qf[IDX3(0, i, j, B->Nf, B->Nf)] =
                   N[2][1][i][j] - N[1][2][i][j];
               B->Qf[IDX3(1, i, j, B->Nf, B->Nf)] =
                   N[0][2][i][j] - N[2][0][i][j];
               B->Qf[IDX3(2, i, j, B->Nf, B->Nf)] =
                   N[1][0][i][j] - N[0][1][i][j];
               /* B->Sf, 3 x Nf x Nf x 3 */
               B->Sf[IDX4(0, i, j, 0, B->Nf, B->Nf, 3)] =
                   -N[1][1][i][j] - N[2][2][i][j];
               B->Sf[IDX4(1, i, j, 1, B->Nf, B->Nf, 3)] =
                   -N[2][2][i][j] - N[0][0][i][j];
               B->Sf[IDX4(2, i, j, 2, B->Nf, B->Nf, 3)] =
                   -N[0][0][i][j] - N[1][1][i][j];
               B->Sf[IDX4(0, i, j, 1, B->Nf, B->Nf, 3)] = N[1][0][i][j];
               B->Sf[IDX4(0, i, j, 2, B->Nf, B->Nf, 3)] = N[2][0][i][j];
               B->Sf[IDX4(1, i, j, 0, B->Nf, B->Nf, 3)] = N[0][1][i][j];
               B->Sf[IDX4(1, i, j, 2, B->Nf, B->Nf, 3)] = N[2][1][i][j];
               B->Sf[IDX4(2, i, j, 0, B->Nf, B->Nf, 3)] = N[0][2][i][j];
               B->Sf[IDX4(2, i, j, 1, B->Nf, B->Nf, 3)] = N[1][2][i][j];
            }
         }
         for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++)
               DestroyMatrix(N[i][j]);
            free(N[i]);
         }
         free(N);

         fclose(infile);
      }
   }

   /* .. Assemble Dyn.Mf */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      for (i = 0; i < B->Nf; i++) {
         for (j = 0; j < B->Nf; j++) {
            D->Mf[B->f0 + i][B->f0 + j] = B->Mf[i][j];
         }
      }
   }

   /* .. Find if Mf is Diagonal */
   for (Ib = 0; Ib < S->Nb; Ib++) {
      B               = &S->B[Ib];
      B->MfIsDiagonal = TRUE;
      for (i = 0; i < B->Nf; i++) {
         for (j = 0; j < B->Nf; j++) {
            if (i != j && B->Mf[i][j] != 0.0)
               B->MfIsDiagonal = FALSE;
         }
      }
   }

   /* .. Bypass flex computations if FlexActive is FALSE */
   if (!S->FlexActive)
      D->Nf = 0;
   /* .. If FlexActive is TRUE, but no flex modes have been defined... */
   if (S->FlexActive && D->Nf == 0) {
      S->FlexActive = FALSE;
      printf("FlexActive set TRUE, but no flex modes defined.\n");
      printf("   Setting FlexActive to FALSE.\n");
   }
}
/**********************************************************************/
void InitNodes(struct BodyType *B)
{
   // TODO: put node sequence in relevant spacecraft file
   if (strcmp(B->NodeFileName, "NONE")) {
      char fileName[40] = {0};
      strcpy(fileName, B->NodeFileName);
      // Replace ".txt" with ".yaml"
      char *typeStr = strstr(fileName, ".txt");
      if (typeStr != NULL)
         strcpy(typeStr, ".yaml");

      struct fy_document *fyd =
          fy_document_build_and_check(NULL, InOutPath, fileName);
      struct fy_node *root = fy_document_root(fyd);
      struct fy_node *node = fy_node_by_path_def(root, "/Nodes");
      B->NumNodes          = fy_node_sequence_item_count(node);
      B->Node = (struct NodeType *)calloc(B->NumNodes, sizeof(struct NodeType));

      struct fy_node *iterNode = NULL;
      WHILE_FY_ITER(node, iterNode)
      {
         struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/Node");
         long In                 = 0;
         if (!fy_node_scanf(seqNode, "/Index %ld", &In)) {
            fprintf(stderr, "Could not find index for node. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         struct NodeType *N = &B->Node[In];
         if (!fy_node_scanf(seqNode, "/Comment %79[^\n]s", N->comment)) {
            fprintf(stderr, "Could not find comment for node. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         assignYAMLToDoubleArray(3, fy_node_by_path_def(seqNode, "/Location"),
                                 N->NomPosB);
      }
      fy_document_destroy(fyd);
   }
   else {
      /* Default to one node at B.cm */
      B->NumNodes = 1;
      B->Node     = (struct NodeType *)calloc(1, sizeof(struct NodeType));
      for (int i = 0; i < 3; i++)
         B->Node[0].PosB[i] = B->cm[i];
      strcpy(B->Node[0].comment, "Mass Center");
   }
}
/**********************************************************************/
void InitPassiveJoint(struct JointType *G, struct SCType *S)
{
   FILE *infile;
   char junk[80], newline;
   long i;

   for (i = 0; i < 3; i++) {
      G->RotSpringCoef[i] = 0.0;
      G->RotDampCoef[i]   = 0.0;
      G->TrnSpringCoef[i] = 0.0;
      G->TrnDampCoef[i]   = 0.0;
   }
   if (strcmp(G->ParmFileName, "NONE")) {
      infile = FileOpen(InOutPath, G->ParmFileName, "r");
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%lf %lf %lf %[^\n] %[\n]", &G->RotSpringCoef[0],
             &G->RotSpringCoef[1], &G->RotSpringCoef[2], junk, &newline);
      fscanf(infile, "%lf %lf %lf %[^\n] %[\n]", &G->RotDampCoef[0],
             &G->RotDampCoef[1], &G->RotDampCoef[2], junk, &newline);
      fscanf(infile, "%lf %lf %lf %[^\n] %[\n]", &G->TrnSpringCoef[0],
             &G->TrnSpringCoef[1], &G->TrnSpringCoef[2], junk, &newline);
      fscanf(infile, "%lf %lf %lf %[^\n] %[\n]", &G->TrnDampCoef[0],
             &G->TrnDampCoef[1], &G->TrnDampCoef[2], junk, &newline);
      fclose(infile);
   }
}
/**********************************************************************/
void InitActuatedJoint(struct JointType *G, struct SCType *S)
{
   long i;

   for (i = 0; i < 3; i++) {
      G->MaxTrq[i]      = 10.0;
      G->MaxAngRate[i]  = 1.0 * D2R;
      G->AngRateGain[i] = G->MaxTrq[i] / G->MaxAngRate[i];
      G->MaxFrc[i]      = 10.0;
      G->MaxPosRate[i]  = 0.01;
      G->PosRateGain[i] = G->MaxFrc[i] / G->MaxPosRate[i];
   }
}
/**********************************************************************/
void InitShakers(struct SCType *S)
{
   FILE *infile;
   long Ish, It;
   struct ShakerType *Sh;
   char response[80], junk[80], newline;

   if (strcmp(S->ShakerFileName, "NONE")) {

      infile = FileOpen(InOutPath, S->ShakerFileName, "r");

      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%ld %[^\n] %[\n]", &S->Nsh, junk, &newline);
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);

      S->Shaker =
          (struct ShakerType *)calloc(S->Nsh, sizeof(struct ShakerType));
      for (Ish = 0; Ish < S->Nsh; Ish++) {
         Sh = &S->Shaker[Ish];
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%ld %ld %[^\n] %[\n]", &Sh->Body, &Sh->Node, junk,
                &newline);
         fscanf(infile, "%s %[^\n] %[\n]", response, junk, &newline);
         Sh->FrcTrq = DecodeString(response);
         fscanf(infile, "%lf %lf %lf %[^\n] %[\n]", &Sh->Axis[0], &Sh->Axis[1],
                &Sh->Axis[2], junk, &newline);
         UNITV(Sh->Axis);
         fscanf(infile, "%ld %[^\n] %[\n]", &Sh->Ntone, junk, &newline);
         if (Sh->Ntone == 0) {
            fscanf(infile, "%[^\n] %[\n]", junk, &newline);
            fscanf(infile, "%[^\n] %[\n]", junk, &newline);
            fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         }
         else {
            Sh->ToneAmp   = (double *)calloc(Sh->Ntone, sizeof(double));
            Sh->ToneFreq  = (double *)calloc(Sh->Ntone, sizeof(double));
            Sh->TonePhase = (double *)calloc(Sh->Ntone, sizeof(double));
            for (It = 0; It < Sh->Ntone; It++) {
               fscanf(infile, "%lf %[^\n] %[\n]", &Sh->ToneAmp[It], junk,
                      &newline);
               fscanf(infile, "%lf %[^\n] %[\n]", &Sh->ToneFreq[It], junk,
                      &newline);
               fscanf(infile, "%lf %[^\n] %[\n]", &Sh->TonePhase[It], junk,
                      &newline);
               Sh->ToneFreq[It]  *= TwoPi;
               Sh->TonePhase[It] *= D2R;
            }
         }
         fscanf(infile, "%s %[^\n] %[\n]", response, junk, &newline);
         Sh->RandomActive = DecodeString(response);
         if (!Sh->RandomActive) {
            fscanf(infile, "%[^\n] %[\n]", junk, &newline);
            fscanf(infile, "%[^\n] %[\n]", junk, &newline);
            Sh->RandomProc = NULL;
            Sh->Lowpass    = NULL;
            Sh->Highpass   = NULL;
         }
         else {
            fscanf(infile, "%lf %lf %[^\n] %[\n]", &Sh->LowBandLimit,
                   &Sh->HighBandLimit, junk, &newline);
            fscanf(infile, "%lf %[^\n] %[\n]", &Sh->RandStd, junk, &newline);
            Sh->RandStd       /= Sh->HighBandLimit - Sh->LowBandLimit;
            Sh->HighBandLimit *= TwoPi;
            Sh->LowBandLimit  *= TwoPi;

            /* Lowpass and Highpass overlap to form Bandpass */
            Sh->RandomProc = CreateRandomProcess(RngSeed + Ish);
            Sh->Lowpass = CreateSecondOrderLowpassFilter(Sh->HighBandLimit, 1.0,
                                                         DTSIM, 1.0E6, 1.0E-12);
            if (Sh->LowBandLimit > 0.0)
               Sh->Highpass = CreateSecondOrderLowpassFilter(
                   Sh->LowBandLimit, 1.0, DTSIM, 1.0E6, 1.0E-12);
         }
      }
      fclose(infile);
   }
}
/**********************************************************************/
void InitWhlDragAndJitter(struct WhlType *W)
{
   FILE *infile;
   struct WhlHarmType *H;
   char junk[80], newline;
   double Stiction, LugrePeriod, w;
   long Ih;

   if (strcmp(W->DragJitterFileName, "NONE")) {
      infile = FileOpen(InOutPath, W->DragJitterFileName, "r");
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      /* Drag Parameters */
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%lf  %[^\n] %[\n]", &W->CoulCoef, junk, &newline);
      fscanf(infile, "%lf  %[^\n] %[\n]", &Stiction, junk, &newline);
      W->StribeckCoef = Stiction - W->CoulCoef;
      if (W->StribeckCoef < 0.0) {
         fprintf(
             stderr,
             "Error: Stiction < Coulomb friction in %s.  Better fix that.\n",
             W->DragJitterFileName);
         exit(EXIT_FAILURE);
      }
      fscanf(infile, "%lf  %[^\n] %[\n]", &W->ViscCoef, junk, &newline);
      fscanf(infile, "%lf  %[^\n] %[\n]", &W->StribeckZone, junk, &newline);
      fscanf(infile, "%lf  %[^\n] %[\n]", &LugrePeriod, junk, &newline);
      w                  = TwoPi / LugrePeriod;
      W->LugreSpringCoef = W->J * w * w;
      W->LugreDampCoef   = 2.0 * W->J * w; /* Critical damping assumed */
      fscanf(infile, "%lf  %[^\n] %[\n]", &W->LugreDampZone, junk, &newline);
      W->z = 0.0;

      /* Jitter Parameters */
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%lf  %[^\n] %[\n]", &W->gamma, junk, &newline);
      W->Jt = 0.5 * W->gamma * W->J;
      fscanf(infile, "%lf  %[^\n] %[\n]", &W->ImbPhase, junk, &newline);
      W->ImbPhase *= D2R;
      fscanf(infile, "%lf %lf %[^\n] %[\n]", &W->LatFreq, &W->LatDamp, junk,
             &newline);
      W->LatFreq *= TwoPi;
      fscanf(infile, "%lf %lf %[^\n] %[\n]", &W->RockFreq, &W->RockDamp, junk,
             &newline);
      W->RockFreq *= TwoPi;
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%ld %[^\n] %[\n]", &W->NumHarm, junk, &newline);
      if (W->NumHarm > 0) {
         W->Harm = (struct WhlHarmType *)calloc(W->NumHarm,
                                                sizeof(struct WhlHarmType));
         for (Ih = 0; Ih < W->NumHarm; Ih++) {
            H = &W->Harm[Ih];
            fscanf(infile, "%[^\n] %[\n]", junk, &newline);
            fscanf(infile, "%lf  %[^\n] %[\n]", &H->n, junk, &newline);
            fscanf(infile, "%lf  %[^\n] %[\n]", &H->Ks, junk, &newline);
            H->Ks *= 1.0E-3 * 0.01;
            fscanf(infile, "%lf  %[^\n] %[\n]", &H->Kd, junk, &newline);
            H->Kd *= 1.0E-3 * 1.0E-4;
            fscanf(infile, "%lf  %[^\n] %[\n]", &H->phase, junk, &newline);
            H->phase *= D2R;
         }
      }
      fclose(infile);
   }
}
/**********************************************************************/
void InitOptics(struct FgsType *F)
{
   FILE *infile;
   // struct SCType *S;
   // struct BodyType *B;
   // struct NodeType *N;
   struct OpticsType *O;
   char junk[256], newline, response[120];
   // double ApPntN[3], FocPntN[3], DetPntN[3], RelPosN[3];
   long Io;
   // long i;
   // long HasFocus;

   if (strcmp(F->OpticsFileName, "NONE")) {
      infile = FileOpen(InOutPath, F->OpticsFileName, "r");
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%ld  %[^\n] %[\n]", &F->Nopt, junk, &newline);
      F->Opt = (struct OpticsType *)calloc(F->Nopt, sizeof(struct OpticsType));
      for (Io = 0; Io < F->Nopt; Io++) {
         O = &F->Opt[Io];
         fscanf(infile, "%[^\n] %[\n]", junk, &newline);
         fscanf(infile, "%ld %ld %ld %[^\n] %[\n]", &O->SC, &O->Body, &O->Node,
                junk, &newline);
         fscanf(infile, "%lf %[^\n] %[\n]", &O->ApRad, junk, &newline);
         O->ApRad /= 2.0;
         fscanf(infile, "%lf %lf %lf %[^\n] %[\n]", &O->Axis[0], &O->Axis[1],
                &O->Axis[2], junk, &newline);
         UNITV(O->Axis);
         fscanf(infile, "%s %[^\n] %[\n]", response, junk, &newline);
         O->Type = DecodeString(response);
         fscanf(infile, "%s %[^\n] %[\n]", response, junk, &newline);
         O->ConicSign = (double)DecodeString(response);
         fscanf(infile, "%lf %[^\n] %[\n]", &O->FocLen, junk, &newline);
         fscanf(infile, "%lf %[^\n] %[\n]", &O->ConicConst, junk, &newline);
      }
      fclose(infile);
      F->HasOptics = TRUE;

      /* Check for Aperture, Detector */
      if (F->Opt[0].Type != OPT_APERTURE) {
         fprintf(stderr,
                 "Optical Train must have Aperture as first element.\n");
         exit(EXIT_FAILURE);
      }
      if (F->Opt[F->Nopt - 1].Type != OPT_DETECTOR) {
         printf("Optical Train must have Detector as last element.\n");
      }

      /* TODO: Find erect/inverted, lump in Det->FocLen */
   }
   else {
      F->HasOptics = FALSE;
   }

   printf("Exiting InitOptics\n");
}
/**********************************************************************/
void InitOrderNDynamics(struct SCType *S)
{
   struct BodyType *B;
   struct JointType *G;
   long Ib, Ig, Id, i, j;

   G              = &S->GN;
   G->Init        = 1;
   G->IsSpherical = 1;
   G->RotDOF      = 3;
   G->TrnDOF      = 3;
   G->Bo          = &S->B[0];
   G->RotSeq      = 123;
   G->TrnSeq      = 123;
   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         G->CGiBi[i][j]     = 0.0;
         G->CBoGo[i][j]     = 0.0;
         G->Pw[i][j]        = 0.0;
         G->Pv[i][j]        = 0.0;
         G->Pwdot[i][j]     = 0.0;
         G->P[i][j]         = 0.0;
         G->P[i][3 + j]     = 0.0;
         G->P[3 + i][j]     = 0.0;
         G->P[3 + i][3 + i] = 0.0;
      }
      G->CGiBi[i][i]     = 1.0;
      G->CBoGo[i][i]     = 1.0;
      G->Pw[i][i]        = 1.0;
      G->Pv[i][i]        = 1.0;
      G->P[i][i]         = 1.0;
      G->P[3 + i][3 + i] = 1.0;
   }
   G->Nu = 6;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B     = &S->B[Ib];
      B->Nd = 0;
   }

   for (Ig = 0; Ig < S->Ng; Ig++) {
      G       = &S->G[Ig];
      G->Init = 1;
      G->Bi   = &S->B[G->Bin];
      G->Bo   = &S->B[G->Bout];
      G->Bi->Nd++;
      G->Nu = G->RotDOF + G->TrnDOF;
   }

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];
      if (B->Nd > 0) {
         B->Gd = (long *)calloc(B->Nd, sizeof(long));
      }
      Id = 0;
      for (Ig = 0; Ig < S->Ng; Ig++) {
         G = &S->G[Ig];
         if (G->Bin == Ib) {
            B->Gd[Id] = Ig;
            Id++;
         }
      }
   }
}
/**********************************************************************/
void InitSpacecraft(struct SCType *S)
{
   long i, j, k;

   char fileName[50];
   strcpy(fileName, S->FileName);
   // Replace ".txt" with ".yaml"
   char *typeStr = strstr(fileName, ".txt");
   if (typeStr != NULL)
      strcpy(typeStr, ".yaml");

   struct fy_document *fyd =
       fy_document_build_and_check(NULL, InOutPath, fileName);

   struct fy_node *root = fy_document_root(fyd);
   struct fy_node *node = NULL, *iterNode = NULL;
   node           = fy_node_by_path_def(root, "/Configuration");
   char dummy[50] = {0};
   if (fy_node_scanf(node,
                     "/Label %39s "
                     "/Sprite File %39s "
                     "/FSW Identifier %49s "
                     "/FSW Sample Time %lf",
                     S->Label, S->SpriteFileName, dummy,
                     &S->FswSampleTime) != 4) {
      fprintf(
          stderr,
          "Could not find spacecraft Configuration information. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   S->FswTag        = DecodeString(dummy);
   S->FswMaxCounter = (long)(S->FswSampleTime / DTSIM + 0.5);
   if (S->FswSampleTime < DTSIM) {
      fprintf(stderr, "Error:  FswSampleTime smaller than DTSIM.\n");
      exit(EXIT_FAILURE);
   }
   S->FswSampleCounter = S->FswMaxCounter;
   S->InitAC           = 1;
   S->InitDSM          = 1;

   node = fy_node_by_path_def(root, "/Orbit");
   if (!fy_node_scanf(node, "/Prop Type %49s", dummy)) {
      fprintf(stderr,
              "Could not find propagation type for spacecraft. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   S->OrbDOF = DecodeString(dummy);
   if (!fy_node_scanf(node, "/Pos Specifier %49s", dummy)) {
      fprintf(stderr,
              "Could not find Position Specifier for spacecraft. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   long useCM = DecodeString(dummy);
   double posVec[3], velVec[3];
   assignYAMLToDoubleArray(3, fy_node_by_path_def(node, "/Pos wrt F"), posVec);
   assignYAMLToDoubleArray(3, fy_node_by_path_def(node, "/Vel wrt F"), velVec);

   node = fy_node_by_path_def(root, "/Attitude");
   char rateFrame, attParm, attFrame;
   double wbn[3], ang[3], qbn[4], CBN[3][3];
   long seq;
   if (fy_node_scanf(node,
                     "Ang Vel Frame %c "
                     "Att Representation %c "
                     "Att Frame %c",
                     &rateFrame, &attParm, &attFrame) != 3) {
      fprintf(stderr,
              "Could not find spacecraft Attitude information. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   assignYAMLToDoubleArray(3, fy_node_by_path_def(node, "/Ang Vel"), wbn);
   for (i = 0; i < 3; i++)
      wbn[i] *= D2R;
   if (attParm == 'Q') {
      assignYAMLToDoubleArray(4, fy_node_by_path_def(node, "/Quaternion"), qbn);
      Q2C(qbn, CBN);
   }
   else {
      getYAMLEulerAngles(fy_node_by_path_def(node, "/Euler Angles"), ang, &seq);
      A2C(seq, ang[0] * D2R, ang[1] * D2R, ang[2] * D2R, CBN);
      C2Q(CBN, qbn);
   }
   switch (attFrame) {
      case 'L': {
         double CBL[3][3];
         /* Adjust CBN */
         for (j = 0; j < 3; j++) {
            for (k = 0; k < 3; k++)
               CBL[j][k] = CBN[j][k];
         }
         MxM(CBL, Orb[S->RefOrb].CLN, CBN);
         C2Q(CBN, qbn);
      } break;
      case 'F': {
         double CBF[3][3];
         /* Adjust CBN */
         for (j = 0; j < 3; j++) {
            for (k = 0; k < 3; k++)
               CBF[j][k] = CBN[j][k];
         }
         MxM(CBF, Frm[S->RefOrb].CN, CBN);
         C2Q(CBN, qbn);
      } break;
      case 'E': {
         /* Adjust CBN */
         double CBE[3][3], CEN[3][3];
         for (j = 0; j < 3; j++) {
            for (k = 0; k < 3; k++)
               CBE[j][k] = CBN[j][k];
         }
         FindCEN(Orb[S->RefOrb].PosN, CEN);
         MxM(CBE, CEN, CBN);
         C2Q(CBN, qbn);
      } break;
   }
   if (rateFrame == 'L') {
      /* Add LVLH rate to wn */
      double wlnb[3];
      MxV(CBN, Orb[S->RefOrb].wln, wlnb);
      for (j = 0; j < 3; j++)
         wbn[j] += wlnb[j];
   }
   MxMT(CBN, Frm[S->RefOrb].CN, S->CF);

   node = fy_node_by_path_def(root, "/Dynamics Flags");
   if (!fy_node_scanf(node, "/Method %49s", dummy)) {
      fprintf(stderr,
              "Could not find spacraft propagation method. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   S->DynMethod = DecodeString(dummy);
   if (!fy_node_scanf(node, "/Mass Reference Point %49s", dummy)) {
      fprintf(stderr,
              "Could not find spacecraft mass reference point. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   S->RefPt = DecodeString(dummy);
   if (fy_node_scanf(node,
                     "/Drag Coefficient %lf "
                     "/Shaker File Name %39[^\n]",
                     &S->DragCoef, S->ShakerFileName) != 2) {
      fprintf(stderr, "Could not find Drag Coefficient or Shaker File Name for "
                      "spacecraft. Exiting...\n");
      exit(EXIT_FAILURE);
   }

   S->ConstraintsRequested =
       getYAMLBool(fy_node_by_path_def(node, "/Compute Constraints"));
   S->FlexActive = getYAMLBool(fy_node_by_path_def(node, "/Flex Active"));
   S->IncludeSecondOrderFlexTerms =
       getYAMLBool(fy_node_by_path_def(node, "/2nd Order Flex"));
   node  = fy_node_by_path_def(root, "/Bodies");
   S->Nb = fy_node_sequence_item_count(node);
   S->Ng = S->Nb - 1;
   S->B  = (struct BodyType *)calloc(S->Nb, sizeof(struct BodyType));
   if (S->B == NULL) {
      fprintf(stderr, "S->B calloc returned null pointer.  Bailing out!\n");
      exit(EXIT_FAILURE);
   }
   if (S->Ng == 0)
      S->G = NULL;
   else {
      S->G = (struct JointType *)calloc(S->Ng, sizeof(struct JointType));
      if (S->G == NULL) {
         fprintf(stderr, "S->G calloc returned null pointer.  Bailing out!\n");
         exit(EXIT_FAILURE);
      }
   }

   /* Load B[0] initial attitude */
   for (j = 0; j < 3; j++) {
      S->B[0].wn[j] = wbn[j];
      for (k = 0; k < 3; k++)
         S->B[0].CN[j][k] = CBN[j][k];
   }
   for (j = 0; j < 4; j++)
      S->B[0].qn[j] = qbn[j];

   /* .. Body Ib */
   iterNode = NULL;
   WHILE_FY_ITER(node, iterNode)
   {
      long Ib;
      struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/Body");
      if (!fy_node_scanf(seqNode, "/Index %ld", &Ib)) {
         fprintf(stderr, "Could not find Body index. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      struct BodyType *B = &S->B[Ib];
      double moi[3], poi[3];
      if (fy_node_scanf(seqNode,
                        "/Mass %lf "
                        "/Geometry File Name %39[^\n] "
                        "/Node File Name %39[^\n] "
                        "/Flex File Name %39[^\n]",
                        &B->mass, B->GeomFileName, B->NodeFileName,
                        B->FlexFileName) != 4) {
         fprintf(stderr,
                 "Could not find spacecraft body %ld configuration "
                 "information. Exiting...\n",
                 Ib);
         exit(EXIT_FAILURE);
      }
      assignYAMLToDoubleArray(3, fy_node_by_path_def(seqNode, "/MOI"), moi);
      assignYAMLToDoubleArray(3, fy_node_by_path_def(seqNode, "/POI"), poi);
      for (i = 0; i < 3; i++)
         B->I[i][i] = moi[i];
      B->I[0][1] = -poi[0];
      B->I[0][2] = -poi[1];
      B->I[1][2] = -poi[2];
      B->I[1][0] = B->I[0][1];
      B->I[2][0] = B->I[0][2];
      B->I[2][1] = B->I[1][2];
      assignYAMLToDoubleArray(3, fy_node_by_path_def(seqNode, "/Pos of CM"),
                              B->cm);
      assignYAMLToDoubleArray(
          3, fy_node_by_path_def(seqNode, "/Constant Momentum"),
          B->EmbeddedMom);
      assignYAMLToDoubleArray(3,
                              fy_node_by_path_def(seqNode, "/Constant Dipole"),
                              B->EmbeddedDipole);
      if (S->RefPt == REFPT_JOINT)
         for (i = 0; i < 3; i++)
            B->c[i] = B->mass * B->cm[i];
      else
         for (i = 0; i < 3; i++)
            B->c[i] = 0.0;

      InitNodes(B);
   }

   /* .. Joint Parameters */
   long SomeJointsLocked = FALSE;
   if (S->Ng > 0) {
      node     = fy_node_by_path_def(root, "/Joints");
      iterNode = NULL;
      WHILE_FY_ITER(node, iterNode)
      {
         long Ig;
         struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/Joint");
         if (!fy_node_scanf(seqNode, "/Index %ld", &Ig)) {
            fprintf(stderr,
                    "Could not find spacecraft Joint index. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         struct JointType *G = &S->G[Ig];

         if (!fy_node_scanf(seqNode, "/Joint Type %49s", dummy)) {
            fprintf(stderr,
                    "Could not find spacecraft Joint %ld's type. Exiting...\n",
                    Ig);
            exit(EXIT_FAILURE);
         }
         G->Type          = DecodeString(dummy);
         long bodyInds[2] = {0};
         assignYAMLToLongArray(
             2, fy_node_by_path_def(seqNode, "/Body Indicies"), bodyInds);
         G->Bin  = bodyInds[0];
         G->Bout = bodyInds[1];
         if (G->Bin > G->Bout) {
            fprintf(stderr,
                    "Yo!  SC[%ld].G[%ld] inner body index (%ld) is greater "
                    "than outer body index (%ld)\n",
                    S->ID, Ig, G->Bin, G->Bout);
            fprintf(stderr,
                    "You must define inner bodies before outer bodies!\n");
            exit(EXIT_FAILURE);
         }
         S->B[G->Bout].Gin = Ig;

         if (fy_node_scanf(seqNode,
                           "/Rot DOF %ld "
                           "/Rot Sequence %ld "
                           "/Rot Type %49s",
                           &G->RotDOF, &G->RotSeq, dummy) != 3) {
            fprintf(stderr,
                    "Could not find spacecraft Joint %ld's rotation "
                    "information. Exiting...\n",
                    Ig);
            exit(EXIT_FAILURE);
         }
         long i3 = G->RotSeq % 10;         /* Pick off third digit */
         long i2 = (G->RotSeq % 100) / 10; /* Extract second digit */
         long i1 = G->RotSeq / 100;        /* Pick off first digit */
         if (i1 == i2 || i1 == i3 || i2 == i3) {
            fprintf(stderr,
                    "Invalid RotSeq %ld for SC[%ld].G[%ld].  Repeated indices "
                    "are not allowed.\n",
                    G->RotSeq, S->ID, Ig);
            exit(EXIT_FAILURE);
         }

         if (fy_node_scanf(seqNode,
                           "/Trn DOF %ld "
                           "/Trn Sequence %ld",
                           &G->TrnDOF, &G->TrnSeq) != 2) {
            fprintf(stderr,
                    "Could not find spacecraft Joint %ld's translational "
                    "information. Exiting...\n",
                    Ig);
            exit(EXIT_FAILURE);
         }
         if (G->TrnSeq < 100) {
            fprintf(stderr,
                    "Invalid TrnSeq %ld for SC[%ld].G[%ld].  All three axes "
                    "required.\n",
                    G->TrnSeq, S->ID, Ig);
            exit(EXIT_FAILURE);
         }
         i3 = G->TrnSeq % 10;         /* Pick off third digit */
         i2 = (G->TrnSeq % 100) / 10; /* Extract second digit */
         i1 = G->TrnSeq / 100;        /* Pick off first digit */
         if (i1 == i2 || i1 == i3 || i2 == i3) {
            fprintf(stderr,
                    "Invalid TrnSeq %ld for SC[%ld].G[%ld].  Repeated indices "
                    "are not allowed.\n",
                    G->TrnSeq, S->ID, Ig);
            exit(EXIT_FAILURE);
         }

         assignYAMLToBoolArray(
             3, fy_node_by_path_def(seqNode, "/Rot DOF Locked"), G->RotLocked);
         assignYAMLToBoolArray(
             3, fy_node_by_path_def(seqNode, "/Trn DOF Locked"), G->TrnLocked);
         for (i = 0; i < 3; i++)
            SomeJointsLocked |= G->RotLocked[i] || G->TrnLocked[i];

         /* Load in initial angles and angular rates */
         assignYAMLToDoubleArray(
             3, fy_node_by_path_def(seqNode, "/Init Angles"), G->Ang);
         assignYAMLToDoubleArray(
             3, fy_node_by_path_def(seqNode, "/Init Angle Rates"), G->AngRate);
         for (k = 0; k < 3; k++) {
            G->Ang[k]     *= D2R;
            G->AngRate[k] *= D2R;
         }
         /* Protect against more inputs than RotDOF */
         for (k = G->RotDOF; k < 3; k++) {
            G->Ang[k]     = 0.0;
            G->AngRate[k] = 0.0;
         }
         /* Load in initial displacements and rates */
         assignYAMLToDoubleArray(
             3, fy_node_by_path_def(seqNode, "/Init Displacement"), G->Pos);
         assignYAMLToDoubleArray(
             3, fy_node_by_path_def(seqNode, "/Init Displacement Rates"),
             G->PosRate);
         /* Protect against more inputs than TrnDOF */
         for (k = G->TrnDOF; k < 3; k++) {
            G->Pos[k]     = 0.0;
            G->PosRate[k] = 0.0;
         }

         getYAMLEulerAngles(fy_node_by_path_def(seqNode, "/Bi-Gi Angles"), ang,
                            &seq);
         A2C(seq, ang[0] * D2R, ang[1] * D2R, ang[2] * D2R, G->CGiBi);
         getYAMLEulerAngles(fy_node_by_path_def(seqNode, "/Bo-Go Angles"), ang,
                            &seq);
         A2C(seq, ang[0] * D2R, ang[1] * D2R, ang[2] * D2R, G->CBoGo);

         double pIn[3], pOut[3];
         assignYAMLToDoubleArray(
             3, fy_node_by_path_def(seqNode, "/Pos wrt Inner Body"), pIn);
         assignYAMLToDoubleArray(
             3, fy_node_by_path_def(seqNode, "/Pos wrt Outer Body"), pOut);

         if (S->RefPt == REFPT_JOINT) {
            for (j = 0; j < 3; j++) {
               G->RigidRin[j]  = pIn[j];
               G->RigidRout[j] = pOut[j];
            }
         }
         else {
            for (j = 0; j < 3; j++) {
               G->RigidRin[j]  = pIn[j] - S->B[G->Bin].cm[j];
               G->RigidRout[j] = pOut[j] - S->B[G->Bout].cm[j];
            }
         }
         if (!fy_node_scanf(seqNode, "/Parm File Name %39[^\n]",
                            G->ParmFileName)) {
            fprintf(stderr,
                    "Could not find spacecraft Joint %ld's parameter file "
                    "name. Exiting...\n",
                    Ig);
            exit(EXIT_FAILURE);
         }

         if (G->Type == PASSIVE_JOINT)
            InitPassiveJoint(G, S);
         if (G->Type == ACTUATED_JOINT)
            InitActuatedJoint(G, S);
      }
   }

   /* .. Wheel parameters */
   S->WhlDragActive =
       getYAMLBool(fy_node_by_path_def(root, "/Wheel Params/Drag"));
   S->WhlJitterActive =
       getYAMLBool(fy_node_by_path_def(root, "/Wheel Params/Jitter"));

   node  = fy_node_by_path_def(root, "/Wheels");
   S->Nw = (node == NULL) ? 0 : fy_node_sequence_item_count(node);
   if (S->Nw > 0) {
      S->Whl   = (struct WhlType *)calloc(S->Nw, sizeof(struct WhlType));
      iterNode = NULL;
      WHILE_FY_ITER(node, iterNode)
      {
         long Iw                 = 0;
         struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/Wheel");
         if (!fy_node_scanf(seqNode, "/Index %ld", &Iw)) {
            fprintf(stderr,
                    "Could not find spacecraft Wheel index. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         struct WhlType *W = &S->Whl[Iw];
         assignYAMLToDoubleArray(3, fy_node_by_path_def(seqNode, "/Axis"),
                                 W->A);
         UNITV(W->A);
         PerpBasis(W->A, W->Uaxis, W->Vaxis);
         if (fy_node_scanf(seqNode,
                           "/Initial Momentum %lf "
                           "/Max Torque %lf "
                           "/Max Momentum %lf "
                           "/Rotor Inertia %lf "
                           "/Body/Index %ld "
                           "/Node %ld "
                           "/Drag-Jitter File Name %39s",
                           &W->H, &W->Tmax, &W->Hmax, &W->J, &W->Body, &W->Node,
                           W->DragJitterFileName) != 7) {
            fprintf(
                stderr,
                "Spacecraft Wheel %ld is improperly configured. Exiting...\n",
                Iw);
            exit(EXIT_FAILURE);
         }
         if (W->Node >= S->B[W->Body].NumNodes) {
            fprintf(stderr, "SC[%ld].Whl[%ld] Node out of range\n", S->ID, Iw);
            exit(EXIT_FAILURE);
         }
         InitWhlDragAndJitter(W);
      }
   }
   else
      S->Whl = NULL;

   /* .. MTB parameters */
   node    = fy_node_by_path_def(root, "/MTBs");
   S->Nmtb = (node == NULL) ? 0 : fy_node_sequence_item_count(node);
   if (S->Nmtb > 0) {
      S->MTB   = (struct MTBType *)calloc(S->Nmtb, sizeof(struct MTBType));
      iterNode = NULL;
      WHILE_FY_ITER(node, iterNode)
      {
         long Im                 = 0;
         struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/MTB");
         if (!fy_node_scanf(seqNode, "/Index %ld", &Im)) {
            fprintf(stderr,
                    "Could not find spacecraft MTB index. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         struct MTBType *MTB = &S->MTB[Im];
         assignYAMLToDoubleArray(3, fy_node_by_path_def(seqNode, "/Axis"),
                                 MTB->A);
         UNITV(MTB->A);
         if (fy_node_scanf(seqNode,
                           "/Saturation %lf "
                           "/Node %ld",
                           &MTB->Mmax, &MTB->Node) != 2) {
            fprintf(stderr,
                    "Could not find configuration for spacecraft MTB %ld. "
                    "Exiting...\n",
                    Im);
            exit(EXIT_FAILURE);
         }
         if (MTB->Node >= S->B[0].NumNodes) {
            fprintf(stderr, "SC[%ld].Whl[%ld] Node out of range\n", S->ID, Im);
            exit(EXIT_FAILURE);
         }
      }
   }
   else
      S->MTB = NULL;

   /* .. Thruster parameters */
   node    = fy_node_by_path_def(root, "/Thrusters");
   S->Nthr = (node == NULL) ? 0 : fy_node_sequence_item_count(node);
   if (S->Nthr > 0) {
      S->Thr   = (struct ThrType *)calloc(S->Nthr, sizeof(struct ThrType));
      iterNode = NULL;
      WHILE_FY_ITER(node, iterNode)
      {
         long It                 = 0;
         struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/Thruster");
         if (!fy_node_scanf(seqNode, "/Index %ld", &It)) {
            fprintf(stderr,
                    "Could not find spacecraft Thruster Index. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         struct ThrType *T = &S->Thr[It];
         assignYAMLToDoubleArray(3, fy_node_by_path_def(seqNode, "/Axis"),
                                 T->A);
         UNITV(T->A);
         if (fy_node_scanf(seqNode,
                           "/Mode %49s "
                           "/Force %lf "
                           "/Body/Index %ld "
                           "/Node %ld",
                           dummy, &T->Fmax, &T->Body, &T->Node) != 4) {
            fprintf(stderr,
                    "Could not find spacecraft Thruster %ld configuration. "
                    "Exiting...\n",
                    It);
            exit(EXIT_FAILURE);
         }
         T->Mode = DecodeString(dummy);
         if (T->Node >= S->B[T->Body].NumNodes) {
            fprintf(stderr, "SC[%ld].Thr[%ld] Node out of range\n", S->ID, It);
            exit(EXIT_FAILURE);
         }
      }
   }
   else
      S->Thr = NULL;

   /* .. Gyro parameters */
   node     = fy_node_by_path_def(root, "/Gyros");
   S->Ngyro = (node == NULL) ? 0 : fy_node_sequence_item_count(node);
   if (S->Ngyro > 0) {
      S->Gyro  = (struct GyroType *)calloc(S->Ngyro, sizeof(struct GyroType));
      iterNode = NULL;
      WHILE_FY_ITER(node, iterNode)
      {
         long Ig                 = 0;
         struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/Gyro");
         if (!fy_node_scanf(seqNode, "/Index %ld", &Ig)) {
            fprintf(stderr,
                    "Could not find spacecraft Gyro Index. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         struct GyroType *Gyro = &S->Gyro[Ig];
         assignYAMLToDoubleArray(3, fy_node_by_path_def(seqNode, "/Axis"),
                                 Gyro->Axis);
         UNITV(Gyro->Axis);
         double biasTime;
         if (fy_node_scanf(seqNode,
                           "/Sample Time %lf "
                           "/Max Rate %lf "
                           "/Scale Factor %lf "
                           "/Quantization %lf "
                           "/Angle Random Walk %lf "
                           "/Bias Stability %lf "
                           "/Bias Stability Timespan %lf "
                           "/Angle Noise %lf "
                           "/Initial Bias %lf "
                           "/Node %ld",
                           &Gyro->SampleTime, &Gyro->MaxRate, &Gyro->Scale,
                           &Gyro->Quant, &Gyro->SigV, &Gyro->SigU, &biasTime,
                           &Gyro->SigE, &Gyro->Bias, &Gyro->Node) != 10) {
            fprintf(
                stderr,
                "Spacecraft Gyro %ld has improper configuration. Exiting...\n",
                Ig);
            exit(EXIT_FAILURE);
         }

         Gyro->MaxCounter = (long)(Gyro->SampleTime / DTSIM + 0.5);
         if (Gyro->SampleTime < DTSIM) {
            fprintf(stderr,
                    "Error:  Gyro[%ld].SampleTime smaller than DTSIM.\n", Ig);
            exit(EXIT_FAILURE);
         }
         Gyro->SampleCounter  = Gyro->MaxCounter;
         Gyro->MaxRate       *= D2R;
         Gyro->Scale          = 1.0 + 1.0E-6 * Gyro->Scale;
         Gyro->Quant         *= D2R / 3600.0;
         Gyro->SigV          *= D2R / 60.0; /* from deg/rt-hr to rad/rt-sec */
         Gyro->SigU          *= D2R / 3600.0 / sqrt(biasTime * 3600.0);
         Gyro->SigE          *= D2R / 3600.0;
         Gyro->Bias          *= D2R / 3600.0;
         if (Gyro->Node >= S->B[0].NumNodes) {
            fprintf(stderr, "SC[%ld].Gyro[%ld] Node out of range\n", S->ID, Ig);
            exit(EXIT_FAILURE);
         }
         Gyro->BiasStabCoef = Gyro->SigU * sqrt(Gyro->SampleTime);
         Gyro->ARWCoef =
             sqrt(Gyro->SigV * Gyro->SigV / Gyro->SampleTime +
                  Gyro->SigU * Gyro->SigU * Gyro->SampleTime / 12.0);
         Gyro->AngNoiseCoef = Gyro->SigE / sqrt(Gyro->SampleTime);
         Gyro->CorrCoef     = 1.0 - Gyro->SampleTime / (biasTime * 3600.0);
         Gyro->Angle        = 0.0;
      }
   }
   else
      S->Gyro = NULL;

   /* .. Magnetometer parameters */
   node    = fy_node_by_path_def(root, "/Magnetometers");
   S->Nmag = (node == NULL) ? 0 : fy_node_sequence_item_count(node);
   if (S->Nmag > 0) {
      S->MAG = (struct MagnetometerType *)calloc(
          S->Nmag, sizeof(struct MagnetometerType));
      iterNode = NULL;
      WHILE_FY_ITER(node, iterNode)
      {
         long Im = 0;
         struct fy_node *seqNode =
             fy_node_by_path_def(iterNode, "/Magnetometer");
         if (!fy_node_scanf(seqNode, "/Index %ld", &Im)) {
            fprintf(
                stderr,
                "Could not find spacecraft Magnetometer Index. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         struct MagnetometerType *MAG = &S->MAG[Im];
         assignYAMLToDoubleArray(3, fy_node_by_path_def(seqNode, "/Axis"),
                                 MAG->Axis);
         UNITV(MAG->Axis);
         if (fy_node_scanf(seqNode,
                           "/Sample Time %lf "
                           "/Saturation %lf "
                           "/Scale Factor %lf "
                           "/Quantization %lf "
                           "/Noise %lf "
                           "/Node %ld",
                           &MAG->SampleTime, &MAG->Saturation, &MAG->Scale,
                           &MAG->Quant, &MAG->Noise, &MAG->Node) != 6) {
            fprintf(stderr,
                    "Could not find spacecraft Magnetometer %ld configuration. "
                    "Exiting...\n",
                    Im);
            exit(EXIT_FAILURE);
         }

         MAG->MaxCounter = (long)(MAG->SampleTime / DTSIM + 0.5);
         if (MAG->SampleTime < DTSIM) {
            fprintf(stderr, "Error:  MAG[%ld].SampleTime smaller than DTSIM.\n",
                    Im);
            exit(EXIT_FAILURE);
         }
         MAG->SampleCounter = MAG->MaxCounter;
         if (MAG->Node >= S->B[0].NumNodes) {
            fprintf(stderr, "SC[%ld].MAG[%ld] Node out of range\n", S->ID, Im);
            exit(EXIT_FAILURE);
         }
         MAG->Scale = 1.0 + 1.0E-6 * MAG->Scale;
      }
   }
   else
      S->MAG = NULL;

   /* .. Coarse Sun Sensor parameters */
   node    = fy_node_by_path_def(root, "/CSSs");
   S->Ncss = (node == NULL) ? 0 : fy_node_sequence_item_count(node);
   if (S->Ncss > 0) {
      S->CSS   = (struct CssType *)calloc(S->Ncss, sizeof(struct CssType));
      iterNode = NULL;
      WHILE_FY_ITER(node, iterNode)
      {
         long Ic                 = 0;
         struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/CSS");
         if (!fy_node_scanf(seqNode, "/Index %ld", &Ic)) {
            fprintf(stderr,
                    "Could not find spacecraft CSS Index. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         struct CssType *CSS = &S->CSS[Ic];
         assignYAMLToDoubleArray(3, fy_node_by_path_def(seqNode, "/Axis"),
                                 CSS->Axis);
         UNITV(CSS->Axis);
         if (fy_node_scanf(seqNode,
                           "/Sample Time %lf "
                           "/Half Cone Angle %lf "
                           "/Scale Factor %lf "
                           "/Quantization %lf "
                           "/Body/Index %ld "
                           "/Node %ld",
                           &CSS->SampleTime, &CSS->FovHalfAng, &CSS->Scale,
                           &CSS->Quant, &CSS->Body, &CSS->Node) != 6) {
            fprintf(stderr,
                    "Spacecraft CSS %ld is improperly configured. Exiting...\n",
                    Ic);
            exit(EXIT_FAILURE);
         }

         CSS->MaxCounter = (long)(CSS->SampleTime / DTSIM + 0.5);
         if (CSS->SampleTime < DTSIM) {
            fprintf(stderr, "Error:  CSS[%ld].SampleTime smaller than DTSIM.\n",
                    Ic);
            exit(EXIT_FAILURE);
         }
         if (CSS->Body >= S->Nb) {
            fprintf(stderr, "SC[%ld].CSS[%ld] Body out of range\n", S->ID, Ic);
            exit(EXIT_FAILURE);
         }
         if (CSS->Node >= S->B[CSS->Body].NumNodes) {
            fprintf(stderr, "SC[%ld].CSS[%ld] Node out of range\n", S->ID, Ic);
            exit(EXIT_FAILURE);
         }
         CSS->Scale       = 1.0 + 1.0E-6 * CSS->Scale;
         CSS->FovHalfAng *= D2R;
         CSS->CosFov      = cos(CSS->FovHalfAng);
      }
   }
   else
      S->CSS = NULL;

   /* .. Fine Sun Sensor parameters */
   node    = fy_node_by_path_def(root, "/FSSs");
   S->Nfss = (node == NULL) ? 0 : fy_node_sequence_item_count(node);
   if (S->Nfss > 0) {
      S->FSS   = (struct FssType *)calloc(S->Nfss, sizeof(struct FssType));
      iterNode = NULL;
      WHILE_FY_ITER(node, iterNode)
      {
         long If                 = 0;
         struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/FSS");
         if (!fy_node_scanf(seqNode, "/Index %ld", &If)) {
            fprintf(stderr,
                    "Could not find spacecraft FSS index. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         struct FssType *FSS = &S->FSS[If];

         getYAMLEulerAngles(fy_node_by_path_def(seqNode, "/Mounting Angles"),
                            ang, &seq);
         A2C(seq, ang[0] * D2R, ang[1] * D2R, ang[2] * D2R, FSS->CB);
         C2Q(FSS->CB, FSS->qb);
         assignYAMLToDoubleArray(2, fy_node_by_path_def(seqNode, "/FOV Size"),
                                 FSS->FovHalfAng);
         if (fy_node_scanf(seqNode,
                           "/Sample Time %lf "
                           "/Boresight Axis %49s "
                           "/Noise Equivalent Angle %lf "
                           "/Quantization %lf "
                           "/Node %ld",
                           &FSS->SampleTime, dummy, &FSS->NEA, &FSS->Quant,
                           &FSS->Node) != 5) {
            fprintf(stderr,
                    "Spacecraft FSS %ld is improperly configured. Exiting...\n",
                    If);
            exit(EXIT_FAILURE);
         }
         FSS->BoreAxis = DecodeString(dummy);

         FSS->MaxCounter = (long)(FSS->SampleTime / DTSIM + 0.5);
         if (FSS->SampleTime < DTSIM) {
            fprintf(stderr, "Error:  FSS[%ld].SampleTime smaller than DTSIM.\n",
                    If);
            exit(EXIT_FAILURE);
         }
         FSS->SampleCounter = FSS->MaxCounter;
         FSS->H_Axis        = (FSS->BoreAxis + 1) % 3;
         FSS->V_Axis        = (FSS->BoreAxis + 2) % 3;
         if (FSS->Node >= S->B[0].NumNodes) {
            fprintf(stderr, "SC[%ld].FSS[%ld] Node out of range\n", S->ID, If);
            exit(EXIT_FAILURE);
         }
         if (!fy_node_scanf(seqNode, "/Type %49s", dummy))
            FSS->type = CONVENTIONAL_FSS;
         else
            FSS->type = DecodeString(dummy);

         for (i = 0; i < 2; i++)
            FSS->FovHalfAng[i] *= 0.5 * D2R;
         FSS->NEA   *= D2R;
         FSS->Quant *= D2R;
      }
   }
   else
      S->FSS = NULL;

   /* .. Star Tracker parameters */
   node   = fy_node_by_path_def(root, "/STs");
   S->Nst = (node == NULL) ? 0 : fy_node_sequence_item_count(node);
   if (S->Nst > 0) {
      S->ST = (struct StarTrackerType *)calloc(S->Nst,
                                               sizeof(struct StarTrackerType));
      iterNode = NULL;
      WHILE_FY_ITER(node, iterNode)
      {
         long Ist                = 0;
         struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/ST");
         if (!fy_node_scanf(seqNode, "/Index %ld", &Ist)) {
            fprintf(
                stderr,
                "Could not find spacecraft Startracker Index. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         struct StarTrackerType *ST = &S->ST[Ist];

         getYAMLEulerAngles(fy_node_by_path_def(seqNode, "/Mounting Angles"),
                            ang, &seq);
         A2C(seq, ang[0] * D2R, ang[1] * D2R, ang[2] * D2R, ST->CB);
         C2Q(ST->CB, ST->qb);
         assignYAMLToDoubleArray(2, fy_node_by_path_def(seqNode, "/FOV Size"),
                                 ST->FovHalfAng);

         double tmp[3] = {0.0};
         assignYAMLToDoubleArray(
             3, fy_node_by_path_def(seqNode, "/Noise Equivalent Angle"), tmp);
         struct fy_node *excAng =
             fy_node_by_path_def(seqNode, "/Exclusion Angles");
         if (fy_node_scanf(excAng,
                           "/Sun %lf "
                           "/Earth %lf "
                           "/Luna %lf",
                           &ST->SunExclAng, &ST->EarthExclAng,
                           &ST->MoonExclAng) != 3) {
            fprintf(stderr,
                    "For spacecraft Startracker %ld, Could not find "
                    "Sun/Earth/Moon Exclusion Angles. Exiting...\n",
                    Ist);
            exit(EXIT_FAILURE);
         }

         if (fy_node_scanf(seqNode,
                           "/Sample Time %lf "
                           "/Boresight Axis %49s "
                           "/Node %ld",
                           &ST->SampleTime, dummy, &ST->Node) != 3) {
            fprintf(stderr,
                    "Spacecraft Startracker %ld is improperly configured. "
                    "Exiting...\n",
                    Ist);
            exit(EXIT_FAILURE);
         }
         ST->BoreAxis   = DecodeString(dummy);
         ST->MaxCounter = (long)(ST->SampleTime / DTSIM + 0.5);
         if (ST->SampleTime < DTSIM) {
            fprintf(stderr, "Error:  ST[%ld].SampleTime smaller than DTSIM.\n",
                    Ist);
            exit(EXIT_FAILURE);
         }
         ST->SampleCounter = ST->MaxCounter;
         ST->H_Axis        = (ST->BoreAxis + 1) % 3;
         ST->V_Axis        = (ST->BoreAxis + 2) % 3;
         if (ST->Node >= S->B[0].NumNodes) {
            fprintf(stderr, "SC[%ld].ST[%ld] Node out of range\n", S->ID, Ist);
            exit(EXIT_FAILURE);
         }
         for (i = 0; i < 2; i++) {
            ST->FovHalfAng[i] *= 0.5 * D2R;
            ST->CosFov[i]      = cos(ST->FovHalfAng[i]);
         }
         ST->SunExclAng      *= D2R;
         ST->EarthExclAng    *= D2R;
         ST->MoonExclAng     *= D2R;
         ST->CosSunExclAng    = cos(ST->SunExclAng);
         ST->CosEarthExclAng  = cos(ST->EarthExclAng);
         ST->CosMoonExclAng   = cos(ST->MoonExclAng);
         for (i = 0; i < 3; i++)
            ST->NEA[(ST->BoreAxis + i) % 3] = tmp[i] * D2R / 3600.0;
      }
   }
   else
      S->ST = NULL;

   /* .. GPS parameters */
   node    = fy_node_by_path_def(root, "/GPSs");
   S->Ngps = (node == NULL) ? 0 : fy_node_sequence_item_count(node);
   if (S->Ngps > 0) {
      S->GPS   = (struct GpsType *)calloc(S->Ngps, sizeof(struct GpsType));
      iterNode = NULL;
      WHILE_FY_ITER(node, iterNode)
      {
         long Ig                 = 0;
         struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/GPS");
         if (!fy_node_scanf(seqNode, "/Index %ld", &Ig)) {
            fprintf(stderr,
                    "Could not find spacecraft GPS Index. Exiting...\n");
         }
         struct GpsType *GPS = &S->GPS[Ig];

         if (fy_node_scanf(seqNode,
                           "/Sample Time %lf "
                           "/Position Noise %lf "
                           "/Velocity Noise %lf "
                           "/Time Noise %lf "
                           "/Node %ld",
                           &GPS->SampleTime, &GPS->PosNoise, &GPS->VelNoise,
                           &GPS->TimeNoise, &GPS->Node) != 5) {
            fprintf(stderr,
                    "Spacecraft GPS %ld is improperly configured. Exiting...\n",
                    Ig);
            exit(EXIT_FAILURE);
         }

         GPS->MaxCounter = (long)(GPS->SampleTime / DTSIM + 0.5);
         if (GPS->SampleTime < DTSIM) {
            fprintf(stderr, "Error:  GPS[%ld].SampleTime smaller than DTSIM.\n",
                    Ig);
            exit(EXIT_FAILURE);
         }
         GPS->SampleCounter = GPS->MaxCounter;
         if (GPS->Node >= S->B[0].NumNodes) {
            fprintf(stderr, "SC[%ld].GPS[%ld] Node out of range\n", S->ID, Ig);
            exit(EXIT_FAILURE);
         }
      }
   }
   else
      S->GPS = NULL;

   /* .. Accelerometer parameters */
   node    = fy_node_by_path_def(root, "/Accelerometers");
   S->Nacc = (node == NULL) ? 0 : fy_node_sequence_item_count(node);
   if (S->Nacc > 0) {
      S->Accel = (struct AccelType *)calloc(S->Nacc, sizeof(struct AccelType));
      iterNode = NULL;
      WHILE_FY_ITER(node, iterNode)
      {
         long Ia = 0;
         struct fy_node *seqNode =
             fy_node_by_path_def(iterNode, "/Accelerometer");
         if (!fy_node_scanf(seqNode, "/Index %ld", &Ia)) {
            fprintf(
                stderr,
                "Could not find spacecraft Accelerometer index. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         struct AccelType *Accel = &S->Accel[Ia];
         assignYAMLToDoubleArray(3, fy_node_by_path_def(seqNode, "/Axis"),
                                 Accel->Axis);
         UNITV(Accel->Axis);
         double biasTime;
         if (fy_node_scanf(seqNode,
                           "/Sample Time %lf "
                           "/Max Acceleration %lf "
                           "/Scale Factor %lf "
                           "/Quantization %lf "
                           "/DV Random Walk %lf "
                           "/Bias Stability %lf "
                           "/Bias Stability Timespan %lf "
                           "/DV Noise %lf "
                           "/Initial Bias %lf "
                           "/Node %ld",
                           &Accel->SampleTime, &Accel->MaxAcc, &Accel->Scale,
                           &Accel->Quant, &Accel->SigV, &Accel->SigU, &biasTime,
                           &Accel->SigE, &Accel->Bias, &Accel->Node) != 10) {
            fprintf(stderr,
                    "Spacecraft Accelerometer %ld is improperly configured. "
                    "Exiting...\n",
                    Ia);
            exit(EXIT_FAILURE);
         }

         Accel->MaxCounter = (long)(Accel->SampleTime / DTSIM + 0.5);
         if (Accel->SampleTime < DTSIM) {
            fprintf(stderr,
                    "Error:  Accel[%ld].SampleTime smaller than DTSIM.\n", Ia);
            exit(EXIT_FAILURE);
         }
         Accel->SampleCounter = Accel->MaxCounter;
         if (Accel->Node >= S->B[0].NumNodes) {
            fprintf(stderr, "SC[%ld].Accel[%ld] Node out of range\n", S->ID,
                    Ia);
            exit(EXIT_FAILURE);
         }
         Accel->Scale         = 1.0 + 1.0E-6 * Accel->Scale;
         Accel->SigV         /= 60.0; /* from m/s/rt-hr to m/s/rt-sec */
         Accel->SigU         /= sqrt(biasTime * 3600.0);
         Accel->BiasStabCoef  = Accel->SigU * sqrt(Accel->SampleTime);
         Accel->DVRWCoef =
             sqrt(Accel->SigV * Accel->SigV / Accel->SampleTime +
                  Accel->SigU * Accel->SigU * Accel->SampleTime / 12.0);
         Accel->DVNoiseCoef = Accel->SigE / sqrt(Accel->SampleTime);
         Accel->CorrCoef    = 1.0 - Accel->SampleTime / (biasTime * 3600.0);
         Accel->DV          = 0.0;
      }
   }
   else
      S->Accel = NULL;

   /* .. Fine Guidance Sensors */
   node    = fy_node_by_path_def(root, "/FGSs");
   S->Nfgs = (node == NULL) ? 0 : fy_node_sequence_item_count(node);
   if (S->Nfgs > 0) {
      S->Fgs   = (struct FgsType *)calloc(S->Nfgs, sizeof(struct FgsType));
      iterNode = NULL;
      WHILE_FY_ITER(node, iterNode)
      {
         long Ifgs               = 0;
         struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/FGS");
         if (!fy_node_scanf(seqNode, "/Index %ld", &Ifgs)) {
            fprintf(stderr,
                    "Could not find spacecraft FGS index. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         struct FgsType *FGS = &S->Fgs[i];
         if (fy_node_scanf(seqNode,
                           "/Sample Time %lf "
                           "/Boresight Axis %49s "
                           "/Noise Equivalent Angle %lf "
                           "/Detector Scale %lf "
                           "/Node %ld "
                           "/Optics File Name %[^\n] "
                           "/PSF Image File %49s "
                           "/Body/Index %ld",
                           &FGS->SampleTime, dummy, &FGS->NEA, &FGS->Scl,
                           &FGS->Node, FGS->OpticsFileName, FGS->PsfFileName,
                           &FGS->Body)) {
            fprintf(stderr,
                    "Spacecraft FGS %ld is improperly configured. Exiting...\n",
                    Ifgs);
            exit(EXIT_FAILURE);
         }
         FGS->BoreAxis = DecodeString(dummy);
         if (FGS->Body >= S->Nb) {
            fprintf(stderr, "SC[%ld].FGS[%ld] Body out of range\n", S->ID,
                    Ifgs);
            exit(EXIT_FAILURE);
         }
         if (FGS->Node >= S->B[FGS->Body].NumNodes) {
            fprintf(stderr, "SC[%ld].FGS[%ld] Node out of range\n", S->ID,
                    Ifgs);
            exit(EXIT_FAILURE);
         }
         FGS->MaxCounter     = (long)(FGS->SampleTime / DTSIM + 0.5);
         FGS->SampleCounter  = FGS->MaxCounter;
         FGS->H_Axis         = (FGS->BoreAxis + 1) % 3;
         FGS->V_Axis         = (FGS->BoreAxis + 2) % 3;
         FGS->NEA           *= A2R;
         FGS->Scl           *= A2R;

         getYAMLEulerAngles(fy_node_by_path_def(seqNode, "/Mounting Angles"),
                            ang, &seq);
         A2C(seq, ang[0] * D2R, ang[1] * D2R, ang[2] * D2R, FGS->CB);
         C2Q(FGS->CB, FGS->qb);
         assignYAMLToDoubleArray(2, fy_node_by_path_def(seqNode, "/FOV Size"),
                                 FGS->FovHalfAng);
         for (i = 0; i < 2; i++)
            FGS->FovHalfAng[i] *= 0.5 * A2R;

         getYAMLEulerAngles(fy_node_by_path_def(seqNode, "/FOV Frame Angles"),
                            ang, &seq);
         A2C(seq, ang[0] * D2R, ang[1] * D2R, ang[2] * D2R, FGS->CR);
         C2Q(FGS->CR, FGS->qr);
         assignYAMLToDoubleArray(2, fy_node_by_path_def(seqNode, "/Guide Star"),
                                 ang);
         FGS->Hr = ang[0] * A2R;
         FGS->Vr = ang[1] * A2R;
         InitOptics(FGS);
         if (strcmp(FGS->PsfFileName, "NONE")) {
            struct PsfType *PSF = &FGS->PSF;
            PSF->Image = PpmToPsf(ModelPath, FGS->PsfFileName, &PSF->Ncol,
                                  &PSF->Nrow, &PSF->BytesPerPixel);
         }
      }
   }
   else
      S->Fgs = NULL;

   /* .. Initialize some Orbit and Formation variables */
   struct OrbitType *O      = &Orb[S->RefOrb];
   struct FormationType *Fr = &Frm[S->RefOrb];
   double pcmn[3], wxr[3], wxrn[3], psn[3], vsn[3], rh[3], vh[3];
   if (useCM) {
      if (Fr->FixedInFrame == 'L') {
         MTxV(Fr->CL, posVec, S->PosEH);
         MTxV(Fr->CL, velVec, S->VelEH);
         if (O->Regime == ORB_ZERO) {
            for (i = 0; i < 3; i++) {
               S->PosR[i] = S->PosEH[i];
               S->VelR[i] = S->VelEH[i];
            }
         }
         else if (O->Regime == ORB_FLIGHT) {
            MTxV(O->CLN, S->PosEH, S->PosR);
            MTxV(O->CLN, S->VelEH, S->VelR);
         }
         else {
            EHRV2RelRV(O->SMA, O->MeanMotion, O->CLN, S->PosEH, S->VelEH,
                       S->PosR, S->VelR);
         }
      }
      else {
         MTxV(Fr->CN, posVec, S->PosR);
         MTxV(Fr->CN, velVec, S->VelR);
         if (O->Regime == ORB_ZERO) {
            for (i = 0; i < 3; i++) {
               S->PosEH[i] = S->PosR[i];
               S->VelEH[i] = S->VelR[i];
            }
         }
         else if (O->Regime == ORB_FLIGHT) {
            MxV(O->CLN, S->PosR, S->PosEH);
            MxV(O->CLN, S->VelR, S->VelEH);
         }
         else {
            RelRV2EHRV(O->SMA, MAGV(O->wln), O->CLN, S->PosR, S->VelR, S->PosEH,
                       S->VelEH);
         }
      }

      MTxV(S->B[0].CN, S->cm, pcmn);
      for (j = 0; j < 3; j++) {
         psn[j] = S->PosR[j] - Fr->PosR[j] - pcmn[j];
      }
      MxV(Fr->CN, psn, S->PosF);
      VxV(S->B[0].wn, S->cm, wxr);
      MTxV(S->B[0].CN, wxr, wxrn);
      for (j = 0; j < 3; j++) {
         vsn[j] = S->VelR[j] - wxrn[j];
      }
      MxV(Fr->CN, vsn, S->VelF);
   }
   else {
      for (j = 0; j < 3; j++) {
         S->PosF[j] = posVec[j];
         S->VelF[j] = velVec[j];
      }
      double psl[3], vsl[3], pfl[3], pcml[3], wxrl[3];
      MTxV(S->B[0].CN, S->cm, pcmn);
      VxV(S->B[0].wn, S->cm, wxr);
      MTxV(S->B[0].CN, wxr, wxrn);
      if (Fr->FixedInFrame == 'L') {
         MTxV(Fr->CL, S->PosF, psl);
         MTxV(Fr->CL, S->VelF, vsl);
         MxV(O->CLN, Fr->PosR, pfl);
         MxV(O->CLN, pcmn, pcml);
         MxV(O->CLN, wxrn, wxrl);
         for (j = 0; j < 3; j++) {
            S->PosEH[j] = pcml[j] + psl[j] + pfl[j];
            S->VelEH[j] = wxrl[j] + vsl[j];
         }
         if (O->Regime == ORB_ZERO) {
            for (i = 0; i < 3; i++) {
               S->PosR[i] = S->PosEH[i];
               S->VelR[i] = S->VelEH[i];
            }
         }
         else if (O->Regime == ORB_FLIGHT) {
            MTxV(O->CLN, S->PosEH, S->PosR);
            MTxV(O->CLN, S->VelEH, S->VelR);
         }
         else {
            EHRV2RelRV(O->SMA, MAGV(O->wln), O->CLN, S->PosEH, S->VelEH,
                       S->PosR, S->VelR);
         }
      }
      else {
         MTxV(Fr->CN, S->PosF, psn);
         MTxV(Fr->CN, S->VelF, vsn);
         for (j = 0; j < 3; j++) {
            S->PosR[j] = pcmn[j] + psn[j] + Fr->PosR[j];
            S->VelR[j] = wxrn[j] + vsn[j];
         }
         if (O->Regime == ORB_ZERO) {
            // TODO: ????? should something be here????
         }
         else if (O->Regime == ORB_FLIGHT) {
            // TODO: ????? should something be here????
         }
         else {
            RelRV2EHRV(O->SMA, MAGV(O->wln), O->CLN, S->PosR, S->VelR, S->PosEH,
                       S->VelEH);
         }
      }
   }
   if (O->use_N_BODY_Vec) {
      for (j = 0; j < 3; j++) {
         S->PosN[j] = O->N_BODY_PosN[j];
         S->VelN[j] = O->N_BODY_VelN[j];
      }
   }
   else {
      for (j = 0; j < 3; j++) {
         S->PosN[j] = O->PosN[j] + S->PosR[j];
         S->VelN[j] = O->VelN[j] + S->VelR[j];
      }
   }
   MTxV(World[O->World].CNH, S->PosN, rh);
   MTxV(World[O->World].CNH, S->VelN, vh);
   for (j = 0; j < 3; j++) {
      S->PosH[j] = World[O->World].PosH[j] + rh[j];
      S->PosH[j] = World[O->World].VelH[j] + vh[j];
   }

   if (O->Regime == ORB_ZERO) {
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++)
            S->CLN[i][j] = 0.0;
         S->CLN[i][i] = 1.0;
         S->wln[i]    = 0.0;
      }
   }
   else if (O->Regime == ORB_FLIGHT) {
      FindENU(S->PosN, World[O->World].w, S->CLN, S->wln);
   }
   else {
      FindCLN(S->PosN, S->VelN, S->CLN, S->wln);
   }

   if (S->DynMethod == DYN_ORDER_N) {
      if (SomeJointsLocked || S->FlexActive || S->ConstraintsRequested) {
         printf("Order-N dynamics doesn't (yet) support flex modes, constraint "
                "computation, or locking joints.\n");
         printf("Switching over to Gaussian Elimination.\n");
         S->DynMethod = DYN_GAUSS_ELIM;
      }
   }

   struct DynType *D = &S->Dyn;
   InitRigidDyn(S);
   InitFlexModes(S);
   InitOrderNDynamics(S);

   D->ActiveState    = (double *)calloc(D->Nu + D->Nf, sizeof(double));
   D->ActiveStateIdx = (long *)calloc(D->Nu + D->Nf, sizeof(long));
   D->COEF           = CreateMatrix(D->Nu + D->Nf, D->Nu + D->Nf);
   D->RHS            = (double *)calloc(D->Nu + D->Nf, sizeof(double));

   MapStateVectorToBodyStates(D->u, D->x, D->h, D->a, D->uf, D->xf, S);
   MotionConstraints(S);
   BodyStatesToNodeStates(S);
   SCMassProps(S);
   FindTotalAngMom(S);
   EchoDyn(S);

   /* .. Load geometry */
   for (j = 0; j < S->Nb; j++) {
      long OldNgeom = Ngeom;
      Geom = LoadWingsObjFile(SCModelPath, S->B[j].GeomFileName, &Matl, &Nmatl,
                              Geom, &Ngeom, &S->B[j].GeomTag,
                              AeroShadowsActive || SolPressShadowsActive);
      if (ContactActive && OldNgeom != Ngeom)
         LoadOctree(&Geom[Ngeom - 1]);
   }

   /* .. Initialize Bounding Box */
   memcpy(&S->BBox, &Geom[S->B[0].GeomTag].BBox,
          sizeof(struct BoundingBoxType));
   UpdateScBoundingBox(S);

   S->EnvTrq.First = 1;

   InitAC(S);

   InitShakers(S);

   InitDSM(S);

   /* .. Loop Gain and Delays allow verification of stability margins in the
    * time domain */
   /* Created by commands */
   S->GainAndDelayActive = FALSE;
   S->LoopGain           = 1.0;
   S->LoopDelay          = 0.0;
   for (i = 0; i < 3; i++) {
      S->IdealAct[i].FrcDelay = NULL;
      S->IdealAct[i].TrqDelay = NULL;
   }
   for (long Iw = 0; Iw < S->Nw; Iw++) {
      S->Whl[Iw].Delay = NULL;
   }
   for (long Im = 0; Im < S->Nmtb; Im++) {
      S->MTB[Im].Delay = NULL;
   }
   for (long It = 0; It < S->Nthr; It++) {
      S->Thr[It].Delay = NULL;
   }
   fy_document_destroy(fyd);
}
/*********************************************************************/
void LoadTdrs(void)
{
   // TODO: configurable TDRS constellation

   /* .. Initialize TDRS */
   struct fy_document *fyd =
       fy_document_build_and_check(NULL, InOutPath, "Inp_TDRS.yaml");
   struct fy_node *root = fy_document_root(fyd);
   struct fy_node *node = fy_node_by_path_def(root, "/TDRSs");
   /* .. 42 TDRS Configuration File */
   struct fy_node *iterNode = NULL;
   WHILE_FY_ITER(node, iterNode)
   {
      struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/TDRS");
      long i                  = 0;
      if (!fy_node_scanf(seqNode, "/Number %ld", &i)) {
         fprintf(stderr, "Could not find TDRS Number. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      if (i > 10)
         continue;

      if (!fy_node_scanf(seqNode, "/Label %39[^\n]s",
                         Tdrs[i - 1].Designation)) {
         fprintf(stderr, "Could not find TDRS %ld's Label. Exiting...\n", i);
         exit(EXIT_FAILURE);
      }
      Tdrs[i - 1].Exists = getYAMLBool(fy_node_by_path_def(seqNode, "/Exists"));
   }
   fy_document_destroy(fyd);
}
/*********************************************************************/
void LoadGravModel(const char *modelPath, struct SphereHarmType *GravModel)
{
   double dum1, dum2;
   long n, m;
   long n_max = -10, m_max = -10;

   GravModel->Type = 0;

   if (strcmp(GravModel->modelFile, "") == 0 || GravModel->N <= 1) {
      GravModel->Norm = NULL;
      GravModel->C    = NULL;
      GravModel->S    = NULL;
      GravModel->N    = 0;
      GravModel->M    = 0;
   }
   else {
      FILE *gravFile  = FileOpen(modelPath, GravModel->modelFile, "r");
      GravModel->C    = CreateMatrix(GravModel->N + 1, GravModel->M + 1);
      GravModel->S    = CreateMatrix(GravModel->N + 1, GravModel->M + 1);
      GravModel->Norm = CreateMatrix(GravModel->N + 3, GravModel->M + 3);

      rewind(gravFile);
      long succesful      = FALSE;
      char buffer[BUFSIZ] = {0};
      while (fgets(buffer, sizeof(buffer), gravFile) != NULL) {
         sscanf(buffer, "%ld %ld %lf %lf", &n, &m, &dum1, &dum2);
         if (n > n_max)
            n_max = n;
         if (m > m_max)
            m_max = m;
         if (n <= GravModel->N && m <= GravModel->M) {
            GravModel->C[n][m] = dum1;
            GravModel->S[n][m] = dum2;
            if (n == GravModel->N && m == GravModel->M) {
               succesful = TRUE;
            }
         }
      }
      fclose(gravFile);
      if (!succesful) {
         fprintf(stderr,
                 "Got to end of Gravity Model File, %s, before fulfilling "
                 "Requested degree and/or order. File has maximum degree %ld, "
                 "maximum order %ld. Exiting...\n",
                 GravModel->modelFile, n_max, m_max);
         exit(EXIT_FAILURE);
      }
      /* Transform from EGM normalization to Neumann normalization */
      // From GMAT Source, Harmonic.cpp, line 289-297
      // for the moment, not sure why up to n+2, m+2
      for (n = 0; n <= GravModel->N + 2; n++) {
         // initializer for following loop
         GravModel->Norm[n][0] = sqrt(2 * (2 * n + 1));
         for (m = 1; m <= n + 2 && m <= GravModel->M + 2; m++) {
            GravModel->Norm[n][m] =
                GravModel->Norm[n][m - 1] / sqrt((n + m) * (n - m + 1));
         }
         GravModel->Norm[n][0] = sqrt(2 * n + 1);
      }
   }
}
/******************************************************************************/
double getDEHeader1041Data(const JPLHeaderType *const hdr_data,
                           const char *grp_1040_name)
{
   // Get data from group 1040/1041 in JPL DE header
   for (int i = 0; i < hdr_data->n_data; i++) {
      if (!strncmp(hdr_data->group_1040[i], grp_1040_name, 9))
         return hdr_data->group_1041[i];
   }
   fprintf(stderr, "Could not find `%s` in group 1040 of file %s. Exiting...\n",
           grp_1040_name, hdr_data->hdr_name);
   exit(EXIT_FAILURE);
}
/*********************************************************************/
void LoadSun(const ephemType ephem, const JDType jd,
             const JPLHeaderType *const jpl_hdr, struct WorldType *const worlds)
{
   const double Zaxis[3] = {0.0, 0.0, 1.0};
   /* Rumor is, Sun's magfield is highly variable, poorly modeled */
   /* by simple dipole.                                           */
   double DipoleAxis[3]    = {0.0, 0.0, 1.0};
   double SunColor[3]      = {1.0, 1.0, 0.9};
   unsigned char Glyph[14] = {0xc0, 0xc0, 0x00, 0x00, 0x18, 0x66, 0x42,
                              0x99, 0x99, 0x42, 0x66, 0x18, 0x00, 0x00};
   long i, j;
   struct WorldType *W;

   W = &worlds[SOL];

   /* Relationships */
   W->Exists = TRUE;
   W->Type   = SUN;
   W->Parent = 0;

   W->Nsat = 9;
   W->Sat  = (WorldID *)calloc(W->Nsat, sizeof(long));
   if (W->Sat == NULL) {
      fprintf(stderr, "W->Sat calloc returned null pointer.  Bailing out!\n");
      exit(EXIT_FAILURE);
   }
   for (i = 0; i < W->Nsat; i++)
      W->Sat[i] = MERCURY + i;

   /* Physical Properties */
   double GM;
   switch (ephem) {
      case EPH_MEAN:
         W->mu  = 1.32715E20;
         W->rad = 6.98E8;
         break;
      case EPH_GMAT421:
      case EPH_GMAT424:
         W->mu  = 1.3271244001799E20;
         W->rad = 6.95990E8;
         break;
      case EPH_SPICE:
         break;
      default:
         // handle the DE lookup cases here
         GM    = getDEHeader1041Data(jpl_hdr, "GMS");
         W->mu = GM * AUd2ms; // 1.3271244004193938E20;
         if (ephem == EPH_DE440) {
            W->rad = 6.95700E8;
         }
         else {
            W->rad = 6.96000E8;
         }
         break;
   }

   /* Default Pyhsical Parameters */
   W->w              = 2.69E-6;
   W->PriMerAngJ2000 = 84.176 * D2R;
   W->RadOfInfluence = 2.0E13; /* Beyond Pluto's Orbit */
   W->DipoleMoment   = 0.0;
   for (j = 0; j < 3; j++) {
      W->DipoleAxis[j]   = DipoleAxis[j];
      W->DipoleOffset[j] = 0.0;
   }
   W->RingInner = 0.0;
   W->RingOuter = 0.0;

#ifdef _ENABLE_SPICE_
   if (EphemOption == EPH_SPICE) { // If we are using SPICE, replace the
                                   // hardcoded values with SPICE values
      double tmp_holder     = 0;
      double tmp_holder3[3] = {0};
      int dim               = 0;

      bodvrd_c("Sun", "GM", 1, &dim, &tmp_holder);
      W->mu = tmp_holder * 1E9;
      bodvrd_c("Sun", "RADII", 3, &dim, tmp_holder3);
      W->rad = tmp_holder3[0] * 1e3;

      bodvrd_c("SUN", "PM", 3, &dim, tmp_holder3);
      W->PriMerAngJ2000 = tmp_holder3[0] * D2R;
      W->w = tmp_holder3[1] * D2R /
             spd_c(); // converts the prime meridian rate in deg/day to rad/s
   }
#endif

   /* Ephemeris */
   W->eph.World = 0;
   W->eph.mu    = W->mu;
   W->eph.SMA   = 0.0;
   W->eph.ecc   = 0.0;
   W->eph.inc   = 0.0;
   W->eph.RAAN  = 0.0;
   W->eph.ArgP  = 0.0;
   W->eph.tp    = 0.0;
   W->eph.anom  = 0.0;
   W->eph.alpha = 0.0;
   W->eph.SLR   = 0.0;
   W->eph.rmin  = 0.0;

   /* Graphical Properties */
   W->Atmo.Exists = FALSE;
   W->HasRing     = FALSE;
   strcpy(W->Name, "Sun");
   strcpy(W->MapFileName, "NONE");
   strcpy(W->GeomFileName, "NONE");
   strcpy(W->ColTexFileName, "NONE");
   strcpy(W->BumpTexFileName, "NONE");
   for (j = 0; j < 3; j++) {
      W->Color[j] = (float)SunColor[j];
   }
   W->Color[3] = 1.0;
   for (j = 0; j < 14; j++)
      W->Glyph[j] = Glyph[j];

   /* State Variables */
   for (i = 0; i < 3; i++) {
      W->eph.PosN[i] = 0.0;
      W->eph.VelN[i] = 0.0;
      for (j = 0; j < 3; j++)
         W->CNH[i][j] = 0.0;
      W->CNH[i][i] = 1.0;
      W->qnh[i]    = 0.0;
   }
   W->qnh[3] = 1.0;
   QxQT(W->qnh, qjh, W->qnj);

   const double j2000sec_tt = JDToDynTime(jd);
   W->PriMerAng = fmod(W->PriMerAngJ2000 + W->w * j2000sec_tt, TwoPi);
   SimpRot(Zaxis, W->PriMerAng, W->CWN);
   C2Q(W->CWN, W->qwn);
}
/*********************************************************************/
void LoadPlanets(const ephemType ephem, const JDType jd,
                 const JPLHeaderType *const jpl_hdr,
                 struct WorldType *const worlds)
{
   struct OrbitType *Eph;
   const double Zaxis[3] = {0.0, 0.0, 1.0};
   double GMST;
   double C_W_TETE[3][3], C_TEME_TETE[3][3], C_TETE_J2000[3][3];

   char PlanetName[10][20]  = {"Sun",     "Mercury", "Venus",  "Earth",
                               "Mars",    "Jupiter", "Saturn", "Uranus",
                               "Neptune", "Pluto"};
   char MapFileName[10][20] = {
       "NONE",        "Rockball",   "Venus.ppm",  "Earth.ppm",   "Mars.ppm",
       "Jupiter.ppm", "Saturn.ppm", "Uranus.ppm", "Neptune.ppm", "Iceball"};
   const char GravFileName[10][20] = {"", "", "", "EGM08.txt", "GMM2B.txt",
                                      "", "", "", "",          ""};
   double Mu_mean[10] = {1.32715E20, 2.18E13,  3.2485E14, 3.986004E14, 4.293E13,
                         1.2761E17,  3.792E16, 5.788E15,  6.8E15,      3.2E14};
   double Rad_mean[10] = {6.98E8, 2.42E6, 6.1E6,  6.378145E6, 3.41E6,
                          7.14E7, 6.04E7, 2.35E7, 2.23E7,     7.0E6};
   double Rad_421[10]  = {6.96000E8, 2.4397E6, 6.0518E6, 6.37814E6, 3.39619E6,
                          7.1492E7,  6.0268E7, 2.5559E7, 2.4764E7,  1.195E6};
   double Rad_424[10]  = {6.96000E8, 2.4397E6, 6.0518E6, 6.37814E6, 3.39619E6,
                          7.1492E7,  6.0268E7, 2.5559E7, 2.4764E7,  1.195E6};
   double Rad_430[10]  = {6.96000E8, 2.4397E6, 6.0518E6, 6.3781366E6, 3.39619E6,
                          7.1492E7,  6.0268E7, 2.5559E7, 2.4764E7,    1.195E6};
   double Rad_440[10] = {6.95700E8, 2.44053E6, 6.0518E6, 6.3781366E6, 3.39619E6,
                         7.1492E7,  6.0268E7,  2.5559E7, 2.4764E7,    1.1883E6};
   double Mu_GMAT[10] = {1.3271244001799E20, 2.2032080486418E13,
                         3.2485859882646E14, 3.986004415E14,
                         4.2828314258067E13, 1.2671276785780E17,
                         3.7940626061137E16, 5.7945490070719E15,
                         6.8365340638793E15, 9.8160088770700E11};
   double Rad_GMAT[10] = {6.95990E8, 2.4397E6, 6.0519E6, 6.3781363E6, 3.397E6,
                          7.1492E7,  6.0268E7, 2.5559E7, 2.5269E7,    1.162E6};

   double Mu[10], Rad[10];
   /* Physical Properties */
   switch (ephem) {
      case EPH_MEAN:
         for (int i = 0; i < 10; ++i)
            Mu[i] = Mu_mean[i];
         for (int i = 0; i < 10; ++i)
            Rad[i] = Rad_mean[i];
         break;
      case EPH_GMAT421:
      case EPH_GMAT424:
         for (int i = 0; i < 10; ++i)
            Mu[i] = Mu_GMAT[i];
         for (int i = 0; i < 10; ++i)
            Rad[i] = Rad_GMAT[i];
         break;
      case EPH_SPICE:
         break;
      default: {
         // handle the DE lookup cases here
         for (WorldID i = SOL + 1; i <= PLUTO; i++) {
            char gm_str[6] = {'\0'};
            if (i == SOL)
               strcpy(gm_str, "GMS");
            if (i == EARTH)
               strcpy(gm_str, "GMB");
            else
               sprintf(gm_str, "GM%u", i);

            Mu[i] = getDEHeader1041Data(jpl_hdr, gm_str) * AUd2ms;
         }
         Mu[EARTH] = Mu[EARTH] / (1.0 + 1.0 / EMRAT);

         switch (ephem) {
            case EPH_DE421:
               for (int i = 0; i < 10; ++i)
                  Rad[i] = Rad_421[i];
               break;
            case EPH_DE424:
               for (int i = 0; i < 10; ++i)
                  Rad[i] = Rad_424[i];
               break;
            case EPH_DE430:
               for (int i = 0; i < 10; ++i)
                  Rad[i] = Rad_430[i];
               break;
            case EPH_DE440:
               for (int i = 0; i < 10; ++i)
                  Rad[i] = Rad_440[i];
               break;
            default:
               break;
         }
      } break;
   }

   /* Default Pyhsical Parameters */
   double J2[10]     = {0.0, 0.0, 0.0, 1.08263E-3, 1.96045E-3,
                        0.0, 0.0, 0.0, 0.0,        0.0};
   double AngVel[10] = {2.69E-6,   1.23E-6,   2.94E-7,  7.292115E-5, 7.0882E-5,
                        1.7659E-4, 1.6728E-4, 1.631E-4, 1.105E-4,    0.0};

   double PoleRA[10]         = {0.0,     281.008, 272.758, 0.0,     317.683,
                                268.057, 40.587,  257.313, 299.333, 133.046};
   double PoleDec[10]        = {90.0,   61.45,  67.16,   0.0,   52.8865,
                                64.496, 83.537, -15.175, 42.95, -6.145};
   double PriMerAngJ2000[10] = {0.0,    329.71, 160.26, 190.16, 176.868,
                                284.95, 38.90,  203.81, 253.18, 236.77};

   double grav_r_ref[10];
   for (int i = 0; i < 10; i++) {
      switch (i) {
         case EARTH:
            grav_r_ref[i] = 6378136.3;
            break;
         default:
            grav_r_ref[i] = Rad[i];
            break;
      }
   }

#ifdef _ENABLE_SPICE_
   if (EphemOption == EPH_SPICE) { // If we are using SPICE, replace the
      // hardcoded values with SPICE values
      const char OrientationName[10][20] = {
          "SUN",     "MERCURY", "VENUS",  "EARTH",   "MARS",
          "JUPITER", "SATURN",  "URANUS", "NEPTUNE", "PLUTO"};
      double tmp_holder     = 0;
      double tmp_holder3[3] = {0};
      int dim               = 0;
      for (int i = SOL; i <= PLUTO; i++) {
         bodvrd_c(PlanetName[i], "GM", 1, &dim, &tmp_holder);
         Mu[i] = tmp_holder * 1E9;

         if ((!strcmp(PlanetName[i], "Pluto")) ||
             (!strcmp(PlanetName[i], "Sun"))) { // Pluto/Sun J2 is not defined
            J2[i] = 0.0;
         }
         else {
            bodvrd_c(PlanetName[i], "J2", 1, &dim, &tmp_holder);
            J2[i] = tmp_holder;
         }

         bodvrd_c(PlanetName[i], "RADII", 3, &dim, tmp_holder3);
         Rad[i] = tmp_holder3[0] * 1e3;

         bodvrd_c(OrientationName[i], "PM", 3, &dim, tmp_holder3);
         PriMerAngJ2000[i] = tmp_holder3[0];
         AngVel[i] =
             tmp_holder3[1] * D2R /
             spd_c(); // converts the prime meridian rate in deg/day to rad/s

         bodvrd_c(OrientationName[i], "POLE_RA", 3, &dim, tmp_holder3);
         PoleRA[i] = tmp_holder3[0];

         bodvrd_c(OrientationName[i], "POLE_DEC", 3, &dim, tmp_holder3);
         PoleDec[i] = tmp_holder3[0];
      }
   }
#endif

   double CNJ[3][3];
   /* Magnetic Field Dipole Strength, Wb-m */
   const double DipoleMoment[10] = {0.0, 0.0, 0.0, 7.943E15, 0.0,
                                    0.0, 0.0, 0.0, 0.0,      0.0};
   /* Magnetic Field Dipole Axis Unit Vector */
   const double DipoleAxis[10][3] = {
       {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0},
       {0.0, 0.0, 1.0}, {-6.53286E-2, 0.186549, -0.980271},
       {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0},
       {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0},
       {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}};
   /* Magnetic Field Dipole Offset from Center, m */
   const double DipoleOffset[10][3] = {
       {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
       {0.0, 0.0, 0.0}, {-3.74461E5, 2.44108E5, -1.58291E5},
       {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
       {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
       {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

   const float Color[11][3] = {{1.0f, 1.0f, 0.9f},                 /* Sun */
                               {0.400318f, 0.347338f, 0.253973f},  /* Mercury */
                               {0.716824f, 0.676952f, 0.623907f},  /* Venus */
                               {0.212f, 0.293502f, 0.522072f},     /* Earth */
                               {0.687493f, 0.454481f, 0.365368f},  /* Mars */
                               {0.793131f, 0.627618f, 0.477430f},  /* Jupiter */
                               {0.705187f, 0.677713f, 0.620916f},  /* Saturn */
                               {0.486074f, 0.584573f, 0.769742f},  /* Uranus */
                               {0.187558f, 0.243884f, 0.413025f},  /* Neptune */
                               {0.268063f, 0.268183f, 0.268204f},  /* Pluto */
                               {0.440417f, 0.441343f, 0.441084f}}; /* Luna */

   const unsigned char Glyph[11][14] = {
       {0xc0, 0xc0, 0x00, 0x00, 0x18, 0x66, 0x42, 0x99, 0x99, 0x42, 0x66, 0x18,
        0x00, 0x00}, /* Sun */
       {0xc0, 0xc0, 0x00, 0x10, 0x7c, 0x10, 0x38, 0x44, 0x82, 0x82, 0x44, 0x38,
        0x44, 0x82}, /* Mercury */
       {0xc0, 0xc0, 0x00, 0x10, 0x10, 0x7c, 0x10, 0x38, 0x44, 0x82, 0x82, 0x44,
        0x38, 0x00}, /* Venus */
       {0xc0, 0xc0, 0x00, 0x00, 0x38, 0x54, 0x92, 0xfe, 0x92, 0x54, 0x38, 0x00,
        0x00, 0x00}, /* Earth */
       {0xc0, 0xc0, 0x00, 0x00, 0x38, 0x44, 0x82, 0x82, 0x82, 0x44, 0x3c, 0x05,
        0x01, 0x07}, /* Mars */
       {0xc0, 0xc0, 0x00, 0x04, 0x04, 0x7f, 0x44, 0x24, 0x14, 0x14, 0x24, 0xc2,
        0x00, 0x00}, /* Jupiter */
       {0xc0, 0xc0, 0x00, 0x0c, 0x08, 0x44, 0x42, 0x42, 0x64, 0x58, 0x40, 0x40,
        0xe0, 0x00}, /* Saturn */
       {0xc0, 0xc0, 0x00, 0x38, 0x44, 0x82, 0x92, 0x82, 0x44, 0x38, 0x10, 0x54,
        0x38, 0x10}, /* Uranus */
       {0xc0, 0xc0, 0x00, 0x10, 0xfe, 0x10, 0x38, 0x54, 0x92, 0x92, 0xd6, 0x92,
        0x00, 0x00}, /* Neptune */
       {0xc0, 0xc0, 0x00, 0x00, 0xf8, 0x80, 0x80, 0xf0, 0x88, 0x88, 0xf0, 0x00,
        0x00, 0x00}, /* Pluto */
       {0xc0, 0xc0, 0x00, 0x00, 0x18, 0x70, 0x60, 0xe0, 0xe0, 0x60, 0x70, 0x18,
        0x00, 0x00}}; /* Luna */
   const long HasAtmo[11] = {0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0};
   long i, j;

   for (i = MERCURY; i <= PLUTO; i++) {
      struct WorldType *W = &worlds[i];
      strcpy(W->Name, PlanetName[i]);
      strcpy(W->MapFileName, MapFileName[i]);
      strcpy(W->ColTexFileName, "NONE");
      strcpy(W->BumpTexFileName, "NONE");
      W->mu             = Mu[i];
      W->J2             = J2[i];
      W->rad            = Rad[i];
      W->w              = AngVel[i];
      W->PriMerAngJ2000 = PriMerAngJ2000[i] * D2R;
      W->Parent         = SOL;
      W->eph.World      = SOL;
      W->eph.mu         = World[SOL].mu;
      W->DipoleMoment   = DipoleMoment[i];
      for (j = 0; j < 3; j++) {
         W->DipoleAxis[j]   = DipoleAxis[i][j];
         W->DipoleOffset[j] = DipoleOffset[i][j];
         W->Color[j]        = Color[i][j];
      }
      W->Color[3] = 1.0;
      for (j = 0; j < 14; j++)
         W->Glyph[j] = Glyph[i][j];
      W->Atmo.Exists = HasAtmo[i];

      if (GravPertActive) {
         /* Gravitation Model */
         struct SphereHarmType *gravModel = &W->GravModel;
         if (!strcmp(gravModel->modelFile, "")) {
            if (!strcmp(GravFileName[i], "") && gravModel->N > 1) {
               fprintf(stderr,
                       "World %s was requested to use a spherical harmonic "
                       "gravity model, but does not have a file configured, "
                       "neither in 'Inp_Sim' nor in source. Add the file to "
                       "the main 'Model' directory and add the file name as a "
                       "'Model File' field to the Gravitation Model for this "
                       "world in 'Inp_Sim'. Exiting...\n",
                       W->Name);
               exit(EXIT_FAILURE);
            }
            strcpy(gravModel->modelFile, GravFileName[i]);
         }
         LoadGravModel(ModelPath, gravModel);
         if (gravModel->C != NULL) {
            if (gravModel->r_ref == 0)
               gravModel->r_ref = grav_r_ref[i];
            W->J2 = -gravModel->C[2][0] / gravModel->Norm[2][0];
         }
      }
   }

   worlds[EARTH].Atmo.GasColor[0]  = 0.17523;
   worlds[EARTH].Atmo.GasColor[1]  = 0.40785;
   worlds[EARTH].Atmo.GasColor[2]  = 1.0;
   worlds[EARTH].Atmo.DustColor[0] = 1.0;
   worlds[EARTH].Atmo.DustColor[1] = 1.0;
   worlds[EARTH].Atmo.DustColor[2] = 1.0;
   worlds[EARTH].Atmo.RayScat[0]   = 5.8E-6;
   worlds[EARTH].Atmo.RayScat[1]   = 13.5E-6;
   worlds[EARTH].Atmo.RayScat[2]   = 33.1E-6;
   worlds[EARTH].Atmo.RayScaleHt   = 8000.0;
   worlds[EARTH].Atmo.MieScat      = 4.0E-6;
   worlds[EARTH].Atmo.MieScaleHt   = 1200.0;
   worlds[EARTH].Atmo.MieG         = 0.76;
   worlds[EARTH].Atmo.MaxHt        = 8.0 * worlds[EARTH].Atmo.RayScaleHt;
   worlds[EARTH].Atmo.rad = worlds[EARTH].rad + worlds[EARTH].Atmo.MaxHt;

   worlds[MARS].Atmo.GasColor[0]  = 0.70588;
   worlds[MARS].Atmo.GasColor[1]  = 0.50196;
   worlds[MARS].Atmo.GasColor[2]  = 0.19608;
   worlds[MARS].Atmo.DustColor[0] = 0.5;
   worlds[MARS].Atmo.DustColor[1] = 0.5;
   worlds[MARS].Atmo.DustColor[2] = 0.5;
   worlds[MARS].Atmo.RayScat[0]   = 19.918E-6;
   worlds[MARS].Atmo.RayScat[1]   = 13.57E-6;
   worlds[MARS].Atmo.RayScat[2]   = 5.75E-6;
   worlds[MARS].Atmo.RayScaleHt   = 11000.0;
   worlds[MARS].Atmo.MieScat      = 0.0E-6;
   worlds[MARS].Atmo.MieScaleHt   = 11000.0;
   worlds[MARS].Atmo.MieG         = 0.76;
   worlds[MARS].Atmo.MaxHt        = 8.0 * worlds[MARS].Atmo.RayScaleHt;
   worlds[MARS].Atmo.rad          = worlds[MARS].rad + worlds[MARS].Atmo.MaxHt;

   JDType jd_tt_j2000 = jd;
   ChangeSystemEpoch(TT_TIME, J2000_EPOCH, &jd_tt_j2000);
   const double j2000sec_tt = JDToDynTime(jd_tt_j2000);
   /* .. Load planetary orbit elements for date of interest */
   for (i = MERCURY; i <= PLUTO; i++) {
      struct WorldType *W = &worlds[i];
      PlanetEphemerides(i, jd_tt_j2000, W->eph.mu, &W->eph.SMA, &W->eph.ecc,
                        &W->eph.inc, &W->eph.RAAN, &W->eph.ArgP, &W->eph.tp,
                        &W->eph.anom, &W->eph.SLR, &W->eph.alpha, &W->eph.rmin,
                        &W->eph.MeanMotion, &W->eph.Period);
      /* TODO: These ephems are expressed in mean-equinox-of-date (MEME) */
      /* Would it be worthwhile to transform to J2000? */
   }

   for (i = MERCURY; i <= PLUTO; i++) {
      struct WorldType *W = &worlds[i];
      if (i != EARTH) {
         A2C(312, (PoleRA[i] + 90.0) * D2R, (90.0 - PoleDec[i]) * D2R, 0.0,
             CNJ);
         MxM(CNJ, worlds[EARTH].CNH, W->CNH);
         C2Q(W->CNH, W->qnh);
         QxQT(W->qnh, qjh, W->qnj);
      }
   }

   /* .. Saturn's Rings */
   worlds[SATURN].HasRing   = 1;
   worlds[SATURN].RingInner = 67258.0E3;
   worlds[SATURN].RingOuter = 181328.0E3;

   for (i = MERCURY; i <= PLUTO; i++) {
      struct WorldType *W = &worlds[i];
      W->RadOfInfluence   = RadiusOfInfluence(W->eph.mu, W->mu, W->eph.SMA);
      W->Type             = PLANET;
   }

   for (i = MERCURY; i <= PLUTO; i++) {
      struct WorldType *W = &worlds[i];
      if (W->Exists) {
         Eph = &W->eph;
         Eph2RV(Eph->mu, Eph->SLR, Eph->ecc, Eph->inc, Eph->RAAN, Eph->ArgP,
                j2000sec_tt - Eph->tp, Eph->PosN, Eph->VelN, &Eph->anom);
         for (j = 0; j < 3; j++)
            W->PosH[j] = Eph->PosN[j];
         W->PriMerAng = fmod(W->PriMerAngJ2000 + W->w * j2000sec_tt, TwoPi);
         SimpRot(Zaxis, W->PriMerAng, W->CWN);
         C2Q(W->CWN, W->qwn);
      }
   }
   /* .. Earth rotation is a special case */
   GMST                    = JD2GMST(jd_tt_j2000);
   worlds[EARTH].PriMerAng = TwoPi * GMST;
   HiFiEarthPrecNute(jd_tt_j2000, C_TEME_TETE, C_TETE_J2000);
   SimpRot(Zaxis, worlds[EARTH].PriMerAng, C_W_TETE);
   MxM(C_W_TETE, C_TETE_J2000, worlds[EARTH].CWN);
   C2Q(worlds[EARTH].CWN, worlds[EARTH].qwn);

   strcpy(worlds[EARTH].BumpTexFileName, "EarthBump.ppm");
}
/*********************************************************************/
void NMoon(const WorldID planet, long *const n_moon, WorldID *const first_moon)
{
   switch (planet) {
      case MERCURY:
      case VENUS:
         *n_moon     = 0;
         *first_moon = 0;
         break;
      case EARTH:
         *n_moon     = 1;
         *first_moon = LUNA;
         break;
      case MARS:
         *n_moon     = 2;
         *first_moon = PHOBOS;
         break;
      case JUPITER:
         *n_moon     = 16;
         *first_moon = IO;
         break;
      case SATURN:
         *n_moon     = 18;
         *first_moon = MIMAS;
         break;
      case URANUS:
         *n_moon     = 5;
         *first_moon = ARIEL;
         break;
      case NEPTUNE:
         *n_moon     = 2;
         *first_moon = TRITON;
         break;
      case PLUTO:
         *n_moon     = 1;
         *first_moon = CHARON;
         break;
      default: {
         char w_name[32] = {'\0'};
         WorldID2String(planet, w_name);
         fprintf(stderr, "Planet %s with id %u has no moons. Exiting...\n",
                 w_name, planet);
         exit(EXIT_FAILURE);
      }
   }
}
/*********************************************************************/
void MoonDefaultData(const WorldID planet, const long Im, char name[40],
                     char map_file_name[30], char grav_file_name[20],
                     char geom_file_name[20], char ori_name[40],
                     double *const j2, double *const mu, double *const rad,
                     double *const w, double *const pole_ra,
                     double *const pole_dec, double *const sma,
                     double *const ecc, double *const inc, double *const raan,
                     double *const omg, double *const mean_anom,
                     DateType *const epoch_date)
{
   char p_name[32] = {'\0'};
   WorldID2String(planet, p_name);
   long n_moon;
   WorldID first_moon;
   NMoon(planet, &n_moon, &first_moon);
   if (Im >= n_moon) {
      fprintf(stderr,
              "Planet %s only has %ld moons and moon number %ld was requested. "
              "Exiting...\n",
              p_name, n_moon, Im);
      exit(EXIT_FAILURE);
   }
   switch (planet) {
      case EARTH: {
         const char Names[][40]            = {"Luna"};
         const char MapFileNames[][40]     = {"Rockball"};
         const char GravFileNames[][20]    = {""};
         const char GeomFileNames[][20]    = {"Phobos.obj"};
         const char OrientationNames[][40] = {"PHOBOS"};
         const double mus[]                = {7.158E5};
         const double rads[]               = {11.1E3};
         const double ws[]                 = {2.66E-6};
         const double j2s[]                = {2.027E-4};
         const double PoleRAs[]            = {0.0};
         const double PoleDecs[]           = {0.0};
         const double SMAs[]               = {384400000.0};
         const double eccs[]               = {0.0549};
         const double incs[]               = {0.0};
         const double RAANs[]              = {0.0};
         const double omgs[]               = {0.0};
         const double MeanAnoms[]          = {0.0};
         const long EpochYears[]           = {2000};
         const long EpochMons[]            = {1};
         const long EpochDays[]            = {1};
         const long EpochHours[]           = {12};

         strcat(name, Names[Im]);
         strcat(map_file_name, MapFileNames[Im]);
         strcat(grav_file_name, GravFileNames[Im]);
         strcat(geom_file_name, GeomFileNames[Im]);
         strcat(ori_name, OrientationNames[Im]);

         *j2        = j2s[Im];
         *mu        = mus[Im];
         *rad       = rads[Im];
         *w         = ws[Im];
         *pole_ra   = PoleRAs[Im];
         *pole_dec  = PoleDecs[Im];
         *sma       = SMAs[Im];
         *ecc       = eccs[Im];
         *inc       = incs[Im];
         *raan      = RAANs[Im];
         *omg       = omgs[Im];
         *mean_anom = MeanAnoms[Im];
         *epoch_date =
             DateTypeInit(TT_TIME, EpochYears[Im], EpochMons[Im], EpochDays[Im],
                          EpochHours[Im], 0, (Rational){0});
      } break;
      case MARS: {
         const char Names[][40]            = {"Phobos", "Deimos"};
         const char MapFileNames[][40]     = {"Rockball", "Rockball"};
         const char GravFileNames[][20]    = {"", ""};
         const char GeomFileNames[][20]    = {"Phobos.obj", ""};
         const char OrientationNames[][40] = {"PHOBOS", "DEIMOS"};
         const double mus[]                = {7.158E5, 9.8E4};
         const double rads[]               = {11.1E3, 6.2E3};
         const double ws[]                 = {0.0, 0.0};
         const double j2s[]                = {0.0, 0.0};
         const double PoleRAs[]            = {0.0, 0.0};
         const double PoleDecs[]           = {0.0, 0.0};
         const double SMAs[]               = {9380.0E3, 23460.0E3};
         const double eccs[]               = {0.0151, 0.0002};
         const double incs[]               = {1.075, 1.793};
         const double RAANs[]              = {164.931, 339.600};
         const double omgs[]               = {150.247, 290.496};
         const double MeanAnoms[]          = {92.474, 296.230};
         const long EpochYears[]           = {1950, 1950};
         const long EpochMons[]            = {1, 1};
         const long EpochDays[]            = {1, 1};

         strcat(name, Names[Im]);
         strcat(map_file_name, MapFileNames[Im]);
         strcat(grav_file_name, GravFileNames[Im]);
         strcat(geom_file_name, GeomFileNames[Im]);
         strcat(ori_name, OrientationNames[Im]);

         *j2         = j2s[Im];
         *mu         = mus[Im];
         *rad        = rads[Im];
         *w          = ws[Im];
         *pole_ra    = PoleRAs[Im];
         *pole_dec   = PoleDecs[Im];
         *sma        = SMAs[Im];
         *ecc        = eccs[Im];
         *inc        = incs[Im];
         *raan       = RAANs[Im];
         *omg        = omgs[Im];
         *mean_anom  = MeanAnoms[Im];
         *epoch_date = DateTypeInit(TT_TIME, EpochYears[Im], EpochMons[Im],
                                    EpochDays[Im], 0, 0, (Rational){0});
      } break;
      case JUPITER: {
         const char Names[][40] = {
             "Io",    "Europa",   "Ganymede", "Callisto", "Amalthea", "Himalia",
             "Elara", "Pasiphae", "Sinope",   "Lysithea", "Carme",    "Ananke",
             "Leda",  "Thebe",    "Adrastea", "Metis"};
         const char MapFileNames[][40] = {
             "NONE", "Iceball", "NONE", "NONE", "NONE", "NONE", "NONE", "NONE",
             "NONE", "NONE",    "NONE", "NONE", "NONE", "NONE", "NONE", "NONE"};
         const char GravFileNames[][20]    = {"", "", "", "", "", "", "", "",
                                              "", "", "", "", "", "", "", ""};
         const char GeomFileNames[][20]    = {"", "", "", "", "", "", "", "",
                                              "", "", "", "", "", "", "", ""};
         const char OrientationNames[][40] = {
             "IO",       "EUROPA",  "GANYMEDE", "CALLISTO",
             "AMALTHEA", "JUPITER", "JUPITER",  "JUPITER",
             "JUPITER",  "JUPITER", "JUPITER",  "JUPITER",
             "JUPITER",  "THEBE",   "ADRASTEA", "METIS"};
         const double mus[]  = {5.959E9, 3202.739E9, 9887.834E9, 7179.289E9,
                                1.38E8,  4.5E8,      5.8E7,      2.0E7,
                                5.0E6,   4.2E6,      8.8E6,      2.0E6,
                                7.3E5,   1.0E8,      5.0E5,      8.0E6};
         const double rads[] = {1821.6E3, 1560.8E3, 2631.2E3, 2410.3E3,
                                83.45E3,  85.0E3,   43.0E3,   30.0E3,
                                19.0E3,   18.0E3,   23.0E3,   14.0E3,
                                10.0E3,   49.3E3,   8.2E3,    21.5E3};
         const double ws[]   = {
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         };
         const double j2s[] = {
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         };
         const double PoleRAs[] = {
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         };
         const double PoleDecs[] = {
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         };
         const double SMAs[] = {4.2180E8,  6.7110E8,  1.07040E9, 1.8827E9,
                                1.814E8,   1.1461E10, 1.1741E10, 2.3624E10,
                                2.3939E10, 1.1717E10, 2.3404E10, 2.1276E10,
                                1.1165E10, 2.219E8,   1.29E8,    1.28E8};
         const double eccs[] = {0.0041, 0.0094, 0.0013, 0.0074, 0.0032, 0.1623,
                                0.2174, 0.4090, 0.2495, 0.1124, 0.2533, 0.2435,
                                0.1636, 0.0176, 0.0018, 0.0012};
         const double incs[] = {
             0.036,   0.466,  0.177,   0.192,   0.380,  27.496, 26.627, 151.431,
             158.109, 28.302, 164.907, 148.889, 27.457, 1.08,   0.054,  0.019};
         const double RAANs[]     = {43.977,  219.106, 63.552,  298.848,
                                     108.946, 57.245,  109.373, 312.990,
                                     303.081, 5.528,   113.738, 7.615,
                                     217.137, 235.694, 228.378, 146.912};
         const double omgs[]      = {84.129,  88.97,   192.417, 52.643,
                                     155.873, 331.995, 143.591, 170.45,
                                     346.394, 49.486,  28.199,  100.619,
                                     272.349, 234.269, 328.047, 297.177};
         const long EpochYears[]  = {1997, 1997, 1997, 1997, 1997, 2000,
                                     2000, 2000, 2000, 2000, 2000, 2000,
                                     2000, 1997, 1997, 1997};
         const long EpochMons[]   = {1, 1, 1, 1, 1, 1, 1, 1,
                                     1, 1, 1, 1, 1, 1, 1, 1};
         const long EpochDays[]   = {16, 16, 16, 16, 16, 1,  1,  1,
                                     1,  1,  1,  1,  1,  16, 16, 16};
         const double MeanAnoms[] = {342.021, 171.016, 317.54,  181.408,
                                     185.194, 68.721,  332.962, 280.193,
                                     168.397, 329.121, 234.027, 248.793,
                                     228.076, 135.956, 135.673, 276.047};

         strcat(name, Names[Im]);
         strcat(map_file_name, MapFileNames[Im]);
         strcat(grav_file_name, GravFileNames[Im]);
         strcat(geom_file_name, GeomFileNames[Im]);
         strcat(ori_name, OrientationNames[Im]);

         *j2         = j2s[Im];
         *mu         = mus[Im];
         *rad        = rads[Im];
         *w          = ws[Im];
         *pole_ra    = PoleRAs[Im];
         *pole_dec   = PoleDecs[Im];
         *sma        = SMAs[Im];
         *ecc        = eccs[Im];
         *inc        = incs[Im];
         *raan       = RAANs[Im];
         *omg        = omgs[Im];
         *mean_anom  = MeanAnoms[Im];
         *epoch_date = DateTypeInit(TT_TIME, EpochYears[Im], EpochMons[Im],
                                    EpochDays[Im], 0, 0, (Rational){0});
      } break;
      case SATURN: {
         const char Names[][40] = {
             "Mimas",      "Enceladus", "Tethys",  "Dione",   "Rhea",
             "Titan",      "Hyperion",  "Iapetus", "Phoebe",  "Janus",
             "Epimetheus", "Helene",    "Telesto", "Calypso", "Atlas",
             "Prometheus", "Pandora",   "Pan"};
         const char MapFileNames[][40] = {
             "NONE", "Iceball2", "NONE", "NONE", "NONE", "NONE",
             "NONE", "NONE",     "NONE", "NONE", "NONE", "NONE",
             "NONE", "NONE",     "NONE", "NONE", "NONE", "NONE"};
         const char GravFileNames[][20] = {"", "", "", "", "", "", "", "", "",
                                           "", "", "", "", "", "", "", "", ""};
         const char GeomFileNames[][20] = {"", "", "", "", "", "", "", "", "",
                                           "", "", "", "", "", "", "", "", ""};
         const char OrientationNames[][40] = {
             "MIMAS",      "ENCELADUS", "TETHYS",  "DIONE",   "RHEA",
             "TITAN",      "SATURN",    "IAPETUS", "PHOEBE",  "JANUS",
             "EPIMETHEUS", "HELENE",    "TELESTO", "CALYPSO", "ATLAS",
             "PROMETHEUS", "PANDORA",   "PAN"};
         const double mus[] = {
             2.53E9, 7.21E9,   4.121E10, 7.3113E10, 1.5407E11, 8.97819E12,
             3.7E8,  1.205E11, 5.531E8,  1.266E8,   3.51E7,    1.7E6,
             4.8E5,  2.4E5,    1.4E5,    1.246E7,   9.95E6,    3.3E5};
         const double rads[]    = {198.8E3,  252.3E3, 536.3E3, 562.5E3, 764.5E3,
                                   2575.5E3, 133.0E3, 734.5E3, 106.6E3, 90.4E3,
                                   58.3E3,   16.0E3,  12.0E3,  9.5E3,   10.E3,
                                   46.8e3,   40.6E3,  12.8E3};
         const double ws[]      = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
         const double j2s[]     = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
         const double PoleRAs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
         const double PoleDecs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
         const double SMAs[]     = {
             1.8554E8,  2.3804E8,  2.9467E8,    3.7742E8, 5.2707E8, 1.22187E9,
             1.50088E9, 3.56084E9, 1.294778E10, 1.5146E8, 1.5141E8, 3.7742E8,
             2.9471E8,  2.9471E8,  1.3767E8,    1.3938E8, 1.4172E8, 1.3358E8};
         const double eccs[]  = {0.0196, 0.0047, 0.0001, 0.0022, 0.001,  0.0288,
                                 0.0274, 0.0283, 0.1635, 0.0068, 0.0098, 0.0071,
                                 0.0002, 0.0005, 0.0012, 0.0022, 0.0042, 0.0};
         const double incs[]  = {1.572, 0.009, 1.091,   0.028, 0.331, 0.28,
                                 0.63,  7.489, 175.986, 0.163, 0.351, 0.213,
                                 1.18,  1.499, 0.003,   0.008, 0.05,  0.001};
         const double RAANs[] = {153.152, 93.204,  330.882, 168.909, 311.531,
                                 24.502,  264.022, 75.831,  241.57,  46.899,
                                 85.244,  40.039,  300.256, 25.327,  0.5,
                                 259.504, 327.215, 40.557};
         const double omgs[]  = {14.352,  211.923, 262.845, 168.82,  256.609,
                                 185.671, 324.183, 275.921, 345.582, 241.778,
                                 312.63,  292.056, 341.795, 234.788, 331.521,
                                 164.389, 83.461,  139.318};
         const long EpochYears[]  = {2004, 2004, 2004, 2004, 2004, 2004,
                                     2004, 2004, 2004, 2004, 2004, 2004,
                                     2004, 2004, 2004, 2004, 2004, 2004};
         const long EpochMons[]   = {1, 1, 1, 1, 1, 1, 1, 1, 1,
                                     1, 1, 1, 1, 1, 1, 1, 1, 1};
         const long EpochDays[]   = {1, 1, 1, 1, 1, 1, 1, 1, 1,
                                     1, 1, 1, 1, 1, 1, 1, 1, 1};
         const double MeanAnoms[] = {
             255.312, 197.047, 189.003, 65.99,   311.551, 15.154,
             295.906, 356.029, 287.593, 242.754, 308.322, 134.07,
             200.143, 101.961, 157.738, 242.224, 202.697, 246.065};

         strcat(name, Names[Im]);
         strcat(map_file_name, MapFileNames[Im]);
         strcat(grav_file_name, GravFileNames[Im]);
         strcat(geom_file_name, GeomFileNames[Im]);
         strcat(ori_name, OrientationNames[Im]);

         *j2         = j2s[Im];
         *mu         = mus[Im];
         *rad        = rads[Im];
         *w          = ws[Im];
         *pole_ra    = PoleRAs[Im];
         *pole_dec   = PoleDecs[Im];
         *sma        = SMAs[Im];
         *ecc        = eccs[Im];
         *inc        = incs[Im];
         *raan       = RAANs[Im];
         *omg        = omgs[Im];
         *mean_anom  = MeanAnoms[Im];
         *epoch_date = DateTypeInit(TT_TIME, EpochYears[Im], EpochMons[Im],
                                    EpochDays[Im], 0, 0, (Rational){0});
      } break;
      case URANUS: {
         const char Names[][40] = {"Ariel", "Umbriel", "Titania", "Oberon",
                                   "Miranda"};
         const char MapFileNames[][40]  = {"NONE", "NONE", "NONE", "NONE",
                                           "NONE"};
         const char GravFileNames[][20] = {"", "", "", "", ""};
         const char GeomFileNames[][20] = {"", "", "", "", ""};
         const double mus[]     = {90.3E9, 78.2E9, 235.3E9, 201.1E9, 4.4E9};
         const double rads[]    = {578.9E3, 584.7E3, 788.9E3, 761.4E3, 235.8E3};
         const double ws[]      = {0.0, 0.0, 0.0, 0.0, 0.0};
         const double j2s[]     = {0.0, 0.0, 0.0, 0.0, 0.0};
         const double PoleRAs[] = {0.0, 0.0, 0.0, 0.0, 0.0};
         const double PoleDecs[] = {0.0, 0.0, 0.0, 0.0, 0.0};
         const double SMAs[]     = {1.909E8, 2.66E8, 4.363E8, 5.835E8, 1.299E8};
         const double eccs[]     = {0.0012, 0.0039, 0.0011, 0.0014, 0.0013};
         const double incs[]     = {0.041, 0.128, 0.079, 0.068, 4.338};
         const double RAANs[]    = {22.394, 33.485, 99.771, 279.771, 326.438};
         const double omgs[]     = {115.349, 84.709, 284.4, 104.4, 68.312};
         const long EpochYears[] = {1980, 1980, 1980, 1980, 1980};
         const long EpochMons[]  = {1, 1, 1, 1, 1};
         const long EpochDays[]  = {1, 1, 1, 1, 1};
         const double MeanAnoms[] = {39.481, 12.469, 24.614, 283.088, 311.33};
         const char OrientationNames[][40] = {"ARIEL", "UMBRIEL", "TITANIA",
                                              "OBERON", "MIRANDA"};

         strcat(name, Names[Im]);
         strcat(map_file_name, MapFileNames[Im]);
         strcat(grav_file_name, GravFileNames[Im]);
         strcat(geom_file_name, GeomFileNames[Im]);
         strcat(ori_name, OrientationNames[Im]);

         *j2         = j2s[Im];
         *mu         = mus[Im];
         *rad        = rads[Im];
         *w          = ws[Im];
         *pole_ra    = PoleRAs[Im];
         *pole_dec   = PoleDecs[Im];
         *sma        = SMAs[Im];
         *ecc        = eccs[Im];
         *inc        = incs[Im];
         *raan       = RAANs[Im];
         *omg        = omgs[Im];
         *mean_anom  = MeanAnoms[Im];
         *epoch_date = DateTypeInit(TT_TIME, EpochYears[Im], EpochMons[Im],
                                    EpochDays[Im], 0, 0, (Rational){0});
      } break;
      case NEPTUNE: {
         const char Names[][40]            = {"Triton", "Nereid"};
         const char MapFileNames[][40]     = {"NONE", "NONE"};
         const char GravFileNames[][20]    = {"", ""};
         const char GeomFileNames[][20]    = {"", ""};
         const double mus[]                = {1427.9E9, 2.06E9};
         const double rads[]               = {1353.4E3, 170.0E3};
         const double ws[]                 = {0.0, 0.0};
         const double j2s[]                = {0.0, 0.0};
         const double PoleRAs[]            = {0.0, 0.0};
         const double PoleDecs[]           = {0.0, 0.0};
         const double SMAs[]               = {3.548E8, 5.5134E9};
         const double eccs[]               = {0.0, 0.7512};
         const double incs[]               = {156.834, 7.232};
         const double RAANs[]              = {172.431, 334.762};
         const double omgs[]               = {344.046, 280.83};
         const long EpochYears[]           = {1989, 1989};
         const long EpochMons[]            = {8, 8};
         const long EpochDays[]            = {25, 25};
         const double MeanAnoms[]          = {264.775, 359.341};
         const char OrientationNames[][40] = {"TRITON", "NEPTUNE"};

         strcat(name, Names[Im]);
         strcat(map_file_name, MapFileNames[Im]);
         strcat(grav_file_name, GravFileNames[Im]);
         strcat(geom_file_name, GeomFileNames[Im]);
         strcat(ori_name, OrientationNames[Im]);

         *j2         = j2s[Im];
         *mu         = mus[Im];
         *rad        = rads[Im];
         *w          = ws[Im];
         *pole_ra    = PoleRAs[Im];
         *pole_dec   = PoleDecs[Im];
         *sma        = SMAs[Im];
         *ecc        = eccs[Im];
         *inc        = incs[Im];
         *raan       = RAANs[Im];
         *omg        = omgs[Im];
         *mean_anom  = MeanAnoms[Im];
         *epoch_date = DateTypeInit(TT_TIME, EpochYears[Im], EpochMons[Im],
                                    EpochDays[Im], 0, 0, (Rational){0});
      } break;
      case PLUTO: {
         const char Names[][40]            = {"Charon"};
         const char MapFileNames[][40]     = {"Iceball"};
         const char GravFileNames[][20]    = {""};
         const char GeomFileNames[][20]    = {""};
         const double mus[]                = {108.0E9};
         const double rads[]               = {593.0E3};
         const double ws[]                 = {0.0};
         const double j2s[]                = {0.0};
         const double PoleRAs[]            = {0.0};
         const double PoleDecs[]           = {0.0};
         const double SMAs[]               = {1.7536E7};
         const double eccs[]               = {0.0022};
         const double incs[]               = {0.001};
         const double RAANs[]              = {85.187};
         const double omgs[]               = {71.255};
         const long EpochYears[]           = {2000};
         const long EpochMons[]            = {1};
         const long EpochDays[]            = {1};
         const long EpochHours[]           = {12};
         const double MeanAnoms[]          = {147.848};
         const char OrientationNames[][40] = {"CHARON"};

         strcat(name, Names[Im]);
         strcat(map_file_name, MapFileNames[Im]);
         strcat(grav_file_name, GravFileNames[Im]);
         strcat(geom_file_name, GeomFileNames[Im]);
         strcat(ori_name, OrientationNames[Im]);

         *j2        = j2s[Im];
         *mu        = mus[Im];
         *rad       = rads[Im];
         *w         = ws[Im];
         *pole_ra   = PoleRAs[Im];
         *pole_dec  = PoleDecs[Im];
         *sma       = SMAs[Im];
         *ecc       = eccs[Im];
         *inc       = incs[Im];
         *raan      = RAANs[Im];
         *omg       = omgs[Im];
         *mean_anom = MeanAnoms[Im];
         *epoch_date =
             DateTypeInit(TT_TIME, EpochYears[Im], EpochMons[Im], EpochDays[Im],
                          EpochHours[Im], 0, (Rational){0});
      } break;
      default: {
         fprintf(
             stderr,
             "Planet %s with id %u has no moon configurations. Exiting...\n",
             p_name, planet);
         exit(EXIT_FAILURE);
      }
   }
}
/*********************************************************************/
void LoadMoons(const ephemType ephem, const JDType jd,
               const JPLHeaderType *const jpl_hdr,
               struct WorldType *const worlds)
{
   // TODO: wrapping in LoadPlanets() into this function? a lot of what is done
   // in LoadPlanets is in here too
   for (WorldID p_id = MERCURY; p_id <= PLUTO; p_id++) {
      struct WorldType *P = &worlds[p_id];
      if (P->Exists) {
         long n_moon = 0;
         WorldID first_moon;
         NMoon(p_id, &n_moon, &first_moon);

         char p_name[32] = {'\0'};
         WorldID2String(p_id, p_name);

         P->Nsat = n_moon;
         P->Sat  = (WorldID *)calloc(n_moon, sizeof(long));
         if (P->Sat == NULL) {
            fprintf(stderr,
                    "%s P->Sat calloc returned null pointer. Exiting...\n",
                    p_name);
            exit(EXIT_FAILURE);
         }

         for (long im = 0; im < n_moon; im++) {
            const WorldID m_id  = im + first_moon;
            struct WorldType *M = &worlds[m_id];

            float color[4]          = {1.0};
            unsigned char glyph[14] = {'\0'};
            char name[40] = {'\0'}, map_file_name[30] = {'\0'},
                 grav_file_name[20] = {'\0'}, geom_file_name[20] = {'\0'},
                 ori_name[40] = {'\0'}, col_tex_f_name[40] = {"NONE"},
                 bmp_tex_f_name[40] = {"NONE"};
            double j2, mu, rad, w, pole_ra, pole_dec, sma, ecc, inc, raan, omg,
                mean_anom;
            DateType epoch_date;
            MoonDefaultData(p_id, im, name, map_file_name, grav_file_name,
                            geom_file_name, ori_name, &j2, &mu, &rad, &w,
                            &pole_ra, &pole_dec, &sma, &ecc, &inc, &raan, &omg,
                            &mean_anom, &epoch_date);
            double primerang_j2000 = 0.0;
#ifdef _ENABLE_SPICE_
            if (EphemOption == EPH_SPICE) {
               // If we are using SPICE, replace the hardcoded values with SPICE
               // values
               int dim;
               double tmp_3[3];
               bodvrd_c(name, "GM", 1, &dim, &mu);
               mu = mu * 1E9;

               if (m_id == LUNA)
                  bodvrd_c("Moon", "J2", 1, &dim, &j2);

               bodvrd_c(name, "RADII", 3, &dim, tmp_3);
               rad = tmp_3[0] * 1e3;

               bodvrd_c(ori_name, "PM", 3, &dim, tmp_3);
               primerang_j2000 = tmp_3[0];
               w               = tmp_3[1] * D2R / spd_c();
               // converts the prime meridian rate in deg/day to rad/s
               bodvrd_c(ori_name, "POLE_RA", 3, &dim, tmp_3);
               pole_ra = tmp_3[0];

               bodvrd_c(ori_name, "POLE_DEC", 3, &dim, tmp_3);
               pole_dec = tmp_3[0];
            }
#endif
            if (m_id == LUNA) {
               // LUNA has a lot of special cases for its parameters
               switch (ephem) {
                  case EPH_MEAN:
                     mu  = 4.902801E12;
                     rad = 1.738E6;
                     break;
                  case EPH_GMAT421:
                  case EPH_GMAT424:
                     mu  = 4.9028005821478E12;
                     rad = 1.7382E6;
                     break;
                  case EPH_SPICE:
                     break;
                  default:
                     // handle the DE cases here
                     mu  = getDEHeader1041Data(jpl_hdr, "GMB");
                     mu  = (mu / (1.0 + EMRAT)) * AUd2ms;
                     rad = 1.7374E6;
                     break;
               }
               strcpy(col_tex_f_name, "LunaCol.ppm");
               strcpy(bmp_tex_f_name, "LunaBump.ppm");

               const float luna_color[4] = {0.440417f, 0.441343f, 0.441084f,
                                            1.0f};

               const unsigned char luna_glyph[14] = {
                   0xc0, 0xc0, 0x00, 0x00, 0x18, 0x70, 0x60,
                   0xe0, 0xe0, 0x60, 0x70, 0x18, 0x00, 0x00};

               for (int i = 0; i < 4; i++)
                  color[i] = luna_color[i];
               for (int i = 0; i < 14; i++)
                  glyph[i] = luna_glyph[i];
            }

            struct OrbitType *E = &M->eph;
            P->Sat[im]          = m_id;

            M->Exists = TRUE;
            M->Parent = p_id;
            strcpy(M->Name, name);
            strcpy(M->MapFileName, map_file_name);
            strcpy(M->ColTexFileName, col_tex_f_name);
            strcpy(M->BumpTexFileName, bmp_tex_f_name);

            M->mu        = mu;
            M->rad       = rad;
            M->w         = w;
            M->PriMerAng = 0.0;
            E->Exists    = TRUE;
            E->Regime    = ORB_CENTRAL;
            E->World     = p_id;
            E->mu        = P->mu;
            E->SMA       = sma;
            E->ecc       = ecc;
            E->inc       = inc * D2R;
            E->RAAN      = raan * D2R;
            E->ArgP      = omg * D2R;

            M->PriMerAngJ2000 = primerang_j2000 * D2R;

            double epoch_j2000_sec = Date2Time(epoch_date);
            E->MeanMotion          = sqrt(E->mu / (E->SMA * E->SMA * E->SMA));
            E->Period              = TwoPi / E->MeanMotion;
            E->tp = epoch_j2000_sec - mean_anom * D2R / E->MeanMotion;
            const double tt_j2000_sec_0 = JDToDynTime(JD_TT_MJD_0);
            while ((E->tp - tt_j2000_sec_0) < -E->Period)
               E->tp += E->Period;
            while ((E->tp - tt_j2000_sec_0) > E->Period)
               E->tp -= E->Period;
            E->alpha = 1.0 / E->SMA;
            E->SLR   = E->SMA * (1.0 - E->ecc * E->ecc);
            E->rmin  = E->SMA * (1.0 - E->ecc);

            const double j2000sec_tt = JDToDynTime(jd);
            E->anom = TrueAnomaly(E->mu, E->SLR, E->ecc, j2000sec_tt - E->tp);
            M->RadOfInfluence = RadiusOfInfluence(P->mu, M->mu, E->SMA);

            if (EphemOption != EPH_SPICE) {
               /* CNH assumed to be same as parent planet */
               for (int i = 0; i < 3; i++) {
                  for (int j = 0; j < 3; j++)
                     M->CNH[i][j] = P->CNH[i][j];
               }
            }
            else {
               double CNJ[3][3];
               A2C(312, (pole_ra + 90.0) * D2R, (90.0 - pole_dec) * D2R, 0.0,
                   CNJ);
               MxM(CNJ, worlds[EARTH].CNH, M->CNH);
               C2Q(M->CNH, M->qnh);
            }

            C2Q(M->CNH, M->qnh);
            QxQT(M->qnh, qjh, M->qnj);
            for (int i = 0; i < 4; i++)
               M->Color[i] = color[i];
            for (int i = 0; i < 14; i++)
               M->Glyph[i] = glyph[i];
            M->Type = MOON;

            /* Gravitation Model */
            if (GravPertActive) {
               struct SphereHarmType *gravModel = &M->GravModel;
               if (!strcmp(gravModel->modelFile, "")) {
                  if (!strcmp(grav_file_name, "") && gravModel->N > 1) {
                     fprintf(
                         stderr,
                         "World %s was requested to use a spherical harmonic "
                         "gravity model, but does not have a file configured, "
                         "neither in 'Inp_Sim' nor in source. Add the file to "
                         "the main 'Model' directory and add the file name as "
                         "a 'Model File' field to the Gravitation Model for "
                         "this world in 'Inp_Sim'. Exiting...\n",
                         M->Name);
                     exit(EXIT_FAILURE);
                  }
                  strcpy(gravModel->modelFile, grav_file_name);
               }
               LoadGravModel(ModelPath, gravModel);
               if (im != LUNA && gravModel->C != NULL) {
                  if (gravModel->r_ref == 0)
                     gravModel->r_ref = rad;
                  M->J2 = -gravModel->C[2][0] / gravModel->Norm[2][0];
               }
            }

            strcpy(M->GeomFileName, geom_file_name);
            if (strcmp(M->GeomFileName, "")) {
               Geom =
                   LoadWingsObjFile(ModelPath, M->GeomFileName, &Matl, &Nmatl,
                                    Geom, &Ngeom, &M->GeomTag, FALSE);
            }
         }
      }
   }
}
/*********************************************************************/
void LoadMinorBodies(const ephemType ephem, const JDType jd,
                     const JPLHeaderType *const jpl_hdr,
                     struct WorldType *const worlds)
{
   // TODO: change minorbodies.txt to yaml
   // TODO: Arbitrarily many minor bodies (this will take effort)
   // TODO: SPICE minor bodies??
   FILE *infile;
   struct WorldType *W;
   struct OrbitType *E;
   char junk[120], newline, response[120];
   long Ib, i;
   double CNJ[3][3], PoleRA, PoleDec, Epoch;
   const double ZAxis[3] = {0.0, 0.0, 1.0};
   char GravFileName[32] = {0};

   infile = FileOpen(ModelPath, "MinorBodies.txt", "r");
   fscanf(infile, "%[^\n] %[\n]", junk, &newline);
   fscanf(infile, "%ld %[^\n] %[\n]", &Nmb, junk, &newline);
   if (Nmb > 10) {
      fprintf(stderr,
              "Only 10 minor bodies are supported.  Adjust NWORLD to suit.\n");
      exit(EXIT_FAILURE);
   }
   for (Ib = 0; Ib < Nmb; Ib++) {
      DateType EpochDate = {0};

      W = &worlds[MINORBODY_0 + Ib];
      E = &W->eph;
      fscanf(infile, "%[^\n] %[\n]", junk, &newline);
      fscanf(infile, "%s %[^\n] %[\n]", response, junk, &newline);
      W->Exists = DecodeString(response);
      fscanf(infile, "\"%[^\"]\" %[^\n] %[\n]", W->Name, junk, &newline);
      fscanf(infile, "%s %[^\n] %[\n]", response, junk, &newline);
      W->Type = DecodeString(response);
      fscanf(infile, "%s %[^\n] %[\n]", W->MapFileName, junk, &newline);
      fscanf(infile, "%s %[^\n] %[\n]", W->GeomFileName, junk, &newline);
      fscanf(infile, "%s %[^\n] %[\n]", W->ColTexFileName, junk, &newline);
      fscanf(infile, "%s %[^\n] %[\n]", W->BumpTexFileName, junk, &newline);
      double mu = 0;
      fscanf(infile, "%lf %[^\n] %[\n]", &mu, junk, &newline);
      if (W->mu == 0) // TODO: maybe, maybe not
         W->mu = mu;
      fscanf(infile, "%lf %[^\n] %[\n]", &W->rad, junk, &newline);
      fscanf(infile, "%s %[^\n] %[\n]", GravFileName, junk, &newline);
      fscanf(infile, "%lf %[^\n] %[\n]", &W->w, junk, &newline);
      fscanf(infile, "%lf %lf %[^\n] %[\n]", &PoleRA, &PoleDec, junk, &newline);
      A2C(312, (PoleRA + 90.0) * D2R, (90.0 - PoleDec) * D2R, 0.0, CNJ);
      MxM(CNJ, worlds[EARTH].CNH, W->CNH);
      C2Q(W->CNH, W->qnh);
      QxQT(W->qnh, qjh, W->qnj);
      E->Exists = TRUE;
      E->Regime = ORB_CENTRAL;
      E->World  = SOL;
      E->mu     = worlds[SOL].mu;
      fscanf(infile, "%lf %[^\n] %[\n]", &E->SMA, junk, &newline);
      fscanf(infile, "%lf %[^\n] %[\n]", &E->ecc, junk, &newline);
      fscanf(infile, "%lf %[^\n] %[\n]", &E->inc, junk, &newline);
      fscanf(infile, "%lf %[^\n] %[\n]", &E->RAAN, junk, &newline);
      fscanf(infile, "%lf %[^\n] %[\n]", &E->ArgP, junk, &newline);
      fscanf(infile, "%ld %ld %ld %ld %[^\n] %[\n]", &EpochDate.Year,
             &EpochDate.Month, &EpochDate.Day, &EpochDate.Hour, junk, &newline);
      fscanf(infile, "%lf %[^\n] %[\n]", &E->anom, junk, &newline);
      Epoch         = Date2Time(EpochDate);
      E->MeanMotion = sqrt(E->mu / (E->SMA * E->SMA * E->SMA));
      E->Period     = TwoPi / E->MeanMotion;
      E->alpha      = 1.0 / E->SMA;
      E->SLR        = E->SMA * (1.0 - E->ecc * E->ecc);
      E->rmin       = E->SMA * (1.0 - E->ecc);
      E->tp = Epoch - TimeSincePeriapsis(E->mu, E->SLR, E->ecc, E->anom);
      const double tt_j2000_sec_0 = JDToDynTime(JD_TT_MJD_0);
      while ((E->tp - tt_j2000_sec_0) < -E->Period)
         E->tp += E->Period;
      while ((E->tp - tt_j2000_sec_0) > E->Period)
         E->tp -= E->Period;

      Geom = LoadWingsObjFile(ModelPath, W->GeomFileName, &Matl, &Nmatl, Geom,
                              &Ngeom, &W->GeomTag, TRUE);
      W->Density = W->mu / (6.67408E-11 * PolyhedronVolume(&Geom[W->GeomTag]));

      W->Parent         = SOL;
      W->Nsat           = 0;
      W->RadOfInfluence = 100.0E3; /* Being generous */
      W->DipoleMoment   = 0.0;
      W->DipoleAxis[2]  = 1.0;
      W->Atmo.Exists    = FALSE;
      W->HasRing        = FALSE;
      for (i = 0; i < 3; i++)
         W->Color[i] = 0.5;
      W->Color[3] = 1.0;

      const double j2000sec_tt = JDToDynTime(jd);
      Eph2RV(E->mu, E->SLR, E->ecc, E->inc, E->RAAN, E->ArgP,
             j2000sec_tt - E->tp, E->PosN, E->VelN, &E->anom);
      for (i = 0; i < 3; i++) {
         W->PosH[i] = E->PosN[i];
         W->VelH[i] = E->VelN[i];
      }
      W->PriMerAng = fmod(W->w * j2000sec_tt, TwoPi);
      SimpRot(ZAxis, W->PriMerAng, W->CWN);
      C2Q(W->CWN, W->qwn);

      /* Gravitation Model */
      if (GravPertActive) {
         if (strcmp(GravFileName, "NONE") == 0)
            strcpy(GravFileName, "");
         struct SphereHarmType *gravModel = &W->GravModel;
         if (!strcmp(gravModel->modelFile, "")) {
            if (!strcmp(GravFileName, "") && gravModel->N > 1) {
               fprintf(stderr,
                       "World %s was requested to use a spherical harmonic "
                       "gravity model, but does not have a file configured, "
                       "neither in 'Inp_Sim' or in source. Add the file to the "
                       "main 'Model' directory and add the file name as a "
                       "'Model File' field to the Gravitation Model for this "
                       "world in 'Inp_Sim'. Exiting...\n",
                       W->Name);
               exit(EXIT_FAILURE);
            }
            strcpy(gravModel->modelFile, GravFileName);
         }
         LoadGravModel(ModelPath, gravModel);
         if (gravModel->C != NULL) {
            if (gravModel->r_ref == 0)
               gravModel->r_ref = W->rad;
            W->J2 = -gravModel->C[2][0] / gravModel->Norm[2][0];
         }
      }
   }
   fclose(infile);
}
/**********************************************************************/
void LoadRegions(void)
{
   struct fy_document *fyd =
       fy_document_build_and_check(NULL, InOutPath, "Inp_Region.yaml");
   struct fy_node *root = fy_document_root(fyd);
   struct fy_node *node = fy_node_by_path_def(root, "/Regions");
   Nrgn                 = fy_node_sequence_item_count(node);
   Rgn = (struct RegionType *)calloc(Nrgn, sizeof(struct RegionType));
   struct fy_node *iterNode = NULL;
   long Ir                  = 0;

   WHILE_FY_ITER(node, iterNode)
   {
      struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/Region");
      struct RegionType *R    = &Rgn[Ir];
      char IsPosW[120] = {0}, world_str[20] = {0};
      R->Exists = getYAMLBool(fy_node_by_path_def(seqNode, "/Exists"));
      if (fy_node_scanf(seqNode,
                        "/Name %19s "
                        "/World %19s "
                        "/Location/Type %119s "
                        "/Coefficients/Elasticity %lf "
                        "/Coefficients/Damping %lf "
                        "/Coefficients/Friction %lf "
                        "/Geometry File Name %39[^\n]",
                        R->Name, world_str, IsPosW, &R->ElastCoef, &R->DampCoef,
                        &R->FricCoef, R->GeomFileName) != 7) {
         fprintf(stderr, "Region has improper configuration. Exiting...\n");
         exit(EXIT_FAILURE);
      }

      R->World = GetWorldID(world_str);
      if (R->World < 0 || R->World > NWORLD) {
         fprintf(
             stderr,
             "Region's World is out of range in LoadRegions.  Bailing out.\n");
         exit(EXIT_FAILURE);
      }
      struct WorldType *W = &World[R->World];

      assignYAMLToDoubleArray(
          3, fy_node_by_path_def(seqNode, "/Location/Position"), R->PosW);
      if (DecodeString(IsPosW)) {
         R->Lng      = atan2(R->PosW[1], R->PosW[0]);
         double MagR = MAGV(R->PosW);
         R->Lat      = asin(R->PosW[2] / MagR);
         R->Alt      = MagR - W->rad;
         A2C(312, R->Lng + HalfPi, HalfPi - R->Lat, 0.0, R->CW);
         /* for(i=0;i<3;i++) R->CRW[i][i] = 1.0; */
         MTxV(W->CWN, R->PosW, R->PosN);
         MxM(R->CW, W->CWN, R->CN);
      }
      else {
         R->Lng      = R->PosW[0] * D2R;
         R->Lat      = R->PosW[1] * D2R;
         R->Alt      = R->PosW[2];
         double MagR = W->rad + R->Alt;
         R->PosW[0]  = MagR * cos(R->Lng) * cos(R->Lat);
         R->PosW[1]  = MagR * sin(R->Lng) * cos(R->Lat);
         R->PosW[2]  = MagR * sin(R->Lat);
         A2C(312, R->Lng + HalfPi, HalfPi - R->Lat, 0.0, R->CW);
         MTxV(W->CWN, R->PosW, R->PosN);
         MxM(R->CW, W->CWN, R->CN);
      }
      R->VelN[0] = -W->w * R->PosN[1];
      R->VelN[1] = W->w * R->PosN[0];
      R->VelN[2] = 0.0;
      R->wn[0]   = 0.0;
      R->wn[1]   = W->w * cos(R->Lat);
      R->wn[2]   = W->w * sin(R->Lat);
      Geom = LoadWingsObjFile(ModelPath, R->GeomFileName, &Matl, &Nmatl, Geom,
                              &Ngeom, &R->GeomTag, TRUE);

      Ir++;
   }
   fy_document_destroy(fyd);
}
/**********************************************************************/
void InitLagrangePoints(void)
{
   long i, j;
   char LagsysName[3][20] = {"Earth-Luna", "Sun-Earth", "Sun-Jupiter"};
   struct LagrangeSystemType *LS;
   struct WorldType *W1, *W2;

   LagSys[EARTHMOON].Body1  = EARTH;
   LagSys[EARTHMOON].Body2  = LUNA;
   LagSys[EARTHMOON].LU     = 385692500.0;
   LagSys[SUNEARTH].Body1   = SOL;
   LagSys[SUNEARTH].Body2   = EARTH;
   LagSys[SUNEARTH].LU      = 149597927000.0;
   LagSys[SUNJUPITER].Body1 = SOL;
   LagSys[SUNJUPITER].Body2 = JUPITER;
   LagSys[SUNJUPITER].LU    = 778547200000.0;

   for (i = 0; i < 3; i++) {
      LS = &LagSys[i];
      W1 = &World[LS->Body1];
      W2 = &World[LS->Body2];
      strcpy(LS->Name, LagsysName[i]);
      if (LS->Exists) {
         if (!(W1->Exists && W2->Exists)) {
            fprintf(stderr,
                    "Lagrange System %s depends on worlds that don't exist.  "
                    "Check Inp_Sim.txt\n",
                    LS->Name);
            exit(EXIT_FAILURE);
         }
         LS->mu1      = W1->mu;
         LS->mu2      = W2->mu;
         LS->rho      = LS->mu2 / (LS->mu1 + LS->mu2);
         LS->SLR      = W2->eph.SLR;
         LS->SMA      = W2->eph.SMA;
         LS->ecc      = W2->eph.ecc;
         LS->inc      = W2->eph.inc;
         LS->RAAN     = W2->eph.RAAN;
         LS->ArgP     = W2->eph.ArgP;
         LS->tp       = W2->eph.tp;
         LS->MeanRate = sqrt(LS->mu1 / LS->SMA) / LS->SMA;
         LS->Period   = TwoPi / LS->MeanRate;

         LS->TU = sqrt(pow(LS->LU, 3) / (LS->mu1 + LS->mu2));
         LS->VU = LS->LU / LS->TU;

         FindLagPtParms(LS);
         for (j = 0; j < 5; j++) {
            FindLagPtPosVel(DynTime, LS, j, LS->LP[j].PosN, LS->LP[j].VelN,
                            LS->CLN);
         }
      }
   }
}
/**********************************************************************/
void UpdateLagrangePoints(void)
{
   long i;
   struct LagrangeSystemType *LS;
   struct WorldType *W1, *W2;

   for (i = 0; i < 3; i++) {
      LS = &LagSys[i];
      W1 = &World[LS->Body1];
      W2 = &World[LS->Body2];
      if (LS->Exists) {
         LS->mu1      = W1->mu;
         LS->mu2      = W2->mu;
         LS->rho      = LS->mu2 / (LS->mu1 + LS->mu2);
         LS->SLR      = W2->eph.SLR;
         LS->SMA      = W2->eph.SMA;
         LS->ecc      = W2->eph.ecc;
         LS->inc      = W2->eph.inc;
         LS->RAAN     = W2->eph.RAAN;
         LS->ArgP     = W2->eph.ArgP;
         LS->tp       = W2->eph.tp;
         LS->MeanRate = sqrt(LS->mu1 / LS->SMA) / LS->SMA;
         LS->Period   = TwoPi / LS->MeanRate;

         // FindLagPtParms(LS);
      }
   }
}
/******************************************************************************/
char *replace_char(char *str, const char find, const char replace)
{
   char *current_pos = strchr(str, find);
   while (current_pos) {
      *current_pos = replace;
      current_pos  = strchr(current_pos, find);
   }
   return str;
}
/******************************************************************************/
long is_line_empty(const char *s)
{
   while (*s) {
      if (!isspace(*s))
         return 0;
      s++;
   }
   return 1;
}
/******************************************************************************/
long InitJplHeader(const ephemType ephem, const char eph_path[128],
                   JPLHeaderType *hdr_data)
{
   // read the header file
#define buf_size 512
   // holds flag if NCOEFF and each group of 1030, 1040, 1041, and 1050 are
   // found
   int grp_found[5] = {0};

   hdr_data->eph = ephem;
   strcpy(hdr_data->eph_path, eph_path);

   strcpy(hdr_data->hdr_name, "header.");
   switch (ephem) {
      case EPH_DE421:
      case EPH_GMAT421: {
         strcpy(hdr_data->eph_str, "421");
      } break;
      case EPH_DE424:
      case EPH_GMAT424: {
         strcpy(hdr_data->eph_str, "424");
      } break;
      case EPH_DE430: {
         strcpy(hdr_data->eph_str, "430");
      } break;
      case EPH_DE440: {
         strcpy(hdr_data->eph_str, "440");
      } break;
      default:
         fprintf(stderr, "Unknown ephem type in InitJplHeader(). Exiting...\n");
         exit(EXIT_FAILURE);
   }
   strcat(hdr_data->hdr_name, hdr_data->eph_str);

   FILE *const hdr_file =
       FileOpen(hdr_data->eph_path, hdr_data->hdr_name, "rt");

   long grp_num        = 0;
   char line[buf_size] = {"\0"};
   while (fgets(line, buf_size, hdr_file)) {
      if (sscanf(line, "KSIZE=%ld NCOEFF=%ld", &grp_num, &hdr_data->n_coeff) ==
          2) {
         grp_found[0] = 1;
         break;
      }
   }
   hdr_data->blk_len   = hdr_data->n_coeff + 2;
   hdr_data->blk_lines = hdr_data->blk_len / 3;

   const char *tok_check = " \n\0";

   while (!grp_found[4] && fgets(line, buf_size, hdr_file)) {
      const int sscanf_check = sscanf(line, "GROUP %ld", &grp_num) == 1;
      if (sscanf_check && grp_num == 1030) {
         while (fgets(line, buf_size, hdr_file)) {
            double jd_days[2] = {0};
            if (sscanf(line, "%lf %lf %lf", &jd_days[0], &jd_days[1],
                       &hdr_data->n_days) == 3) {
               grp_found[1] = 1;
               for (int i = 0; i < 2; i++)
                  hdr_data->jd_range[i] =
                      JDFromDays(jd_days[i], TDB_TIME, MJD_EPOCH);
               break;
            }
         }
      }
      else if (sscanf_check && grp_num == 1040) {
         grp_found[2] = 1;
         while (fgets(line, buf_size, hdr_file)) {
            if (sscanf(line, "%ld", &hdr_data->n_data) == 1) {
               break;
            }
         }
         hdr_data->group_1040 = malloc(hdr_data->n_data * sizeof(char[10]));
         char (*const group_1040_start)[10] = hdr_data->group_1040;

         // Assuming data names in group 1040 start immediately after n_data
         while (fgets(line, buf_size, hdr_file)) {
            const char *tok = strtok(line, tok_check);
            if (!tok)
               break;
            while (tok != NULL) {
               strcpy(*hdr_data->group_1040, tok);
               hdr_data->group_1040++;
               tok = strtok(NULL, tok_check);
            }
         }
         hdr_data->group_1040 = group_1040_start;
      }
      else if (sscanf_check && grp_num == 1041) {
         grp_found[3] = 1;
         while (fgets(line, buf_size, hdr_file)) {
            long n_group_1041 = 0;
            if (sscanf(line, "%ld", &n_group_1041) == 1) {
               // assuming group 1041 is AFTER group 1040
               if (n_group_1041 != hdr_data->n_data) {
                  fprintf(stderr,
                          "The data length for groups 1040 and 1041 in DE "
                          "header file '%s' do not match.  Exiting...\n\tGroup "
                          "1040 dimension: %ld\n\tGroup 1040 dimension: %ld",
                          hdr_data->hdr_name, hdr_data->n_data, n_group_1041);
                  exit(EXIT_FAILURE);
               }
               break;
            }
         }
         hdr_data->group_1041 = calloc(hdr_data->n_data, sizeof(double));
         double *const group_1041_start = hdr_data->group_1041;

         // assuming  group 1041 data is immediately after
         while (fgets(line, buf_size, hdr_file)) {
            replace_char(line, 'D', 'E');
            const char *tok = strtok(line, tok_check);
            if (!tok)
               break;
            while (tok != NULL) {
               *hdr_data->group_1041 = atof(tok);
               hdr_data->group_1041++;
               tok = strtok(NULL, tok_check);
            }
         }
         hdr_data->group_1041 = group_1041_start;
      }
      else if (sscanf_check && grp_num == 1050) {
         grp_found[4] = 1;
         while (fgets(line, buf_size, hdr_file)) {
            if (is_line_empty(line))
               continue;

            for (int i = 0; i < 3; i++) {
               const char *tok = strtok(line, tok_check);
               for (int j = 0; j < 11; j++) {
                  hdr_data->group_1050[j][i] = atoi(tok);
                  tok                        = strtok(NULL, tok_check);
               }
               fgets(line, buf_size, hdr_file);
            }
            break;
         }
      }
   }
#undef buf_size
   fclose(hdr_file);
   return (all_int(5, grp_found));
}
/******************************************************************************/
void FilesMatchingFmt(const char path[128], const char fmt[10],
                      char (*f_names)[256], long *const n_match)
{
   // search for files files in `path` matching glob format `fmt`.
   //  returns the list of matching file names in `f_names`, and the number of
   //  them in `n_files`
   // run once with *n_match=0 to get the number of matches, then run again
   //  after allocating f_names
   // NOTE: f_names will be returned with 'path'
   // TODO: REMOVE 'path' FROM 'f_names'
   // BEWARE, THIS IS ONLY FOR POSIX SYSTEMS
   *n_match             = 0;
   char search_fmt[256] = {0};
   strcpy(search_fmt, path);
   strcat(search_fmt, "/");
   strcat(search_fmt, fmt);

   glob_t results;
   if (glob(search_fmt, 0, NULL, &results) == 0) {
      *n_match = results.gl_pathc;
      if (f_names != NULL)
         for (long i = 0; i < *n_match; i++) {
            strcpy(f_names[i], results.gl_pathv[i]);
         }
   }
   else
      f_names = NULL;
   globfree(&results);
}
/******************************************************************************/
long LoadJplEphems(char EphemPath[128], JPLHeaderType *const jpl_hdr,
                   const JDType jd, struct WorldType *const worlds)
{
   static FILE *infile = NULL;
   long BlockNum, NumEntries;
   long FoundBlock;
   char line[512];
   JDType jd_block[2];
   long i, n, Ic, Iw;
   long Nseg, Start, N;
   struct Cheb3DType *Cheb;

   JDType jd_tdb_mjd = jd, jd_tdb_z = jd;
   ChangeSystemEpoch(TDB_TIME, MJD_EPOCH, &jd_tdb_mjd);
   ChangeSystemEpoch(TDB_TIME, ZERO_EPOCH, &jd_tdb_z);

   if (jpl_hdr->n_data == 0)
      InitJplHeader(EphemOption, EphemPath, jpl_hdr);

   // search for the list of file to use with this EphemOption
   // only need to do this once and keep it around
   static char (*f_names)[256]  = NULL;
   static JDType(*jd_ranges)[2] = NULL;
   static long n_match          = 0;
   if (f_names == NULL) {
      char search_fmt[20] = "ascp*.";
      strcat(search_fmt, jpl_hdr->eph_str);
      FilesMatchingFmt(EphemPath, search_fmt, f_names, &n_match);
      if (!n_match) {
         fprintf(stderr,
                 "Could not find any files in directory '%s' for DE type '%s' "
                 "matching glob format '%s'. Exiting...\n",
                 jpl_hdr->eph_path, jpl_hdr->eph_str, search_fmt);
         exit(EXIT_FAILURE);
      }
      f_names = calloc(n_match, sizeof(char[256]));
      FilesMatchingFmt(EphemPath, search_fmt, f_names, &n_match);
      jd_ranges             = malloc(n_match * sizeof(JDType[2]));
      double jd_rng_days[2] = {0.0};

      // preload the jd ranges for each file for the chosen DE
      for (i = 0; i < n_match; i++) {
         int first_block = 0;
         double dummy[2] = {0.0};

         infile = FileOpen("", f_names[i], "rt");
         while (fgets(line, 512, infile)) {
            if (sscanf(line, "%ld %ld", &BlockNum, &NumEntries) == 2) {
               fgets(line, 512, infile);
               if (sscanf(line, "%lf %lf %lf", &dummy[0], &jd_rng_days[1],
                          &dummy[1]) == 3)
                  if (!first_block) {
                     first_block    = 1;
                     jd_rng_days[0] = dummy[0];
                  }
            }
         }
         // convert to desired Epoch
         for (int j = 0; j < 2; j++) {
            jd_ranges[i][j] = JDFromDays(jd_rng_days[j], TDB_TIME, ZERO_EPOCH);
            ChangeEpoch(GMAT_MJD_EPOCH, &jd_ranges[i][j]);
         }
         fclose(infile);
      }
   }

   // Make sure the chosen JD is covered by desired DE
   if (isless_jd(jd_tdb_mjd, jpl_hdr->jd_range[0]) ||
       isgreater_jd(jd_tdb_mjd, jpl_hdr->jd_range[1])) {
      fprintf(stderr,
              "JD is not contained in DE%s ephem files.  Falling back to "
              "lower-precision planetary ephemerides.\n",
              jpl_hdr->eph_str);
      return (1); // TODO: what do we actually do in this case?
   }

   // Figure out which jd range desired JD is in
   int cur_file = -1;
   for (i = 0; i < n_match; i++) {
      if (isgreaterequal_jd(jd_tdb_mjd, jd_ranges[i][0]) &&
          isless_jd(jd_tdb_mjd, jd_ranges[i][1])) {
         cur_file = i;
         break;
      }
   }
   if (cur_file == -1) {
      fprintf(stderr,
              "Could not find any files in directory '%s' for DE type '%s' "
              "that Julian Date %lf is contained within. Exiting...\n",
              jpl_hdr->eph_path, jpl_hdr->eph_str, JDToDays(jd_tdb_z));
      exit(EXIT_FAILURE);
   }

   // Search found file for block containing chosen JD
   const long blk_len = jpl_hdr->n_coeff + 2;
   double Block[blk_len];

   FoundBlock = 0;
   infile     = FileOpen("", f_names[cur_file], "rt");
   while (!FoundBlock) {
      fgets(line, 512, infile);
      if (sscanf(line, "%ld %ld", &BlockNum, &NumEntries) == 2) {
         fgets(line, 512, infile);
         if (sscanf(line, "%lf %lf %lf", &Block[0], &Block[1], &Block[2]) ==
             3) {
            jd_block[0] = JDFromDays(Block[0], TDB_TIME, ZERO_EPOCH);
            jd_block[1] = JDFromDays(Block[1], TDB_TIME, ZERO_EPOCH);
            if (isgreaterequal_jd(jd_tdb_z, jd_block[0]) &&
                isless_jd(jd_tdb_z, jd_block[1])) {
               FoundBlock = 1;

               for (i = 0; i < 2; i++)
                  ChangeEpoch(GMAT_MJD_EPOCH, &jd_block[i]);
            }
         }
      }
   }

   /* .. Load block */
   for (i = 1; i < jpl_hdr->blk_lines; i++) {
      fgets(line, 512, infile);
      sscanf(line, "%lf %lf %lf", &Block[3 * i], &Block[3 * i + 1],
             &Block[3 * i + 2]);
   }
   fclose(infile);

   /* .. Distribute to Worlds [Starting Entry (1-based), Order, Number of
    * Segments] */
   // Note that the data for 'EARTH' is Earth-Moon barycenter and 'MOON' is the
   // geocentric position of the Moon
   // the order of bodies is the order of columns in block 1050
   static int bodies[11] = {MERCURY, VENUS,   EARTH, MARS, JUPITER, SATURN,
                            URANUS,  NEPTUNE, PLUTO, LUNA, SOL};
   for (int j = 0; j < 11; j++) {
      Iw    = bodies[j];
      Nseg  = jpl_hdr->group_1050[j][2];
      Start = jpl_hdr->group_1050[j][0] - 1;
      N     = jpl_hdr->group_1050[j][1];

      worlds[Iw].eph.Ncheb = Nseg;
      worlds[Iw].eph.Cheb =
          (struct Cheb3DType *)calloc(Nseg, sizeof(struct Cheb3DType));
      for (Ic = 0; Ic < Nseg; Ic++) {
         Cheb                    = &worlds[Iw].eph.Cheb[Ic];
         double jd_blk_diff_days = JDSubToDays(jd_block[0], jd_block[1]);
         Cheb->JD1 = JDAddDays(jd_block[0], ((double)Ic) * (jd_blk_diff_days) /
                                                ((double)Nseg));
         Cheb->JD2 =
             JDSubDays(jd_block[1], ((double)(Nseg - 1 - Ic)) *
                                        (jd_blk_diff_days) / ((double)Nseg));
         Cheb->N = N;
         for (n = 0; n < N; n++) {
            for (i = 0; i < 3; i++) {
               Cheb->Coef[i][n] = Block[Start + N * 3 * Ic + N * i + n];
            }
         }
      }
   }

   /* Specific Earth-Moon Mass Ratio and AU  Definitions */
   EMRAT = getDEHeader1041Data(jpl_hdr, "EMRAT"); // Earth/Moon Mass Ratio
   AU    = getDEHeader1041Data(jpl_hdr, "AU");    // Kilometers per 1 AU

   // Conversion of GM from AU^3/day^2 to m^3/s^2 using DE appropriate values
   AUd2ms = (ipow(AU, 3) / ipow(SEC_PER_DAY, 2)) * 1.0e9;

   return (0);
}
/**********************************************************************/
long UpdateJplEphems(const JDType jd, const JPLHeaderType *const jpl_hdr,
                     struct WorldType *const worlds)
{
   long i, Ic, Iw;
   struct Cheb3DType *Cheb;
   struct OrbitType *Eph;
   struct WorldType *W;
   double u, dudJD, T[20], U[20], P, dPdu;
   double rh[3], vh[3];
   double EarthMoonBaryPosH[3], EarthMoonBaryVelH[3];
   double ZAxis[3] = {0.0, 0.0, 1.0};
   double PosJ[3], VelJ[3], CNJ[3][3];
   double C_W_TETE[3][3] = {{0.0}}, C_TEME_TETE[3][3] = {{0.0}},
          C_TETE_J2000[3][3] = {{0.0}};

   double GMST = JD2GMST(jd);

   JDType jd_tt_j2000 = jd, jd_tdb_mjd = jd;
   ChangeEpoch(J2000_EPOCH, &jd_tt_j2000);
   ChangeSystem(TT_TIME, &jd_tt_j2000);
   JDType jd_tdb_j2000 = jd_tt_j2000;
   ChangeSystem(TDB_TIME, &jd_tdb_j2000);
   const double j2000_sec = JDToDynTime(jd_tt_j2000);

   ChangeEpoch(GMAT_MJD_EPOCH, &jd_tdb_mjd);
   ChangeSystem(TDB_TIME, &jd_tdb_mjd);

   /* .. Initialize Planetary Pos/Vel */
   for (Iw = SOL; Iw <= LUNA; Iw++) {
      W   = &worlds[Iw];
      Eph = &W->eph;
      /* Determine segment */
      Ic = 0;
      while (isgreater_jd(jd_tdb_mjd, Eph->Cheb[Ic].JD2))
         Ic++;
      /* Apply Chebyshev polynomials */
      Cheb  = &Eph->Cheb[Ic];
      dudJD = 2.0 / JDSubToDays(Cheb->JD2, Cheb->JD1);
      u     = JDSubToDays(jd_tdb_mjd, Cheb->JD1) * dudJD - 1.0;
      ChebyPolys(u, Cheb->N, T, U);
      for (i = 0; i < 3; i++) {
         ChebyInterp(T, U, Cheb->Coef[i], Cheb->N, &P, &dPdu);
         PosJ[i] = 1000.0 * P;
         VelJ[i] = 1000.0 * dPdu * dudJD / SEC_PER_DAY;
      }
      QTxV(qjh, PosJ, Eph->PosN);
      QTxV(qjh, VelJ, Eph->VelN);
   }
   /* Adjust for barycenters */
   /* Move planets from barycentric to Sun-centered */
   for (Iw = MERCURY; Iw <= PLUTO; Iw++) {
      W = &worlds[Iw];
      for (i = 0; i < 3; i++) {
         W->eph.PosN[i] -= worlds[SOL].eph.PosN[i];
         W->eph.VelN[i] -= worlds[SOL].eph.VelN[i];
         W->PosH[i]      = W->eph.PosN[i];
         W->VelH[i]      = W->eph.VelN[i];
      }
      /* Calculate PriMerAng for Planets */
      W->PriMerAng = fmod(W->w * j2000_sec, TwoPi);
      SimpRot(ZAxis, W->PriMerAng, W->CWN);
   }
   /* Calculate PriMerAng for Sun */
   worlds[SOL].PriMerAng =
       fmod(worlds[SOL].PriMerAngJ2000 + worlds[SOL].w * j2000_sec, TwoPi);
   SimpRot(ZAxis, worlds[SOL].PriMerAng, worlds[SOL].CWN);
   C2Q(worlds[SOL].CWN, worlds[SOL].qwn);
   /* Move Sun to origin */
   for (i = 0; i < 3; i++) {
      worlds[SOL].PosH[i]     = 0.0;
      worlds[SOL].VelH[i]     = 0.0;
      worlds[SOL].eph.PosN[i] = 0.0;
      worlds[SOL].eph.VelN[i] = 0.0;
   }
   /* Adjust Earth from Earth-Moon barycenter */
   /* (Moon PosVel is geocentric, not from barycenter) */
   for (i = 0; i < 3; i++) {
      EarthMoonBaryPosH[i]       = worlds[LUNA].eph.PosN[i] / (1.0 + EMRAT);
      EarthMoonBaryVelH[i]       = worlds[LUNA].eph.VelN[i] / (1.0 + EMRAT);
      worlds[EARTH].eph.PosN[i] -= EarthMoonBaryPosH[i];
      worlds[EARTH].eph.VelN[i] -= EarthMoonBaryVelH[i];
      worlds[EARTH].PosH[i]      = worlds[EARTH].eph.PosN[i];
      worlds[EARTH].VelH[i]      = worlds[EARTH].eph.VelN[i];
   }
   for (i = 0; i < 3; i++) {
      rh[i]                = worlds[LUNA].eph.PosN[i];
      vh[i]                = worlds[LUNA].eph.VelN[i];
      worlds[LUNA].PosH[i] = worlds[EARTH].PosH[i] + worlds[LUNA].eph.PosN[i];
      worlds[LUNA].VelH[i] = worlds[EARTH].VelH[i] + worlds[LUNA].eph.VelN[i];
   }
   /* Rotate Moon into ECI */
   QxV(qjh, rh, worlds[LUNA].eph.PosN);
   QxV(qjh, vh, worlds[LUNA].eph.VelN);
   worlds[LUNA].PriMerAng = LunaPriMerAng(jd_tdb_j2000);
   SimpRot(ZAxis, worlds[LUNA].PriMerAng, worlds[LUNA].CWN);
   LunaInertialFrame(jd_tdb_j2000, CNJ);
   MxM(CNJ, worlds[EARTH].CNH, worlds[LUNA].CNH);
   C2Q(worlds[LUNA].CNH, worlds[LUNA].qnh);
   QxQT(worlds[LUNA].qnh, qjh, worlds[LUNA].qnj);

   /* .. Earth rotation is a special case */
   worlds[EARTH].PriMerAng = TwoPi * GMST;
   HiFiEarthPrecNute(jd, C_TEME_TETE, C_TETE_J2000);
   SimpRot(ZAxis, worlds[EARTH].PriMerAng, C_W_TETE);
   MxM(C_W_TETE, C_TETE_J2000, worlds[EARTH].CWN);

   for (Iw = MERCURY; Iw <= LUNA; Iw++) {
      Eph = &worlds[Iw].eph;
      RV2Eph(j2000_sec, Eph->mu, Eph->PosN, Eph->VelN, &Eph->SMA, &Eph->ecc,
             &Eph->inc, &Eph->RAAN, &Eph->ArgP, &Eph->anom, &Eph->tp, &Eph->SLR,
             &Eph->alpha, &Eph->rmin, &Eph->MeanMotion, &Eph->Period);
   }
   return (0);
}
/**********************************************************************/
void Rk4JplEphems(JDType jd, long trgtWORLD, struct WorldType *worlds,
                  double trgtPosN[3], double trgtPosH[3], double *trgtPriMerAng,
                  double trgtCNH[3][3])
{
   long i, j, Ic, Iw;
   struct Cheb3DType *Cheb;
   struct OrbitType *Eph;
   struct WorldType *W;
   double u, dudJD, T[20], U[20], P, dPdu;
   double rh[3], PosJ[3], PosN[3], systemBC[3];
   double earthPosN[3], lunaPosN[3], otherPosN[3];
   double earthPosH[3], lunaPosH[3], otherPosH[3];
   double CNJ[3][3] = {0};
   long WRLD[2]     = {EARTH, LUNA}, otherJPL;
   double GMST, timeTT;
   double CNH[3][3] = {0};

   // TODO: premake some of the other jd types that are needed
   ChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, &jd);

   /* .. Initialize position of system barycenter */
   W   = &worlds[SOL];
   Eph = &W->eph;
   /* Determine segment */
   Ic = 0;
   while (isgreater_jd(jd, Eph->Cheb[Ic].JD2))
      Ic++;
   /* Apply Chebyshev polynomials */
   Cheb  = &Eph->Cheb[Ic];
   dudJD = 2.0 / JDSubToDays(Cheb->JD2, Cheb->JD1);
   u     = JDSubToDays(jd, Cheb->JD1) * dudJD - 1.0;
   ChebyPolys(u, Cheb->N, T, U);
   for (i = 0; i < 3; i++) {
      ChebyInterp(T, U, Cheb->Coef[i], Cheb->N, &P, &dPdu);
      PosJ[i] = 1000.0 * P;
   }
   QTxV(qjh, PosJ, systemBC);

   /* Determine which ephemerides math needed */
   otherJPL = (trgtWORLD != SOL && trgtWORLD != EARTH && trgtWORLD != LUNA &&
               trgtWORLD <= PLUTO);

   /* Must compute both Earth/Luna if either selected (how JPL defines ephem
    * measurements) */
   if (trgtWORLD == EARTH || trgtWORLD == LUNA) {
      /* .. Initialize Planetary Pos for EARTH/LUNA */
      for (j = 0; j < 2; ++j) {
         Iw  = WRLD[j];
         W   = &worlds[Iw];
         Eph = &W->eph;
         /* Determine segment */
         Ic = 0;
         while (isgreater_jd(jd, Eph->Cheb[Ic].JD2))
            Ic++;
         /* Apply Chebyshev polynomials */
         Cheb  = &Eph->Cheb[Ic];
         dudJD = 2.0 / JDSubToDays(Cheb->JD2, Cheb->JD1);
         u     = JDSubToDays(jd, Cheb->JD1) * dudJD - 1.0;
         ChebyPolys(u, Cheb->N, T, U);
         for (i = 0; i < 3; i++) {
            ChebyInterp(T, U, Cheb->Coef[i], Cheb->N, &P, &dPdu);
            PosJ[i] = 1000.0 * P;
         }
         QTxV(qjh, PosJ, PosN);
         if (Iw == EARTH) {
            for (i = 0; i < 3; i++)
               earthPosN[i] = PosN[i];
         }
         else if (Iw == LUNA) {
            for (i = 0; i < 3; i++)
               lunaPosN[i] = PosN[i];
         }
      }
      /* Move Earth from barycentric to Sun-centered */
      for (i = 0; i < 3; i++) {
         earthPosN[i] -= systemBC[i];
      }
      /* Adjust Earth from Earth-Moon barycenter */
      /* (Moon PosVel is geocentric, not from barycenter) */
      for (i = 0; i < 3; i++) {
         earthPosN[i] -= lunaPosN[i] / (1.0 + EMRAT);
         earthPosH[i]  = earthPosN[i];
      }
      for (i = 0; i < 3; i++) {
         rh[i]       = lunaPosN[i];
         lunaPosH[i] = earthPosH[i] + lunaPosN[i];
      }
      /* Rotate Moon into ECI */
      QxV(qjh, rh, lunaPosN);
   }
   else if (otherJPL) {
      /* .. Initialize Pos for other planet in JPL ephemerides */
      W   = &worlds[trgtWORLD];
      Eph = &W->eph;
      /* Determine segment */
      Ic = 0;
      while (isgreater_jd(jd, Eph->Cheb[Ic].JD2))
         Ic++;
      /* Apply Chebyshev polynomials */
      Cheb  = &Eph->Cheb[Ic];
      dudJD = 2.0 / JDSubToDays(Cheb->JD2, Cheb->JD1);
      u     = JDSubToDays(jd, Cheb->JD1) * dudJD - 1.0;
      ChebyPolys(u, Cheb->N, T, U);
      for (i = 0; i < 3; i++) {
         ChebyInterp(T, U, Cheb->Coef[i], Cheb->N, &P, &dPdu);
         PosJ[i] = 1000.0 * P;
      }
      QTxV(qjh, PosJ, PosN);
      /* Move planet from barycentric to Sun-centered */
      for (i = 0; i < 3; i++) {
         otherPosN[i]  = PosN[i];
         otherPosN[i] -= systemBC[i];
         otherPosH[i]  = otherPosN[i];
      }
   }

   /* Now perform calculation on Target World */
   if (trgtWORLD == SOL) {
      for (i = 0; i < 3; i++) {
         trgtPosN[i] = 0.0;
         trgtPosH[i] = 0.0;
      }
      /* Calculate PriMerAng for Sun */
      timeTT = JDToDynTime(jd);
      *trgtPriMerAng =
          fmod(worlds[SOL].PriMerAngJ2000 + worlds[SOL].w * timeTT, TwoPi);
   }
   else if (trgtWORLD == EARTH) {
      for (i = 0; i < 3; i++) {
         trgtPosN[i] = earthPosN[i];
         trgtPosH[i] = earthPosH[i];
      }
      /* Calculate PriMerAng for Earth */
      GMST           = JD2GMST(jd);
      *trgtPriMerAng = TwoPi * GMST;
   }
   else if (trgtWORLD == LUNA) {
      for (i = 0; i < 3; i++) {
         trgtPosN[i] = lunaPosN[i];
         trgtPosH[i] = lunaPosH[i];
      }
      /* Calculate PriMerAng for LUNA */
      *trgtPriMerAng = LunaPriMerAng(jd);
   }
   else if (otherJPL) {
      /* Move target from barycentric to Sun-centered */
      for (i = 0; i < 3; i++) {
         trgtPosN[i] = otherPosN[i];
         trgtPosH[i] = otherPosH[i];
      }
      /* Calculate PriMerAng for Sun */
      timeTT = JDToDynTime(jd);
      *trgtPriMerAng =
          fmod(worlds[trgtWORLD].PriMerAngJ2000 + worlds[trgtWORLD].w * timeTT,
               TwoPi);
   }
   else {
      /* Use original position for non JPL epemerides bodies */
      W = &worlds[trgtWORLD];
      for (i = 0; i < 3; i++) {
         trgtPosN[i] = W->eph.PosN[i];
         trgtPosH[i] = W->PosH[i];
      }
      *trgtPriMerAng = W->PriMerAng;
   }

   if (trgtWORLD == LUNA) {
      LunaInertialFrame(jd, CNJ);
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++) {
            CNH[i][j] = worlds[EARTH].CNH[i][j];
         }
      }
      MxM(CNJ, CNH, trgtCNH);
   }
   else {
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++) {
            trgtCNH[i][j] = worlds[trgtWORLD].CNH[i][j];
         }
      }
   }
}
/**********************************************************************/
long LoadEphems(const ephemType ephem, const JDType jd,
                JPLHeaderType *const jpl_hdr, struct WorldType *const worlds)
{
   /* Preload Ephemeris Kernels/Definitions */
   switch (EphemOption) {
      case EPH_MEAN: // No Ephem to Load
         break;
      case EPH_DE430:
      case EPH_DE440:
      case EPH_DE421:
      case EPH_DE424:
      case EPH_GMAT421:
      case EPH_GMAT424:
         return LoadJplEphems(ModelPath, jpl_hdr, jd, worlds);
         break;
      case EPH_SPICE:
#ifdef _ENABLE_SPICE_
         // Load SPICE to get SPICE-provided values for mu, J2, etc
         return LoadSpiceKernels(ModelPath);
#endif
         break;
      default:
         fprintf(stderr, "Unknown Ephem Type. Exiting...\n");
         exit(EXIT_FAILURE);
   }
   return (0);
}
/**********************************************************************/
// this is done the same way every time, might as well do it this way
long UpdateEphems(const ephemType ephem, const JDType jd,
                  const JPLHeaderType *const jpl_hdr,
                  struct WorldType *const worlds)
{
   JDType jd_tdb_mjd = jd;
   ChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, &jd_tdb_mjd);

   long main_ephem_check = 0;
   switch (ephem) {
      case EPH_MEAN: {
         /* If EPH_MEAN, update planets/luna to get higher fidelity model for
          * LUNA */
         main_ephem_check = UpdateMeanEphems(jd, worlds);
      } break;
      case EPH_DE430:
      case EPH_DE440:
      case EPH_DE421:
      case EPH_DE424:
      case EPH_GMAT421:
      case EPH_GMAT424: {
         if (isgreater_jd(jd_tdb_mjd, worlds[SOL].eph.Cheb[1].JD2))
            LoadJplEphems(ModelPath, &JplHeader, jd_tdb_mjd, worlds);
         /* Load Planetary/Luna ephems */
         main_ephem_check = UpdateJplEphems(jd_tdb_mjd, jpl_hdr, worlds);
      } break;
      case EPH_SPICE: {
#ifdef _ENABLE_SPICE_
         main_ephem_check = UpdateSpiceEphems(jd_tdb_mjd, worlds);
#endif
      } break;
      default:
         fprintf(stderr, "Uknown Ephem Type. Exiting...\n");
         exit(EXIT_FAILURE);
         // return 1;  ?
   }

   /* .. Minor Bodies */

   main_ephem_check |= UpdateMinorBodies(jd_tdb_mjd, &worlds[MINORBODY_0]);
   /* .. Other planets' moons */
   if (EphemOption != EPH_SPICE)
      main_ephem_check |= UpdateNonEphemMoons(jd_tdb_mjd, worlds);
   return main_ephem_check;
}
/**********************************************************************/
long UpdateMeanEphems(const JDType jd, struct WorldType *const worlds)
{
   struct OrbitType *Eph;
   struct WorldType *W;

   JDType jd_z = jd;
   ChangeSystemEpoch(UTC_TIME, ZERO_EPOCH, &jd_z);
   const double GMST     = JD2GMST(jd_z);
   const double j2000sec = JDToDynTime(jd);
   double r1[3], rh[3], vh[3];
   const double ZAxis[3] = {0.0, 0.0, 1.0};
   long j, Ip;
   double C_W_TETE[3][3], C_TEME_TETE[3][3], C_TETE_J2000[3][3];

   for (Ip = MERCURY; Ip <= PLUTO; Ip++) {
      if (worlds[Ip].Exists) {
         W = &worlds[Ip];
         /* Call PlanetEphemerides again only for
            ridiculously high accuracy or rather long sims (years) */
         /*PlanetEphemerides(i,JulDay,... */
         Eph = &W->eph;
         Eph2RV(Eph->mu, Eph->SLR, Eph->ecc, Eph->inc, Eph->RAAN, Eph->ArgP,
                j2000sec - Eph->tp, Eph->PosN, Eph->VelN, &Eph->anom);
         for (j = 0; j < 3; j++) {
            W->PosH[j] = Eph->PosN[j];
            W->VelH[j] = Eph->VelN[j];
         }
         W->PriMerAng = fmod(W->PriMerAngJ2000 + W->w * j2000sec, TwoPi);
         SimpRot(ZAxis, W->PriMerAng, W->CWN);
      }
   }
   if (worlds[LUNA].Exists) {
      Eph = &worlds[LUNA].eph;
      /* Meeus computes Luna Position in geocentric ecliptic */
      JDType jd_tdb_z = jd;
      ChangeSystemEpoch(TDB_TIME, J2000_EPOCH, &jd_tdb_z);

      worlds[LUNA].PriMerAng = LunaPriMerAng(jd_tdb_z);
      LunaPosition(jd_tdb_z, rh);
      jd_tdb_z = JDAddDays(jd_tdb_z, 0.01);
      LunaPosition(jd_tdb_z, r1);
      for (j = 0; j < 3; j++)
         vh[j] = (r1[j] - rh[j]) / (864.0);
      /* Convert to Earth's N frame */
      MxV(worlds[EARTH].CNH, rh, Eph->PosN);
      MxV(worlds[EARTH].CNH, vh, Eph->VelN);
      /* Find Luna's osculating elements */
      RV2Eph(j2000sec, Eph->mu, Eph->PosN, Eph->VelN, &Eph->SMA, &Eph->ecc,
             &Eph->inc, &Eph->RAAN, &Eph->ArgP, &Eph->anom, &Eph->tp, &Eph->SLR,
             &Eph->alpha, &Eph->rmin, &Eph->MeanMotion, &Eph->Period);
      SimpRot(ZAxis, worlds[LUNA].PriMerAng, worlds[LUNA].CWN);
      for (j = 0; j < 3; j++) {
         worlds[LUNA].PosH[j] = rh[j] + worlds[EARTH].PosH[j];
         worlds[LUNA].VelH[j] = vh[j] + worlds[EARTH].VelH[j];
      }
   }
   /* .. Earth rotation is a special case */
   worlds[EARTH].PriMerAng = TwoPi * GMST;
   HiFiEarthPrecNute(jd, C_TEME_TETE, C_TETE_J2000);
   SimpRot(ZAxis, worlds[EARTH].PriMerAng, C_W_TETE);
   MxM(C_W_TETE, C_TETE_J2000, worlds[EARTH].CWN);
   return (0);
}
/**********************************************************************/
long UpdateMinorBodies(const JDType jd, struct WorldType *const minor_worlds)
{
   struct OrbitType *Eph;
   struct WorldType *W;
   double ZAxis[3] = {0.0, 0.0, 1.0};
   long j, Imb;

   const double j2000_sec = JDToDynTime(jd);

   /* .. Locate Asteroids and Comets */
   for (Imb = 0; Imb < Nmb; Imb++) {
      if (minor_worlds[Imb].Exists) {
         W   = &minor_worlds[Imb];
         Eph = &W->eph;
         Eph2RV(Eph->mu, Eph->SLR, Eph->ecc, Eph->inc, Eph->RAAN, Eph->ArgP,
                j2000_sec - Eph->tp, Eph->PosN, Eph->VelN, &Eph->anom);
         for (j = 0; j < 3; j++) {
            W->PosH[j] = Eph->PosN[j];
            W->VelH[j] = Eph->VelN[j];
         }
         W->PriMerAng = fmod(W->w * j2000_sec, TwoPi);
         SimpRot(ZAxis, W->PriMerAng, W->CWN);
      }
   }
   return (0);
}
/**********************************************************************/
long UpdateNonEphemMoons(const JDType jd, struct WorldType *const worlds)
{
   struct OrbitType *Eph;
   double rh[3], vh[3];
   double ZAxis[3] = {0.0, 0.0, 1.0};
   long i, Ip, Iw;

   const double j2000_sec = JDToDynTime(jd);

   /* .. Other planets' moons */
   for (Ip = MERCURY; Ip <= PLUTO; Ip++) {
      if (Ip != EARTH && worlds[Ip].Exists) {
         for (long Im = 0; Im < worlds[Ip].Nsat; Im++) {
            Iw  = worlds[Ip].Sat[Im];
            Eph = &worlds[Iw].eph;
            Eph2RV(Eph->mu, Eph->SLR, Eph->ecc, Eph->inc, Eph->RAAN, Eph->ArgP,
                   j2000_sec - Eph->tp, Eph->PosN, Eph->VelN, &Eph->anom);
            worlds[Iw].PriMerAng = fmod(worlds[Iw].w * j2000_sec, TwoPi);
            SimpRot(ZAxis, worlds[Iw].PriMerAng, worlds[Iw].CWN);
            C2Q(worlds[Iw].CWN, worlds[Iw].qwn);
            MTxV(worlds[Ip].CNH, Eph->PosN, rh);
            MTxV(worlds[Ip].CNH, Eph->VelN, vh);
            for (i = 0; i < 3; i++) {
               worlds[Iw].PosH[i] = rh[i] + worlds[Ip].PosH[i];
               worlds[Iw].VelH[i] = vh[i] + worlds[Ip].VelH[i];
            }
         }
      }
   }
   return (0);
}
/**********************************************************************/
#ifdef _ENABLE_SPICE_
long LoadSpiceKernels(char SpicePath[80])
{
   char MetaKernelPath[80];
   strcpy(MetaKernelPath, SpicePath);
   strcat(MetaKernelPath, "spice_kernels/kernels.txt");

   furnsh_c(MetaKernelPath);
   return (0);
}
long UpdateSpiceEphems(const JDType jd, struct WorldType *const worlds)
{
   WorldID Iw, Ip, Im;
   int i;
   double CNJ[3][3];

   JDType jd_tdb_j2000 = jd;
   ChangeSystemEpoch(TDB_TIME, J2000_EPOCH, &jd_tdb_j2000);
   const double JS       = JDToDays(jd_tdb_j2000) * SEC_PER_DAY;
   const double j2000sec = JDToDynTime(jd_tdb_j2000);

   struct OrbitType *Eph;
   struct WorldType *W;
   double Nstate[6], Hstate[6];
   double light_time;
   double ang[3] = {0.0};

   double CWJ[3][3];

   // Read all planets
   for (Iw = MERCURY; Iw <= PLUTO; Iw++) {
      if (worlds[Iw].Exists) {
         W   = &worlds[Iw];
         Eph = &W->eph;
         spkez_c(WorldID2NAIFID(Iw), JS, "ECLIPJ2000", "NONE",
                 WorldID2NAIFID(0), Nstate,
                 &light_time); // State of major bodies in J2000 wrt Sun center

         for (i = 0; i < 3; i++) {
            Eph->PosN[i] = Nstate[i] * 1e3; // Assign inertial positions (m)
            W->PosH[i] =
                Nstate[i] *
                1e3; // Assign suncentric positions = inertial position (m)

            Eph->VelN[i] =
                Nstate[i + 3] * 1e3; // Assign inertial velocity (m/s)
            W->VelH[i] =
                Nstate[i + 3] *
                1e3; // Assign suncentric velocity = inertial velocity (m/s)
         }
         char frame_name[25] = {'\0'};
         WorldID2IAUFrame(Iw, frame_name);

         // CNJ @ EarthCNH = CNH -> CNJ = CNH @ EarthCNH^T
         MxMT(worlds[Iw].CNH, worlds[EARTH].CNH, CNJ);

         pxform_c("J2000", frame_name, JS, CWJ);

         // CWN @ CNJ = CWJ -> CWN = CWJ @ CNJ^T
         MxMT(CWJ, CNJ, worlds[Iw].CWN);
         C2Q(worlds[Iw].CWN, worlds[Iw].qwn);
         logso3(worlds[Iw].CWN, ang);
         worlds[Iw].PriMerAng = ang[2];
      }
   }

   // Read all moons
   for (Ip = EARTH; Ip <= PLUTO; Ip++) {
      if (worlds[Ip].Exists) {
         for (Im = 0; Im < worlds[Ip].Nsat; Im++) {
            Iw  = worlds[Ip].Sat[Im];
            W   = &worlds[Iw];
            Eph = &W->eph;

            spkez_c(
                WorldID2NAIFID(Iw), JS, "ECLIPJ2000", "NONE",
                WorldID2NAIFID(SOL), Hstate,
                &light_time); // State of major bodies in J2000 wrt Sun center

            spkez_c(WorldID2NAIFID(Iw), JS, "ECLIPJ2000", "NONE",
                    WorldID2NAIFID(Ip), Nstate,
                    &light_time); // State of major bodies in J2000 wrt Planet
                                  // center

            for (i = 0; i < 3; i++) {
               Eph->PosN[i] = Nstate[i] * 1e3; // Assign inertial positions (m)
               W->PosH[i]   = Hstate[i] * 1e3;

               Eph->VelN[i] =
                   Nstate[i + 3] * 1e3; // Assign inertial velocity (m/s)
               W->VelH[i] = Hstate[i + 3] * 1e3;
            }

            if (Iw == LUNA) {
               spkez_c(WorldID2NAIFID(Iw), JS, "J2000", "NONE",
                       WorldID2NAIFID(Ip), Nstate,
                       &light_time); // State of major bodies in J2000 wrt
                                     // Planet center
               for (i = 0; i < 3; i++) {
                  Eph->PosN[i] =
                      Nstate[i] * 1e3; // Assign inertial positions (m)
                  Eph->VelN[i] =
                      Nstate[i + 3] * 1e3; // Assign inertial velocity (m/s)
               }
            }

            char frame_name[25] = {'\0'};
            WorldID2IAUFrame(Iw, frame_name);

            pxform_c("J2000", frame_name, JS,
                     CWJ); // matrix from J2000 (ICRF) -> body fixed

            // CNJ @ EarthCNH = CNH -> CNJ = CNH @ EarthCNH^T
            MxMT(worlds[Iw].CNH, worlds[EARTH].CNH, CNJ);

            pxform_c("J2000", frame_name, JS,
                     CWJ); // matrix from J2000 (ICRF) -> body fixed

            // CWN @ CNJ = CWJ -> CWN = CWJ @ CNJ^T
            MxMT(CWJ, CNJ, worlds[Iw].CWN);
            C2Q(worlds[Iw].CWN, worlds[Iw].qwn);
            logso3(worlds[Iw].CWN, ang);
            worlds[Iw].PriMerAng = ang[2];
         }
      }
   }

   for (Iw = MERCURY; Iw <= LUNA; Iw++) {
      Eph = &worlds[Iw].eph;
      RV2Eph(j2000sec, Eph->mu, Eph->PosN, Eph->VelN, &Eph->SMA, &Eph->ecc,
             &Eph->inc, &Eph->RAAN, &Eph->ArgP, &Eph->anom, &Eph->tp, &Eph->SLR,
             &Eph->alpha, &Eph->rmin, &Eph->MeanMotion, &Eph->Period);
   }

   return (0);
}
void Rk4SpiceEphems(JDType jd, WorldID trgtWORLD,
                    struct WorldType *const worlds, double trgtPosN[3],
                    double trgtPosH[3], double *trgtPriMerAng,
                    double trgtCNH[3][3])
{
   // TODO: this idea will take some doing, but should maybe make some wrappers
   // for the spice stuff, that way we don't need to do things like append
   // "IAU_" every time and just need to call the wrapper with our WorldID.
   // would also be nice to have #include "SpiceUsr.h" in one place
   double CNH[3][3];
   double Nstate[6], Hstate[6];
   double light_time;
   char trgtCNH_STRING[25] = {'\0'};
   int i, j;

   ChangeSystemEpoch(TDB_TIME, J2000_EPOCH, &jd);
   const double jd_tdb_j2000_sec = JDToTime(jd);

   SpiceInt tgt_world_naif = WorldID2NAIFID(trgtWORLD);
   WorldID2IAUFrame(trgtWORLD, trgtCNH_STRING);

   if (trgtWORLD == LUNA) {
      spkez_c(tgt_world_naif, jd_tdb_j2000_sec, "ECLIPJ2000", "NONE",
              WorldID2NAIFID(SOL), Hstate, &light_time);
      spkez_c(tgt_world_naif, jd_tdb_j2000_sec, "J2000", "NONE",
              WorldID2NAIFID(EARTH), Nstate, &light_time);
      for (i = 0; i < 3; i++) {
         trgtPosH[i] = Hstate[i] * 1e3;
         trgtPosN[i] = Nstate[i] * 1e3;
      }
   }
   else {
      spkez_c(tgt_world_naif, jd_tdb_j2000_sec, "ECLIPJ2000", "NONE",
              WorldID2NAIFID(SOL), Nstate, &light_time);
      for (i = 0; i < 3; i++) {
         trgtPosH[i] = Nstate[i] * 1e3;
         trgtPosN[i] = Nstate[i] * 1e3;
      }
   }
   pxform_c("J2000", trgtCNH_STRING, jd_tdb_j2000_sec, CNH);
   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         trgtCNH[i][j] = CNH[i][j];
      }
   }
}
#endif
/**********************************************************************/
void LoadConstellations(void)
{

   FILE *infile;
   char junk[120], newline, response[120];
   double RA, Dec;
   long i, j;
   struct ConstellationType *C;

   infile = FileOpen(ModelPath, "Constellations.txt", "r");

   for (i = 0; i < 89; i++) {
      C = &Constell[i];
      fscanf(infile, "%s %s %ld %ld\n", C->Tag, response, &C->Nstars,
             &C->Nlines);
      C->Class = DecodeString(response);

      C->StarVec = CreateMatrix(C->Nstars, 3);

      C->Star1 = (long *)calloc(C->Nlines, sizeof(long));
      C->Star2 = (long *)calloc(C->Nlines, sizeof(long));

      for (j = 0; j < C->Nstars; j++) {
         fscanf(infile, "%lf %lf %[^\n] %[\n]", &RA, &Dec, junk, &newline);
         RA               *= D2R;
         Dec              *= D2R;
         C->StarVec[j][0]  = cos(RA) * cos(Dec);
         C->StarVec[j][1]  = sin(RA) * cos(Dec);
         C->StarVec[j][2]  = sin(Dec);
      }

      for (j = 0; j < C->Nlines; j++) {
         fscanf(infile, "%ld %ld %[^\n] %[\n]", &C->Star1[j], &C->Star2[j],
                junk, &newline);
      }
   }

   fclose(infile);
}
/**********************************************************************/
void LoadSchatten(void)
{
   // TODO: Schatten Table is UTC??
   FILE *infile;
   char junk[120], newline;
   long i;

   infile = FileOpen(ModelPath, "SW_predict_MSAFE_95_50_Pred_Ap.txt", "rt");

   fscanf(infile, "%[^\n] %[\n]", junk, &newline);
   fscanf(infile, "%[^\n] %[\n]", junk, &newline);

   DateType date = {0};
   date.Second   = RATIONAL_ZERO;
   date.system   = TT_TIME;
   for (i = 0; i < 1009; i++) {
      fscanf(infile, "%ld %ld %lf %lf %lf %lf,%[^\n] %[\n]", &date.Year,
             &date.Month, &SchattenTable[1][i], &SchattenTable[2][i],
             &SchattenTable[3][i], &SchattenTable[4][i], junk, &newline);
      JDType jd           = Date2JD(date, GMAT_MJD_EPOCH);
      SchattenTable[0][i] = JDToDays(jd);
   }
   fclose(infile);
}
/**********************************************************************/
void InitSim(int argc, char **argv)
{
   char response[120], response1[120], response2[120];
   long Iorb, Isc, i;
   WorldID Iw;
   long MinorBodiesExist;
   long JunkTag;
   double CGJ[3][3] = {
       {-0.054873956175539, -0.873437182224835, -0.483835031431981},
       {0.494110775064704, -0.444828614979805, 0.746981957785302},
       {-0.867665382947348, -0.198076649977489, 0.455985113757595}};
   double CJH[3][3];

   Pi          = PI;
   TwoPi       = TWOPI;
   HalfPi      = HALFPI;
   SqrtTwo     = SQRTTWO;
   SqrtHalf    = SQRTHALF;
   GoldenRatio = GOLDENRATIO;
   A2R         = D2R / 3600.0;
   R2A         = R2D * 3600.0;

   // Exact Values from GMAT, gives agreement to 0.5 meters for all bodies
   World[EARTH].CNH[0][0] = 1.0;
   World[EARTH].CNH[0][1] = 0.0;
   World[EARTH].CNH[0][2] = 0.0;
   World[EARTH].CNH[1][0] = 0.0;
   World[EARTH].CNH[1][1] = 0.917482062076895741;
   World[EARTH].CNH[1][2] = -0.397777155914121383;
   World[EARTH].CNH[2][0] = 0.0;
   World[EARTH].CNH[2][1] = 0.397777155914121383;
   World[EARTH].CNH[2][2] = 0.917482062076895741;
   C2Q(World[EARTH].CNH, World[EARTH].qnh);
   C2Q(World[EARTH].CNH, qjh);

   // /* Calculate high precise ecliptic value */
   // double ecliptic;
   // ecliptic = -84381.448/3600.0;
   // /* Find high precision DCM/Q for J2000 to Heliocentric Ecliptic rotation*/
   // A2C(123, ecliptic * D2R, 0.0, 0.0, World[EARTH].CNH);
   // C2Q(World[EARTH].CNH, World[EARTH].qnh);
   // C2Q(World[EARTH].CNH, qjh);
   for (i = 0; i < 3; i++)
      World[EARTH].qnj[i] = 0.0;
   World[EARTH].qnj[3] = 1.0;

   sprintf(InOutPath, "./InOut/");
   sprintf(ModelPath, "./Model/");
   if (argc > 1)
      sprintf(InOutPath, "./%s/", argv[1]);
   if (argc > 2)
      sprintf(ModelPath, "./%s/", argv[2]);

   char tempargs[BUFSIZE];
   char *ret;
   DIR *OutDir;
   DIR *ModelDir;

#ifdef __linux__
   char *real_path = realpath("/proc/self/exe", NULL);
   strcpy(ExeDir, real_path);
   ret = strrchr(ExeDir, '/');
   free(real_path);
#elif defined __MINGW32__
   GetModuleFileName(NULL, tempargs, sizeof(tempargs));
   _fullpath(ExeDir, tempargs, sizeof(tempargs));
   ret = strrchr(ExeDir, '\\');
#elif defined _WIN32
   GetModuleFileName(NULL, tempargs, sizeof(tempargs));
   _fullpath(ExeDir, tempargs, sizeof(tempargs));
   ret = strrchr(ExeDir, '\\');
#elif defined _WIN64
   GetModuleFileName(NULL, tempargs, sizeof(tempargs));
   _fullpath(ExeDir, tempargs, sizeof(tempargs));
   ret = strrchr(ExeDir, '\\');
#elif defined __APPLE__
   uint32_t bytes;
   bytes = 1000;
   bytes = sizeof("/0");
   _NSGetExecutablePath("/0", &bytes);
   _NSGetExecutablePath(tempargs, &bytes);
   realpath(tempargs, ExeDir);
   ret = strrchr(ExeDir, '/');
#endif

   if (ret != NULL)
      *ret = '\0';
   ret = strrchr(ExeDir, '.');
   if (ret != NULL)
      *ret = '\0';

   strcpy(ModelPath, ExeDir);
   strcat(ModelPath, "/Model/");

   CLI_ARGS = docopt(argc, argv, /* help */ 1);

   if (CLI_ARGS.indir != NULL) {
      sprintf(InOutPath, "%s", CLI_ARGS.indir);
      strcat(InOutPath, "/");
   }

   if (CLI_ARGS.outdir != NULL) {
      strcpy(OutPath, CLI_ARGS.outdir);
      printf("%s", OutPath);
      strcat(OutPath, "/");
   }

   if (CLI_ARGS.defaultdir != NULL) {
      strcat(CLI_ARGS.defaultdir, "/");
      if (CLI_ARGS.indir == NULL) {
         strcpy(InOutPath, CLI_ARGS.defaultdir);
         strcat(InOutPath, "/InOut/");
      }
      if (CLI_ARGS.outdir == NULL) {
         strcpy(OutPath, CLI_ARGS.defaultdir);
         strcat(OutPath, "/output/");
      }
      if (CLI_ARGS.modeldir == NULL) {
         strcpy(SCModelPath, CLI_ARGS.defaultdir);
         strcat(SCModelPath, "/Model/");
         ModelDir = opendir(SCModelPath);
         if (ModelDir) {
            closedir(ModelDir);
         }
         else if (ENOENT == errno) {
            strcpy(SCModelPath, ModelPath);
         }
      }
   }
   else { /* Default Directories */
      const char *def_mission = "/Demo";
      if (CLI_ARGS.indir == NULL) {
         strcpy(InOutPath, ExeDir);
         strcat(InOutPath, def_mission);
         strcat(InOutPath, "/InOut/");
      }
      if (CLI_ARGS.outdir == NULL) {
         strcpy(OutPath, ExeDir);
         strcat(OutPath, def_mission);
         strcat(OutPath, "/output/");
      }
      if (CLI_ARGS.modeldir == NULL)
         strcpy(SCModelPath, ModelPath);
   }
   OutDir = opendir(OutPath);
   if (OutDir) {
      closedir(OutDir);
   }
   else if (ENOENT == errno) {
#if defined __MINGW32__
      mkdir(OutPath);
#elif defined _WIN32
      mkdir(OutPath);
#elif defined _WIN64
      mkdir(OutPath);
#elif defined __APPLE__
      mkdir(OutPath, 0777);
#elif defined __linux__
      mkdir(OutPath, 0777);
#else
#error "Computing platform not detected!"
#endif
   }

   printf("\nExeDir: %s \n", ExeDir);
   printf("\nInput Path: %s \n", InOutPath);
   printf("Output Path: %s \n", OutPath);
   printf("SC Model Path: %s \n \n", SCModelPath);

   /* .. Read from file Inp_Sim.txt */
   struct fy_document *fyd =
       fy_document_build_and_check(NULL, InOutPath, "Inp_Sim.yaml");

   struct fy_node *root = fy_document_root(fyd);
   struct fy_node *node = fy_node_by_path_def(root, "/Simulation Control");

   /* .. Time Mode */
   /* .. Duration, Step size */
   /* .. File output interval */
   /* .. RNG Seed */
   /* .. Graphics Front End? */
   /* .. Cmd Script File Name */
   if (fy_node_scanf(node,
                     "/Mode %119s "
                     "/Duration %lf "
                     "/Step Size %lf "
                     "/File Interval %lf "
                     "/RNG Seed %ld "
                     "/Command File %999s",
                     response, &STOPTIME, &DTSIM, &DTOUT, &RngSeed,
                     CmdFileName) != 6) {
      fprintf(stderr, "Simulation Control in Inp_Sim is improperly configured. "
                      "Exiting...\n");
      exit(EXIT_FAILURE);
   }
   DTSIM_RAT = double2rational(DTSIM);
   DTOUT_RAT = double2rational(DTOUT);
   TimeMode  = DecodeString(response);
   GLEnable  = getYAMLBool(fy_node_by_path_def(node, "/Enable Graphics"));

   if (CLI_ARGS.graphics != NULL) {
      printf("\n!!!!!! Graphics Overriden !!!!! \n");
      if (strlen(CLI_ARGS.graphics) != 1) {
         if ((strncasecmp(CLI_ARGS.graphics, "TRUE", 5) == 0)) {
            GLEnable = 1;
         }
         else if ((strncasecmp(CLI_ARGS.graphics, "FALSE", 6) == 0)) {
            GLEnable = 0;
         }
         else {
            printf("Cannot Parse Override Option. Graphics Enable = %s \n",
                   GLEnable ? "True" : "False");
         }
      }
      else
         GLEnable = (CLI_ARGS.graphics[0] - '0');

      printf("!!!!!!!!!!!!!!!!!!!!!! \n \n");
   }

   printf("Graphics = %s \n\n", GLEnable ? "TRUE" : "FALSE");

   /* .. Reference Orbits */
   node = fy_node_by_path_def(root, "/Orbits");
   Norb = fy_node_sequence_item_count(node);
   Orb  = NULL;
   Orb  = (struct OrbitType *)calloc(Norb, sizeof(struct OrbitType));
   if (Orb == NULL) {
      fprintf(stderr, "Orb calloc returned null pointer.  Bailing out!\n");
      exit(EXIT_FAILURE);
   }
   Frm = NULL;
   Frm = (struct FormationType *)calloc(Norb, sizeof(struct FormationType));
   if (Frm == NULL) {
      fprintf(stderr, "Frm calloc returned null pointer.  Bailing out!\n");
      exit(EXIT_FAILURE);
   }

   struct fy_node *iterNode = NULL;
   Iorb                     = 0;
   WHILE_FY_ITER(node, iterNode)
   {
      if (!fy_node_scanf(iterNode, "/Name %39[^\n]s", Orb[Iorb].FileName)) {
         fprintf(stderr, "Could not find Orbit name. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      strcat(Orb[Iorb].FileName, ".yaml");
      Orb[Iorb].Exists = getYAMLBool(fy_node_by_path_def(iterNode, "/Enabled"));
      Orb[Iorb].Tag    = Iorb;
      Iorb++;
   }

   /* .. Spacecraft */
   node = fy_node_by_path_def(root, "/SCs");
   Nsc  = fy_node_sequence_item_count(node);
   SC   = NULL;
   SC   = (struct SCType *)calloc(Nsc, sizeof(struct SCType));
   if (SC == NULL) {
      fprintf(stderr, "SC calloc returned null pointer.  Bailing out!\n");
      exit(EXIT_FAILURE);
   }

   iterNode = NULL;
   Isc      = 0;
   WHILE_FY_ITER(node, iterNode)
   {
      if (fy_node_scanf(iterNode,
                        "/Name %49s "
                        "/Orbit %19s",
                        SC[Isc].FileName, response) != 2) {
         fprintf(stderr,
                 "Could not find SC's name and/or its orbit. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      strcat(SC[Isc].FileName, ".yaml");
      strcat(response, ".yaml");
      SC[Isc].RefOrb = -1;
      for (Iorb = 0; Iorb < Norb; Iorb++) {
         if (!strcmp(response, Orb[Iorb].FileName)) {
            SC[Isc].RefOrb = Iorb;
            break;
         }
      }
      if (SC[Isc].RefOrb == -1) {
         fprintf(stderr,
                 "SC[%ld] named %49s is assigned to invalid orbit %19s. "
                 "Exiting...\n",
                 Isc, SC[Isc].FileName, response);
         exit(EXIT_FAILURE);
      }
      SC[Isc].Exists = getYAMLBool(fy_node_by_path_def(iterNode, "/Enabled"));
      if ((SC[Isc].Exists && !Orb[SC[Isc].RefOrb].Exists) ||
          (SC[Isc].RefOrb > Norb)) {
         fprintf(stderr, "Yo!  SC[%ld] is assigned to non-existent Orb[%ld]\n",
                 Isc, SC[Isc].RefOrb);
         exit(EXIT_FAILURE);
      }
      SC[Isc].ID = Isc;
      Isc++;
   }

   /* .. Environment */
   /* .. Date and time (UTC) */
   node              = fy_node_by_path_def(root, "/Time");
   Rational millisec = (Rational){.whole = 0, .num = 0, .den = 1000};
   double sec        = 0;
   if (fy_node_scanf(node,
                     "/Date/Year %ld "
                     "/Date/Month %ld "
                     "/Date/Day %ld "
                     "/Time/Hour %ld "
                     "/Time/Minute %ld "
                     "/Time/Second %lf "
                     "/Time/Millisecond %ld "
                     "/Leap Seconds %lf",
                     &UTC.Year, &UTC.Month, &UTC.Day, &UTC.Hour, &UTC.Minute,
                     &sec, &millisec.num, &LeapSec) != 8) {
      fprintf(stderr, "Time is improperly configured in Inp_Sim. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   UTC.Second = double2rational(sec);
   UTC.Second = RationalAdd(UTC.Second, millisec);

   /* .. Choices for Modeling Solar Activity */
   // TODO: add atmo model properties to world and use this to
   // configure properties
   node     = fy_node_by_path_def(root, "/Perturbation Models");
   iterNode = NULL;
   WHILE_FY_ITER(fy_node_by_path_def(node, "/Atmosphere/Models"), iterNode)
   {
      if (fy_node_scanf(iterNode,
                        "/World %119s "
                        "/Method %119s",
                        response1, response2) != 2) {
         fprintf(stderr,
                 "Could not find World and/or Method for Atmospheric Model. "
                 "Exiting...\n");
         exit(EXIT_FAILURE);
      }
      Iw            = GetWorldID(response1);
      long atmoType = DecodeString(response2);
      double f10p7 = 0.0, geomag = 0.0;
      if (atmoType == USER_ATMO)
         if (fy_node_scanf(iterNode, "/F10.7 %lf /Ap %lf", &f10p7, &geomag) !=
             2) {
            fprintf(
                stderr,
                "Could not find user defined F10.7 and/or Ap. Exiting...\n");
            exit(EXIT_FAILURE);
         }

      switch (Iw) {
         case EARTH:
            AtmoOption  = atmoType;
            Flux10p7    = f10p7;
            GeomagIndex = geomag;
            break;
         default:
            fprintf(stderr,
                    "World %119s does not have a configured atmospheric model. "
                    "Exiting...\n",
                    response1);
            exit(EXIT_FAILURE);
            break;
      }
   }

   /* .. Magnetic Field Model */
   // TODO: make magfield a property of worlds, so each world can have a
   // configurable magnetic field
   // TODO: make magfield coefficent files a field for models?
   iterNode = NULL;
   WHILE_FY_ITER(fy_node_by_path_def(node, "/Magnetic/Models"), iterNode)
   {
      if (fy_node_scanf(iterNode,
                        "/World %119s "
                        "/Method %119s",
                        response1, response2) != 2) {
         fprintf(stderr,
                 "Could not find World and/or Method for Magnetic Model. "
                 "Exiting...\n");
         exit(EXIT_FAILURE);
      }
      Iw           = GetWorldID(response1);
      long magType = DecodeString(response2);
      switch (Iw) {
         case EARTH:
            MagModel.Type = magType;
            break;
         default:
            fprintf(stderr,
                    "World %119s does not have a configured magnetic field "
                    "model. Exiting...\n",
                    response1);
            exit(EXIT_FAILURE);
            break;
      }
      if (magType == IGRF) {
         long N = 0, M = 0;
         if (fy_node_scanf(iterNode,
                           "/Degree %ld "
                           "/Order %ld",
                           &N, &M) != 2) {
            fprintf(stderr,
                    "Could not find Degree and/or Order for Magnetic Field "
                    "Model. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         switch (Iw) {
            case EARTH:
               MagModel.N = N;
               MagModel.M = M;
               break;
            default:
               fprintf(stderr,
                       "World %119s does not have a configured spherical "
                       "harmonic magnetic field model. Exiting...\n",
                       response1);
               exit(EXIT_FAILURE);
               break;
         }
      }
   }

   /* .. Earth, Mars, Luna Gravity Models */
   for (Iw = 0; Iw < NWORLD; Iw++) {
      struct SphereHarmType *gravModel = &World[Iw].GravModel;
      strcpy(gravModel->modelFile, "");
      gravModel->N     = 0;
      gravModel->r_ref = 0;
      World[Iw].mu     = 0;
   }
   iterNode = NULL;
   struct fy_node *grav_model_list =
       fy_node_by_path_def(node, "/Gravitation/Models");
   WHILE_FY_ITER(grav_model_list, iterNode)
   {
      long N = 0, M = 0;
      if (fy_node_scanf(iterNode,
                        "/World %119s "
                        "/Degree %ld "
                        "/Order %ld",
                        response, &N, &M) != 3) {
         fprintf(stderr, "Could not find World, Degree, and/or Order for "
                         "Gravitational Model. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      Iw                               = GetWorldID(response);
      struct SphereHarmType *gravModel = &World[Iw].GravModel;
      gravModel->N                     = N;
      gravModel->M                     = M;

      // Load model file name from Inp_Sim if it is there, otherwise leave blank
      // to use default later
      struct fy_node *file_node = fy_node_by_path_def(iterNode, "/Model File");
      if (file_node != NULL) {
         size_t len            = 0;
         const char *modelFile = fy_node_get_scalar(file_node, &len);
         strncpy(gravModel->modelFile, modelFile, 39);
         gravModel->modelFile[39] = 0; // ensure null termination
      }

      // override the reference radius of the model hardcoded in source, usually
      // hardcoded as the equatorial radius of the body
      struct fy_node *r_node =
          fy_node_by_path_def(iterNode, "/Reference Radius");
      if (r_node != NULL)
         fy_node_scanf(r_node, "/ %lf", &gravModel->r_ref);

      // override the hard coded gravitational parameter. For a minor body,
      // overrides the gravitational parameter in the minor body file
      struct fy_node *gm_node =
          fy_node_by_path_def(iterNode, "/Gravitational Parameter");
      if (gm_node != NULL)
         fy_node_scanf(gm_node, "/ %lf", &World[Iw].mu);
   }

   /* .. Toggle on/off various environmental effects */
   AeroActive = getYAMLBool(fy_node_by_path_def(node, "/Atmosphere/Enabled"));
   AeroShadowsActive =
       getYAMLBool(fy_node_by_path_def(node, "/Atmosphere/Shadows"));
   GGActive =
       getYAMLBool(fy_node_by_path_def(node, "/Gravitation/Gravity Gradient"));
   SolPressActive = getYAMLBool(fy_node_by_path_def(node, "/SRP/Enabled"));
   SolPressShadowsActive =
       getYAMLBool(fy_node_by_path_def(node, "/SRP/Shadows"));
   ResidualDipoleActive =
       getYAMLBool(fy_node_by_path_def(node, "/Magnetic/Residual Mag Moment"));
   GravPertActive =
       getYAMLBool(fy_node_by_path_def(node, "/Gravitation/Enabled"));
   ThrusterPlumesActive =
       getYAMLBool(fy_node_by_path_def(node, "/Thruster Plume"));
   ContactActive = getYAMLBool(fy_node_by_path_def(node, "/Contact"));
   SloshActive   = getYAMLBool(fy_node_by_path_def(node, "/CFD Slosh"));
   AlbedoActive  = getYAMLBool(fy_node_by_path_def(node, "/Albedo on CSS"));
   ComputeEnvTrq =
       getYAMLBool(fy_node_by_path_def(node, "/Output Env Torques to File"));

   /* .. Celestial Bodies */
   if (!fy_node_scanf(root, "/Ephem Type %119s", response)) {
      fprintf(stderr, "Could not find Ephemeris Type in Inp_Sim. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   EphemOption = GetEphemType(response);
#ifndef _ENABLE_SPICE_
   if (EphemOption == EPH_SPICE) {
      fprintf(stderr, "You must compile DeepThought with SPICE in order to use "
                      "SPICE ephemerides. Exiting...\n");
      exit(EXIT_FAILURE);
   }
#endif
   node = fy_node_by_path_def(root, "/Celestial Bodies");
   // I wish this was more programmatic, but it doesn't really need to be
   // I guess
   World[MERCURY].Exists = getYAMLBool(fy_node_by_path_def(node, "/Mercury"));
   World[VENUS].Exists   = getYAMLBool(fy_node_by_path_def(node, "/Venus"));
   World[EARTH].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Earth and Luna"));
   World[MARS].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Mars and its moons"));
   World[JUPITER].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Jupiter and its moons"));
   World[SATURN].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Saturn and its moons"));
   World[URANUS].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Uranus and its moons"));
   World[NEPTUNE].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Neptune and its moons"));
   World[PLUTO].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Pluto and its moons"));
   MinorBodiesExist =
       getYAMLBool(fy_node_by_path_def(node, "/Asteroids and Comets"));

   /* .. Lagrange Point Systems */
   node = fy_node_by_path_def(root, "/Lagrange Systems");
   LagSys[EARTHMOON].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Earth-Moon"));
   LagSys[SUNEARTH].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Sun-Earth"));
   LagSys[SUNJUPITER].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Sun-Jupiter"));

   /* .. Ground Stations */
   node          = fy_node_by_path_def(root, "/Ground Stations");
   Ngnd          = fy_node_sequence_item_count(node);
   GroundStation = (struct GroundStationType *)calloc(
       Ngnd, sizeof(struct GroundStationType));
   iterNode = NULL;
   WHILE_FY_ITER(node, iterNode)
   {
      struct fy_node *seqNode =
          fy_node_by_path_def(iterNode, "/Ground Station");
      long Ignd = 0;
      if (!fy_node_scanf(seqNode, "/Index %ld", &Ignd)) {
         fprintf(stderr, "Could not find Ground Station Index. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      if (fy_node_scanf(seqNode,
                        "/World %119s "
                        "/Longitude %lf "
                        "/Latitude %lf",
                        response, &GroundStation[Ignd].lng,
                        &GroundStation[Ignd].lat) != 3) {
         fprintf(stderr,
                 "Ground Station %ld is improperly configured. Exiting...\n",
                 Ignd);
         exit(EXIT_FAILURE);
      }
      size_t str_len;
      const char *label =
          fy_node_get_scalar(fy_node_by_path_def(seqNode, "/Label"), &str_len);
      strncpy(GroundStation[Ignd].Label, label, str_len > 39 ? 39 : str_len);
      GroundStation[Ignd].World = GetWorldID(response);
      GroundStation[Ignd].Exists =
          getYAMLBool(fy_node_by_path_def(iterNode, "/Ground Station/Enabled"));
   }

   fy_document_destroy(fyd);
   /* .. Load Materials */
   Nmatl = 0;
   Matl  = AddMtlLib(ModelPath, "42.mtl", Matl, &Nmatl);
   ScaleSpecDiffFrac(Matl, Nmatl);

   /* Known bug: First Geom loaded in gets corrupted.
   Kludge fix: Load a sacrificial geom first.  */
   Geom = LoadWingsObjFile(ModelPath, "Point.obj", &Matl, &Nmatl, Geom, &Ngeom,
                           &JunkTag, FALSE);

   /* .. Time */
   TT.system  = TT_TIME;
   UTC.system = UTC_TIME;
   TDB.system = TDB_TIME;
   if (TimeMode == EXTERNAL_TIME) {
      printf("Initializing with External Time\n");
      UTC = RealSystemTime();
   }

   UTC.doy     = MD2DOY(UTC.Year, UTC.Month, UTC.Day);
   SimTime     = 0.0;
   JD_TT_MJD_0 = Date2JD(UTC, J2000_EPOCH);
   CivilTime   = JDToTime(JD_TT_MJD_0);
   ChangeSystemEpoch(TT_TIME, GMAT_MJD_EPOCH, &JD_TT_MJD_0);
   JD_TT_MJD  = JD_TT_MJD_0;
   JD_TDB_MJD = JD_TT_MJD;
   ChangeSystemEpoch(TDB_TIME, GMAT_MJD_EPOCH, &JD_TDB_MJD);

   DynTime    = JDToDynTime(JD_TT_MJD);
   AtomicTime = DynTime - 32.184; /* TAI */
   GpsTime    = AtomicTime - 19.0;

   TT  = JDToDate(JD_TT_MJD, TT_TIME);
   TDB = JDToDate(JD_TDB_MJD, TDB_TIME);

   GpsTimeToGpsDate(GpsTime, &GpsRollover, &GpsWeek, &GpsSecond);

   LoadEphems(EphemOption, JD_TDB_MJD, &JplHeader, World);

   /* .. Load Sun and Planets */
   LoadSun(EphemOption, JD_TDB_MJD, &JplHeader, World);
   LoadPlanets(EphemOption, JD_TDB_MJD, &JplHeader, World);

   /* .. Load Moons */
   LoadMoons(EphemOption, JD_TDB_MJD, &JplHeader, World);

   /* .. Asteroids and Comets */
   if (MinorBodiesExist)
      LoadMinorBodies(EphemOption, JD_TDB_MJD, &JplHeader, World);
   else
      Nmb = 0;

   UpdateEphems(EphemOption, JD_TDB_MJD, &JplHeader, World);

   /* .. Regions */
   LoadRegions();

   /* .. Galactic Frame */
   Q2C(qjh, CJH);
   MxM(CGJ, CJH, CGH);

   /* .. Ground Station Locations */
   for (i = 0; i < Ngnd; i++) {
      if (GroundStation[i].Exists && !World[GroundStation[i].World].Exists) {
         printf("Ground Station[%ld].World doesn't exist.\n", i);
      }
      GroundStation[i].PosW[0] = World[GroundStation[i].World].rad *
                                 cos(GroundStation[i].lng * D2R) *
                                 cos(GroundStation[i].lat * D2R);
      GroundStation[i].PosW[1] = World[GroundStation[i].World].rad *
                                 sin(GroundStation[i].lng * D2R) *
                                 cos(GroundStation[i].lat * D2R);
      GroundStation[i].PosW[2] =
          World[GroundStation[i].World].rad * sin(GroundStation[i].lat * D2R);
   }

   /* .. Note that some moons are so dominated by their planet that   */
   /*    they don't really have a sphere of influence!                */
   /*
   for(Iw=1;Iw<NWORLD;Iw++) {
      if (0.5*World[Iw].RadOfInfluence < World[Iw].rad)
         printf("World %s is bigger than its inner sphere of
   influence\n",World[Iw].Name); if (2.0*World[Iw].RadOfInfluence <
   World[Iw].rad) printf("World %s is bigger than its outer sphere of
   influence\n",World[Iw].Name);
   }
   */

   InitLagrangePoints();
   for (Iorb = 0; Iorb < Norb; Iorb++) {
      if (Orb[Iorb].Exists)
         InitOrbit(&Orb[Iorb], JD_TDB_MJD);
   }
   OrbitMotion(World, Orb, DynTime);
   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         InitSpacecraft(&SC[Isc]);
      }
   }
   long nonDSMFSW = FALSE, DSMFSW = FALSE;
   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         DSMFSW    |= SC[Isc].FswTag == DSM_FSW;
         nonDSMFSW |= SC[Isc].FswTag != DSM_FSW;
         if (nonDSMFSW && DSMFSW) {
            fprintf(stderr,
                    "Mixing DSM_FSW and non DSM_FSW flightsoftware tags is not "
                    "supported. Exiting...\n");
            exit(EXIT_FAILURE);
         }
      }
   }

   LoadTdrs();

   RNG = CreateRandomProcess(RngSeed);

   LoadConstellations();

   LoadSchatten();
}

/* #ifdef __cplusplus
** }
** #endif
*/

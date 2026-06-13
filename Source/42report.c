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
#include "navkit.h"

#include <sys/stat.h>
#include <sys/types.h>

#define PRNT_DBL "%18.24le"

static inline void
_fprintf_vec3(FILE *file, const char *c __attribute__((unused)), vec3_t v)
{
   fprintf(file, PRNT_DBL "%s", v.x, c);
   fprintf(file, PRNT_DBL "%s", v.y, c);
   fprintf(file, PRNT_DBL "%s", v.z, c);
}
static inline void
_fprintf_quat(FILE *file, const char *c __attribute__((unused)), quat_t q)
{
   fprintf(file, PRNT_DBL "%s", q.x, c);
   fprintf(file, PRNT_DBL "%s", q.y, c);
   fprintf(file, PRNT_DBL "%s", q.z, c);
   fprintf(file, PRNT_DBL "%s", q.s, c);
}
static inline void
_fprintf_mat3x3(FILE *file, const char *c __attribute__((unused)), mat3x3_t m)
{
   for (int i = 0; i < 3; i++) {
      vec3_t *v = &m.rows[i];
      fprintf(file, PRNT_DBL "%s", v->x, c);
      fprintf(file, PRNT_DBL "%s", v->y, c);
      fprintf(file, PRNT_DBL "%s", v->z, c);
   }
}
static inline void
_fprintf_jdtype(FILE *file, const char *c __attribute__((unused)), JDType jd)
{
   char jdstr[JD_STR_LEN] = {'\0'};
   jddays2str(jd, jdstr);
   fprintf(file, "%s%s", jdstr, c);
}
static inline void _fprintf_datetype(FILE *file,
                                     const char *c __attribute__((unused)),
                                     DateType date)
{
   fprintf(file, "%ld:%02ld:%02ld:%02ld:%02ld:%09.6lf%s", date.Year, date.Month,
           date.Day, date.Hour, date.Minute, rational2double(date.Second), c);
}
static inline void newline_fflush(FILE *file)
{
   fprintf(file, "\n");
   fflush(file);
}

#define def_prnt_fmt(x, delim)                                                 \
   _Generic((x),                                                               \
       int: "%d" delim,                                                        \
       long: "%ld" delim,                                                      \
       long long: "%lld" delim,                                                \
       unsigned int: "%u" delim,                                               \
       unsigned long: "%lu" delim,                                             \
       unsigned long long: "%llu" delim,                                       \
       float: PRNT_DBL delim,                                                  \
       double: PRNT_DBL delim,                                                 \
       signed char: "%c" delim,                                                \
       unsigned char: "%c" delim,                                              \
       char: "%c" delim,                                                       \
       char *: "%s" delim,                                                     \
       const char *: "%s" delim,                                               \
       default: "" delim)

#define _print_fnc(file, x, delim)                                             \
   _Generic((x),                                                               \
       vec3_t: _fprintf_vec3,                                                  \
       quat_t: _fprintf_quat,                                                  \
       mat3x3_t: _fprintf_mat3x3,                                              \
       DateType: _fprintf_datetype,                                            \
       JDType: _fprintf_jdtype,                                                \
       default: fprintf)((file), def_prnt_fmt(x, delim), (x))

#define file_print(file, x) _print_fnc(file, x, " ")
#define csv_print(file, x)  _print_fnc(file, x, ",")

/* #ifdef __cplusplus
** namespace _42 {
** using namespace Kit;
** #endif
*/

/*********************************************************************/
double FindTotalProjectedArea(struct SCType *S, vec3_t VecN)
{
   struct BodyType *B;
   struct GeomType *G;
   struct PolyType *P;
   double ProjArea = 0.0, VoN;
   vec3_t VecB;
   long Ib, Ipoly;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];

      /* Transform Direction Vector from N to B */
      VecB = MxV(B->CN, VecN);

      G = &Geom[B->GeomTag];
      for (Ipoly = 0; Ipoly < G->Npoly; Ipoly++) {
         P   = &G->Poly[Ipoly];
         VoN = VoV(VecB, P->Norm);
         if (VoN > 0.0)
            ProjArea += VoN * P->Area;
      }
   }
   return (ProjArea);
}
/*********************************************************************/
double FindTotalUnshadedProjectedArea(struct SCType *S, vec3_t VecN)
{
   struct BodyType *B;
   struct GeomType *G;
   struct PolyType *P;
   double ProjArea = 0.0, VoN;
   vec3_t VecB;
   long Ib, Ipoly;

   FindUnshadedAreas(S, VecN);

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];

      /* Transform Direction Vector from N to B */
      VecB = MxV(B->CN, VecN);

      G = &Geom[B->GeomTag];
      for (Ipoly = 0; Ipoly < G->Npoly; Ipoly++) {
         P   = &G->Poly[Ipoly];
         VoN = VoV(VecB, P->Norm);
         if (VoN > 0.0)
            ProjArea += VoN * P->UnshadedArea;
      }
   }
   return (ProjArea);
}
/*********************************************************************/
void MagReport(void)
{
   static FILE *magfile;
   static long First = 1;

   if (First) {
      First   = 0;
      magfile = FileOpen(OutPath, "MagBVB.42", "wt");
   }

   file_print(magfile, SC[0].bvb);
   for (int i = 0; i < 3; i++)
      file_print(magfile, SC[0].MAG[i].Field);
   file_print(magfile, SC[0].AC.bvb);
   newline_fflush(magfile);
}
/*********************************************************************/
void GyroReport(void)
{
   static FILE *gyrofile;
   static long First = 1;

   if (First) {
      First    = 0;
      gyrofile = FileOpen(OutPath, "Gyro.42", "wt");
   }

   file_print(gyrofile, SC[0].B[0].wn);
   for (int i = 0; i < 3; i++)
      file_print(gyrofile, SC[0].Gyro[i].TrueRate);
   for (int i = 0; i < 3; i++)
      file_print(gyrofile, SC[0].Gyro[i].Bias);
   for (int i = 0; i < 3; i++)
      file_print(gyrofile, SC[0].Gyro[i].Angle);
   for (int i = 0; i < 3; i++)
      file_print(gyrofile, SC[0].Gyro[i].MeasRate);
   file_print(gyrofile, SC[0].AC.wbn);
   newline_fflush(gyrofile);
}
/*********************************************************************/
void DSM_AttitudeReport(void)
{
   static FILE **attitudefile;
   static long First = 1;
   long Isc;
   char s[40];

   if (First) {
      attitudefile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_attitude_%02li.42", Isc);
            attitudefile[Isc] = FileOpen(OutPath, s, "wt");
            file_print(attitudefile[Isc], "qbn_0 qbn_1 qbn_2 qbn_3 ");
            file_print(attitudefile[Isc], "wbn_X wbn_Y wbn_Z ");
            newline_fflush(attitudefile[Isc]);
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         file_print(attitudefile[Isc], SC[Isc].B[0].qn);
         file_print(attitudefile[Isc], SC[Isc].B[0].wn);
         newline_fflush(attitudefile[Isc]);
      }
   }
}
/*********************************************************************/
void DSM_AC_AttitudeReport(void)
{
   static FILE **attitudefile;
   static long First = 1;
   long Isc;
   char s[40];

   if (First) {
      attitudefile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_AC_attitude_%02li.42", Isc);
            attitudefile[Isc] = FileOpen(OutPath, s, "wt");
            file_print(attitudefile[Isc], "qbn_0 qbn_1 qbn_2 qbn_3 ");
            file_print(attitudefile[Isc], "wbn_X wbn_Y wbn_Z ");
            newline_fflush(attitudefile[Isc]);
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         file_print(attitudefile[Isc], SC[Isc].AC.qbn);
         file_print(attitudefile[Isc], SC[Isc].AC.wbn);
         newline_fflush(attitudefile[Isc]);
      }
   }
}
/*********************************************************************/
void DSM_InertialReport(void)
{
   static FILE **inertialfile;
   static long First = 1;
   long Isc;
   vec3_t PosL;
   char s[40];

   if (First) {
      inertialfile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_inertial_%02li.42", Isc);
            inertialfile[Isc] = FileOpen(OutPath, s, "wt");
            file_print(inertialfile[Isc], "PosN_X PosN_Y PosN_Z ");
            file_print(inertialfile[Isc], "VelN_X VelN_Y VelN_Z ");
            file_print(inertialfile[Isc], "PosL_X PosL_Y PosL_Z ");
            newline_fflush(inertialfile[Isc]);
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         PosL = MxV(SC[0].CLN, SC[Isc].PosN);
         file_print(inertialfile[Isc], SC[Isc].PosN);
         file_print(inertialfile[Isc], SC[Isc].VelN);
         file_print(inertialfile[Isc], PosL);
         newline_fflush(inertialfile[Isc]);
      }
   }
}
/*********************************************************************/
void DSM_RelativeReport(void)
{
   static FILE **relativefile;
   static long First = 1;
   long Isc;
   char s[40];

   if (First) {
      relativefile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_relative_L_%02li.42", Isc);
            relativefile[Isc] = FileOpen(OutPath, s, "wt");
            file_print(relativefile[Isc], "PosR_X PosR_Y PosR_Z ");
            file_print(relativefile[Isc], "VelR_X VelR_Y VelR_Z ");
            newline_fflush(relativefile[Isc]);
         }
      }
      First = 0;
   }
   for (Isc = 0; Isc < Nsc; Isc++) {
      struct SCType *S = &SC[Isc];
      if (S->Exists) {
         struct OrbitType *O = &Orb[S->RefOrb];
         vec3_t wxr, posr, velr;
         wxr  = VxV(O->wln, S->PosR);
         velr = MxV(O->CLN, S->VelR);
         posr = MxV(O->CLN, wxr);
         velr = VSubV_Elem(velr, posr);
         posr = MxV(O->CLN, S->PosR);
         file_print(relativefile[Isc], posr);
         file_print(relativefile[Isc], velr);
         newline_fflush(relativefile[Isc]);
      }
   }
}
/*********************************************************************/
void DSM_PlanetEphemReport(void)
{
   static FILE **ephemfile;
   static FILE **suntrackfile;
   static long First = 1;
   long Iw;
   char s[50];
   vec3_t svh, svw;
   mat3x3_t CWH;
   double Lat, Lng;

   if (First) {
      static char ephem_dir[BUFSIZE] = {'\0'};
      strcat(ephem_dir, OutPath);
      strcat(ephem_dir, "/ephem/");
      mkdir(ephem_dir, 0777);
      ephemfile    = (FILE **)calloc(NWORLD, sizeof(FILE *));
      suntrackfile = (FILE **)calloc(NWORLD, sizeof(FILE *));
      for (Iw = 0; Iw < NWORLD; Iw++) {
         if (World[Iw].Exists) {
            sprintf(s, "ephem/DSM_ephem_%s.42", World[Iw].Name);
            ephemfile[Iw] = FileOpen(OutPath, s, "wt");
            file_print(ephemfile[Iw], "PosH_X PosH_Y PosH_Z ");
            file_print(ephemfile[Iw], "VelH_X VelH_Y VelH_Z ");
            newline_fflush(ephemfile[Iw]);

            sprintf(s, "ephem/DSM_suntrack_%s.42", World[Iw].Name);
            suntrackfile[Iw] = FileOpen(OutPath, s, "wt");
            file_print(suntrackfile[Iw], "Lat Lon ");
            newline_fflush(suntrackfile[Iw]);
         }
      }
      First = 0;
   }
   for (Iw = 0; Iw < NWORLD; Iw++) { // Skip Sun
      if (World[Iw].Exists) {
         file_print(ephemfile[Iw], World[Iw].PosH);
         file_print(ephemfile[Iw], World[Iw].VelH);

         if (Iw != 0) {
            svh = NegV_Elem(World[Iw].PosH);
            svh = UNITV(svh).v;
            CWH = MxM(World[Iw].CWN, World[Iw].CNH);
            svw = MxV(CWH, svh);

            Lng = atan2(svw.y, svw.x) * R2D;
            Lat = asin(svw.z) * R2D;
         }
         else {
            Lng = 0.0;
            Lat = 0.0;
         }
         file_print(suntrackfile[Iw], Lat);
         file_print(suntrackfile[Iw], Lng);

         newline_fflush(ephemfile[Iw]);
         newline_fflush(suntrackfile[Iw]);
      }
   }
}
/*********************************************************************/
void DSM_AC_InertialReport(void)
{
   static FILE **inertialfile;
   static long First = 1;
   long Isc;
   char s[40];

   if (First) {
      inertialfile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_AC_inertial_%02li.42", Isc);
            inertialfile[Isc] = FileOpen(OutPath, s, "wt");
            file_print(inertialfile[Isc], "PosN_X PosN_Y PosN_Z ");
            file_print(inertialfile[Isc], "VelN_X VelN_Y VelN_Z ");
            newline_fflush(inertialfile[Isc]);
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         file_print(inertialfile[Isc], SC[Isc].AC.PosN);
         file_print(inertialfile[Isc], SC[Isc].AC.VelN);
         newline_fflush(inertialfile[Isc]);
      }
   }
}
/*********************************************************************/
void DSM_StateRot3BodyReport(void)
{
   static FILE **staterotfile;
   static long First = 1;
   long Isc;
   char s[50];
   vec3_t posRot, velRot;
   struct LagrangeSystemType *LS;

   if (First) {
      staterotfile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            LS = &LagSys[Orb[SC[Isc].RefOrb].Sys];
            if (LS->Exists) {
               sprintf(s, "DSM_StateRot3Body_%02li.42", Isc);
               staterotfile[Isc] = FileOpen(OutPath, s, "wt");
               file_print(staterotfile[Isc], "PosR_X PosR_Y PosR_Z ");
               file_print(staterotfile[Isc], "VelR_X VelR_Y VelR_Z ");
               newline_fflush(staterotfile[Isc]);
            }
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         LS = &LagSys[Orb[SC[Isc].RefOrb].Sys];
         if (LS->Exists) {
            StateN2StateRnd(LS, World[LS->Body2].eph.PosN,
                            World[LS->Body2].eph.VelN, SC[Isc].PosN,
                            SC[Isc].VelN, &posRot, &velRot);

            file_print(staterotfile[Isc], posRot);
            file_print(staterotfile[Isc], velRot);
            newline_fflush(staterotfile[Isc]);
         }
      }
   }
}
/*********************************************************************/
void DSM_PosHReport(void)
{
   static FILE **poshfile;
   static long First = 1;
   long Isc;
   char s[50];
   mat3x3_t CNJ;
   vec3_t SC_ECI, SC_LEI, SC_LCI;

   if (First) {
      poshfile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "PosH_%02li.42", Isc);
            poshfile[Isc] = FileOpen(OutPath, s, "wt");
            file_print(poshfile[Isc], "TDB_TIME TT_TIME ");
            file_print(poshfile[Isc], "TDB_JD TT_JD ");
            file_print(poshfile[Isc], "Venus_HC_X Venus_HC_Y Venus_HC_Z ");
            file_print(poshfile[Isc], "Earth_HC_X Earth_HC_Y Earth_HC_Z ");
            file_print(poshfile[Isc], "LUNA_HC_X LUNA_HC_Y LUNA_HC_Z ");
            file_print(poshfile[Isc], "LUNA_EC_X LUNA_EC_Y LUNA_EC_Z ");
            file_print(poshfile[Isc], "Mars_HC_X Mars_HC_Y Mars_HC_Z ");
            file_print(poshfile[Isc],
                       "Jupiter_HC_X Jupiter_HC_Y Jupiter_HC_Z ");
            file_print(poshfile[Isc], "Saturn_HC_X Saturn_HC_Y Saturn_HC_Z ");
            file_print(poshfile[Isc], "SC_PosN_X SC_PosN_Y SC_PosN_Z ");
            file_print(poshfile[Isc], "SC_HC_X SC_HC_Y SC_HC_Z ");
            file_print(poshfile[Isc], "SC_ECI_X SC_ECI_Y SC_ECI_Z ");
            file_print(poshfile[Isc], "SC_LCI_X SC_LCI_Y SC_LCI_Z ");
            file_print(poshfile[Isc], "SC_LEI_X SC_LEI_Y SC_LEI_Z ");
            newline_fflush(poshfile[Isc]);
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         CNJ = GetWorldCNJ(JD_TDB_MJD, World[LUNA].ang_data);
         // LunaInertialFrame(JD_TDB_MJD, CNJ);
         if (Orb[SC[Isc].RefOrb].World == LUNA) {
            SC_LEI = SC[Isc].PosN;
            SC_LCI = MTxV(CNJ, SC_LEI);
            SC_ECI = VAddV_Elem(SC_LCI, World[LUNA].eph.PosN);
         }
         else if (Orb[SC[Isc].RefOrb].World == EARTH) {
            SC_ECI = SC[Isc].PosN;
            SC_LCI = VSubV_Elem(SC_ECI, World[LUNA].eph.PosN);
            SC_LEI = MxV(CNJ, SC_LCI);
         }
         else
            break;
         // TODO
         JDType jd_tdb_j2000 = Date2JD(TDB, J2000_EPOCH);
         JDType jd_tt_j2000  = Date2JD(TT, J2000_EPOCH);
         double tdbTime      = JDToTime(JD_TDB_MJD);

         file_print(poshfile[Isc], JDToDays(jd_tdb_j2000));
         file_print(poshfile[Isc], JDToDays(jd_tt_j2000));
         file_print(poshfile[Isc], tdbTime);
         file_print(poshfile[Isc], DynTime);

         file_print(poshfile[Isc], World[VENUS].PosH);
         file_print(poshfile[Isc], World[EARTH].PosH);
         file_print(poshfile[Isc], World[LUNA].PosH);
         file_print(poshfile[Isc], World[LUNA].eph.PosN);
         file_print(poshfile[Isc], World[MARS].PosH);
         file_print(poshfile[Isc], World[JUPITER].PosH);
         file_print(poshfile[Isc], World[SATURN].PosH);
         file_print(poshfile[Isc], SC[Isc].PosN);
         file_print(poshfile[Isc], SC[Isc].PosH);
         file_print(poshfile[Isc], SC_ECI);
         file_print(poshfile[Isc], SC_LCI);
         file_print(poshfile[Isc], SC_LEI);
         newline_fflush(poshfile[Isc]);
      }
   }
}
// TODO: REWORK TO BE CORRECT FOR ALL LS
/*********************************************************************/
void DSM_Rot3BodyReport(void)
{
   static FILE **rotfile;
   static long First = 1;
   long Isc;
   char s[50];
   vec3_t posRel, posRot, velRel, velRot, z_axis = VEC3_PZAXIS;
   mat3x3_t DCM;
   struct LagrangeSystemType *LS;
   double ang_rot = M_PI;

   if (First) {
      rotfile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            LS = &LagSys[EARTHMOON];
            if (LS->Exists) {
               sprintf(s, "DSM_Rot3Body_%02li.42", Isc);
               rotfile[Isc] = FileOpen(OutPath, s, "wt");
               file_print(rotfile[Isc], "PosR_X PosR_Y PosR_Z ");
               file_print(rotfile[Isc], "VelR_X VelR_Y VelR_Z ");
               newline_fflush(rotfile[Isc]);
            }
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         LS = &LagSys[EARTHMOON];
         if (LS->Exists) {
            if (Orb[SC[Isc].RefOrb].World == LUNA) {
               posRel = SC[Isc].PosN;
               velRel = SC[Isc].VelN;
            }
            else {
               posRel = VSubV_Elem(SC[Isc].PosN, World[LUNA].eph.PosN);
               velRel = VSubV_Elem(SC[Isc].VelN, World[LUNA].eph.VelN);
            }
            posRot = MxV(LS->CLN, posRel);
            velRot = MxV(LS->CLN, velRel);
            DCM    = SimpRot(z_axis, ang_rot);
            posRot = MxV(DCM, posRot);
            velRot = MxV(DCM, velRot);
            file_print(rotfile[Isc], posRot);
            file_print(rotfile[Isc], velRot);
            newline_fflush(rotfile[Isc]);
         }
      }
   }
}
/*********************************************************************/
void DSM_NAV_StateReport(void)
{
   static FILE **stateFile, **covFile, **timeFile;
   static long First = 1;
   long Isc;
   char s[40];

   struct DSMNavType *Nav;

   if (First) {
      stateFile = (FILE **)calloc(Nsc, sizeof(FILE *));
      covFile   = (FILE **)calloc(Nsc, sizeof(FILE *));
      timeFile  = (FILE **)calloc(Nsc, sizeof(FILE *));
      First     = 0;
   }
   for (Isc = 0; Isc < Nsc; Isc++) {
      Nav = &SC[Isc].DSM.DsmNav;
      if (SC[Isc].Exists && Nav->NavigationActive == TRUE &&
          Nav->reportConfigured == FALSE) {
         sprintf(s, "DSM_navstate_%02li.42", Isc);
         stateFile[Isc] = FileOpen(OutPath, s, "wt");
         sprintf(s, "DSM_navtime_%02li.42", Isc);
         timeFile[Isc] = FileOpen(OutPath, s, "wt");
         sprintf(s, "DSM_navcov_%02li.42", Isc);
         covFile[Isc] = FileOpen(OutPath, s, "wt");
         Nav          = &SC[Isc].DSM.DsmNav;
         FOR_STATES(state)
         {
            if (Nav->stateActive[state] == TRUE) {
               switch (state) {
                  // case TIME_STATE:
                  //    fprintf(file[Isc],"time ");
                  //    break;
                  case ROTMAT_STATE:
                     file_print(stateFile[Isc],
                                "CRB_00 CRB_01 CRB_02 CRB_10 CRB_11 "
                                "CRB_12 CRB_20 "
                                "CRB_21 CRB_22 ");
                     break;
                  case QUAT_STATE:
                     file_print(stateFile[Isc], "qbr_x qbr_z qbr_z qbr_s ");
                     break;
                  case OMEGA_STATE:
                     file_print(stateFile[Isc], "wbr_x wbr_z wbr_z ");
                     break;
                  case POS_STATE:
                     file_print(stateFile[Isc], "PosN_x PosN_y PosN_z ");
                     break;
                  case VEL_STATE:
                     file_print(stateFile[Isc], "VelN_x VelN_y VelN_z ");
                     break;
                  default:
                     break;
               }
            }
         }
         newline_fflush(stateFile[Isc]);
         FOR_STATES(state)
         {
            if (Nav->stateActive[state] == TRUE) {
               switch (state) {
                  // case TIME_STATE:
                  //    fprintf(file[Isc],"time ");
                  //    break;
                  case ROTMAT_STATE:
                  case QUAT_STATE:
                     file_print(covFile[Isc], "s_theta_x s_theta_y s_theta_z ");
                     break;
                  case OMEGA_STATE:
                     file_print(covFile[Isc], "s_wbr_x s_wbr_z s_wbr_z ");
                     break;
                  case POS_STATE:
                     file_print(covFile[Isc], "s_PosN_x s_PosN_y s_PosN_z ");
                     break;
                  case VEL_STATE:
                     file_print(covFile[Isc], "s_VelN_x s_VelN_y s_VelN_z ");
                     break;
                  default:
                     break;
               }
            }
         }
         file_print(timeFile[Isc], "time");
         newline_fflush(covFile[Isc]);
         newline_fflush(timeFile[Isc]);
         Nav->reportConfigured = TRUE;
      }
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists && SC[Isc].DSM.DsmNav.NavigationActive == TRUE) {
         long writeTime = FALSE;
         Nav            = &SC[Isc].DSM.DsmNav;
         FOR_STATES(state)
         {
            if (Nav->stateActive[state] == TRUE) {
               writeTime = TRUE;
               switch (state) {
                  case TIME_STATE:
                     file_print(stateFile[Isc], Date2Time(Nav->Date));
                     break;
                  case ROTMAT_STATE:
                     file_print(stateFile[Isc], Nav->CRB);
                     break;
                  case QUAT_STATE:
                     file_print(stateFile[Isc], Nav->qbr);
                     break;
                  case OMEGA_STATE:
                     file_print(stateFile[Isc], Nav->wbr);
                     break;
                  case POS_STATE: {
                     vec3_t tmpV = VAddV_Elem(Nav->PosR, Nav->refPos);
                     tmpV        = MTxV(Nav->refCRN, tmpV);
                     file_print(stateFile[Isc], tmpV);
                  } break;
                  case VEL_STATE: {
                     vec3_t tmpV = VAddV_Elem(Nav->VelR, Nav->refVel);
                     tmpV        = MTxV(Nav->refCRN, tmpV);
                     file_print(stateFile[Isc], tmpV);
                  } break;
                  default:
                     break;
               }
            }
         }
         newline_fflush(stateFile[Isc]);

         if (writeTime) {
            file_print(timeFile[Isc], DynTime);
            newline_fflush(timeFile[Isc]);
         }

         const long navDim = Nav->navDim;
         double m[navDim];
         UnscentedStateTForm(Nav, m, Nav->P);

         FOR_STATES(state)
         {
            int stateInd = Nav->navInd[state];
            if (Nav->stateActive[state] == TRUE)
               for (int i = 0; i < Nav->navSize[state]; i++)
                  file_print(covFile[Isc],
                             sqrt(Nav->P[stateInd + i][stateInd + i]));
         }
         newline_fflush(covFile[Isc]);
      }
   }
}
/*********************************************************************/
// Last time I tried to analyze all the data from this, I ran out of
// memory...
#ifdef REPORT_RESIDUALS
void DSM_NAV_ResidualsReport(const double time, const long Isc, long *First,
                             double **residuals[FIN_SENSOR + 1])
{
   static FILE **residualFile;
   static long configure_files = TRUE;
   char s[40];

   struct DSMNavType *Nav = &SC[Isc].DSM.DsmNav;

   if (configure_files) {
      residualFile    = (FILE **)calloc(Nsc, sizeof(FILE *));
      configure_files = FALSE;
   }

   if (*First) {
      sprintf(s, "DSM_residuals_%02li.42", Isc);
      residualFile[Isc] = FileOpen(OutPath, s, "wt");
      Nav               = &SC[Isc].DSM.DsmNav;
      FILE *file        = residualFile[Isc];
      fprintf(file, "CCSDS_Time ; ");
      FOR_SENSORS(sensor)
      {
         for (int i = 0; i < Nav->nSensor[sensor]; i++) {
            if (Nav->sensorActive[sensor][i] == TRUE) {
               switch (sensor) {
                  case GPS_SENSOR:
                     fprintf(file,
                             "GPS[%02i]_Pos_x ; GPS[%02i]_Pos_y ; "
                             "GPS[%02i]_Pos_z ; GPS[%02i]_Vel_x ; "
                             "GPS[%02i]_Vel_y ; GPS[%02i]_Vel_z ; ",
                             i, i, i, i, i, i);
                     break;
                  case STARTRACK_SENSOR:
                     fprintf(file,
                             "STARTRACK[%02i]_Theta_x ; "
                             "STARTRACK[%02i]_Theta_y ; "
                             "STARTRACK[%02i]_Theta_z ; ",
                             i, i, i);
                     break;
                  case FSS_SENSOR: {
                     const struct FssType *fss = &SC[Isc].FSS[i];
                     switch (fss->type) {
                        case CONVENTIONAL_FSS:
                           fprintf(file,
                                   "FSS[%02i]_Theta_h ; "
                                   "FSS[%02i]_Theta_v ; ",
                                   i, i);
                           break;
                        case GS_FSS:
                           fprintf(file, "FSS[%02i]_Phi ; FSS[%02i]_Theta ; ",
                                   i, i);
                           break;
                     }
                     break;
                  }
                  case CSS_SENSOR:
                     fprintf(file, "CSS[%02i]_Out ; ", i);
                     break;
                  case GYRO_SENSOR:
                     fprintf(file, "GYRO[%02i]_Out ; ", i);
                     break;
                  case MAG_SENSOR:
                     fprintf(file, "MAG[%02i]_Out ; ", i);
                     break;
                  case ACCEL_SENSOR:
                     fprintf(file, "ACCEL[%02i]_Out ; ", i);
                     break;
                  default:
                     ek_exception(
                         EK_THROW,
                         "INIT_SENSOR and/or FIN_SENSOR are not configured "
                         "correctly in navkit.h. Exiting...\n");
                     break;
               }
            }
         }
      }
      newline_fflush(file);
      *First = FALSE;
   }
   Nav        = &SC[Isc].DSM.DsmNav;
   FILE *file = residualFile[Isc];
   fprintf(file, PRNT_DBL " ; ", time);
   FOR_SENSORS(sensor)
   {
      for (int i = 0; i < Nav->nSensor[sensor]; i++) {
         if (Nav->sensorActive[sensor][i] == TRUE) {
            if (residuals[sensor][i] != NULL) {
               switch (sensor) {
                  case GPS_SENSOR:
                     for (int j = 0; j < 6; j++)
                        fprintf(file, PRNT_DBL, residuals[sensor][i][j]);
                     break;
                  case STARTRACK_SENSOR:
                     for (int j = 0; j < 3; j++)
                        fprintf(file, PRNT_DBL, residuals[sensor][i][j]);
                     break;
                  case FSS_SENSOR:
                     for (int j = 0; j < 2; j++)
                        fprintf(file, PRNT_DBL, residuals[sensor][i][j]);
                     break;
                  case CSS_SENSOR:
                  case GYRO_SENSOR:
                  case MAG_SENSOR:
                  case ACCEL_SENSOR:
                     fprintf(file, PRNT_DBL, residuals[sensor][i][0]);
                     break;
                  default:
                     printf("INIT_SENSOR and/or FIN_SENSOR are not configured "
                            "correctly in navkit.h. Exiting...\n");
                     exit(EXIT_FAILURE);
                     break;
               }
            }
            else {
               switch (sensor) {
                  case GPS_SENSOR:
                     for (int j = 0; j < 6; j++)
                        fprintf(file, "nan ");
                     break;
                  case STARTRACK_SENSOR:
                     for (int j = 0; j < 3; j++)
                        fprintf(file, "nan ");
                     break;
                  case FSS_SENSOR:
                     for (int j = 0; j < 2; j++)
                        fprintf(file, "nan ");
                     break;
                  case CSS_SENSOR:
                  case GYRO_SENSOR:
                  case MAG_SENSOR:
                  case ACCEL_SENSOR:
                     fprintf(file, "nan ");
                     break;
                  default:
                     printf("INIT_SENSOR and/or FIN_SENSOR are not configured "
                            "correctly in navkit.c. Exiting...\n");
                     exit(EXIT_FAILURE);
                     break;
               }
            }
            fprintf(file, "; ");
         }
      }
   }
   newline_fflush(file);
}
#endif
/*********************************************************************/
void DSM_ATT_ControlReport(void)
{
   static FILE **attcontrolfile;
   static long First = 1;
   long Isc;
   char s[40];

   if (First) {
      attcontrolfile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_attcontrol_%02li.42", Isc);
            attcontrolfile[Isc] = FileOpen(OutPath, s, "wt");
            file_print(attcontrolfile[Isc], "therr_X therr_Y therr_Z ");
            file_print(attcontrolfile[Isc], "werr_X werr_Y werr_Z ");
            file_print(attcontrolfile[Isc], "Trq_X Trq_Y Trq_Z ");
            file_print(attcontrolfile[Isc],
                       "Dump_Trq_X Dump_Trq_Y Dump_Trq_Z ");
            file_print(attcontrolfile[Isc], "Mcmd_X Mcmd_Y Mcmd_Z ");
            newline_fflush(attcontrolfile[Isc]);
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         file_print(attcontrolfile[Isc], SC[Isc].DSM.therr);
         file_print(attcontrolfile[Isc], SC[Isc].DSM.werr);
         file_print(attcontrolfile[Isc], SC[Isc].DSM.Tcmd);
         file_print(attcontrolfile[Isc], SC[Isc].DSM.dTcmd);
         file_print(attcontrolfile[Isc], SC[Isc].DSM.Mcmd);
         newline_fflush(attcontrolfile[Isc]);
      }
   }
}
/*********************************************************************/
void DSM_POS_ControlReport(void)
{
   static FILE **poscontrolfile;
   static long First = 1;
   long Isc;
   char s[40];

   if (First) {
      poscontrolfile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_poscontrol_%02li.42", Isc);
            poscontrolfile[Isc] = FileOpen(OutPath, s, "wt");
            file_print(poscontrolfile[Isc], "perr_X perr_Y perr_Z ");
            file_print(poscontrolfile[Isc], "verr_X verr_Y verr_Z ");
            file_print(poscontrolfile[Isc], "FcmdN_X FcmdN_Y FcmdN_Z ");
            file_print(poscontrolfile[Isc], "FcmdB_X FcmdB_Y FcmdB_Z ");
            newline_fflush(poscontrolfile[Isc]);
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         file_print(poscontrolfile[Isc], SC[Isc].DSM.perr);
         file_print(poscontrolfile[Isc], SC[Isc].DSM.verr);
         file_print(poscontrolfile[Isc], SC[Isc].DSM.FcmdN);
         file_print(poscontrolfile[Isc], SC[Isc].DSM.FcmdB);
         newline_fflush(poscontrolfile[Isc]);
      }
   }
}
/*********************************************************************/
void DSM_EphemReport(void)
{
   static FILE **ephemfile;
   static long First = 1;
   long Isc;
   char s[40];

   double orb_beta, orb_inc, orb_AOP, orb_RAAN, orb_anom, orb_time;

   if (First) {
      ephemfile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_ephem_%02li.42", Isc);
            ephemfile[Isc] = FileOpen(OutPath, s, "wt");
            file_print(ephemfile[Isc], "Beta_(rad) ");
            file_print(ephemfile[Isc], "INC_(rad) ");
            file_print(ephemfile[Isc], "AOP_(rad) ");
            file_print(ephemfile[Isc], "RAAN_(rad) ");
            file_print(ephemfile[Isc], "TA_(rad) ");
            file_print(ephemfile[Isc], "SMA_(m) ");
            file_print(ephemfile[Isc], "ECC ");
            file_print(ephemfile[Isc], "PERIOD_(s)");
            newline_fflush(ephemfile[Isc]);
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         orb_beta = SolarBeta(SC[Isc].svn, SC[Isc].PosN, SC[Isc].VelN);
         orb_inc  = WrapTo2Pi(Orb[SC[Isc].RefOrb].inc);
         orb_AOP  = WrapTo2Pi(Orb[SC[Isc].RefOrb].ArgP);
         orb_RAAN = WrapTo2Pi(Orb[SC[Isc].RefOrb].RAAN);
         orb_anom = WrapTo2Pi(Orb[SC[Isc].RefOrb].anom);
         orb_time = TwoPi / sqrt(Orb[SC[Isc].RefOrb].mu /
                                 (pow(Orb[SC[Isc].RefOrb].SMA, 3)));
         file_print(ephemfile[Isc], orb_beta);
         file_print(ephemfile[Isc], orb_inc);
         file_print(ephemfile[Isc], orb_AOP);
         file_print(ephemfile[Isc], orb_RAAN);
         file_print(ephemfile[Isc], orb_anom);
         file_print(ephemfile[Isc], Orb[SC[Isc].RefOrb].SMA);
         file_print(ephemfile[Isc], Orb[SC[Isc].RefOrb].ecc);
         file_print(ephemfile[Isc], orb_time);
         newline_fflush(ephemfile[Isc]);
      }
   }
}
/*********************************************************************/
void DSM_WHLReport(void)
{
   static FILE **WHLFile;
   static long First = 1;
   long Isc;
   long i;
   char s[40];

   if (First) {
      WHLFile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_WHL_H_%02li.42", Isc);
            WHLFile[Isc] = FileOpen(OutPath, s, "wt");
            if (SC[Isc].Nw > 0) {
               for (i = 0; i < SC[Isc].Nw; i++) {
                  char whl_str[50] = {'\0'};
                  sprintf(whl_str, "WHL_%ld", i);
                  file_print(WHLFile[Isc], whl_str);
               }
               newline_fflush(WHLFile[Isc]);
            }
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         if (SC[Isc].Nw > 0) {
            for (i = 0; i < SC[Isc].Nw; i++)
               file_print(WHLFile[Isc], SC[Isc].AC.Whl[i].H);
            newline_fflush(WHLFile[Isc]);
         }
      }
   }
}
/*********************************************************************/
void DSM_THRReport(void)
{
   static FILE **THRFile;
   static long First = 1;
   long Isc;
   long i;
   char s[40];

   if (First) {
      THRFile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_THR_%02li.42", Isc);
            THRFile[Isc] = FileOpen(OutPath, s, "wt");
            if (SC[Isc].Nthr > 0) {
               for (i = 0; i < SC[Isc].Nthr; i++) {
                  char whl_str[50] = {'\0'};
                  sprintf(whl_str, "THR_%ld ", i);
                  file_print(THRFile[Isc], whl_str);
               }
               newline_fflush(THRFile[Isc]);
            }
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         if (SC[Isc].Nthr > 0) {
            for (i = 0; i < SC[Isc].Nthr; i++)
               file_print(THRFile[Isc], SC[Isc].AC.Thr[i].PulseWidthCmd);
            newline_fflush(THRFile[Isc]);
         }
      }
   }
}
/*********************************************************************/
void DSM_SVBReport(void)
{
   static FILE **SVBFile;
   static long First = 1;
   long Isc;
   char s[40];

   if (First) {
      SVBFile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_SVB_%02li.42", Isc);
            SVBFile[Isc] = FileOpen(OutPath, s, "wt");
            file_print(SVBFile[Isc], "SVB_X SVB_Y SVB_Z ");
            newline_fflush(SVBFile[Isc]);
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         file_print(SVBFile[Isc], SC[Isc].svb);
         newline_fflush(SVBFile[Isc]);
      }
   }
}
/*********************************************************************/
void DSM_GroundTrackReport(void)
{
   static FILE **gtrackfile;
   static long First = 1;
   long Isc;
   char s[40];
   struct WorldType *W;
   struct SCType *S;
   double Lat, Lng, junk;

   if (First) {
      gtrackfile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_groundtrack_%02li.42", Isc);
            gtrackfile[Isc] = FileOpen(OutPath, s, "wt");
            file_print(gtrackfile[Isc], "Lat Lon ");
            newline_fflush(gtrackfile[Isc]);
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &SC[Isc];
      if (SC[Isc].Exists) {
         W = &World[Orb[S->RefOrb].World];

         SpicePosN2RLngLat(W->CWN, SC[Isc].PosN, &junk, &Lng, &Lat);
         file_print(gtrackfile[Isc], Lat * R2D);
         file_print(gtrackfile[Isc], Lng * R2D);
      }
      newline_fflush(gtrackfile[Isc]);
   }
}
/*********************************************************************/
void OrbPropReport(void)
{
   static FILE *FixedFile;
   static FILE *EnckeFile;
   static FILE *CowellFile;
   static FILE *EulHillFile;
   static long First = 1;

   if (First) {
      First       = 0;
      FixedFile   = FileOpen(OutPath, "PosVelNfixed.42", "w");
      EnckeFile   = FileOpen(OutPath, "PosVelNencke.42", "w");
      CowellFile  = FileOpen(OutPath, "PosVelNcowell.42", "w");
      EulHillFile = FileOpen(OutPath, "PosVelNeulhill.42", "w");
   }

   if (OutFlag) {
      file_print(FixedFile, SC[0].PosN);
      file_print(FixedFile, SC[0].VelN);
      file_print(EnckeFile, SC[1].PosN);
      file_print(EnckeFile, SC[1].VelN);
      file_print(CowellFile, SC[2].PosN);
      file_print(CowellFile, SC[2].VelN);
      file_print(EulHillFile, SC[3].PosN);
      file_print(EulHillFile, SC[3].VelN);
      newline_fflush(FixedFile);
      newline_fflush(EnckeFile);
      newline_fflush(CowellFile);
      newline_fflush(EulHillFile);
   }
}
/*********************************************************************/
void GmatReport(void)
{
   static FILE *outfile;
   static long First = 1;
   long i;

   if (First) {
      First   = 0;
      outfile = FileOpen(OutPath, "PosN9sc.42", "w");
   }

   if (OutFlag) {
      for (i = 0; i < 9; i++)
         file_print(outfile, SC[i].PosN);
      newline_fflush(outfile);
   }
}
/*********************************************************************/
void PerturbReport(void)
{
   static FILE *perturbfile;
   static long First = 1;

   if (First) {
      perturbfile = FileOpen(OutPath, "perturb.42", "wt");
      file_print(perturbfile, "gravTrqB_X gravTrqB_Y gravTrqB_Z ");
      file_print(perturbfile, "gravTrqN_X gravTrqN_Y gravTrqN_Z ");
      file_print(perturbfile, "srpTrqB_X srpTrqB_Y srpTrqB_Z ");
      file_print(perturbfile, "srpTrqN_X srpTrqN_Y srpTrqN_Z ");
      file_print(perturbfile, "aeroTrqB_X aeroTrqB_Y aeroTrqB_Z ");
      file_print(perturbfile, "aeroTrqN_X aeroTrqN_Y aeroTrqN_Z ");
      file_print(perturbfile, "srpFrcB_X srpFrcB_Y srpFrcB_Z ");
      file_print(perturbfile, "srpFrcN_X srpFrcN_Y srpFrcN_Z ");
      file_print(perturbfile, "aeroFrcB_X aeroFrcB_Y aeroFrcB_Z ");
      file_print(perturbfile, "aeroFrcN_X aeroFrcN_Y aeroFrcN_Z ");
      newline_fflush(perturbfile);
      First = 0;
   }

   file_print(perturbfile, SC[0].gravTrqB);
   file_print(perturbfile, SC[0].gravTrqN);
   file_print(perturbfile, SC[0].srpTrqB);
   file_print(perturbfile, SC[0].srpTrqN);
   file_print(perturbfile, SC[0].aeroTrqB);
   file_print(perturbfile, SC[0].aeroTrqN);
   file_print(perturbfile, SC[0].srpFrcB);
   file_print(perturbfile, SC[0].srpFrcN);
   file_print(perturbfile, SC[0].aeroFrcB);
   file_print(perturbfile, SC[0].aeroFrcN);

   newline_fflush(perturbfile);
}
/*********************************************************************/
void NESC_Report()
{
   static FILE *nescfile;
   static long First = 1;
   if (First) {
      First                   = 0;
      nescfile                = FileOpen(OutPath, "NESC_data_file.csv", "wt");
      const char *headers[30] = {"time",
                                 "gePosition_m_X",
                                 "gePosition_m_Y",
                                 "gePosition_m_Z",
                                 "eiPosition_m_X",
                                 "eiPosition_m_Y",
                                 "eiPosition_m_Z",
                                 "eiVelocity_m_s_X",
                                 "eiVelocity_m_s_Y",
                                 "eiVelocity_m_s_Z",
                                 "eiAccel_m_s2_X",
                                 "eiAccel_m_s2_Y",
                                 "eiAccel_m_s2_Z",
                                 "semiMajorAxis_m",
                                 "gast_rad",
                                 "eulerAngle_rad_Roll",
                                 "eulerAngle_rad_Pitch",
                                 "eulerAngle_rad_Yaw",
                                 "eulerAngleWrtEi_rad_Roll",
                                 "eulerAngleWrtEi_rad_Pitch",
                                 "eulerAngleWrtEi_rad_Yaw",
                                 "bodyAngularRateWrtEi_rad_s_Roll",
                                 "bodyAngularRateWrtEi_rad_s_Pitch",
                                 "bodyAngularRateWrtEi_rad_s_Yaw",
                                 "altitudeMsl_m",
                                 "airDensity_kg_m3",
                                 "ambientTemperature_dgK",
                                 "eiGravitation_m_s2_X",
                                 "eiGravitation_m_s2_Y",
                                 "eiGravitation_m_s2_Z"};
      for (int i = 0; i < 30; i++)
         csv_print(nescfile, headers[i]);
      newline_fflush(nescfile);
   }

   struct SCType *S    = &SC[0];
   struct OrbitType *O = &Orb[S->RefOrb];
   struct WorldType *W = &World[O->World];

   vec3_t PosW, PosN, VelN, ang_ei, ang_lvlh;

   PosW = MxV(W->CWN, S->PosN);
   VelN = S->VelN;
   PosN = S->PosN;

   vec3_t wln;
   mat3x3_t CLN;
   FindCLN(S->PosN, S->VelN, &CLN, &wln);

   mat3x3_t CBN = S->B[0].CN;
   mat3x3_t CBL = MxMT(CBN, CLN);
   ang_lvlh     = C2A(321, CBL);
   ang_ei       = C2A(321, CBN);

   vec3_t rpy_lvlh = {.x = ang_lvlh.z, .y = ang_lvlh.y, .z = ang_lvlh.x};
   vec3_t rpy_ei   = {.x = ang_ei.z, .y = ang_ei.y, .z = ang_ei.x};

   vec3_t gravAccN = VAddV_Elem(S->gravPriAccN, S->gravPertAccN);
   // vec3_t gravAccN = S->gravPriAccN;
   vec3_t accN = VAddV_Elem(S->gravPriAccN, SxV(1.0 / S->mass, S->FrcN));

   double SMA, ecc, inc, RAAN, ArgP, anom, tp, SLR, alpha, rmin, MeanMotion,
       Period;
   RV2Eph(DynTime, O->mu, S->PosN, S->VelN, &SMA, &ecc, &inc, &RAAN, &ArgP,
          &anom, &tp, &SLR, &alpha, &rmin, &MeanMotion, &Period);

   DateType date_tt = JDToDate(JD_TT_MJD, TT_TIME);

   vec3_t lla     = ECEFToWGS84(PosW);
   double density = NRLMSISE00(date_tt, PosW, Flux10p7, GeomagIndex);

   csv_print(nescfile, SimTime);                    // time
   csv_print(nescfile, PosW);                       // gePosition_m
   csv_print(nescfile, PosN);                       // eiPosition_m
   csv_print(nescfile, VelN);                       // eiVelocity_m_s
   csv_print(nescfile, accN);                       // eiAccel_m_s2
   csv_print(nescfile, SMA);                        // semiMajorAxis__m
   csv_print(nescfile, JD2GMST(JD_TT_MJD) * TWOPI); // gast_rad (????????????)
   csv_print(nescfile, rpy_lvlh);                   // eulerAngle_rad
   csv_print(nescfile, rpy_ei);                     // eulerAngleWrtEi_rad
   csv_print(nescfile, S->B[0].wn); // bodyAngularRateWrtEi_rad_s (??)
   csv_print(nescfile, lla.z);      // altitudeMsl_m
   csv_print(nescfile, density);    // airDensity_kg_m3
   csv_print(nescfile, 0);          // ambientTemperature_dgK
   csv_print(nescfile, gravAccN);   // eiGravitation_m_s2
   newline_fflush(nescfile);
}
/*********************************************************************/
void Report(void)
{
   static FILE *timefile, *DynTimeFile, *UtcDateFile;
   static FILE **xfile, **ufile, **xffile, **uffile;
   static FILE **ConstraintFile;
   static FILE *PosNfile, *VelNfile, *qbnfile, *wbnfile;
   static FILE *PosWfile, *VelWfile;
   static FILE *PosRfile, *VelRfile;
   static FILE *bvnfile, *bvbfile;
   static FILE *Hvnfile, *KEfile;
   static FILE *Hvbfile;
   static FILE *svnfile, *svbfile;
   static FILE *RPYfile;
   static FILE *Hwhlfile;
   static FILE *MTBfile;
   static FILE *Thrfile;
   static FILE *AlbedoFile;
   static FILE *IllumFile;
   // static FILE *ProjAreaFile;
   static FILE *AccFile;
   static FILE *GpsFile;
   // static FILE *Kepfile;
   // static FILE *EHfile;
   static char First = TRUE;
   long Isc, i;
   struct DynType *D;
   mat3x3_t CBR, CRN, CRL = MAT3X3_EYE;
   struct WorldType *W;
   vec3_t WorldAngVel, wxR, VelN;
   vec3_t PosW, VelW, PosR, VelR;
   // double
   // SMA,ecc,inc,RAAN,ArgP,anom,tp,SLR,alpha,rmin,MeanMotion,Period;
   char s[40];

   if (First) {
      First       = FALSE;
      timefile    = FileOpen(OutPath, "time.42", "w");
      DynTimeFile = FileOpen(OutPath, "DynTime.42", "w");
      UtcDateFile = FileOpen(OutPath, "UTC.42", "w");

      ufile          = (FILE **)calloc(Nsc, sizeof(FILE *));
      xfile          = (FILE **)calloc(Nsc, sizeof(FILE *));
      uffile         = (FILE **)calloc(Nsc, sizeof(FILE *));
      xffile         = (FILE **)calloc(Nsc, sizeof(FILE *));
      ConstraintFile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "u%02ld.42", Isc);
            ufile[Isc] = FileOpen(OutPath, s, "w");
            sprintf(s, "x%02ld.42", Isc);
            xfile[Isc] = FileOpen(OutPath, s, "w");
            if (SC[Isc].FlexActive) {
               sprintf(s, "uf%02ld.42", Isc);
               uffile[Isc] = FileOpen(OutPath, s, "w");
               sprintf(s, "xf%02ld.42", Isc);
               xffile[Isc] = FileOpen(OutPath, s, "w");
            }
            if (SC[Isc].ConstraintsRequested) {
               sprintf(s, "Constraint%02ld.42", Isc);
               ConstraintFile[Isc] = FileOpen(OutPath, s, "w");
            }
         }
      }

      PosNfile = FileOpen(OutPath, "PosN.42", "w");
      VelNfile = FileOpen(OutPath, "VelN.42", "w");
      PosWfile = FileOpen(OutPath, "PosW.42", "w");
      VelWfile = FileOpen(OutPath, "VelW.42", "w");
      PosRfile = FileOpen(OutPath, "PosR.42", "w");
      VelRfile = FileOpen(OutPath, "VelR.42", "w");
      qbnfile  = FileOpen(OutPath, "qbn.42", "w");
      wbnfile  = FileOpen(OutPath, "wbn.42", "w");
      bvnfile  = FileOpen(OutPath, "bvn.42", "w");
      bvbfile  = FileOpen(OutPath, "bvb.42", "w");
      Hvnfile  = FileOpen(OutPath, "Hvn.42", "w");
      Hvbfile  = FileOpen(OutPath, "Hvb.42", "w");
      svnfile  = FileOpen(OutPath, "svn.42", "w");
      svbfile  = FileOpen(OutPath, "svb.42", "w");
      KEfile   = FileOpen(OutPath, "KE.42", "w");
      // ProjAreaFile = FileOpen(OutPath,"ProjArea.42","w");
      RPYfile  = FileOpen(OutPath, "RPY.42", "w");
      Hwhlfile = FileOpen(OutPath, "Hwhl.42", "w");

      if (SC[0].Nmtb > 0) {
         MTBfile = FileOpen(OutPath, "MTB.42", "w");
      }

      if (SC[0].Nthr > 0) {
         Thrfile = FileOpen(OutPath, "Thr.42", "w");
      }

      if (SC[0].Nacc > 0) {
         AccFile = FileOpen(OutPath, "Acc.42", "w");
      }

      if (SC[0].Ncss > 0) {
         AlbedoFile = FileOpen(OutPath, "Albedo.42", "w");
         IllumFile  = FileOpen(OutPath, "Illum.42", "w");
      }
      if (SC[0].Ngps > 0) {
         GpsFile = FileOpen(OutPath, "Gps.42", "w");
      }
   }

   if (OutFlag) {
      file_print(timefile, SimTime);
      file_print(DynTimeFile, DynTime);
      newline_fflush(timefile);
      newline_fflush(DynTimeFile);
      file_print(UtcDateFile, UTC);
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            D = &SC[Isc].Dyn;
            for (i = 0; i < 3; i++)
               file_print(ufile[Isc], D->u[i]);
            newline_fflush(ufile[Isc]);
            for (i = 0; i < 3; i++)
               file_print(xfile[Isc], D->x[i]);
            newline_fflush(xfile[Isc]);
            if (SC[Isc].FlexActive) {
               for (i = 0; i < D->Nf; i++)
                  file_print(uffile[Isc], D->uf[i]);
               newline_fflush(uffile[Isc]);
               for (i = 0; i < D->Nf; i++)
                  file_print(xffile[Isc], D->xf[i]);
               newline_fflush(xffile[Isc]);
            }
            if (SC[Isc].ConstraintsRequested) {
               for (i = 0; i < D->Nc; i++)
                  file_print(ConstraintFile[Isc], D->GenConstraintFrc[i]);
               newline_fflush(ConstraintFile[Isc]);
            }
         }
      }
      if (SC[0].Exists) {
         file_print(PosNfile, SC[0].PosN);
         file_print(VelNfile, SC[0].VelN);
         newline_fflush(PosNfile);
         newline_fflush(VelNfile);
         W                = &World[Orb[SC[0].RefOrb].World];
         WorldAngVel.v[0] = 0.0;
         WorldAngVel.v[1] = 0.0;
         WorldAngVel.v[2] = GetWorldW(JD_TDB_MJD, W);

         wxR  = VxV(WorldAngVel, SC[0].PosN);
         VelN = VSubV_Elem(SC[0].VelN, wxR);
         PosW = MxV(W->CWN, SC[0].PosN);
         VelW = MxV(W->CWN, VelN);
         file_print(PosWfile, PosW);
         file_print(VelWfile, VelW);
         newline_fflush(PosWfile);
         newline_fflush(VelWfile);
         if (Orb[SC[0].RefOrb].Regime == ORB_FLIGHT) {
            PosR = MxV(Rgn[Orb[SC[0].RefOrb].Region].CN, SC[0].PosR);
            VelR = MxV(Rgn[Orb[SC[0].RefOrb].Region].CN, SC[0].VelR);
            file_print(PosRfile, PosR);
            file_print(VelRfile, VelR);
         }
         else {
            file_print(PosRfile, SC[0].PosR);
            file_print(VelRfile, SC[0].VelR);
         }
         newline_fflush(PosRfile);
         newline_fflush(PosRfile);
         file_print(qbnfile, SC[0].B[0].qn);
         file_print(wbnfile, SC[0].B[0].wn);
         file_print(bvnfile, SC[0].bvn);
         file_print(bvbfile, SC[0].bvb);
         file_print(Hvnfile, SC[0].Hvn);
         file_print(Hvbfile, SC[0].Hvb);
         file_print(svnfile, SC[0].svn);
         file_print(svbfile, SC[0].svb);
         file_print(KEfile, FindTotalKineticEnergy(Orb, &SC[0]));
         newline_fflush(qbnfile);
         newline_fflush(wbnfile);
         newline_fflush(bvnfile);
         newline_fflush(bvbfile);
         newline_fflush(Hvnfile);
         newline_fflush(Hvbfile);
         newline_fflush(svnfile);
         newline_fflush(svbfile);
         newline_fflush(KEfile);
         // fprintf(ProjAreaFile, PRNT_DBL PRNT_DBL"\n",
         //    FindTotalProjectedArea(&SC[0],ZAxis),
         //    FindTotalUnshadedProjectedArea(&SC[0],ZAxis));
         CRN            = MxM(CRL, SC[0].CLN);
         CBR            = MxMT(SC[0].B[0].CN, CRN);
         vec3_t eu_angs = C2A(123, CBR);
         file_print(RPYfile, SxV(R2D, eu_angs));
         newline_fflush(RPYfile);
         if (SC[0].Nw > 0) {
            for (i = 0; i < SC[0].Nw; i++) {
               file_print(Hwhlfile, SC[0].Whl[i].H);
            }
            newline_fflush(Hwhlfile);
         }
         if (SC[0].Nmtb > 0) {
            for (i = 0; i < SC[0].Nmtb; i++)
               file_print(MTBfile, SC[0].MTB[i].M);
            newline_fflush(MTBfile);
         }
         if (SC[0].Nthr > 0) {
            for (i = 0; i < SC[0].Nthr; i++)
               file_print(Thrfile, SC[0].Thr[i].F);
            newline_fflush(Thrfile);
         }
         if (SC[0].Nacc > 0) {
            for (i = 0; i < SC[0].Nacc; i++) {
               file_print(AccFile, SC[0].Accel[i].TrueAcc);
               file_print(AccFile, SC[0].Accel[i].MeasAcc);
            }
            newline_fflush(AccFile);
         }
         if (SC[0].Ngps > 0) {
            file_print(GpsFile, SC[0].GPS[0].PosN);
            newline_fflush(GpsFile);
         }
         if (SC[0].Ncss > 0) {
            for (i = 0; i < SC[0].Ncss; i++) {
               file_print(IllumFile, SC[0].CSS[i].Illum);
               file_print(AlbedoFile, SC[0].CSS[i].Albedo);
            }
            newline_fflush(IllumFile);
            newline_fflush(AlbedoFile);
         }

         // MagReport();
         // GyroReport();
         // OrbPropReport();
         // GmatReport();
         PerturbReport();

         if (SC[0].DSM.Init == 1) {
            // DSM_AC_AttitudeReport();

            DSM_AttitudeReport();
            // DSM_AC_InertialReport();
            DSM_InertialReport();
            DSM_RelativeReport();
            DSM_NAV_StateReport();
            // DSM_PlanetEphemReport();
            DSM_ATT_ControlReport();
            DSM_POS_ControlReport();
            DSM_EphemReport();
            DSM_WHLReport();
            DSM_THRReport();
            DSM_SVBReport();
            // DSM_GroundTrackReport();
            DSM_StateRot3BodyReport();
            // DSM_PosHReport();
            DSM_Rot3BodyReport();
         }
         NESC_Report();
      }
   }

   /* An example how to call specialized reporting based on sim case
    */
   /* if (!strcmp(OutPath,"./Potato/")) PotatoReport(); */

   if (CleanUpFlag) {
      fclose(timefile);
   }
}

/* #ifdef __cplusplus
** }
** #endif
*/

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

#define PRNT_DBL      "%18.36le "
#define PRNT_DBL_3VEC PRNT_DBL PRNT_DBL PRNT_DBL

/* #ifdef __cplusplus
** namespace _42 {
** using namespace Kit;
** #endif
*/

/*********************************************************************/
double FindTotalProjectedArea(struct SCType *S, vec3 VecN)
{
   struct BodyType *B;
   struct GeomType *G;
   struct PolyType *P;
   double ProjArea = 0.0, VoN;
   vec3 VecB;
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
double FindTotalUnshadedProjectedArea(struct SCType *S, vec3 VecN)
{
   struct BodyType *B;
   struct GeomType *G;
   struct PolyType *P;
   double ProjArea = 0.0, VoN;
   vec3 VecB;
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

   fprintf(magfile, PRNT_DBL_3VEC PRNT_DBL_3VEC PRNT_DBL_3VEC "\n",
           SC[0].bvb.v[0], SC[0].bvb.v[1], SC[0].bvb.v[2], SC[0].MAG[0].Field,
           SC[0].MAG[1].Field, SC[0].MAG[2].Field, SC[0].AC.bvb.v[0],
           SC[0].AC.bvb.v[1], SC[0].AC.bvb.v[2]);
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

   fprintf(gyrofile,
           PRNT_DBL_3VEC PRNT_DBL_3VEC PRNT_DBL_3VEC PRNT_DBL_3VEC
               PRNT_DBL_3VEC PRNT_DBL_3VEC "\n",
           SC[0].B[0].wn.v[0], SC[0].B[0].wn.v[1], SC[0].B[0].wn.v[2],
           SC[0].Gyro[0].TrueRate, SC[0].Gyro[1].TrueRate,
           SC[0].Gyro[2].TrueRate, SC[0].Gyro[0].Bias, SC[0].Gyro[1].Bias,
           SC[0].Gyro[2].Bias, SC[0].Gyro[0].Angle, SC[0].Gyro[1].Angle,
           SC[0].Gyro[2].Angle, SC[0].Gyro[0].MeasRate, SC[0].Gyro[1].MeasRate,
           SC[0].Gyro[2].MeasRate, SC[0].AC.wbn.v[0], SC[0].AC.wbn.v[1],
           SC[0].AC.wbn.v[2]);
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
            fprintf(attitudefile[Isc], "qbn_0 qbn_1 qbn_2 qbn_3 ");
            fprintf(attitudefile[Isc], "wbn_X wbn_Y wbn_Z ");
            fprintf(attitudefile[Isc], "\n");
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         fprintf(attitudefile[Isc], PRNT_DBL PRNT_DBL PRNT_DBL PRNT_DBL,
                 SC[Isc].B[0].qn.q[0], SC[Isc].B[0].qn.q[1],
                 SC[Isc].B[0].qn.q[2], SC[Isc].B[0].qn.q[3]);
         fprintf(attitudefile[Isc], PRNT_DBL_3VEC, SC[Isc].B[0].wn.v[0],
                 SC[Isc].B[0].wn.v[1], SC[Isc].B[0].wn.v[2]);
         fprintf(attitudefile[Isc], "\n");
      }
      fflush(attitudefile[Isc]);
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
            fprintf(attitudefile[Isc], "qbn_0 qbn_1 qbn_2 qbn_3 ");
            fprintf(attitudefile[Isc], "wbn_X wbn_Y wbn_Z ");
            fprintf(attitudefile[Isc], "\n");
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         fprintf(attitudefile[Isc], PRNT_DBL PRNT_DBL PRNT_DBL PRNT_DBL,
                 SC[Isc].AC.qbn.q[0], SC[Isc].AC.qbn.q[1], SC[Isc].AC.qbn.q[2],
                 SC[Isc].AC.qbn.q[3]);
         fprintf(attitudefile[Isc], PRNT_DBL_3VEC, SC[Isc].AC.wbn.v[0],
                 SC[Isc].AC.wbn.v[1], SC[Isc].AC.wbn.v[2]);
         fprintf(attitudefile[Isc], "\n");
      }
      fflush(attitudefile[Isc]);
   }
}
/*********************************************************************/
void DSM_InertialReport(void)
{
   static FILE **inertialfile;
   static long First = 1;
   long Isc;
   vec3 PosL;
   char s[40];

   if (First) {
      inertialfile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_inertial_%02li.42", Isc);
            inertialfile[Isc] = FileOpen(OutPath, s, "wt");
            fprintf(inertialfile[Isc], "PosN_X PosN_Y PosN_Z ");
            fprintf(inertialfile[Isc], "VelN_X VelN_Y VelN_Z ");
            fprintf(inertialfile[Isc], "PosL_X PosL_Y PosL_Z ");
            fprintf(inertialfile[Isc], "\n");
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         PosL = MxV(SC[0].CLN, SC[Isc].PosN);
         fprintf(inertialfile[Isc], PRNT_DBL_3VEC, SC[Isc].PosN.v[0],
                 SC[Isc].PosN.v[1], SC[Isc].PosN.v[2]);
         fprintf(inertialfile[Isc], PRNT_DBL_3VEC, SC[Isc].VelN.v[0],
                 SC[Isc].VelN.v[1], SC[Isc].VelN.v[2]);
         fprintf(inertialfile[Isc], PRNT_DBL_3VEC, PosL.v[0], PosL.v[1],
                 PosL.v[2]);
         fprintf(inertialfile[Isc], "\n");
      }
      fflush(inertialfile[Isc]);
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
            fprintf(relativefile[Isc], "PosR_X PosR_Y PosR_Z ");
            fprintf(relativefile[Isc], "VelR_X VelR_Y VelR_Z ");
            fprintf(relativefile[Isc], "\n");
         }
      }
      First = 0;
   }
   for (Isc = 0; Isc < Nsc; Isc++) {
      struct SCType *S = &SC[Isc];
      if (S->Exists) {
         struct OrbitType *O = &Orb[S->RefOrb];
         vec3 wxr, posr, velr;
         wxr  = VxV(O->wln, S->PosR);
         velr = MxV(O->CLN, S->VelR);
         posr = MxV(O->CLN, wxr);
         velr = VmVElem(velr, posr);
         posr = MxV(O->CLN, S->PosR);
         fprintf(relativefile[Isc], PRNT_DBL_3VEC, posr.v[0], posr.v[1],
                 posr.v[2]);
         fprintf(relativefile[Isc], PRNT_DBL_3VEC, velr.v[0], velr.v[1],
                 velr.v[2]);
         fprintf(relativefile[Isc], "\n");
      }
      fflush(relativefile[Isc]);
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
   vec3 svh, svw;
   mat3x3 CWH;
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
            fprintf(ephemfile[Iw], "PosH_X PosH_Y PosH_Z ");
            fprintf(ephemfile[Iw], "VelH_X VelH_Y VelH_Z ");
            fprintf(ephemfile[Iw], "\n");

            sprintf(s, "ephem/DSM_suntrack_%s.42", World[Iw].Name);
            suntrackfile[Iw] = FileOpen(OutPath, s, "wt");
            fprintf(suntrackfile[Iw], "Lat Lon ");
            fprintf(suntrackfile[Iw], "\n");
         }
      }
      First = 0;
   }
   for (Iw = 0; Iw < NWORLD; Iw++) { // Skip Sun
      if (World[Iw].Exists) {
         fprintf(ephemfile[Iw], PRNT_DBL_3VEC, World[Iw].PosH.v[0],
                 World[Iw].PosH.v[1], World[Iw].PosH.v[2]);
         fprintf(ephemfile[Iw], PRNT_DBL_3VEC, World[Iw].VelH.v[0],
                 World[Iw].VelH.v[1], World[Iw].VelH.v[2]);
         fprintf(ephemfile[Iw], "\n");

         if (Iw != 0) {
            svh = VNegElem(World[Iw].PosH);
            UNITV(&svh);
            CWH = MxM(World[Iw].CWN, World[Iw].CNH);
            svw = MxV(CWH, svh);

            Lng = atan2(svw.y, svw.x) * R2D;
            Lat = asin(svw.z) * R2D;
         }
         else {
            Lng = 0.0;
            Lat = 0.0;
         }

         fprintf(suntrackfile[Iw], PRNT_DBL PRNT_DBL, Lat, Lng);
         fprintf(suntrackfile[Iw], "\n");
      }
      fflush(ephemfile[Iw]);
      fflush(suntrackfile[Iw]);
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
            fprintf(inertialfile[Isc], "PosN_X PosN_Y PosN_Z ");
            fprintf(inertialfile[Isc], "VelN_X VelN_Y VelN_Z ");
            fprintf(inertialfile[Isc], "\n");
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         fprintf(inertialfile[Isc], PRNT_DBL_3VEC, SC[Isc].AC.PosN.v[0],
                 SC[Isc].AC.PosN.v[1], SC[Isc].AC.PosN.v[2]);
         fprintf(inertialfile[Isc], PRNT_DBL_3VEC, SC[Isc].AC.VelN.v[0],
                 SC[Isc].AC.VelN.v[1], SC[Isc].AC.VelN.v[2]);
         fprintf(inertialfile[Isc], "\n");
      }
      fflush(inertialfile[Isc]);
   }
}
/*********************************************************************/
void DSM_StateRot3BodyReport(void)
{
   static FILE **staterotfile;
   static long First = 1;
   long Isc;
   char s[50];
   vec3 posRot, velRot;
   struct LagrangeSystemType *LS;

   if (First) {
      staterotfile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            LS = &LagSys[Orb[SC[Isc].RefOrb].Sys];
            if (LS->Exists) {
               sprintf(s, "DSM_StateRot3Body_%02li.42", Isc);
               staterotfile[Isc] = FileOpen(OutPath, s, "wt");
               fprintf(staterotfile[Isc], "PosR_X PosR_Y PosR_Z ");
               fprintf(staterotfile[Isc], "VelR_X VelR_Y VelR_Z ");
               fprintf(staterotfile[Isc], "\n");
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

            fprintf(staterotfile[Isc], PRNT_DBL_3VEC, posRot.v[0], posRot.v[1],
                    posRot.v[2]);
            fprintf(staterotfile[Isc], PRNT_DBL_3VEC, velRot.v[0], velRot.v[1],
                    velRot.v[2]);
            fprintf(staterotfile[Isc], "\n");
         }
      }
      fflush(staterotfile[Isc]);
   }
}
/*********************************************************************/
void DSM_PosHReport(void)
{
   static FILE **poshfile;
   static long First = 1;
   long Isc;
   char s[50];
   mat3x3 CNJ;
   vec3 SC_ECI, SC_LEI, SC_LCI;

   if (First) {
      poshfile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "PosH_%02li.42", Isc);
            poshfile[Isc] = FileOpen(OutPath, s, "wt");
            fprintf(poshfile[Isc], "TDB_TIME TT_TIME ");
            fprintf(poshfile[Isc], "TDB_JD TT_JD ");
            fprintf(poshfile[Isc], "Venus_HC_X Venus_HC_Y Venus_HC_Z ");
            fprintf(poshfile[Isc], "Earth_HC_X Earth_HC_Y Earth_HC_Z ");
            fprintf(poshfile[Isc], "LUNA_HC_X LUNA_HC_Y LUNA_HC_Z ");
            fprintf(poshfile[Isc], "LUNA_EC_X LUNA_EC_Y LUNA_EC_Z ");
            fprintf(poshfile[Isc], "Mars_HC_X Mars_HC_Y Mars_HC_Z ");
            fprintf(poshfile[Isc], "Jupiter_HC_X Jupiter_HC_Y Jupiter_HC_Z ");
            fprintf(poshfile[Isc], "Saturn_HC_X Saturn_HC_Y Saturn_HC_Z ");
            fprintf(poshfile[Isc], "SC_PosN_X SC_PosN_Y SC_PosN_Z ");
            fprintf(poshfile[Isc], "SC_HC_X SC_HC_Y SC_HC_Z ");
            fprintf(poshfile[Isc], "SC_ECI_X SC_ECI_Y SC_ECI_Z ");
            fprintf(poshfile[Isc], "SC_LCI_X SC_LCI_Y SC_LCI_Z ");
            fprintf(poshfile[Isc], "SC_LEI_X SC_LEI_Y SC_LEI_Z ");
            fprintf(poshfile[Isc], "\n");
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
            SC_ECI = VpVElem(SC_LCI, World[LUNA].eph.PosN);
         }
         else if (Orb[SC[Isc].RefOrb].World == EARTH) {
            SC_ECI = SC[Isc].PosN;
            SC_LCI = VmVElem(SC_ECI, World[LUNA].eph.PosN);
            SC_LEI = MxV(CNJ, SC_LCI);
         }
         else
            break;
         // TODO
         JDType jd_tdb_j2000 = Date2JD(TDB, J2000_EPOCH);
         JDType jd_tt_j2000  = Date2JD(TT, J2000_EPOCH);
         double tdbTime      = JDToTime(JD_TDB_MJD);
         fprintf(poshfile[Isc], PRNT_DBL PRNT_DBL, JDToDays(jd_tdb_j2000),
                 JDToDays(jd_tt_j2000));
         fprintf(poshfile[Isc], PRNT_DBL PRNT_DBL, tdbTime, DynTime);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[VENUS].PosH.v[0],
                 World[VENUS].PosH.v[1], World[VENUS].PosH.v[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[EARTH].PosH.v[0],
                 World[EARTH].PosH.v[1], World[EARTH].PosH.v[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[LUNA].PosH.v[0],
                 World[LUNA].PosH.v[1], World[LUNA].PosH.v[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[LUNA].eph.PosN.v[0],
                 World[LUNA].eph.PosN.v[1], World[LUNA].eph.PosN.v[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[MARS].PosH.v[0],
                 World[MARS].PosH.v[1], World[MARS].PosH.v[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[JUPITER].PosH.v[0],
                 World[JUPITER].PosH.v[1], World[JUPITER].PosH.v[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[SATURN].PosH.v[0],
                 World[SATURN].PosH.v[1], World[SATURN].PosH.v[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, SC[0].PosN.v[0], SC[0].PosN.v[1],
                 SC[0].PosN.v[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, SC[0].PosH.v[0], SC[0].PosH.v[1],
                 SC[0].PosH.v[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, SC_ECI.x, SC_ECI.y, SC_ECI.z);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, SC_LCI.x, SC_LCI.y, SC_LCI.z);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, SC_LEI.x, SC_LEI.y, SC_LEI.z);
         fprintf(poshfile[Isc], "\n");
      }
      fflush(poshfile[Isc]);
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
   vec3 posRel, posRot, velRel, velRot, z_axis = VEC3_PZAXIS;
   mat3x3 DCM;
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
               fprintf(rotfile[Isc], "PosR_X PosR_Y PosR_Z ");
               fprintf(rotfile[Isc], "VelR_X VelR_Y VelR_Z ");
               fprintf(rotfile[Isc], "\n");
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
               posRel = VmVElem(SC[Isc].PosN, World[LUNA].eph.PosN);
               velRel = VmVElem(SC[Isc].VelN, World[LUNA].eph.VelN);
            }
            posRot = MxV(LS->CLN, posRel);
            velRot = MxV(LS->CLN, velRel);
            DCM    = SimpRot(z_axis, ang_rot);
            posRot = MxV(DCM, posRot);
            velRot = MxV(DCM, velRot);
            fprintf(rotfile[Isc], PRNT_DBL_3VEC, posRot.x, posRot.y, posRot.z);
            fprintf(rotfile[Isc], PRNT_DBL_3VEC, velRot.x, velRot.y, velRot.z);
            fprintf(rotfile[Isc], "\n");
         }
      }
      fflush(rotfile[Isc]);
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
                     fprintf(stateFile[Isc],
                             "CRB_00 CRB_01 CRB_02 CRB_10 CRB_11 CRB_12 CRB_20 "
                             "CRB_21 CRB_22 ");
                     break;
                  case QUAT_STATE:
                     fprintf(stateFile[Isc], "qbr_x qbr_z qbr_z qbr_s ");
                     break;
                  case OMEGA_STATE:
                     fprintf(stateFile[Isc], "wbr_x wbr_z wbr_z ");
                     break;
                  case POS_STATE:
                     fprintf(stateFile[Isc], "PosN_x PosN_y PosN_z ");
                     break;
                  case VEL_STATE:
                     fprintf(stateFile[Isc], "VelN_x VelN_y VelN_z ");
                     break;
                  default:
                     break;
               }
            }
         }
         fprintf(stateFile[Isc], "\n");
         FOR_STATES(state)
         {
            if (Nav->stateActive[state] == TRUE) {
               switch (state) {
                  // case TIME_STATE:
                  //    fprintf(file[Isc],"time ");
                  //    break;
                  case ROTMAT_STATE:
                  case QUAT_STATE:
                     fprintf(covFile[Isc], "s_theta_x s_theta_y s_theta_z ");
                     break;
                  case OMEGA_STATE:
                     fprintf(covFile[Isc], "s_wbr_x s_wbr_z s_wbr_z ");
                     break;
                  case POS_STATE:
                     fprintf(covFile[Isc], "s_PosN_x s_PosN_y s_PosN_z ");
                     break;
                  case VEL_STATE:
                     fprintf(covFile[Isc], "s_VelN_x s_VelN_y s_VelN_z ");
                     break;
                  default:
                     break;
               }
            }
         }
         fprintf(covFile[Isc], "\n");
         fprintf(timeFile[Isc], "time\n");
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
                     fprintf(stateFile[Isc], PRNT_DBL, Date2Time(Nav->Date));
                     break;
                  case ROTMAT_STATE:
                     for (int i = 0; i < 3; i++)
                        fprintf(stateFile[Isc], PRNT_DBL_3VEC,
                                Nav->CRB.rows[i].x, Nav->CRB.rows[i].y,
                                Nav->CRB.rows[i].z);
                     break;
                  case QUAT_STATE:
                     fprintf(stateFile[Isc],
                             PRNT_DBL PRNT_DBL PRNT_DBL PRNT_DBL, Nav->qbr.x,
                             Nav->qbr.y, Nav->qbr.z, Nav->qbr.s);
                     break;
                  case OMEGA_STATE:
                     fprintf(stateFile[Isc], PRNT_DBL_3VEC, Nav->wbr.x,
                             Nav->wbr.y, Nav->wbr.z);
                     break;
                  case POS_STATE: {
                     vec3 tmpV1, tmpV2;
                     tmpV1 = VpVElem(Nav->PosR, Nav->refPos);
                     tmpV2 = MTxV(Nav->refCRN, tmpV1);
                     fprintf(stateFile[Isc], PRNT_DBL_3VEC, tmpV2.x, tmpV2.y,
                             tmpV2.z);
                  } break;
                  case VEL_STATE: {
                     vec3 tmpV1, tmpV2;
                     tmpV1 = VpVElem(Nav->VelR, Nav->refVel);
                     tmpV2 = MTxV(Nav->refCRN, tmpV1);
                     fprintf(stateFile[Isc], PRNT_DBL_3VEC, tmpV2.x, tmpV2.y,
                             tmpV2.z);
                  } break;
                  default:
                     break;
               }
            }
         }
         fprintf(stateFile[Isc], "\n");
         fflush(stateFile[Isc]);

         if (writeTime) {
            fprintf(timeFile[Isc], PRNT_DBL "\n", DynTime);
            fflush(timeFile[Isc]);
         }

         const long navDim = Nav->navDim;
         double m[navDim];
         UnscentedStateTForm(Nav, m, Nav->P);

         FOR_STATES(state)
         {
            int stateInd = Nav->navInd[state];
            if (Nav->stateActive[state] == TRUE) {
               for (int i = 0; i < Nav->navSize[state]; i++) {
                  fprintf(covFile[Isc], PRNT_DBL,
                          sqrt(Nav->P[stateInd + i][stateInd + i]));
               }
            }
         }
         fprintf(covFile[Isc], "\n");
         fflush(covFile[Isc]);
      }
   }
}
/*********************************************************************/
// Last time I tried to analyze all the data from this, I ran out of memory...
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
                                   "FSS[%02i]_Theta_h ; FSS[%02i]_Theta_v ; ",
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
      fprintf(file, "\n");
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
   fprintf(file, "\n");
   fflush(file);
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
            fprintf(attcontrolfile[Isc], "therr_X therr_Y therr_Z ");
            fprintf(attcontrolfile[Isc], "werr_X werr_Y werr_Z ");
            fprintf(attcontrolfile[Isc], "Trq_X Trq_Y Trq_Z ");
            fprintf(attcontrolfile[Isc], "Dump_Trq_X Dump_Trq_Y Dump_Trq_Z ");
            fprintf(attcontrolfile[Isc], "Mcmd_X Mcmd_Y Mcmd_Z ");
            fprintf(attcontrolfile[Isc], "\n");
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         fprintf(attcontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.therr.x,
                 SC[Isc].DSM.therr.y, SC[Isc].DSM.therr.z);
         fprintf(attcontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.werr.x,
                 SC[Isc].DSM.werr.y, SC[Isc].DSM.werr.z);
         fprintf(attcontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.Tcmd.x,
                 SC[Isc].DSM.Tcmd.y, SC[Isc].DSM.Tcmd.z);
         fprintf(attcontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.dTcmd.x,
                 SC[Isc].DSM.dTcmd.y, SC[Isc].DSM.dTcmd.z);
         fprintf(attcontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.Mcmd.x,
                 SC[Isc].DSM.Mcmd.y, SC[Isc].DSM.Mcmd.z);
         fprintf(attcontrolfile[Isc], "\n");
      }
      fflush(attcontrolfile[Isc]);
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
            fprintf(poscontrolfile[Isc], "perr_X perr_Y perr_Z ");
            fprintf(poscontrolfile[Isc], "verr_X verr_Y verr_Z ");
            fprintf(poscontrolfile[Isc], "FcmdN_X FcmdN_Y FcmdN_Z ");
            fprintf(poscontrolfile[Isc], "FcmdB_X FcmdB_Y FcmdB_Z ");
            fprintf(poscontrolfile[Isc], "\n");
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         fprintf(poscontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.perr.x,
                 SC[Isc].DSM.perr.y, SC[Isc].DSM.perr.z);
         fprintf(poscontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.verr.x,
                 SC[Isc].DSM.verr.y, SC[Isc].DSM.verr.z);
         fprintf(poscontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.FcmdN.x,
                 SC[Isc].DSM.FcmdN.y, SC[Isc].DSM.FcmdN.z);
         fprintf(poscontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.FcmdB.x,
                 SC[Isc].DSM.FcmdB.y, SC[Isc].DSM.FcmdB.z);
         fprintf(poscontrolfile[Isc], "\n");
      }
      fflush(poscontrolfile[Isc]);
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
            fprintf(ephemfile[Isc], "Beta_(rad) ");
            fprintf(ephemfile[Isc], "INC_(rad) ");
            fprintf(ephemfile[Isc], "AOP_(rad) ");
            fprintf(ephemfile[Isc], "RAAN_(rad) ");
            fprintf(ephemfile[Isc], "TA_(rad) ");
            fprintf(ephemfile[Isc], "SMA_(m) ");
            fprintf(ephemfile[Isc], "ECC ");
            fprintf(ephemfile[Isc], "PERIOD_(s)");
            fprintf(ephemfile[Isc], "\n");
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
         fprintf(ephemfile[Isc], PRNT_DBL, orb_beta);
         fprintf(ephemfile[Isc], PRNT_DBL, orb_inc);
         fprintf(ephemfile[Isc], PRNT_DBL, orb_AOP);
         fprintf(ephemfile[Isc], PRNT_DBL, orb_RAAN);
         fprintf(ephemfile[Isc], PRNT_DBL, orb_anom);
         fprintf(ephemfile[Isc], PRNT_DBL, Orb[SC[Isc].RefOrb].SMA);
         fprintf(ephemfile[Isc], PRNT_DBL, Orb[SC[Isc].RefOrb].ecc);
         fprintf(ephemfile[Isc], PRNT_DBL, orb_time);
         fprintf(ephemfile[Isc], "\n");
      }
      fflush(ephemfile[Isc]);
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
               for (i = 0; i < SC[Isc].Nw; i++)
                  fprintf(WHLFile[Isc], "WHL_%ld ", i);
               fprintf(WHLFile[Isc], "\n");
            }
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         if (SC[Isc].Nw > 0) {
            for (i = 0; i < SC[Isc].Nw; i++)
               fprintf(WHLFile[Isc], "%lf ", SC[Isc].AC.Whl[i].H);
            fprintf(WHLFile[Isc], "\n");
         }
      }
      fflush(WHLFile[Isc]);
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
               for (i = 0; i < SC[Isc].Nthr; i++)
                  fprintf(THRFile[Isc], "THR_%ld ", i);
               fprintf(THRFile[Isc], "\n");
            }
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         if (SC[Isc].Nthr > 0) {
            for (i = 0; i < SC[Isc].Nthr; i++)
               fprintf(THRFile[Isc], PRNT_DBL, SC[Isc].AC.Thr[i].PulseWidthCmd);
            fprintf(THRFile[Isc], "\n");
         }
      }
      fflush(THRFile[Isc]);
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
            fprintf(SVBFile[Isc], "SVB_X SVB_Y SVB_Z ");
            fprintf(SVBFile[Isc], "\n");
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         fprintf(SVBFile[Isc], PRNT_DBL_3VEC, SC[Isc].svb.x, SC[Isc].svb.y,
                 SC[Isc].svb.z);
         fprintf(SVBFile[Isc], "\n");
      }
      fflush(SVBFile[Isc]);
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
            fprintf(gtrackfile[Isc], "Lat Lon ");
            fprintf(gtrackfile[Isc], "\n");
         }
      }
      First = 0;
   }

   for (Isc = 0; Isc < Nsc; Isc++) {
      S = &SC[Isc];
      if (SC[Isc].Exists) {
         W = &World[Orb[S->RefOrb].World];

         SpicePosN2RLngLat(W->CWN, SC[Isc].PosN, &junk, &Lng, &Lat);
         fprintf(gtrackfile[Isc], PRNT_DBL PRNT_DBL, Lat * R2D, Lng * R2D);
         fprintf(gtrackfile[Isc], "\n");
      }
      fflush(gtrackfile[Isc]);
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
      fprintf(FixedFile, PRNT_DBL_3VEC PRNT_DBL_3VEC "\n", SC[0].PosN.x,
              SC[0].PosN.y, SC[0].PosN.z, SC[0].VelN.x, SC[0].VelN.y,
              SC[0].VelN.z);
      fprintf(EnckeFile, PRNT_DBL_3VEC PRNT_DBL_3VEC "\n", SC[1].PosN.x,
              SC[1].PosN.y, SC[1].PosN.z, SC[1].VelN.x, SC[1].VelN.y,
              SC[1].VelN.z);
      fprintf(CowellFile, PRNT_DBL_3VEC PRNT_DBL_3VEC "\n", SC[2].PosN.x,
              SC[2].PosN.y, SC[2].PosN.z, SC[2].VelN.x, SC[2].VelN.y,
              SC[2].VelN.z);
      fprintf(EulHillFile, PRNT_DBL_3VEC PRNT_DBL_3VEC "\n", SC[3].PosN.x,
              SC[3].PosN.y, SC[3].PosN.z, SC[3].VelN.x, SC[3].VelN.y,
              SC[3].VelN.z);
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
      for (i = 0; i < 9; i++) {
         fprintf(outfile, PRNT_DBL_3VEC, SC[i].PosN.x, SC[i].PosN.y,
                 SC[i].PosN.z);
      }
      fprintf(outfile, "\n");
   }
}
/*********************************************************************/
void PerturbReport(void)
{
   static FILE *perturbfile;
   static long First = 1;

   if (First) {
      perturbfile = FileOpen(OutPath, "perturb.42", "wt");
      fprintf(perturbfile, "gravTrqB_X gravTrqB_Y gravTrqB_Z ");
      fprintf(perturbfile, "gravTrqN_X gravTrqN_Y gravTrqN_Z ");
      fprintf(perturbfile, "srpTrqB_X srpTrqB_Y srpTrqB_Z ");
      fprintf(perturbfile, "srpTrqN_X srpTrqN_Y srpTrqN_Z ");
      fprintf(perturbfile, "aeroTrqB_X aeroTrqB_Y aeroTrqB_Z ");
      fprintf(perturbfile, "aeroTrqN_X aeroTrqN_Y aeroTrqN_Z ");
      fprintf(perturbfile, "srpFrcB_X srpFrcB_Y srpFrcB_Z ");
      fprintf(perturbfile, "srpFrcN_X srpFrcN_Y srpFrcN_Z ");
      fprintf(perturbfile, "aeroFrcB_X aeroFrcB_Y aeroFrcB_Z ");
      fprintf(perturbfile, "aeroFrcN_X aeroFrcN_Y aeroFrcN_Z ");
      fprintf(perturbfile, "\n");
      First = 0;
   }

   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].gravTrqB.x, SC[0].gravTrqB.y,
           SC[0].gravTrqB.z);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].gravTrqN.x, SC[0].gravTrqN.y,
           SC[0].gravTrqN.z);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].srpTrqB.x, SC[0].srpTrqB.y,
           SC[0].srpTrqB.z);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].srpTrqN.x, SC[0].srpTrqN.y,
           SC[0].srpTrqN.z);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].aeroTrqB.x, SC[0].aeroTrqB.y,
           SC[0].aeroTrqB.z);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].aeroTrqN.x, SC[0].aeroTrqN.y,
           SC[0].aeroTrqN.z);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].srpFrcB.x, SC[0].srpFrcB.y,
           SC[0].srpFrcB.z);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].srpFrcN.x, SC[0].srpFrcN.y,
           SC[0].srpFrcN.z);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].aeroFrcB.x, SC[0].aeroFrcB.y,
           SC[0].aeroFrcB.z);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].aeroFrcN.x, SC[0].aeroFrcN.y,
           SC[0].aeroFrcN.z);
   fprintf(perturbfile, "\n");

   fflush(perturbfile);
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
   double Roll, Pitch, Yaw;
   mat3x3 CBR, CRN, CRL = MAT3X3_EYE;
   struct WorldType *W;
   vec3 WorldAngVel, wxR, VelN;
   vec3 PosW, VelW, PosR, VelR;
   // double SMA,ecc,inc,RAAN,ArgP,anom,tp,SLR,alpha,rmin,MeanMotion,Period;
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
      fprintf(timefile, PRNT_DBL "\n", SimTime);
      fprintf(DynTimeFile, PRNT_DBL "\n", DynTime);
      fprintf(UtcDateFile, " %ld:%02ld:%02ld:%02ld:%02ld:%09.6lf\n", UTC.Year,
              UTC.Month, UTC.Day, UTC.Hour, UTC.Minute,
              rational2double(UTC.Second));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            D = &SC[Isc].Dyn;
            fprintf(ufile[Isc], PRNT_DBL_3VEC, D->u[0], D->u[1], D->u[2]);
            fprintf(ufile[Isc], "\n");
            fprintf(xfile[Isc], PRNT_DBL_3VEC, D->x[0], D->x[1], D->x[2]);
            fprintf(xfile[Isc], "\n");
            if (SC[Isc].FlexActive) {
               for (i = 0; i < D->Nf; i++)
                  fprintf(uffile[Isc], PRNT_DBL, D->uf[i]);
               fprintf(uffile[Isc], "\n");
               for (i = 0; i < D->Nf; i++)
                  fprintf(xffile[Isc], PRNT_DBL, D->xf[i]);
               fprintf(xffile[Isc], "\n");
            }
            if (SC[Isc].ConstraintsRequested) {
               for (i = 0; i < D->Nc; i++)
                  fprintf(ConstraintFile[Isc], PRNT_DBL,
                          D->GenConstraintFrc[i]);
               fprintf(ConstraintFile[Isc], "\n");
            }
         }
      }
      if (SC[0].Exists) {
         fprintf(PosNfile, PRNT_DBL_3VEC "\n", SC[0].PosN.x, SC[0].PosN.y,
                 SC[0].PosN.z);
         fprintf(VelNfile, PRNT_DBL_3VEC "\n", SC[0].VelN.x, SC[0].VelN.y,
                 SC[0].VelN.z);
         W                = &World[Orb[SC[0].RefOrb].World];
         WorldAngVel.v[0] = 0.0;
         WorldAngVel.v[1] = 0.0;
         WorldAngVel.v[2] = GetWorldW(JD_TDB_MJD, W);

         wxR  = VxV(WorldAngVel, SC[0].PosN);
         VelN = VmVElem(SC[0].VelN, wxR);
         PosW = MxV(W->CWN, SC[0].PosN);
         VelW = MxV(W->CWN, VelN);
         fprintf(PosWfile, PRNT_DBL PRNT_DBL PRNT_DBL "\n", PosW.x, PosW.y,
                 PosW.z);
         fprintf(VelWfile, PRNT_DBL PRNT_DBL PRNT_DBL "\n", VelW.x, VelW.y,
                 VelW.z);
         if (Orb[SC[0].RefOrb].Regime == ORB_FLIGHT) {
            PosR = MxV(Rgn[Orb[SC[0].RefOrb].Region].CN, SC[0].PosR);
            VelR = MxV(Rgn[Orb[SC[0].RefOrb].Region].CN, SC[0].VelR);
            fprintf(PosRfile, PRNT_DBL_3VEC "\n", PosR.x, PosR.y, PosR.z);
            fprintf(VelRfile, PRNT_DBL_3VEC "\n", VelR.x, VelR.y, VelR.z);
         }
         else {
            fprintf(PosRfile, PRNT_DBL_3VEC "\n", SC[0].PosR.x, SC[0].PosR.y,
                    SC[0].PosR.z);
            fprintf(VelRfile, PRNT_DBL_3VEC "\n", SC[0].VelR.x, SC[0].VelR.y,
                    SC[0].VelR.z);
         }
         fprintf(qbnfile, PRNT_DBL_3VEC PRNT_DBL "\n", SC[0].B[0].qn.x,
                 SC[0].B[0].qn.y, SC[0].B[0].qn.z, SC[0].B[0].qn.s);
         fprintf(wbnfile, PRNT_DBL_3VEC "\n", SC[0].B[0].wn.x, SC[0].B[0].wn.y,
                 SC[0].B[0].wn.z);
         fprintf(bvnfile, PRNT_DBL_3VEC "\n", SC[0].bvn.x, SC[0].bvn.y,
                 SC[0].bvn.z);
         fprintf(bvbfile, PRNT_DBL_3VEC "\n", SC[0].bvb.x, SC[0].bvb.y,
                 SC[0].bvb.z);
         fprintf(Hvnfile, PRNT_DBL_3VEC "\n", SC[0].Hvn.x, SC[0].Hvn.y,
                 SC[0].Hvn.z);
         fprintf(Hvbfile, PRNT_DBL PRNT_DBL PRNT_DBL "\n", SC[0].Hvb.x,
                 SC[0].Hvb.y, SC[0].Hvb.z);
         fprintf(svnfile, PRNT_DBL PRNT_DBL PRNT_DBL "\n", SC[0].svn.x,
                 SC[0].svn.y, SC[0].svn.z);
         fprintf(svbfile, PRNT_DBL PRNT_DBL PRNT_DBL "\n", SC[0].svb.x,
                 SC[0].svb.y, SC[0].svb.z);
         fprintf(KEfile, PRNT_DBL "\n", FindTotalKineticEnergy(Orb, &SC[0]));
         // fprintf(ProjAreaFile, PRNT_DBL PRNT_DBL"\n",
         //    FindTotalProjectedArea(&SC[0],ZAxis),
         //    FindTotalUnshadedProjectedArea(&SC[0],ZAxis));
         CRN = MxM(CRL, SC[0].CLN);
         CBR = MxMT(SC[0].B[0].CN, CRN);
         C2A(123, CBR, &Roll, &Pitch, &Yaw);
         fprintf(RPYfile, PRNT_DBL_3VEC "\n", Roll * R2D, Pitch * R2D,
                 Yaw * R2D);
         if (SC[0].Nw > 0) {
            for (i = 0; i < SC[0].Nw; i++) {
               fprintf(Hwhlfile, "%lf ", SC[0].Whl[i].H);
            }
            fprintf(Hwhlfile, "\n");
         }
         if (SC[0].Nmtb > 0) {
            for (i = 0; i < SC[0].Nmtb; i++)
               fprintf(MTBfile, "%lf ", SC[0].MTB[i].M);
            fprintf(MTBfile, "\n");
         }
         if (SC[0].Nthr > 0) {
            for (i = 0; i < SC[0].Nthr; i++)
               fprintf(Thrfile, "%lf ", SC[0].Thr[i].F);
            fprintf(Thrfile, "\n");
         }
         if (SC[0].Nacc > 0) {
            for (i = 0; i < SC[0].Nacc; i++)
               fprintf(AccFile, PRNT_DBL PRNT_DBL, SC[0].Accel[i].TrueAcc,
                       SC[0].Accel[i].MeasAcc);
            fprintf(AccFile, "\n");
         }
         if (SC[0].Ngps > 0) {
            fprintf(GpsFile, PRNT_DBL_3VEC "\n", SC[0].GPS[0].PosN.x,
                    SC[0].GPS[0].PosN.y, SC[0].GPS[0].PosN.z);
         }
         if (SC[0].Ncss > 0) {
            for (i = 0; i < SC[0].Ncss; i++) {
               fprintf(IllumFile, PRNT_DBL, SC[0].CSS[i].Illum);
               fprintf(AlbedoFile, PRNT_DBL, SC[0].CSS[i].Albedo);
            }
            fprintf(IllumFile, "\n");
            fprintf(AlbedoFile, "\n");
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
            DSM_PosHReport();
            DSM_Rot3BodyReport();
         }
      }
   }

   /* An example how to call specialized reporting based on sim case */
   /* if (!strcmp(OutPath,"./Potato/")) PotatoReport(); */

   if (CleanUpFlag) {
      fclose(timefile);
   }
}

/* #ifdef __cplusplus
** }
** #endif
*/

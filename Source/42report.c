/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifdef _ENABLE_SPICE_
#include "SpiceUsr.h"
#endif
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
double FindTotalProjectedArea(struct SCType *S, double VecN[3])
{
   struct BodyType *B;
   struct GeomType *G;
   struct PolyType *P;
   double ProjArea = 0.0;
   double VecB[3], VoN;
   long Ib, Ipoly;

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];

      /* Transform Direction Vector from N to B */
      MxV(B->CN, VecN, VecB);

      G = &Geom[B->GeomTag];
      for (Ipoly = 0; Ipoly < G->Npoly; Ipoly++) {
         P   = &G->Poly[Ipoly];
         VoN = VoV(VecB, P->Norm);
         if (VoN > 0.0) {
            ProjArea += VoN * P->Area;
         }
      }
   }
   return (ProjArea);
}
/*********************************************************************/
double FindTotalUnshadedProjectedArea(struct SCType *S, double VecN[3])
{
   struct BodyType *B;
   struct GeomType *G;
   struct PolyType *P;
   double ProjArea = 0.0;
   double VecB[3], VoN;
   long Ib, Ipoly;

   FindUnshadedAreas(S, VecN);

   for (Ib = 0; Ib < S->Nb; Ib++) {
      B = &S->B[Ib];

      /* Transform Direction Vector from N to B */
      MxV(B->CN, VecN, VecB);

      G = &Geom[B->GeomTag];
      for (Ipoly = 0; Ipoly < G->Npoly; Ipoly++) {
         P   = &G->Poly[Ipoly];
         VoN = VoV(VecB, P->Norm);
         if (VoN > 0.0) {
            ProjArea += VoN * P->UnshadedArea;
         }
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
           SC[0].bvb[0], SC[0].bvb[1], SC[0].bvb[2], SC[0].MAG[0].Field,
           SC[0].MAG[1].Field, SC[0].MAG[2].Field, SC[0].AC.bvb[0],
           SC[0].AC.bvb[1], SC[0].AC.bvb[2]);
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
           SC[0].B[0].wn[0], SC[0].B[0].wn[1], SC[0].B[0].wn[2],
           SC[0].Gyro[0].TrueRate, SC[0].Gyro[1].TrueRate,
           SC[0].Gyro[2].TrueRate, SC[0].Gyro[0].Bias, SC[0].Gyro[1].Bias,
           SC[0].Gyro[2].Bias, SC[0].Gyro[0].Angle, SC[0].Gyro[1].Angle,
           SC[0].Gyro[2].Angle, SC[0].Gyro[0].MeasRate, SC[0].Gyro[1].MeasRate,
           SC[0].Gyro[2].MeasRate, SC[0].AC.wbn[0], SC[0].AC.wbn[1],
           SC[0].AC.wbn[2]);
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
                 SC[Isc].B[0].qn[0], SC[Isc].B[0].qn[1], SC[Isc].B[0].qn[2],
                 SC[Isc].B[0].qn[3]);
         fprintf(attitudefile[Isc], PRNT_DBL_3VEC, SC[Isc].B[0].wn[0],
                 SC[Isc].B[0].wn[1], SC[Isc].B[0].wn[2]);
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
                 SC[Isc].AC.qbn[0], SC[Isc].AC.qbn[1], SC[Isc].AC.qbn[2],
                 SC[Isc].AC.qbn[3]);
         fprintf(attitudefile[Isc], PRNT_DBL_3VEC, SC[Isc].AC.wbn[0],
                 SC[Isc].AC.wbn[1], SC[Isc].AC.wbn[2]);
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
   double PosL[3];
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
         MxV(SC[0].CLN, SC[Isc].PosN, PosL);
         fprintf(inertialfile[Isc], PRNT_DBL_3VEC, SC[Isc].PosN[0],
                 SC[Isc].PosN[1], SC[Isc].PosN[2]);
         fprintf(inertialfile[Isc], PRNT_DBL_3VEC, SC[Isc].VelN[0],
                 SC[Isc].VelN[1], SC[Isc].VelN[2]);
         fprintf(inertialfile[Isc], PRNT_DBL_3VEC, PosL[0], PosL[1], PosL[2]);
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
         double wxr[3], posr[3], velr[3];
         VxV(O->wln, S->PosR, wxr);
         MxV(O->CLN, S->VelR, velr);
         MxV(O->CLN, wxr, posr);
         for (int i = 0; i < 3; i++)
            velr[i] -= posr[i];
         MxV(O->CLN, S->PosR, posr);
         fprintf(relativefile[Isc], PRNT_DBL_3VEC, posr[0], posr[1], posr[2]);
         fprintf(relativefile[Isc], PRNT_DBL_3VEC, velr[0], velr[1], velr[2]);
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
   double svh[3], svw[3], CWH[3][3];
   double Lat, Lng;

   if (First) {
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
   for (Iw = 1; Iw < NWORLD; Iw++) { // Skip Sun
      if (World[Iw].Exists) {
         fprintf(ephemfile[Iw], PRNT_DBL_3VEC, World[Iw].PosH[0],
                 World[Iw].PosH[1], World[Iw].PosH[2]);
         fprintf(ephemfile[Iw], PRNT_DBL_3VEC, World[Iw].VelH[0],
                 World[Iw].VelH[1], World[Iw].VelH[2]);
         fprintf(ephemfile[Iw], "\n");

         for (int i = 0; i < 3; i++)
            svh[i] = -World[Iw].PosH[i];
         UNITV(svh);
         MxM(World[Iw].CWN, World[Iw].CNH, CWH);
         MxV(CWH, svh, svw);

         Lng = atan2(svw[1], svw[0]) * R2D;
         Lat = asin(svw[2]) * R2D;

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
         fprintf(inertialfile[Isc], PRNT_DBL_3VEC, SC[Isc].AC.PosN[0],
                 SC[Isc].AC.PosN[1], SC[Isc].AC.PosN[2]);
         fprintf(inertialfile[Isc], PRNT_DBL_3VEC, SC[Isc].AC.VelN[0],
                 SC[Isc].AC.VelN[1], SC[Isc].AC.VelN[2]);
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
   double full_N_state[6], posRot[3], velRot[3];
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
            for (int i = 0; i < 3; i++)
               full_N_state[i] = SC[Isc].PosN[i];
            for (int i = 0; i < 3; i++)
               full_N_state[i + 3] = SC[Isc].VelN[i];

            StateN2StateRnd(LS, World[LS->Body2].eph.PosN,
                            World[LS->Body2].eph.VelN, SC[Isc].PosN,
                            SC[Isc].VelN, posRot, velRot);

            fprintf(staterotfile[Isc], PRNT_DBL_3VEC, posRot[0], posRot[1],
                    posRot[2]);
            fprintf(staterotfile[Isc], PRNT_DBL_3VEC, velRot[0], velRot[1],
                    velRot[2]);
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
   long Isc, i;
   char s[50];
   double CNJ[3][3];
   double SC_ECI[3], SC_LEI[3], SC_LCI[3];

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
         if (Orb[SC[Isc].RefOrb].World == LUNA) {
            for (i = 0; i < 3; ++i) {
               SC_LEI[i] = SC[Isc].PosN[i];
            }
            LunaInertialFrame(JD_TDB_MJD, CNJ);
            MTxV(CNJ, SC_LEI, SC_LCI);
            for (i = 0; i < 3; ++i) {
               SC_ECI[i] = SC_LCI[i] + World[LUNA].eph.PosN[i];
            }
         }
         else if (Orb[SC[Isc].RefOrb].World == EARTH) {
            LunaInertialFrame(JD_TDB_MJD, CNJ);
            for (i = 0; i < 3; ++i) {
               SC_ECI[i] = SC[Isc].PosN[i];
               SC_LCI[i] = SC_ECI[i] - World[LUNA].eph.PosN[i];
            }
            MxV(CNJ, SC_LCI, SC_LEI);
         }
         else
            break;
         // TODO
         JDType jd_tdb_j2000 = DateToJD(TDB, J2000_EPOCH, TDB_TIME);
         JDType jd_tt_j2000  = DateToJD(TDB, TT_TIME, TDB_TIME);
         fprintf(poshfile[Isc], PRNT_DBL PRNT_DBL, jd_tdb_j2000.day,
                 jd_tt_j2000.day);
         fprintf(poshfile[Isc], PRNT_DBL PRNT_DBL, TDB.tdbTime, DynTime);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[VENUS].PosH[0],
                 World[VENUS].PosH[1], World[VENUS].PosH[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[EARTH].PosH[0],
                 World[EARTH].PosH[1], World[EARTH].PosH[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[LUNA].PosH[0],
                 World[LUNA].PosH[1], World[LUNA].PosH[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[LUNA].eph.PosN[0],
                 World[LUNA].eph.PosN[1], World[LUNA].eph.PosN[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[MARS].PosH[0],
                 World[MARS].PosH[1], World[MARS].PosH[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[JUPITER].PosH[0],
                 World[JUPITER].PosH[1], World[JUPITER].PosH[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, World[SATURN].PosH[0],
                 World[SATURN].PosH[1], World[SATURN].PosH[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, SC[0].PosN[0], SC[0].PosN[1],
                 SC[0].PosN[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, SC[0].PosH[0], SC[0].PosH[1],
                 SC[0].PosH[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, SC_ECI[0], SC_ECI[1], SC_ECI[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, SC_LCI[0], SC_LCI[1], SC_LCI[2]);
         fprintf(poshfile[Isc], PRNT_DBL_3VEC, SC_LEI[0], SC_LEI[1], SC_LEI[2]);
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
   double posRel[3], posRot[3], velRel[3], velRot[3], DCM[3][3];
   struct LagrangeSystemType *LS;
   double z_axis[3] = {0, 0, 1}, ang_rot = M_PI;

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
               for (int i = 0; i < 3; ++i) {
                  posRel[i] = SC[Isc].PosN[i];
                  velRel[i] = SC[Isc].VelN[i];
               }
            }
            else {
               for (int i = 0; i < 3; ++i) {
                  posRel[i] = SC[Isc].PosN[i] - World[LUNA].eph.PosN[i];
                  velRel[i] = SC[Isc].VelN[i] - World[LUNA].eph.VelN[i];
               }
            }
            MxV(LS->CLN, posRel, posRot);
            MxV(LS->CLN, velRel, velRot);
            SimpRot(z_axis, ang_rot, DCM);
            MxV(DCM, posRot, posRot);
            MxV(DCM, velRot, velRot);
            fprintf(rotfile[Isc], PRNT_DBL_3VEC, posRot[0], posRot[1],
                    posRot[2]);
            fprintf(rotfile[Isc], PRNT_DBL_3VEC, velRot[0], velRot[1],
                    velRot[2]);
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
                     fprintf(stateFile[Isc], PRNT_DBL, DateToTime(Nav->Date));
                     break;
                  case ROTMAT_STATE:
                     for (int i = 0; i < 3; i++)
                        fprintf(stateFile[Isc], PRNT_DBL_3VEC, Nav->CRB[i][0],
                                Nav->CRB[i][1], Nav->CRB[i][2]);
                     break;
                  case QUAT_STATE:
                     fprintf(stateFile[Isc],
                             PRNT_DBL PRNT_DBL PRNT_DBL PRNT_DBL, Nav->qbr[0],
                             Nav->qbr[1], Nav->qbr[2], Nav->qbr[3]);
                     break;
                  case OMEGA_STATE:
                     fprintf(stateFile[Isc], PRNT_DBL_3VEC, Nav->wbr[0],
                             Nav->wbr[1], Nav->wbr[2]);
                     break;
                  case POS_STATE: {
                     double tmpV1[3] = {0.0}, tmpV2[3] = {0.0};
                     for (int i = 0; i < 3; i++)
                        tmpV1[i] = Nav->PosR[i] + Nav->refPos[i];
                     MTxV(Nav->refCRN, tmpV1, tmpV2);
                     fprintf(stateFile[Isc], PRNT_DBL_3VEC, tmpV2[0], tmpV2[1],
                             tmpV2[2]);
                  } break;
                  case VEL_STATE: {
                     double tmpV1[3] = {0.0}, tmpV2[3] = {0.0};
                     for (int i = 0; i < 3; i++)
                        tmpV1[i] = Nav->VelR[i] + Nav->refVel[i];
                     MTxV(Nav->refCRN, tmpV1, tmpV2);
                     fprintf(stateFile[Isc], PRNT_DBL_3VEC, tmpV2[0], tmpV2[1],
                             tmpV2[2]);
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
         fprintf(attcontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.therr[0],
                 SC[Isc].DSM.therr[1], SC[Isc].DSM.therr[2]);
         fprintf(attcontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.werr[0],
                 SC[Isc].DSM.werr[1], SC[Isc].DSM.werr[2]);
         fprintf(attcontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.Tcmd[0],
                 SC[Isc].DSM.Tcmd[1], SC[Isc].DSM.Tcmd[2]);
         fprintf(attcontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.dTcmd[0],
                 SC[Isc].DSM.dTcmd[1], SC[Isc].DSM.dTcmd[2]);
         fprintf(attcontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.Mcmd[0],
                 SC[Isc].DSM.Mcmd[1], SC[Isc].DSM.Mcmd[2]);
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
         fprintf(poscontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.perr[0],
                 SC[Isc].DSM.perr[1], SC[Isc].DSM.perr[2]);
         fprintf(poscontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.verr[0],
                 SC[Isc].DSM.verr[1], SC[Isc].DSM.verr[2]);
         fprintf(poscontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.FcmdN[0],
                 SC[Isc].DSM.FcmdN[1], SC[Isc].DSM.FcmdN[2]);
         fprintf(poscontrolfile[Isc], PRNT_DBL_3VEC, SC[Isc].DSM.FcmdB[0],
                 SC[Isc].DSM.FcmdB[1], SC[Isc].DSM.FcmdB[2]);
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
         fprintf(SVBFile[Isc], PRNT_DBL_3VEC, SC[Isc].svb[0], SC[Isc].svb[1],
                 SC[Isc].svb[2]);
         fprintf(SVBFile[Isc], "\n");
      }
      fflush(SVBFile[Isc]);
   }
}
/*********************************************************************/
#ifdef _ENABLE_SPICE_
void DSM_GroundTrackReport(void)
{
   static FILE **gtrackfile;
   static long First = 1;
   long Isc;
   char s[40];
   struct WorldType *W;
   struct SCType *S;
   double p[3], Lat, Lng, junk;

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

         MxV(W->CWN, SC[Isc].PosN, p);
         reclat_c(p, &junk, &Lng, &Lat);

         fprintf(gtrackfile[Isc], PRNT_DBL PRNT_DBL, Lat * R2D, Lng * R2D);
         fprintf(gtrackfile[Isc], "\n");
      }
      fflush(gtrackfile[Isc]);
   }
}
#endif
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
      fprintf(FixedFile, PRNT_DBL_3VEC PRNT_DBL_3VEC "\n", SC[0].PosN[0],
              SC[0].PosN[1], SC[0].PosN[2], SC[0].VelN[0], SC[0].VelN[1],
              SC[0].VelN[2]);
      fprintf(EnckeFile, PRNT_DBL_3VEC PRNT_DBL_3VEC "\n", SC[1].PosN[0],
              SC[1].PosN[1], SC[1].PosN[2], SC[1].VelN[0], SC[1].VelN[1],
              SC[1].VelN[2]);
      fprintf(CowellFile, PRNT_DBL_3VEC PRNT_DBL_3VEC "\n", SC[2].PosN[0],
              SC[2].PosN[1], SC[2].PosN[2], SC[2].VelN[0], SC[2].VelN[1],
              SC[2].VelN[2]);
      fprintf(EulHillFile, PRNT_DBL_3VEC PRNT_DBL_3VEC "\n", SC[3].PosN[0],
              SC[3].PosN[1], SC[3].PosN[2], SC[3].VelN[0], SC[3].VelN[1],
              SC[3].VelN[2]);
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
         fprintf(outfile, PRNT_DBL_3VEC, SC[i].PosN[0], SC[i].PosN[1],
                 SC[i].PosN[2]);
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

   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].gravTrqB[0], SC[0].gravTrqB[1],
           SC[0].gravTrqB[2]);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].gravTrqN[0], SC[0].gravTrqN[1],
           SC[0].gravTrqN[2]);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].srpTrqB[0], SC[0].srpTrqB[1],
           SC[0].srpTrqB[2]);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].srpTrqN[0], SC[0].srpTrqN[1],
           SC[0].srpTrqN[2]);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].aeroTrqB[0], SC[0].aeroTrqB[1],
           SC[0].aeroTrqB[2]);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].aeroTrqN[0], SC[0].aeroTrqN[1],
           SC[0].aeroTrqN[2]);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].srpFrcB[0], SC[0].srpFrcB[1],
           SC[0].srpFrcB[2]);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].srpFrcN[0], SC[0].srpFrcN[1],
           SC[0].srpFrcN[2]);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].aeroFrcB[0], SC[0].aeroFrcB[1],
           SC[0].aeroFrcB[2]);
   fprintf(perturbfile, PRNT_DBL_3VEC, SC[0].aeroFrcN[0], SC[0].aeroFrcN[1],
           SC[0].aeroFrcN[2]);
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
   double CBR[3][3], CRN[3][3], Roll, Pitch, Yaw;
   struct WorldType *W;
   double WorldAngVel[3], wxR[3], VelN[3];
   double PosW[3], VelW[3], PosR[3], VelR[3];
   double CRL[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
   // double SMA,ecc,inc,RAAN,ArgP,anom,tp,SLR,alpha,rmin,MeanMotion,Period;
   char s[40];
   // double ZAxis[3] = {0.0,0.0,1.0};

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
              UTC.Month, UTC.Day, UTC.Hour, UTC.Minute, UTC.Second);
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
         fprintf(PosNfile, PRNT_DBL_3VEC "\n", SC[0].PosN[0], SC[0].PosN[1],
                 SC[0].PosN[2]);
         fprintf(VelNfile, PRNT_DBL_3VEC "\n", SC[0].VelN[0], SC[0].VelN[1],
                 SC[0].VelN[2]);
         W              = &World[Orb[SC[0].RefOrb].World];
         WorldAngVel[0] = 0.0;
         WorldAngVel[1] = 0.0;
         WorldAngVel[2] = W->w;
         VxV(WorldAngVel, SC[0].PosN, wxR);
         for (i = 0; i < 3; i++)
            VelN[i] = SC[0].VelN[i] - wxR[i];
         MxV(W->CWN, SC[0].PosN, PosW);
         MxV(W->CWN, VelN, VelW);
         fprintf(PosWfile, PRNT_DBL PRNT_DBL PRNT_DBL "\n", PosW[0], PosW[1],
                 PosW[2]);
         fprintf(VelWfile, PRNT_DBL PRNT_DBL PRNT_DBL "\n", VelW[0], VelW[1],
                 VelW[2]);
         if (Orb[SC[0].RefOrb].Regime == ORB_FLIGHT) {
            MxV(Rgn[Orb[SC[0].RefOrb].Region].CN, SC[0].PosR, PosR);
            MxV(Rgn[Orb[SC[0].RefOrb].Region].CN, SC[0].VelR, VelR);
            fprintf(PosRfile, PRNT_DBL_3VEC "\n", PosR[0], PosR[1], PosR[2]);
            fprintf(VelRfile, PRNT_DBL_3VEC "\n", VelR[0], VelR[1], VelR[2]);
         }
         else {
            fprintf(PosRfile, PRNT_DBL_3VEC "\n", SC[0].PosR[0], SC[0].PosR[1],
                    SC[0].PosR[2]);
            fprintf(VelRfile, PRNT_DBL_3VEC "\n", SC[0].VelR[0], SC[0].VelR[1],
                    SC[0].VelR[2]);
         }
         fprintf(qbnfile, PRNT_DBL_3VEC PRNT_DBL "\n", SC[0].B[0].qn[0],
                 SC[0].B[0].qn[1], SC[0].B[0].qn[2], SC[0].B[0].qn[3]);
         fprintf(wbnfile, PRNT_DBL_3VEC "\n", SC[0].B[0].wn[0],
                 SC[0].B[0].wn[1], SC[0].B[0].wn[2]);
         fprintf(bvnfile, PRNT_DBL_3VEC "\n", SC[0].bvn[0], SC[0].bvn[1],
                 SC[0].bvn[2]);
         fprintf(bvbfile, PRNT_DBL_3VEC "\n", SC[0].bvb[0], SC[0].bvb[1],
                 SC[0].bvb[2]);
         fprintf(Hvnfile, PRNT_DBL_3VEC "\n", SC[0].Hvn[0], SC[0].Hvn[1],
                 SC[0].Hvn[2]);
         fprintf(Hvbfile, PRNT_DBL PRNT_DBL PRNT_DBL "\n", SC[0].Hvb[0],
                 SC[0].Hvb[1], SC[0].Hvb[2]);
         fprintf(svnfile, PRNT_DBL PRNT_DBL PRNT_DBL "\n", SC[0].svn[0],
                 SC[0].svn[1], SC[0].svn[2]);
         fprintf(svbfile, PRNT_DBL PRNT_DBL PRNT_DBL "\n", SC[0].svb[0],
                 SC[0].svb[1], SC[0].svb[2]);
         fprintf(KEfile, PRNT_DBL "\n", FindTotalKineticEnergy(Orb, &SC[0]));
         // fprintf(ProjAreaFile, PRNT_DBL PRNT_DBL"\n",
         //    FindTotalProjectedArea(&SC[0],ZAxis),
         //    FindTotalUnshadedProjectedArea(&SC[0],ZAxis));
         MxM(CRL, SC[0].CLN, CRN);
         MxMT(SC[0].B[0].CN, CRN, CBR);
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
            fprintf(GpsFile, PRNT_DBL_3VEC "\n", SC[0].GPS[0].PosN[0],
                    SC[0].GPS[0].PosN[1], SC[0].GPS[0].PosN[2]);
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

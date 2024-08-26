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

   fprintf(magfile, "%le %le %le %le %le %le %le %le %le \n", SC[0].bvb[0],
           SC[0].bvb[1], SC[0].bvb[2], SC[0].MAG[0].Field, SC[0].MAG[1].Field,
           SC[0].MAG[2].Field, SC[0].AC.bvb[0], SC[0].AC.bvb[1],
           SC[0].AC.bvb[2]);
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
           "%le %le %le %le %le %le %le %le %le %le %le %le %le %le %le %le "
           "%le %le \n",
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
         fprintf(attitudefile[Isc], "%18.36le %18.36le %18.36le %18.36le ",
                 SC[Isc].B[0].qn[0], SC[Isc].B[0].qn[1], SC[Isc].B[0].qn[2],
                 SC[Isc].B[0].qn[3]);
         fprintf(attitudefile[Isc], "%18.36le %18.36le %18.36le ",
                 SC[Isc].B[0].wn[0], SC[Isc].B[0].wn[1], SC[Isc].B[0].wn[2]);
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
         fprintf(attitudefile[Isc], "%18.36le %18.36le %18.36le %18.36le ",
                 SC[Isc].AC.qbn[0], SC[Isc].AC.qbn[1], SC[Isc].AC.qbn[2],
                 SC[Isc].AC.qbn[3]);
         fprintf(attitudefile[Isc], "%18.36le %18.36le %18.36le ",
                 SC[Isc].AC.wbn[0], SC[Isc].AC.wbn[1], SC[Isc].AC.wbn[2]);
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
   char s[40];

   if (First) {
      inertialfile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            sprintf(s, "DSM_inertial_%02li.42", Isc);
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
         fprintf(inertialfile[Isc], "%18.36le %18.36le %18.36le ",
                 SC[Isc].PosN[0], SC[Isc].PosN[1], SC[Isc].PosN[2]);
         fprintf(inertialfile[Isc], "%18.36le %18.36le %18.36le ",
                 SC[Isc].VelN[0], SC[Isc].VelN[1], SC[Isc].VelN[2]);
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
         fprintf(relativefile[Isc], "%18.36le %18.36le %18.36le ", posr[0],
                 posr[1], posr[2]);
         fprintf(relativefile[Isc], "%18.36le %18.36le %18.36le ", velr[0],
                 velr[1], velr[2]);
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
   char s[40];
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
         fprintf(ephemfile[Iw], "%18.12le %18.12le %18.12le ",
                 World[Iw].PosH[0], World[Iw].PosH[1], World[Iw].PosH[2]);
         fprintf(ephemfile[Iw], "%18.12le %18.12le %18.12le ",
                 World[Iw].VelH[0], World[Iw].VelH[1], World[Iw].VelH[2]);
         fprintf(ephemfile[Iw], "\n");

         for (int i = 0; i < 3; i++)
            svh[i] = -World[Iw].PosH[i];
         UNITV(svh);
         MxM(World[Iw].CWN, World[Iw].CNH, CWH);
         MxV(CWH, svh, svw);

         Lng = atan2(svw[1], svw[0]) * R2D;
         Lat = asin(svw[2]) * R2D;

         fprintf(suntrackfile[Iw], "%18.12le %18.12le ", Lat, Lng);
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
         fprintf(inertialfile[Isc], "%18.36le %18.36le %18.36le ",
                 SC[Isc].AC.PosN[0], SC[Isc].AC.PosN[1], SC[Isc].AC.PosN[2]);
         fprintf(inertialfile[Isc], "%18.36le %18.36le %18.36le ",
                 SC[Isc].AC.VelN[0], SC[Isc].AC.VelN[1], SC[Isc].AC.VelN[2]);
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

            fprintf(staterotfile[Isc], "%18.12le %18.12le %18.12le ", posRot[0],
                    posRot[1], posRot[2]);
            fprintf(staterotfile[Isc], "%18.12le %18.12le %18.12le ", velRot[0],
                    velRot[1], velRot[2]);
            fprintf(staterotfile[Isc], "\n");
         }
      }
      fflush(staterotfile[Isc]);
   }
}
/*********************************************************************/
void DSM_NAV_StateReport(void)
{
   static FILE **stateFile, **covFile, **timeFile;
   static long First = 1;
   long Isc;
   enum States state;
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
         for (state = INIT_STATE; state <= FIN_STATE; state++) {
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
         for (state = INIT_STATE; state <= FIN_STATE; state++) {
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
         for (state = INIT_STATE; state <= FIN_STATE; state++) {
            if (Nav->stateActive[state] == TRUE) {
               writeTime = TRUE;
               switch (state) {
                  case TIME_STATE:
                     fprintf(stateFile[Isc], "%18.36le ",
                             DateToTime(Nav->Date.Year, Nav->Date.Month,
                                        Nav->Date.Day, Nav->Date.Hour,
                                        Nav->Date.Minute, Nav->Date.Second));
                     break;
                  case ROTMAT_STATE:
                     for (int i = 0; i < 3; i++)
                        for (int j = 0; j < 3; j++)
                           fprintf(stateFile[Isc], "%18.36le ", Nav->CRB[i][j]);
                     break;
                  case QUAT_STATE:
                     for (int i = 0; i < 4; i++)
                        fprintf(stateFile[Isc], "%18.36le ", Nav->qbr[i]);
                     break;
                  case OMEGA_STATE:
                     fprintf(stateFile[Isc], "%18.36le %18.36le %18.36le ",
                             Nav->wbr[0], Nav->wbr[1], Nav->wbr[2]);
                     break;
                  case POS_STATE: {
                     double tmpV1[3] = {0.0}, tmpV2[3] = {0.0};
                     for (int i = 0; i < 3; i++)
                        tmpV1[i] = Nav->PosR[i] + Nav->refPos[i];
                     MTxV(Nav->refCRN, tmpV1, tmpV2);
                     fprintf(stateFile[Isc], "%18.36le %18.36le %18.36le ",
                             tmpV2[0], tmpV2[1], tmpV2[2]);
                  } break;
                  case VEL_STATE: {
                     double tmpV1[3] = {0.0}, tmpV2[3] = {0.0};
                     for (int i = 0; i < 3; i++)
                        tmpV1[i] = Nav->VelR[i] + Nav->refVel[i];
                     MTxV(Nav->refCRN, tmpV1, tmpV2);
                     fprintf(stateFile[Isc], "%18.36le %18.36le %18.36le ",
                             tmpV2[0], tmpV2[1], tmpV2[2]);
                  } break;
                  default:
                     break;
               }
            }
         }
         fprintf(stateFile[Isc], "\n");
         fflush(stateFile[Isc]);

         if (writeTime) {
            fprintf(timeFile[Isc], "%18.36le\n", DynTime);
            fflush(timeFile[Isc]);
         }

         long navDim       = Nav->navDim;
         double **linTForm = GetStateLinTForm(&SC[Isc].DSM.DsmNav);

         MxMTG(Nav->S, Nav->S, Nav->P, navDim, navDim, navDim);
         MxMG(linTForm, Nav->P, Nav->NxN, navDim, navDim, navDim);
         MxMTG(Nav->NxN, linTForm, Nav->NxN2, navDim, navDim, navDim);
         DestroyMatrix(linTForm);
         for (state = INIT_STATE; state <= FIN_STATE; state++) {
            int stateInd = Nav->navInd[state];
            if (Nav->stateActive[state] == TRUE) {
               switch (state) {
                  case TIME_STATE:
                     fprintf(covFile[Isc], "%18.36le ",
                             sqrt(Nav->NxN2[stateInd][stateInd]));
                     break;
                  default:
                     for (int i = 0; i < 3; i++)
                        fprintf(covFile[Isc], "%18.36le ",
                                sqrt(Nav->NxN2[stateInd + i][stateInd + i]));
                     break;
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
#if REPORT_RESIDUALS == TRUE
void DSM_NAV_ResidualsReport(double time, double **residuals[FIN_SENSOR + 1])
{
   static FILE **residualFile;
   static long First = TRUE;
   long Isc;
   enum SensorType sensor;
   char s[40];

   struct DSMNavType *Nav;

   if (First) {
      residualFile = (FILE **)calloc(Nsc, sizeof(FILE *));
      for (Isc = 0; Isc < Nsc; Isc++) {
         sprintf(s, "DSM_residuals_%02li.42", Isc);
         residualFile[Isc] = FileOpen(OutPath, s, "wt");
         Nav               = &SC[Isc].DSM.DsmNav;
         FILE *file        = residualFile[Isc];
         fprintf(file, "Time; ");
         for (sensor = INIT_SENSOR; sensor < FIN_SENSOR; sensor++) {
            if (Nav->sensorActive[sensor] == TRUE) {
               for (int i = 0; i < Nav->nSensor[sensor]; i++) {
                  switch (sensor) {
                     case GPS_SENSOR:
                        fprintf(file,
                                "GPS[%02i]_Pos_x GPS[%02i]_Pos_y "
                                "GPS[%02i]_Pos_z GPS[%02i]_Vel_x "
                                "GPS[%02i]_Vel_y GPS[%02i]_Vel_z ",
                                i, i, i, i, i, i);
                        break;
                     case STARTRACK_SENSOR:
                        fprintf(
                            file,
                            "STARTRACK[%02i]_Theta_x STARTRACK[%02i]_Theta_z "
                            "STARTRACK[%02i]_Theta_z ",
                            i, i, i);
                        break;
                     case FSS_SENSOR:
                        fprintf(file, "FSS[%02i]_Theta_h FSS[%02i]_Theta_v ", i,
                                i);
                        break;
                     case CSS_SENSOR:
                        fprintf(file, "CSS[%02i]_Out ", i);
                        break;
                     case GYRO_SENSOR:
                        fprintf(file, "GYRO[%02i]_Out ", i);
                        break;
                     case MAG_SENSOR:
                        fprintf(file, "MAG[%02i]_Out ", i);
                        break;
                     case ACCEL_SENSOR:
                        fprintf(file, "ACCEL[%02i]_Out ", i);

                        break;
                     default:
                        printf("INIT_SENSOR and/or FIN_SENSOR are not "
                               "configured correctly in "
                               "navkit.c. Exiting...\n");
                        exit(EXIT_FAILURE);
                        break;
                  }
                  fprintf(file, "; ");
               }
            }
         }
         fprintf(file, "\n");
      }
      First = FALSE;
   }
   for (Isc = 0; Isc < Nsc; Isc++) {
      Nav        = &SC[Isc].DSM.DsmNav;
      FILE *file = residualFile[Isc];
      fprintf(file, "%18.36le ; ", time);
      for (sensor = INIT_SENSOR; sensor < FIN_SENSOR; sensor++) {
         if (Nav->sensorActive[sensor] == TRUE) {
            for (int i = 0; i < Nav->nSensor[sensor]; i++) {
               if (residuals[sensor][i] != NULL) {
                  switch (sensor) {
                     case GPS_SENSOR:
                        for (int j = 0; j < 6; j++)
                           fprintf(file, "%18.20le ", residuals[sensor][i][j]);
                        break;
                     case STARTRACK_SENSOR:
                        for (int j = 0; j < 3; j++)
                           fprintf(file, "%18.20le ", residuals[sensor][i][j]);
                        break;
                     case FSS_SENSOR:
                        for (int j = 0; j < 2; j++)
                           fprintf(file, "%18.20le ", residuals[sensor][i][j]);
                        break;
                     case CSS_SENSOR:
                     case GYRO_SENSOR:
                     case MAG_SENSOR:
                     case ACCEL_SENSOR:
                        fprintf(file, "%18.20le ", residuals[sensor][i][0]);
                        break;
                     default:
                        printf("INIT_SENSOR and/or FIN_SENSOR are not "
                               "configured correctly in "
                               "navkit.c. Exiting...\n");
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
                        printf("INIT_SENSOR and/or FIN_SENSOR are not "
                               "configured correctly in "
                               "navkit.c. Exiting...\n");
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
         fprintf(attcontrolfile[Isc], "%18.12le %18.12le %18.12le ",
                 SC[Isc].DSM.therr[0], SC[Isc].DSM.therr[1],
                 SC[Isc].DSM.therr[2]);
         fprintf(attcontrolfile[Isc], "%18.12le %18.12le %18.12le ",
                 SC[Isc].DSM.werr[0], SC[Isc].DSM.werr[1], SC[Isc].DSM.werr[2]);
         fprintf(attcontrolfile[Isc], "%18.12le %18.12le %18.12le ",
                 SC[Isc].DSM.Tcmd[0], SC[Isc].DSM.Tcmd[1], SC[Isc].DSM.Tcmd[2]);
         fprintf(attcontrolfile[Isc], "%18.12le %18.12le %18.12le ",
                 SC[Isc].DSM.dTcmd[0], SC[Isc].DSM.dTcmd[1],
                 SC[Isc].DSM.dTcmd[2]);
         fprintf(attcontrolfile[Isc], "%18.12le %18.12le %18.12le ",
                 SC[Isc].DSM.Mcmd[0], SC[Isc].DSM.Mcmd[1], SC[Isc].DSM.Mcmd[2]);
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
         fprintf(poscontrolfile[Isc], "%18.12le %18.12le %18.12le ",
                 SC[Isc].DSM.perr[0], SC[Isc].DSM.perr[1], SC[Isc].DSM.perr[2]);
         fprintf(poscontrolfile[Isc], "%18.12le %18.12le %18.12le ",
                 SC[Isc].DSM.verr[0], SC[Isc].DSM.verr[1], SC[Isc].DSM.verr[2]);
         fprintf(poscontrolfile[Isc], "%18.12le %18.12le %18.12le ",
                 SC[Isc].DSM.FcmdN[0], SC[Isc].DSM.FcmdN[1],
                 SC[Isc].DSM.FcmdN[2]);
         fprintf(poscontrolfile[Isc], "%18.12le %18.12le %18.12le ",
                 SC[Isc].DSM.FcmdB[0], SC[Isc].DSM.FcmdB[1],
                 SC[Isc].DSM.FcmdB[2]);
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
         fprintf(ephemfile[Isc], "%18.12le ", orb_beta);
         fprintf(ephemfile[Isc], "%18.12le ", orb_inc);
         fprintf(ephemfile[Isc], "%18.12le ", orb_AOP);
         fprintf(ephemfile[Isc], "%18.12le ", orb_RAAN);
         fprintf(ephemfile[Isc], "%18.12le ", orb_anom);
         fprintf(ephemfile[Isc], "%18.12le ", Orb[SC[Isc].RefOrb].SMA);
         fprintf(ephemfile[Isc], "%18.12le ", Orb[SC[Isc].RefOrb].ecc);
         fprintf(ephemfile[Isc], "%18.12le", orb_time);
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
               fprintf(THRFile[Isc], "%lf ", SC[Isc].AC.Thr[i].PulseWidthCmd);
            fprintf(THRFile[Isc], "\n");
         }
      }
      fflush(THRFile[Isc]);
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

         fprintf(gtrackfile[Isc], "%18.12le %18.12le ", Lat * R2D, Lng * R2D);
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
      fprintf(FixedFile,
              "%24.18le %24.18le %24.18le %24.18le %24.18le %24.18le\n",
              SC[0].PosN[0], SC[0].PosN[1], SC[0].PosN[2], SC[0].VelN[0],
              SC[0].VelN[1], SC[0].VelN[2]);
      fprintf(EnckeFile,
              "%24.18le %24.18le %24.18le %24.18le %24.18le %24.18le\n",
              SC[1].PosN[0], SC[1].PosN[1], SC[1].PosN[2], SC[1].VelN[0],
              SC[1].VelN[1], SC[1].VelN[2]);
      fprintf(CowellFile,
              "%24.18le %24.18le %24.18le %24.18le %24.18le %24.18le\n",
              SC[2].PosN[0], SC[2].PosN[1], SC[2].PosN[2], SC[2].VelN[0],
              SC[2].VelN[1], SC[2].VelN[2]);
      fprintf(EulHillFile,
              "%24.18le %24.18le %24.18le %24.18le %24.18le %24.18le\n",
              SC[3].PosN[0], SC[3].PosN[1], SC[3].PosN[2], SC[3].VelN[0],
              SC[3].VelN[1], SC[3].VelN[2]);
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
         fprintf(outfile, "%24.18le %24.18le %24.18le ", SC[i].PosN[0],
                 SC[i].PosN[1], SC[i].PosN[2]);
      }
      fprintf(outfile, "\n");
   }
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
      Hvnfile  = FileOpen(OutPath, "Hvn.42", "w");
      Hvbfile  = FileOpen(InOutPath, "Hvb.42", "w");
      svnfile  = FileOpen(OutPath, "svn.42", "w");
      svbfile  = FileOpen(OutPath, "svb.42", "w");
      KEfile   = FileOpen(OutPath, "KE.42", "w");
      // ProjAreaFile = FileOpen(InOutPath,"ProjArea.42","w");
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
   }

   if (OutFlag) {
      fprintf(timefile, "%lf\n", SimTime);
      fprintf(DynTimeFile, "%lf\n", DynTime);
      fprintf(UtcDateFile, " %ld:%02ld:%02ld:%02ld:%02ld:%09.6lf\n", UTC.Year,
              UTC.Month, UTC.Day, UTC.Hour, UTC.Minute, UTC.Second);
      for (Isc = 0; Isc < Nsc; Isc++) {
         if (SC[Isc].Exists) {
            D = &SC[Isc].Dyn;
            for (i = 0; i < D->Nu; i++)
               fprintf(ufile[Isc], "%18.12le ", D->u[i]);
            fprintf(ufile[Isc], "\n");
            for (i = 0; i < D->Nx; i++)
               fprintf(xfile[Isc], "%18.12le ", D->x[i]);
            fprintf(xfile[Isc], "\n");
            if (SC[Isc].FlexActive) {
               for (i = 0; i < D->Nf; i++)
                  fprintf(uffile[Isc], "%18.12le ", D->uf[i]);
               fprintf(uffile[Isc], "\n");
               for (i = 0; i < D->Nf; i++)
                  fprintf(xffile[Isc], "%18.12le ", D->xf[i]);
               fprintf(xffile[Isc], "\n");
            }
            if (SC[Isc].ConstraintsRequested) {
               for (i = 0; i < D->Nc; i++)
                  fprintf(ConstraintFile[Isc], "%18.12le ",
                          D->GenConstraintFrc[i]);
               fprintf(ConstraintFile[Isc], "\n");
            }
         }
      }
      if (SC[0].Exists) {
         fprintf(PosNfile, "%le %le %le\n", SC[0].PosN[0], SC[0].PosN[1],
                 SC[0].PosN[2]);
         fprintf(VelNfile, "%le %le %le\n", SC[0].VelN[0], SC[0].VelN[1],
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
         fprintf(PosWfile, "%18.12le %18.12le %18.12le\n", PosW[0], PosW[1],
                 PosW[2]);
         fprintf(VelWfile, "%18.12le %18.12le %18.12le\n", VelW[0], VelW[1],
                 VelW[2]);
         if (Orb[SC[0].RefOrb].Regime == ORB_FLIGHT) {
            MxV(Rgn[Orb[SC[0].RefOrb].Region].CN, SC[0].PosR, PosR);
            MxV(Rgn[Orb[SC[0].RefOrb].Region].CN, SC[0].VelR, VelR);
            fprintf(PosRfile, "%le %le %le\n", PosR[0], PosR[1], PosR[2]);
            fprintf(VelRfile, "%le %le %le\n", VelR[0], VelR[1], VelR[2]);
         }
         else {
            fprintf(PosRfile, "%le %le %le\n", SC[0].PosR[0], SC[0].PosR[1],
                    SC[0].PosR[2]);
            fprintf(VelRfile, "%le %le %le\n", SC[0].VelR[0], SC[0].VelR[1],
                    SC[0].VelR[2]);
         }
         fprintf(qbnfile, "%le %le %le %le\n", SC[0].B[0].qn[0],
                 SC[0].B[0].qn[1], SC[0].B[0].qn[2], SC[0].B[0].qn[3]);
         fprintf(wbnfile, "%le %le %le\n", SC[0].B[0].wn[0], SC[0].B[0].wn[1],
                 SC[0].B[0].wn[2]);
         fprintf(Hvnfile, "%18.12le %18.12le %18.12le\n", SC[0].Hvn[0],
                 SC[0].Hvn[1], SC[0].Hvn[2]);
         fprintf(Hvbfile, "%18.12le %18.12le %18.12le\n", SC[0].Hvb[0],
                 SC[0].Hvb[1], SC[0].Hvb[2]);
         fprintf(svnfile, "%18.12le %18.12le %18.12le\n", SC[0].svn[0],
                 SC[0].svn[1], SC[0].svn[2]);
         fprintf(svbfile, "%18.12le %18.12le %18.12le\n", SC[0].svb[0],
                 SC[0].svb[1], SC[0].svb[2]);
         fprintf(KEfile, "%18.12le\n", FindTotalKineticEnergy(&SC[0]));
         // fprintf(ProjAreaFile,"%18.12le %18.12le\n",
         //    FindTotalProjectedArea(&SC[0],ZAxis),
         //    FindTotalUnshadedProjectedArea(&SC[0],ZAxis));
         MxM(CRL, SC[0].CLN, CRN);
         MxMT(SC[0].B[0].CN, CRN, CBR);
         C2A(123, CBR, &Roll, &Pitch, &Yaw);
         fprintf(RPYfile, "%le %le %le\n", Roll * R2D, Pitch * R2D, Yaw * R2D);
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
               fprintf(AccFile, "%le %le ", SC[0].Accel[i].TrueAcc,
                       SC[0].Accel[i].MeasAcc);
            fprintf(AccFile, "\n");
         }
         if (SC[0].Ncss > 0) {
            for (i = 0; i < SC[0].Ncss; i++) {
               fprintf(IllumFile, "%le ", SC[0].CSS[i].Illum);
               fprintf(AlbedoFile, "%le ", SC[0].CSS[i].Albedo);
            }
            fprintf(IllumFile, "\n");
            fprintf(AlbedoFile, "\n");
         }

         // MagReport();
         // GyroReport();
         // OrbPropReport();
         // GmatReport();

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
            // DSM_GroundTrackReport();

            DSM_StateRot3BodyReport();
         }
      }
   }

   /* An example how to call specialized reporting based on sim case */
   /* if (!strcmp(InOutPath,"./Potato/")) PotatoReport(); */

   if (CleanUpFlag) {
      fclose(timefile);
   }
}

/* #ifdef __cplusplus
** }
** #endif
*/

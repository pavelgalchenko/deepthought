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
#include <mathkit.h>
#include <stdio.h>
#include <timekit.h>

/* #ifdef __cplusplus
** namespace _42 {
** using namespace Kit;
** #endif
*/

/**********************************************************************/
/* #define _RADBELT_ */
void Environment(JDType jd, struct WorldType *const worlds,
                 struct OrbitType *const orbs, struct SCType *S)
{
   struct OrbitType *O;
   struct WorldType *P;
   double Alt;
   double PosW[3];
#ifdef _RADBELT_
   int NumEnergies         = 5;
   float ElectronEnergy[5] = {0.15, 0.5, 1.0, 3.0, 4.0};    /* MeV */
   float ProtonEnergy[5]   = {4.0, 10.0, 20.0, 30.0, 50.0}; /* MeV */
   static double **Flux;
   double MagLat;
   static long First = 1;

   if (First) {
      First = 0;
      Flux  = CreateMatrix(4, NumEnergies);
   }
#endif

   O = &orbs[S->RefOrb];
   P = &worlds[O->World];

   /* .. Magnetic Field */
   if (MagModel.Type == DIPOLE) {
      DipoleMagField(P->DipoleMoment, P->DipoleAxis, P->DipoleOffset, S->PosN,
                     P->PriMerAng, S->bvn);
   }
   else if (MagModel.Type == IGRF && O->World == EARTH) {
      IGRFMagField(ModelPath, UTC, MagModel.N, MagModel.M, S->PosN,
                   P->PriMerAng, S->bvn);
   }
   else {
      S->bvn[0] = 0.0;
      S->bvn[1] = 0.0;
      S->bvn[2] = 0.0;
   }

   MxV(S->B[0].CN, S->bvn, S->bvb);

   ChangeSystemEpoch(TT_TIME, GMAT_MJD_EPOCH, &jd);
   DateType date_tt = JDToDate(jd, TT_TIME);

   /* .. Atmospheric Density */
   if (O->World == EARTH) {
      const double jd_day = JDToDays(jd);
      if (AtmoOption == TWOSIGMA_ATMO) {
         Flux10p7 = LinInterp(SchattenTable[0], SchattenTable[1], jd_day, 1009);
         GeomagIndex =
             LinInterp(SchattenTable[0], SchattenTable[3], jd_day, 1009);
      }
      else if (AtmoOption == NOMINAL_ATMO) {
         Flux10p7 = LinInterp(SchattenTable[0], SchattenTable[2], jd_day, 1009);
         GeomagIndex =
             LinInterp(SchattenTable[0], SchattenTable[4], jd_day, 1009);
      }
      /* else USER_ATMO: Flux10p7, GeomagIndex read from Inp_Sim.txt */

      MxV(World[EARTH].CWN, S->PosN, PosW);
      Alt = MAGV(PosW) - World[EARTH].rad;
      if (Alt < 1000.0E3) { /* What is max alt of MSISE00 validity? */
         S->AtmoDensity =
             NRLMSISE00(date_tt.Year, date_tt.doy, date_tt.Hour, date_tt.Minute,
                        date_tt.Second, PosW, Flux10p7, GeomagIndex);
      }
      else
         S->AtmoDensity = 0.0;
   }

   else if (O->World == MARS) {
      S->AtmoDensity = MarsAtmosphereModel(S->PosN);
   }

   else
      S->AtmoDensity = 0.0;

   /* .. Radiation Belt Electron and Proton Fluxes, particles/cm^2/sec */
#ifdef _RADBELT_
   if (O->World == EARTH) {
      MxV(World[EARTH].CWN, S->PosN, PosW);
      UNITV(PosW);
      MagLat = asin(VoV(PosW, World[EARTH].DipoleAxis));
      RadBelt(MAGV(S->PosN) / 1000.0, fabs(MagLat) * R2D, NumEnergies,
              ElectronEnergy, ProtonEnergy, Flux);
   }
#endif
}

/* #ifdef __cplusplus
** }
** #endif
*/

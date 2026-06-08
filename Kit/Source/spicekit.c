/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "spicekit.h"

#ifndef _ENABLE_SPICE_
// will be quite a few unused variables due to the define replacements
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

/**********************************************************************/
// Do some preconfiguration to interact with spice easier
SpiceInt WorldID2NAIFID(WorldID w_id)
{
   static int first                          = 0;
   static SpiceInt naif_id_list[NMAJORWORLD] = {0};
   if (w_id >= NMAJORWORLD) {
      fprintf(stderr, "WorldID2NAIFID() is not configured to handle the "
                      "user configured minor bodies. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   if (!first) {
      first = 1;
      for (WorldID Iw = SOL; Iw < NMAJORWORLD; Iw++) {
         char world_name[32];
         WorldID2String(Iw, world_name);
         SpiceBoolean found = SPICEFALSE;
         bodn2c_c(world_name, &naif_id_list[Iw], &found);
         if (found == SPICEFALSE) {
            fprintf(stderr, "Could not find NAIF ID for body %s. Exiting...\n",
                    world_name);
            exit(EXIT_FAILURE);
         }
      }
   }
   return naif_id_list[w_id];
}
/**********************************************************************/
void WorldID2IAUFrameWorld(WorldID w_id,
                           char iau_frame[SPICE_FRM_STR_BUFF_SIZE])
{
   static int first = 0;

   static char iau_frame_list[NMAJORWORLD][SPICE_FRM_STR_BUFF_SIZE] = {{'\0'}};
   if (w_id >= NMAJORWORLD) {
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
      for (WorldID Iw = SOL; Iw < NMAJORWORLD; Iw++) {
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
         strcat(iau_frame_list[Iw], world_name);
      }
   }
   strcpy(iau_frame, iau_frame_list[w_id]);
}
/**********************************************************************/
void WorldID2IAUFrame(WorldID w_id, char iau_frame[SPICE_FRM_STR_BUFF_SIZE])
{
   static int first                                                 = 0;
   static char iau_frame_list[NMAJORWORLD][SPICE_FRM_STR_BUFF_SIZE] = {{'\0'}};
   if (w_id >= NMAJORWORLD) {
      fprintf(stderr, "WorldID2IAUFrame() is not configured to handle the user "
                      "configured minor bodies. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   if (!first) {
      first = 1;
      for (WorldID Iw = SOL; Iw < NMAJORWORLD; Iw++) {
         strcpy(iau_frame_list[Iw], "IAU_");
         WorldID2IAUFrameWorld(Iw, &iau_frame_list[Iw][4]);
      }
   }
   strcpy(iau_frame, iau_frame_list[w_id]);
}
/**********************************************************************/
/* Does not modify vals if it is not foudn                            */
int SpiceCheckAndGetDbl(const WorldID Iw, ConstSpiceChar *item, SpiceInt start,
                        SpiceInt n, SpiceDouble *vals)
{
   SpiceBoolean found       = SPICEFALSE;
   SpiceChar check_name[64] = {'\0'};
   SpiceInt dim             = n;
   char type                = 0;

   SpiceInt naif_id = WorldID2NAIFID(Iw);
   sprintf(check_name, "BODY%i_%s", naif_id, item);

   dtpool_c(check_name, &found, &dim, &type);
   if (type == 'C') {
      fprintf(stderr,
              "Variable %s for world %u is character data, not a numeric type "
              "in the Spice kernel pool. Exiting...\n",
              check_name, Iw);
      exit(EXIT_FAILURE);
   }
   if (found != SPICEFALSE) {
      if ((start + n) > dim) {
         fprintf(
             stderr,
             "In SpiceCheckAndGetDbl, requested %i values starting at index %i "
             "from item '%s' when its dimension is %i. Exiting...\n",
             n, start, item, dim);
         exit(EXIT_FAILURE);
      }
      double out[dim];
      bodvcd_c(naif_id, item, dim, &dim, out);
      CopyVG(vals, &out[start], n);
   }

   return found;
}
/**********************************************************************/
/* Compute the fixed frame orientaion of 'world' relative to the      */
/* Ecliptic J2000 frame as CWJ                                        */
static SpiceBoolean _frame_found(WorldID world) __attribute__((const));
static SpiceBoolean _frame_found(WorldID world)
{
   static int frm_found[NMAJORWORLD] = {-1};
   if (frm_found[0] == -1) {
      SpiceChar frm_name[SPICE_FRM_STR_BUFF_SIZE] = {'\0'};
      for (WorldID i = SOL; i < NMAJORWORLD; i++) {
         WorldID2IAUFrame(i, frm_name);
         int found = 0;
         namfrm_c(frm_name, &found);
         frm_found[i] = (found != 0) ? SPICETRUE : SPICEFALSE;
      }
   }
   return frm_found[world];
}

int SpiceGetCWH(const JDType jd_epoch, const WorldID world, double CWH[3][3])
{
   const SpiceBoolean found = _frame_found(world);
   if (found) {
      SpiceChar frm_name[SPICE_FRM_STR_BUFF_SIZE] = {'\0'};
      WorldID2IAUFrame(world, frm_name);

      JDType jd_tdb_j2000 =
          JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd_epoch);
      pxform_c("ECLIPJ2000", frm_name, JDToSeconds(jd_tdb_j2000), CWH);
   }

   return found;
}
/**********************************************************************/
/* Compute the fixed frame orientaion of 'world' relative to the      */
/* J2000 frame as CWN                                                 */
int SpiceGetCWJ(const JDType jd_epoch, const WorldID world, double CWJ[3][3])
{
   const SpiceBoolean found = _frame_found(world);
   if (found) {
      SpiceChar frm_name[SPICE_FRM_STR_BUFF_SIZE] = {'\0'};
      WorldID2IAUFrame(world, frm_name);

      JDType jd_tdb_j2000 =
          JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd_epoch);
      pxform_c("J2000", frm_name, JDToSeconds(jd_tdb_j2000), CWJ);
   }

   return found;
}
/**********************************************************************/
int SpiceGetCWorld(const WorldID from, const WorldID to, const JDType jd_epoch,
                   double C[3][3])
{
   const SpiceBoolean found_v[2] = {_frame_found(from), _frame_found(to)};
   const SpiceBoolean found      = all_int(2, found_v);

   if (found) {
      SpiceChar from_name[SPICE_FRM_STR_BUFF_SIZE] = {'\0'},
                to_name[SPICE_FRM_STR_BUFF_SIZE]   = {'\0'};
      WorldID2IAUFrame(from, from_name);
      WorldID2IAUFrame(to, to_name);
      JDType jd_tdb_j2000 =
          JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd_epoch);
      pxform_c(from_name, to_name, JDToSeconds(jd_tdb_j2000), C);
   }

   return found;
}
/**********************************************************************/
AngDataType SpiceGetAngData(const WorldID world, ConstSpiceChar *item)
{
   AngDataType ang_data = ANGDATATYPE_INVALID;
   if (strncmp("PM", item, 2) && strncmp("RA", item, 2) &&
       strncmp("DEC", item, 3)) {
      fprintf(stderr,
              "SpiceGetAngData only accepts 'item' values of 'PM', 'RA', or "
              "'DEC'; instead got %s. Exiting...\n",
              item);
      exit(EXIT_FAILURE);
   }
   ang_data.ang_char = item[0];

   SpiceChar chk_str[64] = {'\0'};
   if (ang_data.ang_char == 'P')
      strcpy(chk_str, item);
   else
      sprintf(chk_str, "POLE_%s", item);

   SpiceBoolean found = SPICEFALSE, fnd_tmp;

   SpiceInt naif_id = WorldID2NAIFID(world), lead_num = 0;
   SpiceChar num_str[12] = {'\0'};
   sprintf(num_str, "%d", naif_id);
   SpiceChar lead_char[2] = {(naif_id >= 0) ? num_str[0] : num_str[1], '\0'};
   sscanf(lead_char, "%d", &lead_num);

   SpiceInt dim = 3;
   if (SpiceCheckAndGetDbl(world, chk_str, 0, dim, ang_data.ang) ==
       SPICEFALSE) {
      ang_data.n_E          = 0;
      ang_data.n_ang        = 0;
      ang_data.nut_prec_E   = NULL;
      ang_data.nut_prec_ang = NULL;
      // signal to caller to use something else, such as parent orientation data
      return ANGDATATYPE_INVALID;
   }

   SpiceChar E_str[64] = {'\0'}, ang_str[64] = {'\0'}, srch_str[24] = {'\0'};
   sprintf(E_str, "BODY%c_NUT_PREC_ANGLES", lead_char[0]);
   sprintf(srch_str, "NUT_PREC_%s", item);
   sprintf(ang_str, "BODY%d_%s", naif_id, srch_str);

   dtpool_c(E_str, &fnd_tmp, &ang_data.n_E, lead_char);
   ang_data.n_E /= 2;
   dtpool_c(ang_str, &found, &ang_data.n_ang, lead_char);
   found &= fnd_tmp;

   if (found == SPICEFALSE) {
      ang_data.n_E          = 0;
      ang_data.n_ang        = 0;
      ang_data.nut_prec_E   = NULL;
      ang_data.nut_prec_ang = NULL;
      return ang_data;
   }
   ang_data.nut_prec_E   = calloc(ang_data.n_E, sizeof(double[2]));
   ang_data.nut_prec_ang = calloc(ang_data.n_ang, sizeof(double));

   SpiceDouble outdat_ang[ang_data.n_E * 2];
   SpiceDouble outdat_pm[ang_data.n_ang];

   bodvcd_c(lead_num, "NUT_PREC_ANGLES", ang_data.n_E * 2, &ang_data.n_E,
            outdat_ang);
   ang_data.n_E /= 2;
   bodvcd_c(naif_id, srch_str, ang_data.n_ang, &ang_data.n_ang, outdat_pm);

   for (int i = 0; i < ang_data.n_E; i++)
      for (int j = 0; j < 2; j++)
         ang_data.nut_prec_E[i][j] = outdat_ang[j + 2 * i];

   CopyVG(ang_data.nut_prec_ang, outdat_pm, ang_data.n_ang);

   return ang_data;
}
/**********************************************************************/
int SpiceSetOrientation(JDType jd, const WorldID Iw, struct WorldType *const W,
                        double earth_CNH[3][3])
{
   if (!W->OrientWorld)
      return 1;
   double CWJ[3][3];

   jd = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);
   if (Iw == EARTH) {
      /* .. Earth rotation is a special case */
      SpiceGetCWJ(jd, Iw, W->CWN);
      pxform_c("ECLIPJ2000", "J2000", JDToTime(jd), W->CNH);
      for (int i = 0; i < 3; i++) {
         W->qnj[i] = 0.0;
         for (int j = 0; j < 3; j++)
            W->CNJ[i][j] = 0.0;
         W->CNJ[i][i] = 1.0;
      }
      W->qnj[3] = 1.0;
   }
   else {
      GetWorldCNJ(jd, W->ang_data, W->CNJ);
      MxM(W->CNJ, earth_CNH, W->CNH);
      SpiceGetCWJ(jd, Iw, CWJ);
      MxMT(CWJ, W->CNJ, W->CWN);

      C2Q(W->CNJ, W->qnj);
   }
   W->PriMerAng = GetWorldAng(jd, &W->ang_data[0]);

   C2Q(W->CWN, W->qwn);
   C2Q(W->CNH, W->qnh);
   return 1;
}
/**********************************************************************/
void SpicePosN2RLngLat(const double cwn[3][3], const double posn[3], double *r,
                       double *lng, double *lat)
{
   double pw[3] = {0.0};
   MxV(cwn, posn, pw);
   reclat_c(pw, r, lng, lat);
}
/**********************************************************************/
long SpiceLoadKernels(char SpicePath[80])
{
   char MetaKernelPath[256];
   strcpy(MetaKernelPath, SpicePath);
   strcat(MetaKernelPath, "spice_kernels/kernels.txt");

   errprt_c("SET", 1, "ALL");
   furnsh_c(MetaKernelPath);
   return (0);
}
/**********************************************************************/
long SpiceUpdateEphems(const JDType jd, struct WorldType *const worlds)
{
   WorldID Iw, Ip, Im;

   JDType jd_tdb_j2000   = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);
   const double JS       = JDToSeconds(jd_tdb_j2000);
   const double j2000sec = JDToDynTime(jd_tdb_j2000);

   struct OrbitType *Eph;
   struct WorldType *W;
   double Nstate[6];
   double light_time;

   struct WorldType *const earth = &worlds[EARTH];
   SpiceSetOrientation(jd_tdb_j2000, EARTH, earth, earth->CNH);
   SpiceSetOrientation(jd_tdb_j2000, SOL, &worlds[SOL], earth->CNH);

   // Read all planets
   for (Iw = SOL; Iw <= PLUTO; Iw++) {
      if (worlds[Iw].Exists) {
         W   = &worlds[Iw];
         Eph = &W->eph;

         // State of major bodies in Ecliptic J2000 wrt Planet center
         spkez_c(WorldID2NAIFID(Iw), JS, "ECLIPJ2000", "NONE",
                 WorldID2NAIFID(SOL), Nstate, &light_time);

         // Inertial pos & vel (m & m/s)
         SxV(1.0e3, Nstate, Eph->PosN);
         SxV(1.0e3, &Nstate[3], Eph->VelN);

         // Heliocentric pos & vel = inertial pos & vel (m & m/s)
         CopyVG(W->PosH, Eph->PosN, 3);
         CopyVG(W->VelH, Eph->VelN, 3);
      }
   }

   struct WorldType *sol = &worlds[SOL];
   /* Adjust for barycenters */
   /* Move planets from barycentric to Sun-centered */
   /*   (THIS SHOULD NOT BE NEEDED DUE TO FRAMES IN ABOVE LOOP)   */
   for (Iw = PLUTO; Iw >= SOL && Iw <= PLUTO; Iw--) {
      // WorldID is typically unsigned, so (((0)--) >= SOL) can be true
      W = &worlds[Iw];
      axpy(-1.0, sol->eph.PosN, W->eph.PosN, 3);
      axpy(-1.0, sol->eph.VelN, W->eph.VelN, 3);
      CopyVG(W->PosH, W->eph.PosN, 3);
      CopyVG(W->VelH, W->eph.VelN, 3);
   }

   // Read all moons
   for (Ip = MERCURY; Ip <= PLUTO; Ip++) {
      struct WorldType *P = &worlds[Ip];
      if (P->Exists) {
         for (Im = 0; Im < P->Nsat; Im++) {
            Iw  = P->Sat[Im];
            W   = &worlds[Iw];
            Eph = &W->eph;

            if (Iw == LUNA) {
               // State of major bodies in J2000 wrt Planet center
               spkez_c(WorldID2NAIFID(Iw), JS, "J2000", "NONE",
                       WorldID2NAIFID(Ip), Nstate, &light_time);
            }
            else {
               // State of major bodies in Ecliptic J2000 wrt Planet center
               spkez_c(WorldID2NAIFID(Iw), JS, "ECLIPJ2000", "NONE",
                       WorldID2NAIFID(Ip), Nstate, &light_time);
            }

            // Inertial pos & vel (m & m/s)
            SxV(1.0e3, Nstate, Eph->PosN);
            SxV(1.0e3, &Nstate[3], Eph->VelN);

            // Heliocentric pos & vel = inertial pos & vel (m & m/s)
            CopyVG(W->PosH, Eph->PosN, 3);
            CopyVG(W->VelH, Eph->VelN, 3);
            axpy(1.0, P->PosH, W->PosH, 3);
            axpy(1.0, P->VelH, W->VelH, 3);
         }
      }
   }

   for (Iw = SOL; Iw < NMAJORWORLD; Iw++) {
      if (worlds[Iw].Exists) {
         if (Iw != EARTH || Iw != SOL)
            SpiceSetOrientation(jd_tdb_j2000, Iw, W, earth->CNH);
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
/**********************************************************************/
void Rk4SpiceEphems(JDType jd, WorldID trgtWORLD,
                    struct WorldType *const worlds __attribute__((unused)),
                    double trgtPosN[3], double trgtPosH[3],
                    double *trgtPriMerAng __attribute__((unused)),
                    double trgtCNH[3][3])
{
   double CNH[3][3];
   double Nstate[6], Hstate[6];
   double light_time;
   char trgtCNH_STRING[SPICE_FRM_STR_BUFF_SIZE] = {'\0'};
   int i, j;

   jd = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);
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

#ifndef _ENABLE_SPICE_
#pragma GCC diagnostic pop
#endif
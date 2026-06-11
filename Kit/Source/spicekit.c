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
#include "dcmkit.h"
#include "defineskit.h"
#include "iokit.h"
#include <threads.h>

#ifndef _ENABLE_SPICE_
// will be quite a few unused variables due to the define replacements
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

/**********************************************************************/
// Do some preconfiguration to interact with spice easier
static once_flag naif_id_init_flag        = ONCE_FLAG_INIT;
static SpiceInt naif_id_list[NMAJORWORLD] = {0};
void init_naif_id()
{
   for (WorldID Iw = SOL; Iw < NMAJORWORLD; Iw++) {
      const char *world_name = WorldID2String(Iw);
      SpiceBoolean found     = SPICEFALSE;
      bodn2c_c(world_name, &naif_id_list[Iw], &found);
      if (found == SPICEFALSE) {
         fprintf(stderr, "Could not find NAIF ID for body %s. Exiting...\n",
                 world_name);
         exit(EXIT_FAILURE);
      }
   }
}
SpiceInt WorldID2NAIFID(WorldID w_id)
{
   if (w_id >= NMAJORWORLD) {
      fprintf(stderr, "WorldID2NAIFID() is not configured to handle the "
                      "user configured minor bodies. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   call_once(&naif_id_init_flag, init_naif_id);
   return naif_id_list[w_id];
}
/**********************************************************************/
static once_flag iau_frame_init = ONCE_FLAG_INIT;
static char iau_frame_list[NMAJORWORLD][SPICE_FRM_STR_BUFF_SIZE] = {{'\0'}};
void init_iau_frame()
{
   for (WorldID Iw = SOL; Iw < NMAJORWORLD; Iw++) {
      char frame_name[32] = {'\0'};
      WorldID id          = Iw;
      SpiceBoolean found  = FALSE;
      while (!found) {
         if (id == -1) {
            fprintf(stderr,
                    "Unable to find IAU_frame for WorldID %i or any of its "
                    "parents. Exiting...\n",
                    Iw);
            exit(EXIT_FAILURE);
         }
         strcpy(frame_name, "IAU_");
         const char *world_name = WorldID2String(id);
         strcpy(&frame_name[4], world_name);
         namfrm_c(frame_name, &found);
         id = GetWorldParent(id);
      }
      strcpy(iau_frame_list[Iw], frame_name);
   }
}
void WorldID2IAUFrame(WorldID w_id, char iau_frame[SPICE_FRM_STR_BUFF_SIZE])
{
   if (w_id >= NMAJORWORLD) {
      fprintf(stderr, "WorldID2IAUFrame() is not configured to handle the user "
                      "configured minor bodies. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   call_once(&iau_frame_init, init_iau_frame);
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

int SpiceGetCWH(const JDType jd_epoch, const WorldID world, mat3x3_t *CWH)
{
   const SpiceBoolean found = _frame_found(world);
   if (found) {
      SpiceChar frm_name[SPICE_FRM_STR_BUFF_SIZE] = {'\0'};
      WorldID2IAUFrame(world, frm_name);

      JDType jd_tdb_j2000 =
          JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd_epoch);
      pxform_c("ECLIPJ2000", frm_name, JDToSeconds(jd_tdb_j2000), CWH->mat);
   }

   return found;
}
/**********************************************************************/
/* Compute the fixed frame orientaion of 'world' relative to the      */
/* J2000 frame as CWN                                                 */
int SpiceGetCWJ(const JDType jd_epoch, const WorldID world, mat3x3_t *CWJ)
{
   const SpiceBoolean found = _frame_found(world);
   if (found) {
      SpiceChar frm_name[SPICE_FRM_STR_BUFF_SIZE] = {'\0'};
      WorldID2IAUFrame(world, frm_name);

      JDType jd_tdb_j2000 =
          JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd_epoch);
      pxform_c("J2000", frm_name, JDToSeconds(jd_tdb_j2000), CWJ->mat);
   }

   return found;
}
/**********************************************************************/
int SpiceGetCWorld(const WorldID from, const WorldID to, const JDType jd_epoch,
                   mat3x3_t *C)
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
      pxform_c(from_name, to_name, JDToSeconds(jd_tdb_j2000), C->mat);
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
                        mat3x3_t earth_CNH)
{
   if (!W->OrientWorld)
      return 1;
   mat3x3_t CWJ;

   jd = JDChangeSystemEpoch(TDB_TIME, J2000_EPOCH, jd);
   if (Iw == EARTH) {
      /* .. Earth rotation is a special case */
      SpiceGetCWJ(jd, Iw, &W->CWN);
      pxform_c("ECLIPJ2000", "J2000", JDToTime(jd), W->CNH.mat);
      W->qnj = QUAT_EYE;
      W->CNJ = MAT3X3_EYE;
   }
   else {
      W->CNJ = GetWorldCNJ(jd, W->ang_data);
      W->CNH = MxM(W->CNJ, earth_CNH);
      SpiceGetCWJ(jd, Iw, &CWJ);
      W->CWN = MxMT(CWJ, W->CNJ);

      W->qnj = C2Q(W->CNJ);
   }
   W->PriMerAng = GetWorldAng(jd, &W->ang_data[0]);

   W->qwn = C2Q(W->CWN);
   W->qnh = C2Q(W->CNH);
   return 1;
}
/**********************************************************************/
void SpicePosN2RLngLat(const mat3x3_t cwn, const vec3_t posn, double *r,
                       double *lng, double *lat)
{
   vec3_t pw = MxV(cwn, posn);
   reclat_c(pw.v, r, lng, lat);
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
         for (int i = 0; i < 3; i++) {
            Eph->PosN.v[i] = 1.0e3 * Nstate[i];
            Eph->VelN.v[i] = 1.0e3 * Nstate[i + 3];
         }

         // Heliocentric pos & vel = inertial pos & vel (m & m/s)
         W->PosH = Eph->PosN;
         W->VelH = Eph->VelN;
      }
   }

   struct WorldType *sol = &worlds[SOL];
   /* Adjust for barycenters */
   /* Move planets from barycentric to Sun-centered */
   /*   (THIS SHOULD NOT BE NEEDED DUE TO FRAMES IN ABOVE LOOP)   */
   for (Iw = PLUTO; Iw >= SOL && Iw <= PLUTO; Iw--) {
      // WorldID is typically unsigned, so (((0)--) >= SOL) can be true
      W           = &worlds[Iw];
      W->eph.PosN = VSubV_Elem(W->eph.PosN, sol->eph.PosN);
      W->eph.VelN = VSubV_Elem(W->eph.VelN, sol->eph.VelN);
      W->PosH     = W->eph.PosN;
      W->VelH     = W->eph.VelN;
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
            Eph->PosN = SxV(1.0e3, DBL_TO_VEC3(Nstate));
            Eph->VelN = SxV(1.0e3, DBL_TO_VEC3(&Nstate[3]));

            // Heliocentric pos & vel = inertial pos & vel (m & m/s)
            W->PosH = VAddV_Elem(Eph->PosN, P->PosH);
            W->VelH = VAddV_Elem(Eph->VelN, P->VelH);
         }
      }
   }

   for (Iw = SOL; Iw < NMAJORWORLD; Iw++) {
      W = &worlds[Iw];
      if (W->Exists && (Iw != EARTH || Iw != SOL))
         SpiceSetOrientation(jd_tdb_j2000, Iw, W, earth->CNH);
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
                    vec3_t *trgtPosN, vec3_t *trgtPosH,
                    double *trgtPriMerAng __attribute__((unused)),
                    mat3x3_t *trgtCNH)
{
   double Nstate[6], Hstate[6];
   double light_time;
   char trgtCNH_STRING[SPICE_FRM_STR_BUFF_SIZE] = {'\0'};
   int i;

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
         trgtPosH->v[i] = Hstate[i] * 1e3;
         trgtPosN->v[i] = Nstate[i] * 1e3;
      }
   }
   else {
      spkez_c(tgt_world_naif, jd_tdb_j2000_sec, "ECLIPJ2000", "NONE",
              WorldID2NAIFID(SOL), Nstate, &light_time);
      for (i = 0; i < 3; i++) {
         trgtPosH->v[i] = Nstate[i] * 1e3;
         trgtPosN->v[i] = Nstate[i] * 1e3;
      }
   }
   pxform_c("J2000", trgtCNH_STRING, jd_tdb_j2000_sec, trgtCNH->mat);
}

#ifndef _ENABLE_SPICE_
#pragma GCC diagnostic pop
#endif
/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __SPICEKIT_H__
#define __SPICEKIT_H__

#include "jdkit.h"
#include "orbkit.h"

#ifdef _ENABLE_SPICE_
#include "SpiceUsr.h"
#else
// Do some defines to throw warnings if the missing spice functions are called
#define _NO_SPICE_WARN_                                                        \
   do {                                                                        \
      fprintf(stderr,                                                          \
              "******************************** WARNING "                      \
              "********************************\n"                             \
              "You must compile DeepThought with SPICE in order to use "       \
              "SPICE ephemerides.\n\tExiting...\n");                           \
      exit(EXIT_FAILURE);                                                      \
   } while (0)

typedef int SpiceInt;
typedef char SpiceChar;
typedef int SpiceBoolean;
typedef double SpiceDouble;
typedef const char ConstSpiceChar;

#define SPICETRUE     (1)
#define SPICEFALSE    (0)
#define errprt_c(...) _NO_SPICE_WARN_
#define furnsh_c(...) _NO_SPICE_WARN_
#define bodvcd_c(...) _NO_SPICE_WARN_
#define dtpool_c(...) _NO_SPICE_WARN_
#define namfrm_c(...) _NO_SPICE_WARN_
#define pxform_c(...) _NO_SPICE_WARN_
#define bodn2c_c(...) _NO_SPICE_WARN_
#define reclat_c(...) _NO_SPICE_WARN_
#define spkez_c(...)  _NO_SPICE_WARN_
#endif

#define SPICE_FRM_STR_BUFF_SIZE (33)

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

__attribute__((const)) SpiceInt WorldID2NAIFID(WorldID w_id);
void WorldID2IAUFrame(WorldID w_id,
                      SpiceChar iau_frame[SPICE_FRM_STR_BUFF_SIZE]);
int SpiceCheckAndGetDbl(WorldID Iw, ConstSpiceChar *item, SpiceInt start,
                        SpiceInt n, SpiceDouble *vals);
int SpiceGetCWH(const JDType jd_epoch, const WorldID world, mat3x3_t *CWH);
int SpiceGetCWJ(const JDType jd_epoch, const WorldID world, mat3x3_t *CWJ);
int SpiceGetCWorld(const WorldID from, const WorldID to, const JDType jd_epoch,
                   mat3x3_t *C);
__attribute__((pure)) AngDataType SpiceGetAngData(const WorldID world,
                                                  ConstSpiceChar *item);
int SpiceSetOrientation(JDType jd, const WorldID Iw, struct WorldType *const W,
                        mat3x3_t earth_CNH);
void Rk4SpiceEphems(JDType jd, WorldID trgtWORLD,
                    struct WorldType *const worlds, vec3_t *trgtPosN,
                    vec3_t *trgtPosH, double *trgtPriMerAng, mat3x3_t *trgtCNH);
void SpicePosN2RLngLat(const mat3x3_t cwn, const vec3_t posn, double *r,
                       double *lng, double *lat);
/* Load defined SPICE kernels from Model/spice_kernels/kernels.txt */
long SpiceLoadKernels(char SpicePath[80]);
/* Update celestial body locations at TT.JulDay using SPICE*/
long SpiceUpdateEphems(const JDType jd, struct WorldType *const worlds);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __ORBKIT_H__ */
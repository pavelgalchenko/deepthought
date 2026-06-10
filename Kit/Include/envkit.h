/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __ENVKIT_H__
#define __ENVKIT_H__

#include "42constants.h"
#include "42types.h"
#include "dcmkit.h"
#include "defineskit.h"
#include "geomkit.h"
#include "iokit.h"
#include "mathkit.h"
#include "timekit.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

__attribute__((pure)) vec3_t SphericalHarmGravForce(const long N, const long M,
                                                    const struct WorldType *W,
                                                    mat3x3_t CWN,
                                                    const double mass,
                                                    const vec3_t pbn);
__attribute__((pure)) vec3_t IGRFMagField(const char *ModelPath,
                                          const DateType UTC, const long N,
                                          const long M, const vec3_t pbn,
                                          const double PriMerAng);
__attribute__((const)) vec3_t DipoleMagField(double DipoleMoment,
                                             vec3_t DipoleAxis,
                                             vec3_t DipoleOffset, vec3_t p,
                                             double PriMerAng);
__attribute__((const)) double KpToAp(double Kp);
__attribute__((pure)) double JacchiaRoberts(vec3_t pbn, vec3_t svn,
                                            double F10p7, double Ap);
__attribute__((const)) double SimpleMSIS(vec3_t pbn, long Col);
__attribute__((const)) double NRLMSISE00(DateType date, vec3_t PosW,
                                         double F10p7, double AP);
__attribute__((const)) double MarsAtmosphereModel(vec3_t r);
__attribute__((const)) pair_mat3x3_t SimpleEarthPrecNute(const JDType JD);
__attribute__((const)) pair_mat3x3_t HiFiEarthPrecNute(const JDType JD);
__attribute__((const)) vec3_t WGS84ToECEF(double glat, double glong,
                                          double alt);
void ECEFToWGS84(vec3_t p, double *glat, double *glong, double *alt);
long PolyhedronGravAcc(struct GeomType *G, double Density, vec3_t PosN,
                       mat3x3_t CWN, vec3_t *const GravAccN);
long PolyhedronGravGrad(struct GeomType *G, double Density, vec3_t PosN,
                        mat3x3_t CWN, mat3x3_t *const GravGradN);
vec3_t GravGradTimesInertia(mat3x3_t g, mat3x3_t I);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __ENVKIT_H__ */

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

__attribute__((pure)) vec3 SphericalHarmGravForce(const long N, const long M,
                                                  const struct WorldType *W,
                                                  mat3x3 CWN, const double mass,
                                                  const vec3 pbn);
__attribute__((pure)) vec3 IGRFMagField(const char *ModelPath,
                                        const DateType UTC, const long N,
                                        const long M, const vec3 pbn,
                                        const double PriMerAng);
__attribute__((const)) vec3 DipoleMagField(double DipoleMoment, vec3 DipoleAxis,
                                           vec3 DipoleOffset, vec3 p,
                                           double PriMerAng);
__attribute__((const)) double KpToAp(double Kp);
__attribute__((pure)) double JacchiaRoberts(vec3 pbn, vec3 svn, double F10p7,
                                            double Ap);
__attribute__((const)) double SimpleMSIS(vec3 pbn, long Col);
__attribute__((const)) double NRLMSISE00(DateType date, vec3 PosW, double F10p7,
                                         double AP);
__attribute__((const)) double MarsAtmosphereModel(vec3 r);
void SimpleEarthPrecNute(double JD, mat3x3 *const C_TEME_TETE,
                         mat3x3 *const C_TETE_J2000);
void HiFiEarthPrecNute(JDType JD, mat3x3 *C_TEME_TETE, mat3x3 *C_TETE_J2000);
__attribute__((const)) vec3 WGS84ToECEF(double glat, double glong, double alt);
void ECEFToWGS84(vec3 p, double *glat, double *glong, double *alt);
long PolyhedronGravAcc(struct GeomType *G, double Density, vec3 PosN,
                       mat3x3 CWN, vec3 *const GravAccN);
long PolyhedronGravGrad(struct GeomType *G, double Density, vec3 PosN,
                        mat3x3 CWN, mat3x3 *const GravGradN);
vec3 GravGradTimesInertia(mat3x3 g, mat3x3 I);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __ENVKIT_H__ */

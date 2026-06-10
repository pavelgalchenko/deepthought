/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __DCMKIT_H__
#define __DCMKIT_H__

#include "mathkit.h"

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

__attribute__((const)) quat C2Q(const mat3x3 C);
__attribute__((const)) mat3x3 Q2C(const quat Q);
__attribute__((const)) mat3x3 A2C(long SEQ, double TH1, double TH2, double TH3);
void C2A(long SEQ, mat3x3 C, double *TH1, double *TH2, double *TH3);
__attribute__((const)) mat3x3 SimpRot(const vec3 AXIS, const double THETA);
__attribute__((const)) vec3 Q2AngleVec(quat Q);
__attribute__((const)) quat QW2QDOT(const quat Q, const vec3 W);
__attribute__((const)) mat3x3 PARAXIS(mat3x3 IB, mat3x3 CBA, double m,
                                      vec3 pba);
void PrincipalMOI(mat3x3 Ib, vec3 *const Ip, mat3x3 *const CPB);
__attribute__((const)) vec3 Q2W(quat q, quat qdot);
void JointPartials(long Init, long IsSpherical, long RotSeq, long TrnSeq,
                   vec3 ang, vec3 sig, mat3x3 *Gamma, vec3 *Gs, vec3 *Gds,
                   vec3 s, mat3x3 *Delta, vec3 *Ds, vec3 *Dds);
__attribute__((const)) vec3 ADOT2W(long IsSpherical, long Seq, vec3 ang,
                                   vec3 u);
__attribute__((const)) vec3 W2ADOT(long Seq, vec3 ang, vec3 w);
__attribute__((const)) mat3x3 W2CDOT(vec3 w, mat3x3 C);
__attribute__((const)) vec3 CDOT2W(mat3x3 C, mat3x3 Cdot);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __DCMKIT_H__ */

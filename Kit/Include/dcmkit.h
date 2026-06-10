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

__attribute__((const)) quat_t C2Q(const mat3x3_t C);
__attribute__((const)) mat3x3_t Q2C(const quat_t Q);
__attribute__((const)) mat3x3_t A2C(long SEQ, double TH1, double TH2,
                                    double TH3);
__attribute__((const)) vec3_t C2A(long SEQ, mat3x3_t C);
__attribute__((const)) mat3x3_t SimpRot(const vec3_t AXIS, const double THETA);
__attribute__((const)) vec3_t Q2AngleVec(quat_t Q);
__attribute__((const)) quat_t QW2QDOT(const quat_t Q, const vec3_t W);
__attribute__((const)) mat3x3_t PARAXIS(mat3x3_t IB, mat3x3_t CBA, double m,
                                        vec3_t pba);
void PrincipalMOI(mat3x3_t Ib, vec3_t *const Ip, mat3x3_t *const CPB);
__attribute__((const)) vec3_t Q2W(quat_t q, quat_t qdot);
void JointPartials(long Init, long IsSpherical, long RotSeq, long TrnSeq,
                   vec3_t ang, vec3_t sig, mat3x3_t *Gamma, vec3_t *Gs,
                   vec3_t *Gds, vec3_t s, mat3x3_t *Delta, vec3_t *Ds,
                   vec3_t *Dds);
__attribute__((const)) vec3_t ADOT2W(long IsSpherical, long Seq, vec3_t ang,
                                     vec3_t u);
__attribute__((const)) vec3_t W2ADOT(long Seq, vec3_t ang, vec3_t w);
__attribute__((const)) mat3x3_t W2CDOT(vec3_t w, mat3x3_t C);
__attribute__((const)) vec3_t CDOT2W(mat3x3_t C, mat3x3_t Cdot);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __DCMKIT_H__ */

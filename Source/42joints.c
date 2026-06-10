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

/* #ifdef __cplusplus
** namespace _42 {
** using namespace Kit;
** #endif
*/

/**********************************************************************/
void PassiveJoint(struct JointType *G, struct SCType *S __attribute__((unused)))
{
   long i;
   vec3 a;

   if (G->IsSpherical) {
      a = Q2AngleVec(G->q);
      for (i = 0; i < 3; i++) {
         G->Trq.v[i] = -G->RotDampCoef.v[i] * G->AngRate.v[i] -
                       G->RotSpringCoef.v[i] * a.v[i];
      }
   }
   else {
      for (i = 0; i < G->RotDOF; i++) {
         G->Trq.v[i] = -G->RotDampCoef.v[i] * G->AngRate.v[i] -
                       G->RotSpringCoef.v[i] * G->Ang.v[i];
      }
   }

   for (i = 0; i < G->TrnDOF; i++) {
      G->Frc.v[i] = -G->TrnDampCoef.v[i] * G->PosRate.v[i] -
                    G->TrnSpringCoef.v[i] * G->Pos.v[i];
   }
}
/**********************************************************************/
/* Simple actively-controlled joint.                                  */
void ActuatedJoint(struct JointType *G,
                   struct SCType *S __attribute__((unused)))
{
   double RateCmd;
   long i;

   for (i = 0; i < G->RotDOF; i++) {
      RateCmd =
          Limit(G->AngRateCmd.v[i], -G->MaxAngRate.v[i], G->MaxAngRate.v[i]);
      G->Trq.v[i] = Limit(-G->AngRateGain.v[i] * (G->AngRate.v[i] - RateCmd),
                          -G->MaxTrq.v[i], G->MaxTrq.v[i]);
   }

   for (i = 0; i < G->TrnDOF; i++) {
      RateCmd =
          Limit(G->PosRateCmd.v[i], -G->MaxPosRate.v[i], G->MaxPosRate.v[i]);
      G->Frc.v[i] = Limit(-G->PosRateGain.v[i] * (G->PosRate.v[i] - RateCmd),
                          -G->MaxFrc.v[i], G->MaxFrc.v[i]);
   }
}
/**********************************************************************/
void StepperMotorJoint(struct JointType *G __attribute__((unused)),
                       struct SCType *S __attribute__((unused)))
{
}
/**********************************************************************/
void TvcJoint(struct JointType *G __attribute__((unused)),
              struct SCType *S __attribute__((unused)))
{
}
/**********************************************************************/
void VibrationIsolatorJoint(struct JointType *G __attribute__((unused)),
                            struct SCType *S __attribute__((unused)))
{
}
/**********************************************************************/
void SloshJoint(struct JointType *G __attribute__((unused)),
                struct SCType *S __attribute__((unused)))
{
}
/**********************************************************************/
void SteeringMirrorJoint(struct JointType *G __attribute__((unused)),
                         struct SCType *S __attribute__((unused)))
{
}
/**********************************************************************/
/* A good place for you to implement a quick-and-dirty model          */
void AdHocJoint(struct JointType *G, struct SCType *S __attribute__((unused)))
{
   long i;

   if (G->IsSpherical) {
      for (i = 0; i < 3; i++)
         G->Trq.v[i] = 0.0;
   }
   else {
      for (i = 0; i < G->RotDOF; i++)
         G->Trq.v[i] = 0.0;
   }

   for (i = 0; i < G->TrnDOF; i++)
      G->Frc.v[i] = 0.0;
}
/**********************************************************************/
void JointFrcTrq(struct JointType *G, struct SCType *S)
{

   switch (G->Type) {
      case PASSIVE_JOINT:
         PassiveJoint(G, S);
         break;
      case ACTUATED_JOINT:
         ActuatedJoint(G, S);
         break;
      // case STEPPER_MOTOR_JOINT:
      //    StepperMotorJoint(G,S);
      //    break;
      // case TVC_JOINT:
      //    TvcJoint(G,S);
      //    break;
      // case VIBRATION_ISOLATOR_JOINT:
      //    VibrationIsolatorJoint(G,S);
      //    break;
      // case SLOSH_JOINT:
      //    SloshJoint(G,S);
      //    break;
      // case STEERING_MIRROR_JOINT:
      //    SteeringMirrorJoint(G,S);
      //    break;
      case AD_HOC_JOINT:
         AdHocJoint(G, S);
         break;
      default:
         fprintf(stderr,
                 "Unknown joint type %ld in JointFrcTrq.  Bailing out.\n",
                 G->Type);
         exit(EXIT_FAILURE);
   }
}

/* #ifdef __cplusplus
** }
** #endif
*/

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
#include "42dsm.h"

#ifdef _ENABLE_RBT_
void RbtFSW(struct SCType *S);
#endif

void AcFsw(struct AcType *AC);
void WriteToSocket(SOCKET Socket, char **Prefix, long Nprefix,
                   long EchoEnabled);
void ReadFromSocket(SOCKET Socket, long EchoEnabled);

/* #ifdef __cplusplus
** namespace _42 {
** using namespace Kit;
** #endif
*/

/**********************************************************************/
long FswCmdInterpreter(char CmdLine[512], double *CmdTime)
{
   long NewCmdProcessed = FALSE;
   long Isc, Ib, Ig, Iw, It, i, Isct, Ibt, Ithr;
   char response[80];
   char FrameChar;
   long Frame;
   struct CmdType *Cmd;
   struct CmdVecType *CV;
   quat_t q;
   vec3_t Ang, VecR, Vec, VecH;
   mat3x3_t C;
   double RA, Dec;
   double Lng, Lat, Alt;
   double wc, amax, vmax;
   long RotSeq;
   char VecString[20], TargetString[20];
   double ThrPulseCmd;
   double ThrLevelCmd;

   if (sscanf(CmdLine, "%lf SC[%ld] qrn = [%lf %lf %lf %lf]", CmdTime, &Isc,
              &q.x, &q.y, &q.z, &q.s) == 6) {
      NewCmdProcessed = TRUE;
      Cmd             = &SC[Isc].AC.Cmd;
      Cmd->Parm       = PARM_QUATERNION;
      Cmd->Frame      = FRAME_N;
      Cmd->qrn        = q;
   }

   else if (sscanf(CmdLine, "%lf SC[%ld] qrl = [%lf %lf %lf %lf]", CmdTime,
                   &Isc, &q.x, &q.y, &q.z, &q.s) == 6) {
      NewCmdProcessed = TRUE;
      Cmd             = &SC[Isc].AC.Cmd;
      Cmd->Parm       = PARM_QUATERNION;
      Cmd->Frame      = FRAME_L;
      Cmd->qrl        = q;
   }

   else if (sscanf(CmdLine, "%lf SC[%ld] FswTag = %s", CmdTime, &Isc,
                   response) == 3) {
      NewCmdProcessed = TRUE;
      SC[Isc].FswTag  = DecodeString(response);
   }

   else if (sscanf(CmdLine,
                   "%lf SC[%ld] Cmd Angles = [%lf %lf %lf] deg, Seq = %ld wrt "
                   "%c Frame",
                   CmdTime, &Isc, &Ang.z, &Ang.y, &Ang.z, &RotSeq,
                   &FrameChar) == 7) {
      NewCmdProcessed = TRUE;
      Cmd             = &SC[Isc].AC.Cmd;
      Cmd->Parm       = PARM_EULER_ANGLES;
      if (FrameChar == 'L')
         Cmd->Frame = FRAME_L;
      else
         Cmd->Frame = FRAME_N;
      for (i = 0; i < 3; i++)
         Cmd->Ang.v[i] = Ang.v[i] * D2R;
      Cmd->RotSeq = RotSeq;
      C           = A2C(RotSeq, Ang.x * D2R, Ang.y * D2R, Ang.z * D2R);
      if (Cmd->Frame == FRAME_L)
         Cmd->qrl = C2Q(C);
      else
         Cmd->qrn = C2Q(C);
   }

   else if (sscanf(CmdLine, "%lf SC[%ld].G[%ld] Cmd Angles = [%lf %lf %lf] deg",
                   CmdTime, &Isc, &Ig, &Ang.x, &Ang.y, &Ang.z) == 6) {
      NewCmdProcessed = TRUE;
      for (i = 0; i < 3; i++)
         SC[Isc].AC.G[Ig].Cmd.Ang.v[i] = Ang.v[i] * D2R;
   }

   else if (sscanf(CmdLine,
                   "%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at RA = "
                   "%lf deg, Dec = %lf deg",
                   CmdTime, &Isc, &Ib, VecString, &VecR.x, &VecR.y, &VecR.z,
                   &RA, &Dec) == 9) {
      NewCmdProcessed = TRUE;
      if (Ib == 0) {
         Cmd = &SC[Isc].AC.Cmd;
      }
      else {
         Ig  = SC[Isc].B[Ib].Gin;
         Cmd = &SC[Isc].AC.G[Ig].Cmd;
      }
      Cmd->Parm = PARM_VECTORS;
      if (!strcmp(VecString, "Primary"))
         CV = &Cmd->PriVec;
      else
         CV = &Cmd->SecVec;
      CV->Mode  = CMD_DIRECTION;
      CV->Frame = FRAME_N;
      VecR      = UNITV(VecR).v;
      CV->R     = VecR;
      CV->N.x   = cos_deg(RA) * cos_deg(Dec);
      CV->N.y   = sin_deg(RA) * cos_deg(Dec);
      CV->N.z   = sin_deg(Dec);
   }

   else if (sscanf(CmdLine,
                   "%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at "
                   "World[%ld] Lng = %lf deg, Lat = %lf deg, Alt = %lf km",
                   CmdTime, &Isc, &Ib, VecString, &VecR.x, &VecR.y, &VecR.z,
                   &Iw, &Lng, &Lat, &Alt) == 11) {
      NewCmdProcessed = TRUE;
      if (Ib == 0) {
         Cmd = &SC[Isc].AC.Cmd;
      }
      else {
         Ig  = SC[Isc].B[Ib].Gin;
         Cmd = &SC[Isc].AC.G[Ig].Cmd;
      }
      Cmd->Parm  = PARM_VECTORS;
      Cmd->Frame = FRAME_N;
      if (!strcmp(VecString, "Primary"))
         CV = &Cmd->PriVec;
      else
         CV = &Cmd->SecVec;
      CV->Mode     = CMD_TARGET;
      CV->Frame    = FRAME_N;
      CV->TrgType  = TARGET_WORLD;
      CV->TrgWorld = Iw;
      VecR         = UNITV(VecR).v;
      CV->R        = VecR;
      CV->W.x = (World[Iw].rad + 1000.0 * Alt) * cos_deg(Lng) * cos_deg(Lat);
      CV->W.y = (World[Iw].rad + 1000.0 * Alt) * sin_deg(Lng) * cos_deg(Lat);
      CV->W.z = (World[Iw].rad + 1000.0 * Alt) * sin_deg(Lat);
   }

   else if (sscanf(CmdLine,
                   "%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at "
                   "World[%ld]",
                   CmdTime, &Isc, &Ib, VecString, &VecR.x, &VecR.y, &VecR.z,
                   &Iw) == 8) {
      NewCmdProcessed = TRUE;
      if (Ib == 0)
         Cmd = &SC[Isc].AC.Cmd;
      else {
         Ig  = SC[Isc].B[Ib].Gin;
         Cmd = &SC[Isc].AC.G[Ig].Cmd;
      }
      Cmd->Parm  = PARM_VECTORS;
      Cmd->Frame = FRAME_N;
      if (!strcmp(VecString, "Primary"))
         CV = &Cmd->PriVec;
      else
         CV = &Cmd->SecVec;
      CV->Mode     = CMD_TARGET;
      CV->Frame    = FRAME_N;
      CV->TrgType  = TARGET_WORLD;
      CV->TrgWorld = Iw;
      VecR         = UNITV(VecR).v;
      CV->R        = VecR;
      CV->W        = VEC3_ZERO;
   }

   else if (sscanf(CmdLine,
                   "%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at "
                   "GroundStation[%ld]",
                   CmdTime, &Isc, &Ib, VecString, &VecR.x, &VecR.y, &VecR.z,
                   &It) == 8) {
      NewCmdProcessed = TRUE;
      if (Ib == 0) {
         Cmd = &SC[Isc].AC.Cmd;
      }
      else {
         Ig  = SC[Isc].B[Ib].Gin;
         Cmd = &SC[Isc].AC.G[Ig].Cmd;
      }
      Cmd->Parm  = PARM_VECTORS;
      Cmd->Frame = FRAME_N;
      if (!strcmp(VecString, "Primary"))
         CV = &Cmd->PriVec;
      else
         CV = &Cmd->SecVec;
      CV->Mode     = CMD_TARGET;
      CV->Frame    = FRAME_N;
      CV->TrgType  = TARGET_WORLD;
      CV->TrgWorld = GroundStation[It].World;
      VecR         = UNITV(VecR).v;
      CV->R        = VecR;
      CV->W        = GroundStation[It].PosW;
   }

   else if (sscanf(CmdLine,
                   "%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at "
                   "SC[%ld].B[%ld] point [%lf %lf %lf]",
                   CmdTime, &Isc, &Ib, VecString, &VecR.x, &VecR.y, &VecR.z,
                   &Isct, &Ibt, &Vec.x, &Vec.y, &Vec.z) == 12) {
      NewCmdProcessed = TRUE;
      if (Ib == 0) {
         Cmd = &SC[Isc].AC.Cmd;
      }
      else {
         Ig  = SC[Isc].B[Ib].Gin;
         Cmd = &SC[Isc].AC.G[Ig].Cmd;
      }
      Cmd->Parm  = PARM_VECTORS;
      Cmd->Frame = FRAME_N;
      if (!strcmp(VecString, "Primary"))
         CV = &Cmd->PriVec;
      else
         CV = &Cmd->SecVec;
      CV->Mode    = CMD_TARGET;
      CV->Frame   = FRAME_N;
      CV->TrgType = TARGET_BODY;
      CV->TrgSC   = Isct;
      CV->TrgBody = Ibt;
      CV->R       = UNITV(VecR).v;
      CV->T       = Vec;
   }

   else if (sscanf(
                CmdLine,
                "%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at SC[%ld]",
                CmdTime, &Isc, &Ib, VecString, &VecR.x, &VecR.y, &VecR.z,
                &Isct) == 8) {
      NewCmdProcessed = TRUE;
      if (Ib == 0) {
         Cmd = &SC[Isc].AC.Cmd;
      }
      else {
         Ig  = SC[Isc].B[Ib].Gin;
         Cmd = &SC[Isc].AC.G[Ig].Cmd;
      }
      Cmd->Parm  = PARM_VECTORS;
      Cmd->Frame = FRAME_N;
      if (!strcmp(VecString, "Primary"))
         CV = &Cmd->PriVec;
      else
         CV = &Cmd->SecVec;
      CV->Mode    = CMD_TARGET;
      CV->Frame   = FRAME_N;
      CV->TrgType = TARGET_SC;
      CV->TrgSC   = Isct;
      CV->R       = UNITV(VecR).v;
   }

   else if (sscanf(CmdLine,
                   "%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at %s",
                   CmdTime, &Isc, &Ib, VecString, &VecR.x, &VecR.y, &VecR.z,
                   TargetString) == 8) {
      NewCmdProcessed = TRUE;
      if (Ib == 0) {
         Cmd = &SC[Isc].AC.Cmd;
      }
      else {
         Ig  = SC[Isc].B[Ib].Gin;
         Cmd = &SC[Isc].AC.G[Ig].Cmd;
      }
      Cmd->Parm  = PARM_VECTORS;
      Cmd->Frame = FRAME_N;
      if (!strcmp(VecString, "Primary"))
         CV = &Cmd->PriVec;
      else
         CV = &Cmd->SecVec;
      CV->Mode  = CMD_TARGET;
      CV->Frame = FRAME_N;
      if (!strcmp(TargetString, "EARTH")) {
         CV->TrgType  = TARGET_WORLD;
         CV->TrgWorld = EARTH;
      }
      else if (!strcmp(TargetString, "MOON")) {
         CV->TrgType  = TARGET_WORLD;
         CV->TrgWorld = LUNA;
      }
      else if (!strcmp(TargetString, "LUNA")) {
         CV->TrgType  = TARGET_WORLD;
         CV->TrgWorld = LUNA;
      }
      else if (!strcmp(TargetString, "MERCURY")) {
         CV->TrgType  = TARGET_WORLD;
         CV->TrgWorld = MERCURY;
      }
      else if (!strcmp(TargetString, "VENUS")) {
         CV->TrgType  = TARGET_WORLD;
         CV->TrgWorld = VENUS;
      }
      else if (!strcmp(TargetString, "MARS")) {
         CV->TrgType  = TARGET_WORLD;
         CV->TrgWorld = MARS;
      }
      else if (!strcmp(TargetString, "JUPITER")) {
         CV->TrgType  = TARGET_WORLD;
         CV->TrgWorld = JUPITER;
      }
      else if (!strcmp(TargetString, "SATURN")) {
         CV->TrgType  = TARGET_WORLD;
         CV->TrgWorld = SATURN;
      }
      else if (!strcmp(TargetString, "URANUS")) {
         CV->TrgType  = TARGET_WORLD;
         CV->TrgWorld = URANUS;
      }
      else if (!strcmp(TargetString, "NEPTUNE")) {
         CV->TrgType  = TARGET_WORLD;
         CV->TrgWorld = NEPTUNE;
      }
      else if (!strcmp(TargetString, "PLUTO")) {
         CV->TrgType  = TARGET_WORLD;
         CV->TrgWorld = PLUTO;
      }
      else if (!strcmp(TargetString, "VELOCITY")) {
         CV->TrgType = TARGET_VELOCITY;
      }
      else if (!strcmp(TargetString, "MAGFIELD")) {
         CV->TrgType = TARGET_MAGFIELD;
      }
      else if (!strcmp(TargetString, "TDRS")) {
         CV->TrgType = TARGET_TDRS;
      }
      else {
         CV->TrgType  = TARGET_WORLD;
         CV->TrgWorld = SOL;
      }
      VecR  = UNITV(VecR).v;
      CV->R = VecR;
      CV->W = VEC3_ZERO;
   }

   else if (sscanf(CmdLine,
                   "%lf Align SC[%ld].B[%ld] %s Vector [%lf %lf %lf] with "
                   "SC[%ld].B[%ld] vector [%lf %lf %lf]",
                   CmdTime, &Isc, &Ib, VecString, &VecR.x, &VecR.y, &VecR.z,
                   &Isct, &Ibt, &Vec.x, &Vec.y, &Vec.z) == 12) {
      NewCmdProcessed = TRUE;
      if (Ib == 0) {
         Cmd = &SC[Isc].AC.Cmd;
      }
      else {
         Ig  = SC[Isc].B[Ib].Gin;
         Cmd = &SC[Isc].AC.G[Ig].Cmd;
      }
      Cmd->Parm  = PARM_VECTORS;
      Cmd->Frame = FRAME_N;
      if (!strcmp(VecString, "Primary"))
         CV = &Cmd->PriVec;
      else
         CV = &Cmd->SecVec;
      CV->Mode    = CMD_DIRECTION;
      CV->Frame   = FRAME_B;
      CV->TrgType = TARGET_BODY;
      CV->TrgSC   = Isct;
      CV->TrgBody = Ibt;
      CV->R       = UNITV(VecR).v;
      CV->T       = Vec;
   }

   else if (sscanf(CmdLine,
                   "%lf Align SC[%ld].B[%ld] %s Vector [%lf %lf %lf] with "
                   "%c-frame Vector [%lf %lf %lf]",
                   CmdTime, &Isc, &Ib, VecString, &VecR.x, &VecR.y, &VecR.z,
                   &FrameChar, &Vec.x, &Vec.y, &Vec.z) == 11) {
      NewCmdProcessed = TRUE;
      if (FrameChar == 'L')
         Frame = FRAME_L;
      else if (FrameChar == 'H') {
         Frame = FRAME_N;
         VecH  = Vec;
         Vec   = MxV(World[Orb[SC[Isc].RefOrb].World].CNH, VecH);
      }
      else
         Frame = FRAME_N;
      if (Ib == 0) {
         Cmd = &SC[Isc].AC.Cmd;
      }
      else {
         Ig  = SC[Isc].B[Ib].Gin;
         Cmd = &SC[Isc].AC.G[Ig].Cmd;
      }
      Cmd->Parm = PARM_VECTORS;
      if (!strcmp(VecString, "Primary"))
         CV = &Cmd->PriVec;
      else
         CV = &Cmd->SecVec;
      CV->Mode  = CMD_DIRECTION;
      CV->Frame = Frame;
      VecR      = UNITV(VecR).v;
      Vec       = UNITV(Vec).v;
      CV->R     = VecR;
      if (Frame == FRAME_L)
         CV->L = Vec;
      else
         CV->N = Vec;
   }

   else if (sscanf(CmdLine, "%lf SC[%ld].AC.Thr[%ld].PulseWidthCmd = %lf",
                   CmdTime, &Isc, &Ithr, &ThrPulseCmd) == 4) {
      NewCmdProcessed                    = TRUE;
      SC[Isc].AC.Thr[Ithr].PulseWidthCmd = ThrPulseCmd;
      SC[Isc].AC.Thr[Ithr].PulseWidthFinTimeStamp =
          JDAddSeconds(JD_TT_MJD, ThrPulseCmd);
   }

   else if (sscanf(CmdLine, "%lf SC[%ld].AC.Thr[%ld].ThrustLevelCmd = %lf",
                   CmdTime, &Isc, &Ithr, &ThrLevelCmd) == 4) {
      NewCmdProcessed                     = TRUE;
      SC[Isc].AC.Thr[Ithr].ThrustLevelCmd = ThrLevelCmd;
   }

   else if (sscanf(CmdLine,
                   "Event Eclipse Entry SC[%ld] qrl = [%lf %lf %lf %lf]", &Isc,
                   &q.x, &q.y, &q.z, &q.s) == 5) {
      *CmdTime =
          SimTime + DTSIM;   /* Allows exiting while loop in CmdInterpreter */
      if (SC[Isc].Eclipse) { /* Will pend on this command until this condition
                                is true */
         NewCmdProcessed = TRUE;
         Cmd             = &SC[Isc].AC.Cmd;
         Cmd->Parm       = PARM_QUATERNION;
         Cmd->Frame      = FRAME_L;
         Cmd->qrl        = q;
      }
   }
   else if (sscanf(CmdLine,
                   "Event Eclipse Exit SC[%ld] qrl = [%lf %lf %lf %lf]", &Isc,
                   &q.x, &q.y, &q.z, &q.s) == 5) {
      *CmdTime =
          SimTime + DTSIM;    /* Allows exiting while loop in CmdInterpreter */
      if (!SC[Isc].Eclipse) { /* Will pend on this command until this condition
                                 is true */
         NewCmdProcessed = TRUE;
         Cmd             = &SC[Isc].AC.Cmd;
         Cmd->Parm       = PARM_QUATERNION;
         Cmd->Frame      = FRAME_L;
         Cmd->qrl        = q;
      }
   }

   else if (sscanf(CmdLine,
                   "Event Eclipse Entry SC[%ld] Cmd Angles = [%lf %lf %lf] "
                   "deg, Seq = %ld wrt %c Frame",
                   &Isc, &Ang.x, &Ang.y, &Ang.z, &RotSeq, &FrameChar) == 6) {
      *CmdTime =
          SimTime + DTSIM;   /* Allows exiting while loop in CmdInterpreter */
      if (SC[Isc].Eclipse) { /* Will pend on this command until this condition
                                is true */
         NewCmdProcessed = TRUE;
         Cmd             = &SC[Isc].AC.Cmd;
         Cmd->Parm       = PARM_EULER_ANGLES;
         if (FrameChar == 'L')
            Cmd->Frame = FRAME_L;
         else
            Cmd->Frame = FRAME_N;
         for (i = 0; i < 3; i++)
            Cmd->Ang.v[i] = Ang.v[i] * D2R;
         Cmd->RotSeq = RotSeq;
         C           = A2C(RotSeq, Ang.x * D2R, Ang.y * D2R, Ang.z * D2R);
         if (Cmd->Frame == FRAME_L)
            Cmd->qrl = C2Q(C);
         else
            Cmd->qrn = C2Q(C);
      }
   }

   else if (sscanf(CmdLine,
                   "Event Eclipse Exit SC[%ld] Cmd Angles = [%lf %lf %lf] deg, "
                   "Seq = %ld wrt %c Frame",
                   &Isc, &Ang.x, &Ang.y, &Ang.z, &RotSeq, &FrameChar) == 6) {
      *CmdTime =
          SimTime + DTSIM;    /* Allows exiting while loop in CmdInterpreter */
      if (!SC[Isc].Eclipse) { /* Will pend on this command until this condition
                                 is true */
         NewCmdProcessed = TRUE;
         Cmd             = &SC[Isc].AC.Cmd;
         Cmd->Parm       = PARM_EULER_ANGLES;
         if (FrameChar == 'L')
            Cmd->Frame = FRAME_L;
         else
            Cmd->Frame = FRAME_N;
         for (i = 0; i < 3; i++)
            Cmd->Ang.v[i] = Ang.v[i] * D2R;
         Cmd->RotSeq = RotSeq;
         C           = A2C(RotSeq, Ang.x * D2R, Ang.y * D2R, Ang.z * D2R);
         if (Cmd->Frame == FRAME_L)
            Cmd->qrl = C2Q(C);
         else
            Cmd->qrn = C2Q(C);
      }
   }

   else if (sscanf(CmdLine,
                   "%lf Set SC[%ld] RampCoastGlide wc = %lf Hz, amax = %lf, "
                   "vmax = %lf",
                   CmdTime, &Isc, &wc, &amax, &vmax) == 5) {
      NewCmdProcessed               = TRUE;
      SC[Isc].AC.PrototypeCtrl.wc   = wc * TwoPi;
      SC[Isc].AC.PrototypeCtrl.amax = amax;
      SC[Isc].AC.PrototypeCtrl.vmax = vmax;
   }

   else if (sscanf(CmdLine,
                   "%lf Spin SC[%ld] about Primary Vector at %lf deg/sec",
                   CmdTime, &Isc, &wc) == 3) {
      NewCmdProcessed = TRUE;
      Cmd             = &SC[Isc].AC.Cmd;

      Cmd->Parm     = PARM_AXIS_SPIN;
      Cmd->SpinRate = wc * D2R;
   }

   return (NewCmdProcessed);
}
/**********************************************************************/
/* Given a relative position and velocity vector, find the angular    */
/* velocity at which the relative position vector is rotating.        */
vec3_t RelMotionToAngRate(vec3_t RelPosN, vec3_t RelVelN)
    __attribute__((const));
vec3_t RelMotionToAngRate(vec3_t RelPosN, vec3_t RelVelN)
{
   double magp, Vpar, magvp;
   vec3_t phat, Axis, Vperp, wn;
   long i;

   magvec3_t uv = UNITV(RelPosN);
   magp         = uv.m;
   phat         = uv.v;

   Axis = VxV(RelPosN, RelVelN);
   Axis = UNITV(Axis).v;

   Vpar = VoV(RelVelN, phat);
   for (i = 0; i < 3; i++)
      Vperp.v[i] = RelVelN.v[i] - Vpar * phat.v[i];
   magvp = MAGV(Vperp);

   wn = VEC3_ZERO;
   for (i = 0; i < 3; i++)
      wn.v[i] += magvp / magp * Axis.v[i];
   return wn;
}
/**********************************************************************/
struct CmdVecType FindCmdVecN(struct SCType *S, struct CmdVecType CV)
{
   struct WorldType *W;
   vec3_t RelPosB, vb, Rhat;
   vec3_t RelPosN, RelPosH, RelVelN, RelVelH;
   vec3_t pcmn, pn, vn, ph, vh;
   double CosPriMerAng, SinPriMerAng;
   double MaxToS, ToS;
   long It, i;

   switch (CV.TrgType) {
      case TARGET_WORLD:
         W            = &World[CV.TrgWorld];
         CosPriMerAng = cos(W->PriMerAng);
         SinPriMerAng = sin(W->PriMerAng);
         pn.x         = CV.W.x * CosPriMerAng - CV.W.y * SinPriMerAng;
         pn.y         = CV.W.x * SinPriMerAng + CV.W.y * CosPriMerAng;
         pn.z         = CV.W.z;
         vn.x         = -CV.W.x * SinPriMerAng - CV.W.y * CosPriMerAng;
         vn.y         = CV.W.x * CosPriMerAng - CV.W.y * SinPriMerAng;
         vn.z         = 0.0;
         if (CV.TrgWorld == Orb[SC->RefOrb].World) {
            for (i = 0; i < 3; i++) {
               RelPosN.v[i] = pn.v[i] - S->PosN.v[i];
               RelVelN.v[i] = vn.v[i] - S->VelN.v[i];
            }
         }
         else {
            ph = MTxV(W->CNH, pn);
            vh = MTxV(W->CNH, vn);
            for (i = 0; i < 3; i++) {
               RelPosH.v[i] = (W->PosH.v[i] + ph.v[i]) - S->PosH.v[i];
               RelVelH.v[i] = (W->VelH.v[i] + vh.v[i]) - S->VelH.v[i];
            }
            RelPosN = MxV(World[Orb[S->RefOrb].World].CNH, RelPosH);
            RelVelN = MxV(World[Orb[S->RefOrb].World].CNH, RelVelH);
         }
         CV.N  = UNITV(RelPosN).v;
         CV.wn = RelMotionToAngRate(RelPosN, RelVelN);
         break;
      case TARGET_SC:
         if (SC[CV.TrgSC].RefOrb == S->RefOrb) {
            for (i = 0; i < 3; i++) {
               RelPosN.v[i] = SC[CV.TrgSC].PosR.v[i] - S->PosR.v[i];
               RelVelN.v[i] = SC[CV.TrgSC].VelR.v[i] - S->VelR.v[i];
            }
         }
         else if (Orb[SC[CV.TrgSC].RefOrb].World == Orb[S->RefOrb].World) {
            for (i = 0; i < 3; i++) {
               RelPosN.v[i] = SC[CV.TrgSC].PosN.v[i] - S->PosN.v[i];
               RelVelN.v[i] = SC[CV.TrgSC].VelN.v[i] - S->VelN.v[i];
            }
         }
         else {
            for (i = 0; i < 3; i++) {
               RelPosH.v[i] = SC[CV.TrgSC].PosH.v[i] - S->PosH.v[i];
               RelVelH.v[i] = SC[CV.TrgSC].VelH.v[i] - S->VelH.v[i];
            }
            RelPosN = MxV(World[Orb[S->RefOrb].World].CNH, RelPosH);
            RelVelN = MxV(World[Orb[S->RefOrb].World].CNH, RelVelH);
         }
         CV.N  = UNITV(RelPosN).v;
         CV.wn = RelMotionToAngRate(RelPosN, RelVelN);
         break;
      case TARGET_BODY:
         pcmn = MTxV(SC[CV.TrgSC].B[0].CN, SC[CV.TrgSC].cm);
         pn   = MTxV(SC[CV.TrgSC].B[CV.TrgBody].CN, CV.T);
         for (i = 0; i < 3; i++)
            RelPosB.v[i] = CV.T.v[i] - SC[CV.TrgSC].B[CV.TrgBody].cm.v[i];
         vb = VxV(SC[CV.TrgSC].B[CV.TrgBody].wn, RelPosB);
         vn = MTxV(SC[CV.TrgSC].B[CV.TrgBody].CN, vb);
         for (i = 0; i < 3; i++) {
            pn.v[i] += SC[CV.TrgSC].B[CV.TrgBody].pn.v[i] - pcmn.v[i];
            vn.v[i] += SC[CV.TrgSC].B[CV.TrgBody].vn.v[i];
         }
         if (SC[CV.TrgSC].RefOrb == S->RefOrb) {
            for (i = 0; i < 3; i++) {
               RelPosN.v[i] = SC[CV.TrgSC].PosR.v[i] + pn.v[i] - S->PosR.v[i];
               RelVelN.v[i] = SC[CV.TrgSC].VelR.v[i] + vn.v[i] - S->VelR.v[i];
            }
         }
         else if (Orb[SC[CV.TrgSC].RefOrb].World == Orb[S->RefOrb].World) {
            for (i = 0; i < 3; i++) {
               RelPosN.v[i] = SC[CV.TrgSC].PosN.v[i] + pn.v[i] - S->PosN.v[i];
               RelVelN.v[i] = SC[CV.TrgSC].VelN.v[i] + vn.v[i] - S->VelN.v[i];
            }
         }
         else {
            ph = MTxV(World[Orb[SC[CV.TrgSC].RefOrb].World].CNH, pn);
            vh = MTxV(World[Orb[SC[CV.TrgSC].RefOrb].World].CNH, vn);
            for (i = 0; i < 3; i++) {
               RelPosH.v[i] = SC[CV.TrgSC].PosH.v[i] + ph.v[i] - S->PosH.v[i];
               RelVelH.v[i] = SC[CV.TrgSC].VelH.v[i] + vh.v[i] - S->VelH.v[i];
            }
            RelPosN = MxV(World[Orb[S->RefOrb].World].CNH, RelPosH);
            RelVelN = MxV(World[Orb[S->RefOrb].World].CNH, RelVelH);
         }
         CV.N  = UNITV(RelPosN).v;
         CV.wn = RelMotionToAngRate(RelPosN, RelVelN);
         break;
      case TARGET_VELOCITY:
         CV.N = S->VelN;
         CV.N = UNITV(CV.N).v;
         break;
      case TARGET_MAGFIELD:
         CV.N = S->bvn;
         CV.N = UNITV(CV.N).v;
         break;
      case TARGET_TDRS:
         CV.N   = VEC3_PZAXIS;
         CV.wn  = VEC3_ZERO;
         MaxToS = -2.0; /* Bogus */
         Rhat   = UNITV(S->PosN).v;
         /* Aim at TDRS closest to Zenith */
         for (It = 0; It < 10; It++) {
            if (Tdrs[It].Exists) {
               for (i = 0; i < 3; i++)
                  RelPosN.v[i] = Tdrs[It].PosN.v[i] - S->PosN.v[i];
               RelPosN = UNITV(RelPosN).v;
               ToS     = VoV(RelPosN, Rhat);
               if (ToS > MaxToS) {
                  MaxToS = ToS;
                  CV.N   = RelPosN;
               }
            }
         }
         break;
      default:
         break;
   }
   return CV;
}
/**********************************************************************/
void ThreeAxisAttitudeCommand(struct SCType *S)
{
   struct JointType *G;
   struct BodyType *B;
   struct CmdType *Cmd;
   struct CmdVecType *PV, *SV;
   mat3x3_t CRN, C, Cdot, CGoGi;
   vec3_t PriVecBi, SecVecBi, PriVecGi, SecVecGi, PriVecGo, SecVecGo;
   quat_t qln;
   long Ig, Bi, i, j;

   Cmd = &S->AC.Cmd;
   PV  = &Cmd->PriVec;
   SV  = &Cmd->SecVec;

   switch (Cmd->Parm) {
      case PARM_EULER_ANGLES:
         C = A2C(Cmd->RotSeq, Cmd->Ang.x, Cmd->Ang.y, Cmd->Ang.z);
         if (Cmd->Frame == FRAME_L)
            Cmd->qrl = C2Q(C);
         else
            Cmd->qrn = C2Q(C);
         [[fallthrough]];
      case PARM_QUATERNION:
         qln = C2Q(S->CLN);
         if (Cmd->Frame == FRAME_L) {
            Cmd->qrn = QxQ(Cmd->qrl, qln);
            Cmd->wrn = QxV(Cmd->qrn, S->wln);
         }
         break;
      case PARM_VECTORS:
         if (PV->Mode == CMD_TARGET)
            *PV = FindCmdVecN(S, *PV);
         else if (PV->Frame == FRAME_N)
            PV->wn = VEC3_ZERO;

         else if (PV->Frame == FRAME_L) {
            PV->N  = MTxV(S->CLN, PV->L);
            PV->wn = S->wln;
         }
         else if (PV->Frame == FRAME_B) {
            PV->N  = MTxV(SC[PV->TrgSC].B[PV->TrgBody].CN, PV->T);
            PV->wn = MTxV(SC[PV->TrgSC].B[PV->TrgBody].CN,
                          SC[PV->TrgSC].B[PV->TrgBody].wn);
         }

         if (SV->Mode == CMD_TARGET)
            *SV = FindCmdVecN(S, *SV);
         else if (SV->Frame == FRAME_N)
            SV->wn = VEC3_ZERO;
         else if (SV->Frame == FRAME_L) {
            SV->N  = MTxV(S->CLN, SV->L);
            SV->wn = S->wln;
         }
         else if (SV->Frame == FRAME_B) {
            SV->N  = MTxV(SC[SV->TrgSC].B[SV->TrgBody].CN, SV->T);
            SV->wn = MTxV(SC[SV->TrgSC].B[SV->TrgBody].CN,
                          SC[SV->TrgSC].B[SV->TrgBody].wn);
         }
         if (MAGV(PV->N) == 0.0 || MAGV(PV->R) == 0.0)
            printf("Warning: Primary Vector not defined for SC[%ld]\n", S->ID);
         if (MAGV(SV->N) == 0.0 || MAGV(SV->R) == 0.0)
            printf("Warning: Secondary Vector not defined for SC[%ld]\n",
                   S->ID);
         CRN      = TRIAD(PV->N, SV->N, PV->R, SV->R);
         Cmd->qrn = C2Q(CRN);
         for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
               Cdot.mat[i][j] =
                   (CRN.mat[i][j] - Cmd->OldCRN.mat[i][j]) / S->AC.DT;
            }
         }
         Cmd->wrn    = CDOT2W(CRN, Cdot);
         Cmd->OldCRN = CRN;

         break;
      default:
         break;
   }

   for (Ig = 0; Ig < S->Ng; Ig++) {
      G   = &S->G[Ig];
      Bi  = G->Bin;
      B   = &S->B[Bi];
      Cmd = &S->AC.G[Ig].Cmd;
      PV  = &Cmd->PriVec;
      SV  = &Cmd->SecVec;

      if (Cmd->Parm == PARM_VECTORS) {
         if (PV->Mode == CMD_TARGET)
            *PV = FindCmdVecN(S, *PV);
         else if (PV->Frame == FRAME_L)
            PV->N = MTxV(S->CLN, PV->L);
         if (SV->Mode == CMD_TARGET)
            *SV = FindCmdVecN(S, *SV);
         else if (SV->Frame == FRAME_L)
            SV->N = MTxV(S->CLN, SV->L);

         if (G->RotDOF == 3) {
            PriVecBi = MxV(B->CN, PV->N);
            SecVecBi = MxV(B->CN, SV->N);
            PriVecGi = MxV(G->CGiBi, PriVecBi);
            SecVecGi = MxV(G->CGiBi, SecVecBi);
            PriVecGo = MTxV(G->CBoGo, PV->R);
            SecVecGo = MTxV(G->CBoGo, SV->R);
            CGoGi    = TRIAD(PriVecGi, SecVecGi, PriVecGo, SecVecGo);
            Cmd->Ang = C2A(G->RotSeq, CGoGi);
         }
         else {
            PriVecBi = MxV(B->CN, PV->N);
            Cmd->Ang = PointGimbalToTarget(G->RotSeq, G->CGiBi, G->CBoGo,
                                           PriVecBi, PV->R);
         }
      }
   }
}
/**********************************************************************/
void SpinnerCommand(struct SCType *S)
{
   struct CmdType *Cmd;
   struct CmdVecType *PV;
   double MagH;
   long i;

   Cmd = &S->AC.Cmd;
   PV  = &Cmd->PriVec;

   if (PV->Frame != FRAME_N) {
      fprintf(stderr,
              "SpinnerCommand requires that Primary Vector be fixed in N\n");
      exit(EXIT_FAILURE);
   }

   *PV = FindCmdVecN(S, *PV);
   for (i = 0; i < 3; i++) {
      Cmd->wrn.v[i] = PV->R.v[i] * Cmd->SpinRate;
   }
   Cmd->Hvr = MxV(S->I, Cmd->wrn);
   MagH     = MAGV(Cmd->Hvr);
   for (i = 0; i < 3; i++) {
      Cmd->Hvn.v[i] = PV->N.v[i] * MagH;
   }
}
/**********************************************************************/
/* This function copies needed parameters from the SC structure to    */
/* the AC structure.                                                 */
void InitAC(struct SCType *S)
{
   long Ib, Ig, i, j, k;
   struct AcType *AC;
   double **A, **Aplus;
   vec3_t r;

   AC = &S->AC;

   S->InitAC = 0;
   AC->Init  = 1;

   AC->ID = S->ID;

   /* Fundamental Constants */
   AC->Pi    = PI;
   AC->TwoPi = TWOPI;

   /* Time, Mass */
   AC->DT   = S->FswSampleTime;
   AC->mass = S->mass;
   AC->cm   = S->cm;
   AC->MOI  = S->I;

   /* Bodies */
   AC->Nb = S->Nb;
   if (AC->Nb > 0) {
      AC->B = (struct AcBodyType *)calloc(AC->Nb, sizeof(struct AcBodyType));
      for (Ib = 0; Ib < AC->Nb; Ib++) {
         AC->B[Ib].mass = S->B[Ib].mass;
         AC->B[Ib].cm   = S->B[Ib].cm;
         AC->B[Ib].MOI  = S->B[Ib].I;
      }
   }

   /* Joints */
   AC->Ng = S->Ng;
   if (AC->Ng > 0) {
      AC->G = (struct AcJointType *)calloc(AC->Ng, sizeof(struct AcJointType));
      for (Ig = 0; Ig < AC->Ng; Ig++) {
         AC->G[Ig].IsSpherical = S->G[Ig].IsSpherical;
         AC->G[Ig].RotDOF      = S->G[Ig].RotDOF;
         AC->G[Ig].TrnDOF      = S->G[Ig].TrnDOF;
         AC->G[Ig].CGiBi       = S->G[Ig].CGiBi;
         AC->G[Ig].CBoGo       = S->G[Ig].CBoGo;

         AC->G[Ig].RotSeq = S->G[Ig].RotSeq;
         AC->G[Ig].TrnSeq = S->G[Ig].TrnSeq;
      }
   }

   /* Gyro Axes */
   AC->Ngyro = S->Ngyro;
   if (AC->Ngyro > 0) {
      AC->Gyro =
          (struct AcGyroType *)calloc(AC->Ngyro, sizeof(struct AcGyroType));
      for (i = 0; i < S->Ngyro; i++) {
         AC->Gyro[i].Axis = S->Gyro[i].Axis;
      }
   }

   /* Magnetometer Axes */
   AC->Nmag = S->Nmag;
   if (AC->Nmag > 0) {
      AC->MAG = (struct AcMagnetometerType *)calloc(
          AC->Nmag, sizeof(struct AcMagnetometerType));
      for (i = 0; i < S->Nmag; i++) {
         AC->MAG[i].Axis = S->MAG[i].Axis;
      }
   }

   /* Coarse Sun Sensors */
   AC->Ncss = S->Ncss;
   if (AC->Ncss > 0) {
      AC->CSS = (struct AcCssType *)calloc(AC->Ncss, sizeof(struct AcCssType));
      for (i = 0; i < S->Ncss; i++) {
         AC->CSS[i].Body  = S->CSS[i].Body;
         AC->CSS[i].Axis  = S->CSS[i].Axis;
         AC->CSS[i].Scale = S->CSS[i].Scale;
      }
   }

   /* Fine Sun Sensors */
   AC->Nfss = S->Nfss;
   if (AC->Nfss > 0) {
      AC->FSS = (struct AcFssType *)calloc(AC->Nfss, sizeof(struct AcFssType));
      for (k = 0; k < S->Nfss; k++) {
         AC->FSS[k].CB       = S->FSS[k].CB;
         AC->FSS[k].qb       = S->FSS[k].qb;
         AC->FSS[k].H_Axis   = S->FSS[k].H_Axis;
         AC->FSS[k].V_Axis   = S->FSS[k].V_Axis;
         AC->FSS[k].BoreAxis = S->FSS[k].BoreAxis;
         AC->FSS[k].type     = S->FSS[k].type;
      }
   }

   /* Star Trackers */
   AC->Nst = S->Nst;
   if (AC->Nst > 0) {
      AC->ST = (struct AcStarTrackerType *)calloc(
          AC->Nst, sizeof(struct AcStarTrackerType));
      for (k = 0; k < S->Nst; k++) {
         AC->ST[k].CB       = S->ST[k].CB;
         AC->ST[k].qb       = S->ST[k].qb;
         AC->ST[k].BoreAxis = S->ST[k].BoreAxis;
      }
   }

   /* GPS */
   AC->Ngps = S->Ngps;
   if (AC->Ngps > 0) {
      AC->GPS = (struct AcGpsType *)calloc(AC->Ngps, sizeof(struct AcGpsType));
   }

   /* Accelerometer Axes */
   AC->Nacc = S->Nacc;
   if (AC->Nacc > 0) {
      AC->Accel =
          (struct AcAccelType *)calloc(AC->Nacc, sizeof(struct AcAccelType));
      for (i = 0; i < S->Nacc; i++) {
         AC->Accel[i].Axis = S->Accel[i].Axis;
      }
   }

   /* Wheels */
   AC->Nwhl = S->Nw;
   if (AC->Nwhl > 0) {
      AC->Whl = (struct AcWhlType *)calloc(AC->Nwhl, sizeof(struct AcWhlType));
      A       = CreateMatrix(3, AC->Nwhl);
      Aplus   = CreateMatrix(AC->Nwhl, 3);
      for (i = 0; i < S->Nw; i++) {
         AC->Whl[i].Body = S->Whl[i].Body;
         AC->Whl[i].Axis = S->Whl[i].A;
         for (j = 0; j < 3; j++) {
            A[j][i] = S->Whl[i].A.v[j];
         }
      }
      if (S->Nw == 1) {
         AC->Whl[0].DistVec = AC->Whl[0].Axis;
      }
      else if (S->Nw >= 2) {
         PINVG(A, Aplus, 3, S->Nw);
         for (i = 0; i < AC->Nwhl; i++)
            for (j = 0; j < 3; j++)
               AC->Whl[i].DistVec.v[j] = Aplus[i][j];
      }
      DestroyMatrix(A);
      DestroyMatrix(Aplus);
      for (i = 0; i < S->Nw; i++) {
         AC->Whl[i].J    = S->Whl[i].J;
         AC->Whl[i].Tmax = S->Whl[i].Tmax;
         AC->Whl[i].Hmax = S->Whl[i].Hmax;
      }
   }

   /* Magnetic Torquer Bars */
   AC->Nmtb = S->Nmtb;
   if (AC->Nmtb > 0) {
      AC->MTB = (struct AcMtbType *)calloc(AC->Nmtb, sizeof(struct AcMtbType));
      A       = CreateMatrix(3, AC->Nmtb);
      Aplus   = CreateMatrix(AC->Nmtb, 3);
      for (i = 0; i < S->Nmtb; i++) {
         AC->MTB[i].Axis = S->MTB[i].A;
         for (j = 0; j < 3; j++)
            A[j][i] = S->MTB[i].A.v[j];
      }
      if (S->Nmtb == 1) {
         AC->MTB[0].DistVec = AC->MTB[0].Axis;
      }
      else if (S->Nmtb >= 2) {
         PINVG(A, Aplus, 3, S->Nmtb);
         for (i = 0; i < AC->Nmtb; i++) {
            for (j = 0; j < 3; j++)
               AC->MTB[i].DistVec.v[j] = Aplus[i][j];
         }
      }
      DestroyMatrix(A);
      DestroyMatrix(Aplus);
      for (i = 0; i < S->Nmtb; i++) {
         AC->MTB[i].Mmax = S->MTB[i].Mmax;
      }
   }

   /* Thrusters */
   AC->Nthr = S->Nthr;
   if (AC->Nthr > 0) {
      AC->Thr = (struct AcThrType *)calloc(AC->Nthr, sizeof(struct AcThrType));
      for (i = 0; i < S->Nthr; i++) {
         AC->Thr[i].Body = S->Thr[i].Body;
         AC->Thr[i].Fmax = S->Thr[i].Fmax;
         AC->Thr[i].Axis = S->Thr[i].A;
         AC->Thr[i].PosB = S->B[S->Thr[i].Body].Node[S->Thr[i].Node].PosB;
         for (j = 0; j < 3; j++)
            r.v[j] = AC->Thr[i].PosB.v[j] - AC->cm.v[j];
         AC->Thr[i].rxA = VxV(r, AC->Thr[i].Axis);
      }
   }

   /* Controllers */
   AC->PrototypeCtrl.Init = 1;
   AC->AdHocCtrl.Init     = 1;
   AC->SpinnerCtrl.Init   = 1;
   AC->MomBiasCtrl.Init   = 1;
   AC->ThreeAxisCtrl.Init = 1;
   AC->IssCtrl.Init       = 1;
   AC->CmgCtrl.Init       = 1;
   AC->ThrCtrl.Init       = 1;
   AC->CfsCtrl.Init       = 1;
   AC->ThrSteerCtrl.Init  = 1;

   AC->PrototypeCtrl.wc   = 0.05 * TwoPi;
   AC->PrototypeCtrl.amax = 0.01;
   AC->PrototypeCtrl.vmax = 0.5 * D2R;

   /* Initialize variables to avoid divide-by-zero before first sensor
    * measurements */
   AC->qbn.qs = 1.0;
   AC->svb.x  = 1.0;
   AC->bvb.x  = 1.0E-4;
}
/**********************************************************************/
/* The effective inertia for a gimbal is assumed to be the moment of  */
/* inertia of the appendage depending from the joint (that is, all    */
/* bodies for which that joint is in the JointPathTable) about that   */
/* joint, with all joints undeflected.                                */
vec3_t FindAppendageInertia(long Ig, struct SCType *S)
{
   struct DynType *D;
   struct JointType *G;
   vec3_t rho, Cr, rhog;
   mat3x3_t CBoG, IBoG, CBoBi, Coi, Csofar;
   long Ib, Jg, k;

   D = &S->Dyn;

   vec3_t Iapp = VEC3_ZERO;
   for (Ib = 1; Ib < S->Nb; Ib++) {
      if (D->JointPathTable[Ib][Ig].InPath) {
         /* Build undeflected rho */
         Jg    = S->B[Ib].Gin;
         rho   = VEC3_ZERO;
         CBoBi = MAT3X3_EYE;
         while (Jg > Ig) {
            G   = &S->G[Jg];
            Coi = MxM(G->CBoGo, G->CGiBi);
            for (k = 0; k < 3; k++)
               rho.v[k] -= G->ro.v[k];
            Cr = MTxV(Coi, rho);
            for (k = 0; k < 3; k++)
               rho.v[k] = Cr.v[k] + G->ri.v[k];
            Csofar = CBoBi;
            CBoBi  = MxM(Csofar, Coi);
            Jg     = S->B[G->Bin].Gin;
         }
         G = &S->G[Ig];
         for (k = 0; k < 3; k++)
            rho.v[k] -= G->ro.v[k];
         rhog = MTxV(G->CBoGo, rho);
         CBoG = MTxM(CBoBi, G->CBoGo);
         /* Parallel axis theorem */
         IBoG = PARAXIS(S->B[Ib].I, CBoG, S->B[Ib].mass, rhog);
         /* Accumulate */
         for (k = 0; k < 3; k++)
            Iapp.v[k] += IBoG.mat[k][k];
      }
   }
   return Iapp;
}
/**********************************************************************/
void MapCmdsToActuators(struct SCType *S)
{
   struct IdealActType *I;
   struct WhlType *W;
   struct MTBType *M;
   struct ThrType *T;
   struct AcType *AC;
   long i, Iw, Im, It;

   AC = &S->AC;

   if (S->GainAndDelayActive) {
      for (i = 0; i < 3; i++) {
         I       = &S->IdealAct[i];
         I->Fcmd = Delay(I->FrcDelay, S->LoopGain * AC->IdealFrc.v[i]);
         I->Tcmd = Delay(I->TrqDelay, S->LoopGain * AC->IdealTrq.v[i]);
      }

      for (Iw = 0; Iw < AC->Nwhl; Iw++) {
         W       = &S->Whl[Iw];
         W->Tcmd = Delay(W->Delay, S->LoopGain * AC->Whl[Iw].Tcmd);
      }
      for (Im = 0; Im < AC->Nmtb; Im++) {
         M       = &S->MTB[Im];
         M->Mcmd = Delay(M->Delay, S->LoopGain * AC->MTB[Im].Mcmd);
      }
      for (It = 0; It < AC->Nthr; It++) {
         T = &S->Thr[It];
         if (T->Mode == THR_PULSED) {
            T->PulseWidthFinTimeStamp = AC->Thr[It].PulseWidthFinTimeStamp;
            T->PulseWidthCmd =
                Delay(T->Delay, S->LoopGain * AC->Thr[It].PulseWidthCmd);
         }
         else
            T->ThrustLevelCmd =
                Delay(T->Delay, S->LoopGain * AC->Thr[It].ThrustLevelCmd);
      }
   }
   else if (S->FswSampleCounter == 0) {
      for (i = 0; i < 3; i++) {
         S->IdealAct[i].Fcmd = AC->IdealFrc.v[i];
         S->IdealAct[i].Tcmd = AC->IdealTrq.v[i];
      }

      for (Iw = 0; Iw < AC->Nwhl; Iw++) {
         S->Whl[Iw].Tcmd = AC->Whl[Iw].Tcmd;
      }
      for (Im = 0; Im < AC->Nmtb; Im++) {
         S->MTB[Im].Mcmd = AC->MTB[Im].Mcmd;
      }
      for (It = 0; It < AC->Nthr; It++) {
         if (S->Thr[It].Mode == THR_PULSED) {
            S->Thr[It].PulseWidthFinTimeStamp =
                AC->Thr[It].PulseWidthFinTimeStamp;
            S->Thr[It].PulseWidthCmd = AC->Thr[It].PulseWidthCmd;
         }
         else
            S->Thr[It].ThrustLevelCmd = AC->Thr[It].ThrustLevelCmd;
      }
   }
}
/**********************************************************************/
/*  This simple control law is suitable for rapid prototyping.        */
void PrototypeFSW(struct SCType *S)
{
   struct AcType *AC;
   struct AcPrototypeCtrlType *C;
   struct BodyType *B;
   struct CmdType *Cmd;
   vec3_t alpha, Iapp, Hvnb, Herr, werr;
   long Ig, i, j;

   AC  = &S->AC;
   C   = &AC->PrototypeCtrl;
   Cmd = &AC->Cmd;

   if (Cmd->Parm == PARM_AXIS_SPIN) {
      if (C->Init) {
         C->Init  = 0;
         C->Kprec = 3.0E-2;
         C->Knute = 1.0;
      }

      SpinnerCommand(S);

      B = &S->B[0];

      Hvnb = MxV(B->CN, Cmd->Hvn);

      for (i = 0; i < 3; i++) {
         Herr.v[i]    = S->Hvb.v[i] - Hvnb.v[i];
         werr.v[i]    = AC->wbn.v[i] - Cmd->wrn.v[i];
         C->Tcmd.v[i] = -C->Knute * werr.v[i];
         if (MAGV(Herr) < 0.5 * MAGV(Cmd->Hvn)) {
            C->Tcmd.v[i] -= C->Kprec * Herr.v[i];
         }
         AC->IdealTrq.v[i] = Limit(C->Tcmd.v[i], -0.1, 0.1);
      }
   }
   else {
      if (C->Init) {
         C->Init = 0;

         for (Ig = 0; Ig < AC->Ng; Ig++) {
            Iapp = FindAppendageInertia(Ig, S);
            for (j = 0; j < 3; j++) {
               FindPDGains(Iapp.v[j], 0.05, 1.0, &AC->G[Ig].AngRateGain.v[j],
                           &AC->G[Ig].AngGain.v[j]);
               AC->G[Ig].MaxAngRate.v[j] = 0.5 * D2R;
               AC->G[Ig].MaxTrq.v[j]     = 0.1;
            }
         }
      }

      /* Find qrn, wrn and joint angle commands */
      ThreeAxisAttitudeCommand(S);
      AC->qrn = AC->Cmd.qrn;

      /* Form attitude error signals */
      AC->qbr  = QxQT(AC->qbn, Cmd->qrn);
      C->therr = Q2AngleVec(AC->qbr);
      C->therr = Q2AngleVec(AC->qbr);
      for (i = 0; i < 3; i++)
         C->werr.v[i] = AC->wbn.v[i] - Cmd->wrn.v[i];

      /* Closed-loop attitude control */
      alpha = VectorRampCoastGlide(C->therr, C->werr, C->wc, C->amax, C->vmax);
      for (i = 0; i < 3; i++)
         AC->IdealTrq.v[i] = AC->MOI.mat[i][i] * alpha.v[i];
   }
}
/**********************************************************************/
/*  SC_Spinner is a one-body spin-stabilized inertial pointer         */
void SpinnerFSW(struct SCType *S)
{

   double B1, B2, magb, magb2;
   double x = 0.0;
   double y = 0.0;
   double w1, w2, w3;
   double CyclicTorque, OrbPeriod, MaxPtgErr;
   long i, Imtb;
   struct AcType *AC;
   struct AcSpinnerCtrlType *C;
   struct AcMtbType *M;

   AC = &S->AC;
   C  = &AC->SpinnerCtrl;

   if (AC->Init) {
      AC->Init = 0;
      AC->DT   = 0.1;
      C->Bold1 = 0.0;
      C->Bold2 = 0.0;
      C->xold  = 0.0;
      C->yold  = 0.0;

      CyclicTorque = 3.0E-4;
      MaxPtgErr    = 1.0 * D2R;
      OrbPeriod =
          TwoPi / sqrt(Orb[S->RefOrb].mu / (pow(Orb[S->RefOrb].SMA, 3)));
      FindSpinnerGains(AC->MOI.mat[2][2],
                       sqrt(AC->MOI.mat[0][0] * AC->MOI.mat[1][1]),
                       CyclicTorque, OrbPeriod, MaxPtgErr, &C->SpinRate,
                       &C->Knute, &C->Kprec);

      C->Ispin  = AC->MOI.mat[2][2];
      C->Itrans = sqrt(AC->MOI.mat[0][0] * AC->MOI.mat[1][1]);
   }

   /* Sun-TAM Attitude Determination */
   if (AC->SunValid) {
      AC->CBN = TRIAD(AC->svn, AC->bvn, AC->svb, AC->bvb);
      C->rvn  = AC->svn;
      C->rvb  = MxV(AC->CBN, C->rvn);
      x       = C->rvb.x;
      y       = C->rvb.y;
   }

   /* Spin rate control */
   B1    = AC->bvb.x;
   B2    = AC->bvb.y;
   magb  = sqrt(B1 * B1 + B2 * B2);
   B1   /= magb;
   B2   /= magb;
   w3    = (B1 * C->Bold2 - B2 * C->Bold1) / AC->DT - C->SpinRate;
   /*      w3 = AC->wbn[2] - C->SpinRate; */
   C->Bold1  = B1;
   C->Bold2  = B2;
   C->Tcmd.z = -C->Kprec * w3;

   /* Precession/nutation control */
   if (AC->SunValid && fabs(w3) < 0.5 * C->SpinRate) {
      w1        = (y - C->yold) / AC->DT + C->SpinRate * x;
      w2        = -(x - C->xold) / AC->DT + C->SpinRate * y;
      C->Tcmd.x = -C->Knute * w1 -
                  C->Kprec * (C->Itrans * w1 - C->Ispin * C->SpinRate * x);
      C->Tcmd.y = -C->Knute * w2 -
                  C->Kprec * (C->Itrans * w2 - C->Ispin * C->SpinRate * y);
      C->xold   = x;
      C->yold   = y;
   }
   else {
      C->Tcmd.x = 0.0;
      C->Tcmd.y = 0.0;
   }

   C->Mcmd = VxV(AC->bvb, C->Tcmd);
   magb2   = VoV(AC->bvb, AC->bvb);
   for (i = 0; i < 3; i++)
      C->Mcmd.v[i] /= magb2;

   for (Imtb = 0; Imtb < AC->Nmtb; Imtb++) {
      M       = &AC->MTB[Imtb];
      M->Mcmd = VoV(M->DistVec, C->Mcmd);
      M->Mcmd = Limit(M->Mcmd, -M->Mmax, M->Mmax);
   }
}
/**********************************************************************/
/* Notional two-body momentum-biased Earth pointer                    */
void MomBiasFSW(struct SCType *S)
{

   double PitchRateError, PitchTcmd;
   vec3_t Zvec = VEC3_PZAXIS;
   vec3_t Tcmd, Bdot, Mcmd;
   double magb2;
   static vec3_t bvbold;
   double PitchRateCmd = -0.001059;
   double Kry          = 5.0;
   double Kpy          = 0.1;
   double Krx          = 0.5;
   double Kpx          = 0.05;
   double Kunl         = 1.0E-4;
   double Kbdot        = 3.0E8;
   double Hwcmd        = -50.0;
   long i;
   struct AcType *AC;
   struct AcMomBiasCtrlType *C;

   AC = &S->AC;
   C  = &AC->MomBiasCtrl;

   if (C->Init) {
      C->Init = 0;
   }

   if (!AC->ES.Valid) { /* Bdot Acquisition */

      AC->Whl[0].Tcmd      = -Kry * (AC->Whl[0].H - Hwcmd);
      bvbold               = AC->bvb;
      AC->G[0].Cmd.Ang     = VEC3_ZERO;
      AC->G[0].Cmd.AngRate = VEC3_ZERO;
      for (i = 0; i < 3; i++) {
         Bdot.v[i]       = (AC->bvb.v[i] - bvbold.v[i]) / AC->DT;
         AC->MTB[i].Mcmd = -Kbdot * Bdot.v[i];
      }
   }
   else { /* Nadir Point */

      /* Pitch Loop */
      PitchRateError  = AC->wbn.y - PitchRateCmd;
      PitchTcmd       = -Kry * PitchRateError - Kpy * AC->ES.Pitch;
      AC->Whl[0].Tcmd = -PitchTcmd - Kunl * (AC->Whl[0].H - Hwcmd);

      /* Roll-Yaw Loop */
      Tcmd.x = -Krx * AC->wbn.x - Kpx * AC->ES.Roll;
      Tcmd.z = -0.5 * Tcmd.x;

      /* Wheel Unload */
      Tcmd.y = -Kunl * (AC->Whl[0].H - Hwcmd);

      /* M = BxT/B^2 */
      Mcmd    = VxV(AC->bvb, Tcmd);
      magb2   = VoV(AC->bvb, AC->bvb);
      Mcmd.x /= magb2;
      Mcmd.y /= magb2;
      Mcmd.z /= magb2;
      for (i = 0; i < 3; i++)
         AC->MTB[i].Mcmd = Mcmd.v[i];

      /* Solar Array Gimbal */
      AC->G[0].Cmd.AngRate.x = -PitchRateCmd;
      if (AC->SunValid) {
         AC->G[0].Cmd.Ang = PointGimbalToTarget(AC->G[0].RotSeq, AC->G[0].CGiBi,
                                                AC->G[0].CBoGo, AC->svb, Zvec);
      }
      else {
         AC->G[0].Cmd.Ang.x += PitchRateCmd * AC->DT;
      }
      if (AC->G[0].Ang.x - AC->G[0].Cmd.Ang.x > Pi)
         AC->G[0].Cmd.Ang.x += TwoPi;
      if (AC->G[0].Ang.x - AC->G[0].Cmd.Ang.x < -Pi)
         AC->G[0].Cmd.Ang.x -= TwoPi;
   }
}
/**********************************************************************/
/* SC_Aura is a three-body three-axis stabilized S/C                */
void ThreeAxisFSW(struct SCType *S)
{
   mat3x3_t CRN;
   quat_t qrn, qbr;
   vec3_t wln, Herr, HxB;
   vec3_t Zvec = VEC3_PZAXIS;
   double AngErr;
   long i, j;
   struct AcType *AC;
   struct AcThreeAxisCtrlType *C;

   AC = &S->AC;
   C  = &AC->ThreeAxisCtrl;

   if (C->Init) {
      C->Init = 0;

      AC->G[0].Cmd.AngRate = VEC3_ZERO;
      AC->G[0].Cmd.Ang     = VEC3_ZERO;
      for (j = 0; j < 3; j++) {
         AC->G[0].MaxAngRate.v[j] = 0.2 * D2R;
         AC->G[0].MaxTrq.v[j]     = 100.0;
         FindPDGains(S->B[1].I.mat[1][1], 0.02 * TwoPi, 1.0,
                     &AC->G[0].AngRateGain.v[j], &AC->G[0].AngGain.v[j]);
      }

      C->Hwcmd = VEC3_ZERO;
      for (i = 0; i < 3; i++) {
         FindPDGains(AC->MOI.mat[i][i], 0.1, 0.7, &C->Kr.v[i], &C->Kp.v[i]);
      }
      C->Kunl = 1.0E6;
   }

   /* Find Attitude Command */
   FindCLN(AC->PosN, AC->VelN, &CRN, &wln);
   qrn = C2Q(CRN);

   /* Form Error Signals */
   qbr = QxQT(AC->qbn, qrn);
   qbr = RECTIFYQ(qbr);

   /* PD Control */
   for (i = 0; i < 3; i++) {
      C->Tcmd.v[i] =
          -C->Kr.v[i] * AC->wbn.v[i] - C->Kp.v[i] * (2.0 * qbr.qv.v[i]);
      AC->Whl[i].Tcmd = -C->Tcmd.v[i];
   }

   /* Momentum Management */
   for (i = 0; i < 3; i++) {
      Herr.v[i] = AC->Whl[i].H - C->Hwcmd.v[i];
   }
   HxB = VxV(Herr, AC->bvb);
   for (i = 0; i < 3; i++)
      AC->MTB[i].Mcmd = C->Kunl * HxB.v[i];

   /* Solar Array Gimbal */
   AC->G[0].Cmd.AngRate.x = wln.y;
   if (AC->SunValid) {
      AC->G[0].Cmd.Ang = PointGimbalToTarget(AC->G[0].RotSeq, AC->G[0].CGiBi,
                                             AC->G[0].CBoGo, AC->svb, Zvec);
   }
   else {
      AC->G[0].Cmd.Ang.x += wln.y * AC->DT;
   }
   if (AC->G[0].Ang.x - AC->G[0].Cmd.Ang.x > Pi)
      AC->G[0].Cmd.Ang.x += TwoPi;
   if (AC->G[0].Ang.x - AC->G[0].Cmd.Ang.x < -Pi)
      AC->G[0].Cmd.Ang.x -= TwoPi;

   AngErr = AC->G[0].Ang.x - AC->G[0].Cmd.Ang.x;
   AC->G[0].Cmd.AngRate.x -=
       AC->G[0].AngGain.x / AC->G[0].AngRateGain.x * AngErr;
   AC->G[0].Cmd.AngRate.x = Limit(
       AC->G[0].Cmd.AngRate.x, -AC->G[0].MaxAngRate.x, AC->G[0].MaxAngRate.x);
}
/**********************************************************************/
void IssFSW(struct SCType *S)
{
   long Ig, i, j;
   struct AcType *AC;
   struct AcIssCtrlType *C;
   const mat3x3_t Identity = MAT3X3_EYE;
   const vec3_t Zvec       = VEC3_PZAXIS;
   double AngErr, MinRoZ, RoZ;
   vec3_t r, rb, tvb, svb, Iapp, GimCmd;
   mat3x3_t CRL, CBL, CBR;

   AC = &S->AC;
   C  = &AC->IssCtrl;

   if (C->Init) {
      C->Init = 0;
      for (Ig = 0; Ig < AC->Ng; Ig++) {
         AC->G[Ig].Cmd.AngRate = VEC3_ZERO;
         AC->G[Ig].Cmd.Ang     = VEC3_ZERO;
         for (j = 0; j < 3; j++) {
            AC->G[Ig].MaxAngRate.v[j] = 0.5 * D2R;
         }
         Iapp = FindAppendageInertia(Ig, S);
         for (j = 0; j < AC->G[Ig].RotDOF; j++) {
            FindPDGains(Iapp.v[j], 0.02 * TwoPi, 1.0,
                        &AC->G[Ig].AngRateGain.v[j], &AC->G[Ig].AngGain.v[j]);
            AC->G[Ig].MaxTrq.v[j] = 0.1 * AC->G[Ig].AngGain.v[j];
         }
      }
      for (i = 0; i < 3; i++)
         FindPDGains(S->I.mat[i][i], 0.02 * TwoPi, 0.7, &C->Kr.v[i],
                     &C->Kp.v[i]);
      C->Tmax = 0.1 * MAX(C->Kp.x, MAX(C->Kp.y, C->Kp.z));
   }

   /* .. Hold LVLH */
   CRL = A2C(213, 0.0 * D2R, 0.0, 0.0);
   CBL = MxMT(S->B[0].CN, S->CLN);
   CBR = MxMT(CBL, CRL);
   /* XVV */
   vec3_t therrv = C2A(321, CBR);
   C->therr      = (vec3_t){.z = therrv.x, .y = therrv.y, .x = therrv.z};
   for (i = 0; i < 3; i++) {
      C->werr.v[i] = AC->wbn.v[i] - S->wln.v[i];
      AC->IdealTrq.v[i] =
          -C->Kp.v[i] * C->therr.v[i] - C->Kr.v[i] * C->werr.v[i];
   }

   /* .. Point Main Solar Arrays */
   svb       = MxV(S->B[0].CN, AC->svn);
   GimCmd    = PointGimbalToTarget(21, Identity, Identity, svb, Zvec);
   GimCmd.x += 5.0 * D2R; /* Avoid lighting artifacts from on-edge polys */
   AC->G[0].Cmd.Ang.x     = GimCmd.x;
   AC->G[1].Cmd.Ang.x     = -GimCmd.x;
   AC->G[0].Cmd.AngRate.x = -S->wln.y;
   AC->G[1].Cmd.AngRate.x = S->wln.y;

   AC->G[2].Cmd.Ang.x = GimCmd.y;
   AC->G[3].Cmd.Ang.x = -GimCmd.y;
   AC->G[4].Cmd.Ang.x = GimCmd.y;
   AC->G[5].Cmd.Ang.x = -GimCmd.y;

   AC->G[6].Cmd.Ang.x = -GimCmd.y;
   AC->G[7].Cmd.Ang.x = GimCmd.y;
   AC->G[8].Cmd.Ang.x = -GimCmd.y;
   AC->G[9].Cmd.Ang.x = GimCmd.y;

   /* .. Point SM Solar Array */
   AC->G[12].Cmd.Ang.x = GimCmd.x;
   AC->G[13].Cmd.Ang.x = -GimCmd.x;

   /* .. Point Radiators */
   GimCmd              = PointGimbalToTarget(1, Identity, Identity, svb, Zvec);
   AC->G[10].Cmd.Ang.x = GimCmd.x + 90.0 * D2R;
   AC->G[11].Cmd.Ang.x = GimCmd.x + 90.0 * D2R;

   /* .. Point HGA */
   /* Select TDRS nearest Zenith */
   MinRoZ = 2.0;
   for (i = 0; i < 10; i++) {
      if (Tdrs[i].Exists) {
         for (j = 0; j < 3; j++)
            r.v[j] = Tdrs[i].PosN.v[j] - S->PosN.v[j];
         r   = UNITV(r).v;
         rb  = MxV(S->B[0].CN, r);
         RoZ = VoV(rb, Zvec);
         if (RoZ < MinRoZ) {
            MinRoZ = RoZ;
            tvb    = rb;
         }
      }
   }
   GimCmd = PointGimbalToTarget(21, S->G[14].CGiBi, Identity, tvb, Zvec);

   AC->G[14].Cmd.Ang.x = Limit(GimCmd.x, -120.0 * D2R, 120.0 * D2R);
   AC->G[14].Cmd.Ang.y = Limit(GimCmd.y, -65.0 * D2R, 65.0 * D2R);

   for (Ig = 0; Ig < AC->Ng; Ig++) {
      for (j = 0; j < AC->G[Ig].RotDOF; j++) {
         AngErr = AC->G[Ig].Ang.v[j] - AC->G[Ig].Cmd.Ang.v[j];
         AngErr = WrapTo2Pi(AngErr) - Pi;
         AC->G[Ig].Cmd.AngRate.v[j] =
             -AC->G[Ig].AngGain.v[j] / AC->G[Ig].AngRateGain.v[j] * AngErr;
         AC->G[Ig].Cmd.AngRate.v[j] =
             Limit(AC->G[Ig].Cmd.AngRate.v[j], -AC->G[Ig].MaxAngRate.v[j],
                   AC->G[Ig].MaxAngRate.v[j]);
      }
   }
}
/**********************************************************************/
void CmgFSW(struct SCType *S)
{
   struct AcType *AC;
   struct AcCmgCtrlType *C;
   quat_t qbl, qbr, H;
   mat3x3_t CBL, CRL;
   vec3_t Axis[4], Gim[4];
   static double MoveTime = 200.0;
   static vec3_t RPYCmd   = {.v = {1.0, 1.0, 1.0}};
   static quat_t qrl      = QUAT_EYE;
   static long Idx        = 0;
   long i;

   AC = &S->AC;
   C  = &AC->CmgCtrl;

   if (C->Init) {
      C->Init = 0;
      for (i = 0; i < 3; i++)
         FindPDGains(AC->MOI.mat[i][i], 0.5, 0.7, &C->Kr.v[i], &C->Kp.v[i]);
      for (i = 0; i < 4; i++) {
         AC->G[i].Cmd.Ang.x     = 0.0;
         AC->G[i].AngGain.x     = 0.0;
         AC->G[i].AngRateGain.x = 100.0;
         AC->G[i].MaxAngRate.x  = 1.0 * D2R;
         AC->G[i].MaxTrq.x      = 5.0;
      }
   }

   MoveTime -= AC->DT;
   if (MoveTime < 0.0) {
      MoveTime = 200.0;
      Idx      = (Idx + 1) % 3;
      if (RPYCmd.v[Idx] > 0.0)
         RPYCmd.v[Idx] = -60.0 * D2R;
      else
         RPYCmd.v[Idx] = 60.0 * D2R;
      CRL = A2C(123, RPYCmd.x, RPYCmd.y, RPYCmd.z);
      qrl = C2Q(CRL);
   }

   CBL     = MxMT(S->B[0].CN, S->CLN);
   qbl     = C2Q(CBL);
   qbr     = QxQT(qbl, qrl);
   qbr     = RECTIFYQ(qbr);
   C->werr = S->B[0].wn;
   for (i = 0; i < 3; i++) {
      C->therr.v[i] = 2.0 * qbr.q[i];
      C->Tcmd.v[i]  = -C->Kr.v[i] * C->werr.v[i] - C->Kp.v[i] * C->therr.v[i];
   }

   for (i = 0; i < 4; i++) {
      Axis[i] = AC->G[i].COI.rows[2];
      Gim[i]  = AC->G[i].COI.rows[0];

      H.q[i] = 75.0;
   }

   CMGLaw4x1DOF(C->Tcmd, Axis, Gim, H, &C->AngRateCmd);

   for (i = 0; i < 4; i++)
      AC->G[i].Cmd.AngRate.x = C->AngRateCmd.q[i];
}
/**********************************************************************/
void ThrFSW(struct SCType *S)
{
   struct AcType *AC;
   struct AcThrType *T;
   struct AcThrCtrlType *C;
   static double MoveTime = 0.0;
   double RollCmd[4]      = {30.0, 0.0, -30.0, 0.0};
   double PitchCmd[4]     = {0.0, 30.0, 0.0, -30.0};
   double YawCmd[4]       = {0.0, 0.0, 0.0, 0.0};
   double PosXcmd[4]      = {0.0, 0.0, 0.0, 0.0};
   double PosYcmd[4]      = {24.0, 0.0, -24.0, 0.0};
   double PosZcmd[4]      = {0.0, 24.0, 0.0, -24.0};
   static mat3x3_t CRL;
   mat3x3_t CRN;
   vec3_t PosRN, PosRL, FcmdB;
   quat_t qrn;
   double FoA, TorxA;
   static long Idx = 0;
   long i;

   AC = &S->AC;
   C  = &AC->ThrCtrl;

   if (C->Init) {
      C->Init = 0;
      for (i = 0; i < 3; i++)
         FindPDGains(AC->MOI.mat[i][i], 0.1, 0.7, &C->Kw.v[i], &C->Kth.v[i]);
      FindPDGains(AC->mass, 0.05, 1.0, &C->Kv, &C->Kp);
   }

   /* .. Commanded Attitude and Position */
   MoveTime -= AC->DT;
   if (MoveTime < 0.0) {
      MoveTime = 1000.0;
      Idx      = (Idx + 1) % 4;
      CRL =
          A2C(123, RollCmd[Idx] * D2R, PitchCmd[Idx] * D2R, YawCmd[Idx] * D2R);
      PosRL.x = PosXcmd[Idx];
      PosRL.y = PosYcmd[Idx];
      PosRL.z = PosZcmd[Idx];
   }
   CRN     = MxM(CRL, S->CLN);
   qrn     = C2Q(CRN);
   AC->qbr = QxQT(AC->qbn, qrn);
   AC->qbr = RECTIFYQ(AC->qbr);
   PosRN   = MTxV(S->CLN, PosRL);

   /* .. Force and Torque Commands */
   for (i = 0; i < 3; i++) {
      AC->Tcmd.v[i] =
          -C->Kw.v[i] * S->B[0].wn.v[i] - C->Kth.v[i] * 2.0 * AC->qbr.q[i];
      AC->Fcmd.v[i] =
          -C->Kv * S->VelR.v[i] - C->Kp * (S->PosR.v[i] - PosRN.v[i]);
      AC->Tcmd.v[i] = Limit(AC->Tcmd.v[i], -4.0, 4.0);
   }
   FcmdB = MxV(S->B[0].CN, AC->Fcmd);
   for (i = 0; i < 3; i++)
      FcmdB.v[i] = Limit(FcmdB.v[i], -2.0, 2.0);
   AC->Fcmd = MTxV(S->B[0].CN, FcmdB);

#if 0
/* .. Ideal Actuators to check out controller before tackling thruster logic */
      for(i=0;i<3;i++) {
         AC->IdealTrq[i] = AC->Tcmd[i];
         AC->IdealFrc[i] = AC->Fcmd[i];
      }
#else
   /* .. Distribute to Thrusters */
   for (i = 0; i < AC->Nthr; i++) {
      T                = &AC->Thr[i];
      T->PulseWidthCmd = 0.0;

      FoA   = VoV(FcmdB, T->Axis);
      TorxA = VoV(AC->Tcmd, T->rxA);
      if (FoA > 0.0 && TorxA > 0.0) {
         T->PulseWidthCmd = (0.25 * FoA + TorxA) / T->Fmax * AC->DT;
      }

      T->PulseWidthCmd          = Limit(T->PulseWidthCmd, 0.0, AC->DT);
      T->PulseWidthFinTimeStamp = JDAddSeconds(JD_TT_MJD, T->PulseWidthCmd);
   }

#endif
}
#if 0
/**********************************************************************/
/* CFS_FSW: A test case to work out interfaces between 42 and a       */
/* CFS flight software configuration.                                 */
void CfsFSW(struct AcType *AC)
{
      struct AcCfsCtrlType *C;
      struct AcJointType *G;
      double L1[3],L2[3],L3[3];
      double Hb[3],HxB[3];
      long i;

      C = &AC->CfsCtrl;
      G = &AC->G[0];

      if (C->Init) {
         C->Init = 0;
         for(i=0;i<3;i++) FindPDGains(AC->MOI[i][i],0.1*TwoPi,0.7,&C->Kr[i],&C->Kp[i]);
         C->Kunl = 1.0E6;
         FindPDGains(100.0,0.2,1.0,&G->AngRateGain[0],&G->AngGain[0]);
         G->MaxAngRate[0] = 1.0*D2R;
         G->MaxTrq[0] = 10.0;
      }

/* .. Sensor Processing */
      GyroProcessing(AC);
      MagnetometerProcessing(AC);
      CssProcessing(AC);
      FssProcessing(AC);
      StarTrackerProcessing(AC);
      GpsProcessing(AC);

/* .. Commanded Attitude */
      L3=UNITV(AC->PosN).v;
      VxV(AC->PosN,AC->VelN,L2);
      L2 = UNITV(L2).v;
      L3 = UNITV(L3).v;
      for(i=0;i<3;i++) {
         L2[i] = -L2[i];
         L3[i] = -L3[i];
      }
      VxV(L2,L3,L1);
      L1 = UNITV(L1).v;
      for(i=0;i<3;i++) {
         AC->CLN[0][i] = L1[i];
         AC->CLN[1][i] = L2[i];
         AC->CLN[2][i] = L3[i];
      }
      C2Q(AC->CLN,AC->qln);
      AC->wln[1] = -MAGV(AC->VelN)/MAGV(AC->PosN);

/* .. Attitude Control */
      QxQT(AC->qbn,AC->qln,AC->qbr);
      RECTIFYQ(AC->qbr);
      for(i=0;i<3;i++) {
         C->therr[i] = Limit(2.0*AC->qbr[i],-0.05,0.05);
         C->werr[i] = AC->wbn[i] - AC->wln[i];
         AC->Tcmd[i] = Limit(-C->Kr[i]*C->werr[i] - C->Kp[i]*C->therr[i],-0.1,0.1);
      }
/* .. Momentum Management */
      for(i=0;i<3;i++) Hb[i] = AC->MOI[i][i]*AC->wbn[i] + AC->Whl[i].H;
      VxV(Hb,AC->bvb,HxB);
      for(i=0;i<3;i++) AC->Mcmd[i] = C->Kunl*HxB[i];

/* .. Solar Array Steering */
      G->Cmd.Ang[0] = atan2(AC->svb[0],AC->svb[2]);
      AngErr = fmod(G->Ang[0]-G->Cmd.Ang[0],TwoPi);
      if (AngErr >  Pi) AngErr -= TwoPi;
      if (AngErr < -Pi) AngErr += TwoPi;
      G->Cmd.AngRate[0] = -G->AngGain[0]/G->AngRateGain[0]*AngErr;
      G->Cmd.AngRate[0] = Limit(G->Cmd.AngRate[0],-G->MaxAngRate[0],G->MaxAngRate[0]);

/* .. Actuator Processing */
      WheelProcessing(AC);
      MtbProcessing(AC);
}
#endif
/**********************************************************************/
/* Put your custom controller here                                    */
void AdHocFSW(struct SCType *S)
{
   struct AcType *AC;
   struct AcAdHocCtrlType *C;
   mat3x3_t CLN, CRN;
   quat_t qrn;
   vec3_t wln;
   const mat3x3_t CRL = {.rows = {VEC3_PYAXIS, VEC3_NZAXIS, VEC3_NXAXIS}};
   long i;

   AC = &S->AC;
   C  = &AC->AdHocCtrl;

   if (C->Init) {
      C->Init = 0;
      for (i = 0; i < 3; i++) {
         FindPDGains(AC->MOI.mat[i][i], 0.1 * TwoPi, 0.7, &C->Kr.v[i],
                     &C->Kp.v[i]);
         // C->Kp[i] *= 0.5;
         // C->Kr[i] *= 0.5;
      }
   }

   /* .. Form attitude error signals */
   FindCLN(AC->PosN, AC->VelN, &CLN, &wln);
   CRN     = MxM(CRL, CLN);
   qrn     = C2Q(CRN);
   AC->qbr = QxQT(AC->qbn, qrn);
   // for(i=0;i<4;i++) AC->qbr[i] = AC->qbn[i];
   AC->qbr = RECTIFYQ(AC->qbr);
   for (i = 0; i < 3; i++) {
      C->therr.v[i] = Limit(2.0 * AC->qbr.qv.v[i], -0.01, 0.01);
      C->werr.v[i]  = AC->wbn.v[i] - wln.v[i];
   }

   /* .. Closed-loop attitude control */
   for (i = 0; i < 3; i++)
      C->Tcmd.v[i] = -C->Kr.v[i] * C->werr.v[i] - C->Kp.v[i] * C->therr.v[i];

   AC->IdealTrq = C->Tcmd;
   // for(i=0;i<3;i++) AC->Whl[i].Tcmd = -C->Tcmd[i];
}
/**********************************************************************/
/*  This function is called at the simulation rate.  Sub-sampling of  */
/*  control loops should be done on a case-by-case basis.             */
/*  Mode handling, command generation, error determination, feedback  */
/*  and failure detection and correction all fall within the scope of */
/*  this file.                                                        */
/**********************************************************************/
void FlightSoftWare(struct SCType *S)
{
#ifdef _AC_STANDALONE_
   struct IpcType *I;
   long Iipc;
#endif
   if (S->FswTag == DSM_FSW) {
      if (S->DSM.DsmNav.NavigationActive == TRUE) {
         struct DSMNavType *Nav = &S->DSM.DsmNav;
         Nav->ccsds_time = CCSDSAddSeconds(Nav->ccsds_time, Nav->subStepSize);
      }
      DsmSensorModule(&S->AC, &S->DSM);
   }

   S->FswSampleCounter++;
   if (S->FswSampleCounter >= S->FswMaxCounter) {
      S->FswSampleCounter = 0;

#ifdef _AC_STANDALONE_
      for (Iipc = 0; Iipc < Nipc; Iipc++) {
         I = &IPC[Iipc];
         if (I->Mode == IPC_ACS && I->AcsID == S->AC.ID) {
            if (I->Init) {
               I->Init               = 0;
               S->AC.ParmLoadEnabled = 1;
               S->AC.ParmDumpEnabled = 1;
               S->AC.EchoEnabled     = 1;

               WriteToSocket(I->Socket, I->Prefix, I->Nprefix, I->EchoEnabled);
               ReadFromSocket(I->Socket, I->EchoEnabled);

               S->AC.ParmLoadEnabled = 0;
               S->AC.ParmDumpEnabled = 0;
            }
            else {
               WriteToSocket(I->Socket, I->Prefix, I->Nprefix, I->EchoEnabled);
               ReadFromSocket(I->Socket, I->EchoEnabled);
            }
         }
      }
#else
      switch (S->FswTag) {
         case PASSIVE_FSW:
            break;
         case PROTOTYPE_FSW:
            PrototypeFSW(S);
            break;
         case AD_HOC_FSW:
            AdHocFSW(S);
            break;
         case SPINNER_FSW:
            SpinnerFSW(S);
            break;
         case MOMBIAS_FSW:
            MomBiasFSW(S);
            break;
         case THREE_AXIS_FSW:
            ThreeAxisFSW(S);
            break;
         case ISS_FSW:
            IssFSW(S);
            break;
         case CMG_FSW:
            CmgFSW(S);
            break;
         case THR_FSW:
            ThrFSW(S);
            break;
         case DSM_FSW:
            DsmFSW(S);
            break;
         case CFS_FSW:
            AcFsw(&S->AC);
            break;
         case RBT_FSW:
#ifdef _ENABLE_RBT_
            RbtFSW(S);
#endif
            break;
      }
#endif
   }

   MapCmdsToActuators(S);
}

/* #ifdef __cplusplus
** }
** #endif
*/

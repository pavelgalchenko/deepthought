#include "42.h"

/**********************************************************************/
void ReadFromCmd(void)
{

      struct SCType *S;
      struct OrbitType *O;
      struct DynType *D;
      long Isc,Iorb,Iw,i;
      char line[512] = "Blank";
      long RequestTimeRefresh = 0;
      long Done;
      double DbleVal[30];
      long LongVal[30];

      /* Placeholder */
         if (EchoEnabled) printf("%s",line);

         if (sscanf(line,"TIME %ld-%ld-%ld:%ld:%lf\n",
            &Year,&doy,&Hour,&Minute,&Second) == 5)
            RequestTimeRefresh = 1;

         if (sscanf(line,"Orb[%ld].PosN = %le %le %le",
            &Iorb,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            Orb[Iorb].PosN[0] = DbleVal[0];
            Orb[Iorb].PosN[1] = DbleVal[1];
            Orb[Iorb].PosN[2] = DbleVal[2];
         }

         if (sscanf(line,"Orb[%ld].VelN = %le %le %le",
            &Iorb,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            Orb[Iorb].VelN[0] = DbleVal[0];
            Orb[Iorb].VelN[1] = DbleVal[1];
            Orb[Iorb].VelN[2] = DbleVal[2];
         }

         if (sscanf(line,"SC[%ld].PosR = %le %le %le",
            &Isc,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            SC[Isc].PosR[0] = DbleVal[0];
            SC[Isc].PosR[1] = DbleVal[1];
            SC[Isc].PosR[2] = DbleVal[2];
            SC[Isc].RequestStateRefresh = 1;
         }

         if (sscanf(line,"SC[%ld].VelR = %le %le %le",
            &Isc,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            SC[Isc].VelR[0] = DbleVal[0];
            SC[Isc].VelR[1] = DbleVal[1];
            SC[Isc].VelR[2] = DbleVal[2];
            SC[Isc].RequestStateRefresh = 1;
         }

         if (sscanf(line,"SC[%ld].svb = %le %le %le",
            &Isc,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            SC[Isc].svb[0] = DbleVal[0];
            SC[Isc].svb[1] = DbleVal[1];
            SC[Isc].svb[2] = DbleVal[2];
            SC[Isc].RequestStateRefresh = 1;
         }

         if (sscanf(line,"SC[%ld].bvb = %le %le %le",
            &Isc,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            SC[Isc].bvb[0] = DbleVal[0];
            SC[Isc].bvb[1] = DbleVal[1];
            SC[Isc].bvb[2] = DbleVal[2];
            SC[Isc].RequestStateRefresh = 1;
         }

         if (sscanf(line,"SC[%ld].Hvb = %le %le %le",
            &Isc,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            SC[Isc].Hvb[0] = DbleVal[0];
            SC[Isc].Hvb[1] = DbleVal[1];
            SC[Isc].Hvb[2] = DbleVal[2];
            SC[Isc].RequestStateRefresh = 1;
         }

         if (sscanf(line,"SC[%ld].AC.svb = %le %le %le",
            &Isc,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            SC[Isc].AC.svb[0] = DbleVal[0];
            SC[Isc].AC.svb[1] = DbleVal[1];
            SC[Isc].AC.svb[2] = DbleVal[2];
         }

         if (sscanf(line,"SC[%ld].AC.bvb = %le %le %le",
            &Isc,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            SC[Isc].AC.bvb[0] = DbleVal[0];
            SC[Isc].AC.bvb[1] = DbleVal[1];
            SC[Isc].AC.bvb[2] = DbleVal[2];
         }

         if (sscanf(line,"SC[%ld].AC.Hvb = %le %le %le",
            &Isc,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            SC[Isc].AC.Hvb[0] = DbleVal[0];
            SC[Isc].AC.Hvb[1] = DbleVal[1];
            SC[Isc].AC.Hvb[2] = DbleVal[2];
         }

         if (sscanf(line,"SC[%ld].AC.G[%ld].Cmd.Ang = %le %le %le",
            &Isc,&i,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 5) {
            SC[Isc].AC.G[i].Cmd.Ang[0] = DbleVal[0];
            SC[Isc].AC.G[i].Cmd.Ang[1] = DbleVal[1];
            SC[Isc].AC.G[i].Cmd.Ang[2] = DbleVal[2];
         }

         if (sscanf(line,"SC[%ld].AC.G[%ld].Cmd.qrl = %le %le %le %le",
            &Isc,&i,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2],
            &DbleVal[3]) == 6) {
            SC[Isc].AC.G[i].Cmd.qrl[0] = DbleVal[0];
            SC[Isc].AC.G[i].Cmd.qrl[1] = DbleVal[1];
            SC[Isc].AC.G[i].Cmd.qrl[2] = DbleVal[2];
            SC[Isc].AC.G[i].Cmd.qrl[3] = DbleVal[3];
         }

         if (sscanf(line,"SC[%ld].AC.G[%ld].Cmd.qrn = %le %le %le %le",
            &Isc,&i,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2],
            &DbleVal[3]) == 6) {
            SC[Isc].AC.G[i].Cmd.qrn[0] = DbleVal[0];
            SC[Isc].AC.G[i].Cmd.qrn[1] = DbleVal[1];
            SC[Isc].AC.G[i].Cmd.qrn[2] = DbleVal[2];
            SC[Isc].AC.G[i].Cmd.qrn[3] = DbleVal[3];
         }

         if (sscanf(line,"SC[%ld].AC.Whl[%ld].Tcmd = %le",
            &Isc,&i,
            &DbleVal[0]) == 3) {
            SC[Isc].AC.Whl[i].Tcmd = DbleVal[0];
         }

         if (sscanf(line,"SC[%ld].AC.MTB[%ld].Mcmd = %le",
            &Isc,&i,
            &DbleVal[0]) == 3) {
            SC[Isc].AC.MTB[i].Mcmd = DbleVal[0];
         }

         if (sscanf(line,"SC[%ld].AC.Thr[%ld].PulseWidthCmd = %le",
            &Isc,&i,
            &DbleVal[0]) == 3) {
            SC[Isc].AC.Thr[i].PulseWidthCmd = DbleVal[0];
         }

         if (sscanf(line,"SC[%ld].AC.Cmd.Ang = %le %le %le",
            &Isc,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            SC[Isc].AC.Cmd.Ang[0] = DbleVal[0];
            SC[Isc].AC.Cmd.Ang[1] = DbleVal[1];
            SC[Isc].AC.Cmd.Ang[2] = DbleVal[2];
         }

         if (sscanf(line,"SC[%ld].AC.Cmd.qrl = %le %le %le %le",
            &Isc,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2],
            &DbleVal[3]) == 5) {
            SC[Isc].AC.Cmd.qrl[0] = DbleVal[0];
            SC[Isc].AC.Cmd.qrl[1] = DbleVal[1];
            SC[Isc].AC.Cmd.qrl[2] = DbleVal[2];
            SC[Isc].AC.Cmd.qrl[3] = DbleVal[3];
         }

         if (sscanf(line,"SC[%ld].AC.Cmd.qrn = %le %le %le %le",
            &Isc,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2],
            &DbleVal[3]) == 5) {
            SC[Isc].AC.Cmd.qrn[0] = DbleVal[0];
            SC[Isc].AC.Cmd.qrn[1] = DbleVal[1];
            SC[Isc].AC.Cmd.qrn[2] = DbleVal[2];
            SC[Isc].AC.Cmd.qrn[3] = DbleVal[3];
         }

         if (sscanf(line,"SC[%ld].B[%ld].wn = %le %le %le",
            &Isc,&i,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 5) {
            SC[Isc].B[i].wn[0] = DbleVal[0];
            SC[Isc].B[i].wn[1] = DbleVal[1];
            SC[Isc].B[i].wn[2] = DbleVal[2];
            SC[Isc].RequestStateRefresh = 1;
         }

         if (sscanf(line,"SC[%ld].B[%ld].qn = %le %le %le %le",
            &Isc,&i,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2],
            &DbleVal[3]) == 6) {
            SC[Isc].B[i].qn[0] = DbleVal[0];
            SC[Isc].B[i].qn[1] = DbleVal[1];
            SC[Isc].B[i].qn[2] = DbleVal[2];
            SC[Isc].B[i].qn[3] = DbleVal[3];
            SC[Isc].RequestStateRefresh = 1;
         }

         if (sscanf(line,"SC[%ld].Whl[%ld].H = %le",
            &Isc,&i,
            &DbleVal[0]) == 3) {
            SC[Isc].Whl[i].H = DbleVal[0];
            SC[Isc].RequestStateRefresh = 1;
         }

         if (sscanf(line,"World[%ld].PosH = %le %le %le",
            &Iw,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            World[Iw].PosH[0] = DbleVal[0];
            World[Iw].PosH[1] = DbleVal[1];
            World[Iw].PosH[2] = DbleVal[2];
         }

         if (sscanf(line,"World[%ld].eph.PosN = %le %le %le",
            &Iw,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            World[Iw].eph.PosN[0] = DbleVal[0];
            World[Iw].eph.PosN[1] = DbleVal[1];
            World[Iw].eph.PosN[2] = DbleVal[2];
         }

         if (sscanf(line,"World[%ld].eph.VelN = %le %le %le",
            &Iw,
            &DbleVal[0],
            &DbleVal[1],
            &DbleVal[2]) == 4) {
            World[Iw].eph.VelN[0] = DbleVal[0];
            World[Iw].eph.VelN[1] = DbleVal[1];
            World[Iw].eph.VelN[2] = DbleVal[2];
         }

         for(Isc=0;Isc<Nsc;Isc++) {
            if (SC[Isc].Exists) {
               if (SC[Isc].AC.ParmDumpEnabled) {
                  if (sscanf(line,"SC[%ld].AC.ID = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.ID = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.EchoEnabled = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.EchoEnabled = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Nb = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.Nb = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Ng = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.Ng = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Nwhl = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.Nwhl = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Nmtb = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.Nmtb = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Nthr = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.Nthr = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Ncmg = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.Ncmg = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Ngyro = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.Ngyro = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Nmag = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.Nmag = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Ncss = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.Ncss = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Nfss = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.Nfss = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Nst = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.Nst = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Ngps = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.Ngps = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Nacc = %ld",
                     &Isc,
                     &LongVal[0]) == 2) {
                     SC[Isc].AC.Nacc = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.DT = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.DT = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.mass = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.mass = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.cm = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.cm[0] = DbleVal[0];
                     SC[Isc].AC.cm[1] = DbleVal[1];
                     SC[Isc].AC.cm[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.MOI = %le %le %le %le %le %le %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2],
                     &DbleVal[3],
                     &DbleVal[4],
                     &DbleVal[5],
                     &DbleVal[6],
                     &DbleVal[7],
                     &DbleVal[8]) == 10) {
                     SC[Isc].AC.MOI[0][0] = DbleVal[0];
                     SC[Isc].AC.MOI[0][1] = DbleVal[1];
                     SC[Isc].AC.MOI[0][2] = DbleVal[2];
                     SC[Isc].AC.MOI[1][0] = DbleVal[3];
                     SC[Isc].AC.MOI[1][1] = DbleVal[4];
                     SC[Isc].AC.MOI[1][2] = DbleVal[5];
                     SC[Isc].AC.MOI[2][0] = DbleVal[6];
                     SC[Isc].AC.MOI[2][1] = DbleVal[7];
                     SC[Isc].AC.MOI[2][2] = DbleVal[8];
                  }

                  if (sscanf(line,"SC[%ld].AC.B[%ld].mass = %le",
                     &Isc,&i,
                     &DbleVal[0]) == 3) {
                     SC[Isc].AC.B[i].mass = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.B[%ld].cm = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.B[i].cm[0] = DbleVal[0];
                     SC[Isc].AC.B[i].cm[1] = DbleVal[1];
                     SC[Isc].AC.B[i].cm[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.B[%ld].MOI = %le %le %le %le %le %le %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2],
                     &DbleVal[3],
                     &DbleVal[4],
                     &DbleVal[5],
                     &DbleVal[6],
                     &DbleVal[7],
                     &DbleVal[8]) == 11) {
                     SC[Isc].AC.B[i].MOI[0][0] = DbleVal[0];
                     SC[Isc].AC.B[i].MOI[0][1] = DbleVal[1];
                     SC[Isc].AC.B[i].MOI[0][2] = DbleVal[2];
                     SC[Isc].AC.B[i].MOI[1][0] = DbleVal[3];
                     SC[Isc].AC.B[i].MOI[1][1] = DbleVal[4];
                     SC[Isc].AC.B[i].MOI[1][2] = DbleVal[5];
                     SC[Isc].AC.B[i].MOI[2][0] = DbleVal[6];
                     SC[Isc].AC.B[i].MOI[2][1] = DbleVal[7];
                     SC[Isc].AC.B[i].MOI[2][2] = DbleVal[8];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].IsUnderActiveControl = %ld",
                     &Isc,&i,
                     &LongVal[0]) == 3) {
                     SC[Isc].AC.G[i].IsUnderActiveControl = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].IsSpherical = %ld",
                     &Isc,&i,
                     &LongVal[0]) == 3) {
                     SC[Isc].AC.G[i].IsSpherical = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].RotDOF = %ld",
                     &Isc,&i,
                     &LongVal[0]) == 3) {
                     SC[Isc].AC.G[i].RotDOF = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].TrnDOF = %ld",
                     &Isc,&i,
                     &LongVal[0]) == 3) {
                     SC[Isc].AC.G[i].TrnDOF = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].RotSeq = %ld",
                     &Isc,&i,
                     &LongVal[0]) == 3) {
                     SC[Isc].AC.G[i].RotSeq = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].TrnSeq = %ld",
                     &Isc,&i,
                     &LongVal[0]) == 3) {
                     SC[Isc].AC.G[i].TrnSeq = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].CGiBi = %le %le %le %le %le %le %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2],
                     &DbleVal[3],
                     &DbleVal[4],
                     &DbleVal[5],
                     &DbleVal[6],
                     &DbleVal[7],
                     &DbleVal[8]) == 11) {
                     SC[Isc].AC.G[i].CGiBi[0][0] = DbleVal[0];
                     SC[Isc].AC.G[i].CGiBi[0][1] = DbleVal[1];
                     SC[Isc].AC.G[i].CGiBi[0][2] = DbleVal[2];
                     SC[Isc].AC.G[i].CGiBi[1][0] = DbleVal[3];
                     SC[Isc].AC.G[i].CGiBi[1][1] = DbleVal[4];
                     SC[Isc].AC.G[i].CGiBi[1][2] = DbleVal[5];
                     SC[Isc].AC.G[i].CGiBi[2][0] = DbleVal[6];
                     SC[Isc].AC.G[i].CGiBi[2][1] = DbleVal[7];
                     SC[Isc].AC.G[i].CGiBi[2][2] = DbleVal[8];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].CBoGo = %le %le %le %le %le %le %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2],
                     &DbleVal[3],
                     &DbleVal[4],
                     &DbleVal[5],
                     &DbleVal[6],
                     &DbleVal[7],
                     &DbleVal[8]) == 11) {
                     SC[Isc].AC.G[i].CBoGo[0][0] = DbleVal[0];
                     SC[Isc].AC.G[i].CBoGo[0][1] = DbleVal[1];
                     SC[Isc].AC.G[i].CBoGo[0][2] = DbleVal[2];
                     SC[Isc].AC.G[i].CBoGo[1][0] = DbleVal[3];
                     SC[Isc].AC.G[i].CBoGo[1][1] = DbleVal[4];
                     SC[Isc].AC.G[i].CBoGo[1][2] = DbleVal[5];
                     SC[Isc].AC.G[i].CBoGo[2][0] = DbleVal[6];
                     SC[Isc].AC.G[i].CBoGo[2][1] = DbleVal[7];
                     SC[Isc].AC.G[i].CBoGo[2][2] = DbleVal[8];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].AngGain = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.G[i].AngGain[0] = DbleVal[0];
                     SC[Isc].AC.G[i].AngGain[1] = DbleVal[1];
                     SC[Isc].AC.G[i].AngGain[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].AngRateGain = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.G[i].AngRateGain[0] = DbleVal[0];
                     SC[Isc].AC.G[i].AngRateGain[1] = DbleVal[1];
                     SC[Isc].AC.G[i].AngRateGain[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].PosGain = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.G[i].PosGain[0] = DbleVal[0];
                     SC[Isc].AC.G[i].PosGain[1] = DbleVal[1];
                     SC[Isc].AC.G[i].PosGain[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].PosRateGain = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.G[i].PosRateGain[0] = DbleVal[0];
                     SC[Isc].AC.G[i].PosRateGain[1] = DbleVal[1];
                     SC[Isc].AC.G[i].PosRateGain[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].MaxAngRate = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.G[i].MaxAngRate[0] = DbleVal[0];
                     SC[Isc].AC.G[i].MaxAngRate[1] = DbleVal[1];
                     SC[Isc].AC.G[i].MaxAngRate[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].MaxPosRate = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.G[i].MaxPosRate[0] = DbleVal[0];
                     SC[Isc].AC.G[i].MaxPosRate[1] = DbleVal[1];
                     SC[Isc].AC.G[i].MaxPosRate[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].MaxTrq = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.G[i].MaxTrq[0] = DbleVal[0];
                     SC[Isc].AC.G[i].MaxTrq[1] = DbleVal[1];
                     SC[Isc].AC.G[i].MaxTrq[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.G[%ld].MaxFrc = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.G[i].MaxFrc[0] = DbleVal[0];
                     SC[Isc].AC.G[i].MaxFrc[1] = DbleVal[1];
                     SC[Isc].AC.G[i].MaxFrc[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.Gyro[%ld].Axis = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.Gyro[i].Axis[0] = DbleVal[0];
                     SC[Isc].AC.Gyro[i].Axis[1] = DbleVal[1];
                     SC[Isc].AC.Gyro[i].Axis[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.MAG[%ld].Axis = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.MAG[i].Axis[0] = DbleVal[0];
                     SC[Isc].AC.MAG[i].Axis[1] = DbleVal[1];
                     SC[Isc].AC.MAG[i].Axis[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.CSS[%ld].Body = %ld",
                     &Isc,&i,
                     &LongVal[0]) == 3) {
                     SC[Isc].AC.CSS[i].Body = LongVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.CSS[%ld].Axis = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.CSS[i].Axis[0] = DbleVal[0];
                     SC[Isc].AC.CSS[i].Axis[1] = DbleVal[1];
                     SC[Isc].AC.CSS[i].Axis[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.CSS[%ld].Scale = %le",
                     &Isc,&i,
                     &DbleVal[0]) == 3) {
                     SC[Isc].AC.CSS[i].Scale = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.FSS[%ld].qb = %le %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2],
                     &DbleVal[3]) == 6) {
                     SC[Isc].AC.FSS[i].qb[0] = DbleVal[0];
                     SC[Isc].AC.FSS[i].qb[1] = DbleVal[1];
                     SC[Isc].AC.FSS[i].qb[2] = DbleVal[2];
                     SC[Isc].AC.FSS[i].qb[3] = DbleVal[3];
                  }

                  if (sscanf(line,"SC[%ld].AC.FSS[%ld].CB = %le %le %le %le %le %le %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2],
                     &DbleVal[3],
                     &DbleVal[4],
                     &DbleVal[5],
                     &DbleVal[6],
                     &DbleVal[7],
                     &DbleVal[8]) == 11) {
                     SC[Isc].AC.FSS[i].CB[0][0] = DbleVal[0];
                     SC[Isc].AC.FSS[i].CB[0][1] = DbleVal[1];
                     SC[Isc].AC.FSS[i].CB[0][2] = DbleVal[2];
                     SC[Isc].AC.FSS[i].CB[1][0] = DbleVal[3];
                     SC[Isc].AC.FSS[i].CB[1][1] = DbleVal[4];
                     SC[Isc].AC.FSS[i].CB[1][2] = DbleVal[5];
                     SC[Isc].AC.FSS[i].CB[2][0] = DbleVal[6];
                     SC[Isc].AC.FSS[i].CB[2][1] = DbleVal[7];
                     SC[Isc].AC.FSS[i].CB[2][2] = DbleVal[8];
                  }

                  if (sscanf(line,"SC[%ld].AC.ST[%ld].qb = %le %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2],
                     &DbleVal[3]) == 6) {
                     SC[Isc].AC.ST[i].qb[0] = DbleVal[0];
                     SC[Isc].AC.ST[i].qb[1] = DbleVal[1];
                     SC[Isc].AC.ST[i].qb[2] = DbleVal[2];
                     SC[Isc].AC.ST[i].qb[3] = DbleVal[3];
                  }

                  if (sscanf(line,"SC[%ld].AC.ST[%ld].CB = %le %le %le %le %le %le %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2],
                     &DbleVal[3],
                     &DbleVal[4],
                     &DbleVal[5],
                     &DbleVal[6],
                     &DbleVal[7],
                     &DbleVal[8]) == 11) {
                     SC[Isc].AC.ST[i].CB[0][0] = DbleVal[0];
                     SC[Isc].AC.ST[i].CB[0][1] = DbleVal[1];
                     SC[Isc].AC.ST[i].CB[0][2] = DbleVal[2];
                     SC[Isc].AC.ST[i].CB[1][0] = DbleVal[3];
                     SC[Isc].AC.ST[i].CB[1][1] = DbleVal[4];
                     SC[Isc].AC.ST[i].CB[1][2] = DbleVal[5];
                     SC[Isc].AC.ST[i].CB[2][0] = DbleVal[6];
                     SC[Isc].AC.ST[i].CB[2][1] = DbleVal[7];
                     SC[Isc].AC.ST[i].CB[2][2] = DbleVal[8];
                  }

                  if (sscanf(line,"SC[%ld].AC.Accel[%ld].PosB = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.Accel[i].PosB[0] = DbleVal[0];
                     SC[Isc].AC.Accel[i].PosB[1] = DbleVal[1];
                     SC[Isc].AC.Accel[i].PosB[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.Accel[%ld].Axis = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.Accel[i].Axis[0] = DbleVal[0];
                     SC[Isc].AC.Accel[i].Axis[1] = DbleVal[1];
                     SC[Isc].AC.Accel[i].Axis[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.Whl[%ld].Axis = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.Whl[i].Axis[0] = DbleVal[0];
                     SC[Isc].AC.Whl[i].Axis[1] = DbleVal[1];
                     SC[Isc].AC.Whl[i].Axis[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.Whl[%ld].DistVec = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.Whl[i].DistVec[0] = DbleVal[0];
                     SC[Isc].AC.Whl[i].DistVec[1] = DbleVal[1];
                     SC[Isc].AC.Whl[i].DistVec[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.Whl[%ld].J = %le",
                     &Isc,&i,
                     &DbleVal[0]) == 3) {
                     SC[Isc].AC.Whl[i].J = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Whl[%ld].Tmax = %le",
                     &Isc,&i,
                     &DbleVal[0]) == 3) {
                     SC[Isc].AC.Whl[i].Tmax = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Whl[%ld].Hmax = %le",
                     &Isc,&i,
                     &DbleVal[0]) == 3) {
                     SC[Isc].AC.Whl[i].Hmax = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.MTB[%ld].Axis = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.MTB[i].Axis[0] = DbleVal[0];
                     SC[Isc].AC.MTB[i].Axis[1] = DbleVal[1];
                     SC[Isc].AC.MTB[i].Axis[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.MTB[%ld].DistVec = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.MTB[i].DistVec[0] = DbleVal[0];
                     SC[Isc].AC.MTB[i].DistVec[1] = DbleVal[1];
                     SC[Isc].AC.MTB[i].DistVec[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.MTB[%ld].Mmax = %le",
                     &Isc,&i,
                     &DbleVal[0]) == 3) {
                     SC[Isc].AC.MTB[i].Mmax = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.Thr[%ld].PosB = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.Thr[i].PosB[0] = DbleVal[0];
                     SC[Isc].AC.Thr[i].PosB[1] = DbleVal[1];
                     SC[Isc].AC.Thr[i].PosB[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.Thr[%ld].Axis = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.Thr[i].Axis[0] = DbleVal[0];
                     SC[Isc].AC.Thr[i].Axis[1] = DbleVal[1];
                     SC[Isc].AC.Thr[i].Axis[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.Thr[%ld].rxA = %le %le %le",
                     &Isc,&i,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 5) {
                     SC[Isc].AC.Thr[i].rxA[0] = DbleVal[0];
                     SC[Isc].AC.Thr[i].rxA[1] = DbleVal[1];
                     SC[Isc].AC.Thr[i].rxA[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.Thr[%ld].Fmax = %le",
                     &Isc,&i,
                     &DbleVal[0]) == 3) {
                     SC[Isc].AC.Thr[i].Fmax = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.PrototypeCtrl.wc = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.PrototypeCtrl.wc = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.PrototypeCtrl.amax = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.PrototypeCtrl.amax = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.PrototypeCtrl.vmax = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.PrototypeCtrl.vmax = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.PrototypeCtrl.Kprec = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.PrototypeCtrl.Kprec = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.PrototypeCtrl.Knute = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.PrototypeCtrl.Knute = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.AdHocCtrl.Kr = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.AdHocCtrl.Kr[0] = DbleVal[0];
                     SC[Isc].AC.AdHocCtrl.Kr[1] = DbleVal[1];
                     SC[Isc].AC.AdHocCtrl.Kr[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.AdHocCtrl.Kp = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.AdHocCtrl.Kp[0] = DbleVal[0];
                     SC[Isc].AC.AdHocCtrl.Kp[1] = DbleVal[1];
                     SC[Isc].AC.AdHocCtrl.Kp[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.SpinnerCtrl.Ispin = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.SpinnerCtrl.Ispin = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.SpinnerCtrl.Itrans = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.SpinnerCtrl.Itrans = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.SpinnerCtrl.SpinRate = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.SpinnerCtrl.SpinRate = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.SpinnerCtrl.Knute = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.SpinnerCtrl.Knute = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.SpinnerCtrl.Kprec = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.SpinnerCtrl.Kprec = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.ThreeAxisCtrl.Kr = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.ThreeAxisCtrl.Kr[0] = DbleVal[0];
                     SC[Isc].AC.ThreeAxisCtrl.Kr[1] = DbleVal[1];
                     SC[Isc].AC.ThreeAxisCtrl.Kr[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.ThreeAxisCtrl.Kp = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.ThreeAxisCtrl.Kp[0] = DbleVal[0];
                     SC[Isc].AC.ThreeAxisCtrl.Kp[1] = DbleVal[1];
                     SC[Isc].AC.ThreeAxisCtrl.Kp[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.ThreeAxisCtrl.Kunl = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.ThreeAxisCtrl.Kunl = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.IssCtrl.Kr = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.IssCtrl.Kr[0] = DbleVal[0];
                     SC[Isc].AC.IssCtrl.Kr[1] = DbleVal[1];
                     SC[Isc].AC.IssCtrl.Kr[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.IssCtrl.Kp = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.IssCtrl.Kp[0] = DbleVal[0];
                     SC[Isc].AC.IssCtrl.Kp[1] = DbleVal[1];
                     SC[Isc].AC.IssCtrl.Kp[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.IssCtrl.Tmax = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.IssCtrl.Tmax = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.CmgCtrl.Kr = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.CmgCtrl.Kr[0] = DbleVal[0];
                     SC[Isc].AC.CmgCtrl.Kr[1] = DbleVal[1];
                     SC[Isc].AC.CmgCtrl.Kr[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.CmgCtrl.Kp = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.CmgCtrl.Kp[0] = DbleVal[0];
                     SC[Isc].AC.CmgCtrl.Kp[1] = DbleVal[1];
                     SC[Isc].AC.CmgCtrl.Kp[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.ThrCtrl.Kw = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.ThrCtrl.Kw[0] = DbleVal[0];
                     SC[Isc].AC.ThrCtrl.Kw[1] = DbleVal[1];
                     SC[Isc].AC.ThrCtrl.Kw[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.ThrCtrl.Kth = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.ThrCtrl.Kth[0] = DbleVal[0];
                     SC[Isc].AC.ThrCtrl.Kth[1] = DbleVal[1];
                     SC[Isc].AC.ThrCtrl.Kth[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.ThrCtrl.Kv = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.ThrCtrl.Kv = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.ThrCtrl.Kp = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.ThrCtrl.Kp = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.CfsCtrl.Kr = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.CfsCtrl.Kr[0] = DbleVal[0];
                     SC[Isc].AC.CfsCtrl.Kr[1] = DbleVal[1];
                     SC[Isc].AC.CfsCtrl.Kr[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.CfsCtrl.Kp = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.CfsCtrl.Kp[0] = DbleVal[0];
                     SC[Isc].AC.CfsCtrl.Kp[1] = DbleVal[1];
                     SC[Isc].AC.CfsCtrl.Kp[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.CfsCtrl.Kunl = %le",
                     &Isc,
                     &DbleVal[0]) == 2) {
                     SC[Isc].AC.CfsCtrl.Kunl = DbleVal[0];
                  }

                  if (sscanf(line,"SC[%ld].AC.ThrSteerCtrl.Kr = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.ThrSteerCtrl.Kr[0] = DbleVal[0];
                     SC[Isc].AC.ThrSteerCtrl.Kr[1] = DbleVal[1];
                     SC[Isc].AC.ThrSteerCtrl.Kr[2] = DbleVal[2];
                  }

                  if (sscanf(line,"SC[%ld].AC.ThrSteerCtrl.Kp = %le %le %le",
                     &Isc,
                     &DbleVal[0],
                     &DbleVal[1],
                     &DbleVal[2]) == 4) {
                     SC[Isc].AC.ThrSteerCtrl.Kp[0] = DbleVal[0];
                     SC[Isc].AC.ThrSteerCtrl.Kp[1] = DbleVal[1];
                     SC[Isc].AC.ThrSteerCtrl.Kp[2] = DbleVal[2];
                  }

               }
            }
         }


         if (!strncmp(line,"[EOF]",5)) {
            Done = 1;
            sprintf(line,"[EOF] reached\n");
         }
      }

      if (RequestTimeRefresh) {
         /* Update AbsTime, SimTime, etc */
         DOY2MD(Year,doy,&Month,&Day);
         AbsTime = DateToAbsTime(Year,Month,Day,Hour,Minute,Second);
         JulDay = AbsTimeToJD(AbsTime);
         JDToGpsTime(JulDay,&GpsRollover,&GpsWeek,&GpsSecond);
         SimTime = AbsTime-AbsTime0;
      }


/* .. Refresh SC states that depend on inputs */

      for(Isc=0;Isc<Nsc;Isc++) {
         if (SC[Isc].RequestStateRefresh) {
            S = &SC[Isc];
            S->RequestStateRefresh = 0;
            if (S->Exists) {
               /* Update  RefOrb */
               O = &Orb[S->RefOrb];
               O->Epoch = AbsTime;
               for(i=0;i<3;i++) {
                  S->PosN[i] = O->PosN[i] + S->PosR[i];
                  S->VelN[i] = O->VelN[i] + S->VelR[i];
               }
               RV2Eph(O->Epoch,O->mu,O->PosN,O->VelN,
                  &O->SMA,&O->ecc,&O->inc,&O->RAAN,
                  &O->ArgP,&O->anom,&O->tp,
                  &O->SLR,&O->alpha,&O->rmin,
                  &O->MeanMotion,&O->Period);
               FindCLN(O->PosN,O->VelN,O->CLN,O->wln);

               /* Update Dyn */
               MapJointStatesToStateVector(S);
               D = &S->Dyn;
               MapStateVectorToBodyStates(D->u,D->x,D->uf,D->xf,S);
               MotionConstraints(S);
            }
         }
      }
}
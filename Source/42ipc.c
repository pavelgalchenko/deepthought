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
#include <time.h>
#include <unistd.h>

#ifdef _ENABLE_GMSEC_
#include "gmseckit.h"
GMSEC_Config cfg;
GMSEC_ConnectionMgr ConnMgr;
GMSEC_Status status;
void WriteToGmsec(GMSEC_ConnectionMgr ConnMgr, GMSEC_Status status);
void ReadFromGmsec(GMSEC_ConnectionMgr ConnMgr, GMSEC_Status status);
#endif

/* #ifdef __cplusplus
** namespace _42 {
** using namespace Kit;
** #endif
*/

void WriteToFile(FILE *StateFile, char **Prefix, long Nprefix,
                 long EchoEnabled);
void WriteToSocket(SOCKET Socket, char **Prefix, long Nprefix,
                   long EchoEnabled);
void ReadFromFile(FILE *StateFile, long EchoEnabled);
void ReadFromSocket(SOCKET Socket, long EchoEnabled);

/*********************************************************************/
void InitInterProcessComm(void)
{
   struct fy_document *fyd =
       fy_document_build_and_check(NULL, InOutPath, "Inp_IPC.yaml");
   struct fy_node *root = fy_document_root(fyd);
   char response[120] = {0}, FileName[80] = {0};

   struct fy_node *node = fy_node_by_path_def(root, "/IPCs");
   Nipc                 = fy_node_sequence_item_count(node);
   IPC = (struct IpcType *)calloc(Nipc, sizeof(struct IpcType));

   long Iipc                = 0;
   struct fy_node *iterNode = NULL;
   WHILE_FY_ITER(node, iterNode)
   {
      struct fy_node *seqNode = fy_node_by_path_def(iterNode, "/IPC");
      struct IpcType *I       = &IPC[Iipc];

      if (fy_node_scanf(seqNode,
                        "/Mode %119s "
                        "/AC ID %ld "
                        "/File Name %79[^\n]s "
                        "/Socket/Host/Name %39[^\n]s "
                        "/Socket/Host/Port %ld",
                        response, &I->AcsID, FileName, I->HostName,
                        &I->Port) != 5) {
         printf("IPC is improperly configured. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      I->Mode = DecodeString(response);
      if (!fy_node_scanf(seqNode, "/Socket/Role %119s", response)) {
         printf("Could not find Socket Role for IPC. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      I->SocketRole = DecodeString(response);
      I->AllowBlocking =
          getYAMLBool(fy_node_by_path_def(seqNode, "/Socket/Blocking"));
      I->EchoEnabled =
          getYAMLBool(fy_node_by_path_def(seqNode, "/Echo to stdout"));
      struct fy_node *prefixNode = fy_node_by_path_def(seqNode, "/Prefixes");
      I->Nprefix                 = fy_node_sequence_item_count(prefixNode);
      I->Prefix                  = (char **)calloc(I->Nprefix, sizeof(char *));
      struct fy_node *prefixIterNode = NULL;
      long Ipx                       = 0;
      WHILE_FY_ITER(prefixNode, prefixIterNode)
      {
         size_t prefLen     = 0;
         const char *prefix = fy_node_get_scalar(prefixIterNode, &prefLen);
         I->Prefix[Ipx]     = (char *)calloc(prefLen + 1, sizeof(char));
         strncpy(I->Prefix[Ipx], prefix, prefLen);
         Ipx++;
      }

      I->Init = 1;

      if (I->Mode == IPC_TX) {
         if (I->SocketRole == IPC_SERVER) {
            I->Socket = InitSocketServer(I->Port, I->AllowBlocking);
         }
         else if (I->SocketRole == IPC_CLIENT) {
            I->Socket =
                InitSocketClient(I->HostName, I->Port, I->AllowBlocking);
         }
#ifdef _ENABLE_GMSEC_
         else if (I->SocketRole == IPC_GMSEC_CLIENT) {
            status  = statusCreate();
            cfg     = configCreate();
            ConnMgr = ConnectToMBServer(I->HostName, I->Port, status, cfg);
            connectionManagerSubscribe(ConnMgr, "GMSEC.42.RX.>", status);
            CheckGmsecStatus(status);
         }
#endif
         else {
            printf("Oops.  Unknown SocketRole %ld for IPC[%ld] in "
                   "InitInterProcessComm.  Bailing out.\n",
                   I->SocketRole, Iipc);
            exit(1);
         }
      }
      else if (I->Mode == IPC_RX) {
         if (I->SocketRole == IPC_SERVER) {
            I->Socket = InitSocketServer(I->Port, I->AllowBlocking);
         }
         else if (I->SocketRole == IPC_CLIENT) {
            I->Socket =
                InitSocketClient(I->HostName, I->Port, I->AllowBlocking);
         }
#ifdef _ENABLE_GMSEC_
         else if (I->SocketRole == IPC_GMSEC_CLIENT) {
            status  = statusCreate();
            cfg     = configCreate();
            ConnMgr = ConnectToMBServer(I->HostName, I->Port, status, cfg);
            connectionManagerSubscribe(ConnMgr, "GMSEC.42.TX.>", status);
            CheckGmsecStatus(status);
         }
#endif
         else {
            printf("Oops.  Unknown SocketRole %ld for IPC[%ld] in "
                   "InitInterProcessComm.  Bailing out.\n",
                   I->SocketRole, Iipc);
            exit(1);
         }
      }
      else if (I->Mode == IPC_TXRX) {
         if (I->SocketRole == IPC_SERVER) {
            I->Socket = InitSocketServer(I->Port, I->AllowBlocking);
         }
         else if (I->SocketRole == IPC_CLIENT) {
            I->Socket =
                InitSocketClient(I->HostName, I->Port, I->AllowBlocking);
         }
#ifdef _ENABLE_GMSEC_
         else if (I->SocketRole == IPC_GMSEC_CLIENT) {
            status  = statusCreate();
            cfg     = configCreate();
            ConnMgr = ConnectToMBServer(I->HostName, I->Port, status, cfg);
            connectionManagerSubscribe(ConnMgr, "GMSEC.42.TXRX.>", status);
            CheckGmsecStatus(status);
         }
#endif
         else {
            printf("Oops.  Unknown SocketRole %ld for IPC[%ld] in "
                   "InitInterProcessComm.  Bailing out.\n",
                   I->SocketRole, Iipc);
            exit(1);
         }
      }
      else if (I->Mode == IPC_ACS) {
         I->Socket = InitSocketServer(I->Port, I->AllowBlocking);
      }
      else if (I->Mode == IPC_WRITEFILE) {
         I->File = FileOpen(InOutPath, FileName, "wt");
      }
      else if (I->Mode == IPC_READFILE) {
         I->File = FileOpen(InOutPath, FileName, "rt");
      }
      else if (I->Mode == IPC_FFTB) {
         I->SocketRole = IPC_CLIENT; /* Spirent is Host */
         I->Socket = InitSocketClient(I->HostName, I->Port, I->AllowBlocking);
      }
      Iipc++;
   }
   fy_document_destroy(fyd);
}
/*********************************************************************/
void InterProcessComm(void)
{
   struct IpcType *I;
   long Iipc;

   for (Iipc = 0; Iipc < Nipc; Iipc++) {
      I = &IPC[Iipc];
      if (I->Mode == IPC_TX) {
         if (I->SocketRole != IPC_GMSEC_CLIENT) {
            WriteToSocket(I->Socket, I->Prefix, I->Nprefix, I->EchoEnabled);
         }
#ifdef _ENABLE_GMSEC_
         else {
            WriteToGmsec(ConnMgr, status, I->Prefix, I->Nprefix,
                         I->EchoEnabled);
         }
#endif
      }
      else if (I->Mode == IPC_RX) {
         if (I->SocketRole != IPC_GMSEC_CLIENT) {
            ReadFromSocket(I->Socket, I->EchoEnabled);
         }
#ifdef _ENABLE_GMSEC_
         else {
            ReadFromGmsec(ConnMgr, status, I->EchoEnabled);
         }
#endif
      }
      else if (I->Mode == IPC_TXRX) {
         if (I->SocketRole != IPC_GMSEC_CLIENT) {
            WriteToSocket(I->Socket, I->Prefix, I->Nprefix, I->EchoEnabled);
            ReadFromSocket(I->Socket, I->EchoEnabled);
         }
#ifdef _ENABLE_GMSEC_
         else {
            WriteToGmsec(ConnMgr, status, I->Prefix, I->Nprefix,
                         I->EchoEnabled);
            ReadFromGmsec(ConnMgr, status, I->EchoEnabled);
         }
#endif
      }
      else if (I->Mode == IPC_WRITEFILE) {
         WriteToFile(I->File, I->Prefix, I->Nprefix, I->EchoEnabled);
      }
      else if (I->Mode == IPC_READFILE) {
         ReadFromFile(I->File, I->EchoEnabled);
      }
#ifdef _ENABLE_FFTB_CODE_
      else if (I->Mode == IPC_FFTB) {
         SendStatesToSpirent();
      }
#endif
   }
}

/* #ifdef __cplusplus
** }
** #endif
*/

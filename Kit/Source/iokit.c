/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "iokit.h"

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/
/**********************************************************************/
struct fy_document *fy_document_build_and_check(const struct fy_parse_cfg *cfg,
                                                char *path, char *fileName)
{
   FILE *f                 = FileOpen(path, fileName, "r");
   struct fy_document *fyd = fy_document_build_from_fp(NULL, f);
   fclose(f);
   if (fy_document_resolve(fyd)) {
      printf("Unable to resolve links in %127s. Exiting...\n", fileName);
      exit(EXIT_FAILURE);
   }

   if (!fyd) {
      printf("Failed to build yaml from %127s. Exiting...\n", fileName);
      fy_document_destroy(fyd);
      exit(EXIT_FAILURE);
   }
   return fyd;
}
/**********************************************************************/
struct fy_node *fy_node_by_path_def(struct fy_node *node, const char *path)
{
   return (fy_node_by_path(node, path, -1, FYNWF_PTR_YAML));
}
/**********************************************************************/
long getYAMLBool(struct fy_node *node)
{
   size_t strLen    = 0;
   const char *data = fy_node_get_scalar(node, &strLen);
   return !strncasecmp(data, "true", strLen);
}
/**********************************************************************/
long assignYAMLToDoubleArray(const long n, struct fy_node *yamlSequence,
                             double dest[])
{
   long i                   = 0;
   struct fy_node *iterNode = NULL;
   WHILE_FY_ITER(yamlSequence, iterNode)
   {
      if (!fy_node_scanf(iterNode, "/ %lf", &dest[i])) {
         char *parentAddress = fy_node_get_parent_address(yamlSequence);
         printf("Problem reading YAML sequence %s in assignYAMLToDoubleArray "
                "in %s on line %d. "
                "Exiting...\n",
                parentAddress, __FILE__, __LINE__);
         exit(EXIT_FAILURE);
      }
      i++;
      if (i == n)
         break;
   }
   return (i);
}
/**********************************************************************/
long assignYAMLToFloatArray(const long n, struct fy_node *yamlSequence,
                            float dest[])
{
   long i                   = 0;
   struct fy_node *iterNode = NULL;
   WHILE_FY_ITER(yamlSequence, iterNode)
   {
      if (!fy_node_scanf(iterNode, "/ %f", &dest[i])) {
         char *parentAddress = fy_node_get_parent_address(yamlSequence);
         printf("Problem reading YAML sequence %s in assignYAMLTofloatArray "
                "in %s on line %d. "
                "Exiting...\n",
                parentAddress, __FILE__, __LINE__);
         exit(EXIT_FAILURE);
      }
      i++;
      if (i == n)
         break;
   }
   return (i);
}
/**********************************************************************/
long assignYAMLToLongArray(const long n, struct fy_node *yamlSequence,
                           long dest[])
{
   long i                   = 0;
   struct fy_node *iterNode = NULL;
   WHILE_FY_ITER(yamlSequence, iterNode)
   {
      if (!fy_node_scanf(iterNode, "/ %ld", &dest[i])) {
         char *parentAddress = fy_node_get_parent_address(yamlSequence);
         printf("Problem reading YAML sequence %s in assignYAMLToDoubleArray "
                "in %s on line %d. Exiting...\n",
                parentAddress, __FILE__, __LINE__);
         exit(EXIT_FAILURE);
      }
      i++;
      if (i == n)
         break;
   }
   return (i);
}
/**********************************************************************/
long assignYAMLToBoolArray(const long n, struct fy_node *yamlSequence,
                           long dest[])
{
   long i                   = 0;
   struct fy_node *iterNode = NULL;
   WHILE_FY_ITER(yamlSequence, iterNode)
   {
      dest[i] = getYAMLBool(iterNode);
      i++;
      if (i == n)
         break;
   }
   return (i);
}
/**********************************************************************/
long getYAMLEulerAngles(struct fy_node *yamlEuler, double angles[3], long *seq)
{
   long i = 0;
   i      = assignYAMLToDoubleArray(
       3, fy_node_by_path(yamlEuler, "/Angles", -1, FYNWF_PTR_YAML), angles);
   i += fy_node_scanf(yamlEuler, "/Sequence %ld", seq);
   if (i != 4) {
      printf("Problem reading Euler Angles in getYAMLEulerAngles in %s on line "
             "%d. Exiting...\n",
             __FILE__, __LINE__);
      exit(EXIT_FAILURE);
   }
   return (i);
}
/**********************************************************************/
FILE *FileOpen(const char *Path, const char *File, const char *CtrlCode)
{
   FILE *FilePtr;
   char FileName[1024];

   strcpy(FileName, Path);
   strcat(FileName, File);
   FilePtr = fopen(FileName, CtrlCode);
   if (FilePtr == NULL) {
      printf("Error opening %s: %s\n", FileName, strerror(errno));
      exit(1);
   }
   return (FilePtr);
}
/**********************************************************************/
void ByteSwapDouble(double *A)
{
   char fwd[8], bak[8];
   long i;

   memcpy(fwd, A, sizeof(double));
   for (i = 0; i < 8; i++)
      bak[i] = fwd[7 - i];
   memcpy(A, bak, sizeof(double));
}
/**********************************************************************/
/*  This function cribbed from an OpenCL example                      */
/*  on the Apple developer site                                       */
int FileToString(const char *file_name, char **result_string,
                 size_t *string_len)
{
   int fd;
   size_t file_len;
   struct stat file_status;
   int ret;

   *string_len = 0;
   fd          = open(file_name, O_RDONLY);
   if (fd == -1) {
      printf("Error opening file %s\n", file_name);
      return -1;
   }
   ret = fstat(fd, &file_status);
   if (ret) {
      printf("Error reading status for file %s\n", file_name);
      return -1;
   }
   file_len = file_status.st_size;

   *result_string = (char *)calloc(file_len + 1, sizeof(char));
   ret            = read(fd, *result_string, file_len);
   if (!ret) {
      printf("Error reading from file %s\n", file_name);
      return -1;
   }

   close(fd);

   *string_len = file_len;
   return 0;
}
/**********************************************************************/
SOCKET InitSocketServer(int Port, int AllowBlocking)
{
#if defined(_WIN32)

   WSADATA wsa;
   SOCKET init_sockfd, sockfd;
   u_long Blocking = 1;

   int clilen;
   struct sockaddr_in Server, Client;

   /* Initialize winsock */
   if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
      printf("Error initializing winsock in InitSocketClient.\n");
      exit(1);
   }

   init_sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if (init_sockfd < 0) {
      printf("Error opening server socket.\n");
      exit(1);
   }
   memset((char *)&Server, 0, sizeof(Server));
   Server.sin_family      = AF_INET;
   Server.sin_addr.s_addr = INADDR_ANY;
   Server.sin_port        = htons(Port);
   if (bind(init_sockfd, (struct sockaddr *)&Server, sizeof(Server)) < 0) {
      printf("Error on binding server socket.\n");
      exit(1);
   }
   printf("Server is listening on port %i\n", Port);
   listen(init_sockfd, 5);
   clilen = sizeof(Client);
   sockfd = accept(init_sockfd, (struct sockaddr *)&Client, &clilen);
   if (sockfd < 0) {
      printf("Error on accepting client socket.\n");
      exit(1);
   }
   printf("Server side of socket established.\n");
   closesocket(init_sockfd);

   /* Keep read() from waiting for message to come */
   if (!AllowBlocking) {
      /*flags = fcntl(sockfd, F_GETFL, 0);*/
      /*fcntl(sockfd,F_SETFL, flags|O_NONBLOCK);*/
      ioctlsocket(sockfd, FIONBIO, &Blocking);
   }

   /* Allow TCP to send small packets (look up Nagle's algorithm) */
   /* Depending on your message sizes, this may or may not improve performance
    */
   // setsockopt(sockfd,IPPROTO_TCP,TCP_NODELAY,&DisableNagle,sizeof(DisableNagle));

   return (sockfd);
#else

   SOCKET init_sockfd, sockfd;
   int flags;
   socklen_t clilen;
   struct sockaddr_in Server, Client;
   int opt          = 1;
   int DisableNagle = 1;

   init_sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if (init_sockfd < 0) {
      printf("Error opening server socket.\n");
      exit(1);
   }

   /* Allowing reuse while in TIME_WAIT might make port available */
   /* more quickly after a socket has been broken */
   if (setsockopt(init_sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) ==
       -1) {
      printf("Error setting socket option.\n");
      exit(1);
   }

   memset((char *)&Server, 0, sizeof(Server));
   Server.sin_family      = AF_INET;
   Server.sin_addr.s_addr = INADDR_ANY;
   Server.sin_port        = htons(Port);
   if (bind(init_sockfd, (struct sockaddr *)&Server, sizeof(Server)) < 0) {
      printf("Error on binding server socket.\n");
      exit(1);
   }
   printf("Server is listening on port %i\n", Port);
   listen(init_sockfd, 5);
   clilen = sizeof(Client);
   sockfd = accept(init_sockfd, (struct sockaddr *)&Client, &clilen);
   if (sockfd < 0) {
      printf("Error on accepting client socket.\n");
      exit(1);
   }
   printf("Server side of socket established.\n");
   close(init_sockfd);

   /* Keep read() from waiting for message to come */
   if (!AllowBlocking) {
      flags = fcntl(sockfd, F_GETFL, 0);
      fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
   }

   /* Allow TCP to send small packets (look up Nagle's algorithm) */
   /* Depending on your message sizes, this may or may not improve performance
    */
   setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &DisableNagle,
              sizeof(DisableNagle));

   return (sockfd);
#endif
}
/**********************************************************************/
SOCKET InitSocketClient(const char *hostname, int Port, int AllowBlocking)
{
#if defined(_WIN32)

   WSADATA wsa; /* winsock */
   SOCKET sockfd;
   u_long Blocking = 1;

   struct sockaddr_in Server;
   struct hostent *Host;

   /* Initialize winsock */
   if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
      printf("Error initializing winsock in InitSocketClient.\n");
      exit(1);
   }
   sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if (sockfd < 0) {
      printf("Error opening client socket.\n");
      exit(1);
   }
   Host = gethostbyname(hostname);
   if (Host == NULL) {
      printf("Server not found by client socket.\n");
      exit(1);
   }
   memset((char *)&Server, 0, sizeof(Server));
   Server.sin_family = AF_INET;
   memcpy((char *)&Server.sin_addr.s_addr, (char *)Host->h_addr,
          Host->h_length);
   Server.sin_port = htons(Port);
   printf("Client connecting to Server on Port %i\n", Port);
   if (connect(sockfd, (struct sockaddr *)&Server, sizeof(Server)) < 0) {
      printf("Error connecting client socket: %s.\n", strerror(errno));
      exit(1);
   }
   printf("Client side of socket established.\n");

   /* Keep read() from waiting for message to come */
   if (!AllowBlocking) {
      /*flags = fcntl(sockfd, F_GETFL, 0);*/
      /*fcntl(sockfd,F_SETFL, flags|O_NONBLOCK);*/
      ioctlsocket(sockfd, FIONBIO, &Blocking);
   }

   return (sockfd);
#else
   SOCKET sockfd;
   int flags;
   struct sockaddr_in Server;
   struct hostent *Host;
   int DisableNagle = 1;

   sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if (sockfd < 0) {
      printf("Error opening client socket.\n");
      exit(1);
   }
   Host = gethostbyname(hostname);
   if (Host == NULL) {
      printf("Server not found by client socket.\n");
      exit(1);
   }
   memset((char *)&Server, 0, sizeof(Server));
   Server.sin_family = AF_INET;
   memcpy((char *)&Server.sin_addr.s_addr, (char *)Host->h_addr,
          Host->h_length);
   Server.sin_port = htons(Port);
   printf("Client connecting to Server on Port %i\n", Port);
   if (connect(sockfd, (struct sockaddr *)&Server, sizeof(Server)) < 0) {
      printf("Error connecting client socket: %s.\n", strerror(errno));
      exit(1);
   }
   printf("Client side of socket established.\n");

   /* Keep read() from waiting for message to come */
   if (!AllowBlocking) {
      flags = fcntl(sockfd, F_GETFL, 0);
      fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
   }

   /* Allow TCP to send small packets (look up Nagle's algorithm) */
   /* Depending on your message sizes, this may or may not improve performance
    */
   setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &DisableNagle,
              sizeof(DisableNagle));

   return (sockfd);
#endif /* _WIN32 */
}

/* #ifdef __cplusplus
** }
** #endif
*/

/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __IOKIT_H__
#define __IOKIT_H__

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

#include "libfyaml.h"
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#ifdef _WIN32
#include <winsock2.h>
#else
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
/* Finesse winsock SOCKET datatype */
#define SOCKET int
#endif
/* #include <sys/un.h> */

#define STR2(x) #x
#define STR(X)  STR2(X)

#define WHILE_FY_ITER(node, iterNode)                                          \
   while (fy_node_sequence_iterate((node), (void **)&(iterNode)) != NULL)

struct fy_document *fy_document_build_and_check(const struct fy_parse_cfg *cfg,
                                                char *path, char *fileName);
struct fy_node *fy_node_by_path_def(struct fy_node *node, const char *path);
long getYAMLBool(struct fy_node *node);
long assignYAMLToDoubleArray(const long n, struct fy_node *yamlSequence,
                             double dest[]);
long assignYAMLToFloatArray(const long n, struct fy_node *yamlSequence,
                            float dest[]);
long assignYAMLToLongArray(const long n, struct fy_node *yamlSequence,
                           long dest[]);
long assignYAMLToBoolArray(const long n, struct fy_node *yamlSequence,
                           long dest[]);
long getYAMLEulerAngles(struct fy_node *yamlEuler, double angles[3], long *seq);

FILE *FileOpen(const char *Path, const char *File, const char *CtrlCode);
void ByteSwapDouble(double *A);
int FileToString(const char *file_name, char **result_string,
                 size_t *string_len);
double *PpmToPsf(const char *path, const char *filename, long *width,
                 long *height, long *BytesPerPixel);

SOCKET InitSocketServer(int Port, int AllowBlocking);
SOCKET InitSocketClient(const char *hostname, int Port, int AllowBlocking);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __IOKIT_H__ */

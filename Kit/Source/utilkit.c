/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "utilkit.h"
#include <stdlib.h>

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/
#define BUFSIZE 1000

void GetExecDir(char exec_dir[BUFSIZE])
{
#ifndef __linux__
   char tempargs[BUFSIZE];
#endif
   char *ret;
#ifdef __linux__
   char *real_path = realpath("/proc/self/exe", NULL);
   strcpy(exec_dir, real_path);
   ret = strrchr(exec_dir, '/');
   free(real_path);
#elif defined __MINGW32__
   GetModuleFileName(NULL, tempargs, sizeof(tempargs));
   _fullpath(exec_dir, tempargs, sizeof(tempargs));
   ret = strrchr(exec_dir, '\\');
#elif defined _WIN32
   GetModuleFileName(NULL, tempargs, sizeof(tempargs));
   _fullpath(exec_dir, tempargs, sizeof(tempargs));
   ret = strrchr(exec_dir, '\\');
#elif defined _WIN64
   GetModuleFileName(NULL, tempargs, sizeof(tempargs));
   _fullpath(exec_dir, tempargs, sizeof(tempargs));
   ret = strrchr(exec_dir, '\\');
#elif defined __APPLE__
   uint32_t bytes;
   bytes = BUFSIZE;
   bytes = sizeof("/0");
   _NSGetExecutablePath("/0", &bytes);
   _NSGetExecutablePath(tempargs, &bytes);
   realpath(tempargs, exec_dir);
   ret = strrchr(exec_dir, '/');
#endif

   if (ret != NULL)
      *ret = '\0';
   ret = strrchr(exec_dir, '.');
   if (ret != NULL)
      *ret = '\0';
}

/*
** #ifdef __cplusplus
** }
** #endif
*/
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
#include <ctype.h>
#include <stdlib.h>

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/
#define BUFSIZE 1000

/**********************************************************************/
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
#elif defined(__MINGW32__) || defined(_WIN32) || defined(_WIN)
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
/**********************************************************************/
void tolower_str(size_t n, char *str)
{
   if (n == 0)
      n = strlen(str);
   char *s = &str[0];
   for (size_t i = 0; i < n; i++) {
      if (s[i] == '\0')
         break;
      s[i] = tolower(s[i]);
   }
}
/**********************************************************************/
void toupper_str(size_t n, char *str)
{
   if (n == 0)
      n = strlen(str);
   char *s = &str[0];
   for (size_t i = 0; i < n; i++) {
      if (s[i] == '\0')
         break;
      s[i] = toupper(s[i]);
   }
}
/**********************************************************************/
void CapitalizeFirst(size_t n, char *str)
{
   tolower_str(n, str);
   str[0] = toupper(str[0]);
}
/**********************************************************************/
void replace_char(char *str, const char find, const char replace)
{
   char *current_pos = strchr(str, find);
   while (current_pos) {
      *current_pos = replace;
      current_pos  = strchr(current_pos, find);
   }
}
/**********************************************************************/
long is_line_empty(const char *s)
{
   while (*s) {
      if (!isspace(*s))
         return 0;
      s++;
   }
   return 1;
}
/**********************************************************************/
void remove_alpha_chars(char *str)
{
   int i = 0; // Read pointer
   int j = 0; // Write pointer

   // Loop until the end of the string
   while (str[i] != '\0') {

      // is the character a digit?
      const int cond1 = isdigit((unsigned char)str[i]);
      // is the character 'e', 'E'

      // If the character is NOT alphabetic, keep it
      if (!isalpha((unsigned char)str[i])) {
         str[j] = str[i];
         j++;
      }
      i++;
   }

   // Null-terminate the newly shortened string
   str[j] = '\0';
}
/**********************************************************************/

/*
** #ifdef __cplusplus
** }
** #endif
*/
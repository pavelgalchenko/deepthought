/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

/**********************************************************************/
/**********************************************************************/
/*  Some simple util functions to be used elsewhere                   */
/**********************************************************************/
/**********************************************************************/

#ifndef __UTILKIT_H__
#define __UTILKIT_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/
void ResolvePath(char *path, const size_t path_len);
void GetParentDirectory(char *path, const size_t path_len);
void GetExecDir(char exec_dir[1000]);
void tolower_str(char *const str, size_t n);
void toupper_str(char *const str, size_t n);
void totitle_str(char *const str, size_t n);
void replace_char(char *str, const char find, const char replace);
__attribute__((pure)) long is_line_empty(const char *s);
void remove_alpha_chars(char *str);

/**********************************************************************/
// Case Independent string comparison
/**********************************************************************/
#if defined(__linux__) || defined(__APPLE__)
#define dt_strcasecmp  strcasecmp
#define dt_strncasecmp strncasecmp
#elif defined(__MINGW32__) || defined(_WIN32) || defined(_WIN)
#define dt_strcasecmp  _stricmp
#define dt_strncasecmp _strnicmp
#endif

#ifndef dt_strncasecmp
int dt_strncasecmp(const char *__s1, const char *__s2, size_t __n)
{
   char s1[__n + 1], s2[__n + 1];
   memset(s1, 0, __n + 1);
   memset(s2, 0, __n + 1);
   strcpy(s1, __s1);
   strcpy(s2, __s2);

   tolower_str(s1, __n);
   tolower_str(s2, __n);
   return strncmp(s1, s2, __n);
}
#endif
#define MAX_STR_LEN (1024)
#ifndef dt_strcasecmp
int dt_strcasecmp(const char *__s1, const char *__s2)
{
   const size_t s1_len = strlen(__s1), s2_len = strlen(__s2);
   if (s1_len > MAX_STR_LEN || s2_len > MAX_STR_LEN) {
      fprintf(stderr,
              "strings too long in dt_strcasecmp (utilkit.h:%d). Exiting...\n",
              __LINE__);
      exit(1);
   }

   const size_t max_len = (s1_len > s2_len) ? s1_len : s2_len;
   return dt_strncasecmp(__s1, __s2, max_len);
}
#endif
#undef MAX_STR_LEN
/**********************************************************************/

/*
** #ifdef __cplusplus
** }
** #endif
*/
#endif /* __UTILKIT_H__ */

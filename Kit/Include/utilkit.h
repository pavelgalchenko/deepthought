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

#include <string.h>

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

void GetExecDir(char exec_dir[1000]);
void tolower_str(size_t n, char *str);
void toupper_str(size_t n, char *str);
void CapitalizeFirst(size_t n, char *str);
char *replace_char(char *str, const char find, const char replace);
__attribute__((pure)) long is_line_empty(const char *s);

/*
** #ifdef __cplusplus
** }
** #endif
*/
#endif /* __UTILKIT_H__ */

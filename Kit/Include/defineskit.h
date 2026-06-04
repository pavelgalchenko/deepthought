/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __DEFINESKIT_H__
#define __DEFINESKIT_H__

#define SEC_PER_DAY (86400)

#define STR2(x) #x
#define STR(X)  STR2(X)

#ifdef __has_builtin
#if __has_builtin(__builtin_ctzl)
#define _ctzl (__builtin_ctzl)
#endif
#if __has_builtin(__builtin_ctz)
#define _ctz (__builtin_ctz)
#endif
#endif

#define CONCAT_PRIMATIVE(a, b) a##b
#define CONCAT_EXPAND(a, b)    CONCAT_PRIMATIVE(a, b)
#if defined(__INT64_C)
#define INT64_MACRO __INT64_C
#elif defined(__INT64_C_SUFFIX__)
#define INT64_MACRO(c) CONCAT_EXPAND(c, __INT64_C_SUFFIX__)
#else
#define INT64_MACRO(c) CONCAT_EXPAND(c, L)
#endif

#if defined(__UINT64_C)
#define UINT64_MACRO __UINT64_C
#elif defined(__UINT64_C_SUFFIX__)
#define UINT64_MACRO(c) CONCAT_EXPAND(c, __UINT64_C_SUFFIX__)
#else
#define UINT64_MACRO(c) CONCAT_EXPAND(c, UL)
#endif

#if !defined(__MINGW32__) && defined(WIN32)
#ifndef isnan(x)
#define isnan(x) ((x) != (x))
#endif
#endif

#ifndef MIN
#define MIN(x, y) ((x) > (y) ? (y) : (x))
#endif
#ifndef MAX
#define MAX(x, y) (((x) >= (y)) ? (x) : (y))
#endif
#ifndef ABS
#define ABS(x) (((x) >= 0) ? (x) : -(x))
#endif
#ifndef SIGN
#define SIGN(x) (((x) >= 0) ? 1 : -1)
#endif

#endif /* __DEFINESKIT_H__ */

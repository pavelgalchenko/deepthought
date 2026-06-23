/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __42CONSTANTS_H__
#define __42CONSTANTS_H__

#define PI             (3.14159265358979323846)     /* pi */
#define TWOPI          (6.28318530717958647693)     /* 2*pi */
#define HALFPI         (1.57079632679489661923)     /* pi/2 */
#define QUARTPI        (7.85398163397448309616e-01) /* pi/4 */
#define ONEPI          (3.18309886183790671538e-01) /* 1/pi */
#define EULER          (2.71828182845904523536)     /* e */
#define R2A            (2.06264806247096355156e+05) /* (180*3600)/pi */
#define R2D            (5.72957795130823208768e+01) /* 180/pi */
#define D2R            (1.74532925199432957692e-02) /* pi/180 */
#define D2A            (3600)                       /* 3600 */
#define A2R            (4.84813681109535993590e-06) /* pi/(180*3600) */
#define A2D            (2.77777777777777777778e-04) /* 1/3600 */
#define SQRTTWO        (1.41421356237309504880)     /* sqrt(2) */
#define SQRTTHREE      (1.73205080756887729353)     /* sqrt(3) */
#define SQRTHALF       (7.07106781186547524401e-01) /* 1/sqrt(2) */
#define GOLDENRATIO    (1.61803398874989484820)     /* (1 + sqrt(5))/2 */
#define SPEED_OF_LIGHT (299792458.0)                /* c */

#define SEC_PER_SIDEREAL_DAY (86164.0905)
#define SEC_PER_DAY          (86400)
#define SEC_PER_HOUR         (3600)
#define HOUR_PER_DAY         (24)
#define SEC_PER_MINUTE       (60)

#endif /* __42CONSTANTS_H__ */

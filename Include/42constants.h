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

#include <math.h>

#ifndef M_PI /* define various constants if they don't exist */
#define M_PI      (3.14159265358979323846) /* pi */
#define M_PI_2    (1.57079632679489661923) /* pi/2 */
#define M_SQRT2   (1.41421356237309504880) /* sqrt(2) */
#define M_SQRT1_2 (0.70710678118654752440) /* 1/sqrt(2) */
#define M_PI_4    (0.78539816339744830962) /* pi/4 */
#define M_1_PI    (0.31830988618379067154) /* 1/pi */
#define M_E       (2.7182818284590452354)  /* e */
#endif

#define PI             M_PI
#define TWOPI          (2.0 * PI)
#define HALFPI         M_PI_2
#define D2R            ((PI) / (180.0))
#define R2D            ((180.0) / (PI))
#define SQRTTWO        M_SQRT2
#define SQRTTHREE      (1.7320508075688772935)
#define SQRTHALF       M_SQRT1_2
#define GOLDENRATIO    (1.6180339887498948482)
#define SPEED_OF_LIGHT (299792458.0)

#endif /* __42CONSTANTS_H__ */

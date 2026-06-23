/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __EARTHORIKIT_H__
#define __EARTHORIKIT_H__

#include "jdkit.h"
#include "mathkit.h"

enum NutEnum { NUT_ITRF_1950, NUT_ITRF_1980, NUT_ITRF_1996 };

__attribute__((const)) double GetUt1UtcOffset(const double jday_utc_mjd);
__attribute__((const)) double JD2GMST(JDType JD);
__attribute__((const)) double HiFiJD2GMST(const JDType jd);
__attribute__((const)) pair_mat3x3_t SimpleEarthPrecNute(const JDType JD);
__attribute__((const)) pair_mat3x3_t HiFiEarthPrecNute(const JDType JD);
__attribute__((const)) pair_dbl_mat3x3_t HiFiEarthCWN(const JDType jd);

#endif /* __EARTHORIKIT_H__ */
/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __TEST_LIB_H__
#define __TEST_LIB_H__

#include "42constants.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int print_result(const long result, const char *str, long len, const int nTabs,
                 const char *trialInfo, long isOkay);

#endif
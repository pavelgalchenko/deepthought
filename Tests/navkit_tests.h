/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __NAVKIT_TESTS_H__
#define __NAVKIT_TESTS_H__

#include "42constants.h"
#include "DSMTypes.h"
#include "navkit.h"
#include "test_lib.h"
#include "timekit.h"

long RunNavKit_Tests();
long DSMMeasType_Tests();
long NavAux_Tests();

#endif
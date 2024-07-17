/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __42DSM_H__
#define __42DSM_H__

#include "42.h"
#include "42dsm.h"
#include "42types.h"
#include "dcmkit.h"
#include "dsmkit.h"
#include "fswkit.h"
#include "iokit.h"
#include "mathkit.h"
#include "navkit.h"
#include <math.h>
#include <stdlib.h>

void DsmFSW(struct SCType *S);
void DsmSensorModule(struct AcType *const AC, struct DSMType *const DSM);

#endif /* __42DSM_H__ */

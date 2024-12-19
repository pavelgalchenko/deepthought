/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "tests.h"

int main()
{
   Pi              = PI;
   TwoPi           = TWOPI;
   HalfPi          = HALFPI;
   SqrtTwo         = SQRTTWO;
   SqrtHalf        = SQRTHALF;
   GoldenRatio     = GOLDENRATIO;
   long successful = 1;

   printf("\n\e[0mMathkit Tests:\e[0m\n");
   successful &=
       print_result(RunMathKit_Tests(), "Mathkit Tests", 14, 0, "", 0, 1);

   printf("\n\e[0mNavkit Tests:\e[0m\n");
   successful &=
       print_result(RunNavKit_Tests(), "Navkit Tests", 13, 0, "", 0, 1);

   printf("\n");
   return (successful ? EXIT_SUCCESS : EXIT_FAILURE);
}
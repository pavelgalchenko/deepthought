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
   long successful = 1;

   printf("\n\e[0mMathkit Tests:\e[0m\n");
   successful &= runMathKit_Tests();
   print_result(successful, "Mathkit Tests", 14, 0, "", 0, 1);

   printf("\n");
   return (successful ? EXIT_SUCCESS : EXIT_FAILURE);
}
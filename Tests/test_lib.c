/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "test_lib.h"

/* #ifdef __cplusplus
** namespace Kit {
** #endif
*/

char *tab_str(const char *str, long len, int nTabs) {
   if (len < 0)
      len = strlen(str);

   char *outStr = calloc(len + nTabs * 4, sizeof(char));
   if (nTabs > 0) {
      for (int k = 1; k < nTabs; k++)
         strcat(outStr, "    ");
      strcat(outStr, "  - ");
   }
   strcat(outStr, str);
   return outStr;
}

void print_result(const long result, const char *str, long len,
                  const int nTabs) {
   if (result)
      print_success(str, len, nTabs);
   else
      print_failure(str, len, nTabs);
}

void print_success(const char *str, long len, const int nTabs) {
   printf("%s \e[32mPASSED\e[0m\n", tab_str(str, len, nTabs));
}

void print_failure(const char *str, long len, const int nTabs) {
   printf("%s \e[1;31mFAILED\e[0m\n", tab_str(str, len, nTabs));
}

/* #ifdef __cplusplus
** }
** #endif
*/

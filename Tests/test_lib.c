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

long TEST_MAT(const long n, const long m, const double actual[n][m],
              const double expected[n][m], const double tol)
{
   for (long i = 0; i < n; i++) {
      for (long j = 0; j < m; j++) {
         if (fabs(actual[i][j] - expected[i][j]) > tol)
            return FALSE;
      }
   }
   return TRUE;
}

long TEST_MATP(const long n, const long m, double **actual, double **expected,
               const double tol)
{
   for (long i = 0; i < n; i++) {
      for (long j = 0; j < m; j++) {
         if (fabs(actual[i][j] - expected[i][j]) > tol)
            return FALSE;
      }
   }
   return TRUE;
}

long TEST_VEC(const long n, const double actual[n], const double expected[n],
              const double tol)
{
   for (long i = 0; i < n; i++) {
      if (fabs(actual[i] - expected[i]) > tol)
         return FALSE;
   }
   return TRUE;
}

char *tab_str(const char *str, long len, int nTabs)
{
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

void print_hdr(const char *str, long len, const int nTabs)
{
   char outStr[1024] = {0};
   char *tabstr      = tab_str(str, len, nTabs);
   strcat(outStr, tabstr);
   free(tabstr);
   strcat(outStr, "\n");
   printf("%s", outStr);
}

int print_result(const long result, const char *str, long len, const int nTabs,
                 const char *trialInfo, const long isOkay,
                 const long printIfPass)
{
   if (!result || printIfPass) {
      char outStr[1024] = {0};
      char *tabstr      = tab_str(str, len, nTabs);
      strcat(outStr, tabstr);
      free(tabstr);

      if (result)
         strcat(outStr, " \e[32mPASSED\e[0m");
      else
         strcat(outStr, " \e[1;31mFAILED\e[0m");
      if (trialInfo[0] != 0) {
         char trialStr[256] = {0};
         snprintf(trialStr, 255, " on trial %s", trialInfo);
         strcat(outStr, trialStr);
      }
      if (isOkay)
         strcat(outStr, "  \e[1;33m~~OKAY~~\e[0m");
      strcat(outStr, "\n");
      printf("%s", outStr);
   }
   return result | isOkay;
}

/* #ifdef __cplusplus
** }
** #endif
*/

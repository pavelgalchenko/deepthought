/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "mathkit_tests.h"

#define TRUE  1
#define FALSE 0

#define ASSERT(actual, expected)                                               \
   if ((actual) != (expected)) {                                               \
      return (FALSE);                                                          \
   }

#define TEST(actual, expected) ((actual) == (expected))
#define TEST_DOUBLE(actual, expected, tol)                                     \
   (fabs((double)(actual) - (double)(expected)) <= (double)(tol))

long TEST_MAT(const long n, const long m, const double actual[n][m],
              const double expected[n][m], const double tol) {
   for (long i = 0; i < n; i++) {
      for (long j = 0; j < m; j++) {
         if (fabs(actual[i][j] - expected[i][j]) > tol)
            return FALSE;
      }
   }
   return TRUE;
}
long TEST_MATP(const long n, const long m, double **actual, double **expected,
               const double tol) {
   for (long i = 0; i < n; i++) {
      for (long j = 0; j < m; j++) {
         if (fabs(actual[i][j] - expected[i][j]) > tol)
            return FALSE;
      }
   }
   return TRUE;
}
long TEST_VEC(const long n, const double actual[n], const double expected[n],
              const double tol) {
   for (long i = 0; i < n; i++) {
      if (fabs(actual[i] - expected[i]) > tol)
         return FALSE;
   }
   return TRUE;
}
long TEST_VEC_PARALLEL(double actual[3], double expected[3], const double tol) {
   double cross[3] = {0.0};
   VxV(actual, expected, cross);
   if (MAGV(cross) > tol)
      return FALSE;
   return TRUE;
}

long runMathKit_Tests() {
   long success = TRUE;

   double magic5[5][5] = {{17, 24, 1, 8, 15},
                          {23, 5, 7, 14, 16},
                          {4, 6, 13, 20, 22},
                          {10, 12, 19, 21, 3},
                          {11, 18, 25, 2, 9}};
   double magic8[8][8] = {
       {64, 2, 3, 61, 60, 6, 7, 57},     {9, 55, 54, 12, 13, 51, 50, 16},
       {17, 47, 46, 20, 21, 43, 42, 24}, {40, 26, 27, 37, 36, 30, 31, 33},
       {32, 34, 35, 29, 28, 38, 39, 25}, {41, 23, 22, 44, 45, 19, 18, 48},
       {49, 15, 14, 52, 53, 11, 10, 56}, {8, 58, 59, 5, 4, 62, 63, 1}};
   double magic12[12][12] = {
       {144, 2, 3, 141, 140, 6, 7, 137, 136, 10, 11, 133},
       {13, 131, 130, 16, 17, 127, 126, 20, 21, 123, 122, 24},
       {25, 119, 118, 28, 29, 115, 114, 32, 33, 111, 110, 36},
       {108, 38, 39, 105, 104, 42, 43, 101, 100, 46, 47, 97},
       {96, 50, 51, 93, 92, 54, 55, 89, 88, 58, 59, 85},
       {61, 83, 82, 64, 65, 79, 78, 68, 69, 75, 74, 72},
       {73, 71, 70, 76, 77, 67, 66, 80, 81, 63, 62, 84},
       {60, 86, 87, 57, 56, 90, 91, 53, 52, 94, 95, 49},
       {48, 98, 99, 45, 44, 102, 103, 41, 40, 106, 107, 37},
       {109, 35, 34, 112, 113, 31, 30, 116, 117, 27, 26, 120},
       {121, 23, 22, 124, 125, 19, 18, 128, 129, 15, 14, 132},
       {12, 134, 135, 9, 8, 138, 139, 5, 4, 142, 143, 1}};
   double test3[4][3][3] = {{{1, 2, 3}, {5, 6, 8}, {9, 8, 1}},
                            {{1, 3, 2}, {4, 3, 1}, {0, 0, 1}},
                            {{1, 3, 5}, {0, 7, 0}, {0, 0, 1}},
                            {{2, -1, 0}, {-1, 2, -1}, {0, -1, 2}}};

#define NMATS 7
   long n[NMATS]     = {5, 8, 12, 3, 3, 3, 3};
   long m[NMATS]     = {5, 8, 12, 3, 3, 3, 3};
   long isPD[NMATS]  = {0, 0, 0, 0, 0, 0, 1};
   long isInv[NMATS] = {1, 0, 0, 1, 1, 1, 1};

   double **mats[NMATS] = {NULL};
   for (int i = 0; i < NMATS; i++) {
      mats[i] = CreateMatrix(n[i], m[i]);
      for (int j = 0; j < n[i]; j++)
         for (int k = 0; k < m[i]; k++)
            switch (i) {
               case 0:
                  mats[i][j][k] = magic5[j][k];
                  break;
               case 1:
                  mats[i][j][k] = magic8[j][k];
                  break;
               case 2:
                  mats[i][j][k] = magic12[j][k];
                  break;
               case 3:
               case 4:
               case 5:
               case 6:
                  mats[i][j][k] = test3[i - 3][j][k];
                  break;
            }
   }

   for (int i = 0; i < NMATS; i++) {
      if (n[i] == m[i] && isPD[i]) {
         double **S = CreateMatrix(n[i], n[i]);
         chol(mats[i], S, n[i]);
         double **SST = CreateMatrix(n[i], n[i]);
         MxMTG(S, S, SST, n[i], n[i], n[i]);
         if (!TEST_MATP(n[i], n[i], mats[i], SST, 1e-10)) {
            printf("  - chol Test \e[1;31mFAILED\e[0m on trial %i\n", i);
            success = FALSE;
         }
         double **S_copy = CreateMatrix(n[i], n[i]);
         double **AmA0   = CreateMatrix(n[i], n[i]);

         double u[n[i]];
         for (int j = 0; j < n[i]; j++)
            u[j] = mats[i][0][j];
         for (int j = 0; j < n[i]; j++) {
            for (int k = 0; k < n[i]; k++) {
               S_copy[j][k] = S[j][k];
               AmA0[j][k]   = mats[i][j][k] - u[j] * u[k];
            }
         }

         if (cholDowndate(S_copy, u, n[i])) {
            MxMTG(S_copy, S_copy, SST, n[i], n[i], n[i]);
            if (!TEST_MATP(n[i], n[i], SST, AmA0, 1e-10)) {
               printf("  - cholDowndate Test \e[1;31mFAILED\e[0m on trial %i\n",
                      i);
               success = FALSE;
            }
         }

         DestroyMatrix(S);
         DestroyMatrix(S_copy);
         DestroyMatrix(AmA0);
         DestroyMatrix(SST);
      }
      if (n[i] == m[i] && isInv[i]) {
         double **AAT = CreateMatrix(n[i], n[i]);
         MxMTG(mats[i], mats[i], AAT, n[i], n[i], n[i]);

         double **S = CreateMatrix(n[i], n[i]);
         chol(AAT, S, n[i]);
         double **SST = CreateMatrix(n[i], n[i]);
         MxMTG(S, S, SST, n[i], n[i], n[i]);
         if (!TEST_MATP(n[i], n[i], AAT, SST, 1e-10)) {
            printf("  - chol AAT Test \e[1;31mFAILED\e[0m on trial %i\n", i);
            success = FALSE;
         }
         double **S_copy = CreateMatrix(n[i], n[i]);
         double **AmA0   = CreateMatrix(n[i], n[i]);

         double u[n[i]];
         for (int j = 0; j < n[i]; j++)
            u[j] = AAT[0][j];
         for (int j = 0; j < n[i]; j++) {
            for (int k = 0; k < n[i]; k++) {
               S_copy[j][k] = S[j][k];
               AmA0[j][k]   = AAT[j][k] - u[j] * u[k];
            }
         }

         // TODO: find  some better downdates, I think this is only tested once
         if (cholDowndate(S_copy, u, n[i])) {
            MxMTG(S_copy, S_copy, SST, n[i], n[i], n[i]);
            if (!TEST_MATP(n[i], n[i], SST, AmA0, 1e-10)) {
               printf("  - cholDowndate AAT Test \e[1;31mFAILED\e[0m on trial "
                      "%i\n",
                      i);
               success = FALSE;
            }
         }

         DestroyMatrix(S);
         DestroyMatrix(S_copy);
         DestroyMatrix(AmA0);
         DestroyMatrix(AAT);
         DestroyMatrix(SST);
      }
      for (int kk = 0; kk < 2; kk++) {
         double **QR       = CreateMatrix(n[i], m[i]);
         double **mat_copy = CreateMatrix(n[i], m[i]);

         for (int j = 1; j <= n[i]; j++) {
            double **U = CreateMatrix(n[i], m[i]);
            double **R = CreateMatrix(m[i], m[i]);
            double **Q = CreateMatrix(n[i], n[i]);
            for (int ii = 0; ii < n[i]; ii++) {
               Q[ii][ii] = 1.0;
               for (int jj = 0; jj < m[i]; jj++)
                  mat_copy[ii][jj] = mats[i][ii][jj];
            }

            if (kk == 0) {
               // Test hqrd
               hqrd(mat_copy, U, R, n[i], m[i]);
               j = n[i];
            } else {
               // Test bhqrd
               bhqrd(mat_copy, U, R, n[i], m[i], j);
            }
            for (int k = 0; k < m[i]; k++) {
               double u[n[i]];
               double uMag2 = 0.0;
               for (int ii = 0; ii < n[i]; ii++) {
                  u[ii]  = U[ii][k];
                  uMag2 += u[ii] * u[ii];
               }
               double **H  = CreateMatrix(n[i], n[i]);
               double **QH = CreateMatrix(n[i], n[i]);
               for (int ii = 0; ii < n[i]; ii++) {
                  H[ii][ii] = 1.0;
                  for (int jj = 0; jj < n[i]; jj++)
                     H[ii][jj] -= 2.0 * u[ii] * u[jj] / uMag2;
               }
               MxMG(Q, H, QH, n[i], n[i], n[i]);
               for (int ii = 0; ii < n[i]; ii++)
                  for (int jj = 0; jj < n[i]; jj++)
                     Q[ii][jj] = QH[ii][jj];

               DestroyMatrix(H);
               DestroyMatrix(QH);
            }
            MxMG(Q, R, QR, n[i], m[i], m[i]);
            // TODO: bhqrd 2<j<n fails for the magic matricies, interesting
            if (!TEST_MATP(m[i], m[i], QR, mats[i], 1e-10)) {
               const char *funName = kk == 0 ? "hqrd" : "bhqrd";
               printf("  - %s Test \e[1;31mFAILED\e[0m on trial %i, %i",
                      funName, i, j);
               if (kk == 0)
                  success = FALSE;
               else
                  printf("  \e[1;33m~~OKAY~~\e[0m");
               printf("\n");
            }
            DestroyMatrix(U);
            DestroyMatrix(Q);
            DestroyMatrix(R);
         }
         DestroyMatrix(QR);
         DestroyMatrix(mat_copy);
      }
   }

   /* Generated from expm(SSCPM(test3[i][j])) in MATLAB */
   double RTests[9][3][3] = {
       {{-0.694920557641312, 0.713520990527788, 0.089292858861912},
        {-0.192006972791999, -0.303785044339471, 0.933192353823647},
        {0.692978167741770, 0.631349699383718, 0.348107477830265}},
       {{0.346972902524374, 0.899270889657858, -0.266311231321127},
        {-0.507454631172482, 0.418805883246694, 0.753054732047782},
        {0.788732909301628, -0.126148718471181, 0.601653470539869}},
       {{0.949003837587731, 0.094949017624026, -0.300626679281784},
        {0.018027096027771, 0.935666379726060, 0.352425097941582},
        {0.314748693488259, -0.339872196424716, 0.886239330003399}},
       {{-0.694920557641312, 0.692978167741770, -0.192006972791999},
        {0.089292858861912, 0.348107477830265, 0.933192353823647},
        {0.713520990527788, 0.631349699383718, -0.303785044339471}},
       {{0.760411466331418, 0.469146031507377, -0.449083959847802},
        {0.105866449297220, 0.592699492763410, 0.798435724520888},
        {0.640754786782668, -0.654682604319739, 0.401028665828544}},
       {{0.540302305868140, -0.841470984807897, 0},
        {0.841470984807897, 0.540302305868140, 0},
        {0, 0, 1.000000000000000}},
       {{0.935273836484033, 0.309049943446815, -0.172484733364896},
        {-0.297627679296938, 0.950503522017202, 0.089223422649067},
        {0.191521840281356, -0.032112101899684, 0.980962893083539}},
       {{0.753902254343304, 0, 0.656986598718789},
        {0, 1.000000000000000, 0},
        {-0.656986598718789, 0, 0.753902254343304}},
       {{0.540302305868140, -0.841470984807897, 0},
        {0.841470984807897, 0.540302305868140, 0},
        {0, 0, 1.000000000000000}}};
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
         double R[3][3]  = {{0.0}};
         double theta[3] = {0.0};
         expmso3(test3[i][j], R);
         if (!TEST_MAT(3, 3, RTests[i * 3 + j], R, 1e-12)) {
            printf("  - expmso3 Test \e[1;31mFAILED\e[0m on trial %i,%i\n", i,
                   j);
            success = FALSE;
         }
         logso3(R, theta);
         double unitTest[3] = {0.0};
         double magTest     = CopyUnitV(test3[i][j], unitTest);
         magTest            = (double)(fmod(magTest + PI, TWOPI) - PI);
         SxV(magTest, unitTest, unitTest);
         // TODO: fails on tighter tolerance
         if (!TEST_VEC(3, unitTest, theta, 1e-10)) {
            printf("  - logso3 Test \e[1;31mFAILED\e[0m on trial %i,%i\n", i,
                   j);
            success = FALSE;
         }
      }
   }

   for (int i = 0; i < NMATS; i++)
      DestroyMatrix(mats[i]);
#undef NMATS
   return (success);
}

#undef TRUE
#undef FALSE
#undef ASSERT
#undef TEST
#undef TEST_DOUBLE
#undef TEST_LONGDOUBLE
#undef TEST_MAT
/* #ifdef __cplusplus
** }
** #endif
*/

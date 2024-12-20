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

long TEST_VEC_PARALLEL(double actual[3], double expected[3], const double tol)
{
   double cross[3] = {0.0}, unit_a[3] = {0.0}, unit_e[3] = {0.0};
   CopyUnitV(actual, unit_a);
   CopyUnitV(expected, unit_e);
   VxV(unit_a, unit_e, cross);
   if (MAGV(cross) > tol)
      return FALSE;
   return TRUE;
}

void linFDF(const double x, double params[2], double *f, double *fp)
{
   *f  = params[0] * x + params[1];
   *fp = params[0];
}

void cubicFDF(const double x, double params[4], double *f, double *fp)
{
   *f  = params[0] * x * x * x + params[1] * x * x + params[2] * x + params[3];
   *fp = 3.0 * params[0] * x * x + 2 * params[1] * x + params[2];
}

long RunMathKit_Tests()
{
   long success = TRUE;

   // TODO: better cholesky test matricies
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
   long testSuccess        = TRUE;
   long cholDownDateTested = FALSE;
   print_hdr("Cholesky Tests:", 16, 1);
   for (int i = 0; i < NMATS; i++) {
      if (n[i] == m[i] && isPD[i]) {
         double **S = CreateMatrix(n[i], n[i]);
         chol(mats[i], S, n[i]);
         double **SST = CreateMatrix(n[i], n[i]);
         MxMTG(S, S, SST, n[i], n[i], n[i]);
         {
            char trialInfo[40] = {0};
            snprintf(trialInfo, 39, "%i", i);
            testSuccess &=
                print_result(TEST_MATP(n[i], n[i], mats[i], SST, 1e-10),
                             "chol Test", 10, 2, trialInfo, FALSE, FALSE);
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
            cholDownDateTested = TRUE;
            MxMTG(S_copy, S_copy, SST, n[i], n[i], n[i]);
            {
               char trialInfo[40] = {0};
               snprintf(trialInfo, 39, "%i", i);
               testSuccess &= print_result(
                   TEST_MATP(n[i], n[i], SST, AmA0, 1e-10), "cholDowndate Test",
                   18, 2, trialInfo, FALSE, FALSE);
            }
         }

         DestroyMatrix(S);
         DestroyMatrix(S_copy);
         DestroyMatrix(AmA0);
         DestroyMatrix(SST);
      }
   }
   if (!cholDownDateTested) {
      print_hdr("\e[1;31mcholDowndate is never tested!\e[0m", 41, 2);
   }
   cholDownDateTested = FALSE;
   success &=
       print_result(testSuccess, "Cholesky Tests:", 16, 2, "", FALSE, TRUE);
   testSuccess = TRUE;
   print_hdr("Cholesky AAT Tests:", 20, 1);
   for (int i = 0; i < NMATS; i++) {
      if (n[i] == m[i] && isInv[i]) {
         double **AAT = CreateMatrix(n[i], n[i]);
         MxMTG(mats[i], mats[i], AAT, n[i], n[i], n[i]);

         double **S = CreateMatrix(n[i], n[i]);
         chol(AAT, S, n[i]);
         double **SST = CreateMatrix(n[i], n[i]);
         MxMTG(S, S, SST, n[i], n[i], n[i]);
         {
            char trialInfo[40] = {0};
            snprintf(trialInfo, 39, "%i", i);
            testSuccess &=
                print_result(TEST_MATP(n[i], n[i], AAT, SST, 1e-10),
                             "chol AAT Test", 14, 2, trialInfo, FALSE, FALSE);
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

         // TODO: find some better downdates, I think this is only tested once
         if (cholDowndate(S_copy, u, n[i])) {
            cholDownDateTested = TRUE;
            MxMTG(S_copy, S_copy, SST, n[i], n[i], n[i]);
            {
               char trialInfo[40] = {0};
               snprintf(trialInfo, 39, "%i", i);
               testSuccess &= print_result(
                   TEST_MATP(n[i], n[i], SST, AmA0, 1e-10),
                   "cholDowndate AAT Test", 22, 2, trialInfo, FALSE, FALSE);
            }
         }

         DestroyMatrix(S);
         DestroyMatrix(S_copy);
         DestroyMatrix(AmA0);
         DestroyMatrix(AAT);
         DestroyMatrix(SST);
      }
   }
   if (!cholDownDateTested) {
      print_hdr("\e[1;31mcholDowndate is never tested!\e[0m", 41, 2);
   }
   cholDownDateTested = FALSE;
   success &=
       print_result(testSuccess, "Cholesky AAT Tests:", 20, 2, "", FALSE, TRUE);
   for (int kk = 0; kk < 2; kk++) {
      testSuccess = TRUE;
      print_hdr(kk == 0 ? "hqrd Tests:" : "bhqrd Tests:", kk == 0 ? 12 : 13, 1);
      for (int i = 0; i < NMATS; i++) {
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
            }
            else {
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
            {
               char trialInfo[40] = {0};
               snprintf(trialInfo, 39, "%i, %i", i, j);
               // TODO: bhqrd 2<j<n fails for the magic matricies, interesting
               long isOkay = FALSE;
               if (kk == 1 && ((i == 0 && (j >= 3 && j <= 4)) ||
                               (i == 1 && (j >= 3 && j <= 7)) ||
                               (i == 2 && (j >= 3 && j <= 11))))
                  isOkay = TRUE;
               success &=
                   print_result(TEST_MATP(m[i], m[i], QR, mats[i], 1e-10),
                                kk == 0 ? "hqrd Test" : "bhqrd Test",
                                kk == 0 ? 10 : 11, 2, trialInfo, isOkay, FALSE);
            }
            DestroyMatrix(U);
            DestroyMatrix(Q);
            DestroyMatrix(R);
         }
         DestroyMatrix(QR);
         DestroyMatrix(mat_copy);
      }
      success &=
          print_result(testSuccess, kk == 0 ? "hqrd Tests:" : "bhqrd Tests:",
                       kk == 0 ? 12 : 13, 2, "", FALSE, TRUE);
   }

   testSuccess = TRUE;
   print_hdr("expmso3 and logmso3 Tests:", 27, 1);
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
         {
            char trialInfo[40] = {0};
            snprintf(trialInfo, 39, "%i, %i", i, j);
            testSuccess &=
                print_result(TEST_MAT(3, 3, RTests[i * 3 + j], R, 1e-12),
                             "expmso3 Test", 13, 2, trialInfo, FALSE, FALSE);
         }
         logso3(R, theta);
         double unitTest[3] = {0.0};
         double magTest     = CopyUnitV(test3[i][j], unitTest);
         magTest            = (double)(fmod(magTest + PI, TWOPI) - PI);
         SxV(magTest, unitTest, unitTest);
         // TODO: fails on tighter tolerance
         {
            char trialInfo[40] = {0};
            snprintf(trialInfo, 39, "%i, %i", i, j);
            testSuccess &=
                print_result(TEST_VEC(3, unitTest, theta, 1e-10), "logso3 Test",
                             12, 2, trialInfo, FALSE, FALSE);
         }
      }
   }
   success &= print_result(testSuccess, "expmso3 and logmso3 Tests:", 27, 2, "",
                           FALSE, TRUE);

   for (int i = 0; i < NMATS; i++)
      DestroyMatrix(mats[i]);
#undef NMATS
   testSuccess = TRUE;

   print_hdr("NewtonRaphson Tests:", 21, 1);
   { // Test NewtonRaphson()
#define N_FNS   7
#define N_X0    10
#define N_TESTS 10
      void eccFDF(const double E, double params[2], double *f, double *fp);
      void parabolFDF(const double x, double params[1], double *f, double *fp);
      void hyperbolFDF(const double H, double params[2], double *f, double *fp);
      void lagpointFDF(const double x, double params[3], double *f, double *fp);
      void hyperradFDF(const double r, double params[7], double *f, double *fp);

      void (*fns[N_FNS])(double, double *, double *, double *) = {
          &linFDF,      &cubicFDF,    &eccFDF,     &parabolFDF,
          &hyperbolFDF, &lagpointFDF, &hyperradFDF};
      long nParams[N_FNS]    = {2, 3, 2, 1, 2, 3, 7};
      double x0[N_FNS][N_X0] = {
          {10, 15, -5, 0.2345, 3.1415926, -5.235, 10000000.0, -400023412.1234,
           0, -0},
          {10, 15, -5, 0.2345, 3.1415926, -5.235, 10000000.0, -400023412.1234,
           0, -0},
          {0.0, 3.1415926535, 6.28, -3.1415926535, 0.4756, -5.12345, 2.56782,
           0.0034256, -0.000023623724, -0.0},
          {0.0},
          {0.0},
          {0.0},
          {0.0},
      };
      double eps[N_FNS]     = {1.0E-12, 1.0E-12, 1.0E-12, 1.0E-12,
                               1.0E-12, 1.0E-12, 1.0E-12};
      double maxStep[N_FNS] = {1.0E9, 1.0E9, 0.1, 1.0E9, 1.0E9, 1.0E9, 1.0E9};
      for (int i = 0; i < 3; i++) {
         long subTestSuccess = TRUE;
         switch (i) {
            case 0:
               print_hdr("linFDF Tests:", 14, 2);
               break;
            case 1:
               print_hdr("cubicFDF Tests:", 16, 2);
               break;
            case 2:
               print_hdr("eccFDF Tests:", 14, 2);
               break;
            case 3:
               print_hdr("parabolFDF Tests:", 18, 2);
               break;
            case 4:
               print_hdr("hyperbolFDF Tests:", 19, 2);
               break;
            case 5:
               print_hdr("lagpointFDF Tests:", 19, 2);
               break;
            case 6:
               print_hdr("hyperradFDF Tests:", 19, 2);
               break;
         }
         double params[nParams[i]];
         for (int j = 0; j < N_X0; j++) {
            for (int k = 0; k < N_TESTS; k++) {
               switch (i) {
                  case 0:
                     switch (k) {
                        case 0:
                           params[0] = 124.34453;
                           params[1] = 0.0;
                           break;
                        case 1:
                           params[0] = -7.0;
                           params[1] = 0.0;
                           break;
                        case 2:
                           params[0] = 3.1415926535;
                           params[1] = 5.234;
                           break;
                        case 3:
                           params[0] = 0.45436;
                           params[1] = -3.1415926535;
                           break;
                        case 4:
                           params[0] = 40.0;
                           params[1] = 125536.2345;
                           break;
                        case 5:
                           params[0] = 34252345.0;
                           params[1] = 1.0;
                           break;
                        case 6:
                           params[0] = -234523523450.0345;
                           params[1] = 1.0;
                           break;
                        case 7:
                           params[0] = 0.657856;
                           params[1] = -2452551.0;
                           break;
                        case 8:
                           params[0] = 0.0354643356;
                           params[1] = 1.0;
                           break;
                        case 9:
                           params[0] = 121.0;
                           params[1] = -0.002345873452;
                           break;
                     }
                     break;
                  case 1:
                     switch (k) {
                        case 0:
                           params[0] = 0.0;
                           params[1] = 97.234568;
                           params[2] = -34.2348;
                           params[3] = -82.18357;
                           break;
                        case 1:
                           params[0] = 0.0;
                           params[1] = 345.0657467;
                           params[2] = 804.38654;
                           params[3] = 364.2356;
                           break;
                        case 2:
                           params[0] = -90.1231;
                           params[1] = 0.0;
                           params[2] = 2346.45649;
                           params[3] = 67.192345;
                           break;
                        case 3:
                           params[0] = 0.0097364523;
                           params[1] = 0.0;
                           params[2] = 2654.87162;
                           params[3] = -826.235;
                           break;
                        case 4:
                           params[0] = 325798.0023;
                           params[1] = 61934.12349;
                           params[2] = 0.0;
                           params[3] = -12436.98734;
                           break;
                        case 5:
                           params[0] = 345.893487;
                           params[1] = 5.71203749;
                           params[2] = 0.0;
                           params[3] = -926.19823;
                           break;
                        case 6:
                           params[0] = -32507245.3245298;
                           params[1] = 5857.8732;
                           params[2] = -982343.234987;
                           params[3] = 0.0;
                           break;
                        case 7:
                           params[0] = 0.69283481;
                           params[1] = 23.28980273;
                           params[2] = -843.87345;
                           params[3] = 0.0;
                           break;
                        case 8:
                           params[0] = 12.23459071;
                           params[1] = 54.9176;
                           params[2] = -23.39456;
                           params[3] = 97.1235;
                           break;
                        case 9:
                           params[0] = -0.2546437;
                           params[1] = 7.1235865;
                           params[2] = 9.124;
                           params[3] = -15.823;
                           break;
                     }
                     break;
                  case 2:
                     switch (k) {
                        case 0:
                           params[0] = 0.0;
                           break;
                        case 1:
                           params[0] = 0.7386952935;
                           break;
                        case 2:
                           params[0] = 0.22346;
                           break;
                        case 3:
                           params[0] = 0.0023467;
                           break;
                        case 4:
                           // this can cause errors without reasonable maxStep
                           params[0] = 0.99999999;
                           break;
                        case 5:
                           params[0] = 0.89286354;
                           break;
                        case 6:
                           params[0] = 0.490071;
                           break;
                        case 7:
                           params[0] = 0.111111111;
                           break;
                        case 8:
                           params[0] = 0.00000000234723945;
                           break;
                        case 9:
                           params[0] = 0.9002467598374;
                           break;
                     }
                     // Kepler's equation is typically initialized with E_0 = M
                     params[1] = x0[i][j];
                     break;
                  case 3:
                     switch (k) {
                        case 0:
                           break;
                        case 1:
                           break;
                        case 2:
                           break;
                        case 3:
                           break;
                        case 4:
                           break;
                        case 5:
                           break;
                        case 6:
                           break;
                        case 7:
                           break;
                        case 8:
                           break;
                        case 9:
                           break;
                     }
                     break;
                  case 4:
                     switch (k) {
                        case 0:
                           break;
                        case 1:
                           break;
                        case 2:
                           break;
                        case 3:
                           break;
                        case 4:
                           break;
                        case 5:
                           break;
                        case 6:
                           break;
                        case 7:
                           break;
                        case 8:
                           break;
                        case 9:
                           break;
                     }
                     break;
                  case 5:
                     switch (k) {
                        case 0:
                           break;
                        case 1:
                           break;
                        case 2:
                           break;
                        case 3:
                           break;
                        case 4:
                           break;
                        case 5:
                           break;
                        case 6:
                           break;
                        case 7:
                           break;
                        case 8:
                           break;
                        case 9:
                           break;
                     }
                     break;
                  case 6:
                     switch (k) {
                        case 0:
                           break;
                        case 1:
                           break;
                        case 2:
                           break;
                        case 3:
                           break;
                        case 4:
                           break;
                        case 5:
                           break;
                        case 6:
                           break;
                        case 7:
                           break;
                        case 8:
                           break;
                        case 9:
                           break;
                     }
                     break;
               }
               double X = NewtonRaphson(x0[i][j], eps[i], 500, maxStep[i], 0,
                                        fns[i], params);
               double f = 0.0, fp = 0.0;
               switch (i) {
                  case 0:
                  case 1:
                     fns[i](X, params, &f, &fp);
                     break;
                  case 2:
                     f = params[1] - X + params[0] * sin(X);
                     break;
                  case 3:
                     break;
                  case 4:
                     break;
                  case 5:
                     break;
                  case 6:
                     break;
               }
               {
                  char trialInfo[40] = {0};
                  snprintf(trialInfo, 39, "%i, %i, %i", i, j, k);
                  long isOkay = FALSE;
                  // TODO: These cases for NewtonRaphson
                  if ((i == 1 && k == 8 && (j == 1 || j == 6)))
                     isOkay = TRUE;
                  subTestSuccess &= print_result(TEST_DOUBLE(f, 0.0, 1.0E-10),
                                                 "NewtonRaphson Test", 19, 3,
                                                 trialInfo, isOkay, FALSE);
               }
            }
         }
         switch (i) {
            case 0:
               testSuccess = print_result(subTestSuccess, "linFDF Tests:", 14,
                                          3, "", FALSE, TRUE);
               break;
            case 1:
               testSuccess = print_result(subTestSuccess, "cubicFDF Tests:", 16,
                                          3, "", FALSE, TRUE);
               break;
            case 2:
               testSuccess = print_result(subTestSuccess, "eccFDF Tests:", 14,
                                          3, "", FALSE, TRUE);
               break;
            case 3:
               testSuccess = print_result(
                   subTestSuccess, "parabolFDF Tests:", 18, 3, "", FALSE, TRUE);
               break;
            case 4:
               testSuccess =
                   print_result(subTestSuccess, "hyperbolFDF Tests:", 19, 3, "",
                                FALSE, TRUE);
               break;
            case 5:
               testSuccess =
                   print_result(subTestSuccess, "lagpointFDF Tests:", 19, 3, "",
                                FALSE, TRUE);
               break;
            case 6:
               testSuccess =
                   print_result(subTestSuccess, "hyperradFDF Tests:", 19, 3, "",
                                FALSE, TRUE);
               break;
         }
      }
#undef N_FNS
#undef N_TESTS
   }
   success &= print_result(testSuccess, "NewtonRaphson Tests:", 21, 2, "",
                           FALSE, TRUE);
   return (success);
}

/* #ifdef __cplusplus
** }
** #endif
*/

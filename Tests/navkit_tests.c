/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "navkit_tests.h"

struct DSMMeasType *DSMMeasType_New(long step, long subStep, long sensorNum,
                                    enum sensorType sensor,
                                    double underWeighting, double probGate,
                                    struct DSMMeasType *nextMeas)
{
   void ConfigureMeas(struct DSMMeasType * meas, enum sensorType sensor);
   struct DSMMeasType *newMeas = malloc(sizeof(struct DSMMeasType));

   ConfigureMeas(newMeas, sensor);
   newMeas->step    = step;
   newMeas->subStep = subStep;
   newMeas->data    = calloc(newMeas->dim, sizeof(double));

   newMeas->sensorNum      = sensorNum;
   newMeas->underWeighting = underWeighting;
   newMeas->probGate       = probGate;

   newMeas->nextMeas = nextMeas;
   return newMeas;
}

long RunNavKit_Tests()
{
   long successful = TRUE;
   print_hdr("DSMMeasType Tests:", 19, 1);
   successful &= print_result(DSMMeasType_Tests(), "DSMMeasType Tests", 18, 2,
                              "", FALSE, TRUE);

   print_hdr("NavAux Tests:", 19, 1);
   successful &=
       print_result(NavAux_Tests(), "NavAux Tests", 13, 2, "", FALSE, TRUE);

   return (successful);
}

long DSMMeasType_Tests()
{
   long successful = TRUE;
   enum sensorType GetSensorValue(char *string);

   // TODO: move some of this stuff to a dsmkit_tests
   ASSERT(GetSensorValue("GPS"), GPS_SENSOR);
   ASSERT(GetSensorValue("STARTRACK"), STARTRACK_SENSOR);
   ASSERT(GetSensorValue("FSS"), FSS_SENSOR);
   ASSERT(GetSensorValue("CSS"), CSS_SENSOR);
   ASSERT(GetSensorValue("GYRO"), GYRO_SENSOR);
   ASSERT(GetSensorValue("MAG"), MAG_SENSOR);
   ASSERT(GetSensorValue("ACCEL"), ACCEL_SENSOR);
   ASSERT(GetSensorValue("  "), NULL_SENSOR);

   struct DSMMeasType *meas1 =
       DSMMeasType_New(30, 0, 0, GPS_SENSOR, 0, 0, NULL);
   struct DSMMeasType *meas2 =
       DSMMeasType_New(40, 0, 0, GPS_SENSOR, 0, 0, NULL);
   struct DSMMeasType *meas3 =
       DSMMeasType_New(30, 0, 0, CSS_SENSOR, 0, 0, NULL);
   struct DSMMeasType *meas4 =
       DSMMeasType_New(30, 1, 0, CSS_SENSOR, 0, 0, NULL);
   struct DSMMeasType *meas5 =
       DSMMeasType_New(30, 1, 1, CSS_SENSOR, 0, 0, NULL);
   struct DSMMeasType *meas6 =
       DSMMeasType_New(30, 1, 0, GYRO_SENSOR, 0, 0, NULL);
   struct DSMMeasType *meas7 =
       DSMMeasType_New(30, 1, 0, MAG_SENSOR, 0, 0, NULL);

   ASSERT(comparator_DSMMeas((void *)&meas1, (void *)&meas2), -1);
   ASSERT(comparator_DSMMeas((void *)&meas2, (void *)&meas1), +1);
   ASSERT(comparator_DSMMeas((void *)&meas1, (void *)&meas1), 0);
   ASSERT(comparator_DSMMeas((void *)&meas3, (void *)&meas4), -1);
   ASSERT(comparator_DSMMeas((void *)&meas4, (void *)&meas3), +1);
   ASSERT(comparator_DSMMeas((void *)&meas3, (void *)&meas3), 0);
   ASSERT(comparator_DSMMeas((void *)&meas1, (void *)&meas3), -1);
   ASSERT(comparator_DSMMeas((void *)&meas3, (void *)&meas1), +1);
   ASSERT(comparator_DSMMeas((void *)&meas4, (void *)&meas5), -1);
   ASSERT(comparator_DSMMeas((void *)&meas5, (void *)&meas4), +1);

   struct DSMMeasListType list[1];
   InitMeasList(list);
   appendMeas(list, meas2);
   ASSERT(list->head, meas2);
   appendMeas(list, meas1);
   ASSERT(list->head->nextMeas, meas1);
   appendMeas(list, meas3);
   ASSERT(list->head->nextMeas->nextMeas, meas3);
   ASSERT(list->head->nextMeas->nextMeas->nextMeas, NULL);
   appendMeas(list, meas6);
   appendMeas(list, meas5);
   appendMeas(list, meas7);
   appendMeas(list, meas4);

   bubbleSort(list);
   struct DSMMeasType *sorted[7] = {meas1, meas3, meas4, meas5,
                                    meas6, meas7, meas2};
   struct DSMMeasType *checkMeas = list->head;
   for (int i = 0; i < 7; i++) {
      char trialInfo[40] = {0};
      snprintf(trialInfo, 39, "%i", i);
      successful &= print_result(TEST(checkMeas, sorted[i]),
                                 "DSMMeasType Bubble Sort Test", 29, 2,
                                 trialInfo, FALSE, FALSE);
      checkMeas   = checkMeas->nextMeas;
   }

   DestroyMeas(meas1);
   DestroyMeas(meas2);
   DestroyMeas(meas3);
   DestroyMeas(meas4);
   DestroyMeas(meas5);
   DestroyMeas(meas6);
   DestroyMeas(meas7);
   return (successful);
}

long NavAux_Tests()
{
   double GetPriMerAng(const long orbCenter, const struct DateType *date);
   long success = TRUE;

   const long testGpsRollover[] = {1, 1, 1, 1, 2, 2, 2};
   const long testGpsWeek[]     = {18, 128, 728, 546, 28, 245, 146};
   const double testGpsSecond[] = {561548.816, 51321.5,  423891.7,
                                   598213.781, 245603.0, 12489.4516,
                                   102961.452};
   for (int i = 0; i < 7; i++) {
      double testSec     = gpsTime2J2000Sec(testGpsRollover[i], testGpsWeek[i],
                                            testGpsSecond[i]);
      double gpsTime     = GpsDateToGpsTime(testGpsRollover[i], testGpsWeek[i],
                                            testGpsSecond[i]);
      char trialInfo[40] = {0};
      snprintf(trialInfo, 39, "%i", i);
      success &=
          print_result(TEST_DOUBLE(gpsTime + 19 + 32.184, testSec, 1e-5),
                       "gpsTime2J2000Sec Test", 22, 2, trialInfo, FALSE, FALSE);

      struct DateType date;
      date.JulDay = TimeToJD(testSec);
      JDToDate(date.JulDay, &date.Year, &date.Month, &date.Day, &date.Hour,
               &date.Minute, &date.Second);
      const double updateAmount[] = {0,        -51.184, 15,  86400,
                                     -94513.4, 215489,  3600};
      for (int j = 0; j < 7; j++) {
         struct DateType date_copy = date;
         updateNavTime(&date_copy, updateAmount[j]);
         double updatedTime =
             DateToTime(date_copy.Year, date_copy.Month, date_copy.Day,
                        date_copy.Hour, date_copy.Minute, date_copy.Second);

         snprintf(trialInfo, 39, "%i, %i", i, j);
         // TODO: timing should probably be in long double
         // otherwise this fails on any tighter tolerance
         success &= print_result(
             TEST_DOUBLE(updatedTime, testSec + updateAmount[j], 1e-4),
             "updateNavTime Test", 19, 2, trialInfo, FALSE, FALSE);
      }

      // This will be deprecated when spice is integrated anyway
      double priMerAng = GetPriMerAng(EARTH, &date);
      double testEarthPriMerAng =
          TwoPi * JD2GMST(date.JulDay - 32.184 / 86400.0);
      snprintf(trialInfo, 39, "%i", i);
      success &=
          print_result(TEST_DOUBLE(priMerAng, testEarthPriMerAng, 1e-8),
                       "GetPriMerAng Test", 18, 2, trialInfo, FALSE, FALSE);
   }
   return (success);
}

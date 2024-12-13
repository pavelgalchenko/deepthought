/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "dataFilter.h"

extern void WriteToSocket(SOCKET Socket, struct AcType *AC);
extern void ReadFromSocket(SOCKET Socket, struct AcType *AC);

struct fy_document *InitFilter(int argc, char **argv)
{
   struct OrbitType *Eph;
   char response[120], response1[120], response2[120];
   double r1[3], rh[3], vh[3];
   double Zaxis[3] = {0.0, 0.0, 1.0};
   long Iorb, Isc, i, j, Ip, Im, Iw, Nm;
   long MinorBodiesExist;
   long JunkTag;
   double CGJ2000[3][3] = {
       {-0.054873956175539, -0.873437182224835, -0.483835031431981},
       {0.494110775064704, -0.444828614979805, 0.746981957785302},
       {-0.867665382947348, -0.198076649977489, 0.455985113757595}};
   double CJ2000H[3][3];

   Pi          = PI;
   TwoPi       = TWOPI;
   HalfPi      = HALFPI;
   SqrtTwo     = SQRTTWO;
   SqrtHalf    = SQRTHALF;
   GoldenRatio = GOLDENRATIO;

   qJ2000H[0] = -0.203123038887;
   qJ2000H[1] = 0.0;
   qJ2000H[2] = 0.0;
   qJ2000H[3] = 0.979153221449;

   sprintf(InOutPath, "./InOut/");
   sprintf(ModelPath, "./Model/");
   if (argc > 1)
      sprintf(InOutPath, "./%s/", argv[1]);
   if (argc > 2)
      sprintf(ModelPath, "./%s/", argv[2]);

   char tempargs[BUFSIZE];
   char *ret;
   DIR *OutDir;
   DIR *ModelDir;

#ifdef __linux__
   strcpy(ExeDir, realpath("/proc/self/exe", NULL));
   ret = strrchr(ExeDir, '/');
#elif defined __MINGW32__
   GetModuleFileName(NULL, tempargs, sizeof(tempargs));
   _fullpath(ExeDir, tempargs, sizeof(tempargs));
   ret = strrchr(ExeDir, '\\');
#elif defined _WIN32
   GetModuleFileName(NULL, tempargs, sizeof(tempargs));
   _fullpath(ExeDir, tempargs, sizeof(tempargs));
   ret = strrchr(ExeDir, '\\');
#elif defined _WIN64
   GetModuleFileName(NULL, tempargs, sizeof(tempargs));
   _fullpath(ExeDir, tempargs, sizeof(tempargs));
   ret = strrchr(ExeDir, '\\');
#elif defined __APPLE__
   uint32_t bytes;
   bytes = 1000;
   bytes = sizeof("/0");
   _NSGetExecutablePath("/0", &bytes);
   _NSGetExecutablePath(tempargs, &bytes);
   realpath(tempargs, ExeDir);
   ret = strrchr(ExeDir, '/');
#endif

   if (ret != NULL)
      *ret = '\0';
   ret = strrchr(ExeDir, '.');
   if (ret != NULL)
      *ret = '\0';

   strcpy(ModelPath, ExeDir);
   strcat(ModelPath, "/Model/");

   CLI_ARGS = docopt(argc, argv, /* help */ 1);

   if (CLI_ARGS.indir != NULL) {
      sprintf(InOutPath, "%s", CLI_ARGS.indir);
      strcat(InOutPath, "/");
   }

   if (CLI_ARGS.outdir != NULL) {
      strcpy(OutPath, CLI_ARGS.outdir);
      printf("%s", OutPath);
      strcat(OutPath, "/");
      OutDir = opendir(OutPath);
      if (OutDir) {
         closedir(OutDir);
      }
      else if (ENOENT == errno) {
#if defined __MINGW32__
         mkdir(OutPath);
#elif defined _WIN32
         mkdir(OutPath);
#elif defined _WIN64
         mkdir(OutPath);
#elif defined __APPLE__
         mkdir(OutPath, 0777);
#elif defined __linux__
         mkdir(OutPath, 0777);
#else
#error "Computing platform not detected!"
#endif
      }
   }

   if (CLI_ARGS.defaultdir != NULL) {
      strcat(CLI_ARGS.defaultdir, "/");
      if (CLI_ARGS.indir == NULL) {
         strcpy(InOutPath, CLI_ARGS.defaultdir);
         strcat(InOutPath, "/InOut/");
      }
      if (CLI_ARGS.outdir == NULL) {
         strcpy(OutPath, CLI_ARGS.defaultdir);
         strcat(OutPath, "/InOut/");
      }
      if (CLI_ARGS.modeldir == NULL) {
         strcpy(SCModelPath, CLI_ARGS.defaultdir);
         strcat(SCModelPath, "/Model/");
         ModelDir = opendir(SCModelPath);
         if (ModelDir) {
            closedir(ModelDir);
         }
         else if (ENOENT == errno) {
            strcpy(SCModelPath, ModelPath);
         }
      }
   }
   else { /* Default Directories */
      if (CLI_ARGS.indir == NULL) {
         strcpy(InOutPath, ExeDir);
         strcat(InOutPath, "/InOut/");
      }
      if (CLI_ARGS.outdir == NULL)
         strcpy(OutPath, InOutPath);
      if (CLI_ARGS.modeldir == NULL)
         strcpy(SCModelPath, ModelPath);
   }

   printf("\nExeDir: %s \n", ExeDir);
   printf("\nInput Path: %s \n", InOutPath);

   /* .. Read from file Inp_Sim.txt */
   struct fy_document *fyd =
       fy_document_build_and_check(NULL, InOutPath, "Inp_Sim.yaml");

   struct fy_node *root = fy_document_root(fyd);
   struct fy_node *node = fy_node_by_path_def(root, "/Simulation Control");

   char config_file[50] = {0};
   fy_node_scanf(node, "/Sensor Config File %49[^\n]", config_file);

   /* .. Time Mode */
   /* .. Duration, Step size */
   /* .. File output interval */
   /* .. RNG Seed */
   /* .. Graphics Front End? */
   /* .. Cmd Script File Name */
   if (fy_node_scanf(node,
                     "/Mode %119s "
                     "/Duration %lf "
                     "/Step Size %lf ",
                     response, &STOPTIME, &DTSIM) != 3) {
      printf("Simulation Control in Inp_Sim is improperly configured. "
             "Exiting...\n");
      exit(EXIT_FAILURE);
   }
   GLEnable = FALSE;

   /* .. Reference Orbits */
   node = fy_node_by_path_def(root, "/Orbits");
   Norb = fy_node_sequence_item_count(node);
   Orb  = NULL;
   Orb  = (struct OrbitType *)calloc(Norb, sizeof(struct OrbitType));
   if (Orb == NULL) {
      printf("Orb calloc returned null pointer.  Bailing out!\n");
      exit(EXIT_FAILURE);
   }
   Frm = NULL;
   Frm = (struct FormationType *)calloc(Norb, sizeof(struct FormationType));
   if (Frm == NULL) {
      printf("Frm calloc returned null pointer.  Bailing out!\n");
      exit(EXIT_FAILURE);
   }

   struct fy_node *iterNode = NULL;
   Iorb                     = 0;
   WHILE_FY_ITER(node, iterNode)
   {
      if (!fy_node_scanf(iterNode, "/Name %39[^\n]s", Orb[Iorb].FileName)) {
         printf("Could not find Orbit name. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      strcat(Orb[Iorb].FileName, ".yaml");
      Orb[Iorb].Exists = getYAMLBool(fy_node_by_path_def(iterNode, "/Enabled"));
      Orb[Iorb].Tag    = Iorb;
      Iorb++;
   }

   /* .. Spacecraft */
   node = fy_node_by_path_def(root, "/SCs");
   Nsc  = fy_node_sequence_item_count(node);
   SC   = NULL;
   SC   = (struct SCType *)calloc(Nsc, sizeof(struct SCType));
   if (SC == NULL) {
      printf("SC calloc returned null pointer.  Bailing out!\n");
      exit(EXIT_FAILURE);
   }

   iterNode = NULL;
   Isc      = 0;
   WHILE_FY_ITER(node, iterNode)
   {
      if (fy_node_scanf(iterNode,
                        "/Name %49s "
                        "/Orbit %19s",
                        SC[Isc].FileName, response) != 2) {
         printf("Could not find SC's name and/or its orbit. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      strcat(SC[Isc].FileName, ".yaml");
      strcat(response, ".yaml");
      SC[Isc].RefOrb = -1;
      for (Iorb = 0; Iorb < Norb; Iorb++) {
         if (!strcmp(response, Orb[Iorb].FileName)) {
            SC[Isc].RefOrb = Iorb;
            break;
         }
      }
      if (SC[Isc].RefOrb == -1) {
         printf("SC[%ld] named %49s is assigned to invalid orbit %19s. "
                "Exiting...\n",
                Isc, SC[Isc].FileName, response);
         exit(EXIT_FAILURE);
      }
      SC[Isc].Exists = getYAMLBool(fy_node_by_path_def(iterNode, "/Enabled"));
      if ((SC[Isc].Exists && !Orb[SC[Isc].RefOrb].Exists) ||
          (SC[Isc].RefOrb > Norb)) {
         printf("Yo!  SC[%ld] is assigned to non-existent Orb[%ld]\n", Isc,
                SC[Isc].RefOrb);
         exit(EXIT_FAILURE);
      }
      SC[Isc].ID = Isc;
      Isc++;
   }

   /* .. Environment */
   /* .. Date and time (UTC) */
   node          = fy_node_by_path_def(root, "/Time");
   long millisec = 0;
   if (fy_node_scanf(node,
                     "/Date/Year %ld "
                     "/Date/Month %ld "
                     "/Date/Day %ld "
                     "/Time/Hour %ld "
                     "/Time/Minute %ld "
                     "/Time/Second %lf "
                     "/Time/Millisecond %ld "
                     "/Leap Seconds %lf",
                     &UTC.Year, &UTC.Month, &UTC.Day, &UTC.Hour, &UTC.Minute,
                     &UTC.Second, &millisec, &LeapSec) != 8) {
      printf("Time is improperly configured in Inp_Sim. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   UTC.Second += millisec / 1000.0;

   /* .. Choices for Modeling Solar Activity */
   // TODO: add atmo model properties to world and use this to
   // configure properties
   node     = fy_node_by_path_def(root, "/Perturbation Models");
   iterNode = NULL;
   WHILE_FY_ITER(fy_node_by_path_def(node, "/Atmosphere/Models"), iterNode)
   {
      if (fy_node_scanf(iterNode,
                        "/World %119s "
                        "/Method %119s",
                        response1, response2) != 2) {
         printf("Could not find World and/or Method for Atmospheric Model. "
                "Exiting...\n");
         exit(EXIT_FAILURE);
      }
      Iw            = DecodeString(response1);
      long atmoType = DecodeString(response2);
      double f10p7 = 0.0, geomag = 0.0;
      if (atmoType == USER_ATMO)
         if (fy_node_scanf(iterNode, "/F10.7 %lf /Ap %lf", &f10p7, &geomag) !=
             2) {
            printf("Could not find user defined F10.7 and/or Ap. Exiting...\n");
            exit(EXIT_FAILURE);
         }

      switch (Iw) {
         case EARTH:
            AtmoOption  = atmoType;
            Flux10p7    = f10p7;
            GeomagIndex = geomag;
            break;
         default:
            printf("World %119s does not have a configured atmospheric model. "
                   "Exiting...\n",
                   response1);
            exit(EXIT_FAILURE);
            break;
      }
   }

   /* .. Magnetic Field Model */
   // TODO: make magfield a property of worlds, so each world can have a
   // configurable magnetic field
   // TODO: make magfield coefficent files a field for models?
   iterNode = NULL;
   WHILE_FY_ITER(fy_node_by_path_def(node, "/Magnetic/Models"), iterNode)
   {
      if (fy_node_scanf(iterNode,
                        "/World %119s "
                        "/Method %119s",
                        response1, response2) != 2) {
         printf("Could not find World and/or Method for Magnetic Model. "
                "Exiting...\n");
         exit(EXIT_FAILURE);
      }
      Iw           = DecodeString(response1);
      long magType = DecodeString(response2);
      switch (Iw) {
         case EARTH:
            MagModel.Type = magType;
            break;
         default:
            printf("World %119s does not have a configured magnetic field "
                   "model. Exiting...\n",
                   response1);
            exit(EXIT_FAILURE);
            break;
      }
      if (magType == IGRF) {
         long N = 0, M = 0;
         if (fy_node_scanf(iterNode,
                           "/Degree %ld "
                           "/Order %ld",
                           &N, &M) != 2) {
            printf("Could not find Degree and/or Order for Magnetic Field "
                   "Model. Exiting...\n");
            exit(EXIT_FAILURE);
         }
         switch (Iw) {
            case EARTH:
               MagModel.N = N;
               MagModel.M = M;
               break;
            default:
               printf("World %119s does not have a configured spherical "
                      "harmonic magnetic field model. Exiting...\n",
                      response1);
               exit(EXIT_FAILURE);
               break;
         }
      }
   }

   /* .. Earth, Mars, Luna Gravity Models */
   // TODO: make gravfield a property of worlds, so each world can have a
   // configurable gravitational
   // TODO: make gravfield coefficent files a field for models?
   iterNode = NULL;
   WHILE_FY_ITER(fy_node_by_path_def(node, "/Gravitation/Models"), iterNode)
   {
      long N = 0, M = 0;
      if (fy_node_scanf(iterNode,
                        "/World %119s "
                        "/Degree %ld "
                        "/Order %ld",
                        response, &N, &M) != 3) {
         printf("Could not find World, Degree, and/or Order for Gravitational "
                "Model. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      Iw = DecodeString(response);
      switch (Iw) {
         case EARTH:
         case MARS:
         case LUNA:
            World[Iw].GravModel.N = N;
            World[Iw].GravModel.M = M;
            break;
         default:
            printf("World %119s does not have a configured spherical harmonic "
                   "gravity model. Exiting...\n",
                   response);
            exit(EXIT_FAILURE);
            break;
      }
   }

   /* .. Toggle on/off various environmental effects */
   AeroActive = getYAMLBool(fy_node_by_path_def(node, "/Atmosphere/Enabled"));
   AeroShadowsActive =
       getYAMLBool(fy_node_by_path_def(node, "/Atmosphere/Shadows"));
   GGActive =
       getYAMLBool(fy_node_by_path_def(node, "/Gravitation/Gravity Gradient"));
   SolPressActive = getYAMLBool(fy_node_by_path_def(node, "/SRP/Enabled"));
   SolPressShadowsActive =
       getYAMLBool(fy_node_by_path_def(node, "/SRP/Shadows"));
   ResidualDipoleActive =
       getYAMLBool(fy_node_by_path_def(node, "/Magnetic/Residual Mag Moment"));
   GravPertActive =
       getYAMLBool(fy_node_by_path_def(node, "/Gravitation/Enabled"));
   ThrusterPlumesActive =
       getYAMLBool(fy_node_by_path_def(node, "/Thruster Plume"));
   ContactActive = getYAMLBool(fy_node_by_path_def(node, "/Contact"));
   SloshActive   = getYAMLBool(fy_node_by_path_def(node, "/CFD Slosh"));
   AlbedoActive  = getYAMLBool(fy_node_by_path_def(node, "/Albedo on CSS"));
   ComputeEnvTrq =
       getYAMLBool(fy_node_by_path_def(node, "/Output Env Torques to File"));

   /* .. Celestial Bodies */
   if (!fy_node_scanf(root, "/Ephem Type %119s", response)) {
      printf("Could not find Ephemeris Type in Inp_Sim. Exiting...\n");
      exit(EXIT_FAILURE);
   }
   EphemOption = DecodeString(response);
   node        = fy_node_by_path_def(root, "/Celestial Bodies");
   // I wish this was more programmatic, but it doesn't really need to be I
   // guess
   World[MERCURY].Exists = getYAMLBool(fy_node_by_path_def(node, "/Mercury"));
   World[VENUS].Exists   = getYAMLBool(fy_node_by_path_def(node, "/Venus"));
   World[EARTH].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Earth and Luna"));
   World[MARS].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Mars and its moons"));
   World[JUPITER].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Jupiter and its moons"));
   World[SATURN].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Saturn and its moons"));
   World[URANUS].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Uranus and its moons"));
   World[NEPTUNE].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Neptune and its moons"));
   World[PLUTO].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Pluto and its moons"));
   MinorBodiesExist =
       getYAMLBool(fy_node_by_path_def(node, "/Asteroids and Comets"));

   /* .. Lagrange Point Systems */
   node = fy_node_by_path_def(root, "/Lagrange Systems");
   LagSys[EARTHMOON].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Earth-Moon"));
   LagSys[SUNEARTH].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Sun-Earth"));
   LagSys[SUNJUPITER].Exists =
       getYAMLBool(fy_node_by_path_def(node, "/Sun-Jupiter"));

   /* .. Ground Stations */
   node          = fy_node_by_path_def(root, "/Ground Stations");
   Ngnd          = fy_node_sequence_item_count(node);
   GroundStation = (struct GroundStationType *)calloc(
       Ngnd, sizeof(struct GroundStationType));
   iterNode = NULL;
   WHILE_FY_ITER(node, iterNode)
   {
      struct fy_node *seqNode =
          fy_node_by_path_def(iterNode, "/Ground Station");
      long Ignd = 0;
      if (!fy_node_scanf(seqNode, "/Index %ld", &Ignd)) {
         printf("Could not find Ground Station Index. Exiting...\n");
         exit(EXIT_FAILURE);
      }
      if (fy_node_scanf(seqNode,
                        "/World %119s "
                        "/Longitude %lf "
                        "/Latitude %lf",
                        response, &GroundStation[Ignd].lng,
                        &GroundStation[Ignd].lat) != 3) {
         printf("Ground Station %ld is improperly configured. Exiting...\n",
                Ignd);
         exit(EXIT_FAILURE);
      }
      size_t str_len;
      const char *label =
          fy_node_get_scalar(fy_node_by_path_def(seqNode, "/Label"), &str_len);
      strncpy(GroundStation[Ignd].Label, label, str_len > 39 ? 39 : str_len);
      GroundStation[Ignd].World = DecodeString(response);
      GroundStation[Ignd].Exists =
          getYAMLBool(fy_node_by_path_def(iterNode, "/Ground Station/Enabled"));
   }

   fy_document_destroy(fyd);
   /* .. Load Materials */
   Nmatl = 0;
   Matl  = AddMtlLib(ModelPath, "42.mtl", Matl, &Nmatl);
   ScaleSpecDiffFrac(Matl, Nmatl);

   /* Known bug: First Geom loaded in gets corrupted.
   Kludge fix: Load a sacrificial geom first.  */
   Geom = LoadWingsObjFile(ModelPath, "Point.obj", &Matl, &Nmatl, Geom, &Ngeom,
                           &JunkTag, FALSE);

   /* .. Time */
   if (TimeMode == EXTERNAL_TIME) {
      printf("Initializing with External Time\n");
      RealSystemTime(&UTC.Year, &UTC.doy, &UTC.Month, &UTC.Day, &UTC.Hour,
                     &UTC.Minute, &UTC.Second, DTSIM);
   }
   CivilTime  = DateToTime(UTC.Year, UTC.Month, UTC.Day, UTC.Hour, UTC.Minute,
                           UTC.Second);
   AtomicTime = CivilTime + LeapSec;
   DynTime0   = AtomicTime + 32.184;
   GpsTime    = AtomicTime - 19.0;
   DynTime    = DynTime0;

   TT.JulDay = TimeToJD(DynTime);
   TimeToDate(DynTime, &TT.Year, &TT.Month, &TT.Day, &TT.Hour, &TT.Minute,
              &TT.Second, DTSIM);
   TT.doy = MD2DOY(TT.Year, TT.Month, TT.Day);

   UTC.JulDay = TimeToJD(CivilTime);
   UTC.doy    = MD2DOY(UTC.Year, UTC.Month, UTC.Day);

   GpsTimeToGpsDate(GpsTime, &GpsRollover, &GpsWeek, &GpsSecond);

/* .. Load Sun and Planets */
#ifdef _ENABLE_SPICE_
   if (EphemOption == EPH_SPICE)
      LoadSpiceKernels(
          ModelPath); // Load SPICE to get SPICE-provided values for mu, J2, etc
#endif

   LoadSun();
   LoadPlanets();

   /* JPL planetary ephems */
   if (EphemOption == EPH_DE430 || EphemOption == EPH_DE440)
      LoadJplEphems(ModelPath, TT.JulDay);
#ifdef _ENABLE_SPICE_
   else if (EphemOption == EPH_SPICE)
      LoadSpiceEphems(DynTime);
#endif
   /* .. Load Moons */
   if (World[EARTH].Exists)
      LoadMoonOfEarth();
   if (World[MARS].Exists)
      LoadMoonsOfMars();
   if (World[JUPITER].Exists)
      LoadMoonsOfJupiter();
   if (World[SATURN].Exists)
      LoadMoonsOfSaturn();
   if (World[URANUS].Exists)
      LoadMoonsOfUranus();
   if (World[NEPTUNE].Exists)
      LoadMoonsOfNeptune();
   if (World[PLUTO].Exists)
      LoadMoonsOfPluto();

   /* .. Asteroids and Comets */
   if (MinorBodiesExist)
      LoadMinorBodies();
   else
      Nmb = 0;

   /* .. Regions */
   LoadRegions();

   /* .. Galactic Frame */
   Q2C(qJ2000H, CJ2000H);
   MxM(CGJ2000, CJ2000H, CGH);

   /* .. Ground Station Locations */
   for (i = 0; i < Ngnd; i++) {
      if (GroundStation[i].Exists && !World[GroundStation[i].World].Exists) {
         printf("Ground Station[%ld].World doesn't exist.\n", i);
      }
      GroundStation[i].PosW[0] = World[GroundStation[i].World].rad *
                                 cos(GroundStation[i].lng * D2R) *
                                 cos(GroundStation[i].lat * D2R);
      GroundStation[i].PosW[1] = World[GroundStation[i].World].rad *
                                 sin(GroundStation[i].lng * D2R) *
                                 cos(GroundStation[i].lat * D2R);
      GroundStation[i].PosW[2] =
          World[GroundStation[i].World].rad * sin(GroundStation[i].lat * D2R);
   }

   /* .. Locate Luna */
   if (World[LUNA].Exists) {
      Eph = &World[LUNA].eph;
      /* Meeus computes Luna Position in geocentric ecliptic */
      LunaPosition(TT.JulDay, rh);
      LunaPosition(TT.JulDay + 0.01, r1);
      for (j = 0; j < 3; j++)
         vh[j] = (r1[j] - rh[j]) / (864.0);
      /* Convert to Earth's N frame */
      MxV(World[EARTH].CNH, rh, Eph->PosN);
      MxV(World[EARTH].CNH, vh, Eph->VelN);
      /* Find Luna's osculating elements */
      RV2Eph(DynTime, Eph->mu, Eph->PosN, Eph->VelN, &Eph->SMA, &Eph->ecc,
             &Eph->inc, &Eph->RAAN, &Eph->ArgP, &Eph->anom, &Eph->tp, &Eph->SLR,
             &Eph->alpha, &Eph->rmin, &Eph->MeanMotion, &Eph->Period);
      World[LUNA].PriMerAng = atan2(Eph->PosN[1], Eph->PosN[0]) + Pi;
      SimpRot(Zaxis, World[LUNA].PriMerAng, World[LUNA].CWN);
      C2Q(World[LUNA].CWN, World[LUNA].qwn);
      for (j = 0; j < 3; j++) {
         World[LUNA].PosH[j] = rh[j] + World[EARTH].PosH[j];
         World[LUNA].VelH[j] = vh[j] + World[EARTH].VelH[j];
      }
   }

   /* .. Other planets' moons */
   for (Ip = MARS; Ip <= PLUTO; Ip++) {
      if (World[Ip].Exists) {
         Nm = World[Ip].Nsat;
         for (Im = 0; Im < Nm; Im++) {
            Iw  = World[Ip].Sat[Im];
            Eph = &World[Iw].eph;
            Eph2RV(Eph->mu, Eph->SLR, Eph->ecc, Eph->inc, Eph->RAAN, Eph->ArgP,
                   DynTime - Eph->tp, Eph->PosN, Eph->VelN, &Eph->anom);
            World[Iw].PriMerAng = fmod(World[i].w * DynTime, TwoPi);
            SimpRot(Zaxis, World[Iw].PriMerAng, World[Iw].CWN);
            C2Q(World[Iw].CWN, World[Iw].qwn);
            MTxV(World[Ip].CNH, Eph->PosN, rh);
            MTxV(World[Ip].CNH, Eph->VelN, vh);
            for (i = 0; i < 3; i++) {
               World[Iw].PosH[i] = rh[i] + World[Ip].PosH[i];
               World[Iw].VelH[i] = vh[i] + World[Ip].VelH[i];
            }
         }
      }
   }

   InitLagrangePoints();
   for (Iorb = 0; Iorb < Norb; Iorb++) {
      if (Orb[Iorb].Exists)
         InitOrbit(&Orb[Iorb]);
   }
   OrbitMotion(DynTime);
   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         InitSpacecraft(&SC[Isc]);
      }
   }
   long nonDSMFSW = FALSE, DSMFSW = FALSE;
   for (Isc = 0; Isc < Nsc; Isc++) {
      if (SC[Isc].Exists) {
         DSMFSW    |= SC[Isc].FswTag == DSM_FSW;
         nonDSMFSW |= SC[Isc].FswTag != DSM_FSW;
         if (nonDSMFSW && DSMFSW) {
            printf("Mixing DSM_FSW and non DSM_FSW flightsoftware tags is not "
                   "supported. Exiting...\n");
            exit(EXIT_FAILURE);
         }
      }
   }

   LoadTdrs();
   LoadSchatten();
   struct fy_document *doc =
       fy_document_build_and_check(NULL, InOutPath, config_file);
   return doc;
}

void SelectStatementFromMapping(struct fy_node *map_node,
                                char *select_statement, size_t *select_size)
{
   struct fy_node *iter_node = NULL;
   WHILE_FY_ITER(map_node, iter_node)
   {
      struct fy_node_pair *iter_pair = NULL;
      while (fy_node_mapping_iterate(iter_node, (void *)&iter_pair) != NULL) {
         struct fy_node *value_node = fy_node_pair_value(iter_pair);
         struct fy_node *data_node  = fy_node_by_path_def(value_node, "/data");
         if (fy_node_is_sequence(data_node)) {
            struct fy_node *data_iter_node = NULL;
            WHILE_FY_ITER(data_node, data_iter_node)
            {
               size_t data_str_len = 0;
               const char *data_str =
                   fy_node_get_scalar(data_iter_node, &data_str_len);
               *select_size +=
                   sprintf(select_statement + *select_size, "%s", ", ");
               strncat(select_statement, data_str, data_str_len);
               *select_size += data_str_len;
            }
         }
         else if (fy_node_is_scalar(data_node)) {
            size_t data_str_len  = 0;
            const char *data_str = fy_node_get_scalar(data_node, &data_str_len);
            *select_size +=
                sprintf(select_statement + *select_size, "%s", ", ");
            strncat(select_statement, data_str, data_str_len);
            *select_size += data_str_len;
         }
      }
   }
}

char *ConstructSQLStatement(struct fy_node *config_root,
                            const enum db_read_config_flags flags,
                            const ccsdsCoarse ccsds_min,
                            const ccsdsCoarse ccsds_max)
{
   const size_t buf_size = 4096; // idk on size for the moment
   size_t select_size = 0, where_size = 0;
   char *select_out = calloc(buf_size, sizeof(char));
   char *where_out  = calloc(buf_size, sizeof(char));

   select_size += sprintf(select_out + select_size, "%s",
                          "SELECT CCSDS_SECONDS, CCSDS_SUBSECS");
   if (flags & DRCF_OUT_FILE)
      select_size += sprintf(select_out + select_size, "%s", ", file");
   else if (flags & DRCF_OUT_DIR_TIME)
      select_size +=
          sprintf(select_out + select_size, "%s", ", directory_time");
   else if (flags & DRCF_OUT_LIVE_PASS)
      select_size += sprintf(select_out + select_size, "%s", ", live_pass");
   else if (flags & DRCF_OUT_TIME_AMBIGUOUS)
      select_size +=
          sprintf(select_out + select_size, "%s", ", time_ambiguous");

   if (ccsds_min > 0)
      where_size +=
          sprintf(where_out + where_size, " AND CCSDS_SECONDS>%u", ccsds_min);
   if (ccsds_max > 0)
      where_size +=
          sprintf(where_out + where_size, " AND CCSDS_SECONDS<%u", ccsds_max);
   if (flags & DRCF_COND_TIME_UNAMBIGUOUS)
      where_size +=
          sprintf(where_out + where_size, "%s", " AND time_ambiguous IS 0");
   else if (flags & DRCF_COND_TIME_AMBIGUOUS)
      where_size +=
          sprintf(where_out + where_size, "%s", " AND time_ambiguous IS 1");

   struct fy_node *config_node =
       fy_node_by_path_def(config_root, "/Data Mapping Configurations");

   const char *key_names[10] = {"/Gyro", "/MAG",   "/CSS", "/FSS", "/ST",
                                "/GPS",  "/Accel", "/Whl", "/MTB", "/THR"};
   const enum db_read_config_flags data_flags[10] = {
       DRCF_DATA_GYRO, DRCF_DATA_MAG, DRCF_DATA_CSS,   DRCF_DATA_FSS,
       DRCF_DATA_ST,   DRCF_DATA_GPS, DRCF_DATA_ACCEL, DRCF_DATA_WHL,
       DRCF_DATA_MTB,  DRCF_DATA_THR};

   for (int i = 0; i < 10; i++) {
      struct fy_node *node = fy_node_by_path_def(config_node, key_names[i]);
      if (node != NULL && (flags & data_flags[i])) {
         if (i >= 1 && i <= 3) {
            const char *name = &key_names[i][1];
            if (flags & DRCF_OUT_VALID)
               select_size +=
                   sprintf(select_out + select_size, ", %s_VALID", name);
            if (flags & DRCF_COND_VALID) {
               where_size +=
                   sprintf(where_out + where_size, " AND %s_VALID IS 1", name);
            }
            else if (flags & DRCF_COND_INVALID) {
               where_size +=
                   sprintf(where_out + where_size, " AND %s_VALID IS 0", name);
            }
         }
         SelectStatementFromMapping(node, select_out, &select_size);
      }
   }

   memmove(where_out + 2, where_out, where_size);
   strncpy(where_out, " WHERE", 6);
   strcat(select_out, where_out);
   free(where_out);
   strcat(select_out, " ORDER BY CCSDS_SECONDS DESC, CCSDS_SUBSECS DESC");
   return select_out;
}

void read_db(const char *db_name, struct SCType *const S,
             struct DateType *db_time)
{
   static sqlite3 *db        = NULL;
   static sqlite3_stmt *stmt = NULL;

   const enum db_read_config_flags db_read_flags =
       DRCF_COND_TIME_UNAMBIGUOUS | DRCF_DATA_SENSORS | DRCF_DATA_ACT;

   const char *yaml_file_name = "snoopi_sensor_map.yaml";
   const char *data_dir       = "/Users/dnewber2/Desktop/Smallsat Analysis/";
   struct fy_document *yaml_config =
       fy_document_build_and_check(NULL, data_dir, yaml_file_name);
   struct fy_node *yaml_config_root = fy_document_root(yaml_config);

   struct DSMMeasListType *meas_list = NULL;

   if (db == NULL) {
      char *const db_file_name = malloc(strlen(data_dir) + strlen(db_name));
      strcat(db_file_name, data_dir);
      strcat(db_file_name, db_name);
      int rc = sqlite3_open(db_file_name, &db);
      free(db_file_name);

      if (rc) {
         fprintf(stderr, "Can't open database %s: %s\n", db_name,
                 sqlite3_errmsg(db));
         exit(EXIT_FAILURE);
      }
      const char *zsql =
          ConstructSQLStatement(yaml_config_root, db_read_flags, 0, 0);
      char *tail;
      rc = sqlite3_prepare_v3(db, zsql, -1, 0, &stmt, NULL);
      if (rc) {
         fprintf(stderr, "Can't compile statement %s: %s\n", zsql,
                 sqlite3_errmsg(db));
         exit(EXIT_FAILURE);
      }
      sqlite3_step(stmt);

      // TODO: get db state up to db_time
      InitMeasList(meas_list);
   }
   else {
      InitMeasList(meas_list);
      // TODO: output db data from last db state up to db_time

      // TODO: get date from current index in db
      struct DateType cur_db_time;

      ccsdsCoarse seconds;
      ccsdsFine subseconds;

      DateToCCSDS(cur_db_time, &seconds, &subseconds);

      // TODO: conversions from data in db to meas_list data
   }

   // Add db data to nav measurement buffer
   appendList(&S->DSM.DsmNav.measList, meas_list);
   free(meas_list);
}

int main(int argc, char **argv)
{

   const char *hostname = "localhost";
   const int port       = 10001;

   /* INIT */
   /* Init SC, World, Orbit                                                   */
   /* Init AC                                                                 */
   /* S->FswTag to DSM_FSW                                                    */
   /* Init time from Inp_Sim.yaml                                             */
   /* Init db                                                                 */

   /* load sensor map yaml                                                    */
   struct fy_document *config_fyd = InitFilter(argc, argv);
   // SOCKET socket                  = InitSocketClient(hostname, port, 1);

   /* LOOP */
   /* Every loop: */
   /*    get time from ipc */
   /*    check if time is ahead of next time in db */
   /*       true: load data up to time after current into Nav->measList */
   /*             remap data if necessary */
   /*    run DsmFSW at AC->DT to current time */
   /*    Set S state from AC state */
   /*    return data on IPC */

   struct SCType *const S  = &SC[0];
   struct AcType *const AC = &S->AC;
   DsmFSW(S);                          // Init DSM
   const char *db_name = "SC_DATA.db"; // the name
   read_db(db_name, S, &UTC);

   while (TRUE) {
      // gets time and other data from socket
      // ReadFromSocket(socket, AC);

      // updated db to time
      read_db(db_name, S, &UTC);

      // run dsm fsw and therefore nav
      DsmFSW(S);

      // write new data to socket
      // TODO: need to set S state from AC here, and that goes out the socket
      // WriteToSocket(socket, AC);
   }

   fy_document_destroy(config_fyd);
   return (EXIT_SUCCESS);
}

/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __42GL_H__
#define __42GL_H__

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "glkit.h"

/*
** #ifdef __cplusplus
** namespace _42 {
** using namespace Kit;
** #endif
*/

enum CAM_MENU {
   N_AXES = 0,
   L_AXES,
   F_AXES,
   B_AXES,
   N_GRID,
   L_GRID,
   F_GRID,
   B_GRID,
   G_GRID,
   CAM_FOV,
   PROX_OPS,
   TDRS,
   CAM_SHADOWS,
   ASTRO,
   TRUTH_VECTORS,
   FSW_VECTORS,
   MILKY_WAY,
   FERMI_SKY,
};
#define CAM_MENU_SIZE (FERMI_SKY + 1)

enum MAP_MENU {
   MAP_CLOCK = 0,
   MAP_TLM_CLOCK,
   MAP_CREDITS,
   MAP_NIGHT,
};
#define MAP_MENU_SIZE (MAP_NIGHT + 1)

#define MONOCULAR 0
#define LEFTEYE   1
#define RIGHTEYE  2
#define ONSCREEN  0
#define OFFSCREEN 1

EXTERN long MapWindowExists, OrreryWindowExists, SphereWindowExists;

EXTERN double RunTime, FrameRate, RealTimeDT;
EXTERN long TimerHasExpired;
EXTERN int TimerDuration;

EXTERN double MouseClickX, MouseClickY;
EXTERN long PauseFlag;
EXTERN long PausedByMouse;
EXTERN long MouseDown;
EXTERN long CamWidth, CamHeight;
EXTERN char CamTitle[80];
EXTERN long CamShow[CAM_MENU_SIZE];
EXTERN char CamShowLabel[CAM_MENU_SIZE][40];
EXTERN long MapWidth, MapHeight;
EXTERN char MapTitle[80];
EXTERN long MapShow[MAP_MENU_SIZE];
EXTERN char MapShowLabel[MAP_MENU_SIZE][40];
EXTERN long ShowHUD;
EXTERN double MouseScaleFactor;
EXTERN long CaptureCam, CamFrame;
EXTERN GLuint SunTexTag;
EXTERN GLuint SunlightTexTag;
EXTERN GLuint SunlightRingTexTag[3];
EXTERN GLuint RockballTexTag;
EXTERN GLuint RockballColCubeTag;
EXTERN GLuint RockballBumpCubeTag;
EXTERN GLuint IceballTexTag;
EXTERN GLuint IceballColCubeTag;
EXTERN GLuint Iceball2TexTag;
EXTERN GLuint Iceball2ColCubeTag;
EXTERN GLuint NullTexTag;
EXTERN GLuint NullColCubeTag;
EXTERN GLuint NullBumpCubeTag;
EXTERN GLuint NullCloudGlossCubeTag;
EXTERN GLuint NullRingTexTag;
EXTERN GLuint ShadowTexTag;
EXTERN GLint MapSunVecLoc;
EXTERN GLint MapCosEclLoc;
EXTERN GLint MoonMapSunVecLoc;
EXTERN GLint MoonMapCosEclLoc;
EXTERN GLuint NASAWatermarkTexTag;
EXTERN GLuint GSFCWatermarkTexTag;
EXTERN GLuint FortyTwoWatermarkTexTag;

EXTERN long OrreryWidth;
EXTERN long OrreryHeight;
EXTERN char OrreryTitle[40];
EXTERN struct OrreryPOVType Orrery;
EXTERN GLuint OrrerySphereList;
EXTERN GLuint OrreryRingList;

EXTERN GLuint DayNightTexTag;
EXTERN GLuint EarthMapTexTag;
EXTERN GLuint SunSpriteTexTag;
EXTERN GLuint AntiSunSpriteTexTag;
EXTERN GLuint MoonSpriteTexTag;
EXTERN GLuint LogoTexTag;
EXTERN GLuint SphereSunSpriteTexTag;
EXTERN GLuint SphereMoonSpriteTexTag;

EXTERN char StarCatFileName[80];
EXTERN double BuckyPf[32][3];
EXTERN long BuckyNeighbor[32][6];

EXTERN double SkyDistance;
EXTERN double GammaCorrection;

EXTERN GLuint MilkyWayList;
EXTERN GLuint StarList[32];
EXTERN GLuint FermiSkyList;
EXTERN GLuint EgretSourceList[32];
EXTERN GLuint FermiSourceList[32];
EXTERN GLuint PulsarList[32];
EXTERN GLuint NSkyGridList;
EXTERN GLuint LSkyGridList;
EXTERN GLuint FSkyGridList;
EXTERN GLuint BSkyGridList;
EXTERN GLuint PlanetList[11];
EXTERN GLuint NightList;
EXTERN GLuint MajSkyGridList;
EXTERN GLuint MinSkyGridList;
EXTERN GLuint SphereList;

EXTERN GLfloat DistantAmbientLightColor[4];
EXTERN GLfloat LocalAmbientLightColor[4];
EXTERN GLfloat DistantDiffuseLightColor[4];
EXTERN GLfloat LocalDiffuseLightColor[4];
EXTERN GLfloat SpecularLightColor[4];
EXTERN GLfloat NBrightColor[4];
EXTERN GLfloat NDimColor[4];
EXTERN GLfloat LBrightColor[4];
EXTERN GLfloat LDimColor[4];
EXTERN GLfloat FBrightColor[4];
EXTERN GLfloat FDimColor[4];
EXTERN GLfloat BBrightColor[4];
EXTERN GLfloat BDimColor[4];
EXTERN GLfloat GBrightColor[4];
EXTERN GLfloat GDimColor[4];
EXTERN GLfloat HUDColor[4];
EXTERN GLfloat TdrsColor[4];

EXTERN struct WidgetType PovWidget;
EXTERN struct WidgetType HostWidget;
EXTERN struct WidgetType TargetWidget;
EXTERN struct WidgetType CamShowWidget;

EXTERN struct WidgetType OrreryWidget;

EXTERN long NumSphereWindowMenuLines;
EXTERN long SphereWindowWidth;
EXTERN long SphereWindowHeight;
EXTERN struct WidgetType CenterWidget;
EXTERN struct WidgetType SphereShowWidget;
EXTERN struct WidgetType VectorsWidget;
EXTERN struct WidgetType FOVsWidget;
EXTERN struct WidgetType GridsWidget;
EXTERN struct WidgetType AxesWidget;
EXTERN long ShowConstellations[3];

EXTERN double PixelScale;

EXTERN long TlmIsStatic;
EXTERN long UseEphFromTlm;
EXTERN long ShowTdrsVis;
EXTERN long ShowWatermark;

EXTERN GLint ColorTexSamplerLoc;
EXTERN GLint BumpTexSamplerLoc;
EXTERN GLint EnvMapSamplerLoc;
EXTERN GLint NoiseTexSamplerLoc;
EXTERN GLint SpectrumTexSamplerLoc;
EXTERN GLint ShadowSamplerLoc;
EXTERN GLint NoiseGainLoc;
EXTERN GLint NoiseBiasLoc;
EXTERN GLint NoiseScaleLoc;
EXTERN GLint NoiseAxisLoc;
EXTERN GLint NoiseTypeLoc;
EXTERN GLint ColorTexEnabledLoc;
EXTERN GLint BumpTexEnabledLoc;
EXTERN GLint ReflectEnabledLoc;
EXTERN GLint NoiseColEnabledLoc;
EXTERN GLint NoiseBumpEnabledLoc;
EXTERN GLint ShadowsEnabledLoc;
EXTERN GLint CNELoc;
EXTERN GLint ShadowMatrixLoc;

EXTERN GLuint SkyCube;
EXTERN GLuint NoiseTex;

EXTERN long ShadowsEnabled;
EXTERN long VREnabled;
EXTERN long SeeThruPassNeeded;
EXTERN GLfloat ShadowMatrix[16];

EXTERN long NearWorld;

EXTERN GLfloat LightPosN[4];

EXTERN GLfloat ShadowFromNMatrix[16]; /* 4x4 Matrix transforms from N frame to
                                         Shadow Texture space */
EXTERN GLfloat CNE[9];                /* DCM between N and Eye frame */

EXTERN char Banner[120];
EXTERN GLfloat BannerColor[4];

void GeomToDisplayLists(struct GeomType *G);
void UpdatePOV(void);
void CamRenderExec(void);
void DrawMap(void);
void DrawOrrery(void);
void DrawUnitSphere(void);
void SetPovOrientation(void);
void InitCamWidgets(void);
void InitOrreryWidget(void);
void InitSphereWidgets(void);
void LoadShadowMapFBO(void);
void Load3DNoise(void);
void LoadCamLists(void);
void LoadCamTextures(void);
void LoadCamShaders(void);
void LoadMapShaders(void);
void ReadGraphicsInpFile(void);
void LoadFOVs(void);
void InitColors(void);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __42GL_H__ */

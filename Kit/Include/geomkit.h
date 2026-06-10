/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __GEOMKIT_H__
#define __GEOMKIT_H__

#include "42constants.h"
#include "dcmkit.h"
#include "defineskit.h"
#include "iokit.h"
#include "mathkit.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

struct SilEdgeType {
   long Body;
   long Iv1, Iv2;
   vec3 PosV1B;
   vec3 PosV2B;
   vec3 PosV1N;
   vec3 PosV2N;
};

struct SilVtxType {
   long Body;
   vec3 PosB;
   vec3 PosN;
};

struct PolyType {
   long Nv;
   long HasTex;
   long HasNorm;
   long HasBump;
   long *V;
   long *Vt;
   long *Vn;
   long *E;
   long Matl;
   double Area;
   vec3 Norm;
   mat3x3 Dyad;     /* For polyhedron gravity */
   vec3 Uhat, Vhat; /* In-plane basis vectors */
   vec3 Centroid;
   double radius;       /* of bounding sphere centered on Centroid */
   double UnshadedArea; /* Variable, accounting for shadowing */
   vec3 UnshadedCtr;    /* Variable, accounting for shadowing */
};

struct EdgeType {
   long Vtx1;  /* Tail */
   long Vtx2;  /* Head */
   long Poly1; /* Left */
   long Poly2; /* Right */
   double Length;
   mat3x3 Dyad; /* For polyhedron gravity */
};

struct BoundingBoxType {
   vec3 max;
   vec3 min;
   vec3 center;
   double radius;
};

struct OctreeCellType {
   long IsEmpty; /* True if cell and all its children are empty */
   vec3 center;
   double radius;
   vec3 min, max;
   long Npoly;
   long *Poly; /* Polys completely within cell, but not contained in any of its
                  children */
   long Child[8];
   long NextOnMiss;
   long NextOnHit;
};

/* Only add an Octree to Geom struct if it'll be used */
struct OctreeType {
   long Noct;    /* Number of occupied cells in Octree */
   long *OctIdx; /* List of indices pointing into occupied cells */
   /* Octree is four layers deep.  585 = 1 + 8 + 64 + 512 */
   struct OctreeCellType OctCell[585];
};

struct KDNodeType {
   long IsRoot;
   long IsLeaf;
   long Depth; /* in KDTree */
   long Axis;  /* 0, 1, or 2 */
   long Npoly;
   long *Poly;
   struct BoundingBoxType BB;
   struct KDNodeType *Parent;
   struct KDNodeType *LowChild;
   struct KDNodeType *HighChild;
};

struct GeomType {
   char ObjFileName[40];
   long Nmatl;
   long Nv;
   long Nvt;
   long Nvn;
   long Npoly;
   long Nedge;
   struct BoundingBoxType BBox;
   vec3 *V;
   vec3 *Vt;
   vec3 *Vn;
   struct PolyType *Poly;
   struct EdgeType *Edge;
   long *Matl;
   int DepthListTag;
   int OpaqueListTag;
   int OpaqueAlphaListTag;
   int SeeThruListTag;
   struct OctreeType *Octree;
   struct KDNodeType *KDTree;
};

/* Material Definitions used both in graphical output and for         */
/* computation of solar pressure forces.                              */
/* Note that Kd+Ks < 1.0 (K=R,G,B)                                    */
struct MatlType {
   char Label[40];
   float Ka[4]; /* Ambient Color */
   float Kd[4]; /* Diffuse Color */
   float Ks[4]; /* Specular Color */
   float Ke[4]; /* Emissive Color */
   float Ns;    /* Shininess */
   float Nu;    /* Shininess in U direction, for BRDF */
   float Nv;    /* Shininess in V direction, for BRDF */
   float Refl;  /* 0.0 = No reflection, 1.0 = Mirror */
   char ColorTexFileName[40];
   char BumpTexFileName[40];
   unsigned int ColorTex;
   unsigned int BumpTex;
   char SpectrumName[40];
   unsigned int SpectrumTex;
   long NoiseColEnabled;
   long NoiseBumpEnabled;
   float NoiseGain[4];
   float NoiseBias;
   float NoiseScale;
   float NoiseAxis[3];
   int NoiseType;
   /* For radiation pressure force computation */
   double SpecFrac;
   double DiffFrac;
};

struct MatlType *AddMtlLib(const char *PathName, const char *MtlLibName,
                           struct MatlType *OldMatl, long *Nmatl);
void ScaleSpecDiffFrac(struct MatlType *Matl, long Nmatl);
void SurfaceForceProps(struct GeomType *G);
void LoadKDTree(struct GeomType *G);
long KDProjectRayOntoGeom(vec3 Source, vec3 DirVec, struct GeomType *G,
                          long *HitPoly, vec3 HitPoint);
void LoadOctree(struct GeomType *G);
long OCProjectRayOntoGeom(vec3 Point, vec3 DirVec, struct GeomType *G,
                          vec3 *ProjPoint, long *ClosestPoly);
struct GeomType *LoadWingsObjFile(const char *ModelPath,
                                  const char *ObjFilename,
                                  struct MatlType **MatlPtr, long *Nmatl,
                                  struct GeomType *Geom, long *Ngeom,
                                  long *GeomTag, long EdgesEnabled);
void WriteGeomToObjFile(struct MatlType *Matl, struct GeomType *Geom,
                        const char *Path, const char *FileName);
__attribute__((pure)) double PolyhedronVolume(struct GeomType *G);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __GEOMKIT_H__ */

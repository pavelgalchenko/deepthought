/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#ifndef __MATHKIT_H__
#define __MATHKIT_H__

/*
** #ifdef __cplusplus
** namespace Kit {
** #endif
*/

typedef union vec3 {
   struct {
      double x;
      double y;
      double z;
   };
   double v[3];
} vec3_t;

typedef struct magvec3 {
   double m; // magnitude
   vec3_t v; // unit_vec
} magvec3_t;

typedef struct pair_vec3 {
   vec3_t first;
   vec3_t second;
} pair_vec3_t;

#define VEC3_ZERO          ((vec3_t){.x = 0, .y = 0, .z = 0})
#define VEC3_PXAXIS        ((vec3_t){.x = 1, .y = 0, .z = 0})
#define VEC3_PYAXIS        ((vec3_t){.x = 0, .y = 1, .z = 0})
#define VEC3_PZAXIS        ((vec3_t){.x = 0, .y = 0, .z = 1})
#define VEC3_NXAXIS        ((vec3_t){.x = -1, .y = 0, .z = 0})
#define VEC3_NYAXIS        ((vec3_t){.x = 0, .y = -1, .z = 0})
#define VEC3_NZAXIS        ((vec3_t){.x = 0, .y = 0, .z = -1})
#define VEC3_INIT(a, b, c) ((vec3_t){.x = (a), .y = (b), .z = (c)})
#define DBL_TO_VEC3(dbl) ((vec3_t){.x = (dbl)[0], .y = (dbl)[1], .z = (dbl)[2]})
#define VEC3_TO_DBL(dbl, vec)                                                  \
   do {                                                                        \
      (dbl)[0] = vec.x;                                                        \
      (dbl)[1] = vec.y;                                                        \
      (dbl)[2] = vec.z;                                                        \
   } while (0)

// Row-Major 3x3 Matrix
typedef union mat3x3 {
   vec3_t rows[3];
   double flat[9];
   double mat[3][3];
} mat3x3_t;

#define MAT3X3_ZERO                                                            \
   ((mat3x3_t){.rows = {(vec3_t){.x = 0, .y = 0, .z = 0},                      \
                        (vec3_t){.x = 0, .y = 0, .z = 0},                      \
                        (vec3_t){.x = 0, .y = 0, .z = 0}}})
#define MAT3X3_EYE ((mat3x3_t){.rows = {VEC3_PXAXIS, VEC3_PYAXIS, VEC3_PZAXIS}})

typedef struct pair_mat3x3 {
   mat3x3_t first;
   mat3x3_t second;
} pair_mat3x3_t;

typedef struct dbl_mat3x3 {
   double dbl;
   mat3x3_t mat;
} dbl_mat3x3_t;

// Vector first Quaternion
typedef union quat {
   struct {
      double x;
      double y;
      double z;
      double s;
   };
   struct {
      vec3_t qv;
      double qs;
   };
   double q[4];
} quat_t;
typedef quat_t vec4_t;

#define QUAT_ZERO ((quat_t){.qv = {.x = 0, .y = 0, .z = 0}, .qs = 0})
#define QUAT_EYE  ((quat_t){.qv = {.x = 0, .y = 0, .z = 0}, .qs = 1})
#define DBL_TO_QUAT(dbl)                                                       \
   ((quat_t){.qv = {.x = (dbl)[0], .y = (dbl)[1], .z = (dbl)[2]},              \
             .qs = (dbl)[3]})
#define QUAT_TO_DBL(dbl, qua)                                                  \
   do {                                                                        \
      (dbl)[0] = qua.x;                                                        \
      (dbl)[1] = qua.y;                                                        \
      (dbl)[2] = qua.z;                                                        \
      (dbl)[3] = qua.s;                                                        \
   } while (0)

// Spherical representation of vec3_t
// instead of having 'theta' and 'phi' angles, instead has the cos and sin
// values for these angles
typedef struct sphere_coord {
   double r, cth, sth, cph, sph;
} sphere_coord_t;

__attribute__((pure)) int any_int(const long n, const int *const vec);
__attribute__((pure)) int all_int(const long n, const int *const vec);
__attribute__((pure)) int any_isnan(const long n, const double *const v);
__attribute__((const)) double signum(const double x);
__attribute__((const)) double sin_deg(double x);
__attribute__((const)) double cos_deg(double x);
__attribute__((const)) double sinc(const double x);
__attribute__((const)) double smootherstep(const double x);
__attribute__((const)) double Limit(double x, double min, double max);
__attribute__((const)) mat3x3_t MxM(const mat3x3_t A, const mat3x3_t B);
__attribute__((const)) mat3x3_t MxMT(const mat3x3_t A, const mat3x3_t B);
__attribute__((const)) mat3x3_t MTxM(const mat3x3_t A, const mat3x3_t B);
__attribute__((const)) mat3x3_t MTxMT(const mat3x3_t A, const mat3x3_t B);
__attribute__((const)) vec3_t VxM(const vec3_t V, const mat3x3_t M);
__attribute__((const)) vec3_t MTxV(const mat3x3_t M, const vec3_t V);
__attribute__((const)) vec3_t MxV(const mat3x3_t M, const vec3_t V);
__attribute__((const)) vec3_t VxMT(const vec3_t V, const mat3x3_t M);
__attribute__((const)) vec3_t SxV(const double S, const vec3_t V);

__attribute__((const)) vec3_t NegV_Elem(const vec3_t A);
__attribute__((const)) vec3_t VAddV_Elem(const vec3_t A, const vec3_t B);
__attribute__((const)) vec3_t VSubV_Elem(const vec3_t A, const vec3_t B);
__attribute__((const)) vec3_t VMulV_Elem(const vec3_t A, const vec3_t B);
__attribute__((const)) vec3_t VDivV_Elem(const vec3_t A, const vec3_t B);
__attribute__((const)) vec3_t LimitElem_bidir(vec3_t x, const vec3_t lim);

__attribute__((const)) mat3x3_t SxM(const double S, const mat3x3_t A);
__attribute__((const)) double det3x3(const mat3x3_t M);
void MINV4(const double A[4][4], double B[4][4]);
__attribute__((const)) mat3x3_t MINV3(const mat3x3_t A);
void MINV2(const double A[2][2], double B[2][2]);
void PINV4x3(const double A[4][3], double Aplus[3][4]);
__attribute__((const)) mat3x3_t MT(const mat3x3_t A);
__attribute__((const)) double VoV(const vec3_t A, const vec3_t B);
__attribute__((const)) vec3_t VxV(const vec3_t A, const vec3_t B);
__attribute__((const)) vec3_t vxMov(const vec3_t w, const mat3x3_t M);
__attribute__((const)) double MAGV(const vec3_t V);
__attribute__((const)) magvec3_t UNITV(vec3_t V);
double CopyUnitV(const vec3_t V, vec3_t *W) __attribute__((deprecated));
__attribute__((const)) mat3x3_t V2CrossM(const vec3_t V);
__attribute__((const)) mat3x3_t V2DoubleCrossM(const vec3_t V);
__attribute__((const)) mat3x3_t VcrossM(const vec3_t V, const mat3x3_t M);
__attribute__((const)) mat3x3_t VcrossMT(const vec3_t V, const mat3x3_t M);
__attribute__((const)) quat_t QxQ(const quat_t A, const quat_t B);
__attribute__((const)) quat_t QTxQ(const quat_t A, const quat_t B);
__attribute__((const)) quat_t QxQT(const quat_t A, const quat_t B);
__attribute__((const)) vec3_t VxQ(const vec3_t Va, const quat_t QAB);
__attribute__((const)) vec3_t QxV(const quat_t QAB, const vec3_t Vb);
__attribute__((const)) vec3_t QTxV(const quat_t QAB, const vec3_t Va);
__attribute__((const)) quat_t UNITQ(quat_t Q);
__attribute__((const)) quat_t RECTIFYQ(quat_t Q);
__attribute__((const)) pair_vec3_t PerpBasis(const vec3_t A);
__attribute__((const)) double fact(long const n);
__attribute__((const)) double oddfact(long const n);
__attribute__((const)) double factDfact(long const n, long const m);
void Legendre(const long N, const long M, const double x,
              double P[N + 1][M + 1], double sdP[N + 1][M + 1]);
__attribute__((pure)) vec3_t SphericalHarmonics(const long N, const long M,
                                                const sphere_coord_t coord,
                                                const double Re, const double K,
                                                double **C, double **S,
                                                double **Norm);
void MxMG(double **A, double **B, double **C, const long N, const long K,
          const long M);
void MxMTG(double **A, double **B, double **C, const long N, const long K,
           const long M);
void MTxMG(double **A, double **B, double **C, const long N, const long K,
           const long M);
void CopyVG(double *const dest, const double *const src, const long n);
void SxVG(const double S, const double *V, double *W, const long n);
void axpy(const double a, const double *const x, double *const y, const long n);
void MxVG(double **M, double *v, double *w, const long n, const long m);
void SxMG(double s, double **A, double **B, const long N, const long M);
void MINVG(double **A, double **AI, const long N);
void FastMINV6(const double A[6][6], double AI[6][6], const long N);
void PINVG(double **A, double **Ai, const long n, const long m);
__attribute__((malloc)) double **CreateMatrix(const long n, const long m);
void DestroyMatrix(double **A);
void LINSOLVE(double **A, double *x, double *b, const long n);
void CholeskySolve(double **A, double *x, double *b, const long n);
void ConjGradSolve(double **A, double *x, double *b, const long n,
                   const double errtol, const long maxiter);
void Bairstow(long n, double *a, const double Tol, double *Real, double *Imag);
double Amoeba(const long N, double *P,
              double CostFunction(double *p, double *Parm), double *CostParm,
              const double scale, const double Tol);
__attribute__((const)) vec3_t FindNormal(const vec3_t V1, const vec3_t V2,
                                         const vec3_t V3);
__attribute__((pure)) double LinInterp(const double *X, const double *Y,
                                       const double x, const long n);
__attribute__((const)) quat_t SphereInterp(quat_t q1, quat_t q2,
                                           const double u);
__attribute__((const)) double CubicInterp1D(double f0, double f1, double x);
__attribute__((const)) double CubicInterp2D(double f00, double f10, double f01,
                                            double f11, double x, double y);
__attribute__((const)) double CubicInterp3D(double f000, double f100,
                                            double f010, double f110,
                                            double f001, double f101,
                                            double f011, double f111, double x,
                                            double y, double z);
double DistanceToLine(vec3_t LineEnd1, vec3_t LineEnd2, vec3_t Point,
                      vec3_t *VecToLine);
long ProjectPointOntoPoly(vec3_t Point, vec3_t DirVec, vec3_t *Vtx, long Nvtx,
                          vec3_t *ProjPoint, double *Distance);
long ProjectPointOntoTriangle(vec3_t A, vec3_t B, vec3_t C, vec3_t DirVec,
                              vec3_t Pt, vec3_t *ProjPt, vec4_t *Bary);
__attribute__((pure)) double CubicSpline(double x, double X[4], double Y[4]);
void ChebyPolys(double u, long n, double T[20], double U[20]);
void ChebyInterp(double T[20], double U[20], double Coef[20], long n, double *P,
                 double *dPdu);
void FindChebyCoefs(double *u, double *P, long Nu, long Nc, double Coef[20]);
void VecToLngLat(vec3_t A, double *lng, double *lat);
__attribute__((const)) double WrapTo2Pi(double OrbVar);
__attribute__((pure)) double BrentsMethod(double a, double b, const double tol,
                                          double (*f)(const double, double *),
                                          double *params);
double NewtonRaphson(double x0, double tol, long nMax, double maxStep,
                     long breakOnZero,
                     double (*fdf)(const double, double *const),
                     double *const params);

__attribute__((const)) sphere_coord_t getTrigSphericalCoords(const vec3_t pbe);
__attribute__((const)) mat3x3_t Adjoint(const mat3x3_t C, const mat3x3_t A);
__attribute__((const)) mat3x3_t AdjointT(const mat3x3_t C, const mat3x3_t A);
void MINVxM3(mat3x3_t A, long m, double B[3][m], double C[3][m]);
void MINVxMG(double **A, double **B, double **C, long N, long m);
void MxMINVG(double **A, double **B, double **C, long N, long m);
__attribute__((const)) mat3x3_t expmso3(vec3_t const theta);
__attribute__((const)) vec3_t logso3(mat3x3_t const R);
void expmTFG(vec3_t const theta, long const n, long const m, vec3_t x[n],
             vec3_t xbar[m], mat3x3_t *R);
__attribute__((pure)) double M1NormG(double **A, long const n, long const m);
__attribute__((pure)) double M2Norm2G(double **A, long const n, long const m);
int cholDowndate(double **S, double u[], long const n);
void chol(double **A, double **S, long const n);
void hqrd(double **A, double **U, double **R, long const n, long const m);
void bhqrd(double **A, double **U, double **R, long const n, long const m,
           long const bSize);

__attribute__((const)) double ipow(double base, long exp);
void expm(double **A, double **e, long const n);
__attribute__((pure)) long isSignificant(int const m, int const n, double **A,
                                         double **B);
void jacobiEValue(double **A, int const n, int const maxIter, double d[n]);
void jacobiEValueEVector(double **A, int const n, int const maxIter, double **V,
                         double d[n]);
__attribute__((malloc)) double **matPow(const long n, double **A,
                                        const unsigned long p);
void QuickMatPow(const long n, double **A, double **As, const long s,
                 double **Ap, const long p);

/*
** #ifdef __cplusplus
** }
** #endif
*/

#endif /* __MATHKIT_H__ */

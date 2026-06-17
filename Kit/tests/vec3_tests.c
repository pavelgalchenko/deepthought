/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */

#include "mathkit.h"
#include <criterion/criterion.h>
#include <criterion/new/assert.h>
#include <criterion/parameterized.h>
#include <criterion/theories.h>
#include <stdio.h>

#define ULP_THRESH (4) // acceptable Units in Last Place variation
#define DBL_THRESH (ULP_THRESH * __DBL_EPSILON__)

#define VEC3STRLEN (128)
static inline void vec32str(vec3_t a, char str[VEC3STRLEN])
{
   sprintf(str, "[%+.16le, %+.16le, %+.16le]", a.x, a.y, a.z);
}

#define vec3_ieee_ulp_eq(a, b, thres)                                          \
   all(ieee_ulp_eq(dbl, (a).x, (b).x, thres),                                  \
       ieee_ulp_eq(dbl, (a).y, (b).y, thres),                                  \
       ieee_ulp_eq(dbl, (a).z, (b).z, thres))

#define vec3_epsilon_eq(a, b, thres)                                           \
   all(epsilon_eq(dbl, (a).x, (b).x, thres),                                   \
       epsilon_eq(dbl, (a).y, (b).y, thres),                                   \
       epsilon_eq(dbl, (a).z, (b).z, thres))

#define VEC3_DATAPOINTS                                                        \
   DataPoints(vec3_t *, &VEC3_ZERO, &VEC3_PXAXIS, &VEC3_PYAXIS, &VEC3_PZAXIS,  \
              &VEC3_NXAXIS, &VEC3_NYAXIS, &VEC3_NZAXIS, &VEC3_ONES,            \
              &VEC3_INIT(1234567.89, 0, 0), &VEC3_INIT(0, 1234567.89, 0),      \
              &VEC3_INIT(0, 0, 1234567.89), &VEC3_INIT(-1234567.89, 0, 0),     \
              &VEC3_INIT(0, -1234567.89, 0), &VEC3_INIT(0, 0, -1234567.89),    \
              &VEC3_INIT(0, 1234567.89, 1234567.89),                           \
              &VEC3_INIT(1234567.89, 0, 1234567.89),                           \
              &VEC3_INIT(1234567.89, 1234567.89, 0),                           \
              &VEC3_INIT(0, 1234567.89, -1234567.89),                          \
              &VEC3_INIT(1234567.89, 0, -1234567.89),                          \
              &VEC3_INIT(1234567.89, -1234567.89, 0),                          \
              &VEC3_INIT(0, -1234567.89, 1234567.89),                          \
              &VEC3_INIT(-1234567.89, 0, 1234567.89),                          \
              &VEC3_INIT(-1234567.89, 1234567.89, 0),                          \
              &VEC3_INIT(0, -1234567.89, -1234567.89),                         \
              &VEC3_INIT(-1234567.89, 0, -1234567.89),                         \
              &VEC3_INIT(-1234567.89, -1234567.89, 0),                         \
              &VEC3_INIT(1234567.89, 1234567.89, 1234567.89),                  \
              &VEC3_INIT(1234567.89, 1234567.89, -1234567.89),                 \
              &VEC3_INIT(1234567.89, -1234567.89, 1234567.89),                 \
              &VEC3_INIT(-1234567.89, 1234567.89, 1234567.89),                 \
              &VEC3_INIT(1234567.89, -1234567.89, -1234567.89),                \
              &VEC3_INIT(-1234567.89, 1234567.89, -1234567.89),                \
              &VEC3_INIT(-1234567.89, -1234567.89, 1234567.89),                \
              &VEC3_INIT(-1234567.89, -1234567.89, -1234567.89))

/**********************************************************************/
#define SUITE_NAME mathkit_vec3
// Testing stringify
TheoryDataPoints(SUITE_NAME, tostr) = {VEC3_DATAPOINTS};
Theory((vec3_t * ap), SUITE_NAME, tostr)
{
   const vec3_t a        = *ap;
   char astr[VEC3STRLEN] = {'\0'};
   vec32str(a, astr);
   vec3_t b;
   sscanf(astr, "[%lf, %lf, %lf]", &b.x, &b.y, &b.z);
   cr_expect(vec3_ieee_ulp_eq(a, b, ULP_THRESH),
             "3-Vector stringify did not preseve the vector with "
             "param:\n\ta=%s",
             astr);
}

// only conditional currently is equivalence
TheoryDataPoints(SUITE_NAME, conditional) = {VEC3_DATAPOINTS, VEC3_DATAPOINTS};
Theory((vec3_t * ap, vec3_t *bp), SUITE_NAME, conditional)
{
   const vec3_t a = *ap, b = *bp;
   char astr[VEC3STRLEN] = {'\0'}, bstr[VEC3STRLEN] = {'\0'};
   vec32str(a, astr);
   vec32str(b, bstr);

   if (a.x == b.x && a.y == b.y && a.z == b.z) {
      cr_assert(_isequal_vec3(a, b),
                "3-Vectors 'a' and 'b' were not equal when they should be "
                "equal with params:\n\ta = %s\n\tb = %s",
                astr, bstr);
   }
   else {
      cr_assert(not(_isequal_vec3(a, b)),
                "3-Vectors 'a' and 'b' were equal when they should not be "
                "equal with params:\n\ta = %s\n\tb = %s",
                astr, bstr);
   }
}
#undef SUITE_NAME
/**********************************************************************/

/**********************************************************************/
// .. START UNARY FUNCTION TESTS
/**********************************************************************/
#define SUITE_NAME mathkit_vec3_unary
// Negation
TheoryDataPoints(SUITE_NAME, negation) = {VEC3_DATAPOINTS};
Theory((vec3_t * vp), SUITE_NAME, negation)
{
   const vec3_t v        = *vp;
   char vstr[VEC3STRLEN] = {'\0'}, otherstr[VEC3STRLEN] = {'\0'},
        other2str[VEC3STRLEN] = {'\0'};
   vec32str(v, vstr);

   // NegV_Elem is its own inverse
   cr_expect(vec3_ieee_ulp_eq(v, NegV_Elem(NegV_Elem(v)), ULP_THRESH),
             "NegV_Elem is not its own inverse with param:\n\ta = %s", vstr);

   // adding the negative results in the additive identity
   cr_expect(
       vec3_epsilon_eq(VAddV_Elem(v, NegV_Elem(v)), VEC3_ZERO, DBL_THRESH),
       "Zero 3-Vector is not additive identity with param:\n\ta = %s", vstr);
}

// Zero is additative identity
TheoryDataPoints(SUITE_NAME, addsubzero) = {VEC3_DATAPOINTS};
Theory((vec3_t * vp), SUITE_NAME, addsubzero)
{
   const vec3_t v        = *vp;
   char vstr[VEC3STRLEN] = {'\0'}, otherstr[VEC3STRLEN] = {'\0'},
        other2str[VEC3STRLEN] = {'\0'};
   vec32str(v, vstr);

   // zero is additive identity
   cr_expect(vec3_ieee_ulp_eq(VAddV_Elem(v, VEC3_ZERO), v, ULP_THRESH),
             "Zero 3-Vector is not additive identity with param:\n\ta = %s",
             vstr);
   // zero is subtractive identity
   cr_expect(vec3_ieee_ulp_eq(VSubV_Elem(v, VEC3_ZERO), v, ULP_THRESH),
             "Zero 3-Vector is not subtractive identity with param:\n\ta = %s",
             vstr);

   // subtracting self results in the additive identity
   cr_expect(vec3_epsilon_eq(VSubV_Elem(v, v), VEC3_ZERO, DBL_THRESH),
             "3-Vector self subtraction does not result in the additive "
             "identity with param:\n\ta = %s",
             vstr);
}

// Unary tests for multiplication/division
TheoryDataPoints(SUITE_NAME, unmuldiv) = {VEC3_DATAPOINTS};
Theory((vec3_t * vp), SUITE_NAME, unmuldiv)
{
   const vec3_t v        = *vp;
   char vstr[VEC3STRLEN] = {'\0'}, otherstr[VEC3STRLEN] = {'\0'},
        other2str[VEC3STRLEN] = {'\0'};
   vec32str(v, vstr);

   // zero maps multiplication to itself
   cr_expect(vec3_ieee_ulp_eq(VMulV_Elem(v, VEC3_ZERO), VEC3_ZERO, ULP_THRESH),
             "Zero 3-Vector does not map multiplication to itself with "
             "param:\n\ta = %s",
             vstr);
   // ones is multiplicative identity
   cr_expect(
       vec3_ieee_ulp_eq(VMulV_Elem(v, VEC3_ONES), v, ULP_THRESH),
       "Ones 3-Vector is not multiplicative identity with param:\n\ta = %s",
       vstr);
   // ones is division identity
   cr_expect(vec3_ieee_ulp_eq(VDivV_Elem(v, VEC3_ONES), v, ULP_THRESH),
             "Zero 3-Vector is not subtractive identity with param:\n\ta = %s",
             vstr);

   cr_assume(v.x != 0 && v.y != 0 && v.z != 0);
   // dividing by self self results in the multiplicative identity
   cr_expect(vec3_ieee_ulp_eq(VDivV_Elem(v, v), VEC3_ONES, ULP_THRESH),
             "3-Vector self division does not result in the multiplicative "
             "identity with param:\n\ta = %s",
             vstr);
}

// Magnitude checks
TheoryDataPoints(SUITE_NAME, magnitude) = {VEC3_DATAPOINTS};
Theory((vec3_t * vp), SUITE_NAME, magnitude)
{
   const vec3_t v        = *vp;
   char vstr[VEC3STRLEN] = {'\0'}, otherstr[VEC3STRLEN] = {'\0'},
        other2str[VEC3STRLEN] = {'\0'};
   vec32str(v, vstr);

   // check magnitude calculation
   double magv = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
   cr_assert(ieee_ulp_eq(dbl, MAGV(v), magv, ULP_THRESH),
             "MAGV function is incorrect with params:\n\tv         = "
             "%s\n\tMAGV(v)   = %le\n\trms v = %le",
             vstr, MAGV(v), magv);

   // ensure MAGV is square root of self dot product
   magv = sqrt(VoV(v, v));
   cr_assert(ieee_ulp_eq(dbl, MAGV(v), magv, ULP_THRESH),
             "MAGV function is incorrect with params:\n\tv         = "
             "%s\n\tMAGV(v)   = %le\n\trms v = %le",
             vstr, MAGV(v), magv);
}

// Product with Zero checks
TheoryDataPoints(SUITE_NAME, zeroproduct) = {VEC3_DATAPOINTS};
Theory((vec3_t * vp), SUITE_NAME, zeroproduct)
{
   const vec3_t v        = *vp;
   char vstr[VEC3STRLEN] = {'\0'}, otherstr[VEC3STRLEN] = {'\0'},
        other2str[VEC3STRLEN] = {'\0'};
   vec32str(v, vstr);
   // check dot product with zero
   double prod = VoV(v, VEC3_ZERO);
   cr_expect(epsilon_eq(dbl, prod, 0.0, DBL_THRESH),
             "Dot product with zero is not zero with params:\n\tv   = "
             "%s\n\tvoz = %le");

   // check cross product with zero
   vec3_t other = VxV(v, VEC3_ZERO);
   vec32str(other, otherstr);
   cr_expect(vec3_epsilon_eq(other, VEC3_ZERO, DBL_THRESH),
             "Cross product with zero is not zero with params:\n\tv   = "
             "%s\n\tvxz = %s",
             v, other);

   // check elementwise product with zero
   other = VMulV_Elem(v, VEC3_ZERO);
   vec32str(other, otherstr);
   cr_expect(vec3_epsilon_eq(other, VEC3_ZERO, DBL_THRESH),
             "ELement-wise product with zero is not zero with params:\n\tv   = "
             "%s\n\tvxz = %s",
             v, other);
}

// Self dot and cross product
TheoryDataPoints(SUITE_NAME, selfdotxprod) = {VEC3_DATAPOINTS};
Theory((vec3_t * vp), SUITE_NAME, selfdotxprod)
{
   const vec3_t v        = *vp;
   char vstr[VEC3STRLEN] = {'\0'}, otherstr[VEC3STRLEN] = {'\0'},
        other2str[VEC3STRLEN] = {'\0'};
   vec32str(v, vstr);

   // check self dot product
   double magv = MAGV(v);
   double prod = VoV(v, v);
   cr_expect(ieee_ulp_eq(dbl, prod, (magv * magv), ULP_THRESH),
             "Dot product with self is not equal to square of magnitude for "
             "params:\n\tv       = %s\n\tMAGV(v) = %le\n\tVoV(v)  = %le",
             vstr, magv, prod);

   // check self cross product
   vec3_t other = VxV(v, v);
   vec32str(other, otherstr);
   cr_expect(vec3_epsilon_eq(other, VEC3_ZERO, DBL_THRESH),
             "Cross product with self is not zero with params:\n\tv   = "
             "%s\n\tvxz = %s",
             v, other);
   mat3x3_t vx = V2CrossM(v);
   other       = MxV(vx, v);
   vec32str(other, otherstr);
   cr_expect(vec3_epsilon_eq(other, VEC3_ZERO, DBL_THRESH),
             "product between cross product matrix and self is not zero with "
             "params:\n\tv   = %s\n\tvxz = %s",
             v, other);
}

// Self outer product
TheoryDataPoints(SUITE_NAME, selfouterprod) = {VEC3_DATAPOINTS};
Theory((vec3_t * vp), SUITE_NAME, selfouterprod)
{
   const vec3_t v        = *vp;
   char vstr[VEC3STRLEN] = {'\0'}, otherstr[VEC3STRLEN] = {'\0'},
        other2str[VEC3STRLEN] = {'\0'};
   vec32str(v, vstr);

   // trace of self outer product is magnitude squared
   mat3x3_t outer = VOuterV(v, v);
   double trace   = MTrace(outer);
   double magv    = MAGV(v);
   cr_expect(ieee_ulp_eq(dbl, trace, magv * magv, ULP_THRESH),
             "Trace of self outer product is not square of magnitude with "
             "params:\n\tv = %s\n\ttrace = %le\n\tmagv  = %le",
             vstr, trace, magv);
   vec3_t other  = MxV(outer, v);
   vec3_t other2 = SxV(magv * magv, v);
   vec32str(other, otherstr);
   vec32str(other2, other2str);
   cr_expect(vec3_ieee_ulp_eq(other, other2, ULP_THRESH),
             "Product with unitized outer product did not result in self "
             "multiplied with magnitude squared with params:\n\t v        = "
             "%s\n\t (vvT)v   = %s\n\t magv     =%le\n\t v*magv^2 = %s",
             vstr, otherstr, magv, other2str);
}

// Perp basis
TheoryDataPoints(SUITE_NAME, perpbasis) = {VEC3_DATAPOINTS};
Theory((vec3_t * vp), SUITE_NAME, perpbasis)
{
   const vec3_t v = *vp;
   vec3_t other;
   char vstr[VEC3STRLEN] = {'\0'}, otherstr[VEC3STRLEN] = {'\0'},
        other2str[VEC3STRLEN] = {'\0'};
   vec32str(v, vstr);

   cr_assume(!(_isequal_vec3(v, VEC3_ZERO)));
   // check perp basis creator
   pair_vec3_t vpair = PerpBasis(v);
   vec32str(vpair.first, otherstr);
   vec32str(vpair.second, other2str);
   other.x = VoV(vpair.first, v);
   other.y = VoV(vpair.second, v);
   other.z = VoV(vpair.first, vpair.second);
   cr_expect(vec3_epsilon_eq(other, VEC3_ZERO, DBL_THRESH),
             "Generated PerpBasis is not perpendicular with params:\n\tv       "
             "     = %s\n\tfirst        = %s\n\tsecond       = %s\n\tfirst*v   "
             "   = %le\n\tsecond*v     = %le\n\tfirst*second = %le",
             vstr, otherstr, other2str, other.x, other.y, other.z);
}

// Unitization
TheoryDataPoints(SUITE_NAME, unitization) = {VEC3_DATAPOINTS};
Theory((vec3_t * vp), SUITE_NAME, unitization)
{
   const vec3_t v = *vp;
   vec3_t other;
   char vstr[VEC3STRLEN] = {'\0'}, otherstr[VEC3STRLEN] = {'\0'},
        other2str[VEC3STRLEN] = {'\0'};
   vec32str(v, vstr);

   cr_assume(!(_isequal_vec3(v, VEC3_ZERO)));
   // check unitzation
   magvec3_t vm = UNITV(v);
   other        = SxV(vm.m, vm.v);
   vec32str(other, otherstr);
   cr_expect(vec3_ieee_ulp_eq(v, other, ULP_THRESH),
             "Unit vector multiplied with magnitude did not yield the original "
             "vector with params:\n\tv    = %s\n\tvhat = %s\n\tmagv = %le",
             vstr, otherstr, vm.m);

   double prod = VoV(vm.v, v);
   vec32str(vm.v, otherstr);
   cr_expect(ieee_ulp_eq(dbl, prod, vm.m, ULP_THRESH),
             "Unit vector is not parallel to original with params:\n\tv    = "
             "%s\n\tvhat = %s\n\tdot  = %le",
             vstr, otherstr, prod);
}

// Longitude Latitude computation
TheoryDataPoints(SUITE_NAME, lnglat) = {VEC3_DATAPOINTS};
Theory((vec3_t * vp), SUITE_NAME, lnglat)
{
   const vec3_t v = *vp;
   vec3_t other;
   char vstr[VEC3STRLEN] = {'\0'}, otherstr[VEC3STRLEN] = {'\0'},
        other2str[VEC3STRLEN] = {'\0'};
   vec32str(v, vstr);

   cr_assume(!(_isequal_vec3(v, VEC3_ZERO)));
   // check lat/long calculation
   magvec3_t vm = UNITV(v);
   double lat, lng;
   VecToLngLat(v, &lng, &lat);
   other = VEC3_INIT(cos(lat) * cos(lng), cos(lat) * sin(lng), sin(lat));
   vec32str(other, otherstr);
   vec32str(vm.v, other2str);
   cr_expect(vec3_epsilon_eq(VSubV_Elem(vm.v, other), VEC3_ZERO, DBL_THRESH),
             "Longitude/Lattitude calculations did not return original unit "
             "vector with params:\n\tv     = %s\n\tvhat  = %s\n\tlng   = "
             "%le\n\tlat   = %le\n\tother = %s",
             vstr, other2str, lng, lat, otherstr);
}
#undef SUITE_NAME
/**********************************************************************/
// .. END UNARY FUNCTION TESTS
/**********************************************************************/

/**********************************************************************/
// .. START BINARY FUNCTION TESTS
/**********************************************************************/
#define SUITE_NAME mathkit_vec3_binary
// addition/subtraction
TheoryDataPoints(SUITE_NAME, addsub) = {VEC3_DATAPOINTS, VEC3_DATAPOINTS};
Theory((vec3_t * ap, vec3_t *bp), SUITE_NAME, addsub)
{
   const vec3_t a = *ap, b = *bp;
   char astr[VEC3STRLEN] = {'\0'}, bstr[VEC3STRLEN] = {'\0'};
   vec32str(a, astr);
   vec32str(b, bstr);

   // commutative addition
   cr_expect(
       vec3_ieee_ulp_eq(VAddV_Elem(a, b), VAddV_Elem(b, a), ULP_THRESH),
       "3-Vector addition is not commutative with params:\n\ta = %s\n\tb = %s",
       astr, bstr);

   // addition inversion
   cr_expect(vec3_ieee_ulp_eq(a, VSubV_Elem(VAddV_Elem(a, b), b), ULP_THRESH),
             "3-Vector subtraction does not invert addition with params:\n\ta "
             "= %s\n\tb = %s",
             astr, bstr);
}

// multiplication/division
TheoryDataPoints(SUITE_NAME, muldiv) = {VEC3_DATAPOINTS, VEC3_DATAPOINTS};
Theory((vec3_t * ap, vec3_t *bp), SUITE_NAME, muldiv)
{
   const vec3_t a = *ap, b = *bp;
   char astr[VEC3STRLEN] = {'\0'}, bstr[VEC3STRLEN] = {'\0'};
   vec32str(a, astr);
   vec32str(b, bstr);

   // commutative multiplication
   cr_expect(vec3_ieee_ulp_eq(VMulV_Elem(a, b), VMulV_Elem(b, a), ULP_THRESH),
             "3-Vector multiplication is not commutative with params:\n\ta = "
             "%s\n\tb = %s",
             astr, bstr);

   cr_assume(b.x != 0 && b.y != 0 && b.z != 0);
   // multiplication inversion
   cr_expect(vec3_ieee_ulp_eq(a, VDivV_Elem(VMulV_Elem(a, b), b), ULP_THRESH),
             "3-Vector division does not invert multiplication with "
             "params:\n\ta = %s\n\tb = %s",
             astr, bstr);
}

// Cauchy-Schwarz Inequality
TheoryDataPoints(SUITE_NAME, cauchyschwarz) = {VEC3_DATAPOINTS,
                                               VEC3_DATAPOINTS};
Theory((vec3_t * ap, vec3_t *bp), SUITE_NAME, cauchyschwarz)
{
   const vec3_t a = *ap, b = *bp;
   char astr[VEC3STRLEN] = {'\0'}, bstr[VEC3STRLEN] = {'\0'};
   vec32str(a, astr);
   vec32str(b, bstr);

   const double mag_a = MAGV(a);
   const double mag_b = MAGV(b);
   const double aob   = VoV(a, b);

   // TODO: any is bugged...
   double cauchyeps = nextafter(mag_a * mag_b, INFINITY) - (mag_a * mag_b);
   // Cauchy-Schwarz Inequality
   cr_expect(
       //  any(lt(dbl, fabs(aob), mag_a * mag_b),
       //      ieee_ulp_eq(dbl, fabs(aob), mag_a * mag_b, 4)),
       le(dbl, fabs(aob), (mag_a * mag_b) + cauchyeps),
       "Cauchy-Schwarz Inequality is not followed with params:\n\ta            "
       "    = %s\n\tb                = %s\n\t|VoV(a,b)|       = %le\n\tMAGV(a) "
       "         = %le\n\tMAGV(b)          = %le\n\tMAGV(a)*MAGV(b)  = "
       "%le\n\tcauchyeps        = %le",
       astr, bstr, fabs(aob), mag_a, mag_b, mag_a * mag_b, cauchyeps);
}

// Cross Product Properties
TheoryDataPoints(SUITE_NAME, crossprodsimple) = {VEC3_DATAPOINTS,
                                                 VEC3_DATAPOINTS};
Theory((vec3_t * ap, vec3_t *bp), SUITE_NAME, crossprodsimple)
{
   const vec3_t a = *ap, b = *bp;
   char astr[VEC3STRLEN] = {'\0'}, bstr[VEC3STRLEN] = {'\0'},
        other1str[VEC3STRLEN] = {'\0'}, other2str[VEC3STRLEN] = {'\0'};
   vec32str(a, astr);
   vec32str(b, bstr);

   const double mag_a2 = VoV(a, a);
   const double mag_b2 = VoV(b, b);
   const double aob    = VoV(a, b);

   // angle between the vectors
   const double cos2ab = aob * aob / (mag_a2 * mag_b2);
   const double sin2ab = 1.0 - cos2ab;

   // mag of cross product is product of mag of arguments and the sin of the
   // angle between
   // looking at square to avoid square root
   vec3_t axb = VxV(a, b);
   vec32str(axb, other1str);
   double mag2 = VoV(axb, axb);
   cr_assume(mag2 > 0);
   cr_expect(ieee_ulp_eq(dbl, mag2, mag_a2 * mag_b2 * sin2ab, ULP_THRESH),
             "Cross product has incorrect magnitude with params:\n\ta = % "
             "s\n\tb = % s\n\taxb = % s\n\tMAGV(a) ^ 2 = % le\n\tMAGV(b) ^ 2 = "
             "% le\n\tMAGV(axb) ^ 2 = % le\n\tsin ^ 2(ab) = % le ",
             astr, bstr, other1str, mag_a2, mag_b2, mag2, sin2ab);

   // check V2crossM
   mat3x3_t ax = V2CrossM(a);
   vec3_t ax_b = MxV(ax, b);
   vec32str(ax_b, other2str);
   cr_expect(vec3_ieee_ulp_eq(axb, ax_b, ULP_THRESH),
             "[ax]b is not the same as axb with params:\n\ta    = %s\n\tb    = "
             "%s\n\taxb  = %s\n\tax_b = %s",
             astr, bstr, other1str, other2str);
}

// Lagrange's Identity
TheoryDataPoints(SUITE_NAME, lagrangeid) = {VEC3_DATAPOINTS, VEC3_DATAPOINTS};
Theory((vec3_t * ap, vec3_t *bp), SUITE_NAME, lagrangeid)
{
   const vec3_t a = *ap, b = *bp;
   char astr[VEC3STRLEN] = {'\0'}, bstr[VEC3STRLEN] = {'\0'},
        other1str[VEC3STRLEN] = {'\0'};
   vec32str(a, astr);
   vec32str(b, bstr);

   const vec3_t axb  = VxV(a, b);
   const double aob  = VoV(a, b);
   const double axb2 = VoV(axb, axb);
   vec32str(axb, other1str);
   // Lagrange's Identity
   cr_expect(ieee_ulp_eq(dbl, VoV(axb, axb) + aob * aob, VoV(a, a) * VoV(b, b),
                         ULP_THRESH),
             "Lagrange's Identity is not true with params:\n\ta            "
             "= %s\n\tb            = %s\n\taxb          = "
             "%s\n\tVoV(axb,axb) = %le\n\tVoV(a,a)     = %le\n\tVoV(b,b)    "
             " = %le\n\tVoV(a,b)     = %le",
             astr, bstr, other1str, VoV(axb, axb), VoV(a, a), VoV(b, b), aob);
}
#undef SUITE_NAME
/**********************************************************************/
// .. END BINARY FUNCTION TESTS
/**********************************************************************/

/**********************************************************************/
// .. START TERNARY FUNCTION TESTS
/**********************************************************************/
#define SUITE_NAME mathkit_vec3_ternary
// scalar triple product
TheoryDataPoints(SUITE_NAME, scalartripleprod) = {
    VEC3_DATAPOINTS, VEC3_DATAPOINTS, VEC3_DATAPOINTS};
Theory((vec3_t * ap, vec3_t *bp, vec3_t *cp), SUITE_NAME, scalartripleprod)
{
   const vec3_t a = *ap, b = *bp, c = *cp;
   char astr[VEC3STRLEN] = {'\0'}, bstr[VEC3STRLEN] = {'\0'},
        cstr[VEC3STRLEN]      = {'\0'};
   char other1str[VEC3STRLEN] = {'\0'}, other2str[VEC3STRLEN] = {'\0'},
        other3str[VEC3STRLEN] = {'\0'}, other4str[VEC3STRLEN] = {'\0'},
        other5str[VEC3STRLEN] = {'\0'}, other6str[VEC3STRLEN] = {'\0'};
   vec32str(a, astr);
   vec32str(b, bstr);
   vec32str(c, cstr);

   const vec3_t bxc = VxV(b, c);
   const vec3_t cxa = VxV(c, a);
   const vec3_t axb = VxV(a, b);
   vec32str(bxc, other1str);
   vec32str(cxa, other2str);
   vec32str(axb, other3str);
   // circular shift equality
   cr_expect(all(ieee_ulp_eq(dbl, VoV(a, bxc), VoV(b, cxa), ULP_THRESH),
                 ieee_ulp_eq(dbl, VoV(a, bxc), VoV(c, axb), ULP_THRESH)),
             "Scalar triple product circular shift equality does not holt with "
             "params:\n\t a   = %s\n\t b   = %s\n\t c   = %s\n\t bxc = %s\n\t "
             "cxa = %s\n\t axb = %s",
             astr, bstr, cstr, other1str, other2str, other3str);
}

// vector triple product
TheoryDataPoints(SUITE_NAME, vectortripleprod) = {
    VEC3_DATAPOINTS, VEC3_DATAPOINTS, VEC3_DATAPOINTS};
Theory((vec3_t * ap, vec3_t *bp, vec3_t *cp), SUITE_NAME, vectortripleprod)
{
   const vec3_t a = *ap, b = *bp, c = *cp;
   char astr[VEC3STRLEN] = {'\0'}, bstr[VEC3STRLEN] = {'\0'},
        cstr[VEC3STRLEN]      = {'\0'};
   char other1str[VEC3STRLEN] = {'\0'}, other2str[VEC3STRLEN] = {'\0'},
        other3str[VEC3STRLEN] = {'\0'};
   vec32str(a, astr);
   vec32str(b, bstr);
   vec32str(c, cstr);

   const vec3_t bxc    = VxV(b, c);
   const vec3_t ax_bxc = VxV(a, bxc);
   vec32str(bxc, other1str);
   vec32str(ax_bxc, other2str);
   const double aoc = VoV(a, c), aob = VoV(a, b);

   double maxval = -INFINITY;
   vec3_t test   = ax_bxc;
   for (int i = 0; i < 3; i++)
      if (fabs(test.v[i]) > maxval)
         maxval = fabs(test.v[i]);
   test = SxV(aoc, b);
   for (int i = 0; i < 3; i++)
      if (fabs(test.v[i]) > maxval)
         maxval = fabs(test.v[i]);
   test = SxV(aob, c);
   for (int i = 0; i < 3; i++)
      if (fabs(test.v[i]) > maxval)
         maxval = fabs(test.v[i]);

   // use this to make the threshhld against the largest magnitude between the
   // values being added and subtracted
   const double threshhld = nextafter(maxval, INFINITY) - maxval;
   vec3_t rhs             = VSubV_Elem(SxV(aoc, b), SxV(aob, c));
   vec32str(rhs, other3str);
   cr_expect(
       vec3_epsilon_eq(ax_bxc, rhs, threshhld),
       "Vector triple product is not true with params:\n\ta               = "
       "%s\n\tb               = %s\n\tc               = %s\n\tVxV(b,c)        "
       "= %s\n\tVxV(a,VxV(b,c)) = %s\n\trhs             = %s\n\tVoV(a,c)       "
       " = %lf\n\tVoV(a,b)        = %lf\n\tthreshhld       = %le",
       astr, bstr, cstr, other1str, other2str, other3str, aoc, aob, threshhld);
}

// Jacobi Identity
TheoryDataPoints(SUITE_NAME, jacobiid) = {VEC3_DATAPOINTS, VEC3_DATAPOINTS,
                                          VEC3_DATAPOINTS};
Theory((vec3_t * ap, vec3_t *bp, vec3_t *cp), SUITE_NAME, jacobiid)
{
   const vec3_t a = *ap, b = *bp, c = *cp;
   char astr[VEC3STRLEN] = {'\0'}, bstr[VEC3STRLEN] = {'\0'},
        cstr[VEC3STRLEN]      = {'\0'};
   char other1str[VEC3STRLEN] = {'\0'}, other2str[VEC3STRLEN] = {'\0'},
        other3str[VEC3STRLEN] = {'\0'}, other4str[VEC3STRLEN] = {'\0'},
        other5str[VEC3STRLEN] = {'\0'}, other6str[VEC3STRLEN] = {'\0'};
   vec32str(a, astr);
   vec32str(b, bstr);
   vec32str(c, cstr);

   const vec3_t bxc = VxV(b, c);
   const vec3_t cxa = VxV(c, a);
   const vec3_t axb = VxV(a, b);
   vec32str(bxc, other1str);
   vec32str(cxa, other2str);
   vec32str(axb, other3str);

   // Jacobi Identity
   const vec3_t ax_bxc = VxV(a, bxc);
   const vec3_t axb_xc = VxV(axb, c);
   const vec3_t axc    = VxV(a, c);
   const vec3_t bx_axc = VxV(b, axc);
   vec32str(axb, other1str);
   vec32str(axb_xc, other2str);
   vec32str(bxc, other3str);
   vec32str(ax_bxc, other4str);
   vec32str(axc, other5str);
   vec32str(bx_axc, other6str);
   cr_expect(
       vec3_ieee_ulp_eq(axb_xc, VSubV_Elem(ax_bxc, bx_axc), ULP_THRESH),
       "Jacobi identity is not true with params\n\ta = %s\n\tb = %s\n\tc = "
       "%s\n\tVxV(a,b) = %s\n\tVxV(VxV(a,b),c) = %s\n\tVxV(b,c) = "
       "%s\n\tVxV(a,VxV(b,c)) = %s\n\tVxV(a,c) = %s\n\tVxV(b,VxV(a,c)) = %s",
       astr, bstr, cstr, other1str, other2str, other3str, other4str, other5str,
       other6str);
}
#undef SUITE_NAME
/**********************************************************************/
// .. END TERNARY FUNCTION TESTS
/**********************************************************************/
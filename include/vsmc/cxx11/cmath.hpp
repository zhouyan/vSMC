#ifndef VSMC_CXX11_CMATH_HPP
#define VSMC_CXX11_CMATH_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_CMATH
#include <cmath>
namespace vsmc {namespace cxx11 {
using std::exp2;
using std::log2;
using std::expm1;
using std::log1p;
using std::cbrt;
using std::hypot;
using std::asinh;
using std::acosh;
using std::atanh;
using std::erf;
using std::erfc;
using std::lgamma;
using std::tgamma;
using std::fpclassify;
using std::isfinite;
using std::isinf;
using std::isnan;
using std::isnormal;
using std::signbit;
} }
#elif VSMC_HAS_C99LIB_MATH
#include <math.h>

#define VSMC_DEFINE_C99_MATH_SPECIAL(name) \
inline float       (name) (float       x) {return ::name##f(x);}             \
inline double      (name) (double      x) {return ::name(x);}                \
inline long double (name) (long double x) {return ::name##l(x);}             \
template <typename T> inline double (name) (T x)                             \
{return ::name(static_cast<double>(x));}

#define VSMC_DEFINE_C99_MATH_FPCLASSIFY(RT, name) \
inline RT (name) (float       x) {return name(x);}                           \
inline RT (name) (double      x) {return name(x);}                           \
inline RT (name) (long double x) {return name(x);}                           \
template <typename T> inline RT (name) (T x)                                 \
{return name(static_cast<double>(x));}

namespace vsmc { namespace cxx11 {
inline float       (hypot) (float       x, float       y) {return ::hypotf(x, y);}
inline double      (hypot) (float       x, double      y) {return ::hypot (x, y);}
inline long double (hypot) (float       x, long double y) {return ::hypotl(x, y);}
inline double      (hypot) (double      x, float       y) {return ::hypot (x, y);}
inline double      (hypot) (double      x, double      y) {return ::hypot (x, y);}
inline long double (hypot) (double      x, long double y) {return ::hypotl(x, y);}
inline long double (hypot) (long double x, float       y) {return ::hypotl(x, y);}
inline long double (hypot) (long double x, double      y) {return ::hypotl(x, y);}
inline long double (hypot) (long double x, long double y) {return ::hypotl(x, y);}
template <typename T1, typename T2> inline double (hypot) (T1 x, T2 y)
{return ::hypot(static_cast<double>(x), static_cast<double>(y));}
VSMC_DEFINE_C99_MATH_SPECIAL(exp2)
VSMC_DEFINE_C99_MATH_SPECIAL(log2)
VSMC_DEFINE_C99_MATH_SPECIAL(expm1)
VSMC_DEFINE_C99_MATH_SPECIAL(log1p)
VSMC_DEFINE_C99_MATH_SPECIAL(cbrt)
VSMC_DEFINE_C99_MATH_SPECIAL(asinh)
VSMC_DEFINE_C99_MATH_SPECIAL(acosh)
VSMC_DEFINE_C99_MATH_SPECIAL(atanh)
VSMC_DEFINE_C99_MATH_SPECIAL(erf)
VSMC_DEFINE_C99_MATH_SPECIAL(erfc)
VSMC_DEFINE_C99_MATH_SPECIAL(lgamma)
VSMC_DEFINE_C99_MATH_SPECIAL(tgamma)
VSMC_DEFINE_C99_MATH_FPCLASSIFY(int, fpclassify)
VSMC_DEFINE_C99_MATH_FPCLASSIFY(bool, isfinite)
VSMC_DEFINE_C99_MATH_FPCLASSIFY(bool, isinf)
VSMC_DEFINE_C99_MATH_FPCLASSIFY(bool, isnan)
VSMC_DEFINE_C99_MATH_FPCLASSIFY(bool, isnormal)
VSMC_DEFINE_C99_MATH_FPCLASSIFY(bool, signbit)
} }
#else // VSMC_HAS_CXX11LIB_CMATH
#include <boost/math/special_functions/expm1.hpp>
#include <boost/math/special_functions/log1p.hpp>
#include <boost/math/special_functions/cbrt.hpp>
#include <boost/math/special_functions/hypot.hpp>
#include <boost/math/special_functions/asinh.hpp>
#include <boost/math/special_functions/acosh.hpp>
#include <boost/math/special_functions/atanh.hpp>
#include <boost/math/special_functions/erf.hpp>
#include <boost/math/special_functions/gamma.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/special_functions/sign.hpp>

namespace vsmc {namespace cxx11 {

inline float (exp2) (float x)
{using std::exp; using std::log; return exp(x * log(2.f));}
inline double (exp2) (double x)
{using std::exp; using std::log; return exp(x * log(2.));}
inline long double (exp2) (long double x)
{using std::exp; using std::log; return exp(x * log(2.l));}
template <typename T> inline double (exp2) (T x)
{using std::exp; using std::log; return exp(static_cast<double>(x) * log(2.));}

inline float (log2) (float x)
{using std::log; return log(x) / log(2.f);}
inline double (log2) (double x)
{using std::log; return log(x) / log(2.);}
inline long double (log2) (long double x)
{using std::log; return log(x) * log(2.l);}
template <typename T> inline double (log2) (T x)
{using std::log; return log(static_cast<double>(x)) / log(2.);}

using boost::math::expm1;
using boost::math::log1p;
using boost::math::cbrt;
using boost::math::hypot;
using boost::math::asinh;
using boost::math::acosh;
using boost::math::atanh;
using boost::math::erf;
using boost::math::erfc;
using boost::math::lgamma;
using boost::math::tgamma;
using boost::math::fpclassify;
using boost::math::isfinite;
using boost::math::isinf;
using boost::math::isnan;
using boost::math::isnormal;
using boost::math::signbit;
} }
#endif // VSMC_HAS_CXX11LIB_CMATH

#endif // VSMC_CXX11_CMATH_HPP

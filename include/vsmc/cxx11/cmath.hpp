//============================================================================
// vSMC/include/vsmc/cxx11/cmath.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_CXX11_CMATH_HPP
#define VSMC_CXX11_CMATH_HPP

#include <vsmc/internal/config.hpp>
#include <cmath>

#if VSMC_HAS_CXX11LIB_CMATH

namespace vsmc { namespace cxx11 {

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

} } // namespace vsmc::cxx11

#elif VSMC_HAS_C99LIB_MATH

#define VSMC_DEFINE_C99_MATH_SPECIAL(name) \
inline float       (name) (float       x) {return ::name##f(x);}             \
inline double      (name) (double      x) {return ::name(x);}                \
inline long double (name) (long double x) {return ::name##l(x);}             \
template <typename T> inline double (name) (T x)                             \
{return ::name(static_cast<double>(x));}

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

} } // namespace vsmc::cxx11

#else // VSMC_HAS_CXX11LIB_CMATH

#include <vsmc/math/constants.hpp>
#include <boost/math/special_functions.hpp>

namespace vsmc { namespace cxx11 {

inline float (exp2) (float x)
{using std::exp; return exp(x * math::ln_2<float>());}
inline double (exp2) (double x)
{using std::exp; return exp(x * math::ln_2<double>());}
inline long double (exp2) (long double x)
{using std::exp; return exp(x * math::ln_2<long double>());}
template <typename T> inline double (exp2) (T x)
{using std::exp; return exp(static_cast<double>(x) * math::ln_2<double>());}

inline float (log2) (float x)
{using std::log; return log(x) * math::ln_inv_2<float>();}
inline double (log2) (double x)
{using std::log; return log(x) * math::ln_inv_2<double>();}
inline long double (log2) (long double x)
{using std::log; return log(x) * math::ln_inv_2<long double>();}
template <typename T> inline double (log2) (T x)
{using std::log; return log(static_cast<double>(x)) * math::ln_inv_2<double>();}

using ::boost::math::expm1;
using ::boost::math::log1p;
using ::boost::math::cbrt;
using ::boost::math::hypot;
using ::boost::math::asinh;
using ::boost::math::acosh;
using ::boost::math::atanh;
using ::boost::math::erf;
using ::boost::math::erfc;
using ::boost::math::lgamma;
using ::boost::math::tgamma;

} } // namespace vsmc::cxx11

#endif // VSMC_HAS_CXX11LIB_CMATH

#endif // VSMC_CXX11_CMATH_HPP

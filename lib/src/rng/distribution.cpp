//============================================================================
// vSMC/lib/src/rng/distribution.cpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#include <vsmc/rng/distribution.hpp>
#include <vsmc/rng/engine.hpp>
#include <vsmc/rng/rng.h>

extern "C" {

#define VSMC_DEFINE_LIB_RNG_DIST(RNGType)                                     \
    RNGType &rng_cpp = *reinterpret_cast<RNGType *>(rng.ptr);                 \
    for (size_t i = 0; i != n; ++i)                                           \
        r[i] = dist(rng_cpp);

#include "uniform_int_rand.cpp"

#include "bernoulli_rand.cpp"

#include "binomial_rand.cpp"

#include "negative_binomial_rand.cpp"

#include "geometric_rand.cpp"

#include "poisson_rand.cpp"

#include "discrete_rand.cpp"

#include "normal_mv_rand.cpp"

#define VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_0(RNGType, Name, name, dist)   \
    inline void vsmc_##dist##_rand_##name(vsmc_rng rng, size_t n, double *r)  \
    {                                                                         \
        ::vsmc::dist##_distribution(                                          \
            *reinterpret_cast<RNGType *>(rng.ptr), n, r);                     \
    }

#define VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_1(RNGType, Name, name, dist)   \
    inline void vsmc_##dist##_rand_##name(                                    \
        vsmc_rng rng, size_t n, double *r, double p1)                         \
    {                                                                         \
        ::vsmc::dist##_distribution(                                          \
            *reinterpret_cast<RNGType *>(rng.ptr), n, r, p1);                 \
    }

#define VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(RNGType, Name, name, dist)   \
    inline void vsmc_##dist##_rand_##name(                                    \
        vsmc_rng rng, size_t n, double *r, double p1, double p2)              \
    {                                                                         \
        ::vsmc::dist##_distribution(                                          \
            *reinterpret_cast<RNGType *>(rng.ptr), n, r, p1, p2);             \
    }

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#ifdef VSMC_RNG_DEFINE_MACRO_NA
#undef VSMC_RNG_DEFINE_MACRO_NA
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_0(RNGType, Name, name, u01)        \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_0(RNGType, Name, name, u01_cc)     \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_0(RNGType, Name, name, u01_co)     \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_0(RNGType, Name, name, u01_oc)     \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_0(RNGType, Name, name, u01_oo)     \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_1(                                 \
        RNGType, Name, name, chi_squared)                                     \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_1(                                 \
        RNGType, Name, name, exponential)                                     \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_1(RNGType, Name, name, rayleigh)   \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_1(RNGType, Name, name, student_t)  \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(RNGType, Name, name, beta)       \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(RNGType, Name, name, cauchy)     \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(                                 \
        RNGType, Name, name, extreme_value)                                   \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(RNGType, Name, name, fisher_f)   \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(RNGType, Name, name, gamma)      \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(RNGType, Name, name, laplace)    \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(RNGType, Name, name, levy)       \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(RNGType, Name, name, logistic)   \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(RNGType, Name, name, lognormal)  \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(RNGType, Name, name, normal)     \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(RNGType, Name, name, pareto)     \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(                                 \
        RNGType, Name, name, uniform_real)                                    \
    VSMC_DEFINE_LIB_RNG_DIST_RAND_DISPATCH_2(RNGType, Name, name, weibull)

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

using vsmc_rng_rand_type_0 = void (*)(vsmc_rng, size_t, double *);

using vsmc_rng_rand_type_1 = void (*)(vsmc_rng, size_t, double *, double);

using vsmc_rng_rand_type_2 = void (*)(
    vsmc_rng, size_t, double *, double, double);

#ifdef VSMC_RNG_DEFINE_MACRO_NA
#undef VSMC_RNG_DEFINE_MACRO_NA
#endif

#define VSMC_RNG_DEFINE_MACRO_NA(RNGType, Name, name) nullptr,

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_u01_rand_##name,

static vsmc_rng_rand_type_0 vsmc_u01_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_u01_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_u01_cc_rand_##name,

static vsmc_rng_rand_type_0 vsmc_u01_cc_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_u01_cc_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_u01_co_rand_##name,

static vsmc_rng_rand_type_0 vsmc_u01_co_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_u01_co_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_u01_oc_rand_##name,

static vsmc_rng_rand_type_0 vsmc_u01_oc_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_u01_oc_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_u01_oo_rand_##name,

static vsmc_rng_rand_type_0 vsmc_u01_oo_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_u01_oo_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    vsmc_chi_squared_rand_##name,

static vsmc_rng_rand_type_1 vsmc_chi_squared_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_chi_squared_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    vsmc_exponential_rand_##name,

static vsmc_rng_rand_type_1 vsmc_exponential_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_exponential_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_rayleigh_rand_##name,

static vsmc_rng_rand_type_1 vsmc_rayleigh_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_rayleigh_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_student_t_rand_##name,

static vsmc_rng_rand_type_1 vsmc_student_t_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_student_t_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_beta_rand_##name,

static vsmc_rng_rand_type_2 vsmc_beta_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_beta_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_cauchy_rand_##name,

static vsmc_rng_rand_type_2 vsmc_cauchy_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_cauchy_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    vsmc_extreme_value_rand_##name,

static vsmc_rng_rand_type_2 vsmc_extreme_value_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_extreme_value_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_fisher_f_rand_##name,

static vsmc_rng_rand_type_2 vsmc_fisher_f_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_fisher_f_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_gamma_rand_##name,

static vsmc_rng_rand_type_2 vsmc_gamma_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_gamma_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_laplace_rand_##name,

static vsmc_rng_rand_type_2 vsmc_laplace_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_laplace_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_levy_rand_##name,

static vsmc_rng_rand_type_2 vsmc_levy_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_levy_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_logistic_rand_##name,

static vsmc_rng_rand_type_2 vsmc_logistic_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_logistic_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_lognormal_rand_##name,

static vsmc_rng_rand_type_2 vsmc_lognormal_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_lognormal_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_normal_rand_##name,

static vsmc_rng_rand_type_2 vsmc_normal_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_normal_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_pareto_rand_##name,

static vsmc_rng_rand_type_2 vsmc_pareto_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_pareto_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    vsmc_uniform_real_rand_##name,

static vsmc_rng_rand_type_2 vsmc_uniform_real_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_uniform_real_rand_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_weibull_rand_##name,

static vsmc_rng_rand_type_2 vsmc_weibull_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_weibull_rand_dispatch

#define VSMC_DEFINE_LIB_RNG_DIST_RAND_0(dist)                                 \
    void vsmc_##dist##_rand(vsmc_rng rng, size_t n, double *r)                \
    {                                                                         \
        vsmc_##dist##_rand_dispatch[static_cast<std::size_t>(rng.type)](      \
            rng, n, r);                                                       \
    }

#define VSMC_DEFINE_LIB_RNG_DIST_RAND_1(dist)                                 \
    void vsmc_##dist##_rand(vsmc_rng rng, size_t n, double *r, double p1)     \
    {                                                                         \
        vsmc_##dist##_rand_dispatch[static_cast<std::size_t>(rng.type)](      \
            rng, n, r, p1);                                                   \
    }

#define VSMC_DEFINE_LIB_RNG_DIST_RAND_2(dist)                                 \
    void vsmc_##dist##_rand(                                                  \
        vsmc_rng rng, size_t n, double *r, double p1, double p2)              \
    {                                                                         \
        vsmc_##dist##_rand_dispatch[static_cast<std::size_t>(rng.type)](      \
            rng, n, r, p1, p2);                                               \
    }

VSMC_DEFINE_LIB_RNG_DIST_RAND_0(u01)
VSMC_DEFINE_LIB_RNG_DIST_RAND_0(u01_cc)
VSMC_DEFINE_LIB_RNG_DIST_RAND_0(u01_co)
VSMC_DEFINE_LIB_RNG_DIST_RAND_0(u01_oc)
VSMC_DEFINE_LIB_RNG_DIST_RAND_0(u01_oo)
VSMC_DEFINE_LIB_RNG_DIST_RAND_1(chi_squared)
VSMC_DEFINE_LIB_RNG_DIST_RAND_1(exponential)
VSMC_DEFINE_LIB_RNG_DIST_RAND_1(rayleigh)
VSMC_DEFINE_LIB_RNG_DIST_RAND_1(student_t)
VSMC_DEFINE_LIB_RNG_DIST_RAND_2(beta)
VSMC_DEFINE_LIB_RNG_DIST_RAND_2(cauchy)
VSMC_DEFINE_LIB_RNG_DIST_RAND_2(extreme_value)
VSMC_DEFINE_LIB_RNG_DIST_RAND_2(fisher_f)
VSMC_DEFINE_LIB_RNG_DIST_RAND_2(gamma)
VSMC_DEFINE_LIB_RNG_DIST_RAND_2(laplace)
VSMC_DEFINE_LIB_RNG_DIST_RAND_2(levy)
VSMC_DEFINE_LIB_RNG_DIST_RAND_2(logistic)
VSMC_DEFINE_LIB_RNG_DIST_RAND_2(lognormal)
VSMC_DEFINE_LIB_RNG_DIST_RAND_2(normal)
VSMC_DEFINE_LIB_RNG_DIST_RAND_2(pareto)
VSMC_DEFINE_LIB_RNG_DIST_RAND_2(uniform_real)
VSMC_DEFINE_LIB_RNG_DIST_RAND_2(weibull)

} // extern "C"

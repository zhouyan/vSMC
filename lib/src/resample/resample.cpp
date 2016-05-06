//============================================================================
// vSMC/lib/src/resample/resample.cpp
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
// SUBSTITUTE GOODS OR SERVICES{} LOSS OF USE, DATA, OR PROFITS{} OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#include <vsmc/resample/resample.h>
#include <vsmc/resample/resample.hpp>
#include <vsmc/rng/engine.hpp>

extern "C" {

size_t vsmc_resample_trans_residual(
    size_t n, size_t m, const double *weight, double *resid, size_t *integ)
{
    return ::vsmc::resample_trans_residual(n, m, weight, resid, integ);
}

void vsmc_resample_trans_u01_rep(size_t n, size_t m, const double *weight,
    const double *u01, size_t *replication)
{
    ::vsmc::resample_trans_u01_rep(n, m, weight, u01, replication);
}

void vsmc_resample_trans_rep_index(
    size_t n, size_t m, const size_t *replication, size_t *index)
{
    ::vsmc::resample_trans_rep_index(n, m, replication, index);
}

#define VSMC_DEFINE_LIB_RESAMPLE_DISPATCH(                                    \
    RNGType, Name, name, Scheme, scheme)                                      \
    inline void vsmc_resample_##scheme##_##name(size_t n, size_t m,           \
        vsmc_rng rng, const double *weight, size_t *replication)              \
    {                                                                         \
        ::vsmc::Resample##Scheme resample;                                    \
        resample(n, m, *reinterpret_cast<RNGType *>(rng.ptr), weight,         \
            replication);                                                     \
    }

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#ifdef VSMC_RNG_DEFINE_MACRO_NA
#undef VSMC_RNG_DEFINE_MACRO_NA
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    VSMC_DEFINE_LIB_RESAMPLE_DISPATCH(                                        \
        RNGType, Name, name, Multinomial, multinomial)                        \
    VSMC_DEFINE_LIB_RESAMPLE_DISPATCH(                                        \
        RNGType, Name, name, Stratified, stratified)                          \
    VSMC_DEFINE_LIB_RESAMPLE_DISPATCH(                                        \
        RNGType, Name, name, Systematic, systematic)                          \
    VSMC_DEFINE_LIB_RESAMPLE_DISPATCH(                                        \
        RNGType, Name, name, Residual, residual)                              \
    VSMC_DEFINE_LIB_RESAMPLE_DISPATCH(                                        \
        RNGType, Name, name, ResidualStratified, residual_stratified)         \
    VSMC_DEFINE_LIB_RESAMPLE_DISPATCH(                                        \
        RNGType, Name, name, ResidualSystematic, residual_systematic)

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

using vsmc_resample_type = void (*)(
    size_t, size_t, vsmc_rng, const double *, size_t *);

#ifdef VSMC_RNG_DEFINE_MACRO_NA
#undef VSMC_RNG_DEFINE_MACRO_NA
#endif

#define VSMC_RNG_DEFINE_MACRO_NA(RNGType, Name, name) nullptr,

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    vsmc_resample_multinomial_##name,

static vsmc_resample_type vsmc_resample_multinomial_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_resample_multinomial_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    vsmc_resample_residual_##name,

static vsmc_resample_type vsmc_resample_residual_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_resample_residual_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    vsmc_resample_stratified_##name,

static vsmc_resample_type vsmc_resample_stratified_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_resample_stratified_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    vsmc_resample_systematic_##name,

static vsmc_resample_type vsmc_resample_systematic_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_resample_systematic_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    vsmc_resample_residual_stratified_##name,

static vsmc_resample_type vsmc_resample_residual_stratified_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_resample_residual_stratified_dispatch

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    vsmc_resample_residual_systematic_##name,

static vsmc_resample_type vsmc_resample_residual_systematic_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_resample_residual_systematic_dispatch

#define VSMC_DEFINE_LIB_RESAMPLE(scheme)                                      \
    void vsmc_resample_##scheme(size_t n, size_t m, vsmc_rng rng,             \
        const double *weight, size_t *replication)                            \
    {                                                                         \
        vsmc_resample_##scheme##_dispatch[static_cast<std::size_t>(           \
            rng.type)](n, m, rng, weight, replication);                       \
    }

VSMC_DEFINE_LIB_RESAMPLE(multinomial)
VSMC_DEFINE_LIB_RESAMPLE(stratified)
VSMC_DEFINE_LIB_RESAMPLE(systematic)
VSMC_DEFINE_LIB_RESAMPLE(residual)
VSMC_DEFINE_LIB_RESAMPLE(residual_stratified)
VSMC_DEFINE_LIB_RESAMPLE(residual_systematic)

} // extern "C"

//============================================================================
// vSMC/lib/src/rng/discrete_rand.cpp
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

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#ifdef VSMC_RNG_DEFINE_MACRO_NA
#undef VSMC_RNG_DEFINE_MACRO_NA
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    inline void vsmc_discrete_rand_##name(vsmc_rng rng, size_t n,             \
        long long *r, size_t m, const double *weight, int normalized)         \
    {                                                                         \
        ::vsmc::DiscreteDistribution<long long> dist;                         \
        RNGType &rng_cpp = *reinterpret_cast<RNGType *>(rng.ptr);             \
        const bool norm = normalized != 0;                                    \
        const double *first = weight;                                         \
        const double *last = weight + m;                                      \
        for (size_t i = 0; i != n; ++i)                                       \
            r[i] = dist(rng_cpp, first, last, norm);                          \
    }

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

using vsmc_discrete_rand_type = void (*)(
    vsmc_rng, size_t, long long *, size_t, const double *, int);

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#ifdef VSMC_RNG_DEFINE_MACRO_NA
#undef VSMC_RNG_DEFINE_MACRO_NA
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_discrete_rand_##name,
#define VSMC_RNG_DEFINE_MACRO_NA(RNGType, Name, name) nullptr,

static vsmc_discrete_rand_type vsmc_discrete_rand_dispatch[] = {

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_discrete_rand_dispatch

void vsmc_discrete_rand(vsmc_rng rng, size_t n, long long *r, size_t m,
    const double *weight, int normalized)
{
    vsmc_discrete_rand_dispatch[static_cast<std::size_t>(rng.type)](
        rng, n, r, m, weight, normalized);
}

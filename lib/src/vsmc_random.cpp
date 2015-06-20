//============================================================================
// vSMC/lib/src/vsmc_random.cpp
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

#include <vsmc/rngc/random.h>
#include <vsmc/rng/engine.hpp>
#include <vsmc/rng/stable_distribution.hpp>

#define VSMC_DEFINE_RNG_RANDOM_DIST                                           \
    ::vsmc::RNG &rng = ::vsmc::internal::rng_cast(rng_ptr);                   \
    for (int i = 0; i != n; ++i)                                              \
        r[i] = dist(rng);

namespace vsmc
{

namespace internal
{

inline RNG &rng_cast(vsmc_rng *rng_ptr)
{
    if (reinterpret_cast<std::uintptr_t>(rng_ptr->state) % 32 == 0)
        return *(reinterpret_cast<RNG *>(rng_ptr->state));
    else if (reinterpret_cast<std::uintptr_t>(rng_ptr->state) % 32 == 8)
        return *(reinterpret_cast<RNG *>(rng_ptr->state + 3));
    else if (reinterpret_cast<std::uintptr_t>(rng_ptr->state) % 32 == 16)
        return *(reinterpret_cast<RNG *>(rng_ptr->state + 2));
    else
        return *(reinterpret_cast<RNG *>(rng_ptr->state + 1));
}

} // namespace internal

} // namespace vsmc

extern "C" {

int vsmc_rng_size() { return sizeof(::vsmc::RNG) + 4; }

void vsmc_rng_init(vsmc_rng *rng_ptr, uint32_t seed)
{
    ::vsmc::RNG &rng = ::vsmc::internal::rng_cast(rng_ptr);
    rng = ::vsmc::RNG(seed);
}

void vsmc_rng_seed(vsmc_rng *rng_ptr, uint32_t seed)
{
    ::vsmc::RNG &rng = ::vsmc::internal::rng_cast(rng_ptr);
    rng.seed(seed);
}

void vsmc_rng_key(vsmc_rng *rng_ptr, uint32_t key[4])
{
    ::vsmc::RNG &rng = ::vsmc::internal::rng_cast(rng_ptr);
    typename ::vsmc::RNG::key_type k;
    std::memcpy(k.data(), key, sizeof(uint32_t) * 4);
    rng.key(k);
}

void vsmc_rng_ctr(vsmc_rng *rng_ptr, uint32_t ctr[4])
{
    ::vsmc::RNG &rng = ::vsmc::internal::rng_cast(rng_ptr);
    typename ::vsmc::RNG::ctr_type c;
    std::memcpy(c.data(), ctr, sizeof(uint32_t) * 4);
    rng.ctr(c);
}

void vsmc_rng_gen(vsmc_rng *rng_ptr, int n, unsigned *r)
{
    ::vsmc::RNG &rng = ::vsmc::internal::rng_cast(rng_ptr);
    for (int i = 0; i != n; ++i)
        r[i] = rng();
}

void vsmc_rng_uniform_int(vsmc_rng *rng_ptr, int n, int *r, int a, int b)
{
    std::uniform_int_distribution<int> dist(a, b);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_uniform_real(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b)
{
    std::uniform_real_distribution<double> dist(a, b);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_bernoulli(vsmc_rng *rng_ptr, int n, int *r, double p)
{
    std::bernoulli_distribution dist(p);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_binomial(vsmc_rng *rng_ptr, int n, int *r, int t, double p)
{
    std::binomial_distribution<int> dist(t, p);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_negative_binomial(
    vsmc_rng *rng_ptr, int n, int *r, int k, double p)
{
    std::negative_binomial_distribution<int> dist(k, p);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_geometric(vsmc_rng *rng_ptr, int n, int *r, double p)
{
    std::geometric_distribution<int> dist(p);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_poisson(vsmc_rng *rng_ptr, int n, int *r, double mean)
{
    std::poisson_distribution<int> dist(mean);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_exponential(vsmc_rng *rng_ptr, int n, double *r, double rate)
{
    std::exponential_distribution<double> dist(rate);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_gamma(
    vsmc_rng *rng_ptr, int n, double *r, double shape, double scale)
{
    std::gamma_distribution<double> dist(shape, scale);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_weibull(
    vsmc_rng *rng_ptr, int n, double *r, double shape, double scale)
{
    std::weibull_distribution<double> dist(shape, scale);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_extreme_value(
    vsmc_rng *rng_ptr, int n, double *r, double location, double scale)
{
    std::extreme_value_distribution<double> dist(location, scale);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_normal(
    vsmc_rng *rng_ptr, int n, double *r, double mean, double sd)
{
    std::normal_distribution<double> dist(mean, sd);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_lognormal(
    vsmc_rng *rng_ptr, int n, double *r, double logmean, double logsd)
{
    std::lognormal_distribution<double> dist(logmean, logsd);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_chi_squared(vsmc_rng *rng_ptr, int n, double *r, double df)
{
    std::chi_squared_distribution<double> dist(df);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_cauchy(
    vsmc_rng *rng_ptr, int n, double *r, double location, double scale)
{
    std::cauchy_distribution<double> dist(location, scale);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_fisher_f(
    vsmc_rng *rng_ptr, int n, double *r, double df1, double df2)
{
    std::fisher_f_distribution<double> dist(df1, df2);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_student_t(vsmc_rng *rng_ptr, int n, double *r, double df)
{
    std::student_t_distribution<double> dist(df);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

void vsmc_rng_stable(vsmc_rng *rng_ptr, int n, double *r, double stability,
    double skewness, double location, double scale)
{
    ::vsmc::StableDistribution<double> dist(
        stability, skewness, location, scale);
    VSMC_DEFINE_RNG_RANDOM_DIST;
}

} // extern "C"

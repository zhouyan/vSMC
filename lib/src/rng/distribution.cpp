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

#include "libvsmc.hpp"

#define VSMC_DEFINE_RNG_DIST                                                  \
    ::vsmc::RNG &rng = ::vsmc::cast(rng_ptr);                                 \
    for (int i = 0; i != n; ++i)                                              \
        r[i] = dist(rng);

#define VSMC_DEFINE_RNG_DIST_0(name)                                          \
    void vsmc_##name##_distribution(vsmc_rng *rng_ptr, int n, double *r)      \
    {                                                                         \
        ::vsmc::name##_distribution<double>(                                  \
            ::vsmc::cast(rng_ptr), static_cast<std::size_t>(n), r);           \
    }

#define VSMC_DEFINE_RNG_DIST_1(name)                                          \
    void vsmc_##name##_distribution(                                          \
        vsmc_rng *rng_ptr, int n, double *r, double p1)                       \
    {                                                                         \
        ::vsmc::name##_distribution<double>(                                  \
            ::vsmc::cast(rng_ptr), static_cast<std::size_t>(n), r, p1);       \
    }

#define VSMC_DEFINE_RNG_DIST_2(name)                                          \
    void vsmc_##name##_distribution(                                          \
        vsmc_rng *rng_ptr, int n, double *r, double p1, double p2)            \
    {                                                                         \
        ::vsmc::name##_distribution<double>(                                  \
            ::vsmc::cast(rng_ptr), static_cast<std::size_t>(n), r, p1, p2);   \
    }

extern "C" {

void vsmc_discrete_distribution(vsmc_rng *rng_ptr, int n, int *r, int m,
    const double *weight, int normalized)
{
    ::vsmc::DiscreteDistribution<int> dist;
    ::vsmc::RNG &rng = ::vsmc::cast(rng_ptr);
    bool norm = normalized != 0;
    const double *first = weight;
    const double *last = weight + m;
    for (int i = 0; i != n; ++i)
        r[i] = dist(rng, first, last, norm);
}

void vsmc_uniform_int_distribution(
    vsmc_rng *rng_ptr, int n, int *r, int a, int b)
{
    std::uniform_int_distribution<int> dist(a, b);
    VSMC_DEFINE_RNG_DIST;
}

void vsmc_bernoulli_distribution(vsmc_rng *rng_ptr, int n, int *r, double p)
{
    std::bernoulli_distribution dist(p);
    VSMC_DEFINE_RNG_DIST;
}

void vsmc_binomial_distribution(
    vsmc_rng *rng_ptr, int n, int *r, int t, double p)
{
    std::binomial_distribution<int> dist(t, p);
    VSMC_DEFINE_RNG_DIST;
}

void vsmc_negative_binomial_distribution(
    vsmc_rng *rng_ptr, int n, int *r, int k, double p)
{
    std::negative_binomial_distribution<int> dist(k, p);
    VSMC_DEFINE_RNG_DIST;
}

void vsmc_geometric_distribution(vsmc_rng *rng_ptr, int n, int *r, double p)
{
    std::geometric_distribution<int> dist(p);
    VSMC_DEFINE_RNG_DIST;
}

void vsmc_poisson_distribution(vsmc_rng *rng_ptr, int n, int *r, double mean)
{
    std::poisson_distribution<int> dist(mean);
    VSMC_DEFINE_RNG_DIST;
}

void vsmc_normal_mv_distribution(vsmc_rng *rng_ptr, int n, double *r, int dim,
    const double *mean, const double *chol)
{
    ::vsmc::RNG &rng = ::vsmc::cast(rng_ptr);
    ::vsmc::normal_mv_distribution<double>(rng, static_cast<std::size_t>(n), r,
        static_cast<std::size_t>(dim), mean, chol);
}

VSMC_DEFINE_RNG_DIST_0(u01)
VSMC_DEFINE_RNG_DIST_1(chi_squared)
VSMC_DEFINE_RNG_DIST_1(exponential)
VSMC_DEFINE_RNG_DIST_1(rayleigh)
VSMC_DEFINE_RNG_DIST_1(student_t)
VSMC_DEFINE_RNG_DIST_2(beta)
VSMC_DEFINE_RNG_DIST_2(cauchy)
VSMC_DEFINE_RNG_DIST_2(extreme_value)
VSMC_DEFINE_RNG_DIST_2(fisher_f)
VSMC_DEFINE_RNG_DIST_2(gamma)
VSMC_DEFINE_RNG_DIST_2(laplace)
VSMC_DEFINE_RNG_DIST_2(levy)
VSMC_DEFINE_RNG_DIST_2(logistic)
VSMC_DEFINE_RNG_DIST_2(lognormal)
VSMC_DEFINE_RNG_DIST_2(normal)
VSMC_DEFINE_RNG_DIST_2(pareto)
VSMC_DEFINE_RNG_DIST_2(uniform_real)
VSMC_DEFINE_RNG_DIST_2(weibull)

} // extern "C"

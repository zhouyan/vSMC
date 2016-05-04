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
    ::vsmc::RNGC &cpp_rng = ::vsmc::cast(rng);                                \
    for (int i = 0; i != n; ++i)                                              \
        r[i] = dist(cpp_rng);

#define VSMC_DEFINE_RNG_DIST_0(Name, name)                                    \
    double vsmc_##name##_rand1(vsmc_rng rng)                                  \
    {                                                                         \
        ::vsmc::Name##Distribution<double> dist;                              \
                                                                              \
        return dist(::vsmc::cast(rng));                                       \
    }                                                                         \
                                                                              \
    void vsmc_##name##_rand(vsmc_rng rng, int n, double *r)                   \
    {                                                                         \
        ::vsmc::name##_distribution<double>(                                  \
            ::vsmc::cast(rng), static_cast<std::size_t>(n), r);               \
    }

#define VSMC_DEFINE_RNG_DIST_1(Name, name)                                    \
    double vsmc_##name##_rand1(vsmc_rng rng, double p1)                       \
    {                                                                         \
        ::vsmc::Name##Distribution<double> dist(p1);                          \
                                                                              \
        return dist(::vsmc::cast(rng));                                       \
    }                                                                         \
                                                                              \
    void vsmc_##name##_rand(vsmc_rng rng, int n, double *r, double p1)        \
    {                                                                         \
        ::vsmc::name##_distribution<double>(                                  \
            ::vsmc::cast(rng), static_cast<std::size_t>(n), r, p1);           \
    }

#define VSMC_DEFINE_RNG_DIST_2(Name, name)                                    \
    double vsmc_##name##_rand1(vsmc_rng rng, double p1, double p2)            \
    {                                                                         \
        ::vsmc::Name##Distribution<double> dist(p1, p2);                      \
                                                                              \
        return dist(::vsmc::cast(rng));                                       \
    }                                                                         \
                                                                              \
    void vsmc_##name##_rand(                                                  \
        vsmc_rng rng, int n, double *r, double p1, double p2)                 \
    {                                                                         \
        ::vsmc::name##_distribution<double>(                                  \
            ::vsmc::cast(rng), static_cast<std::size_t>(n), r, p1, p2);       \
    }

extern "C" {

int vsmc_discrete_rand1(
    vsmc_rng rng, int m, const double *weight, int normalized)
{
    ::vsmc::DiscreteDistribution<int> dist;

    return dist(::vsmc::cast(rng), weight, weight + m, normalized != 0);
}

void vsmc_discrete_rand(
    vsmc_rng rng, int n, int *r, int m, const double *weight, int normalized)
{
    ::vsmc::DiscreteDistribution<int> dist;
    ::vsmc::RNGC &cpp_rng = ::vsmc::cast(rng);
    bool norm = normalized != 0;
    const double *first = weight;
    const double *last = weight + m;
    for (int i = 0; i != n; ++i)
        r[i] = dist(cpp_rng, first, last, norm);
}

int vsmc_uniform_int_rand1(vsmc_rng rng, int a, int b)
{
    std::uniform_int_distribution<int> dist(a, b);

    return dist(::vsmc::cast(rng));
}

void vsmc_uniform_int_rand(vsmc_rng rng, int n, int *r, int a, int b)
{
    std::uniform_int_distribution<int> dist(a, b);
    VSMC_DEFINE_RNG_DIST;
}

int vsmc_bernoulli_rand1(vsmc_rng rng, double p)
{
    std::bernoulli_distribution dist(p);

    return dist(::vsmc::cast(rng));
}

void vsmc_bernoulli_rand(vsmc_rng rng, int n, int *r, double p)
{
    std::bernoulli_distribution dist(p);
    VSMC_DEFINE_RNG_DIST;
}

int vsmc_binomial_rand1(vsmc_rng rng, int t, double p)
{
    std::binomial_distribution<int> dist(t, p);

    return dist(::vsmc::cast(rng));
}

void vsmc_binomial_rand(vsmc_rng rng, int n, int *r, int t, double p)
{
    std::binomial_distribution<int> dist(t, p);
    VSMC_DEFINE_RNG_DIST;
}

int vsmc_negative_binomial_rand1(vsmc_rng rng, int k, double p)
{
    std::negative_binomial_distribution<int> dist(k, p);

    return dist(::vsmc::cast(rng));
}

void vsmc_negative_binomial_rand(vsmc_rng rng, int n, int *r, int k, double p)
{
    std::negative_binomial_distribution<int> dist(k, p);
    VSMC_DEFINE_RNG_DIST;
}

int vsmc_geometric_rand1(vsmc_rng rng, double p)
{
    std::geometric_distribution<int> dist(p);

    return dist(::vsmc::cast(rng));
}

void vsmc_geometric_rand(vsmc_rng rng, int n, int *r, double p)
{
    std::geometric_distribution<int> dist(p);
    VSMC_DEFINE_RNG_DIST;
}

int vsmc_poisson_rand1(vsmc_rng rng, double mean)
{
    std::poisson_distribution<int> dist(mean);

    return dist(::vsmc::cast(rng));
}

void vsmc_poisson_rand(vsmc_rng rng, int n, int *r, double mean)
{
    std::poisson_distribution<int> dist(mean);
    VSMC_DEFINE_RNG_DIST;
}

void vsmc_normal_mv_rand1(
    vsmc_rng rng, double *r, int dim, const double *mean, const double *chol)
{
    ::vsmc::NormalMVDistribution<double> dist(
        static_cast<std::size_t>(dim), mean, chol);
    dist(::vsmc::cast(rng), r);
}

void vsmc_normal_mv_rand(vsmc_rng rng, int n, double *r, int dim,
    const double *mean, const double *chol)
{
    ::vsmc::normal_mv_distribution<double>(::vsmc::cast(rng),
        static_cast<std::size_t>(n), r, static_cast<std::size_t>(dim), mean,
        chol);
}

VSMC_DEFINE_RNG_DIST_0(U01, u01)
VSMC_DEFINE_RNG_DIST_0(U01CC, u01_cc)
VSMC_DEFINE_RNG_DIST_0(U01CO, u01_co)
VSMC_DEFINE_RNG_DIST_0(U01OC, u01_oc)
VSMC_DEFINE_RNG_DIST_0(U01OO, u01_oo)
VSMC_DEFINE_RNG_DIST_1(ChiSquared, chi_squared)
VSMC_DEFINE_RNG_DIST_1(Exponential, exponential)
VSMC_DEFINE_RNG_DIST_1(Rayleigh, rayleigh)
VSMC_DEFINE_RNG_DIST_1(StudentT, student_t)
VSMC_DEFINE_RNG_DIST_2(Beta, beta)
VSMC_DEFINE_RNG_DIST_2(Cauchy, cauchy)
VSMC_DEFINE_RNG_DIST_2(ExtremeValue, extreme_value)
VSMC_DEFINE_RNG_DIST_2(FisherF, fisher_f)
VSMC_DEFINE_RNG_DIST_2(Gamma, gamma)
VSMC_DEFINE_RNG_DIST_2(Laplace, laplace)
VSMC_DEFINE_RNG_DIST_2(Levy, levy)
VSMC_DEFINE_RNG_DIST_2(Logistic, logistic)
VSMC_DEFINE_RNG_DIST_2(Lognormal, lognormal)
VSMC_DEFINE_RNG_DIST_2(Normal, normal)
VSMC_DEFINE_RNG_DIST_2(Pareto, pareto)
VSMC_DEFINE_RNG_DIST_2(UniformReal, uniform_real)
VSMC_DEFINE_RNG_DIST_2(Weibull, weibull)

} // extern "C"

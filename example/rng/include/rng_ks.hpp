//============================================================================
// vSMC/example/rng/include/rng_ks.hpp
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

#ifndef VSMC_EXAMPLE_RNG_KS_HPP
#define VSMC_EXAMPLE_RNG_KS_HPP

#include <vsmc/rng/rng.hpp>

#define VSMC_RNG_KS_1(Name, p1)                                               \
    {                                                                         \
        std::stringstream ss;                                                 \
        ss << #Name << "(" << p1 << ")";                                      \
        vsmc::Name##Distribution<double> dist(p1);                            \
        rng_ks(N, dist, ss.str());                                            \
    }

#define VSMC_RNG_KS_2(Name, p1, p2)                                           \
    {                                                                         \
        std::stringstream ss;                                                 \
        ss << #Name << "(" << p1 << "," << p2 << ")";                         \
        vsmc::Name##Distribution<double> dist(p1, p2);                        \
        rng_ks(N, dist, ss.str());                                            \
    }

template <typename RealType>
double rng_ks(vsmc::NormalDistribution<RealType> &)
{
    return 2.492;
}

template <typename DistType>
double rng_ks(std::size_t n, DistType &dist, double *r)
{
    vsmc::Vector<double> f(n);
    vsmc::Vector<double> head(n);
    vsmc::Vector<double> tail(n);

    std::sort(r, r + n);
    dist.cdf(n, r, f.data());
    vsmc::log(n, f.data(), head.data());
    vsmc::sub(n, 1.0, f.data(), f.data());
    vsmc::log(n, f.data(), tail.data());
    std::reverse(tail.begin(), tail.end());
    vsmc::add(n, head.data(), tail.data(), f.data());
    for (std::size_t i = 0; i != n; ++i)
        f[i] *= 2.0 * i + 1.0;

    return -(n + std::accumulate(f.begin(), f.end(), 0.0) / n);
}

template <typename DistType>
void rng_ks(std::size_t n, DistType &dist, const std::string &name)
{
    std::cout << std::setw(20) << std::left << name;
    const double critic = rng_ks(dist);
    double s = 0;
    std::string t;
    vsmc::RNG rng;
    vsmc::Vector<double> r(n);

    for (std::size_t i = 0; i != n; ++i)
        r[i] = dist(rng);
    s = rng_ks(n, dist, r.data());
    t = s < critic ? "Pass" : "Fail";
    std::cout << std::setw(15) << std::right << s;
    std::cout << std::setw(15) << std::right << t;

    dist(rng, n, r.data());
    s = rng_ks(n, dist, r.data());
    t = s < critic ? "Pass" : "Fail";
    std::cout << std::setw(15) << std::right << s;
    std::cout << std::setw(15) << std::right << t;

    std::cout << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_KS_HPP

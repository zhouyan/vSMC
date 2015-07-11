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

#include "rng_dist.hpp"

#define VSMC_RNG_KS_1(Name, STD, p1)                                          \
    param1[0] = p1;                                                           \
    rng_ks<STD<double>, vsmc::Name##Distribution<double>>(                    \
        N, param1, #Name, names, stat, pval);

#define VSMC_RNG_KS_2(Name, STD, p1, p2)                                      \
    param2[0] = p1;                                                           \
    param2[1] = p2;                                                           \
    rng_ks<STD<double>, vsmc::Name##Distribution<double>>(                    \
        N, param2, #Name, names, stat, pval);

template <typename DistType>
inline double rng_ks_stat(std::size_t n, DistType &dist, double *r)
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

inline double rng_ks_pval(
    std::size_t, vsmc::NormalDistribution<double> &, double s)
{
    if (s < 1.610)
        return 15;
    else if (s < 1.933)
        return 10;
    else if (s < 2.492)
        return 5;
    else if (s < 3.070)
        return 2.5;
    else if (s < 3.875)
        return 1;
    else
        return 0;
}

template <typename STDDistType, typename vSMCDistType, std::size_t K>
inline void rng_ks(std::size_t n, const std::array<double, K> &param,
    const std::string &name, vsmc::Vector<std::string> &names,
    vsmc::Vector<double> &stat, vsmc::Vector<double> &pval)
{
    names.push_back(rng_dist_name(name, param));

    vsmc::Vector<double> r(n);
    vsmc::RNG rng;
    STDDistType dist_std(rng_dist_init<STDDistType>(param));
    vSMCDistType dist_vsmc(rng_dist_init<vSMCDistType>(param));

    for (std::size_t i = 0; i != n; ++i)
        r[i] = dist_std(rng);
    stat.push_back(rng_ks_stat(n, dist_vsmc, r.data()));
    pval.push_back(rng_ks_pval(n, dist_vsmc, stat.back()));

    for (std::size_t i = 0; i != n; ++i)
        r[i] = dist_vsmc(rng);
    stat.push_back(rng_ks_stat(n, dist_vsmc, r.data()));
    pval.push_back(rng_ks_pval(n, dist_vsmc, stat.back()));

    dist_vsmc(rng, n, r.data());
    stat.push_back(rng_ks_stat(n, dist_vsmc, r.data()));
    pval.push_back(rng_ks_pval(n, dist_vsmc, stat.back()));

#if VSMC_HAS_MKL
    vsmc::MKL_SFMT19937 rng_mkl;
    dist_vsmc(rng_mkl, n, r.data());
    stat.push_back(rng_ks_stat(n, dist_vsmc, r.data()));
    pval.push_back(rng_ks_pval(n, dist_vsmc, stat.back()));
#endif
}

inline void rng_ks_output(const vsmc::Vector<std::string> &names,
    const vsmc::Vector<double> &stat, const vsmc::Vector<double> &pval)
{
    std::size_t N = names.size();
    std::size_t R = stat.size() / N;
    std::size_t lwid = 80;
    int twid = 15;
    int Twid = twid * static_cast<int>(R);
    int nwid = static_cast<int>(lwid) - Twid;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::left << std::setw(nwid) << "Distribution";
    std::cout << std::right << std::setw(twid) << "Test (STD)";
    std::cout << std::right << std::setw(twid) << "Test (vSMC)";
    std::cout << std::right << std::setw(twid) << "Test (Batch)";
#if VSMC_HAS_MKL
    std::cout << std::right << std::setw(twid) << "Test (MKL)";
#endif
    std::cout << std::endl;

    std::cout << std::string(lwid, '-') << std::endl;
    for (std::size_t i = 0; i != N; ++i) {
        std::cout << std::left << std::setw(nwid) << names[i];
        for (std::size_t r = 0; r != R; ++r) {
            double value = stat[i * R + r];
            std::cout << std::right << std::setw(twid) << std::fixed << value;
        }
        std::cout << std::endl;
        std::cout << std::left << std::setw(nwid) << ' ';
        for (std::size_t r = 0; r != R; ++r) {
            double value = pval[i * R + r];
            std::stringstream ss;
            if (value > 0)
                ss << "> " << value << '%';
            else
                ss << "< 1%";
            std::cout << std::right << std::setw(twid) << ss.str();
        }
        std::cout << std::endl;
    }
    std::cout << std::string(lwid, '=') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_KS_HPP

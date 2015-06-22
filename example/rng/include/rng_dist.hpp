//============================================================================
// vSMC/example/rng/include/rng_dist.hpp
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

#ifndef VSMC_EXAMPLE_RNG_DIST_HPP
#define VSMC_EXAMPLE_RNG_DIST_HPP

#include "rng_test.hpp"
#include <vsmc/rng/engine.hpp>
#include <vsmc/rng/uniform_real_distribution.hpp>
#include <vsmc/rng/stable_distribution.hpp>
#include <vsmc/utility/stop_watch.hpp>

#define VSMC_RNG_DIST_C_PRE(Dist, name)                                       \
    vsmc::Vector<DistResultType<Dist>> r(N);                                  \
    vsmc::StopWatch watch;                                                    \
    vsmc_rng rng;                                                             \
    vsmc_rng_init(&rng, 1);                                                   \
    watch.start();

#define VSMC_RNG_DIST_C_POST(Dist, name)                                      \
    watch.stop();                                                             \
    sw.push_back(watch);                                                      \
    std::ofstream rnd("rnd");                                                 \
    rnd << r.back() << std::endl;                                             \
    rnd.close();

#define VSMC_RNG_DIST_1_C(Dist, name, p1)                                     \
    {                                                                         \
        std::string dname(#name "(" #p1 ")");                                 \
        VSMC_RNG_DIST_C_PRE(Dist, name);                                      \
        vsmc_rng_##name(&rng, static_cast<int>(N), r.data(), p1);             \
        VSMC_RNG_DIST_C_POST(Dist, name);                                     \
    }

#define VSMC_RNG_DIST_2_C(Dist, name, p1, p2)                                 \
    {                                                                         \
        std::string dname(#name "(" #p1 "," #p2 ")");                         \
        VSMC_RNG_DIST_C_PRE(Dist, name);                                      \
        vsmc_rng_##name(&rng, static_cast<int>(N), r.data(), p1, p2);         \
        VSMC_RNG_DIST_C_POST(Dist, name);                                     \
    }

#define VSMC_RNG_DIST_4_C(Dist, name, p1, p2, p3, p4)                         \
    {                                                                         \
        std::string dname(#name "(" #p1 "," #p2 "," #p3 "," #p4 ")");         \
        VSMC_RNG_DIST_C_PRE(Dist, name);                                      \
        vsmc_rng_##name(&rng, static_cast<int>(N), r.data(), p1, p2, p3, p4); \
        VSMC_RNG_DIST_C_POST(Dist, name);                                     \
    }

#define VSMC_RNG_DIST_1_CPP(Dist, name, p1)                                   \
    {                                                                         \
        Dist dist(p1);                                                        \
        std::string dname(#name "(" #p1 ")");                                 \
        rng_dist(N, dist, dname, names, size, sw);                            \
    }

#define VSMC_RNG_DIST_2_CPP(Dist, name, p1, p2)                               \
    {                                                                         \
        Dist dist(p1, p2);                                                    \
        std::string dname(#name "(" #p1 "," #p2 ")");                         \
        rng_dist(N, dist, dname, names, size, sw);                            \
    }

#define VSMC_RNG_DIST_4_CPP(Dist, name, p1, p2, p3, p4)                       \
    {                                                                         \
        Dist dist(p1, p2, p3, p4);                                            \
        std::string dname(#name "(" #p1 "," #p2 "," #p3 "," #p4 ")");         \
        rng_dist(N, dist, dname, names, size, sw);                            \
    }

#if VSMC_RNG_TEST_C_API
#define VSMC_RNG_DIST_1(Dist, name, p1)                                       \
    VSMC_RNG_DIST_1_CPP(Dist, name, p1);                                      \
    VSMC_RNG_DIST_1_C(Dist, name, p1);
#define VSMC_RNG_DIST_2(Dist, name, p1, p2)                                   \
    VSMC_RNG_DIST_2_CPP(Dist, name, p1, p2);                                  \
    VSMC_RNG_DIST_2_C(Dist, name, p1, p2);
#define VSMC_RNG_DIST_4(Dist, name, p1, p2, p3, p4)                           \
    VSMC_RNG_DIST_4_CPP(Dist, name, p1, p2, p3, p4);                          \
    VSMC_RNG_DIST_4_C(Dist, name, p1, p2, p3, p4);
#else
#define VSMC_RNG_DIST_1 VSMC_RNG_DIST_1_CPP
#define VSMC_RNG_DIST_2 VSMC_RNG_DIST_2_CPP
#define VSMC_RNG_DIST_4 VSMC_RNG_DIST_4_CPP
#endif

template <typename Dist>
class DistResultTypeTrait
{
    public:
    using type = typename Dist::result_type;
};

template <>
class DistResultTypeTrait<std::bernoulli_distribution>
{
    public:
    using type = int;
};

template <typename Dist>
using DistResultType = typename DistResultTypeTrait<Dist>::type;

template <typename Dist>
inline void rng_dist(std::size_t N, Dist &dist, const std::string &name,
    vsmc::Vector<std::string> &names, vsmc::Vector<std::size_t> &size,
    vsmc::Vector<vsmc::StopWatch> &sw)
{
    vsmc::StopWatch watch;
    vsmc::RNG rng;

    watch.start();
    DistResultType<Dist> result = 0;
    for (std::size_t i = 0; i != N; ++i)
        result += dist(rng);
    watch.stop();

    names.push_back(name);
    size.push_back(sizeof(Dist));
    sw.push_back(watch);

    std::ofstream rnd("rnd");
    rnd << result << std::endl;
    rnd.close();
}

#endif // VSMC_EXAMPLE_RNG_DIST_HPP

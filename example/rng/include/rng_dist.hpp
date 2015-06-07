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

#define VSMC_RNG_DIST_1(Dist, p1)                                             \
    {                                                                         \
        Dist dist(p1);                                                        \
        rng_dist(N, dist, #Dist "(" #p1 ")", names, size, sw, bytes);         \
    }

#define VSMC_RNG_DIST_2(Dist, p1, p2)                                         \
    {                                                                         \
        Dist dist(p1, p2);                                                    \
        rng_dist(                                                             \
            N, dist, #Dist "(" #p1 ", " #p2 ")", names, size, sw, bytes);     \
    }

template <typename Dist>
inline void rng_dist(std::size_t N, Dist &dist, const std::string &name,
    vsmc::Vector<std::string> &names, vsmc::Vector<std::size_t> &size,
    vsmc::Vector<vsmc::StopWatch> &sw, vsmc::Vector<std::size_t> &bytes)
{
    vsmc::Rng rng;
    vsmc::StopWatch watch;

    watch.start();
    for (std::size_t i = 0; i != N; ++i)
        dist(rng);
    watch.stop();
    std::ofstream rnd("rnd");
    rnd << dist(rng) << std::endl;
    rnd.close();

    names.push_back(name);
    size.push_back(sizeof(Dist));
    sw.push_back(watch);
    bytes.push_back(N * sizeof(typename Dist::result_type));
}

#endif // VSMC_EXAMPLE_RNG_DIST_HPP

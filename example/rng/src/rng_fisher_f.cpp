//============================================================================
// vSMC/example/rng/src/rng_fisher_f.cpp
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

#include <vsmc/rng/fisher_f_distribution.hpp>
#include <boost/math/distributions/fisher_f.hpp>
#include "rng_dist.hpp"

template <>
inline vsmc::Vector<double>
    rng_dist_partition<vsmc::FisherFDistribution<double>>(
        std::size_t n, vsmc::FisherFDistribution<double> &dist)
{
    return rng_dist_partition_boost(
        n, boost::math::fisher_f_distribution<double>(dist.m(), dist.n()));
}

int main(int argc, char **argv)
{
    VSMC_RNG_DIST_PRE(2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 0.2, 0.2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 0.2, 1);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 0.2, 1.5);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 0.2, 2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 0.2, 3);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 1, 0.2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 1, 1);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 1, 1.5);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 1, 2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 1, 3);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 2, 0.2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 2, 1);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 2, 1.5);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 2, 2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 2, 3);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 3, 0.2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 3, 1);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 3, 1.5);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 3, 2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 3, 3);
    VSMC_RNG_DIST_POST;

    return 0;
}

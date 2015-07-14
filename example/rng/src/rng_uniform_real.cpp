//============================================================================
// vSMC/example/rng/src/rng_uniform_real.cpp
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

#include "rng_dist.hpp"
#include <vsmc/rng/uniform_real_distribution.hpp>

int main(int argc, char **argv)
{
    VSMC_RNG_DIST_PRE(2);
    VSMC_RNG_DIST_2(UniformReal, std::uniform_real_distribution, 0, 1);
    VSMC_RNG_DIST_2(UniformRealCC, vsmc::UniformRealCCDistribution, 0, 1);
    VSMC_RNG_DIST_2(UniformRealCO, vsmc::UniformRealCODistribution, 0, 1);
    VSMC_RNG_DIST_2(UniformRealOC, vsmc::UniformRealOCDistribution, 0, 1);
    VSMC_RNG_DIST_2(UniformRealOO, vsmc::UniformRealOODistribution, 0, 1);
    VSMC_RNG_DIST_POST;

    return 0;
}
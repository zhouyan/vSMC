//============================================================================
// vSMC/example/rng/include/rng_u01.cpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c); 2013-2016, Yan Zhou
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
// INTERRUPTION); HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE);
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#include "rng_u01.hpp"
#include <vsmc/rng/u01_distribution.hpp>
#include "rng_dist.hpp"

int main(int argc, char **argv)
{
    rng_u01_lr<std::uint32_t, float, vsmc::Closed, vsmc::Closed>();
    rng_u01_lr<std::uint64_t, float, vsmc::Closed, vsmc::Closed>();
    rng_u01_lr<std::uint32_t, double, vsmc::Closed, vsmc::Closed>();
    rng_u01_lr<std::uint64_t, double, vsmc::Closed, vsmc::Closed>();
    rng_u01_lr<std::uint32_t, long double, vsmc::Closed, vsmc::Closed>();
    rng_u01_lr<std::uint64_t, long double, vsmc::Closed, vsmc::Closed>();

    rng_u01_lr<std::uint32_t, float, vsmc::Closed, vsmc::Open>();
    rng_u01_lr<std::uint64_t, float, vsmc::Closed, vsmc::Open>();
    rng_u01_lr<std::uint32_t, double, vsmc::Closed, vsmc::Open>();
    rng_u01_lr<std::uint64_t, double, vsmc::Closed, vsmc::Open>();
    rng_u01_lr<std::uint32_t, long double, vsmc::Closed, vsmc::Open>();
    rng_u01_lr<std::uint64_t, long double, vsmc::Closed, vsmc::Open>();

    rng_u01_lr<std::uint32_t, float, vsmc::Open, vsmc::Closed>();
    rng_u01_lr<std::uint64_t, float, vsmc::Open, vsmc::Closed>();
    rng_u01_lr<std::uint32_t, double, vsmc::Open, vsmc::Closed>();
    rng_u01_lr<std::uint64_t, double, vsmc::Open, vsmc::Closed>();
    rng_u01_lr<std::uint32_t, long double, vsmc::Open, vsmc::Closed>();
    rng_u01_lr<std::uint64_t, long double, vsmc::Open, vsmc::Closed>();

    rng_u01_lr<std::uint32_t, float, vsmc::Open, vsmc::Open>();
    rng_u01_lr<std::uint64_t, float, vsmc::Open, vsmc::Open>();
    rng_u01_lr<std::uint32_t, double, vsmc::Open, vsmc::Open>();
    rng_u01_lr<std::uint64_t, double, vsmc::Open, vsmc::Open>();
    rng_u01_lr<std::uint32_t, long double, vsmc::Open, vsmc::Open>();
    rng_u01_lr<std::uint64_t, long double, vsmc::Open, vsmc::Open>();

    std::cout << std::string(50, '=') << std::endl;

    vsmc::Vector<std::array<double, 0>> params(1);
    VSMC_RNG_DIST_TEST(0, U01, std::uniform_real_distribution);
    VSMC_RNG_DIST_TEST(0, U01CC, std::uniform_real_distribution);
    VSMC_RNG_DIST_TEST(0, U01CO, std::uniform_real_distribution);
    VSMC_RNG_DIST_TEST(0, U01OC, std::uniform_real_distribution);
    VSMC_RNG_DIST_TEST(0, U01OO, std::uniform_real_distribution);

    return 0;
}

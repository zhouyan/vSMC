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

int main()
{
    VSMC_RNG_U01_TEST(32, float);
    VSMC_RNG_U01_TEST(64, float);
    VSMC_RNG_U01_TEST(32, double);
    VSMC_RNG_U01_TEST(64, double);
    VSMC_RNG_U01_TEST(32, long double);
    VSMC_RNG_U01_TEST(64, long double);

    std::cout << std::string(80, '=') << std::endl;

    VSMC_RNG_U01_FIXED_POINT_TEST(
        32, 32, float, Closed, Closed, closed, closed);
    VSMC_RNG_U01_FIXED_POINT_TEST(32, 32, float, Closed, Open, closed, open);
    VSMC_RNG_U01_FIXED_POINT_TEST(32, 32, float, Open, Closed, open, closed);
    VSMC_RNG_U01_FIXED_POINT_TEST(32, 32, float, Open, Open, open, open);

    VSMC_RNG_U01_FIXED_POINT_TEST(
        64, 32, float, Closed, Closed, closed, closed);
    VSMC_RNG_U01_FIXED_POINT_TEST(64, 32, float, Closed, Open, closed, open);
    VSMC_RNG_U01_FIXED_POINT_TEST(64, 32, float, Open, Closed, open, closed);
    VSMC_RNG_U01_FIXED_POINT_TEST(64, 32, float, Open, Open, open, open);

    VSMC_RNG_U01_FIXED_POINT_TEST(
        32, 64, double, Closed, Closed, closed, closed);
    VSMC_RNG_U01_FIXED_POINT_TEST(32, 64, double, Closed, Open, closed, open);
    VSMC_RNG_U01_FIXED_POINT_TEST(32, 64, double, Open, Closed, open, closed);
    VSMC_RNG_U01_FIXED_POINT_TEST(32, 64, double, Open, Open, open, open);

    VSMC_RNG_U01_FIXED_POINT_TEST(
        64, 64, double, Closed, Closed, closed, closed);
    VSMC_RNG_U01_FIXED_POINT_TEST(64, 64, double, Closed, Open, closed, open);
    VSMC_RNG_U01_FIXED_POINT_TEST(64, 64, double, Open, Closed, open, closed);
    VSMC_RNG_U01_FIXED_POINT_TEST(64, 64, double, Open, Open, open, open);

#if VSMC_HAS_X86
    VSMC_RNG_U01_FIXED_POINT_TEST(
        32, 80, long double, Closed, Closed, closed, closed);
    VSMC_RNG_U01_FIXED_POINT_TEST(
        32, 80, long double, Closed, Open, closed, open);
    VSMC_RNG_U01_FIXED_POINT_TEST(
        32, 80, long double, Open, Closed, open, closed);
    VSMC_RNG_U01_FIXED_POINT_TEST(32, 80, long double, Open, Open, open, open);

    VSMC_RNG_U01_FIXED_POINT_TEST(
        64, 80, long double, Closed, Closed, closed, closed);
    VSMC_RNG_U01_FIXED_POINT_TEST(
        64, 80, long double, Closed, Open, closed, open);
    VSMC_RNG_U01_FIXED_POINT_TEST(
        64, 80, long double, Open, Closed, open, closed);
    VSMC_RNG_U01_FIXED_POINT_TEST(64, 80, long double, Open, Open, open, open);
#endif // VSMC_HAS_X86

    return 0;
}

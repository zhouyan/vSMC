//============================================================================
// vSMC/example/rng/include/rng_uniform_bits.hpp
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

#include <vsmc/rng/threefry.hpp>
#include <vsmc/rng/uniform_bits_distribution.hpp>
#include <vsmc/utility/stop_watch.hpp>

inline void rng_uniform_bits_test()
{
    const std::size_t n = 1 << 16;
    vsmc::Vector<std::uint64_t> r1(n);
    vsmc::Vector<std::uint64_t> r2(n);
    vsmc::Vector<std::uint64_t> r3(n);
    vsmc::Vector<std::uint32_t> r4(n * 2);
    vsmc::Vector<std::uint32_t> r5(n * 2);
    vsmc::StopWatch watch1;
    vsmc::StopWatch watch2;
    vsmc::StopWatch watch3;
    vsmc::StopWatch watch4;
    vsmc::StopWatch watch5;
    vsmc::Threefry4x64 rng;
    bool passed = true;
    const std::size_t size = sizeof(std::uint64_t) * n;

    rng.seed(101);
    watch1.start();
    vsmc::rng_rand(rng, n, r1.data());
    watch1.stop();

    rng.seed(101);
    watch2.start();
    for (std::size_t i = 0; i != n; ++i)
        r2[i] = vsmc::UniformBits<std::uint64_t>::eval(rng);
    watch2.stop();
    passed = passed && std::memcmp(r1.data(), r2.data(), size) == 0;

    rng.seed(101);
    watch3.start();
    vsmc::uniform_bits_distribution(rng, n, r3.data());
    watch3.stop();
    passed = passed && std::memcmp(r1.data(), r3.data(), size) == 0;

    rng.seed(101);
    watch4.start();
    for (std::size_t i = 0; i != n; ++i)
        r4[i] = vsmc::UniformBits<std::uint32_t>::eval(rng);
    watch4.stop();
    for (std::size_t i = 0; i != n; ++i)
        passed = passed && static_cast<std::uint32_t>(r1[i]) == r4[i];

    rng.seed(101);
    watch5.start();
    vsmc::uniform_bits_distribution(rng, n * 2, r5.data());
    watch5.stop();
    passed = passed && std::memcmp(r1.data(), r5.data(), size) == 0;

    std::cout << watch1.milliseconds() << std::endl;
    std::cout << watch2.milliseconds() << std::endl;
    std::cout << watch3.milliseconds() << std::endl;
    std::cout << watch4.milliseconds() << std::endl;
    std::cout << watch5.milliseconds() << std::endl;
    std::cout << (passed ? "Passed" : "Failed") << std::endl;
}

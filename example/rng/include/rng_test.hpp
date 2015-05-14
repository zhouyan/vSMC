//============================================================================
// vSMC/example/rng/include/rng_test.hpp
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

#ifndef VSMC_EXAMPLE_RNG_TEST_HPP
#define VSMC_EXAMPLE_RNG_TEST_HPP

#include <vsmc/utility/rdtsc.hpp>
#include <vsmc/utility/aligned_memory.hpp>
#include <vsmc/utility/stop_watch.hpp>

#define VSMC_RNG_TEST_PRE(prog)                                               \
    std::size_t N = 1000000;                                                  \
    std::string prog_name(#prog);                                             \
    if (argc > 1)                                                             \
        N = static_cast<std::size_t>(std::atoi(argv[1]));                     \
    std::vector<std::string> names;                                           \
    std::vector<std::size_t> size;                                            \
    std::vector<vsmc::StopWatch> sw;                                          \
    std::vector<std::size_t> bytes;                                           \
    std::vector<std::uint64_t> cycles;

#define VSMC_RNG_TEST(RNG)                                                    \
    rng_test<RNG>(N, #RNG, names, size, sw, bytes, cycles);

#define VSMC_RNG_TEST_POST                                                    \
    rng_output_sw(N, prog_name, names, size, sw, bytes, cycles);

template <typename RNG>
inline void rng_test(std::size_t N, const std::string &name,
    std::vector<std::string> &names, std::vector<std::size_t> &size,
    std::vector<vsmc::StopWatch> &sw, std::vector<std::size_t> &bytes,
    std::vector<std::uint64_t> &cycles)
{
    RNG rng;
    vsmc::StopWatch watch;
#if VSMC_HAS_RDTSCP
    vsmc::RDTSCPCounter counter;
#else
    vsmc::RDTSCCounter counter;
#endif

    typename RNG::result_type result = 0;
    watch.start();
    counter.start();
    for (std::size_t i = 0; i != N; ++i)
        result += rng();
    counter.stop();
    watch.stop();
    std::ofstream rnd("rnd");
    rnd << result << std::endl;
    rnd.close();

    names.push_back(name);
    size.push_back(sizeof(RNG));
    sw.push_back(watch);
    bytes.push_back(N * sizeof(typename RNG::result_type));
    cycles.push_back(counter.cycles());
}

inline void rng_output_sw(std::size_t N, const std::string &prog_name,
    const std::vector<std::string> &names, std::vector<std::size_t> &size,
    const std::vector<vsmc::StopWatch> &sw,
    const std::vector<std::size_t> &bytes, std::vector<std::uint64_t> &cycles)
{
    std::size_t M = names.size();
    if (M == 0)
        return;
    if (sw.size() != M)
        return;

    std::cout << std::string(120, '=') << std::endl;
    std::cout << std::left << std::setw(55) << prog_name;
    std::cout << std::right << std::setw(5) << "Size";
    std::cout << std::right << std::setw(15) << "Time (ms)";
    std::cout << std::right << std::setw(15) << "GB/s";
    std::cout << std::right << std::setw(15) << "cpN";
    std::cout << std::right << std::setw(15) << "cpB";
    std::cout << std::endl;
    std::cout << std::string(120, '-') << std::endl;

    for (std::size_t i = 0; i != M; ++i) {
        double time = sw[i].milliseconds();
        double b = static_cast<double>(bytes[i]);
        double c = static_cast<double>(cycles[i]);
        double gbps = b / time * 1e-6;
        double cpN = c / N;
        double cpB = c / b;
        std::cout << std::left << std::setw(55) << names[i];
        std::cout << std::right << std::setw(5) << size[i];
        std::cout << std::right << std::setw(15) << std::fixed << time;
        std::cout << std::right << std::setw(15) << std::fixed << gbps;
        std::cout << std::right << std::setw(15) << std::fixed << cpN;
        std::cout << std::right << std::setw(15) << std::fixed << cpB;
        std::cout << std::endl;
    }
    std::cout << std::string(120, '=') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_TEST_HPP

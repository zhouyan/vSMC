//============================================================================
// vSMC/example/utility/include/utility_aligned_memory.hpp
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

#ifndef VSMC_EXAMPLE_UTILITY_ALIGNED_MEMORY_HPP
#define VSMC_EXAMPLE_UTILITY_ALIGNED_MEMORY_HPP

#include <vsmc/rng/rng.hpp>
#include <vsmc/utility/aligned_memory.hpp>
#include <vsmc/utility/stop_watch.hpp>

template <typename T, bool ConstructScalar, std::size_t Alignment,
    typename Memory>
inline void aligned_memory_test(std::size_t N, std::size_t m,
    const std::string &tname, const std::string &memory)
{
    std::cout << std::string(80, '=') << std::endl;
    std::cout << std::setw(60) << std::left << "Type name" << std::setw(20)
              << std::right << tname << std::endl;
    std::cout << std::setw(60) << std::left << "ConstructScalar"
              << std::setw(20) << std::right
              << (ConstructScalar ? "True" : "False") << std::endl;
    std::cout << std::setw(60) << std::left << "Alignment" << std::setw(20)
              << std::right << Alignment << std::endl;
    std::cout << std::setw(60) << std::left << "Memory" << std::setw(20)
              << std::right << memory << std::endl;
    std::cout << std::string(80, '-') << std::endl;

    bool passed = true;
    bool zeroed = true;
    vsmc::StopWatch watch_alloc;
    vsmc::StopWatch watch_dealloc;
    vsmc::StopWatch watch_vec;
    vsmc::StopWatch watch_val;
    vsmc::RNG rng;
    std::uniform_int_distribution<std::size_t> runif(N / 2, N * 2);

    using Alloc = vsmc::Allocator<T, ConstructScalar, Alignment, Memory>;
    using Vec = std::vector<T, Alloc>;
    Alloc alloc;

    for (std::size_t i = 0; i != m; ++i) {
        if (!passed)
            break;

        bool flag;
        std::size_t n = runif(rng);

        watch_alloc.start();
        auto ptr = alloc.allocate(n);
        watch_alloc.stop();
        flag = reinterpret_cast<std::uintptr_t>(ptr) % Alignment == 0;
        if (!flag)
            std::cout << "Failed alignment (allocate)" << std::endl;
        passed = passed && flag;

        watch_dealloc.start();
        alloc.deallocate(ptr);
        watch_dealloc.stop();

        T zero = static_cast<T>(0);
        watch_vec.start();
        Vec vec(n);
        watch_vec.stop();
        flag = reinterpret_cast<std::uintptr_t>(vec.data()) % Alignment == 0;
        if (!flag)
            std::cout << "Failed alignment (vector(n))" << std::endl;
        passed = passed && flag;
        flag = true;
        for (std::size_t j = 0; j != n; ++j)
            flag = flag && vsmc::internal::is_equal(vec[j], zero);
        zeroed = zeroed && flag;
        if (ConstructScalar) {
            if (!flag)
                std::cout << "Failed initialization (vector(n))" << std::endl;
            passed = passed && flag;
        }

        T v = static_cast<T>(123.456);
        watch_val.start();
        Vec val(n, v);
        watch_val.stop();
        flag = reinterpret_cast<std::uintptr_t>(val.data()) % Alignment == 0;
        if (!flag)
            std::cout << "Failed alignment (vector(n, val))" << std::endl;
        flag = true;
        for (std::size_t j = 0; j != n; ++j)
            flag = flag && vsmc::internal::is_equal(val[j], v);
        if (!flag)
            std::cout << "Failed initialization (vector(n, val))" << std::endl;
        passed = passed && flag;
    }

    std::cout << std::setw(60) << std::left << "Time (ms) for allocate(n)"
              << std::setw(20) << std::right << std::fixed
              << watch_alloc.milliseconds() << std::endl;
    std::cout << std::setw(60) << std::left << "Time (ms) for deallocate(ptr)"
              << std::setw(20) << std::right << std::fixed
              << watch_dealloc.milliseconds() << std::endl;
    std::cout << std::setw(60) << std::left << "Time (ms) for vector(n)"
              << std::setw(20) << std::right << std::fixed
              << watch_vec.milliseconds() << std::endl;
    std::cout << std::setw(60) << std::left << "Time (ms) for vector(n, val)"
              << std::setw(20) << std::right << std::fixed
              << watch_val.milliseconds() << std::endl;
    std::cout << std::setw(60) << std::left << "Zeroed vector(n)"
              << std::setw(20) << std::right << (zeroed ? "True" : "False")
              << std::endl;
    std::cout << std::setw(60) << std::left << "Test result" << std::setw(20)
              << std::right << (passed ? "Passed" : "Failed") << std::endl;
    std::cout << std::string(80, '-') << std::endl;
}

template <typename T, bool ConstructScalar, std::size_t Alignment>
inline void aligned_memory_test(
    std::size_t n, std::size_t m, const std::string &tname)
{
    aligned_memory_test<T, ConstructScalar, Alignment, vsmc::AlignedMemorySTD>(
        n, m, tname, "AlignedMemorySTD");
#if VSMC_HAS_POSIX || defined(VSMC_MSVC)
    aligned_memory_test<T, ConstructScalar, Alignment, vsmc::AlignedMemorySYS>(
        n, m, tname, "AlignedMemorySYS");
#endif
#if VSMC_HAS_TBB_MALLOC
    aligned_memory_test<T, ConstructScalar, Alignment, vsmc::AlignedMemoryTBB>(
        n, m, tname, "AlignedMemoryTBB");
#endif
#if VSMC_HAS_MKL
    aligned_memory_test<T, ConstructScalar, Alignment, vsmc::AlignedMemoryMKL>(
        n, m, tname, "AlignedMemoryMKL");
#endif
}

template <typename T, bool ConstructScalar>
inline void aligned_memory_test(
    std::size_t n, std::size_t m, const std::string &tname)
{
    // aligned_memory_test<T, ConstructScalar, 8>(n, m, tname);
    // aligned_memory_test<T, ConstructScalar, 16>(n, m, tname);
    aligned_memory_test<T, ConstructScalar, 32>(n, m, tname);
}

template <typename T>
inline void aligned_memory_test(
    std::size_t n, std::size_t m, const std::string &tname)
{
    aligned_memory_test<T, true>(n, m, tname);
    aligned_memory_test<T, false>(n, m, tname);
}

#endif // VSMC_EXAMPLE_UTILITY_ALIGNED_MEMORY_HPP

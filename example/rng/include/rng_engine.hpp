//============================================================================
// vSMC/example/rng/include/rng_engine.hpp
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

#ifndef VSMC_EXAMPLE_RNG_ENGINE_HPP
#define VSMC_EXAMPLE_RNG_ENGINE_HPP

#include <vsmc/rng/engine.hpp>
#include <vsmc/utility/stop_watch.hpp>

inline std::string rng_engine_test(bool pass)
{
    return pass ? "Passed" : "Failed";
}

template <typename RNGType>
inline std::string rng_engine_kat(const RNGType &)
{
    return "N/A";
}

template <typename ResultType, typename T, std::size_t K, std::size_t Rounds>
inline std::string rng_engine_kat(
    vsmc::PhiloxEngine<ResultType, T, K, Rounds> &rng)
{
    std::string filename("rng_engine_kat_Philox");
    filename += std::to_string(K) + "x";
    filename += std::to_string(std::numeric_limits<T>::digits) + ".txt";
    std::ifstream kat(filename);
    vsmc::Vector<T> k;
    k.reserve(1000);
    while (kat) {
        T x;
        kat >> x;
        k.push_back(x);
    }
    kat.close();

    vsmc::Vector<ResultType> r(256);
    rng(256, r.data());

    return std::memcmp(k.data(), r.data(), sizeof(ResultType) * 256) == 0 ?
        "Passed" :
        "Failed";
}

template <typename ResultType, typename T, std::size_t K, std::size_t Rounds>
inline std::string rng_engine_kat(
    vsmc::ThreefryEngine<ResultType, T, K, Rounds> &rng)
{
    std::string filename("rng_engine_kat_Threefry");
    filename += std::to_string(K) + "x";
    filename += std::to_string(std::numeric_limits<T>::digits) + ".txt";
    std::ifstream kat(filename);

    vsmc::Vector<T> k;
    k.reserve(1000);
    while (kat) {
        T x;
        kat >> x;
        k.push_back(x);
    }
    kat.close();

    vsmc::Vector<ResultType> r(256);
    rng(256, r.data());

    return std::memcmp(k.data(), r.data(), sizeof(ResultType) * 256) == 0 ?
        "Passed" :
        "Failed";
}

#if VSMC_HAS_AES_NI

template <typename ResultType, std::size_t Blocks>
inline std::string rng_engine_kat(vsmc::AES128Engine<ResultType, Blocks> &rng)
{
    std::ifstream kat("rng_engine_kat_AES128.txt");
    vsmc::Vector<std::uint64_t> k;
    k.reserve(1000);
    while (kat) {
        std::uint64_t x;
        kat >> x;
        k.push_back(x);
    }
    kat.close();

    vsmc::Vector<ResultType> r(256);
    rng(256, r.data());

    return std::memcmp(k.data(), r.data(), sizeof(ResultType) * 256) == 0 ?
        "Passed" :
        "Failed";
}

template <typename ResultType, std::size_t Blocks>
inline std::string rng_engine_kat(vsmc::AES192Engine<ResultType, Blocks> &rng)
{
    std::ifstream kat("rng_engine_kat_AES192.txt");
    vsmc::Vector<std::uint64_t> k;
    k.reserve(1000);
    while (kat) {
        std::uint64_t x;
        kat >> x;
        k.push_back(x);
    }
    kat.close();

    vsmc::Vector<ResultType> r(256);
    rng(256, r.data());

    return std::memcmp(k.data(), r.data(), sizeof(ResultType) * 256) == 0 ?
        "Passed" :
        "Failed";
}

template <typename ResultType, std::size_t Blocks>
inline std::string rng_engine_kat(vsmc::AES256Engine<ResultType, Blocks> &rng)
{
    std::ifstream kat("rng_engine_kat_AES256.txt");
    vsmc::Vector<std::uint64_t> k;
    k.reserve(1000);
    while (kat) {
        std::uint64_t x;
        kat >> x;
        k.push_back(x);
    }
    kat.close();

    vsmc::Vector<ResultType> r(256);
    rng(256, r.data());

    return std::memcmp(k.data(), r.data(), sizeof(ResultType) * 256) == 0 ?
        "Passed" :
        "Failed";
}

template <typename ResultType, std::size_t Rounds, std::size_t Blocks>
inline std::string rng_engine_kat(
    vsmc::ARSEngine<ResultType, Rounds, Blocks> &rng)
{
    std::ifstream kat("rng_engine_kat_ARS.txt");
    vsmc::Vector<std::uint64_t> k;
    k.reserve(1000);
    while (kat) {
        std::uint64_t x;
        kat >> x;
        k.push_back(x);
    }
    kat.close();

    vsmc::Vector<ResultType> r(256);
    rng(256, r.data());

    return std::memcmp(k.data(), r.data(), sizeof(ResultType) * 256) == 0 ?
        "Passed" :
        "Failed";
}

#endif // VSMC_HAS_AES_NI

template <typename RNGType>
inline void rng_engine(std::size_t N, std::size_t M, int nwid, int swid,
    int twid, const std::string &name)
{
    RNGType rng;
    RNGType rng1;
    RNGType rng2;
    vsmc::Vector<typename RNGType::result_type> r1;
    vsmc::Vector<typename RNGType::result_type> r2;
    r1.reserve(N);
    r2.reserve(N);
    vsmc::StopWatch watch1;
    vsmc::StopWatch watch2;
    std::string kat = rng_engine_kat(rng);

    bool batch = true;
    bool io = true;
    bool discard = true;

    std::uniform_int_distribution<std::size_t> runif(N / 2, N);
    std::size_t num = 0;
    for (std::size_t i = 0; i != M; ++i) {
        std::size_t K = runif(rng);
        num += K;
        r1.resize(K);
        r2.resize(K);

        watch1.start();
        for (std::size_t j = 0; j != K; ++j)
            r1[j] = rng1();
        watch1.stop();

        watch2.start();
        vsmc::rng_rand(rng2, K, r2.data());
        watch2.stop();
        batch = batch && r1 == r2;

        std::stringstream ss;
        ss << rng;
        vsmc::rng_rand(rng, K, r1.data());
        ss >> rng;
        vsmc::rng_rand(rng, K, r2.data());
        io = io && r1 == r2;

        rng1.discard(static_cast<unsigned>(K));
        typename RNGType::result_type next = rng1();
        for (std::size_t j = 0; j != K; ++j)
            rng2();
        bool find = false;
        for (std::size_t j = 0; j != 2; ++j)
            find = find || rng2() == next;
        discard = discard && find;
    }

    double n1 = watch1.nanoseconds() / num;
    double n2 = watch2.nanoseconds() / num;
    double g1 = sizeof(typename RNGType::result_type) / n1;
    double g2 = sizeof(typename RNGType::result_type) / n2;

    std::cout << std::left << std::setw(nwid) << name;
    std::cout << std::right << std::setw(swid) << sizeof(RNGType);
    std::cout << std::right << std::setw(swid) << alignof(RNGType);
    std::cout << std::right << std::setw(twid) << std::fixed << n1;
    std::cout << std::right << std::setw(twid) << std::fixed << n2;
    std::cout << std::right << std::setw(twid) << std::fixed << g1;
    std::cout << std::right << std::setw(twid) << std::fixed << g2;
    std::cout << std::right << std::setw(swid) << kat;
    std::cout << std::right << std::setw(swid) << rng_engine_test(batch);
    std::cout << std::right << std::setw(swid) << rng_engine_test(io);
    std::cout << std::right << std::setw(swid) << rng_engine_test(discard);
    std::cout << std::endl;
}

inline void rng_engine(std::size_t N, std::size_t M)
{
    const int nwid = 20;
    const int swid = 10;
    const int twid = 15;
    const std::size_t lwid = nwid + swid * 6 + twid * 4;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::left << std::setw(nwid) << "RNGType";
    std::cout << std::right << std::setw(swid) << "Size";
    std::cout << std::right << std::setw(swid) << "Align";
    std::cout << std::right << std::setw(twid) << "ns (Loop)";
    std::cout << std::right << std::setw(twid) << "ns (Batch)";
    std::cout << std::right << std::setw(twid) << "GB/s (Loop)";
    std::cout << std::right << std::setw(twid) << "GB/s (Batch)";
    std::cout << std::right << std::setw(swid) << "KAT";
    std::cout << std::right << std::setw(swid) << "Batch";
    std::cout << std::right << std::setw(swid) << "I/O";
    std::cout << std::right << std::setw(swid) << "Discard";
    std::cout << std::endl;
    std::cout << std::string(lwid, '-') << std::endl;

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#ifdef VSMC_RNG_DEFINE_MACRO_MKL
#undef VSMC_RNG_DEFINE_MACRO_MKL
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    rng_engine<RNGType>(N, M, nwid, swid, twid, #Name);

    VSMC_RNG_DEFINE_MACRO(vsmc::Threefry2x32, Threefry2x32, threefry2x32)
    VSMC_RNG_DEFINE_MACRO(vsmc::Threefry4x32, Threefry4x32, threefry4x32)
    VSMC_RNG_DEFINE_MACRO(vsmc::Threefry2x64, Threefry2x64, threefry2x64)
    VSMC_RNG_DEFINE_MACRO(vsmc::Threefry4x64, Threefry4x64, threefry4x64)

#include <vsmc/rng/internal/rng_define_macro.hpp>
#include <vsmc/rng/internal/rng_define_macro_mkl.hpp>

    std::cout << std::string(lwid, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_ENGINE_HPP

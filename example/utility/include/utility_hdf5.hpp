//============================================================================
// vSMC/example/utility/include/utility_hdf5.hpp
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

#ifndef VSMC_EXAMPLE_UTILITY_HDF5_HPP
#define VSMC_EXAMPLE_UTILITY_HDF5_HPP

#include <vsmc/utility/hdf5.hpp>
#include <vsmc/utility/stop_watch.hpp>

#define VSMC_HDF5_TEST(orig, load)                                            \
    hdf5_test<orig, load>(nrow, ncol, #orig, #load);

template <typename OrigType, typename LoadType>
inline bool hdf5_check(
    const vsmc::Vector<OrigType> &orig, const vsmc::Vector<LoadType> &load)
{
    for (std::size_t i = 0; i != orig.size(); ++i)
        if (!vsmc::internal::is_equal(static_cast<LoadType>(orig[i]), load[i]))
            return false;
    return true;
}

template <typename OrigType, typename LoadType>
inline void hdf5_test(std::size_t nrow, std::size_t ncol,
    const std::string &orig_type, const std::string &load_type)
{
    std::cout << std::setw(20) << std::left << orig_type << std::setw(20)
              << std::left << load_type;

    const std::string filename("hdf5.h5");
    const std::size_t n = nrow * ncol;

    vsmc::Vector<OrigType> orig(n);
    for (std::size_t i = 0; i != n; ++i)
        orig[i] = static_cast<OrigType>(i);
    vsmc::StopWatch watch_store;
    watch_store.start();
    vsmc::hdf5store(filename);
    vsmc::hdf5store(filename, "data", true);
    vsmc::hdf5store(n, orig.begin(), filename, "data/x1", true);
    vsmc::hdf5store(n, orig.data(), filename, "data/x2", true);
    vsmc::hdf5store(
        vsmc::RowMajor, nrow, ncol, orig.begin(), filename, "data/m1", true);
    vsmc::hdf5store(
        vsmc::RowMajor, nrow, ncol, orig.data(), filename, "data/m2", true);
    vsmc::hdf5store(
        vsmc::ColMajor, nrow, ncol, orig.begin(), filename, "data/m3", true);
    vsmc::hdf5store(
        vsmc::ColMajor, nrow, ncol, orig.data(), filename, "data/m4", true);
    watch_store.stop();

    bool passed = true;
    vsmc::Vector<LoadType> load(n);
    vsmc::StopWatch watch_load;
    watch_load.start();
    passed = passed && n == vsmc::hdf5load_size(filename, "data/x1");
    passed = passed && n == vsmc::hdf5load_size(filename, "data/x2");
    passed = passed && n == vsmc::hdf5load_size(filename, "data/m1");
    passed = passed && n == vsmc::hdf5load_size(filename, "data/m2");
    passed = passed && n == vsmc::hdf5load_size(filename, "data/m3");
    passed = passed && n == vsmc::hdf5load_size(filename, "data/m4");
    vsmc::hdf5load(filename, "data/x1", load.begin());
    passed = passed && hdf5_check(orig, load);
    vsmc::hdf5load(filename, "data/x2", load.data());
    passed = passed && hdf5_check(orig, load);
    vsmc::hdf5load(filename, "data/m1", load.begin());
    passed = passed && hdf5_check(orig, load);
    vsmc::hdf5load(filename, "data/m2", load.data());
    passed = passed && hdf5_check(orig, load);
    vsmc::hdf5load(filename, "data/m3", load.begin());
    passed = passed && hdf5_check(orig, load);
    vsmc::hdf5load(filename, "data/m4", load.data());
    passed = passed && hdf5_check(orig, load);
    watch_load.stop();

    std::cout << std::setw(20) << std::right << std::fixed
              << watch_store.milliseconds() << std::setw(20) << std::right
              << std::fixed << watch_load.milliseconds() << std::setw(20)
              << std::right << (passed ? "Passed" : "Failed") << std::endl;
    std::cout << std::string(100, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_UTILITY_HDF5_HPP

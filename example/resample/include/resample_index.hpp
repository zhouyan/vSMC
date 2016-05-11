//============================================================================
// vSMC/example/resample/include/resample_index.hpp
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

#ifndef VSMC_EXAMPLE_RESAMPLE_INDEX_HPP
#define VSMC_EXAMPLE_RESAMPLE_INDEX_HPP

#include "resample_test.hpp"

template <vsmc::MatrixLayout Layout>
inline ResampleState<Layout> resample_index_fixed(
    const vsmc::Vector<std::size_t> &size,
    const vsmc::Vector<vsmc::Vector<int>> &value,
    const vsmc::Vector<vsmc::Vector<std::size_t>> &index)
{
    const std::size_t dim = size.size() - 1;
    const std::size_t N = size[dim];
    ResampleState<Layout> state(0);
    state.resize(N, dim);

    vsmc::StopWatch watch;
    watch.start();
    for (std::size_t d = 0; d != dim; ++d) {
        const auto &val = value[d];
        const auto &idx = index[d];
        for (std::size_t i = 0; i != N; ++i)
            state(i, d) = val[i];
        if (Layout == vsmc::RowMajor)
            for (std::size_t i = 0; i != N; ++i)
                for (std::size_t j = 0; j != d + 1; ++j)
                    state(i, j) = state(idx[i], j);
        if (Layout == vsmc::ColMajor)
            for (std::size_t j = 0; j != d + 1; ++j)
                for (std::size_t i = 0; i != N; ++i)
                    state(i, j) = state(idx[i], j);
    }
    watch.stop();
    std::cout << std::setw(60) << std::left << "Time (ms) using direct copy"
              << std::setw(20) << std::right << std::fixed
              << watch.milliseconds() << std::endl;

    return state;
}

template <vsmc::MatrixLayout Layout>
inline ResampleState<Layout> resample_index_rands(
    const vsmc::Vector<std::size_t> &size,
    const vsmc::Vector<vsmc::Vector<int>> &value,
    const vsmc::Vector<vsmc::Vector<std::size_t>> &index)
{
    const std::size_t dim = size.size() - 1;
    ResampleState<Layout> state(size[0]);
    ResampleState<Layout> tmp(0);
    state.resize(size[0], 1);

    vsmc::StopWatch watch;
    watch.start();
    for (std::size_t d = 0; d != dim; ++d) {
        const auto &val = value[d];
        const auto &idx = index[d];
        state.resize_dim(d + 1);
        for (std::size_t i = 0; i != size[d]; ++i)
            state(i, d) = val[i];
        tmp.resize(size[d + 1], d + 1);
        if (Layout == vsmc::RowMajor)
            for (std::size_t i = 0; i != size[d + 1]; ++i)
                for (std::size_t j = 0; j != d + 1; ++j)
                    tmp(i, j) = state(idx[i], j);
        if (Layout == vsmc::ColMajor)
            for (std::size_t j = 0; j != d + 1; ++j)
                for (std::size_t i = 0; i != size[d + 1]; ++i)
                    tmp(i, j) = state(idx[i], j);
        state = std::move(tmp);
    }
    watch.stop();
    std::cout << std::setw(60) << std::left << "Time (ms) using direct copy"
              << std::setw(20) << std::right << std::fixed
              << watch.milliseconds() << std::endl;

    return state;
}

template <vsmc::MatrixLayout Layout>
inline ResampleState<Layout> resample_index_trace(
    const vsmc::Vector<std::size_t> &size,
    const vsmc::Vector<vsmc::Vector<int>> &value,
    const vsmc::Vector<vsmc::Vector<std::size_t>> &index)
{
    const std::size_t dim = size.size() - 1;
    const std::size_t N = size[dim];
    ResampleState<Layout> state(0);
    ResampleState<Layout> idxmat(0);
    state.resize(N, dim);
    idxmat.resize(N, dim);

    vsmc::StopWatch watch;
    watch.start();
    vsmc::ResampleIndex<int> resample_index;
    for (std::size_t d = 0; d != dim; ++d)
        resample_index.push_back(index[d].size(), index[d].begin());
    resample_index.read_index_matrix(Layout, idxmat.data());
    for (std::size_t j = 0; j != dim; ++j) {
        auto val = value[j].data();
        for (std::size_t i = 0; i != N; ++i)
            state(i, j) = val[idxmat(i, j)];
    }
    watch.stop();
    std::cout << std::setw(60) << std::left << "Time (ms) using index trace"
              << std::setw(20) << std::right << std::fixed
              << watch.milliseconds() << std::endl;

    return state;
}

template <vsmc::MatrixLayout Layout1, vsmc::MatrixLayout Layout2>
inline bool resample_index_check(
    const ResampleState<Layout1> &state1, const ResampleState<Layout2> &state2)
{
    if (state1.size() != state2.size())
        return false;
    if (state1.dim() != state2.dim())
        return false;

    const std::size_t N = state1.size();
    const std::size_t dim = state1.dim();
    for (std::size_t i = 0; i != N; ++i)
        for (std::size_t j = 0; j != dim; ++j)
            if (state1(i, j) != state2(i, j))
                return false;

    return true;
}

template <vsmc::MatrixLayout Layout1, vsmc::MatrixLayout Layout2>
inline void resample_index_test(std::size_t N, std::size_t dim, bool fixed)
{
    std::cout << std::string(80, '=') << std::endl;
    std::cout << std::setw(60) << std::left << "Sample size" << std::setw(20)
              << std::right << std::fixed << (fixed ? "Fixed" : "Random")
              << std::endl;
    std::cout << std::setw(60) << std::left << "Matrix layout of trace back"
              << std::setw(20) << std::right << std::fixed
              << (Layout1 == vsmc::RowMajor ? "RowMajor" : "ColMajor")
              << std::endl;
    std::cout << std::setw(60) << std::left << "Matrix layout of direct copy"
              << std::setw(20) << std::right << std::fixed
              << (Layout2 == vsmc::RowMajor ? "RowMajor" : "ColMajor")
              << std::endl;
    std::cout << std::string(80, '-') << std::endl;

    vsmc::RNG rng;
    auto size = resample_size(rng, N, dim, fixed);
    auto value = resample_value(rng, size);
    auto weight = resample_weight(rng, size);
    auto index = resample_index(rng, size, weight);
    bool passed =
        resample_index_check(resample_index_trace<Layout1>(size, value, index),
            fixed ? resample_index_fixed<Layout2>(size, value, index) :
                    resample_index_rands<Layout2>(size, value, index));

    std::cout << std::setw(60) << std::left << "Test result" << std::setw(20)
              << std::right << std::fixed << (passed ? "Passed" : "Failed")
              << std::endl;
    std::cout << std::string(80, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_RESAMPLE_INDEX_HPP

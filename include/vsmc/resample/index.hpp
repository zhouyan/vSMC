//============================================================================
// vSMC/include/vsmc/resample/index.hpp
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

#ifndef VSMC_RESAMPLE_INDEX_HPP
#define VSMC_RESAMPLE_INDEX_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc
{

/// \brief Record and trace resample index
/// \ingroup Resample
template <typename IntType = std::size_t>
class ResampleIndex
{
    public:
    using index_type = IntType;

    /// \brief Number of iterations recorded
    std::size_t iter_size() const { return iter_size_; }

    /// \brief The sample size of the last iteration
    std::size_t size() const { return index_.back().size(); }

    /// \brief The sample size of a given iteration
    std::size_t size(std::size_t iter) const { return index_[iter].size(); }

    /// \brief Reset history
    void reset() { iter_size_ = 0; }

    /// \brief Release memory
    void clear() { index_.clear(); }

    /// \brief Push back history of one iteration without resampling
    void push_back(std::size_t N)
    {
        ++iter_size_;
        resize_identity(N);
        if (index_.size() < iter_size_)
            index_.push_back(identity_);
        else
            index_[iter_size_ - 1] = identity_;
    }

    /// \brief Push back history of one iteration with resampling index
    template <typename InputIter>
    void push_back(std::size_t N, InputIter first)
    {
        push_back(N);
        std::copy_n(first, N, index_[iter_size_ - 1].begin());
    }

    /// \brief Insert at the back of the history without resampling
    void insert(std::size_t N)
    {
        resize_identity(N);
        std::copy_n(identity_.begin(), N, index_[iter_size_ - 1].begin());
    }

    /// \brief Insert at the back of the history with resampling index
    template <typename InputIter>
    void insert(std::size_t N, InputIter first)
    {
        std::copy_n(first, N, index_[iter_size_ - 1].begin());
    }

    /// \brief Insert at a given iteration with resampling index
    template <typename InputIter>
    void insert(std::size_t N, std::size_t iter, InputIter first)
    {
        std::copy_n(first, N, index_[iter].begin());
    }

    /// \brief Get the index given the particle ID and iteration number,
    /// starting with zero.
    index_type index(size_type id, std::size_t iter) const
    {
        std::size_t iter_current = iter_size_ - 1;
        index_type idx = index_.back()[id];
        while (iter_current != iter) {
            --iter_current;
            idx = index_[iter_current][idx];
        }

        return idx;
    }

    private:
    std::size_t iter_size_;
    Vector<index_type> identity_;
    Vector<Vector<index_type>> index_;

    resize_identity(std::size_t N)
    {
        std::size_t n = identity_.size();
        identity_.resize(N);
        if (n < N)
            for (std::size_t i = n; i != N; ++i)
                identity_[i] = static_cast<index_type>(i);
    }
}; // class ResampleIndex

} // namespace vsmc

#endif // VSMC_RESAMPLE_INDEX_HPP

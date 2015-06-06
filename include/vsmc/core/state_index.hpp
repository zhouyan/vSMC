//============================================================================
// vSMC/include/vsmc/core/state_index.hpp
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

#ifndef VSMC_CORE_STATE_INDEX_HPP
#define VSMC_CORE_STATE_INDEX_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_RUNTIME_ASSERT_CORE_STATE_INDEX_ITER(test, func)                 \
    VSMC_RUNTIME_ASSERT(                                                      \
        (test), "**StateIndex::" #func "ITERATION NUMBER OUT OF RANGE")

namespace vsmc
{

/// \brief Record and access state index
/// \ingroup Core
class StateIndex
{
    public:
    StateIndex(std::size_t N) : size_(N), identity_(N)
    {
        for (std::size_t i = 0; i != N; ++i)
            identity_[i] = i;
    }

    std::size_t size() const { return size_; }

    /// \brief Number of iterations recorded
    std::size_t iter_size() const { return iter_size_; }

    /// \brief Reset history
    void reset() { iter_size_ = 0; }

    /// \brief Release memory
    void clear() { index_.clear(); }

    void push_back()
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_INDEX_ITER(
            (index_.size() >= iter_size_), push_back);

        ++iter_size_;
        if (index_.size() < iter_size_)
            index_.push_back(identity_);
        else
            index_[iter_size_ - 1] = identity_;
    }

    template <typename InputIter>
    void push_back(InputIter first)
    {
        push_back();
        std::copy_n(first, size_, index_[iter_size_ - 1].data());
    }

    void insert()
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_INDEX_ITER(
            (iter_size_ > 0 && index_.size() >= iter_size_), insert);

        std::copy_n(identity_.data(), size_, index_[iter_size_ - 1].data());
    }

    template <typename InputIter>
    void insert(InputIter first)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_INDEX_ITER(
            (iter_size_ > 0 && index_.size() >= iter_size_), insert);

        std::copy_n(first, size_, index_[iter_size_ - 1].begin());
    }

    template <typename InputIter>
    void insert(std::size_t iter, InputIter first)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_INDEX_ITER(
            (iter_size_ > iter && index_.size() >= iter_size_), insert);

        std::copy_n(first, size_, index_[iter].begin());
    }

    /// \brief Get the index given the particle ID and iteration number,
    /// starting with zero.
    ///
    /// \details
    /// The index is traced back using the history. The cost is O(iter_size() -
    /// iter)
    std::size_t index(std::size_t id, std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_INDEX_ITER(
            (iter_size_ > iter && index_.size() >= iter_size_), index);

        std::size_t iter_current = iter_size_ - 1;
        std::size_t idx = index_.back()[id];
        while (iter_current != iter) {
            --iter_current;
            idx = index_[iter_current][idx];
        }

        return idx;
    }

    template <MatrixOrder Order>
    Vector<std::size_t> index_matrix() const
    {
        return index_matrix_dispatch(
            std::integral_constant<MatrixOrder, Order>());
    }

    private:
    std::size_t size_;
    std::size_t iter_size_;
    Vector<std::size_t> identity_;
    Vector<Vector<std::size_t>> index_;

    Vector<std::size_t> index_matrix_dispatch(
        std::integral_constant<MatrixOrder, RowMajor>) const
    {
        Vector<std::size_t> idxmat(size_ * iter_size_);
        if (size_ * iter_size_ == 0)
            return idxmat;

        std::size_t *back = idxmat.data() + iter_size_ - 1;
        for (std::size_t i = 0; i != size_; ++i, back += iter_size_)
            *back = index_[iter_size_ - 1][i];
        if (iter_size_ == 1)
            return idxmat;

        for (std::size_t iter = iter_size_ - 1; iter != 0; --iter) {
            const std::size_t *idx = index_[iter - 1].data();
            const std::size_t *last = idxmat.data() + iter;
            std::size_t *next = idxmat.data() + iter - 1;
            for (std::size_t i = 0; i != size_; ++i) {
                *next = idx[*last];
                last += iter_size_;
                next += iter_size_;
            }
        }

        return idxmat;
    }

    Vector<std::size_t> index_matrix_dispatch(
        std::integral_constant<MatrixOrder, ColMajor>) const
    {
        Vector<std::size_t> idxmat(size_ * iter_size_);
        if (size_ * iter_size_ == 0)
            return idxmat;

        std::size_t *back = idxmat.data() + size_ * (iter_size_ - 1);
        for (std::size_t i = 0; i != size_; ++i)
            back[i] = index_[iter_size_ - 1][i];
        if (iter_size_ == 1)
            return idxmat;

        for (std::size_t iter = iter_size_ - 1; iter != 0; --iter) {
            const std::size_t *idx = index_[iter - 1].data();
            const std::size_t *last = idxmat.data() + size_ * iter;
            std::size_t *next = idxmat.data() + size_ * (iter - 1);
            for (std::size_t i = 0; i != size_; ++i)
                next[i] = idx[last[i]];
        }

        return idxmat;
    }
}; // class StateIndex

} // namespace vsmc

#endif // VSMC_CORE_STATE_INDEX_HPP

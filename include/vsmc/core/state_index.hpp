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
    typedef std::size_t size_type;

    StateIndex(size_type N) : size_(N), identity_(N)
    {
        for (size_type i = 0; i != N; ++i)
            identity_[i] = i;
    }

    size_type size() const { return size_; }

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
        VSMC_RUNTIME_ASSERT_CORE_STATE_INDEX_ITER((iter_size_ > 0), insert);

        std::copy_n(identity_.data(), size_, index_[iter_size_ - 1].data());
    }

    template <typename InputIter>
    void insert(InputIter first)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_INDEX_ITER((iter_size_ > 0), insert);

        std::copy_n(first, size_, index_[iter_size_ - 1].begin());
    }

    template <typename InputIter>
    void insert(std::size_t iter, InputIter first)
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_INDEX_ITER((iter_size_ > iter), insert);

        std::copy_n(first, size_, index_[iter].begin());
    }

    /// \brief Get the index given the particle ID and iteration number,
    /// starting with zero.
    ///
    /// \details
    /// The index is traced back using the history. The cost is O(iter_size() -
    /// iter)
    size_type index(size_type id, std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_CORE_STATE_INDEX_ITER((iter_size_ > iter), index);

        std::size_t iter_current = iter_size_ - 1;
        size_type idx = index_.back()[id];
        while (iter_current != iter) {
            --iter_current;
            idx = index_[iter_current][idx];
        }

        return idx;
    }

    /// \brief Read the index matrix traced back using the recorded the history
    ///
    /// \details
    /// The index matrix is considered to be such that each column corresponds
    /// to an iteration.
    template <MatrixOrder Order, typename OutputIter>
    void read_index_matrix(OutputIter first)
    {
        Vector<Vector<size_type>> idxmat(
            index_matrix(std::integral_constant<MatrixOrder, Order>()));

        for (std::size_t i = 0; i != idxmat.size(); ++i)
            first = std::copy(idxmat[i].begin(), idxmat[i].end(), first);
    }

    private:
    size_type size_;
    std::size_t iter_size_;
    Vector<size_type> identity_;
    Vector<Vector<size_type>> index_;

    Vector<Vector<size_type>> index_matrix(
        std::integral_constant<MatrixOrder, RowMajor>) const
    {
        Vector<Vector<size_type>> idxmat(size_);
        for (auto &v : idxmat)
            v.resize(iter_size_);
        if (iter_size_ == 0)
            return idxmat;

        for (size_type i = 0; i != size_; ++i)
            idxmat[i].back() = index_.back()[i];
        if (iter_size_ == 1)
            return idxmat;

        for (std::size_t iter = iter_size_ - 1; iter != 0; --iter) {
            std::size_t iter_next = iter - 1;
            for (std::size_t i = 0; i != size_; ++i)
                idxmat[i][iter_next] = index_[iter_next][idxmat[i][iter]];
        }

        return idxmat;
    }

    Vector<Vector<size_type>> index_matrix(
        std::integral_constant<MatrixOrder, ColMajor>) const
    {
        if (iter_size_ <= 1)
            return index_;

        Vector<Vector<size_type>> idxmat(iter_size_);
        for (auto &v : idxmat)
            v.resize(size_);

        idxmat.back() = index_.back();
        for (std::size_t iter = iter_size_ - 1; iter != 0; --iter) {
            std::size_t iter_next = iter - 1;
            for (std::size_t i = 0; i != size_; ++i)
                idxmat[iter_next][i] = index_[iter_next][idxmat[iter][i]];
        }

        return idxmat;
    }
}; // class StateIndex

} // namespace vsmc

#endif // VSMC_CORE_STATE_INDEX_HPP

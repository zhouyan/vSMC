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

#define VSMC_RUNTIME_ASSERT_RESAMPLE_INDEX_ITER(test, func)                   \
    VSMC_RUNTIME_ASSERT(                                                      \
        (test), "**StateIndex::" #func "ITERATION NUMBER OUT OF RANGE")

namespace vsmc
{

/// \brief Record and trace resample index
/// \ingroup Resample
template <typename IntType = std::size_t>
class ResampleIndex
{
    public:
    using size_type = std::size_t;
    using index_type = IntType;

    ResampleIndex(size_type N) : size_(N), identity_(N)
    {
        for (size_type i = 0; i != N; ++i)
            identity_[i] = static_cast<index_type>(i);
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
        VSMC_RUNTIME_ASSERT_RESAMPLE_INDEX_ITER(
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
        VSMC_RUNTIME_ASSERT_RESAMPLE_INDEX_ITER(
            (iter_size_ > 0 && index_.size() >= iter_size_), insert);

        std::copy_n(identity_.data(), size_, index_[iter_size_ - 1].data());
    }

    template <typename InputIter>
    void insert(InputIter first)
    {
        VSMC_RUNTIME_ASSERT_RESAMPLE_INDEX_ITER(
            (iter_size_ > 0 && index_.size() >= iter_size_), insert);

        std::copy_n(first, size_, index_[iter_size_ - 1].begin());
    }

    template <typename InputIter>
    void insert(std::size_t iter, InputIter first)
    {
        VSMC_RUNTIME_ASSERT_RESAMPLE_INDEX_ITER(
            (iter_size_ > iter && index_.size() >= iter_size_), insert);

        std::copy_n(first, size_, index_[iter].begin());
    }

    /// \brief Get the index given the particle ID and iteration number,
    /// starting with zero.
    ///
    /// \details
    /// The index is traced back using the history. The cost is O(iter_size() -
    /// iter)
    index_type index(size_type id, std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_RESAMPLE_INDEX_ITER(
            (iter_size_ > iter && index_.size() >= iter_size_), index);

        std::size_t iter_current = iter_size_ - 1;
        index_type idx = index_.back()[id];
        while (iter_current != iter) {
            --iter_current;
            idx = index_[iter_current][idx];
        }

        return idx;
    }

    Vector<index_type> index_matrix(MatrixLayout layout) const
    {
        Vector<index_type> idxmat(size_ * iter_size_);

        if (size_ * iter_size_ == 0)
            return idxmat;

        if (layout == RowMajor) {
            index_type *back = idxmat.data() + iter_size_ - 1;
            for (size_type i = 0; i != size_; ++i, back += iter_size_)
                *back = index_[iter_size_ - 1][i];
            if (iter_size_ == 1)
                return idxmat;

            for (std::size_t iter = iter_size_ - 1; iter != 0; --iter) {
                const index_type *idx = index_[iter - 1].data();
                const index_type *last = idxmat.data() + iter;
                index_type *next = idxmat.data() + iter - 1;
                for (size_type i = 0; i != size_; ++i) {
                    *next = idx[static_cast<size_type>(*last)];
                    last += iter_size_;
                    next += iter_size_;
                }
            }
        }

        if (layout == ColMajor) {
            index_type *back = idxmat.data() + size_ * (iter_size_ - 1);
            for (size_type i = 0; i != size_; ++i)
                back[i] = index_[iter_size_ - 1][i];
            if (iter_size_ == 1)
                return idxmat;

            for (std::size_t iter = iter_size_ - 1; iter != 0; --iter) {
                const index_type *idx = index_[iter - 1].data();
                const index_type *last = idxmat.data() + size_ * iter;
                index_type *next = idxmat.data() + size_ * (iter - 1);
                for (size_type i = 0; i != size_; ++i)
                    next[i] = idx[static_cast<size_type>(last[i])];
            }
        }

        return idxmat;
    }

    template <typename OutputIter>
    void read_index_matrix(MatrixLayout layout, OutputIter first) const
    {
        auto idxmat = index_matrix(layout);
        std::copy(idxmat.begin(), idxmat.end(), first);
    }

    private:
    size_type size_;
    std::size_t iter_size_;
    Vector<index_type> identity_;
    Vector<Vector<index_type>> index_;
}; // class ResampleIndex

} // namespace vsmc

#endif // VSMC_RESAMPLE_INDEX_HPP

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

    std::size_t iter_size() const { return index_.size(); }

    size_type index(size_type id, std::size_t iter) const
    {
        std::size_t iter_current = index_.size() - 1;
        size_type idx = index_.back()[id];
        while (iter_current != iter) {
            --iter_current;
            idx = index_[iter_current][idx];
        }

        return idx;
    }

    Vector<Vector<size_type>> index() const
    {
        if (index_.size() <= 1)
            return index_;

        Vector<Vector<size_type>> idx(index_.size());
        for (auto &v : idx)
            v.resize(size_);

        idx.back() = index_.back();
        for (std::size_t iter = index_.size() - 1; iter != 0; --iter) {
            size_type *ct = idx[iter - 1].data();
            const size_type *cf = idx[iter].data();
            const size_type *id = index_[iter - 1].data();
            for (std::size_t i = 0; i != size_; ++i)
                ct[i] = id[cf[i]];
        }

        return idx;
    }

    void push_back() { index_.push_back(identity_); }

    template <typename InputIter>
    void push_back(InputIter first)
    {
        Vector<std::size_t> tmp(size_);
        std::copy_n(first, size_, tmp.begin());
        index_.push_back(std::move(tmp));
    }

    void reset()
    {
        std::copy(identity_.begin(), identity_.end(), index_.back().begin());
    }

    template <typename InputIter>
    void reset(InputIter first)
    {
        std::copy_n(first, size_, index_.back().begin());
    }

    template <typename InputIter>
    void reset(std::size_t iter, InputIter first)
    {
        std::copy_n(first, size_, index_[iter].begin());
    }

    void clear() { index_.clear(); }

    private:
    size_type size_;
    Vector<size_type> identity_;
    Vector<Vector<size_type>> index_;
}; // class StateIndex

} // namespace vsmc

#endif // VSMC_CORE_STATE_INDEX_HPP

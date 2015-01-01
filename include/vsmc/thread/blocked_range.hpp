//============================================================================
// vSMC/include/vsmc/thread/blocked_range.hpp
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

#ifndef VSMC_THREAD_BLOCKED_RANGE_HPP
#define VSMC_THREAD_BLOCKED_RANGE_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_RUNTIME_ASSERT_THREAD_BLOCKED_RANGE(begin, end, func) \
    VSMC_RUNTIME_ASSERT((begin < end), ("**"#func"** INVALID RANGE"))

namespace vsmc {

/// \brief Blocked range
/// \ingroup Thread
template <typename T>
class BlockedRange
{
    public :

    typedef T const_iterator;
    typedef std::size_t size_type;

    BlockedRange () : begin_(), end_(), grainsize_(1) {}

    BlockedRange (T begin, T end, size_type grainsize = 1) :
        begin_(begin), end_(end), grainsize_(grainsize)
    {VSMC_RUNTIME_ASSERT_THREAD_BLOCKED_RANGE(begin, end, BlockedRange);}

    template <typename Split>
    BlockedRange (BlockedRange<T> &other, Split) :
        begin_(other.begin_), end_(other.end_), grainsize_(other.grainsize_)
    {
        if (is_divisible()) {
            begin_ = begin_ + (end_ - begin_) / 2;
            other.end_ = begin_;
        } else {
            begin_ = end_;
        }
    }

    const_iterator begin () const {return begin_;}

    const_iterator end () const {return end_;}

    size_type size () const {return static_cast<size_type>(end_ - begin_);}

    size_type grainsize () const {return grainsize_;}

    bool empty () const {return !(begin_ < end_);}

    bool is_divisible () const {return grainsize_ < size();}

    private :

    const_iterator begin_;
    const_iterator end_;
    size_type grainsize_;
}; // class BlockedRange

} // namespace vsmc

#endif // VSMC_THREAD_BLOCKED_RANGE_HPP

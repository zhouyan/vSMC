//============================================================================
// include/vsmc/thread/blocked_range.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
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

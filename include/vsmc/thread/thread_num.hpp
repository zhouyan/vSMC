//============================================================================
// include/vsmc/thread/thread_num.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_THREAD_THREAD_NUM_HPP
#define VSMC_THREAD_THREAD_NUM_HPP

#include <vsmc/internal/common.hpp>
#include <cstdlib>
#include <thread>
#include <vector>

namespace vsmc {

/// \brief Number of threads used by algorithms
/// \ingroup Thread
class ThreadNum
{
    public :

    static ThreadNum &instance ()
    {
        static ThreadNum num;

        return num;
    }

    std::size_t thread_num () const {return thread_num_;}

    /// \brief Set a new number of threads, return the old number
    std::size_t thread_num (std::size_t num)
    {
        std::size_t old_num = thread_num_;
        thread_num_ = num;

        return old_num;
    }

    template <typename Range>
    std::vector<Range> partition (const Range &range) const
    {
        typedef typename Range::const_iterator size_type;
        size_type N = range.end() - range.begin();
        size_type tn = static_cast<size_type>(thread_num());
        size_type block_size =  0;

        if (N < tn)
            block_size = 1;
        else if (N % tn)
            block_size = N / tn + 1;
        else
            block_size = N / tn;

        std::vector<Range> range_vec;
        range_vec.reserve(thread_num());
        size_type B = range.begin();
        while (N > 0) {
            size_type next = N < block_size ? N : block_size;
            range_vec.push_back(Range(B, B + next));
            B += next;
            N -= next;
        }

        return range_vec;
    }

    private :

    std::size_t thread_num_;

    ThreadNum () : thread_num_(
            static_cast<std::size_t>(1) >
            static_cast<std::size_t>(std::thread::hardware_concurrency()) ?
            static_cast<std::size_t>(1) :
            static_cast<std::size_t>(std::thread::hardware_concurrency()))
    {
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4996)
#endif
        const char *num_str = std::getenv("VSMC_THREAD_NUM");
#ifdef _MSC_VER
#pragma warning(pop)
#endif
        if (num_str) {
            int num = std::atoi(num_str);
            thread_num_ = num > 0 ? static_cast<std::size_t>(num) : 1;
        }
    }

    ThreadNum (const ThreadNum &) = delete;
    ThreadNum &operator= (const ThreadNum &) = delete;
}; // class ThreadInfo

} // namespace vsmc

#endif // VSMC_THREAD_THREAD_NUM_HPP

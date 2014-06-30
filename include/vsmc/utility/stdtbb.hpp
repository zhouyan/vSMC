//============================================================================
// vsmc/utility/stdtbb.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UTILITY_STDTBB_HPP
#define VSMC_UTILITY_STDTBB_HPP

#include <vsmc/internal/common.hpp>
#include <cstdlib>
#include <thread>
#include <vector>

#if VSMC_HAS_CXX11LIB_FUTURE
#include <future>
#endif

#define VSMC_RUNTIME_ASSERT_UTILITY_STDTBB_RANGE(begin, end, func) \
    VSMC_RUNTIME_ASSERT((begin < end), ("**"#func"** INVALID RANGE"))

namespace vsmc {

/// \brief Blocked range
/// \ingroup STDTBB
template <typename T>
class BlockedRange
{
    public :

    typedef T const_iterator;
    typedef std::size_t size_type;

    BlockedRange () : begin_(), end_(), grainsize_(1) {}

    BlockedRange (T begin, T end, size_type grainsize = 1) :
        begin_(begin), end_(end), grainsize_(grainsize)
    {VSMC_RUNTIME_ASSERT_UTILITY_STDTBB_RANGE(begin, end, BlockedRange);}

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

/// \brief C++11 Thread guard
/// \ingroup STDTBB
class ThreadGuard
{
#if VSMC_HAS_CXX11_DELETED_FUNCTIONS
    public :

    ThreadGuard (const ThreadGuard &) = delete;
    ThreadGuard &operator= (const ThreadGuard &) = delete;
#else
    private :

    ThreadGuard (const ThreadGuard &);
    ThreadGuard &operator= (const ThreadGuard &);
#endif

    public :

    ThreadGuard () VSMC_NOEXCEPT {}

    ThreadGuard (std::thread &&thr) VSMC_NOEXCEPT :
        thread_(cxx11::move(thr)) {}

    ThreadGuard (ThreadGuard &&other) VSMC_NOEXCEPT :
        thread_(cxx11::move(other.thread_)) {}

    ThreadGuard &operator= (ThreadGuard &&other) VSMC_NOEXCEPT
    {thread_ = cxx11::move(other.thread_); return *this;}

    ~ThreadGuard () VSMC_NOEXCEPT {if (thread_.joinable()) thread_.join();}

    private :

    std::thread thread_;
}; // class ThreadGuard

/// \brief C++11 Thread informations
/// \ingroup STDTBB
class ThreadInfo
{
    public :

    static ThreadInfo &instance ()
    {
        static ThreadInfo info;

        return info;
    }

    std::size_t thread_num () const {return thread_num_;}

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

    ThreadInfo () : thread_num_(
            static_cast<std::size_t>(1) >
            static_cast<std::size_t>(std::thread::hardware_concurrency()) ?
            static_cast<std::size_t>(1) :
            static_cast<std::size_t>(std::thread::hardware_concurrency()))
    {
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4996)
#endif
        const char *num_str = std::getenv("VSMC_STDTBB_NUM_THREADS");
#ifdef _MSC_VER
#pragma warning(pop)
#endif
        if (num_str) {
            int num = std::atoi(num_str);
            thread_num_ = num > 0 ? static_cast<std::size_t>(num) : 1;
        }
    }

    ThreadInfo (const ThreadInfo &);
    ThreadInfo &operator= (const ThreadInfo &);
}; // class ThreadInfo

/// \brief Parallel for using C++11 concurrency
/// \ingroup STDTBB
///
/// \details
/// Requirement: WorkType:
/// ~~~{.cpp}
/// WorkType work;
/// work(range);
/// ~~~
template <typename Range, typename WorkType>
inline void parallel_for (const Range &range, WorkType &&work)
{
    std::vector<Range> range_vec(ThreadInfo::instance().partition(range));
#if VSMC_HAS_CXX11LIB_FUTURE
    std::vector<std::future<void> > wg;
    wg.reserve(range_vec.size());
    for (std::size_t i = 0; i != range_vec.size(); ++i) {
        wg.push_back(std::async(std::launch::async,
                    std::forward<WorkType>(work), range_vec[i]));
    }
    for (std::size_t i = 0; i != wg.size(); ++i) wg[i].get();
#else
    // start parallelization
    {
        std::vector<ThreadGuard> tg;
        tg.reserve(range_vec.size());
        for (std::size_t i = 0; i != range_vec.size(); ++i) {
            tg.push_back(ThreadGuard(std::thread(std::forward<WorkType>(work),
                            range_vec[i])));
        }
    }
    // stop parallelization
#endif
}

/// \brief Parallel reduce using C++11 concurrency
/// \ingroup STDTBB
///
/// \details
/// Requirement: WorkType
/// ~~~{.cpp}
/// WorkType work;
/// work(range);
/// Work.join(other_work);
/// ~~~
template <typename Range, typename WorkType>
inline void parallel_reduce (const Range &range, WorkType &work)
{
    std::vector<Range> range_vec(ThreadInfo::instance().partition(range));
    std::vector<WorkType> work_vec(range_vec.size(), work);
#if VSMC_HAS_CXX11LIB_FUTURE
    std::vector<std::future<void> > wg;
    wg.reserve(range_vec.size());
    for (std::size_t i = 0; i != range_vec.size(); ++i) {
        wg.push_back(std::async(std::launch::async,
                    std::ref(work_vec[i]), range_vec[i]));
    }
    for (std::size_t i = 0; i != wg.size(); ++i) wg[i].get();
#else
    // start parallelization
    {
        std::vector<ThreadGuard> tg;
        tg.reserve(range_vec.size());
        for (std::size_t i = 0; i != range_vec.size(); ++i) {
            tg.push_back(ThreadGuard(std::thread(std::ref(work_vec[i]),
                            range_vec[i])));
        }
    }
    // stop parallelization
#endif
    for (std::size_t i = 0; i != work_vec.size(); ++i) work.join(work_vec[i]);
}

/// \brief Parallel accumulate using C++11 concurrency
/// \ingroup STDTBB
///
/// \details
/// Requirement: WorkType
/// ~~~{.cpp}
/// WorkType work;
/// work(range, res); // res: T reference type
/// ~~~
template <typename Range, typename T, typename WorkType>
inline T parallel_accumulate (const Range &range, WorkType &&work, T init)
{
    std::vector<Range> range_vec(ThreadInfo::instance().partition(range));
    std::vector<T> result(range_vec.size());
#if VSMC_HAS_CXX11LIB_FUTURE
    std::vector<std::future<void> > wg;
    wg.reserve(range_vec.size());
    for (std::size_t i = 0; i != range_vec.size(); ++i) {
        wg.push_back(std::async(std::launch::async,
                    std::forward<WorkType>(work), range_vec[i],
                    std::ref(result[i])));
    }
    for (std::size_t i = 0; i != wg.size(); ++i) wg[i].get();
#else
    // start parallelization
    {
        std::vector<ThreadGuard> tg;
        tg.reserve(range_vec.size());
        for (std::size_t i = 0; i != range_vec.size(); ++i) {
            tg.push_back(ThreadGuard(std::thread(std::forward<WorkType>(work),
                            range_vec[i], std::ref(result[i]))));
        }
    }
    // stop parallelization
#endif

    T acc(init);
    for (std::size_t i = 0; i != result.size(); ++i)
        acc += result[i];

    return acc;
}

/// \brief Parallel accumulate using C++11 concurrency
/// \ingroup STDTBB
///
/// \details
/// Requirement: WorkType
/// ~~~{.cpp}
/// WorkType work;
/// work(range, res); // res: T reference type
/// ~~~
template <typename Range, typename T, typename Bin, typename WorkType>
inline T parallel_accumulate (const Range &range, WorkType &&work,
        T init, Bin bin_op)
{
    std::vector<Range> range_vec(ThreadInfo::instance().partition(range));
    std::vector<T> result(range_vec.size());
#if VSMC_HAS_CXX11LIB_FUTURE
    std::vector<std::future<void> > wg;
    wg.reserve(range_vec.size());
    for (std::size_t i = 0; i != range_vec.size(); ++i) {
        wg.push_back(std::async(std::launch::async,
                    std::forward<WorkType>(work), range_vec[i],
                    std::ref(result[i])));
    }
    for (std::size_t i = 0; i != wg.size(); ++i) wg[i].get();
#else
    // start parallelization
    {
        std::vector<ThreadGuard> tg;
        tg.reserve(range_vec.size());
        for (std::size_t i = 0; i != range_vec.size(); ++i) {
            tg.push_back(ThreadGuard(std::thread(std::forward<WorkType>(work),
                            range_vec[i], std::ref(result[i]))));
        }
    }
    // stop parallelization
#endif

    T acc(init);
    for (std::size_t i = 0; i != result.size(); ++i)
        acc = bin_op(acc, result[i]);

    return acc;
}

} // namespace vsmc

#endif // VSMC_UTILITY_STDTBB_HPP

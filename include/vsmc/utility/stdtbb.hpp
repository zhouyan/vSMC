#ifndef VSMC_UTILITY_STDTBB_HPP
#define VSMC_UTILITY_STDTBB_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>

#include <vsmc/cxx11/functional.hpp>

#include <cstdlib>
#include <thread>
#include <utility>
#include <vector>

#if VSMC_HAS_CXX11LIB_FUTURE
#include <future>
#endif

namespace vsmc {

/// \brief Blocked range
/// \ingroup STDTBB
template <typename SizeType>
class BlockedRange
{
    public :

    typedef SizeType size_type;

    BlockedRange (size_type begin, size_type end) : begin_(begin), end_(end)
    {
        VSMC_RUNTIME_ASSERT_RANGE(begin, end, BlockedRange);
    }

    size_type begin () const
    {
        return begin_;
    }

    size_type end () const
    {
        return end_;
    }

    private :

    size_type begin_;
    size_type end_;
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

    ThreadGuard (ThreadGuard &&other) VSMC_NOEXCEPT :
        thread_(std::move(other.thread_)) {}

    ThreadGuard &operator= (ThreadGuard &&other) VSMC_NOEXCEPT
    {
        thread_ = std::move(other.thread_);

        return *this;
    }

    ThreadGuard (std::thread &&thr) VSMC_NOEXCEPT : thread_(std::move(thr)) {}

    ~ThreadGuard () VSMC_NOEXCEPT
    {
        if (thread_.joinable())
            thread_.join();
    }

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

    std::size_t thread_num () const
    {
        return thread_num_;
    }

    void thread_num (std::size_t num)
    {
        thread_num_ = num;
    }

    template <typename SizeType>
    std::vector<BlockedRange<SizeType> > partition (
            const BlockedRange<SizeType> &range) const
    {
        SizeType N = range.end() - range.begin();
        SizeType tn = static_cast<SizeType>(thread_num());
        SizeType block_size =  0;

        if (N < tn)
            block_size = 1;
        else if (N % tn)
            block_size = N / tn + 1;
        else
            block_size = N / tn;

        std::vector<BlockedRange<SizeType> > range_vec;
        range_vec.reserve(thread_num());
        SizeType B = range.begin();
        while (N > 0) {
            SizeType next = N < block_size ? N : block_size;
            range_vec.push_back(BlockedRange<SizeType>(B, B + next));
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
        const char *num_str = std::getenv("VSMC_STD_NUM_THREADS");
#ifdef _MSC_VER
#pragma warning(pop)
#endif
        if (num_str) {
            std::size_t num = std::atoi(num_str);
            if (num)
                thread_num_ = num;
        }
    }

    ThreadInfo (const ThreadInfo &);
    ThreadInfo &operator= (const ThreadInfo &);
}; // class ThreadInfo

/// \brief Parallel for using C++11 thread
/// \ingroup STDTBB
///
/// \details
/// Requirement: WorkType:
/// void work (const BlockedRange<size_type> &range);
template <typename SizeType, typename WorkType>
void parallel_for (const BlockedRange<SizeType> &range, WorkType &&work)
{
    std::vector<BlockedRange<SizeType> > range_vec(
            ThreadInfo::instance().partition(range));
#if VSMC_HAS_CXX11LIB_FUTURE
    std::vector<std::future<void> > wg;
    for (typename std::vector<BlockedRange<SizeType> >::iterator
            r = range_vec.begin(); r != range_vec.end(); ++r) {
        wg.push_back(std::async(std::launch::async,
                    std::forward<WorkType>(work), *r));
    }
    for (std::vector<std::future<void> >::iterator
            w = wg.begin(); w != wg.end(); ++w) {w->get();}
#else
    // start parallelization
    {
        std::vector<ThreadGuard> tg;
        for (typename std::vector<BlockedRange<SizeType> >::iterator
                r = range_vec.begin(); r != range_vec.end(); ++r) {
            tg.push_back(ThreadGuard(std::thread(std::forward<WorkType>(work),
                            *r)));
        }
    }
    // stop parallelization
#endif
}

/// \brief Parallel accumulate using C++11 thread
/// \ingroup STDTBB
///
/// \details
/// Requirement: WorkType:
/// void work (const BlockedRange<size_type> &range, T &res);
template <typename SizeType, typename T, typename WorkType>
T parallel_accumulate (const BlockedRange<SizeType> &range, WorkType &&work,
        T init)
{
    std::vector<BlockedRange<SizeType> > range_vec(
            ThreadInfo::instance().partition(range));
    std::vector<T> result(range_vec.size());
    std::size_t i = 0;
#if VSMC_HAS_CXX11LIB_FUTURE
    std::vector<std::future<void> > wg;
    for (typename std::vector<BlockedRange<SizeType> >::iterator
            r = range_vec.begin(); r != range_vec.end(); ++r, ++i) {
        wg.push_back(std::async(std::launch::async,
                    std::forward<WorkType>(work), *r, cxx11::ref(result[i])));
    }
    for (std::vector<std::future<void> >::iterator
            w = wg.begin(); w != wg.end(); ++w) {w->get();}
#else
    // start parallelization
    {
        std::vector<ThreadGuard> tg;
        for (typename std::vector<BlockedRange<SizeType> >::iterator
                r = range_vec.begin(); r != range_vec.end(); ++r, ++i) {
            tg.push_back(ThreadGuard(std::thread(std::forward<WorkType>(work),
                            *r, cxx11::ref(result[i]))));
        }
    }
    // stop parallelization
#endif

    T acc(init);
    for (std::size_t i = 0; i != result.size(); ++i)
        acc += result[i];

    return acc;
}

/// \brief Parallel accumulate using C++11 thread
/// \ingroup STDTBB
///
/// \details
/// Requirement: WorkType:
/// void work (const BlockedRange<size_type> &range, T &res);
template <typename SizeType, typename T, typename BinOp, typename WorkType>
T parallel_accumulate (const BlockedRange<SizeType> &range, WorkType &&work,
        T init, BinOp bin_op)
{
    std::vector<BlockedRange<SizeType> > range_vec(
            ThreadInfo::instance().partition(range));
    std::vector<T> result(range_vec.size());
    std::size_t i =0;
#if VSMC_HAS_CXX11LIB_FUTURE
    std::vector<std::future<void> > wg;
    for (typename std::vector<BlockedRange<SizeType> >::iterator
            r = range_vec.begin(); r != range_vec.end(); ++r, ++i) {
        wg.push_back(std::async(std::launch::async,
                    std::forward<WorkType>(work), *r, cxx11::ref(result[i])));
    }
    for (std::vector<std::future<void> >::iterator
            w = wg.begin(); w != wg.end(); ++w) {w->get();}
#else
    // start parallelization
    {
        std::vector<ThreadGuard> tg;
        for (typename std::vector<BlockedRange<SizeType> >::iterator
                r = range_vec.begin(); r != range_vec.end(); ++r, ++i) {
            tg.push_back(ThreadGuard(std::thread(std::forward<WorkType>(work),
                            *r, cxx11::ref(result[i]))));
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

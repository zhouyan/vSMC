#ifndef VSMC_UTILITY_STDTBB_HPP
#define VSMC_UTILITY_STDTBB_HPP

#include <algorithm>
#include <thread>
#include <vector>

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>

namespace vsmc { namespace thread {

/// \brief C++11 Thread guard
/// \ingroup Thread
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

/// \brief C++11 Thread manager
/// \ingroup Thread
class ThreadManager
{
    public :

    static ThreadManager &instance ()
    {
        static ThreadManager manager;

        return manager;
    }

    unsigned thread_num () const
    {
        return thread_num_;
    }

    void thread_num (unsigned num)
    {
        thread_num_ = num;
    }

    template <typename SizeType, typename OutputIter>
    unsigned partition (SizeType begin, SizeType end,
            OutputIter begin_iter, OutputIter end_iter) const
    {
        VSMC_RUNTIME_ASSERT((begin < end),
                "WRONG RANGE PASSED TO **ThreadManager::partition**");

        SizeType N = end - begin;
        SizeType block_size = N / static_cast<SizeType>(thread_num());
        if (block_size == 0)
            block_size = N;
        else if (N % static_cast<SizeType>(thread_num()))
            ++block_size;

        SizeType current = 0;
        unsigned num = 0;
        while (N > 0) {
            SizeType next_size = std::min VSMC_MINMAX_NO_EXPANSION (
                    N, block_size);
            *begin_iter = current;
            *end_iter = current = *begin_iter + next_size;
            ++begin_iter;
            ++end_iter;
            ++num;
            N -= next_size;
        }

        VSMC_RUNTIME_ASSERT((num <= thread_num()),
                "INVALID THREAD NUMBER **ThreadManager::partition**");

        return num;
    }

    private :

    unsigned thread_num_;

    ThreadManager () : thread_num_(std::max VSMC_MINMAX_NO_EXPANSION (1U,
             static_cast<unsigned>(std::thread::hardware_concurrency())))
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
            unsigned num = std::atoi(num_str);
            if (num)
                thread_num_ = num;
        }
    }

    ThreadManager (const ThreadManager &);
    ThreadManager &operator= (const ThreadManager &);
}; // class ThreadManager

/// \brief Blocked range
/// \ingroup Thread
template <typename SizeType>
class BlockedRange
{
    public :

    typedef SizeType size_type;

    BlockedRange (size_type begin, size_type end) : begin_(begin), end_(end) {}

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

/// \brief Parallel for using C++11 thread
/// \ingroup Thread
///
/// \details
/// Requirement: WorkType:
/// void work (const thread::BlockedRange<size_type> &range);
template <typename SizeType, typename WorkType>
void parallel_for (const BlockedRange<SizeType> &range, WorkType &&work)
{
    const ThreadManager &manager = ThreadManager::instance();
    unsigned thread_num = manager.thread_num();
    std::vector<SizeType> b(thread_num);
    std::vector<SizeType> e(thread_num);
    unsigned num = manager.partition(range.begin(), range.end(),
            b.begin(), e.begin());
    std::vector<ThreadGuard> tg;
    { // start parallelization
        for (unsigned i = 0; i != num; ++i) {
            tg.push_back(ThreadGuard(std::thread(std::forward<WorkType>(work),
                            BlockedRange<SizeType>(b[i], e[i]))));
        }
    } // end parallelization
}

/// \brief Parallel accumulate using C++11 thread
/// \ingroup Thread
///
/// \details
/// Requirement: WorkType:
/// void work (const thread::BlockedRange<size_type> &range, ResultType &res);
template <typename SizeType, typename WorkType, typename ResultType>
void parallel_accumulate (const BlockedRange<SizeType> &range, WorkType &&work,
        ResultType &res)
{
    const ThreadManager &manager = ThreadManager::instance();
    unsigned thread_num = manager.thread_num();
    std::vector<SizeType> b(thread_num);
    std::vector<SizeType> e(thread_num);
    std::vector<ResultType> result(thread_num);
    unsigned num = manager.partition(range.begin(), range.end(),
            b.begin(), e.begin());
    { // start parallelization
        std::vector<ThreadGuard> tg;
        for (unsigned i = 0; i != num; ++i) {
            tg.push_back(ThreadGuard(std::thread(std::forward<WorkType>(work),
                            BlockedRange<SizeType>(b[i], e[i]),
                            std::ref(result[i]))));
        }
    } // end parallelization
    for (unsigned i = 1; i != num; ++i)
        result[0] += result[i];
    res = result[0];
}

/// \brief Parallel accumulate using C++11 thread
/// \ingroup Thread
///
/// \details
/// Requirement: WorkType:
/// void work (const thread::BlockedRange<size_type> &range, ResultType &res);
template <typename SizeType, typename WorkType,
    typename ResultType, typename BinaryOpType>
void parallel_accumulate (const BlockedRange<SizeType> &range, WorkType &&work,
        ResultType &res, BinaryOpType bin_op)
{
    const ThreadManager &manager = ThreadManager::instance();
    unsigned thread_num = manager.thread_num();
    std::vector<SizeType> b(thread_num);
    std::vector<SizeType> e(thread_num);
    std::vector<ResultType> result(thread_num);
    unsigned num = manager.partition(range.begin(), range.end(),
            b.begin(), e.begin());
    { // start parallelization
        std::vector<ThreadGuard> tg;
        for (unsigned i = 0; i != num; ++i) {
            tg.push_back(ThreadGuard(std::thread(std::forward<WorkType>(work),
                            BlockedRange<SizeType>(b[i], e[i]),
                            std::ref(result[i]))));
        }
    } // end parallelization
    for (unsigned i = 1; i != num; ++i)
        result[0] = bin_op(result[0], result[i]);
    res = result[0];
}

} } // namespace vsmc::thread

#endif // VSMC_UTILITY_STDTBB_HPP

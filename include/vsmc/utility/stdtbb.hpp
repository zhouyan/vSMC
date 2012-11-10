#ifndef VSMC_UTILITY_STDTBB_HPP
#define VSMC_UTILITY_STDTBB_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/cxx11/thread.hpp>

namespace vsmc { namespace thread {

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

    unsigned get_thread_num () const
    {
        return thread_num_;
    }

    void set_thread_num (unsigned num)
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
        SizeType block_size = std::max VSMC_MINMAX_NO_EXPANSION (
                static_cast<SizeType>(1),
                N / static_cast<SizeType>(get_thread_num()));

        SizeType current = 0;
        unsigned num = 0;
        while (N > 0) {
            SizeType next_size = std::min VSMC_MINMAX_NO_EXPANSION (
                    N, block_size);
            *begin_iter = current;
            *end_iter = current = *begin_iter + block_size;
            ++begin_iter;
            ++end_iter;
            ++num;
            N -= next_size;
        }

        VSMC_RUNTIME_ASSERT((num <= get_thread_num()),
                "INVALID THREAD NUMBER **ThreadManager::partition**");

        return num;
    }

    private :

    unsigned thread_num_;

    ThreadManager () : thread_num_(std::max VSMC_MINMAX_NO_EXPANSION (1U,
             static_cast<unsigned>(cxx11::thread::hardware_concurrency()))) {}

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

    BlockedRange (size_type begin, size_type end) :
        begin_(begin), end_(end) {}

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
template <typename SizeType, typename WorkType>
void parallel_for (const BlockedRange<SizeType> &range, const WorkType &work)
{
    const ThreadManager &manager = ThreadManager::instance();
    unsigned thread_num = manager.get_thread_num();
    std::vector<SizeType> b(thread_num);
    std::vector<SizeType> e(thread_num);
    unsigned num = manager.partition(range.begin(), range.end(),
            b.begin(), e.begin());
#if VSMC_HAS_CXX11LIB_THREAD
    // TODO safer management of threads group
    std::vector<std::thread> tg;
    for (unsigned i = 0; i != num; ++i) {
        tg.push_back(std::thread(work,
                    BlockedRange<SizeType>(b[i], e[i])));
    }
    for (unsigned i = 0; i != num; ++i)
        tg[i].join();
#else // VSMC_HAS_CXX11LIB_THREAD
    boost::thread_group tg;
    for (unsigned i = 0; i != num; ++i) {
        tg.add_thread(new boost::thread(work,
                    BlockedRange<SizeType>(b[i], e[i])));
    }
    tg.join_all();
#endif // VSMC_HAS_CXX11LIB_THREAD
}

/// \brief Parallel sum using C++11 thread
/// \ingroup Thread
template <typename SizeType, typename WorkType, typename ResultType>
void parallel_sum (const BlockedRange<SizeType> &range, const WorkType &work,
        ResultType &res)
{
    const ThreadManager &manager = ThreadManager::instance();
    unsigned thread_num = manager.get_thread_num();
    std::vector<SizeType> b(thread_num);
    std::vector<SizeType> e(thread_num);
    std::vector<ResultType> result(thread_num);
    unsigned num = manager.partition(range.begin(), range.end(),
            b.begin(), e.begin());
#if VSMC_HAS_CXX11LIB_THREAD
    // TODO safer management of threads group
    std::vector<std::thread> tg;
    for (unsigned i = 0; i != num; ++i) {
        tg.push_back(std::thread(work,
                    BlockedRange<SizeType>(b[i], e[i]),
                    cxx11::ref(result[i])));
    }
    for (unsigned i = 0; i != num; ++i)
        tg[i].join();
#else // VSMC_HAS_CXX11LIB_THREAD
    boost::thread_group tg;
    for (unsigned i = 0; i != num; ++i) {
        tg.add_thread(new boost::thread(work,
                    BlockedRange<SizeType>(b[i], e[i]),
                    cxx11::ref(result[i])));
    }
    tg.join_all();
#endif // VSMC_HAS_CXX11LIB_THREAD
    for (unsigned i = 1; i != num; ++i)
        result[0] += result[i];

    res = result[0];
}

} } // namespace vsmc::thread

#endif // VSMC_UTILITY_STDTBB_HPP

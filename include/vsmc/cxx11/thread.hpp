#ifndef VSMC_CXX11_THREAD_HPP
#define VSMC_CXX11_THREAD_HPP

#include <vsmc/internal/config.hpp>
#include <vector>

#if VSMC_HAS_CXX11LIB_THREAD

#include <thread>
namespace vsmc { namespace cxx11 {
using std::thread;
} }

#else // VSMC_HAS_CXX11LIB_THREAD

#include <boost/thread.hpp>
namespace vsmc { namespace cxx11 {
using boost::thread;
} }

#endif // VSMC_HAS_CXX11LIB_THREAD

namespace vsmc {

namespace thread {

class ThreadManager
{
    public :

    static ThreadManager &create ()
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
        SizeType block_size = std::max(static_cast<SizeType>(1),
                N / static_cast<SizeType>(get_thread_num()));

        SizeType current = 0;
        unsigned num = 0;
        while (N > 0) {
            SizeType next_size = std::min(N, block_size);
            *begin_iter = current;
            *end_iter = current = *begin_iter + block_size;
            ++begin_iter;
            ++end_iter;
            ++num;
            N -= next_size;
        }

        VSMC_RUNTIME_ASSERT((num <= get_thread_num()),
                "WRONG THREAD NUMBER **ThreadManager::partition**");

        return num;
    }

    private :

    unsigned thread_num_;

    ThreadManager () : thread_num_(std::max(1U, static_cast<unsigned>(
                    cxx11::thread::hardware_concurrency()))) {}

    ThreadManager (const ThreadManager &) {}

    const ThreadManager &operator= (const ThreadManager &) {return *this;}
}; // class ThreadManager

template <typename SizeType>
class blocked_range
{
    public :

    typedef SizeType size_type;

    blocked_range (size_type begin, size_type end) :
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
}; // class blocked_range

template <typename SizeType, typename WorkType>
void parallel_for (const blocked_range<SizeType> &range, const WorkType &work)
{
    const ThreadManager &manager = ThreadManager::create();
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
                            blocked_range<SizeType>(b[i], e[i])));
            }
            for (unsigned i = 0; i != num; ++i)
                tg[i].join();
#else // VSMC_HAS_CXX11LIB_THREAD
            boost::thread_group tg;
            for (unsigned i = 0; i != num; ++i) {
                tg.add_thread(new boost::thread(work,
                            blocked_range<SizeType>(b[i], e[i]));
            }
            tg.join_all();
#endif // VSMC_HAS_CXX11LIB_THREAD
}

} // namespace thread

} // namespace vsmc

#endif // VSMC_CXX11_THREAD_HPP

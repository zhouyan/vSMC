#ifndef VSMC_HELPER_PARALLEL_STD_HPP
#define VSMC_HELPER_PARALLEL_STD_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/base.hpp>
#include <vsmc/cxx11/thread.hpp>

namespace vsmc {

namespace thread {

class ThreadManager
{
    public :

    static ThreadManager &reference ()
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

    ThreadManager (const ThreadManager &);
    ThreadManager &operator= (const ThreadManager &);
}; // class ThreadManager

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

template <typename SizeType, typename WorkType>
void parallel_for (const BlockedRange<SizeType> &range, const WorkType &work)
{
    const ThreadManager &manager = ThreadManager::reference();
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

template <typename SizeType, typename WorkType, typename ResultType>
void parallel_sum (const BlockedRange<SizeType> &range, const WorkType &work,
        ResultType &res)
{
    const ThreadManager &manager = ThreadManager::reference();
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

} // namespace thread

/// \brief Particle::value_type subtype
/// \ingroup STDThread
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
template <unsigned Dim, typename T>
class StateSTD : public StateBase<Dim, T>
{
    public :

    typedef StateBase<Dim, T> state_base_type;
    typedef typename state_base_type::size_type  size_type;
    typedef typename state_base_type::state_type state_type;

    explicit StateSTD (size_type N) : StateBase<Dim, T>(N), size_(N) {}

    template <typename IntType>
    void copy (const IntType *copy_from)
    {
        thread::parallel_for(thread::BlockedRange<size_type>(0, size_),
                copy_work_<IntType>(this, copy_from));
    }

    private :

    size_type size_;

    template <typename IntType>
    class copy_work_
    {
        public :

        copy_work_ (StateSTD<Dim, T> *state,
                const IntType *copy_from) :
            state_(state), copy_from_(copy_from) {}

        void operator() (const thread::BlockedRange<size_type> &range) const
        {
            for (size_type to = range.begin(); to != range.end(); ++to)
                state_->copy_particle(copy_from_[to], to);
        }

        private :

        StateSTD<Dim, T> *const state_;
        const IntType *const copy_from_;
    }; // class copy_work_
}; // class StateSTD

/// \brief Sampler<T>::init_type subtype
/// \ingroup STDThread
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class InitializeSTD : public InitializeBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (Particle<T> &particle, void *param)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateSTD, T, InitializeSTD);

        this->initialize_param(particle, param);
        this->pre_processor(particle);
        work_ work(this, &particle);
        unsigned accept;
        thread::parallel_sum(thread::BlockedRange<size_type>(
                    0, particle.value().size()), work, accept);
        this->post_processor(particle);

        return accept;
    }

    protected :

    InitializeSTD () {}
    InitializeSTD (const InitializeSTD<T, Derived> &) {}
    InitializeSTD<T, Derived> &operator=
        (const InitializeSTD<T, Derived> &) {return *this;}
    ~InitializeSTD () {}

    private :

    class work_
    {
        public :

        work_ (InitializeSTD<T, Derived> *init,
                Particle<T> *particle) :
            init_(init), particle_(particle) {}

        void operator() (const thread::BlockedRange<size_type> &range,
                unsigned &accept)
        {
            unsigned acc = 0;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                Particle<T> *const part = particle_;
                acc += init_->initialize_state(SingleParticle<T>(i, part));
            }
            accept = acc;
        }

        private :

        InitializeSTD<T, Derived> *const init_;
        Particle<T> *const particle_;
    }; // class work_
}; // class InitializeSTD

/// \brief Sampler<T>::move_type subtype
/// \ingroup STDThread
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MoveSTD : public MoveBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateSTD, T, MoveSTD);

        this->pre_processor(iter, particle);
        work_ work(this, iter, &particle);
        unsigned accept;
        thread::parallel_sum(thread::BlockedRange<size_type>(
                    0, particle.value().size()), work, accept);
        this->post_processor(iter, particle);

        return accept;
    }

    protected :

    MoveSTD () {}
    MoveSTD (const MoveSTD<T, Derived> &) {}
    MoveSTD<T, Derived> &operator=
        (const MoveSTD<T, Derived> &) {return *this;}
    ~MoveSTD () {}

    private :

    class work_
    {
        public :

        work_ (MoveSTD<T, Derived> *move, unsigned iter,
                Particle<T> *particle):
            move_(move), iter_(iter), particle_(particle) {}

        void operator() (const thread::BlockedRange<size_type> &range,
                unsigned &accept)
        {
            unsigned acc = 0;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                Particle<T> *const part = particle_;
                acc += move_->move_state(iter_, SingleParticle<T>(i, part));
            }
            accept = acc;
        }

        private :

        MoveSTD<T, Derived> *const move_;
        const unsigned iter_;
        Particle<T> *const particle_;
    }; // class work_
}; // class MoveSTD

/// \brief Monitor<T>::eval_type subtype
/// \ingroup STDThread
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MonitorEvalSTD : public MonitorEvalBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    void operator() (unsigned iter, unsigned dim, const Particle<T> &particle,
            double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateSTD, T, MonitorEvalSTD);

        this->pre_processor(iter, particle);
        thread::parallel_for(thread::BlockedRange<size_type>(
                    0, particle.value().size()),
                work_(this, iter, dim, &particle, res));
        this->post_processor(iter, particle);
    }

    protected :

    MonitorEvalSTD () {}
    MonitorEvalSTD (const MonitorEvalSTD<T, Derived> &) {}
    MonitorEvalSTD<T, Derived> &operator=
        (const MonitorEvalSTD<T, Derived> &) {return *this;}
    ~MonitorEvalSTD () {}

    private :

    class work_
    {
        public :

        work_ (MonitorEvalSTD<T, Derived> *monitor,
                unsigned iter, unsigned dim,
                const Particle<T> *particle, double *res) :
            monitor_(monitor), iter_(iter), dim_(dim), particle_(particle),
            res_(res) {}

        void operator() (const thread::BlockedRange<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                double *const r = res_ + i * dim_;
                const Particle<T> *const part = particle_;
                monitor_->monitor_state(iter_, dim_,
                        ConstSingleParticle<T>(i, part), r);
            }
        }

        private :

        MonitorEvalSTD<T, Derived> *const monitor_;
        const unsigned iter_;
        const unsigned dim_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
}; // class MonitorEvalSTD

/// \brief Path<T>::eval_type subtype
/// \ingroup STDThread
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class PathEvalSTD : public PathEvalBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    double operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateSTD, T, PathEvalSTD);

        this->pre_processor(iter, particle);
        thread::parallel_for(thread::BlockedRange<size_type>(
                    0, particle.value().size()),
                work_(this, iter, &particle, res));
        this->post_processor(iter, particle);

        return this->path_width(iter, particle);
    }

    protected :

    PathEvalSTD () {}
    PathEvalSTD (const PathEvalSTD<T, Derived> &) {}
    PathEvalSTD<T, Derived> &operator=
        (const PathEvalSTD<T, Derived> &) {return *this;}
    ~PathEvalSTD () {}

    private :

    class work_
    {
        public :

        work_ (PathEvalSTD<T, Derived> *path, unsigned iter,
                const Particle<T> *particle, double *res) :
            path_(path), iter_(iter), particle_(particle), res_(res) {}

        void operator() (const thread::BlockedRange<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                const Particle<T> *const part = particle_;
                res_[i] = path_->path_state(iter_,
                        ConstSingleParticle<T>(i, part));
            }
        }

        private :

        PathEvalSTD<T, Derived> *const path_;
        const unsigned iter_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
}; // PathEvalSTD

} // namespace vsmc

#endif // VSMC_HELPER_PARALLEL_STD_HPP

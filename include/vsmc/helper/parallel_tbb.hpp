#ifndef VSMC_HELPER_PARALLEL_TBB_HPP
#define VSMC_HELPER_PARALLEL_TBB_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/base.hpp>

#ifdef __clang__
#ifndef TBB_USE_CAPTURED_EXCEPTION
#define TBB_USE_CAPTURED_EXCEPTION 1
#endif
#endif // __clang__

#include <tbb/tbb.h>

/// \defgroup TBB Intel Threading Buidling Block
/// \ingroup Helper
/// \brief Parallelized samplers with Intel TBB

namespace vsmc {

/// \brief Particle::value_type subtype
/// \ingroup TBB
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
/// \tparam Timer The timer
template <unsigned Dim, typename T, typename Timer>
class StateTBB : public StateBase<Dim, T, Timer>
{
    public :

    typedef typename StateBase<Dim, T, Timer>::size_type size_type;
    typedef T state_type;
    typedef Timer timer_type;

    explicit StateTBB (size_type N) : StateBase<Dim, T, Timer>(N), size_(N) {}

    template <typename SizeType>
    void copy (const SizeType *copy_from)
    {
        this->timer().start();
        tbb::parallel_for(tbb::blocked_range<size_type>(0, size_),
                copy_work_<SizeType>(this, copy_from));
        this->timer().stop();
    }

    private :

    size_type size_;

    template <typename SizeType>
    class copy_work_
    {
        public :

        copy_work_ (StateTBB<Dim, T, Timer> *state,
                const SizeType *copy_from) :
            state_(state), copy_from_(copy_from) {}

        void operator () (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type to = range.begin(); to != range.end(); ++to) {
                size_type from = copy_from_[to];
                if (from != to)
                    state_->state().col(to) = state_->state().col(from);
            }
        }

        private :

        StateTBB<Dim, T, Timer> *const state_;
        const SizeType *const copy_from_;
    }; // class work_
}; // class StateTBB

/// \brief Sampler<T>::init_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class InitializeTBB : public InitializeBase<T, Derived>
{
    public :

    typedef typename SizeTypeTrait<T>::type size_type;
    typedef T value_type;

    unsigned operator() (Particle<T> &particle, void *param)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateTBB, T, InitializeTBB);

        this->initialize_param(particle, param);
        this->pre_processor(particle);
        particle.value().timer().start();
        work_ work(this, &particle);
        tbb::parallel_reduce(tbb::blocked_range<size_type>(
                    0, particle.value().size()), work);
        particle.value().timer().stop();
        this->post_processor(particle);

        return work.accept();
    }

    protected :

    InitializeTBB () {}
    InitializeTBB (const InitializeTBB<T, Derived> &) {}
    const InitializeTBB<T, Derived> &operator=
        (const InitializeTBB<T, Derived> &) {return *this;}
    ~InitializeTBB () {}

    private :

    class work_
    {
        public :

        work_ (InitializeTBB<T, Derived> *init,
                Particle<T> *particle) :
            init_(init), particle_(particle), accept_(0) {}

        work_ (const work_ &other, tbb::split) :
            init_(other.init_), particle_(other.particle_), accept_(0) {}

        void operator() (const tbb::blocked_range<size_type> &range)
        {
            unsigned acc = accept_;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                Particle<T> *const part = particle_;
                acc += init_->initialize_state(SingleParticle<T>(i, part));
            }
            accept_ = acc;
        }

        void join (const work_ &other)
        {
            accept_ += other.accept_;
        }

        unsigned accept () const
        {
            return accept_;
        }

        private :

        InitializeTBB<T, Derived> *const init_;
        Particle<T> *const particle_;
        unsigned accept_;
    }; // class work_
}; // class InitializeTBB

/// \brief Sampler<T>::move_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MoveTBB : public MoveBase<T, Derived>
{
    public :

    typedef typename SizeTypeTrait<T>::type size_type;
    typedef T value_type;

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateTBB, T, MoveTBB);

        this->pre_processor(iter, particle);
        particle.value().timer().start();
        work_ work(this, iter, &particle);
        tbb::parallel_reduce(tbb::blocked_range<size_type>(
                    0, particle.value().size()), work);
        particle.value().timer().stop();
        this->post_processor(iter, particle);

        return work.accept();
    }

    protected :

    MoveTBB () {}
    MoveTBB (const MoveTBB<T, Derived> &) {}
    const MoveTBB<T, Derived> &operator=
        (const MoveTBB<T, Derived> &) {return *this;}
    ~MoveTBB () {}

    private :

    class work_
    {
        public :

        work_ (MoveTBB<T, Derived> *move, unsigned iter,
                Particle<T> *particle):
            move_(move), iter_(iter), particle_(particle), accept_(0) {}

        work_ (const work_ &other, tbb::split) :
            move_(other.move_), iter_(other.iter_),
            particle_(other.particle_), accept_(0) {}

        void operator() (const tbb::blocked_range<size_type> &range)
        {
            unsigned acc = accept_;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                Particle<T> *const part = particle_;
                acc += move_->move_state(iter_, SingleParticle<T>(i, part));
            }
            accept_ = acc;
        }

        void join (const work_ &other)
        {
            accept_ += other.accept_;
        }

        unsigned accept () const
        {
            return accept_;
        }

        private :

        MoveTBB<T, Derived> *const move_;
        const unsigned iter_;
        Particle<T> *const particle_;
        unsigned accept_;
    }; // class work_
}; // class MoveTBB

/// \brief Monitor<T>::eval_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MonitorEvalTBB : public MonitorEvalBase<T, Derived>
{
    public :

    typedef typename SizeTypeTrait<T>::type size_type;
    typedef T value_type;

    void operator() (unsigned iter, unsigned dim, const Particle<T> &particle,
            double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateTBB, T, MonitorEvalTBB);

        this->pre_processor(iter, particle);
        particle.value().timer().start();
        tbb::parallel_for(tbb::blocked_range<size_type>(
                    0, particle.value().size()),
                work_(this, iter, dim, &particle, res));
        particle.value().timer().stop();
        this->post_processor(iter, particle);
    }

    protected :

    MonitorEvalTBB () {}
    MonitorEvalTBB (const MonitorEvalTBB<T, Derived> &) {}
    const MonitorEvalTBB<T, Derived> &operator=
        (const MonitorEvalTBB<T, Derived> &) {return *this;}
    ~MonitorEvalTBB () {}

    private :

    class work_
    {
        public :

        work_ (MonitorEvalTBB<T, Derived> *monitor,
                unsigned iter, unsigned dim,
                const Particle<T> *particle, double *res) :
            monitor_(monitor), iter_(iter), dim_(dim), particle_(particle),
            res_(res) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                double *const r = res_ + i * dim_;
                const Particle<T> *const part = particle_;
                monitor_->monitor_state(iter_, dim_,
                        ConstSingleParticle<T>(i, part), r);
            }
        }

        private :

        MonitorEvalTBB<T, Derived> *const monitor_;
        const unsigned iter_;
        const unsigned dim_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
}; // class MonitorEvalTBB

/// \brief Path<T>::eval_type subtype
/// \ingroup TBB
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class PathEvalTBB : public PathEvalBase<T, Derived>
{
    public :

    typedef typename SizeTypeTrait<T>::type size_type;
    typedef T value_type;

    double operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateTBB, T, PathEvalTBB);

        this->pre_processor(iter, particle);
        particle.value().timer().start();
        tbb::parallel_for(tbb::blocked_range<size_type>(
                    0, particle.value().size()),
                work_(this, iter, &particle, res));
        particle.value().timer().stop();
        this->post_processor(iter, particle);

        return this->path_width(iter, particle);
    }

    protected :

    PathEvalTBB () {}
    PathEvalTBB (const PathEvalTBB<T, Derived> &) {}
    const PathEvalTBB<T, Derived> &operator=
        (const PathEvalTBB<T, Derived> &) {return *this;}
    ~PathEvalTBB () {}

    private :

    class work_
    {
        public :

        work_ (PathEvalTBB<T, Derived> *path, unsigned iter,
                const Particle<T> *particle, double *res) :
            path_(path), iter_(iter), particle_(particle), res_(res) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                const Particle<T> *const part = particle_;
                res_[i] = path_->path_state(iter_,
                        ConstSingleParticle<T>(i, part));
            }
        }

        private :

        PathEvalTBB<T, Derived> *const path_;
        const unsigned iter_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
}; // PathEvalTBB

} // namespace vsmc

#endif // VSMC_HELPER_PARALLEL_TBB_HPP

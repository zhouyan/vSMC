#ifndef VSMC_SMP_PARALLEL_TBB_HPP
#define VSMC_SMP_PARALLEL_TBB_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/smp/base.hpp>

#if defined(__clang__) && !defined(_LIBCPP_VERSION) && (__GLIBCXX__ < 20100429)
#ifndef TBB_USE_CAPTURED_EXCEPTION
#define TBB_USE_CAPTURED_EXCEPTION 1
#endif
#endif // __clang__

#include <tbb/tbb.h>

namespace vsmc {

/// \brief Particle::value_type subtype
/// \ingroup TBB
template <std::size_t Dim, typename T>
class StateTBB : public StateBase<Dim, T>
{
    public :

    typedef StateBase<Dim, T> state_base_type;
    typedef typename state_base_type::size_type size_type;
    typedef typename state_base_type::state_type state_type;

    explicit StateTBB (size_type N) : StateBase<Dim, T>(N) {}

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_COPY_SIZE_MISMATCH(TBB);

        tbb::parallel_for(tbb::blocked_range<size_type>(0, this->size()),
                copy_work_<IntType>(this, copy_from));
    }

    private :

    template <typename IntType>
    class copy_work_
    {
        public :

        copy_work_ (StateTBB<Dim, T> *state, const IntType *copy_from) :
            state_(state), copy_from_(copy_from) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type to = range.begin(); to != range.end(); ++to)
                state_->copy_particle(copy_from_[to], to);
        }

        private :

        StateTBB<Dim, T> *const state_;
        const IntType *const copy_from_;
    }; // class copy_work_
}; // class StateTBB

/// \brief Sampler<T>::init_type subtype
/// \ingroup TBB
template <typename T, typename Derived>
class InitializeTBB : public InitializeBase<T, Derived>
{
    public :

    typedef InitializeBase<T, Derived> initialize_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    std::size_t operator() (Particle<T> &particle, void *param)
    {
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        work_ work(this, &particle);
        tbb::parallel_reduce(tbb::blocked_range<size_type>(
                    0, particle.size()), work);
        this->post_processor(particle);

        return work.accept();
    }

    protected :

    InitializeTBB () {}
    InitializeTBB (const InitializeTBB<T, Derived> &) {}
    InitializeTBB<T, Derived> &operator=
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
            std::size_t acc = accept_;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                acc += init_->initialize_state(
                        SingleParticle<T>(i, particle_));
            }
            accept_ = acc;
        }

        void join (const work_ &other)
        {
            accept_ += other.accept_;
        }

        std::size_t accept () const
        {
            return accept_;
        }

        private :

        InitializeTBB<T, Derived> *const init_;
        Particle<T> *const particle_;
        std::size_t accept_;
    }; // class work_
}; // class InitializeTBB

/// \brief Sampler<T>::move_type subtype
/// \ingroup TBB
template <typename T, typename Derived>
class MoveTBB : public MoveBase<T, Derived>
{
    public :

    typedef MoveBase<T, Derived> move_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    std::size_t operator() (std::size_t iter, Particle<T> &particle)
    {
        this->pre_processor(iter, particle);
        work_ work(this, iter, &particle);
        tbb::parallel_reduce(tbb::blocked_range<size_type>(
                    0, particle.size()), work);
        this->post_processor(iter, particle);

        return work.accept();
    }

    protected :

    MoveTBB () {}
    MoveTBB (const MoveTBB<T, Derived> &) {}
    MoveTBB<T, Derived> &operator=
        (const MoveTBB<T, Derived> &) {return *this;}
    ~MoveTBB () {}

    private :

    class work_
    {
        public :

        work_ (MoveTBB<T, Derived> *move, std::size_t iter,
                Particle<T> *particle):
            move_(move), iter_(iter), particle_(particle), accept_(0) {}

        work_ (const work_ &other, tbb::split) :
            move_(other.move_), iter_(other.iter_),
            particle_(other.particle_), accept_(0) {}

        void operator() (const tbb::blocked_range<size_type> &range)
        {
            std::size_t acc = accept_;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                acc += move_->move_state(iter_,
                        SingleParticle<T>(i, particle_));
            }
            accept_ = acc;
        }

        void join (const work_ &other)
        {
            accept_ += other.accept_;
        }

        std::size_t accept () const
        {
            return accept_;
        }

        private :

        MoveTBB<T, Derived> *const move_;
        const std::size_t iter_;
        Particle<T> *const particle_;
        std::size_t accept_;
    }; // class work_
}; // class MoveTBB

/// \brief Monitor<T>::eval_type subtype
/// \ingroup TBB
template <typename T, typename Derived>
class MonitorEvalTBB : public MonitorEvalBase<T, Derived>
{
    public :

    typedef MonitorEvalBase<T, Derived> monitor_eval_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    void operator() (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res)
    {
        this->pre_processor(iter, particle);
        tbb::parallel_for(tbb::blocked_range<size_type>(0, particle.size()),
                work_(this, iter, dim, &particle, res));
        this->post_processor(iter, particle);
    }

    protected :

    MonitorEvalTBB () {}
    MonitorEvalTBB (const MonitorEvalTBB<T, Derived> &) {}
    MonitorEvalTBB<T, Derived> &operator=
        (const MonitorEvalTBB<T, Derived> &) {return *this;}
    ~MonitorEvalTBB () {}

    private :

    class work_
    {
        public :

        work_ (MonitorEvalTBB<T, Derived> *monitor,
                std::size_t iter, std::size_t dim,
                const Particle<T> *particle, double *res) :
            monitor_(monitor), particle_(particle), res_(res),
            iter_(iter), dim_(dim) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                monitor_->monitor_state(iter_, dim_,
                        ConstSingleParticle<T>(i, particle_), res_ + i * dim_);
            }
        }

        private :

        MonitorEvalTBB<T, Derived> *const monitor_;
        const Particle<T> *const particle_;
        double *const res_;
        const std::size_t iter_;
        const std::size_t dim_;
    }; // class work_
}; // class MonitorEvalTBB

/// \brief Path<T>::eval_type subtype
/// \ingroup TBB
template <typename T, typename Derived>
class PathEvalTBB : public PathEvalBase<T, Derived>
{
    public :

    typedef PathEvalBase<T, Derived> path_eval_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    double operator() (std::size_t iter, const Particle<T> &particle,
            double *res)
    {
        this->pre_processor(iter, particle);
        tbb::parallel_for(tbb::blocked_range<size_type>(0, particle.size()),
                work_(this, iter, &particle, res));
        this->post_processor(iter, particle);

        return this->path_width(iter, particle);
    }

    protected :

    PathEvalTBB () {}
    PathEvalTBB (const PathEvalTBB<T, Derived> &) {}
    PathEvalTBB<T, Derived> &operator=
        (const PathEvalTBB<T, Derived> &) {return *this;}
    ~PathEvalTBB () {}

    private :

    class work_
    {
        public :

        work_ (PathEvalTBB<T, Derived> *path, std::size_t iter,
                const Particle<T> *particle, double *res) :
            path_(path), particle_(particle), res_(res), iter_(iter) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                res_[i] = path_->path_state(iter_,
                        ConstSingleParticle<T>(i, particle_));
            }
        }

        private :

        PathEvalTBB<T, Derived> *const path_;
        const Particle<T> *const particle_;
        double *const res_;
        const std::size_t iter_;
    }; // class work_
}; // PathEvalTBB

} // namespace vsmc

#endif // VSMC_SMP_PARALLEL_TBB_HPP

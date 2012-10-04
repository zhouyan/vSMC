#ifndef VSMC_HELPER_PARALLEL_STD_HPP
#define VSMC_HELPER_PARALLEL_STD_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/base.hpp>
#include <vsmc/cxx11/thread.hpp>
#include <tbb/tbb.h>

namespace vsmc {

template <unsigned Dim, typename T, typename Timer>
class StateSTD : public StateBase<Dim, T, Timer>
{
    public :

    typedef StateBase<Dim, T, Timer> state_base_type;
    typedef typename state_base_type::size_type  size_type;
    typedef typename state_base_type::state_type state_type;
    typedef typename state_base_type::timer_type timer_type;

    explicit StateSTD (size_type N) : StateBase<Dim, T, Timer>(N), size_(N) {}

    template <typename IntType>
    void copy (const IntType *copy_from)
    {
        this->timer().start();
        thread::parallel_for(thread::blocked_range<size_type>(0, size_),
                copy_work_<IntType>(this, copy_from));
        this->timer().stop();
    }

    private :

    size_type size_;

    template <typename IntType>
    class copy_work_
    {
        public :

        copy_work_ (StateSTD<Dim, T, Timer> *state,
                const IntType *copy_from) :
            state_(state), copy_from_(copy_from) {}

        void operator() (const thread::blocked_range<size_type> &range) const
        {
            for (size_type to = range.begin(); to != range.end(); ++to)
                state_->copy_particle(copy_from_[to], to);
        }

        private :

        StateSTD<Dim, T, Timer> *const state_;
        const IntType *const copy_from_;
    }; // class copy_work_
}; // class StateSTD

template <typename T, typename Derived>
class InitializeSTD : public InitializeBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (Particle<T> &particle, void *param)
    {
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

    InitializeSTD () {}
    InitializeSTD (const InitializeSTD<T, Derived> &) {}
    const InitializeSTD<T, Derived> &operator=
        (const InitializeSTD<T, Derived> &) {return *this;}
    ~InitializeSTD () {}

    private :

    class work_
    {
        public :

        work_ (InitializeSTD<T, Derived> *init,
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

        InitializeSTD<T, Derived> *const init_;
        Particle<T> *const particle_;
        unsigned accept_;
    }; // class work_
}; // class InitializeSTD

template <typename T, typename Derived>
class MoveSTD : public MoveBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
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

    MoveSTD () {}
    MoveSTD (const MoveSTD<T, Derived> &) {}
    const MoveSTD<T, Derived> &operator=
        (const MoveSTD<T, Derived> &) {return *this;}
    ~MoveSTD () {}

    private :

    class work_
    {
        public :

        work_ (MoveSTD<T, Derived> *move, unsigned iter,
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

        MoveSTD<T, Derived> *const move_;
        const unsigned iter_;
        Particle<T> *const particle_;
        unsigned accept_;
    }; // class work_
}; // class MoveSTD

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
        particle.value().timer().start();
        thread::parallel_for(thread::blocked_range<size_type>(
                    0, particle.value().size()),
                work_(this, iter, dim, &particle, res));
        particle.value().timer().stop();
        this->post_processor(iter, particle);
    }

    protected :

    MonitorEvalSTD () {}
    MonitorEvalSTD (const MonitorEvalSTD<T, Derived> &) {}
    const MonitorEvalSTD<T, Derived> &operator=
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

        void operator() (const thread::blocked_range<size_type> &range) const
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
        particle.value().timer().start();
        thread::parallel_for(thread::blocked_range<size_type>(
                    0, particle.value().size()),
                work_(this, iter, &particle, res));
        particle.value().timer().stop();
        this->post_processor(iter, particle);

        return this->path_width(iter, particle);
    }

    protected :

    PathEvalSTD () {}
    PathEvalSTD (const PathEvalSTD<T, Derived> &) {}
    const PathEvalSTD<T, Derived> &operator=
        (const PathEvalSTD<T, Derived> &) {return *this;}
    ~PathEvalSTD () {}

    private :

    class work_
    {
        public :

        work_ (PathEvalSTD<T, Derived> *path, unsigned iter,
                const Particle<T> *particle, double *res) :
            path_(path), iter_(iter), particle_(particle), res_(res) {}

        void operator() (const thread::blocked_range<size_type> &range) const
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

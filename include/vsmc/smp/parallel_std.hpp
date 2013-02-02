#ifndef VSMC_SMP_PARALLEL_STD_HPP
#define VSMC_SMP_PARALLEL_STD_HPP

#include <vsmc/smp/base.hpp>
#include <vsmc/utility/stdtbb.hpp>

namespace vsmc {

/// \brief Particle::value_type subtype
/// \ingroup STD
template <std::size_t Dim, typename T>
class StateSTD : public StateBase<Dim, T>
{
    public :

    typedef StateBase<Dim, T> state_base_type;
    typedef typename state_base_type::size_type size_type;
    typedef typename state_base_type::state_type state_type;

    explicit StateSTD (size_type N) : StateBase<Dim, T>(N) {}

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_COPY_SIZE_MISMATCH(STD);

        parallel_for(BlockedRange<size_type>(0, N),
                copy_work_<IntType>(this, copy_from));
    }

    private :

    template <typename IntType>
    class copy_work_
    {
        public :

        copy_work_ (StateSTD<Dim, T> *state,
                const IntType *copy_from) :
            state_(state), copy_from_(copy_from) {}

        void operator() (const BlockedRange<size_type> &range) const
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
/// \ingroup STD
template <typename T, typename Derived>
class InitializeSTD : public InitializeBase<T, Derived>
{
    public :

    typedef InitializeBase<T, Derived> initialize_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    std::size_t operator() (Particle<T> &particle, void *param)
    {
        const size_type N = static_cast<size_type>(particle.value().size());
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        std::size_t accept = parallel_accumulate(BlockedRange<size_type>(0, N),
                work_(this, &particle), static_cast<std::size_t>(0));
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

        void operator() (const BlockedRange<size_type> &range,
                std::size_t &accept) const
        {
            std::size_t acc = 0;
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
/// \ingroup STD
template <typename T, typename Derived>
class MoveSTD : public MoveBase<T, Derived>
{
    public :

    typedef MoveBase<T, Derived> move_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    std::size_t operator() (std::size_t iter, Particle<T> &particle)
    {
        const size_type N = static_cast<size_type>(particle.value().size());
        this->pre_processor(iter, particle);
        std::size_t accept = parallel_accumulate(BlockedRange<size_type>(0, N),
                work_(this, iter, &particle), static_cast<std::size_t>(0));
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

        work_ (MoveSTD<T, Derived> *move, std::size_t iter,
                Particle<T> *particle):
            move_(move), iter_(iter), particle_(particle) {}

        void operator() (const BlockedRange<size_type> &range,
                std::size_t &accept) const
        {
            std::size_t acc = 0;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                Particle<T> *const part = particle_;
                acc += move_->move_state(iter_, SingleParticle<T>(i, part));
            }
            accept = acc;
        }

        private :

        MoveSTD<T, Derived> *const move_;
        const std::size_t iter_;
        Particle<T> *const particle_;
    }; // class work_
}; // class MoveSTD

/// \brief Monitor<T>::eval_type subtype
/// \ingroup STD
template <typename T, typename Derived>
class MonitorEvalSTD : public MonitorEvalBase<T, Derived>
{
    public :

    typedef MonitorEvalBase<T, Derived> monitor_eval_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    void operator() (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res)
    {
        const size_type N = static_cast<size_type>(particle.value().size());
        this->pre_processor(iter, particle);
        parallel_for(BlockedRange<size_type>(0, N),
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
                std::size_t iter, std::size_t dim,
                const Particle<T> *particle, double *res) :
            monitor_(monitor), iter_(iter), dim_(dim), particle_(particle),
            res_(res) {}

        void operator() (const BlockedRange<size_type> &range) const
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
        const std::size_t iter_;
        const std::size_t dim_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
}; // class MonitorEvalSTD

/// \brief Path<T>::eval_type subtype
/// \ingroup STD
template <typename T, typename Derived>
class PathEvalSTD : public PathEvalBase<T, Derived>
{
    public :

    typedef PathEvalBase<T, Derived> path_eval_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    double operator() (std::size_t iter, const Particle<T> &particle,
            double *res)
    {
        const size_type N = static_cast<size_type>(particle.value().size());
        this->pre_processor(iter, particle);
        parallel_for(BlockedRange<size_type>(0, N),
                work_(this, iter, &particle, res));
        this->post_processor(iter, particle);

        return this->path_grid(iter, particle);
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

        work_ (PathEvalSTD<T, Derived> *path, std::size_t iter,
                const Particle<T> *particle, double *res) :
            path_(path), iter_(iter), particle_(particle), res_(res) {}

        void operator() (const BlockedRange<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                const Particle<T> *const part = particle_;
                res_[i] = path_->path_state(iter_,
                        ConstSingleParticle<T>(i, part));
            }
        }

        private :

        PathEvalSTD<T, Derived> *const path_;
        const std::size_t iter_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
}; // PathEvalSTD

} // namespace vsmc

#endif // VSMC_SMP_PARALLEL_STD_HPP

#ifndef VSMC_SMP_PARALLEL_PPL_HPP
#define VSMC_SMP_PARALLEL_PPL_HPP

#include <vsmc/smp/base.hpp>
#include <vsmc/utility/ppl_wrapper.hpp>

namespace vsmc {

/// \brief Particle::value_type subtype
/// \ingroup PPL
template <std::size_t Dim, typename T>
class StatePPL : public StateBase<Dim, T>
{
    public :

    typedef StateBase<Dim, T> state_base_type;
    typedef typename state_base_type::size_type size_type;
    typedef typename state_base_type::state_type state_type;

    explicit StatePPL (size_type N) : StateBase<Dim, T>(N) {}

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_COPY_SIZE_MISMATCH(PPL);

        ppl::parallel_for(static_cast<size_type>(0), N,
                copy_work_<IntType>(this, copy_from));
    }

    private :

    template <typename IntType>
    class copy_work_
    {
        public :

        copy_work_ (StatePPL<Dim, T> *state, const IntType *copy_from) :
            state_(state), copy_from_(copy_from) {}

        void operator() (size_type to) const
        {
            state_->copy_particle(copy_from_[to], to);
        }

        private :

        StatePPL<Dim, T> *const state_;
        const IntType *const copy_from_;
    }; // class copy_work_
}; // class StatePPL

/// \brief Sampler<T>::init_type subtype
/// \ingroup PPL
template <typename T, typename Derived>
class InitializePPL : public InitializeBase<T, Derived>
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
        ppl::combinable<std::size_t> accept(accept_init_);
        ppl::parallel_for(static_cast<size_type>(0), N,
                work_(this, &particle, &accept));
        this->post_processor(particle);

        return accept.combine(accept_accu_);
    }

    protected :

    InitializePPL () {}
    InitializePPL (const InitializePPL<T, Derived> &) {}
    InitializePPL<T, Derived> &operator=
        (const InitializePPL<T, Derived> &) {return *this;}
    ~InitializePPL () {}

    private :

    class work_
    {
        public :

        work_ (InitializePPL<T, Derived> *init,
                Particle<T> *particle, ppl::combinable<std::size_t> *accept) :
            init_(init), particle_(particle), accept_(accept) {}

        void operator() (size_type i) const
        {
            accept_->local() += init_->initialize_state(
                    SingleParticle<T>(i, particle_));
        }

        private :

        InitializePPL<T, Derived> *const init_;
        Particle<T> *const particle_;
        ppl::combinable<std::size_t> *const accept_;
    }; // class work_

    static std::size_t accept_init_ () {return 0;}
    static std::size_t accept_accu_ (std::size_t a, std::size_t b)
    {return a + b;}
}; // class InitializePPL

/// \brief Sampler<T>::move_type subtype
/// \ingroup PPL
template <typename T, typename Derived>
class MovePPL : public MoveBase<T, Derived>
{
    public :

    typedef MoveBase<T, Derived> move_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    std::size_t operator() (std::size_t iter, Particle<T> &particle)
    {
        const size_type N = static_cast<size_type>(particle.value().size());
        this->pre_processor(iter, particle);
        ppl::combinable<std::size_t> accept(accept_init_);
        ppl::parallel_for(static_cast<size_type>(0), N,
                work_(this, iter, &particle, &accept));
        this->post_processor(iter, particle);

        return accept.combine(accept_accu_);
    }

    protected :

    MovePPL () {}
    MovePPL (const MovePPL<T, Derived> &) {}
    MovePPL<T, Derived> &operator=
        (const MovePPL<T, Derived> &) {return *this;}
    ~MovePPL () {}

    private :

    class work_
    {
        public :

        work_ (MovePPL<T, Derived> *move, std::size_t iter,
                Particle<T> *particle, ppl::combinable<std::size_t> *accept):
            move_(move), particle_(particle), accept_(accept), iter_(iter) {}

        void operator() (size_type i) const
        {
            accept_->local() += move_->move_state(iter_,
                    SingleParticle<T>(i, particle_));
        }

        private :

        MovePPL<T, Derived> *const move_;
        Particle<T> *const particle_;
        ppl::combinable<std::size_t> *const accept_;
        const std::size_t iter_;
    }; // class work_

    static std::size_t accept_init_ () {return 0;}
    static std::size_t accept_accu_ (std::size_t a, std::size_t b)
    {return a + b;}
}; // class MovePPL

/// \brief Monitor<T>::eval_type subtype
/// \ingroup PPL
template <typename T, typename Derived>
class MonitorEvalPPL : public MonitorEvalBase<T, Derived>
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
        ppl::parallel_for(static_cast<size_type>(0), N,
                work_(this, iter, dim, &particle, res));
        this->post_processor(iter, particle);
    }

    protected :

    MonitorEvalPPL () {}
    MonitorEvalPPL (const MonitorEvalPPL<T, Derived> &) {}
    MonitorEvalPPL<T, Derived> &operator=
        (const MonitorEvalPPL<T, Derived> &) {return *this;}
    ~MonitorEvalPPL () {}

    private :

    class work_
    {
        public :

        work_ (MonitorEvalPPL<T, Derived> *monitor,
                std::size_t iter, std::size_t dim,
                const Particle<T> *particle, double *res) :
            monitor_(monitor), particle_(particle), res_(res),
            iter_(iter), dim_(dim) {}

        void operator() (size_type i) const
        {
            monitor_->monitor_state(iter_, dim_,
                    ConstSingleParticle<T>(i, particle_), res_ + i * dim_);
        }

        private :

        MonitorEvalPPL<T, Derived> *const monitor_;
        const Particle<T> *const particle_;
        double *const res_;
        const std::size_t iter_;
        const std::size_t dim_;
    }; // class work_
}; // class MonitorEvalPPL

/// \brief Path<T>::eval_type subtype
/// \ingroup PPL
template <typename T, typename Derived>
class PathEvalPPL : public PathEvalBase<T, Derived>
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
        ppl::parallel_for(static_cast<size_type>(0), N,
                work_(this, iter, &particle, res));
        this->post_processor(iter, particle);

        return this->path_grid(iter, particle);
    }

    protected :

    PathEvalPPL () {}
    PathEvalPPL (const PathEvalPPL<T, Derived> &) {}
    PathEvalPPL<T, Derived> &operator=
        (const PathEvalPPL<T, Derived> &) {return *this;}
    ~PathEvalPPL () {}

    private :

    class work_
    {
        public :

        work_ (PathEvalPPL<T, Derived> *path, std::size_t iter,
                const Particle<T> *particle, double *res) :
            path_(path), particle_(particle), res_(res), iter_(iter) {}

        void operator() (size_type i) const
        {
            res_[i] = path_->path_state(iter_,
                    ConstSingleParticle<T>(i, particle_));
        }

        private :

        PathEvalPPL<T, Derived> *const path_;
        const Particle<T> *const particle_;
        double *const res_;
        const std::size_t iter_;
    }; // class work_
}; // PathEvalPPL

} // namespace vsmc

#endif // VSMC_SMP_PARALLEL_PPL_HPP

#ifndef VSMC_SMP_BACKEND_PPL_HPP
#define VSMC_SMP_BACKEND_PPL_HPP

#include <vsmc/smp/base.hpp>
#include <vsmc/utility/ppl_wrapper.hpp>

namespace vsmc {

/// \brief Particle::weight_set_type subtype using Parallel Pattern Library
/// \ingroup SMP
template <typename BaseState>
class WeightSetPPL : public traits::WeightSetTypeTrait<BaseState>::type
{
    typedef typename traits::WeightSetTypeTrait<BaseState>::type base;

    public :

    typedef typename traits::SizeTypeTrait<base>::type size_type;

    explicit WeightSetPPL (size_type N) : base(N) {}

    protected :

    void log_weight2weight ()
    {
        const size_type N = static_cast<size_type>(this->size());
        ppl::parallel_for(static_cast<size_type>(0), N,
                log_weight2weight_(
                    this->weight_ptr(), this->log_weight_ptr()));
    }

    void weight2log_weight ()
    {
        const size_type N = static_cast<size_type>(this->size());
        ppl::parallel_for(static_cast<size_type>(0), N,
                weight2log_weight_(
                    this->weight_ptr(), this->log_weight_ptr()));
    }

    private :

    class log_weight2weight_
    {
        public :

        log_weight2weight_ (double *weight, const double *log_weight) :
            weight_(weight), log_weight_(log_weight) {}

        void operator() (size_type i) const
        {
            using std::exp;

            weight_[i] = exp(log_weight_[i]);
        }

        private :

        double *const weight_;
        const double *const log_weight_;
    }; // class log_weight2weight_

    class weight2log_weight_
    {
        public :

        weight2log_weight_ (const double *weight, double *log_weight) :
            weight_(weight), log_weight_(log_weight) {}

        void operator() (size_type i) const
        {
            using std::log;

            log_weight_[i] = log(weight_[i]);
        }

        private :

        const double *const weight_;
        double *const log_weight_;
    }; // class weight2log_weight_
}; // class WeightSetPPL

/// \brief Calculating normalizing constant ratio using Microsoft Parallel
/// Pattern Library
/// \ingroup SMP
class NormalizingConstantPPL : public NormalizingConstant
{
    public :

    NormalizingConstantPPL (std::size_t N) : NormalizingConstant(N) {}

    protected:

    void vd_exp (std::size_t N, double *inc_weight) const
    {ppl::parallel_for(static_cast<std::size_t>(0), N, vd_exp_(inc_weight));}

    private :

    class vd_exp_
    {
        public :

        vd_exp_ (double *inc_weight) : inc_weight_(inc_weight) {}

        void operator() (std::size_t i) const
        {
            using std::exp;

            inc_weight_[i] = exp(inc_weight_[i]);
        }

        private :

        double *const inc_weight_;
    }; // class vd_exp_
}; // class NormalizingConstantPPL

/// \brief Particle::value_type subtype using Parallel Pattern Library
/// \ingroup SMP
template <typename BaseState>
class StatePPL : public BaseState
{
    public :

    typedef typename traits::SizeTypeTrait<BaseState>::type size_type;

    explicit StatePPL (size_type N) : BaseState(N) {}

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

        copy_work_ (StatePPL<BaseState> *state, const IntType *copy_from) :
            state_(state), copy_from_(copy_from) {}

        void operator() (size_type to) const
        {state_->copy_particle(copy_from_[to], to);}

        private :

        StatePPL<BaseState> *const state_;
        const IntType *const copy_from_;
    }; // class copy_work_
}; // class StatePPL

/// \brief Sampler<T>::init_type subtype using Parallel Pattern Library
/// \ingroup SMP
template <typename T, typename Derived>
class InitializePPL : public InitializeBase<T, Derived>
{
    public :


    std::size_t operator() (Particle<T> &particle, void *param)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        ppl::combinable<std::size_t> accept(accept_init_);
        ppl::parallel_for(static_cast<size_type>(0), N,
                work_(this, &particle, &accept));
        this->post_processor(particle);

        return accept.combine(accept_accu_);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY_BASE(PPL, Initialize)

    private :

    class work_
    {
        public :

        typedef typename Particle<T>::size_type size_type;

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

/// \brief Sampler<T>::move_type subtype using Parallel Pattern Library
/// \ingroup SMP
template <typename T, typename Derived>
class MovePPL : public MoveBase<T, Derived>
{
    public :


    std::size_t operator() (std::size_t iter, Particle<T> &particle)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        ppl::combinable<std::size_t> accept(accept_init_);
        ppl::parallel_for(static_cast<size_type>(0), N,
                work_(this, iter, &particle, &accept));
        this->post_processor(iter, particle);

        return accept.combine(accept_accu_);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY_BASE(PPL, Move)

    private :

    class work_
    {
        public :

        typedef typename Particle<T>::size_type size_type;

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

/// \brief Monitor<T>::eval_type subtype using Parallel Pattern Library
/// \ingroup SMP
template <typename T, typename Derived>
class MonitorEvalPPL : public MonitorEvalBase<T, Derived>
{
    public :


    void operator() (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        ppl::parallel_for(static_cast<size_type>(0), N,
                work_(this, iter, dim, &particle, res));
        this->post_processor(iter, particle);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY_BASE(PPL, MonitorEval)

    private :

    class work_
    {
        public :

        typedef typename Particle<T>::size_type size_type;

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

/// \brief Path<T>::eval_type subtype using Parallel Pattern Library
/// \ingroup SMP
template <typename T, typename Derived>
class PathEvalPPL : public PathEvalBase<T, Derived>
{
    public :


    double operator() (std::size_t iter, const Particle<T> &particle,
            double *res)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        ppl::parallel_for(static_cast<size_type>(0), N,
                work_(this, iter, &particle, res));
        this->post_processor(iter, particle);

        return this->path_grid(iter, particle);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY_BASE(PPL, PathEval)

    private :

    class work_
    {
        public :

        typedef typename Particle<T>::size_type size_type;

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

#endif // VSMC_SMP_BACKEND_PPL_HPP

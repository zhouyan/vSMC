#ifndef VSMC_SMP_BACKEND_GCD_HPP
#define VSMC_SMP_BACKEND_GCD_HPP

#include <vsmc/smp/backend_base.hpp>
#include <vsmc/utility/dispatch.hpp>

namespace vsmc {

VSMC_DEFINE_SMP_FORWARD(GCD)

/// \brief Particle::weight_set_type subtype using Apple Grand Central Dispatch
/// \ingroup GCD
template <typename BaseState>
class WeightSetGCD : public traits::WeightSetTypeTrait<BaseState>::type
{
    typedef typename traits::WeightSetTypeTrait<BaseState>::type base;

    public :

    typedef typename traits::SizeTypeTrait<base>::type size_type;

    explicit WeightSetGCD (size_type N) : base(N) {}

    protected :

    void log_weight2weight ()
    {
        using std::exp;

        const size_type N = static_cast<size_type>(this->size());
        work_param_ wp = {this->weight_ptr(), this->log_weight_ptr()};
        queue_.apply_f(N, (void *) &wp, log_weight2weight_);
    }

    void weight2log_weight ()
    {
        using std::log;

        const size_type N = static_cast<size_type>(this->size());
        work_param_ wp = {this->weight_ptr(), this->log_weight_ptr()};
        queue_.apply_f(N, (void *) &wp, weight2log_weight_);
    }

    private :

    DispatchQueue<DispatchGlobal> queue_;

    struct work_param_
    {
        double *const weight;
        double *const log_weight;
    };

    static void log_weight2weight_ (void *wp, std::size_t i)
    {
        using std::exp;

        const work_param_ *const wptr = static_cast<const work_param_ *>(wp);
        wptr->weight[i] = exp(wptr->log_weight[i]);
    }

    static void weight2log_weight_ (void *wp, std::size_t i)
    {
        using std::log;

        const work_param_ *const wptr = static_cast<const work_param_ *>(wp);
        wptr->log_weight[i] = log(wptr->weight[i]);
    }
}; // class WeightSetGCD

/// \brief Calculating normalizing constant ratio using Apple Grand Central
/// Dispatch
/// \ingroup GCD
class NormalizingConstantGCD : public NormalizingConstant
{
    public :

    NormalizingConstantGCD (std::size_t N) : NormalizingConstant(N) {}

    protected:

    void vd_exp (std::size_t N, double *inc_weight) const
    {queue_.apply_f(N, (void *) inc_weight, vd_exp_);}

    private :

    DispatchQueue<DispatchGlobal> queue_;

    static void vd_exp_ (void *wp, std::size_t i)
    {
        using std::exp;

        double *const iptr = static_cast<double *>(wp);
        iptr[i] = exp(iptr[i]);
    }
}; // class NormalizingConstantGCD

/// \brief Particle::value_type subtype usingt Apple Grand Central Dispatch
/// \ingroup GCD
template <typename BaseState>
class StateGCD : public BaseState
{
    public :

    typedef typename traits::SizeTypeTrait<BaseState>::type size_type;

    explicit StateGCD (size_type N) : BaseState(N) {}
}; // class StateGCD

/// \brief Sampler<T>::init_type subtype usingt Apple Grand Central Dispatch
/// \ingroup GCD
template <typename T, typename Derived>
class InitializeGCD : public InitializeBase<T, Derived>
{
    public :

    std::size_t operator() (Particle<T> &particle, void *param)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        accept_.resize(N);
        work_param_ wp = {this, &particle, &accept_[0]};
        queue_.apply_f(N, (void *) &wp, work_);
        this->post_processor(particle);

        std::size_t acc = 0;
        for (size_type i = 0; i != N; ++i)
            acc += accept_[i];

        return acc;
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY_BASE(GCD, Initialize)

    private :

    DispatchQueue<DispatchGlobal> queue_;
    std::vector<std::size_t> accept_;

    struct work_param_
    {
        InitializeGCD<T, Derived> *const dispatcher;
        Particle<T> *const particle;
        std::size_t *const accept;
    };

    static void work_ (void *wp, std::size_t i)
    {
        typedef typename Particle<T>::size_type size_type;
        const work_param_ *const wptr = static_cast<const work_param_ *>(wp);
        wptr->accept[i] = wptr->dispatcher->initialize_state(
                SingleParticle<T>(static_cast<size_type>(i), wptr->particle));
    }
}; // class InitializeGCD

/// \brief Sampler<T>::move_type subtype usingt Apple Grand Central Dispatch
/// \ingroup GCD
template <typename T, typename Derived>
class MoveGCD : public MoveBase<T, Derived>
{
    public :


    std::size_t operator() (std::size_t iter, Particle<T> &particle)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        accept_.resize(N);
        work_param_ wp = {this, &particle, &accept_[0], iter};
        queue_.apply_f(N, (void *) &wp, work_);
        this->post_processor(iter, particle);

        std::size_t acc = 0;
        for (size_type i = 0; i != N; ++i)
            acc += accept_[i];

        return acc;
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY_BASE(GCD, Move)

    private :

    DispatchQueue<DispatchGlobal> queue_;
    std::vector<std::size_t> accept_;

    struct work_param_
    {
        MoveGCD<T, Derived> *const dispatcher;
        Particle<T> *const particle;
        std::size_t *const accept;
        std::size_t iter;
    };

    static void work_ (void *wp, std::size_t i)
    {
        typedef typename Particle<T>::size_type size_type;
        const work_param_ *const wptr = static_cast<const work_param_ *>(wp);
        wptr->accept[i] = wptr->dispatcher->move_state(wptr->iter,
                SingleParticle<T>(static_cast<size_type>(i), wptr->particle));
    }
}; // class MoveGCD

/// \brief Monitor<T>::eval_type subtype usingt Apple Grand Central Dispatch
/// \ingroup GCD
template <typename T, typename Derived>
class MonitorEvalGCD : public MonitorEvalBase<T, Derived>
{
    public :


    void operator() (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        work_param_ wp = {this, &particle, res, iter, dim};
        queue_.apply_f(N, (void *) &wp, work_);
        this->post_processor(iter, particle);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY_BASE(GCD, MonitorEval)

    private :

    DispatchQueue<DispatchGlobal> queue_;

    struct work_param_
    {
        MonitorEvalGCD<T, Derived> *const dispatcher;
        const Particle<T> *const particle;
        double *const res;
        std::size_t iter;
        std::size_t dim;
    };

    static void work_ (void *wp, std::size_t i)
    {
        typedef typename Particle<T>::size_type size_type;
        const work_param_ *const wptr = static_cast<const work_param_ *>(wp);
        wptr->dispatcher->monitor_state(wptr->iter, wptr->dim,
                ConstSingleParticle<T>(
                    static_cast<size_type>(i), wptr->particle),
                wptr->res + i * wptr->dim);
    }
}; // class MonitorEvalGCD

/// \brief Path<T>::eval_type subtype usingt Apple Grand Central Dispatch
/// \ingroup GCD
template <typename T, typename Derived>
class PathEvalGCD : public PathEvalBase<T, Derived>
{
    public :


    double operator() (std::size_t iter, const Particle<T> &particle,
            double *res)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        work_param_ wp = {this, &particle, res, iter};
        queue_.apply_f(N, (void *) &wp, work_);
        this->post_processor(iter, particle);

        return this->path_grid(iter, particle);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY_BASE(GCD, PathEval)

    private :

    DispatchQueue<DispatchGlobal> queue_;

    struct work_param_
    {
        PathEvalGCD<T, Derived> *const dispatcher;
        const Particle<T> *const particle;
        double *const res;
        std::size_t iter;
    };

    static void work_ (void *wp, std::size_t i)
    {
        typedef typename Particle<T>::size_type size_type;
        const work_param_ *const wptr = static_cast<const work_param_ *>(wp);
        wptr->res[i] = wptr->dispatcher->path_state(wptr->iter,
                ConstSingleParticle<T>(
                    static_cast<size_type>(i), wptr->particle));
    }
}; // class PathEvalGCD

}

#endif // VSMC_SMP_BACKEND_GCD_HPP

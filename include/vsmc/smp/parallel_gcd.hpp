#ifndef VSMC_SMP_PARALLEL_GCD_HPP
#define VSMC_SMP_PARALLEL_GCD_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/smp/base.hpp>
#include <dispatch/dispatch.h>

namespace vsmc {

#if !VSMC_HAS_CXX11_ALIAS_TEMPLATES
/// \brief Particle::value_type subtype
/// \ingroup GCD
template <unsigned Dim, typename T>
class StateGCD : public StateBase<Dim, T>
{
    public :

    typedef StateBase<Dim, T> state_base_type;
    typedef typename state_base_type::size_type size_type;
    typedef typename state_base_type::state_type state_type;

    explicit StateGCD (size_type N) : StateBase<Dim, T>(N) {}
}; // class StateGCD
#endif // VSMC_HAS_CXX11_ALIAS_TEMPLATES

/// \brief Sampler<T>::init_type subtype
/// \ingroup GCD
template <typename T, typename Derived>
class InitializeGCD : public InitializeBase<T, Derived>
{
    public :

    typedef InitializeBase<T, Derived> initialize_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (Particle<T> &particle, void *param)
    {
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        accept_.resize(particle.size());
        work_info_ info = {this, &particle};
        dispatch_apply_f(particle.size(),
                dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0),
                (void *) &info, work_);
        this->post_processor(particle);

        return std::accumulate(accept_.begin(), accept_.end(),
                static_cast<unsigned>(0));
    }

    protected :

    InitializeGCD () {}
    InitializeGCD (const InitializeGCD<T, Derived> &) {}
    InitializeGCD<T, Derived> &operator=
        (const InitializeGCD<T, Derived> &) {return *this;}
    ~InitializeGCD () {}

    private :

    std::vector<unsigned> accept_;

    struct work_info_
    {
        InitializeGCD<T, Derived> *const dispatcher;
        Particle<T> *const particle;
    };

    static void work_ (void *info, std::size_t i)
    {
        const work_info_ *const wptr =
            reinterpret_cast<const work_info_ *>(info);
        wptr->dispatcher->accept_[i] =
            wptr->dispatcher->initialize_state(SingleParticle<T>(
                        static_cast<size_type>(i), wptr->particle));
    }
}; // class InitializeGCD

/// \brief Sampler<T>::move_type subtype
/// \ingroup GCD
template <typename T, typename Derived>
class MoveGCD : public MoveBase<T, Derived>
{
    public :

    typedef MoveBase<T, Derived> move_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        this->pre_processor(iter, particle);
        iter_ = iter;
        accept_.resize(particle.size());
        work_info_ info = {this, &particle};
        dispatch_apply_f(particle.size(),
                dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0),
                (void *) &info, work_);
        this->post_processor(iter, particle);

        return std::accumulate(accept_.begin(), accept_.end(),
                static_cast<unsigned>(0));
    }

    protected :

    MoveGCD () {}
    MoveGCD (const MoveGCD<T, Derived> &) {}
    MoveGCD<T, Derived> &operator=
        (const MoveGCD<T, Derived> &) {return *this;}
    ~MoveGCD () {}

    private :

    unsigned iter_;
    std::vector<unsigned> accept_;

    struct work_info_
    {
        MoveGCD<T, Derived> *const dispatcher;
        Particle<T> *const particle;
    };

    static void work_ (void *info, std::size_t i)
    {
        const work_info_ *const wptr =
            reinterpret_cast<const work_info_ *>(info);
        wptr->dispatcher->accept_[i] =
            wptr->dispatcher->move_state(wptr->dispatcher->iter_,
                    SingleParticle<T>(
                        static_cast<size_type>(i), wptr->particle));
    }
}; // class MoveGCD

/// \brief Monitor<T>::eval_type subtype
/// \ingroup GCD
template <typename T, typename Derived>
class MonitorEvalGCD : public MonitorEvalBase<T, Derived>
{
    public :

    typedef MonitorEvalBase<T, Derived> monitor_eval_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    void operator() (unsigned iter, unsigned dim, const Particle<T> &particle,
            double *res)
    {
        this->pre_processor(iter, particle);
        iter_ = iter;
        dim_ = dim;
        res_ = res;
        work_info_ info = {this, &particle};
        dispatch_apply_f(particle.size(),
                dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0),
                (void *) &info, work_);
        this->post_processor(iter, particle);
    }

    protected :

    MonitorEvalGCD () {}
    MonitorEvalGCD (const MonitorEvalGCD<T, Derived> &) {}
    MonitorEvalGCD<T, Derived> &operator=
        (const MonitorEvalGCD<T, Derived> &) {return *this;}
    ~MonitorEvalGCD () {}

    private :

    unsigned iter_;
    unsigned dim_;
    double *res_;

    struct work_info_
    {
        MonitorEvalGCD<T, Derived> *const dispatcher;
        const Particle<T> *const particle;
    };

    static void work_ (void *info, std::size_t i)
    {
        const work_info_ *const wptr =
            reinterpret_cast<const work_info_ *>(info);
        wptr->dispatcher->monitor_state(
                wptr->dispatcher->iter_, wptr->dispatcher->dim_,
                ConstSingleParticle<T>(
                    static_cast<size_type>(i), wptr->particle),
                wptr->dispatcher->res_ + i * wptr->dispatcher->dim_);
    }
}; // class MonitorEvalGCD

/// \brief Path<T>::eval_type subtype
/// \ingroup GCD
template <typename T, typename Derived>
class PathEvalGCD : public PathEvalBase<T, Derived>
{
    public :

    typedef PathEvalBase<T, Derived> path_eval_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    double operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        this->pre_processor(iter, particle);
        iter_ = iter;
        res_ = res;
        work_info_ info = {this, &particle};
        dispatch_apply_f(particle.size(),
                dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0),
                (void *) &info, work_);
        this->post_processor(iter, particle);

        return this->path_width(iter, particle);
    }

    protected :

    PathEvalGCD () {}
    PathEvalGCD (const PathEvalGCD<T, Derived> &) {}
    PathEvalGCD<T, Derived> &operator=
        (const PathEvalGCD<T, Derived> &) {return *this;}
    ~PathEvalGCD () {}

    private :

    unsigned iter_;
    double *res_;

    struct work_info_
    {
        PathEvalGCD<T, Derived> *const dispatcher;
        const Particle<T> *const particle;
    };

    static void work_ (void *info, std::size_t i)
    {
        const work_info_ *const wptr =
            reinterpret_cast<const work_info_ *>(info);
        wptr->dispatcher->res_[i] = wptr->dispatcher->path_state(
                wptr->dispatcher->iter_,
                ConstSingleParticle<T>(
                    static_cast<size_type>(i), wptr->particle));
    }
}; // class PathEvalGCD

}

#endif // VSMC_SMP_PARALLEL_GCD_HPP

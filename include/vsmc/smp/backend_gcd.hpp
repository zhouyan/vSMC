//============================================================================
// vSMC/include/vsmc/smp/backend_gcd.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_SMP_BACKEND_GCD_HPP
#define VSMC_SMP_BACKEND_GCD_HPP

#include <vsmc/smp/backend_base.hpp>
#include <vsmc/gcd/gcd.hpp>

namespace vsmc {

VSMC_DEFINE_SMP_FORWARD(GCD)

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
        work_param_ wp(this, &particle, &accept_[0]);
        queue_.apply_f(N, &wp, work_);
        this->post_processor(particle);

        std::size_t acc = 0;
        for (size_type i = 0; i != N; ++i)
            acc += accept_[i];

        return acc;
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(GCD, Initialize)

    private :

    DispatchQueue<DispatchGlobal> queue_;
    std::vector<std::size_t> accept_;

    struct work_param_
    {
        work_param_ (InitializeGCD<T, Derived> *dptr, Particle<T> *pptr,
                std::size_t *aptr) :
            dispatcher(dptr), particle(pptr), accept(aptr) {}

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
        work_param_ wp(this, &particle, &accept_[0], iter);
        queue_.apply_f(N, &wp, work_);
        this->post_processor(iter, particle);

        std::size_t acc = 0;
        for (size_type i = 0; i != N; ++i)
            acc += accept_[i];

        return acc;
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(GCD, Move)

    private :

    DispatchQueue<DispatchGlobal> queue_;
    std::vector<std::size_t> accept_;

    struct work_param_
    {
        work_param_ (MoveGCD<T, Derived> *dptr, Particle<T> *pptr,
                std::size_t *aptr, std::size_t i) :
            dispatcher(dptr), particle(pptr), accept(aptr), iter(i) {}

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
        work_param_ wp(this, &particle, res, iter, dim);
        queue_.apply_f(N, &wp, work_);
        this->post_processor(iter, particle);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(GCD, MonitorEval)

    private :

    DispatchQueue<DispatchGlobal> queue_;

    struct work_param_
    {
        work_param_ (MonitorEvalGCD<T, Derived> *dptr, const Particle<T> *pptr,
                double *rptr, std::size_t i, std::size_t d) :
            dispatcher(dptr), particle(pptr), res(rptr), iter(i), dim(d) {}

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
        work_param_ wp(this, &particle, res, iter);
        queue_.apply_f(N, &wp, work_);
        this->post_processor(iter, particle);

        return this->path_grid(iter, particle);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(GCD, PathEval)

    private :

    DispatchQueue<DispatchGlobal> queue_;

    struct work_param_
    {
        work_param_ (PathEvalGCD<T, Derived> *dptr, const Particle<T> *pptr,
                double *rptr, std::size_t i) :
            dispatcher(dptr), particle(pptr), res(rptr), iter(i) {}

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

} // namespace vsmc

#endif // VSMC_SMP_BACKEND_GCD_HPP

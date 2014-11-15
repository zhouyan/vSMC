//============================================================================
// vSMC/include/vsmc/smp/backend_std.hpp
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

#ifndef VSMC_SMP_BACKEND_STD_HPP
#define VSMC_SMP_BACKEND_STD_HPP

#include <vsmc/smp/backend_base.hpp>
#include <vsmc/thread/thread.hpp>

namespace vsmc {

VSMC_DEFINE_SMP_FORWARD(STD)

/// \brief Particle::value_type subtype using C++11 concurrency
/// \ingroup STD
template <typename BaseState>
class StateSTD : public BaseState
{
    public :

    typedef typename traits::SizeTypeTrait<BaseState>::type size_type;

    explicit StateSTD (size_type N) : BaseState(N) {}

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_SMP_BACKEND_BASE_COPY_SIZE_MISMATCH(STD);

        parallel_for(BlockedRange<size_type>(0, N),
                copy_work_<IntType>(this, copy_from));
    }

    private :

    template <typename IntType>
    struct copy_work_
    {
        copy_work_ (StateSTD<BaseState> *state, const IntType *copy_from) :
            state_(state), copy_from_(copy_from) {}

        void operator() (const BlockedRange<size_type> &range) const
        {
            for (size_type to = range.begin(); to != range.end(); ++to)
                state_->copy_particle(copy_from_[to], to);
        }

        private :

        StateSTD<BaseState> *const state_;
        const IntType *const copy_from_;
    }; // class copy_work_
}; // class StateSTD

/// \brief Sampler<T>::init_type subtype using C++11 concurrency
/// \ingroup STD
template <typename T, typename Derived>
class InitializeSTD : public InitializeBase<T, Derived>
{
    public :

    std::size_t operator() (Particle<T> &particle, void *param)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        std::size_t accept = parallel_accumulate(BlockedRange<size_type>(0, N),
                work_(this, &particle), static_cast<std::size_t>(0));
        this->post_processor(particle);

        return accept;
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(STD, Initialize)

    private :

    struct work_
    {
        typedef typename Particle<T>::size_type size_type;

        work_ (InitializeSTD<T, Derived> *init,
                Particle<T> *particle) :
            init_(init), particle_(particle) {}

        void operator() (const BlockedRange<size_type> &range,
                std::size_t &accept) const
        {
            Particle<T> *const part = particle_;
            std::size_t acc = 0;
            for (size_type i = range.begin(); i != range.end(); ++i)
                acc += init_->initialize_state(SingleParticle<T>(i, part));
            accept = acc;
        }

        private :

        InitializeSTD<T, Derived> *const init_;
        Particle<T> *const particle_;
    }; // class work_
}; // class InitializeSTD

/// \brief Sampler<T>::move_type subtype using C++11 concurrency
/// \ingroup STD
template <typename T, typename Derived>
class MoveSTD : public MoveBase<T, Derived>
{
    public :

    std::size_t operator() (std::size_t iter, Particle<T> &particle)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        std::size_t accept = parallel_accumulate(BlockedRange<size_type>(0, N),
                work_(this, iter, &particle), static_cast<std::size_t>(0));
        this->post_processor(iter, particle);

        return accept;
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(STD, Move)

    private :

    struct work_
    {
        typedef typename Particle<T>::size_type size_type;

        work_ (MoveSTD<T, Derived> *move, std::size_t iter,
                Particle<T> *particle):
            move_(move), iter_(iter), particle_(particle) {}

        void operator() (const BlockedRange<size_type> &range,
                std::size_t &accept) const
        {
            Particle<T> *const part = particle_;
            const std::size_t iter = iter_;
            std::size_t acc = 0;
            for (size_type i = range.begin(); i != range.end(); ++i)
                acc += move_->move_state(iter, SingleParticle<T>(i, part));
            accept = acc;
        }

        private :

        MoveSTD<T, Derived> *const move_;
        const std::size_t iter_;
        Particle<T> *const particle_;
    }; // class work_
}; // class MoveSTD

/// \brief Monitor<T>::eval_type subtype using C++11 concurrency
/// \ingroup STD
template <typename T, typename Derived>
class MonitorEvalSTD : public MonitorEvalBase<T, Derived>
{
    public :

    void operator() (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        parallel_for(BlockedRange<size_type>(0, N),
                work_(this, iter, dim, &particle, res));
        this->post_processor(iter, particle);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(STD, MonitorEval)

    private :

    struct work_
    {
        typedef typename Particle<T>::size_type size_type;

        work_ (MonitorEvalSTD<T, Derived> *monitor,
                std::size_t iter, std::size_t dim,
                const Particle<T> *particle, double *res) :
            monitor_(monitor), iter_(iter), dim_(dim), particle_(particle),
            res_(res) {}

        void operator() (const BlockedRange<size_type> &range) const
        {
            const Particle<T> *const part = particle_;
            const std::size_t iter = iter_;
            const std::size_t dim = dim_;
            double *const res = res_;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                double *const r = res + i * dim;
                monitor_->monitor_state(iter, dim,
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

/// \brief Path<T>::eval_type subtype using C++11 concurrency
/// \ingroup STD
template <typename T, typename Derived>
class PathEvalSTD : public PathEvalBase<T, Derived>
{
    public :

    double operator() (std::size_t iter, const Particle<T> &particle,
            double *res)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        parallel_for(BlockedRange<size_type>(0, N),
                work_(this, iter, &particle, res));
        this->post_processor(iter, particle);

        return this->path_grid(iter, particle);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(STD, PathEval)

    private :

    struct work_
    {
        typedef typename Particle<T>::size_type size_type;

        work_ (PathEvalSTD<T, Derived> *path, std::size_t iter,
                const Particle<T> *particle, double *res) :
            path_(path), iter_(iter), particle_(particle), res_(res) {}

        void operator() (const BlockedRange<size_type> &range) const
        {
            const Particle<T> *const part = particle_;
            const std::size_t iter = iter_;
            double *const res = res_;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                res[i] = path_->path_state(iter,
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

#endif // VSMC_SMP_BACKEND_STD_HPP

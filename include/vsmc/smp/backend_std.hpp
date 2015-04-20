//============================================================================
// vSMC/include/vsmc/smp/backend_std.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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
#include <vsmc/smp/internal/parallel_work.hpp>
#include <vsmc/thread/thread.hpp>

namespace vsmc
{

VSMC_DEFINE_SMP_FORWARD(STD)

/// \brief Particle::value_type subtype using C++11 concurrency
/// \ingroup STD
template <typename BaseState> class StateSTD : public BaseState
{
    public:
    typedef typename traits::SizeTypeTrait<BaseState>::type size_type;

    explicit StateSTD(size_type N) : BaseState(N) {}

    template <typename IntType>
    void copy(size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_SMP_BACKEND_BASE_COPY_SIZE_MISMATCH(STD);

        parallel_for(BlockedRange<size_type>(0, N),
            internal::ParallelCopyParticle<StateSTD<BaseState>, IntType>(
                         this, copy_from));
    }
}; // class StateSTD

/// \brief Sampler<T>::init_type subtype using C++11 concurrency
/// \ingroup STD
template <typename T, typename Derived>
class InitializeSTD : public InitializeBase<T, Derived>
{
    public:
    std::size_t operator()(Particle<T> &particle, void *param)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        internal::ParallelInitializeState<T, InitializeSTD<T, Derived>> work(
            this, &particle);
        parallel_reduce(BlockedRange<size_type>(0, N), work);
        this->post_processor(particle);

        return work.accept();
    }

    protected:
    VSMC_DEFINE_SMP_IMPL_COPY(STD, Initialize)
}; // class InitializeSTD

/// \brief Sampler<T>::move_type subtype using C++11 concurrency
/// \ingroup STD
template <typename T, typename Derived>
class MoveSTD : public MoveBase<T, Derived>
{
    public:
    std::size_t operator()(std::size_t iter, Particle<T> &particle)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        internal::ParallelMoveState<T, MoveSTD<T, Derived>> work(
            this, iter, &particle);
        parallel_reduce(BlockedRange<size_type>(0, N), work);
        this->post_processor(iter, particle);

        return work.accept();
    }

    protected:
    VSMC_DEFINE_SMP_IMPL_COPY(STD, Move)
}; // class MoveSTD

/// \brief Monitor<T>::eval_type subtype using C++11 concurrency
/// \ingroup STD
template <typename T, typename Derived>
class MonitorEvalSTD : public MonitorEvalBase<T, Derived>
{
    public:
    void operator()(std::size_t iter, std::size_t dim,
        const Particle<T> &particle, double *res)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        parallel_for(BlockedRange<size_type>(0, N),
            internal::ParallelMonitorState<T, MonitorEvalSTD<T, Derived>>(
                         this, iter, dim, &particle, res));
        this->post_processor(iter, particle);
    }

    protected:
    VSMC_DEFINE_SMP_IMPL_COPY(STD, MonitorEval)
}; // class MonitorEvalSTD

/// \brief Path<T>::eval_type subtype using C++11 concurrency
/// \ingroup STD
template <typename T, typename Derived>
class PathEvalSTD : public PathEvalBase<T, Derived>
{
    public:
    double operator()(
        std::size_t iter, const Particle<T> &particle, double *res)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        parallel_for(BlockedRange<size_type>(0, N),
            internal::ParallelPathState<T, PathEvalSTD<T, Derived>>(
                         this, iter, &particle, res));
        this->post_processor(iter, particle);

        return this->path_grid(iter, particle);
    }

    protected:
    VSMC_DEFINE_SMP_IMPL_COPY(STD, PathEval)
}; // PathEvalSTD

} // namespace vsmc

#endif // VSMC_SMP_BACKEND_STD_HPP

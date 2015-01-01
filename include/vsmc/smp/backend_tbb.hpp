//============================================================================
// vSMC/include/vsmc/smp/backend_tbb.hpp
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

#ifndef VSMC_SMP_BACKEND_TBB_HPP
#define VSMC_SMP_BACKEND_TBB_HPP

#include <vsmc/smp/backend_base.hpp>
#include <vsmc/smp/internal/parallel_work.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE(args) \
this->initialize_param(particle, param);                                     \
this->pre_processor(particle);                                               \
internal::ParallelInitializeState<T, InitializeTBB<T, Derived> > work(       \
        this, &particle);                                                    \
::tbb::parallel_reduce args;                                                 \
this->post_processor(particle);                                              \
return work.accept();

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE(args) \
this->pre_processor(iter, particle);                                         \
internal::ParallelMoveState<T, MoveTBB<T, Derived> > work(                   \
        this, iter, &particle);                                              \
::tbb::parallel_reduce args;                                                 \
this->post_processor(iter, particle);                                        \
return work.accept();

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(args) \
this->pre_processor(iter, particle);                                         \
internal::ParallelMonitorState<T, MonitorEvalTBB<T, Derived> > work(         \
        this, iter, dim, &particle, res);                                    \
::tbb::parallel_for args;                                                    \
this->post_processor(iter, particle);

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(args) \
this->pre_processor(iter, particle);                                         \
internal::ParallelPathState<T, PathEvalTBB<T, Derived> > work(               \
        this, iter, &particle, res);                                         \
::tbb::parallel_for args;                                                    \
this->post_processor(iter, particle);                                        \
return this->path_grid(iter, particle);

namespace vsmc {

VSMC_DEFINE_SMP_FORWARD(TBB)

/// \brief Particle::value_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename BaseState>
class StateTBB : public BaseState
{
    public :

    typedef typename traits::SizeTypeTrait<BaseState>::type size_type;

    explicit StateTBB (size_type N) : BaseState(N) {}

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_SMP_BACKEND_BASE_COPY_SIZE_MISMATCH(TBB);
        parallel_copy_run(copy_from, ::tbb::blocked_range<size_type>(0, N));
    }

    protected :

    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range)
    {
        ::tbb::parallel_for(range, internal::ParallelCopyParticle<
                StateTBB<BaseState>, IntType>(this, copy_from));
    }

    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range,
            const ::tbb::auto_partitioner &partitioner)
    {
        ::tbb::parallel_for(range, internal::ParallelCopyParticle<
                StateTBB<BaseState>, IntType>(this, copy_from), partitioner);
    }

    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range,
            const ::tbb::simple_partitioner &partitioner)
    {
        ::tbb::parallel_for(range, internal::ParallelCopyParticle<
                StateTBB<BaseState>, IntType>(this, copy_from), partitioner);
    }

    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range,
            ::tbb::affinity_partitioner &partitioner)
    {
        ::tbb::parallel_for(range, internal::ParallelCopyParticle<
                StateTBB<BaseState>, IntType>(this, copy_from), partitioner);
    }

#if __TBB_TASK_GROUP_CONTEXT
    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range,
            const ::tbb::auto_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        ::tbb::parallel_for(range, internal::ParallelCopyParticle<
                StateTBB<BaseState>, IntType>(this, copy_from), partitioner,
                context);
    }

    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range,
            const ::tbb::simple_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        ::tbb::parallel_for(range, internal::ParallelCopyParticle<
                StateTBB<BaseState>, IntType>(this, copy_from), partitioner,
                context);
    }

    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range,
            ::tbb::affinity_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        ::tbb::parallel_for(range, internal::ParallelCopyParticle<
                StateTBB<BaseState>, IntType>(this, copy_from), partitioner,
                context);
    }
#endif // __TBB_TASK_GROUP_CONTEXT
}; // class StateTBB

/// \brief Sampler<T>::init_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
class InitializeTBB : public InitializeBase<T, Derived>
{
    public :

    std::size_t operator() (Particle<T> &particle, void *param)
    {
        return parallel_run(particle, param,
                ::tbb::blocked_range<typename Particle<T>::size_type>(
                    0, particle.size()));
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(TBB, Initialize)

    std::size_t parallel_run (Particle<T> &particle, void *param,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range)
    {VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE((range, work));}

    std::size_t parallel_run (Particle<T> &particle, void *param,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::auto_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE((
                    range, work, partitioner));
    }

    std::size_t parallel_run (Particle<T> &particle, void *param,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::simple_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE((
                    range, work, partitioner));
    }

    std::size_t parallel_run (Particle<T> &particle, void *param,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            ::tbb::affinity_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE((
                    range, work, partitioner));
    }

#if __TBB_TASK_GROUP_CONTEXT
    std::size_t parallel_run (Particle<T> &particle, void *param,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::auto_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE((
                    range, work, partitioner, context));
    }

    std::size_t parallel_run (Particle<T> &particle, void *param,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::simple_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE((
                    range, work, partitioner, context));
    }

    std::size_t parallel_run (Particle<T> &particle, void *param,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            ::tbb::affinity_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE((
                    range, work, partitioner, context));
    }
#endif // __TBB_TASK_GROUP_CONTEXT
}; // class InitializeTBB

/// \brief Sampler<T>::move_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
class MoveTBB : public MoveBase<T, Derived>
{
    public :

    std::size_t operator() (std::size_t iter, Particle<T> &particle)
    {
        return parallel_run(iter, particle,
                ::tbb::blocked_range<typename Particle<T>::size_type>(
                    0, particle.size()));
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(TBB, Move)

    std::size_t parallel_run (std::size_t iter, Particle<T> &particle,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range)
    {VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE((range, work));}

    std::size_t parallel_run (std::size_t iter, Particle<T> &particle,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::auto_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE((
                    range, work, partitioner));
    }

    std::size_t parallel_run (std::size_t iter, Particle<T> &particle,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::simple_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE((
                    range, work, partitioner));
    }

    std::size_t parallel_run (std::size_t iter, Particle<T> &particle,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            ::tbb::affinity_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE((
                    range, work, partitioner));
    }

#if __TBB_TASK_GROUP_CONTEXT
    std::size_t parallel_run (std::size_t iter, Particle<T> &particle,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::auto_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE((
                    range, work, partitioner, context));
    }

    std::size_t parallel_run (std::size_t iter, Particle<T> &particle,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::simple_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE((
                    range, work, partitioner, context));
    }

    std::size_t parallel_run (std::size_t iter, Particle<T> &particle,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            ::tbb::affinity_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE((
                    range, work, partitioner, context));
    }
#endif // __TBB_TASK_GROUP_CONTEXT
}; // class MoveTBB

/// \brief Monitor<T>::eval_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
class MonitorEvalTBB : public MonitorEvalBase<T, Derived>
{
    public :

    void operator() (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res)
    {
        parallel_run(iter, dim, particle, res,
                ::tbb::blocked_range<typename Particle<T>::size_type>(
                    0, particle.size()));
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(TBB, MonitorEval)

    void parallel_run (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range)
    {VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL((range, work));}

    void parallel_run (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::auto_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL((
                    range, work, partitioner));
    }

    void parallel_run (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::simple_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL((
                    range, work, partitioner));
    }

    void parallel_run (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            ::tbb::affinity_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL((
                    range, work, partitioner));
    }

#if __TBB_TASK_GROUP_CONTEXT
    void parallel_run (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::auto_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL((
                    range, work, partitioner, context));
    }

    void parallel_run (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::simple_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL((
                    range, work, partitioner, context));
    }

    void parallel_run (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            ::tbb::affinity_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL((
                    range, work, partitioner, context));
    }
#endif // __TBB_TASK_GROUP_CONTEXT
}; // class MonitorEvalTBB

/// \brief Path<T>::eval_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
class PathEvalTBB : public PathEvalBase<T, Derived>
{
    public :

    double operator() (std::size_t iter, const Particle<T> &particle,
            double *res)
    {
        return parallel_run(iter, particle, res,
                ::tbb::blocked_range<typename Particle<T>::size_type>(
                    0, particle.size()));
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(TBB, PathEval)

    double parallel_run (std::size_t iter, const Particle<T> &particle,
            double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range)
    {VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL((range, work));}

    double parallel_run (std::size_t iter, const Particle<T> &particle,
            double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::auto_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL((
                    range, work, partitioner));
    }

    double parallel_run (std::size_t iter, const Particle<T> &particle,
            double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::simple_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL((
                    range, work, partitioner));
    }

    double parallel_run (std::size_t iter, const Particle<T> &particle,
            double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            ::tbb::affinity_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL((
                    range, work, partitioner));
    }

#if __TBB_TASK_GROUP_CONTEXT
    double parallel_run (std::size_t iter, const Particle<T> &particle,
            double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::auto_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL((
                    range, work, partitioner, context));
    }

    double parallel_run (std::size_t iter, const Particle<T> &particle,
            double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            const ::tbb::simple_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL((
                    range, work, partitioner, context));
    }

    double parallel_run (std::size_t iter, const Particle<T> &particle,
            double *res,
            const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
            ::tbb::affinity_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL((
                    range, work, partitioner, context));
    }
#endif // __TBB_TASK_GROUP_CONTEXT
}; // PathEvalTBB

} // namespace vsmc

#endif // VSMC_SMP_BACKEND_TBB_HPP

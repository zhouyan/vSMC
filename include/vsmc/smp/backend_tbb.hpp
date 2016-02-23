//============================================================================
// vSMC/include/vsmc/smp/backend_tbb.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#include <tbb/tbb.h>
#include <vsmc/smp/backend_base.hpp>

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE(args)             \
    this->eval_param(particle, param);                                        \
    this->eval_pre(particle);                                                 \
    work_type work(this, &particle);                                          \
    ::tbb::parallel_reduce args;                                              \
    this->eval_post(particle);                                                \
    return work.accept();

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE(args)                   \
    this->eval_pre(iter, particle);                                           \
    work_type work(this, iter, &particle);                                    \
    ::tbb::parallel_reduce args;                                              \
    this->eval_post(iter, particle);                                          \
    return work.accept();

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(args)           \
    this->eval_pre(iter, particle);                                           \
    work_type work(this, iter, dim, &particle, r);                            \
    ::tbb::parallel_for args;                                                 \
    this->eval_post(iter, particle);

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(args)              \
    this->eval_pre(iter, particle);                                           \
    work_type work(this, iter, &particle, r);                                 \
    ::tbb::parallel_for args;                                                 \
    this->eval_post(iter, particle);                                          \
    return this->eval_grid(iter, particle);

namespace vsmc
{

VSMC_DEFINE_SMP_BACKEND_FORWARD(TBB)

/// \brief Particle::value_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename StateBase>
class StateTBB : public StateBase
{
    public:
    using size_type = SizeType<StateBase>;

    explicit StateTBB(size_type N) : StateBase(N) {}

    template <typename IntType>
    void copy(size_type N, const IntType *src_idx)
    {
        parallel_copy_run(src_idx, ::tbb::blocked_range<size_type>(0, N));
    }

    protected:
    template <typename IntType>
    class work_type
    {
        public:
        work_type(StateTBB<StateBase> *state, const IntType *src_idx)
            : state_(state), src_idx_(src_idx)
        {
        }

        void operator()(const ::tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                state_->copy_particle(static_cast<size_type>(src_idx_[i]), i);
            }
        }

        private:
        StateTBB<StateBase> *const state_;
        const IntType *const src_idx_;
    }; // class work_type

    template <typename IntType>
    void parallel_copy_run(
        const IntType *src_idx, const ::tbb::blocked_range<size_type> &range)
    {
        ::tbb::parallel_for(range, work_type<IntType>(this, src_idx));
    }

    template <typename IntType>
    void parallel_copy_run(const IntType *src_idx,
        const ::tbb::blocked_range<size_type> &range,
        const ::tbb::auto_partitioner &partitioner)
    {
        ::tbb::parallel_for(
            range, work_type<IntType>(this, src_idx), partitioner);
    }

    template <typename IntType>
    void parallel_copy_run(const IntType *src_idx,
        const ::tbb::blocked_range<size_type> &range,
        const ::tbb::simple_partitioner &partitioner)
    {
        ::tbb::parallel_for(
            range, work_type<IntType>(this, src_idx), partitioner);
    }

    template <typename IntType>
    void parallel_copy_run(const IntType *src_idx,
        const ::tbb::blocked_range<size_type> &range,
        ::tbb::affinity_partitioner &partitioner)
    {
        ::tbb::parallel_for(
            range, work_type<IntType>(this, src_idx), partitioner);
    }

#if __TBB_TASK_GROUP_CONTEXT
    template <typename IntType>
    void parallel_copy_run(const IntType *src_idx,
        const ::tbb::blocked_range<size_type> &range,
        const ::tbb::auto_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        ::tbb::parallel_for(
            range, work_type<IntType>(this, src_idx), partitioner, context);
    }

    template <typename IntType>
    void parallel_copy_run(const IntType *src_idx,
        const ::tbb::blocked_range<size_type> &range,
        const ::tbb::simple_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        ::tbb::parallel_for(
            range, work_type<IntType>(this, src_idx), partitioner, context);
    }

    template <typename IntType>
    void parallel_copy_run(const IntType *src_idx,
        const ::tbb::blocked_range<size_type> &range,
        ::tbb::affinity_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        ::tbb::parallel_for(
            range, work_type<IntType>(this, src_idx), partitioner, context);
    }
#endif // __TBB_TASK_GROUP_CONTEXT
};     // class StateTBB

/// \brief Sampler<T>::init_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
class InitializeTBB : public InitializeBase<T, Derived>
{
    public:
    std::size_t operator()(Particle<T> &particle, void *param)
    {
        return parallel_run(particle, param,
            ::tbb::blocked_range<typename Particle<T>::size_type>(
                                0, particle.size()));
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(TBB, Initialize)

    class work_type
    {
        public:
        using size_type = typename Particle<T>::size_type;

        work_type(InitializeTBB<T, Derived> *wptr, Particle<T> *pptr)
            : wptr_(wptr), pptr_(pptr), accept_(0)
        {
        }

        work_type(const work_type &other, ::tbb::split)
            : wptr_(other.wptr_), pptr_(other.pptr_), accept_(0)
        {
        }

        void operator()(const ::tbb::blocked_range<size_type> &range)
        {
            for (size_type i = range.begin(); i != range.end(); ++i)
                accept_ += wptr_->eval_sp(pptr_->sp(i));
        }

        void join(const work_type &other) { accept_ += other.accept_; }

        std::size_t accept() const { return accept_; }

        private:
        InitializeTBB<T, Derived> *const wptr_;
        Particle<T> *const pptr_;
        std::size_t accept_;
    }; // class work_type

    std::size_t parallel_run(Particle<T> &particle, void *param,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE((range, work));
    }

    std::size_t parallel_run(Particle<T> &particle, void *param,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::auto_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE(
            (range, work, partitioner));
    }

    std::size_t parallel_run(Particle<T> &particle, void *param,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::simple_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE(
            (range, work, partitioner));
    }

    std::size_t parallel_run(Particle<T> &particle, void *param,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        ::tbb::affinity_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE(
            (range, work, partitioner));
    }

#if __TBB_TASK_GROUP_CONTEXT
    std::size_t parallel_run(Particle<T> &particle, void *param,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::auto_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE(
            (range, work, partitioner, context));
    }

    std::size_t parallel_run(Particle<T> &particle, void *param,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::simple_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE(
            (range, work, partitioner, context));
    }

    std::size_t parallel_run(Particle<T> &particle, void *param,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        ::tbb::affinity_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE(
            (range, work, partitioner, context));
    }
#endif // __TBB_TASK_GROUP_CONTEXT
};     // class InitializeTBB

/// \brief Sampler<T>::move_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
class MoveTBB : public MoveBase<T, Derived>
{
    public:
    std::size_t operator()(std::size_t iter, Particle<T> &particle)
    {
        return parallel_run(iter, particle,
            ::tbb::blocked_range<typename Particle<T>::size_type>(
                                0, particle.size()));
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(TBB, Move)

    class work_type
    {
        public:
        using size_type = typename Particle<T>::size_type;

        work_type(
            MoveTBB<T, Derived> *wptr, std::size_t iter, Particle<T> *pptr)
            : wptr_(wptr), iter_(iter), pptr_(pptr), accept_(0)
        {
        }

        work_type(const work_type &other, ::tbb::split)
            : wptr_(other.wptr_)
            , iter_(other.iter_)
            , pptr_(other.pptr_)
            , accept_(0)
        {
        }

        void operator()(const ::tbb::blocked_range<size_type> &range)
        {
            for (size_type i = range.begin(); i != range.end(); ++i)
                accept_ += wptr_->eval_sp(iter_, pptr_->sp(i));
        }

        void join(const work_type &other) { accept_ += other.accept_; }

        std::size_t accept() const { return accept_; }

        private:
        MoveTBB<T, Derived> *const wptr_;
        const std::size_t iter_;
        Particle<T> *const pptr_;
        std::size_t accept_;
    }; // class work_type

    std::size_t parallel_run(std::size_t iter, Particle<T> &particle,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE((range, work));
    }

    std::size_t parallel_run(std::size_t iter, Particle<T> &particle,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::auto_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE(
            (range, work, partitioner));
    }

    std::size_t parallel_run(std::size_t iter, Particle<T> &particle,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::simple_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE(
            (range, work, partitioner));
    }

    std::size_t parallel_run(std::size_t iter, Particle<T> &particle,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        ::tbb::affinity_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE(
            (range, work, partitioner));
    }

#if __TBB_TASK_GROUP_CONTEXT
    std::size_t parallel_run(std::size_t iter, Particle<T> &particle,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::auto_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE(
            (range, work, partitioner, context));
    }

    std::size_t parallel_run(std::size_t iter, Particle<T> &particle,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::simple_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE(
            (range, work, partitioner, context));
    }

    std::size_t parallel_run(std::size_t iter, Particle<T> &particle,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        ::tbb::affinity_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE(
            (range, work, partitioner, context));
    }
#endif // __TBB_TASK_GROUP_CONTEXT
};     // class MoveTBB

/// \brief Monitor<T>::eval_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
class MonitorEvalTBB : public MonitorEvalBase<T, Derived>
{
    public:
    void operator()(
        std::size_t iter, std::size_t dim, Particle<T> &particle, double *r)
    {
        parallel_run(iter, dim, particle, r,
            ::tbb::blocked_range<typename Particle<T>::size_type>(
                         0, particle.size()));
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(TBB, MonitorEval)

    class work_type
    {
        public:
        using size_type = typename Particle<T>::size_type;

        work_type(MonitorEvalTBB<T, Derived> *wptr, std::size_t iter,
            std::size_t dim, Particle<T> *pptr, double *r)
            : wptr_(wptr), iter_(iter), dim_(dim), pptr_(pptr), r_(r)
        {
        }

        void operator()(const ::tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                wptr_->eval_sp(iter_, dim_, pptr_->sp(i),
                    r_ + static_cast<std::size_t>(i) * dim_);
            }
        }

        private:
        MonitorEvalTBB<T, Derived> *const wptr_;
        const std::size_t iter_;
        const std::size_t dim_;
        Particle<T> *const pptr_;
        double *const r_;
    }; // class work_type

    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL((range, work));
    }

    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::auto_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(
            (range, work, partitioner));
    }

    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::simple_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(
            (range, work, partitioner));
    }

    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        ::tbb::affinity_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(
            (range, work, partitioner));
    }

#if __TBB_TASK_GROUP_CONTEXT
    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::auto_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(
            (range, work, partitioner, context));
    }

    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::simple_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(
            (range, work, partitioner, context));
    }

    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        ::tbb::affinity_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(
            (range, work, partitioner, context));
    }
#endif // __TBB_TASK_GROUP_CONTEXT
};     // class MonitorEvalTBB

/// \brief Path<T>::eval_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
class PathEvalTBB : public PathEvalBase<T, Derived>
{
    public:
    double operator()(std::size_t iter, Particle<T> &particle, double *r)
    {
        return parallel_run(iter, particle, r,
            ::tbb::blocked_range<typename Particle<T>::size_type>(
                                0, particle.size()));
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(TBB, PathEval)

    class work_type
    {
        public:
        using size_type = typename Particle<T>::size_type;

        work_type(PathEvalTBB<T, Derived> *wptr, std::size_t iter,
            Particle<T> *pptr, double *r)
            : wptr_(wptr), iter_(iter), pptr_(pptr), r_(r)
        {
        }

        void operator()(const ::tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i)
                r_[i] = wptr_->eval_sp(iter_, pptr_->sp(i));
        }

        private:
        PathEvalTBB<T, Derived> *const wptr_;
        const std::size_t iter_;
        Particle<T> *const pptr_;
        double *const r_;
    }; // class ParallelPathState

    double parallel_run(std::size_t iter, Particle<T> &particle, double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL((range, work));
    }

    double parallel_run(std::size_t iter, Particle<T> &particle, double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::auto_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(
            (range, work, partitioner));
    }

    double parallel_run(std::size_t iter, Particle<T> &particle, double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::simple_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(
            (range, work, partitioner));
    }

    double parallel_run(std::size_t iter, Particle<T> &particle, double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        ::tbb::affinity_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(
            (range, work, partitioner));
    }

#if __TBB_TASK_GROUP_CONTEXT
    double parallel_run(std::size_t iter, Particle<T> &particle, double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::auto_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(
            (range, work, partitioner, context));
    }

    double parallel_run(std::size_t iter, Particle<T> &particle, double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::simple_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(
            (range, work, partitioner, context));
    }

    double parallel_run(std::size_t iter, Particle<T> &particle, double *r,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        ::tbb::affinity_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(
            (range, work, partitioner, context));
    }
#endif // __TBB_TASK_GROUP_CONTEXT
};     // PathEvalTBB

} // namespace vsmc

#endif // VSMC_SMP_BACKEND_TBB_HPP

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
#include <tbb/tbb.h>

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE(args)             \
    this->initialize_param(particle, param);                                  \
    this->pre_processor(particle);                                            \
    work_type work(this, &particle);                                          \
    ::tbb::parallel_reduce args;                                              \
    this->post_processor(particle);                                           \
    return work.accept();

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE(args)                   \
    this->pre_processor(iter, particle);                                      \
    work_type work(this, iter, &particle);                                    \
    ::tbb::parallel_reduce args;                                              \
    this->post_processor(iter, particle);                                     \
    return work.accept();

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(args)           \
    this->pre_processor(iter, particle);                                      \
    work_type work(this, iter, dim, &particle, res);                          \
    ::tbb::parallel_for args;                                                 \
    this->post_processor(iter, particle);

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(args)              \
    this->pre_processor(iter, particle);                                      \
    work_type work(this, iter, &particle, res);                               \
    ::tbb::parallel_for args;                                                 \
    this->post_processor(iter, particle);                                     \
    return this->path_grid(iter, particle);

namespace vsmc
{

VSMC_DEFINE_SMP_BACKEND_FORWARD(TBB)

/// \brief Particle::value_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename StateBase>
class StateTBB : public StateBase
{
    public:
    typedef typename traits::SizeTypeTrait<StateBase>::type size_type;

    explicit StateTBB(size_type N) : StateBase(N) {}

    template <typename IntType>
    void copy(size_type N, const IntType *copy_from)
    {
        parallel_copy_run(copy_from, ::tbb::blocked_range<size_type>(0, N));
    }

    protected:
    template <typename IntType>
    class work_type
    {
        public:
        work_type(StateTBB<StateBase> *state, const IntType *copy_from)
            : state_(state), copy_from_(copy_from)
        {
        }

        void operator()(const ::tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                state_->copy_particle(
                    static_cast<size_type>(copy_from_[i]), i);
            }
        }

        private:
        StateTBB<StateBase> *const state_;
        const IntType *const copy_from_;
    }; // class work_type

    template <typename IntType>
    void parallel_copy_run(
        const IntType *copy_from, const ::tbb::blocked_range<size_type> &range)
    {
        ::tbb::parallel_for(range, work_type<IntType>(this, copy_from));
    }

    template <typename IntType>
    void parallel_copy_run(const IntType *copy_from,
        const ::tbb::blocked_range<size_type> &range,
        const ::tbb::auto_partitioner &partitioner)
    {
        ::tbb::parallel_for(
            range, work_type<IntType>(this, copy_from), partitioner);
    }

    template <typename IntType>
    void parallel_copy_run(const IntType *copy_from,
        const ::tbb::blocked_range<size_type> &range,
        const ::tbb::simple_partitioner &partitioner)
    {
        ::tbb::parallel_for(
            range, work_type<IntType>(this, copy_from), partitioner);
    }

    template <typename IntType>
    void parallel_copy_run(const IntType *copy_from,
        const ::tbb::blocked_range<size_type> &range,
        ::tbb::affinity_partitioner &partitioner)
    {
        ::tbb::parallel_for(
            range, work_type<IntType>(this, copy_from), partitioner);
    }

#if __TBB_TASK_GROUP_CONTEXT
    template <typename IntType>
    void parallel_copy_run(const IntType *copy_from,
        const ::tbb::blocked_range<size_type> &range,
        const ::tbb::auto_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        ::tbb::parallel_for(
            range, work_type<IntType>(this, copy_from), partitioner, context);
    }

    template <typename IntType>
    void parallel_copy_run(const IntType *copy_from,
        const ::tbb::blocked_range<size_type> &range,
        const ::tbb::simple_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        ::tbb::parallel_for(
            range, work_type<IntType>(this, copy_from), partitioner, context);
    }

    template <typename IntType>
    void parallel_copy_run(const IntType *copy_from,
        const ::tbb::blocked_range<size_type> &range,
        ::tbb::affinity_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        ::tbb::parallel_for(
            range, work_type<IntType>(this, copy_from), partitioner, context);
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
        typedef typename Particle<T>::size_type size_type;

        work_type(InitializeTBB<T, Derived> *init, Particle<T> *pptr)
            : init_(init), pptr_(pptr), accept_(0)
        {
        }

        work_type(const work_type &other, ::tbb::split)
            : init_(other.init_), pptr_(other.pptr_), accept_(0)
        {
        }

        void operator()(const ::tbb::blocked_range<size_type> &range)
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                accept_ +=
                    init_->initialize_state(SingleParticle<T>(i, pptr_));
            }
        }

        void join(const work_type &other) { accept_ += other.accept_; }

        std::size_t accept() const { return accept_; }

        private:
        InitializeTBB<T, Derived> *const init_;
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
        typedef typename Particle<T>::size_type size_type;

        work_type(
            MoveTBB<T, Derived> *move, std::size_t iter, Particle<T> *pptr)
            : move_(move), iter_(iter), pptr_(pptr), accept_(0)
        {
        }

        work_type(const work_type &other, ::tbb::split)
            : move_(other.move_)
            , iter_(other.iter_)
            , pptr_(other.pptr_)
            , accept_(0)
        {
        }

        void operator()(const ::tbb::blocked_range<size_type> &range)
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                accept_ +=
                    move_->move_state(iter_, SingleParticle<T>(i, pptr_));
            }
        }

        void join(const work_type &other) { accept_ += other.accept_; }

        std::size_t accept() const { return accept_; }

        private:
        MoveTBB<T, Derived> *const move_;
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
        std::size_t iter, std::size_t dim, Particle<T> &particle, double *res)
    {
        parallel_run(iter, dim, particle, res,
            ::tbb::blocked_range<typename Particle<T>::size_type>(
                         0, particle.size()));
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(TBB, MonitorEval)

    class work_type
    {
        public:
        typedef typename Particle<T>::size_type size_type;

        work_type(MonitorEvalTBB<T, Derived> *monitor, std::size_t iter,
            std::size_t dim, Particle<T> *pptr, double *res)
            : monitor_(monitor), iter_(iter), dim_(dim), pptr_(pptr), res_(res)
        {
        }

        void operator()(const ::tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                monitor_->monitor_state(iter_, dim_,
                    SingleParticle<T>(i, pptr_),
                    res_ + static_cast<std::size_t>(i) * dim_);
            }
        }

        private:
        MonitorEvalTBB<T, Derived> *const monitor_;
        const std::size_t iter_;
        const std::size_t dim_;
        Particle<T> *const pptr_;
        double *const res_;
    }; // class work_type

    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *res,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL((range, work));
    }

    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *res,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::auto_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(
            (range, work, partitioner));
    }

    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *res,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::simple_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(
            (range, work, partitioner));
    }

    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *res,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        ::tbb::affinity_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(
            (range, work, partitioner));
    }

#if __TBB_TASK_GROUP_CONTEXT
    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *res,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::auto_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(
            (range, work, partitioner, context));
    }

    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *res,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::simple_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(
            (range, work, partitioner, context));
    }

    void parallel_run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *res,
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
    double operator()(std::size_t iter, Particle<T> &particle, double *res)
    {
        return parallel_run(iter, particle, res,
            ::tbb::blocked_range<typename Particle<T>::size_type>(
                                0, particle.size()));
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(TBB, PathEval)

    class work_type
    {
        public:
        typedef typename Particle<T>::size_type size_type;

        work_type(PathEvalTBB<T, Derived> *path, std::size_t iter,
            Particle<T> *pptr, double *res)
            : path_(path), iter_(iter), pptr_(pptr), res_(res)
        {
        }

        void operator()(const ::tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                res_[i] =
                    path_->path_state(iter_, SingleParticle<T>(i, pptr_));
            }
        }

        private:
        PathEvalTBB<T, Derived> *const path_;
        const std::size_t iter_;
        Particle<T> *const pptr_;
        double *const res_;
    }; // class ParallelPathState

    double parallel_run(std::size_t iter, Particle<T> &particle, double *res,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL((range, work));
    }

    double parallel_run(std::size_t iter, Particle<T> &particle, double *res,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::auto_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(
            (range, work, partitioner));
    }

    double parallel_run(std::size_t iter, Particle<T> &particle, double *res,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::simple_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(
            (range, work, partitioner));
    }

    double parallel_run(std::size_t iter, Particle<T> &particle, double *res,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        ::tbb::affinity_partitioner &partitioner)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(
            (range, work, partitioner));
    }

#if __TBB_TASK_GROUP_CONTEXT
    double parallel_run(std::size_t iter, Particle<T> &particle, double *res,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::auto_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(
            (range, work, partitioner, context));
    }

    double parallel_run(std::size_t iter, Particle<T> &particle, double *res,
        const ::tbb::blocked_range<typename Particle<T>::size_type> &range,
        const ::tbb::simple_partitioner &partitioner,
        ::tbb::task_group_context &context)
    {
        VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(
            (range, work, partitioner, context));
    }

    double parallel_run(std::size_t iter, Particle<T> &particle, double *res,
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

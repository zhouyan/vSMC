//============================================================================
// include/vsmc/smp/backend_tbb.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_SMP_BACKEND_TBB_HPP
#define VSMC_SMP_BACKEND_TBB_HPP

#include <vsmc/smp/backend_base.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_INITIALIZE(args) \
    this->initialize_param(particle, param);                                 \
    this->pre_processor(particle);                                           \
    parallel_work work(this, &particle);                                     \
    ::tbb::parallel_reduce args;                                             \
    this->post_processor(particle);                                          \
    return work.accept();

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MOVE(args) \
    this->pre_processor(iter, particle);                                     \
    parallel_work work(this, iter, &particle);                               \
    ::tbb::parallel_reduce args;                                             \
    this->post_processor(iter, particle);                                    \
    return work.accept();

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_MONITOR_EVAL(args) \
    this->pre_processor(iter, particle);                                     \
    parallel_work work(this, iter, dim, &particle, res);                     \
    ::tbb::parallel_for args;                                                \
    this->post_processor(iter, particle);

#define VSMC_DEFINE_SMP_BACKEND_TBB_PARALLEL_RUN_PATH_EVAL(args) \
    this->pre_processor(iter, particle);                                     \
    parallel_work work(this, iter, &particle, res);                          \
    ::tbb::parallel_for args;                                                \
    this->post_processor(iter, particle);                                    \
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
    class parallel_copy_work
    {
        public :

        parallel_copy_work (StateTBB<BaseState> *state,
                const IntType *copy_from) :
            state_(state), copy_from_(copy_from) {}

        void operator() (const ::tbb::blocked_range<size_type> &range) const
        {
            for (size_type to = range.begin(); to != range.end(); ++to)
                state_->copy_particle(copy_from_[to], to);
        }

        private :

        StateTBB<BaseState> *const state_;
        const IntType *const copy_from_;
    }; // class parallel_copy_work

    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range)
    {::tbb::parallel_for(range, parallel_copy_work<IntType>(this, copy_from));}

    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range,
            const ::tbb::auto_partitioner &partitioner)
    {
        ::tbb::parallel_for(range,
                parallel_copy_work<IntType>(this, copy_from), partitioner);
    }

    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range,
            const ::tbb::simple_partitioner &partitioner)
    {
        ::tbb::parallel_for(range,
                parallel_copy_work<IntType>(this, copy_from), partitioner);
    }

    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range,
            ::tbb::affinity_partitioner &partitioner)
    {
        ::tbb::parallel_for(range,
                parallel_copy_work<IntType>(this, copy_from), partitioner);
    }

#if __TBB_TASK_GROUP_CONTEXT
    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range,
            const ::tbb::auto_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        ::tbb::parallel_for(range,
                parallel_copy_work<IntType>(this, copy_from),
                partitioner, context);
    }

    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range,
            const ::tbb::simple_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        ::tbb::parallel_for(range,
                parallel_copy_work<IntType>(this, copy_from),
                partitioner, context);
    }

    template <typename IntType>
    void parallel_copy_run (const IntType *copy_from,
            const ::tbb::blocked_range<size_type> &range,
            ::tbb::affinity_partitioner &partitioner,
            ::tbb::task_group_context &context)
    {
        ::tbb::parallel_for(range,
                parallel_copy_work<IntType>(this, copy_from),
                partitioner, context);
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

    class parallel_work
    {
        public :

        typedef typename Particle<T>::size_type size_type;

        parallel_work (InitializeTBB<T, Derived> *init,
                Particle<T> *particle) :
            init_(init), particle_(particle), accept_(0) {}

        parallel_work (const parallel_work &other, ::tbb::split) :
            init_(other.init_), particle_(other.particle_), accept_(0) {}

        void operator() (const ::tbb::blocked_range<size_type> &range)
        {
            Particle<T> *const part = particle_;
            std::size_t acc = accept_;
            for (size_type i = range.begin(); i != range.end(); ++i)
                acc += init_->initialize_state(SingleParticle<T>(i, part));
            accept_ = acc;
        }

        void join (const parallel_work &other) {accept_ += other.accept_;}

        std::size_t accept () const {return accept_;}

        private :

        InitializeTBB<T, Derived> *const init_;
        Particle<T> *const particle_;
        std::size_t accept_;
    }; // class parallel_work

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

    class parallel_work
    {
        public :

        typedef typename Particle<T>::size_type size_type;

        parallel_work (MoveTBB<T, Derived> *move, std::size_t iter,
                Particle<T> *particle):
            move_(move), iter_(iter), particle_(particle), accept_(0) {}

        parallel_work (const parallel_work &other, ::tbb::split) :
            move_(other.move_), iter_(other.iter_),
            particle_(other.particle_), accept_(0) {}

        void operator() (const ::tbb::blocked_range<size_type> &range)
        {
            Particle<T> *const part = particle_;
            const std::size_t iter = iter_;
            std::size_t acc = accept_;
            for (size_type i = range.begin(); i != range.end(); ++i)
                acc += move_->move_state(iter, SingleParticle<T>(i, part));
            accept_ = acc;
        }

        void join (const parallel_work &other) {accept_ += other.accept_;}

        std::size_t accept () const {return accept_;}

        private :

        MoveTBB<T, Derived> *const move_;
        const std::size_t iter_;
        Particle<T> *const particle_;
        std::size_t accept_;
    }; // class parallel_work

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

    class parallel_work
    {
        public :

        typedef typename Particle<T>::size_type size_type;

        parallel_work (MonitorEvalTBB<T, Derived> *monitor,
                std::size_t iter, std::size_t dim,
                const Particle<T> *particle, double *res) :
            monitor_(monitor), iter_(iter), dim_(dim),
            particle_(particle), res_(res) {}

        void operator() (const ::tbb::blocked_range<size_type> &range) const
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

        MonitorEvalTBB<T, Derived> *const monitor_;
        const std::size_t iter_;
        const std::size_t dim_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class parallel_work

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

    class parallel_work
    {
        public :

        typedef typename Particle<T>::size_type size_type;

        parallel_work (PathEvalTBB<T, Derived> *path, std::size_t iter,
                const Particle<T> *particle, double *res) :
            path_(path), iter_(iter), particle_(particle), res_(res) {}

        void operator() (const ::tbb::blocked_range<size_type> &range) const
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

        PathEvalTBB<T, Derived> *const path_;
        const std::size_t iter_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class parallel_work

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

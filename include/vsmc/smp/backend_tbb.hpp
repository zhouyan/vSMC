#ifndef VSMC_SMP_BACKEND_TBB_HPP
#define VSMC_SMP_BACKEND_TBB_HPP

#include <vsmc/smp/base.hpp>
#include <vsmc/utility/tbb_op.hpp>
#include <tbb/tbb.h>

namespace vsmc {

/// \brief Particle::weight_set_type subtype using Intel Threading Building
/// Blocks
/// \ingroup TBB
template <typename BaseState>
class WeightSetTBB : public traits::WeightSetTypeTrait<BaseState>::type
{
    typedef typename traits::WeightSetTypeTrait<BaseState>::type base;

    public :

    typedef typename traits::SizeTypeTrait<base>::type size_type;

    explicit WeightSetTBB (size_type N) : base(N) {}

    protected :

    void log_weight2weight ()
    {
        const size_type N = static_cast<size_type>(this->size());
        tbb::parallel_for(tbb::blocked_range<size_type>(0, N),
                tbb_op::exp<double>(
                    this->log_weight_ptr(), this->weight_ptr()));
    }

    void weight2log_weight ()
    {
        const size_type N = static_cast<size_type>(this->size());
        tbb::parallel_for(tbb::blocked_range<size_type>(0, N),
                tbb_op::log<double>(
                    this->weight_ptr(), this->log_weight_ptr()));
    }

    void normalize_log_weight ()
    {
        const size_type N = static_cast<size_type>(this->size());
        tbb_op::maximum<double> max_weight(this->log_weight_ptr());
        tbb::parallel_reduce(tbb::blocked_range<size_type>(0, N), max_weight);
        tbb::parallel_for(tbb::blocked_range<size_type>(0, N),
                tbb_op::minus<double>(
                    this->log_weight_ptr(), this->log_weight_ptr(),
                    max_weight.result()));
    }

    void normalize_weight ()
    {
        const size_type N = static_cast<size_type>(this->size());
        tbb_op::summation<double> coeff(this->weight_ptr());
        tbb::parallel_reduce(tbb::blocked_range<size_type>(0, N), coeff);
        tbb::parallel_for(tbb::blocked_range<size_type>(0, N),
                tbb_op::multiplies<double>(
                    this->weight_ptr(), this->weight_ptr(),
                    1 / coeff.result()));
        tbb_op::square_sum<double> ess(this->weight_ptr());
        tbb::parallel_reduce(tbb::blocked_range<size_type>(0, this->size()),
                ess);
        this->set_ess(1 / ess.result());
    }
}; // class WeightSetTBB

/// \brief Calculating normalizing constant ratio using Intel Threading
/// Building Blocks
/// \ingroup TBB
class NormalizingConstantTBB : public NormalizingConstant
{
    public :

    NormalizingConstantTBB (std::size_t N) : NormalizingConstant(N) {}

    protected:

    void vd_exp (std::size_t N, double *inc_weight) const
    {
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, N),
                tbb_op::exp<double>(inc_weight, inc_weight));
    }
}; // class NormalizingConstantTBB

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
        VSMC_RUNTIME_ASSERT_SMP_BASE_COPY_SIZE_MISMATCH(TBB);

        tbb::parallel_for(tbb::blocked_range<size_type>(0, N),
                copy_work_<IntType>(this, copy_from));
    }

    private :

    template <typename IntType>
    class copy_work_
    {
        public :

        copy_work_ (StateTBB<BaseState> *state, const IntType *copy_from) :
            state_(state), copy_from_(copy_from) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type to = range.begin(); to != range.end(); ++to)
                state_->copy_particle(copy_from_[to], to);
        }

        private :

        StateTBB<BaseState> *const state_;
        const IntType *const copy_from_;
    }; // class copy_work_
}; // class StateTBB

/// \brief Sampler<T>::init_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
class InitializeTBB : public InitializeBase<T, Derived>
{
    public :


    std::size_t operator() (Particle<T> &particle, void *param)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        work_ work(this, &particle);
        tbb::parallel_reduce(tbb::blocked_range<size_type>(0, N), work);
        this->post_processor(particle);

        return work.accept();
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY_BASE(TBB, Initialize)

    private :

    class work_
    {
        public :

        typedef typename Particle<T>::size_type size_type;

        work_ (InitializeTBB<T, Derived> *init,
                Particle<T> *particle) :
            init_(init), particle_(particle), accept_(0) {}

        work_ (const work_ &other, tbb::split) :
            init_(other.init_), particle_(other.particle_), accept_(0) {}

        void operator() (const tbb::blocked_range<size_type> &range)
        {
            std::size_t acc = accept_;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                acc += init_->initialize_state(
                        SingleParticle<T>(i, particle_));
            }
            accept_ = acc;
        }

        void join (const work_ &other) {accept_ += other.accept_;}

        std::size_t accept () const {return accept_;}

        private :

        InitializeTBB<T, Derived> *const init_;
        Particle<T> *const particle_;
        std::size_t accept_;
    }; // class work_
}; // class InitializeTBB

/// \brief Sampler<T>::move_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
class MoveTBB : public MoveBase<T, Derived>
{
    public :


    std::size_t operator() (std::size_t iter, Particle<T> &particle)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        work_ work(this, iter, &particle);
        tbb::parallel_reduce(tbb::blocked_range<size_type>(0, N), work);
        this->post_processor(iter, particle);

        return work.accept();
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY_BASE(TBB, Move)

    private :

    class work_
    {
        public :

        typedef typename Particle<T>::size_type size_type;

        work_ (MoveTBB<T, Derived> *move, std::size_t iter,
                Particle<T> *particle):
            move_(move), iter_(iter), particle_(particle), accept_(0) {}

        work_ (const work_ &other, tbb::split) :
            move_(other.move_), iter_(other.iter_),
            particle_(other.particle_), accept_(0) {}

        void operator() (const tbb::blocked_range<size_type> &range)
        {
            std::size_t acc = accept_;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                acc += move_->move_state(iter_,
                        SingleParticle<T>(i, particle_));
            }
            accept_ = acc;
        }

        void join (const work_ &other)
        {
            accept_ += other.accept_;
        }

        std::size_t accept () const
        {
            return accept_;
        }

        private :

        MoveTBB<T, Derived> *const move_;
        const std::size_t iter_;
        Particle<T> *const particle_;
        std::size_t accept_;
    }; // class work_
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
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        tbb::parallel_for(tbb::blocked_range<size_type>(0, N),
                work_(this, iter, dim, &particle, res));
        this->post_processor(iter, particle);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY_BASE(TBB, MonitorEval)

    private :

    class work_
    {
        public :

        typedef typename Particle<T>::size_type size_type;

        work_ (MonitorEvalTBB<T, Derived> *monitor,
                std::size_t iter, std::size_t dim,
                const Particle<T> *particle, double *res) :
            monitor_(monitor), iter_(iter), dim_(dim),
            particle_(particle), res_(res) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                monitor_->monitor_state(iter_, dim_,
                        ConstSingleParticle<T>(i, particle_), res_ + i * dim_);
            }
        }

        private :

        MonitorEvalTBB<T, Derived> *const monitor_;
        const std::size_t iter_;
        const std::size_t dim_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
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
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        tbb::parallel_for(tbb::blocked_range<size_type>(0, N),
                work_(this, iter, &particle, res));
        this->post_processor(iter, particle);

        return this->path_grid(iter, particle);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY_BASE(TBB, PathEval)

    private :

    class work_
    {
        public :

        typedef typename Particle<T>::size_type size_type;

        work_ (PathEvalTBB<T, Derived> *path, std::size_t iter,
                const Particle<T> *particle, double *res) :
            path_(path), iter_(iter), particle_(particle), res_(res) {}

        void operator() (const tbb::blocked_range<size_type> &range) const
        {
            for (size_type i = range.begin(); i != range.end(); ++i) {
                res_[i] = path_->path_state(iter_,
                        ConstSingleParticle<T>(i, particle_));
            }
        }

        private :

        PathEvalTBB<T, Derived> *const path_;
        const std::size_t iter_;
        const Particle<T> *const particle_;
        double *const res_;
    }; // class work_
}; // PathEvalTBB

} // namespace vsmc

#endif // VSMC_SMP_BACKEND_TBB_HPP

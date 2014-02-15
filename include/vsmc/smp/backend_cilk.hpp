#ifndef VSMC_SMP_BACKEND_CILK_HPP
#define VSMC_SMP_BACKEND_CILK_HPP

#include <vsmc/smp/backend_base.hpp>
#include <cilk/cilk.h>
#include <cilk/cilk_api.h>
#include <cilk/reducer_opadd.h>

namespace vsmc {

VSMC_DEFINE_SMP_FORWARD(CILK)

/// \brief Particle::weight_set_type subtype using Intel Cilk Plus
/// \ingroup CILK
template <typename BaseState>
class WeightSetCILK : public traits::WeightSetTypeTrait<BaseState>::type
{
    typedef typename traits::WeightSetTypeTrait<BaseState>::type base;

    public :

    typedef typename traits::SizeTypeTrait<base>::type size_type;

    explicit WeightSetCILK (size_type N) : base(N) {}

    protected :

    void log_weight2weight ()
    {
        using std::exp;

        const size_type N = static_cast<size_type>(this->size());
        double *const weight = this->weight_ptr();
        const double *const log_weight = this->log_weight_ptr();
        cilk_for (size_type i = 0; i != N; ++i)
            weight[i] = exp(log_weight[i]);
    }

    void weight2log_weight ()
    {
        using std::log;

        const size_type N = static_cast<size_type>(this->size());
        const double *const weight = this->weight_ptr();
        double *const log_weight = this->log_weight_ptr();
        cilk_for (size_type i = 0; i != N; ++i)
            log_weight[i] = log(weight[i]);
    }
}; // class WeightSetCILK

/// \brief Calculating normalizing constant ratio using Intel Cilk Plus
/// \ingroup CILK
class NormalizingConstantCILK : public NormalizingConstant
{
    public :

    NormalizingConstantCILK (std::size_t N) : NormalizingConstant(N) {}

    protected:

    void vd_exp (std::size_t N, double *inc_weight) const
    {
        using std::exp;

        cilk_for (std::size_t i = 0; i != N; ++i)
            inc_weight[i] = exp(inc_weight[i]);
    }
}; // class NormalizingConstantCILK

/// \brief Particle::value_type subtype using Intel Cilk Plus
/// \ingroup CILK
template <typename BaseState>
class StateCILK : public BaseState
{
    public :

    typedef typename traits::SizeTypeTrait<BaseState>::type size_type;

    explicit StateCILK (size_type N) : BaseState(N) {}

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_SMP_BACKEND_BASE_COPY_SIZE_MISMATCH(CILK);

        cilk_for (size_type to = 0; to != N; ++to)
            this->copy_particle(copy_from[to], to);
    }
}; // class StateCILK

/// \brief Sampler<T>::init_type subtype using Intel Cilk Plus
/// \ingroup CILK
template <typename T, typename Derived>
class InitializeCILK : public InitializeBase<T, Derived>
{
    public :

    std::size_t operator() (Particle<T> &particle, void *param)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        cilk::reducer_opadd<std::size_t> accept;
        cilk_for (size_type i = 0; i != N; ++i)
            accept += this->initialize_state(SingleParticle<T>(i, &particle));
        this->post_processor(particle);

        return accept.get_value();
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(CILK, Initialize)
}; // class InitializeCILK

/// \brief Sampler<T>::move_type subtype using Intel Cilk Plus
/// \ingroup CILK
template <typename T, typename Derived>
class MoveCILK : public MoveBase<T, Derived>
{
    public :


    std::size_t operator() (std::size_t iter, Particle<T> &particle)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        cilk::reducer_opadd<std::size_t> accept;
        cilk_for (size_type i = 0; i != N; ++i)
            accept += this->move_state(iter, SingleParticle<T>(i, &particle));
        this->post_processor(iter, particle);

        return accept.get_value();
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(CILK, Move)
}; // class MoveCILK

/// \brief Monitor<T>::eval_type subtype using Intel Cilk Plus
/// \ingroup CILK
template <typename T, typename Derived>
class MonitorEvalCILK : public MonitorEvalBase<T, Derived>
{
    public :


    void operator() (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        cilk_for (size_type i = 0; i != N; ++i) {
            this->monitor_state(iter, dim,
                    ConstSingleParticle<T>(i, &particle), res + i * dim);
        }
        this->post_processor(iter, particle);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(CILK, MonitorEval)
}; // class MonitorEvalCILK

/// \brief Path<T>::eval_type subtype using Intel Cilk Plus
/// \ingroup CILK
template <typename T, typename Derived>
class PathEvalCILK : public PathEvalBase<T, Derived>
{
    public :

    double operator() (std::size_t iter, const Particle<T> &particle,
            double *res)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = static_cast<size_type>(particle.size());
        this->pre_processor(iter, particle);
        cilk_for (size_type i = 0; i != N; ++i) {
            res[i] = this->path_state(iter,
                    ConstSingleParticle<T>(i, &particle));
        }
        this->post_processor(iter, particle);

        return this->path_grid(iter, particle);
    }

    protected :

    VSMC_DEFINE_SMP_IMPL_COPY(CILK, PathEval)
}; // class PathEvalCILK

} // namespace vsmc

#endif // VSMC_SMP_BACKEND_CILK_HPP

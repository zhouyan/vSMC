#ifndef VSMC_SMP_PARALLEL_OMP_HPP
#define VSMC_SMP_PARALLEL_OMP_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/smp/base.hpp>
#include <omp.h>

namespace vsmc {

namespace traits {

#if defined(_OPENMP) && _OPENMP >= 200805 // OpenMP 3.0
template <typename T> struct OMPSizeTypeTrait
{
    typedef T type;
};
#else
template <typename T> struct OMPSizeTypeTrait
{
    typedef typename cxx11::make_signed<T>::type type;
};
#endif

} // namespace vsmc::traits

/// \brief Particle::value_type subtype
/// \ingroup OMP
template <std::size_t Dim, typename T>
class StateOMP : public StateBase<Dim, T>
{
    public :

    typedef StateBase<Dim, T> state_base_type;
    typedef typename traits::OMPSizeTypeTrait<
        typename state_base_type::size_type>::type size_type;
    typedef typename state_base_type::state_type state_type;

    explicit StateOMP (size_type N) : StateBase<Dim, T>(N) {}

    template <typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT((N == static_cast<size_type>(this->size())),
                "**StateOMP::copy** SIZE MISMATCH");

#pragma omp parallel for default(none) shared(copy_from)
        for (size_type to = 0; to < static_cast<size_type>(this->size()); ++to)
            this->copy_particle(copy_from[to], to);
    }
}; // class StateOMP

/// \brief Sampler<T>::init_type subtype
/// \ingroup OMP
template <typename T, typename Derived>
class InitializeOMP : public InitializeBase<T, Derived>
{
    public :

    typedef InitializeBase<T, Derived> initialize_base_type;
    typedef typename traits::OMPSizeTypeTrait<
        typename Particle<T>::size_type>::type size_type;
    typedef T value_type;

    std::size_t operator() (Particle<T> &particle, void *param)
    {
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        std::size_t accept = 0;
#pragma omp parallel for reduction(+ : accept) default(none) shared(particle)
        for (size_type i = 0; i < static_cast<size_type>(particle.size()); ++i)
            accept += this->initialize_state(SingleParticle<T>(i, &particle));
        this->post_processor(particle);

        return accept;
    }

    protected :

    InitializeOMP () {}
    InitializeOMP (const InitializeOMP<T, Derived> &) {}
    InitializeOMP<T, Derived> &operator=
        (const InitializeOMP<T, Derived> &) {return *this;}
    ~InitializeOMP () {}
}; // class InitializeOMP

/// \brief Sampler<T>::move_type subtype
/// \ingroup OMP
template <typename T, typename Derived>
class MoveOMP : public MoveBase<T, Derived>
{
    public :

    typedef MoveBase<T, Derived> move_base_type;
    typedef typename traits::OMPSizeTypeTrait<
        typename Particle<T>::size_type>::type size_type;
    typedef T value_type;

    std::size_t operator() (std::size_t iter, Particle<T> &particle)
    {
        this->pre_processor(iter, particle);
        std::size_t accept = 0;
#pragma omp parallel for reduction(+ : accept) default(none) \
        shared(particle, iter)
        for (size_type i = 0; i < static_cast<size_type>(particle.size()); ++i)
            accept += this->move_state(iter, SingleParticle<T>(i, &particle));
        this->post_processor(iter, particle);

        return accept;
    }

    protected :

    MoveOMP () {}
    MoveOMP (const MoveOMP<T, Derived> &) {}
    MoveOMP<T, Derived> &operator=
        (const MoveOMP<T, Derived> &) {return *this;}
    ~MoveOMP () {}
}; // class MoveOMP

/// \brief Monitor<T>::eval_type subtype
/// \ingroup OMP
template <typename T, typename Derived>
class MonitorEvalOMP : public MonitorEvalBase<T, Derived>
{
    public :

    typedef MonitorEvalBase<T, Derived> monitor_eval_base_type;
    typedef typename traits::OMPSizeTypeTrait<
        typename Particle<T>::size_type>::type size_type;
    typedef T value_type;

    void operator() (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res)
    {
        this->pre_processor(iter, particle);
#pragma omp parallel for default(none) shared(particle, iter, dim, res)
        for (size_type i = 0; i < static_cast<size_type>(particle.size()); ++i)
            this->monitor_state(iter, dim,
                    ConstSingleParticle<T>(i, &particle), res + i * dim);
        this->post_processor(iter, particle);
    }

    protected :

    MonitorEvalOMP () {}
    MonitorEvalOMP (const MonitorEvalOMP<T, Derived> &) {}
    MonitorEvalOMP<T, Derived> &operator=
        (const MonitorEvalOMP<T, Derived> &) {return *this;}
    ~MonitorEvalOMP () {}
}; // class MonitorEvalOMP

/// \brief Path<T>::eval_type subtype
/// \ingroup OMP
template <typename T, typename Derived>
class PathEvalOMP : public PathEvalBase<T, Derived>
{
    public :

    typedef PathEvalBase<T, Derived> path_eval_base_type;
    typedef typename traits::OMPSizeTypeTrait<
        typename Particle<T>::size_type>::type size_type;
    typedef T value_type;

    double operator() (std::size_t iter, const Particle<T> &particle,
            double *res)
    {
        this->pre_processor(iter, particle);
#pragma omp parallel for default(none) shared(particle, iter, res)
        for (size_type i = 0; i < static_cast<size_type>(particle.size()); ++i)
            res[i] = this->path_state(iter,
                    ConstSingleParticle<T>(i, &particle));
        this->post_processor(iter, particle);

        return this->path_width(iter, particle);
    }

    protected :

    PathEvalOMP () {}
    PathEvalOMP (const PathEvalOMP<T, Derived> &) {}
    PathEvalOMP<T, Derived> &operator=
        (const PathEvalOMP<T, Derived> &) {return *this;}
    ~PathEvalOMP () {}
}; // class PathEvalOMP

} // namespace vsmc

#endif // VSMC_SMP_PARALLEL_OMP_HPP

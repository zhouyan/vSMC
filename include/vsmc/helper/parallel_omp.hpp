#ifndef VSMC_HELPER_PARALLEL_OMP_HPP
#define VSMC_HELPER_PARALLEL_OMP_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/base.hpp>
#include <omp.h>

/// \defgroup OpenMP OpenMP
/// \ingroup Helper
/// \brief Parallelized samplers with OpenMP

namespace vsmc {

/// \brief Particle::value_type subtype
/// \ingroup OpenMP
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
template <unsigned Dim, typename T, typename Timer>
class StateOMP : public StateBase<Dim, T, Timer>
{
    public :

    typedef StateBase<Dim, T, Timer> state_base_type;
    using typename state_base_type::size_type;
    using typename state_base_type::state_type;
    using typename state_base_type::timer_type;

    explicit StateOMP (size_type N) : StateBase<Dim, T, Timer>(N), size_(N) {}

    template <typename SizeType>
    void copy (const SizeType *copy_from)
    {
#pragma omp parallel for default(none) shared(copy_from)
        for (size_type to = 0; to < size_; ++to)
            this->copy_particle(copy_from[to], to);
    }

    private :

    size_type size_;
}; // class StateOMP

/// \brief Sampler<T>::init_type subtype
/// \ingroup OpenMP
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class InitializeOMP : public InitializeBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (Particle<T> &particle, void *param)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateOMP, T, InitializeOMP);

        this->initialize_param(particle, param);
        this->pre_processor(particle);
        unsigned accept = 0;
        particle.value().timer().start();
#pragma omp parallel for reduction(+ : accept) default(none) shared(particle)
        for (size_type i = 0; i < particle.size(); ++i) {
            SingleParticle<T> sp(i, &particle);
            accept += this->initialize_state(sp);
        }
        particle.value().timer().stop();
        this->post_processor(particle);

        return accept;
    }

    protected :

    InitializeOMP () {}
    InitializeOMP (const InitializeOMP<T, Derived> &) {}
    const InitializeOMP<T, Derived> &operator=
        (const InitializeOMP<T, Derived> &) {return *this;}
    ~InitializeOMP () {}
}; // class InitializeOMP

/// \brief Sampler<T>::move_type subtype
/// \ingroup OpenMP
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MoveOMP : public MoveBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateOMP, T, MoveOMP);

        this->pre_processor(iter, particle);
        unsigned accept = 0;
        particle.value().timer().start();
#pragma omp parallel for reduction(+ : accept) default(none) \
        shared(particle, iter)
        for (size_type i = 0; i < particle.size(); ++i) {
            SingleParticle<T> sp(i, &particle);
            accept += this->move_state(iter, sp);
        }
        particle.value().timer().stop();
        this->post_processor(iter, particle);

        return accept;
    }

    protected :

    MoveOMP () {}
    MoveOMP (const MoveOMP<T, Derived> &) {}
    const MoveOMP<T, Derived> &operator=
        (const MoveOMP<T, Derived> &) {return *this;}
    ~MoveOMP () {}
}; // class MoveOMP

/// \brief Monitor<T>::eval_type subtype
/// \ingroup OpenMP
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MonitorEvalOMP : public MonitorEvalBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    void operator() (unsigned iter, unsigned dim, const Particle<T> &particle,
            double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateOMP, T, MonitorEvalOMP);

        this->pre_processor(iter, particle);
        particle.value().timer().start();
#pragma omp parallel for default(none) shared(particle, iter, dim, res)
        for (size_type i = 0; i < particle.size(); ++i) {
            double *rr = res + i * dim;
            ConstSingleParticle<T> sp(i, &particle);
            this->monitor_state(iter, dim, sp, rr);
        }
        particle.value().timer().stop();
        this->post_processor(iter, particle);
    }

    protected :

    MonitorEvalOMP () {}
    MonitorEvalOMP (const MonitorEvalOMP<T, Derived> &) {}
    const MonitorEvalOMP<T, Derived> &operator=
        (const MonitorEvalOMP<T, Derived> &) {return *this;}
    ~MonitorEvalOMP () {}
}; // class MonitorEvalOMP

/// \brief Path<T>::eval_type subtype
/// \ingroup OpenMP
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class PathEvalOMP : public PathEvalBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    double operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateOMP, T, PathEvalOMP);

        this->pre_processor(iter, particle);
        particle.value().timer().start();
#pragma omp parallel for default(none) shared(particle, iter, res)
        for (size_type i = 0; i < particle.size(); ++i) {
            ConstSingleParticle<T> sp(i, &particle);
            res[i] = this->path_state(iter, sp);
        }
        particle.value().timer().stop();
        this->post_processor(iter, particle);

        return this->path_width(iter, particle);
    }

    protected :

    PathEvalOMP () {}
    PathEvalOMP (const PathEvalOMP<T, Derived> &) {}
    const PathEvalOMP<T, Derived> &operator=
        (const PathEvalOMP<T, Derived> &) {return *this;}
    ~PathEvalOMP () {}
}; // class PathEvalOMP

} // namespace vsmc

#endif // VSMC_HELPER_PARALLEL_OMP_HPP

#ifndef VSMC_HELPER_SEQUENTIAL_HPP
#define VSMC_HELPER_SEQUENTIAL_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/base.hpp>

/// \defgroup Sequential Sequential
/// \ingroup Helper
/// \brief Single threaded sampler

namespace vsmc {

/// \brief Particle::value_type subtype
/// \ingroup Sequential
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
template <unsigned Dim, typename T>
class StateSeq : public internal::StateBase<Dim, T>
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T state_type;

    explicit StateSeq (size_type N) : internal::StateBase<Dim, T>(N) {}

    virtual ~StateSeq () {}

    void copy (size_type from, size_type to)
    {
        this->state().col(to) = this->state().col(from);
    }
}; // class StateSeq

/// \brief Sampler<T>::init_type subtype
/// \ingroup Sequential
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class InitializeSeq : public internal::InitializeBase<T, Derived>
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T value_type;

    virtual ~InitializeSeq () {}

    unsigned operator() (Particle<T> &particle, void *param)
    {
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        unsigned accept = 0;
        for (size_type i = 0; i != particle.size(); ++i)
            accept += this->initialize_state(SingleParticle<T>(i, &particle));
        this->post_processor(particle);

        return accept;
    }
}; // class InitializeSeq

/// \brief Sampler<T>::move_type subtype
/// \ingroup Sequential
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MoveSeq : public internal::MoveBase<T, Derived>
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T value_type;

    virtual ~MoveSeq () {}

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        this->pre_processor(iter, particle);
        unsigned accept = 0;
        for (size_type i = 0; i != particle.size(); ++i)
            accept += this->move_state(iter, SingleParticle<T>(i, &particle));
        this->post_processor(iter, particle);

        return accept;
    }
}; // class MoveSeq

/// \brief Monitor<T>::eval_type subtype
/// \ingroup Sequential
///
/// \tparam T A subtype of StateBase
/// \tparam Dim The dimension of the monitor
template <typename T, unsigned Dim>
class MonitorSeq
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T value_type;

    virtual ~MonitorSeq () {}

    void operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        pre_processor(iter, particle);
        for (size_type i = 0; i != particle.size(); ++i) {
            monitor_state(iter, ConstSingleParticle<T>(i, &particle),
                    res + i * Dim);
        }
        post_processor(iter, particle);
    }

    virtual void monitor_state (unsigned iter, ConstSingleParticle<T> part,
            double *res) = 0;
    virtual void pre_processor (unsigned iter, const Particle<T> &particle) {}
    virtual void post_processor (unsigned iter, const Particle<T> &particle) {}

    static unsigned dim ()
    {
        return Dim;
    }
}; // class MonitorSeq

/// \brief Path<T>::eval_type subtype
/// \ingroup Sequential
///
/// \tparam T A subtype of StateBase
template <typename T>
class PathSeq
{
    public :

    typedef VSMC_SIZE_TYPE size_type;
    typedef T value_type;

    virtual ~PathSeq () {}

    double operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        pre_processor(iter, particle);
        for (size_type i = 0; i != particle.size(); ++i)
            res[i] = path_state(iter, ConstSingleParticle<T>(i, &particle));
        post_processor(iter, particle);

        return width_state(iter, particle);
    }

    virtual double path_state (unsigned iter,
            ConstSingleParticle<T> part) = 0;
    virtual double width_state (unsigned iter,
            const Particle<T> &particle) = 0;
    virtual void pre_processor (unsigned iter, const Particle<T> &particle) {}
    virtual void post_processor (unsigned iter, const Particle<T> &particle) {}
};

} // namespace vsmc

#endif // VSMC_HELPER_SEQUENTIAL_HPP

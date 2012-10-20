#ifndef VSMC_HELPER_SEQUENTIAL_HPP
#define VSMC_HELPER_SEQUENTIAL_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/base.hpp>

namespace vsmc {

/// \brief Particle::value_type subtype
/// \ingroup Sequential
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
template <unsigned Dim, typename T>
class StateSEQ : public StateBase<Dim, T>
{
    public :

    typedef StateBase<Dim, T> state_base_type;
    typedef typename state_base_type::size_type  size_type;
    typedef typename state_base_type::state_type state_type;

    explicit StateSEQ (size_type N) : StateBase<Dim, T>(N), size_(N) {}

    template <typename IntType>
    void copy (const IntType *copy_from)
    {
        for (size_type to = 0; to != size_; ++to)
            this->copy_particle(copy_from[to], to);
    }

    private :

    size_type size_;
}; // class StateSEQ

/// \brief Sampler<T>::init_type subtype
/// \ingroup Sequential
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class InitializeSEQ : public InitializeBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (Particle<T> &particle, void *param)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateSEQ, T, InitializeSEQ);

        this->initialize_param(particle, param);
        this->pre_processor(particle);
        unsigned accept = 0;
        for (size_type i = 0; i != particle.value().size(); ++i)
            accept += this->initialize_state(SingleParticle<T>(i, &particle));
        this->post_processor(particle);

        return accept;
    }

    protected :

    InitializeSEQ () {}
    InitializeSEQ (const InitializeSEQ<T, Derived> &) {}
    InitializeSEQ<T, Derived> &operator=
        (const InitializeSEQ<T, Derived> &) {return *this;}
    ~InitializeSEQ () {}
}; // class InitializeSEQ

/// \brief Sampler<T>::move_type subtype
/// \ingroup Sequential
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MoveSEQ : public MoveBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateSEQ, T, MoveSEQ);

        this->pre_processor(iter, particle);
        unsigned accept = 0;
        for (size_type i = 0; i != particle.value().size(); ++i)
            accept += this->move_state(iter, SingleParticle<T>(i, &particle));
        this->post_processor(iter, particle);

        return accept;
    }

    protected :

    MoveSEQ () {}
    MoveSEQ (const MoveSEQ<T, Derived> &) {}
    MoveSEQ<T, Derived> &operator=
        (const MoveSEQ<T, Derived> &) {return *this;}
    ~MoveSEQ () {}
}; // class MoveSEQ

/// \brief Monitor<T>::eval_type subtype
/// \ingroup Sequential
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MonitorEvalSEQ : public MonitorEvalBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    void operator() (unsigned iter, unsigned dim, const Particle<T> &particle,
            double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateSEQ, T, MonitorEvalSEQ);

        this->pre_processor(iter, particle);
        for (size_type i = 0; i != particle.value().size(); ++i) {
            this->monitor_state(iter, dim,
                    ConstSingleParticle<T>(i, &particle), res + i * dim);
        }
        this->post_processor(iter, particle);
    }

    protected :

    MonitorEvalSEQ () {}
    MonitorEvalSEQ (const MonitorEvalSEQ<T, Derived> &) {}
    MonitorEvalSEQ<T, Derived> &operator=
        (const MonitorEvalSEQ<T, Derived> &) {return *this;}
    ~MonitorEvalSEQ () {}
}; // class MonitorEvalSEQ

/// \brief Path<T>::eval_type subtype
/// \ingroup Sequential
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class PathEvalSEQ : public PathEvalBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    double operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateSEQ, T, PathEvalSEQ);

        this->pre_processor(iter, particle);
        for (size_type i = 0; i != particle.value().size(); ++i) {
            res[i] = this->path_state(iter,
                    ConstSingleParticle<T>(i, &particle));
        }
        this->post_processor(iter, particle);

        return this->path_width(iter, particle);
    }

    protected :

    PathEvalSEQ () {}
    PathEvalSEQ (const PathEvalSEQ<T, Derived> &) {}
    PathEvalSEQ<T, Derived> &operator=
        (const PathEvalSEQ<T, Derived> &) {return *this;}
    ~PathEvalSEQ () {}
}; // class PathEvalSEQ

} // namespace vsmc

#endif // VSMC_HELPER_SEQUENTIAL_HPP

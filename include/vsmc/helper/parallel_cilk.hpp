#ifndef VSMC_HELPER_PARALLEL_CILK_HPP
#define VSMC_HELPER_PARALLEL_CILK_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/helper/base.hpp>
#include <cilk/cilk.h>
#include <cilk/reducer_opadd.h>

namespace vsmc {

/// \brief Particle::value_type subtype
/// \ingroup CILK
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
template <unsigned Dim, typename T>
class StateCILK : public StateBase<Dim, T>
{
    public :

    typedef StateBase<Dim, T> state_base_type;
    typedef typename state_base_type::size_type  size_type;
    typedef typename state_base_type::state_type state_type;

    explicit StateCILK (size_type N) : StateBase<Dim, T>(N), size_(N) {}

    template <typename IntType>
    void copy (const IntType *copy_from)
    {
        cilk_for (size_type to = 0; to != size_; ++to)
            this->copy_particle(copy_from[to], to);
    }

    private :

    size_type size_;
}; // class StateCILK

/// \brief Sampler<T>::init_type subtype
/// \ingroup CILK
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class InitializeCILK : public InitializeBase<T, Derived>
{
    public :

    typedef InitializeBase<T, Derived> initialize_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (Particle<T> &particle, void *param)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateCILK, T, InitializeCILK);

        this->initialize_param(particle, param);
        this->pre_processor(particle);
        cilk::reducer_opadd<unsigned> accept;
        cilk_for (size_type i = 0; i != particle.value().size(); ++i)
            accept += this->initialize_state(SingleParticle<T>(i, &particle));
        this->post_processor(particle);

        return accept.get_value();
    }

    protected :

    InitializeCILK () {}
    InitializeCILK (const InitializeCILK<T, Derived> &) {}
    InitializeCILK<T, Derived> &operator=
        (const InitializeCILK<T, Derived> &) {return *this;}
    ~InitializeCILK () {}
}; // class InitializeCILK

/// \brief Sampler<T>::move_type subtype
/// \ingroup CILK
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MoveCILK : public MoveBase<T, Derived>
{
    public :

    typedef MoveBase<T, Derived> move_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateCILK, T, MoveCILK);

        this->pre_processor(iter, particle);
        cilk::reducer_opadd<unsigned> accept;
        cilk_for (size_type i = 0; i != particle.value().size(); ++i)
            accept += this->move_state(iter, SingleParticle<T>(i, &particle));
        this->post_processor(iter, particle);

        return accept.get_value();
    }

    protected :

    MoveCILK () {}
    MoveCILK (const MoveCILK<T, Derived> &) {}
    MoveCILK<T, Derived> &operator=
        (const MoveCILK<T, Derived> &) {return *this;}
    ~MoveCILK () {}
}; // class MoveCILK

/// \brief Monitor<T>::eval_type subtype
/// \ingroup CILK
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MonitorEvalCILK : public MonitorEvalBase<T, Derived>
{
    public :

    typedef MonitorEvalBase<T, Derived> monitor_eval_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    void operator() (unsigned iter, unsigned dim, const Particle<T> &particle,
            double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateCILK, T, MonitorEvalCILK);

        this->pre_processor(iter, particle);
        cilk_for (size_type i = 0; i != particle.value().size(); ++i) {
            this->monitor_state(iter, dim,
                    ConstSingleParticle<T>(i, &particle), res + i * dim);
        }
        this->post_processor(iter, particle);
    }

    protected :

    MonitorEvalCILK () {}
    MonitorEvalCILK (const MonitorEvalCILK<T, Derived> &) {}
    MonitorEvalCILK<T, Derived> &operator=
        (const MonitorEvalCILK<T, Derived> &) {return *this;}
    ~MonitorEvalCILK () {}
}; // class MonitorEvalCILK

/// \brief Path<T>::eval_type subtype
/// \ingroup CILK
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class PathEvalCILK : public PathEvalBase<T, Derived>
{
    public :

    typedef PathEvalBase<T, Derived> path_eval_base_type;
    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    double operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateCILK, T, PathEvalCILK);

        this->pre_processor(iter, particle);
        cilk_for (size_type i = 0; i != particle.value().size(); ++i) {
            res[i] = this->path_state(iter,
                    ConstSingleParticle<T>(i, &particle));
        }
        this->post_processor(iter, particle);

        return this->path_width(iter, particle);
    }

    protected :

    PathEvalCILK () {}
    PathEvalCILK (const PathEvalCILK<T, Derived> &) {}
    PathEvalCILK<T, Derived> &operator=
        (const PathEvalCILK<T, Derived> &) {return *this;}
    ~PathEvalCILK () {}
}; // class PathEvalCILK

} // namespace vsmc

#endif // VSMC_HELPER_PARALLEL_CILK_HPP

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
/// \tparam Timer The timer
template <unsigned Dim, typename T, typename Timer>
class StateCilk : public StateBase<Dim, T, Timer>
{
    public :

    typedef StateBase<Dim, T, Timer> state_base_type;
    typedef typename state_base_type::size_type  size_type;
    typedef typename state_base_type::state_type state_type;
    typedef typename state_base_type::timer_type timer_type;

    explicit StateCilk (size_type N) : StateBase<Dim, T, Timer>(N), size_(N) {}

    template <typename IntType>
    void copy (const IntType *copy_from)
    {
        cilk_for (size_type to = 0; to != size_; ++to)
            this->copy_particle(copy_from[to], to);
    }

    private :

    size_type size_;
}; // class StateCilk

/// \brief Sampler<T>::init_type subtype
/// \ingroup CILK
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class InitializeCilk : public InitializeBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (Particle<T> &particle, void *param)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateCilk, T, InitializeCilk);

        this->initialize_param(particle, param);
        this->pre_processor(particle);
        cilk::reducer_opadd<unsigned> accept;
        particle.value().timer().start();
        cilk_for (size_type i = 0; i != particle.value().size(); ++i)
            accept += this->initialize_state(SingleParticle<T>(i, &particle));
        particle.value().timer().stop();
        this->post_processor(particle);

        return accept.get_value();
    }

    protected :

    InitializeCilk () {}
    InitializeCilk (const InitializeCilk<T, Derived> &) {}
    InitializeCilk<T, Derived> &operator=
        (const InitializeCilk<T, Derived> &) {return *this;}
    ~InitializeCilk () {}
}; // class InitializeCilk

/// \brief Sampler<T>::move_type subtype
/// \ingroup CILK
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MoveCilk : public MoveBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateCilk, T, MoveCilk);

        this->pre_processor(iter, particle);
        cilk::reducer_opadd<unsigned> accept;
        particle.value().timer().start();
        cilk_for (size_type i = 0; i != particle.value().size(); ++i)
            accept += this->move_state(iter, SingleParticle<T>(i, &particle));
        particle.value().timer().stop();
        this->post_processor(iter, particle);

        return accept.get_value();
    }

    protected :

    MoveCilk () {}
    MoveCilk (const MoveCilk<T, Derived> &) {}
    MoveCilk<T, Derived> &operator=
        (const MoveCilk<T, Derived> &) {return *this;}
    ~MoveCilk () {}
}; // class MoveCilk

/// \brief Monitor<T>::eval_type subtype
/// \ingroup CILK
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class MonitorEvalCilk : public MonitorEvalBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    void operator() (unsigned iter, unsigned dim, const Particle<T> &particle,
            double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateCilk, T, MonitorEvalCilk);

        this->pre_processor(iter, particle);
        particle.value().timer().start();
        cilk_for (size_type i = 0; i != particle.value().size(); ++i) {
            this->monitor_state(iter, dim,
                    ConstSingleParticle<T>(i, &particle), res + i * dim);
        }
        particle.value().timer().stop();
        this->post_processor(iter, particle);
    }

    protected :

    MonitorEvalCilk () {}
    MonitorEvalCilk (const MonitorEvalCilk<T, Derived> &) {}
    MonitorEvalCilk<T, Derived> &operator=
        (const MonitorEvalCilk<T, Derived> &) {return *this;}
    ~MonitorEvalCilk () {}
}; // class MonitorEvalCilk

/// \brief Path<T>::eval_type subtype
/// \ingroup CILK
///
/// \tparam T A subtype of StateBase
template <typename T, typename Derived>
class PathEvalCilk : public PathEvalBase<T, Derived>
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    double operator() (unsigned iter, const Particle<T> &particle, double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateCilk, T, PathEvalCilk);

        this->pre_processor(iter, particle);
        particle.value().timer().start();
        cilk_for (size_type i = 0; i != particle.value().size(); ++i) {
            res[i] = this->path_state(iter,
                    ConstSingleParticle<T>(i, &particle));
        }
        particle.value().timer().stop();
        this->post_processor(iter, particle);

        return this->path_width(iter, particle);
    }

    protected :

    PathEvalCilk () {}
    PathEvalCilk (const PathEvalCilk<T, Derived> &) {}
    PathEvalCilk<T, Derived> &operator=
        (const PathEvalCilk<T, Derived> &) {return *this;}
    ~PathEvalCilk () {}
}; // class PathEvalCilk

} // namespace vsmc

#endif // VSMC_HELPER_PARALLEL_CILK_HPP

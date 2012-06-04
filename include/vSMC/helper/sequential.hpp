#ifndef V_SMC_HELPER_SEQUENTIAL_HPP
#define V_SMC_HELPER_SEQUENTIAL_HPP

#include <vSMC/internal/common.hpp>

namespace vSMC {

/// \brief Sampler<T>::init_type subtype
/// \ingroup Sequential
///
/// \tparam T A subtype of StateBase
template <typename T>
class InitializeSeq
{
    public :

    virtual ~InitializeSeq () {}

    virtual unsigned operator() (Particle<T> &particle, void *param)
    {
        initialize_param(particle, param);
        pre_processor(particle);
        unsigned accept = 0;
        for (typename Particle<T>::size_type i = 0;
                i != particle.size(); ++i) {
            accept += initialize_state(SingleParticle<T>(i, &particle));
        }
        post_processor(particle);

        return accept;
    }

    virtual unsigned initialize_state (SingleParticle<T> part) = 0;
    virtual void initialize_param (Particle<T> &particle, void *param) {}
    virtual void pre_processor (Particle<T> &particle) {}
    virtual void post_processor (Particle<T> &particle) {}
}; // class InitializeSeq

/// \brief Sampler<T>::move_type subtype
/// \ingroup Sequential
///
/// \tparam T A subtype of StateBase
template <typename T>
class MoveSeq
{
    public :

    virtual ~MoveSeq () {}

    virtual unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        pre_processor(iter, particle);
        unsigned accept = 0;
        for (typename Particle<T>::size_type i = 0;
                i != particle.size(); ++i) {
            accept += move_state(iter, SingleParticle<T>(i, &particle));
        }
        post_processor(iter, particle);

        return accept;
    }

    virtual unsigned move_state (unsigned iter, SingleParticle<T> part) = 0;
    virtual void pre_processor (unsigned iter, Particle<T> &particle) {}
    virtual void post_processor (unsigned iter, Particle<T> &particle) {}
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

    virtual ~MonitorSeq () {}

    virtual void operator() (unsigned iter, const Particle<T> &particle,
            double *res)
    {
        pre_processor(iter, particle);
        for (typename Particle<T>::size_type i = 0;
                i != particle.size(); ++i) {
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

    virtual ~PathSeq () {}

    virtual double operator() (unsigned iter, const Particle<T> &particle,
            double *res)
    {
        pre_processor(iter, particle);
        for (typename Particle<T>::size_type i = 0; i != particle.size(); ++i)
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

} // namespace vSMC

#endif // V_SMC_HELPER_SEQUENTIAL_HPP

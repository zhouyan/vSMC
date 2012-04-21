#ifndef V_SMC_HELPER_SINGLE_PARTICLE_HPP
#define V_SMC_HELPER_SINGLE_PARTICLE_HPP

#include <vSMC/internal/config.hpp>
#include <vSMC/core/particle.hpp>

namespace vSMC {

template <typename T>
class SingleParticle
{
    public :

    typedef typename T::state_type state_type;
    typedef typename Particle<T>::rng_type rng_type;

    SingleParticle (std::size_t id, state_type *state,
            double weight, double log_weight, double *new_weight,
            Particle<T> *particle, rng_type *rng) :
        id_(id), state_(state),
        weight_(weight), log_weight_(log_weight), new_weight_(new_weight),
        particle_(particle), rng_(rng) {}

    std::size_t id () const
    {
        return id_;
    }

    state_type *state ()
    {
        return state_;
    }

    const state_type *state () const
    {
        return state_;
    }

    double weight () const
    {
        return weight_;
    }

    double log_weight () const
    {
        return log_weight_;
    }

    void new_weight (double w)
    {
        *new_weight_ = w;
    }

    const Particle<T> &particle () const
    {
        return *particle_;
    }

    rng_type &rng ()
    {
        return *rng_;
    }

    private :

    std::size_t id_;
    state_type *const state_;
    double weight_;
    double log_weight_;
    double *const new_weight_;
    Particle<T> *const particle_;
    typename Particle<T>::rng_type *const rng_;
}; // class SingleParticle

} // namespace vSMC

#endif // V_SMC_HELPER_SINGLE_PARTICLE_HPP

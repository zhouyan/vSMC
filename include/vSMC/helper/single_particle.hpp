#ifndef V_SMC_HELPER_SINGLE_PARTICLE_HPP
#define V_SMC_HELPER_SINGLE_PARTICLE_HPP

#include <vSMC/internal/common.hpp>

namespace vSMC {

/// \brief A thin wrapper over a complete Particle
///
/// \tparam T A subtype of StateBase
template <typename T>
class SingleParticle
{
    public :

    typedef typename T::state_type state_type;
    typedef typename Particle<T>::rng_type rng_type;

    SingleParticle (std::size_t id, double *new_weight,
            Particle<T> *particle) :
        id_(id), new_weight_(new_weight),
        particle_(particle), const_particle_(particle) {}

    SingleParticle (std::size_t id, const Particle<T> *particle) :
        id_(id), new_weight_(NULL),
        particle_(NULL), const_particle_(particle) {}

    std::size_t id () const
    {
        return id_;
    }

    state_type &state (unsigned pos)
    {
        assert(particle_);
        return particle_->value().state(id_, pos);
    }

    const state_type &state (unsigned pos) const
    {
        return const_particle_->value().state(id_, pos);
    }

    state_type *state ()
    {
        assert(particle_);
        return particle_->value().state(id_);
    }

    const state_type *state () const
    {
        return const_particle_->value().state(id_);
    }

    double weight () const
    {
        return const_particle_->weight(id_);
    }

    double log_weight () const
    {
        return const_particle_->log_weight(id_);
    }

    void new_weight (double weight)
    {
        assert(new_weight_);
        *new_weight_ = weight;
    }

    const Particle<T> &particle () const
    {
        return *const_particle_;
    }

    rng_type &rng ()
    {
        assert(particle_);
        return particle_->prng(id_);
    }

    private :

    const std::size_t id_;
    double *const new_weight_;
    Particle<T> *const particle_;
    const Particle<T> *const const_particle_;
}; // class SingleParticle

} // namespace vSMC

#endif // V_SMC_HELPER_SINGLE_PARTICLE_HPP

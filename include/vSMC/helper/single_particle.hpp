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

    typedef V_SMC_INDEX_TYPE size_type;
    typedef typename T::state_type state_type;
    typedef typename Particle<T>::rng_type rng_type;

    SingleParticle (size_type id, Particle<T> *particle) :
        id_(id), particle_(particle) {}

    size_type id () const
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
        assert(particle_);
        return particle_->value().state(id_, pos);
    }

    state_type *state ()
    {
        assert(particle_);
        return particle_->value().state(id_);
    }

    const state_type *state () const
    {
        assert(particle_);
        return particle_->value().state(id_);
    }

    double log_weight () const
    {
        assert(particle_);
        return particle_->log_weight(id_);
    }

    const Particle<T> &particle () const
    {
        assert(particle_);
        return *particle_;
    }

    rng_type &rng ()
    {
        assert(particle_);
        return particle_->prng(id_);
    }

    private :

    const size_type id_;
    Particle<T> *const particle_;
}; // class SingleParticle

/// \brief A thin wrapper over a complete const Particle
///
/// \tparam T A subtype of StateBase
template <typename T>
class ConstSingleParticle
{
    public :

    typedef V_SMC_INDEX_TYPE size_type;
    typedef typename T::state_type state_type;
    typedef typename Particle<T>::rng_type rng_type;

    ConstSingleParticle (size_type id, const Particle<T> *particle) :
        id_(id), particle_(particle) {}

    size_type id () const
    {
        return id_;
    }

    const state_type &state (unsigned pos) const
    {
        assert(particle_);
        return particle_->value().state(id_, pos);
    }

    const state_type *state () const
    {
        assert(particle_);
        return particle_->value().state(id_);
    }

    double weight () const
    {
        assert(particle_);
        return particle_->weight(id_);
    }

    double log_weight () const
    {
        assert(particle_);
        return particle_->log_weight(id_);
    }

    const Particle<T> &particle () const
    {
        assert(particle_);
        return *particle_;
    }

    private :

    const size_type id_;
    const Particle<T> *const particle_;
}; // class SingleParticle

} // namespace vSMC

#endif // V_SMC_HELPER_SINGLE_PARTICLE_HPP

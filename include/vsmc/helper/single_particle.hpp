#ifndef VSMC_HELPER_SINGLE_PARTICLE_HPP
#define VSMC_HELPER_SINGLE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief A thin wrapper over a complete Particle
/// \ingroup Helper
///
/// \tparam T A subtype of StateBase
template <typename T>
class SingleParticle : public SingleParticleTrait
{
    public :

    /// The type of the size of the particle set
    typedef typename Particle<T>::size_type size_type;

    /// The type of the state parameters
    typedef typename T::state_type state_type;

    /// The type of RNG engine
    typedef typename Particle<T>::rng_type rng_type;

    /// \brief Construct a SingleParticle with a given id and a particle set
    ///
    /// \param id The id of the particle, start with zero
    /// \param particle A pointer to the particle set
    ///
    /// \note particle cannot be a const pointer. Unless one explicitly cast
    /// out the constness of the pointer, otherwise this shall results in a
    /// compile time error. vsmc itself will never do this. When one need
    /// access to a const particle set, use the ConstSingleParticle variant
    /// which is exaclty for this purpose, which does not have the mutator like
    /// rng() and write access to states.
    SingleParticle (size_type id, Particle<T> *particle) :
        id_(id), particle_(particle) {}

    /// \brief The id of the particle
    ///
    /// \return The id of the particle this SingleParticle object represents
    size_type id () const
    {
        return id_;
    }

    /// \brief Read and write access to a single parameter
    ///
    /// \param pos The position of the parameter
    ///
    /// \return A reference to the parameter
    state_type &state (unsigned pos)
    {
        assert(particle_);
        return particle_->value().state(id_, pos);
    }

    /// \brief Read only access to a single parameter
    ///
    /// \param pos The position of the parameter
    ///
    /// \return A const reference to the parameter
    const state_type &state (unsigned pos) const
    {
        assert(particle_);
        return particle_->value().state(id_, pos);
    }

    /// \brief Read and write access to all parameters
    ///
    /// \return A pointer to the parameter array
    state_type *state ()
    {
        assert(particle_);
        return particle_->value().state(id_);
    }

    /// \brief Read only access to all parameters
    ///
    /// \return A const pointer to the parameter array
    const state_type *state () const
    {
        assert(particle_);
        return particle_->value().state(id_);
    }

    /// \brief The weight
    ///
    /// \return The weight of the particle this SingleParticle represent
    double weight () const
    {
        assert(particle_);
        return particle_->weight(id_);
    }

    /// \brief The log weight
    ///
    /// \return The log weight of the particle this SingleParticle represent
    double log_weight () const
    {
        assert(particle_);
        return particle_->log_weight(id_);
    }

    /// \brief Read only access to all particles
    ///
    /// \return A const reference to the whole particle set
    const Particle<T> &particle () const
    {
        assert(particle_);
        return *particle_;
    }

    /// \brief RNG engine
    ///
    /// \return A reference to a C++11 RNG engine that unique to this particle
    rng_type &rng ()
    {
        assert(particle_);
        return particle_->rng(id_);
    }

    private :

    const size_type id_;
    Particle<T> *const particle_;
}; // class SingleParticle

/// \brief A const variant to SingleParticle
/// \ingroup Helper
///
/// \tparam T A subtype of StateBase
template <typename T>
class ConstSingleParticle : public ConstSingleParticleTrait
{
    public :

    typedef typename Particle<T>::size_type size_type;
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

} // namespace vsmc

#endif // VSMC_HELPER_SINGLE_PARTICLE_HPP

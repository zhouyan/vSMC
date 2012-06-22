#ifndef VSMC_HELPER_SINGLE_PARTICLE_HPP
#define VSMC_HELPER_SINGLE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief A thin wrapper over a complete Particle
/// \ingroup Helper
///
/// \tparam T A subtype of StateBase
template <typename T>
class SingleParticle
{
    public :

    /// The type of the number of particles
    typedef typename SizeTypeTrait<T>::type size_type;

    /// The type of the particle values
    typedef T value_type;

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

    /// The id of the particle
    size_type id () const
    {
        return id_;
    }

    /// \brief Read and write access to a single parameter
    ///
    /// \param pos The position of the parameter
    state_type &state (unsigned pos)
    {
        assert(particle_);
        return particle_->value().state(id_, pos);
    }

    /// \brief Read only access to a single parameter
    ///
    /// \param pos The position of the parameter
    const state_type &state (unsigned pos) const
    {
        assert(particle_);
        return particle_->value().state(id_, pos);
    }

    /// Read and write access to all parameters through pointer
    state_type *state ()
    {
        assert(particle_);
        return particle_->value().state(id_);
    }

    /// Read only access to all parameters through pointer
    const state_type *state () const
    {
        assert(particle_);
        return particle_->value().state(id_);
    }

    /// The weight of this particle
    double weight () const
    {
        assert(particle_);
        return particle_->weight()[id_];
    }

    /// The log weight of this particle
    double log_weight () const
    {
        assert(particle_);
        return particle_->log_weight()[id_];
    }

    /// Read only access to all particles
    const Particle<T> &particle () const
    {
        assert(particle_);
        return *particle_;
    }

    /// A unique RNG engine for this particle
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
class ConstSingleParticle
{
    public :

    typedef typename SizeTypeTrait<T>::type size_type;
    typedef T value_type;
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
        return particle_->weight()[id_];
    }

    double log_weight () const
    {
        assert(particle_);
        return particle_->log_weight()[id_];
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

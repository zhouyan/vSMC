//============================================================================
// include/vsmc/core/single_particle.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_CORE_SINGLE_PARTICLE_HPP
#define VSMC_CORE_SINGLE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief A thin wrapper over a complete Particle
/// \ingroup Core
template <typename T>
class SingleParticleBase
{
    public :

    SingleParticleBase (typename Particle<T>::size_type id,
            Particle<T> *particle_ptr) :
        id_(id), particle_ptr_(particle_ptr) {}

    typename Particle<T>::size_type id () const {return id_;}

    const Particle<T> &particle () const {return *particle_ptr_;}

    const Particle<T> *particle_ptr () const {return particle_ptr_;}

    typename Particle<T>::rng_type &rng () const
    {return particle_ptr_->rng(id_);}

    protected :

    Particle<T> *mutable_particle_ptr () const {return particle_ptr_;}

    private :

    typename Particle<T>::size_type id_;
    Particle<T> *particle_ptr_;
}; // class SingleParticleBase

/// \brief A const variant to SingleParticle
/// \ingroup Core
template <typename T>
class ConstSingleParticleBase
{
    public :

    ConstSingleParticleBase (typename Particle<T>::size_type id,
            const Particle<T> *particle_ptr) :
        id_(id), particle_ptr_(particle_ptr) {}

    typename Particle<T>::size_type id () const {return id_;}

    const Particle<T> &particle () const {return *particle_ptr_;}

    const Particle<T> *particle_ptr () const {return particle_ptr_;}

    private :

    typename Particle<T>::size_type id_;
    const Particle<T> *particle_ptr_;
}; // class ConstSingleParticleBase

/// \brief A thin wrapper over a complete Particle
/// \ingroup Core
///
/// \details
/// This is the basic SingleParticle available for any type of Particle. To
/// extend it for type `T`. One can either specialize
/// vsmc::traits::SingleParticleBaseTypeTrait<T> or define a class template
/// named `single_particle_type` within `T` with the following minimum
/// requirement.
/// ~~~{.cpp}
/// template <typename S> // S: StateType, such as StateMatrix<Dim, T>
/// struct single_particle_type
/// {
///     typedef IntType size_type;
///     single_particle_type (size_type id, Particle<S> *particle_ptr);
///     size_type id () const;
///     const Particle<S> *particle_ptr () const;
/// };
/// ~~~
/// Usually you can safely derive `single_particle_type<S>` from
/// SingleParticleBase<S> and add methods specific to `S`.
template <typename T>
class SingleParticle :
    public traits::SingleParticleBaseTypeTrait<T>::type
{
    typedef typename traits::SingleParticleBaseTypeTrait<T>::type base;

    public :

    SingleParticle (typename Particle<T>::size_type id,
            Particle<T> *particle_ptr) : base(id, particle_ptr) {}
}; // class SingleParticle

/// \brief A const variant to SingleParticle
/// \ingroup Core
template <typename T>
class ConstSingleParticle :
    public traits::ConstSingleParticleBaseTypeTrait<T>::type
{
    typedef typename traits::ConstSingleParticleBaseTypeTrait<T>::type base;

    public :

    ConstSingleParticle (typename Particle<T>::size_type id,
            const Particle<T> *particle_ptr) : base(id, particle_ptr) {}
}; // class ConstSingleParticle

} // namespace vsmc

#endif // VSMC_CORE_SINGLE_PARTICLE_HPP

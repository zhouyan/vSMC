//============================================================================
// vSMC/include/vsmc/core/single_particle.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_CORE_SINGLE_PARTICLE_HPP
#define VSMC_CORE_SINGLE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc
{

/// \brief A thin wrapper over a complete Particle
/// \ingroup Core
template <typename T>
class SingleParticleBase
{
    public:
    SingleParticleBase(typename Particle<T>::size_type id, Particle<T> *pptr)
        : id_(id), pptr_(pptr)
    {
    }

    typename Particle<T>::size_type id() const { return id_; }

    Particle<T> &particle() const { return *pptr_; }

    typename Particle<T>::rng_type &rng() const { return pptr_->rng(id_); }

    private:
    typename Particle<T>::size_type id_;
    Particle<T> *pptr_;
}; // class SingleParticleBase

/// \brief SingleParticle base class trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(
    SingleParticleBaseType, single_particle_type, SingleParticleBase)

/// \brief A thin wrapper over a complete Particle
/// \ingroup Core
///
/// \details
/// This is the basic SingleParticle available for any type of Particle. To
/// extend it for type `T`. One can either specialize
/// vsmc::SingleParticleBaseTypeTrait<T> or define a class template
/// named `single_particle_type` within `T` with the following minimum
/// requirement.
/// ~~~{.cpp}
/// template <typename S> // S: StateType, such as StateMatrix<Dim, T>
/// struct single_particle_type
/// {
///     typedef IntType size_type;
///     single_particle_type(size_type id, Particle<S> *pptr);
///     size_type id() const;
///     Particle<S> &particle() const;
/// };
/// ~~~
/// Usually you can safely derive `single_particle_type<S>` from
/// SingleParticleBase<S> and add methods specific to `S`.
template <typename T>
class SingleParticle : public SingleParticleBaseType<T>
{
    public:
    SingleParticle(typename Particle<T>::size_type id, Particle<T> *pptr)
        : SingleParticleBaseType<T>(id, pptr)
    {
    }
}; // class SingleParticle

} // namespace vsmc

#endif // VSMC_CORE_SINGLE_PARTICLE_HPP

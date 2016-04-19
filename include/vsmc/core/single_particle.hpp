//============================================================================
// vSMC/include/vsmc/core/single_particle.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#define VSMC_RUNTIME_ASSERT_SINGLE_PARTICLE_COMPARE(sp1, sp2)                 \
    VSMC_RUNTIME_ASSERT((sp1.particle_ptr() == sp2.particle_ptr()),           \
        "COMPARE TWO SingleParticle OBJECTS THAT BELONG TO TWO PARTICLE "     \
        "SYSTEMS");

#define VSMC_RUNTIME_ASSERT_SINGLE_PARTICLE_DIFFERENCE(sp1, sp2)              \
    VSMC_RUNTIME_ASSERT((sp1.particle_ptr() == sp2.particle_ptr()),           \
        "SUBSTRACT TWO SingleParticle OBJECTS THAT BELONG TO TWO PARTICLE "   \
        "SYSTEMS");

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

    Particle<T> *particle_ptr() const { return pptr_; }

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
/// class single_particle_type
/// {
///     public:
///     using size_type = IntType;
///     single_particle_type(size_type id, Particle<S> *pptr);
///     size_type id() const;
///     Particle<S> &particle() const;
/// }; // class single_particle_type
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

    template <typename IntType>
    SingleParticle operator[](IntType n)
    {
        return SingleParticle<T>(static_cast<typename Particle<T>::size_type>(
                                     static_cast<std::ptrdiff_t>(this->id()) +
                                     static_cast<std::ptrdiff_t>(n)),
            this->particle_ptr());
    }

    SingleParticle<T> &operator*() { return *this; }

    const SingleParticle<T> &operator*() const { return *this; }
}; // class SingleParticle

template <typename T>
inline bool operator==(
    const SingleParticle<T> &sp1, const SingleParticle<T> &sp2)
{
    VSMC_RUNTIME_ASSERT_SINGLE_PARTICLE_COMPARE(sp1, sp2);

    return sp1.id() == sp2.id();
}

template <typename T>
inline bool operator!=(
    const SingleParticle<T> &sp1, const SingleParticle<T> &sp2)
{
    VSMC_RUNTIME_ASSERT_SINGLE_PARTICLE_COMPARE(sp1, sp2);

    return sp1.id() != sp2.id();
}

template <typename T>
inline bool operator<(
    const SingleParticle<T> &sp1, const SingleParticle<T> &sp2)
{
    VSMC_RUNTIME_ASSERT_SINGLE_PARTICLE_COMPARE(sp1, sp2);

    return sp1.id() < sp2.id();
}

template <typename T>
inline bool operator>(
    const SingleParticle<T> &sp1, const SingleParticle<T> &sp2)
{
    VSMC_RUNTIME_ASSERT_SINGLE_PARTICLE_COMPARE(sp1, sp2);

    return sp1.id() > sp2.id();
}

template <typename T>
inline bool operator<=(
    const SingleParticle<T> &sp1, const SingleParticle<T> &sp2)
{
    VSMC_RUNTIME_ASSERT_SINGLE_PARTICLE_COMPARE(sp1, sp2);

    return sp1.id() <= sp2.id();
}

template <typename T>
inline bool operator>=(
    const SingleParticle<T> &sp1, const SingleParticle<T> &sp2)
{
    VSMC_RUNTIME_ASSERT_SINGLE_PARTICLE_COMPARE(sp1, sp2);

    return sp1.id() >= sp2.id();
}

template <typename T>
inline SingleParticle<T> &operator++(SingleParticle<T> &sp)
{
    sp = SingleParticle<T>(sp.id() + 1, sp.particle_ptr());

    return sp;
}

template <typename T>
inline SingleParticle<T> operator++(SingleParticle<T> &sp, int)
{
    SingleParticle<T> sp_tmp(sp);
    sp = SingleParticle<T>(sp.id() + 1, sp.particle_ptr());

    return sp_tmp;
}

template <typename T>
inline SingleParticle<T> &operator--(SingleParticle<T> &sp)
{
    sp = SingleParticle<T>(sp.id() - 1, sp.particle_ptr());

    return sp;
}

template <typename T>
inline SingleParticle<T> operator--(SingleParticle<T> &sp, int)
{
    SingleParticle<T> sp_tmp(sp);
    sp = SingleParticle<T>(sp.id() - 1, sp.particle_ptr());

    return sp_tmp;
}

template <typename T, typename IntType>
inline SingleParticle<T> operator+(const SingleParticle<T> &sp, IntType n)
{
    return SingleParticle<T>(static_cast<typename Particle<T>::size_type>(
                                 static_cast<std::ptrdiff_t>(sp.id()) +
                                 static_cast<std::ptrdiff_t>(n)),
        sp.particle_ptr());
}

template <typename T, typename IntType>
inline SingleParticle<T> operator+(IntType n, const SingleParticle<T> &sp)
{
    return SingleParticle<T>(static_cast<typename Particle<T>::size_type>(
                                 static_cast<std::ptrdiff_t>(sp.id()) +
                                 static_cast<std::ptrdiff_t>(n)),
        sp.particle_ptr());
}

template <typename T, typename IntType>
inline SingleParticle<T> operator-(const SingleParticle<T> &sp, IntType n)
{
    return SingleParticle<T>(static_cast<typename Particle<T>::size_type>(
                                 static_cast<std::ptrdiff_t>(sp.id()) -
                                 static_cast<std::ptrdiff_t>(n)),
        sp.particle_ptr());
}

template <typename T, typename IntType>
inline SingleParticle<T> &operator+=(SingleParticle<T> &sp, IntType n)
{
    sp = sp + n;

    return sp;
}

template <typename T, typename IntType>
inline SingleParticle<T> &operator-=(SingleParticle<T> &sp, IntType n)
{
    sp = sp - n;

    return sp;
}

template <typename T>
inline std::ptrdiff_t operator-(
    const SingleParticle<T> &sp1, const SingleParticle<T> &sp2)
{
    VSMC_RUNTIME_ASSERT_SINGLE_PARTICLE_DIFFERENCE(sp1, sp2);

    return static_cast<std::ptrdiff_t>(sp1.id()) -
        static_cast<std::ptrdiff_t>(sp2.id());
}

} // namespace vsmc

#endif // VSMC_CORE_SINGLE_PARTICLE_HPP

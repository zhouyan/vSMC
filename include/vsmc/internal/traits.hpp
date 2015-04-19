//============================================================================
// vSMC/include/vsmc/internal/traits.hpp
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

#ifndef VSMC_INTERNAL_TRAITS_HPP
#define VSMC_INTERNAL_TRAITS_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>
#include <type_traits>

#define VSMC_DEFINE_TYPE_DISPATCH_TRAIT(Outer, Inner, Default)               \
    template <typename T> struct Outer##Trait;                               \
                                                                             \
    namespace internal                                                       \
    {                                                                        \
                                                                             \
    template <typename T> struct Has##Outer##Impl {                          \
        private:                                                             \
        struct char2 {                                                       \
            char c1;                                                         \
            char c2;                                                         \
        };                                                                   \
        template <typename U> static char test(typename U::Inner *);         \
        template <typename U> static char2 test(...);                        \
                                                                             \
        public:                                                              \
        static VSMC_CONSTEXPR const bool value =                             \
            sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char);                   \
    };                                                                       \
                                                                             \
    template <typename T>                                                    \
    struct Has##Outer                                                        \
        : public std::integral_constant<bool, Has##Outer##Impl<T>::value> {  \
    };                                                                       \
                                                                             \
    template <typename T, bool> struct Outer##Dispatch;                      \
                                                                             \
    template <typename T> struct Outer##Dispatch<T, false> {                 \
        typedef Default type;                                                \
    };                                                                       \
                                                                             \
    template <typename T> struct Outer##Dispatch<T, true> {                  \
        typedef typename T::Inner type;                                      \
    };                                                                       \
    }                                                                        \
                                                                             \
    template <typename T> struct Outer##Trait {                              \
        static VSMC_CONSTEXPR const bool value =                             \
            internal::Has##Outer<T>::value;                                  \
        typedef typename internal::Outer##Dispatch<T, value>::type type;     \
    };

#define VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(Outer, Inner, Default)      \
    template <typename T> struct Outer##Trait;                               \
                                                                             \
    namespace internal                                                       \
    {                                                                        \
                                                                             \
    template <typename T> struct Has##Outer##Impl {                          \
        private:                                                             \
        struct char2 {                                                       \
            char c1;                                                         \
            char c2;                                                         \
        };                                                                   \
        template <typename U>                                                \
        static char test(typename U::template Inner<T> *);                   \
        template <typename U> static char2 test(...);                        \
                                                                             \
        public:                                                              \
        static VSMC_CONSTEXPR const bool value =                             \
            sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char);                   \
    };                                                                       \
                                                                             \
    template <typename T>                                                    \
    struct Has##Outer                                                        \
        : public std::integral_constant<bool, Has##Outer##Impl<T>::value> {  \
    };                                                                       \
                                                                             \
    template <typename T, bool> struct Outer##Dispatch;                      \
                                                                             \
    template <typename T> struct Outer##Dispatch<T, false> {                 \
        typedef Default<T> type;                                             \
    };                                                                       \
                                                                             \
    template <typename T> struct Outer##Dispatch<T, true> {                  \
        typedef typename T::template Inner<T> type;                          \
    };                                                                       \
    }                                                                        \
                                                                             \
    template <typename T> struct Outer##Trait {                              \
        static VSMC_CONSTEXPR const bool value =                             \
            internal::Has##Outer<T>::value;                                  \
        typedef typename internal::Outer##Dispatch<T, value>::type type;     \
    };

#define VSMC_DEFINE_METHOD_CHECKER(name, RT, Args)                           \
    template <typename U> struct has_##name##_impl_ {                        \
        private:                                                             \
        struct char2 {                                                       \
            char c1;                                                         \
            char c2;                                                         \
        };                                                                   \
        template <typename V, RT(V::*) Args> struct sfinae_;                 \
        template <typename V, RT(V::*) Args const> struct sfinae_const_;     \
        template <typename V, RT(*) Args> struct sfinae_static_;             \
        template <typename V> static char test(sfinae_<V, &V::name> *);      \
        template <typename V>                                                \
        static char test(sfinae_const_<V, &V::name> *);                      \
        template <typename V>                                                \
        static char test(sfinae_static_<V, &V::name> *);                     \
        template <typename V> static char2 test(...);                        \
                                                                             \
        public:                                                              \
        static VSMC_CONSTEXPR const bool value =                             \
            sizeof(test<U>(VSMC_NULLPTR)) == sizeof(char);                   \
    };                                                                       \
                                                                             \
    template <typename U>                                                    \
    struct has_##name##_                                                     \
        : public std::integral_constant<bool,                                \
                                        has_##name##_impl_<U>::value> {      \
    };

namespace vsmc
{

namespace traits
{

/// \brief Particle::size_type etc., traits
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(SizeType, size_type, std::size_t)

/// \brief Particle::weight_set_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightSetType, weight_set_type, WeightSet)

/// \brief SingleParticle base class trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(SingleParticleBaseType,
                                         single_particle_type,
                                         SingleParticleBase)

/// \brief ConstSingleParticle base class trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(ConstSingleParticleBaseType,
                                         const_single_particle_type,
                                         ConstSingleParticleBase)

#if defined(_OPENMP) && _OPENMP >= 200805  // OpenMP 3.0
template <typename T> struct OMPSizeTypeTrait {
    typedef T type;
};
#else
template <typename T> struct OMPSizeTypeTrait {
    typedef typename std::ptrdiff_t type;
};
#endif

/// \brief SIMD traits
/// \ingroup Traits
template <SIMD> struct SIMDTrait;

/// \brief SSE2 traits
/// \ingroup Traits
template <> struct SIMDTrait<SSE2> {
    static VSMC_CONSTEXPR const std::size_t alignment = 16;
    static VSMC_CONSTEXPR const std::size_t grainsize = 8;
};

/// \brief SSE3 traits
/// \ingroup Traits
template <> struct SIMDTrait<SSE3> : public SIMDTrait<SSE2> {
};

/// \brief SSSE3 traits
/// \ingroup Traits
template <> struct SIMDTrait<SSSE3> : public SIMDTrait<SSE3> {
};

/// \brief SSE4_1 traits
/// \ingroup Traits
template <> struct SIMDTrait<SSE4_1> : public SIMDTrait<SSSE3> {
};

/// \brief SSE4_2 traits
/// \ingroup Traits
template <> struct SIMDTrait<SSE4_2> : public SIMDTrait<SSE4_1> {
};

/// \brief AVX traits
/// \ingroup Traits
template <> struct SIMDTrait<AVX> {
    static VSMC_CONSTEXPR const std::size_t alignment = 32;
    static VSMC_CONSTEXPR const std::size_t grainsize = 8;
};

/// \brief AVX2 traits
/// \ingroup Traits
template <> struct SIMDTrait<AVX2> : public SIMDTrait<AVX> {
};

/// \brief Dimension trait for StateMatrix and StateCL (fixed dimension)
/// \ingroup Traits
template <std::size_t Dim> struct DimTrait {
    static VSMC_CONSTEXPR std::size_t dim() { return Dim; }
};

/// \brief Dimension trait for StateMatrix and StateCL (dynamic dimension)
/// \ingroup Traits
template <> struct DimTrait<Dynamic> {
    DimTrait() : dim_(1) {}

    std::size_t dim() const { return dim_; }

    protected:
    void resize_dim(std::size_t dim) { dim_ = dim; }

    private:
    std::size_t dim_;
};  // struct DimTrait

}  // namespace vsmc::traits

}  // namespace vsmc

#endif  // VSMC_INTERNAL_TRAITS_HPP

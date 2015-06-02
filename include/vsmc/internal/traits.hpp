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

#define VSMC_DEFINE_TYPE_DISPATCH_TRAIT(Outer, Inner, Default)                \
    template <typename T>                                                     \
    struct Outer##Trait;                                                      \
                                                                              \
    namespace internal                                                        \
    {                                                                         \
                                                                              \
    template <typename T>                                                     \
    struct Has##Outer##Impl {                                                 \
        private:                                                              \
        struct char2 {                                                        \
            char c1;                                                          \
            char c2;                                                          \
        };                                                                    \
        template <typename U>                                                 \
        static char test(typename U::Inner *);                                \
        template <typename U>                                                 \
        static char2 test(...);                                               \
                                                                              \
        public:                                                               \
        static constexpr bool value =                                         \
            sizeof(test<T>(nullptr)) == sizeof(char);                         \
    };                                                                        \
                                                                              \
    template <typename T>                                                     \
    struct Has##Outer                                                         \
        : public std::integral_constant<bool, Has##Outer##Impl<T>::value> {   \
    };                                                                        \
                                                                              \
    template <typename T, bool>                                               \
    struct Outer##Dispatch;                                                   \
                                                                              \
    template <typename T>                                                     \
    struct Outer##Dispatch<T, false> {                                        \
        typedef Default type;                                                 \
    };                                                                        \
                                                                              \
    template <typename T>                                                     \
    struct Outer##Dispatch<T, true> {                                         \
        typedef typename T::Inner type;                                       \
    };                                                                        \
    }                                                                         \
                                                                              \
    template <typename T>                                                     \
    struct Outer##Trait {                                                     \
        static constexpr bool value = internal::Has##Outer<T>::value;         \
        typedef typename internal::Outer##Dispatch<T, value>::type type;      \
    };                                                                        \
                                                                              \
    template <typename T>                                                     \
    using Outer = typename Outer##Trait<T>::type;

#define VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(Outer, Inner, Default)       \
    template <typename T>                                                     \
    struct Outer##Trait;                                                      \
                                                                              \
    namespace internal                                                        \
    {                                                                         \
                                                                              \
    template <typename T>                                                     \
    struct Has##Outer##Impl {                                                 \
        private:                                                              \
        struct char2 {                                                        \
            char c1;                                                          \
            char c2;                                                          \
        };                                                                    \
        template <typename U>                                                 \
        static char test(typename U::template Inner<T> *);                    \
        template <typename U>                                                 \
        static char2 test(...);                                               \
                                                                              \
        public:                                                               \
        static constexpr bool value =                                         \
            sizeof(test<T>(nullptr)) == sizeof(char);                         \
    };                                                                        \
                                                                              \
    template <typename T>                                                     \
    struct Has##Outer                                                         \
        : public std::integral_constant<bool, Has##Outer##Impl<T>::value> {   \
    };                                                                        \
                                                                              \
    template <typename T, bool>                                               \
    struct Outer##Dispatch;                                                   \
                                                                              \
    template <typename T>                                                     \
    struct Outer##Dispatch<T, false> {                                        \
        typedef Default<T> type;                                              \
    };                                                                        \
                                                                              \
    template <typename T>                                                     \
    struct Outer##Dispatch<T, true> {                                         \
        typedef typename T::template Inner<T> type;                           \
    };                                                                        \
    }                                                                         \
                                                                              \
    template <typename T>                                                     \
    struct Outer##Trait {                                                     \
        static constexpr bool value = internal::Has##Outer<T>::value;         \
        typedef typename internal::Outer##Dispatch<T, value>::type type;      \
    };                                                                        \
                                                                              \
    template <typename T>                                                     \
    using Outer = typename Outer##Trait<T>::type;

#define VSMC_DEFINE_METHOD_CHECKER(name, RT, Args)                            \
    template <typename U>                                                     \
    struct has_##name##_impl_ {                                               \
        private:                                                              \
        struct char2 {                                                        \
            char c1;                                                          \
            char c2;                                                          \
        };                                                                    \
        template <typename V, RT(V::*) Args>                                  \
        struct sfinae_;                                                       \
        template <typename V, RT(V::*) Args const>                            \
        struct sfinae_const_;                                                 \
        template <typename V, RT(*) Args>                                     \
        struct sfinae_static_;                                                \
        template <typename V>                                                 \
        static char test(sfinae_<V, &V::name> *);                             \
        template <typename V>                                                 \
        static char test(sfinae_const_<V, &V::name> *);                       \
        template <typename V>                                                 \
        static char test(sfinae_static_<V, &V::name> *);                      \
        template <typename V>                                                 \
        static char2 test(...);                                               \
                                                                              \
        public:                                                               \
        static constexpr bool value =                                         \
            sizeof(test<U>(nullptr)) == sizeof(char);                         \
    };                                                                        \
                                                                              \
    template <typename U>                                                     \
    struct has_##name##_                                                      \
        : public std::integral_constant<bool, has_##name##_impl_<U>::value> { \
    };

namespace vsmc
{

/// \brief Particle::size_type etc., traits
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(SizeType, size_type, std::size_t)

} // namespace vsmc

#endif // VSMC_INTERNAL_TRAITS_HPP

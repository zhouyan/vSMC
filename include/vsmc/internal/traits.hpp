#ifndef VSMC_INTERNAL_TRAITS_HPP
#define VSMC_INTERNAL_TRAITS_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/forward.hpp>
#include <vsmc/cxx11/type_traits.hpp>

#include <string>

#define VSMC_DEFINE_TYPE_DISPATCH_TRAIT(Outer, Inner, Default)               \
template <typename T> struct Outer##Trait;                                   \
                                                                             \
namespace internal {                                                         \
template <typename T>                                                        \
struct Has##Outer##Impl                                                      \
{                                                                            \
    private :                                                                \
                                                                             \
    struct char2 {char c1; char c2;};                                        \
    template <typename U> static char test (typename U::Inner *);            \
    template <typename U> static char2 test (...);                           \
                                                                             \
    public :                                                                 \
                                                                             \
    enum {value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char)};            \
};                                                                           \
                                                                             \
template <typename T>                                                        \
struct Has##Outer :                                                          \
    public cxx11::integral_constant<bool, Has##Outer##Impl<T>::value> {};    \
                                                                             \
template <typename T, bool> struct Outer##Dispatch;                          \
                                                                             \
template <typename T> struct Outer##Dispatch<T, false>                       \
{typedef Default type;};                                                     \
                                                                             \
template <typename T> struct Outer##Dispatch<T, true>                        \
{typedef typename T::Inner type;};                                           \
}                                                                            \
                                                                             \
template <typename T> struct Outer##Trait                                    \
{                                                                            \
    enum {value = internal::Has##Outer<T>::value};                           \
    typedef typename internal::Outer##Dispatch<T, value>::type type;         \
}; // VSMC_DEFINE_TYPE_DISPATCH_TRAIT

#define VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(Outer, Inner, Default)      \
template <typename T> struct Outer##Trait;                                   \
                                                                             \
namespace internal {                                                         \
template <typename T>                                                        \
struct Has##Outer##Impl                                                      \
{                                                                            \
    private :                                                                \
                                                                             \
    struct char2 {char c1; char c2;};                                        \
    template <typename U> static char test (typename U::template Inner<T> *);\
    template <typename U> static char2 test (...);                           \
                                                                             \
    public :                                                                 \
                                                                             \
    enum {value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char)};            \
};                                                                           \
                                                                             \
template <typename T>                                                        \
struct Has##Outer :                                                          \
    public cxx11::integral_constant<bool, Has##Outer##Impl<T>::value> {};    \
                                                                             \
template <typename T,  bool> struct Outer##Dispatch;                         \
                                                                             \
template <typename T> struct Outer##Dispatch<T, false>                       \
{typedef Default<T> type;};                                                  \
                                                                             \
template <typename T> struct Outer##Dispatch<T, true>                        \
{typedef typename T::template Inner<T> type;};                               \
}                                                                            \
                                                                             \
template <typename T> struct Outer##Trait                                    \
{                                                                            \
    enum {value = internal::Has##Outer<T>::value};                           \
    typedef typename internal::Outer##Dispatch<T, value>::type type;         \
}; // VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT

namespace vsmc {

namespace traits {

/// \brief Particle::size_type etc., traits
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(SizeType, size_type, std::size_t)

/// \brief Particle::weight_set_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightSetType, weight_set_type, WeightSet)

/// \brief SingleParticle base class trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(SingleParticleBaseType,
        single_particle_type, SingleParticleBase)

/// \brief ConstSingleParticle base class trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(ConstSingleParticleBaseType,
        const_single_particle_type, ConstSingleParticleBase)

#if defined(_OPENMP) && _OPENMP >= 200805 // OpenMP 3.0
template <typename T> struct OMPSizeTypeTrait {typedef T type;};
#else
template <typename T> struct OMPSizeTypeTrait
{typedef typename std::ptrdiff_t type;};
#endif

/// \brief Dimension trait for StateMatrix and StateCL (fixed dimension)
/// \ingroup Traits
template <std::size_t Dim>
struct DimTrait {static VSMC_CONSTEXPR std::size_t dim () {return Dim;}};

/// \brief Dimension trait for StateMatrix and StateCL (dynamic dimension)
/// \ingroup Traits
template <>
struct DimTrait<Dynamic>
{
    DimTrait () : dim_(1) {}

    std::size_t dim () const {return dim_;}

    protected :

    void resize_dim (std::size_t dim) {dim_ = dim;}

    private :

    std::size_t dim_;
}; // struct DimTrait

} // namespace vsmc::traits

} // namespace vsmc

#endif // VSMC_INTERNAL_TRAITS_HPP

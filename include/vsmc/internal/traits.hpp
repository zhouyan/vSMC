#ifndef VSMC_INTERNAL_TRAITS_HPP
#define VSMC_INTERNAL_TRAITS_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>
#include <vsmc/cxx11/type_traits.hpp>

#include <string>

/// \brief Define a type dispatch trait
/// \ingroup Traits
///
/// \details
/// This macro define a class template
/// \code
/// template <typename T> struct OuterTrait;
/// \endcode
/// with the following members
/// - Member enumurator `OuterTrait::value`: true if `T::Inner` exits and is a
/// type
/// - Member type `OuterTrait::type`: same as `T::Inner` if `value == true`,
/// otherwise `Default`.
/// - Three low level implementation class templates are also defined
/// \code
/// template <typename T> struct HasOuterImpl;
/// template <typename T> struct HasOuter;
/// template <typename T, bool> struct OuterDispatch;
/// \endcode
///
/// **Example**
/// \code
/// VSMC_DEFINE_TYPE_DISPATCH_TRAIT(SizeType, size_type, std::size_t);
///
/// struct Empty {};
/// struct Stack {typedef int size_type;};
///
/// SizeTypeTrait<Empty>::value; // false
/// SizeTypeTrait<Empty>::type;  // std::size_t
/// SizeTypeTrait<Stack>::value; // true
/// SizeTypeTrait<Stack>::type;  // Stack::size_type
/// \endcode
#define VSMC_DEFINE_TYPE_DISPATCH_TRAIT(Outer, Inner, Default)               \
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
                                                                             \
template <typename T> struct Outer##Trait                                    \
{                                                                            \
    enum {value = Has##Outer<T>::value};                                     \
    typedef typename Outer##Dispatch<T, value>::type type;                   \
};

/// \brief Define a class template dispatch trait
/// \ingroup Traits
///
/// \details
/// This macro define a class template
/// \code
/// template <typename T, typename V> struct OuterTrait;
/// \endcode
/// with the following members
/// - Member enumurator `OuterTrait::value`: true if `T::Inner` exits and is a
/// class tempalte that can take `V` as its template parameter. The clas
/// template can have multiple template parameters, however all but the first
/// need to have default arguments.
/// - Member type `OuterTrait::type`: same as `T::Inner<T>` if
/// `value == true`, otherwise `Default<V>`.
/// - Three low level implementation class templates are also defined
/// \code
/// template <typename T, typename V> struct HasOuterImpl;
/// template <typename T, typename V> struct HasOuter;
/// template <typename T, typename V, bool> struct OuterDispatch;
/// \endcode
///
/// **Example**
/// \code
/// VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(VecType, vec_type, std::vector);
///
/// struct Empty {};
/// struct Stack { template <typename T> struct vec_type {/*...*/}; };
///
/// VecTypeTrait<Empty, int>::value; // false
/// VecTypeTrait<Empty, int>::type;  // std::vector<int>
/// VecTypeTrait<Stack, int>::value; // true
/// VecTypeTrait<Stack, int>::type;  // Stack::vec_type<int>
/// \endcode
#define VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(Outer, Inner, Default)      \
template <typename T, typename V>                                            \
struct Has##Outer##Impl                                                      \
{                                                                            \
    private :                                                                \
                                                                             \
    struct char2 {char c1; char c2;};                                        \
    template <typename U> static char test (typename U::template Inner<V> *);\
    template <typename U> static char2 test (...);                           \
                                                                             \
    public :                                                                 \
                                                                             \
    enum {value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char)};            \
};                                                                           \
                                                                             \
template <typename T, typename V>                                            \
struct Has##Outer :                                                          \
    public cxx11::integral_constant<bool, Has##Outer##Impl<T, V>::value> {}; \
                                                                             \
template <typename T, typename V, bool> struct Outer##Dispatch;              \
                                                                             \
template <typename T, typename V> struct Outer##Dispatch<T, V, false>        \
{typedef Default<V> type;};                                                  \
                                                                             \
template <typename T, typename V> struct Outer##Dispatch<T, V, true>         \
{typedef typename T::template Inner<V> type;};                               \
                                                                             \
template <typename T, typename V> struct Outer##Trait                        \
{                                                                            \
    enum {value = Has##Outer<T, V>::value};                                  \
    typedef typename Outer##Dispatch<T, V, value>::type type;                \
};

#define VSMC_DEFINE_SMP_MF_CHECKER(name, RT, Args)                           \
template <typename U>                                                        \
struct has_##name##_non_static_                                              \
{                                                                            \
    private :                                                                \
                                                                             \
    struct char2 {char c1; char c2;};                                        \
    template <typename V, RT (V::*) Args> struct sfinae_;                    \
    template <typename V> static char test (sfinae_<V, &V::name> *);         \
    template <typename V> static char2 test (...);                           \
                                                                             \
    public :                                                                 \
                                                                             \
    enum {value = sizeof(test<U>(VSMC_NULLPTR)) == sizeof(char)};            \
};                                                                           \
                                                                             \
template <typename U>                                                        \
struct has_##name##_static_                                                  \
{                                                                            \
    private :                                                                \
                                                                             \
    struct char2 {char c1; char c2;};                                        \
    template <typename V, RT (*) Args> struct sfinae_;                       \
    template <typename V> static char test (sfinae_<V, &V::name> *);         \
    template <typename V> static char2 test (...);                           \
                                                                             \
    public :                                                                 \
                                                                             \
    enum {value = sizeof(test<U>(VSMC_NULLPTR)) == sizeof(char)};            \
};                                                                           \
                                                                             \
template <typename U>                                                        \
struct has_##name##_ : public cxx11::integral_constant<bool,                 \
    has_##name##_non_static_<U>::value ||                                    \
    has_##name##_static_<U>::value> {};

namespace vsmc {

/// \brief Type traits
/// \ingroup Traits
namespace traits {

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(SizeType, size_type, std::size_t)

#if defined(_OPENMP) && _OPENMP >= 200805 // OpenMP 3.0
template <typename T> struct OMPSizeTypeTrait {typedef T type;};
#else
template <typename T> struct OMPSizeTypeTrait
{typedef typename std::ptrdiff_t type;};
#endif

template <std::size_t Dim>
class DimTrait
{
    public :

    static VSMC_CONSTEXPR std::size_t dim () {return Dim;}
};

template <>
class DimTrait<Dynamic>
{
    public :

    DimTrait () : dim_(1) {}

    std::size_t dim () const {return dim_;}

    protected :

    void resize_dim (std::size_t dim) {dim_ = dim;}

    private :

    std::size_t dim_;
};

} // namespace vsmc::traits

} // namespace vsmc

#endif // VSMC_INTERNAL_TRAITS_HPP

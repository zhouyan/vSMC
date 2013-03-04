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
/// vsmc::traits::SizeTypeTrait<Empty>::value; // false
/// vsmc::traits::SizeTypeTrait<Empty>::type;  // std::size_t
/// vsmc::traits::SizeTypeTrait<Stack>::value; // true
/// vsmc::traits::SizeTypeTrait<Stack>::type;  // Stack::size_type
/// \endcode
#define VSMC_DEFINE_TYPE_DISPATCH_TRAIT(Outer, Inner, Default)                \
namespace vsmc { namespace traits {                                           \
                                                                              \
template <typename T>                                                         \
struct Has##Outer##Impl                                                       \
{                                                                             \
    private :                                                                 \
                                                                              \
    struct char2 {char c1; char c2;};                                         \
    template <typename U> static char test (typename U::Inner *);             \
    template <typename U> static char2 test (...);                            \
                                                                              \
    public :                                                                  \
                                                                              \
    enum {value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char)};             \
};                                                                            \
                                                                              \
template <typename T>                                                         \
struct Has##Outer :                                                           \
    public cxx11::integral_constant<bool, Has##Outer##Impl<T>::value> {};     \
                                                                              \
template <typename T, bool> struct Outer##Dispatch                            \
{typedef Default type;};                                                      \
                                                                              \
template <typename T> struct Outer##Dispatch<T, true>                         \
{typedef typename T::Inner type;};                                            \
                                                                              \
template <typename T> struct Outer##Trait                                     \
{                                                                             \
    enum {value = Has##Outer<T>::value};                                      \
    typedef typename Outer##Dispatch<T, value>::type type;                    \
};                                                                            \
                                                                              \
} } // namespace vsmc::traits

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
/// vsmc::traits::VecTypeTrait<Empty, int>::value; // false
/// vsmc::traits::VecTypeTrait<Empty, int>::type;  // std::vector<int>
/// vsmc::traits::VecTypeTrait<Stack, int>::value; // true
/// vsmc::traits::VecTypeTrait<Stack, int>::type;  // Stack::vec_type<int>
/// \endcode
#define VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(Outer, Inner, Default)       \
namespace vsmc { namespace traits {                                           \
                                                                              \
template <typename T, typename V>                                             \
struct Has##Outer##Impl                                                       \
{                                                                             \
    private :                                                                 \
                                                                              \
    struct char2 {char c1; char c2;};                                         \
    template <typename U> static char test (typename U::template Inner<V> *); \
    template <typename U> static char2 test (...);                            \
                                                                              \
    public :                                                                  \
                                                                              \
    enum {value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char)};             \
};                                                                            \
                                                                              \
template <typename T, typename V>                                             \
struct Has##Outer :                                                           \
    public cxx11::integral_constant<bool, Has##Outer##Impl<T, V>::value> {};  \
                                                                              \
template <typename T, typename V, bool> struct Outer##Dispatch                \
{typedef Default<V> type;};                                                   \
                                                                              \
template <typename T, typename V> struct Outer##Dispatch<T, V, true>          \
{typedef typename T::template Inner<V> type;};                                \
                                                                              \
template <typename T, typename V> struct Outer##Trait                         \
{                                                                             \
    enum {value = Has##Outer<T, V>::value};                                   \
    typedef typename Outer##Dispatch<T, V, value>::type type;                 \
};                                                                            \
                                                                              \
} } // namespace vsmc::traits

#define VSMC_DEFINE_MF_CHECKER(Outer, Inner, RT, Args)                        \
namespace vsmc { namespace traits {                                           \
                                                                              \
template <typename T>                                                         \
struct Has##Outer##Impl                                                       \
{                                                                             \
    private :                                                                 \
                                                                              \
    struct char2 {char c1; char c2;};                                         \
    template <typename U, RT (U::*) Args> struct sfinae_;                     \
    template <typename U> static char test (sfinae_<U, &U::Inner> *);         \
    template <typename U> static char2 test (...);                            \
                                                                              \
    public :                                                                  \
                                                                              \
    enum {value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char)};             \
};                                                                            \
                                                                              \
template <typename T>                                                         \
struct Has##Outer :                                                           \
    public cxx11::integral_constant<bool, Has##Outer##Impl<T>::value> {};     \
                                                                              \
} } // namespace vsmc::traits

#define VSMC_DEFINE_STATIC_MF_CHECKER(Outer, Inner, RT, Args)                 \
namespace vsmc { namespace traits {                                           \
                                                                              \
template <typename T>                                                         \
struct HasStatic##Outer##Impl                                                 \
{                                                                             \
    private :                                                                 \
                                                                              \
    struct char2 {char c1; char c2;};                                         \
    template <typename U, RT (*) Args> struct sfinae_;                        \
    template <typename U> static char test (sfinae_<U, &U::Inner> *);         \
    template <typename U> static char2 test (...);                            \
                                                                              \
    public :                                                                  \
                                                                              \
    enum {value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char)};             \
};                                                                            \
                                                                              \
template <typename T>                                                         \
struct HasStatic##Outer :                                                     \
    public cxx11::integral_constant<bool, HasStatic##Outer##Impl<T>::value>{};\
                                                                              \
} } // namespace vsmc::traits

#define VSMC_DEFINE_SMP_MF_CHECKER(name, RT, Args)                            \
template <typename U>                                                         \
struct has_##name##_non_static_                                               \
{                                                                             \
    private :                                                                 \
                                                                              \
    struct char2 {char c1; char c2;};                                         \
    template <typename V, RT (V::*) Args> struct sfinae_;                     \
    template <typename V> static char test (sfinae_<V, &V::name> *);          \
    template <typename V> static char2 test (...);                            \
                                                                              \
    public :                                                                  \
                                                                              \
    enum {value = sizeof(test<U>(VSMC_NULLPTR)) == sizeof(char)};             \
};                                                                            \
                                                                              \
template <typename U>                                                         \
struct has_##name##_static_                                                   \
{                                                                             \
    private :                                                                 \
                                                                              \
    struct char2 {char c1; char c2;};                                         \
    template <typename V, RT (*) Args> struct sfinae_;                        \
    template <typename V> static char test (sfinae_<V, &V::name> *);          \
    template <typename V> static char2 test (...);                            \
                                                                              \
    public :                                                                  \
                                                                              \
    enum {value = sizeof(test<U>(VSMC_NULLPTR)) == sizeof(char)};             \
};                                                                            \
                                                                              \
template <typename U>                                                         \
struct has_##name##_ : public cxx11::integral_constant<bool,                  \
    has_##name##_non_static_<U>::value ||                                     \
    has_##name##_static_<U>::value> {};

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(SizeType, size_type, std::size_t)
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(WeightSetType, weight_set_type, WeightSet);
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(OpenCLDeviceType, opencl_device_type,
        cxx11::false_type)

VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(SingleParticleType,
        single_particle_type, SingleParticleBase)
VSMC_DEFINE_TYPE_TEMPLATE_DISPATCH_TRAIT(ConstSingleParticleType,
        const_single_particle_type, ConstSingleParticleBase)

VSMC_DEFINE_STATIC_MF_CHECKER(CheckOpenCLPlatform, check_opencl_platform,
        bool, (const std::string &))
VSMC_DEFINE_STATIC_MF_CHECKER(CheckOpenCLDevice, check_opencl_device,
        bool, (const std::string &))

namespace vsmc { namespace traits {

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

template <typename D>
struct IsDerivedFromStateCLImpl
{
    private :

    struct char2 {char c1; char c2;};

    template <std::size_t Dim, typename T, typename ID>
    static char test (const StateCL<Dim, T, ID> *);
    static char2 test (...);

    public :

   enum {value = sizeof(test(static_cast<const D *>(0))) == sizeof(char)};
};

template <typename D>
struct IsDerivedFromStateCL :
    public cxx11::integral_constant<bool,IsDerivedFromStateCLImpl<D>::value>{};

template <typename ID, bool>
struct CheckOpenCLPlatformDispatch
{static bool check (const std::string &name) {return true;}};

template <typename ID>
struct CheckOpenCLPlatformDispatch<ID, true>
{
    static bool check (const std::string &name)
    {return ID::check_opencl_platform(name);}
};

template <typename ID, bool>
struct CheckOpenCLDeviceDispatch
{static bool check (const std::string &name) {return true;}};

template <typename ID>
struct CheckOpenCLDeviceDispatch<ID, true>
{
    static bool check (const std::string &name)
    {return ID::check_opencl_device(name);}
};

template <typename ID>
struct CheckOpenCLPlatformTrait :
    public CheckOpenCLPlatformDispatch<
    ID, HasStaticCheckOpenCLPlatform<ID>::value> {};

template <typename ID>
struct CheckOpenCLDeviceTrait :
    public CheckOpenCLDeviceDispatch<
    ID, HasStaticCheckOpenCLDevice<ID>::value> {};

} } // namespace vsmc::traits

#endif // VSMC_INTERNAL_TRAITS_HPP

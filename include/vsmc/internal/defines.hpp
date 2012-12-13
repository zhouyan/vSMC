#ifndef VSMC_INTERNAL_DEFINES_HPP
#define VSMC_INTERNAL_DEFINES_HPP

#include <vsmc/internal/config.hpp>

// Avoid MSVC stupid behavior
#define VSMC_MINMAX_NO_EXPANSION

// Type dispatcher
#define VSMC_DEFINE_TYPE_DISPATCH_TRAIT(OuterType, InnerType, DefaultType)   \
namespace vsmc { namespace traits {                                          \
                                                                             \
template <typename T>                                                        \
struct Has##OuterType##Impl                                                  \
{                                                                            \
    private :                                                                \
                                                                             \
    struct char2 {char c1; char c2;};                                        \
    template <typename U> static char test (typename U::InnerType *);        \
    template <typename U> static char2 test (...);                           \
                                                                             \
    public :                                                                 \
                                                                             \
    enum {value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char)};            \
};                                                                           \
                                                                             \
template <typename T>                                                        \
struct Has##OuterType :                                                      \
    public cxx11::integral_constant <bool, Has##OuterType##Impl<T>::value>   \
{};                                                                          \
                                                                             \
template <typename T, bool> struct OuterType##Dispatch;                      \
                                                                             \
template <typename T> struct OuterType##Dispatch<T, true>                    \
{typedef typename T::InnerType type;};                                       \
                                                                             \
template <typename T> struct OuterType##Dispatch<T, false>                   \
{typedef DefaultType type;};                                                 \
                                                                             \
                                                                             \
template <typename T> struct OuterType##Trait                                \
{                                                                            \
    enum {value = traits::Has##OuterType<T>::value};                         \
    typedef typename traits::OuterType##Dispatch<T, value>::type type;       \
};                                                                           \
                                                                             \
} }

#define VSMC_DEFINE_MEMBER_FUNCTION_CHECKER(OuterMF, InnerMF, RT, Args)      \
namespace vsmc { namespace traits {                                          \
                                                                             \
template <typename T>                                                        \
struct Has##OuterMF##Impl                                                    \
{                                                                            \
    private :                                                                \
                                                                             \
    struct char2 {char c1; char c2;};                                        \
    template <typename U, RT (U::*) Args> struct sfinae_;                    \
    template <typename U> static char test (sfinae_<U, &U::InnerMF> *);      \
    template <typename U> static char2 test(...);                            \
                                                                             \
    public :                                                                 \
                                                                             \
    enum {value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char)};            \
};                                                                           \
                                                                             \
template <typename T>                                                        \
struct Has##OuterMF :                                                        \
    public cxx11::integral_constant<bool, Has##OuterMF##Impl<T>::value>      \
{};                                                                          \
                                                                             \
} }

namespace vsmc {

enum {Dynamic};
enum MatrixOrder     {RowMajor = 101, ColMajor = 102};
enum MatrixTranspose {NoTrans = 111, Trans = 112, ConjTrans = 113};

} // namespace vsmc

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(SizeType, size_type, VSMC_SIZE_TYPE);

#endif // VSMC_INTERNAL_DEFINES_HPP

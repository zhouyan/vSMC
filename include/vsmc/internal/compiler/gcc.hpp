#ifndef VSMC_INTERNAL_COMPILER_GCC_HPP
#define VSMC_INTERNAL_COMPILER_GCC_HPP

#ifdef __GXX_EXPERIMENTAL_CXX0X__

#if __GNUC__ >= 4 && __GNUC_MINOR >= 3
#ifndef VSMC_HAS_CXX11_AUTO_TYPE
#define VSMC_HAS_CXX11_AUTO_TYPE 1
#endif
#else // __GNUC__ >= 4 && __GNUC_MINOR >= 3
#ifndef VSMC_HAS_CXX11_AUTO_TYPE
#define VSMC_HAS_CXX11_AUTO_TYPE 0
#endif
#endif // __GNUC__ >= 4 && __GNUC_MINOR >= 3

#if __GNUC__ >= 4 && __GNUC_MINOR >= 6
#ifndef VSMC_HAS_CXX11_CONSTEXPR
#define VSMC_HAS_CXX11_CONSTEXPR 1
#endif
#else // __GNUC__ >= 4 && __GNUC_MINOR >= 6
#ifndef VSMC_HAS_CXX11_CONSTEXPR
#define VSMC_HAS_CXX11_CONSTEXPR 0
#endif
#endif // __GNUC__ >= 4 && __GNUC_MINOR >= 6

#if __GNUC__ >= 4 && __GNUC_MINOR >= 4
#ifndef VSMC_HAS_CXX11_DECLTYPE
#define VSMC_HAS_CXX11_DECLTYPE 1
#endif
#else // __GNUC__ >= 4 && __GNUC_MINOR >= 4
#ifndef VSMC_HAS_CXX11_DECLTYPE
#define VSMC_HAS_CXX11_DECLTYPE 0
#endif
#endif // __GNUC__ >= 4 && __GNUC_MINOR >= 4

#if __GNUC__ >= 4 && __GNUC_MINOR >= 5
#ifndef VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
#define VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS 1
#endif
#else // __GNUC__ >= 4 && __GNUC_MINOR >= 5
#ifndef VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
#define VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS 0
#endif
#endif // __GNUC__ >= 4 && __GNUC_MINOR >= 5

#endif // __GXX_EXPERIMENTAL_CXX0X__

#endif // VSMC_INTERNAL_COMPILER_GCC_HPP

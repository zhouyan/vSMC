#ifndef VSMC_INTERNAL_COMPILER_GCC_HPP
#define VSMC_INTERNAL_COMPILER_GCC_HPP

#if defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L

#if __GNUC__ >= 4 && __GNUC_MINOR__ >= 3
#ifndef VSMC_HAS_CXX11_AUTO_TYPE
#define VSMC_HAS_CXX11_AUTO_TYPE 1
#endif
#endif

#if __GNUC__ >= 4 && __GNUC_MINOR__ >= 6
#ifndef VSMC_HAS_CXX11_CONSTEXPR
#define VSMC_HAS_CXX11_CONSTEXPR 1
#endif
#endif

#if __GNUC__ >= 4 && __GNUC_MINOR__ >= 4
#ifndef VSMC_HAS_CXX11_DECLTYPE
#define VSMC_HAS_CXX11_DECLTYPE 1
#endif
#endif

#if __GNUC__ >= 4 && __GNUC_MINOR >= 5
#ifndef VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
#define VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS 1
#endif
#endif

#if __GNUC__ >= 4 && __GNUC_MINOR__ >= 6
#ifndef VSMC_HAS_CXX11_NULLPTR
#define VSMC_HAS_CXX11_NULLPTR 1
#endif
#endif

#endif // defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L

#endif // VSMC_INTERNAL_COMPILER_GCC_HPP

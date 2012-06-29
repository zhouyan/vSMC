#ifndef VSMC_INTERNAL_COMPILER_GCC_HPP
#define VSMC_INTERNAL_COMPILER_GCC_HPP

#if __cplusplus >= 201103L

#if __has_feature(cxx_auto_type)
#ifndef VSMC_HAS_CXX11_AUTO_TYPE
#define VSMC_HAS_CXX11_AUTO_TYPE 1
#endif
#endif

#if __has_feature(cxx_constexpr)
#ifndef VSMC_HAS_CXX11_CONSTEXPR
#define VSMC_HAS_CXX11_CONSTEXPR 1
#endif
#endif

#if __has_feature(cxx_decltype)
#ifndef VSMC_HAS_CXX11_DECLTYPE
#define VSMC_HAS_CXX11_DECLTYPE 1
#endif
#endif

#if __has_feature(cxx_explicit_conversions)
#ifndef VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
#define VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS 1
#endif
#endif

#if __has_feature(cxx_nullptr)
#ifndef VSMC_HAS_CXX11_NULLPTR
#define VSMC_HAS_CXX11_NULLPTR 1
#endif
#endif

#endif // __cplusplus >= 201103L

#endif // VSMC_INTERNAL_COMPILER_GCC_HPP

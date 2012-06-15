#ifndef VSMC_INTERNAL_CONFIG_HPP
#define VSMC_INTERNAL_CONFIG_HPP

#ifdef NDEBUG
#define VSMC_NDEBUG
#endif

#ifdef VSMC_USE_STD_FUNCTION
#define VSMC_HAS_CXX11LIB_FUNCTION 1
#endif

#ifdef VSMC_USE_STD_RANDOM
#define VSMC_HAS_CXX11LIB_RANDOM 1
#endif

#ifdef VSMC_USE_STD_TYPE_TRAITS
#define VSMC_HAS_CXX11LIB_TYPE_TRAITS 1
#endif

#include <vsmc/internal/compiler.hpp>

#ifndef VSMC_HAS_CXX11_AUTO_TYPE
#define VSMC_HAS_CXX11_AUTO_TYPE 0
#endif

#ifndef VSMC_HAS_CXX11_CONSTEXPR
#define VSMC_HAS_CXX11_CONSTEXPR 0
#endif

#ifndef VSMC_HAS_CXX11_DECLTYPE
#define VSMC_HAS_CXX11_DECLTYPE 0
#endif

#ifndef VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
#define VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS 0
#endif

#ifndef VSMC_HAS_CXX11LIB_FUNCTION
#define VSMC_HAS_CXX11LIB_FUNCTION 0
#endif

#ifndef VSMC_HAS_CXX11LIB_RANDOM
#define VSMC_HAS_CXX11LIB_RANDOM 0
#endif

#ifndef VSMC_HAS_CXX11LIB_TYPE_TRAITS
#define VSMC_HAS_CXX11LIB_TYPE_TRAITS 0
#endif

#endif // VSMC_INTERNAL_CONFIG_HPP

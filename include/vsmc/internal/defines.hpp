#ifndef VSMC_INTERNAL_DEFINES_HPP
#define VSMC_INTERNAL_DEFINES_HPP

#include <vsmc/internal/config.hpp>

// Avoid MSVC stupid behavior
#define VSMC_MACRO_NO_EXPANSION

#if !defined(NDEBUG) || VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_CRTP_DESTRUCTOR_PREFIX virtual
#else
#define VSMC_CRTP_DESTRUCTOR_PREFIX
#endif

#if  VSMC_HAS_CXX11_CONSTEXPR
#define VSMC_CONSTEXPR constexpr
#else
#define VSMC_CONSTEXPR
#endif

#if VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
#define VSMC_EXPLICIT_OPERATOR explicit
#else
#define VSMC_EXPLICIT_OPERATOR
#endif

#if VSMC_HAS_CXX11_NOEXCEPT
#define VSMC_NOEXCEPT noexcept
#else
#define VSMC_NOEXCEPT
#endif

#if VSMC_HAS_CXX11_NULLPTR && VSMC_HAS_CXX11LIB_FUNCTIONAL
#define VSMC_NULLPTR nullptr
#else
#define VSMC_NULLPTR 0
#endif

namespace vsmc {

enum {Dynamic};

enum MatrixOrder {RowMajor = 101, ColMajor = 102};

enum ResampleScheme {
    Multinomial,
    Residual,
    Stratified,
    Systematic,
    ResidualStratified,
    ResidualSystematic
};

} // namespace vsmc

#endif // VSMC_INTERNAL_DEFINES_HPP

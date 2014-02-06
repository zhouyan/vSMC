#ifndef VSMC_INTERNAL_DEFINES_HPP
#define VSMC_INTERNAL_DEFINES_HPP

#include <vsmc/internal/config.hpp>

// Avoid MSVC stupid behavior
#define VSMC_MACRO_NO_EXPANSION

/// \brief CRTP style base classes (virtual) destructor
/// \ingroup Definitions
///
/// \details
/// This macro is defined to `virtual` if compiled in debug mode, otherwise
/// (`-DNDEBUG`) it is empty.
#if !defined(NDEBUG) || VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_CRTP_DESTRUCTOR_PREFIX virtual
#else
#define VSMC_CRTP_DESTRUCTOR_PREFIX
#endif

/// \brief constexpr
/// \ingroup Definitions
#if  VSMC_HAS_CXX11_CONSTEXPR
#define VSMC_CONSTEXPR constexpr
#else
#define VSMC_CONSTEXPR
#endif

/// \brief Explicit operator
/// \ingroup Definitions
#if VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
#define VSMC_EXPLICIT_OPERATOR explicit
#else
#define VSMC_EXPLICIT_OPERATOR
#endif

/// \brief noexcept
/// \ingroup Definitions
#if VSMC_HAS_CXX11_NOEXCEPT
#define VSMC_NOEXCEPT noexcept
#else
#define VSMC_NOEXCEPT
#endif

/// \brief nullptr
/// \ingroup Definitions
#if VSMC_HAS_CXX11_NULLPTR && VSMC_HAS_CXX11LIB_FUNCTIONAL
#define VSMC_NULLPTR nullptr
#else
#define VSMC_NULLPTR 0
#endif

namespace vsmc {

/// \brief Dynamic dimension
/// \ingroup Definitions
enum {Dynamic};

/// \brief Matrix order
/// \ingroup Definitions
enum MatrixOrder {RowMajor = 101, ColMajor = 102};

} // namespace vsmc

#endif // VSMC_INTERNAL_DEFINES_HPP

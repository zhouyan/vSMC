//============================================================================
// include/vsmc/internal/defines.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distributed under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_INTERNAL_DEFINES_HPP
#define VSMC_INTERNAL_DEFINES_HPP

#include <vsmc/internal/config.hpp>

/// \brief Avoid MSVC stupid behavior: MNE = Macro No Expansion
#define VSMC_MNE

/// \brief CRTP style base classes (virtual) destructor
/// \ingroup Definitions
///
/// \details
/// This macro is defined to `virtual` if compiled in when
/// `VSMC_NO_RUNTIME_ASSERT` is enabled; otherwise it is empty.
#if VSMC_NO_RUNTIME_ASSERT
#define VSMC_CRTP_DESTRUCTOR_PREFIX
#else
#define VSMC_CRTP_DESTRUCTOR_PREFIX virtual
#endif

/// \brief constexpr
/// \ingroup Definitions
#if VSMC_HAS_CXX11_CONSTEXPR
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
enum {
    Dynamic = 0 ///< Used to specify a dimension template parameter is dynamic
}; // enum Dynamic

/// \brief Matrix order
/// \ingroup Definitions
enum MatrixOrder {
    RowMajor = 101, ///< Data are stored row by row in memory
    ColMajor = 102  ///< Data are stored column by column in memory
}; // enum MatrixOrder

/// \brief Class template argument used for scalar variant
/// \ingroup Definitions
struct Scalar
{
    static VSMC_CONSTEXPR const bool is_scalar = true;
    static VSMC_CONSTEXPR const bool is_vector = false;
}; // struct Scalar

/// \brief Class template argument used for vector variant
/// \ingroup Definitions
struct Vector
{
    static VSMC_CONSTEXPR const bool is_scalar = false;
    static VSMC_CONSTEXPR const bool is_vector = true;
}; // struct Vector

/// \brief Function template argument used for position
/// \ingroup Definitions
template <std::size_t N>
struct Position
{
    typedef std::size_t size_type;
    typedef Position<N> type;
    static VSMC_CONSTEXPR const size_type value = N;
    VSMC_CONSTEXPR operator size_type () const {return value;}
    VSMC_CONSTEXPR size_type operator() () const {return value;}
}; // struct Position

} // namespace vsmc

#endif // VSMC_INTERNAL_DEFINES_HPP

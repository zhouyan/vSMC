//============================================================================
// vSMC/include/vsmc/internal/defines.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_INTERNAL_DEFINES_HPP
#define VSMC_INTERNAL_DEFINES_HPP

#include <vsmc/internal/config.hpp>

/// \brief Avoid MSVC stupid behavior: MNE = Macro No Expansion
#define VSMC_MNE

/// \brief A constant expression that give maximum of unsigned integer types
#define VSMC_MAX_UINT(type) static_cast<type>(~(static_cast<type>(0)))

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

namespace vsmc
{

/// \brief SIMD instructions
/// \ingroup Definitions
///
/// \details
/// These constants are used when template functions are specialized for SIMD
/// intructions, such as those in the CString module.
enum SIMD { SSE2, SSE3, SSSE3, SSE4_1, SSE4_2, AVX, AVX2 };

/// \brief Dynamic dimension
/// \ingroup Definitions
enum {
    Dynamic = 0 ///< Used to specify a dimension template parameter is dynamic
};              // enum Dynamic

/// \brief Matrix order
/// \ingroup Definitions
enum MatrixOrder {
    RowMajor = 101, ///< Data are stored row by row in memory
    ColMajor = 102  ///< Data are stored column by column in memory
};                  // enum MatrixOrder

/// \brief Monitor stage
/// \ingroup Definitions
enum MonitorStage {
    MonitorMove,     ///< Monitor evaluated after moves
    MonitorResample, ///< Monitor evaluated after resampling
    MonitorMCMC      ///< Monitor evaluated after MCMC moves
};                   // enum MonitorStage

/// \brief Class template argument used for scalar variant
/// \ingroup Definitions
struct Scalar {
    static constexpr const bool is_scalar = true;
    static constexpr const bool is_vector = false;
    static constexpr const bool is_thread_local = false;
}; // struct Scalar

/// \brief Class template argument used for vector variant
/// \ingroup Definitions
struct Vector {
    static constexpr const bool is_scalar = false;
    static constexpr const bool is_vector = true;
    static constexpr const bool is_thread_local = false;
}; // struct Vector

/// \brief Class template argument used for thread local storage variant
/// \ingroup Definitions
struct ThreadLocal {
    static constexpr const bool is_scalar = false;
    static constexpr const bool is_vector = false;
    static constexpr const bool is_thread_local = true;
}; // struct ThreadLocal

/// \brief Function template argument used for position
/// \ingroup Definitions
template <std::size_t N>
struct Position {
    typedef std::size_t size_type;
    typedef Position<N> type;
    static constexpr const size_type value = N;
    constexpr operator size_type() const { return value; }
    constexpr size_type operator()() const { return value; }
}; // struct Position

} // namespace vsmc

#endif // VSMC_INTERNAL_DEFINES_HPP

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
#include <type_traits>

/// \brief Avoid MSVC stupid behavior: MNE = Macro No Expansion
#define VSMC_MNE

/// \brief A constant expression that give maximum of unsigned integer types
#define VSMC_MAX_UINT(type) static_cast<type>(~(static_cast<type>(0)))

namespace vsmc
{
struct NullType;

/// \brief Dynamic dimension
/// \ingroup Definitions
enum {
    Dynamic = 0 ///< Used to specify a dimension template parameter is dynamic
};

/// \brief Matrix order
/// \ingroup Definitions
enum MatrixOrder {
    RowMajor = 101, ///< Data are stored row by row in memory
    ColMajor = 102  ///< Data are stored column by column in memory
};

/// \brief Matrix Transpose
/// \ingroup Definitions
enum MatrixTrans {
    NoTrans = 111, ///< The matrix shall not be transposed
    Trans = 112    ///< The matrix shall be transposed
};

/// \brief Function template argument used for position
/// \ingroup Definitions
template <std::size_t N>
using Index = std::integral_constant<std::size_t, N>;

#if VSMC_HAS_BACKWARD_COMPATIBILITY
template <std::size_t N>
using Position = Index<N>;
#endif

} // namespace vsmc

#endif // VSMC_INTERNAL_DEFINES_HPP

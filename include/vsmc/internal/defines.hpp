//============================================================================
// vSMC/include/vsmc/internal/defines.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#include <vsmc/internal/config.h>
#include <type_traits>

namespace vsmc
{

/// \brief A placeholder type
/// \ingroup Definitions
class NullType;

/// \brief Dynamic dimension
/// \ingroup Definitions
enum { Dynamic = 0 };

/// \brief Matrix layout
/// \ingroup Definitions
enum MatrixLayout { RowMajor = 101, ColMajor = 102 };

/// \brief Alias to MatrixOrder
/// \ingroup Definitions
using MatrixOrder = MatrixLayout;

/// \brief Resampling schemes
/// \ingroup Definitions
enum ResampleScheme {
    Multinomial,        ///< Multinomial resampling
    Stratified,         ///< Stratified resampling
    Systematic,         ///< Systematic resampling
    Residual,           ///< Residual resampling
    ResidualStratified, ///< Stratified resampling on residuals
    ResidualSystematic  ///< Systematic resampling on residuals
};                      // enum ResampleScheme

} // namespace vsmc

#endif // VSMC_INTERNAL_DEFINES_HPP

//============================================================================
// vSMC/include/vsmc/math/cblas.h
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

#ifndef VSMC_MATH_CBLAS_H
#define VSMC_MATH_CBLAS_H

#include <vsmc/internal/config.h>

/// \brief Integer type of CBLAS routines
/// \ingroup Config
///
/// \details
/// Define this macro if the CBLAS interface has unusual integer types. For
/// example, the CBLAS library use ILP64 while the rest of the program use
/// LP64.
#ifndef VSMC_CBLAS_INT_TYPE
#define VSMC_CBLAS_INT_TYPE int
#endif

#if VSMC_USE_MKL_CBLAS
#include <mkl_cblas.h>
#define VSMC_CBLAS_INT MKL_INT
#else
#include <cblas.h>
#ifndef VSMC_CBLAS_INT
#define VSMC_CBLAS_INT VSMC_CBLAS_INT_TYPE
#endif
#endif

#endif // VSMC_MATH_CBLAS_H

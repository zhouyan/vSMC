//============================================================================
// vSMC/include/rng/internal/rng_define_macro_mkl.hpp
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

#if VSMC_HAS_MKL
VSMC_RNG_DEFINE_MACRO_MKL(::vsmc::MKL_MCG59, MKL_MCG59, mkl_mcg69)
VSMC_RNG_DEFINE_MACRO_MKL(::vsmc::MKL_MCG59_64, MKL_MCG59_64, mkl_mcg69_64)
VSMC_RNG_DEFINE_MACRO_MKL(::vsmc::MKL_MT19937, MKL_MT19937, mkl_mt19937)
VSMC_RNG_DEFINE_MACRO_MKL(::vsmc::MKL_MT19937_64, MKL_MT19937, mkl_mt19937_64)
VSMC_RNG_DEFINE_MACRO_MKL(::vsmc::MKL_MT2203, MKL_MT2203, mkl_mt2203)
VSMC_RNG_DEFINE_MACRO_MKL(::vsmc::MKL_MT2203_64, MKL_MT2203_64, mkl_mt2203_64)
VSMC_RNG_DEFINE_MACRO_MKL(::vsmc::MKL_SFMT19937, MKL_STMT19937, mkl_sftmt19937)
VSMC_RNG_DEFINE_MACRO_MKL(
    ::vsmc::MKL_SFMT19937_64, MKL_STMT19937_64, mkl_sftmt19937_64)
#if VSMC_HAS_RDRAND
VSMC_RNG_DEFINE_MACRO_MKL(::vsmc::MKL_NONDETERM, MKL_NONDETERM, mkl_nondeterm)
VSMC_RNG_DEFINE_MACRO_MKL(
    ::vsmc::MKL_NONDETERM_64, MKL_NONDETERM_64, mkl_nondeterm_64)
#endif // VSMC_HAS_RDRAND
#if INTEL_MKL_VERSION >= 110300
#if VSMC_HAS_AES_NI
VSMC_RNG_DEFINE_MACRO_MKL(::vsmc::MKL_ARS5, MKL_ARS5, mkl_ars5)
VSMC_RNG_DEFINE_MACRO_MKL(::vsmc::MKL_ARS5_64, MKL_ARS5_64, mkl_ars5)
#endif // VSMC_HAS_AES_NI
VSMC_RNG_DEFINE_MACRO_MKL(
    vsmc::MKL_PHILOX4X32X10, MKL_PHILOX4X32X10, mkl_philox4x32x10)
VSMC_RNG_DEFINE_MACRO_MKL(
    vsmc::MKL_PHILOX4X32X10_64, MKL_PHILOX4X32X10_64, mkl_philox4x32x10_64)
#endif // INTEL_MKL_VERSION >= 110300
#endif // VSMC_HAS_MKL

//============================================================================
// vSMC/include/vsmc/rngc/mkl_std.h
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

#ifndef VSMC_RNGC_MKL_STD_H
#define VSMC_RNGC_MKL_STD_H

#include <mkl.h>

#ifdef __cplusplus
extern "C" {
#endif

int vsmc_mkl_std_brng_mt19937(void);
int vsmc_mkl_std_brng_minstd_rand0(void);
int vsmc_mkl_std_brng_minstd_rand(void);
int vsmc_mkl_std_brng_mt19937_64(void);
int vsmc_mkl_std_brng_ranlux24_base(void);
int vsmc_mkl_std_brng_ranlux48_base(void);
int vsmc_mkl_std_brng_ranlux24(void);
int vsmc_mkl_std_brng_ranlux48(void);
int vsmc_mkl_std_brng_knuth_b(void);

int vsmc_mkl_std_brng_xorshift1x32(void);
int vsmc_mkl_std_brng_xorshift2x32(void);
int vsmc_mkl_std_brng_xorshift4x32(void);
int vsmc_mkl_std_brng_xorshift8x32(void);
int vsmc_mkl_std_brng_xorshift16x32(void);
int vsmc_mkl_std_brng_xorshift32x32(void);
int vsmc_mkl_std_brng_xorshift64x32(void);
int vsmc_mkl_std_brng_xorshift128x32(void);
int vsmc_mkl_std_brng_xorshift1x64(void);
int vsmc_mkl_std_brng_xorshift2x64(void);
int vsmc_mkl_std_brng_xorshift4x64(void);
int vsmc_mkl_std_brng_xorshift8x64(void);
int vsmc_mkl_std_brng_xorshift16x64(void);
int vsmc_mkl_std_brng_xorshift32x64(void);
int vsmc_mkl_std_brng_xorshift64x64(void);

int vsmc_mkl_std_brng_xorwow1x32(void);
int vsmc_mkl_std_brng_xorwow2x32(void);
int vsmc_mkl_std_brng_xorwow4x32(void);
int vsmc_mkl_std_brng_xorwow8x32(void);
int vsmc_mkl_std_brng_xorwow16x32(void);
int vsmc_mkl_std_brng_xorwow32x32(void);
int vsmc_mkl_std_brng_xorwow64x32(void);
int vsmc_mkl_std_brng_xorwow128x32(void);
int vsmc_mkl_std_brng_xorwow1x64(void);
int vsmc_mkl_std_brng_xorwow2x64(void);
int vsmc_mkl_std_brng_xorwow4x64(void);
int vsmc_mkl_std_brng_xorwow8x64(void);
int vsmc_mkl_std_brng_xorwow16x64(void);
int vsmc_mkl_std_brng_xorwow32x64(void);
int vsmc_mkl_std_brng_xorwow64x64(void);

int vsmc_mkl_std_brng_philox2x32(void);
int vsmc_mkl_std_brng_philox4x32(void);
int vsmc_mkl_std_brng_philox2x64(void);
int vsmc_mkl_std_brng_philox4x64(void);

int vsmc_mkl_std_brng_threefry2x32(void);
int vsmc_mkl_std_brng_threefry4x32(void);
int vsmc_mkl_std_brng_threefry2x64(void);
int vsmc_mkl_std_brng_threefry4x64(void);
int vsmc_mkl_std_brng_threefry2x32avx2(void);
int vsmc_mkl_std_brng_threefry4x32avx2(void);
int vsmc_mkl_std_brng_threefry2x64avx2(void);
int vsmc_mkl_std_brng_threefry4x64avx2(void);
int vsmc_mkl_std_brng_threefry2x32sse2(void);
int vsmc_mkl_std_brng_threefry4x32sse2(void);
int vsmc_mkl_std_brng_threefry2x64sse2(void);
int vsmc_mkl_std_brng_threefry4x64sse2(void);

int vsmc_mkl_std_brng_aes128_1x32(void);
int vsmc_mkl_std_brng_aes128_2x32(void);
int vsmc_mkl_std_brng_aes128_4x32(void);
int vsmc_mkl_std_brng_aes128_8x32(void);
int vsmc_mkl_std_brng_aes128_1x64(void);
int vsmc_mkl_std_brng_aes128_2x64(void);
int vsmc_mkl_std_brng_aes128_4x64(void);
int vsmc_mkl_std_brng_aes128_8x64(void);
int vsmc_mkl_std_brng_aes192_1x32(void);
int vsmc_mkl_std_brng_aes192_2x32(void);
int vsmc_mkl_std_brng_aes192_4x32(void);
int vsmc_mkl_std_brng_aes192_8x32(void);
int vsmc_mkl_std_brng_aes192_1x64(void);
int vsmc_mkl_std_brng_aes192_2x64(void);
int vsmc_mkl_std_brng_aes192_4x64(void);
int vsmc_mkl_std_brng_aes192_8x64(void);
int vsmc_mkl_std_brng_aes256_1x32(void);
int vsmc_mkl_std_brng_aes256_2x32(void);
int vsmc_mkl_std_brng_aes256_4x32(void);
int vsmc_mkl_std_brng_aes256_8x32(void);
int vsmc_mkl_std_brng_aes256_1x64(void);
int vsmc_mkl_std_brng_aes256_2x64(void);
int vsmc_mkl_std_brng_aes256_4x64(void);
int vsmc_mkl_std_brng_aes256_8x64(void);
int vsmc_mkl_std_brng_ars_1x32(void);
int vsmc_mkl_std_brng_ars_2x32(void);
int vsmc_mkl_std_brng_ars_4x32(void);
int vsmc_mkl_std_brng_ars_8x32(void);
int vsmc_mkl_std_brng_ars_1x64(void);
int vsmc_mkl_std_brng_ars_2x64(void);
int vsmc_mkl_std_brng_ars_4x64(void);
int vsmc_mkl_std_brng_ars_8x64(void);

int vsmc_mkl_std_brng_rdrand16(void);
int vsmc_mkl_std_brng_rdrand32(void);
int vsmc_mkl_std_brng_rdrand64(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // VSMC_RNGC_MKL_STD_H

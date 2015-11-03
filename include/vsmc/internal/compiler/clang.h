//============================================================================
// vSMC/include/vsmc/internal/compiler/clang.h
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

#ifndef VSMC_INTERNAL_COMPILER_CLANG_H
#define VSMC_INTERNAL_COMPILER_CLANG_H

#define VSMC_CLANG_NONEXIST 0xFFFFFFFFUL

#define VSMC_CLANG_VERSION                                                    \
    (__clang_major__ * 10000 + __clang_minor__ * 100 + __clang_patchlevel__)

#ifndef VSMC_INT64
#define VSMC_INT64 long long
#endif

#ifdef __x86_64__
#ifndef VSMC_HAS_INT128
#define VSMC_HAS_INT128 1
#endif
#ifndef VSMC_INT128
#define VSMC_INT128 __int128
#endif
#endif

#ifdef __SSE2__
#ifndef VSMC_HAS_SSE2
#define VSMC_HAS_SSE2 1
#endif
#endif

#ifdef __AVX2__
#ifndef VSMC_HAS_AVX2
#define VSMC_HAS_AVX2 1
#endif
#endif

#ifdef __AES__
#ifndef VSMC_HAS_AES_NI
#define VSMC_HAS_AES_NI 1
#endif
#endif

#ifdef __RDRND__
#ifndef VSMC_HAS_RDRAND
#define VSMC_HAS_RDRAND 1
#endif
#endif

#endif // VSMC_INTERNAL_COMPILER_CLANG_H

//============================================================================
// vSMC/include/vsmc/internal/compiler.h
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

#ifndef VSMC_INTERNAL_COMPILER_H
#define VSMC_INTERNAL_COMPILER_H

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

#ifndef __has_attribute
#define __has_attribute(x) 0
#endif

#ifndef __has_builtin
#define __has_builtin(x) 0
#endif

#ifndef __has_cpp_attribute
#define __has_cpp_attribute(x) 0
#endif

#ifndef __has_extension
#define __has_extension(x) 0
#endif

#ifndef __has_feature
#define __has_feature(x) 0
#endif

#if defined(__OPENCL_VERSION__)
#define VSMC_OPENCL
#include <vsmc/internal/compiler/opencl.h>
#elif defined(__INTEL_COMPILER)
#define VSMC_INTEL
#include <vsmc/internal/compiler/intel.h>
#elif defined(__clang__)
#define VSMC_CLANG
#include <vsmc/internal/compiler/clang.h>
#elif defined(__GNUC__)
#define VSMC_GCC
#include <vsmc/internal/compiler/gcc.h>
#elif defined(_MSC_VER)
#define VSMC_MSVC
#include <vsmc/internal/compiler/msvc.h>
#endif

#ifndef VSMC_OPENCL
#ifdef __cplusplus
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#else
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#endif
#endif

#ifndef UINT64_C
#error __STDC_CONSTANT_MACROS not defined before #include<stdint.h>
#endif

#ifndef VSMC_HAS_RNGC_DOUBLE
#define VSMC_HAS_RNGC_DOUBLE 1
#endif

#ifndef VSMC_STATIC_INLINE
#ifdef __cplusplus
#define VSMC_STATIC_INLINE inline
#else
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
#define VSMC_STATIC_INLINE static inline
#else
#define VSMC_STATIC_INLINE static
#endif
#endif
#endif

#ifndef VSMC_INT64
#define VSMC_INT64 long long
#endif

#ifndef VSMC_HAS_X86
#if defined(__x86__) || defined(__x86_64__)
#define VSMC_HAS_X86 1
#else
#define VSMC_HAS_X86 0
#endif
#endif

#ifndef VSMC_HAS_INT128
#define VSMC_HAS_INT128 0
#endif

#ifndef VSMC_HAS_SSE2
#define VSMC_HAS_SSE2 0
#endif

#ifndef VSMC_HAS_AVX2
#define VSMC_HAS_AVX2 0
#endif

#ifndef VSMC_HAS_AES_NI
#define VSMC_HAS_AES_NI 0
#endif

#ifndef VSMC_HAS_RDRAND
#define VSMC_HAS_RDRAND 0
#endif

#if defined(_OPENMP) && _OPENMP >= 200805
#ifndef VSMC_HAS_OMP
#define VSMC_HAS_OMP 1
#endif
#endif

#endif // VSMC_INTERNAL_COMPILER_HPP

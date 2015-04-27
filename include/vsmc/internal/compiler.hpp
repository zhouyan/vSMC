//============================================================================
// vSMC/include/vsmc/internal/compiler.hpp
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

#ifndef VSMC_INTERNAL_COMPILER_HPP
#define VSMC_INTERNAL_COMPILER_HPP

// Compiler feature check macros

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

#include <cstddef>

#if defined(__APPLE__) || defined(__MACOSX)
#define VSMC_MACOSX
#include <Availability.h>
#define VSMC_MAC_VERSION_MIN_REQUIRED(ver)                                    \
    __MAC_OS_X_VERSION_MIN_REQUIRED >= ver
#else
#define VSMC_MAC_VERSION_MIN_REQUIRED(ver) 0
#endif

#ifdef VSMC_MACOSX
#if VSMC_MAC_VERSION_MIN_REQUIRED(__MAC_10_5)
#ifndef VSMC_HAS_POSIX
#define VSMC_HAS_POSIX 1
#endif
#endif
#else // VSMC_MACOSX
#include <stdlib.h>
#if defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200112L
#ifndef VSMC_HAS_POSIX
#define VSMC_HAS_POSIX 1
#endif
#endif // _POSIX_C_SOURCE >= 200112L
#if defined(_XOPEN_SOURCE) && _XOPEN_SOURCE >= 600
#ifndef VSMC_HAS_POSIX
#define VSMC_HAS_POSIX 1
#endif
#endif // _XOPEN_SOURCE >= 600
#endif // VSMC_MACOSX

#ifndef VSMC_HAS_POSIX
#define VSMC_HAS_POSIX 0
#endif

#if defined(__INTEL_COMPILER)
#define VSMC_INTEL
#include <vsmc/internal/compiler/intel.hpp>
#elif defined(__clang__)
#define VSMC_CLANG
#include <vsmc/internal/compiler/clang.hpp>
#elif defined(__GNUC__)
#define VSMC_GCC
#include <vsmc/internal/compiler/gcc.hpp>
#elif defined(_MSC_VER)
#define VSMC_MSVC
#include <vsmc/internal/compiler/msvc.hpp>
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

#ifndef VSMC_HAS_AES_NI
#define VSMC_HAS_AES_NI 0
#endif

#ifndef VSMC_HAS_RDRAND
#define VSMC_HAS_RDRAND 0
#endif

#ifndef VSMC_HAS_OMP
#ifdef _OPENMP
#define VSMC_HAS_OMP 1
#endif
#endif

// OS features

#if VSMC_MAC_VERSION_MIN_REQUIRED(__MAC_10_7)
#ifndef VSMC_HAS_GCD_LION
#define VSMC_HAS_GCD_LION 1
#endif
#endif

#endif // VSMC_INTERNAL_COMPILER_HPP

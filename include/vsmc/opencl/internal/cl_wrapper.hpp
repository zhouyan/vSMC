//============================================================================
// vSMC/include/vsmc/opencl/internal/cl_wrapper.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

#ifndef VSMC_OPENCL_INTERNAL_CL_WRAPPER_HPP
#define VSMC_OPENCL_INTERNAL_CL_WRAPPER_HPP

#include <vsmc/internal/config.hpp>

#if defined(VSMC_GCC)
#if VSMC_GCC_VERSION >= 40600
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#elif defined(VSMC_MSVC)
#pragma warning(push)
#pragma warning(disable:4996)
#endif

#include <cl.hpp>

#ifndef __CL_ENABLE_EXCEPTIONS
#error __CL_ENABLE_EXCEPTIONS not defined before #include<cl.hpp>
#endif

#if defined(VSMC_GCC)
#if VSMC_GCC_VERSION >= 40600
#pragma GCC diagnostic pop
#endif
#elif defined(VSM_MSVC)
#pragma warning(pop)
#endif

#if defined(CL_VERSION_2_0)
#define VSMC_OPENCL_VERSION 200
#elif defined(CL_VERSION_1_2)
#define VSMC_OPENCL_VERSION 120
#elif defined(CL_VERSION_1_1)
#define VSMC_OPENCL_VERSION 110
#else
#define VSMC_OPENCL_VERSION 100
#endif

#endif // VSMC_OPENCL_INTERNAL_CL_WRAPPER_HPP

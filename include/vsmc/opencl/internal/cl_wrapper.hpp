//============================================================================
// include/vsmc/opencl/internal/cl_wrapper.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distributed under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_OPENCL_INTERNAL_CL_WRAPPER_HPP
#define VSMC_OPENCL_INTERNAL_CL_WRAPPER_HPP

#if defined(VSMC_GCC) && VSMC_GCC_VERSION >= 40600
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#elif defined(VSMC_MSVC)
#pragma warning(push)
#pragma warning(disable:4996)
#endif

#include <cl.hpp>

#ifndef __CL_ENABLE_EXCEPTIONS
#error __CL_ENABLE_EXCEPTIONS not defined before #include<cl.hpp>
#endif

#if defined(VSMC_GCC) && VSMC_GCC_VERSION >= 40600
#pragma GCC diagnostic pop
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

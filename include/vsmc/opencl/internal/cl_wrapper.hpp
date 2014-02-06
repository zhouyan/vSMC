#ifndef VSMC_OPENCL_INTERNAL_CL_WRAPPER_HPP
#define VSMC_OPENCL_INTERNAL_CL_WRAPPER_HPP

#ifndef __CL_ENABLE_EXCEPTIONS
#define __CL_ENABLE_EXCEPTIONS
#endif

#if defined(__GNUC__) && !defined(__INTEL_COMPILER)
#if __GNUC__ >= 4 && __GNUC_MINIOR__ >= 6
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#endif

#include <cl.hpp>

#if defined(__GNUC__) && !defined(__INTEL_COMPILER)
#if __GNUC__ >= 4 && __GNUC_MINIOR__ >= 6
#pragma GCC diagnostic pop
#endif
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

//============================================================================
// vSMC/include/vsmc/opencl/internal/common.hpp
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

#ifndef VSMC_OPENCL_INTERNAL_COMMON_HPP
#define VSMC_OPENCL_INTERNAL_COMMON_HPP

#include <vsmc/internal/common.hpp>

#if defined(__APPLE__) || defined(__MACOSX)
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
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

#if VSMC_OPENCL_VERSION < 120
#error vSMC OpenCL module requires at least version 1.2 runtime
#endif

namespace vsmc
{

namespace internal
{

#if VSMC_NO_RUNTIME_ASSERT
inline void cl_error_check(cl_int, const char *, const char *) {}
#else
inline void cl_error_check(cl_int status, const char *func, const char *clf)
{
    if (status == CL_SUCCESS)
        return;

    std::string msg("**");
    msg += func;
    msg += "** failure";
    msg += "; OpenCL function: ";
    msg += clf;
    msg += "; Error code: ";
    msg += itos(status);

    VSMC_RUNTIME_ASSERT((status == CL_SUCCESS), msg.c_str());
}
#endif

inline int cl_version(const char *version)
{
    if (std::strcmp(version, "2.1") == 0)
        return 210;
    if (std::strcmp(version, "2.0") == 0)
        return 200;
    if (std::strcmp(version, "1.2") == 0)
        return 120;
    if (std::strcmp(version, "1.1") == 0)
        return 110;
    return 100;
}

inline int cl_opencl_version(::cl_device_id device)
{
    char version[64];
    ::cl_int status =
        ::clGetDeviceInfo(device, CL_DEVICE_VERSION, 64, version, nullptr);
    version[10] = '\0';

    return status == CL_SUCCESS ? cl_version(version + 7) : 100;
}

inline int cl_opencl_c_version(::cl_device_id device)
{
    char version[64];
    ::cl_int status = ::clGetDeviceInfo(
        device, CL_DEVICE_OPENCL_C_VERSION, 64, version, nullptr);
    version[12] = '\0';

    return status == CL_SUCCESS ? cl_version(version + 9) : 100;
}

} // namespace vsmc::internal

/// \brief Template parameter type for default behavior
/// \ingroup OpenCL
struct CLDefault;

} // namespace vsmc

#endif // VSMC_OPENCL_INTERNAL_COMMON_HPP

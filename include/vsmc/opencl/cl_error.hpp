//============================================================================
// vSMC/include/vsmc/opencl/cl_error.hpp
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

#ifndef VSMC_OPENCL_CL_ERROR_HPP
#define VSMC_OPENCL_CL_ERROR_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/internal/cl_wrapper.hpp>

#define VSMC_DEFINE_OPENCL_CL_ERROR_ERR(STATUS)                              \
    case STATUS: return #STATUS

namespace vsmc
{

namespace internal
{

inline std::string cl_error_str(cl_int status)
{
    switch (status) {
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_SUCCESS);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_DEVICE_NOT_FOUND);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_DEVICE_NOT_AVAILABLE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_COMPILER_NOT_AVAILABLE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_MEM_OBJECT_ALLOCATION_FAILURE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_OUT_OF_RESOURCES);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_OUT_OF_HOST_MEMORY);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_PROFILING_INFO_NOT_AVAILABLE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_MEM_COPY_OVERLAP);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_IMAGE_FORMAT_MISMATCH);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_IMAGE_FORMAT_NOT_SUPPORTED);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_BUILD_PROGRAM_FAILURE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_MAP_FAILURE);
#if VSMC_OPENCL_VERSION >= 110
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_MISALIGNED_SUB_BUFFER_OFFSET);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(
            CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST);
#endif
#if VSMC_OPENCL_VERSION >= 120
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_COMPILE_PROGRAM_FAILURE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_LINKER_NOT_AVAILABLE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_LINK_PROGRAM_FAILURE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_DEVICE_PARTITION_FAILED);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_KERNEL_ARG_INFO_NOT_AVAILABLE);
#endif
#if VSMC_OPENCL_VERSION >= 200
// No new error code in OpenCL 2.0
#endif

        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_VALUE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_DEVICE_TYPE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_PLATFORM);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_DEVICE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_CONTEXT);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_QUEUE_PROPERTIES);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_COMMAND_QUEUE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_HOST_PTR);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_MEM_OBJECT);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_IMAGE_FORMAT_DESCRIPTOR);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_IMAGE_SIZE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_SAMPLER);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_BINARY);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_BUILD_OPTIONS);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_PROGRAM);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_PROGRAM_EXECUTABLE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_KERNEL_NAME);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_KERNEL_DEFINITION);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_KERNEL);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_ARG_INDEX);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_ARG_VALUE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_ARG_SIZE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_KERNEL_ARGS);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_WORK_DIMENSION);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_WORK_GROUP_SIZE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_WORK_ITEM_SIZE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_GLOBAL_OFFSET);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_EVENT_WAIT_LIST);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_EVENT);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_OPERATION);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_GL_OBJECT);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_BUFFER_SIZE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_MIP_LEVEL);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_GLOBAL_WORK_SIZE);
#if VSMC_OPENCL_VERSION >= 110
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_PROPERTY);
#endif
#if VSMC_OPENCL_VERSION >= 120
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_IMAGE_DESCRIPTOR);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_COMPILER_OPTIONS);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_LINKER_OPTIONS);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_DEVICE_PARTITION_COUNT);
#endif
#if VSMC_OPENCL_VERSION >= 200
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_PIPE_SIZE);
        VSMC_DEFINE_OPENCL_CL_ERROR_ERR(CL_INVALID_DEVICE_QUEUE);
#endif
        default: return "UNKNOWN";
    }
}

} // namespace vsmc::internal

/// \brief OpenCL exception
/// \ingroup OpenCL
///
/// \details
/// This exception class differs from `cl::Error` mostly in that the error
/// string is appended by the macro name of the error code. Therefore, when
/// calling `what`, one might saw `clKernelSetArg:CL_INVALID_MEM_OBJECT` etc.,
/// instead of simply `clKernelSetArg`.
class CLError : public std::runtime_error
{
    public:
    CLError()
        : std::runtime_error(internal::cl_error_str(CL_SUCCESS)),
          status_(CL_SUCCESS)
    {
    }

    explicit CLError(cl_int status)
        : std::runtime_error(internal::cl_error_str(status)), status_(status)
    {
    }

    explicit CLError(const ::cl::Error &err)
        : std::runtime_error(std::string(err.what()) + ":" +
              internal::cl_error_str(err.err())),
          status_(err.err())
    {
    }

    CLError(cl_int status, const char *msg)
        : std::runtime_error(
              std::string(msg) + ":" + internal::cl_error_str(status)),
          status_(status)
    {
    }

    CLError(cl_int status, const std::string &msg)
        : std::runtime_error(msg + ":" + internal::cl_error_str(status)),
          status_(status)
    {
    }

    cl_int err() const { return status_; }

    cl_int status() const { return status_; }

    private:
    cl_int status_;
}; // class CLError

} // namespace vsmc

#endif // VSMC_OPENCL_CL_ERROR_HPP

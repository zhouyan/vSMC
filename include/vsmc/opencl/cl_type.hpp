#ifndef VSMC_OPENCL_CL_TYPE_HPP
#define VSMC_OPENCL_CL_TYPE_HPP

#include <vsmc/opencl/internal/common.hpp>

namespace vsmc
{

/// \brief OpenCL platform type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_platform_id>::type CLPlatform;

/// \brief Make a `std::shared_ptr` object from `cl_platform_id`
/// \ingroup OpenCL
inline std::shared_ptr<CLPlatform> cl_platform_make_shared(
    ::cl_platform_id ptr)
{
    return std::shared_ptr<CLPlatform>(ptr, [](::cl_platform_id) {});
}

/// \brief OpenCL device type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_device_id>::type CLDevice;

/// \brief Make a `std::shared_ptr` object from `cl_device_id`
/// \ingroup OpenCL
inline std::shared_ptr<CLDevice> cl_device_make_shared(::cl_device_id ptr)
{
    return std::shared_ptr<CLDevice>(ptr, [](::cl_device_id p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseDevice(p);
            internal::cl_error_check(
                status, "cl_device_make_shared", "::clReleaseDevice");
        }
    });
}

/// \brief OpenCL context type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_context>::type CLContext;

/// \brief Make a `std::shared_ptr` object from `cl_context`
/// \ingroup OpenCL
inline std::shared_ptr<CLContext> cl_context_make_shared(::cl_context ptr)
{
    return std::shared_ptr<CLContext>(ptr, [](::cl_context p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseContext(p);
            internal::cl_error_check(
                status, "cl_context_make_shared", "::clReleaseContext");
        }
    });
}

/// \brief OpenCL command queue type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_command_queue>::type CLCommandQueue;

/// \brief Make a `std::shared_ptr` object from `cl_command_queue`
/// \ingroup OpenCL
inline std::shared_ptr<CLCommandQueue> cl_command_queue_make_shared(
    ::cl_command_queue ptr)
{
    return std::shared_ptr<CLCommandQueue>(ptr, [](::cl_command_queue p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseCommandQueue(p);
            internal::cl_error_check(status, "cl_command_queue_make_shared",
                "::clReleaseCommandQueue");
        }
    });
}

/// \brief OpenCL memory type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_mem>::type CLMemory;

/// \brief Make a `std::shared_ptr` object from `cl_mem`
/// \ingroup OpenCL
inline std::shared_ptr<CLMemory> cl_memory_make_shared(::cl_mem ptr)
{
    return std::shared_ptr<CLMemory>(ptr, [](::cl_mem p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseMemObject(p);
            internal::cl_error_check(
                status, "cl_memory_make_shared", "::clReleaseMemObject");
        }
    });
}

/// \brief OpenCL program type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_program>::type CLProgram;

/// \brief Make a `std::shared_ptr` object from `cl_program`
/// \ingroup OpenCL
inline std::shared_ptr<CLProgram> cl_program_make_shared(::cl_program ptr)
{
    return std::shared_ptr<CLProgram>(ptr, [](::cl_program p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseProgram(p);
            internal::cl_error_check(
                status, "cl_program_make_shared", "::clReleaseProgram");
        }
    });
}

/// \brief OpenCL kernel type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_kernel>::type CLKernel;

/// \brief Make a `std::shared_ptr` object from `cl_kernel`
/// \ingroup OpenCL
inline std::shared_ptr<CLKernel> cl_kernel_make_shared(::cl_kernel ptr)
{
    return std::shared_ptr<CLKernel>(ptr, [](::cl_kernel p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseKernel(p);
            internal::cl_error_check(
                status, "cl_kernel_make_shared", "::clReleaseKernel");
        }
    });
}

/// \brief OpenCL event type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_event>::type CLEvent;

/// \brief Make a `std::shared_ptr` object from `cl_event`
/// \ingroup OpenCL
inline std::shared_ptr<CLEvent> cl_event_make_shared(::cl_event ptr)
{
    return std::shared_ptr<CLEvent>(ptr, [](::cl_event p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseEvent(p);
            internal::cl_error_check(
                status, "cl_event_make_shared", "::clReleaseEvent");
        }
    });
}

/// \brief OpenCL sampler type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_sampler>::type CLSampler;

/// \brief Make a `std::shared_ptr` object from `cl_sampler`
/// \ingroup OpenCL
inline std::shared_ptr<CLSampler> cl_sampler_make_shared(::cl_sampler ptr)
{
    return std::shared_ptr<CLSampler>(ptr, [](::cl_sampler p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseSampler(p);
            internal::cl_error_check(
                status, "cl_sampler_make_shared", "::clReleaseSampler");
        }
    });
}

} // namespace vsmc

#endif // VSMC_OPENCL_CL_TYPE_HPP

#ifndef VSMC_OPENCL_INTERNAL_COMMON_HPP
#define VSMC_OPENCL_INTERNAL_COMMON_HPP

#include <vsmc/internal/common.hpp>

#ifdef VSMC_MACOSX
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
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

} // namespace vsmc::internal

/// \brief OpenCL platform type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_platform_id>::type CLPlatform;

/// \brief Make a `std::shared_ptr` object from `cl_platform_id`
/// \ingroup OpenCL
inline std::shared_ptr<CLPlatform> make_cl_platform_ptr(::cl_platform_id ptr)
{
    return std::shared_ptr<CLPlatform>(ptr, [](::cl_platform_id) {});
}

/// \brief OpenCL device type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_device_id>::type CLDevice;

/// \brief Make a `std::shared_ptr` object from `cl_device_id`
/// \ingroup OpenCL
inline std::shared_ptr<CLDevice> make_cl_device_ptr(::cl_device_id ptr)
{
    return std::shared_ptr<CLDevice>(ptr, [](::cl_device_id p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseDevice(p);
            internal::cl_error_check(
                status, "make_cl_device_ptr", "::clReleaseDevice");
        }
    });
}

/// \brief OpenCL context type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_context>::type CLContext;

/// \brief Make a `std::shared_ptr` object from `cl_context`
/// \ingroup OpenCL
inline std::shared_ptr<CLContext> make_cl_context_ptr(::cl_context ptr)
{
    return std::shared_ptr<CLContext>(ptr, [](::cl_context p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseContext(p);
            internal::cl_error_check(
                status, "make_cl_context_ptr", "::clReleaseContext");
        }
    });
}

/// \brief OpenCL command queue type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_command_queue>::type CLCommandQueue;

/// \brief Make a `std::shared_ptr` object from `cl_command_queue`
/// \ingroup OpenCL
inline std::shared_ptr<CLCommandQueue> make_cl_command_queue_ptr(
    ::cl_command_queue ptr)
{
    return std::shared_ptr<CLCommandQueue>(ptr, [](::cl_command_queue p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseCommandQueue(p);
            internal::cl_error_check(status, "make_cl_command_queue_ptr",
                "::clReleaseCommandQueue");
        }
    });
}

/// \brief OpenCL memory type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_mem>::type CLMemory;

/// \brief Make a `std::shared_ptr` object from `cl_mem`
/// \ingroup OpenCL
inline std::shared_ptr<CLMemory> make_cl_memory_ptr(::cl_mem ptr)
{
    return std::shared_ptr<CLMemory>(ptr, [](::cl_mem p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseMemObject(p);
            internal::cl_error_check(
                status, "make_cl_memory_ptr", "::clReleaseMemObject");
        }
    });
}

/// \brief OpenCL program type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_program>::type CLProgram;

/// \brief Make a `std::shared_ptr` object from `cl_program`
/// \ingroup OpenCL
inline std::shared_ptr<CLProgram> make_cl_program_ptr(::cl_program ptr)
{
    return std::shared_ptr<CLProgram>(ptr, [](::cl_program p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseProgram(p);
            internal::cl_error_check(
                status, "make_cl_program_ptr", "::clReleaseProgram");
        }
    });
}

/// \brief OpenCL kernel type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_kernel>::type CLKernel;

/// \brief Make a `std::shared_ptr` object from `cl_kernel`
/// \ingroup OpenCL
inline std::shared_ptr<CLKernel> make_cl_kernel_ptr(::cl_kernel ptr)
{
    return std::shared_ptr<CLKernel>(ptr, [](::cl_kernel p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseKernel(p);
            internal::cl_error_check(
                status, "make_cl_kernel_ptr", "::clReleaseKernel");
        }
    });
}

/// \brief OpenCL event type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_event>::type CLEvent;

/// \brief Make a `std::shared_ptr` object from `cl_event`
/// \ingroup OpenCL
inline std::shared_ptr<CLEvent> make_cl_event_ptr(::cl_event ptr)
{
    return std::shared_ptr<CLEvent>(ptr, [](::cl_event p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseEvent(p);
            internal::cl_error_check(
                status, "make_cl_event_ptr", "::clReleaseEvent");
        }
    });
}

/// \brief OpenCL sampler type
/// \ingroup OpenCL
typedef std::remove_pointer<::cl_sampler>::type CLSampler;

/// \brief Make a `std::shared_ptr` object from `cl_sampler`
/// \ingroup OpenCL
inline std::shared_ptr<CLSampler> make_cl_sampler_ptr(::cl_sampler ptr)
{
    return std::shared_ptr<CLSampler>(ptr, [](::cl_sampler p) {
        if (p != nullptr) {
            ::cl_int status = ::clReleaseSampler(p);
            internal::cl_error_check(
                status, "make_cl_sampler_ptr", "::clReleaseSampler");
        }
    });
}

} // namespace vsmc

#endif // VSMC_OPENCL_INTERNAL_COMMON_HPP

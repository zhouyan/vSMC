//============================================================================
// vSMC/include/vsmc/opencl/cl_type.hpp
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

#ifndef VSMC_OPENCL_CL_TYPE_HPP
#define VSMC_OPENCL_CL_TYPE_HPP

#include <vsmc/opencl/internal/common.hpp>

#if defined(VSMC_INTEL)
#pragma warning(push)
#pragma warning(disable : 1478)
#endif

namespace vsmc
{

class CLNDRange;
class CLDevice;
class CLPlatform;
class CLContext;
class CLEvent;
class CLMemory;
class CLSampler;
class CLProgram;
class CLKernel;
class CLCommandQueue;

/// \brief OpenCL resource management base class
/// \ingroup OpenCL
template <typename CLPtr, typename Derived>
class CLBase
{
    public:
    using pointer = CLPtr;
    using element_type = typename std::remove_pointer<CLPtr>::type;

    CLBase() : ptr_(nullptr, [](pointer p) { Derived::release(p); }) {}

    void reset(pointer ptr = nullptr)
    {
        if (ptr != ptr_.get())
            ptr_.reset(ptr, [](pointer p) { Derived::release(p); });
    }

    void swap(CLBase<CLPtr, Derived> &other) { ptr_.swap(other.ptr_); }

    pointer get() const { return ptr_.get(); }

    pointer ptr() const { return ptr_.get(); }

    long use_count() const { return ptr_.use_count(); }

    bool unique() const { return ptr_.unique(); }

    explicit operator bool() const { return bool(ptr_); }

    /// \brief Call `clGet*Info` functions
    ///
    /// \details
    /// For `char[]` type parameters, call this function with `std::string`
    /// output parameter `pram`.
    /// For other pointer type parameters, call this function with
    /// `std::vector<T>` output parameter `param`
    template <typename ParamType>
    ::cl_int get_info(::cl_uint param_name, ParamType &param) const
    {
        return Derived::get_info_param(
            ptr_.get(), param_name, sizeof(ParamType), &param, nullptr);
    }

    template <typename T>
    ::cl_int get_info(::cl_uint param_name, std::vector<T> &param) const
    {
        ::cl_int status = CL_SUCCESS;

        std::size_t num = 0;
        status =
            Derived::get_info_param(ptr_.get(), param_name, 0, nullptr, &num);
        if (status != CL_SUCCESS || num % sizeof(T) != 0 || num < sizeof(T))
            return status;

        std::vector<T> vec(num / sizeof(T));
        status = Derived::get_info_param(
            ptr_.get(), param_name, num, vec.data(), nullptr);
        if (status != CL_SUCCESS)
            return status;

        param = std::move(vec);

        return status;
    }

    ::cl_int get_info(::cl_uint param_name, std::string &param) const
    {
        std::vector<char> vec;
        ::cl_int status = get_info(param_name, vec);
        vec.push_back(0);
        if (status != CL_SUCCESS)
            return status;

        param = std::string(static_cast<const char *>(vec.data()));

        return status;
    }

    private:
    std::shared_ptr<element_type> ptr_;
}; // class CLBase

/// \brief Comparison of equality of two CLBase objects
/// \ingroup OpenCL
template <typename CLPtr, typename Derived>
inline bool operator==(
    const CLBase<CLPtr, Derived> &ptr1, const CLBase<CLPtr, Derived> &ptr2)
{
    return ptr1.get() == ptr2.get();
}

/// \brief Comparison of inequality of two CLBase objects
/// \ingroup OpenCL
template <typename CLPtr, typename Derived>
inline bool operator!=(
    const CLBase<CLPtr, Derived> &ptr1, const CLBase<CLPtr, Derived> &ptr2)
{
    return ptr1.get() == ptr2.get();
}

/// \brief Swap two CLBase objects
/// \ingroup OpenCL
template <typename CLPtr, typename Derived>
inline void swap(
    const CLBase<CLPtr, Derived> &ptr1, const CLBase<CLPtr, Derived> &ptr2)
{
    ptr1.swap(ptr2);
}

/// \brief OpenCL NDRange object
/// \ingroup OpenCL
class CLNDRange
{
    public:
    CLNDRange() : null_(true) {}

    explicit CLNDRange(std::size_t x, std::size_t y = 0, std::size_t z = 0)
        : null_(false)
    {
        std::get<0>(range_) = x;
        std::get<1>(range_) = y;
        std::get<2>(range_) = z;
    }

    const std::size_t *data() const { return null_ ? nullptr : range_.data(); }

    private:
    bool null_;
    std::array<std::size_t, 3> range_;
}; // class CLNDRange

/// \brief OpenCL `cl_device_id` wrapper
/// \ingroup OpenCL
class CLDevice : public CLBase<::cl_device_id, CLDevice>
{
    public:
    explicit CLDevice(::cl_device_id ptr = nullptr) { reset(ptr); }

    /// \brief `clCreateSubDevices`
    std::vector<CLDevice> sub_devices(
        const ::cl_device_partition_property *properties) const
    {
        ::cl_int status = CL_SUCCESS;

        ::cl_uint num = 0;
        status = ::clCreateSubDevices(get(), properties, 0, nullptr, &num);
        internal::cl_error_check(
            status, "CLDevice::sub_devices", "::clCreateSubDevices");

        std::vector<::cl_device_id> vec(num);
        status =
            ::clCreateSubDevices(get(), properties, num, vec.data(), nullptr);
        internal::cl_error_check(
            status, "CLDevice::sub_devices", "::clCreateSubDevices");

        std::vector<CLDevice> dev;
        for (auto ptr : vec)
            dev.push_back(CLDevice(ptr));

        return dev;
    }

    static ::cl_int get_info_param(::cl_device_id device,
        ::cl_device_info param_name, std::size_t param_value_size,
        void *param_value, std::size_t *param_value_size_ret)
    {
        ::cl_int status = ::clGetDeviceInfo(device, param_name,
            param_value_size, param_value, param_value_size_ret);
        internal::cl_error_check(
            status, "CLDevice::get_info", "::clGetDeviceInfo");

        return status;
    }

    static ::cl_int release(::cl_device_id ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        if (internal::cl_opencl_version(ptr) < 120)
            return CL_SUCCESS;

        ::cl_int status = ::clReleaseDevice(ptr);
        internal::cl_error_check(
            status, "CLDevice::release", "::clReleaseDevice");

        return status;
    }
}; // class CLDevice

/// \brief OpenCL `cl_platform_id` wrapper
/// \ingroup OpenCL
class CLPlatform : public CLBase<::cl_platform_id, CLPlatform>
{
    public:
    explicit CLPlatform(::cl_platform_id ptr = nullptr) { reset(ptr); }

    /// \brief `clGetPlatformIDs`
    static std::vector<CLPlatform> platforms()
    {
        ::cl_int status = CL_SUCCESS;

        ::cl_uint num = 0;
        status = ::clGetPlatformIDs(0, nullptr, &num);
        internal::cl_error_check(
            status, "CLPlatform::get_platform", "::clGetPlatformIDs");

        std::vector<::cl_platform_id> vec(num);
        status = ::clGetPlatformIDs(num, vec.data(), nullptr);
        internal::cl_error_check(
            status, "CLPlatform::get_platform", "::clGetPlatformIDs");

        std::vector<CLPlatform> plat;
        if (status == CL_SUCCESS)
            for (auto ptr : vec)
                plat.push_back(CLPlatform(ptr));

        return plat;
    }

    /// \brief `clUnloadPlatformCompiler`
    ::cl_int unload_compiler() const
    {
        ::cl_int status = ::clUnloadPlatformCompiler(get());
        internal::cl_error_check(status, "CLPlatform::unload_compiler",
            "::clUnloadPlatformCompiler");

        return status;
    }

    /// \brief `clGetDeviceIDs`
    std::vector<CLDevice> get_device(::cl_device_type dev_type) const
    {
        ::cl_int status = CL_SUCCESS;

        ::cl_uint num = 0;
        status = ::clGetDeviceIDs(get(), dev_type, 0, nullptr, &num);
        if (status == CL_DEVICE_NOT_FOUND)
            return std::vector<CLDevice>();
        internal::cl_error_check(
            status, "CLPlatform::get_device", "::clGetDeviceIDs");

        std::vector<::cl_device_id> vec(num);
        status = ::clGetDeviceIDs(get(), dev_type, num, vec.data(), nullptr);
        internal::cl_error_check(
            status, "CLPlatform::get_device", "::clGetDeviceIDs");

        std::vector<CLDevice> dev;
        if (status == CL_SUCCESS)
            for (auto ptr : vec)
                dev.push_back(CLDevice(ptr));

        return dev;
    }

    static ::cl_int get_info_param(::cl_platform_id platform,
        ::cl_platform_info param_name, std::size_t param_value_size,
        void *param_value, std::size_t *param_value_size_ret)
    {
        ::cl_int status = ::clGetPlatformInfo(platform, param_name,
            param_value_size, param_value, param_value_size_ret);
        internal::cl_error_check(
            status, "CLPlatform::get_info", "::clGetPlatformInfo");

        return status;
    }

    static ::cl_int release(::cl_platform_id) { return CL_SUCCESS; }
}; // class CLPlatform

/// \brief OpenCL `cl_context` wrapper
/// \ingroup OpenCL
class CLContext : public CLBase<::cl_context, CLContext>
{
    public:
    explicit CLContext(::cl_context ptr = nullptr) { reset(ptr); }

    /// \brief `clCreateContext`
    CLContext(const ::cl_context_properties *properties,
        const std::vector<CLDevice> &devices)
    {
        ::cl_int status = CL_SUCCESS;

        std::vector<::cl_device_id> dptr;
        for (const auto &dev : devices)
            dptr.push_back(dev.get());

        ::cl_context ptr = ::clCreateContext(properties,
            static_cast<::cl_uint>(devices.size()), dptr.data(), nullptr,
            nullptr, &status);
        internal::cl_error_check(
            status, "CLContext::CLContext", "::clCreateContext");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clCreateContext`
    CLContext(
        const ::cl_context_properties *properties, const CLDevice &device)
    {
        ::cl_int status = CL_SUCCESS;

        ::cl_device_id dptr = device.get();
        ::cl_context ptr =
            ::clCreateContext(properties, 1, &dptr, nullptr, nullptr, &status);
        internal::cl_error_check(
            status, "CLContext::CLContext", "::clCreateContext");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clCreateContextFromType`
    CLContext(
        const ::cl_context_properties &properties, ::cl_device_type dev_type)
    {
        ::cl_int status = CL_SUCCESS;

        ::cl_context ptr = ::clCreateContextFromType(
            &properties, dev_type, nullptr, nullptr, &status);
        internal::cl_error_check(
            status, "CLContext::CLContext", "::clCreateContextFromType");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clGetSupportedImageFormats`
    std::vector<::cl_image_format> get_supported_image_formats(
        ::cl_mem_flags flags, ::cl_mem_object_type image_type) const
    {
        ::cl_int status = CL_SUCCESS;

        ::cl_uint num = 0;
        status = ::clGetSupportedImageFormats(
            get(), flags, image_type, 0, nullptr, &num);
        internal::cl_error_check(status,
            "CLContext::get_supported_image_format",
            "::clGetSupportedImageFormats");

        std::vector<::cl_image_format> vec(num);
        status = ::clGetSupportedImageFormats(
            get(), flags, image_type, num, vec.data(), nullptr);
        internal::cl_error_check(status,
            "CLContext::get_supported_image_format",
            "::clGetSupportedImageFormats");

        return vec;
    }

    std::vector<CLDevice> get_device() const
    {
        std::vector<::cl_device_id> vec;
        ::cl_int status = get_info(CL_CONTEXT_DEVICES, vec);
        if (status != CL_SUCCESS)
            return std::vector<CLDevice>();

        std::vector<CLDevice> dev;
        for (auto ptr : vec)
            dev.push_back(CLDevice(ptr));

        return dev;
    }

    static ::cl_int get_info_param(::cl_context context,
        ::cl_context_info param_name, std::size_t param_value_size,
        void *param_value, std::size_t *param_value_size_ret)
    {
        ::cl_int status = ::clGetContextInfo(context, param_name,
            param_value_size, param_value, param_value_size_ret);
        internal::cl_error_check(
            status, "CLContext::get_info", "::clGetContextInfo");

        return status;
    }

    static ::cl_int release(::cl_context ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        ::cl_int status = ::clReleaseContext(ptr);
        internal::cl_error_check(
            status, "CLContext::release", "::clReleaseContext");

        return status;
    }
}; // class CLContext

/// \brief OpenCL `cl_event` wrapper
/// \ingroup OpenCL
class CLEvent : public CLBase<::cl_event, CLEvent>
{
    public:
    explicit CLEvent(::cl_event ptr = nullptr) { reset(ptr); }

    /// \brief `clCreateUserEvent`
    explicit CLEvent(const CLContext &context)
    {
        ::cl_int status = CL_SUCCESS;
        ::cl_event ptr = ::clCreateUserEvent(context.get(), &status);
        internal::cl_error_check(
            status, "CLEvent::CLEvent", "::clCreateUserEvent");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clSetUserEventStatus`
    ::cl_int set_status(::cl_int execution_status) const
    {
        ::cl_int status = ::clSetUserEventStatus(get(), execution_status);
        internal::cl_error_check(
            status, "CLEvent::set_status", "::clSetUserEventStatus");

        return status;
    }

    CLContext get_context() const
    {
        ::cl_context ptr = nullptr;
        ::cl_int status = get_info(CL_EVENT_CONTEXT, ptr);
        if (status != CL_SUCCESS)
            return CLContext();

        return CLContext(ptr);
    }

    inline CLCommandQueue get_command_queue() const;

    /// \brief `clGetEventProfilingInfo`
    ::cl_ulong profiling_command_queued() const
    {
        ::cl_ulong queued = 0;
        ::cl_int status = ::clGetEventProfilingInfo(get(),
            CL_PROFILING_COMMAND_QUEUED, sizeof(::cl_ulong), &queued, nullptr);
        internal::cl_error_check(status, "CLEvent::profiling_command_queued",
            "::clGetEventProfilingInfo");

        return queued;
    }

    /// \brief `clGetEventProfilingInfo`
    ::cl_ulong profiling_command_submit() const
    {
        ::cl_ulong submit = 0;
        ::cl_int status = ::clGetEventProfilingInfo(get(),
            CL_PROFILING_COMMAND_SUBMIT, sizeof(::cl_ulong), &submit, nullptr);
        internal::cl_error_check(status, "CLEvent::profiling_command_submit",
            "::clGetEventProfilingInfo");

        return submit;
    }

    /// \brief `clGetEventProfilingInfo`
    ::cl_ulong profiling_command_start() const
    {
        ::cl_ulong start = 0;
        ::cl_int status = ::clGetEventProfilingInfo(get(),
            CL_PROFILING_COMMAND_START, sizeof(::cl_ulong), &start, nullptr);
        internal::cl_error_check(status, "CLEvent::profiling_command_start",
            "::clGetEventProfilingInfo");

        return start;
    }

    /// \brief `clGetEventProfilingInfo`
    ::cl_ulong profiling_command_end() const
    {
        ::cl_ulong end = 0;
        ::cl_int status = ::clGetEventProfilingInfo(get(),
            CL_PROFILING_COMMAND_END, sizeof(::cl_ulong), &end, nullptr);
        internal::cl_error_check(status, "CLEvent::profiling_command_end",
            "::clGetEventProfilingInfo");

        return end;
    }

    /// \brief `clWaitForEvents`
    ::cl_int wait() const
    {
        ::cl_event ptr = get();
        ::cl_int status = ::clWaitForEvents(1, &ptr);
        internal::cl_error_check(status, "CLEvent::wait", "::clWaitForEvents");

        return status;
    }

    /// \brief `clWaitForEvents`
    static ::cl_int wait(const std::vector<CLEvent> &events)
    {
        if (events.size() == 0)
            return CL_SUCCESS;

        std::vector<::cl_event> vec;
        for (const auto &event : events)
            vec.push_back(event.get());
        ::cl_int status =
            ::clWaitForEvents(static_cast<::cl_uint>(vec.size()), vec.data());
        internal::cl_error_check(status, "CLEvent::wait", "::clWaitForEvents");

        return status;
    }

    static ::cl_int get_info_param(::cl_event event,
        ::cl_event_info param_name, std::size_t param_value_size,
        void *param_value, std::size_t *param_value_size_ret)
    {
        ::cl_int status = ::clGetEventInfo(event, param_name, param_value_size,
            param_value, param_value_size_ret);
        internal::cl_error_check(
            status, "CLEvent::get_info", "::clGetEventInfo");

        return status;
    }

    static ::cl_int release(::cl_event ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        ::cl_int status = ::clReleaseEvent(ptr);
        internal::cl_error_check(
            status, "CLEvent::release", "::clReleaseEvent");

        return status;
    }
}; // class CLEvent

/// \brief OpenCL `cl_mem` wrapper
/// \ingroup OpenCL
class CLMemory : public CLBase<::cl_mem, CLMemory>
{
    public:
    explicit CLMemory(::cl_mem ptr = nullptr) { reset(ptr); }

    /// \brief `clCreateBuffer`
    CLMemory(const CLContext &context, ::cl_mem_flags flags, std::size_t size,
        void *host_ptr)
    {
        ::cl_int status = CL_SUCCESS;
        ::cl_mem ptr =
            ::clCreateBuffer(context.get(), flags, size, host_ptr, &status);
        internal::cl_error_check(
            status, "CLMemory::CLMemory", "::clCreateBuffer");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clCreateImage`
    CLMemory(const CLContext &context, ::cl_mem_flags flags,
        const ::cl_image_format &image_format,
        const ::cl_image_desc &image_desc, void *host_ptr)
    {
        ::cl_int status = CL_SUCCESS;
        ::cl_mem ptr = ::clCreateImage(context.get(), flags, &image_format,
            &image_desc, host_ptr, &status);
        internal::cl_error_check(
            status, "CLMemory::CLMemory", "::clCreateImage");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clCreateSubBuffer`
    CLMemory sub_buffer(::cl_mem_flags flags,
        ::cl_buffer_create_type buffer_create_type,
        const void *buffer_create_info)
    {
        ::cl_int status = CL_SUCCESS;
        ::cl_mem ptr = ::clCreateSubBuffer(
            get(), flags, buffer_create_type, buffer_create_info, &status);
        internal::cl_error_check(
            status, "CLMemory::sub_buffer", "::clCreateSubBuffer");

        return CLMemory(ptr);
    }

    // clGetImageInfo

    static ::cl_int get_info_param(::cl_mem mem, ::cl_mem_info param_name,
        std::size_t param_value_size, void *param_value,
        std::size_t *param_value_size_ret)
    {
        ::cl_int status = ::clGetMemObjectInfo(mem, param_name,
            param_value_size, param_value, param_value_size_ret);
        internal::cl_error_check(
            status, "CLMemory::get_info", "::clGetMemObjectInfo");

        return status;
    }

    static ::cl_int release(::cl_mem ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        ::cl_int status = ::clReleaseMemObject(ptr);
        internal::cl_error_check(
            status, "CLMemory::release", "::clReleaseMemObject");

        return status;
    }
}; // class CLMemory

/// \brief OpenCL `cl_sampler` wrapper
/// \ingroup OpenCL
class CLSampler : public CLBase<::cl_sampler, CLSampler>
{
    public:
    explicit CLSampler(::cl_sampler ptr = nullptr) { reset(ptr); }

    /// \brief `clCreateSampler`
    CLSampler(const CLContext &context, ::cl_bool normalized_coords,
        ::cl_addressing_mode addressing_mode, ::cl_filter_mode filter_mode)
    {
        ::cl_int status = CL_SUCCESS;
        ::cl_sampler ptr = ::clCreateSampler(context.get(), normalized_coords,
            addressing_mode, filter_mode, &status);
        internal::cl_error_check(
            status, "CLSampler::CLSampler", "::clCreateSampler");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    CLContext get_context() const
    {
        ::cl_context ptr = nullptr;
        ::cl_int status = get_info(CL_SAMPLER_CONTEXT, ptr);
        if (status != CL_SUCCESS)
            return CLContext();

        return CLContext(ptr);
    }

    static ::cl_int get_info_param(::cl_sampler sampler,
        ::cl_sampler_info param_name, std::size_t param_value_size,
        void *param_value, std::size_t *param_value_size_ret)
    {
        ::cl_int status = ::clGetSamplerInfo(sampler, param_name,
            param_value_size, param_value, param_value_size_ret);
        internal::cl_error_check(
            status, "CLSampler::get_info", "::clGetSamplerInfo");

        return status;
    }

    static ::cl_int release(::cl_sampler ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        ::cl_int status = ::clReleaseSampler(ptr);
        internal::cl_error_check(
            status, "CLSampler::release", "::clReleaseSampler");

        return status;
    }
}; // class CLSampler

/// \brief OpenCL `cl_program` wrapper
/// \ingroup OpenCL
class CLProgram : public CLBase<::cl_program, CLProgram>
{
    public:
    explicit CLProgram(::cl_program ptr = nullptr) { reset(ptr); }

    /// \brief `clCreateProgramWithSource`
    CLProgram(
        const CLContext &context, const std::vector<std::string> &sources)
    {
        std::vector<const char *> strings;
        std::vector<std::size_t> lengths;
        for (const auto &src : sources) {
            strings.push_back(src.c_str());
            lengths.push_back(src.size());
        }
        ::cl_int status = CL_SUCCESS;
        ::cl_program ptr = ::clCreateProgramWithSource(context.get(),
            static_cast<::cl_uint>(sources.size()), strings.data(),
            lengths.data(), &status);
        internal::cl_error_check(
            status, "CLProgram::CLProgram", "::clCreateProgramWithSource");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clCreateProgramWithSource`
    CLProgram(const CLContext &context, const std::string &source)
    {
        const char *string = source.c_str();
        std::size_t length = source.size();
        ::cl_int status = CL_SUCCESS;
        ::cl_program ptr = ::clCreateProgramWithSource(
            context.get(), 1, &string, &length, &status);
        internal::cl_error_check(
            status, "CLProgram::CLProgram", "::clCreateProgramWithSource");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clCreateProgramWithBinary`
    CLProgram(const CLContext &context,
        const std::vector<std::pair<CLDevice, std::vector<unsigned char>>>
            &device_binaries,
        std::vector<::cl_int> &binary_status)
    {
        std::vector<::cl_device_id> devices;
        std::vector<const unsigned char *> binaries;
        std::vector<std::size_t> lengths;
        for (const auto &dev_bin : device_binaries) {
            devices.push_back(dev_bin.first.get());
            binaries.push_back(dev_bin.second.data());
            lengths.push_back(dev_bin.second.size());
        }
        binary_status.resize(device_binaries.size());
        ::cl_int status = CL_SUCCESS;
        ::cl_program ptr = ::clCreateProgramWithBinary(context.get(),
            static_cast<::cl_uint>(device_binaries.size()), devices.data(),
            lengths.data(), binaries.data(), binary_status.data(), &status);
        internal::cl_error_check(
            status, "CLProgram::CLProgram", "::clCreateProgramWithBinary");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clCreateProgramWithBinary`
    CLProgram(const CLContext &context, const CLDevice &device,
        const std::vector<unsigned char> &binary, ::cl_int &binary_status)
    {
        ::cl_device_id dev = device.get();
        const unsigned char *bin = binary.data();
        std::size_t length = binary.size();
        ::cl_int status = CL_SUCCESS;
        ::cl_program ptr = ::clCreateProgramWithBinary(
            context.get(), 1, &dev, &length, &bin, &binary_status, &status);
        internal::cl_error_check(
            status, "CLProgram::CLProgram", "::clCreateProgramWithBinary");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clCreateProgramWithBuiltInKernels`
    CLProgram(const CLContext &context, const std::vector<CLDevice> &devices,
        const std::string &kernel_name)
    {
        std::vector<::cl_device_id> dptr;
        for (const auto &dev : devices)
            dptr.push_back(dev.get());
        ::cl_int status = CL_SUCCESS;
        ::cl_program ptr = ::clCreateProgramWithBuiltInKernels(context.get(),
            static_cast<::cl_uint>(dptr.size()), dptr.data(),
            kernel_name.c_str(), &status);
        internal::cl_error_check(status, "CLProgram::CLProgram",
            "::clCreateProgramWithBuiltInKernels");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clCreateProgramWithBuiltInKernels`
    CLProgram(const CLContext &context, const CLDevice &device,
        const std::string &kernel_name)
    {
        ::cl_device_id dptr = device.get();
        ::cl_int status = CL_SUCCESS;
        ::cl_program ptr = ::clCreateProgramWithBuiltInKernels(
            context.get(), 1, &dptr, kernel_name.c_str(), &status);
        internal::cl_error_check(status, "CLProgram::CLProgram",
            "::clCreateProgramWithBuiltInKernels");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clLinkProgram`
    CLProgram(const CLContext &context, const std::vector<CLDevice> &devices,
        const std::string &options,
        const std::vector<CLProgram> &input_programs)
    {
        std::vector<::cl_device_id> dptr;
        for (const auto &dev : devices)
            dptr.push_back(dev.get());

        std::vector<::cl_program> pptr;
        for (const auto &prg : input_programs)
            pptr.push_back(prg.get());

        ::cl_int status = CL_SUCCESS;
        ::cl_program ptr = ::clLinkProgram(context.get(),
            static_cast<::cl_uint>(dptr.size()), dptr.data(), options.c_str(),
            static_cast<::cl_uint>(pptr.size()), pptr.data(), nullptr, nullptr,
            &status);
        internal::cl_error_check(
            status, "CLProgram::CLProgram", "::clLinkProgram");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clBuildProgram`
    ::cl_int build(
        const std::vector<CLDevice> &devices, const std::string &options) const
    {
        std::vector<::cl_device_id> dptr;
        for (const auto &dev : devices)
            dptr.push_back(dev.get());
        ::cl_int status =
            ::clBuildProgram(get(), static_cast<::cl_uint>(dptr.size()),
                dptr.data(), options.c_str(), nullptr, nullptr);
        internal::cl_error_check(
            status, "CLProgram::build", "::clBuildProgram");

        return status;
    }

    /// \brief `clCompileProgram`
    ::cl_int compile(const std::vector<CLDevice> &devices,
        const std::string &options,
        const std::vector<std::pair<CLProgram, std::string>>
            &input_headers_and_include_names) const
    {
        std::vector<::cl_device_id> dptr;
        for (const auto &dev : devices)
            dptr.push_back(dev.get());

        std::vector<::cl_program> input_headers;
        std::vector<const char *> include_names;
        for (const auto &hn : input_headers_and_include_names) {
            input_headers.push_back(hn.first.get());
            include_names.push_back(hn.second.c_str());
        }

        ::cl_int status = ::clCompileProgram(get(),
            static_cast<::cl_uint>(dptr.size()), dptr.data(), options.c_str(),
            static_cast<::cl_uint>(input_headers_and_include_names.size()),
            input_headers.data(), include_names.data(), nullptr, nullptr);
        internal::cl_error_check(
            status, "CLProgram::compile", "::clCompileProgram");

        return status;
    }

    /// \brief `clGetProgramBuildInfo`
    ::cl_build_status build_status(const CLDevice &device) const
    {
        ::cl_build_status val = CL_BUILD_SUCCESS;
        ::cl_int status = ::clGetProgramBuildInfo(get(), device.get(),
            CL_PROGRAM_BUILD_STATUS, sizeof(::cl_build_status), &val, nullptr);
        internal::cl_error_check(
            status, "CLProgram::build_status", "::clGetProgramBuildInfo");

        return val;
    }

    /// \brief `clGetProgramBuildInfo`
    std::string build_options(const CLDevice &device) const
    {
        ::cl_int status = CL_SUCCESS;

        std::size_t num = 0;
        status = ::clGetProgramBuildInfo(
            get(), device.get(), CL_PROGRAM_BUILD_OPTIONS, 0, nullptr, &num);
        internal::cl_error_check(
            status, "CLProgram::build_options", "::clGetProgramBuildInfo");
        if (status != CL_SUCCESS || num == 0)
            return std::string();

        std::vector<char> vec(num);
        status = ::clGetProgramBuildInfo(get(), device.get(),
            CL_PROGRAM_BUILD_OPTIONS, num, vec.data(), nullptr);
        vec.push_back(0);
        internal::cl_error_check(
            status, "CLProgram::build_options", "::clGetProgramBuildInfo");
        if (status != CL_SUCCESS)
            return std::string();

        return std::string(static_cast<const char *>(vec.data()));
    }

    /// \brief `clGetProgramBuildInfo`
    std::string build_log(const CLDevice &device) const
    {
        ::cl_int status = CL_SUCCESS;

        std::size_t num = 0;
        status = ::clGetProgramBuildInfo(
            get(), device.get(), CL_PROGRAM_BUILD_LOG, 0, nullptr, &num);
        internal::cl_error_check(
            status, "CLProgram::build_log", "::clGetProgramBuildInfo");
        if (status != CL_SUCCESS || num == 0)
            return std::string();

        std::vector<char> vec(num);
        status = ::clGetProgramBuildInfo(get(), device.get(),
            CL_PROGRAM_BUILD_LOG, num, vec.data(), nullptr);
        vec.push_back(0);
        internal::cl_error_check(
            status, "CLProgram::build_log", "::clGetProgramBuildInfo");
        if (status != CL_SUCCESS)
            return std::string();

        return std::string(static_cast<const char *>(vec.data()));
    }

    /// \brief `clGetProgramBuildInfo`
    ::cl_program_binary_type binary_type(const CLDevice &device) const
    {
        ::cl_program_binary_type val = CL_PROGRAM_BINARY_TYPE_NONE;
        ::cl_int status = ::clGetProgramBuildInfo(get(), device.get(),
            CL_PROGRAM_BINARY_TYPE, sizeof(::cl_program_binary_type), &val,
            nullptr);
        internal::cl_error_check(
            status, "CLProgram::binary_type", "::clGetProgramBuildInfo");

        return val;
    }

    /// \brief `clCreateKernelsInProgram`
    inline std::vector<CLKernel> get_kernels() const;

    CLContext get_context() const
    {
        ::cl_context ptr = nullptr;
        ::cl_int status = get_info(CL_PROGRAM_CONTEXT, ptr);
        if (status != CL_SUCCESS)
            return CLContext();

        return CLContext(ptr);
    }

    std::vector<CLDevice> get_device() const
    {
        std::vector<::cl_device_id> vec;
        ::cl_int status = get_info(CL_PROGRAM_DEVICES, vec);
        if (status != CL_SUCCESS)
            return std::vector<CLDevice>();

        std::vector<CLDevice> dev;
        for (auto ptr : vec)
            dev.push_back(CLDevice(ptr));

        return dev;
    }

    static ::cl_int get_info_param(::cl_program program,
        ::cl_program_info param_name, std::size_t param_value_size,
        void *param_value, std::size_t *param_value_size_ret)
    {
        ::cl_int status = ::clGetProgramInfo(program, param_name,
            param_value_size, param_value, param_value_size_ret);
        internal::cl_error_check(
            status, "CLProgram::get_info", "::clGetProgramInfo");

        return status;
    }

    static ::cl_int release(::cl_program ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        ::cl_int status = ::clReleaseProgram(ptr);
        internal::cl_error_check(
            status, "CLProgram::release", "::clReleaseProgram");

        return status;
    }
}; // class CLProgram

/// \brief OpenCL `cl_kernel` wrapper
/// \ingroup OpenCL
class CLKernel : public CLBase<::cl_kernel, CLKernel>
{
    public:
    explicit CLKernel(::cl_kernel ptr = nullptr) { reset(ptr); }

    /// \brief `clCreateKernel`
    CLKernel(const CLProgram &program, const std::string &kernel_name)
    {
        ::cl_int status = CL_SUCCESS;
        ::cl_kernel ptr =
            ::clCreateKernel(program.get(), kernel_name.c_str(), &status);
        internal::cl_error_check(
            status, "CLKernel::CLKernel", "::clCreateKernel");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    /// \brief `clSetKernelArg`
    template <typename T>
    ::cl_int set_arg(::cl_uint arg_index, const T &arg) const
    {
        ::cl_int status = ::clSetKernelArg(get(), arg_index, sizeof(T), &arg);
        internal::cl_error_check(
            status, "CLKernel::set_arg", "::clSetKernelArgs");

        return status;
    }

    /// \brief `clSetKernelArg`
    ::cl_int set_arg(::cl_uint arg_index, const CLMemory &arg) const
    {
        ::cl_mem mem = arg.get();
        ::cl_int status =
            ::clSetKernelArg(get(), arg_index, sizeof(::cl_mem), &mem);
        internal::cl_error_check(
            status, "CLKernel::set_arg", "::clSetKernelArgs");

        return status;
    }

    /// \brief `clGetKernelWorkGroupInfo`
    std::array<std::size_t, 3> global_work_size(const CLDevice &device) const
    {
        std::array<std::size_t, 3> val;
        val.fill(0);
        ::cl_int status = ::clGetKernelWorkGroupInfo(get(), device.get(),
            CL_KERNEL_GLOBAL_WORK_SIZE, sizeof(std::size_t) * 3, val.data(),
            nullptr);
        internal::cl_error_check(status, "CLKernel::global_work_size",
            "::clGetKernelWorkGroupInfo");

        return val;
    }

    /// \brief `clGetKernelWorkGroupInfo`
    std::size_t work_group_size(const CLDevice &device) const
    {
        std::size_t val = 0;
        ::cl_int status = ::clGetKernelWorkGroupInfo(get(), device.get(),
            CL_KERNEL_WORK_GROUP_SIZE, sizeof(std::size_t), &val, nullptr);
        internal::cl_error_check(
            status, "CLKernel::work_group_size", "::clGetKernelWorkGroupInfo");

        return val;
    }

    /// \brief `clGetKernelWorkGroupInfo`
    std::array<std::size_t, 3> compile_work_group_size(
        const CLDevice &device) const
    {
        std::array<std::size_t, 3> val;
        val.fill(0);
        ::cl_int status = ::clGetKernelWorkGroupInfo(get(), device.get(),
            CL_KERNEL_COMPILE_WORK_GROUP_SIZE, sizeof(std::size_t) * 3,
            val.data(), nullptr);
        internal::cl_error_check(status, "CLKernel::compile_work_group_size",
            "::clGetKernelWorkGroupInfo");

        return val;
    }

    /// \brief `clGetKernelWorkGroupInfo`
    ::cl_ulong local_mem_size(const CLDevice &device) const
    {
        std::size_t val = 0;
        ::cl_int status = ::clGetKernelWorkGroupInfo(get(), device.get(),
            CL_KERNEL_LOCAL_MEM_SIZE, sizeof(std::size_t), &val, nullptr);
        internal::cl_error_check(
            status, "CLKernel::local_mem_size", "::clGetKernelWorkGroupInfo");

        return val;
    }

    /// \brief `clGetKernelWorkGroupInfo`
    std::size_t preferred_work_group_size_multiple(
        const CLDevice &device) const
    {
        std::size_t val = 0;
        ::cl_int status = ::clGetKernelWorkGroupInfo(get(), device.get(),
            CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, sizeof(std::size_t),
            &val, nullptr);
        internal::cl_error_check(status,
            "CLKernel::preferred_work_group_size_multiple",
            "::clGetKernelWorkGroupInfo");

        return val;
    }

    /// \brief `clGetKernelWorkGroupInfo`
    ::cl_ulong private_mem_size(const CLDevice &device) const
    {
        std::size_t val = 0;
        ::cl_int status = ::clGetKernelWorkGroupInfo(get(), device.get(),
            CL_KERNEL_PRIVATE_MEM_SIZE, sizeof(std::size_t), &val, nullptr);
        internal::cl_error_check(status, "CLKernel::private_mem_szie",
            "::clGetKernelWorkGroupInfo");

        return val;
    }

    /// \brief `clGetKernelArgInfo`
    ::cl_kernel_arg_address_qualifier arg_address_qualifier(
        ::cl_uint arg_index) const
    {
        ::cl_kernel_arg_address_qualifier val = 0;
        ::cl_int status = ::clGetKernelArgInfo(get(), arg_index,
            CL_KERNEL_ARG_ADDRESS_QUALIFIER,
            sizeof(::cl_kernel_arg_address_qualifier), &val, nullptr);
        internal::cl_error_check(
            status, "CLKernel::arg_address_qualifier", "::clGetKernelArgInfo");

        return val;
    }

    /// \brief `clGetKernelArgInfo`
    ::cl_kernel_arg_access_qualifier arg_access_qualifier(
        ::cl_uint arg_index) const
    {
        ::cl_kernel_arg_access_qualifier val = 0;
        ::cl_int status = ::clGetKernelArgInfo(get(), arg_index,
            CL_KERNEL_ARG_ACCESS_QUALIFIER,
            sizeof(::cl_kernel_arg_access_qualifier), &val, nullptr);
        internal::cl_error_check(
            status, "CLKernel::arg_access_qualifier", "::clGetKernelArgInfo");

        return val;
    }

    /// \brief `clGetKernelArgInfo`
    std::string arg_type_name(::cl_uint arg_index) const
    {
        ::cl_int status = CL_SUCCESS;

        std::size_t num = 0;
        status = ::clGetKernelArgInfo(
            get(), arg_index, CL_KERNEL_ARG_TYPE_NAME, 0, nullptr, &num);
        internal::cl_error_check(
            status, "CLKernel::arg_type_name", "::clGetKernelArgInfo");
        if (status != CL_SUCCESS || num == 0)
            return std::string();

        std::vector<char> vec(num);
        status = ::clGetKernelArgInfo(get(), arg_index,
            CL_KERNEL_ARG_TYPE_NAME, num, vec.data(), nullptr);
        vec.push_back(0);
        internal::cl_error_check(
            status, "CLKernel::arg_type_name", "::clGetKernelArgInfo");
        if (status != CL_SUCCESS)
            return std::string();

        return std::string(static_cast<const char *>(vec.data()));
    }

    /// \brief `clGetKernelArgInfo`
    ::cl_kernel_arg_type_qualifier arg_type_qualifier(
        ::cl_uint arg_index) const
    {
        ::cl_kernel_arg_type_qualifier val = 0;
        ::cl_int status = ::clGetKernelArgInfo(get(), arg_index,
            CL_KERNEL_ARG_TYPE_QUALIFIER,
            sizeof(::cl_kernel_arg_type_qualifier), &val, nullptr);
        internal::cl_error_check(
            status, "CLKernel::arg_type_qualifier", "::clGetKernelArgInfo");

        return val;
    }

    /// \brief `clGetKernelArgInfo`
    std::string arg_name(::cl_uint arg_index) const
    {
        ::cl_int status = CL_SUCCESS;

        std::size_t num = 0;
        status = ::clGetKernelArgInfo(
            get(), arg_index, CL_KERNEL_ARG_NAME, 0, nullptr, &num);
        internal::cl_error_check(
            status, "CLKernel::arg_name", "::clGetKernelArgInfo");
        if (status != CL_SUCCESS || num == 0)
            return std::string();

        std::vector<char> vec(num);
        status = ::clGetKernelArgInfo(
            get(), arg_index, CL_KERNEL_ARG_NAME, num, vec.data(), nullptr);
        vec.push_back(0);
        internal::cl_error_check(
            status, "CLKernel::arg_name", "::clGetKernelArgInfo");
        if (status != CL_SUCCESS)
            return std::string();

        return std::string(static_cast<const char *>(vec.data()));
    }

    static ::cl_int get_info_param(::cl_kernel kernel,
        ::cl_kernel_info param_name, std::size_t param_value_size,
        void *param_value, std::size_t *param_value_size_ret)
    {
        ::cl_int status = ::clGetKernelInfo(kernel, param_name,
            param_value_size, param_value, param_value_size_ret);
        internal::cl_error_check(
            status, "CLKernel::get_info", "::clGetKernelInfo");

        return status;
    }

    static ::cl_int release(::cl_kernel ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        ::cl_int status = ::clReleaseKernel(ptr);
        internal::cl_error_check(
            status, "CLKernel::release", "::clReleaseKernel");

        return status;
    }
}; // class CLKernel

/// \brief OpenCL `cl_command_queue` wrapper
/// \ingroup OpenCL
class CLCommandQueue : public CLBase<::cl_command_queue, CLCommandQueue>
{
    public:
    explicit CLCommandQueue(::cl_command_queue ptr = nullptr) { reset(ptr); }

    /// \brief `clCreateCommandQueue`
    CLCommandQueue(const CLContext &context, const CLDevice &device,
        ::cl_command_queue_properties properties)
    {
        ::cl_int status = CL_SUCCESS;
        ::cl_command_queue ptr = ::clCreateCommandQueue(
            context.get(), device.get(), properties, &status);
        internal::cl_error_check(status, "CLCommandQueue::CLCommandQueue",
            "::clCreateCommandQueue");

        if (status == CL_SUCCESS)
            reset(ptr);
    }

    CLContext get_context() const
    {
        ::cl_context ptr = nullptr;
        ::cl_int status = get_info(CL_QUEUE_CONTEXT, ptr);
        if (status != CL_SUCCESS)
            return CLContext();

        return CLContext(ptr);
    }

    CLDevice get_device() const
    {
        ::cl_device_id ptr = nullptr;
        ::cl_int status = get_info(CL_QUEUE_DEVICE, ptr);
        if (status != CL_SUCCESS)
            return CLDevice();

        return CLDevice(ptr);
    }

    /// \brief `clEnqueueNDRangeKernel`
    ::cl_int enqueue_nd_range_kernel(const CLKernel &kernel,
        ::cl_uint work_dim, const CLNDRange &global_work_offset,
        const CLNDRange &global_work_size, const CLNDRange &local_work_size,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueNDRangeKernel(get(), kernel.get(),
            work_dim, global_work_offset.data(), global_work_size.data(),
            local_work_size.data(), static_cast<::cl_uint>(eptrs.size()),
            eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_nd_range_kernel",
            "::clEnqueueNDRangeKernel");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueTask`
    ::cl_int enqueue_task(const CLKernel &kernel,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueTask(get(), kernel.get(),
            static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(
            status, "CLCommandQueue::enqueue_task", "::clEnqueueTask");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    // clEnqueueNativeKernel

    /// \brief `clEnqueueReadBuffer`
    ::cl_int enqueue_read_buffer(const CLMemory &buffer,
        ::cl_bool blocking_read, std::size_t offset, std::size_t size,
        void *ptr, const std::vector<CLEvent> &event_wait_list,
        CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueReadBuffer(get(), buffer.get(),
            blocking_read, offset, size, ptr,
            static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status, "CLCommandQueue::enqueue_read_buffer",
            "::clEnqueueReadBuffer");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueWriteBuffer`
    ::cl_int enqueue_write_buffer(const CLMemory &buffer,
        ::cl_bool blocking_write, std::size_t offset, std::size_t size,
        void *ptr, const std::vector<CLEvent> &event_wait_list,
        CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueWriteBuffer(get(), buffer.get(),
            blocking_write, offset, size, ptr,
            static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_write_buffer", "::clEnqueueWriteBuffer");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueReadBufferRect`
    ::cl_int enqueue_read_buffer_rect(const CLMemory &buffer,
        ::cl_bool blocking_read,
        const std::array<std::size_t, 3> &buffer_origin,
        const std::array<std::size_t, 3> &host_origin,
        const std::array<std::size_t, 3> &region, std::size_t buffer_row_pitch,
        std::size_t buffer_slice_pitch, std::size_t host_row_pitch,
        std::size_t host_slice_pitch, void *ptr,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueReadBufferRect(get(), buffer.get(),
            blocking_read, buffer_origin.data(), host_origin.data(),
            region.data(), buffer_row_pitch, buffer_slice_pitch,
            host_row_pitch, host_slice_pitch, ptr,
            static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_read_buffer_rect",
            "::clEnqueueReadBufferRect");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueWriteBufferRect`
    ::cl_int enqueue_write_buffer_rect(const CLMemory &buffer,
        ::cl_bool blocking_write,
        const std::array<std::size_t, 3> &buffer_origin,
        const std::array<std::size_t, 3> &host_origin,
        const std::array<std::size_t, 3> &region, std::size_t buffer_row_pitch,
        std::size_t buffer_slice_pitch, std::size_t host_row_pitch,
        std::size_t host_slice_pitch, const void *ptr,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueWriteBufferRect(get(), buffer.get(),
            blocking_write, buffer_origin.data(), host_origin.data(),
            region.data(), buffer_row_pitch, buffer_slice_pitch,
            host_row_pitch, host_slice_pitch, ptr,
            static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_write_buffer_rect",
            "::clEnqueueWriteBufferRect");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueCopyBuffer`
    ::cl_int enqueue_copy_buffer(const CLMemory &src_buffer,
        const CLMemory &dst_buffer, std::size_t src_offset,
        std::size_t dst_offset, std::size_t size,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueCopyBuffer(get(), src_buffer.get(),
            dst_buffer.get(), src_offset, dst_offset, size,
            static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQeueue::enqueue_copy_buffer", "::clEnqueueCopyBuffer");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueCopyBufferRect`
    ::cl_int enqueue_copy_buffer_rect(const CLMemory &src_buffer,
        const CLMemory &dst_buffer,
        const std::array<std::size_t, 3> &src_origin,
        const std::array<std::size_t, 3> &dst_origin,
        const std::array<std::size_t, 3> &region, std::size_t src_row_pitch,
        std::size_t src_slice_pitch, std::size_t dst_row_pitch,
        std::size_t dst_slice_pitch,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueCopyBufferRect(get(), src_buffer.get(),
            dst_buffer.get(), src_origin.data(), dst_origin.data(),
            region.data(), src_row_pitch, src_slice_pitch, dst_row_pitch,
            dst_slice_pitch, static_cast<::cl_uint>(eptrs.size()),
            eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_copy_buffer_rect",
            "::clEnqueueCopyBufferRect");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueFillBuffer`
    ::cl_int enqueue_fill_buffer(const CLMemory &buffer, const void *pattern,
        std::size_t pattern_size, std::size_t offset, std::size_t size,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueFillBuffer(get(), buffer.get(), pattern,
            pattern_size, offset, size, static_cast<::cl_uint>(eptrs.size()),
            eptrs.data(), &eptr);
        internal::cl_error_check(status, "CLCommandQueue::enqueue_fill_buffer",
            "::clEnqueueFillBuffer");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueMapBuffer`
    void *enqueue_map_buffer(const CLMemory &buffer, ::cl_bool blocking_map,
        ::cl_map_flags map_flags, std::size_t offset, std::size_t size,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = CL_SUCCESS;
        void *ptr = ::clEnqueueMapBuffer(get(), buffer.get(), blocking_map,
            map_flags, offset, size, static_cast<::cl_uint>(eptrs.size()),
            eptrs.data(), &eptr, &status);
        internal::cl_error_check(status, "CLCommandQueue::enqueue_map_buffer",
            "::clEnqueueMapBuffer");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return ptr;
    }

    /// \brief `clEnqueueReadImage`
    ::cl_int enqueue_read_image(const CLMemory &image, ::cl_bool blocking_read,
        const std::array<std::size_t, 3> &origin,
        const std::array<std::size_t, 3> &region, std::size_t row_pitch,
        std::size_t slice_pitch, void *ptr,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status =
            ::clEnqueueReadImage(get(), image.get(), blocking_read,
                origin.data(), region.data(), row_pitch, slice_pitch, ptr,
                static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status, "CLCommandQueue::enqueue_read_image",
            "::clEnqueueReadImage");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueWriteImage`
    ::cl_int enqueue_write_image(const CLMemory &image,
        ::cl_bool blocking_write, const std::array<std::size_t, 3> &origin,
        const std::array<std::size_t, 3> &region, std::size_t row_pitch,
        std::size_t slice_pitch, void *ptr,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status =
            ::clEnqueueWriteImage(get(), image.get(), blocking_write,
                origin.data(), region.data(), row_pitch, slice_pitch, ptr,
                static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status, "CLCommandQueue::enqueue_write_image",
            "::clEnqueueWriteImage");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueCopyImage`
    ::cl_int enqueue_copy_image(const CLMemory &src_image,
        const CLMemory &dst_image,
        const std::array<std::size_t, 3> &src_origin,
        const std::array<std::size_t, 3> &dst_origin,
        const std::array<std::size_t, 3> &region,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status =
            ::clEnqueueCopyImage(get(), src_image.get(), dst_image.get(),
                src_origin.data(), dst_origin.data(), region.data(),
                static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status, "CLCommandQueue::enqueue_copy_image",
            "::clEnqueueCopyImage");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueFillImage`
    ::cl_int enqueue_fill_image(const CLMemory &image, const void *fill_color,
        const std::array<std::size_t, 3> &origin,
        const std::array<std::size_t, 3> &region,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueFillImage(get(), image.get(), fill_color,
            origin.data(), region.data(), static_cast<::cl_uint>(eptrs.size()),
            eptrs.data(), &eptr);
        internal::cl_error_check(status, "CLCommandQueue::enqueue_fill_image",
            "::clEnqueueFillImage");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueMapImage`
    void *enqueue_map_image(const CLMemory &image, ::cl_bool blocking_map,
        ::cl_map_flags map_flags, const std::array<std::size_t, 3> &origin,
        const std::array<std::size_t, 3> &region, std::size_t &image_row_pitch,
        std::size_t &image_slice_pitch,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = CL_SUCCESS;
        void *ptr = ::clEnqueueMapImage(get(), image.get(), blocking_map,
            map_flags, origin.data(), region.data(), &image_row_pitch,
            &image_slice_pitch, static_cast<::cl_uint>(eptrs.size()),
            eptrs.data(), &eptr, &status);
        internal::cl_error_check(status, "CLCommandQueue::enqueue_map_image",
            "::clEnqueueMapImage");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return ptr;
    }

    /// \brief `clEnqueueCopyImageToBuffer`
    ::cl_int enqueue_copy_image_to_buffer(const CLMemory &src_image,
        const CLMemory &dst_buffer,
        const std::array<std::size_t, 3> &src_origin,
        const std::array<std::size_t, 3> &region, std::size_t dst_offset,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueCopyImageToBuffer(get(), src_image.get(),
            dst_buffer.get(), src_origin.data(), region.data(), dst_offset,
            static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_copy_image_to_buffer",
            "::clEnqueueCopyImageToBuffer");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueCopyBufferToImage`
    ::cl_int enqueue_copy_buffer_to_image(const CLMemory &src_buffer,
        const CLMemory &dst_image, std::size_t src_offset,
        const std::array<std::size_t, 3> &dst_origin,
        const std::array<std::size_t, 3> &region,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueCopyBufferToImage(get(), src_buffer.get(),
            dst_image.get(), src_offset, dst_origin.data(), region.data(),
            static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_copy_buffer_to_image",
            "::clEnqueueCopyBufferToImage");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueUnmapMemObject`
    ::cl_int enqueue_unmap_mem_object(const CLMemory &memobj, void *mapped_ptr,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status =
            ::clEnqueueUnmapMemObject(get(), memobj.get(), mapped_ptr,
                static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_unmap_mem_object",
            "::clEnqueueUnmapMemObject");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueMigrateMemObjects`
    ::cl_int enqueue_migrate_mem_objects(
        const std::vector<CLMemory> &mem_objects,
        ::cl_mem_migration_flags flags,
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_mem> mptrs;
        for (const auto &m : mem_objects)
            mptrs.push_back(m.get());

        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueMigrateMemObjects(get(),
            static_cast<::cl_uint>(mptrs.size()), mptrs.data(), flags,
            static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_migrate_mem_objects",
            "::clEnqueueMigrateMemObjects");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueMarkerWithWaitList`
    ::cl_int enqueue_marker_with_wait_list(
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueMarkerWithWaitList(
            get(), static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_marker_with_wait_list",
            "::clEnqueueMarkerWithWaitList");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clEnqueueBarrierWithWaitList`
    ::cl_int enqueue_barrier_with_wait_list(
        const std::vector<CLEvent> &event_wait_list, CLEvent &event) const
    {
        std::vector<::cl_event> eptrs;
        for (const auto &e : event_wait_list)
            eptrs.push_back(e.get());
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueBarrierWithWaitList(
            get(), static_cast<::cl_uint>(eptrs.size()), eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_barrier_with_wait_list",
            "::clEnqueueBarrierWithWaitList");
        if (status == CL_SUCCESS)
            event.reset(eptr);

        return status;
    }

    /// \brief `clFlush`
    ::cl_int flush() const
    {
        ::cl_int status = ::clFlush(get());
        internal::cl_error_check(status, "CLCommandQueue::flush", "::clFlush");

        return status;
    }

    /// \brief `clFinish`
    ::cl_int finish() const
    {
        ::cl_int status = ::clFinish(get());
        internal::cl_error_check(
            status, "CLCommandQueue::finish", "::clFinish");

        return status;
    }

    static ::cl_int get_info_param(::cl_command_queue command_queue,
        ::cl_command_queue_info param_name, std::size_t param_value_size,
        void *param_value, std::size_t *param_value_size_ret)
    {
        ::cl_int status = ::clGetCommandQueueInfo(command_queue, param_name,
            param_value_size, param_value, param_value_size_ret);
        internal::cl_error_check(
            status, "CLCommandQueue::get_info", "::clGetCommandQueueInfo");

        return status;
    }

    static ::cl_int release(::cl_command_queue ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        ::cl_int status = ::clReleaseCommandQueue(ptr);
        internal::cl_error_check(
            status, "CLCommandQueue::release", "::clReleaseCommandQueue");

        return status;
    }
}; // class CLCommandQueue

inline CLCommandQueue CLEvent::get_command_queue() const
{
    ::cl_command_queue ptr = nullptr;
    ::cl_int status = get_info(CL_EVENT_COMMAND_QUEUE, ptr);
    if (status != CL_SUCCESS)
        return CLCommandQueue();

    return CLCommandQueue(ptr);
}

inline std::vector<CLKernel> CLProgram::get_kernels() const
{
    ::cl_int status = CL_SUCCESS;

    ::cl_uint num = 0;
    status = ::clCreateKernelsInProgram(get(), 0, nullptr, &num);
    internal::cl_error_check(
        status, "CLProgram::get_kernels", "::clCreateKernelsInProgram");

    std::vector<::cl_kernel> vec(num);
    status = ::clCreateKernelsInProgram(get(), num, vec.data(), nullptr);
    internal::cl_error_check(
        status, "CLProgram::get_kernels", "::clCreateKernelsInProgram");

    std::vector<CLKernel> kern;
    for (auto ptr : vec)
        kern.push_back(CLKernel(ptr));

    return kern;
}

} // namespace vsmc

#if defined(VSMC_INTEL)
#pragma warning(pop)
#endif

#endif // VSMC_OPENCL_CL_TYPE_HPP

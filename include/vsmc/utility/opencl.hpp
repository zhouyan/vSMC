//============================================================================
// vSMC/include/vsmc/utility/opencl.hpp
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

#ifndef VSMC_UTILITY_OPENCL_HPP
#define VSMC_UTILITY_OPENCL_HPP

#include <vsmc/internal/common.hpp>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
#endif

#ifndef CL_VERSION_1_2
#error OpenCL 1.2 support required
#endif

#ifdef VSMC_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif

#ifdef VSMC_GCC
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#ifdef VSMC_INTEL
#pragma warning(push)
#pragma warning(disable : 1478)
#endif

#define VSMC_DEFINE_UTILITY_OPENCL_GET_INFO(Class, type, name, Name)          \
    ::cl_int name##_info(::cl_##type##_info param_name,                       \
        std::size_t param_value_size, void *param_value,                      \
        std::size_t *param_value_size_ret) const                              \
    {                                                                         \
        return internal::cl_error_check(                                      \
            ::clGet##Name##Info(get(), param_name, param_value_size,          \
                param_value, param_value_size_ret),                           \
            "CL" #Class "::" #name "_info", "::clGet" #Name "Info");          \
    }                                                                         \
                                                                              \
    template <typename ParamType>                                             \
    ::cl_int name##_info(                                                     \
        ::cl_##type##_info param_name, ParamType &param_value) const          \
    {                                                                         \
        ParamType v;                                                          \
        ::cl_int status =                                                     \
            name##_info(param_name, sizeof(ParamType), &v, nullptr);          \
        if (status == CL_SUCCESS)                                             \
            param_value = v;                                                  \
                                                                              \
        return status;                                                        \
    }                                                                         \
                                                                              \
    template <typename ParamType>                                             \
    ::cl_int name##_info(::cl_##type##_info param_name,                       \
        std::vector<ParamType> &param_value) const                            \
    {                                                                         \
        ::cl_int status = CL_SUCCESS;                                         \
                                                                              \
        std::size_t param_value_size = 0;                                     \
        status = name##_info(param_name, 0, nullptr, &param_value_size);      \
        if (status != CL_SUCCESS)                                             \
            return status;                                                    \
                                                                              \
        std::vector<ParamType> v(param_value_size / sizeof(ParamType));       \
        status =                                                              \
            name##_info(param_name, param_value_size, v.data(), nullptr);     \
        if (status == CL_SUCCESS)                                             \
            param_value = std::move(v);                                       \
                                                                              \
        return status;                                                        \
    }                                                                         \
                                                                              \
    ::cl_int name##_info(                                                     \
        ::cl_##type##_info param_name, std::string &param_value) const        \
    {                                                                         \
        std::vector<char> v;                                                  \
        ::cl_int status = name##_info(param_name, v);                         \
        v.push_back('\0');                                                    \
        if (status == CL_SUCCESS)                                             \
            param_value = static_cast<const char *>(v.data());                \
                                                                              \
        return status;                                                        \
    }

namespace vsmc
{

namespace internal
{

#if VSMC_NO_RUNTIME_ASSERT
inline ::cl_int cl_error_check(::cl_int status, const char *, const char *)
{
    return status;
}
#else  // VSMC_NO_RUNTIME_ASSERT
inline ::cl_int cl_error_check(::cl_int status, const char *cpp, const char *c)
{
    if (status == CL_SUCCESS)
        return status;

    std::string msg;
    msg += "**";
    msg += cpp;
    msg += "**";
    msg += " failed";
    msg += "; OpenCL function: ";
    msg += c;
    msg += "; Error code: ";
    msg += itos(status);

    VSMC_RUNTIME_ASSERT((status == CL_SUCCESS), msg.c_str());

    return status;
}
#endif // VSMC_NO_RUNTIME_ASSERT

template <typename CLType>
inline std::vector<typename CLType::pointer> cl_vec_cpp2c(
    ::cl_uint n, const CLType *ptr)
{
    std::vector<typename CLType::pointer> vec;
    for (::cl_uint i = 0; i != n; ++i)
        vec.push_back(ptr[i].get());

    return vec;
}

template <typename CLType>
inline std::vector<CLType> cl_vec_c2cpp(
    ::cl_uint n, const typename CLType::pointer *ptr)
{
    std::vector<CLType> vec;
    for (::cl_uint i = 0; i != n; ++i)
        vec.push_back(CLType(ptr[i]));

    return vec;
}

} // namespace vsmc::internal

class CLNDRange;
class CLDevice;
class CLPlatform;
class CLContextProperties;
class CLContext;
class CLEvent;
class CLMemory;
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

    void reset(pointer ptr)
    {
        if (ptr != ptr_.get())
            ptr_.reset(ptr, [](pointer p) { Derived::release(p); });
    }

    void swap(CLBase<CLPtr, Derived> &other) { ptr_.swap(other.ptr_); }

    pointer get() const { return ptr_.get(); }

    long use_count() const { return ptr_.use_count(); }

    bool unique() const { return ptr_.unique(); }

    explicit operator bool() const { return bool(ptr_); }

    protected:
    void reset_ptr(pointer ptr) { reset(ptr); }

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
    return ptr1.get() != ptr2.get();
}

/// \brief Swap two CLBase objects
/// \ingroup OpenCL
template <typename CLPtr, typename Derived>
inline void swap(
    const CLBase<CLPtr, Derived> &ptr1, const CLBase<CLPtr, Derived> &ptr2)
{
    ptr1.swap(ptr2);
}

/// \brief OpenCL NDRange concept
/// \ingroup OpenCL
class CLNDRange
{
    public:
    CLNDRange() : dim_(0), range_({0, 0, 0}) {}

    explicit CLNDRange(std::size_t x) : dim_(1), range_({x, 0, 0}) {}

    CLNDRange(std::size_t x, std::size_t y) : dim_(2), range_({x, y, 0}) {}

    CLNDRange(std::size_t x, std::size_t y, std::size_t z)
        : dim_(3), range_({x, y, z})
    {
    }

    std::size_t dim() const { return dim_; }

    const std::size_t *data() const
    {
        return dim_ == 0 ? nullptr : range_.data();
    }

    private:
    const std::size_t dim_;
    const Array<std::size_t, 3> range_;
}; // class CLNDRange

/// \brief OpenCL `cl_device_id`
/// \ingroup OpenCL
class CLDevice : public CLBase<::cl_device_id, CLDevice>
{
    public:
    explicit CLDevice(::cl_device_id ptr = nullptr) { reset_ptr(ptr); }

    /// \brief `clCreateSubDevices`
    std::vector<CLDevice> creat_sub_devices(
        const ::cl_device_partition_property *properties) const
    {
        ::cl_uint n = 0;
        if (internal::cl_error_check(
                ::clCreateSubDevices(get(), properties, 0, nullptr, &n),
                "CLDevice::sub_devices",
                "::clCreateSubDevices") != CL_SUCCESS) {
            return std::vector<CLDevice>();
        }

        std::vector<::cl_device_id> vec(n);
        if (internal::cl_error_check(::clCreateSubDevices(get(), properties, n,
                                         vec.data(), nullptr),
                "CLDevice::sub_devices",
                "::clCreateSubDevices") != CL_SUCCESS) {
            return std::vector<CLDevice>();
        }

        return internal::cl_vec_c2cpp<CLDevice>(n, vec.data());
    }

    /// \brief `clGetDeviceInfo`
    VSMC_DEFINE_UTILITY_OPENCL_GET_INFO(Device, device, get, Device)

    /// \brief `clReleaseDevice`
    static ::cl_int release(::cl_device_id ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        return internal::cl_error_check(
            ::clReleaseDevice(ptr), "CLDevice::release", "::clReleaseDevice");
    }
}; // class CLDevice

/// \brief OpenCL `cl_platform_id`
/// \ingroup OpenCL
class CLPlatform : public CLBase<::cl_platform_id, CLPlatform>
{
    public:
    explicit CLPlatform(::cl_platform_id ptr = nullptr) { reset_ptr(ptr); }

    /// \brief `clGetDeviceIDs`
    std::vector<CLDevice> get_device(::cl_device_type device_type) const
    {
        ::cl_uint n = 0;
        if (internal::cl_error_check(
                ::clGetDeviceIDs(get(), device_type, 0, nullptr, &n),
                "CLPlatform::get_device", "::clGetDeviceIDs") != CL_SUCCESS) {
            return std::vector<CLDevice>();
        }

        std::vector<::cl_device_id> vec(n);
        if (internal::cl_error_check(
                ::clGetDeviceIDs(get(), device_type, n, vec.data(), nullptr),
                "CLPlatform::get_device", "::clGetDeviceIDs") != CL_SUCCESS) {
            return std::vector<CLDevice>();
        }

        return internal::cl_vec_c2cpp<CLDevice>(n, vec.data());
    }

    /// \brief `clUnloadPlatformCompiler`
    ::cl_int unload_compiler() const
    {
        return internal::cl_error_check(::clUnloadPlatformCompiler(get()),
            "CLPlatform::unload_compiler", "::clUnloadPlatformCompiler");
    }

    /// \brief `clGetPlatformInfo`
    VSMC_DEFINE_UTILITY_OPENCL_GET_INFO(Platform, platform, get, Platform)

    static ::cl_int release(::cl_platform_id) { return CL_SUCCESS; }
}; // class CLPlatform

/// \brief `clGetPlatformIDs`
/// \ingroup OpenCL
inline std::vector<CLPlatform> cl_get_platform()
{
    ::cl_uint n = 0;
    if (internal::cl_error_check(::clGetPlatformIDs(0, nullptr, &n),
            "CLPlatform::get_platform", "::clGetPlatformIDs") != CL_SUCCESS) {
        return std::vector<CLPlatform>();
    }

    std::vector<::cl_platform_id> vec(n);
    if (internal::cl_error_check(::clGetPlatformIDs(n, vec.data(), nullptr),
            "CLPlatform::get_platform", "::clGetPlatformIDs") != CL_SUCCESS) {
        return std::vector<CLPlatform>();
    }

    return internal::cl_vec_c2cpp<CLPlatform>(n, vec.data());
}

/// \brief OpenCL `cl_context_properties`
/// \ingroup OpenCL
class CLContextProperties
{
    public:
    explicit CLContextProperties(const CLPlatform &platform)
        : properties_({CL_CONTEXT_PLATFORM,
              reinterpret_cast<::cl_context_properties>(platform.get()), 0, 0,
              0})
    {
    }

    CLContextProperties(
        const CLPlatform &platform, ::cl_bool interop_user_sync)
        : properties_({CL_CONTEXT_PLATFORM,
              reinterpret_cast<::cl_context_properties>(platform.get()),
              CL_CONTEXT_INTEROP_USER_SYNC, interop_user_sync, 0})
    {
    }

    const ::cl_context_properties *data() const { return properties_.data(); }

    private:
    const Array<::cl_context_properties, 5> properties_;
}; // class CLContextProperty

/// \brief OpenCL `cl_context`
/// \ingroup OpenCL
class CLContext : public CLBase<::cl_context, CLContext>
{
    public:
    using pfn_notify_type = void(CL_CALLBACK *)(
        const char *, const void *, std::size_t, void *);

    explicit CLContext(::cl_context ptr = nullptr) { reset_ptr(ptr); }

    /// \brief `clCreateContext`
    CLContext(const CLContextProperties &properties, ::cl_uint num_devices,
        const CLDevice *devices, pfn_notify_type pfn_notify = nullptr,
        void *user_data = nullptr)
    {
        auto vec = internal::cl_vec_cpp2c(num_devices, devices);
        ::cl_int status = CL_SUCCESS;
        ::cl_context ptr = ::clCreateContext(properties.data(), num_devices,
            vec.data(), pfn_notify, user_data, &status);
        if (internal::cl_error_check(status, "CLContext::CLContext",
                "::clCreateContext") == CL_SUCCESS) {
            reset_ptr(ptr);
        }
    }

    /// \brief `clCreateContextFromType`
    CLContext(const CLContextProperties &properties,
        ::cl_device_type device_type, pfn_notify_type pfn_notify = nullptr,
        void *user_data = nullptr)
    {
        ::cl_int status = CL_SUCCESS;
        ::cl_context ptr = ::clCreateContextFromType(
            properties.data(), device_type, pfn_notify, user_data, &status);
        if (internal::cl_error_check(status, "CLContext::CLContext",
                "::clCreateContextFromType") == CL_SUCCESS) {
            reset_ptr(ptr);
        }
    }

    /// \brief `CL_CONTEXT_DEVICES`
    std::vector<CLDevice> get_device() const
    {
        std::vector<::cl_device_id> vec;
        if (get_info(CL_CONTEXT_DEVICES, vec) != CL_SUCCESS)
            return std::vector<CLDevice>();

        return internal::cl_vec_c2cpp<CLDevice>(
            static_cast<::cl_uint>(vec.size()), vec.data());
    }

    /// \brief `clGetContextInfo`
    VSMC_DEFINE_UTILITY_OPENCL_GET_INFO(Context, context, get, Context)

    /// \brief `clReleaseContext`
    static ::cl_int release(::cl_context ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        return internal::cl_error_check(::clReleaseContext(ptr),
            "CLContext::release", "::clReleaseContext");
    }
}; // class CLContext

/// \brief OpenCL `cl_event`
/// \ingroup OpenCL
class CLEvent : public CLBase<::cl_event, CLEvent>
{
    public:
    explicit CLEvent(::cl_event ptr = nullptr) { reset_ptr(ptr); }

    /// \brief `clCreateUserEvent`
    explicit CLEvent(const CLContext &context)
    {
        ::cl_int status = CL_SUCCESS;
        ::cl_event ptr = ::clCreateUserEvent(context.get(), &status);
        if (internal::cl_error_check(status, "CLEvent::CLEvent",
                "::clCreateUserEvent") == CL_SUCCESS) {
            reset_ptr(ptr);
        }
    }

    /// \brief `clSetUserEventStatus`
    ::cl_int set_status(::cl_int execution_status) const
    {
        return internal::cl_error_check(
            ::clSetUserEventStatus(get(), execution_status),
            "CLEvent::set_status", "::clSetUserEventStatus");
    }

    /// \brief `CL_EVENT_CONTEXT`
    CLContext get_context() const
    {
        ::cl_context ptr = nullptr;
        return get_info(CL_EVENT_CONTEXT, ptr) == CL_SUCCESS ? CLContext(ptr) :
                                                               CLContext();
    }

    /// \brief `CL_EVENT_COMMAND_QUEUE`
    inline CLCommandQueue get_command_queue() const;

    /// \brief `clWaitForEvents`
    ::cl_int wait() const
    {
        ::cl_event ptr = get();
        return internal::cl_error_check(
            ::clWaitForEvents(1, &ptr), "CLEvent::wait", "::clWaitForEvents");
    }

    /// \brief `clWaitForEvents`
    static ::cl_int wait(::cl_uint num_events, CLEvent *events)
    {
        if (num_events == 0)
            return CL_SUCCESS;

        auto vec = internal::cl_vec_cpp2c(num_events, events);
        return internal::cl_error_check(
            ::clWaitForEvents(num_events, vec.data()), "CLEvent::wait",
            "::clWaitForEvents");
    }

    /// \brief `clGetEventInfo`
    VSMC_DEFINE_UTILITY_OPENCL_GET_INFO(Event, event, get, Event)

    /// \brief `clGetEventProfilingInfo`
    VSMC_DEFINE_UTILITY_OPENCL_GET_INFO(
        Event, profiling, get_profiling, EventProfiling)

    /// \brief `clReleaseEvent`
    static ::cl_int release(::cl_event ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        return internal::cl_error_check(
            ::clReleaseEvent(ptr), "CLEvent::release", "::clReleaseEvent");
    }
}; // class CLEvent

/// \brief OpenCL `cl_mem`
/// \ingroup OpenCL
class CLMemory : public CLBase<::cl_mem, CLMemory>
{
    public:
    explicit CLMemory(::cl_mem ptr = nullptr) { reset_ptr(ptr); }

    /// \brief `clCreateBuffer`
    CLMemory(const CLContext &context, ::cl_mem_flags flags, std::size_t size,
        void *host_ptr = nullptr)
    {
        ::cl_int status = CL_SUCCESS;
        ::cl_mem ptr =
            ::clCreateBuffer(context.get(), flags, size, host_ptr, &status);
        if (internal::cl_error_check(status, "CLMemory::CLMemory",
                "::clCreateBuffer") == CL_SUCCESS) {
            reset_ptr(ptr);
        }
    }

    /// \brief `clCreateSubBuffer`
    CLMemory sub_buffer(::cl_mem_flags flags,
        ::cl_buffer_create_type buffer_create_type,
        const void *buffer_create_info = nullptr)
    {
        ::cl_int status = CL_SUCCESS;
        ::cl_mem ptr = ::clCreateSubBuffer(
            get(), flags, buffer_create_type, buffer_create_info, &status);
        return internal::cl_error_check(status, "CLMemory::sub_buffer",
                   "::clCreateSubBuffer") == CL_SUCCESS ?
            CLMemory(ptr) :
            CLMemory();
    }

    /// \brief `clGetMemObjectInfo`
    VSMC_DEFINE_UTILITY_OPENCL_GET_INFO(Memory, mem, get, MemObject)

    /// \brief `clReleaseMemObject`
    static ::cl_int release(::cl_mem ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        return internal::cl_error_check(::clReleaseMemObject(ptr),
            "CLMemory::release", "::clReleaseMemObject");
    }
}; // class CLMemory

/// \brief OpenCL `cl_program`
/// \ingroup OpenCL
class CLProgram : public CLBase<::cl_program, CLProgram>
{
    public:
    using pfn_notify_type = void(CL_CALLBACK *)(::cl_program, void *);

    explicit CLProgram(::cl_program ptr = nullptr) { reset_ptr(ptr); }

    /// \brief `clCreateProgramWithSource`
    CLProgram(
        const CLContext &context, ::cl_uint count, const std::string *strings)
    {
        std::vector<const char *> str;
        std::vector<std::size_t> len;
        for (::cl_uint i = 0; i != count; ++i) {
            str.push_back(strings[i].c_str());
            len.push_back(strings[i].size());
        }

        ::cl_int status = CL_SUCCESS;
        ::cl_program ptr = ::clCreateProgramWithSource(
            context.get(), count, str.data(), len.data(), &status);
        if (internal::cl_error_check(status, "CLProgram::CLProgram",
                "::clCreateProgramWithSource") == CL_SUCCESS) {
            reset_ptr(ptr);
        }
    }

    /// \brief `clCreateProgramWithBinary`
    CLProgram(const CLContext &context, ::cl_uint num_devices,
        const CLDevice *devices, const std::vector<unsigned char> *binaries)
    {
        auto vec = internal::cl_vec_cpp2c(num_devices, devices);
        std::vector<const unsigned char *> bin;
        std::vector<std::size_t> len;
        for (::cl_uint i = 0; i != num_devices; ++i) {
            bin.push_back(binaries[i].data());
            len.push_back(binaries[i].size());
        }

        ::cl_int status = CL_SUCCESS;
        std::vector<::cl_int> binary_status(num_devices, CL_SUCCESS);
        ::cl_program ptr =
            ::clCreateProgramWithBinary(context.get(), num_devices, vec.data(),
                len.data(), bin.data(), binary_status.data(), &status);
        bool binary_success = true;
        for (auto bs : binary_status) {
            if (internal::cl_error_check(bs, "CLProgram::CLProgram",
                    "::clCreateProgramWithBinary") != CL_SUCCESS) {
                binary_success = false;
                break;
            }
        }
        if (binary_success &&
            internal::cl_error_check(status, "CLProgram::CLProgram",
                "::clCreateProgramWithBinary") == CL_SUCCESS) {
            reset_ptr(ptr);
        }
    }

    /// \brief `clCreateProgramWithBuiltInKernels`
    CLProgram(const CLContext &context, ::cl_uint num_devices,
        const CLDevice *devices, const std::string &kernel_names)
    {
        auto vec = internal::cl_vec_cpp2c(num_devices, devices);
        ::cl_int status = CL_SUCCESS;
        ::cl_program ptr = ::clCreateProgramWithBuiltInKernels(context.get(),
            num_devices, vec.data(), kernel_names.c_str(), &status);
        if (internal::cl_error_check(status, "CLProgram::CLProgram",
                "::clCreateProgramWithBuiltInKernels") == CL_SUCCESS) {
            reset_ptr(ptr);
        }
    }

    /// \brief `clLinkProgram`
    CLProgram(const CLContext &context, ::cl_uint num_devices,
        const CLDevice *devices, const std::string &options = std::string(),
        ::cl_uint num_input_programs = 0,
        const CLProgram *input_programs = nullptr,
        pfn_notify_type pfn_notify = nullptr, void *user_data = nullptr)
    {
        auto dvec = internal::cl_vec_cpp2c(num_devices, devices);
        auto pvec = internal::cl_vec_cpp2c(num_input_programs, input_programs);
        ::cl_int status = CL_SUCCESS;
        ::cl_program ptr = ::clLinkProgram(context.get(), num_devices,
            dvec.data(), options.c_str(), num_input_programs, pvec.data(),
            pfn_notify, user_data, &status);
        if (internal::cl_error_check(status, "CLProgram::CLProgram",
                "::clLinkProgram") == CL_SUCCESS) {
            reset_ptr(ptr);
        }
    }

    /// \brief `clBuildProgram`
    ::cl_int build(::cl_uint num_devices, const CLDevice *devices,
        const std::string &options = std::string(),
        pfn_notify_type pfn_notify = nullptr, void *user_data = nullptr) const
    {
        auto vec = internal::cl_vec_cpp2c(num_devices, devices);
        ::cl_int status = ::clBuildProgram(get(), num_devices, vec.data(),
            options.c_str(), pfn_notify, user_data);
#if !VSMC_NO_RUNTIME_ASSERT
        if (status != CL_SUCCESS) {
            for (::cl_uint i = 0; i != num_devices; ++i) {
                std::cerr << std::string(80, '=');
                std::string name;
                devices[i].get_info(CL_DEVICE_NAME, name);
                std::cerr << "Device: " << name << std::endl;
                std::cerr << std::string(80, '-');
                std::cerr << "Status: ";
                switch (build_status(devices[i])) {
                    case CL_BUILD_NONE:
                        std::cerr << "CL_BUILD_NONE" << std::endl;
                        break;
                    case CL_BUILD_ERROR:
                        std::cerr << "CL_BUILD_ERROR" << std::endl;
                        break;
                    case CL_BUILD_SUCCESS:
                        std::cerr << "CL_BUILD_SUCCESS" << std::endl;
                        break;
                    case CL_BUILD_IN_PROGRESS:
                        std::cerr << "CL_BUILD_IN_PROGRESS" << std::endl;
                        break;
                    default: break;
                }
                std::cerr << std::string(80, '-');
                std::cerr << "Options: " << build_options(devices[i])
                          << std::endl;
                std::cerr << std::string(80, '-');
                std::cerr << build_log(devices[i]) << std::endl;
                std::cerr << std::string(80, '-');
            }
        }
#endif

        return internal::cl_error_check(
            status, "CLProgram::build", "::clBuildProgram");
    }

    /// \brief `clCompileProgram`
    ::cl_int compile(::cl_uint num_devices, const CLDevice *devices,
        const std::string &options = std::string(),
        ::cl_uint num_input_headers = 0,
        const CLProgram *input_headers = nullptr,
        const std::string *header_include_names = nullptr,
        pfn_notify_type pf_notify = nullptr, void *user_data = nullptr)
    {
        auto dvec = internal::cl_vec_cpp2c(num_devices, devices);
        auto pvec = internal::cl_vec_cpp2c(num_input_headers, input_headers);
        std::vector<const char *> inc_names;
        for (::cl_uint i = 0; i != num_input_headers; ++i)
            inc_names.push_back(header_include_names[i].c_str());

        return internal::cl_error_check(
            ::clCompileProgram(get(), num_devices, dvec.data(),
                options.c_str(), num_input_headers, pvec.data(),
                inc_names.data(), pf_notify, user_data),
            "CLProgram::compile", "::clCompileProgram");
    }

    /// \brief `CL_PROGRAM_BUILD_STATUS`
    ::cl_build_status build_status(const CLDevice &device) const
    {
        ::cl_build_status v;
        get_build_info(device, CL_PROGRAM_BUILD_STATUS, v);

        return v;
    }

    /// \brief `CL_PROGRAM_BUILD_OPTIONS`
    std::string build_options(const CLDevice &device) const
    {
        std::string v;
        get_build_info(device, CL_PROGRAM_BUILD_OPTIONS, v);

        return v;
    }

    /// \brief `CL_PROGRAM_BUILD_LOG`
    std::string build_log(const CLDevice &device) const
    {
        std::string v;
        get_build_info(device, CL_PROGRAM_BUILD_LOG, v);

        return v;
    }

    /// \brief `clCreateKernelsInProgram`
    inline std::vector<CLKernel> create_kernels() const;

    /// \brief `CL_PROGRAM_CONTEXT`
    CLContext get_context() const
    {
        ::cl_context ptr = nullptr;
        return get_info(CL_PROGRAM_CONTEXT, ptr) == CL_SUCCESS ?
            CLContext(ptr) :
            CLContext();
    }

    /// \brief `CL_PROGRAM_DEVICES`
    std::vector<CLDevice> get_device() const
    {
        std::vector<::cl_device_id> vec;
        if (get_info(CL_PROGRAM_DEVICES, vec) != CL_SUCCESS)
            return std::vector<CLDevice>();

        return internal::cl_vec_c2cpp<CLDevice>(
            static_cast<::cl_uint>(vec.size()), vec.data());
    }

    /// \brief `clGetProgramInfo`
    VSMC_DEFINE_UTILITY_OPENCL_GET_INFO(Program, program, get, Program)

    /// \brief `clGetProgramBuildInfo`
    ::cl_int get_build_info(const CLDevice &device,
        ::cl_program_build_info param_name, std::size_t param_value_size,
        void *param_value, std::size_t *param_value_size_ret) const
    {
        return internal::cl_error_check(
            ::clGetProgramBuildInfo(get(), device.get(), param_name,
                param_value_size, param_value, param_value_size_ret),
            "CLKernel::get_build_info", "::clGetProgramBuildInfo");
    }

    template <typename ParamType>
    ::cl_int get_build_info(const CLDevice &device,
        ::cl_program_build_info param_name, ParamType &param_value) const
    {
        ParamType v;
        ::cl_int status =
            get_build_info(device, param_name, sizeof(ParamType), &v, nullptr);
        if (status == CL_SUCCESS)
            param_value = v;

        return status;
    }

    template <typename ParamType>
    ::cl_int get_build_info(const CLDevice &device,
        ::cl_program_build_info param_name,
        std::vector<ParamType> &param_value) const
    {
        ::cl_int status = CL_SUCCESS;

        std::size_t param_value_size = 0;
        status =
            get_build_info(device, param_name, 0, nullptr, &param_value_size);
        if (status != CL_SUCCESS)
            return status;

        std::vector<ParamType> v(param_value_size / sizeof(ParamType));
        status = get_build_info(
            device, param_name, param_value_size, v.data(), nullptr);
        if (status == CL_SUCCESS)
            param_value = std::move(v);

        return status;
    }

    ::cl_int get_build_info(const CLDevice &device,
        ::cl_program_build_info param_name, std::string &param_value) const
    {
        std::vector<char> v;
        ::cl_int status = get_build_info(device, param_name, v);
        v.push_back('\0');
        if (status == CL_SUCCESS)
            param_value = static_cast<const char *>(v.data());

        return status;
    }

    /// \brief `clReleaseProgram`
    static ::cl_int release(::cl_program ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        return internal::cl_error_check(::clReleaseProgram(ptr),
            "CLProgram::release", "::clReleaseProgram");
    }
}; // class CLProgram

/// \brief OpenCL `cl_kernel`
/// \ingroup OpenCL
class CLKernel : public CLBase<::cl_kernel, CLKernel>
{
    public:
    explicit CLKernel(::cl_kernel ptr = nullptr) { reset_ptr(ptr); }

    /// \brief `clCreateKernel`
    CLKernel(const CLProgram &program, const std::string &kernel_name)
    {
        ::cl_int status = CL_SUCCESS;
        ::cl_kernel ptr =
            ::clCreateKernel(program.get(), kernel_name.c_str(), &status);
        if (internal::cl_error_check(status, "CLKernel::CLKernel",
                "::clCreateKernel") == CL_SUCCESS) {
            reset_ptr(ptr);
        }
    }

    /// \brief `clSetKernelArg`
    template <typename T>
    ::cl_int set_arg(::cl_uint arg_index, const T &arg) const
    {
        return internal::cl_error_check(
            ::clSetKernelArg(get(), arg_index, sizeof(T), &arg),
            "CLKernel::set_arg", "::clSetKernelArgs");
    }

    /// \brief `clSetKernelArg`
    ::cl_int set_arg(::cl_uint arg_index, const CLMemory &arg) const
    {
        ::cl_mem mem = arg.get();
        return internal::cl_error_check(
            ::clSetKernelArg(get(), arg_index, sizeof(::cl_mem), &mem),
            "CLKernel::set_arg", "::clSetKernelArgs");
    }

    VSMC_DEFINE_UTILITY_OPENCL_GET_INFO(Kernel, kernel, get, Kernel)

    /// \brief `clGetKernelWorkGroupInfo`
    ::cl_ulong get_work_group_info(const CLDevice &device,
        ::cl_kernel_work_group_info param_name, std::size_t param_value_size,
        void *param_value, std::size_t *param_value_size_ret) const
    {
        return internal::cl_error_check(
            ::clGetKernelWorkGroupInfo(get(), device.get(), param_name,
                param_value_size, param_value, param_value_size_ret),
            "CLKernel::get_work_group_info", "::clGetKernelWorkGroupInfo");
    }

    template <typename ParamType>
    ::cl_int get_work_group_info(const CLDevice &device,
        ::cl_kernel_work_group_info param_name, ParamType &param_value) const
    {
        ParamType v;
        ::cl_int status = get_work_group_info(
            device, param_name, sizeof(ParamType), &v, nullptr);
        if (status == CL_SUCCESS)
            param_value = v;

        return status;
    }

    template <typename ParamType>
    ::cl_int get_work_group_info(const CLDevice &device,
        ::cl_kernel_work_group_info param_name,
        std::vector<ParamType> &param_value) const
    {
        ::cl_int status = CL_SUCCESS;

        std::size_t param_value_size = 0;
        status = get_work_group_info(
            device, param_name, 0, nullptr, &param_value_size);
        if (status != CL_SUCCESS)
            return status;

        std::vector<ParamType> v(param_value_size / sizeof(ParamType));
        status = get_work_group_info(
            device, param_name, param_value_size, v.data(), nullptr);
        if (status == CL_SUCCESS)
            param_value = std::move(v);

        return status;
    }

    ::cl_int get_work_group_info(const CLDevice &device,
        ::cl_kernel_work_group_info param_name, std::string &param_value) const
    {
        std::vector<char> v;
        ::cl_int status = get_work_group_info(device, param_name, v);
        v.push_back('\0');
        if (status == CL_SUCCESS)
            param_value = static_cast<const char *>(v.data());

        return status;
    }

    /// \brief `clGetKernelArgInfo`
    ::cl_ulong get_arg_info(::cl_uint arg_indx,
        ::cl_kernel_arg_info param_name, std::size_t param_value_size,
        void *param_value, std::size_t *param_value_size_ret) const
    {
        return internal::cl_error_check(
            ::clGetKernelArgInfo(get(), arg_indx, param_name, param_value_size,
                param_value, param_value_size_ret),
            "CLKernel::get_arg_info", "::clGetKernelArgInfo");
    }

    template <typename ParamType>
    ::cl_int get_arg_info(::cl_uint arg_indx, ::cl_kernel_arg_info param_name,
        ParamType &param_value) const
    {
        ParamType v;
        ::cl_int status =
            get_arg_info(arg_indx, param_name, sizeof(ParamType), &v, nullptr);
        if (status == CL_SUCCESS)
            param_value = v;

        return status;
    }

    template <typename ParamType>
    ::cl_int get_arg_info(::cl_uint arg_indx, ::cl_kernel_arg_info param_name,
        std::vector<ParamType> &param_value) const
    {
        ::cl_int status = CL_SUCCESS;

        std::size_t param_value_size = 0;
        status =
            get_arg_info(arg_indx, param_name, 0, nullptr, &param_value_size);
        if (status != CL_SUCCESS)
            return status;

        std::vector<ParamType> v(param_value_size / sizeof(ParamType));
        status = get_arg_info(
            arg_indx, param_name, param_value_size, v.data(), nullptr);
        if (status == CL_SUCCESS)
            param_value = std::move(v);

        return status;
    }

    ::cl_int get_arg_info(::cl_uint arg_indx, ::cl_kernel_arg_info param_name,
        std::string &param_value) const
    {
        std::vector<char> v;
        ::cl_int status = get_arg_info(arg_indx, param_name, v);
        v.push_back('\0');
        if (status == CL_SUCCESS)
            param_value = static_cast<const char *>(v.data());

        return status;
    }

    /// \brief `clReleaseKernel`
    static ::cl_int release(::cl_kernel ptr)
    {
        if (ptr == nullptr)
            return CL_SUCCESS;

        return internal::cl_error_check(
            ::clReleaseKernel(ptr), "CLKernel::release", "::clReleaseKernel");
    }
}; // class CLKernel

/// \brief OpenCL `cl_command_queue`
/// \ingroup OpenCL
class CLCommandQueue : public CLBase<::cl_command_queue, CLCommandQueue>
{
    public:
    explicit CLCommandQueue(::cl_command_queue ptr = nullptr)
    {
        reset_ptr(ptr);
    }

    /// \brief `clCreateCommandQueue`
    CLCommandQueue(const CLContext &context, const CLDevice &device,
        ::cl_command_queue_properties properties = 0)
    {
        ::cl_int status = CL_SUCCESS;
        ::cl_command_queue ptr = ::clCreateCommandQueue(
            context.get(), device.get(), properties, &status);
        if (internal::cl_error_check(status, "CLCommandQueue::CLCommandQueue",
                "::clCreateCommandQueue") == CL_SUCCESS) {
            reset_ptr(ptr);
        }
    }

    /// \brief `CL_QUEUE_CONTEXT`
    CLContext get_context() const
    {
        ::cl_context ptr = nullptr;
        return get_info(CL_QUEUE_CONTEXT, ptr) == CL_SUCCESS ? CLContext(ptr) :
                                                               CLContext();
    }

    /// \brief `CL_QUEUE_DEVICE`
    CLDevice get_device() const
    {
        ::cl_device_id ptr = nullptr;
        return get_info(CL_QUEUE_DEVICE, ptr) == CL_SUCCESS ? CLDevice(ptr) :
                                                              CLDevice();
    }

    /// \brief `clEnqueueNDRangeKernel`
    ::cl_int enqueue_nd_range_kernel(const CLKernel &kernel,
        ::cl_uint work_dim, const CLNDRange &global_work_offset,
        const CLNDRange &global_work_size, const CLNDRange &local_work_size,
        ::cl_uint num_events_in_wait_list = 0,
        const CLEvent *event_wait_list = nullptr,
        CLEvent *event = nullptr) const
    {
        auto eptrs =
            internal::cl_vec_cpp2c(num_events_in_wait_list, event_wait_list);
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueNDRangeKernel(get(), kernel.get(),
            work_dim, global_work_offset.data(), global_work_size.data(),
            local_work_size.data(), num_events_in_wait_list, eptrs.data(),
            &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_nd_range_kernel",
            "::clEnqueueNDRangeKernel");
        if (status == CL_SUCCESS && event != nullptr)
            event->reset(eptr);

        return status;
    }

    /// \brief `clEnqueueReadBuffer`
    ::cl_int enqueue_read_buffer(const CLMemory &buffer,
        ::cl_bool blocking_read, std::size_t offset, std::size_t size,
        void *ptr, ::cl_uint num_events_in_wait_list = 0,
        const CLEvent *event_wait_list = nullptr,
        CLEvent *event = nullptr) const
    {
        auto eptrs =
            internal::cl_vec_cpp2c(num_events_in_wait_list, event_wait_list);
        ::cl_event eptr = nullptr;

        ::cl_int status =
            ::clEnqueueReadBuffer(get(), buffer.get(), blocking_read, offset,
                size, ptr, num_events_in_wait_list, eptrs.data(), &eptr);
        internal::cl_error_check(status, "CLCommandQueue::enqueue_read_buffer",
            "::clEnqueueReadBuffer");
        if (status == CL_SUCCESS && event != nullptr)
            event->reset(eptr);

        return status;
    }

    /// \brief `clEnqueueWriteBuffer`
    ::cl_int enqueue_write_buffer(const CLMemory &buffer,
        ::cl_bool blocking_write, std::size_t offset, std::size_t size,
        void *ptr, ::cl_uint num_events_in_wait_list = 0,
        const CLEvent *event_wait_list = nullptr,
        CLEvent *event = nullptr) const
    {
        auto eptrs =
            internal::cl_vec_cpp2c(num_events_in_wait_list, event_wait_list);
        ::cl_event eptr = nullptr;

        ::cl_int status =
            ::clEnqueueWriteBuffer(get(), buffer.get(), blocking_write, offset,
                size, ptr, num_events_in_wait_list, eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_write_buffer", "::clEnqueueWriteBuffer");
        if (status == CL_SUCCESS && event != nullptr)
            event->reset(eptr);

        return status;
    }

    /// \brief `clEnqueueReadBufferRect`
    ::cl_int enqueue_read_buffer_rect(const CLMemory &buffer,
        ::cl_bool blocking_read, const Array<std::size_t, 3> &buffer_origin,
        const Array<std::size_t, 3> &host_origin,
        const Array<std::size_t, 3> &region, std::size_t buffer_row_pitch,
        std::size_t buffer_slice_pitch, std::size_t host_row_pitch,
        std::size_t host_slice_pitch, void *ptr,
        ::cl_uint num_events_in_wait_list = 0,
        const CLEvent *event_wait_list = nullptr,
        CLEvent *event = nullptr) const
    {
        auto eptrs =
            internal::cl_vec_cpp2c(num_events_in_wait_list, event_wait_list);
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueReadBufferRect(get(), buffer.get(),
            blocking_read, buffer_origin.data(), host_origin.data(),
            region.data(), buffer_row_pitch, buffer_slice_pitch,
            host_row_pitch, host_slice_pitch, ptr, num_events_in_wait_list,
            eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_read_buffer_rect",
            "::clEnqueueReadBufferRect");
        if (status == CL_SUCCESS && event != nullptr)
            event->reset(eptr);

        return status;
    }

    /// \brief `clEnqueueWriteBufferRect`
    ::cl_int enqueue_write_buffer_rect(const CLMemory &buffer,
        ::cl_bool blocking_write, const Array<std::size_t, 3> &buffer_origin,
        const Array<std::size_t, 3> &host_origin,
        const Array<std::size_t, 3> &region, std::size_t buffer_row_pitch,
        std::size_t buffer_slice_pitch, std::size_t host_row_pitch,
        std::size_t host_slice_pitch, const void *ptr,
        ::cl_uint num_events_in_wait_list = 0,
        const CLEvent *event_wait_list = nullptr,
        CLEvent *event = nullptr) const
    {
        auto eptrs =
            internal::cl_vec_cpp2c(num_events_in_wait_list, event_wait_list);
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueWriteBufferRect(get(), buffer.get(),
            blocking_write, buffer_origin.data(), host_origin.data(),
            region.data(), buffer_row_pitch, buffer_slice_pitch,
            host_row_pitch, host_slice_pitch, ptr, num_events_in_wait_list,
            eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_write_buffer_rect",
            "::clEnqueueWriteBufferRect");
        if (status == CL_SUCCESS && event != nullptr)
            event->reset(eptr);

        return status;
    }

    /// \brief `clEnqueueCopyBuffer`
    ::cl_int enqueue_copy_buffer(const CLMemory &src_buffer,
        const CLMemory &dst_buffer, std::size_t src_offset,
        std::size_t dst_offset, std::size_t size,
        ::cl_uint num_events_in_wait_list = 0,
        const CLEvent *event_wait_list = nullptr,
        CLEvent *event = nullptr) const
    {
        auto eptrs =
            internal::cl_vec_cpp2c(num_events_in_wait_list, event_wait_list);
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueCopyBuffer(get(), src_buffer.get(),
            dst_buffer.get(), src_offset, dst_offset, size,
            num_events_in_wait_list, eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQeueue::enqueue_copy_buffer", "::clEnqueueCopyBuffer");
        if (status == CL_SUCCESS && event != nullptr)
            event->reset(eptr);

        return status;
    }

    /// \brief `clEnqueueCopyBufferRect`
    ::cl_int enqueue_copy_buffer_rect(const CLMemory &src_buffer,
        const CLMemory &dst_buffer, const Array<std::size_t, 3> &src_origin,
        const Array<std::size_t, 3> &dst_origin,
        const Array<std::size_t, 3> &region, std::size_t src_row_pitch,
        std::size_t src_slice_pitch, std::size_t dst_row_pitch,
        std::size_t dst_slice_pitch, ::cl_uint num_events_in_wait_list = 0,
        const CLEvent *event_wait_list = nullptr,
        CLEvent *event = nullptr) const
    {
        auto eptrs =
            internal::cl_vec_cpp2c(num_events_in_wait_list, event_wait_list);
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueCopyBufferRect(get(), src_buffer.get(),
            dst_buffer.get(), src_origin.data(), dst_origin.data(),
            region.data(), src_row_pitch, src_slice_pitch, dst_row_pitch,
            dst_slice_pitch, num_events_in_wait_list, eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_copy_buffer_rect",
            "::clEnqueueCopyBufferRect");
        if (status == CL_SUCCESS && event != nullptr)
            event->reset(eptr);

        return status;
    }

    /// \brief `clEnqueueFillBuffer`
    ::cl_int enqueue_fill_buffer(const CLMemory &buffer, const void *pattern,
        std::size_t pattern_size, std::size_t offset, std::size_t size,
        ::cl_uint num_events_in_wait_list = 0,
        const CLEvent *event_wait_list = nullptr,
        CLEvent *event = nullptr) const
    {
        auto eptrs =
            internal::cl_vec_cpp2c(num_events_in_wait_list, event_wait_list);
        ::cl_event eptr = nullptr;

        ::cl_int status =
            ::clEnqueueFillBuffer(get(), buffer.get(), pattern, pattern_size,
                offset, size, num_events_in_wait_list, eptrs.data(), &eptr);
        internal::cl_error_check(status, "CLCommandQueue::enqueue_fill_buffer",
            "::clEnqueueFillBuffer");
        if (status == CL_SUCCESS && event != nullptr)
            event->reset(eptr);

        return status;
    }

    /// \brief `clEnqueueMapBuffer`
    void *enqueue_map_buffer(const CLMemory &buffer, ::cl_bool blocking_map,
        ::cl_map_flags map_flags, std::size_t offset, std::size_t size,
        ::cl_uint num_events_in_wait_list = 0,
        const CLEvent *event_wait_list = nullptr,
        CLEvent *event = nullptr) const
    {
        auto eptrs =
            internal::cl_vec_cpp2c(num_events_in_wait_list, event_wait_list);
        ::cl_event eptr = nullptr;

        ::cl_int status = CL_SUCCESS;
        void *ptr = ::clEnqueueMapBuffer(get(), buffer.get(), blocking_map,
            map_flags, offset, size, num_events_in_wait_list, eptrs.data(),
            &eptr, &status);
        internal::cl_error_check(status, "CLCommandQueue::enqueue_map_buffer",
            "::clEnqueueMapBuffer");
        if (status == CL_SUCCESS && event != nullptr)
            event->reset(eptr);

        return ptr;
    }

    /// \brief `clEnqueueUnmapMemObject`
    ::cl_int enqueue_unmap_mem_object(const CLMemory &memobj, void *mapped_ptr,
        ::cl_uint num_events_in_wait_list = 0,
        const CLEvent *event_wait_list = nullptr,
        CLEvent *event = nullptr) const
    {
        auto eptrs =
            internal::cl_vec_cpp2c(num_events_in_wait_list, event_wait_list);
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueUnmapMemObject(get(), memobj.get(),
            mapped_ptr, num_events_in_wait_list, eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_unmap_mem_object",
            "::clEnqueueUnmapMemObject");
        if (status == CL_SUCCESS && event != nullptr)
            event->reset(eptr);

        return status;
    }

    /// \brief `clEnqueueMigrateMemObjects`
    ::cl_int enqueue_migrate_mem_objects(::cl_uint num_mem_objects,
        const CLMemory *mem_objects, ::cl_mem_migration_flags flags,
        ::cl_uint num_events_in_wait_list = 0,
        const CLEvent *event_wait_list = nullptr,
        CLEvent *event = nullptr) const
    {
        auto mptrs = internal::cl_vec_cpp2c(num_mem_objects, mem_objects);
        auto eptrs =
            internal::cl_vec_cpp2c(num_events_in_wait_list, event_wait_list);
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueMigrateMemObjects(get(), num_mem_objects,
            mptrs.data(), flags, num_events_in_wait_list, eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_migrate_mem_objects",
            "::clEnqueueMigrateMemObjects");
        if (status == CL_SUCCESS && event != nullptr)
            event->reset(eptr);

        return status;
    }

    /// \brief `clEnqueueMarkerWithWaitList`
    ::cl_int enqueue_marker_with_wait_list(
        ::cl_uint num_events_in_wait_list = 0,
        const CLEvent *event_wait_list = nullptr,
        CLEvent *event = nullptr) const
    {
        auto eptrs =
            internal::cl_vec_cpp2c(num_events_in_wait_list, event_wait_list);
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueMarkerWithWaitList(
            get(), num_events_in_wait_list, eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_marker_with_wait_list",
            "::clEnqueueMarkerWithWaitList");
        if (status == CL_SUCCESS && event != nullptr)
            event->reset(eptr);

        return status;
    }

    /// \brief `clEnqueueBarrierWithWaitList`
    ::cl_int enqueue_barrier_with_wait_list(
        ::cl_uint num_events_in_wait_list = 0,
        const CLEvent *event_wait_list = nullptr,
        CLEvent *event = nullptr) const
    {
        auto eptrs =
            internal::cl_vec_cpp2c(num_events_in_wait_list, event_wait_list);
        ::cl_event eptr = nullptr;

        ::cl_int status = ::clEnqueueBarrierWithWaitList(
            get(), num_events_in_wait_list, eptrs.data(), &eptr);
        internal::cl_error_check(status,
            "CLCommandQueue::enqueue_barrier_with_wait_list",
            "::clEnqueueBarrierWithWaitList");
        if (status == CL_SUCCESS && event != nullptr)
            event->reset(eptr);

        return status;
    }

    /// \brief `clFlush`
    ::cl_int flush() const
    {
        return internal::cl_error_check(
            ::clFlush(get()), "CLCommandQueue::flush", "::clFlush");
    }

    /// \brief `clFinish`
    ::cl_int finish() const
    {
        return internal::cl_error_check(
            ::clFinish(get()), "CLCommandQueue::finish", "::clFinish");
    }

    /// \brief `clGetCommandQueueInfo`
    VSMC_DEFINE_UTILITY_OPENCL_GET_INFO(
        CommandQueue, command_queue, get, CommandQueue)

    /// \brief `clReleaseCommandQueue`
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
    return get_info(CL_EVENT_COMMAND_QUEUE, ptr) == CL_SUCCESS ?
        CLCommandQueue(ptr) :
        CLCommandQueue();
}

inline std::vector<CLKernel> CLProgram::create_kernels() const
{
    ::cl_int status = CL_SUCCESS;

    ::cl_uint n = 0;
    if (internal::cl_error_check(
            ::clCreateKernelsInProgram(get(), 0, nullptr, &n),
            "CLProgram::create_kernels",
            "::clCreateKernelsInProgram") != CL_SUCCESS) {
        return std::vector<CLKernel>();
    }

    std::vector<::cl_kernel> vec(n);
    if (internal::cl_error_check(
            ::clCreateKernelsInProgram(get(), n, vec.data(), nullptr),
            "CLProgram::create_kernels",
            "::clCreateKernelsInProgram") != CL_SUCCESS) {
        return std::vector<CLKernel>();
    }

    return internal::cl_vec_c2cpp<CLKernel>(n, vec.data());
}

} // namespace vsmc

#ifdef VSMC_CLANG
#pragma clang diagnostic pop
#endif

#ifdef VSMC_GCC
#pragma GCC diagnostic pop
#endif

#ifdef VSMC_INTEL
#pragma warning(pop)
#endif

#endif // VSMC_UTILITY_OPENCL_HPP

//============================================================================
// vSMC/include/vsmc/opencl/cl_manager.hpp
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

#ifndef VSMC_OPENCL_CL_MANAGER_HPP
#define VSMC_OPENCL_CL_MANAGER_HPP

#include <vsmc/opencl/internal/common.hpp>
#include <vsmc/opencl/cl_setup.hpp>
#include <vsmc/opencl/cl_manip.hpp>
#include <vsmc/opencl/cl_query.hpp>

#define VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(func)                     \
    VSMC_RUNTIME_ASSERT((setup()),                                            \
        "**CLManager::" #func "** CAN ONLY BE CALLED AFTER TRUE "             \
        "**CLManager::setup**")

#define VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_PLATFORM                 \
    VSMC_RUNTIME_WARNING(                                                     \
        false, "**CLManager::setup** FAILED TO SETUP A PLATFORM")

#define VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_CONTEXT                  \
    VSMC_RUNTIME_WARNING(                                                     \
        false, "**CLManager::setup** FAILED TO SETUP A CONTEXT")

#define VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_DEVICE                   \
    VSMC_RUNTIME_WARNING(                                                     \
        false, "**CLManager::setup** FAILED TO SETUP A DEVICE")

#define VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_COMMAND_QUEUE            \
    VSMC_RUNTIME_WARNING(                                                     \
        false, "**CLManager::setup** FAILED TO SETUP A COMMAND_QUEUE")

#define VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(func, block, event)      \
    VSMC_RUNTIME_WARNING((block || event != nullptr),                         \
        "**CLManager::" #func " NOT BLOCKING BUT WITH NULL EVENT")

namespace vsmc
{

/// \brief OpenCL Manager
/// \ingroup OpenCL
///
/// \details
/// Each instance of CLManager is an singleton. Different `ID` template
/// parameter create distinct singletons. Each singleton manages a specific
/// OpenCL device. However, it is possible for different singletons to manage
/// the same device.
///
/// The `ID` template parameter, apart from ensuring that different IDs create
/// distinct singletons, it can also provide additional information about
/// which
/// device CLManager shall choose by default through the singleton CLSetup
/// with
/// the same `ID` template argument.
///
/// It is important to configure the platform and device to be used through
/// CLSetup before calling CLManager::instance for the first time. If nothing
/// is done by the user, the default behavior is to use
/// `CL_DEVICE_TYPE_DEFAULT` type device, and set the platform to be the first
/// one that contain such as device, and the device to the first one that is
/// of
/// such a type. The user can change the platform name, device vendor name,
/// device name, and device type through CLSetup. In case of names, only
/// partial match is requried. For example,
/// ~~~{.cpp}
/// CLSetup<CLDefault> &setup = CLSetup<CLDefault>::instance();
/// setup.platform("Apple");
/// setup.device_vendor("Intel");
/// setup.device_type("GPU);
/// setup.device_name("Iris");
/// CLManager<CLDefault> &manager = CLManager<CLDefault>::instance();
/// ~~~
/// If compiled on a recent MacBook Pro (late 2013 model), then the Iris Pro
/// GPU from Intel will be used. Note that in this case, actually specify
/// ~~~{.cpp}
/// setup.device_type("GPU");
/// setup.device_name("Iris")
/// ~~~
/// or
/// ~~~{.cpp}
/// setup.device_type("GPU");
/// setup.device_vedor("Intel")
/// ~~~
/// is enough. However, if one specify
/// ~~~{.cpp}
/// setup.device_type("CPU");
/// setup.device_vedor("NVIDIA")
/// ~~~
/// Then the setup will fail, since there is no device with the specified
/// combinations. Also note that, specification such as
/// ~~~{.cpp}
/// setup.device_vendor("NVIDIA")
/// ~~~
/// may not be enough to lead to successful setup. The default device type
/// `CL_DEVICE_TYPE_DEFAULT` may not be GPU. To be safe, if one need to use
/// CLSetup, at least specify the device type. It can be set through values of
/// type `cl_device_type` or a string with values "GPU", "CPU", "Accelerator".
/// Other string values are silently ignored and the default is used.
///
/// Before using a CLManager, it is important to check that CLManager::setup
/// returns `true`.
template <typename ID = CLDefault>
class CLManager
{
    public:
    typedef ID cl_id;

    CLManager(const CLManager<ID> &) = delete;

    CLManager<ID> &operator=(const CLManager<ID> &) = delete;

    /// \brief Get an instance of the manager singleton
    static CLManager<ID> &instance()
    {
        static CLManager<ID> manager;

        return manager;
    }

    /// \brief The minimum OpenCL version supported by all devices in the
    /// context of this manager
    ///
    /// \sa CLQuery::opencl_version
    int opencl_version() const { return opencl_version_; }

    /// \brief The minimum OpenCL C version supported by all devices in the
    /// context of this manager
    ///
    /// \sa CLQuery::opencl_version
    int opencl_c_version() const { return opencl_c_version_; }

    /// \brief The platform currently being used
    const std::shared_ptr<CLPlatform> &platform() const { return platform_; }

    /// \brief The context currently being used
    const std::shared_ptr<CLContext> &context() const { return context_; }

    /// \brief The device currently being used
    const std::shared_ptr<CLDevice> &device() const { return device_; }

    /// \brief The vector of all device that is in the context of this manager
    const std::vector<std::shared_ptr<CLDevice>> &device_vec() const
    {
        return device_vec_;
    }

    /// \brief The command queue currently being used
    const std::shared_ptr<CLCommandQueue> &command_queue() const
    {
        return command_queue_;
    }

    /// \brief Whether the platform, context, device and command queue has
    /// been
    /// setup correctly
    bool setup() const { return setup_; }

    /// \brief Try to setup the platform, context, device and command queue
    /// using the given device type
    bool setup(::cl_device_type dev)
    {
        setup_ = false;
        setup_cl_manager(dev);

        return setup_;
    }

    /// \brief Set the platform, context, device and command queue manually
    ///
    /// \details
    /// After this member function call setup() will return `true` in future
    /// calls
    bool setup(::cl_platform_id plat, ::cl_context ctx, ::cl_device_id dev,
        ::cl_command_queue cmd)
    {
        setup_ = false;
        platform_ = make_cl_platform_ptr(plat);
        context_ = make_cl_context_ptr(ctx);
        device_ = make_cl_device_ptr(dev);
        command_queue_ = make_cl_command_queue_ptr(cmd);
        set_device_vec();
        check_opencl_version();

        setup_ = true;

        return setup_;
    }

    /// \brief Create an OpenCL buffer of a given type and number of elements
    template <typename CLType>
    std::shared_ptr<CLMemory> create_buffer(std::size_t num,
        ::cl_mem_flags flag = CL_MEM_READ_WRITE,
        void *host_ptr = nullptr) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(create_buffer);

        if (num == 0)
            return make_cl_memory_ptr(nullptr);

        ::cl_int status;
        ::cl_mem buffer = ::clCreateBuffer(
            context_.get(), flag, sizeof(CLType) * num, host_ptr, &status);
        internal::cl_error_check(
            status, "CLManager::create_buffer", "::clCreateBuffer");

        return make_cl_memory_ptr(buffer);
    }

    /// \brief Read an OpenCL buffer of a given type and number of elements
    /// into an iterator
    template <typename CLType, typename OutputIter>
    void read_buffer(::cl_mem buf, std::size_t num, OutputIter first,
        std::size_t offset = 0, ::cl_uint num_events_in_wait_list = 0,
        ::cl_event *event_wait_list = nullptr, ::cl_event *event = nullptr,
        bool block = true) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(read_buffer);
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(
            read_buffer, block, event);

        std::vector<CLType> buffer(num);
        ::cl_int status = ::clEnqueueReadBuffer(command_queue_.get(), buf,
            static_cast<::cl_bool>(block), sizeof(CLType) * offset,
            sizeof(CLType) * num, buffer.data(), num_events_in_wait_list,
            event_wait_list, event);
        internal::cl_error_check(
            status, "CLManager::read_buffer", "::clEnqueueReadBuffer");
        std::copy(buffer.begin(), buffer.end(), first);
    }

    /// \brief Read an OpenCL buffer of a given type and number of elements
    /// into a pointer
    template <typename CLType>
    void read_buffer(::cl_mem buf, std::size_t num, CLType *first,
        std::size_t offset = 0, ::cl_uint num_events_in_wait_list = 0,
        ::cl_event *event_wait_list = nullptr, ::cl_event *event = nullptr,
        bool block = true) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(read_buffer);
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(
            read_buffer, block, event);

        ::cl_int status = ::clEnqueueReadBuffer(command_queue_.get(), buf,
            static_cast<::cl_bool>(block), sizeof(CLType) * offset,
            sizeof(CLType) * num, first, num_events_in_wait_list,
            event_wait_list, event);
        internal::cl_error_check(
            status, "CLManager::read_buffer", "::clEnqueueReadBuffer");
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from an iterator
    template <typename CLType, typename InputIter>
    void write_buffer(::cl_mem buf, std::size_t num, InputIter first,
        std::size_t offset = 0, ::cl_uint num_events_in_wait_list = 0,
        ::cl_event *event_wait_list = nullptr, ::cl_event *event = nullptr,
        bool block = true) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(write_buffer);
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(
            write_buffer, block, event);

        std::vector<CLType> buffer(num);
        std::copy_n(first, num, buffer.data());
        ::cl_int status = ::clEnqueueWriteBuffer(command_queue_.get(), buf,
            static_cast<::cl_bool>(block), sizeof(CLType) * offset,
            sizeof(CLType) * num, buffer.data(), num_events_in_wait_list,
            event_wait_list, event);
        internal::cl_error_check(
            status, "CLManager::write_buffer", "::clEnqueueWriteBuffer");
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from a pointer
    template <typename CLType>
    void write_buffer(::cl_mem buf, std::size_t num, const CLType *first,
        std::size_t offset = 0, ::cl_uint num_events_in_wait_list = 0,
        ::cl_event *event_wait_list = nullptr, ::cl_event *event = nullptr,
        bool block = true) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(write_buffer);
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(
            write_buffer, block, event);

        ::cl_int status = ::clEnqueueWriteBuffer(command_queue_.get(), buf,
            static_cast<::cl_bool>(block), sizeof(CLType) * offset,
            sizeof(CLType) * num, const_cast<CLType *>(first),
            num_events_in_wait_list, event_wait_list, event);
        internal::cl_error_check(
            status, "CLManager::write_buffer", "::clEnqueueWriteBuffer");
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from a pointer
    template <typename CLType>
    void write_buffer(::cl_mem buf, std::size_t num, CLType *first,
        std::size_t offset = 0, ::cl_uint num_events_in_wait_list = 0,
        ::cl_event *event_wait_list = nullptr, ::cl_event *event = nullptr,
        bool block = true) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(write_buffer);
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(
            write_buffer, block, event);

        ::cl_int status = ::clEnqueueWriteBuffer(command_queue_.get(), buf,
            static_cast<::cl_bool>(block), sizeof(CLType) * offset,
            sizeof(CLType) * num, first, num_events_in_wait_list,
            event_wait_list, event);
        internal::cl_error_check(
            status, "CLManager::write_buffer", "::clEnqueueWriteBuffer");
    }

    /// \brief Copy an OpenCL buffer into another of a given type and number
    /// of
    /// elements
    template <typename CLType>
    void copy_buffer(::cl_mem src, ::cl_mem dst, std::size_t num,
        std::size_t src_offset = 0, std::size_t dst_offset = 0,
        ::cl_uint num_events_in_wait_list = 0,
        ::cl_event *event_wait_list = nullptr, ::cl_event *event = nullptr,
        bool block = true) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(copy_buffer);
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(
            copy_buffer, block, event);

        ::cl_int status = CL_SUCCESS;

        ::cl_event e = ::clCreateUserEvent(context_.get(), &status);
        internal::cl_error_check(
            status, "CLManager::copy_buffer", "::clCreateUserEvent");
        ::cl_event *eptr = event == nullptr ? &e : event;

        status = ::clEnqueueCopyBuffer(command_queue_.get(), src, dst,
            sizeof(CLType) * src_offset, sizeof(CLType) * dst_offset,
            sizeof(CLType) * num, num_events_in_wait_list, event_wait_list,
            eptr);
        internal::cl_error_check(
            status, "CLManager::copy_buffer", "::clEnqueueCopyBuffer");

        if (block) {
            status = ::clWaitForEvents(1, eptr);
            internal::cl_error_check(
                status, "CLManager::copy_buffer", "::clWaitForEvents");
            status = ::clReleaseEvent(e);
            internal::cl_error_check(
                status, "CLManager::copy_buffer", "::clReleaseEvent");
        }
    }

    /// \brief Run a given kernel with one dimensional global size and local
    /// size on the current command queue
    ///
    /// \details
    /// OpenCL requires that `global_size` is a multiple of `local_size`. This
    /// function will round `N` if it is not already a multiple of
    /// `local_size`. In the kernel it is important to check that
    /// `get_global_id(0)` is not out of range.
    ///
    /// For example, say we have kernel that should be applied to `N`
    /// elements.
    /// But the most efficient local size `K` does not divide `N`. Instead of
    /// calculate the correct global size yourself, you can simple call
    /// `run_kernel(kern, N, K)`. But within the kernel, you need to check
    /// `get_global_id(0) < N`
    void run_kernel(::cl_kernel kern, std::size_t N,
        std::size_t local_size = 0, ::cl_uint num_events_in_wait_list = 0,
        ::cl_event *event_wait_list = nullptr, ::cl_event *event = nullptr,
        bool block = true) const
    {
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(run_kernel, block, event);

        ::cl_int status = CL_SUCCESS;

        ::cl_event e = ::clCreateUserEvent(context_.get(), &status);
        internal::cl_error_check(
            status, "CLManager::copy_buffer", "::clCreateUserEvent");
        ::cl_event *eptr = event == nullptr ? &e : event;

        const std::size_t global_offset = 0;
        const std::size_t global_work_size =
            get_global_work_size(N, local_size);
        status = ::clEnqueueNDRangeKernel(command_queue_.get(), kern, 1,
            &global_offset, &global_work_size, &local_size,
            num_events_in_wait_list, event_wait_list, eptr);

        if (block) {
            status = ::clWaitForEvents(1, eptr);
            internal::cl_error_check(
                status, "CLManager::copy_buffer", "::clWaitForEvents");
            status = ::clReleaseEvent(e);
            internal::cl_error_check(
                status, "CLManager::copy_buffer", "::clReleaseEvent");
        }
    }

    /// \brief Create a program given a vector of sources within the current
    /// context
    std::shared_ptr<CLProgram> create_program(
        const std::vector<std::string> &source) const
    {
        std::vector<const char *> strings;
        std::vector<std::size_t> lengths;
        for (const auto &src : source) {
            strings.push_back(src.data());
            lengths.push_back(src.size());
        }

        ::cl_int status = CL_SUCCESS;
        ::cl_program ptr = ::clCreateProgramWithSource(context_.get(),
            static_cast<::cl_uint>(source.size()), strings.data(),
            lengths.data(), &status);
        internal::cl_error_check(status, "CLManager::create_program",
            "::clCreateProgramWithSource");

        return make_cl_program_ptr(ptr);
    }

    /// \brief Create a program given the source within the current context
    std::shared_ptr<CLProgram> create_program(const std::string &source) const
    {
        return create_program(std::vector<std::string>(1, source));
    }

    private:
    std::shared_ptr<CLPlatform> platform_;
    std::shared_ptr<CLContext> context_;
    std::shared_ptr<CLDevice> device_;
    std::shared_ptr<CLCommandQueue> command_queue_;
    std::vector<std::shared_ptr<CLDevice>> device_vec_;

    bool setup_;
    CLSetup<ID> &setup_default_;
    int opencl_version_;
    int opencl_c_version_;

    CLManager() : setup_(false), setup_default_(CLSetup<ID>::instance())
    {
        setup_cl_manager(setup_default_.device_type());
    }

    void check_opencl_version()
    {
        opencl_version_ = CLQuery::opencl_version(device_.get());
        opencl_c_version_ = CLQuery::opencl_c_version(device_.get());
        for (std::size_t i = 0; i != device_vec_.size(); ++i) {
            int ocl = CLQuery::opencl_version(device_vec_[i].get());
            int oclc = CLQuery::opencl_c_version(device_vec_[i].get());
            if (opencl_version_ > ocl)
                opencl_version_ = ocl;
            if (opencl_c_version_ > ocl)
                opencl_c_version_ = oclc;
        }
    }

    void setup_cl_manager(::cl_device_type dev_type)
    {
        setup_ = false;
        ::cl_int status = CL_SUCCESS;

        bool setup_platform = platform_filter(dev_type);
        if (!setup_platform) {
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_PLATFORM;
            return;
        }

        ::cl_uint num_dev = 0;
        status =
            ::clGetDeviceIDs(platform_.get(), dev_type, 0, nullptr, &num_dev);
        if (status != CL_SUCCESS || num_dev == 0) {
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_CONTEXT;
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_DEVICE;
            return;
        }

        std::vector<::cl_device_id> dev_pool_ptr(num_dev);
        status = ::clGetDeviceIDs(
            platform_.get(), dev_type, num_dev, dev_pool_ptr.data(), nullptr);
        if (status != CL_SUCCESS) {
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_CONTEXT;
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_DEVICE;
            return;
        }

        std::vector<std::shared_ptr<CLDevice>> dev_pool;
        for (auto ptr : dev_pool_ptr)
            dev_pool.push_back(make_cl_device_ptr(ptr));
        std::vector<std::shared_ptr<CLDevice>> dev_select;
        device_filter(dev_pool, dev_select);
        if (dev_select.size() == 0) {
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_CONTEXT;
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_DEVICE;
            return;
        }

        std::vector<::cl_device_id> dev_select_ptr;
        for (const auto &dev : dev_select)
            dev_select_ptr.push_back(dev.get());
        ::cl_context_properties context_properties[] = {CL_CONTEXT_PLATFORM,
            reinterpret_cast<::cl_context_properties>(platform_.get()), 0};
        ::cl_context ctx = ::clCreateContext(context_properties,
            static_cast<::cl_uint>(dev_select_ptr.size()),
            dev_select_ptr.data(), nullptr, nullptr, &status);
        if (status != CL_SUCCESS) {
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_CONTEXT;
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_DEVICE;
            return;
        }
        context_ = make_cl_context_ptr(ctx);

        bool setup_device = set_device_vec();
        if (!setup_device) {
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_DEVICE;
            return;
        }
        device_ = device_vec_.front();

        ::cl_command_queue cmd =
            ::clCreateCommandQueue(context_.get(), device_.get(), 0, &status);
        if (status != CL_SUCCESS) {
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_COMMAND_QUEUE;
            return;
        }
        command_queue_ = make_cl_command_queue_ptr(cmd);

        check_opencl_version();

        setup_ = true;
    }

    bool platform_filter(::cl_device_type dev_type)
    {
        ::cl_int status = CL_SUCCESS;

        ::cl_uint num_plat = 0;
        status = ::clGetPlatformIDs(0, nullptr, &num_plat);
        if (status != CL_SUCCESS)
            return false;

        std::vector<::cl_platform_id> plat_vec_ptr(num_plat);
        status = ::clGetPlatformIDs(num_plat, plat_vec_ptr.data(), nullptr);
        if (status != CL_SUCCESS)
            return false;

        std::vector<std::shared_ptr<CLPlatform>> plat_vec;
        for (auto ptr : plat_vec_ptr)
            plat_vec.push_back(make_cl_platform_ptr(ptr));

        // If not using default platform
        if (!setup_default_.default_platform()) {
            for (const auto &plat : plat_vec) {
                std::size_t name_size = 0;
                status = ::clGetPlatformInfo(
                    plat.get(), CL_PLATFORM_NAME, 0, nullptr, &name_size);
                if (status != CL_SUCCESS)
                    continue;
                std::vector<char> name_vec(name_size);
                status = ::clGetPlatformInfo(plat.get(), CL_PLATFORM_NAME,
                    name_size, name_vec.data(), nullptr);
                if (status != CL_SUCCESS)
                    continue;
                name_vec.push_back(0);
                std::string name(static_cast<const char *>(name_vec.data()));
                if (setup_default_.check_platform(name)) {
                    platform_ = plat;
                    return true;
                }
            }

            return false;
        }

        // Using default platform: finding the first that has the device type
        for (const auto &plat : plat_vec) {
            ::cl_uint num_dev = 0;
            status = ::clGetDeviceIDs(
                platform_.get(), dev_type, 0, nullptr, &num_dev);
            if (status != CL_SUCCESS)
                continue;
            std::vector<::cl_device_id> dev_pool_ptr(num_dev);
            status = ::clGetDeviceIDs(platform_.get(), dev_type, num_dev,
                dev_pool_ptr.data(), nullptr);
            if (status != CL_SUCCESS)
                continue;
            std::vector<std::shared_ptr<CLDevice>> dev_pool;
            for (auto ptr : dev_pool_ptr)
                dev_pool.push_back(make_cl_device_ptr(ptr));
            std::vector<std::shared_ptr<CLDevice>> dev_select;
            device_filter(dev_pool, dev_select);
            if (dev_select.size() != 0) {
                platform_ = plat;
                return true;
            }
        }

        return false;
    }

    void device_filter(const std::vector<std::shared_ptr<CLDevice>> &dev_pool,
        std::vector<std::shared_ptr<CLDevice>> &dev_select)
    {
        ::cl_int status = CL_SUCCESS;
        std::vector<bool> dev_select_idx(dev_pool.size(), true);

        // Not using the default device vendor
        if (!setup_default_.default_device_vendor()) {
            for (std::size_t d = 0; d != dev_pool.size(); ++d) {
                std::size_t name_size = 0;
                status = ::clGetDeviceInfo(dev_pool[d].get(), CL_DEVICE_VENDOR,
                    0, nullptr, &name_size);
                if (status != CL_SUCCESS) {
                    dev_select_idx[d] = false;
                    continue;
                }
                std::vector<char> name_vec(name_size);
                status = ::clGetDeviceInfo(dev_pool[d].get(), CL_DEVICE_VENDOR,
                    name_size, name_vec.data(), nullptr);
                if (status != CL_SUCCESS) {
                    dev_select_idx[d] = false;
                    continue;
                }
                name_vec.push_back(0);
                std::string name(static_cast<const char *>(name_vec.data()));
                if (!setup_default_.check_device_vendor(name))
                    dev_select_idx[d] = false;
            }
        }

        // Not using the default device
        if (!setup_default_.default_device()) {
            for (std::size_t d = 0; d != dev_pool.size(); ++d) {
                std::size_t name_size = 0;
                status = ::clGetDeviceInfo(
                    dev_pool[d].get(), CL_DEVICE_NAME, 0, nullptr, &name_size);
                if (status != CL_SUCCESS) {
                    dev_select_idx[d] = false;
                    continue;
                }
                std::vector<char> name_vec(name_size);
                status = ::clGetDeviceInfo(dev_pool[d].get(), CL_DEVICE_NAME,
                    name_size, name_vec.data(), nullptr);
                if (status != CL_SUCCESS) {
                    dev_select_idx[d] = false;
                    continue;
                }
                name_vec.push_back(0);
                std::string name(static_cast<const char *>(name_vec.data()));
                if (!setup_default_.check_device(name))
                    dev_select_idx[d] = false;
            }
        }

        for (std::size_t d = 0; d != dev_pool.size(); ++d)
            if (dev_select_idx[d])
                dev_select.push_back(dev_pool[d]);
    }

    bool set_device_vec()
    {
        ::cl_uint num_dev = 0;
        ::cl_int status = CL_SUCCESS;
        status = clGetContextInfo(context_.get(), CL_CONTEXT_NUM_DEVICES,
            sizeof(::cl_uint), &num_dev, nullptr);
        if (status != CL_SUCCESS)
            return false;

        std::vector<::cl_device_id> dev_vec(num_dev);
        status = clGetContextInfo(context_.get(), CL_CONTEXT_DEVICES,
            sizeof(::cl_device_id) * num_dev, dev_vec.data(), nullptr);
        if (status != CL_SUCCESS)
            return false;

        device_vec_.clear();
        for (auto ptr : dev_vec)
            device_vec_.push_back(make_cl_device_ptr(ptr));

        return true;
    }

    std::size_t get_global_work_size(
        std::size_t N, std::size_t local_size) const
    {
        if (local_size == 0)
            return N;

        if (N % local_size == 0)
            return N;

        return (N / local_size + 1) * local_size;
    }
}; // clss CLManager

} // namespace vsmc

#endif // VSMC_OPENCL_CL_MANAGER_HPP

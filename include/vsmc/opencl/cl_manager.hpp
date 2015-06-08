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
#include <vsmc/opencl/cl_manip.hpp>
#include <vsmc/opencl/cl_setup.hpp>
#include <vsmc/opencl/cl_type.hpp>

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
    using cl_id = ID;

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
    int opencl_version() const { return opencl_version_; }

    /// \brief The minimum OpenCL C version supported by all devices in the
    /// context of this manager
    int opencl_c_version() const { return opencl_c_version_; }

    /// \brief The platform currently being used
    const CLPlatform &platform() const { return platform_; }

    /// \brief The context currently being used
    const CLContext &context() const { return context_; }

    /// \brief The device currently being used
    const CLDevice &device() const { return device_; }

    /// \brief The vector of all device that is in the context of this manager
    const std::vector<CLDevice> &device_vec() const { return device_vec_; }

    /// \brief The command queue currently being used
    const CLCommandQueue &command_queue() const { return command_queue_; }

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
    bool setup(const CLPlatform &plat, const CLContext &ctx,
        const CLDevice &dev, const CLCommandQueue &cmd)
    {
        setup_ = false;
        platform_ = plat;
        context_ = ctx;
        device_ = dev;
        device_vec_ = context_.get_device();
        command_queue_ = cmd;
        check_opencl_version();

        setup_ = true;

        return setup_;
    }

    /// \brief Create an OpenCL buffer of a given type and number of elements
    template <typename CLType>
    CLMemory create_buffer(std::size_t num,
        ::cl_mem_flags flags = CL_MEM_READ_WRITE,
        void *host_ptr = nullptr) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(create_buffer);

        if (num == 0)
            return CLMemory(nullptr);

        return CLMemory(context_, flags, sizeof(CLType) * num, host_ptr);
    }

    /// \brief Read an OpenCL buffer of a given type and number of elements
    /// into an iterator
    template <typename CLType, typename OutputIter>
    ::cl_int read_buffer(const CLMemory &buf, std::size_t num,
        OutputIter first, std::size_t offset = 0,
        const std::vector<CLEvent> &event_wait_list =
            std::vector<CLEvent>()) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(read_buffer);

        CLEvent event;
        std::vector<CLType> buffer(num);
        ::cl_int status = command_queue_.enqueue_read_buffer(buf, CL_TRUE,
            sizeof(CLType) * offset, sizeof(CLType) * num, buffer.data(),
            event_wait_list, event);
        if (status == CL_SUCCESS)
            std::copy(buffer.begin(), buffer.end(), first);

        return status;
    }

    /// \brief Read an OpenCL buffer of a given type and number of elements
    /// into a pointer
    template <typename CLType>
    ::cl_int read_buffer(const CLMemory &buf, std::size_t num, CLType *first,
        std::size_t offset = 0, const std::vector<CLEvent> &event_wait_list =
                                    std::vector<CLEvent>()) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(read_buffer);

        CLEvent event;

        return command_queue_.enqueue_read_buffer(buf, CL_TRUE,
            sizeof(CLType) * offset, sizeof(CLType) * num, first,
            event_wait_list, event);
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from an iterator
    template <typename CLType, typename InputIter>
    ::cl_int write_buffer(const CLMemory &buf, std::size_t num,
        InputIter first, std::size_t offset = 0,
        const std::vector<CLEvent> &event_wait_list =
            std::vector<CLEvent>()) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(write_buffer);

        CLEvent event;
        std::vector<CLType> buffer(num);
        std::copy_n(first, num, buffer.data());

        return command_queue_.enqueue_write_buffer(buf, CL_TRUE,
            sizeof(CLType) * offset, sizeof(CLType) * num, buffer.data(),
            event_wait_list, event);
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from a pointer
    template <typename CLType>
    ::cl_int write_buffer(const CLMemory &buf, std::size_t num,
        const CLType *first, std::size_t offset = 0,
        const std::vector<CLEvent> &event_wait_list =
            std::vector<CLEvent>()) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(write_buffer);

        CLEvent event;

        return command_queue_.enqueue_write_buffer(buf, CL_TRUE,
            sizeof(CLType) * offset, sizeof(CLType) * num,
            const_cast<CLType *>(first), event_wait_list, event);
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from a pointer
    template <typename CLType>
    ::cl_int write_buffer(const CLMemory &buf, std::size_t num, CLType *first,
        std::size_t offset = 0, const std::vector<CLEvent> &event_wait_list =
                                    std::vector<CLEvent>()) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(write_buffer);

        CLEvent event;

        return command_queue_.enqueue_write_buffer(buf, CL_TRUE,
            sizeof(CLType) * offset, sizeof(CLType) * num, first,
            event_wait_list, event);
    }

    /// \brief Copy an OpenCL buffer into another of a given type and number
    /// of
    /// elements
    template <typename CLType>
    ::cl_int copy_buffer(const CLMemory &src, const CLMemory &dst,
        std::size_t num, std::size_t src_offset = 0,
        std::size_t dst_offset = 0,
        const std::vector<CLEvent> &event_wait_list =
            std::vector<CLEvent>()) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(copy_buffer);

        CLEvent event;
        ::cl_int status = command_queue_.enqueue_copy_buffer(src, dst,
            sizeof(CLType) * src_offset, sizeof(CLType) * dst_offset,
            sizeof(CLType) * num, event_wait_list, event);
        if (status == CL_SUCCESS)
            return event.wait();

        return status;
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
    ///
    /// One can specify `local_size` to zero such that vSMC will try to select
    /// the optimal local and global size automatically.
    ::cl_int run_kernel(const CLKernel &kern, std::size_t N,
        std::size_t local_size = 0,
        const std::vector<CLEvent> event_wait_list =
            std::vector<CLEvent>()) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(run_kernel);

        std::size_t gsize = 0;
        std::size_t lsize = local_size;
        if (local_size == 0)
            cl_preferred_work_size(N, kern, device_, gsize, lsize);
        else
            gsize = cl_min_global_size(N, local_size);

        CLEvent event;
        ::cl_int status =
            command_queue_.enqueue_nd_range_kernel(kern, 1, CLNDRange(),
                CLNDRange(gsize), CLNDRange(lsize), event_wait_list, event);
        if (status == CL_SUCCESS)
            return event.wait();

        return status;
    }

    /// \brief Create a program given a vector of sources within the current
    /// context
    CLProgram create_program(const std::vector<std::string> &sources) const
    {
        return CLProgram(context_, sources);
    }

    /// \brief Create a program given the source within the current context
    CLProgram create_program(const std::string &source) const
    {
        return CLProgram(context_, source);
    }

    private:
    CLPlatform platform_;
    CLContext context_;
    CLDevice device_;
    CLCommandQueue command_queue_;
    std::vector<CLDevice> device_vec_;

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
        opencl_version_ = internal::cl_opencl_version(device_.get());
        opencl_c_version_ = internal::cl_opencl_c_version(device_.get());
        for (std::size_t i = 0; i != device_vec_.size(); ++i) {
            int ocl = internal::cl_opencl_version(device_vec_[i].get());
            int oclc = internal::cl_opencl_c_version(device_vec_[i].get());
            if (opencl_version_ > ocl)
                opencl_version_ = ocl;
            if (opencl_c_version_ > ocl)
                opencl_c_version_ = oclc;
        }
    }

    void setup_cl_manager(::cl_device_type dev_type)
    {
        setup_ = false;

        if (!platform_filter(dev_type)) {
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_PLATFORM;
            return;
        }

        std::vector<CLDevice> dev_pool(platform_.get_device(dev_type));
        std::vector<CLDevice> dev_select(device_filter(dev_pool));
        if (dev_select.size() == 0) {
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_CONTEXT;
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_DEVICE;
            return;
        }

        ::cl_context_properties properties[] = {CL_CONTEXT_PLATFORM,
            reinterpret_cast<::cl_context_properties>(platform_.get()), 0};
        context_ = CLContext(properties, dev_select);
        if (!bool(context_)) {
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_CONTEXT;
            return;
        }

        device_vec_ = context_.get_device();
        if (device_vec_.empty()) {
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_DEVICE;
            return;
        }
        device_ = device_vec_.front();

        command_queue_ = CLCommandQueue(context_, device_, 0);
        if (!bool(command_queue_)) {
            VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_COMMAND_QUEUE;
            return;
        }

        check_opencl_version();

        setup_ = true;
    }

    bool platform_filter(::cl_device_type dev_type)
    {
        std::vector<CLPlatform> plat_vec(CLPlatform::platforms());
        if (plat_vec.empty())
            return false;

        // If not using default platform
        if (!setup_default_.default_platform()) {
            for (const auto &plat : plat_vec) {
                std::string name;
                plat.get_info(CL_PLATFORM_NAME, name);
                if (setup_default_.check_platform(name)) {
                    platform_ = plat;
                    return true;
                }
            }

            return false;
        }

        // Using default platform: finding the first that has the device type
        for (const auto &plat : plat_vec) {
            std::vector<CLDevice> dev_pool(plat.get_device(dev_type));
            std::vector<CLDevice> dev_select(device_filter(dev_pool));
            if (!dev_select.empty()) {
                platform_ = plat;
                return true;
            }
        }

        return false;
    }

    std::vector<CLDevice> device_filter(const std::vector<CLDevice> &dev_pool)
    {
        std::vector<bool> dev_select_idx(dev_pool.size(), true);
        std::vector<bool> dev_select_idx_igpu(dev_pool.size(), true);
        std::string name;
        ::cl_device_type type;

        // Filter out Intel GPU
        for (std::size_t d = 0; d != dev_pool.size(); ++d) {
            dev_pool[d].get_info(CL_DEVICE_TYPE, type);
            dev_pool[d].get_info(CL_DEVICE_VENDOR, name);
            if ((type & CL_DEVICE_TYPE_GPU) != 0 &&
                name.find("Intel") != std::string::npos) {
                dev_select_idx_igpu[d] = false;
            }
        }

        // Not using the default device vendor
        if (!setup_default_.default_device_vendor()) {
            for (std::size_t d = 0; d != dev_pool.size(); ++d) {
                dev_pool[d].get_info(CL_DEVICE_VENDOR, name);
                if (!setup_default_.check_device_vendor(name))
                    dev_select_idx[d] = false;
            }
        }

        // Not using the default device
        if (!setup_default_.default_device()) {
            for (std::size_t d = 0; d != dev_pool.size(); ++d) {
                dev_pool[d].get_info(CL_DEVICE_NAME, name);
                if (!setup_default_.check_device(name))
                    dev_select_idx[d] = false;
            }
        }

        // determine if we should remove intel GPU
        std::size_t num = 0;
        for (std::size_t d = 0; d != dev_pool.size(); ++d)
            if (dev_select_idx[d] && dev_select_idx_igpu[d])
                ++num;
        if (num > 0) {
            for (std::size_t d = 0; d != dev_pool.size(); ++d)
                if (!dev_select_idx_igpu[d])
                    dev_select_idx[d] = false;
        }

        std::vector<CLDevice> dev_select;
        for (std::size_t d = 0; d != dev_pool.size(); ++d)
            if (dev_select_idx[d])
                dev_select.push_back(dev_pool[d]);

        return dev_select;
    }
}; // clss CLManager

} // namespace vsmc

#endif // VSMC_OPENCL_CL_MANAGER_HPP

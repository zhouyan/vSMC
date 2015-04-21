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

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/cl_setup.hpp>
#include <vsmc/opencl/cl_manip.hpp>
#include <vsmc/opencl/cl_query.hpp>
#include <vsmc/opencl/internal/cl_wrapper.hpp>
#include <vsmc/utility/stop_watch.hpp>

#define VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(func)                    \
    VSMC_RUNTIME_ASSERT((setup()),                                           \
        ("**CLManager::" #func "** CAN ONLY BE CALLED AFTER TRUE "           \
         "**CLManager::setup**"));

#define VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_PLATFORM                \
    VSMC_RUNTIME_WARNING(setup_platform,                                     \
        ("**CLManager::setup** FAILED TO SETUP A PLATFORM"));

#define VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_CONTEXT                 \
    VSMC_RUNTIME_WARNING(                                                    \
        setup_context, ("**CLManager::setup** FAILED TO SETUP A CONTEXT"));

#define VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_DEVICE                  \
    VSMC_RUNTIME_WARNING(                                                    \
        setup_device, ("**CLManager::setup** FAILED TO SETUP A DEVICE"));

#define VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_COMMAND_QUEUE           \
    VSMC_RUNTIME_WARNING(setup_command_queue,                                \
        ("**CLManager::setup** FAILED TO SETUP A COMMAND_QUEUE"));

#define VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(func, block, event)     \
    VSMC_RUNTIME_WARNING((block || event != nullptr),                        \
        ("**CLManager::" #func " NOT BLOCKING BUT WITH NULL EVENT"))

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
    const ::cl::Platform &platform() const { return platform_; }

    /// \brief The context currently being used
    const ::cl::Context &context() const { return context_; }

    /// \brief The device currently being used
    const ::cl::Device &device() const { return device_; }

    /// \brief The vector of all device that is in the context of this manager
    const std::vector<::cl::Device> &device_vec() const
    {
        return device_vec_;
    }

    /// \brief The command queue currently being used
    const ::cl::CommandQueue &command_queue() const { return command_queue_; }

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
    bool setup(const ::cl::Platform &plat, const ::cl::Context &ctx,
        const ::cl::Device &dev, const ::cl::CommandQueue &cmd)
    {
        setup_ = false;
        platform_ = plat;
        context_ = ctx;
        device_ = dev;
        device_vec_ = context_.getInfo<CL_CONTEXT_DEVICES>();
        command_queue_ = cmd;
        check_opencl_version();
        setup_ = true;

        return setup_;
    }

    /// \brief Create an OpenCL buffer of a given type and number of elements
    template <typename CLType>
    ::cl::Buffer create_buffer(std::size_t num,
        ::cl_mem_flags flag = CL_MEM_READ_WRITE,
        void *host_ptr = nullptr) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(create_buffer);

        if (num == 0)
            return ::cl::Buffer();

        return ::cl::Buffer(context_, flag, sizeof(CLType) * num, host_ptr);
    }

    /// \brief Read an OpenCL buffer of a given type and number of elements
    /// into an iterator
    template <typename CLType, typename OutputIter>
    void read_buffer(const ::cl::Buffer &buf, std::size_t num,
        OutputIter first, std::size_t offset = 0,
        const std::vector<::cl::Event> *events = nullptr,
        ::cl::Event *event = nullptr, bool block = true) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(read_buffer);
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(
            read_buffer, block, event);

        std::vector<CLType> buffer(num);
        command_queue_.enqueueReadBuffer(buf, static_cast<::cl_bool>(block),
            sizeof(CLType) * offset, sizeof(CLType) * num, buffer.data(),
            events, event);
        std::copy(buffer.begin(), buffer.end(), first);
    }

    /// \brief Read an OpenCL buffer of a given type and number of elements
    /// into a pointer
    template <typename CLType>
    void read_buffer(const ::cl::Buffer &buf, std::size_t num, CLType *first,
        std::size_t offset = 0,
        const std::vector<::cl::Event> *events = nullptr,
        ::cl::Event *event = nullptr, bool block = true) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(read_buffer);
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(
            read_buffer, block, event);

        command_queue_.enqueueReadBuffer(buf, static_cast<::cl_bool>(block),
            sizeof(CLType) * offset, sizeof(CLType) * num, first, events,
            event);
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from an iterator
    template <typename CLType, typename InputIter>
    void write_buffer(const ::cl::Buffer &buf, std::size_t num,
        InputIter first, std::size_t offset = 0,
        const std::vector<::cl::Event> *events = nullptr,
        ::cl::Event *event = nullptr, bool block = true) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(write_buffer);
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(
            write_buffer, block, event);

        std::vector<CLType> buffer(num);
        std::copy_n(first, num, buffer.data());
        command_queue_.enqueueWriteBuffer(buf, static_cast<::cl_bool>(block),
            sizeof(CLType) * offset, sizeof(CLType) * num, buffer.data(),
            events, event);
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from a pointer
    template <typename CLType>
    void write_buffer(const ::cl::Buffer &buf, std::size_t num,
        const CLType *first, std::size_t offset = 0,
        const std::vector<::cl::Event> *events = nullptr,
        ::cl::Event *event = nullptr, bool block = true) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(write_buffer);
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(
            write_buffer, block, event);

        command_queue_.enqueueWriteBuffer(buf, static_cast<::cl_bool>(block),
            sizeof(CLType) * offset, sizeof(CLType) * num,
            const_cast<CLType *>(first), events, event);
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from a pointer
    template <typename CLType>
    void write_buffer(const ::cl::Buffer &buf, std::size_t num, CLType *first,
        std::size_t offset = 0,
        const std::vector<::cl::Event> *events = nullptr,
        ::cl::Event *event = nullptr, bool block = true) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(write_buffer);
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(
            write_buffer, block, event);

        command_queue_.enqueueWriteBuffer(buf, static_cast<::cl_bool>(block),
            sizeof(CLType) * offset, sizeof(CLType) * num, first, events,
            event);
    }

    /// \brief Copy an OpenCL buffer into another of a given type and number
    /// of
    /// elements
    template <typename CLType>
    void copy_buffer(const ::cl::Buffer &src, const ::cl::Buffer &dst,
        std::size_t num, std::size_t src_offset = 0,
        std::size_t dst_offset = 0,
        const std::vector<::cl::Event> *events = nullptr,
        ::cl::Event *event = nullptr, bool block = true) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_CL_MANAGER_SETUP(copy_buffer);
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(
            copy_buffer, block, event);

        ::cl::Event e;
        ::cl::Event *eptr = event == nullptr ? &e : event;
        command_queue_.enqueueCopyBuffer(src, dst,
            sizeof(CLType) * src_offset, sizeof(CLType) * dst_offset,
            sizeof(CLType) * num, events, eptr);
        if (block)
            eptr->wait();
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
    void run_kernel(const ::cl::Kernel &kern, std::size_t N,
        std::size_t local_size = 0,
        const std::vector<::cl::Event> *events = nullptr,
        ::cl::Event *event = nullptr, bool block = true) const
    {
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_BLOCK(
            run_kernel, block, event);

        ::cl::Event e;
        ::cl::Event *eptr = event == nullptr ? &e : event;
        command_queue_.enqueueNDRangeKernel(kern, ::cl::NullRange,
            get_global_nd_range(N, local_size),
            get_local_nd_range(local_size), events, eptr);
        if (block)
            eptr->wait();
    }

    /// \brief Run the kernel with all local size that are multiples of the
    /// preferred factor, return the local size that is the fatest.
    ///
    /// \param kern The kernel to be profiled
    /// \param N The global size
    /// \param func A functor that has the following signature,
    /// ~~~{.cpp}
    /// void func (::cl::Kernel &kern)
    /// ~~~
    /// It will be applied to `kern` before it is run each time
    /// \param lmin The minimum local size to be considered. This function
    /// will
    /// consider all local sizes that are a multiple of this value. If `lmin =
    /// 0` (the default), then the preferred multiplier queried from the
    /// device
    /// is used. If its value is bigger than the allowed maximum local size,
    /// then it is treated as if it is set to zero.
    /// \param repeat The number of repeatition of runs. The profiling is done
    /// by run the kernel once to heat it up, and then repeat runs for this
    /// given value. The time of the later is measured and compared for each
    /// considered local size.
    ///
    /// \note This function relies on StopWatch to work correctly.
    template <typename Func>
    typename std::enable_if<!std::is_same<Func, std::size_t>::value &&
            !std::is_convertible<Func, std::size_t>::value,
        std::size_t>::type
        profile_kernel(::cl::Kernel &kern, std::size_t N, const Func &func,
            std::size_t lmin = 0, std::size_t repeat = 10)
    {
        cl::size_t<3> reqd_size;
        try {
            kern.getWorkGroupInfo(
                device_, CL_KERNEL_COMPILE_WORK_GROUP_SIZE, &reqd_size);
        } catch (const ::cl::Error &) {
            reqd_size[0] = 0;
        }

        if (reqd_size[0] != 0)
            return reqd_size[0];

        std::size_t factor;
        std::size_t lmax;
        std::size_t mmax;
        cl_minmax_local_size(kern, device_, factor, lmax, mmax);
        if (lmax == 0)
            return 0;

        if (lmin != 0 && lmin <= lmax) {
            factor = lmin;
            mmax = lmax / factor;
            lmax = mmax * factor;
        }

        if (mmax < 2)
            return lmax;

        double time = std::numeric_limits<double>::max VSMC_MNE();
        std::size_t lsize = lmax;
        vsmc::StopWatch watch;
        Func f(func);
        for (std::size_t m = mmax; m >= 1; --m) {
            std::size_t l = m * factor;
            f(kern);
            run_kernel(kern, N, l);
            watch.reset();
            watch.start();
            for (std::size_t r = 0; r != repeat; ++r) {
                f(kern);
                run_kernel(kern, N, l);
            }
            watch.stop();
            if (time > watch.milliseconds()) {
                time = watch.milliseconds();
                lsize = l;
            }
        }

        return lsize;
    }

    std::size_t profile_kernel(::cl::Kernel &kern, std::size_t N,
        std::size_t lmin = 0, std::size_t repeat = 3)
    {
        return profile_kernel(kern, N, profile_kernel_func_(), lmin, repeat);
    }

    /// \brief Create a program given the source within the current context
    ::cl::Program create_program(const std::string &source) const
    {
        return ::cl::Program(context_, source);
    }

    /// \brief Create a program given a vector of sources within the current
    /// context
    ::cl::Program create_program(const std::vector<std::string> &source) const
    {
        std::vector<std::pair<const char *, std::size_t>> src(source.size());
        for (std::size_t i = 0; i != source.size(); ++i)
            src[i] = std::make_pair(source[i].data(), source[i].size());

        return ::cl::Program(context_, src);
    }

    /// \brief Create a program given binaries within the current context
    ///
    /// \param binary A vector of binaries. The binary buffers are stored in
    /// `std::string`
    /// \param devices The devices for which the program shall be created. If
    /// it is `NULL`, then the program will be created for all devices in the
    /// current context
    /// \param status Return the status of loading the binaries for each
    /// device. It is ignored if `NULL`.
    ::cl::Program create_program(const std::vector<std::string> &binary,
        const std::vector<::cl::Device> *devices,
        std::vector<::cl_int> *status = nullptr) const
    {
        std::vector<std::pair<const void *, std::size_t>> bin(binary.size());
        for (std::size_t i = 0; i != binary.size(); ++i) {
            bin[i] =
                std::make_pair(static_cast<const void *>(binary[i].data()),
                    binary[i].size());
        }

        return devices == nullptr ?
            ::cl::Program(context_, device_vec_, bin, status) :
            ::cl::Program(context_, *devices, bin, status);
    }

    private:
    struct profile_kernel_func_ {
        void operator()(::cl::Kernel &) const {}
    };

    ::cl::Platform platform_;
    ::cl::Context context_;
    ::cl::Device device_;
    std::vector<::cl::Device> device_vec_;
    ::cl::CommandQueue command_queue_;

    bool setup_;
    CLSetup<ID> &setup_default_;
    int opencl_version_;
    int opencl_c_version_;

    CLManager() : setup_(false), setup_default_(CLSetup<ID>::instance())
    {
        setup_cl_manager(setup_default_.device_type());
    }

    CLManager(const CLManager<ID> &);

    CLManager<ID> &operator=(const CLManager<ID> &);

    void check_opencl_version()
    {
        opencl_version_ = CLQuery::opencl_version(device_);
        opencl_c_version_ = CLQuery::opencl_c_version(device_);
        for (std::size_t i = 0; i != device_vec_.size(); ++i) {
            int ocl = CLQuery::opencl_version(device_vec_[i]);
            int oclc = CLQuery::opencl_c_version(device_vec_[i]);
            if (opencl_version_ > ocl)
                opencl_version_ = ocl;
            if (opencl_c_version_ > ocl)
                opencl_c_version_ = oclc;
        }
    }

    void setup_cl_manager(::cl_device_type dev_type)
    {
        setup_ = false;

        bool setup_platform = platform_filter(dev_type);
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_PLATFORM;
        if (!setup_platform)
            return;

        bool setup_context = false;
        bool setup_device = false;
        try {
            std::vector<::cl::Device> dev_pool;
            std::vector<::cl::Device> dev_select;
            platform_.getDevices(dev_type, &dev_pool);
            device_filter(dev_pool, dev_select);
            if (dev_select.size() != 0) {
                ::cl_context_properties context_properties[] = {
                    CL_CONTEXT_PLATFORM,
                    reinterpret_cast<::cl_context_properties>(platform_()),
                    0};
                context_ = ::cl::Context(dev_select, context_properties);
                setup_context = true;
                device_vec_ = context_.getInfo<CL_CONTEXT_DEVICES>();
                device_ = device_vec_[0];
                setup_device = true;
            }
        } catch (const ::cl::Error &) {
        }
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_CONTEXT;
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_DEVICE;
        if (!setup_context)
            return;
        if (!setup_device)
            return;

        bool setup_command_queue = false;
        try {
            command_queue_ = ::cl::CommandQueue(context_, device_, 0);
            setup_command_queue = true;
        } catch (const ::cl::Error &) {
        }
        VSMC_RUNTIME_WARNING_OPENCL_CL_MANAGER_SETUP_COMMAND_QUEUE;
        if (!setup_command_queue)
            return;

        check_opencl_version();

        setup_ = true;
    }

    bool platform_filter(::cl_device_type dev_type)
    {
        std::vector<::cl::Platform> platform_vec;
        try {
            ::cl::Platform::get(&platform_vec);
        } catch (const ::cl::Error &) {
            platform_vec.clear();
        }
        if (platform_vec.size() == 0)
            return false;

        // If not using default platform
        if (!setup_default_.default_platform()) {
            for (std::size_t p = 0; p != platform_vec.size(); ++p) {
                try {
                    std::string name;
                    platform_vec[p].getInfo(CL_PLATFORM_NAME, &name);
                    if (setup_default_.check_platform(name)) {
                        platform_ = platform_vec[p];
                        return true;
                    }
                } catch (const ::cl::Error &) {
                }
            }

            return false;
        }

        // Using default platform: finding the first that has the device type
        for (std::size_t p = 0; p != platform_vec.size(); ++p) {
            try {
                std::vector<::cl::Device> dev_pool;
                std::vector<::cl::Device> dev_select;
                platform_vec[p].getDevices(dev_type, &dev_pool);
                device_filter(dev_pool, dev_select);
                if (dev_select.size() != 0) {
                    platform_ = platform_vec[p];
                    return true;
                }
            } catch (const ::cl::Error &) {
            }
        }

        return false;
    }

    void device_filter(const std::vector<::cl::Device> &dev_pool,
        std::vector<::cl::Device> &dev_select)
    {
        std::vector<bool> dev_select_idx(dev_pool.size(), true);

        // Not using the default device vendor
        if (!setup_default_.default_device_vendor()) {
            for (std::size_t d = 0; d != dev_pool.size(); ++d) {
                try {
                    std::string str;
                    dev_pool[d].getInfo(CL_DEVICE_VENDOR, &str);
                    if (!setup_default_.check_device_vendor(str))
                        dev_select_idx[d] = false;
                } catch (const ::cl::Error &) {
                    dev_select_idx[d] = false;
                }
            }
        }

        // Not using the default device
        if (!setup_default_.default_device()) {
            for (std::size_t d = 0; d != dev_pool.size(); ++d) {
                try {
                    std::string str;
                    dev_pool[d].getInfo(CL_DEVICE_NAME, &str);
                    if (!setup_default_.check_device(str))
                        dev_select_idx[d] = false;
                } catch (const ::cl::Error &) {
                    dev_select_idx[d] = false;
                }
            }
        }

        for (std::size_t d = 0; d != dev_pool.size(); ++d) {
            if (dev_select_idx[d]) {
                try {
                    dev_select.push_back(dev_pool[d]);
                } catch (const ::cl::Error &) {
                }
            }
        }
    }

    ::cl::NDRange get_global_nd_range(
        std::size_t N, std::size_t local_size) const
    {
        if (local_size == 0)
            return ::cl::NDRange(N);

        if (N % local_size == 0)
            return ::cl::NDRange(N);

        return ::cl::NDRange((N / local_size + 1) * local_size);
    }

    ::cl::NDRange get_local_nd_range(std::size_t local_size) const
    {
        return local_size == 0 ? ::cl::NullRange : ::cl::NDRange(local_size);
    }
}; // clss CLManager

} // namespace vsmc

#endif // VSMC_OPENCL_CL_MANAGER_HPP

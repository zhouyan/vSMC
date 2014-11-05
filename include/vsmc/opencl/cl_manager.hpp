//============================================================================
// include/vsmc/opencl/cl_manager.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distributed under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_OPENCL_CL_MANAGER_HPP
#define VSMC_OPENCL_CL_MANAGER_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/cl_setup.hpp>

#define VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(func) \
    VSMC_RUNTIME_ASSERT((setup()),                                           \
            ("**CLManager::"#func"** CAN ONLY BE CALLED AFTER TRUE "         \
             "**CLManager::setup**"));

#define VSMC_RUNTIME_WARNING_CL_MANAGER_SETUP_PLATFORM \
    VSMC_RUNTIME_WARNING(setup_platform,                                     \
            ("**CLManager::setup** FAILED TO SETUP A PLATFORM"));

#define VSMC_RUNTIME_WARNING_CL_MANAGER_SETUP_CONTEXT \
    VSMC_RUNTIME_WARNING(setup_context,                                      \
            ("**CLManager::setup** FAILED TO SETUP A CONTEXT"));

#define VSMC_RUNTIME_WARNING_CL_MANAGER_SETUP_DEVICE \
    VSMC_RUNTIME_WARNING(setup_device,                                       \
            ("**CLManager::setup** FAILED TO SETUP A DEVICE"));

#define VSMC_RUNTIME_WARNING_CL_MANAGER_SETUP_COMMAND_QUEUE \
    VSMC_RUNTIME_WARNING(setup_command_queue,                                \
            ("**CLManager::setup** FAILED TO SETUP A COMMAND_QUEUE"));

namespace vsmc {

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
/// distinct singletons, it can also provide additional information about which
/// device CLManager shall choose by default through the singleton CLSetup with
/// the same `ID` template argument.
///
/// It is important to configure the platform and device to be used through
/// CLSetup before calling CLManager::instance for the first time. If nothing
/// is done by the user, the default behavior is to use
/// `CL_DEVICE_TYPE_DEFAULT` type device, and set the platform to be the first
/// one that contain such as device, and the device to the first one that is of
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
    public :

    typedef ID id;

    /// \brief Get an instance of the manager singleton
    static CLManager<ID> &instance ()
    {
        static CLManager<ID> manager;

        return manager;
    }

    /// \brief The platform currently being used
    const ::cl::Platform &platform () const {return platform_;}

    /// \brief The context currently being used
    const ::cl::Context &context () const {return context_;}

    /// \brief The device currently being used
    const ::cl::Device &device () const {return device_;}

    /// \brief The vector of all device that the manager found in the platform
    const std::vector< ::cl::Device> &device_vec () const {return device_vec_;}

    /// \brief The command queue currently being used
    const ::cl::CommandQueue &command_queue () const {return command_queue_;}

    /// \brief Whether the platform, context, device and command queue has been
    /// setup correctly
    bool setup () const {return setup_;}

    /// \brief Try to setup the platform, context, device and command queue
    /// using the given device type
    bool setup (cl_device_type dev)
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
    bool setup (const ::cl::Platform &plat, const ::cl::Context &ctx,
            const ::cl::Device &dev, const ::cl::CommandQueue &cmd)
    {
        setup_ = false;
        platform_ = plat;
        context_ = ctx;
        device_ = dev;
        device_vec_ = context_.getInfo<CL_CONTEXT_DEVICES>();
        command_queue_ = cmd;
        setup_ = true;

        return setup_;
    }

    /// \brief Print build log
    template <typename CharT, typename Traits>
    void print_build_log (const ::cl::Program &program,
            std::basic_ostream<CharT, Traits> &os = std::cout)
    {
        cl_build_status status = CL_BUILD_SUCCESS;
        std::string line(78, '=');
        line += "\n";
        std::string log;
        std::string dname;

        for (std::vector< ::cl::Device>::const_iterator
                diter = device_vec_.begin();
                diter != device_vec_.end(); ++diter) {
            program.getBuildInfo(*diter, CL_PROGRAM_BUILD_STATUS, &status);
            if (status != CL_BUILD_SUCCESS) {
                program.getBuildInfo(device_, CL_PROGRAM_BUILD_LOG, &log);
                diter->getInfo(static_cast<cl_device_info>(CL_DEVICE_NAME),
                        &dname);
                os << line << "Build failed for : " << dname << std::endl;
                os << line << log << std::endl << line << std::endl;
            }
        }
    }

    /// \brief Create an OpenCL buffer of a given type and number of elements
    template <typename CLType>
    ::cl::Buffer create_buffer (std::size_t num) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(create_buffer);

        if (num == 0)
            return ::cl::Buffer();

        return ::cl::Buffer(context_, CL_MEM_READ_WRITE, sizeof(CLType) * num);
    }

    /// \brief Create an OpenCL buffer of a given type from a range of elements
    template <typename CLType, typename InputIter>
    ::cl::Buffer create_buffer (InputIter first, InputIter last,
            const std::vector< ::cl::Event> *events = VSMC_NULLPTR,
            ::cl::Event *event = VSMC_NULLPTR) const
    {
        using std::distance;

        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(create_buffer);

        std::size_t num = static_cast<std::size_t>(
                std::abs(distance(first, last)));

        if (num == 0)
            return ::cl::Buffer();

        ::cl::Buffer buf(create_buffer<CLType>(num));
        write_buffer<CLType>(buf, num, first, 0, events, event);

        return buf;
    }

    /// \brief Read an OpenCL buffer of a given type and number of elements
    /// into an iterator
    template <typename CLType, typename OutputIter>
    OutputIter read_buffer (const ::cl::Buffer &buf, std::size_t num,
            OutputIter first, std::size_t offset = 0,
            const std::vector< ::cl::Event> *events = VSMC_NULLPTR,
            ::cl::Event *event = VSMC_NULLPTR) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(read_buffer);

        std::vector<CLType> buffer(num);
        command_queue_.finish();
        command_queue_.enqueueReadBuffer(buf, CL_TRUE, offset,
                sizeof(CLType) * num, static_cast<void *>(&buffer[0]),
                events, event);

        for (std::size_t i = 0; i != num; ++i, ++first)
            *first = buffer[i];

        return first;
    }

    /// \brief Read an OpenCL buffer of a given type and number of elements
    /// into a pointer
    template <typename CLType>
    CLType *read_buffer (const ::cl::Buffer &buf, std::size_t num,
            CLType *first, std::size_t offset = 0,
            const std::vector< ::cl::Event> *events = VSMC_NULLPTR,
            ::cl::Event *event = VSMC_NULLPTR) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(read_buffer);

        command_queue_.finish();
        command_queue_.enqueueReadBuffer(buf, CL_TRUE, offset,
                sizeof(CLType) * num, static_cast<void *>(first),
                events, event);

        return first + num;
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from an iterator
    template <typename CLType, typename InputIter>
    InputIter write_buffer (const ::cl::Buffer &buf, std::size_t num,
            InputIter first, std::size_t offset = 0,
            const std::vector< ::cl::Event> *events = VSMC_NULLPTR,
            ::cl::Event *event = VSMC_NULLPTR) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(write_buffer);

        std::vector<CLType> buffer(num);
        for (std::size_t i = 0; i != num; ++i, ++first)
            buffer[i] = *first;
        command_queue_.finish();
        command_queue_.enqueueWriteBuffer(buf, CL_TRUE, offset,
                sizeof(CLType) * num, static_cast<void *>(&buffer[0]),
                events, event);

        return first;
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from a pointer
    template <typename CLType>
    const CLType *write_buffer (const ::cl::Buffer &buf, std::size_t num,
            const CLType *first, std::size_t offset = 0,
            const std::vector< ::cl::Event> *events = VSMC_NULLPTR,
            ::cl::Event *event = VSMC_NULLPTR) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(write_buffer);

        command_queue_.finish();
        command_queue_.enqueueWriteBuffer(buf, CL_TRUE, offset,
                sizeof(CLType) * num,
                static_cast<void *>(const_cast<CLType *>(first)),
                events, event);

        return first + num;
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from a pointer
    template <typename CLType>
    CLType *write_buffer (const ::cl::Buffer &buf, std::size_t num,
            CLType *first, std::size_t offset = 0,
            const std::vector< ::cl::Event> *events = VSMC_NULLPTR,
            ::cl::Event *event = VSMC_NULLPTR) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(write_buffer);

        command_queue_.finish();
        command_queue_.enqueueWriteBuffer(buf, CL_TRUE, offset,
                sizeof(CLType) * num, static_cast<void *>(first),
                events, event);

        return first + num;
    }

    /// \brief Copy an OpenCL buffer into another of a given type and number of
    /// elements
    template <typename CLType>
    void copy_buffer (const ::cl::Buffer &src, const ::cl::Buffer &dst,
            std::size_t num,
            std::size_t src_offset = 0, std::size_t dst_offset = 0,
            const std::vector< ::cl::Event> *events = VSMC_NULLPTR,
            ::cl::Event *event = VSMC_NULLPTR) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(copy_buffer);

        command_queue_.finish();
        command_queue_.enqueueCopyBuffer(src, dst, src_offset, dst_offset,
                sizeof(CLType) * num, events, event);
        command_queue_.finish();
    }

    /// \brief Create a program given the source within the current context
    ::cl::Program create_program (const std::string &source) const
    {return ::cl::Program(context_, source);}

    /// \brief Run a given kernel with one dimensional global size and local
    /// size on the current command queue
    ///
    /// \details
    /// OpenCL requires that `global_size` is a multiple of `local_size`. This
    /// function will round `N` if it is not already a multiple of
    /// `local_size`. In the kernel it is important to check that
    /// `get_global_id(0)` is not out of range.
    ///
    /// For example, say we have kernel that should be applied to `N` elements.
    /// But the most efficient local size `K` does not divide `N`. Instead of
    /// calculate the correct global size yourself, you can simple call
    /// `run_kernel(kern, N, K)`. But within the kernel, you need to check
    /// `get_global_id(0) < N`
    void run_kernel (const ::cl::Kernel &kern, std::size_t N,
            std::size_t local_size = 0,
            const std::vector< ::cl::Event> *events = VSMC_NULLPTR,
            ::cl::Event *event = VSMC_NULLPTR) const
    {
        command_queue_.finish();
        command_queue_.enqueueNDRangeKernel(kern, ::cl::NullRange,
                get_global_nd_range(N, local_size),
                get_local_nd_range(local_size), events, event);
        command_queue_.finish();
    }

    private :

    ::cl::Platform platform_;
    ::cl::Context context_;
    ::cl::Device device_;
    std::vector< ::cl::Device> device_vec_;
    ::cl::CommandQueue command_queue_;

    bool setup_;
    CLSetup<ID> &setup_default_;

    CLManager () : setup_(false), setup_default_(CLSetup<ID>::instance())
    {setup_cl_manager(setup_default_.device_type());}

    CLManager (const CLManager<ID> &);

    CLManager<ID> &operator= (const CLManager<ID> &);

    void setup_cl_manager (cl_device_type dev_type)
    {
        setup_ = false;

        bool setup_platform = platform_filter(dev_type);
        VSMC_RUNTIME_WARNING_CL_MANAGER_SETUP_PLATFORM;
        if (!setup_platform) return;

        bool setup_context = false;
        bool setup_device = false;
        try {
            std::vector< ::cl::Device> dev_pool;
            std::vector< ::cl::Device> dev_select;
            platform_.getDevices(dev_type, &dev_pool);
            device_filter(dev_pool, dev_select);
            if (dev_select.size() != 0) {
                cl_context_properties context_properties[] = {
                    static_cast<cl_context_properties>(CL_CONTEXT_PLATFORM),
                    reinterpret_cast<cl_context_properties>(platform_()), 0
                };
                context_ = ::cl::Context(dev_select, context_properties);
                setup_context = true;
                device_vec_ = context_.getInfo<CL_CONTEXT_DEVICES>();
                device_ = device_vec_[0];
                setup_device = true;
            }
        } catch (::cl::Error) {}
        VSMC_RUNTIME_WARNING_CL_MANAGER_SETUP_CONTEXT;
        VSMC_RUNTIME_WARNING_CL_MANAGER_SETUP_DEVICE;
        if (!setup_context) return;
        if (!setup_device) return;

        bool setup_command_queue = false;
        try {
            command_queue_ = ::cl::CommandQueue(context_, device_, 0);
            setup_command_queue = true;
        } catch (::cl::Error) {}
        VSMC_RUNTIME_WARNING_CL_MANAGER_SETUP_COMMAND_QUEUE;
        if (!setup_command_queue) return;

        setup_ = true;
    }

    bool platform_filter (cl_device_type dev_type)
    {
        std::vector< ::cl::Platform> platform_vec;
        try {
            ::cl::Platform::get(&platform_vec);
        } catch (::cl::Error) {
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
                } catch (::cl::Error) {}
            }
        }

        // Using default platform: finding the first that has the device type
        for (std::size_t p = 0; p != platform_vec.size(); ++p) {
            try {
                std::vector< ::cl::Device> dev_pool;
                std::vector< ::cl::Device> dev_select;
                platform_vec[p].getDevices(dev_type, &dev_pool);
                device_filter(dev_pool, dev_select);
                if (dev_select.size() != 0) {
                    platform_ = platform_vec[p];
                    return true;
                }
            } catch (::cl::Error) {}
        }

        return false;
    }

    void device_filter (const std::vector< ::cl::Device> &dev_pool,
            std::vector< ::cl::Device> &dev_select)
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
                } catch (::cl::Error) {
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
                } catch (::cl::Error) {
                    dev_select_idx[d] = false;
                }
            }
        }

        for (std::size_t d = 0; d != dev_pool.size(); ++d) {
            if (dev_select_idx[d]) {
                try {
                    dev_select.push_back(dev_pool[d]);
                } catch (::cl::Error) {}
            }
        }
    }

    ::cl::NDRange get_global_nd_range (
            std::size_t N, std::size_t local_size) const
    {
        if (local_size == 0)
            return ::cl::NDRange(N);

        if (N % local_size == 0)
            return ::cl::NDRange(N);

        return ::cl::NDRange((N / local_size + 1) * local_size);
    }

    ::cl::NDRange get_local_nd_range (std::size_t local_size) const
    {return local_size == 0 ? ::cl::NullRange : ::cl::NDRange(local_size);}
}; // clss CLManager

} // namespace vsmc

#endif // VSMC_OPENCL_CL_MANAGER_HPP

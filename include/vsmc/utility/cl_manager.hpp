#ifndef VSMC_UTILITY_CL_MANAGER_HPP
#define VSMC_UTILITY_CL_MANAGER_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/cl_wrapper.hpp>

namespace vsmc {

struct CLDefault
{
    typedef cxx11::integral_constant<cl_device_type, CL_DEVICE_TYPE_DEFAULT>
        opencl_device_type;
};

/// \brief OpenCL Manager
/// \ingroup Utility
///
/// \details
/// Each instance of CLManager is an singleton. Different `ID` template
/// parameter create distinct singletons. Each singleton manages a specific
/// OpenCL device. However, it is possible for different singletons to manage
/// the same device.
///
/// The `ID` template parameter, apart from ensuring that different IDs create
/// distinct singletons, it can also provide additional information about which
/// device CLManager shall choose by default. Therefore it can optionally be a
/// policy class.
///
/// At initialization, the constructor of the singleton check if the following
/// member type, data and member functions exit,
///
/// \code
/// cl_device_type ID::opencl_device_type::value;
/// \endcode
/// The device type. The cl_device_type integer that determine which type of
/// device to use Normally this shall be enough to narrow the choice of device
/// of CLManager If missing, `CL_DEVICE_TYPE_DEFAULT` is used.
///
/// \code
/// static bool ID::check_opencl_platform (const std::string &name);
/// \endcode
/// The name of the platform. If there are multiple OpenCL platforms
/// available, CLManager check each of the platform's name against this
/// function until one returns `true`. For example, say there are both the
/// Intel and AMD platform available for CPU through OpenCL ICD. One can write
/// the following in this policy class
/// \code
/// static bool check_opencl_platform (const std::string &name)
/// {return name.find(std::string("Intel")) != std::string::npos;}
/// \endcode
/// Then, only the Intel platform will be used even the AMD one is found first.
///
/// \code
/// static bool ID::check_opencl_device (const std::string &name);
/// \endcode
/// Similar to `check_opencl_platform` but for a given device name. If there
/// are multiple OpenCL device for a given platform, then this can help to
/// narrow down the selection further.
///
/// \code
/// static bool ID::check_opencl_device_vendor (const std::string &name);
/// \endcode
/// Similar to `check_opencl_device` but only check the vendor of the device. If
/// there are multiple OpenCL device for a given platform with a given type,
/// then this can help to narrow down the selection further. Note that this
/// check will only be performed if `check_opencl_device` is not present.
///
/// \note
/// Before using a CLManager, it is important to check that
/// `CLManager::instance().setup()` returns `true`. Possible reasons of
/// returning `false` include no OpenCL device found at all, or no device match
/// the desired device type, or platform or device name is found. In this case,
/// the user need to setup the CLManager manually.
template <typename ID>
class CLManager
{
    public :

    /// \brief Get an instance of the manager singleton
    static CLManager<ID> &instance ()
    {
        static CLManager<ID> manager;

        return manager;
    }

    /// \brief The platform currently being used
    const cl::Platform &platform () const {return platform_;}

    /// \brief The vector of all platforms that the manager found
    const std::vector<cl::Platform> &platform_vec () const
    {return platform_vec_;}

    /// \brief The context currently being used
    const cl::Context &context () const {return context_;}

    /// \brief The device currently being used
    const cl::Device &device () const {return device_;}

    /// \brief The vector of all device that the manager found in the platform
    const std::vector<cl::Device> &device_vec () const {return device_vec_;}

    /// \brief The command queue currently being used
    const cl::CommandQueue &command_queue () const {return command_queue_;}

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
    bool setup (const cl::Platform &plat, const cl::Context &ctx,
            const cl::Device &dev, const cl::CommandQueue &cmd)
    {
        setup_ = false;
        platform_ = plat;
        cl::Platform::get(&platform_vec_);
        context_ = ctx;
        device_ = dev;
        device_vec_ = context_.getInfo<CL_CONTEXT_DEVICES>();
        command_queue_ = cmd;
        setup_ = true;

        return setup_;
    }

    /// \brief Create an OpenCL buffer of a given type and number of elements
    template<typename CLType>
    cl::Buffer create_buffer (std::size_t num) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(create_buffer);

        if (!num)
            return cl::Buffer();

        return cl::Buffer(context_, CL_MEM_READ_WRITE, sizeof(CLType) * num);
    }

    /// \brief Create an OpenCL buffer of a given type from a range of elements
    template<typename CLType, typename InputIter>
    cl::Buffer create_buffer (InputIter first, InputIter last) const
    {
        using std::distance;

        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(create_buffer);

        std::size_t num = static_cast<std::size_t>(
                std::abs(distance(first, last)));

        if (!num)
            return cl::Buffer();

        cl::Buffer buf(create_buffer<CLType>(num));
        write_buffer<CLType>(buf, num, first);

        return buf;
    }

    /// \brief Read an OpenCL buffer of a given type and number of elements
    /// into an iterator
    template <typename CLType, typename OutputIter>
    OutputIter read_buffer (const cl::Buffer &buf, std::size_t num,
            OutputIter first) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(read_buffer);

        CLType *temp = read_buffer_pool<CLType>(num);
        command_queue_.finish();
        command_queue_.enqueueReadBuffer(buf, CL_TRUE, 0,
                sizeof(CLType) * num, (void *) temp);

        for (std::size_t i = 0; i != num; ++i, ++first)
            *first = temp[i];

        return first;
    }

    /// \brief Read an OpenCL buffer of a given type and number of elements
    /// into an pointer
    template <typename CLType>
    CLType *read_buffer (const cl::Buffer &buf, std::size_t num,
            CLType *first) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(read_buffer);

        command_queue_.finish();
        command_queue_.enqueueReadBuffer(buf, CL_TRUE, 0,
                sizeof(CLType) * num, (void *) first);

        return first + num;
    }

    /// \brief Read an OpenCL buffer of a given type and number of elements
    /// into an pointer
    template <typename CLType, typename InputIter>
    InputIter write_buffer (const cl::Buffer &buf, std::size_t num,
            InputIter first) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(write_buffer);

        CLType *temp = write_buffer_pool<CLType>(num);
        for (std::size_t i = 0; i != num; ++i, ++first)
            temp[i] = *first;
        command_queue_.finish();
        command_queue_.enqueueWriteBuffer(buf, CL_TRUE, 0,
                sizeof(CLType) * num, (void *) temp);

        return first;
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from an iterator
    template <typename CLType>
    const CLType *write_buffer (const cl::Buffer &buf, std::size_t num,
            const CLType *first) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(write_buffer);

        command_queue_.finish();
        command_queue_.enqueueWriteBuffer(buf, CL_TRUE, 0,
                sizeof(CLType) * num, (void *) first);

        return first + num;
    }

    /// \brief Write an OpenCL buffer of a given type and number of elements
    /// from an pointer
    template <typename CLType>
    CLType *write_buffer (const cl::Buffer &buf, std::size_t num,
            CLType *first) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(write_buffer);

        command_queue_.finish();
        command_queue_.enqueueWriteBuffer(buf, CL_TRUE, 0,
                sizeof(CLType) * num, (void *) first);

        return first + num;
    }

    /// \brief Copy an OpenCL buffer into another of a given type and number of
    /// elements
    template <typename CLType>
    void copy_buffer (const cl::Buffer &src, std::size_t num,
            const cl::Buffer &dst) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(copy_buffer);

        command_queue_.finish();
        command_queue_.enqueueCopyBuffer(src, dst, 0, 0, sizeof(CLType) * num);
        command_queue_.finish();
    }

    /// \brief Create a program given the source within the current context
    cl::Program create_program (const std::string &source) const
    {return cl::Program(context_, source);}

    /// \brief Run a given kernel with one dimensional global size and local
    /// size on the current command queue
    ///
    /// \details
    /// OpenCL requires that `global_size` is a multiple of `local_size`. This
    /// function will round `global_size` if it is not already a multiple of
    /// `local_size`. In the kernel it is important to check that
    /// `get_global_id(0)` is not out of range.
    ///
    /// For example, say we have kernel that should be applied to `N` elements.
    /// But the most efficient local size `K` does not divide `N`. Instead of
    /// calculate the correct global size yourself, you can simple call
    /// `run_kernel(kern, N, K)`. But within the kernel, you need to check
    /// `get_global_id(0) < N`
    void run_kernel (const cl::Kernel &kern,
            std::size_t global_size, std::size_t local_size) const
    {
        command_queue_.finish();
        command_queue_.enqueueNDRangeKernel(kern, cl::NullRange,
                get_global_nd_range(global_size, local_size),
                get_local_nd_range(global_size, local_size));
        command_queue_.finish();
    }

#if VSMC_HAS_CXX11_VARIADIC_TEMPLATES
    template <typename Arg1, typename... Args>
    void set_kernel_args (cl::Kernel &kern, std::size_t offset,
            const Arg1 &arg1, const Args &...args)
    {
        kern.setArg(offset, arg1);
        set_kernel_args(kern, offset + 1, args...);
    }
#else // VSMC_HAS_CXX11_VARIADIC_TEMPLATES
    template <typename Arg1>
    void set_kernel_args (cl::Kernel &kern, std::size_t offset,
            const Arg1 &arg1)
    {
        kern.setArg(offset, arg1);
    }

    template <typename Arg1, typename Arg2>
    void set_kernel_args (cl::Kernel &kern, std::size_t offset,
            const Arg1 &arg1, const Arg2 &arg2)
    {
        kern.setArg(offset, arg1);
        set_kernel_args(kern, offset + 1, arg2);
    }

    template <typename Arg1, typename Arg2, typename Arg3>
    void set_kernel_args (cl::Kernel &kern, std::size_t offset,
            const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3)
    {
        kern.setArg(offset, arg1);
        set_kernel_args(kern, offset + 1, arg2, arg3);
    }

    template <typename Arg1, typename Arg2, typename Arg3, typename Arg4>
    void set_kernel_args (cl::Kernel &kern, std::size_t offset,
            const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
            const Arg4 &arg4)
    {
        kern.setArg(offset, arg1);
        set_kernel_args(kern, offset + 1, arg2, arg3, arg4);
    }

    template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
             typename Arg5>
    void set_kernel_args (cl::Kernel &kern, std::size_t offset,
            const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
            const Arg4 &arg4, const Arg5 &arg5)
    {
        kern.setArg(offset, arg1);
        set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5);
    }

    template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
             typename Arg5, typename Arg6>
    void set_kernel_args (cl::Kernel &kern, std::size_t offset,
            const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
            const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6)
    {
        kern.setArg(offset, arg1);
        set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6);
    }

    template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
             typename Arg5, typename Arg6, typename Arg7>
    void set_kernel_args (cl::Kernel &kern, std::size_t offset,
            const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
            const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6,
            const Arg7 &arg7)
    {
        kern.setArg(offset, arg1);
        set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6, arg7);
    }

    template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
             typename Arg5, typename Arg6, typename Arg7, typename Arg8>
    void set_kernel_args (cl::Kernel &kern, std::size_t offset,
            const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
            const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6,
            const Arg7 &arg7, const Arg8 &arg8)
    {
        kern.setArg(offset, arg1);
        set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6, arg7,
                arg8);
    }

    template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
             typename Arg5, typename Arg6, typename Arg7, typename Arg8,
             typename Arg9>
    void set_kernel_args (cl::Kernel &kern, std::size_t offset,
            const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
            const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6,
            const Arg7 &arg7, const Arg8 &arg8, const Arg9 &arg9)
    {
        kern.setArg(offset, arg1);
        set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6, arg7,
                arg8, arg9);
    }
#endif // VSMC_HAS_CXX11_VARIADIC_TEMPLATES

    private :

    cl::Platform platform_;
    std::vector<cl::Platform> platform_vec_;
    cl::Context context_;
    cl::Device device_;
    std::vector<cl::Device> device_vec_;
    cl::CommandQueue command_queue_;

    bool setup_;

    mutable std::size_t read_buffer_pool_bytes_;
    mutable std::size_t write_buffer_pool_bytes_;
    mutable void *read_buffer_pool_;
    mutable void *write_buffer_pool_;

    CLManager () :
        setup_(false), read_buffer_pool_bytes_(0), write_buffer_pool_bytes_(0),
        read_buffer_pool_(VSMC_NULLPTR), write_buffer_pool_(VSMC_NULLPTR)
    {
        cl_device_type dev = traits::OpenCLDeviceTypeTrait<ID>::value ?
            traits::OpenCLDeviceTypeTrait<ID>::type::value :
            CLDefault::opencl_device_type::value;
        setup_cl_manager(dev);
    }

    CLManager (const CLManager<ID> &);
    CLManager<ID> &operator= (const CLManager<ID> &);

    ~CLManager ()
    {
        std::free(read_buffer_pool_);
        std::free(write_buffer_pool_);
    }

    void setup_cl_manager (cl_device_type dev)
    {
        try {
            cl::Platform::get(&platform_vec_);
        } catch (cl::Error) {
            platform_vec_.clear();
        }

        bool setup_platform = false;
        if (traits::HasStaticCheckOpenCLPlatform<ID>::value) {
            for (std::size_t p = 0; p != platform_vec_.size(); ++p) {
                std::string name;
                try {
                    platform_vec_[p].getInfo(CL_PLATFORM_NAME, &name);
                    if (traits::CheckOpenCLPlatformTrait<ID>::check(name)) {
                        platform_ = platform_vec_[p];
                        setup_platform = true;
                        break;
                    }
                } catch (cl::Error) {
                    platform_ = cl::Platform();
                }
            }
        } else if (platform_vec_.size() != 0) {
            platform_ = platform_vec_[0];
            setup_platform = true;
        }
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP_PLATFORM;

        bool setup_context = false;
        try {
            cl_context_properties context_properties[] = {
                CL_CONTEXT_PLATFORM,
                (cl_context_properties)(platform_)(), 0
            };
            context_ = cl::Context(dev, context_properties);
            device_vec_ = context_.getInfo<CL_CONTEXT_DEVICES>();
            setup_context = true;
        } catch (cl::Error) {
            context_ = cl::Context();
            device_vec_.clear();
        }
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP_CONTEXT;

        bool setup_device = false;
        if (traits::HasStaticCheckOpenCLDevice<ID>::value) {
            for (std::size_t d = 0; d != device_vec_.size(); ++d) {
                std::string name;
                try {
                    device_vec_[d].getInfo(CL_DEVICE_NAME, &name);
                    if (traits::CheckOpenCLDeviceTrait<ID>::check(name)) {
                        device_ = device_vec_[d];
                        setup_device = true;
                        break;
                    }
                } catch (cl::Error) {
                    device_ = cl::Device();
                }
            }
        } else if (traits::HasStaticCheckOpenCLDeviceVendor<ID>::value) {
            for (std::size_t d = 0; d != device_vec_.size(); ++d) {
                std::string name;
                try {
                    device_vec_[d].getInfo(CL_DEVICE_VENDOR, &name);
                    if (traits::CheckOpenCLDeviceVendorTrait<ID>::check(name)) {
                        device_ = device_vec_[d];
                        setup_device = true;
                        break;
                    }
                } catch (cl::Error) {
                    device_ = cl::Device();
                }
            }
        } else if (device_vec_.size() != 0) {
            device_ = device_vec_[0];
            setup_device = true;
        }
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP_DEVICE;

        bool setup_command_queue = false;
        try {
            command_queue_ = cl::CommandQueue(context_, device_, 0);
            setup_command_queue = true;
        } catch (cl::Error) {
            command_queue_ = cl::CommandQueue();
        }
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP_COMMAND_QUEUE;

        setup_ = setup_platform && setup_context
            && setup_device && setup_command_queue;
    }

    template <typename CLType>
    CLType *read_buffer_pool (std::size_t num) const
    {
        std::size_t new_bytes = num * sizeof(CLType);
        if (new_bytes > read_buffer_pool_bytes_) {
            std::free(read_buffer_pool_);
            read_buffer_pool_ = std::malloc(new_bytes);
            if (!read_buffer_pool_ && new_bytes)
                throw std::bad_alloc();
            read_buffer_pool_bytes_ = new_bytes;
        }

        return static_cast<CLType *>(read_buffer_pool_);
    }

    template <typename CLType>
    CLType *write_buffer_pool (std::size_t num) const
    {
        std::size_t new_bytes = num * sizeof(CLType);
        if (new_bytes > write_buffer_pool_bytes_) {
            std::free(write_buffer_pool_);
            write_buffer_pool_ = std::malloc(new_bytes);
            if (!write_buffer_pool_ && new_bytes)
                throw std::bad_alloc();
            write_buffer_pool_bytes_ = new_bytes;
        }

        return static_cast<CLType *>(write_buffer_pool_);
    }

    cl::NDRange get_global_nd_range (
            std::size_t global_size, std::size_t local_size) const
    {
        return (local_size && global_size % local_size) ?
            cl::NDRange((global_size / local_size + 1) * local_size):
            cl::NDRange(global_size);
    }

    cl::NDRange get_local_nd_range (
            std::size_t global_size, std::size_t local_size) const
    {return local_size ? cl::NDRange(local_size) : cl::NullRange;}

#if VSMC_HAS_CXX11_VARIADIC_TEMPLATES
    void set_kernel_args (cl::Kernel &, std::size_t) {}
#endif // VSMC_HAS_CXX11_VARIADIC_TEMPLATES
}; // clss CLManager

} // namespace vsmc

#endif // VSMC_UTILITY_CL_MANAGER_HPP

#ifndef VSMC_UTILITY_CL_MANAGER_HPP
#define VSMC_UTILITY_CL_MANAGER_HPP

#define __CL_ENABLE_EXCEPTIONS

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/cl.hpp>

namespace vsmc { namespace opencl {

template <bool, bool, typename IterType>
struct GetHostPtrDispatch
{
    static void *get (IterType iter)
    {
        return VSMC_NULLPTR;
    }
};

template <typename IterType>
struct GetHostPtrDispatch<true, true, IterType>
{
    static void *get (IterType iter)
    {
        return (void *) iter;
    }
};

template <typename CLType, typename IterType>
struct GetHostPtr
{
    static void *get (IterType iter)
    {
        typedef typename cxx11::remove_cv<IterType>::type ptr_type;
        typedef typename cxx11::remove_pointer<ptr_type>::type val_type;
        typedef typename cxx11::remove_cv<val_type>::type host_type;
        typedef typename cxx11::remove_cv<CLType>::type device_type;

        return GetHostPtrDispatch<
            cxx11::is_pointer<IterType>::value,
            cxx11::is_same<host_type, device_type>::value,
            IterType>::get(iter);
    }
};

struct Default
{
    typedef cxx11::integral_constant<cl_device_type, CL_DEVICE_TYPE_DEFAULT>
        opencl_device_type;
};

struct CPU
{
    typedef cxx11::integral_constant<cl_device_type, CL_DEVICE_TYPE_CPU>
        opencl_device_type;
};

struct GPU
{
    typedef cxx11::integral_constant<cl_device_type, CL_DEVICE_TYPE_GPU>
        opencl_device_type;
};

struct Accelerator
{
    typedef cxx11::integral_constant<
        cl_device_type, CL_DEVICE_TYPE_ACCELERATOR> opencl_device_type;
};

/// \brief OpenCL Manager
/// \ingroup OpenCL
template <typename ID>
class CLManager
{
    public :

    typedef std::size_t size_type;

    static CLManager<ID> &instance ()
    {
        static CLManager<ID> manager;

        return manager;
    }

    const cl::Platform &platform () const
    {
        return platform_;
    }

    const std::vector<cl::Platform> &platform_vec () const
    {
        return platform_vec_;
    }

    const cl::Context &context () const
    {
        return context_;
    }

    const cl::Device &device () const
    {
        return device_;
    }

    const std::vector<cl::Device> &device_vec () const
    {
        return device_vec_;
    }

    const cl::CommandQueue &command_queue () const
    {
        return command_queue_;
    }

    bool setup () const
    {
        return setup_;
    }

    void setup (cl_device_type dev)
    {
        setup_ = false;
        setup_cl_manager(dev);
    }

    void setup (const cl::Platform &plat, const cl::Context &ctx,
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
    }

    template<typename CLType>
    cl::Buffer create_buffer (size_type num) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(create_buffer);
        VSMC_RUNTIME_ASSERT((num),
                "CALL vsmc::CLManager::crate_buffer WITH ZERO SIZE");

        if (!num)
            return cl::Buffer();

        return cl::Buffer(context_, CL_MEM_READ_WRITE, sizeof(CLType) * num);
    }

    template<typename CLType, typename InputIter>
    cl::Buffer create_buffer (InputIter first, InputIter last) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(create_buffer);

        size_type num = 0;
        for (InputIter i = first; i != last; ++i)
            ++num;
        VSMC_RUNTIME_ASSERT((num),
                "ATTEMPT TO CALL vsmc::CLManager::crate_buffer "
                "WITH ZERO SIZE")

        if (!num)
            return cl::Buffer();

        cl::Buffer buf(create_buffer<CLType>(num));
        write_buffer<CLType>(buf, num, first);

        return buf;
    }

    template <typename CLType, typename OutputIter>
    OutputIter read_buffer (const cl::Buffer &buf, size_type num,
            OutputIter first) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(read_buffer);

        void *host_ptr = GetHostPtr<CLType, OutputIter>::get(first);
        command_queue_.finish();
        if (host_ptr) {
            command_queue_.enqueueReadBuffer(buf, 1, 0, sizeof(CLType) * num,
                    host_ptr);
            return first + num;
        } else {
            CLType *temp = read_buffer_pool<CLType>(num);
            command_queue_.enqueueReadBuffer(buf, 1, 0,
                    sizeof(CLType) * num, (void *) temp);
            return std::copy(temp, temp + num, first);
        }
    }

    template <typename CLType, typename InputIter>
    InputIter write_buffer (const cl::Buffer &buf, size_type num,
            InputIter first) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(write_buffer);

        void *host_ptr = GetHostPtr<CLType, InputIter>::get(first);
        command_queue_.finish();
        if (host_ptr) {
            command_queue_.enqueueWriteBuffer(buf, 1, 0, sizeof(CLType) * num,
                    host_ptr);
        } else {
            CLType *temp = write_buffer_pool<CLType>(num);
            std::copy(first, first + num, temp);
            command_queue_.enqueueWriteBuffer(buf, 1, 0,
                    sizeof(CLType) * num, (void *) temp);
        }

        return first + num;
    }

    cl::Program create_program (const std::string &source) const
    {
        return cl::Program(context_, source);
    }

    private :

    cl::Platform platform_;
    std::vector<cl::Platform> platform_vec_;
    cl::Context context_;
    cl::Device device_;
    std::vector<cl::Device> device_vec_;
    cl::CommandQueue command_queue_;

    bool setup_;

    mutable size_type read_buffer_pool_bytes_;
    mutable size_type write_buffer_pool_bytes_;
    mutable void *read_buffer_pool_;
    mutable void *write_buffer_pool_;

    CLManager () :
        setup_(false), read_buffer_pool_bytes_(0), write_buffer_pool_bytes_(0),
        read_buffer_pool_(VSMC_NULLPTR), write_buffer_pool_(VSMC_NULLPTR)
    {
        cl_device_type dev = traits::OpenCLDeviceTypeTrait<ID>::type::value;
        if (!dev)
            dev = Default::opencl_device_type::value;
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

        for (std::vector<cl::Platform>::size_type p = 0;
                p != platform_vec_.size(); ++p) {
            try {
                platform_ = platform_vec_[p];

                cl_context_properties context_properties[] = {
                    CL_CONTEXT_PLATFORM,
                    (cl_context_properties)(platform_)(), 0
                };
                context_ = cl::Context(dev, context_properties);

                device_vec_ = context_.getInfo<CL_CONTEXT_DEVICES>();
                device_ = device_vec_[0];

                command_queue_ = cl::CommandQueue(context_, device_, 0);

                setup_ = true;
                break;
            } catch (cl::Error) {
                platform_      = cl::Platform();
                context_       = cl::Context();
                device_        = cl::Device();
                command_queue_ = cl::CommandQueue();
                device_vec_.clear();
            }
        }
    }

    template <typename CLType>
    CLType *read_buffer_pool (size_type num) const
    {
        size_type new_bytes = num * sizeof(CLType);
        if (new_bytes > read_buffer_pool_bytes_) {
            std::free(read_buffer_pool_);
            read_buffer_pool_ = std::malloc(new_bytes);
            if (!read_buffer_pool_ && new_bytes)
                throw std::bad_alloc();
            read_buffer_pool_bytes_ = new_bytes;
        }

        return reinterpret_cast<CLType *>(read_buffer_pool_);
    }

    template <typename CLType>
    CLType *write_buffer_pool (size_type num) const
    {
        size_type new_bytes = num * sizeof(CLType);
        if (new_bytes > write_buffer_pool_bytes_) {
            std::free(write_buffer_pool_);
            write_buffer_pool_ = std::malloc(new_bytes);
            if (!write_buffer_pool_ && new_bytes)
                throw std::bad_alloc();
            write_buffer_pool_bytes_ = new_bytes;
        }

        return reinterpret_cast<CLType *>(write_buffer_pool_);
    }
}; // clss CLManager

} } // namespace vsmc::opencl

namespace {

vsmc::opencl::CLManager<vsmc::opencl::Default>
    &vSMCOpenCLCLManagerInstanceDefault =
    vsmc::opencl::CLManager<vsmc::opencl::Default>::instance();

vsmc::opencl::CLManager<vsmc::opencl::CPU>
    &vSMCOpenCLCLManagerInstanceCPU =
    vsmc::opencl::CLManager<vsmc::opencl::CPU>::instance();

vsmc::opencl::CLManager<vsmc::opencl::GPU>
    &vSMCOpenCLCLManagerInstanceGPU =
    vsmc::opencl::CLManager<vsmc::opencl::GPU>::instance();

vsmc::opencl::CLManager<vsmc::opencl::Accelerator>
    &vSMCOpenCLCLManagerInstanceAccelerator =
    vsmc::opencl::CLManager<vsmc::opencl::Accelerator>::instance();
}

#endif // VSMC_UTILITY_CL_MANAGER_HPP

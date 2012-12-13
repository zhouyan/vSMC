#ifndef VSMC_UTILITY_CL_MANAGER_HPP
#define VSMC_UTILITY_CL_MANAGER_HPP

#define __CL_ENABLE_EXCEPTIONS

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/cl.hpp>

namespace vsmc { namespace cl {

/// \brief OpenCL Device Manager
/// \ingroup CLMG
class DeviceManager
{
    public :

    static DeviceManager &instance ()
    {
        static DeviceManager manager;

        return manager;
    }

    const std::vector<cl::Platform> &platform () const
    {
        return platform_;
    }

    void platform (const std::vector<cl::Platform> &plat)
    {
        platform_ = plat;
        platform_created_ = true;
    }

    bool platform_created () const
    {
        return platform_created_;
    }

    const cl::Context &context () const
    {
        return context_;
    }

    void context (const cl::Context &ctx)
    {
        context_ = ctx;
        context_created_ = true;
        setup_buffer();
    }

    bool context_created () const
    {
        return context_created_;
    }

    const std::vector<cl::Device> &device () const
    {
        return device_;
    }

    void device (const std::vector<cl::Device> &dev)
    {
        device_ = dev;
    }

    bool device_created () const
    {
        return device_created_;
    }

    const cl::CommandQueue &command_queue () const
    {
        return command_queue_;
    }

    void command_queue (const cl::CommandQueue &queue)
    {
        command_queue_ = queue;
        command_queue_created_ = true;
    }

    bool command_queue_created () const
    {
        return command_queue_created_;
    }

    void setup (cl_device_type dev_type)
    {
        if (!platform_created_ || !platform_.size())
            cl::Platform::get(&platform_);
        platform_created_ = true;

        cl_context_properties context_properties[] = {
            CL_CONTEXT_PLATFORM, (cl_context_properties)(platform_[0])(), 0
        };
        context_ = cl::Context(dev_type, context_properties);
        context_created_ = true;
        setup_buffer();

        device_= context_.getInfo<CL_CONTEXT_DEVICES>();
        device_created_ = true;

        command_queue_ = cl::CommandQueue(context_, device_[0], 0);
        command_queue_created_ = true;
    }

    bool setup () const
    {
        return platform_created_ && context_created_ && device_created_ &&
            command_queue_created_;
    }

    template<typename CLType>
    cl::Buffer create_buffer (size_type num) const
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_CONTEXT(create_buffer);

        return cl::Buffer(cl::DeviceManager::instance().context(),
                CL_MEM_READ_WRITE, sizeof(CLType) * num);
    }

    template<typename CLType, typename InputIter>
    cl::Buffer create_buffer (InputIter first, InputIter last) const
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_SETUP(create_buffer);

        size_type num = 0;
        for (InputIter i = first; i != last; ++i)
            ++num;
        cl::Buffer buf(create_buffer<CLType>(num));
        if (!num)
            return buf;

        write_buffer<CLType>(buf, num, first);

        return buf;
    }

    template <typename CLType, typename OutputIter>
    void read_buffer (const cl::Buffer &buf, size_type num,
            OutputIter first) const
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_SETUP(read_buffer);

        void *host_ptr = internal::GetHostPtr<CLType, OutputIter>::get(first);
        command_queue_.finish();
        if (host_ptr) {
            command_queue_.enqueueReadBuffer(buf, 1, 0, sizeof(CLType) * num,
                    host_ptr);
        } else {
            CLType *temp = read_buffer_pool<CLType>(num);
            command_queue_.enqueueReadBuffer(buf, 1, 0,
                    sizeof(CLType) * num, (void *) temp);
            std::copy(temp, temp + num, first);
        }
    }

    template <typename CLType, typename InputIter>
    void write_buffer (const cl::Buffer &buf, size_type num,
            InputIter first) const
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_SETUP(write_buffer);

        void *host_ptr = internal::GetHostPtr<CLType, InputIter>::get(first);
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
    }

    cl::Kernel create_kernel (const std::string &name) const
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(create_kernel);

        cl::Kernel kernel;
        try {
            kernel = cl::Kernel(program_, name.c_str());
        } catch (cl::Error &err) {
            std::cerr << "Failed to create kernel: \""
                << name <<  "\"" << std::endl;
            std::cerr << err.err() << " : " << err.what() << std::endl;
            throw err;
        }

        return kernel;
    }

    void run_kernel (const cl::Kernel &ker) const
    {
        command_queue_.finish();
        command_queue_.enqueueNDRangeKernel(ker,
                cl::NullRange, global_nd_range(), local_nd_range());
        command_queue_.finish();
    }

    private :

    std::vector<cl::Platform> platform_;
    cl::Context context_;
    std::vector<cl::Device> device_;
    cl::CommandQueue command_queue_;

    mutable size_type read_buffer_pool_bytes_;
    mutable size_type write_buffer_pool_bytes_;
    mutable void *read_buffer_pool_;
    mutable void *write_buffer_pool_;

    DeviceManager () :
        platform_created_(false), context_created_(false),
        device_created_(false), command_queue_created_(false)
    {}

    DeviceManager (const DeviceManager &);
    DeviceManager &operator= (const DeviceManager &);

    ~DeviceManager ()
    {
        std::free(read_buffer_pool_);
        std::free(write_buffer_pool_);
    }

    template <typename CLType>
    CLType *read_buffer_pool (size_type num) const
    {
        size_type new_bytes = num * sizeof(CLType);
        if (new_bytes > read_buffer_pool_bytes_) {
            read_buffer_pool_bytes_ = new_bytes;
            std::free(read_buffer_pool_);
            read_buffer_pool_ = std::malloc(read_buffer_pool_bytes_);
            if (!read_buffer_pool_ && read_buffer_pool_bytes_)
                throw std::bad_alloc();
        }

        return reinterpret_cast<CLType *>(read_buffer_pool_);
    }

    template <typename CLType>
    CLType *write_buffer_pool (size_type num) const
    {
        size_type new_bytes = num * sizeof(CLType);
        if (new_bytes > write_buffer_pool_bytes_) {
            write_buffer_pool_bytes_ = new_bytes;
            std::free(write_buffer_pool_);
            write_buffer_pool_ = std::malloc(write_buffer_pool_bytes_);
            if (!write_buffer_pool_ && write_buffer_pool_bytes_)
                throw std::bad_alloc();
        }

        return reinterpret_cast<CLType *>(write_buffer_pool_);
    }
}; // clss DeviceManager

} } // namespace vsmc::cl

#endif // VSMC_UTILITY_CL_MANAGER_HPP

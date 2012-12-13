#ifndef VSMC_UTILITY_CL_MANAGER_HPP
#define VSMC_UTILITY_CL_MANAGER_HPP

#define __CL_ENABLE_EXCEPTIONS

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/cl.hpp>

#define VSMC_RUNTIME_ASSERT_CL_MANAGER_CONTEXT(func) \
    VSMC_RUNTIME_ASSERT((context_created()), ( \
                "**vsmc::CLManager::"#func"** can only be called after true " \
                "**vsmc::CLManager::context_created**")); \

#define VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(func) \
    VSMC_RUNTIME_ASSERT((setup()), ( \
                "**vsmc::CLManager::"#func"** can only be called after true " \
                "**vsmc::CLManager::setup**")); \

namespace vsmc {

namespace traits {

template <bool, bool, typename IterType>
class GetHostPtrDispatch
{
    public :

    static void *get (IterType iter)
    {
        return VSMC_NULLPTR;
    }
};

template <typename IterType>
class GetHostPtrDispatch<true, true, IterType>
{
    public :

    static void *get (IterType iter)
    {
        return (void *) iter;
    }
};

template <typename CLType, typename IterType>
class GetHostPtr
{
    public :

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

} // namespace vsmc::traits

namespace clmgr {

/// \brief OpenCL Manager
/// \ingroup CLMGR
class CLManager
{
    public :

    typedef std::size_t size_type;

    static CLManager &instance ()
    {
        static CLManager manager;

        return manager;
    }

    const cl::Platform &platform () const
    {
        return platform_;
    }

    void platform (const cl::Platform &plat)
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
    }

    bool context_created () const
    {
        return context_created_;
    }

    const cl::Device &device () const
    {
        return device_;
    }

    void device (const cl::Device &dev)
    {
        device_ = dev;
        device_created_ = true;
    }

    bool device_created () const
    {
        return device_created_;
    }

    const std::vector<cl::Device> &device_vec () const
    {
        return device_vec_;
    }

    void device_vec (const std::vector<cl::Device> &dev_vec)
    {
        device_vec_ = dev_vec;
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
        if (platform_vec_.empty())
            cl::Platform::get(&platform_vec_);
        if (!platform_created_)
            platform_ = platform_vec_[0];
        platform_created_ = true;

        if (!context_created_) {
            cl_context_properties context_properties[] = {
                CL_CONTEXT_PLATFORM, (cl_context_properties)(platform_)(), 0
            };
            context_ = cl::Context(dev_type, context_properties);
        }
        context_created_ = true;

        if (device_vec_.empty())
            device_vec_ = context_.getInfo<CL_CONTEXT_DEVICES>();
        if (!device_created_)
            device_ = device_vec_[0];
        device_created_ = true;

        if (!command_queue_created_)
            command_queue_ = cl::CommandQueue(context_, device_, 0);
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
        VSMC_RUNTIME_ASSERT_CL_MANAGER_CONTEXT(create_buffer);
        VSMC_RUNTIME_ASSERT((num),
                "ATTEMPT TO CALL vsmc::CLManager::crate_buffer "
                "WITH ZERO SIZE");

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
    void read_buffer (const cl::Buffer &buf, size_type num,
            OutputIter first) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(read_buffer);

        void *host_ptr = traits::GetHostPtr<CLType, OutputIter>::get(first);
        command_queue_.finish();
        std::clock_t start = std::clock();
        if (host_ptr) {
            command_queue_.enqueueReadBuffer(buf, 1, 0, sizeof(CLType) * num,
                    host_ptr);
        } else {
            CLType *temp = read_buffer_pool<CLType>(num);
            command_queue_.enqueueReadBuffer(buf, 1, 0,
                    sizeof(CLType) * num, (void *) temp);
            std::copy(temp, temp + num, first);
        }
        time_reading_buffer_ += std::clock() - start;
    }

    template <typename CLType, typename InputIter>
    void write_buffer (const cl::Buffer &buf, size_type num,
            InputIter first) const
    {
        VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(write_buffer);

        void *host_ptr = traits::GetHostPtr<CLType, InputIter>::get(first);
        command_queue_.finish();
        std::clock_t start = std::clock();
        if (host_ptr) {
            command_queue_.enqueueWriteBuffer(buf, 1, 0, sizeof(CLType) * num,
                    host_ptr);
        } else {
            CLType *temp = write_buffer_pool<CLType>(num);
            std::copy(first, first + num, temp);
            command_queue_.enqueueWriteBuffer(buf, 1, 0,
                    sizeof(CLType) * num, (void *) temp);
        }
        time_writing_buffer_ += std::clock() - start;
    }

    void reset_timer ()
    {
        time_reading_buffer_ = 0;
        time_writing_buffer_ = 0;
    }

    double time_reading_buffer () const
    {
        return static_cast<double>(time_reading_buffer_) / CLOCKS_PER_SEC;
    }

    double time_writing_buffer () const
    {
        return static_cast<double>(time_writing_buffer_) / CLOCKS_PER_SEC;
    }

    cl::Program create_program (const std::string &source) const
    {
        return cl::Program(context_, source);
    }

    private :

    std::vector<cl::Platform> platform_vec_;
    std::vector<cl::Device> device_vec_;
    cl::Platform platform_;
    cl::Context context_;
    cl::Device device_;
    cl::CommandQueue command_queue_;

    bool platform_created_;
    bool context_created_;
    bool device_created_;
    bool command_queue_created_;

    mutable size_type read_buffer_pool_bytes_;
    mutable size_type write_buffer_pool_bytes_;
    mutable void *read_buffer_pool_;
    mutable void *write_buffer_pool_;

    mutable std::clock_t time_reading_buffer_;
    mutable std::clock_t time_writing_buffer_;

    CLManager () :
        platform_created_(false), context_created_(false),
        device_created_(false), command_queue_created_(false),
        read_buffer_pool_bytes_(0), write_buffer_pool_bytes_(0),
        read_buffer_pool_(VSMC_NULLPTR), write_buffer_pool_(VSMC_NULLPTR),
        time_reading_buffer_(0), time_writing_buffer_(0)
    {
        cl_device_type dev_type[] = {
            CL_DEVICE_TYPE_GPU,
            CL_DEVICE_TYPE_CPU,
            CL_DEVICE_TYPE_ALL
        };

        for (int i = 0; i != 3; ++i) {
            try {
                setup(dev_type[i]);
            } catch (cl::Error) {
                platform_vec_.clear();
                device_vec_.clear();
                platform_ = cl::Platform();
                context_ = cl::Context();
                device_ = cl::Device();
                command_queue_ = cl::CommandQueue();

                platform_created_ = false;
                context_created_ = false;
                device_created_ = false;
                command_queue_created_ = false;
            }

            if (setup())
                break;
        }
    }

    CLManager (const CLManager &);
    CLManager &operator= (const CLManager &);

    ~CLManager ()
    {
        std::free(read_buffer_pool_);
        std::free(write_buffer_pool_);
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

} } // namespace vsmc::clmgr

namespace {
vsmc::clmgr::CLManager &vSMCCLMGRCLManagerInstance =
    vsmc::clmgr::CLManager::instance();
}

#endif // VSMC_UTILITY_CL_MANAGER_HPP

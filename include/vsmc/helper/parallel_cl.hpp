#ifndef VSMC_HELPER_PARALLEL_CL_HPP
#define VSMC_HELPER_PARALLEL_CL_HPP

#define __CL_ENABLE_EXCEPTIONS

#include <vsmc/internal/common.hpp>
#include <vsmc/core/rng.hpp>
#include <vsmc/helper/parallel_cl/cl.hpp>

#define VSMC_RUNTIME_ASSERT_STATE_CL_CONTEXT(func) \
    VSMC_RUNTIME_ASSERT((context_created()), ( \
                "**StateCL::"#func"** can only be called after successful " \
                "**StateCL::context_created**")); \

#define VSMC_RUNTIME_ASSERT_STATE_CL_SETUP(func) \
    VSMC_RUNTIME_ASSERT((setup()), ( \
                "**StateCL::"#func"** can only be called after successful " \
                "**StateCL::setup**")); \

#define VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(func) \
    VSMC_RUNTIME_ASSERT((build()), ( \
                "**StateCL::"#func"** can only be called after successful " \
                "**StateCL::setup**")); \

namespace vsmc {

namespace internal {

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

} // namespace vsmc::internal

/// \brief Particle::value_type subtype
/// \ingroup OpenCL
template <unsigned Dim, typename T>
class StateCL
{
    public :

    typedef cl_ulong size_type;
    typedef T state_type;

    explicit StateCL (size_type N) :
        dim_(Dim), size_(N), local_size_(0),
        read_buffer_pool_bytes_(0), write_buffer_pool_bytes_(0),
        read_buffer_pool_(VSMC_NULLPTR), write_buffer_pool_(VSMC_NULLPTR),
        platform_created_(false), context_created_(false),
        device_created_(false), command_queue_created_(false),
        program_created_(false), build_(false),
        state_host_(dim_ * N), weight_host_(N), accept_host_(N)
    {
        local_size(0);
    }

    virtual ~StateCL ()
    {
        std::free(read_buffer_pool_);
        std::free(write_buffer_pool_);
    }

    unsigned dim () const
    {
        return dim_;
    }

    void resize_dim (unsigned dim)
    {
        VSMC_STATIC_ASSERT((Dim == Dynamic),
                USE_METHOD_resize_dim_WITH_A_FIXED_SIZE_StateCL_OBJECT);
        VSMC_RUNTIME_ASSERT_STATE_CL_CONTEXT(resize_dim);

        state_host_.resize(dim * size_);
        state_device_ = create_buffer<T>(dim * size_);
        dim_ = dim;
    }

    size_type size () const
    {
        return size_;
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

    const cl::Program &program () const
    {
        return program_;
    }

    void program (const cl::Program &prg)
    {
        program_ = prg;
        program_created_ = true;
    }

    bool program_created () const
    {
        return program_created_;
    }

    void local_size (size_type lsize)
    {
        local_size_ = lsize;

        if (lsize)
            local_nd_range_ = cl::NDRange(lsize);
        else
            local_nd_range_ = cl::NullRange;

        if (lsize && size_ % lsize)
            global_nd_range_ = cl::NDRange((size_ / lsize + 1) * lsize);
        else
            global_nd_range_ = cl::NDRange(size_);
    }

    void local_size () const
    {
        return local_size_;
    }

    cl::NDRange global_nd_range () const
    {
        return global_nd_range_;
    }

    cl::NDRange local_nd_range () const
    {
        return local_nd_range_;
    }

    const cl::Buffer &state_device () const
    {
        return state_device_;
    }

    const double *state_host () const
    {
        read_buffer<state_type>(state_device_, dim_ * size_, &state_host_[0]);

        return &state_host_[0];
    }

    const cl::Buffer &weight_device () const
    {
        return weight_device_;
    }

    const double *weight_host () const
    {
        read_buffer<state_type>(weight_device_, size_, &weight_host_[0]);

        return &weight_host_[0];
    }

    const cl::Buffer &accept_device () const
    {
        return accept_device_;
    }

    const cl_uint *accept_host () const
    {
        read_buffer<cl_uint>(accept_device_, size_, &accept_host_[0]);

        return &accept_host_[0];
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

    void build (const std::string &source, const std::string &flags)
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_SETUP(build);

        if (!program_created_) {
            std::stringstream ss;
            ss << "#ifndef VSMC_USE_RANDOM123\n";
            ss << "#define VSMC_USE_RANDOM123 " << VSMC_USE_RANDOM123 << '\n';
            ss << "#endif\n";
            if (sizeof(T) == sizeof(cl_float))
                ss << "typedef float state_type;\n";
            else if (sizeof(T) == sizeof(cl_double))
                ss << "typedef double state_type;\n";
            else
                throw std::runtime_error("Unsupported CL data type");
            ss << "typedef struct state_struct {\n";
            for (unsigned d = 0; d != dim_; ++d)
                ss << "state_type param" << d + 1 << ";\n";
            ss << "} state_struct;\n";
            ss << "typedef ulong size_type;\n";
            ss << "__constant size_type Size = " << size_ << "UL;\n";
            ss << "__constant uint Dim = " << dim_ << ";\n";
            VSMC_SEED_TYPE &seed = VSMC_SEED_TYPE::instance();
            ss << "__constant ulong Seed = " << seed.get() << "UL;\n";
            seed.skip(size_);
            ss << "#include <vsmc/helper/parallel_cl/common.cl>\n";
            ss << source << '\n';
            program_ = cl::Program(context_, ss.str());
            program_created_ = true;
        }

        try {
            program_.build(device_, flags.c_str());
            program_.getBuildInfo(device_[0], CL_PROGRAM_BUILD_LOG,
                    &build_log_);
        } catch (cl::Error &err) {
            std::string log;
            std::cerr << "===========================" << std::endl;
            std::cerr << "Error: vsmc: OpenCL program Build failed"
                << std::endl;
            program_.getBuildInfo(device_[0], CL_PROGRAM_BUILD_OPTIONS, &log);
            std::cerr << "===========================" << std::endl;
            std::cerr << "Build options:" << std::endl;
            std::cerr << "---------------------------" << std::endl;
            std::cerr << log << std::endl;
            program_.getInfo(CL_PROGRAM_SOURCE, &log);
            std::cerr << "===========================" << std::endl;
            std::cerr << "Build source:" << std::endl;
            std::cerr << "---------------------------" << std::endl;
            std::cerr << log << std::endl;
            program_.getBuildInfo(device_[0], CL_PROGRAM_BUILD_LOG,
                    &build_log_);
            std::cerr << "===========================" << std::endl;
            std::cerr << "Build log:" << std::endl;
            std::cerr << "---------------------------" << std::endl;
            std::cerr << build_log_ << std::endl;
            std::cerr << "===========================" << std::endl;
            throw err;
        }
        build_ = true;

        kernel_copy_ = create_kernel("copy");
    }

    bool build () const
    {
        return build_;
    }

    std::string build_log () const
    {
        return build_log_;
    }

    template<typename CLType>
    cl::Buffer create_buffer (size_type num) const
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_CONTEXT(create_buffer);

        return cl::Buffer(context_, CL_MEM_READ_WRITE, sizeof(CLType) * num);
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
        VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(create_kernel)

        return cl::Kernel(program_, name.c_str());
    }

    void run_parallel (const cl::Kernel &ker) const
    {
        command_queue_.finish();
        command_queue_.enqueueNDRangeKernel(ker,
                cl::NullRange, global_nd_range(), local_nd_range());
        command_queue_.finish();
    }

    template<typename IntType>
    void copy (const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(create_kernel)

        write_buffer<size_type>(copy_device_, size_, copy_from);
        kernel_copy_.setArg(0, state_device_);
        kernel_copy_.setArg(1, copy_device_);
        run_parallel(kernel_copy_);
    }

    private :

    unsigned dim_;
    size_type size_;
    size_type local_size_;

    mutable size_type read_buffer_pool_bytes_;
    mutable size_type write_buffer_pool_bytes_;
    mutable void *read_buffer_pool_;
    mutable void *write_buffer_pool_;

    std::vector<cl::Platform> platform_;
    cl::Context context_;
    std::vector<cl::Device> device_;
    cl::CommandQueue command_queue_;
    cl::Program program_;
    cl::Kernel kernel_copy_;

    bool platform_created_;
    bool context_created_;
    bool device_created_;
    bool command_queue_created_;
    bool program_created_;
    bool build_;
    std::string build_log_;

    cl::NDRange global_nd_range_;
    cl::NDRange local_nd_range_;

    cl::Buffer state_device_;
    cl::Buffer weight_device_;
    cl::Buffer accept_device_;

    mutable std::vector<double> state_host_;
    mutable std::vector<double> weight_host_;
    mutable std::vector<cl_uint> accept_host_;

    cl::Buffer copy_device_;

    void setup_buffer ()
    {
        state_device_  = create_buffer<T>(dim_ * size_);
        weight_device_ = create_buffer<T>(size_);
        accept_device_ = create_buffer<cl_uint>(size_);
        copy_device_   = create_buffer<size_type>(size_);
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
}; // class StateCL

/// \brief Sampler<T>::init_type subtype
/// \ingroup OpenCL
template <typename T>
class InitializeCL
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (Particle<T> &particle, void *param)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateCL, T, InitializeCL);

        set_kernel(particle);
        initialize_param(particle, param);
        pre_processor(particle);
        particle.value().run_parallel(kernel_);
        post_processor(particle);
        const cl_uint *accept = particle.value().accept_host();

        return std::accumulate(accept, accept + particle.size(),
                static_cast<cl_uint>(0));
    }

    virtual void initialize_param (Particle<T> &, void *) {}
    virtual void initialize_state (std::string &) = 0;
    virtual void pre_processor (Particle<T> &) {}
    virtual void post_processor (Particle<T> &) {}

    cl::Kernel &kernel ()
    {
        return kernel_;
    }

    const cl::Kernel &kernel () const
    {
        return kernel_;
    }

    void set_kernel (const Particle<T> &particle)
    {
        std::string kname;
        initialize_state(kname);

        if (kernel_name_ != kname) {
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
        }

        kernel_.setArg(0, particle.value().state_device());
        kernel_.setArg(1, particle.value().weight_device());
        kernel_.setArg(2, particle.value().accept_device());
    }

    protected :

    InitializeCL () {}

    InitializeCL (const InitializeCL<T> &other) :
        kernel_(other.kernel_), kernel_name_(other.kernel_name_) {}

    InitializeCL<T> &operator= (const InitializeCL<T> &other)
    {
        if (this != &other) {
            kernel_ = other.kernel_;
            kernel_name_ = other.kernel_name_;
        }

        return *this;
    }

    virtual ~InitializeCL () {}

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
}; // class InitializeCL

/// \brief Sampler<T>::move_type subtype
/// \ingroup OpenCL
template <typename T>
class MoveCL
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateCL, T, MoveCL);

        set_kernel(iter, particle);
        pre_processor(iter, particle);
        particle.value().run_parallel(kernel_);
        post_processor(iter, particle);
        const cl_uint *accept = particle.value().accept_host();

        return std::accumulate(accept, accept + particle.size(),
                static_cast<cl_uint>(0));
    }

    virtual void move_state (unsigned iter, std::string &) = 0;
    virtual void pre_processor (unsigned iter, Particle<T> &) {}
    virtual void post_processor (unsigned iter, Particle<T> &) {}

    cl::Kernel &kernel ()
    {
        return kernel_;
    }

    const cl::Kernel &kernel () const
    {
        return kernel_;
    }

    void set_kernel (unsigned iter, const Particle<T> &particle)
    {
        std::string kname;
        move_state(iter, kname);

        if (kernel_name_ != kname) {
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
        }

        kernel_.setArg(0, (cl_uint) iter);
        kernel_.setArg(1, particle.value().state_device());
        kernel_.setArg(2, particle.value().weight_device());
        kernel_.setArg(3, particle.value().accept_device());
    }

    protected :

    MoveCL () {}

    MoveCL (const MoveCL<T> &other) :
        kernel_(other.kernel_), kernel_name_(other.kernel_name_) {}

    MoveCL<T> &operator= (const MoveCL<T> &other)
    {
        if (this != &other) {
            kernel_ = other.kernel_;
            kernel_name_ = other.kernel_name_;
        }

        return *this;
    }

    virtual ~MoveCL () {}

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
}; // class MoveCL

/// \brief Monitor<T>::eval_type subtype
/// \ingroup OpenCL
template <typename T>
class MonitorEvalCL
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    void operator() (unsigned iter, unsigned dim, const Particle<T> &particle,
            double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateCL, T, MonitorEvalCL);

        set_kernel(iter, dim, particle);
        pre_processor(iter, particle);
        particle.value().run_parallel(kernel_);
        particle.value().template read_buffer<typename T::state_type>(
                    buffer_device_, particle.value().size() * dim, res);
        post_processor(iter, particle);
    }

    virtual void monitor_state (unsigned iter, std::string &) = 0;
    virtual void pre_processor (unsigned iter, const Particle<T> &) {}
    virtual void post_processor (unsigned iter, const Particle<T> &) {}

    cl::Kernel kernel ()
    {
        return kernel_;
    }

    const cl::Kernel kernel () const
    {
        return kernel_;
    }

    void set_kernel (unsigned iter, unsigned dim, const Particle<T> &particle)
    {
        std::string kname;
        monitor_state(iter, kname);

        if (kernel_name_ != kname) {
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
            buffer_device_ = particle.value().template
                create_buffer<typename T::state_type>(
                        particle.value().size() * dim);
        }

        kernel_.setArg(0, (cl_uint) iter);
        kernel_.setArg(1, (cl_uint) dim);
        kernel_.setArg(2, particle.value().state_device());
        kernel_.setArg(3, buffer_device_);
    }

    protected :

    MonitorEvalCL () {}

    MonitorEvalCL (const MonitorEvalCL<T> &other) :
        kernel_(other.kernel_), kernel_name_(other.kernel_name_),
        buffer_device_(other.buffer_device_) {}

    MonitorEvalCL<T> &operator= (const MonitorEvalCL<T> &other)
    {
        if (this != &other) {
            kernel_ = other.kernel_;
            kernel_name_ = other.kernel_name_;
            buffer_device_ = other.buffer_device_;
        }

        return *this;
    }

    virtual ~MonitorEvalCL () {}

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    cl::Buffer buffer_device_;
}; // class MonitorEvalCL

/// \brief Path<T>::eval_type subtype
/// \ingroup OpenCL
template <typename T>
class PathEvalCL
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    double operator() (unsigned iter, const Particle<T> &particle,
        double *res)
    {
        VSMC_STATIC_ASSERT_STATE_TYPE(StateCL, T, PathEvalCL);

        set_kernel(iter, particle);
        pre_processor(iter, particle);
        particle.value().run_parallel(kernel_);
        particle.value().template read_buffer<typename T::state_type>(
                buffer_device_, particle.value().size(), res);
        post_processor(iter, particle);

        return this->path_width(iter, particle);
    }

    virtual void path_state (unsigned iter, std::string &) = 0;
    virtual double path_width (unsigned iter, const Particle<T> &) = 0;
    virtual void pre_processor (unsigned iter, const Particle<T> &) {}
    virtual void post_processor (unsigned iter, const Particle<T> &) {}

    cl::Kernel kernel ()
    {
        return kernel_;
    }

    const cl::Kernel kernel () const
    {
        return kernel_;
    }

    void set_kernel (unsigned iter, const Particle<T> &particle)
    {
        std::string kname;
        path_state(iter, kname);

        if (kernel_name_ != kname) {
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
            buffer_device_ = particle.value().template
                create_buffer<typename T::state_type>(
                        particle.value().size());
        }

        kernel_.setArg(0, (cl_uint) iter);
        kernel_.setArg(1, particle.value().state_device());
        kernel_.setArg(2, buffer_device_);
    }

    protected :

    PathEvalCL () {}

    PathEvalCL (const PathEvalCL<T> &other) :
        kernel_(other.kernel_), kernel_name_(other.kernel_name_),
        buffer_device_(other.buffer_device_) {}

    PathEvalCL<T> &operator= (const PathEvalCL<T> &other)
    {
        if (this != &other) {
            kernel_ = other.kernel_;
            kernel_name_ = other.kernel_name_;
            buffer_device_ = other.buffer_device_;
        }

        return *this;
    }

    virtual ~PathEvalCL () {}

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    cl::Buffer buffer_device_;
}; // class PathEvalCL

} // namespace vsmc

#endif // VSMC_HELPER_PARALLEL_CL_HPP

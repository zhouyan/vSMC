#ifndef VSMC_CL_PARALLEL_CL_HPP
#define VSMC_CL_PARALLEL_CL_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/cl_manager.hpp>

#define VSMC_STATIC_ASSERT_STATE_CL_TYPE(type) \
    VSMC_STATIC_ASSERT((cxx11::is_same<type, cl_float>::value \
             || cxx11::is_same<type, cl_double>::value), \
            USE_StateCL_WITH_A_STATE_TYPE_OTHER_THAN_cl_float_AND_cl_double)

#define VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(func) \
    VSMC_RUNTIME_ASSERT((build()), ( \
                "**CLManager::"#func"** can only be called after true " \
                "**CLManager::setup**")); \

namespace vsmc {

namespace internal {

template <typename> void set_cl_state_type (std::stringstream &);

template<>
void set_cl_state_type<cl_float>(std::stringstream &ss)
{
    ss << "typedef float state_type;\n";
    ss << "#define U01_OPEN_OPEN_32     u01_open_open_32_24\n";
    ss << "#define U01_OPEN_CLOSED_32   u01_open_closed_32_24\n";
    ss << "#define U01_CLOSED_OPEN_32   u01_closed_closed_32_24\n";
    ss << "#define U01_CLOSED_CLOSED_32 u01_closed_closed_32_24\n";
}

template<>
void set_cl_state_type<cl_double>(std::stringstream &ss)
{
    ss << "typedef double state_type;\n";
    ss << "#define U01_OPEN_OPEN_32     u01_open_open_32_53\n";
    ss << "#define U01_OPEN_CLOSED_32   u01_open_closed_32_53\n";
    ss << "#define U01_CLOSED_OPEN_32   u01_closed_closed_32_53\n";
    ss << "#define U01_CLOSED_CLOSED_32 u01_closed_closed_32_53\n";
    ss << "#define U01_OPEN_OPEN_64     u01_open_open_64_53\n";
    ss << "#define U01_OPEN_CLOSED_64   u01_open_closed_64_53\n";
    ss << "#define U01_CLOSED_OPEN_64   u01_closed_closed_64_53\n";
    ss << "#define U01_CLOSED_CLOSED_64 u01_closed_closed_64_53\n";
}

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
        dim_(Dim == Dynamic ? 1 : Dim), size_(N), local_size_(0),
        program_created_(false), build_(false),
        state_host_(dim_ * N), accept_host_(N), time_running_kernel_(0)
    {
        VSMC_STATIC_ASSERT_STATE_CL_TYPE(T);
        local_size(0);
        clmgr::CLManager &manager = clmgr::CLManager::instance();
        state_device_  = manager.create_buffer<state_type>(dim_ * size_);
        accept_device_ = manager.create_buffer<state_type>(size_);
        copy_device_   = manager.create_buffer<size_type>(size_);
    }

    unsigned dim () const
    {
        return dim_;
    }

    void resize_dim (unsigned dim)
    {
        VSMC_STATIC_ASSERT((Dim == Dynamic),
                USE_METHOD_resize_dim_WITH_A_FIXED_SIZE_StateCL_OBJECT);

        state_device_ = clmgr::CLManager::instance()
            .create_buffer<T>(dim * size_);
        state_host_.resize(dim * size_);
        dim_ = dim;
    }

    size_type size () const
    {
        return size_;
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
        clmgr::CLManager::instance().read_buffer<state_type>(
                state_device_, dim_ * size_, &state_host_[0]);

        return &state_host_[0];
    }

    const cl::Buffer &accept_device () const
    {
        return accept_device_;
    }

    const cl_uint *accept_host () const
    {
        clmgr::CLManager::instance().read_buffer<cl_uint>(
                accept_device_, size_, &accept_host_[0]);

        return &accept_host_[0];
    }

    void build (const std::string &source, const std::string &flags)
    {
        VSMC_RUNTIME_ASSERT((!build_),
                "**StateCL::build**: Program already build");

        clmgr::CLManager &manager = clmgr::CLManager::instance();

        if (!program_created_) {
            std::stringstream ss;
            ss << "#if defined(cl_khr_fp64)\n";
            ss << "#pragma OPENCL EXTENSION cl_khr_fp64 : enable\n";
            ss << "#elif defined(cl_amd_fp64)\n";
            ss << "#pragma OPENCL EXTENSION cl_amd_fp64 : enable\n";
            ss << "#endif\n";

            ss << "#if defined(cl_khr_fp64) || defined(cl_amd_fp64)\n";
            ss << "#define R123_USE_U01_DOUBLE 1\n";
            ss << "#else\n";
            ss << "#define R123_USE_U01_DOUBLE 0\n";
            ss << "#endif\n";

            ss << "#ifndef VSMC_USE_RANDOM123\n";
            ss << "#define VSMC_USE_RANDOM123 " << VSMC_USE_RANDOM123 << '\n';
            ss << "#endif\n";

            internal::set_cl_state_type<T>(ss);
            ss << "typedef struct state_struct {\n";
            for (unsigned d = 0; d != dim_; ++d)
                ss << "state_type param" << d + 1 << ";\n";
            ss << "} state_struct;\n";

            ss << "typedef ulong size_type;\n";
            ss << "#define Size " << size_ << "UL\n";
            ss << "#define Dim  " << dim_  << "U\n";
            VSMC_SEED_TYPE &seed = VSMC_SEED_TYPE::instance();
            ss << "#define Seed " << seed.get() << "UL\n";
            seed.skip(size_);
            ss << "#include <vsmc/cl/device.h>\n";
            ss << source << '\n';
            program_ = manager.create_program(ss.str());
            program_created_ = true;
        }

        try {
            program_.build(manager.device_vec(), flags.c_str());
            program_.getBuildInfo(manager.device(), CL_PROGRAM_BUILD_LOG,
                    &build_log_);
        } catch (cl::Error &err) {
            std::string log;
            std::cerr << "===========================" << std::endl;
            std::cerr << "Error: vSMC: OpenCL program Build failed"
                << std::endl;
            std::cerr << err.err() << " : " << err.what() << std::endl;
            program_.getBuildInfo(manager.device(),
                    CL_PROGRAM_BUILD_OPTIONS, &log);
            std::cerr << "===========================" << std::endl;
            std::cerr << "Build options:" << std::endl;
            std::cerr << "---------------------------" << std::endl;
            std::cerr << log << std::endl;
            program_.getInfo(CL_PROGRAM_SOURCE, &log);
            std::cerr << "===========================" << std::endl;
            std::cerr << "Build source:" << std::endl;
            std::cerr << "---------------------------" << std::endl;
            std::cerr << log << std::endl;
            program_.getBuildInfo(manager.device(), CL_PROGRAM_BUILD_LOG,
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

    cl::Kernel create_kernel (const std::string &name) const
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(create_kernel);

        return cl::Kernel(program_, name.c_str());
    }

    void run_kernel (const cl::Kernel &ker) const
    {
        clmgr::CLManager &manager = clmgr::CLManager::instance();
        manager.command_queue().finish();
        std::clock_t start = std::clock();
        manager.command_queue().enqueueNDRangeKernel(ker,
                cl::NullRange, global_nd_range(), local_nd_range());
        manager.command_queue().finish();
        time_running_kernel_ += std::clock() - start;
    }

    void reset_timer ()
    {
        time_running_kernel_ = 0;
    }

    double time_running_kernel () const
    {
        return static_cast<double>(time_running_kernel_) / CLOCKS_PER_SEC;
    }

    template<typename IntType>
    void copy (const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(copy);

        clmgr::CLManager::instance().write_buffer<size_type>(
                copy_device_, size_, copy_from);
        kernel_copy_.setArg(0, state_device_);
        kernel_copy_.setArg(1, copy_device_);
        run_kernel(kernel_copy_);
    }

    private :

    unsigned dim_;
    size_type size_;
    size_type local_size_;

    cl::Program program_;
    cl::Kernel kernel_copy_;

    bool program_created_;
    bool build_;
    std::string build_log_;

    cl::NDRange global_nd_range_;
    cl::NDRange local_nd_range_;

    cl::Buffer state_device_;
    cl::Buffer accept_device_;

    mutable std::vector<double> state_host_;
    mutable std::vector<cl_uint> accept_host_;

    cl::Buffer copy_device_;

    mutable std::clock_t time_running_kernel_;
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
        particle.value().run_kernel(kernel_);
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
        kernel_.setArg(1, particle.value().accept_device());
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
        particle.value().run_kernel(kernel_);
        post_processor(iter, particle);
        const cl_uint *accept = particle.value().accept_host();

        return std::accumulate(accept, accept + particle.size(),
                static_cast<cl_uint>(0));
    }

    virtual void move_state (unsigned, std::string &) = 0;
    virtual void pre_processor (unsigned, Particle<T> &) {}
    virtual void post_processor (unsigned, Particle<T> &) {}

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

        kernel_.setArg<cl_uint>(0, iter);
        kernel_.setArg(1, particle.value().state_device());
        kernel_.setArg(2, particle.value().accept_device());
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
        particle.value().run_kernel(kernel_);
        clmgr::CLManager::instance().read_buffer<typename T::state_type>(
                    buffer_device_, particle.value().size() * dim, res);
        post_processor(iter, particle);
    }

    virtual void monitor_state (unsigned, std::string &) = 0;
    virtual void pre_processor (unsigned, const Particle<T> &) {}
    virtual void post_processor (unsigned, const Particle<T> &) {}

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
            buffer_device_ = clmgr::CLManager::instance()
                .create_buffer<typename T::state_type>(
                        particle.value().size() * dim);
        }

        kernel_.setArg<cl_uint>(0, iter);
        kernel_.setArg<cl_uint>(1, dim);
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
        particle.value().run_kernel(kernel_);
        clmgr::CLManager::instance().read_buffer<typename T::state_type>(
                buffer_device_, particle.value().size(), res);
        post_processor(iter, particle);

        return this->path_width(iter, particle);
    }

    virtual void path_state (unsigned, std::string &) = 0;
    virtual double path_width (unsigned, const Particle<T> &) = 0;
    virtual void pre_processor (unsigned, const Particle<T> &) {}
    virtual void post_processor (unsigned, const Particle<T> &) {}

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
            buffer_device_ = clmgr::CLManager::instance()
                .create_buffer<typename T::state_type>(
                        particle.value().size());
        }

        kernel_.setArg<cl_uint>(0, iter);
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

#endif // VSMC_CL_PARALLEL_CL_HPP

#ifndef VSMC_OPENCL_PARALLEL_CL_HPP
#define VSMC_OPENCL_PARALLEL_CL_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/rng.hpp>
#include <vsmc/utility/opencl/manager.hpp>

namespace vsmc {

namespace internal {

template <typename> void set_cl_state_type (std::stringstream &);

template<>
void set_cl_state_type<cl_float>(std::stringstream &ss)
{
    ss << "typedef float state_type;\n";
    ss << "#define VSMC_STATE_TYPE_IS_FLOAT  1\n";
    ss << "#define VSMC_STATE_TYPE_IS_DOUBLE 0\n";
}

template<>
void set_cl_state_type<cl_double>(std::stringstream &ss)
{
    ss << "typedef double state_type;\n";
    ss << "#define VSMC_STATE_TYPE_IS_FLOAT  0\n";
    ss << "#define VSMC_STATE_TYPE_IS_DOUBLE 1\n";
}

} // namespace vsmc::internal

/// \brief Particle::value_type subtype
/// \ingroup CL
template <unsigned Dim, typename T, typename CLManagerID>
class StateCL
{
    public :

    typedef cl_ulong size_type;
    typedef T state_type;
    typedef opencl::CLManager<CLManagerID> cl_manager_type;

    explicit StateCL (size_type N) :
        dim_(Dim == Dynamic ? 1 : Dim), size_(N),
        cl_manager_(cl_manager_type::instance()), build_(false), build_id_(0),
        state_buffer_(cl_manager_.template create_buffer<state_type>(
                dim_ * size_)),
        copy_from_buffer_(cl_manager_.template create_buffer<size_type>(size_))
    {
        VSMC_STATIC_ASSERT_STATE_CL_VALUE_TYPE(T);
    }

    StateCL (const StateCL<Dim, T, CLManagerID> &other) :
        dim_(other.dim_), size_(other.size_), cl_manager_(other.cl_manager_),
        program_(other.program_), kernel_copy_(other.kernel_copy_),
        build_(other.build_), build_id_(0),
        state_buffer_(cl_manager_.template create_buffer<state_type>(
                dim_ * size_)),
        copy_from_buffer_(cl_manager_.template create_buffer<size_type>(size_))
    {
        cl_manager_.template copy_buffer<state_type>(
                other.state_buffer_, dim_ * size_, state_buffer_);
    }

    StateCL<Dim, T, CLManagerID> &operator= (
            const StateCL<Dim, T, CLManagerID> &other)
    {
        if (this != &other) {
            dim_         = other.dim_;
            size_        = other.size_;
            program_     = other.program_;
            kernel_copy_ = other.kernel_copy_;
            build_       = other.build_;
            build_id_    = 0;

            state_buffer_ =
                cl_manager_.template create_buffer<state_type>(dim_ * size_);
            copy_from_buffer_ =
                cl_manager_.template create_buffer<size_type>(size_);
            cl_manager_.template copy_buffer<state_type>(
                    other.state_buffer_, dim_ * size_, state_buffer_);
        }

        return *this;
    }

    unsigned dim () const
    {
        return dim_;
    }

    void resize_dim (unsigned dim)
    {
        VSMC_STATIC_ASSERT((Dim == Dynamic),
                USE_METHOD_resize_dim_WITH_A_FIXED_SIZE_StateCL_OBJECT);

        state_buffer_ = cl_manager_.template create_buffer<T>(dim * size_);
        dim_ = dim;
    }

    size_type size () const
    {
        return size_;
    }

    cl_manager_type &cl_manager () const
    {
        return cl_manager_;
    }

    const cl::Buffer &state_buffer () const
    {
        return state_buffer_;
    }

    const cl::Program &program () const
    {
        return program_;
    }

    void build (const std::string &source,
            const std::string &flags = std::string())
    {
        ++build_id_;

        std::stringstream ss;

        ss << "#ifndef VSMC_USE_RANDOM123\n";
        ss << "#define VSMC_USE_RANDOM123 " << VSMC_USE_RANDOM123 << '\n';
        ss << "#endif\n";
        ss << "#if defined(cl_khr_fp64)\n";
        ss << "#pragma OPENCL EXTENSION cl_khr_fp64 : enable\n";
        ss << "#elif defined(cl_amd_fp64)\n";
        ss << "#pragma OPENCL EXTENSION cl_amd_fp64 : enable\n";
        ss << "#endif\n";

        internal::set_cl_state_type<T>(ss);
        ss << "typedef ulong size_type;\n";
        ss << "#define Size " << size_ << "UL\n";
        ss << "#define Dim  " << dim_  << "U\n";
        ss << "#define Seed " << VSMC_SEED_TYPE::instance().get() << "UL\n";
        ss << "#include <vsmc/opencl/device_cl.h>\n";
        ss << source << '\n';
        VSMC_SEED_TYPE::instance().skip(
                static_cast<VSMC_SEED_TYPE::result_type>(size_));

        try {
            program_ = cl_manager_.create_program(ss.str());
            program_.build(cl_manager_.device_vec(), flags.c_str());
            program_.getBuildInfo(cl_manager_.device(), CL_PROGRAM_BUILD_LOG,
                    &build_log_);
            build_ = true;
        } catch (cl::Error &err) {
            std::string log;
            std::cerr << "===========================" << std::endl;
            std::cerr << "Error: vSMC: OpenCL program Build failed"
                << std::endl;
            std::cerr << err.err() << " : " << err.what() << std::endl;
            program_.getBuildInfo(cl_manager_.device(),
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
            program_.getBuildInfo(cl_manager_.device(), CL_PROGRAM_BUILD_LOG,
                    &build_log_);
            std::cerr << "===========================" << std::endl;
            std::cerr << "Build log:" << std::endl;
            std::cerr << "---------------------------" << std::endl;
            std::cerr << build_log_ << std::endl;
            std::cerr << "===========================" << std::endl;
            throw err;
        }

        kernel_copy_ = create_kernel("copy");
    }

    bool build () const
    {
        return build_;
    }

    int build_id () const
    {
        return build_id_;
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

    template<typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(copy);
        VSMC_RUNTIME_ASSERT((N == size_), "**StateCL::copy** SIZE MISMATCH");

        cl_manager_.template write_buffer<size_type>(
                copy_from_buffer_, size_, copy_from);
        kernel_copy_.setArg(0, state_buffer_);
        kernel_copy_.setArg(1, copy_from_buffer_);
        cl_manager_.run_kernel(kernel_copy_, size_, 0);
    }

    private :

    unsigned dim_;
    size_type size_;

    cl_manager_type &cl_manager_;

    cl::Program program_;
    cl::Kernel kernel_copy_;

    bool build_;
    int build_id_;
    std::string build_log_;

    cl::Buffer state_buffer_;
    cl::Buffer copy_from_buffer_;
}; // class StateCL

/// \brief Sampler<T>::init_type subtype
/// \ingroup CL
template <typename T>
class InitializeCL : public opencl::LocalSize
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (Particle<T> &particle, void *param)
    {
        VSMC_STATIC_ASSERT_STATE_CL_TYPE(T, InitializeCL);

        if (accept_host_.size() != particle.size()) {
            accept_host_.resize(particle.size());
            accept_buffer_ = particle.value().cl_manager().template
                create_buffer<cl_uint>(particle.size());
        }

        set_kernel(particle);
        initialize_param(particle, param);
        pre_processor(particle);
        particle.value().cl_manager().run_kernel(
                kernel_, particle.size(), local_size());
        post_processor(particle);

        particle.value().cl_manager().template read_buffer<cl_uint>(
                accept_buffer_, particle.size(), &accept_host_[0]);

        return std::accumulate(accept_host_.begin(), accept_host_.end(),
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

        if (build_id_ != particle.value().build_id()
                || kernel_name_ != kname) {
            build_id_ = particle.value().build_id();
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
        }

        kernel_.setArg(0, particle.value().state_buffer());
        kernel_.setArg(1, accept_buffer_);
    }

    protected :

    InitializeCL () : build_id_(-1) {}

    InitializeCL (const InitializeCL<T> &other) :
        build_id_(other.build_id_),
        kernel_(other.kernel_), kernel_name_(other.kernel_name_) {}

    InitializeCL<T> &operator= (const InitializeCL<T> &other)
    {
        if (this != &other) {
            build_id_ = other.build_id_;
            kernel_ = other.kernel_;
            kernel_name_ = other.kernel_name_;
        }

        return *this;
    }

    virtual ~InitializeCL () {}

    private :

    int build_id_;
    cl::Kernel kernel_;
    std::string kernel_name_;
    std::vector<cl_uint> accept_host_;
    cl::Buffer accept_buffer_;
}; // class InitializeCL

/// \brief Sampler<T>::move_type subtype
/// \ingroup CL
template <typename T>
class MoveCL : public opencl::LocalSize
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        VSMC_STATIC_ASSERT_STATE_CL_TYPE(T, MoveCL);

        if (accept_host_.size() != particle.size()) {
            accept_host_.resize(particle.size());
            accept_buffer_ = particle.value().cl_manager().template
                create_buffer<cl_uint>(particle.size());
        }

        set_kernel(iter, particle);
        pre_processor(iter, particle);
        particle.value().cl_manager().run_kernel(
                kernel_, particle.size(), local_size());
        post_processor(iter, particle);

        particle.value().cl_manager().template read_buffer<cl_uint>(
                accept_buffer_, particle.size(), &accept_host_[0]);

        return std::accumulate(accept_host_.begin(), accept_host_.end(),
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

        if (build_id_ != particle.value().build_id()
                || kernel_name_ != kname) {
            build_id_ = particle.value().build_id();
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
        }

        kernel_.setArg<cl_uint>(0, iter);
        kernel_.setArg(1, particle.value().state_buffer());
        kernel_.setArg(2, accept_buffer_);
    }

    protected :

    MoveCL () : build_id_(-1) {}

    MoveCL (const MoveCL<T> &other) :
        build_id_(other.build_id_),
        kernel_(other.kernel_), kernel_name_(other.kernel_name_) {}

    MoveCL<T> &operator= (const MoveCL<T> &other)
    {
        if (this != &other) {
            build_id_ = other.build_id_;
            kernel_ = other.kernel_;
            kernel_name_ = other.kernel_name_;
        }

        return *this;
    }

    virtual ~MoveCL () {}

    private :

    int build_id_;
    cl::Kernel kernel_;
    std::string kernel_name_;
    std::vector<cl_uint> accept_host_;
    cl::Buffer accept_buffer_;
}; // class MoveCL

/// \brief Monitor<T>::eval_type subtype
/// \ingroup CL
template <typename T>
class MonitorEvalCL : public opencl::LocalSize
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    void operator() (unsigned iter, unsigned dim, const Particle<T> &particle,
            double *res)
    {
        VSMC_STATIC_ASSERT_STATE_CL_TYPE(T, MonitorEvalCL);

        if (buffer_size_ != particle.size()) {
            buffer_ = particle.value().cl_manager().template
                create_buffer<typename T::state_type>(
                        particle.value().size() * dim);
            buffer_size_ = particle.size();
        }

        set_kernel(iter, dim, particle);
        pre_processor(iter, particle);
        particle.value().cl_manager().run_kernel(
                kernel_, particle.size(), local_size());
        particle.value().cl_manager().template
            read_buffer<typename T::state_type>(
                    buffer_, particle.value().size() * dim, res);
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

        if (build_id_ != particle.value().build_id()
                || kernel_name_ != kname) {
            build_id_ = particle.value().build_id();
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
        }

        kernel_.setArg<cl_uint>(0, iter);
        kernel_.setArg<cl_uint>(1, dim);
        kernel_.setArg(2, particle.value().state_buffer());
        kernel_.setArg(3, buffer_);
    }

    protected :

    MonitorEvalCL () : build_id_(-1), buffer_size_(0) {}

    MonitorEvalCL (const MonitorEvalCL<T> &other) :
        build_id_(other.build_id_),
        kernel_(other.kernel_), kernel_name_(other.kernel_name_),
        buffer_size_(other.buffer_size_), buffer_(other.buffer_) {}

    MonitorEvalCL<T> &operator= (const MonitorEvalCL<T> &other)
    {
        if (this != &other) {
            build_id_ = other.build_id_;
            kernel_ = other.kernel_;
            kernel_name_ = other.kernel_name_;
            buffer_size_ = other.buffer_size_;
            buffer_ = other.buffer_;
        }

        return *this;
    }

    virtual ~MonitorEvalCL () {}

    private :

    int build_id_;
    cl::Kernel kernel_;
    std::string kernel_name_;
    size_type buffer_size_;
    cl::Buffer buffer_;
}; // class MonitorEvalCL

/// \brief Path<T>::eval_type subtype
/// \ingroup CL
template <typename T>
class PathEvalCL : public opencl::LocalSize
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    double operator() (unsigned iter, const Particle<T> &particle,
        double *res)
    {
        VSMC_STATIC_ASSERT_STATE_CL_TYPE(T, PathEvalCL);

        if (buffer_size_ != particle.size()) {
            buffer_ = particle.value().cl_manager().template
                create_buffer<typename T::state_type>(particle.value().size());
            buffer_size_ = particle.size();
        }

        set_kernel(iter, particle);
        pre_processor(iter, particle);
        particle.value().cl_manager().run_kernel(
                kernel_, particle.size(), local_size());
        particle.value().cl_manager().template
            read_buffer<typename T::state_type>(
                    buffer_, particle.value().size(), res);
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

        if (build_id_ != particle.value().build_id()
                || kernel_name_ != kname) {
            build_id_ = particle.value().build_id();
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
        }

        kernel_.setArg<cl_uint>(0, iter);
        kernel_.setArg(1, particle.value().state_buffer());
        kernel_.setArg(2, buffer_);
    }

    protected :

    PathEvalCL () : build_id_(-1), buffer_size_(0) {}

    PathEvalCL (const PathEvalCL<T> &other) :
        build_id_(other.build_id_),
        kernel_(other.kernel_), kernel_name_(other.kernel_name_),
        buffer_size_(other.buffer_size_), buffer_(other.buffer_) {}

    PathEvalCL<T> &operator= (const PathEvalCL<T> &other)
    {
        if (this != &other) {
            build_id_ = other.build_id_;
            kernel_ = other.kernel_;
            kernel_name_ = other.kernel_name_;
            buffer_size_ = other.buffer_size_;
            buffer_ = other.buffer_;
        }

        return *this;
    }

    virtual ~PathEvalCL () {}

    private :

    int build_id_;
    cl::Kernel kernel_;
    std::string kernel_name_;
    size_type buffer_size_;
    cl::Buffer buffer_;
}; // class PathEvalCL

} // namespace vsmc

#endif // VSMC_OPENCL_PARALLEL_CL_HPP

#ifndef VSMC_OPENCL_BACKEND_CL_HPP
#define VSMC_OPENCL_BACKEND_CL_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/cl_configure.hpp>
#include <vsmc/utility/cl_manager.hpp>
#include <vsmc/utility/seed.hpp>

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

/// \brief Particle::value_type subtype using OpenCL
/// \ingroup OpenCL
///
/// \details
/// Currently the template parameter `T` is limited to `cl_float` or
/// `cl_double`.
template <std::size_t Dim, typename T, typename ID>
class StateCL
{
    public :

    typedef cl_ulong size_type;
    typedef T state_type;
    typedef CLManager<ID> manager_type;

    explicit StateCL (size_type N) :
        dim_(Dim == Dynamic ? 1 : Dim), size_(N), build_(false), build_id_(0),
        state_buffer_(manager().template create_buffer<state_type>(
                dim_ * size_)),
        copy_from_buffer_(manager().template create_buffer<size_type>(size_))
    {VSMC_STATIC_ASSERT_STATE_CL_VALUE_TYPE(T);}

    StateCL (const StateCL<Dim, T, ID> &other) :
        dim_(other.dim_), size_(other.size_),
        program_(other.program_), kernel_copy_(other.kernel_copy_),
        build_(other.build_), build_id_(0),
        state_buffer_(manager().template create_buffer<state_type>(
                dim_ * size_)),
        copy_from_buffer_(manager().template create_buffer<size_type>(size_))
    {
        manager().template copy_buffer<state_type>(
                other.state_buffer_, dim_ * size_, state_buffer_);
    }

    StateCL<Dim, T, ID> &operator= (const StateCL<Dim, T, ID> &other)
    {
        if (this != &other) {
            dim_         = other.dim_;
            size_        = other.size_;
            program_     = other.program_;
            kernel_copy_ = other.kernel_copy_;
            build_       = other.build_;
            build_id_    = 0;

            state_buffer_ =
                manager().template create_buffer<state_type>(dim_ * size_);
            copy_from_buffer_ =
                manager().template create_buffer<size_type>(size_);
            manager().template copy_buffer<state_type>(
                    other.state_buffer_, dim_ * size_, state_buffer_);
        }

        return *this;
    }

    std::size_t dim () const {return dim_;}

    void resize_dim (std::size_t dim)
    {
        VSMC_STATIC_ASSERT_DYNAMIC_DIM_RESIZE;

        state_buffer_ =
            manager().template create_buffer<state_type>(dim * size_);
        dim_ = dim;
    }

    size_type size () const {return size_;}

    /// \brief The instance of the CLManager signleton associated with this
    /// value collcection
    static manager_type &manager () {return manager_type::instance();}

    /// \brief The OpenCL buffer that stores the state values
    const cl::Buffer &state_buffer () const {return state_buffer_;}

    /// \brief The OpenCL program associated with this value collection
    const cl::Program &program () const {return program_;}

    /// \brief Build the OpenCL program from source
    ///
    /// \param source The source of the program
    /// \param flags The OpenCL compiler flags, e.g., `-I`
    ///
    /// \details
    /// Note that a few macros and headers are included before the user
    /// supplied `source`. Say the template parameter `T` of this class is set
    /// to `cl_double`, `Dim` set to `4`, and there are 1000 particles, then
    /// the complete source, which acutally get compiled looks like the
    /// following
    /// \code
    /// #if defined(cl_khr_fp64)
    /// #pragma OPENCL EXTENSION cl_khr_fp64 : enable
    /// #elif defined(cl_amd_fp64)
    /// #pragma OPENCL EXTENSION cl_amd_fp64 : enable
    /// #endif
    ///
    /// typedef float state_type;
    /// #define VSMC_STATE_TYPE_IS_FLOAT  1
    /// #define VSMC_STATE_TYPE_IS_DOUBLE 0
    ///
    /// typedef ulong size_type;
    /// #define Size 1000UL;
    /// #define Dim 4UL;
    /// #define Seed 101UL;
    /// // The actual number if VSMC_SEED_TYPE::instance().get()
    ///
    /// #include <vsmc/opencl/device.h>
    ///
    /// // ... User source
    /// \endcode
    /// In summary, macros (and thus C compile time constant) for the number of
    /// particles, the dimension of the states, a seed are defined. A typedef
    /// `size_type` and `state_type` are defined. The double precision
    /// extension are enabled if avaialble. And two macros
    /// `VSMC_STATE_TYPE_IS_FLOAT` and `VSMC_STATE_TYPE_IS_DOUBLE` are provided
    /// for conditional compile when the precison matters.
    ///
    /// After build, `VSMC_SEED_TYPE::instance().skip(N)` is called with `N`
    /// being the nubmer of particles.
    ///
    /// \note
    /// This function may throw if the build attempt fails.
    void build (const std::string &source,
            const std::string &flags = std::string())
    {
        ++build_id_;

        std::stringstream ss;

        ss << "#if defined(cl_khr_fp64)\n";
        ss << "#pragma OPENCL EXTENSION cl_khr_fp64 : enable\n";
        ss << "#elif defined(cl_amd_fp64)\n";
        ss << "#pragma OPENCL EXTENSION cl_amd_fp64 : enable\n";
        ss << "#endif\n";

        internal::set_cl_state_type<T>(ss);
        ss << "typedef ulong size_type;\n";
        ss << "#define Size " << size_ << "UL\n";
        ss << "#define Dim  " << dim_  << "UL\n";
        ss << "#define Seed " << VSMC_SEED_TYPE::instance().get() << "UL\n";
        ss << "#include <vsmc/opencl/device.h>\n";
        ss << source << '\n';
        VSMC_SEED_TYPE::instance().skip(
                static_cast<VSMC_SEED_TYPE::result_type>(size_));

        try {
            program_ = manager().create_program(ss.str());
            program_.build(manager().device_vec(), flags.c_str());
            program_.getInfo(CL_PROGRAM_SOURCE, &build_source_);
            program_.getBuildInfo(manager().device(),
                    CL_PROGRAM_BUILD_OPTIONS, &build_options_);
            program_.getBuildInfo(manager().device(), CL_PROGRAM_BUILD_LOG,
                    &build_log_);
            build_ = true;
        } catch (cl::Error &err) {
            program_.getInfo(CL_PROGRAM_SOURCE, &build_source_);
            program_.getBuildInfo(manager().device(),
                    CL_PROGRAM_BUILD_OPTIONS, &build_options_);
            program_.getBuildInfo(manager().device(), CL_PROGRAM_BUILD_LOG,
                    &build_log_);
            std::stringstream log_ss;
            log_ss << "====================================================\n";
            log_ss << "Error: vSMC: OpenCL program Build failed\n";
            log_ss << err.err() << " : " << err.what() << '\n';
            log_ss << "====================================================\n";
            log_ss << "Build options:\n";
            log_ss << "----------------------------------------------------\n";
            log_ss << build_options_ << '\n';
            log_ss << "====================================================\n";
            log_ss << "Build source:\n";
            log_ss << "----------------------------------------------------\n";
            log_ss << build_source_ << '\n';
            log_ss << "====================================================\n";
            log_ss << "Build log:\n";
            log_ss << "----------------------------------------------------\n";
            log_ss << build_log_ << '\n';
            log_ss << "====================================================\n";
            std::fprintf(stderr, "%s", log_ss.str().c_str());
            throw err;
        }

        kernel_copy_ = create_kernel("copy");
    }

    /// \brief Whether the last attempted building success
    bool build () const {return build_;}

    /// \brief The build id of the last attempted of building
    ///
    /// \details
    /// This function returns a non-decreasing sequence of integers
    int build_id () const {return build_id_;}

    /// \brief The source of the program of the last attempted building
    std::string build_source () const {return build_source_;}

    /// \brief The build options of the program of the last attempted building
    std::string build_options () const {return build_options_;}

    /// \brief The log of the program of the last attempted building
    std::string build_log () const {return build_log_;}

    /// \brief Create kernel with the current program
    ///
    /// \note If build() does not return `true`, then calling this is an error
    cl::Kernel create_kernel (const std::string &name) const
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(create_kernel);

        return cl::Kernel(program_, name.c_str());
    }

    template<typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(copy);
        VSMC_RUNTIME_ASSERT_STATE_COPY_SIZE_MISMATCH(CL);

        manager().template write_buffer<size_type>(
                copy_from_buffer_, size_, copy_from);
        kernel_copy_.setArg(0, state_buffer_);
        kernel_copy_.setArg(1, copy_from_buffer_);
        manager().run_kernel(kernel_copy_, size_, 0);
    }

    template <typename OutputStream>
    OutputStream &print (OutputStream &os, std::size_t iter = 0,
            char sepchar = ' ', char eolchar = '\n') const
    {
        state_host_.resize(dim_ * size_);
        manager().template read_buffer<state_type>(
                state_buffer_, dim_ * size_, &state_host_[0]);
        for (size_type i = 0; i != size_; ++i) {
            os << iter << sepchar;
            for (std::size_t d = 0; d != this->dim() - 1; ++d)
                os << state_host_[i * dim_ + d] << sepchar;
            os << state_host_[i * dim_ + dim_ - 1] << eolchar;
        }

        return os;

    }

    private :

    std::size_t dim_;
    size_type size_;

    cl::Program program_;
    cl::Kernel kernel_copy_;

    bool build_;
    int build_id_;
    std::string build_source_;
    std::string build_options_;
    std::string build_log_;

    cl::Buffer state_buffer_;
    cl::Buffer copy_from_buffer_;

    mutable std::vector<state_type> state_host_;
}; // class StateCL

/// \brief Sampler<T>::init_type subtype using OpenCL
/// \ingroup OpenCL
///
/// \details
/// Kernel requirement (`initialize_state`)
/// \code
/// __kernel
/// void kern (__global state_type *state, __global ulong *accept);
/// \endcode
/// - Kernels can have additonal arguments and set by the user in
/// `pre_processor`.
/// - `state` has size `N * Dim` where `accept` has size `N`.
/// - The declaration does not have to much this, but the first arguments will
/// be set by InitializeCL::opeartor(). For example
/// \code
/// type struct {
///     state_type v1;
///     state_type v2;
///     // ...
///     state_type vDim;
/// } param;
/// __kernel
/// void kern (__global param *state, __global ulong *accept);
/// \endcode
/// is also acceptable, but now `state` has to be treat as a length `N` array.
/// In summary, on the host side, it is a `cl::Buffer` object being passed to
/// the kernel, which is not much unlike `void *` pointer.
template <typename T, typename>
class InitializeCL : public CLConfigure
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    std::size_t operator() (Particle<T> &particle, void *param)
    {
        VSMC_STATIC_ASSERT_STATE_CL_TYPE(T, InitializeCL);

        if (accept_host_.size() != particle.size()) {
            accept_host_.resize(particle.size());
            accept_buffer_ = particle.value().manager().template
                create_buffer<cl_ulong>(particle.size());
        }

        set_kernel(particle);
        initialize_param(particle, param);
        pre_processor(particle);
        particle.value().manager().run_kernel(
                kernel_, particle.size(), local_size());
        post_processor(particle);

        particle.value().manager().template read_buffer<cl_ulong>(
                accept_buffer_, particle.size(), &accept_host_[0]);

        cl_ulong acc = 0;
        for (size_type i = 0; i != particle.size(); ++i)
            acc += accept_host_[i];

        return acc;
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

    const cl::Kernel &kernel () const
    {
        return kernel_;
    }

    cl::Kernel &kernel ()
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
            set_preferred_local_size(
                    kernel_, particle.value().manager().device());
        }

        kernel_.setArg(0, particle.value().state_buffer());
        kernel_.setArg(1, accept_buffer_);
    }

    virtual void initialize_param (Particle<T> &, void *) {}
    virtual void initialize_state (std::string &) = 0;
    virtual void pre_processor (Particle<T> &) {}
    virtual void post_processor (Particle<T> &) {}

    private :

    int build_id_;
    cl::Kernel kernel_;
    std::string kernel_name_;
    std::vector<cl_ulong> accept_host_;
    cl::Buffer accept_buffer_;
}; // class InitializeCL

/// \brief Sampler<T>::move_type subtype using OpenCL
/// \ingroup OpenCL
///
/// \details
/// Kernel requirement (`move_state`)
/// \code
/// __kernel
/// void kern (ulong iter, __global state_type *state, __global ulong *accept);
/// \endcode
template <typename T, typename>
class MoveCL : public CLConfigure
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    std::size_t operator() (std::size_t iter, Particle<T> &particle)
    {
        VSMC_STATIC_ASSERT_STATE_CL_TYPE(T, MoveCL);

        if (accept_host_.size() != particle.size()) {
            accept_host_.resize(particle.size());
            accept_buffer_ = particle.value().manager().template
                create_buffer<cl_ulong>(particle.size());
        }

        set_kernel(iter, particle);
        pre_processor(iter, particle);
        particle.value().manager().run_kernel(
                kernel_, particle.size(), local_size());
        post_processor(iter, particle);

        particle.value().manager().template read_buffer<cl_ulong>(
                accept_buffer_, particle.size(), &accept_host_[0]);

        cl_ulong acc = 0;
        for (size_type i = 0; i != particle.size(); ++i)
            acc += accept_host_[i];

        return acc;
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

    const cl::Kernel &kernel () const
    {
        return kernel_;
    }

    cl::Kernel &kernel ()
    {
        return kernel_;
    }

    void set_kernel (std::size_t iter, const Particle<T> &particle)
    {
        std::string kname;
        move_state(iter, kname);

        if (build_id_ != particle.value().build_id()
                || kernel_name_ != kname) {
            build_id_ = particle.value().build_id();
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
            set_preferred_local_size(
                    kernel_, particle.value().manager().device());
        }

        kernel_.setArg<cl_ulong>(0, static_cast<cl_ulong>(iter));
        kernel_.setArg(1, particle.value().state_buffer());
        kernel_.setArg(2, accept_buffer_);
    }

    virtual void move_state (std::size_t, std::string &) = 0;
    virtual void pre_processor (std::size_t, Particle<T> &) {}
    virtual void post_processor (std::size_t, Particle<T> &) {}

    private :

    int build_id_;
    cl::Kernel kernel_;
    std::string kernel_name_;
    std::vector<cl_ulong> accept_host_;
    cl::Buffer accept_buffer_;
}; // class MoveCL

/// \brief Monitor<T>::eval_type subtype using OpenCL
/// \ingroup OpenCL
///
/// \details
/// Kernel requirement (`monitor_state`)
/// \code
/// __kernel
/// void kern (ulong iter, ulong dim, __global state_type *state,
///            __global state_type *res);
/// \endcode
template <typename T, typename>
class MonitorEvalCL : public CLConfigure
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    void operator() (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res)
    {
        VSMC_STATIC_ASSERT_STATE_CL_TYPE(T, MonitorEvalCL);

        if (buffer_size_ != particle.size()) {
            buffer_ = particle.value().manager().template
                create_buffer<typename T::state_type>(
                        particle.value().size() * dim);
            buffer_size_ = particle.size();
        }

        set_kernel(iter, dim, particle);
        pre_processor(iter, particle);
        particle.value().manager().run_kernel(
                kernel_, particle.size(), local_size());
        particle.value().manager().template
            read_buffer<typename T::state_type>(
                    buffer_, particle.value().size() * dim, res);
        post_processor(iter, particle);
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

    const cl::Kernel &kernel () const
    {
        return kernel_;
    }

    cl::Kernel &kernel ()
    {
        return kernel_;
    }

    void set_kernel (std::size_t iter, std::size_t dim,
            const Particle<T> &particle)
    {
        std::string kname;
        monitor_state(iter, kname);

        if (build_id_ != particle.value().build_id()
                || kernel_name_ != kname) {
            build_id_ = particle.value().build_id();
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
            set_preferred_local_size(
                    kernel_, particle.value().manager().device());
        }

        kernel_.setArg<cl_ulong>(0, static_cast<cl_ulong>(iter));
        kernel_.setArg<cl_ulong>(1, static_cast<cl_ulong>(dim));
        kernel_.setArg(2, particle.value().state_buffer());
        kernel_.setArg(3, buffer_);
    }

    virtual void monitor_state (std::size_t, std::string &) = 0;
    virtual void pre_processor (std::size_t, const Particle<T> &) {}
    virtual void post_processor (std::size_t, const Particle<T> &) {}

    private :

    int build_id_;
    cl::Kernel kernel_;
    std::string kernel_name_;
    size_type buffer_size_;
    cl::Buffer buffer_;
}; // class MonitorEvalCL

/// \brief Path<T>::eval_type subtype using OpenCL
/// \ingroup OpenCL
///
/// \details
/// Kernel requirement (`path_state`)
/// \code
/// __kernel
/// void kern (ulong iter, __global state_type *state,
///            __global state_type *res);
/// \endcode
template <typename T, typename>
class PathEvalCL : public CLConfigure
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    double operator() (std::size_t iter, const Particle<T> &particle,
        double *res)
    {
        VSMC_STATIC_ASSERT_STATE_CL_TYPE(T, PathEvalCL);

        if (buffer_size_ != particle.size()) {
            buffer_ = particle.value().manager().template
                create_buffer<typename T::state_type>(particle.value().size());
            buffer_size_ = particle.size();
        }

        set_kernel(iter, particle);
        pre_processor(iter, particle);
        particle.value().manager().run_kernel(
                kernel_, particle.size(), local_size());
        particle.value().manager().template
            read_buffer<typename T::state_type>(
                    buffer_, particle.value().size(), res);
        post_processor(iter, particle);

        return this->path_grid(iter, particle);
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

    const cl::Kernel &kernel () const
    {
        return kernel_;
    }

    cl::Kernel &kernel ()
    {
        return kernel_;
    }

    void set_kernel (std::size_t iter, const Particle<T> &particle)
    {
        std::string kname;
        path_state(iter, kname);

        if (build_id_ != particle.value().build_id()
                || kernel_name_ != kname) {
            build_id_ = particle.value().build_id();
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
            set_preferred_local_size(
                    kernel_, particle.value().manager().device());
        }

        kernel_.setArg<cl_ulong>(0, static_cast<cl_ulong>(iter));
        kernel_.setArg(1, particle.value().state_buffer());
        kernel_.setArg(2, buffer_);
    }

    virtual void path_state (std::size_t, std::string &) = 0;
    virtual double path_grid (std::size_t, const Particle<T> &) = 0;
    virtual void pre_processor (std::size_t, const Particle<T> &) {}
    virtual void post_processor (std::size_t, const Particle<T> &) {}

    private :

    int build_id_;
    cl::Kernel kernel_;
    std::string kernel_name_;
    size_type buffer_size_;
    cl::Buffer buffer_;
}; // class PathEvalCL

} // namespace vsmc

#endif // VSMC_OPENCL_BACKEND_CL_HPP

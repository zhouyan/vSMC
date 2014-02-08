#ifndef VSMC_OPENCL_BACKEND_CL_HPP
#define VSMC_OPENCL_BACKEND_CL_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/cl_manager.hpp>
#include <vsmc/opencl/cl_manip.hpp>
#include <vsmc/rng/seed.hpp>

#define VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_BUILD(func) \
    VSMC_RUNTIME_ASSERT((build()),                                           \
            ("**StateCL::"#func"** CAN ONLY BE CALLED AFTER true "           \
             "**StateCL::build**"));

#define VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_STATE_SIZE(state_size) \
    VSMC_RUNTIME_ASSERT((state_size >= 1), ("STATE SIZE IS LESS THAN 1"))

#define VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_COPY_SIZE_MISMATCH \
    VSMC_RUNTIME_ASSERT((N == size_), ("**StateCL::copy** SIZE MISMATCH"))


namespace vsmc {

namespace internal {

template <typename> void set_cl_fp_type (std::stringstream &);

template<>
inline void set_cl_fp_type<cl_float>(std::stringstream &ss)
{
    ss << "typedef float fp_type;\n";
    ss << "#define VSMC_FP_TYPE_IS_FLOAT  1\n";
    ss << "#define VSMC_FP_TYPE_IS_DOUBLE 0\n";
}

template<>
inline void set_cl_fp_type<cl_double>(std::stringstream &ss)
{
    ss << "#if defined(cl_khr_fp64)\n";
    ss << "#pragma OPENCL EXTENSION cl_khr_fp64 : enable\n";
    ss << "#elif defined(cl_amd_fp64)\n";
    ss << "#pragma OPENCL EXTENSION cl_amd_fp64 : enable\n";
    ss << "#endif\n";

    ss << "typedef double fp_type;\n";
    ss << "#define VSMC_FP_TYPE_IS_FLOAT  0\n";
    ss << "#define VSMC_FP_TYPE_IS_DOUBLE 1\n";
}

} // namespace vsmc::internal

namespace traits {

template <typename D>
struct IsDerivedFromStateCLImpl
{
    private :

    struct char2 {char c1; char c2;};

    template <std::size_t Dim, typename T, typename ID>
    static char test (const StateCL<Dim, T, ID> *);
    static char2 test (...);

    public :

   enum {value = sizeof(test(static_cast<const D *>(0))) == sizeof(char)};
};

template <typename D>
struct IsDerivedFromStateCL :
    public cxx11::integral_constant<bool, IsDerivedFromStateCLImpl<D>::value>{};

} // namespace vsmc::traits

template <typename, typename Derived = NullType> class InitializeCL;
template <typename, typename Derived = NullType> class MoveCL;
template <typename, typename Derived = NullType> class MonitorEvalCL;
template <typename, typename Derived = NullType> class PathEvalCL;

/// \brief Configure OpenCL runtime behavior (used by MoveCL etc)
/// \ingroup OpenCL
class ConfigureCL
{
    public :

    ConfigureCL () : local_size_(0) {}

    std::size_t local_size () const {return local_size_;}

    void local_size (std::size_t new_size) {local_size_ = new_size;}

    void local_size (std::size_t N,
            const cl::Kernel &kern, const cl::Device &dev)
    {
        std::size_t global_size;
        cl_preferred_work_size(N, kern, dev, global_size, local_size_);
    };

    private :

    std::size_t local_size_;
}; // class ConfigureCL

/// \brief Particle::value_type subtype using OpenCL
/// \ingroup OpenCL
template <std::size_t StateSize, typename FPType, typename ID>
class StateCL
{
    public :

    typedef cl_ulong size_type;
    typedef FPType fp_type;
    typedef CLManager<ID> manager_type;

    explicit StateCL (size_type N) :
        state_size_(StateSize == Dynamic ? 1 : StateSize),
        size_(N), build_(false), build_id_(0),
        state_buffer_(manager().template create_buffer<char>(
                state_size_ * size_)),
        copy_from_buffer_(manager().template create_buffer<size_type>(size_))
        {}

    StateCL (const StateCL<StateSize, FPType, ID> &other) :
        state_size_(other.state_size_), size_(other.size_),
        program_(other.program_), kernel_copy_(other.kernel_copy_),
        configure_copy_(other.configure_copy_), build_(other.build_),
        build_id_(other.build_id_), build_source_(other.build_source_),
        build_options_(other.build_options_),
        state_buffer_(manager().template create_buffer<char>(
                state_size_ * size_)),
        copy_from_buffer_(manager().template create_buffer<size_type>(size_))
    {
        manager().template copy_buffer<char>(
                other.state_buffer_, state_size_ * size_, state_buffer_);
    }

    StateCL<StateSize, FPType, ID> &operator=
        (const StateCL<StateSize, FPType, ID> &other)
    {
        if (this != &other) {
            state_size_     = other.state_size_;
            size_           = other.size_;
            program_        = other.program_;
            kernel_copy_    = other.kernel_copy_;
            configure_copy_ = other.configure_copy_;
            build_          = other.build_;
            build_id_       = other.build_id_;
            build_source_   = other.build_source_;
            build_options_  = other.build_options_;

            state_buffer_ =
                manager().template create_buffer<char>(state_size_ * size_);
            copy_from_buffer_ =
                manager().template create_buffer<size_type>(size_);
            manager().template copy_buffer<char>(
                    other.state_buffer_, state_size_ * size_, state_buffer_);
        }

        return *this;
    }

    size_type size () const {return size_;}

    std::size_t state_size () const {return state_size_;}

    void resize_state (std::size_t state_size)
    {
        VSMC_STATIC_ASSERT_DYNAMIC_STATE_SIZE_RESIZE(StateSize);
        VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_STATE_SIZE(state_size);

        state_buffer_ =
            manager().template create_buffer<char>(state_size * size_);
        state_size_ = state_size;
    }

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
    /// supplied `source`. Say the template parameter `StateSize == 4`,
    /// `FPType` of this class is set to `cl_double`, and there are `1000`
    /// particles, then the complete source, which acutally get compiled looks
    /// like the following
    /// \code
    /// typedef float fp_type;
    /// #define VSMC_FP_TYPE_TYPE_IS_FLOAT  1
    /// #define VSMC_FP_TYPE_TYPE_IS_DOUBLE 0
    ///
    /// #if defined(cl_khr_fp64)
    /// #pragma OPENCL EXTENSION cl_khr_fp64 : enable
    /// #elif defined(cl_amd_fp64)
    /// #pragma OPENCL EXTENSION cl_amd_fp64 : enable
    /// #endif
    ///
    /// typedef ulong size_type;
    /// #define Size 1000UL;
    /// #define StateSize 4UL;
    /// #define Seed 101UL;
    /// // The actual seed is vsmc::Seed::instance().get()
    ///
    /// #include <vsmc/opencl/internal/device.h>
    ///
    /// // ... User source, passed by the source argument
    /// \endcode
    /// After build, `vsmc::Seed::instance().skip(N)` is called with `N`
    /// being the nubmer of particles.
    void build (const std::string &source,
            const std::string &flags = std::string())
    {
        VSMC_STATIC_ASSERT_STATE_CL_FP_TYPE(fp_type);

        ++build_id_;

        std::stringstream ss;
        internal::set_cl_fp_type<fp_type>(ss);

        ss << "typedef ulong size_type;\n";
        ss << "#define Size " << size_ << "UL\n";
        ss << "#define StateSize " << state_size_ << "UL\n";
        ss << "#define Seed " << Seed::instance().get() << "UL\n";

        ss << "typedef struct {\n";
        for (std::size_t i = 0; i != state_size_; ++i)
            ss << "char c" << i << ";\n";
        ss << "} copy_state_struct;\n";

        ss << "#include <vsmc/opencl/internal/device.h>\n";
        ss << source << '\n';
        Seed::instance().skip(static_cast<Seed::result_type>(size_));

        try {
            program_ = manager().create_program(ss.str());
            program_.build(manager().device_vec(), flags.c_str());
            program_.getInfo(CL_PROGRAM_SOURCE, &build_source_);
            program_.getBuildInfo(manager().device(),
                    CL_PROGRAM_BUILD_OPTIONS, &build_options_);
            build_ = true;
        } catch (...) {
            manager().print_build_log(program_, std::cout);
            throw;
        }

        kernel_copy_ = create_kernel("copy");
        configure_copy_.local_size(size_, kernel_copy_, manager().device());
    }

    /// \brief Whether the last attempted building success
    bool build () const {return build_;}

    /// \brief The build id of the last attempted of building
    ///
    /// \details
    /// This function returns a non-decreasing sequence of integers
    int build_id () const {return build_id_;}

    /// \brief The source of the program of the last attempted building
    const char *build_source () const {return build_source_.c_str();}

    /// \brief The build options of the program of the last attempted building
    const char *build_options () const {return build_options_.c_str();}

    /// \brief Create kernel with the current program
    ///
    /// \details
    /// If build() does not return `true`, then calling this is an error
    cl::Kernel create_kernel (const std::string &name) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_BUILD(create_kernel);

        return cl::Kernel(program_, name.c_str());
    }

    template<typename IntType>
    void copy (size_type N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_BUILD(copy);
        VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_COPY_SIZE_MISMATCH;

        manager().template write_buffer<size_type>(
                copy_from_buffer_, size_, copy_from);
        kernel_copy_.setArg(0, state_buffer_);
        kernel_copy_.setArg(1, copy_from_buffer_);
        manager().run_kernel(
                kernel_copy_, size_, configure_copy_.local_size());
    }

    ConfigureCL &configure_copy () {return configure_copy_;}

    const ConfigureCL &configure_copy () const {return configure_copy_;}

    private :

    std::size_t state_size_;
    size_type size_;

    cl::Program program_;
    cl::Kernel kernel_copy_;
    ConfigureCL configure_copy_;

    bool build_;
    int build_id_;
    std::string build_source_;
    std::string build_options_;

    cl::Buffer state_buffer_;
    cl::Buffer copy_from_buffer_;
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
/// - `state` has size `N * StateSize` where `accept` has size `N`.
/// - The declaration does not have to much this, but the first arguments will
/// be set by InitializeCL::opeartor(). For example
/// \code
/// type struct {
///     state_type v1;
///     state_type v2;
///     // ...
///     state_type vDim; // vDim = StateSize / state_type
/// } param;
/// __kernel
/// void kern (__global param *state, __global ulong *accept);
/// \endcode
/// is also acceptable, but now `state` has to be treat as a length `N` array.
/// In summary, on the host side, it is a `cl::Buffer` object being passed to
/// the kernel, which is not much unlike `void *` pointer.
template <typename T, typename>
class InitializeCL
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    /// \brief The index offset of additional kernel arguments set by the user
    ///
    /// \details
    /// The first user supplied additional argument shall have index
    /// `kernel_args_offset`
    const static std::size_t kernel_args_offset = 2;

    std::size_t operator() (Particle<T> &particle, void *param)
    {
        VSMC_STATIC_ASSERT_STATE_CL_TYPE(T, InitializeCL);

        std::size_t bsize = particle.size();
        if (accept_host_.size() != bsize) {
            accept_host_.resize(bsize);
            accept_buffer_ = particle.value().manager().template
                create_buffer<cl_ulong>(bsize);
        }

        set_kernel(particle);
        initialize_param(particle, param);
        pre_processor(particle);
        particle.value().manager().run_kernel(
                kernel_, particle.size(), configure_.local_size());
        post_processor(particle);

        particle.value().manager().template read_buffer<cl_ulong>(
                accept_buffer_, particle.size(), &accept_host_[0]);

        cl_ulong acc = 0;
        for (size_type i = 0; i != particle.size(); ++i)
            acc += accept_host_[i];

        return acc;
    }

    virtual void initialize_param (Particle<T> &, void *) {}
    virtual void initialize_state (std::string &) = 0;
    virtual void pre_processor (Particle<T> &) {}
    virtual void post_processor (Particle<T> &) {}

    protected :

    InitializeCL () : build_id_(-1) {}

    InitializeCL (const InitializeCL<T> &other) :
        configure_(other.configure_), build_id_(other.build_id_),
        kernel_(other.kernel_), kernel_name_(other.kernel_name_) {}

    InitializeCL<T> &operator= (const InitializeCL<T> &other)
    {
        configure_   = other.configure_;
        build_id_    = other.build_id_;
        kernel_      = other.kernel_;
        kernel_name_ = other.kernel_name_;

        return *this;
    }

    virtual ~InitializeCL () {}

    ConfigureCL &configure () {return configure_;}

    const ConfigureCL &configure () const {return configure_;}

    cl::Kernel &kernel () {return kernel_;}

    const cl::Kernel &kernel () const {return kernel_;}

    void set_kernel (const Particle<T> &particle)
    {
        std::string kname;
        initialize_state(kname);

        if (build_id_ != particle.value().build_id()
                || kernel_name_ != kname) {
            build_id_ = particle.value().build_id();
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
            configure_.local_size(particle.size(),
                    kernel_, particle.value().manager().device());
        }

        kernel_.setArg(0, particle.value().state_buffer());
        kernel_.setArg(1, accept_buffer_);
    }

    private :

    ConfigureCL configure_;
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
class MoveCL
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    /// \brief The index offset of additional kernel arguments set by the user
    ///
    /// \details
    /// The first user supplied additional argument shall have index
    /// `kernel_args_offset`
    const static std::size_t kernel_args_offset = 3;

    std::size_t operator() (std::size_t iter, Particle<T> &particle)
    {
        VSMC_STATIC_ASSERT_STATE_CL_TYPE(T, MoveCL);

        std::size_t bsize = static_cast<std::size_t>(particle.size());
        if (accept_host_.size() != bsize) {
            accept_host_.resize(bsize);
            accept_buffer_ = particle.value().manager().template
                create_buffer<cl_ulong>(bsize);
        }

        set_kernel(iter, particle);
        pre_processor(iter, particle);
        particle.value().manager().run_kernel(
                kernel_, particle.size(), configure_.local_size());
        post_processor(iter, particle);

        particle.value().manager().template read_buffer<cl_ulong>(
                accept_buffer_, particle.size(), &accept_host_[0]);

        cl_ulong acc = 0;
        for (size_type i = 0; i != particle.size(); ++i)
            acc += accept_host_[i];

        return acc;
    }

    virtual void move_state (std::size_t, std::string &) = 0;
    virtual void pre_processor (std::size_t, Particle<T> &) {}
    virtual void post_processor (std::size_t, Particle<T> &) {}

    protected :

    MoveCL () : build_id_(-1) {}

    MoveCL (const MoveCL<T> &other) :
        configure_(other.configure_), build_id_(other.build_id_),
        kernel_(other.kernel_), kernel_name_(other.kernel_name_) {}

    MoveCL<T> &operator= (const MoveCL<T> &other)
    {
        configure_   = other.configure_;
        build_id_    = other.build_id_;
        kernel_      = other.kernel_;
        kernel_name_ = other.kernel_name_;

        return *this;
    }

    virtual ~MoveCL () {}

    ConfigureCL &configure () {return configure_;}

    const ConfigureCL &configure () const {return configure_;}

    cl::Kernel &kernel () {return kernel_;}

    const cl::Kernel &kernel () const {return kernel_;}

    void set_kernel (std::size_t iter, const Particle<T> &particle)
    {
        std::string kname;
        move_state(iter, kname);

        if (build_id_ != particle.value().build_id()
                || kernel_name_ != kname) {
            build_id_ = particle.value().build_id();
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
            configure_.local_size(particle.size(),
                    kernel_, particle.value().manager().device());
        }

        kernel_.setArg<cl_ulong>(0, static_cast<cl_ulong>(iter));
        kernel_.setArg(1, particle.value().state_buffer());
        kernel_.setArg(2, accept_buffer_);
    }

    private :

    ConfigureCL configure_;
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
///            __global fp_type *res);
/// \endcode
template <typename T, typename>
class MonitorEvalCL
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    /// \brief The index offset of additional kernel arguments set by the user
    ///
    /// \details
    /// The first user supplied additional argument shall have index
    /// `kernel_args_offset`
    const static std::size_t kernel_args_offset = 4;

    void operator() (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res)
    {
        VSMC_STATIC_ASSERT_STATE_CL_TYPE(T, MonitorEvalCL);

        std::size_t bsize = static_cast<std::size_t>(particle.size()) * dim;
        if (buffer_size_ != bsize) {
            buffer_ = particle.value().manager().template
                create_buffer<typename T::fp_type>(bsize);
            buffer_size_ = bsize;
        }

        set_kernel(iter, dim, particle);
        pre_processor(iter, particle);
        particle.value().manager().run_kernel(
                kernel_, particle.size(), configure_.local_size());
        particle.value().manager().template
            read_buffer<typename T::fp_type>(
                    buffer_, particle.value().size() * dim, res);
        post_processor(iter, particle);
    }

    virtual void monitor_state (std::size_t, std::string &) = 0;
    virtual void pre_processor (std::size_t, const Particle<T> &) {}
    virtual void post_processor (std::size_t, const Particle<T> &) {}

    protected :

    MonitorEvalCL () : build_id_(-1), buffer_size_(0) {}

    MonitorEvalCL (const MonitorEvalCL<T> &other) :
        configure_(other.configure_), build_id_(other.build_id_),
        kernel_(other.kernel_), kernel_name_(other.kernel_name_),
        buffer_size_(0) {}

    MonitorEvalCL<T> &operator= (const MonitorEvalCL<T> &other)
    {
        configure_   = other.configure_;
        build_id_    = other.build_id_;
        kernel_      = other.kernel_;
        kernel_name_ = other.kernel_name_;

        return *this;
    }

    virtual ~MonitorEvalCL () {}

    ConfigureCL &configure () {return configure_;}

    const ConfigureCL &configure () const {return configure_;}

    cl::Kernel &kernel () {return kernel_;}

    const cl::Kernel &kernel () const {return kernel_;}

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
            configure_.local_size(particle.size(),
                    kernel_, particle.value().manager().device());
        }

        kernel_.setArg<cl_ulong>(0, static_cast<cl_ulong>(iter));
        kernel_.setArg<cl_ulong>(1, static_cast<cl_ulong>(dim));
        kernel_.setArg(2, particle.value().state_buffer());
        kernel_.setArg(3, buffer_);
    }

    private :

    ConfigureCL configure_;
    int build_id_;
    cl::Kernel kernel_;
    std::string kernel_name_;
    std::size_t buffer_size_;
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
class PathEvalCL
{
    public :

    typedef typename Particle<T>::size_type size_type;
    typedef T value_type;

    /// \brief The index offset of additional kernel arguments set by the user
    ///
    /// \details
    /// The first user supplied additional argument shall have index
    /// `kernel_args_offset`
    const static std::size_t kernel_args_offset = 3;

    double operator() (std::size_t iter, const Particle<T> &particle,
        double *res)
    {
        VSMC_STATIC_ASSERT_STATE_CL_TYPE(T, PathEvalCL);

        std::size_t bsize = static_cast<std::size_t>(particle.size());
        if (buffer_size_ != bsize) {
            buffer_ = particle.value().manager().template
                create_buffer<typename T::fp_type>(bsize);
            buffer_size_ = bsize;
        }

        set_kernel(iter, particle);
        pre_processor(iter, particle);
        particle.value().manager().run_kernel(
                kernel_, particle.size(), configure_.local_size());
        particle.value().manager().template
            read_buffer<typename T::fp_type>(
                    buffer_, particle.value().size(), res);
        post_processor(iter, particle);

        return this->path_grid(iter, particle);
    }

    virtual void path_state (std::size_t, std::string &) = 0;
    virtual double path_grid (std::size_t, const Particle<T> &) = 0;
    virtual void pre_processor (std::size_t, const Particle<T> &) {}
    virtual void post_processor (std::size_t, const Particle<T> &) {}

    protected :

    PathEvalCL () : build_id_(-1), buffer_size_(0) {}

    PathEvalCL (const PathEvalCL<T> &other) :
        configure_(other.configure_), build_id_(other.build_id_),
        kernel_(other.kernel_), kernel_name_(other.kernel_name_),
        buffer_size_(0) {}

    PathEvalCL<T> &operator= (const PathEvalCL<T> &other)
    {
        configure_   = other.configure_;
        build_id_    = other.build_id_;
        kernel_      = other.kernel_;
        kernel_name_ = other.kernel_name_;

        return *this;
    }

    virtual ~PathEvalCL () {}

    ConfigureCL &configure () {return configure_;}

    const ConfigureCL &configure () const {return configure_;}

    cl::Kernel &kernel () {return kernel_;}

    const cl::Kernel &kernel () const {return kernel_;}

    void set_kernel (std::size_t iter, const Particle<T> &particle)
    {
        std::string kname;
        path_state(iter, kname);

        if (build_id_ != particle.value().build_id()
                || kernel_name_ != kname) {
            build_id_ = particle.value().build_id();
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_);
            configure_.local_size(particle.size(),
                    kernel_, particle.value().manager().device());
        }

        kernel_.setArg<cl_ulong>(0, static_cast<cl_ulong>(iter));
        kernel_.setArg(1, particle.value().state_buffer());
        kernel_.setArg(2, buffer_);
    }

    private :

    ConfigureCL configure_;
    int build_id_;
    cl::Kernel kernel_;
    std::string kernel_name_;
    std::size_t buffer_size_;
    cl::Buffer buffer_;
}; // class PathEvalCL

} // namespace vsmc

#endif // VSMC_OPENCL_BACKEND_CL_HPP

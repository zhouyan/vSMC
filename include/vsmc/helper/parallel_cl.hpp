#ifndef VSMC_HELPER_PARALLEL_CL_HPP
#define VSMC_HELPER_PARALLEL_CL_HPP

#define __CL_ENABLE_EXCEPTIONS

#include <vsmc/internal/common.hpp>
#include <vsmc/core/rng.hpp>
#include <vsmc/helper/base.hpp>
#include <vsmc/helper/parallel_cl/cl.hpp>
#include <vsmc/helper/parallel_cl/query.hpp>

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

/// \defgroup OpenCL OpenCL
/// \ingroup Helper
/// \brief Parallelized sampler with OpenCL

namespace vsmc {

/// \brief Particle::value_type subtype
/// \ingroup OpenCL
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector The default
/// \tparam Timer class The timer used for profiling run_parallel().
/// The default is NullTimer, which does nothing but provide the compatible
/// inteferce.  Timer::start() and Timer::stop() are called automatically
/// when entering and exiting run_parallel(). This shall provide how much time
/// are spent on the parallel code (plus a small overhead of scheduling).
template <unsigned Dim, typename T, typename Timer>
class StateCL
{
    public :

    /// The type of the number of particles
    typedef cl_ulong size_type;

    /// The type of state parameters (cl_float or cl_double)
    typedef T state_type;

    /// The type of timer
    typedef Timer timer_type;

    /// The type of the matrix of states returned by state_host()
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> state_mat_type;

    /// The type of the vector of weights returned by weight_host()
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> weight_vec_type;

    /// The type of the vector of accept counts returned by accept_host()
    typedef Eigen::Matrix<cl_uint, Eigen::Dynamic, 1> accept_vec_type;

    explicit StateCL (size_type N) :
        size_(N), read_buffer_pool_bytes_(0), write_buffer_pool_bytes_(0),
        read_buffer_pool_(VSMC_NULLPTR), write_buffer_pool_(VSMC_NULLPTR),
        platform_created_(false), context_created_(false),
        device_created_(false), command_queue_created_(false),
        program_created_(false), build_(false),
        state_host_(Dim, N), weight_host_(N), accept_host_(N)
    {
        local_size(0);
    }

    virtual ~StateCL ()
    {
        std::free(read_buffer_pool_);
        std::free(write_buffer_pool_);
    }

    /// The dimension of the problem
    static unsigned dim ()
    {
        return Dim;
    }

    /// The number of particles
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

    /// \brief Set a new local group size
    ///
    /// \param lsize The size of the work group.
    void local_size (std::size_t lsize)
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

    /// The current local size as set by local_size(lsize)
    void local_size () const
    {
        return local_size_;
    }

    /// The global cl::NDRange used by kernel calls
    cl::NDRange global_nd_range () const
    {
        return global_nd_range_;
    }

    /// The local cl::NDRange used by kernel calls
    cl::NDRange local_nd_range () const
    {
        return local_nd_range_;
    }

    /// Read only access to the memory buffer on the device that stores the
    /// states
    const cl::Buffer &state_device () const
    {
        return state_device_;
    }

    /// Read only access to the states
    const state_mat_type &state_host () const
    {
        read_buffer<state_type>(state_device_, size_*Dim, state_host_.data());

        return state_host_;
    }

    /// \brief Read only access to the device buffer object that stores the
    /// weights temparorie.
    ///
    /// \sa InitializeCL, MoveCL
    const cl::Buffer &weight_device () const
    {
        return weight_device_;
    }

    /// \brief Read only access to the weights stored by weight_device()
    ///
    /// \sa InitializeCL, MoveCL
    const weight_vec_type &weight_host () const
    {
        read_buffer<state_type>(weight_device_, size_, weight_host_.data());

        return weight_host_;
    }

    /// \brief Read only access to the device buffer object that stores the
    /// accept counts temparorie.
    ///
    /// \sa InitializeCL, MoveCL
    const cl::Buffer &accept_device () const
    {
        return accept_device_;
    }

    /// \brief Read only access to the accept counts stored by accept_device()
    ///
    /// \sa InitializeCL, MoveCL
    const accept_vec_type &accept_host () const
    {
        read_buffer<cl_uint>(accept_device_, size_, accept_host_.data());

        return accept_host_;
    }

    /// \brief Setup the OpenCL environment
    ///
    /// \param dev_type The type of the device intended to use
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

    /// \brief Check the status of setup
    ///
    /// \return \b true if the platform, context, device and command queue are
    /// all created, either by call setup(cl_device_type) or set by user.
    bool setup () const
    {
        return platform_created_ && context_created_ && device_created_ &&
            command_queue_created_;
    }

    /// \brief Build the program
    ///
    /// \param source The source in C-Style string format (NOT THE FILE NAME)
    /// \param flags The compiler flags passed to the OpenCL compiler
    ///
    /// \note The environment has to be setup properly either through
    /// setup(cl_device_type) or set by user.
    ///
    /// \note When building the program, the user can assume the following
    /// happen before the user source being processed.
    ///
    /// \li A type \c state_type which is the same as the template parameter
    /// T, and thus StateCL<Dim, T>::state_type is defined
    ///
    /// \li A type \c state_struct which looks like the following are defined
    ///
    /// \li A type \c size_type which is large enough to hold the number of
    /// particles and unlike \c size_t, can be passed as kernel argumentsa, is
    /// defined
    ///
    /// \li A constant \c Size of type \c size_type which the same as the size
    /// of this particle set is defined
    ///
    /// \li A constant \c Dim of type \c uint which is the same as the
    /// template parameter \c Dim is defined
    ///
    /// \li A constant \c Seed of type \c ulong which is not used by the
    /// host.  And the next size() seeds will be reserved for the the device.
    /// In other words, when next time the host request a seed, it will
    /// be <tt>Seed + size()</tt>. It is only relevant if RngSetPrl is used
    /// in the host Particle and the same parallel RNG is going to be used in
    /// the device. The default is that the host Particle StateCL will use
    /// RngSetSeq.
    ///
    /// \li The Random123 library's \c philox.h, \c threefry.h, and \c u01.h
    /// headers are included unless <tt>VSMC_USE_RANDOM123=0</tt>.
    ///
    /// The following is how the above looks like, assuming we constructed a
    /// particle set with type StateCL<4, cl_float>(1000). It is as if the
    /// user source implicitly included a header file containing the
    /// following code.
    /// \code
    /// #ifndef VSMC_USE_RANDOM123
    /// #define VSMC_USE_RANDOM123 1
    /// #endif
    ///
    /// typedef float state_type;
    ///
    /// typedef {
    ///     state_type param1;
    ///     state_type param2;
    ///     state_type param3;
    ///     state_type param4;
    /// } state_struct;
    ///
    /// typedef ulong size_type;
    ///
    /// __constant size_type Size = 1000;
    /// __constant uint Dim = 4;
    /// __constant ulong Seed = 0;
    ///
    /// #if VSMC_USE_RANDOM123
    /// #include <Random123/philox.h>
    /// #include <Random123/threefry.h>
    /// #include <Random123/u01.h>
    /// #endif
    /// \endcode
    void build (const char *source, const char *flags)
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
            for (unsigned d = 0; d != Dim; ++d)
                ss << "state_type param" << d + 1 << ";\n";
            ss << "} state_struct;\n";
            ss << "typedef ulong size_type;\n";
            ss << "__constant size_type Size = " << size_ << "UL;\n";
            ss << "__constant uint Dim = " << Dim << ";\n";
            rng::Seed &seed = rng::Seed::create();
            ss << "__constant ulong Seed = " << seed.get() << "UL;\n";
            seed.skip(size_);
            ss << "#include <vsmc/helper/parallel_cl/common.cl>\n";
            ss << source << '\n';
            program_ = cl::Program(context_, ss.str());
            program_created_ = true;
        }

        try {
            program_.build(device_, flags);
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

    /// \brief Check the status of build
    ///
    /// \return \b true if the last call to build is successful.
    bool build () const
    {
        return build_;
    }

    /// The last build log
    std::string build_log () const
    {
        return build_log_;
    }

    /// \brief Create a device buffer with given number of elements and type
    ///
    /// \tparam CLType An OpenCL type
    /// \param num The number of elements
    template<typename CLType>
    cl::Buffer create_buffer (std::size_t num) const
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_CONTEXT(create_buffer);

        return cl::Buffer(context_, CL_MEM_READ_WRITE, sizeof(CLType) * num);
    }

    /// \brief Create a device buffer with input from host iterators
    ///
    /// \tparam CLType An OpenCL Type
    /// \tparam InputIter The type of input iterator reading the host data
    /// \param first The begin of the input
    /// \param last The end of the input
    template<typename CLType, typename InputIter>
    cl::Buffer create_buffer (InputIter first, InputIter last) const
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_SETUP(create_buffer);

        std::size_t num = 0;
        for (InputIter i = first; i != last; ++i)
            ++num;
        cl::Buffer buf(create_buffer<CLType>(num));
        if (!num)
            return buf;

        write_buffer<CLType>(buf, num, first);

        return buf;
    }

    /// \brief Read a device buffer into a host iterator
    ///
    /// \tparam CLType An OpenCL Type
    /// \tparam OutputIter The type of output iterator writing the host data
    /// \param buf The device buffer to be read
    /// \param num The number of elements in the device buffer
    /// \param first The begin of the output
    template <typename CLType, typename OutputIter>
    void read_buffer (const cl::Buffer &buf, std::size_t num,
            OutputIter first) const
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_SETUP(create_buffer);

        typedef typename internal::remove_cv<OutputIter>::type ptr_type;
        typedef typename internal::remove_pointer<ptr_type>::type val_type;
        typedef typename internal::remove_cv<val_type>::type host_type;
        typedef typename internal::remove_cv<CLType>::type device_type;

        if (internal::is_pointer<OutputIter>::value) {
            if (internal::is_same<host_type, device_type>::value) {
                command_queue_.finish();
                timer_read_buffer_.start();
                command_queue_.enqueueReadBuffer(buf, 1, 0,
                        sizeof(CLType) * num, (void *) first);
                timer_read_buffer_.stop();
                return;
            }
        }

        command_queue_.finish();
        timer_read_buffer_.start();
        CLType *temp = read_buffer_pool<CLType>(num);
        command_queue_.enqueueReadBuffer(buf, 1, 0,
                sizeof(CLType) * num, (void *) temp);
        std::copy(temp, temp + num, first);
        timer_read_buffer_.stop();
    }

    /// \brief Write to a device buffer from a host iterator
    ///
    /// \tparam CLType An OpenCL Type
    /// \tparam InputIter The type of input iterator reading the host data
    /// \param buf The device buffer to be write
    /// \param num The number of elements in the device buffer
    /// \param first The begin of the input
    template <typename CLType, typename InputIter>
    void write_buffer (const cl::Buffer &buf, std::size_t num,
            InputIter first) const
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_SETUP(create_buffer);

        typedef typename internal::remove_cv<InputIter>::type ptr_type;
        typedef typename internal::remove_pointer<ptr_type>::type val_type;
        typedef typename internal::remove_cv<val_type>::type host_type;
        typedef typename internal::remove_cv<CLType>::type device_type;

        if (internal::is_pointer<InputIter>::value) {
            if (internal::is_same<host_type, device_type>::value) {
                command_queue_.finish();
                timer_write_buffer_.start();
                command_queue_.enqueueWriteBuffer(buf, 1, 0,
                        sizeof(CLType) * num, (void *) first);
                timer_write_buffer_.stop();
                return;
            }
        }

        command_queue_.finish();
        timer_write_buffer_.start();
        CLType *temp = write_pool<CLType>(num);
        std::copy(first, first + num, temp);
        command_queue_.enqueueWriteBuffer(buf, 1, 0,
                sizeof(CLType) * num, (void *) temp);
        timer_write_buffer_.stop();
    }

    /// \brief Create a kernel from a given name
    ///
    /// \param name The name of the kernel
    ///
    /// \note The program has to be built before call this member
    cl::Kernel create_kernel (const char *name) const
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(create_kernel)

        return cl::Kernel(program_, name);
    }

    /// \brief Run a kernel in parallel on the device
    ///
    /// \param ker The kernel to run
    ///
    /// \note The program has to be built before call this member. The kernel
    /// will be run with global size returned by global_nd_range() and local
    /// size returned by local_nd_range(), which shall be suitable for most
    /// use case. Both are one dimension NDRange. For more complex parallel
    /// patterns, users need to call OpenCL API themselves.
    void run_parallel (const cl::Kernel &ker) const
    {
        command_queue_.finish();
        timer_.start();
        command_queue_.enqueueNDRangeKernel(ker,
                cl::NullRange, global_nd_range(), local_nd_range());
        command_queue_.finish();
        timer_.stop();
    }

    const timer_type &timer () const
    {
        return timer_;
    }

    const timer_type &timer_read_buffer () const
    {
        return timer_read_buffer_;
    }

    const timer_type &timer_write_buffer () const
    {
        return timer_write_buffer_;
    }

    template<typename SizeType>
    void copy (const SizeType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(create_kernel)

        write_buffer<size_type>(copy_device_, size_, copy_from);
        kernel_copy_.setArg(0, state_device_);
        kernel_copy_.setArg(1, copy_device_);
        run_parallel(kernel_copy_);
    }

    private :

    size_type size_;

    std::size_t local_size_;

    mutable std::size_t read_buffer_pool_bytes_;
    mutable std::size_t write_buffer_pool_bytes_;
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

    mutable state_mat_type state_host_;
    mutable weight_vec_type weight_host_;
    mutable accept_vec_type accept_host_;

    cl::Buffer copy_device_;

    timer_type timer_;
    timer_type timer_read_buffer_;
    timer_type timer_write_buffer_;

    void setup_buffer ()
    {
        state_device_  = create_buffer<T>(size_ * Dim);
        weight_device_ = create_buffer<T>(size_);
        accept_device_ = create_buffer<cl_uint>(size_);
        copy_device_   = create_buffer<size_type>(size_);
    }

    /// \internal The reallocation of read_buffer_pool and write_buffer_pool
    /// shall not impact the performance significantly. They only expand and
    /// never shrink. In addition, sicne num is usually the size of particle
    /// set or with a multiple of Dim, after a few times (usually after the
    /// initialization), the memory will not be needed to expand anymore

    template <typename HostType>
    HostType *read_buffer_pool(std::size_t num) const
    {
        std::size_t new_bytes = num * sizeof(HostType);
        if (new_bytes > read_buffer_pool_bytes_) {
            read_buffer_pool_bytes_ = new_bytes;
            std::free(read_buffer_pool_);
            read_buffer_pool_ = std::malloc(read_buffer_pool_bytes_);
            if (!read_buffer_pool_ && read_buffer_pool_bytes_)
                throw std::bad_alloc();
        }

        return reinterpret_cast<HostType *>(read_buffer_pool_);
    }

    template <typename HostType>
    HostType *write_pool(std::size_t num) const
    {
        std::size_t new_bytes = num * sizeof(HostType);
        if (new_bytes > write_buffer_pool_bytes_) {
            write_buffer_pool_bytes_ = new_bytes;
            std::free(write_buffer_pool_);
            write_buffer_pool_ = std::malloc(write_buffer_pool_bytes_);
            if (!write_buffer_pool_ && write_buffer_pool_bytes_)
                throw std::bad_alloc();
        }

        return reinterpret_cast<HostType *>(write_buffer_pool_);
    }
}; // class StateCL

/// \brief Sampler<T>::init_type subtype
/// \ingroup OpenCL
///
/// \tparam T A subtype of StateCL
///
/// \note
/// A valid kernel declaration looks like
/// \code
/// __kernel void init (state_struct *state, state_type *weight, uint *accept)
/// \endcode
/// There can has other user supplied arguments as long as the first three are
/// as above. Among them, \c state is StateCL::state_device(), \c weight is
/// StateCL::weight_device() and \c accept is StateCL::accept_device(). After
/// the call to the operator(), they can be read into the host through
/// StateCL::weight_host() etc.
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

        return particle.value().accept_host().sum();
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
        // TODO More robust checking of kernel change
        std::string kname;
        initialize_state(kname);

        if (kernel_name_ != kname) {
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_.c_str());
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

    ~InitializeCL () {}

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
}; // class InitializeCL

/// \brief Sampler<T>::move_type subtype
/// \ingroup OpenCL
///
/// \tparam T A subtype of StateCL
///
/// \note
/// A valid kernel declaration looks like
/// \code
/// __kernel void move (uint iter, state_struct *state,
///     state_type *weight, uint *accept)
/// \endcode
/// There can has other user supplied arguments as long as the first four are
/// as above. Among them, \c state is StateCL::state_device(), \c weight is
/// StateCL::weight_device() and \c accept is StateCL::accept_device(). After
/// the call to the operator(), they can be read into the host through
/// StateCL::weight_host() etc.
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

        return particle.value().accept_host().sum();
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
        // TODO More robust checking of kernel change
        std::string kname;
        move_state(iter, kname);

        if (kernel_name_ != kname) {
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_.c_str());
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

    ~MoveCL () {}

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
}; // class MoveCL

/// \brief Monitor<T>::eval_type subtype
/// \ingroup OpenCL
///
/// \tparam T A Subtype of StateCL
///
/// \note
/// A valid kernel declaration looks like
/// \code
/// __kernel void monitor_eval (uint iter, uint dim, state_struct *state,
///     state_type *buffer)
/// \endcode
/// where \c buffer is a row major \c dim by \c size matrix. There can has
/// other user supplied arguments as long as the first four is as above
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
        // TODO More robust checking of kernel change
        std::string kname;
        monitor_state(iter, kname);

        if (kernel_name_ != kname) {
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_.c_str());
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

    ~MonitorEvalCL () {}

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    cl::Buffer buffer_device_;
}; // class MonitorEvalCL

/// \brief Path<T>::eval_type subtype
/// \ingroup OpenCL
///
/// \tparam T A subtype of StateCL
///
/// \note
/// A valid kernel declaration looks like
/// \code
/// __kernel void path_eval (uint iter, state_struct *state,
///     state_type *buffer)
/// \endcode
/// There can has other user supplied arguments as long as the first three is
/// as above
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
        // TODO More robust checking of kernel change
        std::string kname;
        path_state(iter, kname);

        if (kernel_name_ != kname) {
            kernel_name_ = kname;
            kernel_ = particle.value().create_kernel(kernel_name_.c_str());
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

    ~PathEvalCL () {}

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    cl::Buffer buffer_device_;
}; // class PathEvalCL

} // namespace vsmc

#endif // VSMC_HELPER_PARALLEL_CL_HPP

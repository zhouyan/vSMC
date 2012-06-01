#ifndef V_SMC_HELPER_PARALLEL_CL_HPP
#define V_SMC_HELPER_PARALLEL_CL_HPP

#define __CL_ENABLE_EXCEPTIONS

#include <vSMC/internal/common.hpp>
#include <vSMC/helper/parallel_cl/cl.hpp>

namespace vSMC {

template <typename T>
void cl_pre_resampling (T &state)
{
    state.pre_resampling();
}

template <typename T>
void cl_post_resampling (T &state)
{
    state.post_resampling();
}

/// \brief Particle type class for helping implementing SMC using OpenCL
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
///
/// \note If the compiler generated copy construtor and assignement operator
/// are not fine for the derived class, the derived class has to copy this base
/// class itself.
template <unsigned Dim, typename T>
class StateCL
{
    public :

    typedef V_SMC_INDEX_TYPE size_type;
    typedef T state_type;
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> state_mat_type;
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> weight_vec_type;
    typedef Eigen::Matrix<cl_uint, Eigen::Dynamic, 1> accept_vec_type;
    typedef Eigen::Matrix<cl_uint, Eigen::Dynamic, 1> copy_vec_type;

    explicit StateCL (size_type N) :
        size_(N),
        platform_created_(false), context_created_(false),
        device_created_(false), command_queue_created_(false),
        program_created_(false), build_(false),
        global_size_(N), local_size_(0),
        state_host_(Dim, N), weight_host_(N), accept_host_(N), copy_host_(N)
    {}

    virtual ~StateCL () {}

    static unsigned dim ()
    {
        return Dim;
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
        setup_buffer();
        context_created_ = true;
    }

    const bool context_created () const
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

    cl::NDRange global_nd_range () const
    {
        return cl::NDRange(global_size_);
    }

    cl::NDRange local_nd_range () const
    {
        if (local_size_)
            return cl::NDRange(local_size_);
        else
            return cl::NullRange;
    }

    void local_size (std::size_t size)
    {
        local_size_ = size;
        if (local_size_ && size_ % local_size_)
            global_size_ = (size_ / local_size_ + 1) * local_size_;
        else
            global_size_ = size_;
    }

    const cl::Buffer &state_device () const
    {
        return state_device_;
    }

    const state_mat_type &state_host () const
    {
        command_queue_.enqueueReadBuffer(state_device_, 1, 0,
                sizeof(T) * size_ * Dim, (void *) state_host_.data());
        return state_host_;
    }

    const cl::Buffer &weight_device () const
    {
        return weight_device_;
    }

    const weight_vec_type &weight_host () const
    {
        command_queue_.enqueueReadBuffer(weight_device_, 1, 0,
                sizeof(T) * size_, (void *) weight_host_.data());
        return weight_host_;
    }

    const cl::Buffer &accept_device () const
    {
        return accept_device_;
    }

    const accept_vec_type &accept_host () const
    {
        command_queue_.enqueueReadBuffer(accept_device_, 1, 0,
                sizeof(cl_uint) * size_, (void *) accept_host_.data());
        return accept_host_;
    }

    void setup (cl_device_type dev_type = CL_DEVICE_TYPE_GPU)
    {
        cl::Platform::get(&platform_);
        platform_created_ = true;

        cl_context_properties context_properties[] = {
            CL_CONTEXT_PLATFORM, (cl_context_properties)(platform_[0])(), 0
        };
        context_ = cl::Context(dev_type, context_properties);
        setup_buffer();
        context_created_ = true;

        device_= context_.getInfo<CL_CONTEXT_DEVICES>();
        device_created_ = true;

        command_queue_ = cl::CommandQueue(context_, device_[0], 0);
        command_queue_created_ = true;
    }

    void setup_buffer ()
    {
        state_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(T) * size_ * Dim);
        weight_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(T) * size_);
        accept_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(cl_uint) * size_);
        copy_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(cl_uint) * size_);
    }

    template <typename State>
    void build (const char *source, const char *flags, Sampler<State> &sampler)
    {
        if (!(platform_created_ && context_created_ &&
                    command_queue_created_ && device_created_))
        {
            setup();
        }

        if (!program_created_) {
            std::stringstream ss;
            ss << "__constant uint Dim = " << Dim << ";\n";
            if (sizeof(T) == sizeof(cl_float))
                ss << "typedef float state_type;\n";
            else if (sizeof(T) == sizeof(cl_double))
                ss << "typedef double state_type;\n";
            ss << "typedef struct {\n";
            for (unsigned d = 0; d != Dim; ++d)
                ss << "state_type param" << d + 1 << ";\n";
            ss << "} state_struct;\n";
            ss << "#include <vSMC/helper/parallel_cl/common.cl>\n";
            ss << source << '\n';
            cl::Program::Sources src(1, std::make_pair(ss.str().c_str(), 0));
            program_ = cl::Program(context_, src);
            program_created_ = true;
        }

        program_.build(device_, flags);

        kernel_copy_ = cl::Kernel(program_, "copy");
        sampler.particle().pre_resampling(
                cl_pre_resampling<StateCL<Dim, T> >);
        sampler.particle().post_resampling(
                cl_post_resampling<StateCL<Dim, T> >);
        build_ = true;
    }

    bool build () const
    {
        return build_;
    }

    virtual void copy (size_type from, size_type to)
    {
        copy_host_[to] = from;
    }

    void pre_resampling ()
    {
        for (size_type i = 0; i != size_; ++i)
            copy_host_[i] = i;
    }

    void post_resampling ()
    {
        assert(build_);

        command_queue_.enqueueWriteBuffer(copy_device_, 1, 0,
                sizeof(cl_uint) * size_, (void *) copy_host_.data());
        kernel_copy_.setArg(0, (std::size_t) size_);
        kernel_copy_.setArg(1, state_device_);
        kernel_copy_.setArg(2, copy_device_);
        command_queue_.enqueueNDRangeKernel(kernel_copy_, cl::NullRange,
                global_nd_range(), local_nd_range());
    }

    private :

    size_type size_;

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

    std::size_t global_size_;
    std::size_t local_size_;

    cl::Buffer state_device_;
    cl::Buffer weight_device_;
    cl::Buffer accept_device_;
    cl::Buffer copy_device_;

    mutable state_mat_type state_host_;
    mutable weight_vec_type weight_host_;
    mutable accept_vec_type accept_host_;
    mutable copy_vec_type copy_host_;
}; // class StateCL

/// \brief Sampler::init_type subtype
///
/// \tparam T A subtype of StateCL
template <typename T>
class InitializeCL
{
    public :

    typedef function<void (Particle<T> &, void *)> initialize_param_type;
    typedef function<void (Particle<T> &)> pre_processor_type;
    typedef function<void (Particle<T> &)> post_processor_type;

    explicit InitializeCL (const char *kernel_name) :
        kernel_name_(kernel_name), kernel_created_(false) {}

    InitializeCL (const InitializeCL<T> &other) :
        kernel_(other.kernel_), kernel_name_(other.kernel_name_),
        kernel_created_(other.kernel_created_) {}

    const InitializeCL<T> &operator= (const InitializeCL<T> &other)
    {
        if (this != &other) {
            kernel_ = other.kernel_;
            kernel_name_ = other.kernel_name_;
            kernel_created_ = other.kernel_created_;
        }

        return *this;
    }

    virtual ~InitializeCL () {}

    virtual unsigned operator() (Particle<T> &particle, void *param)
    {
        create_kernel(particle);
        initialize_param(particle, param);
        pre_processor(particle);
        // TODO more control over local size
        particle.value().command_queue().enqueueNDRangeKernel(kernel_,
                cl::NullRange,
                particle.value().global_nd_range(),
                particle.value().local_nd_range());
        // TODO more efficient weight copying
        const typename T::weight_vec_type &weight =
            particle.value().weight_host();
        if (sizeof(typename T::state_type) == sizeof(double)) {
            particle.set_log_weight(
                    reinterpret_cast<const double *>(weight.data()));
        } else {
            weight_.resize(particle.size());
            for (typename Particle<T>::size_type i = 0;
                    i != particle.size(); ++i) {
                weight_[i] = weight[i];
            }
            particle.set_log_weight(weight_);
        }
        post_processor(particle);

        return particle.value().accept_host().sum();
    }

    virtual void initialize_param (Particle<T> &particle, void *param) {}
    virtual void pre_processor (Particle<T> &particle) {}
    virtual void post_processor (Particle<T> &particle) {}

    cl::Kernel &kernel ()
    {
        return kernel_;
    }

    const cl::Kernel &kernel () const
    {
        return kernel_;
    }

    void create_kernel (const Particle<T> &particle)
    {
        assert(particle.value().build());

        if (!kernel_created_) {
            kernel_ = cl::Kernel(
                    particle.value().program(), kernel_name_.c_str());
            kernel_created_ = true;
        }

        kernel_.setArg(0, (std::size_t) particle.size());
        kernel_.setArg(1, particle.value().state_device());
        kernel_.setArg(2, particle.value().weight_device());
        kernel_.setArg(3, particle.value().accept_device());
    }

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    bool kernel_created_;
    typename Particle<T>::weight_type weight_;
};

/// \brief Sampler::move_type subtype
///
/// \tparam T A subtype of StateCL
template <typename T>
class MoveCL
{
    public :

    typedef function<void (unsigned, Particle<T> &)> pre_processor_type;
    typedef function<void (unsigned, Particle<T> &)> post_processor_type;

    explicit MoveCL (const char *kernel_name) :
        kernel_name_(kernel_name), kernel_created_(false) {}

    MoveCL (const MoveCL<T> &other) :
        kernel_(other.kernel_), kernel_name_(other.kernel_name_),
        kernel_created_(other.kernel_created_) {}

    const MoveCL<T> &operator= (const MoveCL<T> &other)
    {
        if (this != &other) {
            kernel_ = other.kernel_;
            kernel_name_ = other.kernel_name_;
            kernel_created_ = other.kernel_created_;
        }

        return *this;
    }

    virtual ~MoveCL () {}

    virtual unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        create_kernel(iter, particle);
        pre_processor(iter, particle);
        // TODO more control over local size
        particle.value().command_queue().enqueueNDRangeKernel(kernel_,
                cl::NullRange,
                particle.value().global_nd_range(),
                particle.value().local_nd_range());
        // TODO more efficient weight copying
        const typename T::weight_vec_type &weight =
            particle.value().weight_host();
        if (sizeof(typename T::state_type) == sizeof(double)) {
            particle.add_log_weight(
                    reinterpret_cast<const double *>(weight.data()));
        } else {
            weight_.resize(particle.size());
            for (typename Particle<T>::size_type i = 0;
                    i != particle.size(); ++i) {
                weight_[i] = weight[i];
            }
            particle.add_log_weight(weight_);
        }
        post_processor(iter, particle);

        return particle.value().accept_host().sum();
    }

    virtual void pre_processor (unsigned iter, Particle<T> &particle) {}
    virtual void post_processor (unsigned iter, Particle<T> &particle) {}

    cl::Kernel &kernel ()
    {
        return kernel_;
    }

    const cl::Kernel &kernel () const
    {
        return kernel_;
    }

    void create_kernel (unsigned iter, const Particle<T> &particle)
    {
        assert(particle.value().build());

        if (!kernel_created_) {
            kernel_ = cl::Kernel(
                    particle.value().program(), kernel_name_.c_str());
            kernel_created_ = true;
        }

        kernel_.setArg(0, (std::size_t) particle.size());
        kernel_.setArg(1, (cl_uint) iter);
        kernel_.setArg(2, particle.value().state_device());
        kernel_.setArg(3, particle.value().weight_device());
        kernel_.setArg(4, particle.value().accept_device());
    }

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    bool kernel_created_;
    typename Particle<T>::weight_type weight_;
}; // class MoveCL

/// \brief Non-direct Monitor::eval_type subtype
///
/// \tparam T A Subtype of StateCL
/// \tparam Dim The dimension of the monitor
///
/// \note Currently Dim cannot be larger than particle set size
template <typename T, unsigned Dim>
class MonitorCL
{
    public :

    typedef function<void (unsigned, const Particle<T> &)> pre_processor_type;
    typedef function<void (unsigned, const Particle<T> &)> post_processor_type;

    explicit MonitorCL (const char *kernel_name) :
        kernel_name_(kernel_name), kernel_created_(false) {}

    MonitorCL (const MonitorCL<T, Dim> &other) :
        kernel_(other.kernel_), kernel_name_(other.kernel_name_),
        kernel_created_(other.kernel_created_),
        buffer_device_(other.buffer_device_) {}

    const MonitorCL<T, Dim> &operator= (const MonitorCL<T, Dim> &other)
    {
        if (this != &other) {
            kernel_         = other.kernel_;
            kernel_name_    = other.kernel_name_;
            kernel_created_ = other.kernel_created_;
            buffer_device_  = other.buffer_device_;
        }

        return *this;
    }

    virtual ~MonitorCL () {}

    virtual void operator() (unsigned iter, const Particle<T> &particle,
        double *res)
    {
        create_kernel(iter, particle);
        pre_processor(iter, particle);
        particle.value().command_queue().enqueueNDRangeKernel(kernel_,
                cl::NullRange,
                particle.value().global_nd_range(),
                particle.value().local_nd_range());
        if (sizeof(typename T::state_type) == sizeof(double)) {
            particle.value().command_queue().enqueueReadBuffer(buffer_device_,
                    1, 0,
                    sizeof(typename T::state_type) * particle.size() * Dim,
                    (void *) res);
        } else {
            buffer_host_.resize(Dim, particle.size());
            particle.value().command_queue().enqueueReadBuffer(buffer_device_,
                    1, 0,
                    sizeof(typename T::state_type) * particle.size() * Dim,
                    (void *) buffer_host_.data());
            Eigen::Map<Eigen::MatrixXd> res_mat(res, Dim, particle.size());
            for (unsigned d = 0; d != Dim; ++d) {
                for (typename Particle<T>::size_type i = 0;
                        i != particle.size(); ++i)
                    res_mat(d, i) = buffer_host_(d, i);
            }
        }
        post_processor(iter, particle);
    }

    virtual void pre_processor (unsigned iter, const Particle<T> &particle) {}
    virtual void post_processor (unsigned iter, const Particle<T> &particle) {}

    static unsigned dim ()
    {
        return Dim;
    }

    cl::Kernel kernel ()
    {
        return kernel_;
    }

    const cl::Kernel kernel () const
    {
        return kernel_;
    }

    void create_kernel (unsigned iter, const Particle<T> &particle)
    {
        assert(particle.value().build());

        if (!kernel_created_) {
            kernel_ = cl::Kernel(
                    particle.value().program(), kernel_name_.c_str());
            buffer_device_ = cl::Buffer(particle.value().context(),
                    CL_MEM_READ_WRITE,
                    sizeof(typename T::state_type) * Dim * particle.size());
            kernel_created_ = true;
        }

        kernel_.setArg(0, (std::size_t) particle.size());
        kernel_.setArg(1, (cl_uint) iter);
        kernel_.setArg(2, (cl_uint) Dim);
        kernel_.setArg(3, particle.value().state_device());
        kernel_.setArg(4, buffer_device_);
    }

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    bool kernel_created_;
    cl::Buffer buffer_device_;
    typename T::state_mat_type buffer_host_;
}; // class MonitorCL

/// \brief Non-direct Path::eval_type subtype
///
/// \tparam T A subtype of StateCL
template <typename T>
class PathCL
{
    public :

    typedef function<double (unsigned, const Particle<T> &)>
        width_state_type;
    typedef function<void (unsigned, const Particle<T> &)> pre_processor_type;
    typedef function<void (unsigned, const Particle<T> &)> post_processor_type;

    explicit PathCL (const char *kernel_name) :
        kernel_name_(kernel_name), kernel_created_(false) {}

    PathCL (const PathCL<T> &other) :
        kernel_(other.kernel_), kernel_name_(other.kernel_name_),
        kernel_created_(other.kernel_created_),
        buffer_device_(other.buffer_device_) {}

    const PathCL<T> &operator= (const PathCL<T> &other)
    {
        if (this != &other) {
            kernel_         = other.kernel_;
            kernel_name_    = other.kernel_name_;
            kernel_created_ = other.kernel_created_;
            buffer_device_  = other.buffer_device_;
        }

        return *this;
    }

    virtual ~PathCL () {}

    virtual double operator() (unsigned iter, const Particle<T> &particle,
        double *res)
    {
        create_kernel(iter, particle);
        pre_processor(iter, particle);
        particle.value().command_queue().enqueueNDRangeKernel(kernel_,
                cl::NullRange,
                particle.value().global_nd_range(),
                particle.value().local_nd_range());
        if (sizeof(typename T::state_type) == sizeof(double)) {
            particle.value().command_queue().enqueueReadBuffer(buffer_device_,
                    1, 0, sizeof(typename T::state_type) * particle.size(),
                    (void *) res);
        } else {
            buffer_host_.resize(particle.size());
            particle.value().command_queue().enqueueReadBuffer(buffer_device_,
                    1, 0, sizeof(typename T::state_type) * particle.size(),
                    (void *) buffer_host_.data());
            for (typename Particle<T>::size_type i = 0;
                        i != particle.size(); ++i) {
                    res[i] = buffer_host_[i];
            }
        }
        post_processor(iter, particle);

        return this->width_state(iter, particle);
    }

    virtual double width_state (unsigned iter,
            const Particle<T> &particle) = 0;
    virtual void pre_processor (unsigned iter, const Particle<T> &particle) {}
    virtual void post_processor (unsigned iter, const Particle<T> &particle) {}

    cl::Kernel kernel ()
    {
        return kernel_;
    }

    const cl::Kernel kernel () const
    {
        return kernel_;
    }

    void create_kernel (unsigned iter, const Particle<T> &particle)
    {
        assert(particle.value().build());

        if (!kernel_created_) {
            kernel_ = cl::Kernel(
                    particle.value().program(), kernel_name_.c_str());
            buffer_device_ = cl::Buffer(particle.value().context(),
                    CL_MEM_READ_WRITE,
                    sizeof(typename T::state_type) * particle.size());
            kernel_created_ = true;
        }

        kernel_.setArg(0, (std::size_t) particle.size());
        kernel_.setArg(1, (cl_uint) iter);
        kernel_.setArg(2, particle.value().state_device());
        kernel_.setArg(3, buffer_device_);
    }

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    bool kernel_created_;
    cl::Buffer buffer_device_;
    typename T::weight_vec_type buffer_host_;
}; // class PathCL

} // namespace vSMC

#endif // V_SMC_HELPER_PARALLEL_CL_HPP

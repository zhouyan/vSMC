#ifndef V_SMC_HELPER_PARALLEL_CL_HPP
#define V_SMC_HELPER_PARALLEL_CL_HPP

#define __CL_ENABLE_EXCEPTIONS

#include <vSMC/internal/common.hpp>
#include <vSMC/helper/parallel_cl/cl.hpp>

namespace vSMC {

template <typename T>
void cl_pre_copy (T &state)
{
    state.pre_copy();
}

template <typename T>
void cl_post_copy (T &state)
{
    state.post_copy();
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
        size_(N), build_(false), context_created_(false),
        state_host_(Dim, N), weight_host_(N), accept_host_(N), copy_host_(N)
    {
        try {
            cl_initialize();
        } catch (cl::Error err) {
            std::cerr << err.what() << std::endl;
            std::abort();
        }
        // TODO Assert that T is either cl_float of cl_double
    }

    virtual ~StateCL () {}

    static unsigned dim ()
    {
        return Dim;
    }

    size_type size () const
    {
        return size_;
    }
    const cl::CommandQueue &command_queue () const
    {
        return command_queue_;
    }

    void command_queue (const cl::CommandQueue &queue)
    {
        command_queue_ = queue;
    }

    const cl::Context &context () const
    {
        return context_;
    }

    void context (const cl::CommandQueue &ctx)
    {
        context_ = ctx;
        context_created_ = true;
    }

    const bool context_created () const
    {
        return context_created_;
    }

    const cl::Program &program () const
    {
        return program_;
    }

    void program (const cl::CommandQueue &prg)
    {
        program_ = prg;
    }

    const std::vector<cl::Platform> &platform () const
    {
        return platform_;
    }

    void platform (const std::vector<cl::Platform> &plat)
    {
        platform_ = plat;
    }

    const std::vector<cl::Device> &device () const
    {
        return device_;
    }

    void device (const std::vector<cl::Device> &dev)
    {
        device_ = dev;
    }

    const state_mat_type &state_host () const
    {
        command_queue_.enqueueReadBuffer(state_device_, 1, 0,
                sizeof(T) * size_ * Dim, (void *) state_host_.data());
        return state_host_;
    }

    const cl::Buffer &state_device () const
    {
        return state_device_;
    }

    const weight_vec_type &weight_host () const
    {
        command_queue_.enqueueReadBuffer(weight_device_, 1, 0,
                sizeof(T) * size_, (void *) weight_host_.data());
        return weight_host_;
    }

    const cl::Buffer &weight_device () const
    {
        return weight_device_;
    }

    const accept_vec_type &accept_host () const
    {
        command_queue_.enqueueReadBuffer(accept_device_, 1, 0,
                sizeof(cl_uint) * size_, (void *) accept_host_.data());
        return accept_host_;
    }

    const cl::Buffer &accept_device () const
    {
        return accept_device_;
    }

    template <typename State>
    void build (const char *source, const char *flags, Sampler<State> &sampler)
    {
        assert(context_created_);

        device_= context_.getInfo<CL_CONTEXT_DEVICES>();

        command_queue_ = cl::CommandQueue(context_, device_[0], 0);

        state_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(T) * size_ * Dim);
        weight_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(T) * size_);
        accept_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(cl_uint) * size_);
        copy_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(cl_uint) * size_);

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

        sampler.particle().pre_resampling(cl_pre_copy<StateCL<Dim, T> >);
        sampler.particle().post_resampling(cl_post_copy<StateCL<Dim, T> >);

        try {
            cl::Program::Sources src(1, std::make_pair(ss.str().c_str(), 0));
            program_ = cl::Program(context_, src);
            program_.build(device_, flags);
            kernel_copy_ = cl::Kernel(program_, "copy");
        } catch (cl::Error err) {
            std::string str;
            device_[0].getInfo((cl_device_info) CL_DEVICE_NAME, &str);
            std::cerr
                << "Device: " << str << std::endl
                << "Build options: "
                << program_.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(device_[0])
                << std::endl
                << "Build log: "
                << program_.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device_[0])
                << std::endl;
            build_ = false;
            throw cl::Error(err);
        }

        build_ = true;
    }

    bool build () const
    {
        return build_;
    }

    void copy (size_type from, size_type to)
    {
        copy_host_[to] = from;
    }

    void pre_copy ()
    {
        for (size_type i = 0; i != size_; ++i)
            copy_host_[i] = i;
    }

    void post_copy ()
    {
        assert(build_);

        command_queue_.enqueueWriteBuffer(copy_device_, 1, 0,
                sizeof(cl_uint) * size_, (void *) copy_host_.data());

        kernel_copy_.setArg(0, (std::size_t) size_);
        kernel_copy_.setArg(1, state_device_);
        kernel_copy_.setArg(2, copy_device_);
        command_queue_.enqueueNDRangeKernel(kernel_copy_,
                cl::NullRange, cl::NDRange(size_), cl::NullRange);
    }

    private :

    size_type size_;

    cl::CommandQueue command_queue_;
    cl::Context context_;
    cl::Program program_;
    std::vector<cl::Platform> platform_;
    std::vector<cl::Device> device_;

    bool build_;
    bool context_created_;

    cl::Kernel kernel_copy_;

    mutable state_mat_type state_host_;
    mutable weight_vec_type weight_host_;
    mutable accept_vec_type accept_host_;
    mutable copy_vec_type copy_host_;

    cl::Buffer state_device_;
    cl::Buffer weight_device_;
    cl::Buffer accept_device_;
    cl::Buffer copy_device_;

    void cl_initialize ()
    {
        cl::Platform::get(&platform_);

        cl_context_properties context_properties[] = {
            CL_CONTEXT_PLATFORM, (cl_context_properties)(platform_[0])(), 0
        };

        if (!context_created_) {
            try {
                context_ = cl::Context(CL_DEVICE_TYPE_GPU, context_properties);
                context_created_ = true;
            } catch (cl::Error err) {
                std::cerr << "Failed to create context for GPU." << std::endl;
                std::cerr << "Continue with trying CPU." << std::endl;
                std::cerr << "Error message:" << std::endl;
                std::cerr << err.what() << std::endl;
            }
        }

        if (!context_created_) {
            try {
                context_ = cl::Context(CL_DEVICE_TYPE_CPU, context_properties);
                context_created_ = true;
            } catch (cl::Error err) {
                std::cerr << "Failed to create context for CPU." << std::endl;
                std::cerr << "Continue with trying ALL." << std::endl;
                std::cerr << "Error message:" << std::endl;
                std::cerr << err.what() << std::endl;
            }
        }

        if (!context_created_) {
            try {
                context_ = cl::Context(CL_DEVICE_TYPE_ALL, context_properties);
                context_created_ = true;
            } catch (cl::Error err) {
                std::cerr << "Failed to create context for ALL." << std::endl;
                std::cerr << "Error message:" << std::endl;
                std::cerr << err.what() << std::endl;
            }
        }
    }
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

    virtual unsigned operator() (Particle<T> &particle, void *param)
    {
        create_kernel(particle);
        initialize_param(particle, param);
        pre_processor(particle);
        // TODO more control over local size
        particle.value().command_queue().enqueueNDRangeKernel(kernel_,
                cl::NullRange, cl::NDRange(particle.size()), cl::NullRange);
        // TODO more efficient weight copying
        const typename T::weight_vec_type &weight =
            particle.value().weight_host();
        weight_.resize(particle.size());
        for (typename Particle<T>::size_type i = 0; i != particle.size(); ++i)
            weight_[i] = weight[i];
        particle.set_log_weight(weight_);
        post_processor(particle);

        return particle.value().accept_host().sum();
    }

    virtual void initialize_param (Particle<T> &particle, void *param) {}
    virtual void pre_processor (Particle<T> &particle) {}
    virtual void post_processor (Particle<T> &particle) {}

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

    virtual unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        create_kernel(iter, particle);
        pre_processor(iter, particle);
        // TODO more control over local size
        particle.value().command_queue().enqueueNDRangeKernel(kernel_,
                cl::NullRange, cl::NDRange(particle.size()), cl::NullRange);
        // TODO more efficient weight copying
        const typename T::weight_vec_type &weight =
            particle.value().weight_host();
        weight_.resize(particle.size());
        for (typename Particle<T>::size_type i = 0; i != particle.size(); ++i)
            weight_[i] = weight[i];
        particle.add_log_weight(weight_);
        post_processor(iter, particle);

        return particle.value().accept_host().sum();
    }

    virtual void pre_processor (unsigned iter, Particle<T> &particle) {}
    virtual void post_processor (unsigned iter, Particle<T> &particle) {}

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    bool kernel_created_;
    typename Particle<T>::weight_type weight_;
}; // class MoveCL

/// \brief Direct Monitor::eval_type subtype
///
/// \tparam T A Subtype of StateCL
/// \tparam Dim The dimension of the monitor
template <typename T, unsigned Dim>
class MonitorCL
{
    public :

    explicit MonitorCL (const char *kernel_name) :
        kernel_name_(kernel_name), kernel_created_(false), result_host_(Dim) {}

    void operator() (unsigned iter, const Particle<T> &particle,
        double *res)
    {
        create_kernel();
        particle.value().command_queue().enqueueNDRangeKernel(kernel_,
                cl::NullRange, cl::NDRange(particle.size()), cl::NullRange);
        particle.value().command_queue().enqueueNDRangeKernel(kernel_eval_,
                cl::NullRange, cl::NDRange(particle.size()), cl::NullRange);
        particle.value().command_queue().enqueueReadBuffer(result_device_,
                1, 0, sizeof(typename T::state_type) * Dim,
                (void *) result_host_.data());
        for (unsigned i = 0; i != Dim; ++i)
            res[i] = result_host_[i];
    }

    static unsigned dim ()
    {
        return Dim;
    }

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    bool kernel_created_;
    cl::Buffer buffer_device_;

    cl::Kernel kernel_eval_;
    cl::Buffer result_device_;
    Eigen::Matrix<typename T::state_type, Eigen::Dynamic, 1> result_host_;

    void create_kernel (unsigned iter, const Particle<T> &particle)
    {
        assert(particle.value().build());

        if (!kernel_created_) {
            kernel_ = cl::Kernel(
                    particle.value().program(), kernel_name_.c_str());
            kernel_eval_ = cl::Kernel(
                    particle.value().program(), "monitor_eval");
            kernel_created_ = true;
            buffer_device_ = cl::Buffer(particle.value().context(),
                    CL_MEM_READ_WRITE,
                    sizeof(typename T::state_type) * Dim * particle.size());
            result_device_ = cl::Buffer(particle.value().context(),
                    CL_MEM_READ_WRITE, sizeof(typename T::state_type) * Dim);
        }

        kernel_.setArg(0, (std::size_t) particle.size());
        kernel_.setArg(1, (cl_uint) iter);
        kernel_.setArg(2, particle.value().state_device());
        kernel_.setArg(3, buffer_device_);

        kernel_eval_.setArg(0, (std::size_t) particle.size());
        kernel_eval_.setArg(1, (cl_int) particle.resampled());
        kernel_eval_.setArg(2, buffer_device_);
        kernel_eval_.setArg(3, particle.value().weight_device());
        kernel_eval_.setArg(4, result_device_);
    }
}; // class MonitorCL

} // namespace vSMC

#endif // V_SMC_HELPER_PARALLEL_CL_HPP

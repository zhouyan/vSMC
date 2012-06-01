#ifndef V_SMC_HELPER_PARALLEL_CL_HPP
#define V_SMC_HELPER_PARALLEL_CL_HPP

#define __CL_ENABLE_EXCEPTIONS

#include <vSMC/internal/common.hpp>
#include <vSMC/helper/parallel_cl/cl.hpp>

namespace vSMC {

/// \tparam T CL type
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
        size_(N), build_(false),
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

    cl::CommandQueue &command_queue ()
    {
        return command_queue_;
    }

    const cl::CommandQueue &command_queue () const
    {
        return command_queue_;
    }

    cl::Context &context ()
    {
        return context_;
    }

    const cl::Context &context () const
    {
        return context_;
    }

    cl::Program &program ()
    {
        return program_;
    }

    const cl::Program &program () const
    {
        return program_;
    }

    std::vector<cl::Platform> &platform ()
    {
        return platform_;
    }

    const std::vector<cl::Platform> &platform () const
    {
        return platform_;
    }

    std::vector<cl::Device> &device ()
    {
        return device_;
    }

    const std::vector<cl::Device> &device () const
    {
        return device_;
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

    cl::Buffer &weight_device ()
    {
        return weight_device_;
    }

    const accept_vec_type &accept_host () const
    {
        command_queue_.enqueueReadBuffer(accept_device_, 1, 0,
                sizeof(cl_uint) * size_, (void *) accept_host_.data());
        return accept_host_;
    }

    cl::Buffer &accept_device ()
    {
        return accept_device_;
    }

    void build (const char *source, const char *flags)
    {
        std::stringstream ss;
        ss << "#include <vSMC/helper/parallel_cl/common.h>\n";
        ss << "__constant uint Dim = " << Dim << ";\n";
        if (sizeof(T) == sizeof(cl_float))
            ss << "typedef float state_type;\n";
        else if (sizeof(T) == sizeof(cl_double))
            ss << "typedef double state_type;\n";
        ss << "typedef struct {\n";
        for (unsigned d = 0; d != Dim; ++d)
            ss << "state_type param" << d + 1 << ";\n";
        ss << "} state_struct;\n";
        ss << "__kernel void copy\n";
        ss << "(__global state_struct *state, __global uint *from)\n";
        ss << "{\n";
        ss << "size_t i = get_global_id(0);\n";
        ss << "state[i] = state[from[i]];\n";
        ss << "}\n";
        ss << source << '\n';

        try {
            cl::Program::Sources src(1, std::make_pair(ss.str().c_str(), 0));
            program_ = cl::Program(context_, src);
            program_.build(device_, flags);
            kernel_copy_ = cl::Kernel(program_, "copy");
        } catch (cl::Error err) {
            std::cerr
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

        kernel_copy_.setArg(0, state_device_);
        kernel_copy_.setArg(1, copy_device_);
        command_queue_.enqueueWriteBuffer(copy_device_, 1, 0,
                sizeof(cl_uint) * size_, (void *) copy_host_.data());
        command_queue_.enqueueNDRangeKernel(kernel_copy_,
                cl::NullRange, cl::NDRange(size_), cl::NullRange);
        command_queue_.enqueueBarrier();
    }

    private :

    size_type size_;

    cl::CommandQueue command_queue_;
    cl::Context context_;
    cl::Program program_;
    std::vector<cl::Platform> platform_;
    std::vector<cl::Device> device_;

    bool build_;

    cl::Kernel kernel_copy_;

    state_mat_type state_host_;
    weight_vec_type weight_host_;
    accept_vec_type accept_host_;
    copy_vec_type copy_host_;

    cl::Buffer state_device_;
    cl::Buffer weight_device_;
    cl::Buffer accept_device_;
    cl::Buffer copy_device_;

    void cl_initialize ()
    {
        cl::Platform::get(&platform_);

        bool context_created = false;
        cl_context_properties context_properties[] = {
            CL_CONTEXT_PLATFORM, (cl_context_properties)(platform_[0])(), 0
        };

        if (!context_created) {
            try {
                context_ = cl::Context(CL_DEVICE_TYPE_GPU, context_properties);
                context_created = true;
            } catch (cl::Error err) {
                std::cerr << "Failed to create context for GPU." << std::endl;
                std::cerr << "Continue with trying CPU." << std::endl;
                std::cerr << "Error message:" << std::endl;
                std::cerr << err.what() << std::endl;
            }
        }

        if (!context_created) {
            try {
                context_ = cl::Context(CL_DEVICE_TYPE_CPU, context_properties);
                context_created = true;
            } catch (cl::Error err) {
                std::cerr << "Failed to create context for CPU." << std::endl;
                std::cerr << "Continue with trying ALL." << std::endl;
                std::cerr << "Error message:" << std::endl;
                std::cerr << err.what() << std::endl;
            }
        }

        if (!context_created) {
            try {
                context_ = cl::Context(CL_DEVICE_TYPE_ALL, context_properties);
                context_created = true;
            } catch (cl::Error err) {
                std::cerr << "Failed to create context for ALL." << std::endl;
                throw cl::Error(err);
            }
        }

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
    }
}; // class StateCL

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

    void create_kernel (Particle<T> &particle)
    {
        assert(particle.value().build());

        if (!kernel_created_) {
            kernel_ = cl::Kernel(
                    particle.value().program(), kernel_name_.c_str());
            kernel_created_ = true;
        }

        kernel_.setArg(0, particle.value().state_device());
        kernel_.setArg(1, particle.value().weight_device());
        kernel_.setArg(2, particle.value().accept_device());
    }

    virtual unsigned operator() (Particle<T> &particle, void *param)
    {
        initialize_param(particle, param);
        pre_processor(particle);
        // TODO more control over local size
        particle.value().command_queue().enqueueNDRangeKernel(kernel_,
                cl::NullRange, cl::NDRange(particle.size()), cl::NullRange);
        particle.value().command_queue().enqueueBarrier();
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

    virtual void pre_processor (Particle<T> &particle)
    {
        create_kernel(particle);
    }

    virtual void post_processor (Particle<T> &particle) {}

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    bool kernel_created_;
    typename Particle<T>::weight_type weight_;
};

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

    void create_kernel (unsigned iter, Particle<T> &particle)
    {
        assert(particle.value().build());

        if (!kernel_created_) {
            kernel_ = cl::Kernel(
                    particle.value().program(), kernel_name_.c_str());
            kernel_created_ = true;
        }

        kernel_.setArg(0, (cl_uint) iter);
        kernel_.setArg(1, particle.value().state_device());
        kernel_.setArg(2, particle.value().weight_device());
        kernel_.setArg(3, particle.value().accept_device());
    }

    virtual unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        pre_processor(iter, particle);
        // TODO more control over local size
        particle.value().command_queue().enqueueNDRangeKernel(kernel_,
                cl::NullRange, cl::NDRange(particle.size()), cl::NullRange);
        particle.value().command_queue().enqueueBarrier();
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

    virtual void pre_processor (unsigned iter, Particle<T> &particle)
    {
        create_kernel(iter, particle);
    }

    virtual void post_processor (unsigned iter, Particle<T> &particle) {}

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    bool kernel_created_;
    typename Particle<T>::weight_type weight_;
}; // class MoveCL

} // namespace vSMC

#endif // V_SMC_HELPER_PARALLEL_CL_HPP

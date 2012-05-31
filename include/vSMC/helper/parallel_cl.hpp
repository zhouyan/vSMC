#ifndef V_SMC_HELPER_PARALLEL_CL_HPP
#define V_SMC_HELPER_PARALLEL_CL_HPP

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
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynmaic> state_mat_type;
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> weight_vec_type;
    typedef Eigen::Matrix<cl_uint, Eigen::Dynamic, 1> accept_vec_type;

    explicit StateBase (size_type N) :
        size_(N), state_host_(Dim, N), weight_host_(N), accept_host_(N)
    {
        try {
            cl_initialize();
        } catch (cl::Error err) {
            std::cerr << err.what() << std::endl;
            std::abort();
        }
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

    std::vector<cl::Device> &device ()
    {
        return device_;
    }

    const std::vector<cl::Device> &device () const
    {
        return device_;
    }

    std::vector<cl::Platform> &platform ()
    {
        return platform_;
    }

    const std::vector<cl::Platform> &platform () const
    {
        return platform_;
    }

    const state_mat_type &state_host () const
    {
        command_queue_.enqueueReadBuffer(state_device_, 1, 0, size() * dim(),
                (void *) state_host_.data());
        return state_host_;
    }

    const cl::Buffer &state_device () const
    {
        return state_device_;
    }

    const weight_vec_type &weight_host () const
    {
        command_queue_.enqueueReadBuffer(weight_device_, 1, 0, size() * dim(),
                (void *) weight_host_.data());
        return weight_host_;
    }

    cl::Buffer &weight_device ()
    {
        return weight_device_;
    }

    const accept_vec_type &accept_host () const
    {
        command_queue_.enqueueReadBuffer(accept_device_, 1, 0, size() * dim(),
                (void *) accept_host_.data());
        return accept_host_;
    }

    cl::Buffer &accept_device ()
    {
        return accept_device_;
    }

    // TODO copy

    private :

    size_type size_;
    cl::CommandQueue command_queue_;
    cl::Context context_;
    cl::Program program_;
    std::vector<cl::Device> device_;
    std::vector<cl::Platform> platform_;

    state_mat_type state_host_;
    weight_vec_type weight_host_;
    accept_vec_type accept_host_;

    cl::Buffer state_device_;
    cl::Buffer weight_device_;
    cl::Buffer accept_device_;

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
            } catch (cl::Error err) {
                std::cerr << "Failed to create context for ALL." << std::endl;
                throw cl::Error(err);
            }
        }

        device_= context_.getInfo<CL_CONTEXT_DEVICES>();

        command_queue_ = cl::CommandQueue(context_, device_[0], 0);

        state_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(T) * size() * dim(), state_host_.data());
        weight_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(T) * size(), weight_host_.data());
        accept_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(cl_uint) * size(), accept_host_.data());
    }
}; // class StateCL

/// \tparam T A subtype of StateCL
template <typename T>
class InitializeCL
{
    public :

    explicit InitializeCL (const char *kernel_name) :
        kernel_name_(kernel_name), kernel_created_(false) {}

    InitializeCL (const InitializeCL<T> &other) :
        kernel_(other.kernel_), kernel_name_(other.kernel_name_),
        kernel_created_(other.kerenel_created_) {}

    const InitializeCL<T> &operator= (const InitializeCL<T> &other)
    {
        kernel_ = other.kernel_;
        kernel_name_ = other.kernel_name_;
        kernel_created_ = other.kernel_created_;
    }

    virtual ~InitializeCL () {}

    virtual unsigned operator() (Particle<T> &particle, void *param)
    {
        if (param) {
            param = static_cast<const char *>(param);
            std::stringstream ss;

            if (sizeof(typename T::state_type) == sizeof(cl_float))
                ss << "typedef float state_type\n";
            else if (sizeof(typename T::state_type) == sizeof(cl_double))
                ss << "typedef double state_type\n";
            else
                throw std::runtime_error("Unsupported OpenCL types");

            ss << "struct state_vector {\n";
            for (unsigned d = 0; d != particle.value().dim(); ++d)
                ss << "state_type param" << d + 1 << ";\n";
            ss << "}\n";

            // TODO create program

            kernel_created_ = false;
        }

        if (!kernel_created_) {
            kernel_ = cl::Kernel(
                    particle.value().program(), kernel_name_.c_str());
        }

        kernel_.setArg(0, particle.value().state_device());
        kernel_.setArg(1, particle.value().weight_device());
        kernel_.setArg(2, particle.value().accept_device());

        // TODO more control over local size
        particle.value().command_queue().enqueueNDRangeKernel(kernel_,
                cl::NullRange, cl::NDRange(particle.size()), cl::NullRange);

        // TODO more efficient weight copying
        typename T::weight_vec_type &weight = particle.value().weight_host();
        for (typename T::size_type i = 0; i != particle.size(); ++i)
            weight_[i] = weight[i];
        particle.set_log_weight(weight_);

        return particle.value().accept_host().sum();
    }

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    bool kernel_created_;
    typename Particle<T>::weight_type weight_;
}

/// \tparam T A subtype of StateCL
template <typename T>
class MoveCL
{
    public :

    explicit MoveCL (const char *kernel_name) :
        kernel_name_(kernel_name), kernel_created_(false) {}

    MoveCL (const MoveCL<T> &other) :
        kernel_(other.kernel_), kernel_name_(other.kernel_name_),
        kernel_created_(other.kerenel_created_) {}

    const MoveCL<T> &operator= (const MoveCL<T> &other)
    {
        kernel_ = other.kernel_;
        kernel_name_ = other.kernel_name_;
        kernel_created_ = other.kernel_created_;
    }

    virtual ~MoveCL () {}

    unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        if (!kernel_created_) {
            kernel_ = cl::Kernel(
                    particle.value().program(), kernel_name_.c_str());
        }

        kernel_.setArg(0, iter);
        kernel_.setArg(1, particle.value().state_device());
        kernel_.setArg(2, particle.value().weight_device());
        kernel_.setArg(3, particle.value().accept_device());

        // TODO more control over local size
        particle.value().command_queue().enqueueNDRangeKernel(kernel_,
                cl::NullRange, cl::NDRange(particle.size()), cl::NullRange);

        // TODO more efficient weight copying
        typename T::weight_vec_type &weight = particle.value().weight_host();
        for (typename T::size_type i = 0; i != particle.size(); ++i)
            weight_[i] = weight[i];
        particle.add_log_weight(weight_);

        return particle.value().accept_host().sum();
    }

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    bool kernel_created_;
    typename Particle<T>::weight_type weight_;
}; // class MoveCL

} // namespace vSMC

#endif // V_SMC_HELPER_PARALLEL_CL_HPP

#ifndef V_SMC_HELPER_PARALLEL_CL_HPP
#define V_SMC_HELPER_PARALLEL_CL_HPP

#include <vSMC/internal/common.hpp>
#include <vSMC/helper/parallel_cl/cl.hpp>

namespace vSMC {

template <unsigned Dim, typename T>
void pre_copy (Particle<StateCL<Dim, T> > &particle)
{
    particle.value().pre_copy();
}

template <unsigned Dim, typename T>
void post_copy (Particle<StateCL<Dim, T> > &particle)
{
    particle.value().post_copy();
}

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
    typedef Eigen::Matrix<cl_uint, Eigen::Dynamic, 1> copy_vec_type;

    explicit StateBase (size_type N) :
        size_(N), program_created_(false),
        state_host_(Dim, N), weight_host_(N), accept_host_(N), copy_host_(N)
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
        command_queue_.enqueueReadBuffer(state_device_, 1, 0, size_ * Dim,
                (void *) state_host_.data());
        return state_host_;
    }

    const cl::Buffer &state_device () const
    {
        return state_device_;
    }

    const weight_vec_type &weight_host () const
    {
        command_queue_.enqueueReadBuffer(weight_device_, 1, 0, size_,
                (void *) weight_host_.data());
        return weight_host_;
    }

    cl::Buffer &weight_device ()
    {
        return weight_device_;
    }

    const accept_vec_type &accept_host () const
    {
        command_queue_.enqueueReadBuffer(accept_device_, 1, 0, size_,
                (void *) accept_host_.data());
        return accept_host_;
    }

    cl::Buffer &accept_device ()
    {
        return accept_device_;
    }

    void compile ()
    {
        // TODO compile
    }

    bool program_created () const
    {
        return program_created_;
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
        assert(program_created_);

        kernel_copy_.setArg(0, state_device_);
        kernel_copy_.setArg(1, copy_device_);
        command_queue_.enqueueWriteBuffer(copy_device_, 1, 0, size_,
                (void *) copy_host_.data());
        command_queue_.enqueueNDRangeKernel(kernel_copy_,
                cl::NullRange, cl::NDRange(cl::NullRange), cl::NullRange);
    }

    private :

    size_type size_;

    cl::CommandQueue command_queue_;
    cl::Context context_;
    cl::Program program_;
    std::vector<cl::Platform> platform_;
    std::vector<cl::Device> device_;

    bool program_created_;

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
                sizeof(T) * size_ * Dim, state_host_.data());
        weight_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(T) * size_, weight_host_.data());
        accept_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(cl_uint) * size_, accept_host_.data());
        copy_device_ = cl::Buffer(context_, CL_MEM_READ_WRITE,
                sizeof(cl_uint) * size_, copy_host_.data());
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
        assert(particle.value().program_created());

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
        assert(particle.value().program_created());

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

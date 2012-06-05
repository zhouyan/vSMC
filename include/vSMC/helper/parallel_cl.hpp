#ifndef V_SMC_HELPER_PARALLEL_CL_HPP
#define V_SMC_HELPER_PARALLEL_CL_HPP

#define __CL_ENABLE_EXCEPTIONS

#include <vSMC/internal/common.hpp>
#include <vSMC/helper/parallel_cl/cl.hpp>
#include <vSMC/helper/parallel_cl/query.hpp>

namespace vSMC {

/// \brief Particle<T>::value_type subtype
/// \ingroup OpenCL
///
/// \tparam Dim The dimension of the state parameter vector
/// \tparam T The type of the value of the state parameter vector
template <unsigned Dim, typename T>
class StateCL : public StateCLTrait
{
    public :

    /// The type of the size of the particle set
    typedef V_SMC_INDEX_TYPE size_type;

    /// The type of state parameters (cl_float or cl_double)
    typedef T state_type;

    /// The type of the matrix of states returned by state_host()
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> state_mat_type;

    /// The type of the vector of weights returned by weight_host()
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> weight_vec_type;

    /// The type of the vector of accept counts returned by accept_host()
    typedef Eigen::Matrix<cl_uint, Eigen::Dynamic, 1> accept_vec_type;

    /// The type of the vector of copy sources
    typedef Eigen::Matrix<cl_uint, Eigen::Dynamic, 1> copy_vec_type;

    explicit StateCL (size_type N) :
        size_(N),
        platform_created_(false), context_created_(false),
        device_created_(false), command_queue_created_(false),
        program_created_(false), build_(false),
        global_size_(N), local_size_(0),
        state_host_(Dim, N), weight_host_(N), accept_host_(N), copy_host_(N)
    {}

    /// \brief The dimension of the problem
    ///
    /// \return The dimension of the parameter vector
    static unsigned dim ()
    {
        return Dim;
    }

    /// \brief The dimension of the problem
    ///
    /// \return The dimension of the parameter vector
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

    /// \brief Set a new local group size
    ///
    /// \param lsize The size of the work group.
    void local_size (std::size_t lsize)
    {
        local_size_ = lsize;
        if (local_size_ && size_ % local_size_)
            global_size_ = (size_ / local_size_ + 1) * local_size_;
        else
            global_size_ = size_;
    }

    /// \brief The global cl::NDRange used by kernel calls
    ///
    /// \return A cl::NDRange object that is valid for the current setting of
    /// local group size and larger than the particle set size.
    cl::NDRange global_nd_range () const
    {
        return cl::NDRange(global_size_);
    }

    /// \brief The local cl::NDRange used by kernel calls
    ///
    /// \return A cl::NDRange object that can be used as the group size when
    /// call a kernel. It is the same size of set by local_size() or
    /// cl::NullRange if local_size() is set with zero.
    cl::NDRange local_nd_range () const
    {
        if (local_size_)
            return cl::NDRange(local_size_);
        else
            return cl::NullRange;
    }

    /// \brief Read only access to the memory buffer on the device that stores
    /// the states
    ///
    /// \return A const reference to the cl::Buffer object. It is read only
    /// only with regard to the host code. The device code have read and write
    /// access to the buffer.
    const cl::Buffer &state_device () const
    {
        return state_device_;
    }

    /// \brief Read only access to the states
    ///
    /// \return A const reference to a matrix of the states which have the same
    /// contents as stored in the device memory buffer of state_device().
    ///
    /// \note This call will read the buffer from the device first, and
    /// therefore is expensive.
    const state_mat_type &state_host () const
    {
        read_buffer<state_type>(state_device_, size_*Dim, state_host_.data());

        return state_host_;
    }

    /// \brief Read only access to the memory buffer on the device that stores
    /// the weights
    ///
    /// \return A const reference to the cl::Buffer object. It is read only
    /// only with regard to the host code. The device code have read and write
    /// access to the buffer.
    ///
    /// \note Despite the names, this is not the current weights of the
    /// particle system. It is intended to be used by clients to store and
    /// manipulate the weights, for example the incremental weights. For access
    /// to actual weights and log weights, always use the interface of
    /// Particle.
    const cl::Buffer &weight_device () const
    {
        return weight_device_;
    }

    /// \brief Read only access to the weights
    ///
    /// \return A const reference to a matrix of the states which have the same
    /// contents as stored in the device memory buffer of weight_device().
    ///
    /// \note This call will read the buffer from the device first, and
    /// therefore is expensive.
    const weight_vec_type &weight_host () const
    {
        read_buffer<state_type>(weight_device_, size_, weight_host_.data());

        return weight_host_;
    }

    /// \brief Read only access to the memory buffer on the device that stores
    /// the accept counts
    ///
    /// \return A const reference to the cl::Buffer object. It is read only
    /// only with regard to the host code. The device code have read and write
    /// access to the buffer.
    ///
    /// \note Despite the names, this is not the current accept counts of the
    /// particle system. It is intended to be used by clients to store and
    /// manipulate the accept counts.
    const cl::Buffer &accept_device () const
    {
        return accept_device_;
    }

    /// \brief Read only access to the accept counts
    ///
    /// \return A const reference to a matrix of the states which have the same
    /// contents as stored in the device memory buffer of accept_device().
    ///
    /// \note This call will read the buffer from the device first, and
    /// therefore is expensive.
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

    /// \brief Check the status of build
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
    /// \note The call to build() will call setup() unless the user have set
    /// platform, context, device and command_queue correctly himself. If these
    /// are set by the user, than the library does not check for its validity.
    ///
    /// \note When building the program, the user can assume the following
    /// \li The Rand123 library's \c philox.h, \c threefry.h, and \c u01.h
    /// headers are included before the user source.
    /// \li A type \c state_type which is the same as the host \c state_type
    /// are defined before the user source and looks like
    /// \code
    /// typedef cl_float state_type;
    /// \endcode
    /// given that the template parameter \c T is \c cl_float
    /// \li A type \c state_struct which looks like the following are defined
    /// before the user souce.
    /// \code
    /// typedef {
    ///     state_type param1;
    ///     state_type param2;
    ///     state_type param3;
    ///     state_type param4;
    /// } state_struct;
    /// \endcode
    /// given that the template parameter \c Dim is 4.
    /// \li A constant \c Dim of type \c uint is defined before the user source
    /// which is the same as the dimension of this StateCL object and looks
    /// like
    /// \code
    /// __constant uint Dim = 4;
    /// \endcode
    /// given that the template parameter \c T is \c cl_float
    void build (const char *source, const char *flags)
    {
        assert(setup());

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
        build_ = true;
    }

    /// \brief Check the status of build
    ///
    /// \return \b true if the last call to build is successful.
    bool build () const
    {
        return build_;
    }

    /// \brief Create a device buffer with given number of elements and type
    ///
    /// \tparam CLType An OpenCL type
    /// \param num The number of elements
    ///
    /// \return A cl::Buffer object corresponding to the newly created buffer
    template<typename CLType>
    cl::Buffer buffer (std::size_t num) const
    {
        assert(setup());

        return cl::Buffer(context_, CL_MEM_READ_WRITE, sizeof(CLType) * num);
    }

    /// \brief Create a device buffer with input from host iterators
    ///
    /// \tparam CLType An OpenCL Type
    /// \tparam InputIter The type of input iterator reading the host data
    /// \param first The begin of the input
    /// \param last The end of the input
    ///
    /// \return A cl::Buffer object corresponding to the newly created buffer
    /// which contains the input from the host. 
    template<typename CLType, typename InputIter>
    cl::Buffer buffer (InputIter first, InputIter last) const
    {
        assert(setup());

        std::size_t num = 0;
        for (InputIter i = first; i != last; ++i)
            ++num;
        cl::Buffer buf(buffer<CLType>(num));
        if (!num)
            return buf;

        write_buffer<CLType>(buf, num, first);

        return buf;
    }

    /// \brief Read a device buffer input a host iterator
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
        assert(setup());

        if (internal::is_pointer<OutputIter>::value) {
            if (sizeof(typename internal::remove_pointer<OutputIter>::type) ==
                    sizeof(CLType)) {
                command_queue_.enqueueReadBuffer(buf, 1, 0,
                        sizeof(CLType) * num, (void *) first);
                return;
            }
        }


        std::vector<CLType> temp(num);
        command_queue_.enqueueReadBuffer(buf, 1, 0,
                sizeof(CLType) * num, (void *) temp.data());
        std::copy(temp.begin(), temp.end(), first);
    }

    /// \brief Write to a device buffer from a host iterator
    ///
    /// \tparam CLType An OpenCL Type
    /// \tparam Input The type of input iterator reading the host data
    /// \param buf The device buffer to be write
    /// \param num The number of elements in the device buffer
    /// \param first The begin of the input
    template <typename CLType, typename InputIter>
    void write_buffer (const cl::Buffer &buf, std::size_t num,
            InputIter first) const
    {
        assert(setup());

        if (internal::is_pointer<InputIter>::value) {
            if (sizeof(typename internal::remove_pointer<InputIter>::type) ==
                    sizeof(CLType)) {
                command_queue_.enqueueWriteBuffer(buf, 1, 0,
                        sizeof(CLType) * num, (void *) first);
                return;
            }
        }

        std::vector<CLType> temp(num);
        std::copy(first, first + num, temp.begin());
        command_queue_.enqueueWriteBuffer(buf, 1, 0,
                sizeof(CLType) * num, (void *) temp.data());
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

        write_buffer<cl_uint>(copy_device_, size_, copy_host_.data());
        kernel_copy_.setArg(0, (std::size_t) size_);
        kernel_copy_.setArg(1, state_device_);
        kernel_copy_.setArg(2, copy_device_);
        command_queue_.enqueueNDRangeKernel(kernel_copy_,
                cl::NullRange, global_nd_range(), local_nd_range());
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
}; // class StateCL

/// \brief Sampler<T>::init_type subtype
/// \ingroup OpenCL
///
/// \tparam T A subtype of StateCL
///
/// \note
/// A valid kernel declaration looks like
/// \code
/// __kernel void init (
///     size_t size, state_struct *state,
///     state_type *weight, uint *accept)
/// \endcode
/// There can has other user supplied arguments as long as the first four is as
/// above
template <typename T>
class InitializeCL : public InitializeCLTrait
{
    public :

    virtual unsigned operator() (Particle<T> &particle, void *param)
    {
        create_kernel(particle);
        initialize_param(particle, param);
        pre_processor(particle);
        // TODO more control over local size
        particle.value().command_queue().enqueueNDRangeKernel(
                kernel_, cl::NullRange,
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
    virtual void initialize_state (std::string &kernel_name) = 0;
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
        // TODO More robust checking of kernel change
        assert(particle.value().build());
        std::string kname;
        initialize_state(kname);

        if (kernel_name_ != kname) {
            kernel_name_ = kname;
            kernel_ = cl::Kernel(
                    particle.value().program(), kernel_name_.c_str());
        }

        kernel_.setArg(0, (std::size_t) particle.size());
        kernel_.setArg(1, particle.value().state_device());
        kernel_.setArg(2, particle.value().weight_device());
        kernel_.setArg(3, particle.value().accept_device());
    }

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    typename Particle<T>::weight_type weight_;
};

/// \brief Sampler<T>::move_type subtype
/// \ingroup OpenCL
///
/// \tparam T A subtype of StateCL
///
/// \note
/// A valid kernel declaration looks like
/// \code
/// __kernel void move (
///     size_t size, uint iter, state_struct *state,
///     state_type *weight, uint *accept)
/// \endcode
/// There can has other user supplied arguments as long as the first five is as
/// above
template <typename T>
class MoveCL : public MoveCLTrait
{
    public :

    virtual unsigned operator() (unsigned iter, Particle<T> &particle)
    {
        create_kernel(iter, particle);
        pre_processor(iter, particle);
        // TODO more control over local size
        particle.value().command_queue().enqueueNDRangeKernel(
                kernel_, cl::NullRange,
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

    virtual void move_state (unsigned iter, std::string &kernel_name) = 0;
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
        // TODO More robust checking of kernel change
        assert(particle.value().build());
        std::string kname;
        move_state(iter, kname);

        if (kernel_name_ != kname) {
            kernel_name_ = kname;
            kernel_ = cl::Kernel(
                    particle.value().program(), kernel_name_.c_str());
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
    typename Particle<T>::weight_type weight_;
}; // class MoveCL

/// \brief Monitor<T>::eval_type subtype
/// \ingroup OpenCL
///
/// \tparam T A Subtype of StateCL
/// \tparam Dim The dimension of the monitor
///
/// \note
/// A valid kernel declaration looks like
/// \code
/// __kernel void monitor_eval (
///     size_t size, uint iter, uint dim, state_struct *state,
///     state_type *buffer)
/// \endcode
/// where \c buffer is a row major \c dim by \c size matrix. There can has
/// other user supplied arguments as long as the first five is as above
///
/// \note Currently Dim cannot be larger than particle set size
template <typename T, unsigned Dim>
class MonitorCL : public MonitorCLTrait
{
    public :

    virtual void operator() (unsigned iter, const Particle<T> &particle,
            double *res)
    {
        create_kernel(iter, particle);
        pre_processor(iter, particle);
        particle.value().command_queue().enqueueNDRangeKernel(
                kernel_, cl::NullRange,
                particle.value().global_nd_range(),
                particle.value().local_nd_range());
        if (sizeof(typename T::state_type) == sizeof(double)) {
            particle.value().template read_buffer<typename T::state_type>(
                    buffer_device_, particle.size() * Dim, res);
        } else {
            buffer_host_.resize(Dim, particle.size());
            particle.value().template read_buffer<typename T::state_type>(
                    buffer_device_,
                    particle.size() * Dim, buffer_host_.data());
            std::copy(buffer_host_.data(),
                    buffer_host_.data() + particle.size() * Dim, res);
        }
        post_processor(iter, particle);
    }

    virtual void monitor_state (unsigned iter, std::string &kernel_name) = 0;
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
        // TODO More robust checking of kernel change
        assert(particle.value().build());
        std::string kname;
        monitor_state(iter, kname);

        if (kernel_name_ != kname) {
            kernel_name_ = kname;
            kernel_ = cl::Kernel(
                    particle.value().program(), kernel_name_.c_str());
            buffer_device_ = cl::Buffer(particle.value().context(),
                    CL_MEM_READ_WRITE,
                    sizeof(typename T::state_type) * Dim * particle.size());
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
    cl::Buffer buffer_device_;
    typename T::state_mat_type buffer_host_;
}; // class MonitorCL

/// \brief Path<T>::eval_type subtype
/// \ingroup OpenCL
///
/// \tparam T A subtype of StateCL
///
/// \note
/// A valid kernel declaration looks like
/// \code
/// __kernel void path_eval (
///     size_t size, uint iter, state_struct *state,
///     state_type *buffer)
/// \endcode
/// There can has other user supplied arguments as long as the first four is as
/// above
template <typename T>
class PathCL : public PathCLTrait
{
    public :

    virtual double operator() (unsigned iter, const Particle<T> &particle,
        double *res)
    {
        create_kernel(iter, particle);
        pre_processor(iter, particle);
        particle.value().command_queue().enqueueNDRangeKernel(
                kernel_, cl::NullRange,
                particle.value().global_nd_range(),
                particle.value().local_nd_range());
        if (sizeof(typename T::state_type) == sizeof(double)) {
            particle.value().template read_buffer<typename T::state_type>(
                    buffer_device_, particle.size(), res);
        } else {
            buffer_host_.resize(particle.size());
            particle.value().template read_buffer<typename T::state_type>(
                    buffer_device_, particle.size(), buffer_host_.data());
            std::copy(buffer_host_.data(),
                    buffer_host_.data() + particle.size(), res);
        }
        post_processor(iter, particle);

        return this->width_state(iter, particle);
    }

    virtual void path_state (unsigned iter, std::string &kernel_name) = 0;
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
        // TODO More robust checking of kernel change
        assert(particle.value().build());
        std::string kname;
        path_state(iter, kname);

        if (kernel_name_ != kname) {
            kernel_name_ = kname;
            kernel_ = cl::Kernel(
                    particle.value().program(), kernel_name_.c_str());
            buffer_device_ = cl::Buffer(particle.value().context(),
                    CL_MEM_READ_WRITE,
                    sizeof(typename T::state_type) * particle.size());
        }

        kernel_.setArg(0, (std::size_t) particle.size());
        kernel_.setArg(1, (cl_uint) iter);
        kernel_.setArg(2, particle.value().state_device());
        kernel_.setArg(3, buffer_device_);
    }

    private :

    cl::Kernel kernel_;
    std::string kernel_name_;
    cl::Buffer buffer_device_;
    typename T::weight_vec_type buffer_host_;
}; // class PathCL

} // namespace vSMC

#endif // V_SMC_HELPER_PARALLEL_CL_HPP

#ifndef VSMC_UTILITY_CL_MANIP_HPP
#define VSMC_UTILITY_CL_MANIP_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/internal/cl_wrapper.hpp>

namespace vsmc {

/// \brief Configure OpenCL runtime behavior (used by MoveCL etc)
/// \ingroup OpenCL
class ConfigureCL
{
    public :

    ConfigureCL () : local_size_(0) {}

    std::size_t local_size () const {return local_size_;}

    void local_size (std::size_t new_size) {local_size_ = new_size;}

    void local_size (const cl::Kernel &kern, const cl::Device &dev)
    {local_size(preferred_local_size(kern, dev));}

    static std::size_t preferred_local_size (
            const cl::Kernel &kern, const cl::Device &dev)
    {
        std::size_t max_s;
        std::size_t mul_s;
        kern.getWorkGroupInfo(dev, CL_KERNEL_WORK_GROUP_SIZE, &max_s);
        kern.getWorkGroupInfo(dev,
                CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, &mul_s);

        return (max_s / mul_s) * mul_s;
    }

    static std::size_t preferred_global_size (
            std::size_t N, std::size_t local_size)
    {
        return (local_size && N % local_size) ?
            (N / local_size + 1) * local_size : N;
    }

    private :

    std::size_t local_size_;
}; // class ConfigureCL

/// \brief Print build log
/// \ingroup OpenCL
template <typename ID>
inline void cl_print_build_log (cl::Program &program)
{
    CLManager<ID> &manager = CLManager<ID>::instance();

    cl_build_status status = CL_BUILD_SUCCESS;
    std::string line(78, '=');
    line += "\n";
    std::string log;
    std::string dname;

    for (std::vector<cl::Device>::const_iterator
            diter = manager.device_vec().begin();
            diter != manager.device_vec().end(); ++diter) {
        program.getBuildInfo(*diter, CL_PROGRAM_BUILD_STATUS, &status);
        if (status != CL_BUILD_SUCCESS) {
            program.getBuildInfo(manager.device(), CL_PROGRAM_BUILD_LOG, &log);
            diter->getInfo((cl_device_info) CL_DEVICE_NAME, &dname);
            std::cout << line << "Build failed for : " << dname << std::endl;
            std::cout << line << log << std::endl << line << std::endl;
        }
    }
}

#if VSMC_HAS_CXX11_VARIADIC_TEMPLATES

inline void cl_set_kernel_args (cl::Kernel &, std::size_t) {}

/// \brief Set OpenCL kernel arguments
/// \ingroup OpenCL
///
/// \details
/// The first argument `arg1` is set with index `offset` and the next with
/// index `offset + 1` and so on. For C++11 implementations that support
/// variadic template, there can be arbitrary number of arguments.
/// Otherwise this function supports up to 16 arguments.
template <typename Arg1, typename... Args>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Args &...args)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, args...);
}

#else // VSMC_HAS_CXX11_VARIADIC_TEMPLATES

template <typename Arg1>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1)
{
    kern.setArg(offset, arg1);
}

template <typename Arg1, typename Arg2>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2);
}

template <typename Arg1, typename Arg2, typename Arg3>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
            const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
    typename Arg5>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
    typename Arg5, typename Arg6>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
    typename Arg5, typename Arg6, typename Arg7>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6,
        const Arg7 &arg7)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6,
            arg7);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
    typename Arg5, typename Arg6, typename Arg7, typename Arg8>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6,
        const Arg7 &arg7, const Arg8 &arg8)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6,
            arg7, arg8);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
         typename Arg5, typename Arg6, typename Arg7, typename Arg8,
    typename Arg9>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6,
        const Arg7 &arg7, const Arg8 &arg8, const Arg9 &arg9)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6,
            arg7, arg8, arg9);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
         typename Arg5, typename Arg6, typename Arg7, typename Arg8,
    typename Arg9, typename Arg10>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6,
        const Arg7 &arg7, const Arg8 &arg8, const Arg9 &arg9,
        const Arg10 &arg10)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6,
            arg7, arg8, arg9, arg10);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
         typename Arg5, typename Arg6, typename Arg7, typename Arg8,
    typename Arg9, typename Arg10, typename Arg11>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6,
        const Arg7 &arg7, const Arg8 &arg8, const Arg9 &arg9,
        const Arg10 &arg10, const Arg11 &arg11)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6,
            arg7, arg8, arg9, arg10, arg11);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
         typename Arg5, typename Arg6, typename Arg7, typename Arg8,
    typename Arg9, typename Arg10, typename Arg11, typename Arg12>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6,
        const Arg7 &arg7, const Arg8 &arg8, const Arg9 &arg9,
        const Arg10 &arg10, const Arg11 &arg11, const Arg12 &arg12)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6,
            arg7, arg8, arg9, arg10, arg11, arg12);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
         typename Arg5, typename Arg6, typename Arg7, typename Arg8,
         typename Arg9, typename Arg10, typename Arg11, typename Arg12,
    typename Arg13>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6,
        const Arg7 &arg7, const Arg8 &arg8, const Arg9 &arg9,
        const Arg10 &arg10, const Arg11 &arg11, const Arg12 &arg12,
        const Arg13 &arg13)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6,
            arg7, arg8, arg9, arg10, arg11, arg12, arg13);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
         typename Arg5, typename Arg6, typename Arg7, typename Arg8,
         typename Arg9, typename Arg10, typename Arg11, typename Arg12,
    typename Arg13, typename Arg14>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6,
        const Arg7 &arg7, const Arg8 &arg8, const Arg9 &arg9,
        const Arg10 &arg10, const Arg11 &arg11, const Arg12 &arg12,
        const Arg13 &arg13, const Arg14 &arg14)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6,
            arg7, arg8, arg9, arg10, arg11, arg12, arg13, arg14);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
         typename Arg5, typename Arg6, typename Arg7, typename Arg8,
         typename Arg9, typename Arg10, typename Arg11, typename Arg12,
    typename Arg13, typename Arg14, typename Arg15>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6,
        const Arg7 &arg7, const Arg8 &arg8, const Arg9 &arg9,
        const Arg10 &arg10, const Arg11 &arg11, const Arg12 &arg12,
        const Arg13 &arg13, const Arg14 &arg14, const Arg15 &arg15)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6,
            arg7, arg8, arg9, arg10, arg11, arg12, arg13, arg14, arg15);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
         typename Arg5, typename Arg6, typename Arg7, typename Arg8,
         typename Arg9, typename Arg10, typename Arg11, typename Arg12,
    typename Arg13, typename Arg14, typename Arg15, typename Arg16>
inline void cl_set_kernel_args (cl::Kernel &kern, std::size_t offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6,
        const Arg7 &arg7, const Arg8 &arg8, const Arg9 &arg9,
        const Arg10 &arg10, const Arg11 &arg11, const Arg12 &arg12,
        const Arg13 &arg13, const Arg14 &arg14, const Arg15 &arg15,
        const Arg16 &arg16)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6,
            arg7, arg8, arg9, arg10, arg11, arg12, arg13, arg14, arg15,
            arg16);
}

#endif // VSMC_HAS_CXX11_VARIADIC_TEMPLATES

} // namespace vsmc

#endif // VSMC_UTILITY_CL_MANIP_HPP

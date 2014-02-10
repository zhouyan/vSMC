#ifndef VSMC_OPENCL_CL_MANIP_HPP
#define VSMC_OPENCL_CL_MANIP_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/internal/cl_wrapper.hpp>

namespace vsmc {

/// \brief The maximum local size that is a multiple of the preferred factor
/// \ingroup OpenCL
inline std::size_t cl_maximum_local_size (
        const cl::Kernel &kern, const cl::Device &dev)
{
    std::size_t max_s;
    std::size_t mul_s;
    try {
        kern.getWorkGroupInfo(dev, CL_KERNEL_WORK_GROUP_SIZE, &max_s);
        kern.getWorkGroupInfo(dev,
                CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, &mul_s);
    } catch (cl::Error) {
        return 0;
    }

    return (max_s / mul_s) * mul_s;
}

/// \brief The minimum global size that is a multiple of the local size
/// \ingroup OpenCL
inline std::size_t cl_minimum_global_size (
        std::size_t N, std::size_t local_size)
{
    if (local_size == 0)
        return N;

    return (local_size && N % local_size) ?
        (N / local_size + 1) * local_size : N;
}

/// \brief The preferred global and local size
/// \ingroup OpenCL
///
/// \return The difference between the preferred global size and the N
inline std::size_t cl_preferred_work_size (std::size_t N,
        const cl::Kernel &kern, const cl::Device &dev,
        std::size_t &global_size, std::size_t &local_size)
{
    std::size_t max_s;
    std::size_t min_local;
    try {
        kern.getWorkGroupInfo(dev, CL_KERNEL_WORK_GROUP_SIZE, &max_s);
        kern.getWorkGroupInfo(dev,
                CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, &min_local);
    } catch (cl::Error) {
        global_size = N;
        local_size = 0;

        return 0;
    }

    std::size_t max_local = (max_s / min_local) * min_local;
    std::size_t mul_local = max_local / min_local;

    std::size_t pref_local  = max_local;
    std::size_t pref_global = cl_minimum_global_size(N, max_local);
    std::size_t pref_diff   = pref_global - N;

    for (std::size_t i = mul_local; i != 0; --i) {
        std::size_t local  = i * min_local;
        std::size_t global = cl_minimum_global_size(N, local);
        std::size_t diff   = global - N;
        if (diff < pref_diff) {
            pref_local  = local;
            pref_global = global;
            pref_diff   = diff;
        }
    }

    global_size = pref_global;
    local_size = pref_local;

    return pref_diff;
}

#if VSMC_HAS_CXX11_VARIADIC_TEMPLATES

inline void cl_set_kernel_args (cl::Kernel &, cl_uint) {}

/// \brief Set OpenCL kernel arguments
/// \ingroup OpenCL
///
/// \details
/// The first argument `arg1` is set with index `offset` and the next with
/// index `offset + 1` and so on. For C++11 implementations that support
/// variadic template, there can be arbitrary number of arguments (or limited
/// by the compiler's implementation).  Otherwise this function supports up to
/// 16 arguments.
template <typename Arg1, typename... Args>
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
        const Arg1 &arg1, const Args &...args)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, args...);
}

#else // VSMC_HAS_CXX11_VARIADIC_TEMPLATES

template <typename Arg1>
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
        const Arg1 &arg1)
{
    kern.setArg(offset, arg1);
}

template <typename Arg1, typename Arg2>
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
        const Arg1 &arg1, const Arg2 &arg2)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2);
}

template <typename Arg1, typename Arg2, typename Arg3>
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
            const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4>
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
    typename Arg5>
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
    typename Arg5, typename Arg6>
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
        const Arg1 &arg1, const Arg2 &arg2, const Arg3 &arg3,
        const Arg4 &arg4, const Arg5 &arg5, const Arg6 &arg6)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, arg2, arg3, arg4, arg5, arg6);
}

template <typename Arg1, typename Arg2, typename Arg3, typename Arg4,
    typename Arg5, typename Arg6, typename Arg7>
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
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
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
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
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
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
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
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
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
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
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
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
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
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
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
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
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
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
inline void cl_set_kernel_args (cl::Kernel &kern, cl_uint offset,
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

#endif // VSMC_OPENCL_CL_MANIP_HPP

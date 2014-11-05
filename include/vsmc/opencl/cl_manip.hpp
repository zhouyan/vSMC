//============================================================================
// include/vsmc/opencl/cl_manip.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distributed under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_OPENCL_CL_MANIP_HPP
#define VSMC_OPENCL_CL_MANIP_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/internal/cl_wrapper.hpp>

namespace vsmc {

/// \brief Query the preferred factor of local size
/// \ingroup OpenCL
///
/// \param kern An OpenCL kernel
/// \param dev An OpenCL device
/// \param factor Multiplier factor of local size for optimzied performance
/// \param lmax Maximum of the local size
/// \param mmax Maximum of the multiplier of the factor
inline void cl_minmax_local_size (
        const ::cl::Kernel &kern, const ::cl::Device &dev,
        std::size_t &factor, std::size_t &lmax, std::size_t &mmax)
{
    try {
        kern.getWorkGroupInfo(dev,
                CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, &factor);
        kern.getWorkGroupInfo(dev,
                CL_KERNEL_WORK_GROUP_SIZE, &lmax);
        if (factor == 0 || factor > lmax) {
            factor = lmax = mmax = 0;
            return;
        }
        mmax = lmax / factor;
    } catch (::cl::Error) {
        factor = lmax = mmax = 0;
    }
}

/// \brief The minimum global size that is a multiple of the local size
/// \ingroup OpenCL
inline std::size_t cl_min_global_size (std::size_t N, std::size_t local_size)
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
        const ::cl::Kernel &kern, const ::cl::Device &dev,
        std::size_t &global_size, std::size_t &local_size)
{
    cl::size_t<3> reqd_size;
    try {
        kern.getWorkGroupInfo(dev,
                CL_KERNEL_COMPILE_WORK_GROUP_SIZE, &reqd_size);
    } catch (::cl::Error) {
        reqd_size[0] = 0;
    }

    if (reqd_size[0] != 0) {
        local_size = reqd_size[0];
        global_size = cl_min_global_size(N, local_size);

        return global_size - N;
    }

    std::size_t factor;
    std::size_t lmax;
    std::size_t mmax;
    cl_minmax_local_size(kern, dev, factor, lmax, mmax);
    if (lmax == 0) {
        global_size = N;
        local_size = 0;

        return global_size - N;
    }

    local_size = factor;
    global_size = cl_min_global_size(N, local_size);
    std::size_t diff_size = global_size - N;
    for (std::size_t m = 1; m <= mmax; ++m) {
        std::size_t l = m * factor;
        std::size_t g = cl_min_global_size(N, l);
        std::size_t d = g - N;
        if (d < diff_size) {
            local_size = l;
            global_size = g;
            diff_size = d;
        }
    }

    return diff_size;
}

inline void cl_set_kernel_args (::cl::Kernel &, cl_uint) {}

#if VSMC_HAS_CXX11_VARIADIC_TEMPLATES
/// \brief Set OpenCL kernel arguments
/// \ingroup OpenCL
///
/// \details
/// The first argument `arg1` is set with index `offset` and the next with
/// index `offset + 1` and so on. For C++11 implementations that support
/// variadic template, there can be arbitrary number of arguments (or limited
/// by the compiler's implementation). Otherwise this function supports up to
/// 16 arguments.
template <typename Arg1, typename... Args>
inline void cl_set_kernel_args (::cl::Kernel &kern, cl_uint offset,
        const Arg1 &arg1, const Args &... args)
{
    kern.setArg(offset, arg1);
    cl_set_kernel_args(kern, offset + 1, args...);
}
#else // VSMC_HAS_CXX11_VARIADIC_TEMPLATES
#include <vsmc/opencl/internal/cl_set_kernel_args.hpp>
#endif // VSMC_HAS_CXX11_VARIADIC_TEMPLATES

} // namespace vsmc

#endif // VSMC_OPENCL_CL_MANIP_HPP

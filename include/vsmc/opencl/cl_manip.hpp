//============================================================================
// include/vsmc/opencl/cl_manip.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_OPENCL_CL_MANIP_HPP
#define VSMC_OPENCL_CL_MANIP_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/internal/cl_wrapper.hpp>

namespace vsmc {

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

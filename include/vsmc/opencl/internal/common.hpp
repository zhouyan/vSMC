#ifndef VSMC_OPENCL_INTERNAL_COMMON_HPP
#define VSMC_OPENCL_INTERNAL_COMMON_HPP

#include <vsmc/internal/common.hpp>

#ifdef VSMC_MACOSX
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
#endif

namespace vsmc
{

namespace internal
{

#if VSMC_NO_RUNTIME_ASSERT
inline void cl_error_check(cl_int, const char *, const char *) {}
#else
inline void cl_error_check(cl_int status, const char *func, const char *clf)
{
    if (status == CL_SUCCESS)
        return;

    std::string msg("**");
    msg += func;
    msg += "** failure";
    msg += "; OpenCL function: ";
    msg += clf;
    msg += "; Error code: ";
    msg += itos(status);

    VSMC_RUNTIME_ASSERT((status == CL_SUCCESS), msg.c_str());
}
#endif

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_OPENCL_INTERNAL_COMMON_HPP

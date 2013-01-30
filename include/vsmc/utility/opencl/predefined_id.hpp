#ifndef VSMC_UTILITY_OPENCL_PREDEFINED_ID_HPP
#define VSMC_UTILITY_OPENCL_PREDEFINED_ID_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>
#include <vsmc/cxx11/type_traits.hpp>
#include <vsmc/utility/opencl/cl_wrapper.hpp>

#include <string>

namespace vsmc { namespace opencl {

struct All
{
    typedef cxx11::integral_constant<cl_device_type, CL_DEVICE_TYPE_ALL>
        opencl_device_type;
};

struct CPU
{
    typedef cxx11::integral_constant<cl_device_type, CL_DEVICE_TYPE_CPU>
        opencl_device_type;
};

struct GPU
{
    typedef cxx11::integral_constant<cl_device_type, CL_DEVICE_TYPE_GPU>
        opencl_device_type;
};

struct Accelerator
{
    typedef cxx11::integral_constant<cl_device_type,
        CL_DEVICE_TYPE_ACCELERATOR> opencl_device_type;
};

struct AMD
{
    static bool check_opencl_platform (const std::string &name)
    {return name.find(std::string("AMD")) != std::string::npos;}
};

struct Apple
{
    static bool check_opencl_platform (const std::string &name)
    {return name.find(std::string("Apple")) != std::string::npos;}
};

struct Intel
{
    static bool check_opencl_platform (const std::string &name)
    {return name.find(std::string("Intel")) != std::string::npos;}
};

struct NVIDIA
{
    static bool check_opencl_platform (const std::string &name)
    {return name.find(std::string("NVIDIA")) != std::string::npos;}
};

struct AMDCPU : public AMD, public CPU {};
struct AMDGPU : public AMD, public GPU {};

struct AppleCPU : public Apple, public CPU {};
struct AppleGPU : public Apple, public GPU {};

struct IntelCPU : public Intel, public CPU {};
struct IntelGPU : public Intel, public GPU {};

struct NVIDIACPU : public NVIDIA, public CPU {};
struct NVIDIAGPU : public NVIDIA, public GPU {};

} } // namespace vsmc::opencl

#endif // VSMC_UTILITY_OPENCL_PREDEFINED_ID_HPP

//============================================================================
// vSMC/include/vsmc/opencl/cl_query.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_OPENCL_CL_QUERY_HPP
#define VSMC_OPENCL_CL_QUERY_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/internal/cl_wrapper.hpp>

namespace vsmc {

/// \brief Query OpenCL information
/// \ingroup OpenCL
class CLQuery
{
    public :

    /// \brief Return the OpenCL version of a device
    ///
    /// \return The integer values and the OpenCL versions are mapped as the
    /// following,
    /// - OpenCL 1.0: 100
    /// - OpenCL 1.1: 110
    /// - OpenCL 1.2: 120
    /// - OpenCL 2.0: 200
    ///
    /// \note If the version information returned by `CL_DEVICE_VERSION` does
    /// not follow the specification, then this function will return `100`
    /// (OpenCL 1.0) to play safe..
    static int opencl_version (const ::cl::Device &dev)
    {
        std::string version;
        dev.getInfo(static_cast< ::cl_device_info>(CL_DEVICE_VERSION),
                &version);

        return check_opencl_version(version.substr(7, 3));
    }

    /// \brief Return the OpenCL C version of a device
    ///
    /// \sa opencl_version
    static int opencl_c_version (const ::cl::Device &dev)
    {
        std::string version;
        dev.getInfo(static_cast< ::cl_device_info>(CL_DEVICE_OPENCL_C_VERSION),
                &version);

        return check_opencl_version(version.substr(9, 3));
    }

    /// \brief Query all information
    template <typename CharT, typename Traits>
    static std::basic_ostream<CharT, Traits> &info (
            std::basic_ostream<CharT, Traits> &os)
    {
        if (!os.good())
            return os;

        std::vector< ::cl::Platform> platform;
        ::cl::Platform::get(&platform);
        for (std::vector< ::cl::Platform>::const_iterator p = platform.begin();
                p != platform.end(); ++p)
            info(os, *p);
        print_equal(os);

        return os;
    }

    /// \brief Query platform information
    template <typename CharT, typename Traits>
    static std::basic_ostream<CharT, Traits> &info (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Platform &plat)
    {
        if (!os.good())
            return os;

        print_equal(os);

        print_info_val<std::string, ::cl_platform_info>(os, plat,
                CL_PLATFORM_NAME, "CL_PLATFORM_NAME");
        print_info_val<std::string, ::cl_platform_info>(os, plat,
                CL_PLATFORM_VENDOR, "CL_PLATFORM_VENDOR");
        print_info_val<std::string, ::cl_platform_info>(os, plat,
                CL_PLATFORM_VERSION, "CL_PLATFORM_VERSION");
        print_info_val<std::string, ::cl_platform_info>(os, plat,
                CL_PLATFORM_PROFILE, "CL_PLATFORM_PROFILE");
        print_info_val<std::string, ::cl_platform_info>(os, plat,
                CL_PLATFORM_EXTENSIONS, "CL_PLATFORM_EXTENSIONS");

        std::vector< ::cl::Device> device;
        plat.getDevices(CL_DEVICE_TYPE_ALL, &device);
        for (std::vector< ::cl::Device>::const_iterator d = device.begin();
                d != device.end(); ++d)
            info(os, *d);

        return os;
    }

    /// \brief Query device information
    template <typename CharT, typename Traits>
    static std::basic_ostream<CharT, Traits> &info (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Device &dev)
    {
        if (!os.good())
            return os;

        print_dash(os);
        print_dev_version(os, dev);
        os << '\n';
        print_dev_processor(os, dev);
        os << '\n';
        print_dev_memory(os, dev);
        os << '\n';
        print_dev_vector(os, dev);
#if VSMC_OPENCL_VERSION >= 120
        if (opencl_version(dev) >= 120) {
            os << '\n';
            print_info_val<std::string, ::cl_device_info>(os, dev,
                    CL_DEVICE_BUILT_IN_KERNELS,
                    "CL_DEVICE_BUILT_IN_KERNELS");
            os << '\n';
            print_dev_single_fp_config(os, dev);
            os << '\n';
            print_dev_double_fp_config(os, dev);
            os << '\n';
            print_dev_image_support(os, dev);
        }
#endif // VSMC_OPENCL_VERSION >= 120

        return os;
    }

    /// \brief Query context information
    template <typename CharT, typename Traits>
    static std::basic_ostream<CharT, Traits> &info (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Context &ctx)
    {
        if (!os.good())
            return os;

        std::vector< ::cl::Device> device;
        ctx.getInfo(static_cast< ::cl_context_info>(CL_CONTEXT_DEVICES),
                &device);
        for (std::vector< ::cl::Device>::const_iterator d = device.begin();
                d != device.end(); ++d)
            info(os, *d);

        return os;
    }

    /// \brief Query program informaiton
    template <typename CharT, typename Traits>
    static std::basic_ostream<CharT, Traits> &info (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Program &prog)
    {
        if (!os.good())
            return os;

        print_info_val<cl_uint, ::cl_program_info>(os, prog,
                CL_PROGRAM_NUM_DEVICES, "CL_PROGRAM_NUM_DEVICES");
        print_info_val<std::size_t, ::cl_program_info>(os, prog,
                CL_PROGRAM_BINARY_SIZES, "CL_PROGRAM_BINARY_SIZES", "byte");

        return os;
    }

    /// \brief Query kernel information
    template <typename CharT, typename Traits>
    static std::basic_ostream<CharT, Traits> &info (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Kernel &kern)
    {
        if (!os.good())
            return os;

        ::cl::Context ctx;
        kern.getInfo(static_cast< ::cl_kernel_info>(CL_KERNEL_CONTEXT), &ctx);
        std::vector< ::cl::Device> device;
        ctx.getInfo(static_cast< ::cl_kernel_info>(CL_CONTEXT_DEVICES),
                &device);
        for (std::vector< ::cl::Device>::const_iterator d = device.begin();
                d != device.end(); ++d) {
            print_info_val<std::string, ::cl_device_info>(os, *d,
                    CL_DEVICE_NAME,
                    "CL_DEVICE_NAME");
            print_info_val<std::string, cl_kernel_info>(os, kern,
                    CL_KERNEL_FUNCTION_NAME,
                    "CL_KERNEL_FUNCTION_NAME");
            print_info_val<cl_uint, cl_kernel_info>(os, kern,
                    CL_KERNEL_NUM_ARGS,
                    "CL_KERNEL_NUM_ARGS");
            print_kernwginfo_val<std::size_t>(os, kern, *d,
                    CL_KERNEL_WORK_GROUP_SIZE,
                    "CL_KERNEL_WORK_GROUP_SIZE");
            print_kernwginfo_val<std::size_t>(os, kern, *d,
                    CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE,
                    "CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE");
            print_kernwginfo_val<cl_ulong>(os, kern, *d,
                    CL_KERNEL_LOCAL_MEM_SIZE,
                    "CL_KERNEL_LOCAL_MEM_SIZE", "byte");
            print_kernwginfo_val<cl_ulong>(os, kern, *d,
                    CL_KERNEL_PRIVATE_MEM_SIZE,
                    "CL_KERNEL_PRIVATE_MEM_SIZE", "byte");
        }

        return os;
    }

    /// \brief Query kernel information given a program the name of kernel and
    /// a program
    template <typename CharT, typename Traits>
    static std::basic_ostream<CharT, Traits> &info (
            std::basic_ostream<CharT, Traits> &os,
            const ::cl::Program &prog, const std::string &kname)
    {
        if (!os.good())
            return os;

        ::cl::Kernel kern(prog, kname.c_str());
        info(os, kern);

        return os;
    }

    private :

    static int check_opencl_version (const std::string &version)
    {
#if VSMC_OPENCL_VERSION >= 200
        if (version == std::string("2.0"))
            return 200;
#endif
#if VSMC_OPENCL_VERSION >= 120
        if (version == std::string("1.2"))
            return 120;
#endif
#if VSMC_OPENCL_VERSION >= 110
        if (version == std::string("1.1"))
            return 110;
#endif
        return 100;
    }

    template <typename CharT, typename Traits>
    static void print_equal (std::basic_ostream<CharT, Traits> &os)
    {os << std::string(90, '=') << '\n';}

    template <typename CharT, typename Traits>
    static void print_dash (std::basic_ostream<CharT, Traits> &os)
    {os << std::string(90, '-') << '\n';}

    template <typename CharT, typename Traits>
    static void print_dev_version (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Device &dev)
    {
        print_info_val<std::string, ::cl_device_info>(os, dev,
                CL_DEVICE_NAME, "CL_DEVICE_NAME");
        print_dev_type(os, dev);
        print_info_val<std::string, ::cl_device_info>(os, dev,
                CL_DEVICE_VENDOR, "CL_DEVICE_VENDOR");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_VENDOR_ID, "CL_DEVICE_VENDOR_ID");
        print_info_val<std::string, ::cl_device_info>(os, dev,
                CL_DRIVER_VERSION, "CL_DRIVER_VERSION");
        print_info_val<std::string, ::cl_device_info>(os, dev,
                CL_DEVICE_PROFILE, "CL_DEVICE_PROFILE");
        print_info_val<std::string, ::cl_device_info>(os, dev,
                CL_DEVICE_VERSION, "CL_DEVICE_VERSION");
        print_info_val<std::string, ::cl_device_info>(os, dev,
                CL_DEVICE_OPENCL_C_VERSION, "CL_DEVICE_OPENCL_C_VERSION");
        print_info_val<std::string, ::cl_device_info>(os, dev,
                CL_DEVICE_EXTENSIONS, "CL_DEVICE_EXTENSIONS");
    }

    template <typename CharT, typename Traits>
    static void print_dev_type (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Device &dev)
    {
        ::cl_device_type type;
        std::string info;
        dev.getInfo(static_cast< ::cl_device_info>(CL_DEVICE_TYPE), &type);

        append_bit_field< ::cl_device_type>(CL_DEVICE_TYPE_CPU,
                type, "CL_DEVICE_TYPE_CPU", info);
        append_bit_field< ::cl_device_type>(CL_DEVICE_TYPE_GPU,
                type, "CL_DEVICE_TYPE_GPU", info);
        append_bit_field< ::cl_device_type>(CL_DEVICE_TYPE_ACCELERATOR,
                type, "CL_DEVICE_TYPE_ACCELERATOR", info);
        append_bit_field< ::cl_device_type>(CL_DEVICE_TYPE_DEFAULT,
                type, "CL_DEVICE_TYPE_DEFAULT", info);
        append_bit_field< ::cl_device_type>(CL_DEVICE_TYPE_CUSTOM,
                type, "CL_DEVICE_TYPE_CUSTOM", info);

        print_name(os, "CL_DEVICE_TYPE");
        print_val(os, info);
        os << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_dev_processor (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Device &dev)
    {
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_MAX_CLOCK_FREQUENCY,
                "CL_DEVICE_MAX_CLOCK_FREQUENCY", "MHz");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_MAX_COMPUTE_UNITS,
                "CL_DEVICE_MAX_COMPUTE_UNITS");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS,
                "CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS");
        print_info_val<std::vector<std::size_t>, ::cl_device_info>(os, dev,
                CL_DEVICE_MAX_WORK_ITEM_SIZES,
                "CL_DEVICE_MAX_WORK_ITEM_SIZES");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
                CL_DEVICE_MAX_WORK_GROUP_SIZE,
                "CL_DEVICE_MAX_WORK_GROUP_SIZE");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
                CL_DEVICE_PROFILING_TIMER_RESOLUTION,
                "CL_DEVICE_PROFILING_TIMER_RESOLUTION", "ns");
    }

    template <typename CharT, typename Traits>
    static void print_dev_memory (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Device &dev)
    {
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
                CL_DEVICE_MAX_PARAMETER_SIZE,
                "CL_DEVICE_MAX_PARAMETER_SIZE", "byte");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_ADDRESS_BITS,
                "CL_DEVICE_ADDRESS_BITS", "bit");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_MEM_BASE_ADDR_ALIGN,
                "CL_DEVICE_MEM_BASE_ADDR_ALIGN", "bit");
        print_info_val<cl_ulong, ::cl_device_info>(os, dev,
                CL_DEVICE_MAX_MEM_ALLOC_SIZE,
                "CL_DEVICE_MAX_MEM_ALLOC_SIZE", "byte");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE,
                "CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE", "byte");
        print_info_val<cl_ulong, ::cl_device_info>(os, dev,
                CL_DEVICE_GLOBAL_MEM_CACHE_SIZE,
                "CL_DEVICE_GLOBAL_MEM_CACHE_SIZE", "byte");
        print_info_val<cl_ulong, ::cl_device_info>(os, dev,
                CL_DEVICE_GLOBAL_MEM_SIZE,
                "CL_DEVICE_GLOBAL_MEM_SIZE", "byte");
        print_info_val<cl_ulong, ::cl_device_info>(os, dev,
                CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE,
                "CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE", "byte");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_MAX_CONSTANT_ARGS,
                "CL_DEVICE_MAX_CONSTANT_ARGS");
        print_info_val<cl_ulong, ::cl_device_info>(os, dev,
                CL_DEVICE_LOCAL_MEM_SIZE,
                "CL_DEVICE_LOCAL_MEM_SIZE", "byte");
        print_info_val<cl_bool, ::cl_device_info>(os, dev,
                CL_DEVICE_ERROR_CORRECTION_SUPPORT,
                "CL_DEVICE_ERROR_CORRECTION_SUPPORT");
    }

    template <typename CharT, typename Traits>
    static void print_dev_vector (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Device &dev)
    {
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF");
#if VSMC_OPENCL_VERSION >= 110
        if (opencl_version(dev) >= 110) {
            os << '\n';
            print_info_val<cl_uint, ::cl_device_info>(os, dev,
                    CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR,
                    "CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR");
            print_info_val<cl_uint, ::cl_device_info>(os, dev,
                    CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT,
                    "CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT");
            print_info_val<cl_uint, ::cl_device_info>(os, dev,
                    CL_DEVICE_NATIVE_VECTOR_WIDTH_INT,
                    "CL_DEVICE_NATIVE_VECTOR_WIDTH_INT");
            print_info_val<cl_uint, ::cl_device_info>(os, dev,
                    CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG,
                    "CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG");
            print_info_val<cl_uint, ::cl_device_info>(os, dev,
                    CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT,
                    "CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT");
            print_info_val<cl_uint, ::cl_device_info>(os, dev,
                    CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE,
                    "CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE");
            print_info_val<cl_uint, ::cl_device_info>(os, dev,
                    CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF,
                    "CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF");
        }
#endif // VSMC_OPENCL_VERSION >= 110
    }

#if VSMC_OPENCL_VERSION >= 120
    template <typename CharT, typename Traits>
    static void print_dev_single_fp_config (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Device &dev)
    {
        ::cl_device_fp_config config;
        std::string info;
        dev.getInfo(static_cast< ::cl_device_info>(CL_DEVICE_SINGLE_FP_CONFIG),
                &config);

        append_bit_field< ::cl_device_fp_config>(CL_FP_DENORM,
                config, "CL_FP_DENORM", info);
        append_bit_field< ::cl_device_fp_config>(CL_FP_INF_NAN,
                config, "CL_FP_INF_NAN", info);
        append_bit_field< ::cl_device_fp_config>(CL_FP_ROUND_TO_NEAREST,
                config, "CL_FP_ROUND_TO_NEAREST", info);
        append_bit_field< ::cl_device_fp_config>(CL_FP_ROUND_TO_ZERO,
                config, "CL_FP_ROUND_TO_ZERO", info);
        append_bit_field< ::cl_device_fp_config>(CL_FP_ROUND_TO_INF,
                config, "CL_FP_ROUND_TO_INF", info);
        append_bit_field< ::cl_device_fp_config>(CL_FP_FMA,
                config, "CL_FP_FMA", info);
        append_bit_field< ::cl_device_fp_config>(CL_FP_SOFT_FLOAT,
                config, "CL_FP_SOFT_FLOAT", info);
        append_bit_field< ::cl_device_fp_config>(
                CL_FP_CORRECTLY_ROUNDED_DIVIDE_SQRT,
                config, "CL_FP_CORRECTLY_ROUNDED_DIVIDE_SQRT", info);

        print_name(os, "CL_DEVICE_SINGLE_FP_CONFIG");
        print_val(os, info);
        os << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_dev_double_fp_config (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Device &dev)
    {
        ::cl_device_fp_config config;
        std::string info;
        dev.getInfo(static_cast< ::cl_device_info>(CL_DEVICE_DOUBLE_FP_CONFIG),
                &config);

        append_bit_field< ::cl_device_fp_config>(CL_FP_DENORM,
                config, "CL_FP_DENORM", info);
        append_bit_field< ::cl_device_fp_config>(CL_FP_INF_NAN,
                config, "CL_FP_INF_NAN", info);
        append_bit_field< ::cl_device_fp_config>(CL_FP_ROUND_TO_NEAREST,
                config, "CL_FP_ROUND_TO_NEAREST", info);
        append_bit_field< ::cl_device_fp_config>(CL_FP_ROUND_TO_ZERO,
                config, "CL_FP_ROUND_TO_ZERO", info);
        append_bit_field< ::cl_device_fp_config>(CL_FP_ROUND_TO_INF,
                config, "CL_FP_ROUND_TO_INF", info);
        append_bit_field< ::cl_device_fp_config>(CL_FP_FMA,
                config, "CL_FP_FMA", info);
        append_bit_field< ::cl_device_fp_config>(CL_FP_SOFT_FLOAT,
                config, "CL_FP_SOFT_FLOAT", info);

        print_name(os, "CL_DEVICE_DOUBLE_FP_CONFIG");
        print_val(os, info);
        os << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_dev_image_support (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Device &dev)
    {
        cl_bool support;
        dev.getInfo(static_cast< ::cl_device_info>(CL_DEVICE_IMAGE_SUPPORT),
                &support);
        if (support == 0)
            return;
        print_info_val<cl_bool, ::cl_device_info>(os, dev,
                CL_DEVICE_IMAGE_SUPPORT,
                "CL_DEVICE_IMAGE_SUPPORT");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_MAX_READ_IMAGE_ARGS,
                "CL_DEVICE_MAX_READ_IMAGE_ARGS");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_MAX_WRITE_IMAGE_ARGS,
                "CL_DEVICE_MAX_WRITE_IMAGE_ARGS");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
                CL_DEVICE_IMAGE2D_MAX_WIDTH,
                "CL_DEVICE_IMAGE2D_MAX_WIDTH", "pixel");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
                CL_DEVICE_IMAGE2D_MAX_HEIGHT,
                "CL_DEVICE_IMAGE2D_MAX_HEIGHT", "pixel");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
                CL_DEVICE_IMAGE3D_MAX_WIDTH,
                "CL_DEVICE_IMAGE3D_MAX_WIDTH", "pixel");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
                CL_DEVICE_IMAGE3D_MAX_HEIGHT,
                "CL_DEVICE_IMAGE3D_MAX_HEIGHT", "pixel");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
                CL_DEVICE_IMAGE3D_MAX_DEPTH,
                "CL_DEVICE_IMAGE3D_MAX_DEPTH", "pixel");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
                CL_DEVICE_IMAGE_MAX_BUFFER_SIZE,
                "CL_DEVICE_IMAGE_MAX_BUFFER_SIZE", "pixel");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
                CL_DEVICE_IMAGE_MAX_ARRAY_SIZE,
                "CL_DEVICE_IMAGE_MAX_ARRAY_SIZE");
        print_info_val<cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_MAX_SAMPLERS,
                "CL_DEVICE_MAX_SAMPLERS");
    }
#endif // VSMC_OPENCL_VERSION >= 120

    template <typename T>
    static void append_bit_field (T info, T val,
            const std::string &name, std::string &orig)
    {
        if ((info & val) != 0) {
            if (orig.length() != 0)
                orig.append(" ");
            orig.append(name);
        }
    }

    template <typename T, typename CLInfoType,
        typename ObjType, typename InfoType, typename CharT, typename Traits>
    static void print_info_val (std::basic_ostream<CharT, Traits> &os,
            const ObjType &obj, InfoType info,
            const std::string &name, const std::string &unit = "")
    {
        T val;
        obj.getInfo(static_cast<CLInfoType>(info), &val);
        print_name(os, name);
        print_val(os, val);
        os << ' ' << unit << '\n';
    }

    template <typename T, typename CharT, typename Traits>
    static void print_kernwginfo_val (std::basic_ostream<CharT, Traits> &os,
            const ::cl::Kernel &kern, const ::cl::Device &dev,
            cl_kernel_work_group_info info,
            const std::string &name, const std::string &unit = "")
    {
        T val;
        kern.getWorkGroupInfo(dev,
                static_cast<cl_kernel_work_group_info>(info), &val);
        print_name(os, name);
        print_val(os, val);
        os << ' ' << unit << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_name (
            std::basic_ostream<CharT, Traits> &os, const std::string &name)
    {
        if (name.size() <= 40) {
            char buffer[41];
            std::memset(buffer, ' ', 40);
            for (std::size_t i = 0; i != name.size(); ++i)
                buffer[i] = name[i];
            buffer[40] = '\0';
            os << const_cast<const char *>(buffer) << ' ';
        } else {
            os << name << ' ';
        }
    }

    template <typename T, typename CharT, typename Traits>
    static void print_val (std::basic_ostream<CharT, Traits> &os, const T &val)
    {os << val;}

    template <typename CharT, typename Traits>
    static void print_val (std::basic_ostream<CharT, Traits> &os,
            const std::string &val)
    {
        if (val.size() <= 60) {
            os << val;
            return;
        }

        char sepchar;
        std::size_t pos;
        if ((pos = val.find(' ')) != std::string::npos)
            sepchar = ' ';
        else if ((pos = val.find(';')) != std::string::npos)
            sepchar = ';';
        else {
            os << val;
            return;
        }

        std::string::size_type init = 0;
        std::vector<std::string> val_tmp;
        std::vector<std::string> val_vec;
        while (pos != std::string::npos) {
            val_tmp.push_back(val.substr(init, pos - init + 1));
            init = pos + 1;
            pos = val.find(sepchar, init);
        }
        for (std::size_t i = 0; i != val_tmp.size(); ++i) {
            if (val_tmp[i] != std::string(" "))
                val_vec.push_back(val_tmp[i]);
        }

        if (val_vec.size() == 0)
            return;

        if (val_vec.size() == 1) {
            os << val_vec[0];
            return;
        }

        os << val_vec[0] << '\n';
        for (std::size_t i = 1; i != val_vec.size() - 1; ++i) {
            for (std::size_t s = 0; s != 41; ++s)
                os << ' ';
            os << val_vec[i] << '\n';
        }
        for (std::size_t i = 0; i != 41; ++i)
            os << ' ';
        os << val_vec.back();
    }

    template <typename T, typename CharT, typename Traits>
    static void print_val (std::basic_ostream<CharT, Traits> &os,
            const std::vector<T> &val)
    {
        for (typename std::vector<T>::const_iterator v = val.begin();
                v != val.end(); ++v)
            os << *v << ' ';
    }
}; //class CLQuery

/// \brief Query OpenCL information using CLQuery
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const CLQuery &)
{return CLQuery::info(os);}

} // namespace vsmc

namespace cl {

/// \brief Query device information in a given platform
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const Platform &plat)
{return vsmc::CLQuery::info(os, plat);}

/// \brief Query device information in a given context
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const Context &ctx)
{return vsmc::CLQuery::info(os, ctx);}

/// \brief Query device information
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const Device &dev)
{return vsmc::CLQuery::info(os, dev);}

/// \brief Query program information
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const Program &prog)
{return vsmc::CLQuery::info(os, prog);}

/// \brief Query kernel information
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const Kernel &kern)
{return vsmc::CLQuery::info(os, kern);}

} // namespace cl

#endif // VSMC_OPENCL_CL_QUERY_HPP

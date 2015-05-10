//============================================================================
// vSMC/include/vsmc/opencl/cl_query.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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

#include <vsmc/opencl/internal/common.hpp>
#include <vsmc/opencl/cl_type.hpp>

namespace vsmc
{

/// \brief OpenCL device features
/// \ingroup OpenCL
enum OpenCLDeviceFeature {
    OpenCLDeviceDoubleFP,    ///< Double precision floating points
    OpenCLDeviceImageSupport ///< Image support
};

/// \brief Query OpenCL information
/// \ingroup OpenCL
class CLQuery
{
    public:
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
    static int opencl_version(const CLDevice &dev)
    {
        return internal::cl_opencl_version(dev.get());
    }

    /// \brief Return the OpenCL C version of a device
    ///
    /// \sa opencl_version
    static int opencl_c_version(const CLDevice &dev)
    {
        return internal::cl_opencl_c_version(dev.get());
    }

    /// \brief Check if a device feature exists
    template <OpenCLDeviceFeature feat>
    static bool has_feature(const CLDevice &dev)
    {
        return check_feature(
            dev, std::integral_constant<OpenCLDeviceFeature, feat>());
    }

    /// \brief Check if a device type exists in a platform
    template <::cl_device_type DevType>
    static bool has_device(const CLPlatform &plat)
    {
        return plat.get_device(DevType).size() != 0;
    }

    /// \brief Check if a device type exists in any platform
    template <::cl_device_type DevType>
    static bool has_device()
    {
        std::vector<CLPlatform> plat_vec(CLPlatform::platforms());
        for (const auto &plat : plat_vec)
            if (has_device<DevType>(plat))
                return true;

        return false;
    }

    /// \brief Query all information
    template <typename CharT, typename Traits>
    static std::basic_ostream<CharT, Traits> &info(
        std::basic_ostream<CharT, Traits> &os)
    {
        if (!os.good())
            return os;

        std::vector<CLPlatform> plat_vec(CLPlatform::platforms());
        for (const auto &plat : plat_vec)
            info(os, plat);
        print_equal(os);

        return os;
    }

    /// \brief Query platform information
    template <typename CharT, typename Traits>
    static std::basic_ostream<CharT, Traits> &info(
        std::basic_ostream<CharT, Traits> &os, const CLPlatform &plat)
    {
        if (!os.good())
            return os;

        print_equal(os);

        print_info_val<std::string, ::cl_platform_info>(
            os, plat, CL_PLATFORM_NAME, "CL_PLATFORM_NAME");
        print_info_val<std::string, ::cl_platform_info>(
            os, plat, CL_PLATFORM_VENDOR, "CL_PLATFORM_VENDOR");
        print_info_val<std::string, ::cl_platform_info>(
            os, plat, CL_PLATFORM_VERSION, "CL_PLATFORM_VERSION");
        print_info_val<std::string, ::cl_platform_info>(
            os, plat, CL_PLATFORM_PROFILE, "CL_PLATFORM_PROFILE");
        print_plat_extensions(os, plat);

        std::vector<CLDevice> dev_vec(plat.get_device(CL_DEVICE_TYPE_ALL));
        for (const auto &dev : dev_vec)
            info(os, dev);

        return os;
    }

    /// \brief Query device information
    template <typename CharT, typename Traits>
    static std::basic_ostream<CharT, Traits> &info(
        std::basic_ostream<CharT, Traits> &os, const CLDevice &dev)
    {
        if (!os.good())
            return os;

        print_dash(os);
        print_dev_version(os, dev);
        os << '\n';
        print_dev_extensions(os, dev);
        os << '\n';
        print_dev_processor(os, dev);
        os << '\n';
        print_dev_memory(os, dev);
        os << '\n';
        print_dev_vector(os, dev);
        os << '\n';
        print_dev_single_fp_config(os, dev);
        os << '\n';
        print_dev_double_fp_config(os, dev);
        if (opencl_version(dev) >= 120) {
            os << '\n';
            print_info_val<std::string, ::cl_device_info>(os, dev,
                CL_DEVICE_BUILT_IN_KERNELS, "CL_DEVICE_BUILT_IN_KERNELS");
            os << '\n';
            print_dev_image_support(os, dev);
        }

        return os;
    }

    /// \brief Query context information
    template <typename CharT, typename Traits>
    static std::basic_ostream<CharT, Traits> &info(
        std::basic_ostream<CharT, Traits> &os, const CLContext &ctx)
    {
        if (!os.good())
            return os;

        std::vector<CLDevice> dev_vec(ctx.get_device());
        for (const auto &dev : dev_vec)
            info(os, dev);

        return os;
    }

    private:
    static bool check_feature(const CLDevice &dev,
        std::integral_constant<OpenCLDeviceFeature, OpenCLDeviceDoubleFP>)
    {
        std::string info;
        dev.get_info(CL_DEVICE_EXTENSIONS, info);
        if (info.find("cl_khr_fp64") != std::string::npos)
            return true;
        ::cl_device_fp_config config;
        dev.get_info(CL_DEVICE_DOUBLE_FP_CONFIG, config);
        if (config != 0)
            return true;
        return false;
    }

    static bool check_feature(const CLDevice &dev,
        std::integral_constant<OpenCLDeviceFeature, OpenCLDeviceImageSupport>)
    {
        ::cl_bool support;
        dev.get_info(CL_DEVICE_IMAGE_SUPPORT, support);
        if (support != 0)
            return true;
        return false;
    }

    template <typename CharT, typename Traits>
    static void print_equal(std::basic_ostream<CharT, Traits> &os)
    {
        os << std::string(90, '=') << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_dash(std::basic_ostream<CharT, Traits> &os)
    {
        os << std::string(90, '-') << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_plat_extensions(
        std::basic_ostream<CharT, Traits> &os, const CLPlatform &plat)
    {
        std::string info;
        plat.get_info(CL_PLATFORM_EXTENSIONS, info);
        print_name(os, "CL_PLATFORM_EXTENSIONS");
        print_val(os, split_string(info));
        os << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_dev_version(
        std::basic_ostream<CharT, Traits> &os, const CLDevice &dev)
    {
        print_info_val<std::string, ::cl_device_info>(
            os, dev, CL_DEVICE_NAME, "CL_DEVICE_NAME");
        print_dev_type(os, dev);
        print_info_val<std::string, ::cl_device_info>(
            os, dev, CL_DEVICE_VENDOR, "CL_DEVICE_VENDOR");
        print_info_val<::cl_uint, ::cl_device_info>(
            os, dev, CL_DEVICE_VENDOR_ID, "CL_DEVICE_VENDOR_ID");
        print_info_val<std::string, ::cl_device_info>(
            os, dev, CL_DRIVER_VERSION, "CL_DRIVER_VERSION");
        print_info_val<std::string, ::cl_device_info>(
            os, dev, CL_DEVICE_PROFILE, "CL_DEVICE_PROFILE");
        print_info_val<std::string, ::cl_device_info>(
            os, dev, CL_DEVICE_VERSION, "CL_DEVICE_VERSION");
        print_info_val<std::string, ::cl_device_info>(
            os, dev, CL_DEVICE_OPENCL_C_VERSION, "CL_DEVICE_OPENCL_C_VERSION");
    }

    template <typename CharT, typename Traits>
    static void print_dev_extensions(
        std::basic_ostream<CharT, Traits> &os, const CLDevice &dev)
    {
        std::string info;
        dev.get_info(CL_DEVICE_EXTENSIONS, info);
        print_name(os, "CL_DEVICE_EXTENSIONS");
        print_val(os, split_string(info));
        os << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_dev_type(
        std::basic_ostream<CharT, Traits> &os, const CLDevice &dev)
    {
        ::cl_device_type type;
        std::vector<std::string> info;
        dev.get_info(CL_DEVICE_TYPE, type);

        append_bit_field<::cl_device_type>(
            CL_DEVICE_TYPE_CPU, type, "CL_DEVICE_TYPE_CPU", info);
        append_bit_field<::cl_device_type>(
            CL_DEVICE_TYPE_GPU, type, "CL_DEVICE_TYPE_GPU", info);
        append_bit_field<::cl_device_type>(CL_DEVICE_TYPE_ACCELERATOR, type,
            "CL_DEVICE_TYPE_ACCELERATOR", info);
        append_bit_field<::cl_device_type>(
            CL_DEVICE_TYPE_DEFAULT, type, "CL_DEVICE_TYPE_DEFAULT", info);
        append_bit_field<::cl_device_type>(
            CL_DEVICE_TYPE_CUSTOM, type, "CL_DEVICE_TYPE_CUSTOM", info);

        print_name(os, "CL_DEVICE_TYPE");
        print_val(os, info);
        os << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_dev_processor(
        std::basic_ostream<CharT, Traits> &os, const CLDevice &dev)
    {
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_MAX_CLOCK_FREQUENCY, "CL_DEVICE_MAX_CLOCK_FREQUENCY",
            "MHz");
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_MAX_COMPUTE_UNITS, "CL_DEVICE_MAX_COMPUTE_UNITS");
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS,
            "CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS");
        print_info_val<std::vector<std::size_t>, ::cl_device_info>(os, dev,
            CL_DEVICE_MAX_WORK_ITEM_SIZES, "CL_DEVICE_MAX_WORK_ITEM_SIZES");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
            CL_DEVICE_MAX_WORK_GROUP_SIZE, "CL_DEVICE_MAX_WORK_GROUP_SIZE");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
            CL_DEVICE_PROFILING_TIMER_RESOLUTION,
            "CL_DEVICE_PROFILING_TIMER_RESOLUTION", "ns");
    }

    template <typename CharT, typename Traits>
    static void print_dev_memory(
        std::basic_ostream<CharT, Traits> &os, const CLDevice &dev)
    {
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
            CL_DEVICE_MAX_PARAMETER_SIZE, "CL_DEVICE_MAX_PARAMETER_SIZE",
            "byte");
        print_info_val<::cl_uint, ::cl_device_info>(
            os, dev, CL_DEVICE_ADDRESS_BITS, "CL_DEVICE_ADDRESS_BITS", "bit");
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_MEM_BASE_ADDR_ALIGN, "CL_DEVICE_MEM_BASE_ADDR_ALIGN",
            "bit");
        print_info_val<::cl_ulong, ::cl_device_info>(os, dev,
            CL_DEVICE_MAX_MEM_ALLOC_SIZE, "CL_DEVICE_MAX_MEM_ALLOC_SIZE",
            "byte");
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE,
            "CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE", "byte");
        print_info_val<::cl_ulong, ::cl_device_info>(os, dev,
            CL_DEVICE_GLOBAL_MEM_CACHE_SIZE, "CL_DEVICE_GLOBAL_MEM_CACHE_SIZE",
            "byte");
        print_info_val<::cl_ulong, ::cl_device_info>(os, dev,
            CL_DEVICE_GLOBAL_MEM_SIZE, "CL_DEVICE_GLOBAL_MEM_SIZE", "byte");
        print_info_val<::cl_ulong, ::cl_device_info>(os, dev,
            CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE,
            "CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE", "byte");
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_MAX_CONSTANT_ARGS, "CL_DEVICE_MAX_CONSTANT_ARGS");
        print_info_val<::cl_ulong, ::cl_device_info>(os, dev,
            CL_DEVICE_LOCAL_MEM_SIZE, "CL_DEVICE_LOCAL_MEM_SIZE", "byte");
        print_info_val<::cl_bool, ::cl_device_info>(os, dev,
            CL_DEVICE_ERROR_CORRECTION_SUPPORT,
            "CL_DEVICE_ERROR_CORRECTION_SUPPORT");
    }

    template <typename CharT, typename Traits>
    static void print_dev_vector(
        std::basic_ostream<CharT, Traits> &os, const CLDevice &dev)
    {
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR,
            "CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR");
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT,
            "CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT");
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT,
            "CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT");
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG,
            "CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG");
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT,
            "CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT");
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE,
            "CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE");
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF,
            "CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF");
        if (opencl_version(dev) >= 110) {
            os << '\n';
            print_info_val<::cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR");
            print_info_val<::cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT");
            print_info_val<::cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_INT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_INT");
            print_info_val<::cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG");
            print_info_val<::cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT");
            print_info_val<::cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE");
            print_info_val<::cl_uint, ::cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF");
        }
    }

    template <typename CharT, typename Traits>
    static void print_dev_single_fp_config(
        std::basic_ostream<CharT, Traits> &os, const CLDevice &dev)
    {
        ::cl_device_fp_config config;
        std::vector<std::string> info;
        dev.get_info(CL_DEVICE_SINGLE_FP_CONFIG, config);

        append_bit_field<::cl_device_fp_config>(
            CL_FP_DENORM, config, "CL_FP_DENORM", info);
        append_bit_field<::cl_device_fp_config>(
            CL_FP_INF_NAN, config, "CL_FP_INF_NAN", info);
        append_bit_field<::cl_device_fp_config>(
            CL_FP_ROUND_TO_NEAREST, config, "CL_FP_ROUND_TO_NEAREST", info);
        append_bit_field<::cl_device_fp_config>(
            CL_FP_ROUND_TO_ZERO, config, "CL_FP_ROUND_TO_ZERO", info);
        append_bit_field<::cl_device_fp_config>(
            CL_FP_ROUND_TO_INF, config, "CL_FP_ROUND_TO_INF", info);
        append_bit_field<::cl_device_fp_config>(
            CL_FP_FMA, config, "CL_FP_FMA", info);
        append_bit_field<::cl_device_fp_config>(
            CL_FP_SOFT_FLOAT, config, "CL_FP_SOFT_FLOAT", info);
        append_bit_field<::cl_device_fp_config>(
            CL_FP_CORRECTLY_ROUNDED_DIVIDE_SQRT, config,
            "CL_FP_CORRECTLY_ROUNDED_DIVIDE_SQRT", info);

        print_name(os, "CL_DEVICE_SINGLE_FP_CONFIG");
        print_val(os, info);
        os << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_dev_double_fp_config(
        std::basic_ostream<CharT, Traits> &os, const CLDevice &dev)
    {
        if (!has_feature<OpenCLDeviceDoubleFP>(dev))
            return;

        ::cl_device_fp_config config;
        std::vector<std::string> info;
        dev.get_info(CL_DEVICE_DOUBLE_FP_CONFIG, config);

        append_bit_field<::cl_device_fp_config>(
            CL_FP_DENORM, config, "CL_FP_DENORM", info);
        append_bit_field<::cl_device_fp_config>(
            CL_FP_INF_NAN, config, "CL_FP_INF_NAN", info);
        append_bit_field<::cl_device_fp_config>(
            CL_FP_ROUND_TO_NEAREST, config, "CL_FP_ROUND_TO_NEAREST", info);
        append_bit_field<::cl_device_fp_config>(
            CL_FP_ROUND_TO_ZERO, config, "CL_FP_ROUND_TO_ZERO", info);
        append_bit_field<::cl_device_fp_config>(
            CL_FP_ROUND_TO_INF, config, "CL_FP_ROUND_TO_INF", info);
        append_bit_field<::cl_device_fp_config>(
            CL_FP_FMA, config, "CL_FP_FMA", info);
        append_bit_field<::cl_device_fp_config>(
            CL_FP_SOFT_FLOAT, config, "CL_FP_SOFT_FLOAT", info);

        print_name(os, "CL_DEVICE_DOUBLE_FP_CONFIG");
        print_val(os, info);
        os << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_dev_image_support(
        std::basic_ostream<CharT, Traits> &os, const CLDevice &dev)
    {
        if (!has_feature<OpenCLDeviceImageSupport>(dev))
            return;

        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_MAX_READ_IMAGE_ARGS, "CL_DEVICE_MAX_READ_IMAGE_ARGS");
        print_info_val<::cl_uint, ::cl_device_info>(os, dev,
            CL_DEVICE_MAX_WRITE_IMAGE_ARGS, "CL_DEVICE_MAX_WRITE_IMAGE_ARGS");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
            CL_DEVICE_IMAGE2D_MAX_WIDTH, "CL_DEVICE_IMAGE2D_MAX_WIDTH",
            "pixel");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
            CL_DEVICE_IMAGE2D_MAX_HEIGHT, "CL_DEVICE_IMAGE2D_MAX_HEIGHT",
            "pixel");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
            CL_DEVICE_IMAGE3D_MAX_WIDTH, "CL_DEVICE_IMAGE3D_MAX_WIDTH",
            "pixel");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
            CL_DEVICE_IMAGE3D_MAX_HEIGHT, "CL_DEVICE_IMAGE3D_MAX_HEIGHT",
            "pixel");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
            CL_DEVICE_IMAGE3D_MAX_DEPTH, "CL_DEVICE_IMAGE3D_MAX_DEPTH",
            "pixel");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
            CL_DEVICE_IMAGE_MAX_BUFFER_SIZE, "CL_DEVICE_IMAGE_MAX_BUFFER_SIZE",
            "pixel");
        print_info_val<std::size_t, ::cl_device_info>(os, dev,
            CL_DEVICE_IMAGE_MAX_ARRAY_SIZE, "CL_DEVICE_IMAGE_MAX_ARRAY_SIZE");
        print_info_val<::cl_uint, ::cl_device_info>(
            os, dev, CL_DEVICE_MAX_SAMPLERS, "CL_DEVICE_MAX_SAMPLERS");
    }

    template <typename T>
    static void append_bit_field(T info, T val, const std::string &name,
        std::vector<std::string> &strvec)
    {
        if ((info & val) != 0)
            strvec.push_back(name);
    }

    template <typename T>
    static std::string byte_string(const T &val, std::true_type)
    {
        std::size_t B = static_cast<std::size_t>(val);
        std::size_t K = 1024;
        std::size_t M = 1024 * K;
        std::size_t G = 1024 * M;
        std::stringstream ss;
        if (B >= G)
            ss << (B % G == 0 ? B / G : static_cast<double>(B) / G) << 'G';
        else if (B >= M)
            ss << (B % M == 0 ? B / M : static_cast<double>(B) / M) << 'M';
        else if (B >= K)
            ss << (B % K == 0 ? B / K : static_cast<double>(B) / K) << 'K';
        else
            ss << B;

        return ss.str();
    }

    template <typename T>
    static std::string byte_string(const T &, std::false_type)
    {
        return std::string();
    }

    static std::vector<std::string> split_string(const std::string &str)
    {
        std::istringstream ss(str);
        return std::vector<std::string>(std::istream_iterator<std::string>(ss),
            std::istream_iterator<std::string>());
    }

    template <typename T, typename CLInfoType, typename ObjType,
        typename InfoType, typename CharT, typename Traits>
    static void print_info_val(std::basic_ostream<CharT, Traits> &os,
        const ObjType &obj, InfoType info, const std::string &name,
        const std::string &unit = "")
    {
        T val;
        obj.get_info(static_cast<CLInfoType>(info), val);
        print_name(os, name);
        if (std::is_integral<T>::value && unit == std::string("byte"))
            print_val(os, byte_string(val, std::is_integral<T>()));
        else
            print_val(os, val);
        os << ' ' << unit << '\n';
    }

    template <typename CharT, typename Traits>
    static void print_name(
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
    static void print_val(std::basic_ostream<CharT, Traits> &os, const T &val)
    {
        os << val;
    }

    template <typename T, typename CharT, typename Traits>
    static void print_val(
        std::basic_ostream<CharT, Traits> &os, const std::vector<T> &val)
    {
        for (std::size_t i = 0; i != val.size(); ++i)
            os << val[i] << ' ';
    }

    template <typename CharT, typename Traits>
    static void print_val(std::basic_ostream<CharT, Traits> &os,
        const std::vector<std::string> &val)
    {
        if (val.size() == 0)
            return;

        if (val.size() == 1) {
            os << val[0];
            return;
        }

        os << val[0] << '\n';
        for (std::size_t i = 1; i != val.size() - 1; ++i) {
            os << std::string(41, ' ') << val[i] << '\n';
        }
        os << std::string(41, ' ') << val.back();
    }
}; // class CLQuery

/// \brief Query OpenCL information using CLQuery
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const CLQuery &)
{
    return CLQuery::info(os);
}

/// \brief Query device information in a given platform
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const CLPlatform &plat)
{
    return vsmc::CLQuery::info(os, plat);
}

/// \brief Query device information in a given context
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const CLContext &ctx)
{
    return vsmc::CLQuery::info(os, ctx);
}

/// \brief Query device information
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const CLDevice &dev)
{
    return vsmc::CLQuery::info(os, dev);
}

} // namespace vsmc

#endif // VSMC_OPENCL_CL_QUERY_HPP

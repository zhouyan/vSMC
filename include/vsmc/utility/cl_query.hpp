#ifndef VSMC_UTILITY_CL_QUERY_HPP
#define VSMC_UTILITY_CL_QUERY_HPP

#include <vsmc/utility/cl.hpp>
#include <iostream>
#include <iomanip>

namespace vsmc { namespace clmgr {

/// \brief Query information of OpenCL devices
/// \ingroup CLMGR
class QueryCL
{
    public :

    template<typename CharT, typename Traits>
    static void print (std::basic_ostream<CharT, Traits> &os)
    {
        std::vector<cl::Platform> platform;
        cl::Platform::get(&platform);
        for (std::vector<cl::Platform>::const_iterator p = platform.begin();
                p != platform.end(); ++p)
            print(os, *p);
    }

    template<typename CharT, typename Traits>
    static void print (std::basic_ostream<CharT, Traits> &os,
            const cl::Platform &plat)
    {
        print_equal(os);
        print_plat(os, plat);
        std::vector<cl::Device> device;
        plat.getDevices(CL_DEVICE_TYPE_ALL, &device);
        for (std::vector<cl::Device>::const_iterator d = device.begin();
                d != device.end(); ++d)
            print(os, *d);
    }

    template <typename CharT, typename Traits>
    static void print (std::basic_ostream<CharT, Traits> &os,
            const cl::Context &ctx)
    {
        print_equal(os);

        std::vector<cl_context_properties> info;
        ctx.getInfo(CL_CONTEXT_PROPERTIES, &info);
        if (!(info.size() % 3)) {
            for (std::vector<cl_context_properties>::const_iterator
                    i = info.begin(); i != info.end(); i += 3)
            {
                if (*i == CL_CONTEXT_PLATFORM )
                    print_plat(os, cl::Platform((cl_platform_id)*(i + 1)));
            }
        }

        std::vector<cl::Device> device;
        ctx.getInfo(CL_CONTEXT_DEVICES, &device);
        for (std::vector<cl::Device>::const_iterator d = device.begin();
                d != device.end(); ++d)
            print(os, *d);
    }

    template<typename CharT, typename Traits>
    static void print (std::basic_ostream<CharT, Traits> &os,
            const cl::Device &dev)
    {
        print_dash(os);

        print_dev_val<std::string>(os, dev,
                CL_DEVICE_NAME, "CL_DEVICE_NAME");
        print_dev_type(os, dev);
        print_dev_val<std::string>(os, dev,
                CL_DEVICE_VENDOR, "CL_DEVICE_VENDOR");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_VENDOR_ID, "CL_DEVICE_VENDOR_ID");
        print_dev_val<std::string>(os, dev,
                CL_DRIVER_VERSION, "CL_DRIVER_VERSION");
        print_dev_val<std::string>(os, dev,
                CL_DEVICE_PROFILE, "CL_DEVICE_PROFILE");
        print_dev_val<std::string>(os, dev,
                CL_DEVICE_VERSION, "CL_DEVICE_VERSION");
        print_dev_val<std::string>(os, dev,
                CL_DEVICE_OPENCL_C_VERSION, "CL_DEVICE_OPENCL_C_VERSION");
        print_dev_val<std::string>(os, dev,
                CL_DEVICE_EXTENSIONS, "CL_DEVICE_EXTENSIONS");

        os << '\n';

        print_dev_val<cl_bool>(os, dev,
                CL_DEVICE_AVAILABLE, "CL_DEVICE_AVAILABLE");
        print_dev_val<cl_bool>(os, dev,
                CL_DEVICE_COMPILER_AVAILABLE, "CL_DEVICE_COMPILER_AVAILABLE");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_CLOCK_FREQUENCY,
                "CL_DEVICE_MAX_CLOCK_FREQUENCY", "MHz");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_ADDRESS_BITS,
                "CL_DEVICE_ADDRESS_BITS", "bits");
        print_dev_val<cl_ulong>(os, dev,
                CL_DEVICE_MAX_MEM_ALLOC_SIZE,
                "CL_DEVICE_MAX_MEM_ALLOC_SIZE", "bytes");
        print_dev_sfp_config(os, dev);
        print_dev_dfp_config(os, dev);

        os << '\n';

        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_COMPUTE_UNITS,
                "CL_DEVICE_MAX_COMPUTE_UNITS");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS,
                "CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS");
        print_dev_val<std::vector<std::size_t> >(os, dev,
                CL_DEVICE_MAX_WORK_ITEM_SIZES,
                "CL_DEVICE_MAX_WORK_ITEM_SIZES");
        print_dev_val<std::size_t>(os, dev,
                CL_DEVICE_MAX_WORK_GROUP_SIZE,
                "CL_DEVICE_MAX_WORK_GROUP_SIZE");

        os << '\n';

        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF");

        os << '\n';

        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_INT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_INT");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF");

        os << '\n';

        print_dev_val<cl_bool>(os, dev,
                CL_DEVICE_IMAGE_SUPPORT,
                "CL_DEVICE_IMAGE_SUPPORT");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_READ_IMAGE_ARGS,
                "CL_DEVICE_MAX_READ_IMAGE_ARGS");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_WRITE_IMAGE_ARGS,
                "CL_DEVICE_MAX_WRITE_IMAGE_ARGS");
        print_dev_val<std::size_t>(os, dev,
                CL_DEVICE_IMAGE2D_MAX_WIDTH,
                "CL_DEVICE_IMAGE2D_MAX_WIDTH");
        print_dev_val<std::size_t>(os, dev,
                CL_DEVICE_IMAGE2D_MAX_HEIGHT,
                "CL_DEVICE_IMAGE2D_MAX_HEIGHT");
        print_dev_val<std::size_t>(os, dev,
                CL_DEVICE_IMAGE3D_MAX_WIDTH,
                "CL_DEVICE_IMAGE3D_MAX_WIDTH");
        print_dev_val<std::size_t>(os, dev,
                CL_DEVICE_IMAGE3D_MAX_HEIGHT,
                "CL_DEVICE_IMAGE3D_MAX_HEIGHT");
        print_dev_val<std::size_t>(os, dev,
                CL_DEVICE_IMAGE3D_MAX_DEPTH,
                "CL_DEVICE_IMAGE3D_MAX_DEPTH");
        // print_dev_val<std::size_t>(os, dev,
        //         CL_DEVICE_IMAGE_MAX_BUFFER_SIZE,
        //         "CL_DEVICE_IMAGE_MAX_BUFFER_SIZE");
        // print_dev_val<std::size_t>(os, dev,
        //         CL_DEVICE_IMAGE_MAX_ARRAY_SIZE,
        //         "CL_DEVICE_IMAGE_MAX_ARRAY_SIZE");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_SAMPLERS,
                "CL_DEVICE_MAX_SAMPLERS");

        os << '\n';

        print_dev_val<std::size_t>(os, dev,
                CL_DEVICE_MAX_PARAMETER_SIZE,
                "CL_DEVICE_MAX_PARAMETER_SIZE", "bytes");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_MEM_BASE_ADDR_ALIGN,
                "CL_DEVICE_MEM_BASE_ADDR_ALIGN", "bits");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE,
                "CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE", "bytes");
        print_dev_val<cl_ulong>(os, dev,
                CL_DEVICE_GLOBAL_MEM_CACHE_SIZE,
                "CL_DEVICE_GLOBAL_MEM_CACHE_SIZE", "bytes");
        print_dev_val<cl_ulong>(os, dev,
                CL_DEVICE_GLOBAL_MEM_SIZE,
                "CL_DEVICE_GLOBAL_MEM_SIZE", "bytes");
        print_dev_val<cl_ulong>(os, dev,
                CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE,
                "CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE", "bytes");
        print_dev_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_CONSTANT_ARGS,
                "CL_DEVICE_MAX_CONSTANT_ARGS");
        print_dev_val<cl_ulong>(os, dev,
                CL_DEVICE_LOCAL_MEM_SIZE,
                "CL_DEVICE_LOCAL_MEM_SIZE");
        print_dev_val<cl_bool>(os, dev,
                CL_DEVICE_ERROR_CORRECTION_SUPPORT,
                "CL_DEVICE_ERROR_CORRECTION_SUPPORT");
        print_dev_val<cl_bool>(os, dev,
                CL_DEVICE_HOST_UNIFIED_MEMORY,
                "CL_DEVICE_HOST_UNIFIED_MEMORY");
        print_dev_val<std::size_t>(os, dev,
                CL_DEVICE_PROFILING_TIMER_RESOLUTION,
                "CL_DEVICE_PROFILING_TIMER_RESOLUTION", "ns");
        print_dev_val<cl_bool>(os, dev,
                CL_DEVICE_ENDIAN_LITTLE,
                "CL_DEVICE_ENDIAN_LITTLE");
    }

    private :

    template<typename CharT, typename Traits>
    static void print_equal (std::basic_ostream<CharT, Traits> &os,
            unsigned length = 78)
    {
        for (unsigned l = 0; l != length; ++l)
            os << '=';
        os << '\n';
    }

    template<typename CharT, typename Traits>
    static void print_dash (std::basic_ostream<CharT, Traits> &os,
            unsigned length = 78)
    {
        for (unsigned l = 0; l != length; ++l)
            os << '-';
        os << '\n';
    }

    template<typename CharT, typename Traits>
    static void print_plat (std::basic_ostream<CharT, Traits> &os,
            const cl::Platform &plat)
    {
        std::string info;

        plat.getInfo(CL_PLATFORM_NAME, &info);
        os << std::setw(40) << std::left
            << "CL_PLATFORM_NAME" << info << '\n';

        plat.getInfo(CL_PLATFORM_VENDOR, &info);
        os << std::setw(40) << std::left
            << "CL_PLATFORM_VENDOR" << info << '\n';

        plat.getInfo(CL_PLATFORM_VERSION, &info);
        os << std::setw(40) << std::left
            << "CL_PLATFORM_VERSION" << info << '\n';

        plat.getInfo(CL_PLATFORM_PROFILE, &info);
        os << std::setw(40) << std::left
            << "CL_PLATFORM_PROFILE" << info << '\n';

        plat.getInfo(CL_PLATFORM_EXTENSIONS, &info);
        os << std::setw(40) << std::left
            << "CL_PLATFORM_EXTENSIONS" << info << '\n';
    }

    template<typename T, typename CharT, typename Traits>
    static void print_dev_val (std::basic_ostream<CharT, Traits> &os,
            const cl::Device &dev,
            cl_device_info info, const std::string &name,
            const std::string &unit = "")
    {
        T val;
        dev.getInfo(info, &val);
        os << std::setw(40) << std::left
            << name;
        print_val(os, val);
        os << ' ' << unit << '\n';
    }

    template<typename CharT, typename Traits>
    static void print_dev_type (std::basic_ostream<CharT, Traits> &os,
            const cl::Device &dev)
    {
        cl_device_type type;
        dev.getInfo(CL_DEVICE_TYPE, &type);
        std::string info;
        switch (type) {
            case CL_DEVICE_TYPE_CPU :
                info = "CL_DEVICE_TYPE_CPU";
                break;
            case CL_DEVICE_TYPE_GPU :
                info = "CL_DEVICE_TYPE_GPU";
                break;
            case CL_DEVICE_TYPE_ACCELERATOR :
                info = "CL_DEVICE_TYPE_ACCELERATOR";
                break;
            case CL_DEVICE_TYPE_DEFAULT :
                info = "CL_DEVICE_TYPE_DEFAULT";
                break;
        }
        os << std::setw(40) << std::left << "CL_DEVICE_TYPE" << info << '\n';
    }

    template<typename CharT, typename Traits>
    static void print_dev_sfp_config (std::basic_ostream<CharT, Traits> &os,
            const cl::Device &dev)
    {
        cl_device_fp_config val;
        std::string info;
        dev.getInfo(CL_DEVICE_SINGLE_FP_CONFIG, &val);
        append_bit_field<cl_device_fp_config>(CL_FP_DENORM,
                val, "CL_FP_DENORM", info);
        append_bit_field<cl_device_fp_config>(CL_FP_INF_NAN,
                val, "CL_FP_INF_NAN", info);
        append_bit_field<cl_device_fp_config>(CL_FP_ROUND_TO_NEAREST,
                val, "CL_FP_ROUND_TO_NEAREST", info);
        append_bit_field<cl_device_fp_config>(CL_FP_ROUND_TO_ZERO,
                val, "CL_FP_ROUND_TO_ZERO", info);
        append_bit_field<cl_device_fp_config>(CL_FP_ROUND_TO_INF,
                val, "CL_FP_ROUND_TO_INF", info);
        append_bit_field<cl_device_fp_config>(CL_FP_FMA,
                val, "CL_FP_FMA", info);
        append_bit_field<cl_device_fp_config>(CL_FP_SOFT_FLOAT,
                val, "CL_FP_SOFT_FLOAT", info);
        // append_bit_field<cl_device_fp_config>(
        //         CL_FP_CORRECTLY_ROUNDED_DIVIDE_SQRT,
        //         val, "CL_FP_CORRECTLY_ROUNDED_DIVIDE_SQRT", info);
        os << std::setw(40) << std::left
            << "CL_DEVICE_SINGLE_FP_CONFIG" << info << '\n';
    }

    template<typename CharT, typename Traits>
    static void print_dev_dfp_config (std::basic_ostream<CharT, Traits> &os,
            const cl::Device &dev)
    {
        cl_device_fp_config val;
        std::string info;
        dev.getInfo(CL_DEVICE_DOUBLE_FP_CONFIG, &val);
        append_bit_field<cl_device_fp_config>(CL_FP_DENORM,
                val, "CL_FP_DENORM", info);
        append_bit_field<cl_device_fp_config>(CL_FP_INF_NAN,
                val, "CL_FP_INF_NAN", info);
        append_bit_field<cl_device_fp_config>(CL_FP_ROUND_TO_NEAREST,
                val, "CL_FP_ROUND_TO_NEAREST", info);
        append_bit_field<cl_device_fp_config>(CL_FP_ROUND_TO_ZERO,
                val, "CL_FP_ROUND_TO_ZERO", info);
        append_bit_field<cl_device_fp_config>(CL_FP_ROUND_TO_INF,
                val, "CL_FP_ROUND_TO_INF", info);
        append_bit_field<cl_device_fp_config>(CL_FP_FMA,
                val, "CL_FP_FMA", info);
        append_bit_field<cl_device_fp_config>(CL_FP_SOFT_FLOAT,
                val, "CL_FP_SOFT_FLOAT", info);
        os << std::setw(40) << std::left
            << "CL_DEVICE_DOUBLE_FP_CONFIG" << info << '\n';
    }

    template <typename T>
    static void append_bit_field (T info, T val,
            const std::string &name, std::string &orig)
    {
        if (info & val) {
            if (orig.length())
                orig.append(" | ");
            orig.append(name);
        }
    }

    template<typename T, typename CharT, typename Traits>
    static void print_val (std::basic_ostream<CharT, Traits> &os, const T &val)
    {
        os << val;
    }

    template<typename CharT, typename Traits>
    static void print_val (std::basic_ostream<CharT, Traits> &os,
            const std::vector<std::size_t> &val)
    {
        for (std::vector<std::size_t>::const_iterator v = val.begin();
                v != val.end(); ++v)
            os << *v << ' ';
    }

}; //class QueryCL

/// \brief Print information of all platforms and devices
/// \ingroup OpenCL
template<typename CharT, typename Traits>
std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const vsmc::QueryCL &query)
{
    query.print(os);
    return os;
}

} } // namespace vsmc::clmgr

namespace std {

/// \brief Print information of all devices in a given platform
/// \ingroup OpenCL
template<typename CharT, typename Traits>
basic_ostream<CharT, Traits> &operator<< (
        basic_ostream<CharT, Traits> &os, const cl::Platform &plat)
{
    vsmc::clmgr::QueryCL::print(os, plat);
    return os;
}

/// \brief Print information of all devices in a given context
/// \ingroup OpenCL
template<typename CharT, typename Traits>
basic_ostream<CharT, Traits> &operator<< (
        basic_ostream<CharT, Traits> &os, const cl::Context &ctx)
{
    vsmc::clmgr::QueryCL::print(os, ctx);
    return os;
}

/// \brief Print information a given device
/// \ingroup OpenCL
template<typename CharT, typename Traits>
basic_ostream<CharT, Traits> &operator<< (
        basic_ostream<CharT, Traits> &os, const cl::Device &dev)
{
    vsmc::clmgr::QueryCL::print(os, dev);
    return os;
}

} // namespace std

#endif // VSMC_UTILITY_CL_QUERY_HPP

//============================================================================
// include/vsmc/opencl/cl_query.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_OPENCL_CL_QUERY_HPP
#define VSMC_OPENCL_CL_QUERY_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/internal/cl_wrapper.hpp>
#include <cstring>
#include <iostream>
#include <string>

namespace vsmc {

/// \brief Query OpenCL information
/// \ingroup OpenCL
class CLQuery
{
    public :

    /// \brief Query all information
    template <typename CharT, typename Traits>
    static void info (std::basic_ostream<CharT, Traits> &os)
    {
        std::vector< ::cl::Platform> platform;
        ::cl::Platform::get(&platform);
        for (std::vector< ::cl::Platform>::const_iterator p = platform.begin();
                p != platform.end(); ++p)
            info(os, *p);
        print_equal(os);
    }

    /// \brief Query platform information
    template <typename CharT, typename Traits>
    static void info (std::basic_ostream<CharT, Traits> &os,
            const ::cl::Platform &plat)
    {
        print_equal(os);

        print_info_val<std::string, cl_platform_info>(os, plat,
                CL_PLATFORM_NAME, "CL_PLATFORM_NAME");
        print_info_val<std::string, cl_platform_info>(os, plat,
                CL_PLATFORM_VENDOR, "CL_PLATFORM_VENDOR");
        print_info_val<std::string, cl_platform_info>(os, plat,
                CL_PLATFORM_VERSION, "CL_PLATFORM_VERSION");
        print_info_val<std::string, cl_platform_info>(os, plat,
                CL_PLATFORM_PROFILE, "CL_PLATFORM_PROFILE");
        print_info_val<std::string, cl_platform_info>(os, plat,
                CL_PLATFORM_EXTENSIONS, "CL_PLATFORM_EXTENSIONS");

        std::vector< ::cl::Device> device;
        plat.getDevices(CL_DEVICE_TYPE_ALL, &device);
        for (std::vector< ::cl::Device>::const_iterator d = device.begin();
                d != device.end(); ++d)
            info(os, *d);
    }

    /// \brief Query device information
    template <typename CharT, typename Traits>
    static void info (std::basic_ostream<CharT, Traits> &os,
            const ::cl::Device &dev)
    {
        print_dash(os);

        print_info_val<std::string, cl_device_info>(os, dev,
                CL_DEVICE_NAME, "CL_DEVICE_NAME");
        print_dev_type(os, dev);
        print_info_val<std::string, cl_device_info>(os, dev,
                CL_DEVICE_VENDOR, "CL_DEVICE_VENDOR");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_VENDOR_ID, "CL_DEVICE_VENDOR_ID");
        print_info_val<std::string, cl_device_info>(os, dev,
                CL_DRIVER_VERSION, "CL_DRIVER_VERSION");
        print_info_val<std::string, cl_device_info>(os, dev,
                CL_DEVICE_PROFILE, "CL_DEVICE_PROFILE");
        print_info_val<std::string, cl_device_info>(os, dev,
                CL_DEVICE_VERSION, "CL_DEVICE_VERSION");
        print_info_val<std::string, cl_device_info>(os, dev,
                CL_DEVICE_OPENCL_C_VERSION, "CL_DEVICE_OPENCL_C_VERSION");
        print_info_val<std::string, cl_device_info>(os, dev,
                CL_DEVICE_EXTENSIONS, "CL_DEVICE_EXTENSIONS");
        os << '\n';
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_MAX_COMPUTE_UNITS,
                "CL_DEVICE_MAX_COMPUTE_UNITS");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS,
                "CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS");
        print_info_val<std::vector<std::size_t> , cl_device_info>(os, dev,
                CL_DEVICE_MAX_WORK_ITEM_SIZES,
                "CL_DEVICE_MAX_WORK_ITEM_SIZES");
        print_info_val<std::size_t, cl_device_info>(os, dev,
                CL_DEVICE_MAX_WORK_GROUP_SIZE,
                "CL_DEVICE_MAX_WORK_GROUP_SIZE");
        os << '\n';
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_MAX_CLOCK_FREQUENCY,
                "CL_DEVICE_MAX_CLOCK_FREQUENCY", "MHz");
        print_info_val<std::size_t, cl_device_info>(os, dev,
                CL_DEVICE_MAX_PARAMETER_SIZE,
                "CL_DEVICE_MAX_PARAMETER_SIZE", "byte");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_ADDRESS_BITS,
                "CL_DEVICE_ADDRESS_BITS", "bit");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_MEM_BASE_ADDR_ALIGN,
                "CL_DEVICE_MEM_BASE_ADDR_ALIGN", "bit");
        print_info_val<cl_ulong, cl_device_info>(os, dev,
                CL_DEVICE_MAX_MEM_ALLOC_SIZE,
                "CL_DEVICE_MAX_MEM_ALLOC_SIZE", "byte");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE,
                "CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE", "byte");
        print_info_val<cl_ulong, cl_device_info>(os, dev,
                CL_DEVICE_GLOBAL_MEM_CACHE_SIZE,
                "CL_DEVICE_GLOBAL_MEM_CACHE_SIZE", "byte");
        print_info_val<cl_ulong, cl_device_info>(os, dev,
                CL_DEVICE_GLOBAL_MEM_SIZE,
                "CL_DEVICE_GLOBAL_MEM_SIZE", "byte");
        print_info_val<cl_ulong, cl_device_info>(os, dev,
                CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE,
                "CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE", "byte");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_MAX_CONSTANT_ARGS,
                "CL_DEVICE_MAX_CONSTANT_ARGS");
        print_info_val<cl_ulong, cl_device_info>(os, dev,
                CL_DEVICE_LOCAL_MEM_SIZE,
                "CL_DEVICE_LOCAL_MEM_SIZE", "byte");
        print_info_val<cl_bool, cl_device_info>(os, dev,
                CL_DEVICE_ERROR_CORRECTION_SUPPORT,
                "CL_DEVICE_ERROR_CORRECTION_SUPPORT");
        print_info_val<std::size_t, cl_device_info>(os, dev,
                CL_DEVICE_PROFILING_TIMER_RESOLUTION,
                "CL_DEVICE_PROFILING_TIMER_RESOLUTION", "ns");
        os << '\n';
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF");
        os << '\n';
#if VSMC_OPENCL_VERSION >= 110
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_INT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_INT");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE");
        print_info_val<cl_uint, cl_device_info>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF");
#endif // VSMC_OPENCL_VERSION >= 110
    }

    /// \brief Query context information
    template <typename CharT, typename Traits>
    static void info (std::basic_ostream<CharT, Traits> &os,
            const ::cl::Context &ctx)
    {
        std::vector< ::cl::Device> device;
        ctx.getInfo(static_cast<cl_context_info>(CL_CONTEXT_DEVICES), &device);
        for (std::vector< ::cl::Device>::const_iterator d = device.begin();
                d != device.end(); ++d)
            info(os, *d);
    }

    /// \brief Query program informaiton
    template <typename CharT, typename Traits>
    static void info (std::basic_ostream<CharT, Traits> &os,
            const ::cl::Program &prog)
    {
        print_info_val<cl_uint, cl_program_info>(os, prog,
                CL_PROGRAM_NUM_DEVICES, "CL_PROGRAM_NUM_DEVICES");
        print_info_val<std::size_t, cl_program_info>(os, prog,
                CL_PROGRAM_BINARY_SIZES, "CL_PROGRAM_BINARY_SIZES", "byte");
    }

    /// \brief Query kernel information
    template <typename CharT, typename Traits>
    static void info (std::basic_ostream<CharT, Traits> &os,
            const ::cl::Kernel &kern)
    {
        ::cl::Context ctx;
        kern.getInfo(static_cast<cl_kernel_info>(CL_KERNEL_CONTEXT), &ctx);
        std::vector< ::cl::Device> device;
        ctx.getInfo(static_cast<cl_kernel_info>(CL_CONTEXT_DEVICES), &device);
        for (std::vector< ::cl::Device>::const_iterator d = device.begin();
                d != device.end(); ++d) {
            print_info_val<std::string, cl_device_info>(os, *d,
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
    }

    private :

    template <typename CharT, typename Traits>
    static void print_equal (std::basic_ostream<CharT, Traits> &os)
    {os << std::string(90, '=') << '\n';}

    template <typename CharT, typename Traits>
    static void print_dash (std::basic_ostream<CharT, Traits> &os)
    {os << std::string(90, '-') << '\n';}

    template <typename CharT, typename Traits>
    static void print_dev_type (
            std::basic_ostream<CharT, Traits> &os, const ::cl::Device &dev)
    {
        cl_device_type type;
        std::string info;
        dev.getInfo(static_cast<cl_device_info>(CL_DEVICE_TYPE), &type);

        append_bit_field<cl_device_type>(CL_DEVICE_TYPE_CPU,
                type, "CL_DEVICE_TYPE_CPU", info);
        append_bit_field<cl_device_type>(CL_DEVICE_TYPE_GPU,
                type, "CL_DEVICE_TYPE_GPU", info);
        append_bit_field<cl_device_type>(CL_DEVICE_TYPE_ACCELERATOR,
                type, "CL_DEVICE_TYPE_ACCELERATOR", info);
        append_bit_field<cl_device_type>(CL_DEVICE_TYPE_DEFAULT,
                type, "CL_DEVICE_TYPE_DEFAULT", info);
        append_bit_field<cl_device_type>(CL_DEVICE_TYPE_CUSTOM,
                type, "CL_DEVICE_TYPE_CUSTOM", info);

        print_name(os, "CL_DEVICE_TYPE");
        print_val(os, info);
        os << '\n';
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

        std::string::size_type pos = val.find(' ');
        if (pos == std::string::npos) {
            os << val;
            return;
        }

        std::string::size_type init = 0;
        std::vector<std::string> val_tmp;
        std::vector<std::string> val_vec;
        while (pos != std::string::npos) {
            val_tmp.push_back(val.substr(init, pos - init + 1));
            init = pos + 1;
            pos = val.find(' ', init);
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
{CLQuery::info(os); return os;}

} // namespace vsmc

namespace cl {

/// \brief Query device information in a given platform
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const Platform &plat)
{vsmc::CLQuery::info(os, plat); return os;}

/// \brief Query device information in a given context
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const Context &ctx)
{vsmc::CLQuery::info(os, ctx); return os;}

/// \brief Query device information
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const Device &dev)
{vsmc::CLQuery::info(os, dev); return os;}

/// \brief Query program information
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const Program &prog)
{vsmc::CLQuery::info(os, prog); return os;}

/// \brief Query kernel information
/// \ingroup OpenCL
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const Kernel &kern)
{vsmc::CLQuery::info(os, kern); return os;}

} // namespace cl

#endif // VSMC_OPENCL_CL_QUERY_HPP

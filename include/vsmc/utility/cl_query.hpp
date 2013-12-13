#ifndef VSMC_UTILITY_CL_QUERY_HPP
#define VSMC_UTILITY_CL_QUERY_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/cl_wrapper.hpp>

namespace vsmc {

/// \brief Query information of OpenCL devices
/// \ingroup CLUtility
class CLQuery
{
    public :

    template<typename OutputStream>
    static void print (OutputStream &os)
    {
        std::vector<cl::Platform> platform;
        cl::Platform::get(&platform);
        for (std::vector<cl::Platform>::const_iterator p = platform.begin();
                p != platform.end(); ++p)
            print(os, *p);
    }

    template<typename OutputStream>
    static void print (OutputStream &os, const cl::Platform &plat)
    {
        print_equal(os);

        print_info_val<std::string>(os, plat,
                CL_PLATFORM_NAME, "CL_PLATFORM_NAME");
        print_info_val<std::string>(os, plat,
                CL_PLATFORM_VENDOR, "CL_PLATFORM_VENDOR");
        print_info_val<std::string>(os, plat,
                CL_PLATFORM_VERSION, "CL_PLATFORM_VERSION");
        print_info_val<std::string>(os, plat,
                CL_PLATFORM_PROFILE, "CL_PLATFORM_PROFILE");
        print_info_val<std::string>(os, plat,
                CL_PLATFORM_EXTENSIONS, "CL_PLATFORM_EXTENSIONS");

        std::vector<cl::Device> device;
        plat.getDevices(CL_DEVICE_TYPE_ALL, &device);
        for (std::vector<cl::Device>::const_iterator d = device.begin();
                d != device.end(); ++d)
            print(os, *d);
    }

    template <typename OutputStream>
    static void print (OutputStream &os, const cl::Context &ctx)
    {
        print_equal(os);

        std::vector<cl::Device> device;
        ctx.getInfo(CL_CONTEXT_DEVICES, &device);
        for (std::vector<cl::Device>::const_iterator d = device.begin();
                d != device.end(); ++d)
            print(os, *d);
    }

    template<typename OutputStream>
    static void print (OutputStream &os, const cl::Device &dev)
    {
        print_dash(os);

        print_info_val<std::string>(os, dev,
                CL_DEVICE_NAME, "CL_DEVICE_NAME");
        print_dev_type(os, dev);
        print_info_val<std::string>(os, dev,
                CL_DEVICE_VENDOR, "CL_DEVICE_VENDOR");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_VENDOR_ID, "CL_DEVICE_VENDOR_ID");
        print_info_val<std::string>(os, dev,
                CL_DRIVER_VERSION, "CL_DRIVER_VERSION");
        print_info_val<std::string>(os, dev,
                CL_DEVICE_PROFILE, "CL_DEVICE_PROFILE");
        print_info_val<std::string>(os, dev,
                CL_DEVICE_VERSION, "CL_DEVICE_VERSION");
        print_info_val<std::string>(os, dev,
                CL_DEVICE_OPENCL_C_VERSION, "CL_DEVICE_OPENCL_C_VERSION");
        print_info_val<std::string>(os, dev,
                CL_DEVICE_EXTENSIONS, "CL_DEVICE_EXTENSIONS");

        os << '\n';

        print_info_val<cl_bool>(os, dev,
                CL_DEVICE_AVAILABLE, "CL_DEVICE_AVAILABLE");
        print_info_val<cl_bool>(os, dev,
                CL_DEVICE_COMPILER_AVAILABLE, "CL_DEVICE_COMPILER_AVAILABLE");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_CLOCK_FREQUENCY,
                "CL_DEVICE_MAX_CLOCK_FREQUENCY", "MHz");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_ADDRESS_BITS,
                "CL_DEVICE_ADDRESS_BITS", "bit");
        print_info_val<cl_ulong>(os, dev,
                CL_DEVICE_MAX_MEM_ALLOC_SIZE,
                "CL_DEVICE_MAX_MEM_ALLOC_SIZE", "byte");
        print_dev_sfp_config(os, dev);
        print_dev_dfp_config(os, dev);

        os << '\n';

        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_COMPUTE_UNITS,
                "CL_DEVICE_MAX_COMPUTE_UNITS");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS,
                "CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS");
        print_info_val<std::vector<std::size_t> >(os, dev,
                CL_DEVICE_MAX_WORK_ITEM_SIZES,
                "CL_DEVICE_MAX_WORK_ITEM_SIZES");
        print_info_val<std::size_t>(os, dev,
                CL_DEVICE_MAX_WORK_GROUP_SIZE,
                "CL_DEVICE_MAX_WORK_GROUP_SIZE");

        os << '\n';

        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF");

        os << '\n';

        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_INT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_INT");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF");

        os << '\n';

        print_info_val<cl_bool>(os, dev,
                CL_DEVICE_IMAGE_SUPPORT,
                "CL_DEVICE_IMAGE_SUPPORT");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_READ_IMAGE_ARGS,
                "CL_DEVICE_MAX_READ_IMAGE_ARGS");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_WRITE_IMAGE_ARGS,
                "CL_DEVICE_MAX_WRITE_IMAGE_ARGS");
        print_info_val<std::size_t>(os, dev,
                CL_DEVICE_IMAGE2D_MAX_WIDTH,
                "CL_DEVICE_IMAGE2D_MAX_WIDTH", "pixel");
        print_info_val<std::size_t>(os, dev,
                CL_DEVICE_IMAGE2D_MAX_HEIGHT,
                "CL_DEVICE_IMAGE2D_MAX_HEIGHT", "pixel");
        print_info_val<std::size_t>(os, dev,
                CL_DEVICE_IMAGE3D_MAX_WIDTH,
                "CL_DEVICE_IMAGE3D_MAX_WIDTH", "pixel");
        print_info_val<std::size_t>(os, dev,
                CL_DEVICE_IMAGE3D_MAX_HEIGHT,
                "CL_DEVICE_IMAGE3D_MAX_HEIGHT", "pixel");
        print_info_val<std::size_t>(os, dev,
                CL_DEVICE_IMAGE3D_MAX_DEPTH,
                "CL_DEVICE_IMAGE3D_MAX_DEPTH", "pixel");
        // print_info_val<std::size_t>(os, dev,
        //         CL_DEVICE_IMAGE_MAX_BUFFER_SIZE,
        //         "CL_DEVICE_IMAGE_MAX_BUFFER_SIZE");
        // print_info_val<std::size_t>(os, dev,
        //         CL_DEVICE_IMAGE_MAX_ARRAY_SIZE,
        //         "CL_DEVICE_IMAGE_MAX_ARRAY_SIZE");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_SAMPLERS,
                "CL_DEVICE_MAX_SAMPLERS");

        os << '\n';

        print_info_val<std::size_t>(os, dev,
                CL_DEVICE_MAX_PARAMETER_SIZE,
                "CL_DEVICE_MAX_PARAMETER_SIZE", "byte");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_MEM_BASE_ADDR_ALIGN,
                "CL_DEVICE_MEM_BASE_ADDR_ALIGN", "bit");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE,
                "CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE", "byte");
        print_info_val<cl_ulong>(os, dev,
                CL_DEVICE_GLOBAL_MEM_CACHE_SIZE,
                "CL_DEVICE_GLOBAL_MEM_CACHE_SIZE", "byte");
        print_info_val<cl_ulong>(os, dev,
                CL_DEVICE_GLOBAL_MEM_SIZE,
                "CL_DEVICE_GLOBAL_MEM_SIZE", "byte");
        print_info_val<cl_ulong>(os, dev,
                CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE,
                "CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE", "byte");
        print_info_val<cl_uint>(os, dev,
                CL_DEVICE_MAX_CONSTANT_ARGS,
                "CL_DEVICE_MAX_CONSTANT_ARGS");
        print_info_val<cl_ulong>(os, dev,
                CL_DEVICE_LOCAL_MEM_SIZE,
                "CL_DEVICE_LOCAL_MEM_SIZE", "byte");
        print_info_val<cl_bool>(os, dev,
                CL_DEVICE_ERROR_CORRECTION_SUPPORT,
                "CL_DEVICE_ERROR_CORRECTION_SUPPORT");
        print_info_val<cl_bool>(os, dev,
                CL_DEVICE_HOST_UNIFIED_MEMORY,
                "CL_DEVICE_HOST_UNIFIED_MEMORY");
        print_info_val<std::size_t>(os, dev,
                CL_DEVICE_PROFILING_TIMER_RESOLUTION,
                "CL_DEVICE_PROFILING_TIMER_RESOLUTION", "ns");
        print_info_val<cl_bool>(os, dev,
                CL_DEVICE_ENDIAN_LITTLE,
                "CL_DEVICE_ENDIAN_LITTLE");
    }

    template <typename OutputStream>
    static void print (OutputStream &os, const cl::Program &prog)
    {
        print_dash(os);

        print_info_val<cl_uint>(os, prog,
                CL_PROGRAM_NUM_DEVICES, "CL_PROGRAM_NUM_DEVICES");
        print_info_val<std::size_t>(os, prog,
                CL_PROGRAM_BINARY_SIZES, "CL_PROGRAM_BINARY_SIZES", "byte");
    }

    template <typename OutputStream>
    static void print (OutputStream &os, const cl::Kernel &kern)
    {
        print_dash(os);

        cl::Context ctx;
        kern.getInfo(CL_KERNEL_CONTEXT, &ctx);
        std::vector<cl::Device> device;
        ctx.getInfo(CL_CONTEXT_DEVICES, &device);
        for (std::vector<cl::Device>::const_iterator d = device.begin();
                d != device.end(); ++d) {
            print_info_val<std::string>(os, *d,
                    CL_DEVICE_NAME, "CL_DEVICE_NAME");
            print_info_val<std::string>(os, kern,
                    CL_KERNEL_FUNCTION_NAME, "CL_KERNEL_FUNCTION_NAME");
            print_info_val<cl_uint>(os, kern,
                    CL_KERNEL_NUM_ARGS, "CL_KERNEL_NUM_ARGS");
            print_kernwginfo_val<std::size_t>(os, kern, *d,
                    CL_KERNEL_WORK_GROUP_SIZE, "CL_KERNEL_WORK_GROUP_SIZE");
            print_kernwginfo_val<std::size_t>(os, kern, *d,
                    CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE,
                    "CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE");
            print_kernwginfo_val<cl_ulong>(os, kern, *d,
                    CL_KERNEL_LOCAL_MEM_SIZE, "CL_KERNEL_LOCAL_MEM_SIZE",
                    "byte");
            print_kernwginfo_val<cl_ulong>(os, kern, *d,
                    CL_KERNEL_PRIVATE_MEM_SIZE, "CL_KERNEL_PRIVATE_MEM_SIZE",
                    "byte");
        }
    }

    private :

    template<typename OutputStream>
    static void print_equal (OutputStream &os)
    {
        for (int l = 0; l != 78; ++l)
            os << '=';
        os << '\n';
    }

    template<typename OutputStream>
    static void print_dash (OutputStream &os)
    {
        for (int l = 0; l != 78; ++l)
            os << '-';
        os << '\n';
    }

    template<typename OutputStream>
    static void print_dev_type (OutputStream &os, const cl::Device &dev)
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
        print_name(os, "CL_DEVICE_TYPE");
        print_val(os, info);
        os << '\n';
    }

    template<typename OutputStream>
    static void print_dev_sfp_config (OutputStream &os, const cl::Device &dev)
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

        print_name(os, "CL_DEVICE_SINGLE_FP_CONFIG");
        print_val(os, info);
        os << '\n';
    }

    template<typename OutputStream>
    static void print_dev_dfp_config (OutputStream &os, const cl::Device &dev)
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

        print_name(os, "CL_DEVICE_DOUBLE_FP_CONFIG");
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

    template<typename T, typename OutputStream,
        typename CLObj, typename InfoType>
    static void print_info_val (OutputStream &os,
            const CLObj &obj, InfoType info,
            const std::string &name, const std::string &unit = "")
    {
        T val;
        obj.getInfo(info, &val);
        print_name(os, name);
        print_val(os, val);
        os << ' ' << unit << '\n';
    }

    template <typename T, typename OutputStream>
    static void print_kernwginfo_val (OutputStream &os,
            const cl::Kernel &kern, const cl::Device &dev,
            cl_kernel_work_group_info info,
            const std::string &name, const std::string &unit = "")
    {
        T val;
        kern.getWorkGroupInfo(dev, info, &val);
        print_name(os, name);
        print_val(os, val);
        os << ' ' << unit << '\n';
    }

    template <typename OutputStream>
    static void print_name (OutputStream &os, const std::string &name)
    {os << std::setw(40) << std::left << name << ' ';}

    template <typename OutputStream, typename T>
    static void print_val (OutputStream &os, const T &val) {os << val;}

    template<typename OutputStream, typename T>
    static void print_val (OutputStream &os, const std::vector<T> &val)
    {
        for (typename std::vector<T>::const_iterator v = val.begin();
                v != val.end(); ++v)
            os << *v << ' ';
    }
}; //class CLQuery

/// \brief Print information of all platforms and devices
/// \ingroup CLUtility
template<typename OutputStream>
inline OutputStream &operator<< (OutputStream &os, const CLQuery &query)
{query.print(os); return os;}

} // namespace vsmc

namespace cl {

/// \brief Print information of all devices in a given platform
/// \ingroup CLUtility
template <typename OutputStream>
inline OutputStream &operator<< (OutputStream &os, const Platform &plat)
{vsmc::CLQuery::print(os, plat); return os;}

/// \brief Print information of all devices in a given context
/// \ingroup CLUtility
template <typename OutputStream>
inline OutputStream &operator<< (OutputStream &os, const Context &ctx)
{vsmc::CLQuery::print(os, ctx); return os;}

/// \brief Print information a given device
/// \ingroup CLUtility
template <typename OutputStream>
OutputStream &operator<< (OutputStream &os, const Device &dev)
{vsmc::CLQuery::print(os, dev); return os;}

/// \brief Print information a given program
/// \ingroup CLUtility
template <typename OutputStream>
inline OutputStream &operator<< (OutputStream &os, const Program &prog)
{vsmc::CLQuery::print(os, prog); return os;}

/// \brief Print information a given kernel
/// \ingroup CLUtility
template <typename OutputStream>
inline OutputStream &operator<< (OutputStream &os, const Kernel &kern)
{vsmc::CLQuery::print(os, kern); return os;}

} // namespace cl

#endif // VSMC_UTILITY_CL_QUERY_HPP

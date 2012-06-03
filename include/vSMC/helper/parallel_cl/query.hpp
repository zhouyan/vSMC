#ifndef V_SMC_HELPER_PARALLEL_CL_QUERY_HPP
#define V_SMC_HELPER_PARALLEL_CL_QUERY_HPP

namespace vSMC {

class QueryCL
{
    public :

    template<typename CharT, typename Traits>
    static void print (std::basic_ostream<CharT, Traits> &os = std::cout)
    {
        std::vector<cl::Platform> platform;
        cl::Platform::get(&platform);
        for (std::vector<cl::Platform>::const_iterator p = platform.begin();
                p != platform.end(); ++p) {
            print_equal(os);
            print(os, *p);
        }
        print_equal(os);
    }

    template<typename CharT, typename Traits>
    static void print (std::basic_ostream<CharT, Traits> &os,
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

        std::vector<cl::Device> device;
        plat.getDevices(CL_DEVICE_TYPE_ALL, &device);
        for (std::vector<cl::Device>::const_iterator d = device.begin();
                d != device.end(); ++d) {
            print_dash(os);
            print(os, *d);
        }
        print_dash(os);
    }

    template<typename CharT, typename Traits>
    static void print (std::basic_ostream<CharT, Traits> &os,
            const cl::Device &dev)
    {
        print_dev<std::string>(os, dev,
                CL_DEVICE_NAME, "CL_DEVICE_NAME");
        print_dev<std::string>(os, dev,
                CL_DEVICE_VENDOR, "CL_DEVICE_VENDOR");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_VENDOR_ID, "CL_DEVICE_VENDOR_ID");
        print_dev<std::string>(os, dev,
                CL_DRIVER_VERSION, "CL_DRIVER_VERSION");
        print_dev<std::string>(os, dev,
                CL_DEVICE_PROFILE, "CL_DEVICE_PROFILE");
        print_dev<std::string>(os, dev,
                CL_DEVICE_VERSION, "CL_DEVICE_VERSION");
        print_dev<std::string>(os, dev,
                CL_DEVICE_OPENCL_C_VERSION, "CL_DEVICE_OPENCL_C_VERSION");
        print_dev<std::string>(os, dev,
                CL_DEVICE_EXTENSIONS, "CL_DEVICE_EXTENSIONS");

        os << '\n';

        print_dev<cl_bool>(os, dev,
                CL_DEVICE_AVAILABLE, "CL_DEVICE_AVAILABLE");
        print_dev<cl_bool>(os, dev,
                CL_DEVICE_COMPILER_AVAILABLE, "CL_DEVICE_COMPILER_AVAILABLE");
        // print_dev<cl_bool>(os, dev,
        //         CL_DEVICE_LINKER_AVAILABLE, "CL_DEVICE_LINKER_AVAILABLE");
        // print_dev<std::string>(os, dev,
        //         CL_DEVICE_BUILT_IN_KERNELS, "CL_DEVICE_BUILT_IN_KERNELS");
        // print_dev<std::size_t>(os, dev,
        //         CL_DEVICE_PRINTF_BUFFER_SIZE,
        //         "CL_DEVICE_PRINTF_BUFFER_SIZE", "bytes");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_MAX_CLOCK_FREQUENCY,
                "CL_DEVICE_MAX_CLOCK_FREQUENCY", "MHz");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_ADDRESS_BITS,
                "CL_DEVICE_ADDRESS_BITS", "bits");
        print_dev<cl_ulong>(os, dev,
                CL_DEVICE_MAX_MEM_ALLOC_SIZE,
                "CL_DEVICE_MAX_MEM_ALLOC_SIZE", "bytes");

        os << '\n';

        print_dev<cl_uint>(os, dev,
                CL_DEVICE_MAX_COMPUTE_UNITS,
                "CL_DEVICE_MAX_COMPUTE_UNITS");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS,
                "CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS");
        print_dev<std::size_t>(os, dev,
                CL_DEVICE_MAX_WORK_GROUP_SIZE,
                "CL_DEVICE_MAX_WORK_GROUP_SIZE");

        os << '\n';

        print_dev<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF,
                "CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF");

        os << '\n';

        print_dev<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_INT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_INT");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF,
                "CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF");

        os << '\n';

        print_dev<cl_bool>(os, dev,
                CL_DEVICE_IMAGE_SUPPORT,
                "CL_DEVICE_IMAGE_SUPPORT");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_MAX_READ_IMAGE_ARGS,
                "CL_DEVICE_MAX_READ_IMAGE_ARGS");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_MAX_WRITE_IMAGE_ARGS,
                "CL_DEVICE_MAX_WRITE_IMAGE_ARGS");
        print_dev<std::size_t>(os, dev,
                CL_DEVICE_IMAGE2D_MAX_WIDTH,
                "CL_DEVICE_IMAGE2D_MAX_WIDTH");
        print_dev<std::size_t>(os, dev,
                CL_DEVICE_IMAGE2D_MAX_HEIGHT,
                "CL_DEVICE_IMAGE2D_MAX_HEIGHT");
        print_dev<std::size_t>(os, dev,
                CL_DEVICE_IMAGE3D_MAX_WIDTH,
                "CL_DEVICE_IMAGE3D_MAX_WIDTH");
        print_dev<std::size_t>(os, dev,
                CL_DEVICE_IMAGE3D_MAX_HEIGHT,
                "CL_DEVICE_IMAGE3D_MAX_HEIGHT");
        print_dev<std::size_t>(os, dev,
                CL_DEVICE_IMAGE3D_MAX_DEPTH,
                "CL_DEVICE_IMAGE3D_MAX_DEPTH");
        // print_dev<std::size_t>(os, dev,
        //         CL_DEVICE_IMAGE_MAX_BUFFER_SIZE,
        //         "CL_DEVICE_IMAGE_MAX_BUFFER_SIZE");
        // print_dev<std::size_t>(os, dev,
        //         CL_DEVICE_IMAGE_MAX_ARRAY_SIZE,
        //         "CL_DEVICE_IMAGE_MAX_ARRAY_SIZE");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_MAX_SAMPLERS,
                "CL_DEVICE_MAX_SAMPLERS");

        os << '\n';

        print_dev<std::size_t>(os, dev,
                CL_DEVICE_MAX_PARAMETER_SIZE,
                "CL_DEVICE_MAX_PARAMETER_SIZE", "bytes");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_MEM_BASE_ADDR_ALIGN,
                "CL_DEVICE_MEM_BASE_ADDR_ALIGN", "bits");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE,
                "CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE", "bytes");
        print_dev<cl_ulong>(os, dev,
                CL_DEVICE_GLOBAL_MEM_CACHE_SIZE,
                "CL_DEVICE_GLOBAL_MEM_CACHE_SIZE", "bytes");
        print_dev<cl_ulong>(os, dev,
                CL_DEVICE_GLOBAL_MEM_SIZE,
                "CL_DEVICE_GLOBAL_MEM_SIZE", "bytes");
        print_dev<cl_ulong>(os, dev,
                CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE,
                "CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE", "bytes");
        print_dev<cl_uint>(os, dev,
                CL_DEVICE_MAX_CONSTANT_ARGS,
                "CL_DEVICE_MAX_CONSTANT_ARGS");
        print_dev<cl_ulong>(os, dev,
                CL_DEVICE_LOCAL_MEM_SIZE,
                "CL_DEVICE_LOCAL_MEM_SIZE");
        print_dev<cl_bool>(os, dev,
                CL_DEVICE_ERROR_CORRECTION_SUPPORT,
                "CL_DEVICE_ERROR_CORRECTION_SUPPORT");
        print_dev<cl_bool>(os, dev,
                CL_DEVICE_HOST_UNIFIED_MEMORY,
                "CL_DEVICE_HOST_UNIFIED_MEMORY");
        print_dev<std::size_t>(os, dev,
                CL_DEVICE_PROFILING_TIMER_RESOLUTION,
                "CL_DEVICE_PROFILING_TIMER_RESOLUTION", "ns");
        print_dev<cl_bool>(os, dev,
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

    template<typename CharT, typename Traits, typename T>
    static void print_vec (std::basic_ostream<CharT, Traits> &os,
            std::vector<T> &vec)
    {
        for (typename std::vector<T>::const_iterator v = vec.begin();
                v != vec.end(); ++v)
            os << *v;
        os << '\n';
    }

    template<typename T, typename CharT, typename Traits>
    static void print_dev (std::basic_ostream<CharT, Traits> &os,
            const cl::Device &dev,
            cl_device_info info, const std::string &name,
            const std::string &unit = "")
    {
        T val;
        dev.getInfo(info, &val);
        os << std::setw(40) << std::left
            << name << val << ' ' << unit << '\n';
    }

}; //class QueryCL

template<typename CharT, typename Traits>
std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const vSMC::QueryCL &query)
{
    query.print(os);
    return os;
}

} // namespace vSMC

#endif // V_SMC_HELPER_PARALLEL_CL_QUERY_HPP

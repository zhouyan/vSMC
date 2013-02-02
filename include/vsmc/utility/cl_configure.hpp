#ifndef VSMC_UTILITY_CL_CONFIGURE_HPP
#define VSMC_UTILITY_CL_CONFIGURE_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>
#include <vsmc/utility/cl_wrapper.hpp>

namespace vsmc {

class CLConfigure
{
    public :

    CLConfigure () : local_size_(0) {}

    std::size_t local_size () const
    {
        return local_size_;
    }

    void local_size (std::size_t new_size)
    {
        local_size_ = new_size;
    }

    protected :

    void set_preferred_local_size (
            const cl::Kernel &kern, const cl::Device &dev)
    {
        local_size(get_preferred_local_size(kern, dev));
    }

    static std::size_t get_preferred_local_size (
            const cl::Kernel &kern, const cl::Device &dev)
    {
        std::size_t max_s;
        std::size_t mul_s;
        kern.getWorkGroupInfo(dev, CL_KERNEL_WORK_GROUP_SIZE, &max_s);
        kern.getWorkGroupInfo(dev,
                CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, &mul_s);

        return (max_s / mul_s) * mul_s;
    }

    private :

    std::size_t local_size_;
}; // class CLConfigure

} // namespace vsmc

#endif // VSMC_UTILITY_CL_CONFIGURE_HPP

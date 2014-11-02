//============================================================================
// include/vsmc/opencl/cl_setup.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_OPENCL_CL_SETUP_HPP
#define VSMC_OPENCL_CL_SETUP_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/internal/cl_wrapper.hpp>

namespace vsmc {

/// \brief Configure the default behavior of CLManager
/// \ingroup OpenCL
template <typename ID = CLDefault>
class CLSetup
{
    public :

    typedef ID id;

    static CLSetup<ID> &instance ()
    {
        static CLSetup<ID> config;

        return config;
    }

    /// \brief Default string value of the device, vendor, platform names
    const std::string &default_name () const {return default_;}

    /// \brief Set the device type using a string value
    ///
    /// \param name One of "GPU", "CPU", "Accelerator". Other values are
    /// treated as setting the default device type
    /// \return false if the default value is using, otherwise true
    bool device_type (const std::string &name)
    {
        if (name == std::string("CPU"))
            device_type(CL_DEVICE_TYPE_CPU);
        else if (name == std::string("GPU"))
            device_type(CL_DEVICE_TYPE_GPU);
        else if (name == std::string("Accelerator"))
            device_type(CL_DEVICE_TYPE_ACCELERATOR);
        else {
            device_type(CL_DEVICE_TYPE_DEFAULT);
            return false;
        }

        return true;
    }

    void device_type   (cl_device_type type)     {device_type_   = type;}
    void device        (const std::string &name) {device_        = name;}
    void device_vendor (const std::string &name) {device_vendor_ = name;}
    void platform      (const std::string &name) {platform_      = name;}

    cl_device_type device_type       () const {return device_type_;}
    const std::string &device        () const {return device_;}
    const std::string &device_vendor () const {return device_vendor_;}
    const std::string &platform      () const {return platform_;}

    bool default_device_type () const
    {return device_type_ == CL_DEVICE_TYPE_DEFAULT;}

    bool default_device        () const {return default_ == device_;}
    bool default_device_vendor () const {return default_ == device_vendor_;}
    bool default_platform      () const {return default_ == platform_;}

    bool check_device (const std::string &name) const
    {return check_name(name, device_);}

    bool check_device_vendor (const std::string &name) const
    {return check_name(name, device_vendor_);}

    bool check_platform (const std::string &name) const
    {return check_name(name, platform_);}

    private :

    cl_device_type device_type_;
    std::string default_;
    std::string device_;
    std::string device_vendor_;
    std::string platform_;

    CLSetup () :
        device_type_(CL_DEVICE_TYPE_DEFAULT), default_("vSMCOpenCLDefault"),
        device_(default_), device_vendor_(default_), platform_(default_) {}

    CLSetup (const CLSetup<ID> &);
    CLSetup<ID> &operator= (const CLSetup<ID> &);

    bool check_name (const std::string &name, const std::string &stored) const
    {
        if (stored == default_)
            return true;
        return name.find(stored) != std::string::npos;
    }
}; // class CLSetup

} // namespace vsmc

#endif // VSMC_OPENCL_CL_SETUP_HPP

//============================================================================
// vSMC/include/vsmc/opencl/cl_setup.hpp
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

    void device_type   (::cl_device_type type)   {device_type_   = type;}
    void device        (const std::string &name) {device_        = name;}
    void device_vendor (const std::string &name) {device_vendor_ = name;}
    void platform      (const std::string &name) {platform_      = name;}

    ::cl_device_type device_type     () const {return device_type_;}
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

    ::cl_device_type device_type_;
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

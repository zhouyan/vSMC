//============================================================================
// vSMC/vSMCExample/gmm/src/gmm_smc_cl.cpp
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

#include "gmm_smc_cl.hpp"

int main (int argc, char **argv)
{
#include "options_main.hpp"
#include "gmm_options_cl.hpp"
#include "options_process.hpp"

    vsmc::CLSetup<gmm_device> &gmm_setup =
        vsmc::CLSetup<gmm_device>::instance();
    gmm_setup.platform(PlatformName);
    gmm_setup.device_vendor(DeviceVendorName);
    gmm_setup.device_type(DeviceType);

    if (!vsmc::CLManager<gmm_device>::instance().setup()) {
        std::cout << "Failed to setup OpenCL environment" << std::endl;
        std::cout << "Platform name: " << PlatformName << std::endl;
        std::cout << "Device type:   " << DeviceType << std::endl;
        std::cout << "Device vendor: " << DeviceVendorName << std::endl;
        return -1;
    }

    if (FPTypeBits == 32) {
        gmm_do_smc<cl_float>(ParticleNum, IterNum, DataNum, DataFile,
                Threshold, vSMCIncPath, R123IncPath, SM, CM, Repeat);
    } else if (FPTypeBits == 64) {
        gmm_do_smc<cl_double>(ParticleNum, IterNum, DataNum, DataFile,
                Threshold, vSMCIncPath, R123IncPath, SM, CM, Repeat);
    } else {
        std::fprintf(stderr, "cl_type_bits has to be 32 or 64\n");
        return -1;
    }

    return 0;
}

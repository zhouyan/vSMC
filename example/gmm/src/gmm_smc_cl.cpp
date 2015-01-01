//============================================================================
// vSMC/example/gmm/src/gmm_smc_cl.cpp
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

#include "gmm_smc_cl.hpp"

int main (int argc, char **argv)
{
#include "options_main.hpp"
#include "gmm_options_cl.hpp"
#include "options_process.hpp"

    std::string platform_name;
    std::string device_type;
    std::string device_vendor_name;
    std::string build_option;
    std::size_t local_size;
    std::size_t particle_num;

#ifdef VSMC_GMM_SMC_CL_MPI
    vsmc::MPIEnvironment env(argc, argv);
    boost::mpi::communicator world(vsmc::MPICommunicator<>::instance().get(),
            boost::mpi::comm_duplicate);
    std::size_t R = static_cast<std::size_t>(world.rank());
    std::size_t S = static_cast<std::size_t>(world.size());
    platform_name = PlatformName.size() >= S ?
        PlatformName[R] : PlatformName.front();
    device_type = DeviceType.size() >= S ?
        DeviceType[R] : DeviceType.front();
    device_vendor_name = DeviceVendorName.size() >= S ?
        DeviceVendorName[R] : DeviceVendorName.front();
    build_option = BuildOption.size() >= S ?
        BuildOption[R] : BuildOption.front();
    local_size = LocalSize.size() >= S ?
        LocalSize[R] : LocalSize.front();
    particle_num = ParticleNum.size() >= S ?
        ParticleNum[R] : ParticleNum.front();
#else
    platform_name = PlatformName.front();
    device_type = DeviceType.front();
    device_vendor_name = DeviceVendorName.front();
    build_option = BuildOption.front();
    local_size = LocalSize.front();
    particle_num = ParticleNum.front();
#endif

    try {
        vsmc::CLSetup<> &gmm_setup = vsmc::CLSetup<>::instance();
        gmm_setup.platform(platform_name);
        gmm_setup.device_type(device_type);
        gmm_setup.device_vendor(device_vendor_name);
        if (!vsmc::CLManager<>::instance().setup()) {
            std::cout << "Failed to setup OpenCL environment" << std::endl;
            std::cout << "Platform name: " << platform_name << std::endl;
            std::cout << "Device type:   " << device_type << std::endl;
            std::cout << "Device vendor: " << device_vendor_name << std::endl;
            return 0;
        }

        if (FPTypeBits == 32) {
            gmm_do_smc<cl_float>(particle_num, IterNum, DataNum, DataFile,
                    Threshold, vSMCIncPath, R123IncPath, build_option,
                    local_size, SM, CM, Repeat);
        } else if (FPTypeBits == 64) {
            gmm_do_smc<cl_double>(particle_num, IterNum, DataNum, DataFile,
                    Threshold, vSMCIncPath, R123IncPath, build_option,
                    local_size, SM, CM, Repeat);
        } else {
            std::cout << "cl_type_bits has to be 32 or 64" << std::endl;
        }
    } catch (cl::Error &err) {
        std::cout << "Runtime Error" << std::endl;
        std::cout << err.err() << std::endl;
        std::cout << err.what() << std::endl;
    } catch (...) {
        std::cout << "Runtime Error" << std::endl;
    }

    return 0;
}

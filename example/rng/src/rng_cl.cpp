//============================================================================
// vSMC/example/rng/src/rng_cl.cpp
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

#include "rng_cl.hpp"

int main(int argc, char **argv)
{
    if (argc < 6) {
        std::cout << "Usage: " << argv[0] << " <platform name>"
                  << " <device vendor>"
                  << " <device type>"
                  << " <fp type>"
                  << " <number of samples>" << std::endl;
        return -1;
    }

    vsmc::CLSetup<> &rng_setup = vsmc::CLSetup<>::instance();
    rng_setup.platform(argv[1]);
    rng_setup.device_vendor(argv[2]);
    rng_setup.device_type(argv[3]);

    if (!vsmc::CLManager<>::instance().setup()) {
        std::cout << "Failed to setup OpenCL environment" << std::endl;
        std::cout << "Platform name: " << argv[1] << std::endl;
        std::cout << "Device type:   " << argv[2] << std::endl;
        std::cout << "Device vendor: " << argv[3] << std::endl;
        return -1;
    }
    vsmc::CLManager<> &manager = vsmc::CLManager<>::instance();

    bool use_double = false;
    if (std::string(argv[4]) == std::string("double"))
        use_double = true;

    std::size_t N = static_cast<std::size_t>(std::atoi(argv[5]));

    std::ifstream src_file("rng_cl.cl");
    std::string src((std::istreambuf_iterator<char>(src_file)),
        (std::istreambuf_iterator<char>()));
    src_file.close();
    src = use_double ? "#define VSMC_HAS_OPENCL_DOUBLE 1\n" + src :
                       "#define VSMC_HAS_OPENCL_DOUBLE 0\n" + src;

    std::string opt;
    for (int i = 6; i != argc; ++i) {
        opt += " ";
        opt += argv[i];
    }

    vsmc::CLProgram program(manager.create_program(src));
    cl_int status = program.build(manager.device_vec(), opt);
    if (status != CL_SUCCESS) {
        std::vector<vsmc::CLDevice> dev_vec(program.get_device());
        std::string equal(75, '=');
        std::string dash(75, '-');
        std::string name;
        for (const auto &dev : dev_vec) {
            dev.get_info(CL_DEVICE_NAME, name);
            std::cout << equal << std::endl;
            if (program.build_status(dev) == CL_BUILD_SUCCESS)
                std::cout << "Build success for " << name << std::endl;
            else
                std::cout << "Build failure for " << name << std::endl;
            std::cout << dash << std::endl;
            std::cout << program.build_log(dev) << std::endl;
        }
        std::cout << equal << std::endl;
        return -1;
    }

     if (use_double)
         rng_cl<cl_double>(N, program);
     else
         rng_cl<cl_float>(N, program);

    return 0;
}

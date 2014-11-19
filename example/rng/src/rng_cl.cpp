//============================================================================
// vSMC/vSMCExample/rng/src/rng_cl.cpp
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

#include "rng_cl.hpp"

int main (int argc, char **argv)
{
    if (argc < 6) {
        std::cout << "Usage: " << argv[0]
            << " <platform name>"
            << " <device vendor>"
            << " <device type>"
            << " <fp type>"
            << " <number of samples>"
            << std::endl;
        return -1;
    }

    vsmc::CLSetup<rng_device> &rng_setup =
        vsmc::CLSetup<rng_device>::instance();
    rng_setup.platform(argv[1]);
    rng_setup.device_vendor(argv[2]);
    rng_setup.device_type(argv[3]);

    if (!vsmc::CLManager<rng_device>::instance().setup()) {
        std::cout << "Failed to setup OpenCL environment" << std::endl;
        std::cout << "Platform name: " << argv[1] << std::endl;
        std::cout << "Device type:   " << argv[2] << std::endl;
        std::cout << "Device vendor: " << argv[3] << std::endl;
        return -1;
    }
    vsmc::CLManager<rng_device> &manager =
        vsmc::CLManager<rng_device>::instance();

    bool use_double = false;
    if (std::string(argv[4]) == std::string("double"))
        use_double = true;

    std::size_t N = static_cast<std::size_t>(std::atoi(argv[5]));

    std::ifstream src_file("rng_cl.cl");
    std::string src(
            (std::istreambuf_iterator<char>(src_file)),
            (std::istreambuf_iterator<char>()));
    src_file.close();
    src = use_double ?
        "#define VSMC_HAS_OPENCL_DOUBLE 1\n" + src:
        "#define VSMC_HAS_OPENCL_DOUBLE 0\n" + src;

    std::string opt;
    for (int i = 6; i != argc; ++i) {
        opt += " ";
        opt += argv[i] ;
    }

    cl::Program program = manager.create_program(src);
    std::string name;
    manager.platform().getInfo(static_cast<cl_device_info>(CL_PLATFORM_NAME),
            &name);

    std::ofstream output_log;
    output_log.open("rng_cl.log");
    try {
        program.build(manager.device_vec(), opt.c_str());
        manager.print_build_log(program, output_log);
    } catch (...) {
        manager.print_build_log(program, output_log);
        output_log.close();
        output_log.clear();
        throw;
    }
    output_log.close();
    output_log.clear();

    std::cout << std::string(120, '=') << std::endl;
    std::cout << "Using platform: " << name << std::endl;
    manager.device().getInfo(static_cast<cl_device_info>(CL_DEVICE_NAME),
            &name);
    std::cout << "Using device:   " << name << std::endl;
    std::cout << "Using fp type:  " << (use_double ? "double" : "float")
        << std::endl;
    std::cout << std::string(120, '-') << std::endl;
    std::cout << "Number of samples: " << N << std::endl;

    if (use_double)
        rng_cl<cl_double>(N, program);
    else
        rng_cl<cl_float>(N, program);

    return 0;
}

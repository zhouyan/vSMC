//============================================================================
// vSMC/example/pf/src/pf_cl.cpp
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

#include "pf_cl.hpp"

int main (int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << argv[0]
            << " <input file>"
            << " <output file>"
            << " <optional OpenCL compiler options>"
            << std::endl;
        return -1;
    }

    if (!vsmc::CLManager<vsmc::CLDefault>::instance().setup()) {
        std::cout << "Failed to setup OpenCL environment" << std::endl;
        return -1;
    }

    vsmc::Sampler<cv> sampler(ParticleNum);
    std::ifstream src_file("pf_cl.cl");
    std::string src(
            (std::istreambuf_iterator<char>(src_file)),
            (std::istreambuf_iterator<char>()));
    src_file.close();
    std::string opt;
    for (int i = 3; i != argc; ++i) {
        opt += " ";
        opt += argv[i];
    }
    sampler.particle().value().build(src, opt);

    std::string name;
    sampler.particle().value().manager().platform().getInfo(
            static_cast<cl_device_info>(CL_PLATFORM_NAME), &name);
    std::cout << "Using platform: " << name << std::endl;
    sampler.particle().value().manager().device().getInfo(
            static_cast<cl_device_info>(CL_DEVICE_NAME), &name);
    std::cout << "Using device:   " << name << std::endl;

    sampler.init(cv_init()).move(cv_move(), true).monitor("pos", 2,
            vsmc::MonitorEvalAdapter<cv, vsmc::MonitorEvalCL>("cv_est"));
    sampler.monitor("pos").name(0) = "pos.x";
    sampler.monitor("pos").name(1) = "pos.y";

    PF_CV_DO(Multinomial);
    PF_CV_DO(Residual);
    PF_CV_DO(Stratified);
    PF_CV_DO(Systematic);
    PF_CV_DO(ResidualStratified);
    PF_CV_DO(ResidualSystematic);

    return 0;
}

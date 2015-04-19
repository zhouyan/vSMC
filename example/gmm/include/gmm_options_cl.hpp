//============================================================================
// vSMC/example/gmm/include/gmm_options_cl.hpp
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

std::string vSMCIncPath;
std::string R123IncPath;
std::vector<std::string> PlatformName;
std::vector<std::string> DeviceType;
std::vector<std::string> DeviceVendorName;
std::vector<std::string> BuildOption;
std::vector<std::size_t> LocalSize;
std::size_t FPTypeBits;
std::vector<std::size_t> ParticleNum;
double Threshold;
std::size_t IterNum;
std::string DataFile;
std::size_t DataNum;
std::size_t SM;
std::size_t CM;
Config.add("particle_num", "Particle number", &ParticleNum, 1000)
    .add("iter_num", "Iteration number", &IterNum, 100)
    .add("prior2", "Alias to iter_num", &IterNum, 100)
    .add("data_num", "Number of data", &DataNum, 100)
    .add("data_file", "Name of data file", &DataFile, "gmm.data")
    .add("simple_model", "Enable simple model with components number", &SM, 4)
    .add("complex_model",
         "Enable complex model with components number",
         &CM,
         5)
    .add("vsmc_inc_path", "vSMC include path", &vSMCIncPath, ".")
    .add("r123_inc_path", "Random123 include path", &R123IncPath, ".")
    .add("cl_local_size", "Local size of mcmc moves", &LocalSize, 0)
    .add("cl_platform_name",
         "Platform name",
         &PlatformName,
         "vSMCOpenCLDefault")
    .add("cl_device_type", "Device type", &DeviceType, "vSMCOpenCLDefault")
    .add("cl_device_vendor",
         "Device vendor",
         &DeviceVendorName,
         "vSMCOpenCLDefault")
    .add("cl_fp_type_bits",
         "Bits of OpenCL fp type (32: float; 64: double)",
         &FPTypeBits,
         32)
    .add("cl_build_option",
         "Additional build options for OpenCL",
         &BuildOption,
         "")
    .add("threshold",
         "Threshold for resampling, only used by SMC",
         &Threshold,
         0.5);

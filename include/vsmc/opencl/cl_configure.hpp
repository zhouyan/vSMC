//============================================================================
// vSMC/include/vsmc/opencl/cl_configure.hpp
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

#ifndef VSMC_OPENCL_CL_CONFIGURE_HPP
#define VSMC_OPENCL_CL_CONFIGURE_HPP

#include <vsmc/opencl/internal/common.hpp>
#include <vsmc/opencl/cl_manip.hpp>

namespace vsmc
{

/// \brief Configure OpenCL runtime behavior (used by MoveCL etc)
/// \ingroup OpenCL
class CLConfigure
{
    public:
    CLConfigure() : local_size_(0) {}

    std::size_t local_size() const { return local_size_; }

    void local_size(std::size_t new_size) { local_size_ = new_size; }

    void local_size(std::size_t N, const CLKernel &kern, const CLDevice &dev)
    {
        std::size_t global_size;
        cl_preferred_work_size(N, kern, dev, global_size, local_size_);
    }

    private:
    std::size_t local_size_;
}; // class CLConfigure

} // namespace vsmc

#endif // VSMC_OPENCL_CL_CONFIGURE_HPP

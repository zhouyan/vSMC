//============================================================================
// vSMC/lib/src/core/monitor.cpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#include "libvsmc.hpp"

extern "C" {

void vsmc_monitor_malloc(vsmc_monitor *monitor_ptr, int dim,
    vsmc_monitor_eval_type eval, int record_only, vSMCMonitorStage stage)
{
    auto cpp_eval = [eval](std::size_t iter, std::size_t d,
        ::vsmc::ParticleC &cpp_particle, double *res) {
        vsmc_particle particle = {&cpp_particle};
        eval(static_cast<int>(iter), static_cast<int>(d), particle, res);
    };
    auto ptr = ::vsmc::AlignedAllocator<::vsmc::MonitorC>::allocate(1);
    new (ptr)::vsmc::MonitorC(static_cast<std::size_t>(dim), cpp_eval,
        record_only != 0, static_cast<::vsmc::MonitorStage>(stage));
    monitor_ptr->ptr = ptr;
}

void vsmc_monitor_free(vsmc_monitor *monitor_ptr)
{
    ::vsmc::AlignedAllocator<::vsmc::MonitorC>::deallocate(
        ::vsmc::cast(monitor_ptr), 1);
    monitor_ptr->ptr = nullptr;
}

void vsmc_monitor_assign(vsmc_monitor dst, vsmc_monitor src)
{
    ::vsmc::cast(dst) = ::vsmc::cast(src);
}

} // extern "C"

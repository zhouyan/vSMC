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

vsmc_monitor vsmc_monitor_new(int dim, vsmc_monitor_eval_type eval,
    int record_only, vSMCMonitorStage stage)
{
    auto ptr =
        new ::vsmc::MonitorC(static_cast<std::size_t>(dim), ::vsmc::cast(eval),
            record_only != 0, static_cast<::vsmc::MonitorStage>(stage));
    vsmc_monitor monitor = {ptr};

    return monitor;
}

void vsmc_monitor_delete(vsmc_monitor *monitor_ptr)
{
    delete ::vsmc::cast(monitor_ptr);
    monitor_ptr->ptr = nullptr;
}

void vsmc_monitor_assign(vsmc_monitor monitor, vsmc_monitor other)
{
    ::vsmc::cast(monitor) = ::vsmc::cast(other);
}

int vsmc_monitor_dim(vsmc_monitor monitor)
{
    return static_cast<int>(::vsmc::cast(monitor).dim());
}

int vsmc_monitor_record_only(vsmc_monitor monitor)
{
    return ::vsmc::cast(monitor).record_only();
}

vSMCMonitorStage vsmc_monitor_stage(vsmc_monitor monitor)
{
    return static_cast<vSMCMonitorStage>(::vsmc::cast(monitor).stage());
}

int vsmc_monitor_iter_size(vsmc_monitor monitor)
{
    return static_cast<int>(::vsmc::cast(monitor).iter_size());
}

void vsmc_monitor_reserve(vsmc_monitor monitor, int num)
{
    ::vsmc::cast(monitor).reserve(static_cast<std::size_t>(num));
}

int vsmc_monitor_empty(vsmc_monitor monitor)
{
    return ::vsmc::cast(monitor).empty();
}

void vsmc_monitor_set_name(vsmc_monitor monitor, int id, const char *name)
{
    ::vsmc::cast(monitor).name(static_cast<std::size_t>(id)) = name;
}

int vsmc_monitor_get_name(vsmc_monitor monitor, int id, char *name)
{
    std::string str(::vsmc::cast(monitor).name(static_cast<std::size_t>(id)));
    std::size_t size = (str.size() + 1) * sizeof(char);
    if (name != nullptr)
        std::memcpy(name, str.c_str(), size);

    return static_cast<int>(size);
}

int vsmc_monitor_index(vsmc_monitor monitor, int iter)
{
    return iter < 0 ? static_cast<int>(::vsmc::cast(monitor).index()) :
                      static_cast<int>(::vsmc::cast(monitor).index(
                          static_cast<std::size_t>(iter)));
}

int *vsmc_monitor_read_index(vsmc_monitor monitor, int *first)
{
    return ::vsmc::cast(monitor).read_index(first);
}

double vsmc_monitor_record(vsmc_monitor monitor, int id, int iter)
{
    return iter < 0 ?
        ::vsmc::cast(monitor).record(static_cast<std::size_t>(id)) :
        ::vsmc::cast(monitor).record(
            static_cast<std::size_t>(id), static_cast<std::size_t>(iter));
}

double *vsmc_monitor_read_record(vsmc_monitor monitor, int id, double *first)
{
    return ::vsmc::cast(monitor).read_record(
        static_cast<std::size_t>(id), first);
}

double *vsmc_monitor_read_record_matrix(
    vsmc_monitor monitor, vSMCMatrixLayout layout, double *first)
{
    return ::vsmc::cast(monitor).read_record_matrix(
        static_cast<::vsmc::MatrixLayout>(layout), first);
}

void vsmc_monitor_set_eval(
    vsmc_monitor monitor, vsmc_monitor_eval_type new_eval)
{
    ::vsmc::cast(monitor).set_eval(::vsmc::cast(new_eval));
}

void vsmc_monitor_eval(vsmc_monitor monitor, int iter, vsmc_particle particle,
    vSMCMonitorStage stage)
{
    ::vsmc::cast(monitor).eval(static_cast<std::size_t>(iter),
        ::vsmc::cast(particle), static_cast<::vsmc::MonitorStage>(stage));
}

void vsmc_monitor_clear(vsmc_monitor monitor)
{
    ::vsmc::cast(monitor).clear();
}

int vsmc_monitor_recording(vsmc_monitor monitor)
{
    return ::vsmc::cast(monitor).recording();
}

void vsmc_monitor_turn_on(vsmc_monitor monitor)
{
    ::vsmc::cast(monitor).turn_on();
}

void vsmc_monitor_turn_off(vsmc_monitor monitor)
{
    ::vsmc::cast(monitor).turn_off();
}

} // extern "C"

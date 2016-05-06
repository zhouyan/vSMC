//============================================================================
// vSMC/lib/src/utility/stop_watch.cpp
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

#include <vsmc/utility/stop_watch.hpp>
#include <vsmc/utility/utility.h>

vsmc_stop_watch vsmc_stop_watch_new(void)
{
    auto ptr = new ::vsmc::StopWatch();
    vsmc_stop_watch stop_watch = {ptr};

    return stop_watch;
}

void vsmc_stop_watch_delete(vsmc_stop_watch *stop_watch_ptr)
{
    delete reinterpret_cast<::vsmc::StopWatch *>(stop_watch_ptr->ptr);
    stop_watch_ptr->ptr = nullptr;
}

void vsmc_stop_watch_assign(vsmc_stop_watch stop_watch, vsmc_stop_watch other)
{
    *reinterpret_cast<::vsmc::StopWatch *>(stop_watch.ptr) =
        *reinterpret_cast<::vsmc::StopWatch *>(other.ptr);
}

int vsmc_stop_watch_running(vsmc_stop_watch stop_watch)
{
    return reinterpret_cast<::vsmc::StopWatch *>(stop_watch.ptr)->running();
}

int vsmc_stop_watch_start(vsmc_stop_watch stop_watch)
{
    return reinterpret_cast<::vsmc::StopWatch *>(stop_watch.ptr)->start();
}

int vsmc_stop_watch_stop(vsmc_stop_watch stop_watch)
{
    return reinterpret_cast<::vsmc::StopWatch *>(stop_watch.ptr)->stop();
}

void vsmc_stop_watch_reset(vsmc_stop_watch stop_watch)
{
    reinterpret_cast<::vsmc::StopWatch *>(stop_watch.ptr)->reset();
}

double vsmc_stop_watch_cycles(vsmc_stop_watch stop_watch)
{
    return reinterpret_cast<::vsmc::StopWatch *>(stop_watch.ptr)->cycles();
}

double vsmc_stop_watch_nanoseconds(vsmc_stop_watch stop_watch)
{
    return reinterpret_cast<::vsmc::StopWatch *>(stop_watch.ptr)
        ->nanoseconds();
}

double vsmc_stop_watch_microseconds(vsmc_stop_watch stop_watch)
{
    return reinterpret_cast<::vsmc::StopWatch *>(stop_watch.ptr)
        ->microseconds();
}

double vsmc_stop_watch_milliseconds(vsmc_stop_watch stop_watch)
{
    return reinterpret_cast<::vsmc::StopWatch *>(stop_watch.ptr)
        ->milliseconds();
}

double vsmc_stop_watch_seconds(vsmc_stop_watch stop_watch)
{
    return reinterpret_cast<::vsmc::StopWatch *>(stop_watch.ptr)->seconds();
}

double vsmc_stop_watch_minutes(vsmc_stop_watch stop_watch)
{
    return reinterpret_cast<::vsmc::StopWatch *>(stop_watch.ptr)->minutes();
}

double vsmc_stop_watch_hours(vsmc_stop_watch stop_watch)
{
    return reinterpret_cast<::vsmc::StopWatch *>(stop_watch.ptr)->hours();
}

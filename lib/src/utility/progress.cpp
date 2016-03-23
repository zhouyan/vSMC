//============================================================================
// vSMC/lib/src/utility/progress.cpp
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

void vsmc_progress_malloc(vsmc_progress *progress_ptr)
{
    auto ptr = ::vsmc::Allocator<::vsmc::Progress>::allocate(1);
    new (ptr)::vsmc::Progress(std::cout);
    progress_ptr->ptr = ptr;
}

void vsmc_progress_free(vsmc_progress *progress_ptr)
{
    ::vsmc::Allocator<::vsmc::Progress>::deallocate(
        ::vsmc::cast(progress_ptr), 1);
    progress_ptr->ptr = nullptr;
}

void vsmc_progress_start(vsmc_progress progress, int total, const char *msg,
    int length, int show_iter, double interval_s)
{
    ::vsmc::cast(progress).start(static_cast<std::size_t>(total),
        msg == nullptr ? std::string() : msg, static_cast<std::size_t>(length),
        show_iter != 0, interval_s);
}

void vsmc_progress_stop(vsmc_progress progress, int finished)
{
    ::vsmc::cast(progress).stop(finished != 0);
}

void vsmc_progress_increment(vsmc_progress progress, int step)
{
    ::vsmc::cast(progress).increment(static_cast<std::size_t>(step));
}

void vsmc_progress_message(vsmc_progress progress, const char *msg)
{
    ::vsmc::cast(progress).message(msg == nullptr ? std::string() : msg);
}

} // extern "C"

//============================================================================
// vSMC/lib/src/utility/program_option.cpp
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

vsmc_program_option_map vsmc_program_option_map_new(int silent)
{
    auto ptr = new ::vsmc::ProgramOptionMap(silent != 0);
    vsmc_program_option_map program_option_map = {ptr};

    return program_option_map;
}

void vsmc_program_option_map_delete(
    vsmc_program_option_map *program_option_map_ptr)
{
    delete ::vsmc::cast(program_option_map_ptr);
    program_option_map_ptr->ptr = nullptr;
}

void vsmc_program_option_map_assign(
    vsmc_program_option_map program_option_map, vsmc_program_option_map other)
{
    ::vsmc::cast(program_option_map) = ::vsmc::cast(other);
}

void vsmc_program_option_map_add(vsmc_program_option_map program_option_map,
    const char *name, const char *desc, double *ptr)
{
    ::vsmc::cast(program_option_map).add(name, desc, ptr);
}

void vsmc_program_option_map_add_val(
    vsmc_program_option_map program_option_map, const char *name,
    const char *desc, double *ptr, double val)
{
    ::vsmc::cast(program_option_map).add(name, desc, ptr, val);
}

void vsmc_program_option_map_add_int(
    vsmc_program_option_map program_option_map, const char *name,
    const char *desc, int *ptr)
{
    ::vsmc::cast(program_option_map).add(name, desc, ptr);
}

void vsmc_program_option_map_add_val_int(
    vsmc_program_option_map program_option_map, const char *name,
    const char *desc, int *ptr, int val)
{
    ::vsmc::cast(program_option_map).add(name, desc, ptr, val);
}

void vsmc_program_option_map_remove(
    vsmc_program_option_map program_option_map, const char *name)
{
    ::vsmc::cast(program_option_map).remove(name);
}

void vsmc_program_option_map_process(
    vsmc_program_option_map program_option_map, int argc, char *const *argv)
{
    ::vsmc::cast(program_option_map).process(argc, argv, std::cout);
}

void vsmc_program_option_map_print_help(
    vsmc_program_option_map program_option_map)
{
    ::vsmc::cast(program_option_map).print_help(std::cout);
}

void vsmc_program_option_map_count(
    vsmc_program_option_map program_option_map, const char *name)
{
    ::vsmc::cast(program_option_map).count(name);
}

int vsmc_program_option_map_help(vsmc_program_option_map program_option_map)
{
    return ::vsmc::cast(program_option_map).help();
}

void vsmc_program_option_map_silent(
    vsmc_program_option_map program_option_map, int flag)
{
    ::vsmc::cast(program_option_map).silent(flag != 0);
}

} // extern "C"

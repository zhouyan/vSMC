//============================================================================
// vSMC/lib/src/utility/hdf5io.cpp
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

int vsmc_hdf5size(const char *file_name, const char *data_name)
{
    return static_cast<int>(::vsmc::hdf5size(file_name, data_name));
}

double *vsmc_hdf5load(
    const char *file_name, const char *data_name, double *first)
{
    return ::vsmc::hdf5load<double>(file_name, data_name, first);
}

int *vsmc_hdf5load_int(
    const char *file_name, const char *data_name, int *first)
{
    return ::vsmc::hdf5load<int>(file_name, data_name, first);
}

void vsmc_hdf5store_new(const char *file_name)
{
    ::vsmc::hdf5store_new(file_name);
}

void vsmc_hdf5store_matrix(vSMCMatrixLayout layout, int nrow, int ncol,
    const char *file_name, const char *data_name, const double *first,
    int append)
{
    ::vsmc::hdf5store_matrix<double>(static_cast<::vsmc::MatrixLayout>(layout),
        static_cast<std::size_t>(nrow), static_cast<std::size_t>(ncol),
        file_name, data_name, first, append != 0);
}

void vsmc_hdf5store_matrix_int(vSMCMatrixLayout layout, int nrow, int ncol,
    const char *file_name, const char *data_name, const int *first, int append)
{
    ::vsmc::hdf5store_matrix<int>(static_cast<::vsmc::MatrixLayout>(layout),
        static_cast<std::size_t>(nrow), static_cast<std::size_t>(ncol),
        file_name, data_name, first, append != 0);
}

void vsmc_hdf5store_list_empty(
    const char *file_name, const char *data_name, int append)
{
    ::vsmc::hdf5store_list_empty(file_name, data_name, append != 0);
}

void vsmc_hdf5store_list_insert(int N, const char *file_name,
    const char *data_name, const double *first, const char *vname)
{
    ::vsmc::hdf5store_list_insert<double>(
        static_cast<std::size_t>(N), file_name, data_name, first, vname);
}

void vsmc_hdf5store_list_insert_int(int N, const char *file_name,
    const char *data_name, const int *first, const char *vname)
{
    ::vsmc::hdf5store_list_insert<int>(
        static_cast<std::size_t>(N), file_name, data_name, first, vname);
}

void vsmc_hdf5store_state_matrix(vsmc_state_matrix state_matrix,
    const char *file_name, const char *data_name, int append)
{
    ::vsmc::hdf5store(
        ::vsmc::cast(state_matrix), file_name, data_name, append != 0);
}

void vsmc_hdf5store_particle(vsmc_particle particle, const char *file_name,
    const char *data_name, int append)
{
    ::vsmc::hdf5store(
        ::vsmc::cast(particle), file_name, data_name, append != 0);
}

void vsmc_hdf5store_monitor(vsmc_monitor monitor, const char *file_name,
    const char *data_name, int append)
{
    ::vsmc::hdf5store(
        ::vsmc::cast(monitor), file_name, data_name, append != 0);
}

void vsmc_hdf5store_sampler(vsmc_sampler sampler, const char *file_name,
    const char *data_name, int append)
{
    ::vsmc::hdf5store(
        ::vsmc::cast(sampler), file_name, data_name, append != 0);
}

} // extern "C"

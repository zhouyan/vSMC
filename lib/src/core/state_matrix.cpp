//============================================================================
// vSMC/lib/src/core/state_matrix.cpp
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

void vsmc_state_matrix_malloc(
    vsmc_state_matrix *state_matrix_ptr, int n, int dim)
{
    auto ptr = ::vsmc::AlignedAllocator<::vsmc::StateMatrixC>::allocate(1);
    new (ptr)::vsmc::StateMatrixC(n);
    ptr->resize_dim(static_cast<std::size_t>(dim));
    state_matrix_ptr->ptr = ptr;
}

void vsmc_state_matrix_free(vsmc_state_matrix *state_matrix_ptr)
{
    ::vsmc::AlignedAllocator<::vsmc::StateMatrixC>::deallocate(
        &::vsmc::cast(state_matrix_ptr), 1);
    state_matrix_ptr->ptr = nullptr;
}

void vsm_state_matrix_assign(
    vsmc_state_matrix *dst, const vsmc_state_matrix *src)
{
    ::vsmc::cast(dst) = ::vsmc::cast(src);
}

int vsmc_state_matrix_dim(const vsmc_state_matrix *state_matrix_ptr)
{
    return static_cast<int>(::vsmc::cast(state_matrix_ptr).dim());
}

void vsmc_state_matrix_resize_dim(vsmc_state_matrix *state_matrix_ptr, int n)
{
    ::vsmc::cast(state_matrix_ptr).resize_dim(static_cast<std::size_t>(n));
}

int vsmc_state_matrix_size(const vsmc_state_matrix *state_matrix_ptr)
{
    return static_cast<int>(::vsmc::cast(state_matrix_ptr).size());
}

double vsmc_state_matrix_get(
    const vsmc_state_matrix *state_matrix_ptr, int id, int pos)
{
    return ::vsmc::cast(state_matrix_ptr)
        .state(static_cast<std::size_t>(id), static_cast<std::size_t>(pos));
}

void vsmc_state_matrix_set(
    vsmc_state_matrix *state_matrix_ptr, int id, int pos, double s)
{
    ::vsmc::cast(state_matrix_ptr)
        .state(static_cast<std::size_t>(id), static_cast<std::size_t>(pos)) =
        s;
}

double *vsmc_state_matrix_data(vsmc_state_matrix *state_matrix_ptr)
{
    return ::vsmc::cast(state_matrix_ptr).data();
}

double *vsmc_state_matrix_row_data(vsmc_state_matrix *state_matrix_ptr, int id)
{
    return ::vsmc::cast(state_matrix_ptr)
        .row_data(static_cast<std::size_t>(id));
}

void vsmc_state_matrix_copy(
    vsmc_state_matrix *state_matrix_ptr, int N, const int *index)
{
    ::vsmc::cast(state_matrix_ptr).copy(static_cast<std::size_t>(N), index);
}

void vsmc_state_matrix_copy_particle(
    vsmc_state_matrix *state_matrix_ptr, int src, int dst)
{
    ::vsmc::cast(state_matrix_ptr)
        .copy_particle(
            static_cast<std::size_t>(src), static_cast<std::size_t>(dst));
}

} // extern "C"

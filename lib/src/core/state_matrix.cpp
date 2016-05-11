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

#include <vsmc/core/core.h>
#include "libvsmc.hpp"

extern "C" {

vsmc_state_matrix vsmc_state_matrix_new(size_t n, size_t dim)
{
    return {new ::vsmc::StateMatrixC(n, dim)};
}

void vsmc_state_matrix_delete(vsmc_state_matrix *state_matrix_ptr)
{
    delete ::vsmc::cast(state_matrix_ptr);
    state_matrix_ptr->ptr = nullptr;
}

void vsm_state_matrix_assign(
    vsmc_state_matrix state_matrix, vsmc_state_matrix other)
{
    ::vsmc::cast(state_matrix) = ::vsmc::cast(other);
}

size_t vsmc_state_matrix_size(vsmc_state_matrix state_matrix)
{
    return ::vsmc::cast(state_matrix).size();
}

size_t vsmc_state_matrix_dim(vsmc_state_matrix state_matrix)
{
    return ::vsmc::cast(state_matrix).dim();
}

void vsmc_state_matrix_resize(
    vsmc_state_matrix state_matrix, size_t n, size_t dim)
{
    ::vsmc::cast(state_matrix).resize(n, dim);
}

void vsmc_state_matrix_reserve(
    vsmc_state_matrix state_matrix, size_t n, size_t dim)
{
    ::vsmc::cast(state_matrix).reserve(n, dim);
}

void vsmc_state_matrix_shrink_to_fit(vsmc_state_matrix state_matrix)
{
    ::vsmc::cast(state_matrix).shrink_to_fit();
}

double vsmc_state_matrix_get(
    vsmc_state_matrix state_matrix, size_t id, size_t pos)
{
    return ::vsmc::cast(state_matrix)(id, pos);
}

void vsmc_state_matrix_set(
    vsmc_state_matrix state_matrix, size_t id, size_t pos, double state)
{
    ::vsmc::cast(state_matrix)(id, pos) = state;
}

double *vsmc_state_matrix_data(vsmc_state_matrix state_matrix)
{
    return ::vsmc::cast(state_matrix).data();
}

double *vsmc_state_matrix_row_data(vsmc_state_matrix state_matrix, size_t id)
{
    return ::vsmc::cast(state_matrix).row_data(id);
}

void vsmc_state_matrix_read_state(
    vsmc_state_matrix state_matrix, size_t pos, double *first)
{
    ::vsmc::cast(state_matrix).read_state(pos, first);
}

void vsmc_state_matrix_read_state_matrix(
    vsmc_state_matrix state_matrix, vSMCMatrixLayout layout, double *first)
{
    ::vsmc::cast(state_matrix)
        .read_state_matrix(static_cast<::vsmc::MatrixLayout>(layout), first);
}

void vsmc_state_matrix_select(
    vsmc_state_matrix state_matrix, size_t n, const size_t *index)
{
    ::vsmc::cast(state_matrix).select(n, index);
}

void vsmc_state_matrix_duplicate(
    vsmc_state_matrix state_matrix, size_t src, size_t dst)
{
    ::vsmc::cast(state_matrix).duplicate(src, dst);
}

} // extern "C"

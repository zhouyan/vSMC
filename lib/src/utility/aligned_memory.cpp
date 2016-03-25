//============================================================================
// vSMC/lib/src/utility/aligned_memory.cpp
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

void *vsmc_malloc(size_t n, int alignment)
{
    return ::vsmc::AlignedMemory::aligned_malloc(
        n, static_cast<std::size_t>(alignment));
}

void vsmc_free(void *ptr) { ::vsmc::AlignedMemory::aligned_free(ptr); }

vsmc_vector vsmc_vector_new(int size)
{
    vsmc_vector vector = {
        static_cast<double *>(
            vsmc_malloc(static_cast<std::size_t>(size) * sizeof(double), 32)),
        size};

    return vector;
}

void vsmc_vector_delete(vsmc_vector *vector_ptr)
{
    vsmc_free(vector_ptr->data);
    vector_ptr->data = nullptr;
    vector_ptr->size = 0;
}

void vsmc_vector_resize(vsmc_vector *vector_ptr, int size)
{
    if (vector_ptr->size != size) {
        vsmc_vector_delete(vector_ptr);
        *vector_ptr = vsmc_vector_new(size);
    }
}

vsmc_vector_int vsmc_vector_int_new(int size)
{
    vsmc_vector_int vector_int = {
        static_cast<int *>(
            vsmc_malloc(static_cast<std::size_t>(size) * sizeof(int), 32)),
        size};

    return vector_int;
}

void vsmc_vector_int_delete(vsmc_vector_int *vector_int_ptr)
{
    vsmc_free(vector_int_ptr->data);
    vector_int_ptr->data = nullptr;
    vector_int_ptr->size = 0;
}

void vsmc_vector_int_resize(vsmc_vector_int *vector_int_ptr, int size)
{
    if (vector_int_ptr->size != size) {
        vsmc_vector_int_delete(vector_int_ptr);
        *vector_int_ptr = vsmc_vector_int_new(size);
    }
}

vsmc_vector_raw vsmc_vector_raw_new(int size)
{
    vsmc_vector_raw vector_raw = {
        static_cast<unsigned char *>(vsmc_malloc(
            static_cast<std::size_t>(size) * sizeof(unsigned char), 32)),
        size};

    return vector_raw;
}

void vsmc_vector_raw_delete(vsmc_vector_raw *vector_raw_ptr)
{
    vsmc_free(vector_raw_ptr->data);
    vector_raw_ptr->data = nullptr;
    vector_raw_ptr->size = 0;
}

void vsmc_vector_raw_resize(vsmc_vector_raw *vector_raw_ptr, int size)
{
    if (vector_raw_ptr->size != size) {
        vsmc_vector_raw_delete(vector_raw_ptr);
        *vector_raw_ptr = vsmc_vector_raw_new(size);
    }
}

} // extern "C"

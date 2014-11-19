//============================================================================
// vSMC/example/rng/include/rng_output_data_hdf5.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

#ifndef VSMC_EXAMPLE_RNG_OUTPUT_DATA_HDF5_HPP
#define VSMC_EXAMPLE_RNG_OUTPUT_DATA_HDF5_HPP

#include <vsmc/utility/hdf5io.hpp>
#include <fstream>

template <typename T>
inline void rng_output_data (const std::string &base_name,
        const std::vector<std::string> &vnames,
        const std::vector<std::vector<T> > &values)
{
    if (vnames.size() != values.size())
        return;

    if (vnames.size() == 0)
        return;

    if (values[0].size() == 0)
        return;

    std::size_t M = vnames.size();
    std::size_t N = values[0].size();
    std::vector<const T *> vptr(M);
    for (std::size_t j = 0; j != M; ++j)
        vptr[j] = &values[j][0];
    vsmc::hdf5store_data_frame<T>(N, M, (base_name + ".hdf5"), base_name,
            vptr.begin(), vnames.begin());
}

#endif // VSMC_EXAMPLE_RNG_OUTPUT_DATA_HDF5_HPP

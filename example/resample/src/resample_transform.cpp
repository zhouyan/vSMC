//============================================================================
// vSMC/example/resample/src/resample_transform.cpp
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

#include "resample_transform.hpp"

int main(int argc, char **argv)
{
    std::size_t N = 1000;
    if (argc > 1)
        N = static_cast<std::size_t>(std::atoi(argv[1]));

    std::size_t n = 1000;
    if (argc > 2)
        n = static_cast<std::size_t>(std::atoi(argv[2]));

    resample_trans_rep_index_test<vsmc::Multinomial>(
        N, n, "Multinomial", true);
    resample_trans_rep_index_test<vsmc::Multinomial>(
        N, n, "Multinomial", false);

    resample_trans_rep_index_test<vsmc::Stratified>(N, n, "Stratified", true);
    resample_trans_rep_index_test<vsmc::Stratified>(N, n, "Stratified", false);

    resample_trans_rep_index_test<vsmc::Systematic>(N, n, "Systematic", true);
    resample_trans_rep_index_test<vsmc::Systematic>(N, n, "Systematic", false);

    resample_trans_rep_index_test<vsmc::Residual>(N, n, "Residual", true);
    resample_trans_rep_index_test<vsmc::Residual>(N, n, "Residual", false);

    resample_trans_rep_index_test<vsmc::ResidualStratified>(
        N, n, "ResidualStratified", true);
    resample_trans_rep_index_test<vsmc::ResidualStratified>(
        N, n, "ResidualStratified", false);

    resample_trans_rep_index_test<vsmc::ResidualSystematic>(
        N, n, "ResidualSystematic", true);
    resample_trans_rep_index_test<vsmc::ResidualSystematic>(
        N, n, "ResidualSystematic", false);

    return 0;
}

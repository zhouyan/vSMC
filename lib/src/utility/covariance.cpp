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

int vsmc_cov_chol(int dim, const double *cov, double *chol,
    vSMCMatrixLayout layout, int upper, int packed)
{
    return ::vsmc::cov_chol(static_cast<std::size_t>(dim), cov, chol,
        static_cast<::vsmc::MatrixLayout>(layout), upper != 0, packed != 0);
}

vsmc_covariance vsmc_covariance_new(void)
{
    auto ptr = new ::vsmc::Covariance<double>();
    vsmc_covariance covariance = {ptr};

    return covariance;
}

void vsmc_covariance_delete(vsmc_covariance *covariance_ptr)
{
    delete ::vsmc::cast(covariance_ptr);
    covariance_ptr->ptr = nullptr;
}

void vsmc_covariance_assign(vsmc_covariance covariance, vsmc_covariance other)
{
    ::vsmc::cast(covariance) = ::vsmc::cast(other);
}

void vsmc_covariance_compute(vsmc_covariance covariance,
    vSMCMatrixLayout layout, int n, int dim, const double *x, const double *w,
    double *mean, double *cov, vSMCMatrixLayout cov_layout, int cov_upper,
    int cov_packed)
{
    ::vsmc::cast(covariance)(static_cast<::vsmc::MatrixLayout>(layout),
        static_cast<std::size_t>(n), static_cast<std::size_t>(dim), x, w, mean,
        cov, static_cast<::vsmc::MatrixLayout>(cov_layout), cov_upper != 0,
        cov_packed != 0);
}

} // extern "C"

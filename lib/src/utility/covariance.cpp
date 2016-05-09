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

#include <vsmc/utility/covariance.hpp>
#include <vsmc/utility/utility.h>

extern "C" {

vsmc_covariance vsmc_covariance_new(void)
{
    return {new ::vsmc::Covariance<double>()};
}

void vsmc_covariance_delete(vsmc_covariance *covariance_ptr)
{
    delete reinterpret_cast<::vsmc::Covariance<double> *>(covariance_ptr->ptr);
    covariance_ptr->ptr = nullptr;
}

void vsmc_covariance_assign(vsmc_covariance covariance, vsmc_covariance other)
{
    *reinterpret_cast<::vsmc::Covariance<double> *>(covariance.ptr) =
        *reinterpret_cast<::vsmc::Covariance<double> *>(other.ptr);
}

void vsmc_covariance_compute(vsmc_covariance covariance,
    vSMCMatrixLayout layout, size_t n, size_t p, const double *x,
    const double *w, double *mean, double *cov, vSMCMatrixLayout cov_layout,
    int cov_upper, int cov_packed)
{
    (*reinterpret_cast<::vsmc::Covariance<double> *>(covariance.ptr))(
        static_cast<::vsmc::MatrixLayout>(layout), n, p, x, w, mean, cov,
        static_cast<::vsmc::MatrixLayout>(cov_layout), cov_upper != 0,
        cov_packed != 0);
}

} // extern "C"

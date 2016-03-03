//============================================================================
// vSMC/include/vsmc/utility/covariance.hpp
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

#ifndef VSMC_UTILITY_COVARIANCE_HPP
#define VSMC_UTILITY_COVARIANCE_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc
{

/// \brief Covariance
/// \ingroup Covaraince
template <typename RealType = double>
class Covaraince
{
    public:
    void operator()(MatrixLayout layout, std::size_t N, std::size_t dim,
        const result_type *x, const result_type *w, result_type *mean,
        result_type *cov, MatrixLayout cov_layout = RowMajor,
        bool cov_upper = false, bool cov_packed = false)
    {
#if VSMC_USE_MKL_VSL
        if (N * dim == 0)
            return;

        if (x == nullptr)
            return;

        MKL_INT p = static_cast<MKL_INT>(dim);
        MKL_INT n = static_cast<MKL_INT>(N);
        MKL_INT xstorage = layout == RowMajor ? VSL_SS_MATRIX_STORAGE_COLS :
                                                VSL_SS_MATRIX_STORAGE_ROWS;
        MKL_INT cov_storage = storage(cov_layout, cov_upper, cov_packed);
        unsigned MKL_INT64 estimates = 0;
        if (mean != nullptr)
            estimates |= VSL_SS_MEAN;
        if (cov != nullptr)
            estimates |= VSL_SS_COV;

        MKLSSTask<result_type> task(&p, &n, &xstorage, x, w, nullptr);
        task.edit_cov_cor(mean, cov, &cov_storage, nullptr, nullptr);
        task.compute(estimates, VSL_SS_METHOD_FAST);
#else

#endif
    }

    private:
#if VSMC_USE_MKL_VSL
    MKL_INT storage(MatrixLayout layout, bool upper, bool packed)
    {
        if (!packed)
            return VSL_SS_MATRIX_STORAGE_FULL;

        if (layout == RowMajor)
            return upper ? VSL_SS_MATRIX_STORAGE_U_PACKED :
                           VSL_SS_MATRIX_STORAGE_L_PACKED;

        return upper ? VSL_SS_MATRIX_STORAGE_L_PACKED :
                       VSL_SS_MATRIX_STORAGE_U_PACKED;
    }
#else
    Vector<RealType> mean_;
    Vector<RealType> covariance_;
    Vector<RealType> buffer_;
#endif
}; // class Covariance

} // namespace vsmc

#endif // VSMC_UTILITY_COVARIANCE_HPP

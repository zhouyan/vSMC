//============================================================================
// vSMC/include/vsmc/integrate/is_integrate.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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

#ifndef VSMC_INTEGRATE_ISINTEGRATE_HPP
#define VSMC_INTEGRATE_ISINTEGRATE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/math/cblas.hpp>

namespace vsmc
{

/// \brief Compute the importance sampling integration of multivariate
/// variable
/// \ingroup Integrate
class ISIntegrate
{
    public:
    typedef std::size_t size_type;

    /// \brief Compute the importance sampling integration
    ///
    /// \param N Number of particles
    /// \param dim Number of variables
    /// \param hX An `N` by `dim` row major matrix, each row `i` contains
    /// \f$h(X_i) = (h_1(X_i),\dots,h_d(X_i))\f$
    /// \param W Normalized weights, an `N`-vector
    /// \param Eh The importance sampling estiamtes of \f$E[h(X)] =
    /// [h(X)]'W\f$
    void operator()(size_type N, size_type dim, const double *hX,
        const double *W, double *Eh) const
    {
        if (N == 0 || dim == 0)
            return;

        if (dim == 1) {
            *Eh = math::dot(N, hX, W);
            return;
        }

#ifdef VSMC_CBLAS_INT
        ::cblas_dgemv(::CblasColMajor, ::CblasNoTrans,
            static_cast<VSMC_CBLAS_INT>(dim), static_cast<VSMC_CBLAS_INT>(N),
            1, hX, static_cast<VSMC_CBLAS_INT>(dim), W, 1, 0, Eh, 1);
#else
        for (size_type d = 0; d != dim; ++d)
            Eh[d] = 0;
        for (size_type i = 0; i != N; ++i, ++W) {
            for (size_type d = 0; d != dim; ++d, ++hX)
                Eh[d] += (*W) * (*hX);
        }
#endif
    }
}; // class ISIntegrate

} // namespace vsmc

#endif // VSMC_INTEGRATE_ISINTEGRATE_HPP

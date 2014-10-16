//============================================================================
// include/vsmc/integrate/is_integrate.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_INTEGRATE_ISINTEGRATE_HPP
#define VSMC_INTEGRATE_ISINTEGRATE_HPP

#include <vsmc/internal/common.hpp>

#if VSMC_USE_MKL_CBLAS
#include <mkl_cblas.h>
#elif VSMC_USE_VECLIB_CBLAS
#include <vecLib/cblas.h>
#endif

namespace vsmc {

/// \brief Compute the importance sampling integration of multivariate variable
/// \ingroup Integrate
class ISIntegrate
{
    public :

    typedef std::size_t size_type;

    /// \brief Compute the importance sampling integration
    ///
    /// \param N Number of particles
    /// \param dim Number of variables
    /// \param hX An `N` by `dim` row major matrix, each row `i` contains
    /// \f$h(X_i) = (h_1(X_i),\dots,h_d(X_i))\f$
    /// \param W Normalized weights, an `N`-vector
    /// \param Eh The importance sampling estiamtes of \f$E[h(X)] = [h(X)]'W\f$
    void operator() (size_type N, size_type dim,
            const double *hX, const double *W, double *Eh) const
    {
        if (N == 0 || dim == 0)
            return;

        if (dim == 1) {
            double sum = 0;
            for (size_type i = 0; i != N; ++i)
                sum += hX[i] * W[i];
            *Eh = sum;
            return;
        }

#if VSMC_USE_MKL_CBLAS
        ::cblas_dgemv(::CblasColMajor, ::CblasNoTrans,
                static_cast<MKL_INT>(dim), static_cast<MKL_INT>(N),
                1, hX, static_cast<MKL_INT>(dim), W, 1, 0, Eh, 1);
#elif VSMC_USE_VECLIB_CBLAS
        ::cblas_dgemv(::CblasColMajor, ::CblasNoTrans,
                static_cast<int>(dim), static_cast<int>(N),
                1, hX, static_cast<int>(dim), W, 1, 0, Eh, 1);
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

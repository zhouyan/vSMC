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
    /// \param hX A `N` by `dim` row major matrix, each row `i` contains
    /// \f$h(X_i) = (h_1(X_i), h_2(X_i), \dots, h_d(X_i))\f$
    /// \param W Normalized weights
    /// \param Eh The importance sampling estiamtes of \f$E[h(X)]\f$
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

        for (size_type d = 0; d != dim; ++d)
            Eh[d] = 0;
        for (size_type i = 0; i != N; ++i, ++W) {
            for (size_type d = 0; d != dim; ++d, ++hX)
                Eh[d] += (*W) * (*hX);
        }
    }
}; // class ISIntegrate

} // namespace vsmc

#endif // VSMC_INTEGRATE_ISINTEGRATE_HPP

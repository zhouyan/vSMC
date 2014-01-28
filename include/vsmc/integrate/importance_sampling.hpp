#ifndef VSMC_INTEGRATE_IMPORTANCE_SAMPLING_HPP
#define VSMC_INTEGRATE_IMPORTANCE_SAMPLING_HPP

#include <vsmc/internal/common.hpp>

#undef VSMC_USE_CBLAS
#undef VSMC_INTEGRATE_INT

#if VSMC_USE_MKL // MKL
#include <mkl_cblas.h>
#define VSMC_USE_CBLAS 1
#define VSMC_INTEGRATE_INT MKL_INT
#elif VSMC_USE_VECLIB // vecLib
#include <vecLib/cblas.h>
#define VSMC_USE_CBLAS 1
#define VSMC_INTEGRATE_INT int
#elif VSMC_USE_GENERIC_CBLAS // Generic CBlas
#define VSMC_USE_CBLAS 1
#ifdef VSMC_GENERIC_CBLAS_INT
#define VSMC_INTEGRATE_INT VSMC_GENERIC_CBLAS_INT
#else
#define VSMC_INTEGRATE_INT int
#endif
#else // No known CBlas
#define VSMC_INTEGRATE_INT std::size_t
#endif

namespace vsmc {

/// \brief Compute the importance sampling integration of multivariate variable
/// \ingroup Integrate
class ISIntegrate
{
    public :

    typedef VSMC_INTEGRATE_INT size_type;

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
        if (N == 0)
            return;
#if VSMC_USE_CBLAS
        cblas_dgemv(CblasColMajor, CblasNoTrans,
                dim, N, 1, hX, dim, W, 1, 0, Eh, 1);
#else
        for (size_type d = 0; d != dim; ++d)
            Eh[d] = 0;
        for (size_type i = 0; i != N; ++i) {
            double w = W[i];
            for (size_type d = 0; d != dim; ++d)
                Eh[d] += w * hX[i * dim + d];
        }
#endif
    }

    private :

    static bool is_align (void *ptr) {return ((unsigned long) ptr & 15) == 0;}
}; // class ISIntegrate

} // namespace vsmc

#endif // VSMC_INTEGRATE_IMPORTANCE_SAMPLING_HPP

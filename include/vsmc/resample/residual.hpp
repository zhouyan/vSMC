//============================================================================
// vsmc/resample/residual.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_RESAMPLE_RESIDUAL_HPP
#define VSMC_RESAMPLE_RESIDUAL_HPP

#include <vsmc/resample/common.hpp>

namespace vsmc {

namespace internal {

typedef cxx11::integral_constant<ResampleScheme, Residual>
    ResampleResidual;

} // namespace vsmc::internal

/// \brief Residual resampling
/// \ingroup Resample
template <>
class Resample<internal::ResampleResidual>
{
    public :

    template <typename IntType, typename RngType>
    void operator() (std::size_t M, std::size_t N, RngType &rng,
            const double *weight, IntType *replication)
    {
        using std::modf;

        residual_.resize(M);
        integral_.resize(M);
        double *const rptr = &residual_[0];
        double *const iptr = &integral_[0];
        for (std::size_t i = 0; i != M; ++i)
            rptr[i] = modf(N * weight[i], iptr + i);
        double rsum = 0;
        for (std::size_t i = 0; i != M; ++i)
            rsum += rptr[i];
        for (std::size_t i = 0; i != M; ++i)
            rptr[i] /= rsum;

        IntType R = 0;
        for (std::size_t i = 0; i != M; ++i)
            R += static_cast<IntType>(iptr[i]);
        std::size_t NN = N - static_cast<std::size_t>(R);
        u01_.resize(NN);
        double *const uptr = &u01_[0];
        cxx11::uniform_real_distribution<double> runif(0, 1);
        for (std::size_t i = 0; i != NN; ++i)
            uptr[i] = runif(rng);
        inversion_(M, NN, rptr, uptr, replication);

        for (std::size_t i = 0; i != N; ++i)
            replication[i] += static_cast<IntType>(iptr[i]);
    }

    private :

    internal::Inversion inversion_;
    std::vector<double> residual_;
    std::vector<double> integral_;
    std::vector<double> u01_;
}; // Residual resampling

} // namespace vsmc

#endif // VSMC_RESAMPLE_RESIDUAL_HPP

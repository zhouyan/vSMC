//============================================================================
// include/vsmc/resample/residual_systematic.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distributed under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_RESAMPLE_RESIDUAL_SYSTEMATIC_HPP
#define VSMC_RESAMPLE_RESIDUAL_SYSTEMATIC_HPP

#include <vsmc/resample/common.hpp>

namespace vsmc {

namespace internal {

typedef cxx11::integral_constant<ResampleScheme, ResidualSystematic>
    ResampleResidualSystematic;

} // namespace vsmc::internal

/// \brief Residual systematic resampling
/// \ingroup Resample
template <>
class Resample<internal::ResampleResidualSystematic>
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
        const double delta = 1.0 / N;
        const double u = runif(rng) * delta;
        for (std::size_t i = 0; i != NN; ++i)
            uptr[i] = u + i * delta;
        inversion_(M, NN, rptr, uptr, replication, true);

        for (std::size_t i = 0; i != N; ++i)
            replication[i] += static_cast<IntType>(iptr[i]);
    }

    private :

    internal::Inversion inversion_;
    std::vector<double, AlignedAllocator<double> > residual_;
    std::vector<double, AlignedAllocator<double> > integral_;
    std::vector<double, AlignedAllocator<double> > u01_;
}; // Residual systematic resampling

} // namespace vsmc

#endif // VSMC_RESAMPLE_RESIDUAL_SYSTEMATIC_HPP

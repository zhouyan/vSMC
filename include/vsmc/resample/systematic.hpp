//============================================================================
// include/vsmc/resample/systematic.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_RESAMPLE_SYSTEMATIC_HPP
#define VSMC_RESAMPLE_SYSTEMATIC_HPP

#include <vsmc/resample/common.hpp>

namespace vsmc {

namespace internal {

typedef cxx11::integral_constant<ResampleScheme, Systematic>
    ResampleSystematic;

} // namespace vsmc::internal

/// \brief Systematic resampling
/// \ingroup Resample
template <>
class Resample<internal::ResampleSystematic>
{
    public :

    template <typename IntType, typename RngType>
    void operator() (std::size_t M, std::size_t N, RngType &rng,
            const double *weight, IntType *replication)
    {
        u01_.resize(N);
        double *const uptr = &u01_[0];
        cxx11::uniform_real_distribution<double> runif(0, 1);
        const double delta = 1.0 / N;
        const double u = runif(rng) * delta;
        for (std::size_t i = 0; i != N; ++i)
            uptr[i] = u + i * delta;
        inversion_(M, N, weight, uptr, replication, true);
    }

    private :

    internal::Inversion inversion_;
    std::vector<double> u01_;
}; // Systematic resampling

} // namespace vsmc

#endif // VSMC_RESAMPLE_SYSTEMATIC_HPP

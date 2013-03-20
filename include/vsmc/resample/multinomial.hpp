#ifndef VSMC_RESAMPLE_MULTINOMIAL_HPP
#define VSMC_RESAMPLE_MULTINOMIAL_HPP

#include <vsmc/resample/common.hpp>

namespace vsmc {

namespace internal {

typedef cxx11::integral_constant<ResampleScheme, Multinomial>
    ResampleMultinomial;

} // namespace vsmc::internal

/// \brief Multinomial resampling
/// \ingroup Resample
template <>
class Resample<internal::ResampleMultinomial>
{
    public :

    template <typename IntType, typename RngType>
    void operator() (std::size_t N, RngType &rng, const double *weight,
            IntType *replication)
    {
        internal::multinomial(N, static_cast<IntType>(N), rng,
                weight, replication);
        internal::normalize_replication(N, replication);
    }
}; // Mulitnomial resampling

} // namespace vsmc

#endif //  VSMC_RESAMPLE_MULTINOMIAL_HPP

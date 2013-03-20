#ifndef VSMC_RESAMPLE_STRATIFIED_HPP
#define VSMC_RESAMPLE_STRATIFIED_HPP

#include <vsmc/resample/common.hpp>

namespace vsmc {

namespace internal {

typedef cxx11::integral_constant<ResampleScheme, Stratified>
    ResampleStratified;

} // namespace vsmc::internal

/// \brief Stratified resampling
/// \ingroup Resample
template <>
class Resample<internal::ResampleStratified>
{
    public :

    template <typename IntType, typename RngType>
    void operator() (std::size_t N, RngType &rng, const double *weight,
            IntType *replication)
    {
        for (std::size_t i = 0; i != N; ++i)
            replication[i] = 0;

        std::size_t j = 0;
        std::size_t k = 0;
        cxx11::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng);
        double cw = weight[0];
        while (j != N) {
            while (j < cw * N - u && j != N) {
                ++replication[k];
                u = unif(rng);
                ++j;
            }
            if (k == N - 1)
                break;
            cw += weight[++k];
        }
        internal::normalize_replication(N, replication);
    }
}; // Stratified resampling

} // namespace vsmc

#endif // VSMC_RESAMPLE_STRATIFIED_HPP

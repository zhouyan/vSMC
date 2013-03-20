#ifndef VSMC_RESAMPLE_SYSTEMATIC_HPP
#define VSMC_RESAMPLE_SYSTEMATIC_HPP

#include <vsmc/resample/common.hpp>

namespace vsmc {

namespace internal {

typedef cxx11::integral_constant<ResampleScheme, Systematic>
    ResampleSystematic;

} // namespace vsmc::internal

/// \brief Systematic resampling
/// \ingroup Core
template <>
class Resample<internal::ResampleSystematic>
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
                ++j;
            }
            if (k == N - 1)
                break;
            cw += weight[++k];
        }
        internal::normalize_replication(N, replication);
    }
}; // Systematic resampling

} // namespace vsmc

#endif // VSMC_RESAMPLE_SYSTEMATIC_HPP

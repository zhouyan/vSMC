#ifndef VSMC_RESAMPLE_RESIDUAL_STRATIFIED_HPP
#define VSMC_RESAMPLE_RESIDUAL_STRATIFIED_HPP

#include <vsmc/resample/common.hpp>

namespace vsmc {

namespace internal {

typedef cxx11::integral_constant<ResampleScheme, ResidualStratified>
    ResampleResidualStratified;

} // namespace vsmc::internal

/// \brief Residual stratified resampling
/// \ingroup Resample
template <>
class Resample<internal::ResampleResidualStratified>
{
    public :

    template <typename IntType, typename RngType>
    void operator() (std::size_t N, RngType &rng, const double *weight,
            IntType *replication)
    {
        using std::modf;

        residual_.resize(N);
        integral_.resize(N);
        double *const rptr = &residual_[0];
        double *const iptr = &integral_[0];
        for (std::size_t i = 0; i != N; ++i) {
            replication[i] = 0;
            rptr[i] = modf(N * weight[i], iptr + i);
        }
        double dsize = 0;
        for (std::size_t i = 0; i != N; ++i)
            dsize += rptr[i];
        for (std::size_t i = 0; i != N; ++i)
            rptr[i] /= dsize;
        std::size_t j = 0;
        std::size_t k = 0;
        std::size_t size = static_cast<std::size_t>(dsize);
        cxx11::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng);
        double cw = rptr[0];
        while (j != size) {
            while (j < cw * size - u && j != size) {
                ++replication[k];
                u = unif(rng);
                ++j;
            }
            if (k == N - 1)
                break;
            cw += rptr[++k];
        }
        for (std::size_t i = 0; i != N; ++i)
            replication[i] += static_cast<IntType>(iptr[i]);
        internal::normalize_replication(N, replication);
    }

    private :

    std::vector<double> residual_;
    std::vector<double> integral_;
}; // Residual stratified resampling

} // namespace vsmc

#endif // VSMC_RESAMPLE_RESIDUAL_STRATIFIED_HPP

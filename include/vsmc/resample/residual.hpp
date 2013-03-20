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
    void operator() (std::size_t N, RngType &rng, const double *weight,
            IntType *replication)
    {
        using std::modf;

        residual_.resize(N);
        integral_.resize(N);
        double *const rptr = &residual_[0];
        double *const iptr = &integral_[0];
        for (std::size_t i = 0; i != N; ++i)
            rptr[i] = modf(N * weight[i], iptr + i);
        double dsize = 0;
        for (std::size_t i = 0; i != N; ++i)
            dsize += rptr[i];
        for (std::size_t i = 0; i != N; ++i)
            rptr[i] /= dsize;
        IntType size = static_cast<IntType>(dsize);
        internal::multinomial(N, size, rng, rptr, replication);
        for (std::size_t i = 0; i != N; ++i)
            replication[i] += static_cast<IntType>(iptr[i]);
        internal::normalize_replication(N, replication);
    }

    private :

    std::vector<double> residual_;
    std::vector<double> integral_;
}; // Residual resampling

} // namespace vsmc

#endif // VSMC_RESAMPLE_RESIDUAL_HPP

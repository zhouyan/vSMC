#ifndef VSMC_CORE_RESAMPLE_HPP
#define VSMC_CORE_RESAMPLE_HPP

#include <vsmc/internal/common.hpp>

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(ResampleRngType, resample_rng_type,
        vsmc::cxx11::mt19937)

namespace vsmc {

namespace internal {

template <typename SizeType, typename RngType>
inline void multinomial (SizeType N, SizeType S, RngType &rng,
        const double *weight, SizeType *replication)
{
    double sum_w = 0;
    double acc_w = 0;
    SizeType acc_s = 0;

    for (SizeType i = 0; i != N; ++i) {
        replication[i] = 0;
        sum_w += weight[i];
    }

    for (SizeType i = 0; i != N; ++i) {
        if (acc_s < S && weight[i] > 0) {
            double p = weight[i] / (sum_w - acc_w);
            p = p < 0 ? 0 : p;
            p = p > 1 ? 1 : p;
            long s = static_cast<long>(S - acc_s);
            cxx11::binomial_distribution<long> binom(s, p);
            replication[i] = static_cast<SizeType>(binom(rng));
        }
        acc_w += weight[i];
        acc_s += replication[i];
    }
}

template <typename SizeType>
inline void normalize_replication (SizeType N, SizeType *replication)
{
    SizeType sum = 0;
    SizeType max_i = 0;
    SizeType max_v = replication[0];
    for (SizeType i = 0; i != N; ++i) {
        SizeType r = replication[i];
        if (r > max_v) {
            max_i = i;
            max_v = r;
        }
        sum += r;
    }
    replication[max_i] += N - sum;
}

} // namespace vsmc::internal

/// \brief Transform replication numbers to parent particle locations
/// \ingroup Core
///
/// \details This one shall be used in place of the default if resampling
/// algorithms output parent locations directly instead of replication number
class ResampleCopyFromReplicationNoAaction
{
    public :

    template <typename IntType1, typename IntType2>
    void operator() (std::size_t N, const IntType1 *, IntType2 *) const {}
}; // class ResampleCopyFromReplicationNoAaction

/// \brief Transform replication numbers to parent particle locations
/// \ingroup Core
class ResampleCopyFromReplication
{
    public :

    template <typename IntType1, typename IntType2>
    void operator() (std::size_t N,
            const IntType1 *replication, IntType2 *copy_from) const
    {
        std::size_t from = 0;
        std::size_t time = 0;
        for (std::size_t to = 0; to != N; ++to) {
            if (replication[to]) {
                copy_from[to] = to;
            } else {
                // replication[to] has zero child, copy from elsewhere
                if (replication[from] - time <= 1) {
                    // only 1 child left on replication[from]
                    time = 0;
                    do // move from to some position with at least 2 children
                        ++from;
                    while (replication[from] < 2);
                }
                copy_from[to] = from;
                ++time;
            }
        }
    }
}; // class ResampleCopyFromReplication

class ResamplePostCopy
{
    public :

    template <typename WeightSetType>
    void operator() (WeightSetType &weight_set) const
    {weight_set.set_equal_weight();}
}; // class ResamplePostCopy

/// \brief Multinomial resampling
/// \ingroup Core
template <>
class Resample<cxx11::integral_constant<ResampleScheme, Multinomial> >
{
    public :

    template <typename SizeType, typename RngType>
    void operator() (SizeType N, RngType &rng, const double *weight,
            SizeType *replication)
    {
        internal::multinomial(N, N, rng, weight, replication);
        internal::normalize_replication(N, replication);
    }
}; // Mulitnomial resampling

/// \brief Residual resampling
/// \ingroup Core
template <>
class Resample<cxx11::integral_constant<ResampleScheme, Residual> >
{
    public :

    template <typename SizeType, typename RngType>
    void operator() (SizeType N, RngType &rng, const double *weight,
            SizeType *replication)
    {
        using std::modf;

        residual_.resize(N);
        integral_.resize(N);
        double *const rptr = &residual_[0];
        double *const iptr = &integral_[0];
        for (SizeType i = 0; i != N; ++i)
            rptr[i] = modf(N * weight[i], iptr + i);
        double dsize = 0;
        for (SizeType i = 0; i != N; ++i)
            dsize += rptr[i];
        SizeType size = static_cast<SizeType>(dsize);
        for (SizeType i = 0; i != N; ++i)
            rptr[i] /= dsize;
        internal::multinomial(N, size, rng, rptr, replication);
        for (SizeType i = 0; i != N; ++i)
            replication[i] += static_cast<SizeType>(iptr[i]);
        internal::normalize_replication(N, replication);
    }

    private :

    std::vector<double> residual_;
    std::vector<double> integral_;
}; // Residual resampling

/// \brief Stratified resampling
/// \ingroup Core
template <>
class Resample<cxx11::integral_constant<ResampleScheme, Stratified> >
{
    public :

    template <typename SizeType, typename RngType>
    void operator() (SizeType N, RngType &rng, const double *weight,
            SizeType *replication)
    {
        for (SizeType i = 0; i != N; ++i)
            replication[i] = 0;

        SizeType j = 0;
        SizeType k = 0;
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

/// \brief Systematic resampling
/// \ingroup Core
template <>
class Resample<cxx11::integral_constant<ResampleScheme, Systematic> >
{
    public :

    template <typename SizeType, typename RngType>
    void operator() (SizeType N, RngType &rng, const double *weight,
            SizeType *replication)
    {
        for (SizeType i = 0; i != N; ++i)
            replication[i] = 0;

        SizeType j = 0;
        SizeType k = 0;
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

/// \brief Residual stratified resampling
/// \ingroup Core
template <>
class Resample<cxx11::integral_constant<ResampleScheme, ResidualStratified> >
{
    public :

    template <typename SizeType, typename RngType>
    void operator() (SizeType N, RngType &rng, const double *weight,
            SizeType *replication)
    {
        using std::modf;

        residual_.resize(N);
        integral_.resize(N);
        double *const rptr = &residual_[0];
        double *const iptr = &integral_[0];
        for (SizeType i = 0; i != N; ++i) {
            replication[i] = 0;
            rptr[i] = modf(N * weight[i], iptr + i);
        }
        double dsize = 0;
        for (SizeType i = 0; i != N; ++i)
            dsize += rptr[i];
        SizeType size = static_cast<SizeType>(dsize);
        for (SizeType i = 0; i != N; ++i)
            rptr[i] /= dsize;
        SizeType j = 0;
        SizeType k = 0;
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
        for (SizeType i = 0; i != N; ++i)
            replication[i] += static_cast<SizeType>(iptr[i]);
        internal::normalize_replication(N, replication);
    }

    private :

    std::vector<double> residual_;
    std::vector<double> integral_;
}; // Residual stratified resampling

/// \brief Residual systematic resampling
/// \ingroup Core
template <>
class Resample<cxx11::integral_constant<ResampleScheme, ResidualSystematic> >
{
    public :

    template <typename SizeType, typename RngType>
    void operator() (SizeType N, RngType &rng, const double *weight,
            SizeType *replication)
    {
        using std::modf;

        residual_.resize(N);
        integral_.resize(N);
        double *const rptr = &residual_[0];
        double *const iptr = &integral_[0];
        for (SizeType i = 0; i != N; ++i) {
            replication[i] = 0;
            rptr[i] = modf(N * weight[i], iptr + i);
        }
        double dsize = 0;
        for (SizeType i = 0; i != N; ++i)
            dsize += rptr[i];
        SizeType size = static_cast<SizeType>(dsize);
        for (SizeType i = 0; i != N; ++i)
            rptr[i] /= dsize;
        SizeType j = 0;
        SizeType k = 0;
        cxx11::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng);
        double cw = rptr[0];
        while (j != size) {
            while (j < cw * size - u && j != size) {
                ++replication[k];
                ++j;
            }
            if (k == N - 1)
                break;
            cw += rptr[++k];
        }
        for (SizeType i = 0; i != N; ++i)
            replication[i] += static_cast<SizeType>(iptr[i]);
        internal::normalize_replication(N, replication);
    }

    private :

    std::vector<double> residual_;
    std::vector<double> integral_;
}; // Residual systematic resampling

} // namespace vsmc

#endif // VSMC_CORE_RESAMPLE_HPP

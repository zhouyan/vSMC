#ifndef VSMC_CORE_RESAMPLE_HPP
#define VSMC_CORE_RESAMPLE_HPP

#include <vsmc/internal/common.hpp>

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
        for (SizeType i = 0; i != N; ++i)
            residual_[i] = modf(N * weight[i], &integral_[i]);
        double dsize = 0;
        for (SizeType i = 0; i != N; ++i)
            dsize += residual_[i];
        SizeType size = static_cast<SizeType>(dsize);
        for (SizeType i = 0; i != N; ++i)
            residual_[i] /= dsize;
        internal::multinomial(N, size, rng, &residual_[0], replication);
        for (SizeType i = 0; i != N; ++i)
            replication[i] += static_cast<SizeType>(integral_[i]);
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
        for (SizeType i = 0; i != N; ++i) {
            replication[i] = 0;
            residual_[i] = modf(N * weight[i], &integral_[i]);
        }
        double dsize = 0;
        for (SizeType i = 0; i != N; ++i)
            dsize += residual_[i];
        SizeType size = static_cast<SizeType>(dsize);
        for (SizeType i = 0; i != N; ++i)
            residual_[i] /= dsize;
        SizeType j = 0;
        SizeType k = 0;
        cxx11::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng);
        double cw = residual_[0];
        while (j != size) {
            while (j < cw * size - u && j != size) {
                ++replication[k];
                u = unif(rng);
                ++j;
            }
            if (k == N - 1)
                break;
            cw += residual_[++k];
        }
        for (SizeType i = 0; i != N; ++i)
            replication[i] += static_cast<SizeType>(integral_[i]);
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
        for (SizeType i = 0; i != N; ++i) {
            replication[i] = 0;
            residual_[i] = modf(N * weight[i], &integral_[i]);
        }
        double dsize = 0;
        for (SizeType i = 0; i != N; ++i)
            dsize += residual_[i];
        SizeType size = static_cast<SizeType>(dsize);
        for (SizeType i = 0; i != N; ++i)
            residual_[i] /= dsize;
        SizeType j = 0;
        SizeType k = 0;
        cxx11::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng);
        double cw = residual_[0];
        while (j != size) {
            while (j < cw * size - u && j != size) {
                ++replication[k];
                ++j;
            }
            if (k == N - 1)
                break;
            cw += residual_[++k];
        }
        for (SizeType i = 0; i != N; ++i)
            replication[i] += static_cast<SizeType>(integral_[i]);
        internal::normalize_replication(N, replication);
    }

    private :

    std::vector<double> residual_;
    std::vector<double> integral_;
}; // Residual systematic resampling

} // namespace vsmc

#endif // VSMC_CORE_RESAMPLE_HPP

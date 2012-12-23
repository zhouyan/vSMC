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
                typedef typename cxx11::make_signed<SizeType>::type s_t;
                s_t s = S - acc_s;
                double p = weight[i] / (sum_w - acc_w);
                if (p < 0) {
                    p = 0;
                    assert(p > -1e-6);
                }
                if (p > 1) {
                    p = 1;
                    assert(p - 1 < 1e-6);
                }
                cxx11::binomial_distribution<s_t> binom(s, p);
                replication[i] = binom(rng);
            }
            acc_w += weight[i];
            acc_s += replication[i];
        }
}

template <typename SizeType>
inline void normalize_replication (SizeType N, SizeType *replication)
{
    SizeType sum = std::accumulate(replication, replication + N,
            static_cast<SizeType>(0));
    if (sum != N) {
        SizeType *id_max = std::max_element(replication, replication + N);
        *id_max += N - sum;
    }
}

} // namespace vsmc::internal

/// \brief Resample scheme
/// \ingroup Resampling
enum ResampleScheme {
    Multinomial,         ///< Multinomial resampling
    Residual,            ///< Reisudal resampling
    Stratified,          ///< Startified resampling
    Systematic,          ///< Systematic resampling
    ResidualStratified , ///< Stratified resampling on the residuals
    ResidualSystematic   ///< Systematic resampling on the residuals
}; // enum ResamleScheme

/// \brief Int-to-Type struct template for identifying a resampling scheme
/// \ingroup Resampling
template <typename EnumType, EnumType S> struct ResampleType {};

/// \brief Resample class template
/// \ingroup Resampling
template <typename ResType>
class Resample {};

/// \brief Multinomial resampling
/// \ingroup Resampling
template <>
class Resample<ResampleType<ResampleScheme, Multinomial> >
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
/// \ingroup Resampling
template <>
class Resample<ResampleType<ResampleScheme, Residual> >
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
        double dsize = std::accumulate(residual_.begin(), residual_.end(),
                static_cast<double>(0));
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
/// \ingroup Resampling
template <>
class Resample<ResampleType<ResampleScheme, Stratified> >
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
/// \ingroup Resampling
template <>
class Resample<ResampleType<ResampleScheme, Systematic> >
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
/// \ingroup Resampling
template <>
class Resample<ResampleType<ResampleScheme, ResidualStratified> >
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

        double dsize = std::accumulate(residual_.begin(), residual_.end(),
                static_cast<double>(0));
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
/// \ingroup Resampling
template <>
class Resample<ResampleType<ResampleScheme, ResidualSystematic> >
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

        double dsize = std::accumulate(residual_.begin(), residual_.end(),
                static_cast<double>(0));
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

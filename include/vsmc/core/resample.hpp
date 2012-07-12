#ifndef VSMC_CORE_RESAMPLE_HPP
#define VSMC_CORE_RESAMPLE_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

namespace internal {

template <typename SizeType, typename RngSetType>
inline void weight2replication (SizeType N, SizeType S, RngSetType &rng_set,
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
                replication[i] = binom(rng_set.rng(i));
            }
            acc_w += weight[i];
            acc_s += replication[i];
        }
}

} // namespace vsmc::internal

/// \brief Resample scheme
/// \ingroup Core
enum ResampleScheme {
    MULTINOMIAL,         ///< Multinomial resampling
    RESIDUAL,            ///< Reisudal resampling
    STRATIFIED,          ///< Startified resampling
    SYSTEMATIC,          ///< Systematic resampling
    RESIDUAL_STRATIFIED, ///< Stratified resampling on the residuals
    RESIDUAL_SYSTEMATIC  ///< Systematic resampling on the residuals
}; // enum ResamleScheme

/// \brief Int-to-Type struct template for resampling scheme
/// \ingroup Core
template <typename EnumType, EnumType S> struct ResampleType {};

/// \brief Resample class template
/// \ingroup Core
template <typename ResType, typename SizeType, typename RngSetType>
class Resample {};

/// \brief Multinomial resampling
/// \ingroup Core
template <typename SizeType, typename RngSetType>
class Resample<ResampleType<ResampleScheme, MULTINOMIAL>,
      SizeType, RngSetType>
{
    public :

    void operator() (SizeType N, RngSetType &rng_set,
            const double *weight, SizeType *replication)
    {
        internal::weight2replication(N, N, rng_set, weight, replication);
    }
}; // Mulitnomial resampling

/// \brief Residual resampling
/// \ingroup Core
template <typename SizeType, typename RngSetType>
class Resample<ResampleType<ResampleScheme, RESIDUAL>,
      SizeType, RngSetType>
{
    public :

    void operator() (SizeType N, RngSetType &rng_set,
            const double *weight, SizeType *replication)
    {
        using std::modf;

        residual_.resize(N);
        integral_.resize(N);
        for (SizeType i = 0; i != N; ++i)
            residual_[i] = modf(N * weight[i], &integral_[i]);
        SizeType S = static_cast<SizeType>(residual_.sum());
        internal::weight2replication(N, S, rng_set, weight, replication);
        for (SizeType i = 0; i != N; ++i)
            replication[i] += static_cast<SizeType>(integral_[i]);
    }

    private :

    Eigen::VectorXd residual_;
    Eigen::VectorXd integral_;
}; // Residual resampling

/// \brief Stratified resampling
/// \ingroup Core
template <typename SizeType, typename RngSetType>
class Resample<ResampleType<ResampleScheme, STRATIFIED>,
      SizeType, RngSetType>
{
    public :

    void operator() (SizeType N, RngSetType &rng_set,
            const double *weight, SizeType *replication)
    {
        for (SizeType i = 0; i != N; ++i)
            replication[i] = 0;

        SizeType j = 0;
        SizeType k = 0;
        cxx11::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng_set.rng(0));
        double cw = weight[0];
        while (j != N) {
            while (j < cw * N - u && j != N) {
                ++replication[k];
                u = unif(rng_set.rng(j));
                ++j;
            }
            if (k == N - 1)
                break;
            cw += weight[++k];
        }
    }
}; // Stratified resampling

/// \brief Systematic resampling
/// \ingroup Core
template <typename SizeType, typename RngSetType>
class Resample<ResampleType<ResampleScheme, SYSTEMATIC>,
      SizeType, RngSetType>
{
    public :

    void operator() (SizeType N, RngSetType &rng_set,
            const double *weight, SizeType *replication)
    {
        for (SizeType i = 0; i != N; ++i)
            replication[i] = 0;

        SizeType j = 0;
        SizeType k = 0;
        cxx11::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng_set.rng(0));
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
    }
}; // Systematic resampling

/// \brief Residual stratified resampling
/// \ingroup Core
template <typename SizeType, typename RngSetType>
class Resample<ResampleType<ResampleScheme, RESIDUAL_STRATIFIED>,
      SizeType, RngSetType>
{
    public :

    void operator() (SizeType N, RngSetType &rng_set,
            const double *weight, SizeType *replication)
    {
        using std::modf;

        residual_.resize(N);
        integral_.resize(N);
        for (SizeType i = 0; i != N; ++i) {
            replication[i] = 0;
            residual_[i] = modf(N * weight[i], &integral_[i]);
        }

        double dsize = (residual_.sum());
        SizeType size = static_cast<SizeType>(dsize);
        residual_ /= dsize;
        SizeType j = 0;
        SizeType k = 0;
        cxx11::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng_set.rng(0));
        double cw = residual_[0];
        while (j != size) {
            while (j < cw * size - u && j != size) {
                ++replication[k];
                u = unif(rng_set.rng(j));
                ++j;
            }
            if (k == N - 1)
                break;
            cw += residual_[++k];
        }
        for (SizeType i = 0; i != N; ++i)
            replication[i] += static_cast<SizeType>(integral_[i]);
    }

    private :

    Eigen::VectorXd residual_;
    Eigen::VectorXd integral_;
}; // Residual stratified resampling

/// \brief Residual systematic resampling
/// \ingroup Core
template <typename SizeType, typename RngSetType>
class Resample<ResampleType<ResampleScheme, RESIDUAL_SYSTEMATIC>,
      SizeType, RngSetType>
{
    public :

    void operator() (SizeType N, RngSetType &rng_set,
            const double *weight, SizeType *replication)
    {
        using std::modf;

        residual_.resize(N);
        integral_.resize(N);
        for (SizeType i = 0; i != N; ++i) {
            replication[i] = 0;
            residual_[i] = modf(N * weight[i], &integral_[i]);
        }

        double dsize = (residual_.sum());
        SizeType size = static_cast<SizeType>(dsize);
        residual_ /= dsize;
        SizeType j = 0;
        SizeType k = 0;
        cxx11::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng_set.rng(0));
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
    }

    private :

    Eigen::VectorXd residual_;
    Eigen::VectorXd integral_;
}; // Residual systematic resampling

} // namespace vsmc

#endif // VSMC_CORE_RESAMPLE_HPP

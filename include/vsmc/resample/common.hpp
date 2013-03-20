#ifndef VSMC_RESAMPLE_COMMON_HPP
#define VSMC_RESAMPLE_COMMON_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

namespace traits {

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(ResampleRngType, resample_rng_type,
        vsmc::cxx11::mt19937)
} // namespace vsmc::traits

namespace internal {

// N: Number of weights
// S: Total number of mass
template <typename IntType, typename RngType>
inline void multinomial (std::size_t N, IntType S, RngType &rng,
        const double *weight, IntType *res)
{
    double sum_w = 0;
    for (std::size_t i = 0; i != N; ++i) {
        res[i] = 0;
        sum_w += weight[i];
    }

    double acc_w = 0;
    IntType acc_s = 0;
    for (std::size_t i = 0; i != N; ++i) {
        if (acc_s < S && weight[i] > 0) {
            double p = weight[i] / (sum_w - acc_w);
            p = p < 0 ? 0 : p;
            p = p > 1 ? 1 : p;
            long s = static_cast<long>(S - acc_s);
            cxx11::binomial_distribution<long> binom(s, p);
            res[i] = static_cast<IntType>(binom(rng));
        }
        acc_w += weight[i];
        acc_s += res[i];
    }
}

template <typename IntType>
inline void normalize_replication (std::size_t N, IntType *replication)
{
    IntType sum = 0;
    std::size_t max_i = 0;
    IntType max_v = replication[0];
    for (std::size_t i = 0; i != N; ++i) {
        const IntType r = replication[i];
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
/// \ingroup Resample
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
/// \ingroup Resample
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

} // namespace vsmc

#endif // VSMC_RESAMPLE_COMMON_HPP

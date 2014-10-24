//============================================================================
// include/vsmc/resample/common.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_RESAMPLE_COMMON_HPP
#define VSMC_RESAMPLE_COMMON_HPP

#include <cstring>
#include <algorithm>
#include <vector>

#include <vsmc/internal/common.hpp>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/rng/threefry.hpp>

/// \brief Default RNG type for resampling
/// \ingroup Config
#ifndef VSMC_RESAMPLE_RNG_TYPE
#define VSMC_RESAMPLE_RNG_TYPE ::vsmc::Threefry4x64
#endif

namespace vsmc {

namespace internal {

class Inversion
{
    public :

    // Given N U01 random variates,
    // Compute M replication numbers based on M weights
    template <typename IntType>
    void operator() (std::size_t M, std::size_t N,
            const double *weight, const double *u01, IntType *replication,
            bool usorted = false)
    {
        if (M == 0)
            return;

        if (M == 1) {
            *replication = static_cast<IntType>(N);
            return;
        }

        std::memset(replication, 0, sizeof(IntType) * M);

        if (N == 0)
            return;

        accw_.resize(M);
        accw_[0] = weight[0];
        for (std::size_t i = 1; i != M; ++i)
            accw_[i] = accw_[i - 1] + weight[i];
        accw_.back() = 1;

        u01_.resize(N);
        std::memcpy(&u01_[0], u01, sizeof(double) * N);
        if (!usorted)
            std::sort(u01_.begin(), u01_.end());
        if (u01_.back() > 1)
            u01_.back() = 1;

        std::size_t offset = 0;
        for (std::size_t i = 0; i != M; ++i) {
            while (offset != N && u01_[offset] <= accw_[i]) {
                ++replication[i];
                ++offset;
            }
        }
    }

    private :

    std::vector<double> accw_;
    std::vector<double> u01_;
}; // class Inversion

} // namespace vsmc::internal

/// \brief Resampling schemes
/// \ingroup Resample
enum ResampleScheme {
    Multinomial,        ///< Multinomial resampling
    Residual,           ///< Residual resampling
    Stratified,         ///< Stratified resampling
    Systematic,         ///< Systematic resampling
    ResidualStratified, ///< Stratified resampling on residuals
    ResidualSystematic  ///< Systematic resampling on residuals
}; // enum ResampleScheme

/// \brief Resample forward decleration
/// \ingroup Resample
template <typename> class Resample;

/// \brief Resampling type of the built-in schemes
/// \ingroup Resample
template <ResampleScheme Scheme>
struct ResampleType
{typedef Resample<cxx11::integral_constant<ResampleScheme, Scheme> > type;};

/// \brief Transform replication numbers to parent particle locations
/// \ingroup Resample
///
/// \details This one shall be used in place of the default if resampling
/// algorithms output parent locations directly instead of replication number
class ResampleCopyFromReplicationNoAaction
{
    public :

    template <typename IntType1, typename IntType2>
    void operator() (std::size_t, std::size_t,
            const IntType1 *, IntType2 *) const {}
}; // class ResampleCopyFromReplicationNoAaction

/// \brief Transform replication numbers to parent particle locations
/// \ingroup Resample
class ResampleCopyFromReplication
{
    public :

    template <typename IntType1, typename IntType2>
    void operator() (std::size_t M, std::size_t N,
            const IntType1 *replication, IntType2 *copy_from) const
    {
        if (M == N) {
            IntType1 time = 0;
            IntType2 from = 0;
            for (std::size_t to = 0; to != N; ++to) {
                if (replication[to] != 0) {
                    copy_from[to] = static_cast<IntType2>(to);
                } else {
                    // replication[to] has zero child, copy from elsewhere
                    if (replication[from] < time + 2) {
                        // only 1 child left on replication[from]
                        time = 0;
                        do // move from to a position with at least 2 children
                            ++from;
                        while (replication[from] < 2);
                    }
                    copy_from[to] = from;
                    ++time;
                }
            }
        } else {
            std::size_t to = 0;
            for (std::size_t from = 0; from != M; ++from)
                for (IntType1 i = 0; i != replication[from]; ++i)
                    copy_from[to++] = static_cast<IntType2>(from);
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

namespace traits {

/// \brief Particle::resample_rng_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(ResampleRngType, resample_rng_type,
        VSMC_RESAMPLE_RNG_TYPE)

/// \brief Particle::resample_copy_from_replication_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(ResampleCopyFromReplicationType,
        resample_copy_from_replication_type, ResampleCopyFromReplication)

/// \brief Particle::resample_post_copy_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(ResamplePostCopyType,
        resample_post_copy_type, ResamplePostCopy)

} // namespace vsmc::traits

} // namespace vsmc

#endif // VSMC_RESAMPLE_COMMON_HPP

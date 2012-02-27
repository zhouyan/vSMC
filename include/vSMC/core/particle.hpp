#ifndef V_SMC_CORE_PARTICLE_HPP
#define V_SMC_CORE_PARTICLE_HPP

#include <algorithm>
#include <limits>
#include <cstddef>
#include <mkl_cblas.h>
#include <boost/function.hpp>
#include <boost/math/special_functions/log1p.hpp>
#include <boost/random/binomial_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <Random123/aes.h>
#include <Random123/ars.h>
#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <vDist/tool/buffer.hpp>

/// The Parallel RNG (based on Rand123) seed, unsigned
#ifndef V_SMC_PRNG_SEED
#define V_SMC_PRNG_SEED 0xdeadbeefL
#endif // V_SMC_PRNG_SEED

/// The Parallel RNG (based on Rand123) type, philox or threefry
#ifndef V_SMC_PRNG_TYPE
#define V_SMC_PRNG_TYPE Threefry4x64
#endif // V_SMC_PRNG_TYPE

/// The type used to extract random bits
#ifndef V_SMC_PRNG_UINT_TYPE
#define V_SMC_PRNG_UINT_TYPE unsigned
#endif // V_SMC_PRNG_UINT_TYPE

#define V_SMC_PRNG_IDX_MAX sizeof(rng_type::ctr_type) / sizeof(rint_type)

namespace vSMC {

/// Resample scheme
enum ResampleScheme {MULTINOMIAL, RESIDUAL, STRATIFIED, SYSTEMATIC};

/// \brief Particle class
///
/// Particle class store the particle set and arrays of weights and log
/// weights. It provides access to particle values as well as weights. It
/// computes and manages resources for ESS, resampling, etc, tasks unique to
/// each iteration.
template <typename T>
class Particle
{
    public :

    /// \brief Type of the internal RNG (Random123 C++ type)
    typedef r123::V_SMC_PRNG_TYPE rng_type;
    /// \brief Type of the internal unsigned random integer
    typedef V_SMC_PRNG_UINT_TYPE rint_type;

    /// \brief Construct a Particle object with given number of particles
    ///
    /// \param N The number of particles
    Particle (std::size_t N) :
        size_(N), value_(N),
        weight_(N), log_weight_(N), inc_weight_(N), replication_(N),
        ess_(0), resampled_(false), zconst_(0), estimate_zconst_(false),
        binom_(size_, 0.5), brng_(V_SMC_PRNG_SEED),
        rbase_(std::numeric_limits<rint_type>::max()),
        ridx_max_(V_SMC_PRNG_IDX_MAX),
        ridx_(N), rbit_(N), rctr_(N), rkey_(N)
    {
        for (std::size_t i = 0; i != N; ++i) {
            rng_type::ctr_type c = {{}};
            rng_type::key_type k = {{}};
            c[0] = i;
            k[0] = V_SMC_PRNG_SEED + i;
            rctr_[i] = c;
            rkey_[i] = k;
            ridx_[i] = 0;
            rbit_[i].c = crng_(rctr_[i], rkey_[i]);
        }

        double equal_weight = 1.0 / size_;
        for (std::size_t i = 0; i != size_; ++i) {
            weight_[i] = equal_weight;
            log_weight_[i] = 0;
        }
    }

    /// \brief Size of the particle set
    ///
    /// \return The number of particles
    std::size_t size () const
    {
        return size_;
    }

    /// \brief Read and write access to particle values
    ///
    /// \return A reference to the particle values, type (T &)
    ///
    /// \note The Particle class guarantee that during the life type of the
    /// object, the reference returned by this member will no be a dangle
    /// handler.
    T &value ()
    {
        return value_;
    }

    /// \brief Read only access to particle values
    ///
    /// \return A const reference to the particle values
    const T &value () const
    {
        return value_;
    }

    /// \brief Read only access to the weights
    ///
    /// \return A const pointer to the weights
    ///
    /// \note The Particle class guarantee that during the life type of the
    /// object, the pointer returned by this always valid and point to the
    /// same address
    const double *weight_ptr () const
    {
        return weight_.get();
    }

    /// \brief Read only access to the log weights
    ///
    /// \return A const pointer to the log weights
    const double *log_weight_ptr () const
    {
        return log_weight_.get();
    }

    /// \brief Set the log weights
    ///
    /// \param [in] new_weight New log weights
    void set_log_weight (const double *new_weight)
    {
        cblas_dcopy(size_, new_weight, 1, log_weight_, 1);
        set_weight();
    }

    /// \brief Add to the log weights
    ///
    /// \param [in] inc_weight Incremental log weights
    void add_log_weight (const double *inc_weight)
    {
        if (estimate_zconst_) {
            for (std::size_t i = 0; i != size_; ++i)
                inc_weight_[i] = std::exp(inc_weight[i]);
            zconst_ += std::log(cblas_ddot(size_, weight_, 1, inc_weight_, 1));
        }

        for (std::size_t i = 0; i != size_; ++i)
            log_weight_[i] += inc_weight[i];
        set_weight();
    }

    /// \brief The ESS (Effective Sample Size)
    ///
    /// \return The value of ESS for current particle set
    double ess () const
    {
        return ess_;
    }

    /// \brief Get indicator of resampling
    ///
    /// \return A bool value, \b true if the current iteration was resampled
    bool resampled () const
    {
        return resampled_;
    }

    /// \brief Set indicator of resampling
    ///
    /// \param resampled \b true if the current iteration was resampled
    void resampled (bool resampled)
    {
        resampled_ = resampled;
    }

    /// \brief Get the value of SMC normalizing constant
    ///
    /// \return SMC normalizng constant estimate
    double zconst () const
    {
        return zconst_;
    }

    /// \brief Toggle whether or not record SMC normalizing constant
    ///
    /// \param estimate_zconst Start estimating normalzing constant if true
    void zconst (bool estimate_zconst)
    {
        estimate_zconst_ = estimate_zconst;
    }

    /// \brief Reset the value of SMC normalizing constant
    void reset_zconst ()
    {
        zconst_ = 0;
    }

    /// \brief Perform resampling
    ///
    /// \param scheme The resampling scheme, see ResamplingScheme
    void resample (ResampleScheme scheme)
    {
        switch (scheme) {
            case MULTINOMIAL :
                resample_multinomial();
                break;
            case RESIDUAL :
                resample_residual();
                break;
            case STRATIFIED :
                resample_stratified();
                break;
            case SYSTEMATIC :
                resample_systematic();
                break;
            default :
                throw std::runtime_error(
                        "ERROR: vSMC::Particle::resample: "
                        "Unknown Resample Scheme");
        }
    }

    /// \brief Generate an uniform random integer
    rint_type rint (std::size_t id)
    {
        if (ridx_[id] == ridx_max_) {
            ridx_[id] = 0;
            rctr_[id][0] += size_;
            rbit_[id].c = crng_(rctr_[id], rkey_[id]);
        }

        return rbit_[id].n[ridx_[id]++];
    }

    /// \brief Generate an [0,1] uniform random variate
    double runif (std::size_t id)
    {
        return rint(id) / rbase_;
    }

    double runif (std::size_t id, double min, double max)
    {
        return runif(id) * (max - min) + min;
    }

    double rnorm (std::size_t id, double mean, double sd)
    {
        static const double pi2 = 6.283185307179586476925286766559005768394;

        double u1 = runif(id);
        double u2 = runif(id);

        return std::sqrt(-2 * std::log(u1)) * std::sin(pi2 * u2) * sd + mean;
    }

    double rlnorm (std::size_t id, double meanlog, double sdlog)
    {
        return std::exp(rnorm(id, meanlog, sdlog));
    }

    double rcauchy (std::size_t id, double location, double scale)
    {
        static const double pi = 3.141592653589793238462643383279502884197;

        return scale * std::tan(pi * (runif(id) - 0.5)) + location;
    }

    double rexp (std::size_t id, double scale)
    {
        return -scale * boost::math::log1p(-runif(id));
    }

    double rlaplace (std::size_t id, double location, double scale)
    {
        double u = runif(id) - 0.5;

        return u > 0 ?
            location - scale * boost::math::log1p(-2 * u):
            location + scale * boost::math::log1p(2 * u);
    }

    double rweibull (std::size_t id, double shape, double scale)
    {
        return scale * std::pow(
                -boost::math::log1p(-runif(id)), 1 / shape);
    }

    // TODO GAMMA
    double rgamma (std::size_t id, double shape, double scale)
    {
        return 0;
    }

    double rchisq (std::size_t id, double df)
    {
        return rgamma(id, 0.5 * df, 2);
    }

    double rfdist (std::size_t id, double df1, double df2)
    {
        return rchisq(id, df1) / rchisq(id, df2) * df2 / df1;
    }

    double rtdist (std::size_t id, double df)
    {
        return rnorm(id, 0, 1) / std::sqrt(rchisq(id, df) / df);
    }

    // TODO BETA
    double rbeta (std::size_t id, double shape1, double shape2)
    {
        return 0;
    }

    private :

    std::size_t size_;
    T value_;

    vDist::tool::Buffer<double> weight_;
    vDist::tool::Buffer<double> log_weight_;
    vDist::tool::Buffer<double> inc_weight_;
    vDist::tool::Buffer<int> replication_;

    double ess_;
    bool resampled_;
    double zconst_;
    bool estimate_zconst_;

    typedef boost::random::binomial_distribution<int, double> binom_type;
    binom_type binom_;
    boost::random::mt19937 brng_;

    rng_type crng_;
    double rbase_;
    unsigned ridx_max_;
    union uni {rng_type::ctr_type c; rint_type n[V_SMC_PRNG_IDX_MAX];};

    vDist::tool::Buffer<unsigned> ridx_;
    vDist::tool::Buffer<uni> rbit_;
    vDist::tool::Buffer<rng_type::ctr_type> rctr_;
    vDist::tool::Buffer<rng_type::key_type> rkey_;

    void set_weight ()
    {
        double max_weight = -std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i != size_; ++i)
            max_weight = std::max(max_weight, log_weight_[i]);
        for (std::size_t i = 0; i != size_; ++i) {
            log_weight_[i] -= max_weight;
            weight_[i] = std::exp(log_weight_[i]);
        }
        double sum = cblas_dasum(size_, weight_, 1);
        cblas_dscal(size_, 1 / sum, weight_, 1);
        ess_ = 1 / cblas_ddot(size_, weight_, 1, weight_, 1);
    }

    void weight2replication ()
    {
        double tp = 0;
        for (std::size_t i = 0; i != size_; ++i)
            tp += weight_[i];

        double sum_p = 0;
        std::size_t sum_n = 0;
        for (std::size_t i = 0; i != size_; ++i) {
            replication_[i] = 0;
            if (sum_n < size_ && weight_[i] > 0) {
                binom_type::param_type
                    param(size_ - sum_n, weight_[i] / (tp - sum_p));
                replication_[i] = binom_(brng_, param);
            }
            sum_p += weight_[i];
            sum_n += replication_[i];
        }
    }

    void resample_multinomial ()
    {
        weight2replication();
        resample_do();
    }

    void resample_residual ()
    {
        // Reuse weight and log_weight. weight: act as the fractional part of
        // N * weight. log_weight: act as the integral part of N * weight.
        // They all will be reset to equal weights after resampling.  So it is
        // safe to modify them here.
        for (std::size_t i = 0; i != size_; ++i)
            weight_[i] = std::modf(size_ * weight_[i], log_weight_ + i);
        std::size_t size = size_;
        size -= cblas_dasum(size_, log_weight_, 1);
        weight2replication();
        for (std::size_t i = 0; i != size_; ++i)
            replication_[i] += log_weight_[i];
        resample_do();
    }

    void resample_stratified () {}
    void resample_systematic () {}

    void resample_do ()
    {
        std::size_t from = 0;
        std::size_t time = 0;

        for (std::size_t to = 0; to != size_; ++to) {
            if (!replication_[to]) {
                // replication_[to] has zero child, copy from elsewhere
                if (replication_[from] - time <= 1) {
                    // only 1 child left on replication_[from]
                    time = 0;
                    do // move from to some position with at least 2 children
                        ++from;
                    while (replication_[from] < 2);
                }
                value_.copy(from, to);
                ++time;
            }
        }

        ess_ = size_;
        double equal_weight = 1.0 / size_;
        for (std::size_t i = 0; i != size_; ++i) {
            weight_[i] = equal_weight;
            log_weight_[i] = 0;
        }
    }
}; // class Particle

} // namespace vSMC

#endif // V_SMC_CORE_PARTICLE_HPP

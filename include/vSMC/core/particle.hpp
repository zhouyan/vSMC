#ifndef V_SMC_CORE_PARTICLE_HPP
#define V_SMC_CORE_PARTICLE_HPP

#include <algorithm>
#include <limits>
#include <cstddef>
#include <mkl_cblas.h>
#include <mkl_vml.h>
#include <boost/function.hpp>
#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <vDist/rng/gsl.hpp>
#include <vDist/tool/buffer.hpp>
#include <vDist/tool/eblas.hpp>

/// The Parallel RNG (based on Rand123) seed
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

#define V_SMC_PRNG_IDX_MAX \
    (sizeof(rng_type::ctr_type) / sizeof(V_SMC_PRNG_UINT_TYPE))

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

    /// \brief Type of of the internal RNG, one of the 64 bit Rand123 type
    typedef r123::V_SMC_PRNG_TYPE rng_type;

    /// \brief Construct a Particle object with given number of particles
    ///
    /// \param N The number of particles
    Particle (std::size_t N) :
        size_(N), value_(N),
        weight_(N), log_weight_(N), inc_weight_(N), replication_(N),
        ess_(0), resampled_(false), zconst_(0), estimate_zconst_(false),
        rbase_(0), rbit_(N), ridx_(N), rctr_(N), rkey_(N)
    {
        int shift = sizeof(V_SMC_PRNG_UINT_TYPE) * 8 - 1;
        rbase_ = 0.5 / (1U<<shift);

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
            vdExp(size_, inc_weight, inc_weight_);
            zconst_ += std::log(cblas_ddot(size_, weight_, 1, inc_weight_, 1));
        }

        vdAdd(size_, log_weight_, inc_weight, log_weight_);
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
    /// \param [in] rng A gsl rng object pointer
    void resample (ResampleScheme scheme, const gsl_rng *rng)
    {
        switch (scheme) {
            case MULTINOMIAL :
                resample_multinomial(rng);
                break;
            case RESIDUAL :
                resample_residual(rng);
                break;
            case STRATIFIED :
                resample_stratified(rng);
                break;
            case SYSTEMATIC :
                resample_systematic(rng);
                break;
            default :
                throw std::runtime_error(
                        "ERROR: vSMC::Particle::resample: "
                        "Unknown Resample Scheme");
        }
    }

    /// \brief Generate an uniform random integer
    ///
    /// \param id Any integer, usually the id of the particle
    ///
    /// \return A random integer on the space of V_SMC_PRNG_UINT_TYPE
    V_SMC_PRNG_UINT_TYPE ruint (std::size_t id)
    {
        if (ridx_[id] == V_SMC_PRNG_IDX_MAX) {
            ridx_[id] = 0;
            rctr_[id][0] += size_;
            rbit_[id].c = crng_(rctr_[id], rkey_[id]);
        }

        return rbit_[id].n[ridx_[id]++];
    }

    /// \brief Generate an [0,1] uniform random variate
    ///
    /// \param id Any integer, usually the id of the particle
    ///
    /// \return A random variate uniform distributed on [0,1]
    double runif (std::size_t id)
    {
        return ruint(id) * rbase_;
    }

    double rbeta (std::size_t id, double a, double b) {return 0;}

    double rcauchy (std::size_t id, double location, double scale) {return 0;}

    double rchisq (std::size_t id, double a, double b) {return 0;}

    double rexp (std::size_t id, double scale) {return 0;}

    double rf (std::size_t id, double df1, double df2) {return 0;}

    double rgamma (std::size_t id, double shape, double scale) {return 0;}

    double rlaplace (std::size_t id, double location, double scale) {return 0;}

    double rlnorm (std::size_t id, double mealog, double sdlog) {return 0;}

    /// \brief Generate an Normal random variate
    ///
    /// \param id Any integer, usually the id of the particle
    /// \param mean The mean of the random variable
    /// \param sd The SD of the random variable
    ///
    /// \return A random variate Normally distributed as N(mean, sd^2) 
    double rnorm (std::size_t id, double mean, double sd)
    {
        double u1 = runif(id);
        double u2 = runif(id);

        return std::sqrt(-2 * log(u1)) * std::cos(2 * M_PI * u2) * sd + mean;
    }

    double rt (std::size_t id, double df) {}

    /// \brief Generate an uniform random variate
    ///
    /// \param id Any integer, usually the id of the particle
    /// \param min The minimum of the random variable
    /// \param max The maximum of the random variable
    ///
    /// \return A random variate uniform distributed on [min,max]
    double runif (std::size_t id, double min, double max)
    {
        return runif(id) * (max - min) + min;
    }

    double dweibull (std::size_t id, double df) {}

    private :

    std::size_t size_;
    T value_;
    vDist::tool::Buffer<double> weight_;
    vDist::tool::Buffer<double> log_weight_;
    vDist::tool::Buffer<double> inc_weight_;
    vDist::tool::Buffer<unsigned> replication_;
    double ess_;
    bool resampled_;
    double zconst_;
    bool estimate_zconst_;

    union uni {
        rng_type::ctr_type c; V_SMC_PRNG_UINT_TYPE n[V_SMC_PRNG_IDX_MAX];};
    double rbase_;
    rng_type crng_;
    vDist::tool::Buffer<uni> rbit_;
    vDist::tool::Buffer<int> ridx_;
    vDist::tool::Buffer<rng_type::ctr_type> rctr_;
    vDist::tool::Buffer<rng_type::key_type> rkey_;

    void set_weight ()
    {
        double max_weight = -std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i != size_; ++i)
            max_weight = std::max(max_weight, log_weight_[i]);
        for (std::size_t i = 0; i != size_; ++i)
            log_weight_[i] -= max_weight;
        vdExp(size_, log_weight_, weight_);
        double sum = cblas_dasum(size_, weight_, 1);
        cblas_dscal(size_, 1 / sum, weight_, 1);
        ess_ = 1 / cblas_ddot(size_, weight_, 1, weight_, 1);
    }

    void resample_multinomial (const gsl_rng *rng)
    {
        gsl_ran_multinomial(rng, size_, size_, weight_, replication_);
        resample_do();
    }

    void resample_residual (const gsl_rng *rng)
    {
        /// \internal Reuse weight and log_weight.
        /// weight: act as the fractional part of N * weight.
        /// log_weight: act as the integral part of N * weight.
        /// They all will be reset to equal weights after resampling.
        /// So it is safe to modify them here.
        vDist::tool::dyatx(size_, size_, weight_, 1, log_weight_, 1);
        vdModf(size_, log_weight_, log_weight_, weight_);
        std::size_t size = size_;
        size -= cblas_dasum(size_, log_weight_, 1);
        gsl_ran_multinomial(rng, size_, size, weight_, replication_);
        for (std::size_t i = 0; i != size_; ++i)
            replication_[i] += log_weight_[i];
        resample_do();
    }

    void resample_stratified (const gsl_rng *rng) {}
    void resample_systematic (const gsl_rng *rng) {}

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
        vDist::tool::dfill(size_, 1.0 / size_, weight_, 1);
        vDist::tool::dfill(size_, 0, log_weight_, 1);
        ess_ = size_;
    }
}; // class Particle

} // namespace vSMC

#endif // V_SMC_CORE_PARTICLE_HPP

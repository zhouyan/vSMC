#ifndef V_SMC_CORE_PARTICLE_HPP
#define V_SMC_CORE_PARTICLE_HPP

#include <cmath>
#include <cstddef>
#include <mkl_cblas.h>
#include <boost/function.hpp>
#include <boost/random/binomial_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <vSMC/core/buffer.hpp>
#include <vSMC/core/rng.hpp>

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

    typedef V_SMC_RNG_TYPE rng_type;

    /// \brief Construct a Particle object with given number of particles
    ///
    /// \param N The number of particles
    Particle (std::size_t N, rng_type::seed_type seed = V_SMC_RNG_SEED) :
        size_(N), value_(N),
        weight_(N), log_weight_(N), inc_weight_(N), replication_(N),
        ess_(0), resampled_(false), zconst_(0), prng_(N)
    {
        rng_type rng(seed);
        rng.step_size(size_);
        for (std::size_t i = 0; i != size_; ++i) {
            rng.advance_ctr(i);
            prng_[i] = rng;
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
        return weight_.data();
    }

    /// \brief Read only access to the log weights
    ///
    /// \return A const pointer to the log weights
    const double *log_weight_ptr () const
    {
        return log_weight_.data();
    }

    /// \brief Set the log weights
    ///
    /// \param [in] new_weight New log weights
    void set_log_weight (const double *new_weight, double delta = 1)
    {
        cblas_dcopy(size_, new_weight, 1, log_weight_.data(), 1);
        cblas_dscal(size_, delta, log_weight_.data(), 1);
        set_weight();
    }

    /// \brief Add to the log weights
    ///
    /// \param [in] inc_weight Incremental log weights
    void add_log_weight (const double *inc_weight, double delta = 1,
            bool add_zconst = true)
    {
        if (add_zconst) {
            for (std::size_t i = 0; i != size_; ++i)
                inc_weight_[i] = std::exp(delta * inc_weight[i]);
            zconst_ += std::log(cblas_ddot(size_, weight_.data(), 1,
                        inc_weight_.data(), 1));
        }

        for (std::size_t i = 0; i != size_; ++i)
            log_weight_[i] += delta * inc_weight[i];
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

    rng_type *prng ()
    {
        return prng_.data();
    }

    rng_type &prng (std::size_t id)
    {
        return prng_[id];
    }

    private :

    std::size_t size_;
    T value_;

    internal::Buffer<double> weight_;
    internal::Buffer<double> log_weight_;
    internal::Buffer<double> inc_weight_;
    internal::Buffer<int> replication_;

    double ess_;
    bool resampled_;
    double zconst_;

    internal::Buffer<rng_type> prng_;
    typedef boost::random::binomial_distribution<int, double> binom_type;
    binom_type binom_;

    void set_weight ()
    {
        double max_weight = log_weight_[0];
        for (std::size_t i = 0; i != size_; ++i)
            if (log_weight_[i] > max_weight)
                max_weight = log_weight_[i];
        for (std::size_t i = 0; i != size_; ++i) {
            log_weight_[i] -= max_weight;
            weight_[i] = std::exp(log_weight_[i]);
        }
        double sum = cblas_dasum(size_, weight_.data(), 1);
        cblas_dscal(size_, 1 / sum, weight_.data(), 1);
        ess_ = 1 / cblas_ddot(size_, weight_.data(), 1, weight_.data(), 1);
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
                replication_[i] = binom_(prng_[i], param);
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
            weight_[i] = std::modf(size_ * weight_[i], log_weight_.data() + i);
        std::size_t size = size_;
        size -= cblas_dasum(size_, log_weight_.data(), 1);
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

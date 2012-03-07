#ifndef V_SMC_CORE_PARTICLE_HPP
#define V_SMC_CORE_PARTICLE_HPP

#include <vector>
#include <cmath>
#include <cstddef>
#include <boost/function.hpp>
#include <boost/random/binomial_distribution.hpp>
#include <boost/random/uniform_01.hpp>
#include <Eigen/Dense>
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
    /// \param seed The seed to the parallel RNG system
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

        weight_.setConstant(1.0 / size_);
        log_weight_.setConstant(0);
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

    const Eigen::VectorXd &weight () const
    {
        return weight_;
    }

    const Eigen::VectorXd &log_weight () const
    {
        return log_weight_;
    }

    /// \brief Set the log weights
    ///
    /// \param new_weight New log weights
    /// \param delta A multiplier appiled to new_weight
    void set_log_weight (const double *new_weight, double delta = 1)
    {
        Eigen::Map<const Eigen::VectorXd> w(new_weight, size_);
        log_weight_ = delta * w;
        set_weight();
    }

    /// \brief Add to the log weights
    ///
    /// \param inc_weight Incremental log weights
    /// \param delta A multiplier applied to inc_weight
    /// \param add_zconst Whether this incremental weights should contribute
    /// the esitmates of normalizing constants
    void add_log_weight (const double *inc_weight, double delta = 1,
            bool add_zconst = true)
    {
        Eigen::Map<const Eigen::VectorXd> w(inc_weight, size_);
        if (add_zconst) {
            inc_weight_ = (delta * w).array().exp();
            zconst_ += std::log(weight_.dot(inc_weight_));
        }
        log_weight_ += delta * w;
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

    rng_type &prng (std::size_t id)
    {
        return prng_[id];
    }

    private :

    std::size_t size_;
    T value_;

    Eigen::VectorXd weight_;
    Eigen::VectorXd log_weight_;
    Eigen::VectorXd inc_weight_;
    Eigen::VectorXi replication_;

    double ess_;
    bool resampled_;
    double zconst_;

    std::vector<rng_type> prng_;
    typedef boost::random::binomial_distribution<> binom_type;
    binom_type binom_;

    void set_weight ()
    {
        double max_weight = log_weight_.maxCoeff();
        log_weight_ = log_weight_.array() - max_weight;
        weight_ = log_weight_.array().exp();
        double sum = weight_.sum();
        weight_ *= 1 / sum;
        ess_ = 1 / weight_.squaredNorm();
    }

    void resample_multinomial ()
    {
        weight2replication(size_);
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
        std::size_t size = size_ - log_weight_.sum();
        weight2replication(size);
        for (std::size_t i = 0; i != size_; ++i)
            replication_[i] += log_weight_[i];
        resample_do();
    }

    void resample_stratified ()
    {
        replication_.setConstant(0);
        std::size_t j = 0;
        std::size_t k = 0;
        boost::random::uniform_01<> unif;
        double u = unif(prng_[0]);
        double cw = weight_[0];
        while (j != size_) {
            while (j < cw * size_ - u && j != size_) {
                ++replication_[k];
                u = unif(prng_[j++]);
            }
            cw += weight_[++k];
        }
        resample_do();
    }

    void resample_systematic ()
    {
        replication_.setConstant(0);
        std::size_t j = 0;
        std::size_t k = 0;
        boost::random::uniform_01<> unif;
        double u = unif(prng_[0]);
        double cw = weight_[0];
        while (j != size_) {
            while (j < cw * size_ - u && j != size_) {
                ++replication_[k];
		++j;
            }
            cw += weight_[++k];
        }
        resample_do();
    }

    void weight2replication (std::size_t size)
    {
        double tp = weight_.sum();
        double sum_p = 0;
        std::size_t sum_n = 0;
        replication_.setConstant(0);
        for (std::size_t i = 0; i != size_; ++i) {
            if (sum_n < size && weight_[i] > 0) {
                binom_type::param_type
                    param(size - sum_n, weight_[i] / (tp - sum_p));
                replication_[i] = binom_(prng_[i], param);
            }
            sum_p += weight_[i];
            sum_n += replication_[i];
        }
    }

    void resample_do ()
    {
	// Some times the nuemrical round error can cause the total childs
	// differ from number of particles
	std::size_t sum = replication_.sum();
	if (sum != size_) {
            Eigen::VectorXd::Index id_max;
            replication_.maxCoeff(&id_max);
            replication_[id_max] += size_ - sum;
	}

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
        weight_.setConstant(1.0 / size_);
        log_weight_.setConstant(0);
    }
}; // class Particle

} // namespace vSMC

#endif // V_SMC_CORE_PARTICLE_HPP

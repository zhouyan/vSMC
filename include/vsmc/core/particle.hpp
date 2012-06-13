#ifndef VSMC_CORE_PARTICLE_HPP
#define VSMC_CORE_PARTICLE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/internal/resampling.hpp>

namespace vsmc {

/// \brief Particle class representing the whole particle set
/// \ingroup Core
///
/// \tparam T Requirement:
/// \li Constructor compatible with
/// \code T(Particle<T>::size_type N) \endcode
/// \li member function copy method compatible with
/// \code copy(Particle<T>::size_type from, Particle<T>::size_type to) \endcode
template <typename T>
class Particle
{
    public :

    /// The type of the number of particles
    typedef VSMC_SIZE_TYPE size_type;

    /// The type of the particle values
    typedef T value_type;

#ifdef VSMC_USE_SEQUENTIAL_RNG
    /// The type of the sequential pseudo random number generator C++11 engine
    typedef VSMC_SEQRNG_TYPE rng_type;
#else
    /// The type of the Counter-based random number generator
    typedef VSMC_CBRNG_TYPE cbrng_type;

    /// The type of the Counter-based random number generator C++11 engine
    typedef rng::Engine<cbrng_type> rng_type;
#endif

    /// The integer type of the seed
    typedef rng_type::result_type seed_type;

    /// The type of the weight and log weight vectors
    typedef Eigen::VectorXd weight_type;

    /// \brief Construct a Particle object with a given number of particles
    ///
    /// \param N The number of particles
    /// \param seed The seed to the parallel RNG system
    ///
    /// \post All weights are initialized to be euqal to each other
    explicit Particle (size_type N, seed_type seed = VSMC_RNG_SEED) :
        size_(N), value_(N), ess_(N),
        weight_(N), log_weight_(N), inc_weight_(N), replication_(N),
        ess_cached_(false), weight_cached_(false), log_weight_cached_(false),
        resampled_(false), zconst_(0), seed_(seed)
#ifndef VSMC_USE_SEQUENTIAL_RNG
        , prng_(N)
#endif
    {
        reset_rng();
        set_equal_weight();
    }

    /// Size of the particle set
    size_type size () const
    {
        return size_;
    }

    /// Read and write access to particle values
    value_type &value ()
    {
        return value_;
    }

    /// Read only access to particle values
    const value_type &value () const
    {
        return value_;
    }

    /// Read only access to the weights
    const weight_type &weight () const
    {
        if (!weight_cached_) {
            weight_ = log_weight().array().exp();
            double sum = weight_.sum();
            weight_ *= 1 / sum;
            weight_cached_ = true;
        }

        return weight_;
    }

    /// Read only access to the log weights
    const weight_type &log_weight () const
    {
        if (!log_weight_cached_) {
            double max_weight = log_weight_.maxCoeff();
            log_weight_ = log_weight_.array() - max_weight;
            log_weight_cached_ = true;
        }

        return log_weight_;
    }

    /// Set equal weights for all particles
    void set_equal_weight ()
    {
        ess_ = size_;
        weight_.setConstant(1.0 / size_);
        log_weight_.setConstant(0);

        ess_cached_ = true;
        weight_cached_ = true;
        log_weight_cached_ = true;
    }

    /// \brief Set the log weights with an iterator
    ///
    /// \param first The position to start the reading, it shall be valid after
    /// increments of size() times.
    /// \param delta A multiplier appiled to the new log weights
    template <typename InputIter>
    void set_log_weight (InputIter first, double delta = 1)
    {
        size_type i = 0;
        while (i != size_) {
            log_weight_[i] = *first;
            ++first;
            ++i;
        }
        log_weight_ *= delta;
        set_log_weight();
    }

    /// Set the log weights with a pointer
    void set_log_weight (const double *new_weight, double delta = 1)
    {
        Eigen::Map<const weight_type> w(new_weight, size_);
        set_log_weight(w, delta);
    }

    /// Set the log weights with a weight_type object
    void set_log_weight (const weight_type &new_weight, double delta = 1)
    {
        log_weight_ = delta * new_weight.head(size_);
        set_log_weight();
    }

    /// \brief Add to the log weights with an iterator
    ///
    /// \param first The position to start the reading, it shall be valid after
    /// increments of size() times.
    /// \param delta A multiplier appiled to the new incremental log weights
    /// \param add_zconst Whether this incremental weights shall contribute to
    /// the SMC normalizing constant estimate
    template <typename InputIter>
    void add_log_weight (InputIter first, double delta = 1,
            bool add_zconst = true)
    {
        using std::log;

        size_type i = 0;
        while (i != size_) {
            inc_weight_[i] = *first;
            ++first;
            ++i;
        }
        inc_weight_ *= delta;
        log_weight_ += inc_weight_;
        if (add_zconst)
            zconst_ += log(weight().dot(inc_weight_.array().exp().matrix()));
        set_log_weight();
    }

    /// Add to the log weights with a pointer
    void add_log_weight (const double *inc_weight, double delta = 1,
            bool add_zconst = true)
    {
        Eigen::Map<const weight_type> w(inc_weight, size_);
        add_log_weight(w, delta, add_zconst);
    }

    /// Add to the log weights with a weight_object object
    void add_log_weight (const weight_type &inc_weight, double delta = 1,
            bool add_zconst = true)
    {
        using std::log;

        inc_weight_ = delta * inc_weight.head(size_);
        log_weight_ += inc_weight_;
        if (add_zconst)
            zconst_ += log(weight().dot(inc_weight_.array().exp().matrix()));
        set_log_weight();
    }

    /// The current ESS (Effective Sample Size)
    double ess () const
    {
        if (!ess_cached_) {
            ess_ = 1 / weight().squaredNorm();
            ess_cached_ = true;
        }

        return ess_;
    }

    /// Whether resampling was performed when resampling(scheme, threshold) was
    /// last called.
    bool resampled () const
    {
        return resampled_;
    }

    /// Get the value of the logarithm of SMC normalizing constant
    double zconst () const
    {
        return zconst_;
    }

    /// Reset the value of logarithm of SMC normalizing constant to zero
    void reset_zconst ()
    {
        zconst_ = 0;
    }

    /// \brief Perform resampling if ess() < threshold * size()
    ///
    /// \param scheme The resampling scheme, see ResamplingScheme
    /// \param threshold The threshold for resampling
    void resample (ResampleScheme scheme, double threshold)
    {
        assert(replication_.size() == size());

        // call to ess() will normalize weight first
        resampled_ = ess() < threshold * size_;

        if (resampled_) {
            internal::pre_resampling(&value_);
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
                case RESIDUAL_STRATIFIED :
                    resample_residual_stratified ();
                    break;
                default :
                    resample_stratified();
                    break;
            }
            resample_do();
            internal::post_resampling(&value_);
        }
    }

    /// \brief Get a C++11 RNG engine
    ///
    /// \param id The position of the particle, 0 to size() - 1
    ///
    /// \return A reference to a C++11 RNG engine unique to particle at
    /// position id, and independent of others if \c VSMC_USE_SEQUENTIAL_RNG is
    /// not defined. Otherwise, it is the same pseudo RNG for any value of \c
    /// id
    rng_type &rng (size_type id)
    {
#ifdef VSMC_USE_SEQUENTIAL_RNG
        return srng_;
#else
        return prng_[id];
#endif
    }

    /// Reset the RNG system with a given seed
    void reset_rng (seed_type seed)
    {
        seed_ = seed;
        reset_rng();
    }

    /// Reset the parallel RNG system with the last used seed
    void reset_rng ()
    {
#ifdef VSMC_USE_SEQUENTIAL_RNG
        srng_ = rng_type(seed_);
#else
        for (size_type i = 0; i != size_; ++i)
            prng_[i] = rng_type(seed_ + i);
#endif
    }

    private :

    typedef Eigen::Matrix<size_type, Eigen::Dynamic, 1> replication_type;

    size_type size_;
    value_type value_;
    mutable double ess_;

    mutable weight_type weight_;
    mutable weight_type log_weight_;
    mutable weight_type inc_weight_;
    mutable replication_type replication_;

    mutable bool ess_cached_;
    mutable bool weight_cached_;
    mutable bool log_weight_cached_;

    bool resampled_;
    double zconst_;

    seed_type seed_;
#ifdef VSMC_USE_SEQUENTIAL_RNG
    rng_type srng_;
#else
    std::deque<rng_type> prng_;
#endif

    void set_log_weight ()
    {
        ess_cached_ = false;
        weight_cached_ = false;
        log_weight_cached_ = false;

        assert(weight_.size() == size_);
        assert(log_weight_.size() == size_);
        assert(inc_weight_.size() == size_);
    }

    void resample_multinomial ()
    {
        weight2replication(size_);
    }

    void resample_residual ()
    {
        /// \internal Reuse weight and log_weight. weight: act as the
        /// fractional part of N * weight. log_weight: act as the integral
        /// part of N * weight.  They all will be reset to equal weights after
        /// resampling.  So it is safe to modify them here.

        using std::modf;

        for (size_type i = 0; i != size_; ++i)
            weight_[i] = modf(size_ * weight_[i], log_weight_.data() + i);
        weight2replication(weight_.sum());
        for (size_type i = 0; i != size_; ++i)
            replication_[i] += log_weight_[i];
    }

    void resample_stratified ()
    {
        replication_.setConstant(0);
        size_type j = 0;
        size_type k = 0;
        rng::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng(0));
        double cw = weight_[0];
        while (j != size_) {
            while (j < cw * size_ - u && j != size_) {
                ++replication_[k];
                u = unif(rng(j));
                ++j;
            }
            if (k == size_ - 1)
                break;
            cw += weight_[++k];
        }
    }

    void resample_systematic ()
    {
        replication_.setConstant(0);
        size_type j = 0;
        size_type k = 0;
        rng::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng(0));
        double cw = weight_[0];
        while (j != size_) {
            while (j < cw * size_ - u && j != size_) {
                ++replication_[k];
                ++j;
            }
            if (k == size_ - 1)
                break;
            cw += weight_[++k];
        }
    }

    void resample_residual_stratified ()
    {
        using std::modf;

        replication_.setConstant(0);
        for (size_type i = 0; i != size_; ++i)
            weight_[i] = modf(size_ * weight_[i], log_weight_.data() + i);
        size_type size = weight_.sum();
        weight_ /= size;
        size_type j = 0;
        size_type k = 0;
        rng::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng(0));
        double cw = weight_[0];
        while (j != size) {
            while (j < cw * size - u && j != size) {
                ++replication_[k];
                u = unif(rng(j));
                ++j;
            }
            if (k == size_ - 1)
                break;
            cw += weight_[++k];
        }
        for (size_type i = 0; i != size_; ++i)
            replication_[i] += log_weight_[i];
    }

    void resample_residual_systematic ()
    {
        using std::modf;

        replication_.setConstant(0);
        for (size_type i = 0; i != size_; ++i)
            weight_[i] = modf(size_ * weight_[i], log_weight_.data() + i);
        size_type size = weight_.sum();
        weight_ /= size;
        size_type j = 0;
        size_type k = 0;
        rng::uniform_real_distribution<double> unif(0,1);
        double u = unif(rng(0));
        double cw = weight_[0];
        while (j != size) {
            while (j < cw * size - u && j != size) {
                ++replication_[k];
                ++j;
            }
            if (k == size_ - 1)
                break;
            cw += weight_[++k];
        }
        for (size_type i = 0; i != size_; ++i)
            replication_[i] += log_weight_[i];
    }

    void weight2replication (size_type size)
    {
        double tp = weight_.sum();
        double sum_p = 0;
        size_type sum_n = 0;
        replication_.setConstant(0);
        for (size_type i = 0; i != size_; ++i) {
            if (sum_n < size && weight_[i] > 0) {
                rng::binomial_distribution<size_type> binom(
                        size - sum_n, weight_[i] / (tp - sum_p));
                replication_[i] = binom(rng(i));
            }
            sum_p += weight_[i];
            sum_n += replication_[i];
        }
    }

    void resample_do ()
    {
        // Some times the nuemrical round error can cause the total childs
        // differ from number of particles
        size_type sum = replication_.sum();
        if (sum != size_) {
            size_type id_max;
            replication_.maxCoeff(&id_max);
            replication_[id_max] += size_ - sum;
        }

        size_type from = 0;
        size_type time = 0;
        for (size_type to = 0; to != size_; ++to) {
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
        set_equal_weight();
    }
}; // class Particle

} // namespace vsmc

#endif // VSMC_CORE_PARTICLE_HPP

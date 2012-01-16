#ifndef V_SMC_PARTICLE_HPP
#define V_SMC_PARTICLE_HPP

#include <algorithm>
#include <limits>
#include <cstddef>
#include <mkl_cblas.h>
#include <mkl_vml.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <boost/function.hpp>
#include <vDist/tool/buffer.hpp>
#include <vDist/tool/eblas.hpp>

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

    /// The type of copy a particle
    typedef boost::function<void (std::size_t, std::size_t, T &)> copy_type;

    /// \brief Particle does not have a default constructor
    ///
    /// \param N The number of particles
    /// \param copy A pointer to the function that can copy particle from one
    /// position to another position
    Particle (std::size_t N, const copy_type &copy) :
        size_(N), value_(N), weight_(N), log_weight_(N), replication_(N),
        copy_(copy), ess_(0), resampled_(false) {}

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
    /// \note Any operations that change the state of the particle set (e.g.,
    /// setting log weights or resampling) may invalidate the reference.
    T &value ()
    {
        return value_;
    }
    
    /// \brief Read only access to particle values
    ///
    /// \return A const reference to the particle values, type (const T &)
    /// \note Any operations that change the state of the particle set (e.g.,
    /// setting log weights or resampling) may invalidate the reference.
    const T &value () const
    {
        return value_;
    }

    /// \brief Read only access to the weights
    ///
    /// \return A const pointer to the weights, type (const double *)
    /// \note Any operations that change the state of the particle set (e.g.,
    /// setting log weights or resampling) may invalidate the pointer.
    const double *get_weight_ptr () const
    {
        return weight_.get();
    }

    /// \brief Read only access to the log weights
    ///
    /// \return A const pointer to the log weights, type (const double *)
    /// \note Any operations that change the state of the particle set (e.g.,
    /// setting log weights or resampling) may invalidate the pointer.
    const double *get_log_weight_ptr () const
    {
        return log_weight_.get();
    }

    /// \brief Set the log weights of the particle sets
    ///
    /// \param [in] new_weight New log weights
    void set_log_weight (const double *new_weight)
    {
        cblas_dcopy(size_, new_weight, 1, log_weight_, 1);
        set_weight();
    }

    /// \brief Add to the log weights of the particle sets
    ///
    /// \param [in] inc_weight Incremental log weights
    void add_log_weight (const double *inc_weight)
    {
        vdAdd(size_, log_weight_, inc_weight, log_weight_);
        set_weight();
    }

    /// \brief The ESS (Effective Sample Size)
    ///
    /// \return The value of ESS for current particle set
    double get_ESS () const
    {
        return ess_;
    }

    /// \brief Get indicator of resampling
    ///
    /// \return A bool value, \b true if the this iteration was resampled
    bool get_resample () const
    {
        return resampled_;
    }

    /// \brief Set indicator of resampling
    ///
    /// \param resampled \b true if the this iteration was resampled
    void set_resample (bool resampled)
    {
        return resampled_ = resampled;
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

    private :

    std::size_t size_;
    T value_;
    vDist::tool::Buffer<double> weight_;
    vDist::tool::Buffer<double> log_weight_;
    vDist::tool::Buffer<unsigned> replication_;
    copy_type copy_;
    double ess_;
    bool resampled_;

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

        for (std::size_t to = 0; to != size_; ++to)
        {
            if (!replication_[to]) {
                // replication_[to] has zero child, copy from elsewhere
                if (replication_[from] - time <= 1) {
                    // only 1 child left on replication_[from]
                    time = 0;
                    do // move from to some position with at least 2 children
                        ++from;
                    while (replication_[from] < 2);
                }
                copy_(from, to, value_);
                ++time;
            }
        }
        vDist::tool::dfill(size_, 1.0 / size_, weight_, 1);
        vDist::tool::dfill(size_, 0, log_weight_, 1);
        ess_ = size_;
    }
}; // class Particle

} // namespace vSMC

#endif // V_SMC_PARTICLE_HPP

#ifndef V_SMC_PARTICLE_HPP
#define V_SMC_PARTICLE_HPP

#include <algorithm>
#include <limits>
#include <cstddef>
#include <mkl_vml.h>
#include <gsl/gsl_cblas.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <vDist/utilities/buffer.hpp>
#include <vDist/utilities/eblas.hpp>

namespace vSMC {

/// Resample scheme
enum ResampleScheme {MULTINOMIAL, RESIDUAL, STRATIFIED, SYSTEMATIC};

/// \brief Particle class
///
/// Particle class store the particle set and arrays of weights and log
/// weights. It provides access to particle values as well as weights. It
/// computes and manages resources for ESS, resampling, etc, tasks unique to
/// each iteration.
template <class T>
class Particle
{
    public :

    /// The type of copy a particle
    typedef void (*copy_type) (std::size_t, std::size_t, T &);

    /// \brief Particle does not have a default constructor
    ///
    /// \param N The number of particles
    /// \param copy A pointer to the function that can copy particle from one
    /// position to another position
    Particle (std::size_t N, copy_type copy) :
        pnum(N), pval(N), weight(N), log_weight(N), rep(N),
        copy_particle(copy) {}

    /// \brief Size of the particle set
    ///
    /// \return The number of particles
    std::size_t size () const
    {
        return pnum;
    }

    /// \brief Read and write access to particle values
    ///
    /// \return A reference to the particle values, type (T &)
    /// \note Any operations that change the state of the particle (e.g.,
    /// setting log weights or resampling) set can invalidate the reference.
    T &value ()
    {
        return pval;
    }
    
    /// \brief Read only access to particle values
    ///
    /// \return A const reference to the particle values, type (const T &)
    /// \note Any operations that change the state of the particle (e.g.,
    /// setting log weights or resampling) set can invalidate the reference.
    const T &value () const
    {
        return pval;
    }

    /// \brief Read only access to the weights
    ///
    /// \return A const pointer to the weights, type (const double *)
    /// \note Any operations that change the state of the particle (e.g.,
    /// setting log weights or resampling) set can invalidate the pointer.
    const double *getWeightPtr () const
    {
        return weight.get();
    }

    /// \brief Read only access to the log weights
    ///
    /// \return A const pointer to the log weights, type (const double *)
    /// \note Any operations that change the state of the particle (e.g.,
    /// setting log weights or resampling) set can invalidate the pointer.
    const double *getLogWeightPtr () const
    {
        return log_weight.get();
    }

    /// \brief Set the log weights of the particle sets
    ///
    /// \param [in] new_weight New log weights
    void setLogWeight (const double *new_weight)
    {
        cblas_dcopy(pnum, new_weight, 1, log_weight, 1);
        set_weight();
    }

    /// \brief Add to the log weights of the particle sets
    ///
    /// \param [in] inc_weight Incremental log weights
    void addLogWeight (const double *inc_weight)
    {
        vdAdd(pnum, log_weight, inc_weight, log_weight);
        set_weight();
    }

    /// \brief The ESS (Effective Sample Size)
    ///
    /// \return The value of ESS for current particle set
    double ESS () const
    {
        return 1 / cblas_ddot(pnum, weight, 1, weight, 1);
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

    std::size_t pnum;
    T pval;
    vDist::internal::Buffer<double> weight;
    vDist::internal::Buffer<double> log_weight;
    vDist::internal::Buffer<unsigned> rep;
    copy_type copy_particle;

    void set_weight ()
    {
        double max_weight = -std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i != pnum; ++i)
            max_weight = std::max(max_weight, log_weight[i]);
        for (std::size_t i = 0; i != pnum; ++i)
            log_weight[i] -= max_weight;
        vdExp(pnum, log_weight, weight);
        double sum = cblas_dasum(pnum, weight, 1);
        cblas_dscal(pnum, 1 / sum, weight, 1);
    }

    void resample_multinomial (const gsl_rng *rng)
    {
        gsl_ran_multinomial(rng, pnum, pnum, weight, rep);
        resample_do();
    }

    void resample_residual (const gsl_rng *rng)
    {
        /// \internal Reuse weight and log_weight.
        /// weight: act as the fractional part of N * weight.
        /// log_weight: act as the integral part of N * weight.
        /// They all will be reset to equal weights after resampling.
        /// So it is safe to modify them here.
        vDist::dyatx(pnum, pnum, weight, 1, log_weight, 1);
        vdModf(pnum, log_weight, log_weight, weight);
        std::size_t size = pnum;
        size -= cblas_dasum(pnum, log_weight, 1);
        gsl_ran_multinomial(rng, pnum, size, weight, rep);
        for (std::size_t i = 0; i != pnum; ++i)
            rep[i] += log_weight[i];
        resample_do();
    }

    void resample_stratified (const gsl_rng *rng) {}
    void resample_systematic (const gsl_rng *rng) {}

    void resample_do ()
    {
        std::size_t from = 0;
        std::size_t time = 0;

        for (std::size_t to = 0; to != pnum; ++to)
        {
            if (!rep[to]) {
                // rep[to] has zero child, copy from elsewhere
                if (rep[from] - time <= 1) {
                    // only 1 child left on rep[from]
                    time = 0;
                    ++from;
                    while (rep[from] < 2)
                        // rep[from] shall has at least 2 child, 1 for itself
                        ++from;
                }
                copy_particle(from, to, pval);
                ++time;
            }
        }
        vDist::dfill(pnum, 1.0 / pnum, weight, 1);
        vDist::dfill(pnum, 0, log_weight, 1);
    }
}; // class Particle

} // namespace vSMC

#endif // V_SMC_PARTICLE_HPP

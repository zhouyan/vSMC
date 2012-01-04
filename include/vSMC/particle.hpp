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

    Particle (std::size_t N, void (*copy)(std::size_t, std::size_t, T &)) :
        pnum(N), pval(N), weight(N), log_weight(N),
        copy_particle(copy) {}

    std::size_t size () const
    {
        return pnum;
    }

    T &value ()
    {
        return pval;
    }
    
    const T &value () const
    {
        return pval;
    }

    const double *getWeightPtr () const
    {
        return weight.get();
    }

    const double *getLogWeightPtr () const
    {
        return log_weight.get();
    }

    void setLogWeight (const double *new_weight)
    {
        cblas_dcopy(pnum, new_weight, 1, log_weight, 1);
        set_weight();
    }

    void addLogWeight (const double *inc_weight)
    {
        vdAdd(pnum, log_weight, inc_weight, log_weight);
        set_weight();
    }

    double ESS () const
    {
        return 1 / cblas_ddot(pnum, weight, 1, weight, 1);
    }

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
                resample_residual(rng);
        }
    }

    private :

    std::size_t pnum;
    T pval;

    double sum_weight;
    vDist::internal::Buffer<double> weight;
    vDist::internal::Buffer<double> log_weight;

    void (*copy_particle) (std::size_t, std::size_t, T &);

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
        vDist::internal::Buffer<unsigned> rep(pnum);

        gsl_ran_multinomial(rng, pnum, pnum, weight, rep);
        resample_do(rep);
    }

    void resample_residual (const gsl_rng *rng)
    {
        vDist::internal::Buffer<unsigned> rep(pnum);
        vDist::internal::Buffer<double> uweight(pnum);
        vDist::internal::Buffer<double> prob(pnum);

        vDist::dyatx(pnum, pnum, weight, 1, uweight, 1);
        vdModf(pnum, uweight, uweight, prob);
        std::size_t size = pnum;
        size -= cblas_dasum(pnum, uweight, 1);
        gsl_ran_multinomial(rng, pnum, size, prob, rep);
        for (std::size_t i = 0; i != pnum; ++i)
            rep[i] += uweight[i];
        resample_do(rep);
    }

    void resample_stratified (const gsl_rng *rng) {}
    void resample_systematic (const gsl_rng *rng) {}

    void resample_do (const unsigned *rep)
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

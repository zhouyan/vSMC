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

enum ResampleScheme {MULTINOMIAL, RESIDUAL, STRATIFIED, SYSTEMATIC};

template <class T>
class Particle
{
    public :

    Particle (std::size_t N, void (*copy)(std::size_t, std::size_t, T &)) :
        particle_num(N), particle(N), weight(N), log_weight(N),
        copy_particle(copy) {}

    std::size_t size () const
    {
        return particle_num;
    }

    T &Value ()
    {
        return particle;
    }
    
    const T &Value () const
    {
        return particle;
    }

    const double *Weight () const
    {
        return weight.get();
    }

    const double *LogWeight () const
    {
        return log_weight.get();
    }

    void SetLogWeight (const double *new_weight)
    {
        cblas_dcopy(particle_num, new_weight, 1, log_weight, 1);
        set_weight();
    }

    void AddLogWeight (const double *inc_weight)
    {
        vdAdd(particle_num, log_weight, inc_weight, log_weight);
        set_weight();
    }

    double ESS () const
    {
        return 1 / cblas_ddot(particle_num, weight, 1, weight, 1);
    }

    void Resample (ResampleScheme scheme, const gsl_rng *rng)
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

    std::size_t particle_num;
    T particle;

    double sum_weight;
    vDist::internal::Buffer<double> weight;
    vDist::internal::Buffer<double> log_weight;

    void (*copy_particle) (std::size_t, std::size_t, T &);

    void set_weight ()
    {
        double max_weight = -std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i != particle_num; ++i)
            max_weight = std::max(max_weight, log_weight[i]);
        for (std::size_t i = 0; i != particle_num; ++i)
            log_weight[i] -= max_weight;
        vdExp(particle_num, log_weight, weight);
        double sum = cblas_dasum(particle_num, weight, 1);
        cblas_dscal(particle_num, 1 / sum, weight, 1);
    }

    void resample_multinomial (const gsl_rng *rng)
    {
        vDist::internal::Buffer<unsigned> rep(particle_num);

        gsl_ran_multinomial(rng, particle_num, particle_num, weight, rep);
        resample_do(rep);
    }

    void resample_residual (const gsl_rng *rng)
    {
        vDist::internal::Buffer<unsigned> rep(particle_num);
        vDist::internal::Buffer<double> uweight(particle_num);
        vDist::internal::Buffer<double> prob(particle_num);

        vDist::dyatx(particle_num, particle_num, weight, 1, uweight, 1);
        vdModf(particle_num, uweight, uweight, prob);
        std::size_t size = particle_num;
        size -= cblas_dasum(particle_num, uweight, 1);
        gsl_ran_multinomial(rng, particle_num, size, prob, rep);
        for (std::size_t i = 0; i != particle_num; ++i)
            rep[i] += uweight[i];
        resample_do(rep);
    }

    void resample_stratified (const gsl_rng *rng) {}
    void resample_systematic (const gsl_rng *rng) {}

    void resample_do (const unsigned *rep)
    {
        std::size_t from = 0;
        std::size_t time = 0;

        for (std::size_t to = 0; to != particle_num; ++to)
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
                copy_particle(from, to, particle);
                ++time;
            }
        }
        vDist::dfill(particle_num, 1.0 / particle_num, weight, 1);
        vDist::dfill(particle_num, 0, log_weight, 1);
    }
}; // class Particle

} // namespace vSMC

#endif // V_SMC_PARTICLE_HPP

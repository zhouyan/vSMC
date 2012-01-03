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

namespace vSMC {

enum ResampleScheme {MULTINOMIAL, RESIDUAL, STRATIFIED, SYSTEMATIC};

template <class T>
class Particle
{
    public :

    Particle (std::size_t N, void (*copy)(std::size_t, std::size_t, T &)) :
        particle_num(N), particle(N),
        sum_weight(0), weight(N), log_weight(N),
        replication(N), copy_particle(copy) {}

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

    const double SumWeight () const
    {
        return sum_weight;
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
        return sum_weight * sum_weight / cblas_dnrm2(particle_num, weight, 1);
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

    typedef vDist::internal::Buffer<double> dBuffer;
    typedef vDist::internal::Buffer<unsigned> uBuffer;

    std::size_t particle_num;
    T particle;

    double sum_weight;
    dBuffer weight;
    dBuffer log_weight;

    uBuffer replication;
    void (*copy_particle) (std::size_t, std::size_t, T &);

    void set_weight ()
    {
        double max_weight = -std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i != particle_num; ++i)
            max_weight = std::max(max_weight, log_weight[i]);
        for (std::size_t i = 0; i != particle_num; ++i)
            log_weight[i] -= max_weight;
        vdExp(particle_num, log_weight, weight);
        sum_weight = cblas_dasum(particle_num, weight, 1);
    }

    void resample_multinomial (const gsl_rng *rng)
    {
        gsl_ran_multinomial(rng, particle_num, particle_num,
                weight, replication);

        resample_do();
    }

    void resample_residual (const gsl_rng *rng)
    {
        cblas_dscal(particle_num, particle_num, weight, 1);
        vdFloor(particle_num, weight, weight);

        std::size_t size = particle_num;
        size -= sum_weight;
        gsl_ran_multinomial(rng, particle_num, size, weight, replication);

        for (std::size_t i = 0; i != particle_num; ++i)
            replication[i] += weight[i];

        resample_do();
    }

    void resample_stratified (const gsl_rng *rng) {}
    void resample_systematic (const gsl_rng *rng) {}

    void resample_do ()
    {
        std::size_t from = 0;
        std::size_t time = 0;

        for (std::size_t i = 0; i != particle_num; ++i)
        {
            if (!replication[i]) { // replication[i] has zero child
                if (time == replication[from]) {
                    // all childs of replication[from] are already copied
                    time = 0;
                    ++from;
                    while (!replication[from])
                        ++from;
                }
                copy_particle(from, i, particle);
                ++time;
            }
        }
    }
}; // class Particle

} // namespace vSMC

#endif // V_SMC_PARTICLE_HPP

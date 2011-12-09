#ifndef V_SMC_PARTICLE_HPP
#define V_SMC_PARTICLE_HPP

#include <limits>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <ctime>
#include <mkl_vml.h>
#include <mkl_vsl.h>
#include <gsl/gsl_rng.hp>
#include <gsl/gsl_randist.h>
#include <vDist/utilities/service.hpp>

namespace vSMC {

enum ResampleScheme {MULTINOMIAL, RESIDUAL, STRATIFIED, SYSTEMATIC};

template <class PartType>
class Particle
{
    public :

    Particle (std::size_t N,
            void (*copy)(std::size_t, std::size_t, PartType &)) :
        particle_num(N), particle(N), weight(N), log_weight(N),
        weight_r1m(0), weight_r2m(0), replication(N), copy_particle(copy)
    {
        vsldSSNewTask(&ess_task, 1, N, VSL_SS_MATRIX_STORAGE_COLS,
                weight, NULL, NULL);
        vsldSSEditMoments(ess_task, &weight_r1m, &weight_r2m,
                NULL, NULL, NULL, NULL, NULL);
        std::time_t curr_time = std::time(NULL);
        std::size_t max_int = std::numeric_limits<int>::max();
        std::size_t rng_seed = curr_time % max_int + 1;
        rng_gsl = gsl_rng_alloc(gsl_rng_mt19937);
        gsl_rng_set(rng_gsl, rng_seed);
    }

    Particle (const Particle<PartType> &part)
        particle_num (part.particle_num),
        particle     (part.particle),
        weight       (part.weight),
        log_weight   (part.log_weight),
        weight_r1m   (part.weight_r1m),
        weight_r2m   (part.weight_r2m),
        replication  (part.replication),
        copy_particle(part.copy_particle)
        {
        }

    ~Particle ()
    {
        vslSSDeleteTask(ess_task);
        gsl_rng_free(rng_gsl);
    }

    inline void SetLogWeight (const double *new_weight);
    inline void AddLogWeight (const double *inc_weight);

    inline double ESS ();
    inline void Resample (ResampleScheme scheme);

    private :

    typedef vDist::internal::Buffer<double> dBuffer;
    typedef vDist::internal::Buffer<std::size_t> uBuffer;

    std::size_t particle_num;
    PartType particle;
    dBuffer weight;
    dBuffer log_weight;
    uBuffer replication;
    double weight_r1m;
    double weight_r2m;

    VSLSSTaskPtr ess_task;
    gsl_rng *rng_gsl;

    void (*copy_particle) (
            std::size_t n1, std::size_t n2, PartType &particles);

    inline void require_weight ();

    inline void resample_multinomial ();
    inline void resample_residual ();
    inline void resample_stratified ();
    inline void resample_systematic ();
    inline void resample_do ();
}; // class Particle

template <class PartType>
double Particle<PartType>::SetLogWeight (const double *new_weight)
{
    cblas_dcopy(particle_num, new_weight, 1, log_weight, 1);
}

template <class PartType>
double Particle<PartType>::AddLogWeight (const double *inc_weight)
{
    vdAdd(particle_num, log_weight, inc_weight, log_weight);
}

template <class PartType>
void Particle<PartType>::require_weight ()
{
    vdExp(particle_num, log_weight, weight);
}

template <class PartType>
double Particle<PartType>::ESS ()
{
    require_weight();
    vsldSSCompute(ess_task, VSL_SS_MEAN|VSL_SS_2R_MOM, VSL_SS_METHOD_TBS);

    return particle_num * weight_r1m * weight_r1m / weight_r2m;
}


template <class PartType>
void Particle<PartType>::Resample (ResampleScheme scheme)
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
            resample_residual();
    }
}

template <class PartType>
void Particle<PartType>::resample_multinomial ()
{
    require_weight();
    gsl_ran_multinomial(rng_gsl, particle_num, particle_num,
            weight, replication);
    resample_do();
}

template <class PartType>
void Particle<PartType>::resample_residual ()
{
    require_weight();
    cblas_dscal(particle_num, particle_num, weight, 1);
    vdFloor(particle_num, weight, weight);
    std::size_t size = particle_num - cblas_dasum(particle_num, weight, 1);
    gsl_ran_multinomial(rng_gsl, particle_num, size, weight, replication);
    for (std::size_t i = 0; i != particle_num, ++i)
        replication[i] += weight[i];
    resample_do();
}

template <class PartType>
void Particle<PartType>::resample_stratified ()
{
    resample_do();
}

template <class PartType>
void Particle<PartType>::resample_systematic ()
{
    resample_do();
}

template <class PartType>
void Particle<PartType>::resample_do ()
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
            (*copy_particle)(from, i, partile_set);
            ++time;
        }
    }
}

} // namespace vSMC

#endif // V_SMC_PARTICLE_HPP

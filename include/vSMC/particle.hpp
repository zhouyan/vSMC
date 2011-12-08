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

template <class PartContainer>
class ParticleSet
{
    public :

    Particle (std::size_t N,
            void (*copy)(std::size_t, std::size_t, PartContainer &)) :
        particle_num(N), particle_set(N), weight(N), log_weight(N),
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

    ~Particle ()
    {
        vslSSDeleteTask(ess_task);
        gsl_rng_free(rng_gsl);
    }

    inline double ESS ();

    inline void SetLogWeight (const double *new_weight);
    inline void AddLogWeight (const double *inc_weight);

    inline void Resample (ResampleScheme scheme);

    private :

    typedef vDist::internal::Buffer<double> dBuffer;
    typedef vDist::internal::Buffer<std::size_t> uBuffer;

    std::size_t particle_num;
    PartContainer particle_set;
    dBuffer weight;
    dBuffer log_weight;
    uBuffer replication;
    double weight_r1m;
    double weight_r2m;

    VSLSSTaskPtr ess_task;
    gsl_rng *rng_gsl;

    void (*copy_particle) (
            std::size_t n1, std::size_t n2, PartContainer &particles);

    inline void normalize_log_weight ();

    inline void resample_multinomial ();
    inline void resample_residual ();
    inline void resample_stratified ();
    inline void resample_systematic ();
    inline void resample_do ();
};

template <class PartContainer>
double ParticleSet<PartContainer>::ESS ()
{
    vdExp(particle_num, log_weight, weight);
    vsldSSCompute(ess_task, VSL_SS_MEAN|VSL_SS_2R_MOM, VSL_SS_METHOD_TBS);

    return particle_num * weight_r1m * weight_r1m / weight_r2m;
}

template <class PartContainer>
double ParticleSet<PartContainer>::SetLogWeight (const double *new_weight)
{
    cblas_dcopy(particle_num, new_weight, 1, log_weight, 1);
    normalize_log_weight();
}

template <class PartContainer>
double ParticleSet<PartContainer>::AddLogWeight (const double *inc_weight)
{
    vdAdd(particle_num, log_weight, inc_weight, log_weight);
    normalize_log_weight();
}

template <class PartContainer>
double ParticleSet<PartContainer>::normalize_log_weight ()
{
}

template <class PartContainer>
void ParticleSet<PartContainer>::Resample (ResampleScheme scheme)
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

template <class PartContainer>
void ParticleSet<PartContainer>::resample_multinomial ()
{
    resample_do();
}

template <class PartContainer>
void ParticleSet<PartContainer>::resample_residual ()
{
    resample_do();
}

template <class PartContainer>
void ParticleSet<PartContainer>::resample_stratified ()
{
    resample_do();
}

template <class PartContainer>
void ParticleSet<PartContainer>::resample_systematic ()
{
    resample_do();
}

template <class PartContainer>
void ParticleSet<PartContainer>::resample_do ()
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

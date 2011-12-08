#ifndef V_SMC_PARTICLE_HPP
#define V_SMC_PARTICLE_HPP

#include <vector>
#include <cstddef>
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
        copy_particle(copy)
    {
        vsldSSNewTask(&ess_task, 1, N, VSL_SS_MATRIX_STORAGE_COLS,
                weight, NULL, NULL);
        vsldSSEditMoments(ess_task, NULL, ess_inv,
                NULL, NULL, NULL, NULL, NULL);
    }

    ~Particle ()
    {
        vslSSDeleteTask(ess_task);
    }

    inline double ESS ();

    inline void SetWeight    (const double *new_weight);
    inline void MulWeight    (const double *inc_weight);
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
    double ess_inv;

    VSLSSTaskPtr ess_task;

    void (*copy_particle) (
            std::size_t n1, std::size_t n2, PartContainer &particles);

    inline void resample_multinomial ();
    inline void resample_residual ();
    inline void resample_stratified ();
    inline void resample_systematic ();
    inline void resample_do (std::size_t *rep);
};

template <class PartContainer>
double ParticleSet<PartContainer>::ESS ()
{
    vsldSSCompute(ess_task, VSL_SS_2R_MOM, VSL_SS_METHOD_TBS);

    return 1 / ess_inv / particle_num;
}

template <class PartContainer>
double ParticleSet<PartContainer>::SetWeight (const double *new_weight)
{
    cblas_dcopy(particle_num, new_weight, 1, weight, 1);
    vdLn(particle_num, weight, log_weight);
}

template <class PartContainer>
double ParticleSet<PartContainer>::MulWeight (const double *inc_weight)
{
    vdMul(particle_num, weight, inc_weight, weight);
    vdLn(particle_num, weight, log_weight);
}

template <class PartContainer>
double ParticleSet<PartContainer>::SetLogWeight (const double *new_weight)
{
    cblas_dcopy(particle_num, new_weight, 1, log_weight, 1);
    vdExp(particle_num, log_weight, weight);
}

template <class PartContainer>
double ParticleSet<PartContainer>::AddLogWeight (const double *inc_weight)
{
    vdAdd(particle_num, log_weight, inc_weight, log_weight);
    vdExp(particle_num, log_weight, weight);
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
    uBuffer rep(particle_num);

    resample_do(rep);
}

template <class PartContainer>
void ParticleSet<PartContainer>::resample_residual ()
{
    uBuffer rep(particle_num);

    resample_do(rep);
}

template <class PartContainer>
void ParticleSet<PartContainer>::resample_stratified ()
{
    uBuffer rep(particle_num);

    resample_do(rep);
}

template <class PartContainer>
void ParticleSet<PartContainer>::resample_systematic ()
{
    uBuffer rep(particle_num);

    resample_do(rep);
}

template <class PartContainer>
void ParticleSet<PartContainer>::resample_do (std::size_t *rep)
{
    std::size_t from = 0;
    std::size_t time = 0;

    for (std::size_t i = 0; i != particle_num; ++i)
    {
        if (!rep[i]) { // rep[i] has zero child
            if (time == rep[from]) { // all childs of rep[from] are copied
                time = 0;
                ++from;
                while (!rep[from])
                    ++from;
            }
            (*copy_particle)(from, i, partile_set);
            ++time;
        }
    }
}

} // namespace vSMC

#endif // V_SMC_PARTICLE_HPP

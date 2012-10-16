#ifndef VSMC_CAPI_CORE_DEF_HPP
#define VSMC_CAPI_CORE_DEF_HPP

#include <vsmc.hpp>
#include <vsmc_capi.h>

namespace vsmc { namespace capi {

typedef Sampler<vsmc::StateSEQ <vsmc::Dynamic, double> > SamplerSEQ;
typedef Sampler<vsmc::StateCILK<vsmc::Dynamic, double> > SamplerCILK;
typedef Sampler<vsmc::StateOMP <vsmc::Dynamic, double> > SamplerOMP;
typedef Sampler<vsmc::StateSTD <vsmc::Dynamic, double> > SamplerSTD;
typedef Sampler<vsmc::StateTBB <vsmc::Dynamic, double> > SamplerTBB;

typedef Particle<vsmc::StateSEQ <vsmc::Dynamic, double> > ParticleSEQ;
typedef Particle<vsmc::StateCILK<vsmc::Dynamic, double> > ParticleCILK;
typedef Particle<vsmc::StateOMP <vsmc::Dynamic, double> > ParticleOMP;
typedef Particle<vsmc::StateSTD <vsmc::Dynamic, double> > ParticleSTD;
typedef Particle<vsmc::StateTBB <vsmc::Dynamic, double> > ParticleTBB;

typedef Monitor<vsmc::StateSEQ <vsmc::Dynamic, double> > MonitorSEQ;
typedef Monitor<vsmc::StateCILK<vsmc::Dynamic, double> > MonitorCILK;
typedef Monitor<vsmc::StateOMP <vsmc::Dynamic, double> > MonitorOMP;
typedef Monitor<vsmc::StateSTD <vsmc::Dynamic, double> > MonitorSTD;
typedef Monitor<vsmc::StateTBB <vsmc::Dynamic, double> > MonitorTBB;

typedef Path<vsmc::StateSEQ <vsmc::Dynamic, double> > PathSEQ;
typedef Path<vsmc::StateCILK<vsmc::Dynamic, double> > PathCILK;
typedef Path<vsmc::StateOMP <vsmc::Dynamic, double> > PathOMP;
typedef Path<vsmc::StateSTD <vsmc::Dynamic, double> > PathSTD;
typedef Path<vsmc::StateTBB <vsmc::Dynamic, double> > PathTBB;

} } // namespace vsmc::capi

/* Sequential */
#define VSMC_SAMPLER_PTR_CASE_SEQ(ptr, prefix, postfix) \
    case VSMC_BASE_SEQ : \
        prefix reinterpret_cast<vsmc::capi::SamplerSEQ *>(ptr) postfix ;\
        break;
#define VSMC_PARTICLE_PTR_CASE_SEQ(ptr, prefix, postfix) \
    case VSMC_BASE_SEQ : \
        prefix reinterpret_cast<vsmc::capi::ParticleSEQ *>(ptr) postfix ;\
        break;

/* Default */
#define VSMC_SAMPLER_PTR_CASE_DEFAULT(ptr, prefix, postfix) \
    default : \
        prefix reinterpret_cast<vsmc::capi::SamplerSEQ *>(ptr) postfix ;\
        break;
#define VSMC_PARTICLE_PTR_CASE_DEFAULT(ptr, prefix, postfix) \
    default : \
        prefix reinterpret_cast<vsmc::capi::ParticleSEQ *>(ptr) postfix ;\
        break;

/* Intel Cilk */
#if VSMC_USE_CILK
#define VSMC_SAMPLER_PTR_CASE_CILK(ptr, prefix, postfix) \
    case VSMC_BASE_CILK : \
        prefix reinterpret_cast<vsmc::capi::SamplerCILK *>(ptr) postfix ;\
        break;
#define VSMC_PARTICLE_PTR_CASE_CILK(ptr, prefix, postfix) \
    case VSMC_BASE_CILK : \
        prefix reinterpret_cast<vsmc::capi::ParticleCILK *>(ptr) postfix ;\
        break;
#else // VSMC_USE_CIL
#define VSMC_SAMPLER_PTR_CASE_CILK(ptr, prefix, postfix)
#define VSMC_PARTICLE_PTR_CASE_CILK(ptr, prefix, postfix)
#endif // VSMC_USE_CILK

/* OpenMP */
#if VSMC_USE_OMP
#define VSMC_SAMPLER_PTR_CASE_OMP(ptr, prefix, postfix) \
    case VSMC_BASE_OMP : \
        prefix reinterpret_cast<vsmc::capi::SamplerOMP *>(ptr) postfix ;\
        break;
#define VSMC_PARTICLE_PTR_CASE_OMP(ptr, prefix, postfix) \
    case VSMC_BASE_OMP : \
        prefix reinterpret_cast<vsmc::capi::ParticleOMP *>(ptr) postfix ;\
        break;
#else // VSMC_USE_OMP
#define VSMC_SAMPLER_PTR_CASE_OMP(ptr, prefix, postfix)
#define VSMC_PARTICLE_PTR_CASE_OMP(ptr, prefix, postfix)
#endif // VSMC_USE_OMP

/* std::thread */
#if VSMC_USE_STD
#define VSMC_SAMPLER_PTR_CASE_STD(ptr, prefix, postfix) \
    case VSMC_BASE_STD : \
        prefix reinterpret_cast<vsmc::capi::SamplerSTD *>(ptr) postfix ;\
        break;
#define VSMC_PARTICLE_PTR_CASE_STD(ptr, prefix, postfix) \
    case VSMC_BASE_STD : \
        prefix reinterpret_cast<vsmc::capi::ParticleSTD *>(ptr) postfix ;\
        break;
#else // VSMC_USE_STD
#define VSMC_SAMPLER_PTR_CASE_STD(ptr, prefix, postfix)
#define VSMC_PARTICLE_PTR_CASE_STD(ptr, prefix, postfix)
#endif // VSMC_USE_STD

/* Intel TBB */
#if VSMC_USE_TBB
#define VSMC_SAMPLER_PTR_CASE_TBB(ptr, prefix, postfix) \
    case VSMC_BASE_TBB : \
        prefix reinterpret_cast<vsmc::capi::SamplerTBB *>(ptr) postfix ;\
        break;
#define VSMC_PARTICLE_PTR_CASE_TBB(ptr, prefix, postfix) \
    case VSMC_BASE_TBB : \
        prefix reinterpret_cast<vsmc::capi::ParticleTBB *>(ptr) postfix ;\
        break;
#else // VSMC_USE_TBB
#define VSMC_SAMPLER_PTR_CASE_TBB(ptr, prefix, postfix)
#define VSMC_PARTICLE_PTR_CASE_TBB(ptr, prefix, postfix)
#endif // VSMC_USE_TBB

/* Resample scheme switch */
#define VSMC_RESAMPLE_SCHEME_SWITCH(scheme, valid, scheme_macro) \
    switch (scheme_macro) { \
        case VSMC_RESAMPLE_MULTINOMIAL :         \
            scheme = vsmc::MULTINOMIAL;          \
            valid = 1;                           \
            break;                               \
        case VSMC_RESAMPLE_RESIDUAL :            \
            scheme = vsmc::RESIDUAL;             \
            valid = 1;                           \
            break;                               \
        case VSMC_RESAMPLE_STRATIFIED :          \
            scheme = vsmc::STRATIFIED;           \
            valid = 1;                           \
            break;                               \
        case VSMC_RESAMPLE_SYSTEMATIC :          \
            scheme = vsmc::SYSTEMATIC;           \
            valid = 1;                           \
            break;                               \
        case VSMC_RESAMPLE_RESIDUAL_STRATIFIED : \
            scheme = vsmc::RESIDUAL_STRATIFIED;  \
            valid = 1;                           \
            break;                               \
        case VSMC_RESAMPLE_RESIDUAL_SYSTEMATIC : \
            scheme = vsmc::RESIDUAL_SYSTEMATIC;  \
            valid = 1;                           \
            break;                               \
        default :                                \
            scheme = vsmc::STRATIFIED;           \
            valid = 0;                           \
    }


/* Sampler switch */
#define VSMC_SAMPLER_PTR_SWITCH(ptr, base_type, prefix, postfix) \
    switch (base_type) { \
        VSMC_SAMPLER_PTR_CASE_SEQ(ptr, prefix, postfix); \
        VSMC_SAMPLER_PTR_CASE_CILK(ptr, prefix, postfix);    \
        VSMC_SAMPLER_PTR_CASE_OMP(ptr, prefix, postfix);     \
        VSMC_SAMPLER_PTR_CASE_STD(ptr, prefix, postfix);     \
        VSMC_SAMPLER_PTR_CASE_TBB(ptr, prefix, postfix);     \
        VSMC_SAMPLER_PTR_CASE_DEFAULT(ptr, prefix, postfix); \
    }
#define VSMC_SAMPLER_INFO_SWITCH(info, prefix, postfix) \
    VSMC_SAMPLER_PTR_SWITCH( \
            info.sampler_ptr, info.base_type, prefix, postfix)

/* Particle switch */
#define VSMC_PARTICLE_PTR_SWITCH(ptr, base_type, prefix, postfix) \
    switch (base_type) { \
        VSMC_PARTICLE_PTR_CASE_SEQ(ptr, prefix, postfix); \
        VSMC_PARTICLE_PTR_CASE_CILK(ptr, prefix, postfix);    \
        VSMC_PARTICLE_PTR_CASE_OMP(ptr, prefix, postfix);     \
        VSMC_PARTICLE_PTR_CASE_STD(ptr, prefix, postfix);     \
        VSMC_PARTICLE_PTR_CASE_TBB(ptr, prefix, postfix);     \
        VSMC_PARTICLE_PTR_CASE_DEFAULT(ptr, prefix, postfix); \
    }
#define VSMC_PARTICLE_INFO_SWITCH(info, prefix, postfix) \
    VSMC_PARTICLE_PTR_SWITCH( \
            info.particle_ptr, info.base_type, prefix, postfix)

#endif // VSMC_CAPI_CORE_DEF_HPP

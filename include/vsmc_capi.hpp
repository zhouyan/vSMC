#ifndef VSMC_CAPI_CORE_DEF_HPP
#define VSMC_CAPI_CORE_DEF_HPP

#include <vsmc.hpp>
#include <vsmc_capi.h>

namespace vsmc { namespace capi {

typedef vsmc::StateSEQ <vsmc::Dynamic, double> StateSEQ;
typedef vsmc::StateCILK<vsmc::Dynamic, double> StateCILK;
typedef vsmc::StateOMP <vsmc::Dynamic, double> StateOMP;
typedef vsmc::StateSTD <vsmc::Dynamic, double> StateSTD;
typedef vsmc::StateTBB <vsmc::Dynamic, double> StateTBB;

typedef Sampler<StateSEQ>  SamplerSEQ;
typedef Sampler<StateCILK> SamplerCILK;
typedef Sampler<StateOMP>  SamplerOMP;
typedef Sampler<StateSTD>  SamplerSTD;
typedef Sampler<StateTBB>  SamplerTBB;

typedef Particle<StateSEQ>  ParticleSEQ;
typedef Particle<StateCILK> ParticleCILK;
typedef Particle<StateOMP>  ParticleOMP;
typedef Particle<StateSTD>  ParticleSTD;
typedef Particle<StateTBB>  ParticleTBB;

typedef Monitor<StateSEQ>  MonitorSEQ;
typedef Monitor<StateCILK> MonitorCILK;
typedef Monitor<StateOMP>  MonitorOMP;
typedef Monitor<StateSTD>  MonitorSTD;
typedef Monitor<StateTBB>  MonitorTBB;

typedef Path<StateSEQ>  PathSEQ;
typedef Path<StateCILK> PathCILK;
typedef Path<StateOMP>  PathOMP;
typedef Path<StateSTD>  PathSTD;
typedef Path<StateTBB>  PathTBB;

} } // namespace vsmc::capi

// Resample scheme switch
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

// default
#define VSMC_STATE_PTR_CASE_DEFAULT(ptr, prefix, postfix) \
    default : \
        prefix reinterpret_cast<vsmc::capi::StateSEQ *>(ptr) postfix;\
        break;
#define VSMC_SAMPLER_PTR_CASE_DEFAULT(ptr, prefix, postfix) \
    default : \
        prefix reinterpret_cast<vsmc::capi::SamplerSEQ *>(ptr) postfix ;\
        break;
#define VSMC_PARTICLE_PTR_CASE_DEFAULT(ptr, prefix, postfix) \
    default : \
        prefix reinterpret_cast<vsmc::capi::ParticleSEQ *>(ptr) postfix ;\
        break;
#define VSMC_MONITOR_PTR_CASE_DEFAULT(ptr, prefix, postfix) \
    default : \
        prefix reinterpret_cast<vsmc::capi::MonitorSEQ *>(ptr) postfix ;\
        break;
#define VSMC_PATH_PTR_CASE_DEFAULT(ptr, prefix, postfix) \
    default : \
        prefix reinterpret_cast<vsmc::capi::PathSEQ *>(ptr) postfix ;\
        break;

// case VSMC_BASE_SEQ
#define VSMC_STATE_PTR_CASE_SEQ(ptr, prefix, postfix) \
    case VSMC_BASE_SEQ : \
        prefix reinterpret_cast<vsmc::capi::StateSEQ *>(ptr) postfix ;\
        break;
#define VSMC_SAMPLER_PTR_CASE_SEQ(ptr, prefix, postfix) \
    case VSMC_BASE_SEQ : \
        prefix reinterpret_cast<vsmc::capi::SamplerSEQ *>(ptr) postfix ;\
        break;
#define VSMC_PARTICLE_PTR_CASE_SEQ(ptr, prefix, postfix) \
    case VSMC_BASE_SEQ : \
        prefix reinterpret_cast<vsmc::capi::ParticleSEQ *>(ptr) postfix ;\
        break;
#define VSMC_MONITOR_PTR_CASE_SEQ(ptr, prefix, postfix) \
    case VSMC_BASE_SEQ : \
        prefix reinterpret_cast<vsmc::capi::MonitorSEQ *>(ptr) postfix ;\
        break;
#define VSMC_PATH_PTR_CASE_SEQ(ptr, prefix, postfix) \
    case VSMC_BASE_SEQ : \
        prefix reinterpret_cast<vsmc::capi::PathSEQ *>(ptr) postfix ;\
        break;

// case VSMC_BASE_CILK
#if VSMC_USE_CILK
#define VSMC_STATE_PTR_CASE_CILK(ptr, prefix, postfix) \
    case VSMC_BASE_CILK : \
        prefix reinterpret_cast<vsmc::capi::StateCILK *>(ptr) postfix ;\
        break;
#define VSMC_SAMPLER_PTR_CASE_CILK(ptr, prefix, postfix) \
    case VSMC_BASE_CILK : \
        prefix reinterpret_cast<vsmc::capi::SamplerCILK *>(ptr) postfix ;\
        break;
#define VSMC_PARTICLE_PTR_CASE_CILK(ptr, prefix, postfix) \
    case VSMC_BASE_CILK : \
        prefix reinterpret_cast<vsmc::capi::ParticleCILK *>(ptr) postfix ;\
        break;
#define VSMC_MONITOR_PTR_CASE_CILK(ptr, prefix, postfix) \
    case VSMC_BASE_CILK : \
        prefix reinterpret_cast<vsmc::capi::MonitorCILK *>(ptr) postfix ;\
        break;
#define VSMC_PATH_PTR_CASE_CILK(ptr, prefix, postfix) \
    case VSMC_BASE_CILK : \
        prefix reinterpret_cast<vsmc::capi::PathCILK *>(ptr) postfix ;\
        break;
#else // VSMC_USE_CILK
#define VSMC_STATE_PTR_CASE_CILK(ptr, prefix, postfix)
#define VSMC_SAMPLER_PTR_CASE_CILK(ptr, prefix, postfix)
#define VSMC_PARTICLE_PTR_CASE_CILK(ptr, prefix, postfix)
#define VSMC_MONITOR_PTR_CASE_CILK(ptr, prefix, postfix)
#define VSMC_PATH_PTR_CASE_CILK(ptr, prefix, postfix)
#endif // VSMC_USE_CILK

// cast VSMC_BASE_OMP
#if VSMC_USE_OMP
#define VSMC_STATE_PTR_CASE_OMP(ptr, prefix, postfix) \
    case VSMC_BASE_OMP : \
        prefix reinterpret_cast<vsmc::capi::StateOMP *>(ptr) postfix ;\
        break;
#define VSMC_SAMPLER_PTR_CASE_OMP(ptr, prefix, postfix) \
    case VSMC_BASE_OMP : \
        prefix reinterpret_cast<vsmc::capi::SamplerOMP *>(ptr) postfix ;\
        break;
#define VSMC_PARTICLE_PTR_CASE_OMP(ptr, prefix, postfix) \
    case VSMC_BASE_OMP : \
        prefix reinterpret_cast<vsmc::capi::ParticleOMP *>(ptr) postfix ;\
        break;
#define VSMC_MONITOR_PTR_CASE_OMP(ptr, prefix, postfix) \
    case VSMC_BASE_OMP : \
        prefix reinterpret_cast<vsmc::capi::MonitorOMP *>(ptr) postfix ;\
        break;
#define VSMC_PATH_PTR_CASE_OMP(ptr, prefix, postfix) \
    case VSMC_BASE_OMP : \
        prefix reinterpret_cast<vsmc::capi::PathOMP *>(ptr) postfix ;\
        break;
#else // VSMC_USE_OMP
#define VSMC_STATE_PTR_CASE_OMP(ptr, prefix, postfix)
#define VSMC_SAMPLER_PTR_CASE_OMP(ptr, prefix, postfix)
#define VSMC_PARTICLE_PTR_CASE_OMP(ptr, prefix, postfix)
#define VSMC_MONITOR_PTR_CASE_OMP(ptr, prefix, postfix)
#define VSMC_PATH_PTR_CASE_OMP(ptr, prefix, postfix)
#endif // VSMC_USE_OMP

// case VSMC_BASE_STD
#if VSMC_USE_STD
#define VSMC_STATE_PTR_CASE_STD(ptr, prefix, postfix) \
    case VSMC_BASE_STD : \
        prefix reinterpret_cast<vsmc::capi::StateSTD *>(ptr) postfix ;\
        break;
#define VSMC_SAMPLER_PTR_CASE_STD(ptr, prefix, postfix) \
    case VSMC_BASE_STD : \
        prefix reinterpret_cast<vsmc::capi::SamplerSTD *>(ptr) postfix ;\
        break;
#define VSMC_PARTICLE_PTR_CASE_STD(ptr, prefix, postfix) \
    case VSMC_BASE_STD : \
        prefix reinterpret_cast<vsmc::capi::ParticleSTD *>(ptr) postfix ;\
        break;
#define VSMC_MONITOR_PTR_CASE_STD(ptr, prefix, postfix) \
    case VSMC_BASE_STD : \
        prefix reinterpret_cast<vsmc::capi::MonitorSTD *>(ptr) postfix ;\
        break;
#define VSMC_PATH_PTR_CASE_STD(ptr, prefix, postfix) \
    case VSMC_BASE_STD : \
        prefix reinterpret_cast<vsmc::capi::PathSTD *>(ptr) postfix ;\
        break;
#else // VSMC_USE_STD
#define VSMC_STATE_PTR_CASE_STD(ptr, prefix, postfix)
#define VSMC_SAMPLER_PTR_CASE_STD(ptr, prefix, postfix)
#define VSMC_PARTICLE_PTR_CASE_STD(ptr, prefix, postfix)
#define VSMC_MONITOR_PTR_CASE_STD(ptr, prefix, postfix)
#define VSMC_PATH_PTR_CASE_STD(ptr, prefix, postfix)
#endif // VSMC_USE_STD

// case VSMC_BASE_TBB
#if VSMC_USE_TBB
#define VSMC_STATE_PTR_CASE_TBB(ptr, prefix, postfix) \
    case VSMC_BASE_TBB : \
        prefix reinterpret_cast<vsmc::capi::StateTBB *>(ptr) postfix ;\
        break;
#define VSMC_SAMPLER_PTR_CASE_TBB(ptr, prefix, postfix) \
    case VSMC_BASE_TBB : \
        prefix reinterpret_cast<vsmc::capi::SamplerTBB *>(ptr) postfix ;\
        break;
#define VSMC_PARTICLE_PTR_CASE_TBB(ptr, prefix, postfix) \
    case VSMC_BASE_TBB : \
        prefix reinterpret_cast<vsmc::capi::ParticleTBB *>(ptr) postfix ;\
        break;
#define VSMC_MONITOR_PTR_CASE_TBB(ptr, prefix, postfix) \
    case VSMC_BASE_TBB : \
        prefix reinterpret_cast<vsmc::capi::MonitorTBB *>(ptr) postfix ;\
        break;
#define VSMC_PATH_PTR_CASE_TBB(ptr, prefix, postfix) \
    case VSMC_BASE_TBB : \
        prefix reinterpret_cast<vsmc::capi::PathTBB *>(ptr) postfix ;\
        break;
#else // VSMC_USE_TBB
#define VSMC_STATE_PTR_CASE_TBB(ptr, prefix, postfix)
#define VSMC_SAMPLER_PTR_CASE_TBB(ptr, prefix, postfix)
#define VSMC_PARTICLE_PTR_CASE_TBB(ptr, prefix, postfix)
#define VSMC_MONITOR_PTR_CASE_TBB(ptr, prefix, postfix)
#define VSMC_PATH_PTR_CASE_TBB(ptr, prefix, postfix)
#endif // VSMC_USE_TBB

// State switch
#define VSMC_STATE_PTR_SWITCH(ptr, base_type, prefix, postfix) \
    switch (base_type) { \
        VSMC_STATE_PTR_CASE_SEQ(ptr, prefix, postfix); \
        VSMC_STATE_PTR_CASE_CILK(ptr, prefix, postfix);    \
        VSMC_STATE_PTR_CASE_OMP(ptr, prefix, postfix);     \
        VSMC_STATE_PTR_CASE_STD(ptr, prefix, postfix);     \
        VSMC_STATE_PTR_CASE_TBB(ptr, prefix, postfix);     \
        VSMC_STATE_PTR_CASE_DEFAULT(ptr, prefix, postfix); \
    }
#define VSMC_STATE_SWITCH(state, prefix, postfix) \
    VSMC_STATE_PTR_SWITCH( \
            state.state_ptr, state.base_type, prefix, postfix)

// Sampler switch
#define VSMC_SAMPLER_PTR_SWITCH(ptr, base_type, prefix, postfix) \
    switch (base_type) { \
        VSMC_SAMPLER_PTR_CASE_SEQ(ptr, prefix, postfix); \
        VSMC_SAMPLER_PTR_CASE_CILK(ptr, prefix, postfix);    \
        VSMC_SAMPLER_PTR_CASE_OMP(ptr, prefix, postfix);     \
        VSMC_SAMPLER_PTR_CASE_STD(ptr, prefix, postfix);     \
        VSMC_SAMPLER_PTR_CASE_TBB(ptr, prefix, postfix);     \
        VSMC_SAMPLER_PTR_CASE_DEFAULT(ptr, prefix, postfix); \
    }
#define VSMC_SAMPLER_SWITCH(sampler, prefix, postfix) \
    VSMC_SAMPLER_PTR_SWITCH( \
            sampler.sampler_ptr, sampler.base_type, prefix, postfix)

// Particle switch
#define VSMC_PARTICLE_PTR_SWITCH(ptr, base_type, prefix, postfix) \
    switch (base_type) { \
        VSMC_PARTICLE_PTR_CASE_SEQ(ptr, prefix, postfix); \
        VSMC_PARTICLE_PTR_CASE_CILK(ptr, prefix, postfix);    \
        VSMC_PARTICLE_PTR_CASE_OMP(ptr, prefix, postfix);     \
        VSMC_PARTICLE_PTR_CASE_STD(ptr, prefix, postfix);     \
        VSMC_PARTICLE_PTR_CASE_TBB(ptr, prefix, postfix);     \
        VSMC_PARTICLE_PTR_CASE_DEFAULT(ptr, prefix, postfix); \
    }
#define VSMC_PARTICLE_SWITCH(particle, prefix, postfix) \
    VSMC_PARTICLE_PTR_SWITCH( \
            particle.particle_ptr, particle.base_type, prefix, postfix)

// Monitor switch
#define VSMC_MONITOR_PTR_SWITCH(ptr, base_type, prefix, postfix) \
    switch (base_type) { \
        VSMC_MONITOR_PTR_CASE_SEQ(ptr, prefix, postfix); \
        VSMC_MONITOR_PTR_CASE_CILK(ptr, prefix, postfix);    \
        VSMC_MONITOR_PTR_CASE_OMP(ptr, prefix, postfix);     \
        VSMC_MONITOR_PTR_CASE_STD(ptr, prefix, postfix);     \
        VSMC_MONITOR_PTR_CASE_TBB(ptr, prefix, postfix);     \
        VSMC_MONITOR_PTR_CASE_DEFAULT(ptr, prefix, postfix); \
    }
#define VSMC_MONITOR_SWITCH(monitor, prefix, postfix) \
    VSMC_MONITOR_PTR_SWITCH( \
            monitor.monitor_ptr, monitor.base_type, prefix, postfix)

// Path switch
#define VSMC_PATH_PTR_SWITCH(ptr, base_type, prefix, postfix) \
    switch (base_type) { \
        VSMC_PATH_PTR_CASE_SEQ(ptr, prefix, postfix); \
        VSMC_PATH_PTR_CASE_CILK(ptr, prefix, postfix);    \
        VSMC_PATH_PTR_CASE_OMP(ptr, prefix, postfix);     \
        VSMC_PATH_PTR_CASE_STD(ptr, prefix, postfix);     \
        VSMC_PATH_PTR_CASE_TBB(ptr, prefix, postfix);     \
        VSMC_PATH_PTR_CASE_DEFAULT(ptr, prefix, postfix); \
    }
#define VSMC_PATH_SWITCH(path, prefix, postfix) \
    VSMC_PATH_PTR_SWITCH( \
            path.path_ptr, path.base_type, prefix, postfix)

#endif // VSMC_CAPI_CORE_DEF_HPP

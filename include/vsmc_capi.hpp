#ifndef VSMC_CAPI_CORE_DEF_HPP
#define VSMC_CAPI_CORE_DEF_HPP

#include <vsmc.hpp>
#include <vsmc_capi.h>

namespace vsmc { namespace capi {

template <template <unsigned, typename> class> struct BaseType;
template<> struct BaseType<StateSEQ>  {enum {base_type = VSMC_BASE_SEQ}; };
template<> struct BaseType<StateCILK> {enum {base_type = VSMC_BASE_CILK};};
template<> struct BaseType<StateOMP>  {enum {base_type = VSMC_BASE_OMP}; };
template<> struct BaseType<StateSTD>  {enum {base_type = VSMC_BASE_STD}; };
template<> struct BaseType<StateTBB>  {enum {base_type = VSMC_BASE_TBB}; };

typedef StateSEQ <Dynamic, double> ValueSEQ;
typedef StateCILK<Dynamic, double> ValueCILK;
typedef StateOMP <Dynamic, double> ValueOMP;
typedef StateSTD <Dynamic, double> ValueSTD;
typedef StateTBB <Dynamic, double> ValueTBB;

typedef Sampler<ValueSEQ>  SamplerSEQ;
typedef Sampler<ValueCILK> SamplerCILK;
typedef Sampler<ValueOMP>  SamplerOMP;
typedef Sampler<ValueSTD>  SamplerSTD;
typedef Sampler<ValueTBB>  SamplerTBB;

typedef Particle<ValueSEQ>  ParticleSEQ;
typedef Particle<ValueCILK> ParticleCILK;
typedef Particle<ValueOMP>  ParticleOMP;
typedef Particle<ValueSTD>  ParticleSTD;
typedef Particle<ValueTBB>  ParticleTBB;

typedef Monitor<ValueSEQ>  MonitorSEQ;
typedef Monitor<ValueCILK> MonitorCILK;
typedef Monitor<ValueOMP>  MonitorOMP;
typedef Monitor<ValueSTD>  MonitorSTD;
typedef Monitor<ValueTBB>  MonitorTBB;

typedef Path<ValueSEQ>  PathSEQ;
typedef Path<ValueCILK> PathCILK;
typedef Path<ValueOMP>  PathOMP;
typedef Path<ValueSTD>  PathSTD;
typedef Path<ValueTBB>  PathTBB;

typedef InitializeAdapter<ValueSEQ,  InitializeSEQ>  InitAdapterSEQ;
typedef InitializeAdapter<ValueCILK, InitializeCILK> InitAdapterCILK;
typedef InitializeAdapter<ValueOMP,  InitializeOMP>  InitAdapterOMP;
typedef InitializeAdapter<ValueSTD,  InitializeSTD>  InitAdapterSTD;
typedef InitializeAdapter<ValueTBB,  InitializeTBB>  InitAdapterTBB;

typedef MoveAdapter<ValueSEQ,  MoveSEQ>  MoveAdapterSEQ;
typedef MoveAdapter<ValueCILK, MoveCILK> MoveAdapterCILK;
typedef MoveAdapter<ValueOMP,  MoveOMP>  MoveAdapterOMP;
typedef MoveAdapter<ValueSTD,  MoveSTD>  MoveAdapterSTD;
typedef MoveAdapter<ValueTBB,  MoveTBB>  MoveAdapterTBB;

typedef MonitorEvalAdapter<ValueSEQ,  MonitorEvalSEQ>  MonitorEvalAdapterSEQ;
typedef MonitorEvalAdapter<ValueCILK, MonitorEvalCILK> MonitorEvalAdapterCILK;
typedef MonitorEvalAdapter<ValueOMP,  MonitorEvalOMP>  MonitorEvalAdapterOMP;
typedef MonitorEvalAdapter<ValueSTD,  MonitorEvalSTD>  MonitorEvalAdapterSTD;
typedef MonitorEvalAdapter<ValueTBB,  MonitorEvalTBB>  MonitorEvalAdapterTBB;

typedef PathEvalAdapter<ValueSEQ,  PathEvalSEQ>  PathEvalAdapterSEQ;
typedef PathEvalAdapter<ValueCILK, PathEvalCILK> PathEvalAdapterCILK;
typedef PathEvalAdapter<ValueOMP,  PathEvalOMP>  PathEvalAdapterOMP;
typedef PathEvalAdapter<ValueSTD,  PathEvalSTD>  PathEvalAdapterSTD;
typedef PathEvalAdapter<ValueTBB,  PathEvalTBB>  PathEvalAdapterTBB;

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
#define VSMC_VALUE_PTR_CASE_DEFAULT(ptr, prefix, postfix) \
    default : \
        prefix reinterpret_cast<vsmc::capi::ValueSEQ *>(ptr) postfix;\
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
#define VSMC_INIT_PTR_CASE_DEFAULT(ptr, prefix, postfix) \
    default : \
        prefix reinterpret_cast<vsmc::capi::InitAdapterSEQ *>(ptr) postfix;\
        break;
#define VSMC_MOVE_PTR_CASE_DEFAULT(ptr, prefix, postfix) \
    default : \
        prefix reinterpret_cast<vsmc::capi::MoveAdapterSEQ *>(ptr) postfix;\
        break;
#define VSMC_MEVAL_PTR_CASE_DEFAULT(ptr, prefix, postfix) \
    default : \
        prefix reinterpret_cast<vsmc::capi::MonitorEvalAdapterSEQ *>(ptr) \
        postfix; break;

// case VSMC_BASE_SEQ
#define VSMC_VALUE_PTR_CASE_SEQ(ptr, prefix, postfix) \
    case VSMC_BASE_SEQ : \
        prefix reinterpret_cast<vsmc::capi::ValueSEQ *>(ptr) postfix ;\
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
#define VSMC_INIT_PTR_CASE_SEQ(ptr, prefix, postfix) \
    case VSMC_BASE_SEQ : \
        prefix reinterpret_cast<vsmc::capi::InitAdapterSEQ *>(ptr) postfix;\
        break;
#define VSMC_MOVE_PTR_CASE_SEQ(ptr, prefix, postfix) \
    case VSMC_BASE_SEQ : \
        prefix reinterpret_cast<vsmc::capi::MoveAdapterSEQ *>(ptr) postfix;\
        break;
#define VSMC_MEVAL_PTR_CASE_SEQ(ptr, prefix, postfix) \
    case VSMC_BASE_SEQ : \
        prefix reinterpret_cast<vsmc::capi::MonitorEvalAdapterSEQ *>(ptr) \
        postfix; break;

// case VSMC_BASE_CILK
#if VSMC_USE_CILK
#define VSMC_VALUE_PTR_CASE_CILK(ptr, prefix, postfix) \
    case VSMC_BASE_CILK : \
        prefix reinterpret_cast<vsmc::capi::ValueCILK *>(ptr) postfix ;\
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
#define VSMC_INIT_PTR_CASE_CILK(ptr, prefix, postfix) \
    case VSMC_BASE_CILK : \
        prefix reinterpret_cast<vsmc::capi::InitAdapterCILK *>(ptr) postfix;\
        break;
#define VSMC_MOVE_PTR_CASE_CILK(ptr, prefix, postfix) \
    case VSMC_BASE_CILK : \
        prefix reinterpret_cast<vsmc::capi::MoveAdapterCILK *>(ptr) postfix;\
        break;
#define VSMC_MEVAL_PTR_CASE_CILK(ptr, prefix, postfix) \
    case VSMC_BASE_CILK : \
        prefix reinterpret_cast<vsmc::capi::MonitorEvalAdapterCILK *>(ptr) \
        postfix; break;
#else // VSMC_USE_CILK
#define VSMC_VALUE_PTR_CASE_CILK(ptr, prefix, postfix)
#define VSMC_SAMPLER_PTR_CASE_CILK(ptr, prefix, postfix)
#define VSMC_PARTICLE_PTR_CASE_CILK(ptr, prefix, postfix)
#define VSMC_MONITOR_PTR_CASE_CILK(ptr, prefix, postfix)
#define VSMC_PATH_PTR_CASE_CILK(ptr, prefix, postfix)
#define VSMC_INIT_PTR_CASE_CILK(ptr, prefix, postfix)
#define VSMC_MOVE_PTR_CASE_CILK(ptr, prefix, postfix)
#define VSMC_MEVAL_PTR_CASE_CILK(ptr, prefix, postfix)
#endif // VSMC_USE_CILK

// cast VSMC_BASE_OMP
#if VSMC_USE_OMP
#define VSMC_VALUE_PTR_CASE_OMP(ptr, prefix, postfix) \
    case VSMC_BASE_OMP : \
        prefix reinterpret_cast<vsmc::capi::ValueOMP *>(ptr) postfix ;\
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
#define VSMC_INIT_PTR_CASE_OMP(ptr, prefix, postfix) \
    case VSMC_BASE_OMP : \
        prefix reinterpret_cast<vsmc::capi::InitAdapterOMP *>(ptr) postfix;\
        break;
#define VSMC_MOVE_PTR_CASE_OMP(ptr, prefix, postfix) \
    case VSMC_BASE_OMP : \
        prefix reinterpret_cast<vsmc::capi::MoveAdapterOMP *>(ptr) postfix;\
        break;
#define VSMC_MEVAL_PTR_CASE_OMP(ptr, prefix, postfix) \
    case VSMC_BASE_OMP : \
        prefix reinterpret_cast<vsmc::capi::MonitorEvalAdapterOMP *>(ptr) \
        postfix; break;
#else // VSMC_USE_OMP
#define VSMC_VALUE_PTR_CASE_OMP(ptr, prefix, postfix)
#define VSMC_SAMPLER_PTR_CASE_OMP(ptr, prefix, postfix)
#define VSMC_PARTICLE_PTR_CASE_OMP(ptr, prefix, postfix)
#define VSMC_MONITOR_PTR_CASE_OMP(ptr, prefix, postfix)
#define VSMC_PATH_PTR_CASE_OMP(ptr, prefix, postfix)
#define VSMC_INIT_PTR_CASE_OMP(ptr, prefix, postfix)
#define VSMC_MOVE_PTR_CASE_OMP(ptr, prefix, postfix)
#define VSMC_MEVAL_PTR_CASE_OMP(ptr, prefix, postfix)
#endif // VSMC_USE_OMP

// case VSMC_BASE_STD
#if VSMC_USE_STD
#define VSMC_VALUE_PTR_CASE_STD(ptr, prefix, postfix) \
    case VSMC_BASE_STD : \
        prefix reinterpret_cast<vsmc::capi::ValueSTD *>(ptr) postfix ;\
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
#define VSMC_INIT_PTR_CASE_STD(ptr, prefix, postfix) \
    case VSMC_BASE_STD : \
        prefix reinterpret_cast<vsmc::capi::InitAdapterSTD *>(ptr) postfix;\
        break;
#define VSMC_MOVE_PTR_CASE_STD(ptr, prefix, postfix) \
    case VSMC_BASE_STD : \
        prefix reinterpret_cast<vsmc::capi::MoveAdapterSTD *>(ptr) postfix;\
        break;
#define VSMC_MEVAL_PTR_CASE_STD(ptr, prefix, postfix) \
    case VSMC_BASE_STD : \
        prefix reinterpret_cast<vsmc::capi::MonitorEvalAdapterSTD *>(ptr) \
        postfix; break;
#else // VSMC_USE_STD
#define VSMC_VALUE_PTR_CASE_STD(ptr, prefix, postfix)
#define VSMC_SAMPLER_PTR_CASE_STD(ptr, prefix, postfix)
#define VSMC_PARTICLE_PTR_CASE_STD(ptr, prefix, postfix)
#define VSMC_MONITOR_PTR_CASE_STD(ptr, prefix, postfix)
#define VSMC_PATH_PTR_CASE_STD(ptr, prefix, postfix)
#define VSMC_INIT_PTR_CASE_STD(ptr, prefix, postfix)
#define VSMC_MOVE_PTR_CASE_STD(ptr, prefix, postfix)
#define VSMC_MEVAL_PTR_CASE_STD(ptr, prefix, postfix)
#endif // VSMC_USE_STD

// case VSMC_BASE_TBB
#if VSMC_USE_TBB
#define VSMC_VALUE_PTR_CASE_TBB(ptr, prefix, postfix) \
    case VSMC_BASE_TBB : \
        prefix reinterpret_cast<vsmc::capi::ValueTBB *>(ptr) postfix ;\
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
#define VSMC_INIT_PTR_CASE_TBB(ptr, prefix, postfix) \
    case VSMC_BASE_TBB : \
        prefix reinterpret_cast<vsmc::capi::InitAdapterTBB *>(ptr) postfix;\
        break;
#define VSMC_MOVE_PTR_CASE_TBB(ptr, prefix, postfix) \
    case VSMC_BASE_TBB : \
        prefix reinterpret_cast<vsmc::capi::MoveAdapterTBB *>(ptr) postfix;\
        break;
#define VSMC_MEVAL_PTR_CASE_TBB(ptr, prefix, postfix) \
    case VSMC_BASE_TBB : \
        prefix reinterpret_cast<vsmc::capi::MonitorEvalAdapterTBB *>(ptr) \
        postfix; break;
#else // VSMC_USE_TBB
#define VSMC_VALUE_PTR_CASE_TBB(ptr, prefix, postfix)
#define VSMC_SAMPLER_PTR_CASE_TBB(ptr, prefix, postfix)
#define VSMC_PARTICLE_PTR_CASE_TBB(ptr, prefix, postfix)
#define VSMC_MONITOR_PTR_CASE_TBB(ptr, prefix, postfix)
#define VSMC_PATH_PTR_CASE_TBB(ptr, prefix, postfix)
#define VSMC_INIT_PTR_CASE_TBB(ptr, prefix, postfix)
#define VSMC_MOVE_PTR_CASE_TBB(ptr, prefix, postfix)
#define VSMC_MEVAL_PTR_CASE_TBB(ptr, prefix, postfix)
#endif // VSMC_USE_TBB

// Value switch
#define VSMC_VALUE_PTR_SWITCH(ptr, base_type, prefix, postfix) \
    switch (base_type) { \
        VSMC_VALUE_PTR_CASE_SEQ(ptr, prefix, postfix); \
        VSMC_VALUE_PTR_CASE_CILK(ptr, prefix, postfix);    \
        VSMC_VALUE_PTR_CASE_OMP(ptr, prefix, postfix);     \
        VSMC_VALUE_PTR_CASE_STD(ptr, prefix, postfix);     \
        VSMC_VALUE_PTR_CASE_TBB(ptr, prefix, postfix);     \
        VSMC_VALUE_PTR_CASE_DEFAULT(ptr, prefix, postfix); \
    }
#define VSMC_VALUE_SWITCH(value, prefix, postfix) \
    VSMC_VALUE_PTR_SWITCH( \
            value.value_ptr, value.base_type, prefix, postfix)

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

// Init switch
#define VSMC_INIT_PTR_SWITCH(ptr, base_type, prefix, postfix) \
    switch (base_type) { \
        VSMC_INIT_PTR_CASE_SEQ(ptr, prefix, postfix); \
        VSMC_INIT_PTR_CASE_CILK(ptr, prefix, postfix);    \
        VSMC_INIT_PTR_CASE_OMP(ptr, prefix, postfix);     \
        VSMC_INIT_PTR_CASE_STD(ptr, prefix, postfix);     \
        VSMC_INIT_PTR_CASE_TBB(ptr, prefix, postfix);     \
        VSMC_INIT_PTR_CASE_DEFAULT(ptr, prefix, postfix); \
    }
#define VSMC_INIT_SWITCH(init, prefix, postfix) \
    VSMC_INIT_PTR_SWITCH( \
            init.initialize_ptr, init.base_type, prefix, postfix)

// Move switch
#define VSMC_MOVE_PTR_SWITCH(ptr, base_type, prefix, postfix) \
    switch (base_type) { \
        VSMC_MOVE_PTR_CASE_SEQ(ptr, prefix, postfix); \
        VSMC_MOVE_PTR_CASE_CILK(ptr, prefix, postfix);    \
        VSMC_MOVE_PTR_CASE_OMP(ptr, prefix, postfix);     \
        VSMC_MOVE_PTR_CASE_STD(ptr, prefix, postfix);     \
        VSMC_MOVE_PTR_CASE_TBB(ptr, prefix, postfix);     \
        VSMC_MOVE_PTR_CASE_DEFAULT(ptr, prefix, postfix); \
    }
#define VSMC_MOVE_SWITCH(move, prefix, postfix) \
    VSMC_MOVE_PTR_SWITCH( \
            move.move_ptr, move.base_type, prefix, postfix)

// MonitorEval switch
#define VSMC_MEVAL_PTR_SWITCH(ptr, base_type, prefix, postfix) \
    switch (base_type) { \
        VSMC_MEVAL_PTR_CASE_SEQ(ptr, prefix, postfix); \
        VSMC_MEVAL_PTR_CASE_CILK(ptr, prefix, postfix);    \
        VSMC_MEVAL_PTR_CASE_OMP(ptr, prefix, postfix);     \
        VSMC_MEVAL_PTR_CASE_STD(ptr, prefix, postfix);     \
        VSMC_MEVAL_PTR_CASE_TBB(ptr, prefix, postfix);     \
        VSMC_MEVAL_PTR_CASE_DEFAULT(ptr, prefix, postfix); \
    }
#define VSMC_MEVAL_SWITCH(meval, prefix, postfix) \
    VSMC_MEVAL_PTR_SWITCH( \
            meval.monitor_eval_ptr, meval.base_type, prefix, postfix)

#endif // VSMC_CAPI_CORE_DEF_HPP

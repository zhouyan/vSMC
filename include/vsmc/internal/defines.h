//============================================================================
// vSMC/include/vsmc/internal/defines.h
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_INTERNAL_DEFINES_H
#define VSMC_INTERNAL_DEFINES_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/// \addtogroup C_API_Definitions
/// @{

/// \brief vsmc::MatrixLayout
typedef enum {
    vSMCRowMajor = 101, ///< vsmc::RowMajor
    vSMCColMajor = 102  ///< vsmc::ColMajor
} vSMCMatrixLayout;

/// \brief vsmc::ResampleScheme
typedef enum {
    vSMCMultinomial,        ///< vsmc::Multinomial
    vSMCStratified,         ///< vsmc::Stratified
    vSMCSystematic,         ///< vsmc::Systematic
    vSMCResidual,           ///< vsmc::Residual
    vSMCResidualStratified, ///< vsmc::ResidualStratified
    vSMCResidualSystematic  ///< vsmc::ResidualSystematic
} vSMCResampleScheme;

/// \brief vsmc::MonitorStage
typedef enum {
    vSMCMonitorMove,     ///< vsmc::MonitorMove
    vSMCMonitorResample, ///< vsmc::MonitorResample
    vSMCMonitorMCMC      ///< vsmc::MonitorMCMC
} vSMCMonitorStage;

/// \brief RNG types
typedef enum {
    vSMCRNG,    ///< vsmc::RNG
    vSMCRNG_64, ///< vsmc::RNG_64

    vSMCRNGMini,    ///< vsmc::RNGMini
    vSMCRNGMini_64, ///< vsmc::RNGMini_64

    vSMCPhilox,    ///< vsmc::Philox
    vSMCPhilox_64, ///< vsmc::Philox_64

    vSMCThreefry,    ///< vsmc::Threefry
    vSMCThreefry_64, ///< vsmc::Threefry_64

    vSMCAES128,    ///< vsmc::AES128
    vSMCAES128_64, ///< vsmc::AES128_64

    vSMCAES192,    ///< vsmc::AES192
    vSMCAES192_64, ///< vsmc::AES192_64

    vSMCAES256,    ///< vsmc::AES256
    vSMCAES256_64, ///< vsmc::AES256_64

    vSMCARS,    ///< vsmc::ARS
    vSMCARS_64, ///< vsmc::ARS_64

    vSMCPhilox2x32,    ///< vsmc::Philox2x32
    vSMCPhilox4x32,    ///< vsmc::Philox4x32
    vSMCPhilox2x64,    ///< vsmc::Philox2x64
    vSMCPhilox4x64,    ///< vsmc::Philox4x64
    vSMCPhilox2x32_64, ///< vsmc::Philox2x32_64
    vSMCPhilox4x32_64, ///< vsmc::Philox4x32_64
    vSMCPhilox2x64_64, ///< vsmc::Philox2x64_64
    vSMCPhilox4x64_64, ///< vsmc::Philox4x64_64

    vSMCThreefry2x32,     ///< vsmc::Threefry2x32
    vSMCThreefry4x32,     ///< vsmc::Threefry4x32
    vSMCThreefry2x64,     ///< vsmc::Threefry2x64
    vSMCThreefry4x64,     ///< vsmc::Threefry4x64
    vSMCThreefry8x64,     ///< vsmc::Threefry8x64
    vSMCThreefry16x64,    ///< vsmc::Threefry16x64
    vSMCThreefry2x32_64,  ///< vsmc::Threefry2x32_64
    vSMCThreefry4x32_64,  ///< vsmc::Threefry4x32_64
    vSMCThreefry2x64_64,  ///< vsmc::Threefry2x64_64
    vSMCThreefry4x64_64,  ///< vsmc::Threefry4x64_64
    vSMCThreefry8x64_64,  ///< vsmc::Threefry8x64_64
    vSMCThreefry16x64_64, ///< vsmc::Threefry16x64_64

    vSMCAES128x1,    ///< vsmc::AES128x1
    vSMCAES128x2,    ///< vsmc::AES128x2
    vSMCAES128x4,    ///< vsmc::AES128x4
    vSMCAES128x8,    ///< vsmc::AES128x8
    vSMCAES128x1_64, ///< vsmc::AES128x1_64
    vSMCAES128x2_64, ///< vsmc::AES128x2_64
    vSMCAES128x4_64, ///< vsmc::AES128x4_64
    vSMCAES128x8_64, ///< vsmc::AES128x8_64

    vSMCAES192x1,    ///< vsmc::AES192x1
    vSMCAES192x2,    ///< vsmc::AES192x2
    vSMCAES192x4,    ///< vsmc::AES192x4
    vSMCAES192x8,    ///< vsmc::AES192x8
    vSMCAES192x1_64, ///< vsmc::AES192x1_64
    vSMCAES192x2_64, ///< vsmc::AES192x2_64
    vSMCAES192x4_64, ///< vsmc::AES192x4_64
    vSMCAES192x8_64, ///< vsmc::AES192x8_64

    vSMCAES256x1,    ///< vsmc::AES256x1
    vSMCAES256x2,    ///< vsmc::AES256x2
    vSMCAES256x4,    ///< vsmc::AES256x4
    vSMCAES256x8,    ///< vsmc::AES256x8
    vSMCAES256x1_64, ///< vsmc::AES256x1_64
    vSMCAES256x2_64, ///< vsmc::AES256x2_64
    vSMCAES256x4_64, ///< vsmc::AES256x4_64
    vSMCAES256x8_64, ///< vsmc::AES256x8_64

    vSMCARSx1,    ///< vsmc::ARSx1
    vSMCARSx2,    ///< vsmc::ARSx2
    vSMCARSx4,    ///< vsmc::ARSx4
    vSMCARSx8,    ///< vsmc::ARSx8
    vSMCARSx1_64, ///< vsmc::ARSx1_64
    vSMCARSx2_64, ///< vsmc::ARSx2_64
    vSMCARSx4_64, ///< vsmc::ARSx4_64
    vSMCARSx8_64, ///< vsmc::ARSx8_64

    vSMCRDRAND16, ///< vsmc::RDRAND16
    vSMCRDRAND32, ///< vsmc::RDRAND32
    vSMCRDRAND64  ///< vsmc::RDRAND64
} vSMCRNGType;

/// \brief SMP backends
typedef enum {
    vSMCBackendSEQ,
    vSMCBackendSTD,
    vSMCBackendOMP,
    vSMCBackendTBB
} vSMCBackendSMP;

/// \brief vSMC RNG types
typedef struct {
    void *ptr;
    vSMCRNGType type;
} vsmc_rng;

/// \brief vsmc::StateMatrix<vsmc::RowMajor, vsmc::Dynamic, double>
typedef struct {
    void *ptr;
} vsmc_state_matrix;

/// \brief vsmc::Weight
typedef struct {
    void *ptr;
} vsmc_weight;

/// \brief vsmc::SingleParticle
typedef struct {
    double *state;
    size_t id;
} vsmc_single_particle;

/// \brief vsmc::Particle
typedef struct {
    void *ptr;
} vsmc_particle;

/// \brief vsmc::Sampler
typedef struct {
    void *ptr;
} vsmc_sampler;

/// \brief vsmc::Monitor
typedef struct {
    void *ptr;
} vsmc_monitor;

/// \brief vsmc::Covariance<double>
typedef struct {
    void *ptr;
} vsmc_covariance;

/// \brief vsmc::StopWatch
typedef struct {
    void *ptr;
} vsmc_stop_watch;

/// \brief vsmc::Sampler::init_type
typedef size_t (*vsmc_sampler_init_type)(vsmc_particle, void *);

/// \brief vsmc::Sampler::move_type
typedef size_t (*vsmc_sampler_move_type)(size_t, vsmc_particle);

/// \brief vsmc::Monitor::eval_type
typedef void (*vsmc_monitor_eval_type)(
    size_t, size_t, vsmc_particle, double *);

/// \brief vsmc::InitializeSMP
typedef struct {
    size_t (*eval_sp)(vsmc_single_particle);
    void (*eval_param)(vsmc_particle, void *);
    void (*eval_pre)(vsmc_particle);
    void (*eval_post)(vsmc_particle);
    vSMCBackendSMP backend;
} vsmc_sampler_init_smp_type;

/// \brief vsmc::MoveSMP
typedef struct {
    size_t (*eval_sp)(size_t, vsmc_single_particle);
    void (*eval_pre)(size_t, vsmc_particle);
    void (*eval_post)(size_t, vsmc_particle);
    vSMCBackendSMP backend;
} vsmc_sampler_move_smp_type;

/// \brief vsmc::MonitorEvalSMP
typedef struct {
    void (*eval_sp)(size_t, size_t, vsmc_single_particle, double *);
    void (*eval_pre)(size_t, vsmc_particle);
    void (*eval_post)(size_t, vsmc_particle);
    vSMCBackendSMP backend;
} vsmc_monitor_eval_smp_type;

/// @} C_API_Definitions

#ifdef __cplusplus
} // extern "C"
#endif

#endif // VSMC_INTERNAL_DEFINES_H

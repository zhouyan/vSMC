//============================================================================
// vSMC/include/vsmc/utility/utility.h
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

#ifndef VSMC_UTILITY_UTILITY_H
#define VSMC_UTILITY_UTILITY_H

#include <vsmc/internal/common.h>

#ifdef __cplusplus
extern "C" {
#endif

/// \addtogroup C_API_Utility_AlignedMemory
/// @{

/// \brief `vsmc::AlignedMemory::aligned_malloc`
void *vsmc_malloc(size_t n, size_t alignment);

/// \brief `vsmc::AlignedMemory::aligned_free`
void vsmc_free(void *ptr);

/// @} C_API_Utility_AlignedMemory

/// \addtogroup C_API_Utility_Covariance
/// @{

/// \brief `vsmc::Covariance::Covariance`
vsmc_covariance vsmc_covariance_new(void);

/// \brief `vsmc::Covariance::~Covariance`
void vsmc_covariance_delete(vsmc_covariance *covariance_ptr);

/// \brief `vsmc::Covariance::operator=`
void vsmc_covariance_assign(vsmc_covariance covariance, vsmc_covariance other);

/// \brief `vsmc::Covariance::operator()`
void vsmc_covariance_compute(vsmc_covariance covariance,
    vSMCMatrixLayout layout, size_t n, size_t p, const double *x,
    const double *w, double *mean, double *cov, vSMCMatrixLayout cov_layout,
    int cov_upper, int cov_packed);

/// @} C_API_Utility_Covariance

/// \addtogroup C_API_Utility_HDF5
/// @{

/// \brief `vsmc::hdf5store`
void vsmc_hdf5store_state_matrix(vsmc_state_matrix state_matrix,
    const char *filename, const char *dataname, int append);

/// \brief `vsmc::hdf5store`
void vsmc_hdf5store_particle(vsmc_particle particle, const char *filename,
    const char *dataname, int append);

/// \brief `vsmc::hdf5store`
void vsmc_hdf5store_monitor(vsmc_monitor monitor, const char *filename,
    const char *dataname, int append);

/// \brief `vsmc::hdf5store`
void vsmc_hdf5store_sampler(vsmc_sampler sampler, const char *filename,
    const char *dataname, int append);

/// @} C_API_Utility_HDF5

/// \addtogroup C_API_Utility_StopWatch
/// @{

/// \brief `vsmc::StopWatch::StopWatch`
vsmc_stop_watch vsmc_stop_watch_new(void);

/// \brief `vsmc::StopWatch::~StopWatch`
void vsmc_stop_watch_delete(vsmc_stop_watch *stop_watch_ptr);

/// \brief `vsmc::StopWatch::operator=`
void vsmc_stop_watch_assign(vsmc_stop_watch stop_watch, vsmc_stop_watch other);

/// \brief `vsmc::StopWatch::running`
int vsmc_stop_watch_running(vsmc_stop_watch stop_watch);

/// \brief `vsmc::StopWatch::start`
int vsmc_stop_watch_start(vsmc_stop_watch stop_watch);

/// \brief `vsmc::StopWatch::stop`
int vsmc_stop_watch_stop(vsmc_stop_watch stop_watch);

/// \brief `vsmc::StopWatch::reset`
void vsmc_stop_watch_reset(vsmc_stop_watch stop_watch);

/// \brief `vsmc::StopWatch::cycles`
double vsmc_stop_watch_cycles(vsmc_stop_watch stop_watch);

/// \brief `vsmc::StopWatch::nanoseconds`
double vsmc_stop_watch_nanoseconds(vsmc_stop_watch stop_watch);

/// \brief `vsmc::StopWatch::microseconds`
double vsmc_stop_watch_microseconds(vsmc_stop_watch stop_watch);

/// \brief `vsmc::StopWatch::milliseconds`
double vsmc_stop_watch_milliseconds(vsmc_stop_watch stop_watch);

/// \brief `vsmc::StopWatch::seconds`
double vsmc_stop_watch_seconds(vsmc_stop_watch stop_watch);

/// \brief `vsmc::StopWatch::minutes`
double vsmc_stop_watch_minutes(vsmc_stop_watch stop_watch);

/// \brief `vsmc::StopWatch::hours`
double vsmc_stop_watch_hours(vsmc_stop_watch stop_watch);

/// @} C_API_Utility_StopWatch

#ifdef __cplusplus
} // extern "C"
#endif

#endif // VSMC_UTILITY_UTILITY_H

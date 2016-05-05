//============================================================================
// vSMC/include/vsmc/vsmc.h
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

#ifndef VSMC_VSMC_H
#define VSMC_VSMC_H

/// \defgroup C_API C API
/// \brief Interfacing with C
///
/// \defgroup C_API_Definitions Enumerators, placeholders and macros
/// \ingroup C_API
///
/// \defgroup C_API_Core Core
/// \ingroup C_API
///
/// \defgroup C_API_Core_StateMatrix vsmc::StateMatrix
/// \ingroup C_API_Core
///
/// \defgroup C_API_Core_Weight vsmc::Weight
/// \ingroup C_API_Core
///
/// \defgroup C_API_Core_Particle vsmc::Particle
/// \ingroup C_API_Core
///
/// \defgroup C_API_Core_Monitor vsmc::Monitor
/// \ingroup C_API_Core
///
/// \defgroup C_API_Core_Sampler vsmc::Sampler
/// \ingroup C_API_Core
///
/// \defgroup C_API_Resample Resample algorithms
/// \ingroup C_API
///
/// \defgroup C_API_RNG Random number generating
/// \ingroup C_API
///
/// \defgroup C_API_RNG_RNG RNG engines
/// \ingroup C_API_RNG
///
/// \defgroup C_API_RNG_Seed vsmc::Seed
/// \ingroup C_API_RNG
///
/// \defgroup C_API_RNG_U01Sequence U01 sequence
/// \ingroup C_API_RNG
///
/// \defgroup C_API_RNG_DISTRIBUITON Distribution
/// \ingroup C_API_RNG
///
/// \defgroup C_API_RNG_MKLRNG Intel Math Kernel Library
/// \ingroup C_API_RNG
///
/// \defgroup C_API_RNG_RandomWalk Random walk
/// \ingroup C_API_RNG
///
/// \defgroup C_API_SMP Symmetric multiprocessing
/// \ingroup C_API
///
/// \defgroup C_API_SMP_SEQ Sequential
/// \ingroup C_API_SMP
///
/// \defgroup C_API_SMP_STD Standard library
/// \ingroup C_API_SMP
///
/// \defgroup C_API_SMP_OMP OpenMP
/// \ingroup C_API_SMP
///
/// \defgroup C_API_SMP_TBB Intel Threading Building Blocks
/// \ingroup C_API_SMP
///
/// \defgroup C_API_Utility Utility
/// \ingroup C_API
///
/// \defgroup C_API_Utility_AlignedMemory Aligned memory allocation
/// \ingroup C_API_Utility
///
/// \defgroup C_API_Utility_Covariance Covariance
/// \ingroup C_API_Utility
///
/// \defgroup C_API_Utility_HDF5IO HDF5 objects IO
/// \ingroup C_API_Utility
///
/// \defgroup C_API_Utility_ProgramOption Program option
/// \ingroup C_API_Utility
///
/// \defgroup C_API_Utility_Progress Progress
/// \ingroup C_API_Utility
///
/// \defgroup C_API_Utility_StopWatch Stop watch
/// \ingroup C_API_Utility

#include <vsmc/core/core.h>
#include <vsmc/resample/resample.h>
#include <vsmc/rng/rng.h>
#include <vsmc/smp/smp.h>
#include <vsmc/utility/utility.h>

#endif // VSMC_VSMC_H

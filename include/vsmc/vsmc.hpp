//============================================================================
// include/vsmc/vsmc.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_HPP
#define VSMC_HPP

/// \defgroup Config Configuration
/// \brief Configuration macros and default values if undefined

/// \defgroup Definitions Enumerators, placeholders and macros
/// \brief Enumerator, placeholder and macro definitions

/// \defgroup Traits Traits
/// \brief Trait classes

/// \defgroup Core Core
/// \brief Constructing samplers with operations on the whole particle set

/// \defgroup Dispatch Grand Central Dispatch
/// \brief C++ wrapper of Apple GCD

/// \defgroup Resample Resampling algorithms
/// \brief Resampling algorithm functor classes

/// \defgroup Adapter Adapter
/// \brief Adapter class templates for constructing concrete objects

/// \defgroup Integrate Integration
/// \brief Numerical integration

/// \defgroup MPI Message Passing Interface
/// \brief Parallel samplers using MPI

/// \defgroup OpenCL OpenCL
/// \brief Parallel sampler using OpenCL

/// \defgroup SMP Symmetric Multiprocessing
/// \brief Parallel samplers using multi-threading on SMP architecture

/// \defgroup CILK Intel Cilk Plus
/// \ingroup SMP
/// \brief Parallel samplers using Intel Cilk Plus

/// \defgroup GCD Grand Central Dispatch
/// \ingroup SMP
/// \brief Parallel samplers using Apple GCD

/// \defgroup OMP OpenMP
/// \ingroup SMP
/// \brief Parallel samplers using OpenMP

/// \defgroup PPL Parallel Pattern Library
/// \ingroup SMP
/// \brief Parallel samplers using Microsoft PPL

/// \defgroup SEQ Sequential
/// \ingroup SMP
/// \brief Sequential samplers

/// \defgroup STD C++11 concurrency
/// \ingroup SMP
/// \brief Parallel samplers using C++11 concurrency

/// \defgroup TBB Intel Threading Building Blocks
/// \ingroup SMP
/// \brief Parallel samplers using Intel TBB

/// \defgroup Math Mathematics
/// \brief Mathematical utilities

/// \defgroup CBLAS C BLAS
/// \brief Selected C BLAS like routines

/// \defgroup Constants Constants
/// \ingroup Math
/// \brief Mathematical constants

/// \defgroup vMath Vector math functions
/// \ingroup Math
/// \brief Math functions on vectors (optional optimization through Intel MKL)

/// \defgroup RNG Random number generating
/// \brief Random number generating engines and utilities

/// \defgroup AESNIRNG AES-NI
/// \ingroup RNG
/// \brief Random number generating using AES-NI

/// \defgroup CLRNG OpenCL
/// \ingroup RNG
/// \brief Random number generating in OpenCL kernels

/// \defgroup Distribution Distribution
/// \ingroup RNG
/// \brief Distribution random varaites

/// \defgroup GSLRNG GSL
/// \ingroup RNG
/// \brief Random number generating using GSL RNG

/// \defgroup MKLRNG MKL
/// \ingroup RNG
/// \brief Random number generating using MKL RNG

/// \defgroup R123RNG Random123
/// \ingroup RNG
/// \brief Random number generating using Random123 RNG

/// \defgroup RDRNG Intel DRNG
/// \ingroup RNG
/// \brief Random number generating using Intel RDRAND and RDSEED instructions

/// \defgroup RNGWrapper Wrapper
/// \ingroup RNG
/// \brief C++11 RNG engines that wrap other RNG generators

/// \defgroup U01 U01
/// \ingroup RNG
/// \brief Converting random integers to random floating points

/// \defgroup Xorshift Xorshift
/// \ingroup RNG
/// \brief Random number generating using the Xorshift algorithm

/// \defgroup Thread Thread
/// \brief C++11 threading support

/// \defgroup Utility Utility
/// \brief Utilities independent of other part of the library

/// \defgroup Array Array
/// \ingroup Utility
/// \brief Container with static size but possible dynamic memory allocation

/// \defgroup CPUID CPUID
/// \ingroup Utility
/// \brief Query CPUID information

/// \defgroup Counter Counter
/// \ingroup Utility
/// \brief Using Array of unsinged integers as counters

/// \defgroup HDF5IO HDF5 objects saving
/// \ingroup Utility
/// \brief Functions for saving objects in HDF5 format

/// \defgroup Option Program option
/// \ingroup Utility
/// \brief Process program command line options

/// \defgroup Progress Progress
/// \ingroup Utility
/// \brief Display progress while algorithms proceed

/// \defgroup RDTSCP RDTSCP
/// \ingroup Utility
/// \brief CPU clock cycles count using RDTSCP

/// \defgroup StopWatch Stop watch
/// \ingroup Utility
/// \brief Utilities for measuring the time of procedures

#include <vsmc/internal/config.hpp>

#include <vsmc/core/monitor.hpp>
#include <vsmc/core/normalizing_constant.hpp>
#include <vsmc/core/particle.hpp>
#include <vsmc/core/path.hpp>
#include <vsmc/core/sampler.hpp>
#include <vsmc/core/single_particle.hpp>
#include <vsmc/core/state_matrix.hpp>
#include <vsmc/core/weight_set.hpp>

#include <vsmc/cxx11/cmath.hpp>
#include <vsmc/cxx11/functional.hpp>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/cxx11/type_traits.hpp>

#include <vsmc/utility/program_option.hpp>
#include <vsmc/utility/stop_watch.hpp>

#endif // VSMC_HPP

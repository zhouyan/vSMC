#ifndef VSMC_HPP
#define VSMC_HPP

#include <vsmc/core/sampler.hpp>
#include <vsmc/smp/adapter.hpp>
#include <vsmc/smp/backend_seq.hpp>
#include <vsmc/integrate/nintegrate_seq.hpp>

#endif // VSMC_HPP

/// \defgroup Config Configuration
/// \brief Configuration macros and default values if undefined

/// \defgroup Compiler Compiler configration
/// \brief Compiler related configuration macros. Default value varies with
/// different compilers
/// \ingroup Config

/// \defgroup Definitions Constants definitions
/// \brief Enumerators and macro definitions

/// \defgroup Traits Traits
/// \brief Trait classes

/// \defgroup Core Core
/// \brief Constructing samplers with operations on the whole particle set

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

/// \defgroup GCD Apple Grand Central Dispatch
/// \ingroup SMP
/// \brief Parallel samplers using Apple GCD

/// \defgroup OMP OpenMP
/// \ingroup SMP
/// \brief Parallel samplers using OpenMP

/// \defgroup PPL Microsoft Parallel Pattern Library
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

/// \defgroup Constants Constants
/// \ingroup Math
/// \brief Mathematical constants

/// \defgroup RNG Random number generating
/// \brief Utilities for managing RNG engines

/// \defgroup R123RNG Random123 RNG
/// \ingroup RNG
/// \brief Random number generating using Random123 RNG

/// \defgroup MKLRNG MKL RNG
/// \ingroup RNG
/// \brief Random number generating using MKL RNG

/// \defgroup Utility Utility
/// \brief Utilities independent of other part of the library

/// \defgroup Backup Object backup
/// \ingroup Utility
/// \brief Backup and restore objects

/// \defgroup HDF5Save HDF5 objects saving
/// \ingroup Utility
/// \brief Functions for saving objects in HDF5 format

/// \defgroup Option Program options
/// \ingroup Utility
/// \brief Process program command line options

/// \defgroup STDTBB STDTBB
/// \ingroup Utility
/// \brief Parallel constructs using C++11 concurrency

/// \defgroup StopWatch Stop watch
/// \ingroup Utility
/// \brief Utilities for measuring the time of procedures

/// \defgroup TBBOp TBB/STDTBB operators
/// \ingroup Utility
/// \brief Operators for Intel TBB and STDTBB

/// \defgroup Tuple Tuple manipulation
/// \ingroup Utility
/// \brief Classes for manipulating C++11 tuple

/// \defgroup Config Configuration
/// \brief Configuration macros and default values if undefined

/// \defgroup Compiler Compiler configration
/// \brief Compiler related configuration macros
/// \ingroup Config

/// \defgroup ThirdParty Third-party libraries
/// \brief Third-party related configuration macros
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

/// \defgroup Constants Constants
/// \ingroup Math
/// \brief Mathematical constants

/// \defgroup IntegerFunction Integer functions
/// \ingroup Math
/// \brief Compile time evaluation of integer functions

/// \defgroup RNG Random number generating
/// \brief Utilities for managing RNG engines

/// \defgroup AESNIRNG AES-NI
/// \ingroup RNG
/// \brief Random number generating using AES-NI

/// \defgroup CLRNG OpenCL
/// \ingroup RNG
/// \brief Random number generating in OpenCL kernels

/// \defgroup GSLRNG GSL
/// \ingroup RNG
/// \brief Random number generating using GSL RNG

/// \defgroup MKLRNG MKL
/// \ingroup RNG
/// \brief Random number generating using MKL RNG

/// \defgroup R123RNG Random123
/// \ingroup RNG
/// \brief Random number generating using Random123 RNG

/// \defgroup RDRNG RdRand and RdSeed
/// \ingroup RNG
/// \brief Random number generating using Intel RdRand and RdSeed

/// \defgroup RNGWrapper Wrapper
/// \ingroup RNG
/// \brief C++11 RNG engines that wrap other RNG generators

/// \defgroup Xorshift Xorshift
/// \ingroup RNG
/// \brief Random number generating using the Xorshift algorithm

/// \defgroup Utility Utility
/// \brief Utilities independent of other part of the library

/// \defgroup Backup Object backup
/// \ingroup Utility
/// \brief Backup and restore objects

/// \defgroup CPUID CPUID
/// \ingroup Utility
/// \brief Query CPUID features

/// \defgroup Dispatch Grand Central Dispatch
/// \ingroup Utility
/// \brief C++ wrapper of Apple GCD

/// \defgroup HDF5IO HDF5 objects saving
/// \ingroup Utility
/// \brief Functions for saving objects in HDF5 format

/// \defgroup Option Program option
/// \ingroup Utility
/// \brief Process program command line options

/// \defgroup Array Array
/// \ingroup Utility
/// \brief Container with static size but possible dynamic memory allocation

/// \defgroup Counter Counter
/// \ingroup Utility
/// \brief Using Array of unsinged integers as counters

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
/// \brief Classes for manipulating and using C++11 tuple like type lists

#ifndef VSMC_HPP
#define VSMC_HPP

#include <vsmc/internal/config.hpp>

#include <vsmc/core/adapter.hpp>
#include <vsmc/core/monitor.hpp>
#include <vsmc/core/normalizing_constant.hpp>
#include <vsmc/core/particle.hpp>
#include <vsmc/core/path.hpp>
#include <vsmc/core/sampler.hpp>
#include <vsmc/core/single_particle.hpp>
#include <vsmc/core/weight_set.hpp>

#include <vsmc/cxx11/cmath.hpp>
#include <vsmc/cxx11/functional.hpp>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/cxx11/type_traits.hpp>

#include <vsmc/integrate/is_integrate.hpp>
#include <vsmc/integrate/nintegrate_base.hpp>
#include <vsmc/integrate/nintegrate_newton_cotes.hpp>

#include <vsmc/math/constants.hpp>
#include <vsmc/math/integer_function.hpp>

#if VSMC_USE_MPI
#include <vsmc/mpi/backend_mpi.hpp>
#include <vsmc/mpi/mpi_manager.hpp>
#endif

#if VSMC_USE_OPENCL
#include <vsmc/opencl/adapter.hpp>
#include <vsmc/opencl/backend_cl.hpp>
#include <vsmc/opencl/cl_manager.hpp>
#include <vsmc/opencl/cl_manip.hpp>
#include <vsmc/opencl/cl_query.hpp>
#endif

#include <vsmc/resample/basic.hpp>
#include <vsmc/resample/multinomial.hpp>
#include <vsmc/resample/residual.hpp>
#include <vsmc/resample/residual_stratified.hpp>
#include <vsmc/resample/residual_systematic.hpp>
#include <vsmc/resample/stratified.hpp>
#include <vsmc/resample/systematic.hpp>

#include <vsmc/rng/rng_set.hpp>
#include <vsmc/rng/seed.hpp>
#include <vsmc/rng/uniform_real_distribution.hpp>
#include <vsmc/rng/generator_wrapper.hpp>
#include <vsmc/rng/philox.hpp>
#include <vsmc/rng/threefry.hpp>
#include <vsmc/rng/xorshift.hpp>
#include <vsmc/rng/xor_combine_engine.hpp>
#if VSMC_USE_MKL
#include <vsmc/rng/mkl_rng.hpp>
#endif
#include <vsmc/rng/urng.h>
#include <vsmc/rng/gammak1.h>
#include <vsmc/rng/normal01.h>
#include <vsmc/rng/u01.h>

#include <vsmc/smp/adapter.hpp>
#include <vsmc/smp/backend_base.hpp>
#if VSMC_USE_CILK
#include <vsmc/smp/backend_cilk.hpp>
#endif
#if VSMC_USE_GCD
#include <vsmc/smp/backend_gcd.hpp>
#endif
#if VSMC_USE_OMP
#include <vsmc/smp/backend_omp.hpp>
#endif
#if VSMC_USE_PPL
#include <vsmc/smp/backend_ppl.hpp>
#endif
#include <vsmc/smp/backend_seq.hpp>
#if VSMC_HAS_CXX11LIB_THREAD
#include <vsmc/smp/backend_std.hpp>
#endif
#if VSMC_USE_TBB
#include <vsmc/smp/backend_tbb.hpp>
#endif
#include <vsmc/smp/state_matrix.hpp>
#if VSMC_HAS_CXX11LIB_TUPLE
#include <vsmc/smp/state_tuple.hpp>
#endif

#include <vsmc/utility/backup.hpp>
#if VSMC_USE_GCD
#include <vsmc/utility/dispatch.hpp>
#endif
#if VSMC_USE_HDF5
#include <vsmc/utility/hdf5_save.hpp>
#endif
#include <vsmc/utility/program_option.hpp>
#if VSMC_HAS_CXX11LIB_THREAD
#include <vsmc/utility/stdtbb.hpp>
#endif
#include <vsmc/utility/stop_watch.hpp>
#include <vsmc/utility/tbb_op.hpp>
#if VSMC_HAS_CXX11LIB_TUPLE
#include <vsmc/utility/tuple_manip.hpp>
#endif

#endif // VSMC_HPP

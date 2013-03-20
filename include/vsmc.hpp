#ifndef VSMC_HPP
#define VSMC_HPP

#include <vsmc/core/sampler.hpp>
#include <vsmc/smp/adapter.hpp>
#include <vsmc/smp/backend_seq.hpp>
// #include <vsmc/smp/backend_cilk.hpp>
// #include <vsmc/smp/backend_gcd.hpp>
// #include <vsmc/smp/backend_omp.hpp>
// #include <vsmc/smp/backend_ppl.hpp>
// #include <vsmc/smp/backend_std.hpp>
// #include <vsmc/smp/backend_tbb.hpp>
// #include <vsmc/mpi/backend_mpi.hpp>
// #include <vsmc/opencl/backend_cl.hpp>
#include <vsmc/utility/integrate/numeric_seq.hpp>
// #include <vsmc/utility/integrate/numeric_cilk.hpp>
// #include <vsmc/utility/integrate/numeric_gcd.hpp>
// #include <vsmc/utility/integrate/numeric_omp.hpp>
// #include <vsmc/utility/integrate/numeric_ppl.hpp>
// #include <vsmc/utility/integrate/numeric_std.hpp>
// #include <vsmc/utility/integrate/numeric_tbb.hpp>

#endif // VSMC_HPP

/// \defgroup Config Configuration
/// \brief Configuration macros and default values if undefined

/// \defgroup Compiler Compiler configration
/// \brief Compiler related configuration macros. Default value varies with
/// different compilers
/// \ingroup Config

/// \defgroup Core Core
/// \brief Constructing samplers with operations on the whole particle set

/// \defgroup Adapter Adapter
/// \brief Adapter class templates for constructing concrete objects

/// \defgroup OpenCL OpenCL
/// \brief Parallel sampler using OpenCL

/// \defgroup SMP Symmetric Multiprocessing
/// \brief Parallel samplers using multi-threading on SMP architecture

/// \defgroup MPI Message Passing Interface
/// \brief Parallel samplers using MPI

/// \defgroup Traits Traits
/// \brief Trait classes

/// \defgroup Utility Utility
/// \brief Utilities independent of other part of the library

/// \defgroup Integrate Integration
/// \ingroup Utility
/// \brief Numerical integration

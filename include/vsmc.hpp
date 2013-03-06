#ifndef VSMC_HPP
#define VSMC_HPP

#include <vsmc/core/sampler.hpp>
#include <vsmc/smp/adapter.hpp>
#include <vsmc/smp/backend_seq.hpp>
#include <vsmc/utility/integrate/numeric_seq.hpp>

#endif // VSMC_HPP

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
///
/// \defgroup Random Random distributions
/// \ingroup Utility
/// \brief C++11 style random distributions

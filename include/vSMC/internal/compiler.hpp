#ifndef VSMC_INTERNAL_COMPILER_HPP
#define VSMC_INTERNAL_COMPILER_HPP

#if defined(__INTEL_COMPILER)
#include <vsmc/internal/compiler/intel.hpp>
#elif defined(__clang__)
#include <vsmc/internal/compiler/clang.hpp>
#elif defined(__GNUC__)
#include <vsmc/internal/compiler/gcc.hpp>
#elif defined(_MSC_VER)
#include <vsmc/internal/compiler/msvc.hpp>
#endif

#endif // VSMC_INTERNAL_COMPILER_HPP

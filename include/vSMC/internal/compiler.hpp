#ifndef V_SMC_INTERNAL_COMPILER_HPP
#define V_SMC_INTERNAL_COMPILER_HPP

#if defined(__INTEL_COMPILER)
#include <vSMC/internal/compiler/intel.hpp>
#elif defined(__clang__)
#include <vSMC/internal/compiler/clang.hpp>
#elif defined(__GNUC__)
#include <vSMC/internal/compiler/gcc.hpp>
#elif defined(_MSC_FULL_VER)
#include <vSMC/internal/compiler/msvc.hpp>
#endif

#endif // V_SMC_INTERNAL_COMPILER_HPP
